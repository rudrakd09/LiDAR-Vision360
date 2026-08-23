"""Background TCP ingestion of the structured Python perception stream -- the backend's own
`PerceptionTCPClient`, in Python.

Runs in its own thread (not asyncio) because it uses a plain blocking `socket.recv()` loop --
exactly the same pattern `PerceptionTCPClient.cs` uses on a background .NET thread, and for the
same reason: a slow/stalled/absent server connection must never block anything else in this
process (FastAPI's own event loop keeps serving REST/WebSocket requests regardless of this
thread's state).

Reuses `perception.streaming.framing.MessageFramer` and `perception.streaming.protocol.
classify_frame_id`/`FrameIdStatus`/`is_frame_id_acceptable` directly -- the exact same functions
`streaming.server`/its own tests already exercise on the Python producer side -- rather than a
third reimplementation of the wire format (Unity's C# port is the only place that *has* to
reimplement this, being a different language/runtime; this is an ordinary same-language Python
consumer, architecturally just another Unity).
"""

from __future__ import annotations

import asyncio
import socket
import threading
import time
from typing import Any

from common.logging import get_logger
from streaming.framing import MessageFramer
from streaming.protocol import classify_frame_id, is_frame_id_acceptable

from .config import Settings
from .db import Database
from .models_db import ClearanceEvent, CollisionEvent, SensorEvent, SessionRecord, TTCEvent, TrackRecord
from .state import LatestState
from .ws import LiveBroadcastHub

logger = get_logger(__name__)


class PerceptionIngestor:
    def __init__(self, settings: Settings, state: LatestState, db: Database, hub: LiveBroadcastHub, loop: asyncio.AbstractEventLoop) -> None:
        self.settings = settings
        self.state = state
        self.db = db
        self.hub = hub
        self.loop = loop

        self._running = False
        self._thread: threading.Thread | None = None
        self._last_accepted_frame_id: int | None = None

        # Session/sequence management (see docs/architecture.md "Session and sequence
        # management") -- the Edge-minted `session_id` this connection is currently tracking, and
        # the one it most recently replaced (kept only so a stale message that somehow still
        # carries that superseded id can be rejected outright -- see
        # `_apply_session_boundary_if_needed`). Both reset to `None` on every fresh TCP connect
        # (see `_run_loop`), same lifetime as `_last_accepted_frame_id` above.
        self._wire_session_id: str | None = None
        self._superseded_wire_session_id: str | None = None

        # In-memory cache of this session's own track roster (track_id -> latest snapshot +
        # this table's own DB row id) -- updated from `data.tracked_objects` on EVERY frame
        # (pure Python dict update, no DB I/O, so this never slows down the ingestion loop), but
        # only ever WRITTEN to the database on a `track_created`/`track_lost` LiveState event or
        # at session close (see `_upsert_track_cache`/`_persist_track_created`/
        # `_persist_track_lost`/`_flush_active_tracks_on_close`) -- one INSERT per track, one
        # UPDATE per loss, never one write per frame per track (see docs/architecture.md
        # "PostgreSQL as history, WebSocket as live transport" -- persistence must never become a
        # per-frame bottleneck on the real-time path).
        self._track_cache: dict[str, dict[str, Any]] = {}

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="perception-ingestor")
        self._thread.start()
        logger.info("[INGEST] Started, will connect to %s:%d.", self.settings.streaming_host, self.settings.streaming_json_port)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3)

    def _run_loop(self) -> None:
        while self._running:
            self.state.connection.state = "connecting"
            self.state.connection.host = self.settings.streaming_host
            self.state.connection.port = self.settings.streaming_json_port
            sock: socket.socket | None = None
            try:
                sock = socket.create_connection((self.settings.streaming_host, self.settings.streaming_json_port), timeout=5)
                sock.settimeout(1.0)
                self.state.connection.state = "connected"
                self.state.connection.connected_at = time.time()
                logger.info("[INGEST] Connected to %s:%d.", self.settings.streaming_host, self.settings.streaming_json_port)
                # A fresh TCP connection means a fresh producer on the other end (a brand new
                # `scripts/serve_unity_bridge.py` run -- a different scenario, or the same one
                # restarted) -- its `frame_id`/`sequence_number` sequence starts back at 0
                # (`datasources.simulated.SimulatedDataSource.__init__`), completely unrelated to
                # whatever frame_id the *previous* producer last reached. Carrying
                # `_last_accepted_frame_id` over from that previous connection would make
                # `classify_frame_id` see every one of this new producer's frames as
                # OUT_OF_ORDER (new_frame_id < stale last_frame_id) and silently drop them in
                # `_handle_frame` -- `state.record_frame` (and therefore `latest_frame`/
                # `source_id`/objects/risk/clearance -- everything the dashboard reads) would
                # then stay frozen on the old producer's last frame until the new one's counter
                # happened to climb back past the old high-water mark -- exactly the reported
                # "dashboard shows stale data from the previous scenario after switching bridge
                # runs" bug this fixes. Event-transition judging itself now lives entirely at the
                # Edge (`pipeline.LiveStateBuilder.events`, see docs/architecture.md "Dashboard
                # and Unity as pure LiveState consumers") -- this ingestor only persists what the
                # Edge already decided, so there is no local "last risk/clearance level" of its
                # own left to reset here. `_track_cache` IS reset -- a new session's tracks have
                # nothing to do with the previous one's (fresh ObjectTracker, track_ids restart at
                # "track-1").
                self._last_accepted_frame_id = None
                self._wire_session_id = None
                self._superseded_wire_session_id = None
                self._track_cache = {}
                # Clears latest_frame/objects/tracks/tracking-history/session_frames_received
                # immediately -- not only once the new session's first frame (and therefore its
                # real session_id) is known. Without this, GET /debug/live-frame, GET /api/latest,
                # and /ws/live's next snapshot would keep serving the PREVIOUS producer's last
                # frame for the entire window between "socket connected" and "first message of the
                # new session actually parsed" -- see LatestState.reset_for_new_session's own
                # docstring.
                self.state.reset_for_new_session(session_id=None, source_id=None)
                self._open_session()

                framer = MessageFramer()
                while self._running:
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("remote closed the connection")
                    for message in framer.feed(chunk):
                        # Deliberately broad and per-message, not just around the socket read:
                        # a bug in dispatch (a malformed/unexpected payload shape, a transient DB
                        # error on the event-transition write, ...) must never kill this whole
                        # background thread -- if it did, `connection.state` would stay frozen at
                        # "connected" forever with no further frames ever processed, which is
                        # indistinguishable from "everything is fine" to any REST/WebSocket caller
                        # (see docs/cloud.md "Known limitations" -- found via a real dashboard bug
                        # report, not a hypothetical: `AttributeError: 'str' object has no
                        # attribute 'get'` from a malformed `risk`/`clearance` field was directly
                        # reproduced killing this thread with the exact old code, see
                        # TestIngestionSurvivesBadMessages). One bad message is logged and
                        # skipped, mirroring the same "one bad scan must not kill the whole run"
                        # principle scripts/serve_unity_bridge.py already applies to the
                        # perception pipeline itself.
                        try:
                            self._dispatch(message)
                        except Exception:  # noqa: BLE001 -- see comment above
                            logger.exception("[INGEST] Error handling message (type=%s) -- skipping, connection stays up.", message.get("message_type"))
            except OSError as e:
                logger.warning("[INGEST] Connection error: %s", e)
            finally:
                if sock is not None:
                    sock.close()
                self._close_session()

            if not self._running:
                break
            self.state.connection.state = "reconnecting"
            time.sleep(self.settings.streaming_reconnect_interval_s)

        self.state.connection.state = "disconnected"

    def _apply_session_boundary_if_needed(self, envelope_session_id: str | None, envelope_source_id: str | None) -> bool:
        """`False` means "reject this message outright, apply nothing" -- it claims a
        `session_id` this ingestor has already moved past (a stale/out-of-order message from a
        now-superseded session; "old sessions must never overwrite new sessions"). `True` means
        "safe to dispatch," having already reset all this-session state first if
        `envelope_session_id` represents a genuinely new session boundary.

        `None`/missing `envelope_session_id` (an older producer that predates this field, or a
        message type built without it) is accepted unconditionally -- there is nothing to compare
        against, so this degrades to the exact same frame_id-only staleness check this backend
        already had, not a regression.
        """
        if not envelope_session_id:
            return True

        if envelope_session_id == self._wire_session_id:
            return True  # the common case -- same session already being tracked

        if envelope_session_id == self._superseded_wire_session_id:
            logger.warning(
                "[INGEST] Rejected message from superseded session_id=%s (current session_id=%s).",
                envelope_session_id, self._wire_session_id,
            )
            return False

        # A genuinely new session boundary. In this project's architecture this always coincides
        # with a fresh TCP reconnect (see _run_loop, which already reset everything once the
        # socket connected) -- this branch mainly confirms the new session_id and records the
        # replaced one for the superseded-rejection check above; it is also a defensive backstop
        # if a session_id ever changed without a reconnect.
        if self._wire_session_id is not None:
            logger.info(
                "[INGEST] New session detected: %s -> %s (source_id=%s) -- resetting session state.",
                self._wire_session_id, envelope_session_id, envelope_source_id,
            )
            self._superseded_wire_session_id = self._wire_session_id
        self._wire_session_id = envelope_session_id
        self._last_accepted_frame_id = None
        self._track_cache = {}
        self.state.reset_for_new_session(envelope_session_id, envelope_source_id)
        return True

    def _dispatch(self, message: dict[str, Any]) -> None:
        if message.get("transmission_timestamp") is not None:
            self.state.connection.last_transmission_timestamp = message["transmission_timestamp"]

        if not self._apply_session_boundary_if_needed(message.get("session_id"), message.get("source_id")):
            return

        message_type = message.get("message_type")
        if message_type == "PERCEPTION_FRAME":
            self._handle_frame(message)
        elif message_type == "HEARTBEAT":
            self.state.record_heartbeat()
            self._broadcast({"type": "heartbeat", "data": message.get("data")})
        elif message_type == "SYSTEM_STATUS":
            self._handle_system_status(message)
            self._broadcast({"type": "status", "data": message.get("data")})
        elif message_type == "ERROR":
            self._broadcast({"type": "error", "data": message.get("data")})
        # unknown message_type -- ignore, matching PerceptionTCPClient.cs's own tolerance

    def _handle_system_status(self, message: dict[str, Any]) -> None:
        data = message.get("data") or {}
        if data.get("source_id"):
            self.state.connection.source_id = data["source_id"]
        if data.get("scan_rate_hz") is not None:
            self.state.connection.scan_rate_hz = data["scan_rate_hz"]

    def _handle_frame(self, message: dict[str, Any]) -> None:
        frame_id = message.get("frame_id")
        if frame_id is None:
            return
        status = classify_frame_id(self._last_accepted_frame_id, frame_id)
        if not is_frame_id_acceptable(status):
            self.state.connection.duplicate_or_out_of_order_dropped += 1
            return
        self._last_accepted_frame_id = frame_id

        data = message.get("data") or {}

        # `source_id` also lives on the SYSTEM_STATUS message, but that is only ever published
        # once per bridge *run* (see docs/cloud.md "Known limitations"), so a client that connects
        # after it already went out -- any late-connecting dashboard/backend, not a hypothetical --
        # would otherwise never learn it and show "no active session" despite frames actively
        # flowing. `data.source_id` is on *every* PERCEPTION_FRAME
        # (`serialization.unity_protocol.build_frame_message`'s own `source_id` field, always
        # present), so use that as the primary source and let SYSTEM_STATUS only add
        # `scan_rate_hz` (which frame data doesn't carry).
        source_id = data.get("source_id")
        if source_id:
            self.state.connection.source_id = source_id

        # Real, per-scan-measured rate from the Edge's own LiveState.performance_metrics (see
        # perception/src/pipeline/live_state.py) -- distinct from SYSTEM_STATUS's scan_rate_hz
        # (the *configured target*, sent once per run and prone to going stale, see docs/cloud.md
        # "Known limitations"). `None` if the producer didn't supply performance_metrics (e.g. an
        # older bridge run, or this frame is literally the first one this session).
        performance = data.get("performance_metrics") or {}
        if performance.get("measured_scan_rate_hz") is not None:
            self.state.connection.measured_scan_rate_hz = performance["measured_scan_rate_hz"]

        self.state.record_frame(data, frame_id)
        self._update_track_cache(data)
        self._persist_events_from_frame(data, frame_id)
        # `broadcast_at`: real wall-clock time this backend is about to broadcast over /ws/live --
        # lets a connected dashboard measure its OWN WebSocket-leg latency
        # (Date.now() - broadcast_at*1000) directly from an actual timestamp, never assumed/
        # fabricated. See docs/architecture.md "Real-time performance monitoring".
        self._broadcast({"type": "frame", "frame_id": frame_id, "data": data, "broadcast_at": time.time()})

    def _open_session(self) -> None:
        with self.db.session() as db_session:
            record = SessionRecord(status="active")
            db_session.add(record)
            db_session.commit()
            self.state.current_session_id = record.id

    def _close_session(self) -> None:
        session_id = self.state.current_session_id
        if session_id is None:
            return
        self._flush_active_tracks_on_close(session_id)
        with self.db.session() as db_session:
            record = db_session.get(SessionRecord, session_id)
            if record is not None:
                record.ended_at = time.time()
                record.status = "ended"
                record.source_id = self.state.connection.source_id
                record.scan_rate_hz = self.state.connection.scan_rate_hz
                record.frame_count = self.state.connection.frames_received
                record.edge_session_id = self._wire_session_id
                db_session.commit()
        self.state.current_session_id = None

    # --- Track persistence (see PerceptionIngestor.__init__'s own docstring on _track_cache:
    # updated every frame in memory only, written to the database only on track_created/
    # track_lost/session-close). ---

    def _update_track_cache(self, data: dict[str, Any]) -> None:
        for obj in data.get("tracked_objects") or []:
            track_id = obj.get("track_id")
            if not track_id:
                continue
            entry = self._track_cache.setdefault(track_id, {})
            entry["classification"] = obj.get("classification")
            entry["sensor_source"] = obj.get("sensor_source")
            entry["last_seen_at"] = obj.get("last_seen") or data.get("timestamp")
            entry["frames_tracked"] = obj.get("frames_tracked")
            entry.setdefault("first_seen_at", obj.get("first_seen") or data.get("timestamp"))

    def _persist_track_created(self, track_id: str, timestamp: float | None) -> None:
        snapshot = self._track_cache.get(track_id, {})
        record = TrackRecord(
            session_id=self.state.current_session_id or "unknown",
            source_id=self.state.connection.source_id,
            track_id=track_id,
            classification=snapshot.get("classification"),
            sensor_source=snapshot.get("sensor_source"),
            first_seen_at=snapshot.get("first_seen_at") or timestamp or time.time(),
            last_seen_at=snapshot.get("last_seen_at") or timestamp or time.time(),
            frames_tracked=snapshot.get("frames_tracked"),
            status="active",
        )
        with self.db.session() as db_session:
            db_session.add(record)
            db_session.commit()
            snapshot["db_id"] = record.id
        logger.info("[INGEST] Track created: %s (session %s).", track_id, self.state.current_session_id)

    def _persist_track_lost(self, track_id: str, timestamp: float | None) -> None:
        snapshot = self._track_cache.pop(track_id, {})
        db_id = snapshot.get("db_id")
        with self.db.session() as db_session:
            record = db_session.get(TrackRecord, db_id) if db_id else None
            if record is None:
                # track_created's own INSERT failed/was never reached (e.g. this track_id was
                # already active when the backend connected mid-session) -- create the row now
                # rather than silently dropping the "lost" fact.
                record = TrackRecord(
                    session_id=self.state.current_session_id or "unknown",
                    source_id=self.state.connection.source_id,
                    track_id=track_id,
                    classification=snapshot.get("classification"),
                    sensor_source=snapshot.get("sensor_source"),
                    first_seen_at=snapshot.get("first_seen_at") or timestamp or time.time(),
                    last_seen_at=snapshot.get("last_seen_at") or timestamp or time.time(),
                    frames_tracked=snapshot.get("frames_tracked"),
                )
                db_session.add(record)
            record.status = "lost"
            record.last_seen_at = snapshot.get("last_seen_at") or timestamp or record.last_seen_at
            record.frames_tracked = snapshot.get("frames_tracked", record.frames_tracked)
            db_session.commit()
        logger.info("[INGEST] Track lost: %s (session %s).", track_id, self.state.current_session_id)

    def _flush_active_tracks_on_close(self, session_id: str) -> None:
        """Write the final known snapshot for every track still active when the SESSION ended
        (the producer disconnected) -- distinct from `track_lost` (the tracker itself decided the
        track was gone while the session continued). Status is left `"active"` -- honest: these
        tracks were never actually confirmed lost, the session just stopped reporting."""
        if not self._track_cache:
            return
        with self.db.session() as db_session:
            for track_id, snapshot in self._track_cache.items():
                db_id = snapshot.get("db_id")
                record = db_session.get(TrackRecord, db_id) if db_id else None
                if record is None:
                    continue
                record.last_seen_at = snapshot.get("last_seen_at", record.last_seen_at)
                record.frames_tracked = snapshot.get("frames_tracked", record.frames_tracked)
            db_session.commit()

    # --- Event persistence -- sourced directly from the Edge's own `data["events"]`
    # (`pipeline.LiveStateBuilder.events`, see docs/architecture.md "Dashboard and Unity as pure
    # LiveState consumers") -- this backend no longer independently re-derives when a transition
    # happened; it only persists what the Edge already decided, enriched with same-frame detail
    # (`data.risk`/`data.clearance`) for the richer DB columns those events don't themselves carry. ---

    def _persist_events_from_frame(self, data: dict[str, Any], frame_id: int) -> None:
        for event in data.get("events") or []:
            event_type = event.get("event_type")
            try:
                if event_type == "collision":
                    self._persist_collision_event(event, data, frame_id)
                elif event_type == "clearance":
                    self._persist_clearance_event(event, data, frame_id)
                elif event_type == "ttc_change":
                    self._persist_ttc_event(event, data, frame_id)
                elif event_type == "track_created":
                    self._persist_track_created(event.get("track_id"), event.get("timestamp"))
                elif event_type == "track_lost":
                    self._persist_track_lost(event.get("track_id"), event.get("timestamp"))
                elif event_type == "sensor":
                    self._persist_sensor_event(event, data, frame_id)
                # "tracking_state_changed" -- deliberately not persisted as its own DB row: it
                # fires on nearly every track's TENTATIVE->CONFIRMED graduation (routine, not a
                # safety-relevant moment), and its info is already implicit in TrackRecord's own
                # lifecycle (created -> [confirmed after enough hits] -> lost). Still broadcast
                # live (see docs/cloud.md) and available in LiveState.events on the wire itself.
            except Exception:  # noqa: BLE001 -- one bad event must not stop the others or kill the ingestion thread, same "one bad message" tolerance _run_loop already applies
                logger.exception("[INGEST] Error persisting event (type=%s) -- skipping.", event_type)

    def _persist_collision_event(self, event: dict[str, Any], data: dict[str, Any], frame_id: int) -> None:
        risk = data.get("risk") or {}
        most_critical = risk.get("most_critical") or {}
        db_event = CollisionEvent(
            session_id=self.state.current_session_id or "unknown",
            source_id=self.state.connection.source_id,
            frame_id=frame_id, timestamp=event.get("timestamp") or time.time(),
            risk_level=event.get("new_value"), previous_risk_level=event.get("previous_value"),
            track_id=event.get("track_id") or most_critical.get("track_id"),
            classification=most_critical.get("classification"),
            distance_m=most_critical.get("distance"), ttc_s=most_critical.get("ttc"),
            collision_predicted=most_critical.get("collision_predicted"),
            reason="; ".join(most_critical.get("reason") or []),
        )
        with self.db.session() as db_session:
            db_session.add(db_event)
            db_session.commit()
        logger.info("[INGEST] Collision risk transition: %s -> %s (frame %s).", event.get("previous_value"), event.get("new_value"), frame_id)

    def _persist_clearance_event(self, event: dict[str, Any], data: dict[str, Any], frame_id: int) -> None:
        clearance = data.get("clearance") or {}
        db_event = ClearanceEvent(
            session_id=self.state.current_session_id or "unknown",
            source_id=self.state.connection.source_id,
            frame_id=frame_id, timestamp=event.get("timestamp") or time.time(),
            overall_status=event.get("new_value"), previous_status=event.get("previous_value"),
            min_direction=clearance.get("min_direction"), min_clearance_m=clearance.get("min_clearance_m"),
            corridor_width_m=clearance.get("corridor_width_m"), reason="; ".join(clearance.get("reason") or []),
        )
        with self.db.session() as db_session:
            db_session.add(db_event)
            db_session.commit()
        logger.info("[INGEST] Clearance status transition: %s -> %s (frame %s).", event.get("previous_value"), event.get("new_value"), frame_id)

    def _persist_ttc_event(self, event: dict[str, Any], data: dict[str, Any], frame_id: int) -> None:
        track_id = event.get("track_id")
        result = next((r for r in ((data.get("risk") or {}).get("results") or []) if r.get("track_id") == track_id), {})
        db_event = TTCEvent(
            session_id=self.state.current_session_id or "unknown",
            source_id=self.state.connection.source_id,
            frame_id=frame_id, timestamp=event.get("timestamp") or time.time(),
            track_id=track_id, previous_value=event.get("previous_value"), new_value=event.get("new_value"),
            ttc_s=result.get("ttc"), classification=result.get("classification"), distance_m=result.get("distance"),
            reason="; ".join(result.get("reason") or []),
        )
        with self.db.session() as db_session:
            db_session.add(db_event)
            db_session.commit()
        logger.info("[INGEST] TTC transition: track %s %s -> %s (frame %s).", track_id, event.get("previous_value"), event.get("new_value"), frame_id)

    def _persist_sensor_event(self, event: dict[str, Any], data: dict[str, Any], frame_id: int) -> None:
        lidar = (data.get("sensor_status") or {}).get("lidar") or {}
        db_event = SensorEvent(
            session_id=self.state.current_session_id or "unknown",
            source_id=self.state.connection.source_id,
            frame_id=frame_id, timestamp=event.get("timestamp") or time.time(),
            sensor_type="lidar", previous_status=event.get("previous_value"), new_status=event.get("new_value"),
            valid_percentage=lidar.get("valid_percentage"), summary=event.get("summary"),
        )
        with self.db.session() as db_session:
            db_session.add(db_event)
            db_session.commit()
        logger.info("[INGEST] Sensor status transition: %s -> %s (frame %s).", event.get("previous_value"), event.get("new_value"), frame_id)

    def _broadcast(self, message: dict[str, Any]) -> None:
        """Schedule a broadcast onto the main asyncio event loop from this background thread --
        never awaited here (this thread has no event loop of its own)."""
        try:
            asyncio.run_coroutine_threadsafe(self.hub.broadcast(message), self.loop)
        except RuntimeError:
            pass  # event loop already closed (shutdown race) -- nothing to do

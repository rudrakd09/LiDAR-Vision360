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
from .models_db import ClearanceEvent, CollisionEvent, SessionRecord
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
        self._last_risk_level: str | None = None
        self._last_clearance_status: str | None = None

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
                        self._dispatch(message)
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

    def _dispatch(self, message: dict[str, Any]) -> None:
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
        self.state.connection.source_id = data.get("source_id")
        self.state.connection.scan_rate_hz = data.get("scan_rate_hz")

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
        self.state.record_frame(data, frame_id)
        self._detect_and_persist_transitions(data, frame_id, message.get("timestamp"))
        self._broadcast({"type": "frame", "frame_id": frame_id, "data": data})

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
        with self.db.session() as db_session:
            record = db_session.get(SessionRecord, session_id)
            if record is not None:
                record.ended_at = time.time()
                record.status = "ended"
                record.source_id = self.state.connection.source_id
                record.scan_rate_hz = self.state.connection.scan_rate_hz
                record.frame_count = self.state.connection.frames_received
                db_session.commit()
        self.state.current_session_id = None

    def _detect_and_persist_transitions(self, data: dict[str, Any], frame_id: int, timestamp: float | None) -> None:
        risk = data.get("risk")
        if risk is not None:
            level = risk.get("overall_risk")
            if level != self._last_risk_level:
                self._persist_collision_event(risk, level, frame_id, timestamp)
                self._last_risk_level = level

        clearance = data.get("clearance")
        if clearance is not None:
            status = clearance.get("overall_status")
            if status != self._last_clearance_status:
                self._persist_clearance_event(clearance, status, frame_id, timestamp)
                self._last_clearance_status = status

    def _persist_collision_event(self, risk: dict, level: str, frame_id: int, timestamp: float | None) -> None:
        most_critical = risk.get("most_critical") or {}
        event = CollisionEvent(
            session_id=self.state.current_session_id or "unknown",
            frame_id=frame_id, timestamp=timestamp or time.time(),
            risk_level=level, previous_risk_level=self._last_risk_level,
            track_id=most_critical.get("track_id"), classification=most_critical.get("classification"),
            distance_m=most_critical.get("distance"), ttc_s=most_critical.get("ttc"),
            reason="; ".join(most_critical.get("reason") or []),
        )
        with self.db.session() as db_session:
            db_session.add(event)
            db_session.commit()
        logger.info("[INGEST] Collision risk transition: %s -> %s (frame %s).", self._last_risk_level, level, frame_id)

    def _persist_clearance_event(self, clearance: dict, status: str, frame_id: int, timestamp: float | None) -> None:
        event = ClearanceEvent(
            session_id=self.state.current_session_id or "unknown",
            frame_id=frame_id, timestamp=timestamp or time.time(),
            overall_status=status, previous_status=self._last_clearance_status,
            min_direction=clearance.get("min_direction"), min_clearance_m=clearance.get("min_clearance_m"),
            corridor_width_m=clearance.get("corridor_width_m"), reason="; ".join(clearance.get("reason") or []),
        )
        with self.db.session() as db_session:
            db_session.add(event)
            db_session.commit()
        logger.info("[INGEST] Clearance status transition: %s -> %s (frame %s).", self._last_clearance_status, status, frame_id)

    def _broadcast(self, message: dict[str, Any]) -> None:
        """Schedule a broadcast onto the main asyncio event loop from this background thread --
        never awaited here (this thread has no event loop of its own)."""
        try:
            asyncio.run_coroutine_threadsafe(self.hub.broadcast(message), self.loop)
        except RuntimeError:
            pass  # event loop already closed (shutdown race) -- nothing to do

"""Thread-safe in-memory latest-state + bounded ring buffer -- the backend's equivalent of
Unity's `PerceptionTCPClient` fields (`State`, `LastFrameId`, ...), shared between the ingestion
thread (the only writer) and REST/WebSocket handlers (readers) via one lock.

Never grows without bound: `frames` is a `deque(maxlen=Settings.backend_ring_buffer_size)`, the
same "bounded, drop-oldest" principle `perception.streaming.LatestFrameQueue` already established
on the Python->Unity side, applied here to the backend's own recent-frame cache. Per-track history
(`_track_history`) is bounded the same way, per track, at `Settings.backend_track_history_length`.

**No tracking algorithm lives here.** Every field recorded below is copied verbatim from what
`tracking.ObjectTracker` (Phase 7) already put in the frame's own `objects[]` entries -- this
class only remembers a bounded window of what it was already told, per track_id, so a caller can
ask "how did this specific track move over the last N scans" without re-deriving it from scratch
on every request.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any


@dataclass
class ConnectionInfo:
    state: str = "disconnected"  # "disconnected" | "connecting" | "connected" | "reconnecting"
    host: str = ""
    port: int = 0
    connected_at: float | None = None
    last_message_at: float | None = None
    frames_received: int = 0  # cumulative for this backend PROCESS's lifetime -- survives reconnects, unchanged behavior (see session_frames_received below for the per-session count)
    duplicate_or_out_of_order_dropped: int = 0  # same "whole process lifetime" scope as frames_received
    last_frame_id: int | None = None
    source_id: str | None = None
    scan_rate_hz: float | None = None  # from SYSTEM_STATUS -- the *configured target* rate, sent once per run (can go stale -- see docs/cloud.md "Known limitations")

    # --- Session/sequence management (see docs/architecture.md "Session and sequence
    # management") -- the Edge-minted identity, distinct from `backend.state.LatestState.
    # current_session_id` (this backend's OWN DB SessionRecord primary key, one per ingestion TCP
    # connection -- a separate bookkeeping concept that continues to exist unchanged). This is the
    # id `GET /debug/stream-status`/`GET /api/status` report as `session_id`, and the one
    # `PerceptionIngestor` compares incoming messages against to detect a session boundary. ---
    session_id: str | None = None
    session_frames_received: int = 0  # resets to 0 every new-session boundary -- see LatestState.reset_for_new_session
    last_transmission_timestamp: float | None = None  # envelope's own transmission_timestamp of the most recent message -- for latency_ms, see routes/debug.py
    measured_scan_rate_hz: float | None = None  # from the Edge's own, per-scan-measured LiveState.performance_metrics.measured_scan_rate_hz -- real, not the possibly-stale SYSTEM_STATUS value above

    # --- Last ERROR message from the producer (e.g. Phase-3 hardware mode's
    # HARDWARE_DATA_UNAVAILABLE, with a concrete reason). Cleared the moment a real PERCEPTION_FRAME
    # lands again (record_frame), so a recovered stream never keeps showing a stale error. Surfaced
    # by GET /debug/stream-status so a person can see WHY the stream is not delivering. ---
    last_error_code: str | None = None
    last_error_message: str | None = None
    last_error_at: float | None = None


def compute_session_status(connection: ConnectionInfo, now: float, stale_threshold_s: float) -> str:
    """"active" | "stale" | "disconnected" -- deliberately a *separate* concept from `connection.
    state` (the raw TCP state): a socket can stay "connected" while the bridge process on the
    other end has stalled without actually closing it, and a fresh connection with no message yet
    is not the same thing as one that *was* receiving messages and stopped. See docs/cloud.md
    "Session lifecycle" -- this is what fixes "Backend->Bridge: connected" / "Session: no active
    session" ever being shown as if they were the same fact.
    """
    if connection.state != "connected":
        return "disconnected"
    if connection.last_message_at is None:
        return "active"  # freshly connected, grace period before the first message -- not stale
    return "active" if (now - connection.last_message_at) <= stale_threshold_s else "stale"


class LatestState:
    """One instance for the process lifetime (constructed in `main.py`'s lifespan, stored on
    `app.state`)."""

    def __init__(self, ring_buffer_size: int, track_grace_period_s: float, track_history_length: int = 50) -> None:
        self._lock = threading.Lock()
        self._frames: deque[dict[str, Any]] = deque(maxlen=ring_buffer_size)
        self._latest_frame: dict[str, Any] | None = None

        self._track_last_seen: dict[str, tuple[dict[str, Any], float]] = {}  # track_id -> (object dict, last_seen_at)
        self._track_first_seen: dict[str, float] = {}
        self._track_frame_count: dict[str, int] = {}
        self._track_history: dict[str, deque[dict[str, Any]]] = {}

        self._track_grace_period_s = track_grace_period_s
        self._track_history_length = track_history_length
        self.connection = ConnectionInfo()
        self.current_session_id: str | None = None

    def reset_for_new_session(self, session_id: str | None, source_id: str | None) -> None:
        """Called the moment a new session boundary is detected (see `backend.ingestion.
        PerceptionIngestor`) -- either a fresh TCP reconnect (session_id/source_id not known yet,
        both `None`: the previous producer's socket just dropped, and nothing about the next one
        -- new scenario, hardware, or the same scenario restarted -- is known yet) or a genuine
        new `session_id` observed on an already-open connection (a defensive backstop for "old
        sessions must never overwrite new sessions" -- see `PerceptionIngestor._dispatch`).

        Clears every piece of this-session state so a caller reading ANY of `latest_frame`/
        `tracks_roster`/`track_history` immediately after this call sees a clean slate, never the
        previous session's data -- this is what makes `GET /debug/live-frame` (and `GET
        /api/latest`, and `/ws/live`'s next snapshot) never serve a stale frame across a scenario
        switch, even during the brief window before the new session's first real frame arrives.
        Objects/tracks/tracking history/TTC/clearance/risk all live inside `_latest_frame` and the
        per-track dicts below -- clearing those clears all of them in one place, since none of
        them has a separate cache anywhere else in this class. The event timeline is intentionally
        NOT touched here -- it is DB-backed and already session-scoped by `LatestState.
        current_session_id` (see routes/events.py's own default-session-filter), which updates
        independently, via `PerceptionIngestor._open_session`, the moment this same reconnect
        opens its own new `SessionRecord`.
        """
        with self._lock:
            self._frames.clear()
            self._latest_frame = None
            self._track_last_seen.clear()
            self._track_first_seen.clear()
            self._track_frame_count.clear()
            self._track_history.clear()
            self.connection.session_id = session_id
            self.connection.source_id = source_id
            self.connection.session_frames_received = 0
            self.connection.last_frame_id = None
            self.connection.measured_scan_rate_hz = None

    def record_error(self, code: str | None, message: str | None) -> None:
        """Remember the most recent producer ERROR message (see `ConnectionInfo.last_error_*`)."""
        with self._lock:
            self.connection.last_error_code = code
            self.connection.last_error_message = message
            self.connection.last_error_at = time.time() if code is not None else None

    def record_frame(self, frame_data: dict[str, Any], frame_id: int | None) -> None:
        now = time.time()
        with self._lock:
            self._frames.append(frame_data)
            self._latest_frame = frame_data
            self.connection.frames_received += 1
            self.connection.session_frames_received += 1
            self.connection.last_frame_id = frame_id
            self.connection.last_message_at = now
            # A real frame arrived -> any prior "stream unavailable" error is no longer current.
            self.connection.last_error_code = None
            self.connection.last_error_message = None
            self.connection.last_error_at = None

            timestamp = frame_data.get("timestamp")
            for obj in frame_data.get("objects") or []:
                track_id = obj.get("track_id")
                if not track_id:
                    continue
                self._track_last_seen[track_id] = (obj, now)
                if track_id not in self._track_first_seen:
                    self._track_first_seen[track_id] = now
                self._track_frame_count[track_id] = self._track_frame_count.get(track_id, 0) + 1
                self._append_track_history(track_id, obj, frame_id, timestamp)

            self._prune_stale_tracks(now)

    def _append_track_history(self, track_id: str, obj: dict[str, Any], frame_id: int | None, timestamp: Any) -> None:
        history = self._track_history.setdefault(track_id, deque(maxlen=self._track_history_length))
        centroid = obj.get("centroid") or {}
        velocity = obj.get("velocity")
        history.append({
            "frame_id": frame_id,
            "timestamp": timestamp,
            "x": centroid.get("x"),
            "y": centroid.get("y"),
            "vx": velocity.get("vx") if velocity else None,
            "vy": velocity.get("vy") if velocity else None,
            "distance": obj.get("distance"),
            "classification": obj.get("classification"),
            "tracking_state": obj.get("tracking_state"),
        })

    def record_heartbeat(self) -> None:
        with self._lock:
            self.connection.last_message_at = time.time()

    def _prune_stale_tracks(self, now: float) -> None:
        stale = [tid for tid, (_, seen_at) in self._track_last_seen.items() if now - seen_at > self._track_grace_period_s]
        for tid in stale:
            del self._track_last_seen[tid]
            self._track_first_seen.pop(tid, None)
            self._track_frame_count.pop(tid, None)
            self._track_history.pop(tid, None)

    @property
    def latest_frame(self) -> dict[str, Any] | None:
        with self._lock:
            return self._latest_frame

    def recent_frames(self, limit: int | None = None) -> list[dict[str, Any]]:
        with self._lock:
            items = list(self._frames)
        return items[-limit:] if limit else items

    def tracks_roster(self) -> list[dict[str, Any]]:
        """Every track_id seen within `track_grace_period_s`, most-recent sighting only --
        distinct from `latest_frame["objects"]` (this frame only) -- see `Settings.
        backend_track_grace_period_s`'s own docstring for the Unity-side precedent this mirrors.
        Includes the same first-seen/last-seen/frames-tracked summary fields `track_summary()`
        returns for a single track, so a dashboard table doesn't need one request per row just to
        show them."""
        now = time.time()
        with self._lock:
            self._prune_stale_tracks(now)
            return [
                dict(
                    obj,
                    seconds_since_seen=round(now - seen_at, 3),
                    first_seen_at=self._track_first_seen.get(track_id),
                    last_seen_at=seen_at,
                    frames_tracked=self._track_frame_count.get(track_id, 0),
                )
                for track_id, (obj, seen_at) in self._track_last_seen.items()
            ]

    def track_summary(self, track_id: str) -> dict[str, Any] | None:
        """`None` if `track_id` isn't currently known (never seen, or pruned after
        `track_grace_period_s` with no sighting) -- a 404 at the route layer, not an empty/zeroed
        object standing in for "doesn't exist"."""
        now = time.time()
        with self._lock:
            self._prune_stale_tracks(now)
            entry = self._track_last_seen.get(track_id)
            if entry is None:
                return None
            obj, last_seen_at = entry
            return {
                "track_id": track_id,
                "current": obj,
                "first_seen_at": self._track_first_seen.get(track_id),
                "last_seen_at": last_seen_at,
                "seconds_since_seen": round(now - last_seen_at, 3),
                "frames_tracked": self._track_frame_count.get(track_id, 0),
                "history_length": len(self._track_history.get(track_id, ())),
            }

    def track_history(self, track_id: str, limit: int | None = None) -> list[dict[str, Any]] | None:
        """`None` if `track_id` isn't currently known -- same distinction as `track_summary`.
        `[]` (empty list, not `None`) is a real, valid answer for a track that exists but hasn't
        recorded a history point yet (shouldn't normally happen, since a history point is
        recorded the same moment a track first becomes known, but kept honest rather than assumed
        impossible)."""
        with self._lock:
            if track_id not in self._track_last_seen:
                return None
            items = list(self._track_history.get(track_id, ()))
        return items[-limit:] if limit else items

    def session_status(self, stale_threshold_s: float) -> str:
        return compute_session_status(self.connection, time.time(), stale_threshold_s)

    def snapshot_for_new_client(self, stale_threshold_s: float) -> dict[str, Any]:
        """What a freshly-connected WebSocket client receives immediately, before any new frame
        arrives -- mirrors the "send current state, then subscribe" pattern. Includes
        `session_status` (see `compute_session_status`) so a client that only ever reads this one
        snapshot still gets an accurate answer, not a field silently missing -- see
        docs/cloud.md "Session lifecycle" for why the WebSocket snapshot on its own is not enough
        to keep this fresh for the lifetime of a long connection (the dashboard additionally
        re-polls `GET /api/status` periodically for exactly that reason)."""
        with self._lock:
            return {
                "connection": _connection_dict(self.connection, self.session_status(stale_threshold_s)),
                "latest_frame": self._latest_frame,
            }


def _connection_dict(info: ConnectionInfo, session_status: str) -> dict[str, Any]:
    return {
        "state": info.state,
        "host": info.host,
        "port": info.port,
        "connected_at": info.connected_at,
        "last_message_at": info.last_message_at,
        "frames_received": info.frames_received,
        "duplicate_or_out_of_order_dropped": info.duplicate_or_out_of_order_dropped,
        "last_frame_id": info.last_frame_id,
        "source_id": info.source_id,
        "scan_rate_hz": info.scan_rate_hz,
        "session_status": session_status,
        "session_id": info.session_id,
        "session_frames_received": info.session_frames_received,
        "measured_scan_rate_hz": info.measured_scan_rate_hz,
        "last_error_code": info.last_error_code,
        "last_error_message": info.last_error_message,
    }

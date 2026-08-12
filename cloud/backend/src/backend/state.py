"""Thread-safe in-memory latest-state + bounded ring buffer -- the backend's equivalent of
Unity's `PerceptionTCPClient` fields (`State`, `LastFrameId`, ...), shared between the ingestion
thread (the only writer) and REST/WebSocket handlers (readers) via one lock.

Never grows without bound: `frames` is a `deque(maxlen=Settings.backend_ring_buffer_size)`, the
same "bounded, drop-oldest" principle `perception.streaming.LatestFrameQueue` already established
on the Python->Unity side, applied here to the backend's own recent-frame cache.
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
    frames_received: int = 0
    duplicate_or_out_of_order_dropped: int = 0
    last_frame_id: int | None = None
    source_id: str | None = None
    scan_rate_hz: float | None = None


class LatestState:
    """One instance for the process lifetime (constructed in `main.py`'s lifespan, stored on
    `app.state`)."""

    def __init__(self, ring_buffer_size: int, track_grace_period_s: float) -> None:
        self._lock = threading.Lock()
        self._frames: deque[dict[str, Any]] = deque(maxlen=ring_buffer_size)
        self._latest_frame: dict[str, Any] | None = None
        self._track_last_seen: dict[str, tuple[dict[str, Any], float]] = {}  # track_id -> (object dict, last_seen_at)
        self._track_grace_period_s = track_grace_period_s
        self.connection = ConnectionInfo()
        self.current_session_id: str | None = None

    def record_frame(self, frame_data: dict[str, Any], frame_id: int | None) -> None:
        now = time.time()
        with self._lock:
            self._frames.append(frame_data)
            self._latest_frame = frame_data
            self.connection.frames_received += 1
            self.connection.last_frame_id = frame_id
            self.connection.last_message_at = now

            for obj in frame_data.get("objects") or []:
                track_id = obj.get("track_id")
                if track_id:
                    self._track_last_seen[track_id] = (obj, now)

            self._prune_stale_tracks(now)

    def record_heartbeat(self) -> None:
        with self._lock:
            self.connection.last_message_at = time.time()

    def _prune_stale_tracks(self, now: float) -> None:
        stale = [tid for tid, (_, seen_at) in self._track_last_seen.items() if now - seen_at > self._track_grace_period_s]
        for tid in stale:
            del self._track_last_seen[tid]

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
        backend_track_grace_period_s`'s own docstring for the Unity-side precedent this mirrors."""
        now = time.time()
        with self._lock:
            self._prune_stale_tracks(now)
            return [dict(obj, seconds_since_seen=round(now - seen_at, 3)) for obj, seen_at in self._track_last_seen.values()]

    def snapshot_for_new_client(self) -> dict[str, Any]:
        """What a freshly-connected WebSocket client receives immediately, before any new frame
        arrives -- mirrors the "send current state, then subscribe" pattern."""
        with self._lock:
            return {
                "connection": _connection_dict(self.connection),
                "latest_frame": self._latest_frame,
            }


def _connection_dict(info: ConnectionInfo) -> dict[str, Any]:
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
    }

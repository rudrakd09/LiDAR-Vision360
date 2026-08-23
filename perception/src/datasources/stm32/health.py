"""Per-channel sensor health/status tracking for the STM32 hardware adapter.

Deliberately Edge-side and per-*sensor-channel* (lidar / radar), distinct from
`cloud/backend/src/backend/state.ConnectionInfo` (which tracks the backend's own TCP connection
to the bridge, one layer downstream) -- this is "is the physical A3M1/R121 data actually still
arriving over the STM32 link," the thing `STM32Source` itself needs to decide when to raise
`STM32TimeoutError`/attempt a reconnect, and what `sensor_status` in `LiveState` should ultimately
report for a hardware run (see `pipeline/live_state.py`'s existing `"radar": None` -- to be
replaced with a real reading once real radar data actually flows; see the hardware integration
checklist).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class ChannelHealth:
    """Health for one logical channel (e.g. `"lidar"` or `"radar"`)."""

    connected: bool = False
    frames_received: int = 0
    frames_dropped: int = 0
    duplicate_or_out_of_order: int = 0
    protocol_errors: int = 0
    last_received_at: float | None = None
    last_error: str | None = None

    def is_stale(self, timeout_s: float, *, now: float | None = None) -> bool:
        """True if this channel has never received data, or hasn't in over `timeout_s` seconds."""
        if self.last_received_at is None:
            return True
        return (now if now is not None else time.time()) - self.last_received_at > timeout_s


@dataclass
class HealthMonitor:
    """Owns one `ChannelHealth` per channel name, plus the source-level connection state."""

    channels: dict[str, ChannelHealth] = field(default_factory=dict)
    connected: bool = False
    connected_at: float | None = None
    disconnected_at: float | None = None
    reconnect_attempts: int = 0
    last_reconnect_error: str | None = None

    def channel(self, name: str) -> ChannelHealth:
        return self.channels.setdefault(name, ChannelHealth())

    def mark_connected(self) -> None:
        self.connected = True
        self.connected_at = time.time()
        self.disconnected_at = None

    def mark_disconnected(self) -> None:
        self.connected = False
        self.disconnected_at = time.time()
        for ch in self.channels.values():
            ch.connected = False

    def record_frame(self, channel: str, *, dropped: int = 0, duplicate_or_out_of_order: bool = False) -> None:
        ch = self.channel(channel)
        ch.connected = True
        ch.frames_received += 1
        ch.frames_dropped += dropped
        if duplicate_or_out_of_order:
            ch.duplicate_or_out_of_order += 1
        ch.last_received_at = time.time()

    def record_protocol_error(self, channel: str, message: str) -> None:
        ch = self.channel(channel)
        ch.protocol_errors += 1
        ch.last_error = message

    def record_reconnect_attempt(self, error: str | None = None) -> None:
        self.reconnect_attempts += 1
        if error is not None:
            self.last_reconnect_error = error

    def snapshot(self) -> dict:
        """Plain-dict summary, suitable for logging or eventually surfacing through
        `sensor_status` -- deliberately not a pydantic model here, since this is an internal
        Edge-side diagnostic structure, not part of the wire protocol."""
        return {
            "connected": self.connected,
            "connected_at": self.connected_at,
            "disconnected_at": self.disconnected_at,
            "reconnect_attempts": self.reconnect_attempts,
            "last_reconnect_error": self.last_reconnect_error,
            "channels": {
                name: {
                    "connected": ch.connected,
                    "frames_received": ch.frames_received,
                    "frames_dropped": ch.frames_dropped,
                    "duplicate_or_out_of_order": ch.duplicate_or_out_of_order,
                    "protocol_errors": ch.protocol_errors,
                    "last_received_at": ch.last_received_at,
                    "last_error": ch.last_error,
                }
                for name, ch in self.channels.items()
            },
        }

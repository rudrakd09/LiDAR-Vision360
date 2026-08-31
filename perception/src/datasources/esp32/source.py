"""`ESP32Source` -- the Edge PC's receiver for processed perception frames arriving over the
STM32 -> ESP32 -> Wi-Fi link.

    ESP32 --(ESP32Transport)--> ESP32Source --STM32ProcessedFrame--> (adapter) --> LiveState

`ESP32Source` is the hardware-mode counterpart of `simulator.SimulatorSource`. It does **not**
run any perception algorithm and does **not** produce a raw `ScanFrame` -- the STM32 already did
the perception; this class only:

* receives bytes from a replaceable `ESP32Transport`,
* decodes+validates them into a Phase-2 `models.stm32_processed.STM32ProcessedFrame`
  (via a `datasources.stm32.processed` deserializer -- `JsonProcessedFrameCodec` by default,
  swappable for the real wire codec once specified),
* enforces sequence continuity (reusing `datasources.stm32.sequence.SequenceValidator`),
* tracks connection health, staleness, dropped frames, and reconnection with backoff,
* and **never fabricates a frame** -- when no fresh valid frame is available it returns `None`
  and reports a reason (`ESP32SourceStatus.reason`), so the caller shows "HARDWARE DATA
  UNAVAILABLE" rather than stale or synthetic data.

Lifecycle mirrors `LiDARDataSource` (`connect` / `disconnect` / `is_connected`), plus `poll()`
(returns the newest not-yet-returned fresh frame, or `None`) and `status` (an `ESP32SourceStatus`
snapshot). It is **not** a `LiDARDataSource` subclass -- the return type differs -- but it
implements the small `ProcessedFrameSource` protocol for symmetry.
"""

from __future__ import annotations

import time
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum

from common.config import Settings, get_settings
from common.logging import get_logger
from models.stm32_processed import STM32ProcessedFrame

from ..stm32.processed import (
    JsonProcessedFrameCodec,
    ProcessedFrameDeserializer,
    STM32ContractVersionError,
    STM32ProcessedFrameError,
)
from ..stm32.sequence import SequenceValidator
from .errors import ESP32ConfigurationError, ESP32ConnectionError, ESP32TransportError
from .transport import ESP32Transport, build_transport

logger = get_logger(__name__)


class ESP32SourceState(str, Enum):
    """Where the ESP32 link stands right now."""

    UNAVAILABLE = "unavailable"      # not configured, or reconnection exhausted -- no data path
    CONNECTING = "connecting"        # transport open, waiting for the first valid frame
    CONNECTED = "connected"          # fresh valid frames flowing
    STALE = "stale"                  # transport open, but newest frame older than the stale threshold
    RECONNECTING = "reconnecting"    # link dropped, retrying with backoff
    DISCONNECTED = "disconnected"    # link closed/errored (transient, pre-reconnect)


_UNAVAILABLE_STATES = {
    ESP32SourceState.UNAVAILABLE,
    ESP32SourceState.CONNECTING,
    ESP32SourceState.STALE,
    ESP32SourceState.RECONNECTING,
    ESP32SourceState.DISCONNECTED,
}


@dataclass
class ESP32SourceStatus:
    """Immutable snapshot of the link's health -- what a debug endpoint / dashboard needs to show
    "HARDWARE DATA UNAVAILABLE" with a concrete reason, and the freshness/rate/drop counters."""

    state: ESP32SourceState
    reason: str | None
    session_id: str
    connected_at: float | None = None
    last_frame_at: float | None = None          # Edge wall-clock receive time of the newest accepted frame
    last_frame_timestamp: float | None = None   # that frame's own metadata.timestamp
    last_frame_id: int | None = None
    last_sequence_number: int | None = None
    frame_age_s: float | None = None
    frames_received: int = 0
    frames_rejected: int = 0
    dropped_frames: int = 0
    sequence_gaps: int = 0
    reconnect_attempts: int = 0
    receive_rate_hz: float | None = None

    @property
    def is_fresh(self) -> bool:
        return self.state is ESP32SourceState.CONNECTED

    @property
    def is_unavailable(self) -> bool:
        return self.state in _UNAVAILABLE_STATES


class ProcessedFrameSource(ABC):
    """Minimal contract for anything that yields validated `STM32ProcessedFrame`s -- the
    processed-perception analogue of `datasources.LiDARDataSource`. `ESP32Source` is the only
    implementation today; a future direct STM32<->Edge wired link carrying processed frames would
    be another."""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    @abstractmethod
    def poll(self) -> STM32ProcessedFrame | None: ...

    @property
    @abstractmethod
    def status(self) -> ESP32SourceStatus: ...


class _Throttle:
    """Fire at most once per `interval_s` -- for logs that would otherwise repeat every poll."""

    def __init__(self, interval_s: float) -> None:
        self._interval = interval_s
        self._last = 0.0

    def ready(self, now: float) -> bool:
        if now - self._last >= self._interval:
            self._last = now
            return True
        return False

    def reset(self) -> None:
        self._last = 0.0


class ESP32Source(ProcessedFrameSource):
    """See module docstring.

    Args:
        settings: config (`Settings.esp32_*`). Defaults to `common.config.get_settings()`.
        transport: inject a concrete `ESP32Transport` (tests, or a real one once specified).
            When `None`, built from `Settings.esp32_transport` via `transport.build_transport`
            (`"mock"` -> scripted; unset/unknown -> `ESP32ConfigurationError` at `connect()`).
        deserializer: how to turn received bytes into an `STM32ProcessedFrame`. Defaults to
            `JsonProcessedFrameCodec` (the Phase-2 reference codec) -- replace with the real
            wire codec once the hardware team provides one.
        now_fn: injectable clock, for deterministic tests.
        max_messages_per_poll: drain at most this many messages per `poll()` so a burst backlog
            can't starve the caller's loop; the newest wins anyway.
    """

    def __init__(
        self,
        settings: Settings | None = None,
        *,
        transport: ESP32Transport | None = None,
        deserializer: ProcessedFrameDeserializer | None = None,
        now_fn=time.time,
        max_messages_per_poll: int = 32,
    ) -> None:
        self._settings = settings or get_settings()
        self._injected_transport = transport
        self._transport: ESP32Transport | None = transport
        self._deserializer: ProcessedFrameDeserializer = deserializer or JsonProcessedFrameCodec()
        self._now = now_fn
        self._max_messages_per_poll = max_messages_per_poll

        self._state = ESP32SourceState.UNAVAILABLE
        self._reason: str | None = "not connected"
        self.session_id = str(uuid.uuid4())

        self._seq = SequenceValidator(modulus=self._settings.esp32_sequence_modulus)
        self._latest_frame: STM32ProcessedFrame | None = None
        self._last_returned_frame_id: int | None = None
        self._last_frame_at: float | None = None
        self._connected_at: float | None = None

        self._frames_received = 0
        self._frames_rejected = 0
        self._dropped_frames = 0
        self._sequence_gaps = 0
        self._reconnect_attempts = 0
        self._recv_rate_hz: float | None = None

        self._backoff_s = 0.0
        self._next_reconnect_at = 0.0

        self._log_stale = _Throttle(2.0)
        self._log_rejected = _Throttle(2.0)
        self._log_gap = _Throttle(1.0)
        self._log_frame = _Throttle(2.0)

    # --- lifecycle ---------------------------------------------------------------------------

    def connect(self) -> None:
        """Open the transport and start a fresh session. Raises `ESP32ConfigurationError` if the
        transport is not configured -- never falls back to simulation."""
        if self._transport is None:
            try:
                self._transport = build_transport(self._settings)
            except ValueError as e:
                self._state = ESP32SourceState.UNAVAILABLE
                self._reason = f"ESP32 transport not configured: {e}"
                raise ESP32ConfigurationError(self._reason) from e

        self._begin_session(reason="connecting")
        try:
            self._transport.open()
        except ESP32TransportError as e:
            self._state = ESP32SourceState.DISCONNECTED
            self._reason = f"ESP32 transport failed to open: {e}"
            self._schedule_reconnect()
            logger.warning("[ESP32] transport open failed: %s (will retry).", e)
            return
        self._connected_at = self._now()
        self._state = ESP32SourceState.CONNECTING
        self._reason = "waiting for first processed frame from ESP32"
        logger.info("[ESP32] connected -- session %s, transport %s.", self.session_id, type(self._transport).__name__)

    def disconnect(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001 -- best-effort teardown
                pass
        self._state = ESP32SourceState.DISCONNECTED
        self._reason = "disconnected by caller"
        logger.info("[ESP32] disconnected (session %s, %d frame(s) received).", self.session_id, self._frames_received)

    def is_connected(self) -> bool:
        return (
            self._transport is not None
            and self._transport.is_open()
            and self._state in (ESP32SourceState.CONNECTED, ESP32SourceState.STALE, ESP32SourceState.CONNECTING)
        )

    # --- polling ---------------------------------------------------------------------------

    def poll(self) -> STM32ProcessedFrame | None:
        """Drain whatever the transport has, update health, and return the newest fresh frame
        that has not been returned yet -- or `None` (no new/fresh data; inspect `status`)."""
        now = self._now()

        if self._transport is None or not self._transport.is_open():
            self._attempt_reconnect(now)
            if self._transport is None or not self._transport.is_open():
                return None

        # A transport failure mid-drain does NOT discard frames that were already read this poll:
        # they are genuine, fresh data. We return the newest one (if any), then surface the
        # disconnect so the NEXT poll reports UNAVAILABLE.
        failure_reason = self._drain(now)
        if failure_reason is None:
            self._refresh_freshness(now)

        result: STM32ProcessedFrame | None = None
        if self._latest_frame is not None and (failure_reason is not None or self._state is ESP32SourceState.CONNECTED):
            if self._latest_frame.metadata.frame_id != self._last_returned_frame_id:
                self._last_returned_frame_id = self._latest_frame.metadata.frame_id
                result = self._latest_frame

        if failure_reason is not None:
            self._on_transport_failure(failure_reason)
        return result

    @property
    def status(self) -> ESP32SourceStatus:
        now = self._now()
        age = None if self._last_frame_at is None else max(0.0, now - self._last_frame_at)
        return ESP32SourceStatus(
            state=self._state,
            reason=self._reason,
            session_id=self.session_id,
            connected_at=self._connected_at,
            last_frame_at=self._last_frame_at,
            last_frame_timestamp=None if self._latest_frame is None else self._latest_frame.metadata.timestamp,
            last_frame_id=None if self._latest_frame is None else self._latest_frame.metadata.frame_id,
            last_sequence_number=None if self._latest_frame is None else self._latest_frame.metadata.sequence_number,
            frame_age_s=age,
            frames_received=self._frames_received,
            frames_rejected=self._frames_rejected,
            dropped_frames=self._dropped_frames,
            sequence_gaps=self._sequence_gaps,
            reconnect_attempts=self._reconnect_attempts,
            receive_rate_hz=self._recv_rate_hz,
        )

    # --- internals -----------------------------------------------------------------------------

    def _begin_session(self, *, reason: str) -> None:
        """New session identity + cleared per-session state -- called on connect and on every
        successful reconnect, so a reconnected stream never carries the previous run's sequence
        expectations, counters, or last frame forward."""
        self.session_id = str(uuid.uuid4())
        self._seq = SequenceValidator(modulus=self._settings.esp32_sequence_modulus)
        self._latest_frame = None
        self._last_returned_frame_id = None
        self._last_frame_at = None
        self._frames_received = 0
        self._frames_rejected = 0
        self._dropped_frames = 0
        self._sequence_gaps = 0
        self._recv_rate_hz = None
        self._reason = reason
        for t in (self._log_stale, self._log_rejected, self._log_gap, self._log_frame):
            t.reset()

    def _drain(self, now: float) -> str | None:
        """Read up to `max_messages_per_poll` messages. Returns a human-readable reason string if
        the transport failed mid-drain (frames read before the failure are still ingested), else
        `None`."""
        assert self._transport is not None
        for _ in range(self._max_messages_per_poll):
            try:
                raw = self._transport.receive(self._settings.esp32_read_timeout_s)
            except ESP32TransportError as e:
                return f"ESP32 disconnected: {e}"
            if raw is None:
                break
            self._ingest(raw, now)
        return None

    def _ingest(self, raw: bytes, now: float) -> None:
        try:
            frame = self._deserializer.decode(raw)
        except STM32ContractVersionError as e:
            self._frames_rejected += 1
            self._reason = f"protocol error: {e}"
            if self._log_rejected.ready(now):
                logger.warning("[ESP32] frame rejected -- protocol/version error: %s", e)
            return
        except STM32ProcessedFrameError as e:
            self._frames_rejected += 1
            self._reason = "invalid frame"
            if self._log_rejected.ready(now):
                logger.warning("[ESP32] frame rejected -- invalid/malformed: %s", e)
            return

        seq_result = self._seq.validate(frame.metadata.sequence_number)
        if seq_result.is_duplicate_or_out_of_order:
            if self._log_rejected.ready(now):
                logger.warning(
                    "[ESP32] dropping duplicate/out-of-order frame seq=%d.",
                    frame.metadata.sequence_number,
                )
            return
        if seq_result.dropped_count > 0:
            self._dropped_frames += seq_result.dropped_count
            self._sequence_gaps += 1
            if self._log_gap.ready(now):
                logger.warning(
                    "[ESP32] sequence gap -- %d frame(s) missing before seq=%d (total dropped=%d).",
                    seq_result.dropped_count, frame.metadata.sequence_number, self._dropped_frames,
                )
            # still accepted -- real-time visualisation needs the newest state, not every frame

        # frame_id monotonicity is a second, independent guard (Phase-2 metadata.frame_id).
        if (
            self._latest_frame is not None
            and frame.metadata.frame_id <= self._latest_frame.metadata.frame_id
            and not seq_result.is_first
        ):
            if self._log_rejected.ready(now):
                logger.warning(
                    "[ESP32] dropping frame with non-increasing frame_id=%d (last=%d).",
                    frame.metadata.frame_id, self._latest_frame.metadata.frame_id,
                )
            return

        if self._last_frame_at is not None:
            dt = now - self._last_frame_at
            if dt > 0:
                inst = 1.0 / dt
                self._recv_rate_hz = inst if self._recv_rate_hz is None else 0.7 * self._recv_rate_hz + 0.3 * inst
        self._latest_frame = frame
        self._last_frame_at = now
        self._frames_received += 1
        if self._log_frame.ready(now):
            logger.info(
                "[ESP32] frame received -- frame_id=%d seq=%d objects=%d risk=%s (received=%d).",
                frame.metadata.frame_id, frame.metadata.sequence_number,
                len(frame.objects), frame.risk.overall_risk.value, self._frames_received,
            )

    def _refresh_freshness(self, now: float) -> None:
        if self._transport is None or not self._transport.is_open():
            return
        if self._latest_frame is None:
            self._state = ESP32SourceState.CONNECTING
            # keep a concrete rejection reason (invalid/protocol) visible rather than masking it
            # with the generic "waiting" message
            if self._frames_rejected == 0:
                self._reason = "waiting for first processed frame from ESP32"
            # still nothing after the heartbeat timeout -> treat as a dead link
            if self._connected_at is not None and now - self._connected_at > self._settings.esp32_heartbeat_timeout_s:
                self._on_transport_failure(
                    f"timeout: no processed frame received within {self._settings.esp32_heartbeat_timeout_s:.0f}s of connecting"
                )
            return

        age = now - (self._last_frame_at or now)
        if age > self._settings.esp32_heartbeat_timeout_s:
            self._on_transport_failure(f"timeout: no processed frame for {age:.1f}s")
            return
        if age > self._settings.esp32_frame_stale_after_s:
            if self._state is not ESP32SourceState.STALE and self._log_stale.ready(now):
                logger.warning("[ESP32] stale -- newest processed frame is %.1fs old (threshold %.1fs).",
                               age, self._settings.esp32_frame_stale_after_s)
            self._state = ESP32SourceState.STALE
            self._reason = f"stale data: newest processed frame is {age:.1f}s old"
            return

        self._state = ESP32SourceState.CONNECTED
        self._reason = None

    def _on_transport_failure(self, reason: str) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass
        self._state = ESP32SourceState.DISCONNECTED
        self._reason = reason
        logger.warning("[ESP32] link down: %s", reason)
        self._schedule_reconnect()

    def _schedule_reconnect(self) -> None:
        self._backoff_s = (
            self._settings.esp32_reconnect_initial_backoff_s
            if self._backoff_s == 0.0
            else min(self._settings.esp32_reconnect_max_backoff_s, self._backoff_s * 2)
        )
        self._next_reconnect_at = self._now() + self._backoff_s

    def _attempt_reconnect(self, now: float) -> None:
        max_attempts = self._settings.esp32_max_reconnect_attempts
        if max_attempts is not None and self._reconnect_attempts >= max_attempts:
            if self._state is not ESP32SourceState.UNAVAILABLE:
                logger.error("[ESP32] giving up -- reconnection exhausted after %d attempt(s).", self._reconnect_attempts)
            self._state = ESP32SourceState.UNAVAILABLE
            self._reason = f"reconnection exhausted after {self._reconnect_attempts} attempt(s)"
            return
        if now < self._next_reconnect_at:
            self._state = ESP32SourceState.RECONNECTING
            return
        if self._transport is None:
            return

        self._reconnect_attempts += 1
        self._state = ESP32SourceState.RECONNECTING
        logger.info("[ESP32] reconnect attempt %d ...", self._reconnect_attempts)
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._transport.open()
        except ESP32TransportError as e:
            self._reason = f"reconnect failed: {e}"
            self._schedule_reconnect()
            logger.warning("[ESP32] reconnect attempt %d failed: %s", self._reconnect_attempts, e)
            return

        self._begin_session(reason="reconnected -- waiting for first processed frame")
        self._connected_at = now
        self._backoff_s = 0.0
        self._state = ESP32SourceState.CONNECTING
        logger.info("[ESP32] reconnected -- new session %s.", self.session_id)


__all__ = [
    "ESP32Source",
    "ESP32SourceState",
    "ESP32SourceStatus",
    "ProcessedFrameSource",
]

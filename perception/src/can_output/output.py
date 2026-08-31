"""`STM32CANOutput` -- the STM32 -> Vehicle-ECU CAN transmit stage, modelled in software.

    STM32ProcessedFrame  --submit()-->  [validate + encode]  --queue-->  tick()  --> CANTransport --> ECU

**This is a software model of a stage the STM32 firmware performs in the real vehicle.** It runs
against the Phase-2 `STM32ProcessedFrame` contract so the design, validation, transmission
control, and error handling are exercised now, and so the firmware team can adopt the message
catalogue / mapping directly. The Edge does not drive a real vehicle CAN bus.

Transmission control (requirement 5):
  * **periodic** -- each message with `transmission_mode` PERIODIC / PERIODIC_AND_EVENT is due
    every `cycle_time_ms` (or `config`-level default); `tick()` emits only what is due.
  * **event** -- SAFETY_STATE (PERIODIC_AND_EVENT) is also flagged "send now" whenever
    `overall_risk` changes between submits.
  * a hard `config.max_transmit_rate_hz` cap bounds total frames/sec -- never "indefinitely at an
    arbitrary rate".
  * `sequence_number` -- the output's own rolling tx counter (wrap = `config.sequence_modulus`).

Error handling (requirement 7): `CANBusError` -> BUS_OFF + exponential-backoff recovery;
`CANTransmitError` -> counted, frame re-queued once; bounded queue -> drop-oldest + counted;
health is exposed via `.health`.

**Independence (requirement 8):** this module imports nothing from `datasources.esp32`. The two
outputs consume the same `STM32ProcessedFrame` and share no state.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum

from common.logging import get_logger
from models.stm32_processed import STM32ProcessedFrame

from .encoder import CANFrameEncoder
from .errors import CANBusError, CANConfigurationError, CANTransmitError
from .frame import CANFrame
from .message_spec import CANMessageSpec, CANOutputConfig, CANTransmissionMode
from .payload import extract_payloads
from .transport import CANTransport

logger = get_logger(__name__)


class CANOutputState(str, Enum):
    DISABLED = "disabled"          # stop()ped / never started
    UNAVAILABLE = "unavailable"    # transport won't open / config not ready
    READY = "ready"                # open, nothing sent yet
    ACTIVE = "active"              # sending
    BUS_OFF = "bus_off"            # bus-level failure -- recovering
    RECOVERING = "recovering"      # between recovery attempts
    ERROR = "error"


@dataclass
class CANOutputHealth:
    state: CANOutputState
    reason: str | None
    encoded: bool                  # True = bit-packed frames; False = semantic-only (layout PENDING)
    frames_built: int = 0
    frames_sent: int = 0
    frames_dropped: int = 0        # queue overflow
    validation_failures: int = 0
    tx_errors: int = 0             # recoverable single-frame failures
    bus_off_events: int = 0
    reconnect_attempts: int = 0
    queue_depth: int = 0
    last_tx_at: float | None = None
    last_error: str | None = None
    pending_hardware_parameters: dict = field(default_factory=dict)


@dataclass
class _Pending:
    frame: CANFrame
    retries: int = 0


class STM32CANOutput:
    def __init__(self, config: CANOutputConfig, transport: CANTransport, *, now_fn=time.time) -> None:
        self._config = config
        self._transport = transport
        self._now = now_fn
        self._encoder = CANFrameEncoder(config, now_fn=now_fn)

        self._state = CANOutputState.DISABLED
        self._reason: str | None = "not started"
        self._queue: deque[_Pending] = deque()
        self._tx_seq = 0

        # per-message "next due" wall-clock time (periodic scheduling)
        self._next_due: dict[str, float] = {}
        self._event_pending: set[str] = set()
        self._last_overall_risk: str | None = None

        self._backoff_s = 0.0
        self._next_recovery_at = 0.0
        self._last_frame_sent_at = 0.0  # only advanced on an ACTUAL send -- drives the rate cap

        self._h = CANOutputHealth(
            state=self._state, reason=self._reason,
            encoded=all(m.is_fully_specified for m in config.messages) and bool(config.messages),
            pending_hardware_parameters=config.pending_hardware_parameters(),
        )

    # --- lifecycle --------------------------------------------------------------------------

    def start(self) -> None:
        if not self._config.messages:
            raise CANConfigurationError("CANOutputConfig has no messages -- nothing to transmit.")
        if self._config.require_fully_specified and not self._config.is_ready_for_hardware:
            raise CANConfigurationError(
                "require_fully_specified=True but the CAN configuration is incomplete; still PENDING: "
                + "; ".join(f"{k}: {', '.join(v)}" for k, v in self._config.pending_hardware_parameters().items())
            )
        try:
            self._transport.open()
        except CANBusError as e:
            self._set_state(CANOutputState.UNAVAILABLE, f"CAN transport unavailable: {e}")
            self._schedule_recovery()
            logger.warning("[SIM-CAN] transport open failed: %s (will retry).", e)
            return
        self._set_state(CANOutputState.READY, None)
        logger.info(
            "[SIM-CAN] STM32 CAN output started -- backend=%s, encoded=%s%s.",
            type(self._transport).__name__, self._h.encoded,
            "" if self._h.encoded else " (semantic-only; CAN layout PENDING hardware spec)",
        )

    def stop(self) -> None:
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001 -- best-effort
            pass
        self._set_state(CANOutputState.DISABLED, "stopped")

    # --- ingest --------------------------------------------------------------------------

    def submit(self, frame: STM32ProcessedFrame) -> None:
        """Build this frame's CAN messages, validate, enqueue. A message whose signals fail
        validation is dropped and counted; the others still go. Event-mode SAFETY_STATE is
        flagged for immediate send when `overall_risk` changed since the last submit."""
        payloads = extract_payloads(frame)
        seq = self._next_tx_seq()
        frames, errors = self._encoder.encode_frame(payloads, sequence_number=seq)
        self._h.frames_built += len(frames)
        if errors:
            self._h.validation_failures += len(errors)
            self._h.last_error = errors[-1]
            for msg in errors:
                logger.warning("[SIM-CAN] signal validation failed, message dropped: %s", msg)

        risk = frame.risk.overall_risk.value
        if risk != self._last_overall_risk:
            self._event_pending.add("SAFETY_STATE")
            self._last_overall_risk = risk

        for f in frames:
            self._enqueue(f)

    # --- pump --------------------------------------------------------------------------

    def tick(self, now: float | None = None) -> int:
        """Send whatever is due (periodic schedule + event flags), rate-capped. Returns the
        number of frames actually transmitted this call. Also drives bus recovery."""
        now = self._now() if now is None else now

        if self._state in (CANOutputState.UNAVAILABLE, CANOutputState.BUS_OFF, CANOutputState.RECOVERING):
            self._attempt_recovery(now)
            if not self._transport.is_open():
                return 0

        budget = self._rate_budget(now)
        sent = 0
        while budget > 0 and self._queue:
            pend = self._queue[0]
            if not self._due(pend.frame, now):
                # nothing at the head is due yet -> stop (queue is roughly time-ordered)
                break
            self._queue.popleft()
            try:
                self._transport.send(pend.frame)
            except CANBusError as e:
                self._queue.appendleft(pend)  # keep it for after recovery
                self._on_bus_error(str(e))
                return sent
            except CANTransmitError as e:
                self._h.tx_errors += 1
                self._h.last_error = str(e)
                if pend.retries == 0:
                    pend.retries += 1
                    self._queue.append(pend)  # one retry, at the back
                else:
                    self._h.frames_dropped += 1
                logger.warning("[SIM-CAN] transmit failed (%s) -- %s.", e, "re-queued once" if pend.retries == 1 else "dropped")
                budget -= 1
                continue
            sent += 1
            budget -= 1
            self._h.frames_sent += 1
            self._h.last_tx_at = now
            self._last_frame_sent_at = now
            self._mark_transmitted(pend.frame, now)
            if self._state is not CANOutputState.ACTIVE:
                self._set_state(CANOutputState.ACTIVE, None)

        self._h.queue_depth = len(self._queue)
        return sent

    # --- health --------------------------------------------------------------------------

    @property
    def health(self) -> CANOutputHealth:
        self._h.state = self._state
        self._h.reason = self._reason
        self._h.queue_depth = len(self._queue)
        self._h.pending_hardware_parameters = self._config.pending_hardware_parameters()
        return self._h

    @property
    def state(self) -> CANOutputState:
        return self._state

    # --- internals --------------------------------------------------------------------------

    def _next_tx_seq(self) -> int:
        s = self._tx_seq
        self._tx_seq += 1
        if self._config.sequence_modulus:
            self._tx_seq %= self._config.sequence_modulus
        return s if not self._config.sequence_modulus else (s % self._config.sequence_modulus)

    def _enqueue(self, frame: CANFrame) -> None:
        if len(self._queue) >= self._config.queue_max_frames:
            self._queue.popleft()  # drop-oldest (streaming-server precedent)
            self._h.frames_dropped += 1
            self._h.last_error = "queue overflow -- oldest frame dropped"
            logger.warning("[SIM-CAN] queue overflow (max=%d) -- dropped oldest.", self._config.queue_max_frames)
        self._queue.append(_Pending(frame))
        self._h.queue_depth = len(self._queue)

    def _spec(self, name: str) -> CANMessageSpec | None:
        return self._config.message(name)

    def _due(self, frame: CANFrame, now: float) -> bool:
        spec = self._spec(frame.message_name)
        if spec is None:
            return True
        if spec.name in self._event_pending:
            return True
        if spec.transmission_mode is CANTransmissionMode.EVENT:
            return False  # pure event message: only sends when flagged
        cycle_ms = spec.cycle_time_ms or self._config.default_cycle_time_ms
        if cycle_ms is None:
            return True  # no period known yet -> send as it arrives (still rate-capped overall)
        return now >= self._next_due.get(spec.name, 0.0)

    def _mark_transmitted(self, frame: CANFrame, now: float) -> None:
        spec = self._spec(frame.message_name)
        if spec is None:
            return
        self._event_pending.discard(spec.name)
        cycle_ms = spec.cycle_time_ms or self._config.default_cycle_time_ms
        if cycle_ms is not None:
            self._next_due[spec.name] = now + cycle_ms / 1000.0

    def _rate_budget(self, now: float) -> int:
        """How many frames `tick()` may transmit right now. `0` until at least `1 / max_rate`
        seconds have elapsed since the last actual send; then one per elapsed interval, hard-
        capped at ~one second's worth so a long idle can never license an unbounded burst."""
        rate = self._config.max_transmit_rate_hz
        min_interval = 1.0 / rate
        if self._last_frame_sent_at <= 0.0:
            return max(1, int(rate))  # first send of the run
        dt = now - self._last_frame_sent_at
        if dt < min_interval:
            return 0
        return min(int(dt / min_interval), max(1, int(rate)))

    def _on_bus_error(self, reason: str) -> None:
        self._h.bus_off_events += 1
        self._h.last_error = reason
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001
            pass
        self._set_state(CANOutputState.BUS_OFF, f"CAN bus error: {reason}")
        self._schedule_recovery()
        logger.warning("[SIM-CAN] bus error: %s -- recovering with backoff.", reason)

    def _schedule_recovery(self) -> None:
        self._backoff_s = (
            self._config.bus_recovery_initial_backoff_s if self._backoff_s == 0.0
            else min(self._config.bus_recovery_max_backoff_s, self._backoff_s * 2)
        )
        self._next_recovery_at = self._now() + self._backoff_s

    def _attempt_recovery(self, now: float) -> None:
        if now < self._next_recovery_at:
            self._set_state(CANOutputState.RECOVERING, self._reason)
            return
        self._h.reconnect_attempts += 1
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._transport.open()
        except CANBusError as e:
            self._reason = f"recovery failed: {e}"
            self._schedule_recovery()
            logger.warning("[SIM-CAN] recovery attempt %d failed: %s", self._h.reconnect_attempts, e)
            return
        self._backoff_s = 0.0
        self._set_state(CANOutputState.READY, None)
        logger.info("[SIM-CAN] bus recovered after %d attempt(s).", self._h.reconnect_attempts)

    def _set_state(self, state: CANOutputState, reason: str | None) -> None:
        self._state = state
        self._reason = reason
        self._h.state = state
        self._h.reason = reason


__all__ = ["STM32CANOutput", "CANOutputState", "CANOutputHealth"]

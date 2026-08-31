"""Replaceable CAN transport -- where `CANFrame`s actually go.

**No physical CAN interface is implemented or required.** The abstraction (`CANTransport`) is
what a real backend (e.g. a `python-can` `Bus`) plugs into later; until then two software sinks
ship:

* `MockCANTransport` -- records every frame; fully injectable failure modes (bus-off, transmit
  failure, drop). For tests. Labelled **SIMULATED CAN OUTPUT** on open.
* `LoggingCANTransport` -- writes each frame as a `[SIM-CAN]` log line. A transparent, harmless
  sink for demos / the optional edge-runner hook.

`build_can_transport(settings)` resolves `Settings.can_output_backend` (`"mock"` | `"logging"`;
a real name is deliberately NOT accepted -- the CAN bus parameters are unspecified).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum

from common.logging import get_logger

from .errors import CANBusError, CANConfigurationError, CANTransmitError
from .frame import CANFrame

logger = get_logger(__name__)


class CANTransportState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    BUS_OFF = "bus_off"
    ERROR = "error"


class CANTransport(ABC):
    """open / close / is_open / send(frame). `send` raises `CANBusError` for a bus-level failure
    (bus-off, unplugged) and `CANTransmitError` for a recoverable single-frame failure."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def send(self, frame: CANFrame) -> None: ...

    @property
    @abstractmethod
    def state(self) -> CANTransportState: ...


class MockCANTransport(CANTransport):
    """In-memory transport for tests. **SIMULATED CAN OUTPUT -- not a real bus.**

    Test knobs:
      * `fail_next_sends(n)`         -> next n `send()` calls raise `CANTransmitError`
      * `set_bus_off()` / `clear_bus_off()` -> `send()`/`open()` raise `CANBusError` until cleared
      * `unavailable`               -> `open()` raises `CANBusError` (interface missing)
    """

    def __init__(self) -> None:
        self.sent: list[CANFrame] = []
        self.open_count = 0
        self._state = CANTransportState.CLOSED
        self._fail_sends = 0
        self._bus_off = False
        self.unavailable = False

    def open(self) -> None:
        if self.unavailable:
            self._state = CANTransportState.ERROR
            raise CANBusError("SIMULATED CAN: interface unavailable.")
        if self._bus_off:
            self._state = CANTransportState.BUS_OFF
            raise CANBusError("SIMULATED CAN: bus-off, cannot open.")
        self._state = CANTransportState.OPEN
        self.open_count += 1
        logger.warning("[SIM-CAN] *** SIMULATED CAN OUTPUT opened -- NOT a real CAN bus. ***")

    def close(self) -> None:
        self._state = CANTransportState.CLOSED

    def is_open(self) -> bool:
        return self._state is CANTransportState.OPEN

    def send(self, frame: CANFrame) -> None:
        if self._bus_off:
            self._state = CANTransportState.BUS_OFF
            raise CANBusError("SIMULATED CAN: bus-off during send.")
        if not self.is_open():
            raise CANBusError("SIMULATED CAN: send() while not open.")
        if self._fail_sends > 0:
            self._fail_sends -= 1
            raise CANTransmitError("SIMULATED CAN: injected transmit failure.")
        self.sent.append(frame)

    @property
    def state(self) -> CANTransportState:
        return self._state

    # --- test knobs ---
    def fail_next_sends(self, n: int) -> None:
        self._fail_sends = n

    def set_bus_off(self) -> None:
        self._bus_off = True
        self._state = CANTransportState.BUS_OFF

    def clear_bus_off(self) -> None:
        self._bus_off = False


class LoggingCANTransport(CANTransport):
    """Writes each frame as an `[SIM-CAN]` log line. Never fails. Demo / hook sink only."""

    def __init__(self) -> None:
        self._open = False
        self.count = 0

    def open(self) -> None:
        self._open = True
        logger.warning("[SIM-CAN] *** SIMULATED CAN OUTPUT (logging sink) -- NOT a real CAN bus. ***")

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def send(self, frame: CANFrame) -> None:
        if not self._open:
            raise CANBusError("logging CAN sink: send() while not open.")
        self.count += 1
        logger.info("[SIM-CAN] %s", frame)

    @property
    def state(self) -> CANTransportState:
        return CANTransportState.OPEN if self._open else CANTransportState.CLOSED


def build_can_transport(settings) -> CANTransport:
    backend = (getattr(settings, "can_output_backend", None) or "mock").strip().lower()
    if backend == "mock":
        return MockCANTransport()
    if backend == "logging":
        return LoggingCANTransport()
    raise CANConfigurationError(
        f"can_output_backend={backend!r} is not implemented. The CAN bus parameters (bitrate, "
        f"interface, IDs, layout) are unspecified; only 'mock' and 'logging' software sinks "
        f"exist. Provide a real can_output.transport.CANTransport subclass once the CAN/DBC "
        f"spec is available."
    )


__all__ = [
    "CANTransport", "CANTransportState",
    "MockCANTransport", "LoggingCANTransport", "build_can_transport",
]

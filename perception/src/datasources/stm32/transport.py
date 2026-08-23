"""Byte-transport abstraction the STM32 connection manager reads/writes through.

`Transport` is deliberately a separate seam from `STM32ConnectionManager` (which owns
reconnect/backoff/health) so the actual I/O (`SerialTransport`, pyserial-backed) can be swapped
for a fake in tests without needing a real serial port or hardware -- see `test_stm32_transport.py`
and `test_stm32_source.py`'s use of an in-memory fake transport.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from .errors import STM32ConnectionError


class Transport(ABC):
    """Minimal byte-stream transport: open/close, non-blocking-ish bounded reads, writes."""

    @abstractmethod
    def open(self) -> None:
        """Open the underlying channel. Raises `STM32ConnectionError` on failure."""

    @abstractmethod
    def close(self) -> None:
        """Close the underlying channel. Safe to call even if not open."""

    @abstractmethod
    def is_open(self) -> bool:
        ...

    @abstractmethod
    def read(self, size: int) -> bytes:
        """Reads up to `size` bytes, waiting at most this transport's configured timeout.
        Returns `b""` on timeout (no data available) -- never raises for "nothing arrived yet"."""

    @abstractmethod
    def write(self, data: bytes) -> int:
        """Writes `data`, returns the number of bytes actually written."""


class SerialTransport(Transport):
    """Real UART transport, backed by `pyserial` (`serial.Serial`).

    `pyserial` is imported lazily (inside `open()`, not at module import time) so that importing
    `datasources.stm32` never requires it to be installed unless a caller actually opens a serial
    connection -- the same "deferred import" pattern `scripts/sensor_source.py` already uses for
    the `simulator` package, for the analogous reason (a pure-simulation deployment has no need
    for this dependency at all).
    """

    def __init__(self, port: str, baudrate: int, timeout_s: float) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._serial = None

    def open(self) -> None:
        try:
            import serial  # noqa: PLC0415 -- deliberate deferred import, see class docstring
        except ImportError as e:
            raise STM32ConnectionError(
                "pyserial is not installed. Install it (`pip install pyserial`, or `pip install "
                "-e './perception[dev]'` which now includes it) to use STM32Source in hardware "
                "mode."
            ) from e

        try:
            self._serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s)
        except serial.SerialException as e:
            raise STM32ConnectionError(f"Could not open serial port {self.port!r} at {self.baudrate} baud: {e}") from e

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 -- best-effort close, never raise on teardown
                pass
        self._serial = None

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read(self, size: int) -> bytes:
        if self._serial is None:
            raise STM32ConnectionError("SerialTransport.read() called before open().")
        try:
            return self._serial.read(size)
        except Exception as e:  # noqa: BLE001 -- any pyserial I/O failure during a live read is a connection problem
            raise STM32ConnectionError(f"Serial read failed on {self.port!r}: {e}") from e

    def write(self, data: bytes) -> int:
        if self._serial is None:
            raise STM32ConnectionError("SerialTransport.write() called before open().")
        try:
            return self._serial.write(data)
        except Exception as e:  # noqa: BLE001
            raise STM32ConnectionError(f"Serial write failed on {self.port!r}: {e}") from e

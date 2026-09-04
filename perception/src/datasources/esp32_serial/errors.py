"""Exceptions for the ESP32 USB-serial raw-measurement link (`datasources.esp32_serial`).

Deliberately separate from `datasources.esp32.errors` (the Wi-Fi *processed-frame* gateway) and
from `datasources.stm32.errors` (the unconfigured binary-UART scaffolding): this package speaks a
different, fully-specified protocol -- one ASCII `A:<deg> , D:<mm>` measurement per line -- and a
caller must be able to catch its failures without also catching those two unrelated links'.

`ESP32SerialError` is the broad base meaning "anything about the ESP32 USB serial link went
wrong". Subtypes distinguish the cases that warrant different handling:

* `ESP32SerialConfigurationError` -- the port/baud rate is unusable or `pyserial` is missing.
  Raised at `connect()` time, before any port is opened, and never retried: retrying cannot fix a
  configuration problem. Raised **instead of ever falling back to simulated data**, per this
  project's "LIVE mode must never generate simulation points" rule.
* `ESP32SerialConnectionError` -- opening the port failed, or reconnection was exhausted
  (`esp32_serial_max_reconnect_attempts`). The reader reports DISCONNECTED with a reason.
* `ESP32SerialTimeoutError` -- no *complete* scan arrived within `esp32_serial_scan_timeout_s`.
  Recoverable and expected during a hardware hiccup: the Edge loop logs it, publishes an ERROR
  message to connected clients, and keeps running rather than crashing (see
  `scripts/serve_unity_bridge.py`'s per-scan error handling).
"""

from __future__ import annotations


class ESP32SerialError(RuntimeError):
    """Base class for every error raised by `datasources.esp32_serial`."""


class ESP32SerialConfigurationError(ESP32SerialError):
    """The serial link is misconfigured (unusable port/baud rate, or `pyserial` not installed).

    Raised by `connect()` before opening the port, and never retried.
    """


class ESP32SerialConnectionError(ESP32SerialError):
    """Opening the COM port failed, or reconnection attempts were exhausted."""


class ESP32SerialTimeoutError(ESP32SerialError):
    """No complete 360 degree scan was assembled within the configured scan timeout.

    Recoverable -- the caller should log/report and continue, not abort the run.
    """


__all__ = [
    "ESP32SerialError",
    "ESP32SerialConfigurationError",
    "ESP32SerialConnectionError",
    "ESP32SerialTimeoutError",
]

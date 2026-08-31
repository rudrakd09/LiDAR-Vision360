"""Exceptions for the ESP32 gateway link (`datasources.esp32`).

`ESP32Error` is the broad base a caller can catch to mean "anything about the STM32->ESP32->Edge
link went wrong". Subtypes distinguish the cases that warrant different handling:

* `ESP32ConfigurationError` -- the transport is not configured (`Settings.esp32_transport` unset,
  or an unknown value). Raised at `connect()` time, before any link is opened, and never caught
  and retried -- retrying cannot fix a config problem. Mirrors `STM32ConfigurationError`; in
  particular it is raised **instead of ever falling back to simulation data** in hardware mode.
* `ESP32TransportError` -- the transport itself failed (link dropped, socket error, ...). The
  `ESP32Source` catches this, marks itself DISCONNECTED with a reason, and schedules a reconnect.
* `ESP32ConnectionError` -- reconnection was attempted and exhausted, or an explicit connect
  failed. The source goes UNAVAILABLE.
"""

from __future__ import annotations


class ESP32Error(RuntimeError):
    """Base class for every error raised by `datasources.esp32`."""


class ESP32ConfigurationError(ESP32Error):
    """The ESP32 transport is not configured (or is set to an unknown value).

    Raised by `ESP32Source.connect()` before opening any link, and never retried. In hardware
    mode this is raised rather than silently substituting simulated data -- per this project's
    "`DATA_SOURCE=hardware` must never generate simulation objects" rule.
    """


class ESP32TransportError(ESP32Error):
    """The underlying transport failed mid-stream (link lost, socket error, framing failure).

    Recoverable: `ESP32Source` marks itself DISCONNECTED with a human-readable reason and
    reconnects with backoff.
    """


class ESP32ConnectionError(ESP32Error):
    """An explicit connect failed, or reconnection was exhausted (`esp32_max_reconnect_attempts`).
    The source transitions to UNAVAILABLE and reports the reason."""


__all__ = [
    "ESP32Error",
    "ESP32ConfigurationError",
    "ESP32TransportError",
    "ESP32ConnectionError",
]

"""Exception hierarchy for the STM32 hardware adapter (`datasources.stm32`).

Every error a real STM32 run can hit is one of these, so `scripts/serve_unity_bridge.py` (and any
other caller) can catch `STM32Error` broadly (mirroring the "one bad scan must not kill the whole
bridge" tolerance it already applies elsewhere) or a specific subtype when it needs to react
differently -- e.g. `STM32TimeoutError`/`STM32ConnectionError` are handled internally by
`STM32ConnectionManager`'s own reconnect logic, while `STM32ConfigurationError` is never caught
and retried, since retrying cannot fix a configuration problem.
"""

from __future__ import annotations


class STM32Error(RuntimeError):
    """Base class for every error raised by `datasources.stm32`."""


class STM32ConfigurationError(STM32Error):
    """Required wire-protocol configuration is missing or invalid.

    Raised instead of ever silently falling back to simulated/fabricated data -- per this
    project's explicit "`DATA_SOURCE=hardware` must never generate simulated objects" rule.
    Always raised BEFORE any serial port is opened, so a misconfigured hardware run fails loudly
    and immediately rather than opening a port it cannot correctly use.
    """


class STM32ConnectionError(STM32Error):
    """The transport (serial port) could not be opened, or reconnection was exhausted."""


class STM32TimeoutError(STM32Error):
    """No data was received from the STM32 within the configured read timeout."""


class STM32ProtocolError(STM32Error):
    """One frame failed validation (CRC/checksum mismatch, malformed header, unknown
    message-type, ...). Callers are expected to skip the offending frame and continue -- the same
    "one bad message does not kill the stream" tolerance `serve_unity_bridge.py` already applies
    to pipeline errors -- rather than treat this as fatal.
    """

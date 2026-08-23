"""STM32 hardware adapter internals (Phase 8 architecture scaffolding).

`datasources.stm32_source.STM32Source` (the public `SensorSource` implementation selected by
`Settings.data_source == "hardware"`) composes everything in this package:

- `transport` -- byte I/O (`SerialTransport`, pyserial-backed).
- `connection_manager` -- connect/disconnect/reconnect-with-backoff.
- `framing` -- turns a raw byte stream into discrete frames (`FrameCodec`).
- `crc` -- checksum algorithms for frame validation.
- `sequence` -- dropped-frame / duplicate detection (`SequenceValidator`).
- `health` -- per-channel connection/health tracking (`HealthMonitor`).
- `parsers` -- `LiDARMessageParser`/`RadarMessageParser` interfaces, converting one validated
  frame's payload into `LiDARPoint`s / a `RadarReading`.

Every piece here is independently unit-testable against synthetic byte sequences (see
`perception/tests/test_stm32_*.py`) without a real serial port or hardware attached.
"""

from .crc import ChecksumAlgorithm, checksum_length_bytes, compute_checksum, verify_checksum
from .errors import (
    STM32ConfigurationError,
    STM32ConnectionError,
    STM32Error,
    STM32ProtocolError,
    STM32TimeoutError,
)
from .framing import FrameCodec
from .health import ChannelHealth, HealthMonitor
from .parsers import (
    LiDARMessageParser,
    RadarMessageParser,
    UnconfiguredLiDARParser,
    UnconfiguredRadarParser,
    decode_header_field,
)
from .sequence import SequenceCheckResult, SequenceValidator
from .transport import SerialTransport, Transport
from .connection_manager import STM32ConnectionManager

__all__ = [
    "ChecksumAlgorithm",
    "checksum_length_bytes",
    "compute_checksum",
    "verify_checksum",
    "STM32Error",
    "STM32ConfigurationError",
    "STM32ConnectionError",
    "STM32TimeoutError",
    "STM32ProtocolError",
    "FrameCodec",
    "ChannelHealth",
    "HealthMonitor",
    "LiDARMessageParser",
    "RadarMessageParser",
    "UnconfiguredLiDARParser",
    "UnconfiguredRadarParser",
    "decode_header_field",
    "SequenceCheckResult",
    "SequenceValidator",
    "Transport",
    "SerialTransport",
    "STM32ConnectionManager",
]

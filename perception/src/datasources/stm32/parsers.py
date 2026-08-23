"""LiDAR/radar message parser interfaces, and the header-field decoding shared by both.

`LiDARMessageParser`/`RadarMessageParser` are the seam item 17/18 of the Phase 8 architecture ask
names: given one validated, complete frame's raw bytes, produce `LiDARPoint`s or a `RadarReading`.
Neither concrete "real" implementation exists yet -- the payload field layout (where in the frame
angle/distance/range/velocity live, at what width/scale) is exactly the kind of hardware-specific
detail this phase's instructions say not to guess. `UnconfiguredLiDARParser`/
`UnconfiguredRadarParser` are the default, always-installed implementations: calling `.parse()` on
either raises `STM32ConfigurationError` immediately, so `STM32Source` can NEVER silently return
fabricated points in hardware mode. Once the real spec is known, implement a new subclass of the
appropriate ABC (see docs/hardware-integration.md "Hardware integration checklist") and pass it to
`STM32Source(lidar_parser=..., radar_parser=...)` -- no other code needs to change.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from models.lidar import LiDARPoint
from models.radar import RadarReading

from .errors import STM32ConfigurationError


def decode_header_field(frame: bytes, *, offset: int, width: int, byte_order: str) -> int:
    """Reads an unsigned integer of `width` bytes at `offset` within `frame`, per `byte_order`
    (`"little"` | `"big"`). Shared by message-type and sequence-number extraction -- both are the
    same mechanical operation, just at different configured offsets/widths."""
    if offset < 0 or offset + width > len(frame):
        raise ValueError(f"Field at offset {offset} width {width} does not fit within a {len(frame)}-byte frame.")
    return int.from_bytes(frame[offset : offset + width], byteorder=byte_order)  # type: ignore[arg-type]


class LiDARMessageParser(ABC):
    """Converts one validated LiDAR-type frame's raw bytes into `LiDARPoint`s."""

    @abstractmethod
    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int) -> list[LiDARPoint]:
        ...


class RadarMessageParser(ABC):
    """Converts one validated radar-type frame's raw bytes into a `RadarReading`."""

    @abstractmethod
    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int, source_id: str) -> RadarReading:
        ...


_NOT_CONFIGURED_MESSAGE = (
    "STM32 protocol configuration incomplete: {modality} payload field layout required "
    "(angle/distance/scaling for LiDAR, or range/velocity/scaling for radar -- see "
    "docs/hardware-integration.md 'Hardware integration checklist')."
)


class UnconfiguredLiDARParser(LiDARMessageParser):
    """Default LiDAR parser installed until a real one is provided. Always raises."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int) -> list[LiDARPoint]:
        raise STM32ConfigurationError(_NOT_CONFIGURED_MESSAGE.format(modality="LiDAR"))


class UnconfiguredRadarParser(RadarMessageParser):
    """Default radar parser installed until a real one is provided. Always raises."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int, source_id: str) -> RadarReading:
        raise STM32ConfigurationError(_NOT_CONFIGURED_MESSAGE.format(modality="radar"))

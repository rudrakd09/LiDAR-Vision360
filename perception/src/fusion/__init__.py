"""Sensor fusion (Phase 9) -- see docs/fusion.md and `engine.FusionEngine`'s own docstring for
the full picture. Combines a LiDAR-derived `TrackedScan` with a `RadarReading` (Phase 8,
`models.radar`) into a fused `TrackedScan`, using only a clean, protocol-independent
`RadarReading` interface -- no R121 CAN protocol knowledge lives here or anywhere in this package.
"""

from .association import AssociationCandidate, find_associations, radar_target_position, radial_velocity_vector
from .engine import FusionEngine
from .validation import is_reading_fresh, is_target_valid

__all__ = [
    "FusionEngine",
    "AssociationCandidate",
    "find_associations",
    "radar_target_position",
    "radial_velocity_vector",
    "is_reading_fresh",
    "is_target_valid",
]

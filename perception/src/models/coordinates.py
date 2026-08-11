"""Canonical output model for the coordinate-transformation stage (Phase 4).

Reuses `CartesianPoint` (already defined in `models/lidar.py` since Phase 0) for points rather
than inventing a parallel representation -- it already carries exactly `angle`, `distance`, `x`,
`y`, and `timestamp`. See docs/coordinates.md for the full write-up and coordinate convention.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint
from .preprocessing import ScanQualityStatistics


class CartesianScan(BaseModel):
    """The result of running a `PreprocessedScan` through `coordinates.CoordinateTransformer`.

    `points` holds one `CartesianPoint` per retained input point (angle, distance, and timestamp
    preserved unchanged; `x`/`y` newly computed), in the same order as the input --
    `angle -> distance -> (x, y)` correspondence and ordering are both preserved.
    `quality_statistics` is carried through unchanged from the source `PreprocessedScan` (`None`
    only if this scan wasn't derived from one) so downstream consumers -- and any future
    dashboard -- don't need to reach back to an earlier stage's result to see it.
    """

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float
    points: list[CartesianPoint] = Field(default_factory=list)
    quality_statistics: ScanQualityStatistics | None = None

    @property
    def point_count(self) -> int:
        return len(self.points)

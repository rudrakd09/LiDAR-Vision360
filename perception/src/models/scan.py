"""Canonical scan-frame model: a single full 360° sweep, plus its downstream detections."""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint, LiDARPoint
from .objects import DetectedObject


class ScanFrame(BaseModel):
    """One complete LiDAR scan, as produced by any `LiDARDataSource`.

    `points` holds the raw polar measurements (always populated by the data source).
    `cartesian_points` and `objects` are populated by later pipeline stages (Phase 4 and
    Phase 5+ respectively) and are `None` until then -- a frame fresh off a data source will
    only have `points` set.
    """

    scan_id: str = Field(..., description="Unique identifier for this scan frame.")
    sequence_number: int = Field(..., ge=0, description="Monotonically increasing frame counter from the data source.")
    source_id: str = Field(..., description="Identifier of the LiDARDataSource that produced this frame (e.g. 'simulated', 'stm32-uart').")
    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) the scan was captured/generated.")
    points: list[LiDARPoint] = Field(default_factory=list)
    cartesian_points: list[CartesianPoint] | None = Field(default=None, description="Populated by coordinate conversion (Phase 4).")
    objects: list[DetectedObject] | None = Field(default=None, description="Populated by clustering/classification (Phase 5-6).")

    @property
    def point_count(self) -> int:
        return len(self.points)

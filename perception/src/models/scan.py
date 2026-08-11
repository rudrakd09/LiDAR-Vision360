"""Canonical scan-frame model: a single full 360° sweep, plus its downstream detections."""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint, LiDARPoint
from .objects import DetectedObject


class ScanFrame(BaseModel):
    """One complete LiDAR scan, as produced by any `LiDARDataSource`.

    `points` holds the raw polar measurements (always populated by the data source).

    `cartesian_points` and `objects` are early (Phase 0) placeholders reserved for possible
    future use; they are left `None` unless a caller sets them directly and no pipeline code
    populates them. The pipeline that actually shipped follows a different, now-established
    pattern instead -- each stage produces its own dedicated top-level scan model
    (`ScanFrame` -> `preprocessing.Preprocessor` -> `models.preprocessing.PreprocessedScan` ->
    `coordinates.CoordinateTransformer` -> `models.coordinates.CartesianScan` -> ...) rather than
    accumulating optional fields onto one growing frame type. See docs/data-model.md and
    docs/coordinates.md.
    """

    scan_id: str = Field(..., description="Unique identifier for this scan frame.")
    sequence_number: int = Field(..., ge=0, description="Monotonically increasing frame counter from the data source.")
    source_id: str = Field(..., description="Identifier of the LiDARDataSource that produced this frame (e.g. 'simulated', 'stm32-uart').")
    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) the scan was captured/generated.")
    points: list[LiDARPoint] = Field(default_factory=list)
    cartesian_points: list[CartesianPoint] | None = Field(default=None, description="Reserved, unused placeholder -- see class docstring. Not populated by coordinates.CoordinateTransformer; that produces a separate CartesianScan instead.")
    objects: list[DetectedObject] | None = Field(default=None, description="Reserved, unused placeholder -- see class docstring.")

    @property
    def point_count(self) -> int:
        return len(self.points)

"""Canonical output model for the preprocessing stage (Phase 3).

Deliberately reuses `LiDARPoint` for retained points rather than inventing a parallel
representation -- a clean, retained measurement is still just an (angle, distance, timestamp)
polar point. See docs/preprocessing.md for the full pipeline write-up and the semantics of the
four counts below.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import LiDARPoint


class ScanQualityStatistics(BaseModel):
    """Aggregate quality metrics for one scan, computed by the preprocessing stage.

    Percentages are relative to `PreprocessedScan.total_count`. The four distance statistics are
    computed over `PreprocessedScan.points` (the final retained/filtered measurements) and are
    `None` when there are no retained points (empty or completely invalid scan) -- callers must
    not assume they are always populated.
    """

    valid_percentage: float = Field(..., ge=0.0, le=100.0)
    invalid_percentage: float = Field(..., ge=0.0, le=100.0)
    outlier_percentage: float = Field(..., ge=0.0, le=100.0)
    mean_distance: float | None = None
    median_distance: float | None = None
    minimum_distance: float | None = None
    maximum_distance: float | None = None


class PreprocessedScan(BaseModel):
    """The cleaned, validated result of running a `ScanFrame` through `preprocessing.Preprocessor`.

    `points` holds only the final retained measurements (passed validation, not flagged as an
    outlier), angle-sorted, with noise filtering (and optional temporal filtering) already
    applied to their `distance`. No Cartesian conversion happens here -- that is Phase 4.

    Count semantics (see docs/preprocessing.md for the full pipeline breakdown):
    - `total_count`: measurements in the source `ScanFrame`.
    - `invalid_count`: measurements that failed validation (non-finite, out of configured
      range, or explicitly flagged invalid by the data source).
    - `valid_count`: `total_count - invalid_count` (passed validation; may still include
      points later removed as outliers).
    - `outlier_count`: of the valid measurements, how many were removed as local outliers.
    - `len(points) == valid_count - outlier_count`.
    """

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float
    points: list[LiDARPoint] = Field(default_factory=list)

    total_count: int = Field(..., ge=0)
    valid_count: int = Field(..., ge=0)
    invalid_count: int = Field(..., ge=0)
    outlier_count: int = Field(..., ge=0)

    quality_statistics: ScanQualityStatistics

    @property
    def point_count(self) -> int:
        return len(self.points)

"""Scan-quality statistics: the numbers a future sensor-health monitor / dashboard will consume."""

from __future__ import annotations

import statistics

from models.lidar import LiDARPoint
from models.preprocessing import ScanQualityStatistics


def compute_quality_statistics(
    total_count: int, valid_count: int, invalid_count: int, outlier_count: int, retained_points: list[LiDARPoint]
) -> ScanQualityStatistics:
    """Percentages are relative to `total_count` (0.0 for all three if the scan was empty --
    there is nothing to be valid/invalid/an-outlier out of). Distance statistics are computed
    over `retained_points` (the final, clean output) and are `None` if it's empty."""
    if total_count > 0:
        valid_pct = valid_count / total_count * 100.0
        invalid_pct = invalid_count / total_count * 100.0
        outlier_pct = outlier_count / total_count * 100.0
    else:
        valid_pct = invalid_pct = outlier_pct = 0.0

    distances = [p.distance for p in retained_points]
    if distances:
        mean_d: float | None = statistics.fmean(distances)
        median_d: float | None = statistics.median(distances)
        min_d: float | None = min(distances)
        max_d: float | None = max(distances)
    else:
        mean_d = median_d = min_d = max_d = None

    return ScanQualityStatistics(
        valid_percentage=round(valid_pct, 4),
        invalid_percentage=round(invalid_pct, 4),
        outlier_percentage=round(outlier_pct, 4),
        mean_distance=round(mean_d, 4) if mean_d is not None else None,
        median_distance=round(median_d, 4) if median_d is not None else None,
        minimum_distance=round(min_d, 4) if min_d is not None else None,
        maximum_distance=round(max_d, 4) if max_d is not None else None,
    )

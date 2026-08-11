"""Per-measurement validation: is this `LiDARPoint` physically usable at all?

Note on `LiDARPoint`'s own pydantic field constraints (`angle` in `[0, 360)`, `distance >= 0`):
those already reject most malformed values (including NaN, since `NaN >= 0` is `False`) for any
point constructed through the normal, validated path (every current data source uses it). This
module re-checks finiteness and range anyway, as defense in depth against any future code path
that bypasses model validation (e.g. a bulk/unchecked construction for performance, or a
corrupted value arriving via `model_construct`) -- see `docs/preprocessing.md` "Limitations" and
`perception/tests/test_preprocessing_validation.py`, which exercises this directly using
`LiDARPoint.model_construct(...)` to simulate such input. `distance` has no pydantic upper bound,
so `inf` *can* reach here through the normal path -- this module is what actually catches it.
"""

from __future__ import annotations

import math
from collections import Counter
from enum import Enum

from models.lidar import LiDARPoint


class InvalidReason(str, Enum):
    """Why a measurement failed validation. Values that would otherwise be indistinguishable in
    aggregate counts alone are broken out here to support useful logging and future
    sensor-health diagnostics (Phase 24)."""

    SENSOR_FLAGGED_INVALID = "sensor_flagged_invalid"  # LiDARPoint.valid was already False.
    NON_FINITE_ANGLE = "non_finite_angle"
    NON_FINITE_DISTANCE = "non_finite_distance"
    BELOW_MIN_RANGE = "below_min_range"
    ABOVE_MAX_RANGE = "above_max_range"
    MALFORMED = "malformed"  # Unexpected error while validating; treated as invalid, not fatal.


def classify_point(point: LiDARPoint, min_range_m: float, max_range_m: float) -> InvalidReason | None:
    """Return why `point` is invalid, or `None` if it passes every validation rule."""
    try:
        if not point.valid:
            return InvalidReason.SENSOR_FLAGGED_INVALID
        if not math.isfinite(point.angle):
            return InvalidReason.NON_FINITE_ANGLE
        if not math.isfinite(point.distance):
            return InvalidReason.NON_FINITE_DISTANCE
        if point.distance < min_range_m:
            return InvalidReason.BELOW_MIN_RANGE
        if point.distance > max_range_m:
            return InvalidReason.ABOVE_MAX_RANGE
        return None
    except (TypeError, ValueError):
        # A point somehow holding a non-numeric value would raise here rather than crash the
        # whole scan; treat it as invalid and let the caller log it.
        return InvalidReason.MALFORMED


def validate_points(
    points: list[LiDARPoint], min_range_m: float, max_range_m: float
) -> tuple[list[LiDARPoint], int, Counter]:
    """Split `points` into (valid_points, invalid_count, reason_counts).

    `valid_points` preserves input order (angle-sorting happens later in the pipeline, once
    outlier detection needs it). `reason_counts` is a `Counter[InvalidReason]`, useful for
    logging/diagnostics even though `PreprocessedScan` itself only carries the aggregate count.
    """
    valid_points: list[LiDARPoint] = []
    reason_counts: Counter = Counter()

    for point in points:
        reason = classify_point(point, min_range_m, max_range_m)
        if reason is None:
            valid_points.append(point)
        else:
            reason_counts[reason] += 1

    return valid_points, sum(reason_counts.values()), reason_counts

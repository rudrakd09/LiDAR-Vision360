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

This module is also where the perception pipeline drops **self returns** -- measurements off the
ego vehicle's own body or the sensor mount, which are never environmental objects -- before the
coordinate transform, clustering, classification, tracking, mapping, and collision/clearance, so
they can never seed a cluster, a track, a "critical object", or a 0.00 m directional clearance:

* `BELOW_MIN_VALID_DISTANCE` -- a small radial guard (`Settings.min_valid_distance_m`): a return
  at, or essentially at, the sensor origin, plus any zero / negative / non-finite distance
  (rejected regardless of the configured threshold).
* `INSIDE_EGO_FOOTPRINT` -- a *geometric* guard (`common.geometry.EgoFootprint`, built from
  `vehicle_length_m` / `vehicle_width_m` / `lidar_mount_*`): a return whose `(x, y)` falls inside
  the real configured vehicle body rectangle. This is what catches a self-return *arc* a few tens
  of centimetres out (an arc of body points forms a cluster near the origin that the radial
  guard, checking each point's own distance, does not remove). It is not a distance threshold --
  an obstacle just outside the body is kept.
"""

from __future__ import annotations

import math
from collections import Counter
from enum import Enum

from common.geometry import EgoFootprint
from models.lidar import LiDARPoint


class InvalidReason(str, Enum):
    """Why a measurement failed validation. Values that would otherwise be indistinguishable in
    aggregate counts alone are broken out here to support useful logging and future
    sensor-health diagnostics (Phase 24)."""

    SENSOR_FLAGGED_INVALID = "sensor_flagged_invalid"  # LiDARPoint.valid was already False.
    NON_FINITE_ANGLE = "non_finite_angle"
    NON_FINITE_DISTANCE = "non_finite_distance"
    # Distance is zero, negative, or below Settings.min_valid_distance_m -- a return at
    # (essentially) the sensor origin, not an environmental object.
    BELOW_MIN_VALID_DISTANCE = "below_min_valid_distance"
    # The return's (x, y) is inside the ego vehicle's own body rectangle -- a self return off the
    # vehicle / sensor mount (common.geometry.EgoFootprint). Not a real obstacle.
    INSIDE_EGO_FOOTPRINT = "inside_ego_footprint"
    BELOW_MIN_RANGE = "below_min_range"
    ABOVE_MAX_RANGE = "above_max_range"
    MALFORMED = "malformed"  # Unexpected error while validating; treated as invalid, not fatal.


def classify_point(
    point: LiDARPoint,
    min_range_m: float,
    max_range_m: float,
    min_valid_distance_m: float = 0.0,
    ego_footprint: EgoFootprint | None = None,
) -> InvalidReason | None:
    """Return why `point` is invalid, or `None` if it passes every validation rule.

    `min_valid_distance_m` is the near-field radial cutoff (`Settings.min_valid_distance_m`); a
    distance that is zero, negative, or below it is rejected as `BELOW_MIN_VALID_DISTANCE`.
    `ego_footprint`, when supplied, rejects a return whose `(x, y)` lands inside the ego vehicle
    body as `INSIDE_EGO_FOOTPRINT` -- the geometric self-return guard. Both default to
    off/permissive so a caller that only cares about the raw sensor range can omit them and still
    get zero / negative / non-finite rejection.
    """
    try:
        if not point.valid:
            return InvalidReason.SENSOR_FLAGGED_INVALID
        if not math.isfinite(point.angle):
            return InvalidReason.NON_FINITE_ANGLE
        if not math.isfinite(point.distance):
            return InvalidReason.NON_FINITE_DISTANCE
        if point.distance <= 0.0 or point.distance < min_valid_distance_m:
            # At/near the sensor origin: the ego vehicle or the sensor's own housing, never a
            # real environmental object. Rejected before it can reach clustering/tracking.
            return InvalidReason.BELOW_MIN_VALID_DISTANCE
        if ego_footprint is not None and ego_footprint.contains_polar(point.angle, point.distance):
            # The return lands inside the ego vehicle body -- a self return off the vehicle /
            # sensor mount. Catches a self-return *arc* (a cluster near the origin) that the
            # per-point radial check above cannot, since each individual point's distance can sit
            # a few tens of centimetres out while the cluster centroid hugs the origin.
            return InvalidReason.INSIDE_EGO_FOOTPRINT
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
    points: list[LiDARPoint],
    min_range_m: float,
    max_range_m: float,
    min_valid_distance_m: float = 0.0,
    ego_footprint: EgoFootprint | None = None,
) -> tuple[list[LiDARPoint], int, Counter]:
    """Split `points` into (valid_points, invalid_count, reason_counts).

    `valid_points` preserves input order (angle-sorting happens later in the pipeline, once
    outlier detection needs it). `reason_counts` is a `Counter[InvalidReason]`, useful for
    logging/diagnostics even though `PreprocessedScan` itself only carries the aggregate count.
    `min_valid_distance_m` and `ego_footprint` are forwarded to `classify_point` (the two
    self-return guards).
    """
    valid_points: list[LiDARPoint] = []
    reason_counts: Counter = Counter()

    for point in points:
        reason = classify_point(point, min_range_m, max_range_m, min_valid_distance_m, ego_footprint)
        if reason is None:
            valid_points.append(point)
        else:
            reason_counts[reason] += 1

    return valid_points, sum(reason_counts.values()), reason_counts

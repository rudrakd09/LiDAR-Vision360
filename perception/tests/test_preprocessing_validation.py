"""Tests for preprocessing.validation: per-measurement validity checks."""

import math

from models.lidar import LiDARPoint
from preprocessing.validation import InvalidReason, classify_point, validate_points

MIN_RANGE, MAX_RANGE = 0.05, 12.0
# The near-field self-return cutoff (Settings.min_valid_distance_m default): larger than
# MIN_RANGE, so a return between the two is a self/ego return, not a too-close real object.
MIN_VALID = 0.3


def _point(angle=0.0, distance=5.0, valid=True, timestamp=0.0) -> LiDARPoint:
    return LiDARPoint(angle=angle, distance=distance, timestamp=timestamp, valid=valid)


class TestClassifyPoint:
    def test_valid_point_passes(self):
        assert classify_point(_point(distance=5.0), MIN_RANGE, MAX_RANGE) is None

    def test_nan_distance_rejected(self):
        # LiDARPoint's own pydantic constraint (`ge=0.0`) already rejects NaN at normal
        # construction time (NaN >= 0 is False); model_construct() bypasses that validation to
        # simulate a malformed point arriving through some future non-validated path, so this
        # module's own defense-in-depth check can be exercised directly.
        point = LiDARPoint.model_construct(angle=0.0, distance=float("nan"), timestamp=0.0, valid=True, intensity=None)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.NON_FINITE_DISTANCE

    def test_infinite_distance_rejected(self):
        # Unlike NaN, `inf >= 0.0` is True, so LiDARPoint's own constraint does NOT reject this
        # at construction (there is no upper bound field constraint) -- this module is what
        # actually catches it.
        point = _point(distance=math.inf)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.NON_FINITE_DISTANCE

    def test_non_finite_angle_rejected(self):
        point = LiDARPoint.model_construct(angle=math.nan, distance=5.0, timestamp=0.0, valid=True, intensity=None)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.NON_FINITE_ANGLE

    def test_below_minimum_range_rejected(self):
        point = _point(distance=MIN_RANGE - 0.01)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.BELOW_MIN_RANGE

    def test_above_maximum_range_rejected(self):
        point = _point(distance=MAX_RANGE + 0.01)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.ABOVE_MAX_RANGE

    def test_at_exact_range_bounds_is_valid(self):
        assert classify_point(_point(distance=MIN_RANGE), MIN_RANGE, MAX_RANGE) is None
        assert classify_point(_point(distance=MAX_RANGE), MIN_RANGE, MAX_RANGE) is None

    def test_sensor_flagged_invalid_takes_priority(self):
        # distance=0.0 would also fail the range check, but the sensor's own valid=False signal
        # should be the reported reason.
        point = _point(distance=0.0, valid=False)
        assert classify_point(point, MIN_RANGE, MAX_RANGE) == InvalidReason.SENSOR_FLAGGED_INVALID


class TestNearFieldSelfReturnCutoff:
    """`min_valid_distance_m` -- the ego-vehicle / sensor-housing return at (essentially) the
    sensor origin. Rejected here, before clustering/tracking, so it cannot form a phantom track."""

    def test_zero_distance_is_rejected_even_without_a_configured_cutoff(self):
        # D=0 ("no return", per docs/data-model.md) must never be treated as an object at the
        # origin -- rejected regardless of the min_valid_distance_m value.
        assert classify_point(_point(distance=0.0), MIN_RANGE, MAX_RANGE) == InvalidReason.BELOW_MIN_VALID_DISTANCE
        assert classify_point(_point(distance=0.0), MIN_RANGE, MAX_RANGE, 0.0) == InvalidReason.BELOW_MIN_VALID_DISTANCE

    def test_negative_distance_is_rejected(self):
        # Can't happen through normal LiDARPoint construction (ge=0.0); model_construct simulates
        # a corrupted value reaching this stage.
        point = LiDARPoint.model_construct(angle=0.0, distance=-1.5, timestamp=0.0, valid=True, intensity=None)
        assert classify_point(point, MIN_RANGE, MAX_RANGE, MIN_VALID) == InvalidReason.BELOW_MIN_VALID_DISTANCE

    def test_distance_below_cutoff_is_rejected_as_self_return(self):
        # 0.05 m: above the raw sensor floor (MIN_RANGE) -- the value the simulator's clamp
        # produces for an obstacle edge sweeping across the origin -- but below MIN_VALID.
        point = _point(distance=0.05)
        assert classify_point(point, MIN_RANGE, MAX_RANGE, MIN_VALID) == InvalidReason.BELOW_MIN_VALID_DISTANCE

    def test_distance_just_below_cutoff_is_rejected(self):
        point = _point(distance=MIN_VALID - 1e-6)
        assert classify_point(point, MIN_RANGE, MAX_RANGE, MIN_VALID) == InvalidReason.BELOW_MIN_VALID_DISTANCE

    def test_distance_at_cutoff_is_valid(self):
        assert classify_point(_point(distance=MIN_VALID), MIN_RANGE, MAX_RANGE, MIN_VALID) is None

    def test_legitimate_nearby_object_just_outside_cutoff_is_preserved(self):
        # A real obstacle right in front of the sensor but clear of the self-return region.
        assert classify_point(_point(distance=MIN_VALID + 0.05), MIN_RANGE, MAX_RANGE, MIN_VALID) is None
        assert classify_point(_point(distance=0.5), MIN_RANGE, MAX_RANGE, MIN_VALID) is None

    def test_normal_environmental_distance_is_unaffected(self):
        assert classify_point(_point(distance=5.0), MIN_RANGE, MAX_RANGE, MIN_VALID) is None

    def test_non_finite_distance_still_reported_as_non_finite_not_self_return(self):
        # Ordering: the NaN/inf check comes first, so the reason stays specific.
        assert classify_point(_point(distance=math.inf), MIN_RANGE, MAX_RANGE, MIN_VALID) == InvalidReason.NON_FINITE_DISTANCE

    def test_validate_points_counts_and_drops_a_ring_of_self_returns(self):
        # A full ring of origin returns (the simulator's clamped 0.05 m) plus real 5 m points.
        ring = [_point(angle=float(a), distance=0.05) for a in range(0, 360, 4)]
        real = [_point(angle=float(a) + 1.0, distance=5.0) for a in range(0, 360, 4)]
        valid, invalid_count, reasons = validate_points(ring + real, MIN_RANGE, MAX_RANGE, MIN_VALID)

        assert len(valid) == len(real)
        assert all(p.distance == 5.0 for p in valid)
        assert invalid_count == len(ring)
        assert reasons[InvalidReason.BELOW_MIN_VALID_DISTANCE] == len(ring)
        assert InvalidReason.BELOW_MIN_RANGE not in reasons


class TestEgoFootprintMask:
    """`INSIDE_EGO_FOOTPRINT` -- a return whose (x, y) is inside the ego vehicle body is a self
    return, rejected regardless of its own distance. This is what catches a self-return *arc*
    (a cluster near the origin) that the per-point radial cutoff cannot."""

    def _footprint(self, **overrides):
        from common.config import Settings
        from common.geometry import EgoFootprint

        return EgoFootprint.from_settings(Settings(_env_file=None, **overrides))

    def test_arc_point_a_few_tens_of_cm_out_is_rejected_when_inside_the_body(self):
        ego = self._footprint()
        # 0.42 m at 135deg -> (-0.30, 0.30): well outside MIN_VALID as a raw distance, but inside
        # the 2.25 x 0.9 m body.
        point = _point(angle=135.0, distance=0.42)
        assert classify_point(point, MIN_RANGE, MAX_RANGE, 0.0, ego) == InvalidReason.INSIDE_EGO_FOOTPRINT
        # ...and without the footprint that same point is perfectly valid
        assert classify_point(point, MIN_RANGE, MAX_RANGE, 0.0, None) is None

    def test_real_obstacle_outside_the_body_is_untouched(self):
        ego = self._footprint()
        assert classify_point(_point(angle=0.0, distance=3.0), MIN_RANGE, MAX_RANGE, 0.0, ego) is None
        assert classify_point(_point(angle=90.0, distance=1.5), MIN_RANGE, MAX_RANGE, 0.0, ego) is None

    def test_bumper_mount_keeps_a_genuine_half_metre_obstacle_ahead(self):
        ego = self._footprint(lidar_mount_x_m=2.2)
        assert classify_point(_point(angle=0.0, distance=0.5), MIN_RANGE, MAX_RANGE, 0.0, ego) is None
        # a return 1 m behind the bumper sensor is into the body -> still rejected
        assert classify_point(_point(angle=180.0, distance=1.0), MIN_RANGE, MAX_RANGE, 0.0, ego) == InvalidReason.INSIDE_EGO_FOOTPRINT

    def test_validate_points_counts_a_rejected_self_return_arc(self):
        ego = self._footprint()
        arc = [_point(angle=float(a), distance=0.44) for a in range(100, 216, 2)]  # rear-left body arc
        wall = [_point(angle=float(a % 360), distance=6.0) for a in range(-15, 16)]
        valid, invalid_count, reasons = validate_points(arc + wall, MIN_RANGE, MAX_RANGE, 0.0, ego)
        assert reasons[InvalidReason.INSIDE_EGO_FOOTPRINT] == len(arc)
        assert invalid_count == len(arc)
        assert len(valid) == len(wall)


class TestValidatePoints:
    def test_all_valid_scan_passes_through(self):
        points = [_point(angle=float(i)) for i in range(10)]
        valid, invalid_count, reasons = validate_points(points, MIN_RANGE, MAX_RANGE)
        assert len(valid) == 10
        assert invalid_count == 0
        assert sum(reasons.values()) == 0

    def test_empty_scan_is_handled_safely(self):
        valid, invalid_count, reasons = validate_points([], MIN_RANGE, MAX_RANGE)
        assert valid == []
        assert invalid_count == 0
        assert dict(reasons) == {}

    def test_completely_invalid_scan_is_handled_safely(self):
        points = [_point(angle=float(i), valid=False) for i in range(5)]
        valid, invalid_count, reasons = validate_points(points, MIN_RANGE, MAX_RANGE)
        assert valid == []
        assert invalid_count == 5
        assert reasons[InvalidReason.SENSOR_FLAGGED_INVALID] == 5

    def test_mixed_scan_splits_correctly(self):
        points = [
            _point(angle=0.0, distance=5.0),  # valid
            _point(angle=1.0, distance=MAX_RANGE + 1.0),  # above range
            _point(angle=2.0, distance=MIN_RANGE - 0.01),  # below range
            _point(angle=3.0, valid=False),  # sensor-flagged
            _point(angle=4.0, distance=6.0),  # valid
        ]
        valid, invalid_count, reasons = validate_points(points, MIN_RANGE, MAX_RANGE)
        assert len(valid) == 2
        assert invalid_count == 3
        assert reasons[InvalidReason.ABOVE_MAX_RANGE] == 1
        assert reasons[InvalidReason.BELOW_MIN_RANGE] == 1
        assert reasons[InvalidReason.SENSOR_FLAGGED_INVALID] == 1

    def test_order_is_preserved(self):
        points = [_point(angle=float(a), distance=5.0) for a in (30, 10, 20)]
        valid, _, _ = validate_points(points, MIN_RANGE, MAX_RANGE)
        assert [p.angle for p in valid] == [30.0, 10.0, 20.0]

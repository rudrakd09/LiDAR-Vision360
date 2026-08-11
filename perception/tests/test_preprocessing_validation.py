"""Tests for preprocessing.validation: per-measurement validity checks."""

import math

from models.lidar import LiDARPoint
from preprocessing.validation import InvalidReason, classify_point, validate_points

MIN_RANGE, MAX_RANGE = 0.05, 12.0


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

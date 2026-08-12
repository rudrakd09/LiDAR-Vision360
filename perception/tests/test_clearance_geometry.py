"""Tests for clearance.geometry: quadrant classification and per-direction nearest-point scan."""

from clearance.geometry import classify_quadrant, nearest_in_each_quadrant
from collision.geometry import vehicle_footprint
from common.config import Settings
from models.clearance import ClearanceDirection
from models.collision import VehicleState
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint

DEFAULT_SETTINGS = Settings(_env_file=None)


def _scan(points: list[CartesianPoint]) -> CartesianScan:
    return CartesianScan(scan_id="s", sequence_number=0, source_id="unit-test", timestamp=0.0, points=points)


def _point(x: float, y: float, valid: bool = True, distance: float | None = None) -> CartesianPoint:
    import math

    dist = distance if distance is not None else math.hypot(x, y)
    angle = math.degrees(math.atan2(y, x)) % 360
    return CartesianPoint(angle=angle, distance=dist, timestamp=0.0, valid=valid, x=x, y=y)


class TestClassifyQuadrant:
    def test_dead_ahead_is_front(self):
        assert classify_quadrant(along=10.0, lateral=0.0) == ClearanceDirection.FRONT

    def test_dead_left_is_left(self):
        assert classify_quadrant(along=0.0, lateral=10.0) == ClearanceDirection.LEFT

    def test_dead_right_is_right(self):
        assert classify_quadrant(along=0.0, lateral=-10.0) == ClearanceDirection.RIGHT

    def test_dead_behind_is_rear(self):
        assert classify_quadrant(along=-10.0, lateral=0.0) == ClearanceDirection.REAR

    def test_every_point_is_classified_into_exactly_one_quadrant(self):
        import math

        for deg in range(0, 360, 5):
            rad = math.radians(deg)
            along, lateral = math.cos(rad) * 5.0, math.sin(rad) * 5.0
            result = classify_quadrant(along, lateral)
            assert result in ClearanceDirection

    def test_boundary_at_45_degrees_is_left_not_front(self):
        # atan2(1,1) == 45deg exactly -- classify_quadrant's front band is [-45, 45), so 45 itself
        # belongs to LEFT, not FRONT (no gap/overlap at the seam).
        assert classify_quadrant(along=1.0, lateral=1.0) == ClearanceDirection.LEFT

    def test_boundary_at_negative_45_degrees_is_front(self):
        assert classify_quadrant(along=1.0, lateral=-1.0) == ClearanceDirection.FRONT


class TestNearestInEachQuadrant:
    def test_empty_scan_defaults_every_direction_to_range_minus_envelope(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        readings = nearest_in_each_quadrant(_scan([]), VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].distance_m == round(DEFAULT_SETTINGS.lidar_range_max_m - footprint.envelope_front, 4)
        assert all(r.world_point is None for r in readings.values())

    def test_single_point_ahead_sets_front_only(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(6.0, 0.0)])
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].distance_m == round(6.0 - footprint.envelope_front, 4)
        assert readings[ClearanceDirection.FRONT].world_point.x == 6.0
        # other directions untouched -- still their empty-scan default
        assert readings[ClearanceDirection.REAR].world_point is None

    def test_closest_point_in_quadrant_wins(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(8.0, 0.0), _point(4.0, 0.0), _point(6.0, 0.0)])
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].distance_m == round(4.0 - footprint.envelope_front, 4)
        assert readings[ClearanceDirection.FRONT].world_point.x == 4.0

    def test_point_beyond_max_range_is_ignored(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(DEFAULT_SETTINGS.lidar_range_max_m + 5.0, 0.0, distance=DEFAULT_SETTINGS.lidar_range_max_m + 5.0)])
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].world_point is None  # fell back to the default, not the out-of-range point

    def test_invalid_point_is_ignored(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(2.0, 0.0, valid=False)])
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].world_point is None

    def test_point_inside_safety_envelope_floors_at_zero_not_negative(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(0.5, 0.0)])  # well inside envelope_front (~3.25m default)
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].distance_m == 0.0

    def test_four_points_populate_all_four_directions_independently(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        scan = _scan([_point(6.0, 0.0), _point(-6.0, 0.0), _point(0.0, 6.0), _point(0.0, -6.0)])
        readings = nearest_in_each_quadrant(scan, VehicleState(), footprint, DEFAULT_SETTINGS)
        assert readings[ClearanceDirection.FRONT].world_point.x == 6.0
        assert readings[ClearanceDirection.REAR].world_point.x == -6.0
        assert readings[ClearanceDirection.LEFT].world_point.y == 6.0
        assert readings[ClearanceDirection.RIGHT].world_point.y == -6.0

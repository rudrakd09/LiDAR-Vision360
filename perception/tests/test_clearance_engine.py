"""Tests for clearance.engine.ClearanceEngine: end-to-end scan -> ClearanceAssessment, mirroring
test_collision_engine.py's builder-helper + numbered-section convention."""

import math

from clearance import ClearanceEngine
from common.config import Settings
from models.clearance import ClearanceDirection, ClearanceState
from models.collision import VehicleState
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint

DEFAULT_SETTINGS = Settings(_env_file=None)


def _point(x: float, y: float, valid: bool = True) -> CartesianPoint:
    return CartesianPoint(angle=math.degrees(math.atan2(y, x)) % 360, distance=math.hypot(x, y), timestamp=0.0, valid=valid, x=x, y=y)


def _scan(points: list[CartesianPoint], seq: int = 1) -> CartesianScan:
    return CartesianScan(scan_id="s", sequence_number=seq, source_id="unit-test", timestamp=1000.0, points=points)


def _engine() -> ClearanceEngine:
    return ClearanceEngine(settings=DEFAULT_SETTINGS)


# --- 1. Basic evaluation -------------------------------------------------------------------

class TestBasicEvaluation:
    def test_empty_scan_is_safe_in_every_direction(self):
        result = _engine().evaluate(_scan([]))
        assert result.overall_status == ClearanceState.SAFE
        assert result.front.nearest_point is None

    def test_defaults_to_stationary_vehicle_when_none_supplied(self):
        result = _engine().evaluate(_scan([_point(1.0, 0.0)]))
        assert result.overall_status is not None  # ran without error using the default VehicleState

    def test_every_direction_present_on_the_result(self):
        result = _engine().evaluate(_scan([]))
        assert {result.front.direction, result.rear.direction, result.left.direction, result.right.direction} == set(ClearanceDirection)


# --- 2. Wall-in-front-style scenario ----------------------------------------------------------

class TestWallInFront:
    def test_close_wall_ahead_drives_front_clearance_down(self):
        result = _engine().evaluate(_scan([_point(x, 0.0) for x in [2.0, 2.05, 1.95]] + [_point(0.0, 20.0), _point(0.0, -20.0)]))
        assert result.min_direction == ClearanceDirection.FRONT
        assert result.overall_status in (ClearanceState.LOW_CLEARANCE, ClearanceState.CRITICAL)

    def test_far_wall_ahead_is_safe(self):
        result = _engine().evaluate(_scan([_point(11.0, 0.0)]))
        assert result.overall_status == ClearanceState.SAFE


# --- 3. Narrow corridor (06_narrow_corridor-style) ----------------------------------------------

class TestNarrowCorridor:
    def test_symmetric_narrow_walls_reduce_corridor_width(self):
        wide = _engine().evaluate(_scan([_point(0.0, 5.0), _point(0.0, -5.0)]))
        narrow = _engine().evaluate(_scan([_point(0.0, 1.3), _point(0.0, -1.3)]))
        assert narrow.corridor_width_m < wide.corridor_width_m

    def test_corridor_width_formula(self):
        result = _engine().evaluate(_scan([_point(0.0, 3.0), _point(0.0, -3.0)]))
        from collision.geometry import vehicle_footprint

        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        expected = result.left.distance_m + footprint.envelope_left + footprint.envelope_right + result.right.distance_m
        assert result.corridor_width_m == round(expected, 4)


# --- 4. Min-direction tie-breaking --------------------------------------------------------------

class TestMinDirectionSelection:
    def test_reports_the_actual_closest_direction(self):
        result = _engine().evaluate(_scan([_point(0.0, 0.6)]))  # much closer on the left than default elsewhere
        assert result.min_direction == ClearanceDirection.LEFT
        assert result.min_clearance_m == result.left.distance_m


# --- 5. Explainability -----------------------------------------------------------------------

class TestExplainability:
    def test_reason_is_never_empty(self):
        # A real forward return (well outside the min_valid_distance_m self-return radius).
        result = _engine().evaluate(_scan([_point(2.0, 0.0)]))
        assert len(result.reason) > 0

    def test_scan_metadata_is_preserved(self):
        result = _engine().evaluate(_scan([], seq=42))
        assert result.sequence_number == 42
        assert result.scan_id == "s"
        assert result.source_id == "unit-test"


# --- 6. Moving vehicle does not change purely-geometric clearance ------------------------------

class TestVehicleState:
    def test_vehicle_speed_does_not_affect_clearance_geometry(self):
        # Unlike collision (which reasons about closing speed/TTC), clearance is purely geometric
        # -- vehicle_state.speed_mps must not change the result, only its pose does.
        stationary = _engine().evaluate(_scan([_point(3.0, 0.0)]), vehicle_state=VehicleState(speed_mps=0.0))
        moving = _engine().evaluate(_scan([_point(3.0, 0.0)]), vehicle_state=VehicleState(speed_mps=8.0))
        assert stationary.front.distance_m == moving.front.distance_m

"""Tests for collision.engine.CollisionRiskEngine: the spec's full edge-case and scenario
checklist -- basic distance, stationary/moving vehicle+obstacle combinations, relative motion,
TTC, multiple objects, explainability, and boundary conditions."""

from common.config import Settings
from collision import CollisionRiskEngine
from models.collision import RiskLevel, VehicleState
from models.mapping import VehiclePose
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.tracking import TrackedScan

DEFAULT_SETTINGS = Settings(_env_file=None)


def _object(
    x: float, y: float, vx: float = 0.0, vy: float = 0.0, track_id: str = "t1",
    classification: ObjectClassification = ObjectClassification.VEHICLE_LIKE,
    width: float = 1.8, depth: float = 1.8, has_velocity: bool = True,
) -> DetectedObject:
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=x, y=y), width=width, depth=depth,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=classification, confidence=0.9,
        velocity=Velocity2D(vx=vx, vy=vy) if has_velocity else None,
        tracking_state=TrackingState.CONFIRMED,
        movement_state=MovementState.MOVING if has_velocity and (vx or vy) else (MovementState.STATIONARY if has_velocity else MovementState.UNKNOWN),
        track_age=5, track_hits=5, track_misses=0, timestamp=0.0,
    )


def _scan(objects: list[DetectedObject]) -> TrackedScan:
    return TrackedScan(
        scan_id="s", sequence_number=0, source_id="unit-test", timestamp=1000.0,
        objects=objects, noise_points=[], object_count=len(objects), noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _engine() -> CollisionRiskEngine:
    return CollisionRiskEngine(settings=DEFAULT_SETTINGS)


# --- 1. Basic distance ------------------------------------------------------------------------

class TestBasicDistance:
    def test_far_object_is_safe(self):
        result = _engine().evaluate(_scan([_object(30.0, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.SAFE

    def test_nearby_object_gets_elevated_risk(self):
        result = _engine().evaluate(_scan([_object(1.5, 0.0)])).results[0]
        assert result.risk_level in (RiskLevel.WARNING, RiskLevel.CRITICAL)


# --- 2. Vehicle stationary ---------------------------------------------------------------------

class TestVehicleStationary:
    def test_stationary_obstacle_far_away_no_false_imminent_collision(self):
        # This phase's own explicit example: a wall 10m ahead of a stationary vehicle.
        result = _engine().evaluate(_scan([_object(10.0, 0.0, width=6.0, depth=0.2, classification=ObjectClassification.WALL)])).results[0]
        assert result.risk_level == RiskLevel.SAFE

    def test_moving_obstacle_approaching_stationary_vehicle_increases_risk(self):
        far = _engine().evaluate(_scan([_object(15.0, 0.0, vx=-3.0)])).results[0]
        near = _engine().evaluate(_scan([_object(3.0, 0.0, vx=-3.0)])).results[0]
        severity = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}
        assert severity[near.risk_level] >= severity[far.risk_level]
        assert near.risk_level != RiskLevel.SAFE


# --- 3. Vehicle moving -------------------------------------------------------------------------

class TestVehicleMoving:
    def test_stationary_obstacle_in_front_of_moving_vehicle_gets_ttc(self):
        result = _engine().evaluate(_scan([_object(15.0, 0.0)]), vehicle_state=VehicleState(speed_mps=5.0)).results[0]
        assert result.ttc is not None
        assert result.ttc > 0.0

    def test_stationary_obstacle_outside_path_no_collision(self):
        result = _engine().evaluate(_scan([_object(15.0, 10.0)]), vehicle_state=VehicleState(speed_mps=5.0)).results[0]
        assert result.in_projected_path is False
        assert result.risk_level == RiskLevel.SAFE

    def test_obstacle_behind_vehicle_no_forward_collision(self):
        result = _engine().evaluate(_scan([_object(-20.0, 0.0)]), vehicle_state=VehicleState(speed_mps=5.0)).results[0]
        assert result.in_projected_path is False

    def test_side_obstacle_outside_footprint_no_collision(self):
        result = _engine().evaluate(_scan([_object(5.0, 8.0)]), vehicle_state=VehicleState(speed_mps=5.0)).results[0]
        assert result.risk_level == RiskLevel.SAFE


# --- 4. Relative motion ------------------------------------------------------------------------

class TestRelativeMotion:
    def test_object_moving_toward_vehicle(self):
        result = _engine().evaluate(_scan([_object(10.0, 0.0, vx=-2.0)])).results[0]
        assert result.relative_velocity.vx == -2.0
        assert result.ttc is not None

    def test_object_moving_away(self):
        result = _engine().evaluate(_scan([_object(10.0, 0.0, vx=2.0)])).results[0]
        assert result.ttc is None

    def test_object_crossing_vehicle_path(self):
        vehicle = VehicleState(speed_mps=2.0)
        result = _engine().evaluate(_scan([_object(6.0, -6.0, vy=1.5, width=0.5, depth=0.5)]), vehicle_state=vehicle).results[0]
        assert result.collision_predicted is True

    def test_object_moving_parallel_never_converges(self):
        # Object moving in the same direction, offset laterally beyond the footprint -- parallel
        # motion, closing speed along heading is 0 (same forward speed as vehicle).
        vehicle = VehicleState(speed_mps=5.0)
        result = _engine().evaluate(_scan([_object(10.0, 5.0, vx=5.0, vy=0.0)]), vehicle_state=vehicle).results[0]
        assert result.risk_level == RiskLevel.SAFE


# --- 5. TTC edge cases -------------------------------------------------------------------------

class TestTTCEdgeCases:
    def test_zero_relative_velocity(self):
        result = _engine().evaluate(_scan([_object(10.0, 0.0, vx=0.0)])).results[0]
        assert result.ttc is None

    def test_negative_closing_velocity_moving_away(self):
        result = _engine().evaluate(_scan([_object(10.0, 0.0, vx=3.0)])).results[0]
        assert result.ttc is None

    def test_infinite_undefined_ttc_reported_as_none_not_a_huge_number(self):
        result = _engine().evaluate(_scan([_object(10.0, 0.0, vx=0.0)])).results[0]
        assert result.ttc is None  # not float('inf'), not some large sentinel

    def test_ttc_beyond_prediction_horizon_still_reported(self):
        tiny_speed = DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 1.2
        result = _engine().evaluate(_scan([_object(50.0, 0.0, vx=-tiny_speed)])).results[0]
        assert result.ttc is not None
        assert result.ttc > DEFAULT_SETTINGS.collision_prediction_horizon_s
        assert result.collision_predicted is False  # simulation itself is horizon-bounded


# --- 6. Multiple objects -----------------------------------------------------------------------

class TestMultipleObjects:
    def test_multiple_objects_different_risk_levels(self):
        objects = [_object(30.0, 0.0, track_id="safe1"), _object(4.5, 0.0, track_id="warn1"), _object(1.0, 0.0, track_id="crit1")]
        assessment = _engine().evaluate(_scan(objects))
        levels = {r.track_id: r.risk_level for r in assessment.results}
        assert levels["safe1"] == RiskLevel.SAFE
        assert levels["warn1"] == RiskLevel.WARNING
        assert levels["crit1"] == RiskLevel.CRITICAL

    def test_correct_most_critical_object_selected(self):
        objects = [_object(30.0, 0.0, track_id="safe1"), _object(4.5, 0.0, track_id="warn1"), _object(1.0, 0.0, track_id="crit1")]
        assessment = _engine().evaluate(_scan(objects))
        assert assessment.overall_risk == RiskLevel.CRITICAL
        assert assessment.most_critical_object.track_id == "crit1"

    def test_overall_risk_matches_highest_individual_risk(self):
        objects = [_object(30.0, 0.0, track_id="a"), _object(4.5, 0.0, track_id="b")]
        assessment = _engine().evaluate(_scan(objects))
        assert assessment.overall_risk == RiskLevel.WARNING

    def test_tie_break_prefers_lower_ttc(self):
        # Two CRITICAL objects, different TTC -- most_critical_object should be the more imminent.
        objects = [
            _object(1.9, 0.0, track_id="less-imminent"),
            _object(1.5, 0.0, vx=-3.0, track_id="more-imminent"),
        ]
        assessment = _engine().evaluate(_scan(objects))
        assert assessment.most_critical_object.track_id == "more-imminent"


# --- 7. Boundaries -----------------------------------------------------------------------------

class TestBoundaries:
    def test_object_exactly_on_warning_distance_boundary(self):
        result = _engine().evaluate(_scan([_object(DEFAULT_SETTINGS.collision_warning_distance_m, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.WARNING

    def test_object_exactly_on_critical_distance_boundary(self):
        result = _engine().evaluate(_scan([_object(DEFAULT_SETTINGS.collision_critical_distance_m, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.CRITICAL

    def test_object_exactly_on_safety_margin_lateral_boundary_is_in_path(self):
        from collision.geometry import vehicle_footprint
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        result = _engine().evaluate(_scan([_object(5.0, footprint.envelope_left)])).results[0]
        assert result.in_projected_path is True


# --- Edge cases (spec section 17) ---------------------------------------------------------------

class TestEdgeCases:
    def test_no_tracked_objects(self):
        assessment = _engine().evaluate(_scan([]))
        assert assessment.object_count == 0
        assert assessment.results == []
        assert assessment.overall_risk == RiskLevel.SAFE
        assert assessment.most_critical_object is None

    def test_missing_velocity_does_not_crash_and_notes_the_assumption(self):
        result = _engine().evaluate(_scan([_object(3.0, 0.0, has_velocity=False)])).results[0]
        assert result.risk_level is not None
        assert any("not yet reliable" in r for r in result.reason)

    def test_invalid_geometry_zero_dimensions_does_not_crash(self):
        result = _engine().evaluate(_scan([_object(5.0, 0.0, width=0.0, depth=0.0)])).results[0]
        assert result.risk_level is not None

    def test_object_outside_prediction_horizon_does_not_crash(self):
        result = _engine().evaluate(_scan([_object(100.0, 0.0, vx=-0.01)])).results[0]
        assert result.collision_predicted is False

    def test_multiple_simultaneous_hazards(self):
        objects = [_object(1.0, 0.0, track_id="c1"), _object(1.2, 0.0, track_id="c2", classification=ObjectClassification.POLE_LIKE)]
        assessment = _engine().evaluate(_scan(objects))
        assert assessment.overall_risk == RiskLevel.CRITICAL
        assert sum(1 for r in assessment.results if r.risk_level == RiskLevel.CRITICAL) == 2

    def test_zero_ttc_denominator_never_raises(self):
        # closing_speed exactly 0 must resolve to None, never a ZeroDivisionError.
        _engine().evaluate(_scan([_object(5.0, 0.0, vx=0.0, vy=0.0)]))  # must not raise


# --- Self-return guard -------------------------------------------------------------------------

class TestSelfReturnGuard:
    """A track whose centroid is within min_valid_distance_m of the sensor (a coasting near-origin
    track / ego self return) must not drive risk or TTC -- it ages out via the normal lifecycle."""

    def test_near_origin_track_is_never_critical(self):
        d = DEFAULT_SETTINGS.min_valid_distance_m - 0.1  # ~0.2 m, the live-rig phantom distance
        result = _engine().evaluate(_scan([_object(d, 0.0, vx=-1.0, track_id="phantom")])).results[0]
        assert result.risk_level == RiskLevel.SAFE
        assert result.ttc is None
        assert result.in_projected_path is False
        assert result.collision_predicted is False
        assert any("self-return" in r or "self return" in r for r in result.reason)

    def test_near_origin_track_does_not_raise_overall_risk(self):
        objects = [
            _object(0.15, 0.0, vx=-2.0, track_id="phantom"),   # self return, would be CRITICAL without the guard
            _object(30.0, 0.0, track_id="far"),                 # genuinely safe
        ]
        assessment = _engine().evaluate(_scan(objects))
        assert assessment.overall_risk == RiskLevel.SAFE
        # still counted -- object_count stays honest, it just isn't dangerous
        assert len(assessment.results) == 2

    def test_real_object_just_outside_the_radius_is_still_assessed_normally(self):
        d = DEFAULT_SETTINGS.min_valid_distance_m + 0.5  # 0.8 m -- a real, very close obstacle
        result = _engine().evaluate(_scan([_object(d, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.CRITICAL  # genuinely close & in path


# --- Explainability ------------------------------------------------------------------------------

class TestExplainability:
    def test_every_result_has_a_nonempty_reason(self):
        for obj in [_object(30.0, 0.0), _object(4.5, 0.0), _object(1.0, 0.0)]:
            result = _engine().evaluate(_scan([obj])).results[0]
            assert result.reason
            assert all(isinstance(line, str) and line for line in result.reason)

    def test_critical_reason_mentions_the_triggering_condition(self):
        result = _engine().evaluate(_scan([_object(1.0, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.CRITICAL
        assert any("Distance" in r or "TTC" in r or "intersect" in r for r in result.reason)

    def test_warning_reason_mentions_the_triggering_condition(self):
        result = _engine().evaluate(_scan([_object(4.5, 0.0)])).results[0]
        assert result.risk_level == RiskLevel.WARNING
        assert any("Distance" in r or "TTC" in r or "intersect" in r for r in result.reason)


# --- Non-standard vehicle pose --------------------------------------------------------------------

class TestNonIdentityVehiclePose:
    def test_evaluate_object_directly(self):
        engine = _engine()
        obj = _object(5.0, 0.0)
        result = engine.evaluate_object(obj, VehicleState(), timestamp=0.0)
        assert result.track_id == obj.track_id

    def test_rotated_and_translated_vehicle_pose_does_not_crash_and_shifts_distance(self):
        engine = _engine()
        obj = _object(5.0, 0.0)
        default_result = engine.evaluate_object(obj, VehicleState(), timestamp=0.0)
        shifted_state = VehicleState(pose=VehiclePose(x=2.0, y=0.0, heading=45.0))
        shifted_result = engine.evaluate_object(obj, shifted_state, timestamp=0.0)
        assert shifted_result.distance != default_result.distance

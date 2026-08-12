"""Tests for models.collision: RiskLevel, VehicleState, CollisionRiskResult, CollisionAssessment."""

import pytest
from pydantic import ValidationError

from models.collision import CollisionAssessment, CollisionRiskResult, RiskLevel, VehicleState
from models.mapping import VehiclePose
from models.objects import ObjectClassification, Point2D, Velocity2D


class TestRiskLevel:
    def test_values(self):
        assert RiskLevel.SAFE.value == "safe"
        assert RiskLevel.WARNING.value == "warning"
        assert RiskLevel.CRITICAL.value == "critical"


class TestVehicleState:
    def test_defaults_to_stationary_at_identity_pose(self):
        state = VehicleState()
        assert state.pose == VehiclePose(x=0.0, y=0.0, heading=0.0)
        assert state.speed_mps == 0.0

    def test_accepts_explicit_pose_and_speed(self):
        state = VehicleState(pose=VehiclePose(x=1.0, y=2.0, heading=90.0), speed_mps=5.0)
        assert state.pose.heading == 90.0
        assert state.speed_mps == 5.0


def _result(**overrides) -> CollisionRiskResult:
    defaults = dict(
        track_id="t1", classification=ObjectClassification.VEHICLE_LIKE,
        distance=5.0, relative_position=Point2D(x=5.0, y=0.0), relative_velocity=Velocity2D(vx=0.0, vy=0.0),
        relative_speed=0.0, in_projected_path=True, ttc=None, collision_predicted=False,
        risk_level=RiskLevel.SAFE, reason=["not close"], timestamp=0.0,
    )
    defaults.update(overrides)
    return CollisionRiskResult(**defaults)


class TestCollisionRiskResult:
    def test_valid_construction(self):
        result = _result()
        assert result.risk_level == RiskLevel.SAFE
        assert result.predicted_collision_time is None
        assert result.predicted_collision_position is None

    def test_risk_score_bounds_enforced(self):
        with pytest.raises(ValidationError):
            _result(risk_score=1.5)

    def test_ttc_non_negative(self):
        with pytest.raises(ValidationError):
            _result(ttc=-1.0)

    def test_ttc_none_allowed(self):
        result = _result(ttc=None)
        assert result.ttc is None


class TestCollisionAssessment:
    def test_empty_results_is_valid(self):
        assessment = CollisionAssessment(
            scan_id="s", sequence_number=0, source_id="t", timestamp=0.0,
            results=[], object_count=0, overall_risk=RiskLevel.SAFE, most_critical_object=None,
        )
        assert assessment.object_count == 0
        assert assessment.most_critical_object is None

    def test_holds_multiple_results(self):
        results = [_result(track_id="a"), _result(track_id="b", risk_level=RiskLevel.CRITICAL)]
        assessment = CollisionAssessment(
            scan_id="s", sequence_number=0, source_id="t", timestamp=0.0,
            results=results, object_count=2, overall_risk=RiskLevel.CRITICAL, most_critical_object=results[1],
        )
        assert assessment.object_count == 2
        assert assessment.most_critical_object.track_id == "b"

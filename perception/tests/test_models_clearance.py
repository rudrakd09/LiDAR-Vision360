"""Tests for models.clearance: DirectionalClearance / ClearanceAssessment validation."""

import pytest
from pydantic import ValidationError

from models.clearance import ClearanceAssessment, ClearanceDirection, ClearanceState, DirectionalClearance
from models.objects import Point2D


def _directional(direction: ClearanceDirection, distance: float = 5.0) -> DirectionalClearance:
    return DirectionalClearance(direction=direction, distance_m=distance)


class TestDirectionalClearance:
    def test_nearest_point_defaults_to_none(self):
        d = _directional(ClearanceDirection.FRONT)
        assert d.nearest_point is None

    def test_accepts_a_world_point(self):
        d = DirectionalClearance(direction=ClearanceDirection.LEFT, distance_m=2.0, nearest_point=Point2D(x=0.0, y=2.0))
        assert d.nearest_point.y == 2.0

    def test_negative_distance_is_rejected(self):
        with pytest.raises(ValidationError):
            DirectionalClearance(direction=ClearanceDirection.FRONT, distance_m=-1.0)


class TestClearanceAssessment:
    def test_full_construction(self):
        assessment = ClearanceAssessment(
            scan_id="s", sequence_number=1, source_id="test", timestamp=0.0,
            front=_directional(ClearanceDirection.FRONT, 4.0),
            rear=_directional(ClearanceDirection.REAR, 8.0),
            left=_directional(ClearanceDirection.LEFT, 3.0),
            right=_directional(ClearanceDirection.RIGHT, 6.0),
            min_clearance_m=3.0, min_direction=ClearanceDirection.LEFT,
            corridor_width_m=11.4, overall_status=ClearanceState.SAFE, reason=["ok"],
        )
        assert assessment.min_direction == ClearanceDirection.LEFT
        assert assessment.overall_status == ClearanceState.SAFE

    def test_negative_corridor_width_is_rejected(self):
        with pytest.raises(ValidationError):
            ClearanceAssessment(
                scan_id="s", sequence_number=1, source_id="test", timestamp=0.0,
                front=_directional(ClearanceDirection.FRONT), rear=_directional(ClearanceDirection.REAR),
                left=_directional(ClearanceDirection.LEFT), right=_directional(ClearanceDirection.RIGHT),
                min_clearance_m=5.0, min_direction=ClearanceDirection.FRONT,
                corridor_width_m=-1.0, overall_status=ClearanceState.SAFE, reason=[],
            )

    def test_state_enum_values_match_docs_collision_md(self):
        # docs/collision.md's own Phase 10 stub specified these four exact state names.
        assert {s.value for s in ClearanceState} == {"safe", "caution", "low_clearance", "critical"}

"""Unit tests for `fusion.association` -- spatial/range gating and greedy nearest-neighbor
matching, all against synthetic `DetectedObject`/`RadarTarget` instances.
"""

from __future__ import annotations

import math
import time

import pytest

from common.config import Settings
from fusion.association import find_associations, radar_target_position, radial_velocity_vector
from models.objects import DetectedObject, Point2D
from models.radar import RadarTarget


def _obj(object_id: str, x: float, y: float, distance: float | None = None) -> DetectedObject:
    return DetectedObject(
        object_id=object_id, track_id=object_id, centroid=Point2D(x=x, y=y),
        width=1.0, depth=1.0, distance=distance if distance is not None else math.hypot(x, y),
        timestamp=time.time(),
    )


def _settings(**overrides) -> Settings:
    return Settings(_env_file=None, **overrides)


class TestRadarTargetPosition:
    def test_none_without_angle(self):
        target = RadarTarget(range_m=10.0)
        assert radar_target_position(target) is None

    def test_position_at_zero_degrees(self):
        target = RadarTarget(range_m=10.0, angle_deg=0.0)
        pos = radar_target_position(target)
        assert pos.x == pytest.approx(10.0)
        assert pos.y == pytest.approx(0.0)

    def test_position_at_ninety_degrees(self):
        target = RadarTarget(range_m=10.0, angle_deg=90.0)
        pos = radar_target_position(target)
        assert pos.x == pytest.approx(0.0, abs=1e-9)
        assert pos.y == pytest.approx(10.0)


class TestRadialVelocityVector:
    def test_closing_along_forward_axis(self):
        vx, vy = radial_velocity_vector(velocity_mps=-3.0, bearing_rad=0.0)
        assert vx == pytest.approx(-3.0)
        assert vy == pytest.approx(0.0)

    def test_receding_along_lateral_axis(self):
        vx, vy = radial_velocity_vector(velocity_mps=2.0, bearing_rad=math.pi / 2)
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(2.0)


class TestFindAssociations:
    def test_no_targets_no_associations(self):
        objects = [_obj("o1", 5.0, 0.0)]
        assert find_associations(objects, [], _settings()) == []

    def test_no_objects_no_associations(self):
        targets = [RadarTarget(range_m=5.0, angle_deg=0.0)]
        assert find_associations([], targets, _settings()) == []

    def test_close_match_is_associated(self):
        objects = [_obj("o1", 5.0, 0.0)]
        targets = [RadarTarget(range_m=5.0, angle_deg=0.0)]
        result = find_associations(objects, targets, _settings())
        assert len(result) == 1
        assert result[0].object_index == 0
        assert result[0].target_index == 0

    def test_far_apart_is_not_associated(self):
        objects = [_obj("o1", 5.0, 0.0)]
        targets = [RadarTarget(range_m=20.0, angle_deg=90.0)]  # far away, different bearing
        result = find_associations(objects, targets, _settings(fusion_association_max_distance_m=1.5))
        assert result == []

    def test_range_gate_rejects_even_if_spatially_close(self):
        # Object at (5, 0) (range 5), a target angled to land spatially near it but reporting a
        # very different range -- the independent range gate must still reject this pair.
        objects = [_obj("o1", 5.0, 0.0, distance=5.0)]
        targets = [RadarTarget(range_m=50.0, angle_deg=0.0)]
        result = find_associations(objects, targets, _settings(fusion_association_max_distance_m=100.0, fusion_association_max_range_diff_m=2.0))
        assert result == []

    def test_target_without_angle_is_never_associated(self):
        objects = [_obj("o1", 5.0, 0.0)]
        targets = [RadarTarget(range_m=5.0)]  # no angle_deg
        assert find_associations(objects, targets, _settings()) == []

    def test_one_to_one_nearest_wins(self):
        # Two LiDAR objects near the same radar target -- only the closer one gets it.
        objects = [_obj("near", 5.1, 0.0), _obj("far", 5.5, 0.0)]
        targets = [RadarTarget(range_m=5.0, angle_deg=0.0)]
        result = find_associations(objects, targets, _settings(fusion_association_max_distance_m=1.0))
        assert len(result) == 1
        assert objects[result[0].object_index].object_id == "near"

    def test_each_object_and_target_used_at_most_once(self):
        objects = [_obj("o1", 5.0, 0.0), _obj("o2", 10.0, 0.0)]
        targets = [RadarTarget(range_m=5.0, angle_deg=0.0), RadarTarget(range_m=10.0, angle_deg=0.0)]
        result = find_associations(objects, targets, _settings())
        assert len(result) == 2
        assert {r.object_index for r in result} == {0, 1}
        assert {r.target_index for r in result} == {0, 1}

"""Tests for obstacle intersection and motion (simulator.obstacles)."""

import pytest

from simulator.geometry import Vec2, ray_direction
from simulator.obstacles import PoleObstacle, RectangleObstacle, WallObstacle, velocity_from_heading


class TestWallObstacle:
    def test_intersect_distance(self):
        wall = WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0))
        t = wall.intersect(Vec2(0.0, 0.0), ray_direction(0.0))
        assert t == pytest.approx(5.0)

    def test_step_is_a_no_op(self):
        wall = WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0))
        wall.step(1.0)
        assert wall.p1 == Vec2(5.0, -10.0)
        assert wall.p2 == Vec2(5.0, 10.0)


class TestPoleObstacle:
    def test_intersect_distance(self):
        pole = PoleObstacle(Vec2(0.0, 3.0), radius=0.15)
        t = pole.intersect(Vec2(0.0, 0.0), ray_direction(90.0))
        assert t == pytest.approx(2.85)

    def test_static_pole_does_not_move(self):
        pole = PoleObstacle(Vec2(1.0, 1.0), radius=0.1)
        pole.step(5.0)
        assert pole.center == Vec2(1.0, 1.0)

    def test_moving_pole_updates_position(self):
        velocity = velocity_from_heading(2.0, 90.0)  # 2 m/s in +y
        pole = PoleObstacle(Vec2(0.0, -6.0), radius=0.1, velocity=velocity)
        pole.step(0.5)
        assert pole.center.x == pytest.approx(0.0)
        assert pole.center.y == pytest.approx(-5.0)


class TestRectangleObstacle:
    def test_front_face_intersect_distance(self):
        rect = RectangleObstacle(Vec2(6.0, 0.0), width=1.8, depth=4.5)
        t = rect.intersect(Vec2(0.0, 0.0), ray_direction(0.0))
        assert t == pytest.approx(3.75)

    def test_ray_missing_rectangle_entirely(self):
        rect = RectangleObstacle(Vec2(6.0, 0.0), width=1.8, depth=4.5)
        t = rect.intersect(Vec2(0.0, 0.0), ray_direction(90.0))
        assert t is None

    def test_moving_rectangle_updates_position(self):
        velocity = velocity_from_heading(2.0, 180.0)  # 2 m/s in -x
        rect = RectangleObstacle(Vec2(10.0, 0.0), width=1.8, depth=4.5, velocity=velocity)
        rect.step(1.0)
        assert rect.center.x == pytest.approx(8.0)
        assert rect.center.y == pytest.approx(0.0)


def test_velocity_from_heading_components():
    v = velocity_from_heading(2.0, 0.0)
    assert v.x == pytest.approx(2.0)
    assert v.y == pytest.approx(0.0, abs=1e-9)

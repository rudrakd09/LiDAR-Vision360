"""Tests for simulator.environment.Environment: nearest-hit ray-casting and stepping."""

import pytest

from simulator.environment import Environment
from simulator.geometry import Vec2
from simulator.obstacles import PoleObstacle, WallObstacle, velocity_from_heading


class TestEmptyEnvironment:
    def test_no_obstacles_means_no_return(self):
        env = Environment([])
        for angle in (0.0, 45.0, 90.0, 180.0, 270.0):
            assert env.true_distance(Vec2(0.0, 0.0), angle, range_max_m=12.0) is None


class TestMultipleObstacles:
    def test_nearest_obstacle_wins(self):
        env = Environment(
            [
                WallObstacle(Vec2(8.0, -10.0), Vec2(8.0, 10.0)),
                WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0)),
            ]
        )
        d = env.true_distance(Vec2(0.0, 0.0), 0.0, range_max_m=12.0)
        assert d == pytest.approx(5.0)

    def test_obstacles_at_different_angles_are_independent(self):
        env = Environment(
            [
                WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0)),
                PoleObstacle(Vec2(0.0, 3.0), radius=0.15),
            ]
        )
        assert env.true_distance(Vec2(0.0, 0.0), 0.0, range_max_m=12.0) == pytest.approx(5.0)
        assert env.true_distance(Vec2(0.0, 0.0), 90.0, range_max_m=12.0) == pytest.approx(2.85)


class TestRangeLimits:
    def test_obstacle_beyond_max_range_is_ignored(self):
        env = Environment([WallObstacle(Vec2(20.0, -10.0), Vec2(20.0, 10.0))])
        assert env.true_distance(Vec2(0.0, 0.0), 0.0, range_max_m=12.0) is None

    def test_obstacle_within_max_range_is_detected(self):
        env = Environment([WallObstacle(Vec2(10.0, -10.0), Vec2(10.0, 10.0))])
        assert env.true_distance(Vec2(0.0, 0.0), 0.0, range_max_m=12.0) == pytest.approx(10.0)


class TestStep:
    def test_step_advances_moving_obstacles_and_leaves_static_ones(self):
        wall = WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0))
        pole = PoleObstacle(Vec2(0.0, -6.0), radius=0.1, velocity=velocity_from_heading(2.0, 90.0))
        env = Environment([wall, pole])
        env.step(0.5)
        assert wall.p1 == Vec2(5.0, -10.0)
        assert pole.center.y == pytest.approx(-5.0)

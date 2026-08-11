"""Tests for the ray-casting geometry primitives (simulator.geometry)."""

import math

import pytest

from simulator.geometry import Vec2, intersect_ray_circle, intersect_ray_segment, rectangle_corners, ray_direction


class TestRayDirection:
    def test_angle_zero_points_along_positive_x(self):
        d = ray_direction(0.0)
        assert d.x == pytest.approx(1.0)
        assert d.y == pytest.approx(0.0, abs=1e-9)

    def test_angle_ninety_points_along_positive_y(self):
        d = ray_direction(90.0)
        assert d.x == pytest.approx(0.0, abs=1e-9)
        assert d.y == pytest.approx(1.0)


class TestIntersectRaySegment:
    def test_perpendicular_wall_hit_at_expected_distance(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        t = intersect_ray_segment(origin, direction, Vec2(5.0, -10.0), Vec2(5.0, 10.0))
        assert t == pytest.approx(5.0)

    def test_segment_not_crossed_by_ray_is_a_miss(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        # Segment entirely above the ray's line (y in [5, 10]) -- the ray travels along y=0.
        t = intersect_ray_segment(origin, direction, Vec2(5.0, 5.0), Vec2(5.0, 10.0))
        assert t is None

    def test_segment_behind_ray_origin_is_a_miss(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        t = intersect_ray_segment(origin, direction, Vec2(-5.0, -10.0), Vec2(-5.0, 10.0))
        assert t is None

    def test_parallel_segment_is_a_miss(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        t = intersect_ray_segment(origin, direction, Vec2(1.0, 1.0), Vec2(5.0, 1.0))
        assert t is None


class TestIntersectRayCircle:
    def test_pole_hit_at_expected_surface_distance(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(90.0)
        t = intersect_ray_circle(origin, direction, Vec2(0.0, 3.0), 0.15)
        assert t == pytest.approx(2.85)

    def test_circle_behind_ray_is_a_miss(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(90.0)
        t = intersect_ray_circle(origin, direction, Vec2(0.0, -3.0), 0.15)
        assert t is None

    def test_circle_off_axis_is_a_miss(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        t = intersect_ray_circle(origin, direction, Vec2(5.0, 5.0), 0.5)
        assert t is None

    def test_origin_inside_circle_returns_exit_point(self):
        origin = Vec2(0.0, 0.0)
        direction = ray_direction(0.0)
        t = intersect_ray_circle(origin, direction, Vec2(0.0, 0.0), 2.0)
        assert t == pytest.approx(2.0)


class TestRectangleCorners:
    def test_axis_aligned_corners(self):
        corners = rectangle_corners(Vec2(0.0, 0.0), width=2.0, depth=4.0, rotation_deg=0.0)
        expected = {(2.0, 1.0), (2.0, -1.0), (-2.0, -1.0), (-2.0, 1.0)}
        actual = {(round(c.x, 6), round(c.y, 6)) for c in corners}
        assert actual == expected

    def test_rotation_preserves_corner_distance_from_center(self):
        center = Vec2(3.0, -2.0)
        corners = rectangle_corners(center, width=2.0, depth=4.0, rotation_deg=37.0)
        expected_radius = math.hypot(2.0, 1.0)
        for c in corners:
            assert math.hypot(c.x - center.x, c.y - center.y) == pytest.approx(expected_radius)

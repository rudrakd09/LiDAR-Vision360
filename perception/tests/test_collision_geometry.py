"""Tests for collision.geometry: vehicle footprint, frame rotation/translation, relative motion,
and the projected-path membership test."""

import math

import pytest

from collision.geometry import (
    in_projected_path,
    object_half_extents,
    relative_motion,
    rotate_to_heading,
    to_vehicle_frame,
    vehicle_footprint,
    vehicle_velocity_vector,
)
from common.config import Settings
from models.collision import VehicleState
from models.mapping import VehiclePose
from models.objects import Point2D, Velocity2D

DEFAULT_SETTINGS = Settings(_env_file=None)


class TestVehicleFootprint:
    def test_half_extents_from_settings(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert footprint.half_length_front == pytest.approx(DEFAULT_SETTINGS.vehicle_length_m / 2.0)
        assert footprint.half_width == pytest.approx(DEFAULT_SETTINGS.vehicle_width_m / 2.0)

    def test_envelope_adds_margins(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert footprint.envelope_front == pytest.approx(footprint.half_length_front + DEFAULT_SETTINGS.front_safety_margin_m)
        assert footprint.envelope_rear == pytest.approx(footprint.half_length_rear + DEFAULT_SETTINGS.rear_safety_margin_m)
        assert footprint.envelope_left == pytest.approx(footprint.half_width + DEFAULT_SETTINGS.left_safety_margin_m)
        assert footprint.envelope_right == pytest.approx(footprint.half_width + DEFAULT_SETTINGS.right_safety_margin_m)

    def test_front_and_rear_margin_asymmetric_by_default(self):
        # This project's default: more buffer ahead than behind.
        assert DEFAULT_SETTINGS.front_safety_margin_m > DEFAULT_SETTINGS.rear_safety_margin_m


class TestObjectHalfExtents:
    def test_along_uses_depth_lateral_uses_width(self):
        half_along, half_lateral = object_half_extents(width=2.0, depth=1.0, settings=DEFAULT_SETTINGS)
        assert half_along == pytest.approx(0.5)
        assert half_lateral == pytest.approx(1.0)

    def test_wide_but_shallow_object_does_not_inflate_along_extent(self):
        # A vehicle's visible near-face: wide (lateral) but very shallow (radial), matching what
        # a 2D LiDAR actually observes -- the along-axis half-extent must track `depth`, not the
        # much larger `width`.
        half_along, half_lateral = object_half_extents(width=1.7, depth=0.05, settings=DEFAULT_SETTINGS)
        assert half_along < half_lateral

    def test_floored_at_minimum_radius(self):
        half_along, half_lateral = object_half_extents(width=0.01, depth=0.01, settings=DEFAULT_SETTINGS)
        assert half_along == pytest.approx(DEFAULT_SETTINGS.collision_minimum_object_radius_m)
        assert half_lateral == pytest.approx(DEFAULT_SETTINGS.collision_minimum_object_radius_m)


class TestRotateToHeading:
    def test_zero_heading_is_identity(self):
        assert rotate_to_heading(3.0, 4.0, 0.0) == pytest.approx((3.0, 4.0))

    def test_90_degrees(self):
        # Vehicle facing world +y (heading=90): world +x is then to the vehicle's *right*
        # (facing north, east is on your right) -> lateral is negative.
        along, lateral = rotate_to_heading(1.0, 0.0, 90.0)
        assert (along, lateral) == pytest.approx((0.0, -1.0), abs=1e-9)

    def test_180_degrees_negates_both(self):
        along, lateral = rotate_to_heading(1.0, 0.0, 180.0)
        assert (along, lateral) == pytest.approx((-1.0, 0.0), abs=1e-9)

    def test_270_degrees(self):
        # Vehicle facing world -y (heading=270): world +x is then to the vehicle's *left*.
        along, lateral = rotate_to_heading(1.0, 0.0, 270.0)
        assert (along, lateral) == pytest.approx((0.0, 1.0), abs=1e-9)


class TestToVehicleFrame:
    def test_translates_by_vehicle_position(self):
        state = VehicleState(pose=VehiclePose(x=2.0, y=3.0, heading=0.0))
        along, lateral = to_vehicle_frame(5.0, 3.0, state)
        assert (along, lateral) == pytest.approx((3.0, 0.0))

    def test_rotates_by_vehicle_heading(self):
        state = VehicleState(pose=VehiclePose(x=0.0, y=0.0, heading=90.0))
        along, lateral = to_vehicle_frame(0.0, 5.0, state)
        assert (along, lateral) == pytest.approx((5.0, 0.0), abs=1e-9)


class TestVehicleVelocityVector:
    def test_stationary_is_zero(self):
        assert vehicle_velocity_vector(VehicleState()) == (0.0, 0.0)

    def test_moving_forward_along_heading_zero(self):
        vx, vy = vehicle_velocity_vector(VehicleState(speed_mps=5.0))
        assert (vx, vy) == pytest.approx((5.0, 0.0))

    def test_moving_forward_along_heading_90(self):
        vx, vy = vehicle_velocity_vector(VehicleState(pose=VehiclePose(heading=90.0), speed_mps=5.0))
        assert (vx, vy) == pytest.approx((0.0, 5.0), abs=1e-9)


class TestRelativeMotion:
    def test_stationary_vehicle_stationary_object(self):
        position, velocity = relative_motion(Point2D(x=5.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), VehicleState())
        assert (position.x, position.y) == (5.0, 0.0)
        assert (velocity.vx, velocity.vy) == (0.0, 0.0)

    def test_missing_object_velocity_treated_as_stationary(self):
        position, velocity = relative_motion(Point2D(x=5.0, y=0.0), None, VehicleState())
        assert (velocity.vx, velocity.vy) == (0.0, 0.0)

    def test_moving_vehicle_subtracts_its_velocity(self):
        position, velocity = relative_motion(Point2D(x=5.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), VehicleState(speed_mps=3.0))
        # Object stationary in world frame, vehicle moving forward at 3 m/s -> object appears to
        # approach at -3 m/s in the vehicle's relative frame.
        assert velocity.vx == pytest.approx(-3.0)

    def test_position_relative_to_moving_vehicle_pose(self):
        state = VehicleState(pose=VehiclePose(x=2.0, y=0.0))
        position, _ = relative_motion(Point2D(x=5.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), state)
        assert position.x == pytest.approx(3.0)


class TestInProjectedPath:
    def test_directly_ahead_is_in_path(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert in_projected_path(Point2D(x=5.0, y=0.0), VehicleState(), footprint, DEFAULT_SETTINGS) is True

    def test_far_to_the_side_is_not_in_path(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert in_projected_path(Point2D(x=5.0, y=10.0), VehicleState(), footprint, DEFAULT_SETTINGS) is False

    def test_behind_within_rear_margin_is_in_path(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        behind = -(footprint.envelope_rear - 0.1)
        assert in_projected_path(Point2D(x=behind, y=0.0), VehicleState(), footprint, DEFAULT_SETTINGS) is True

    def test_far_behind_is_not_in_path(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert in_projected_path(Point2D(x=-50.0, y=0.0), VehicleState(), footprint, DEFAULT_SETTINGS) is False

    def test_beyond_sensor_range_is_not_in_path(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        beyond = DEFAULT_SETTINGS.lidar_range_max_m + 1.0
        assert in_projected_path(Point2D(x=beyond, y=0.0), VehicleState(), footprint, DEFAULT_SETTINGS) is False

    def test_lateral_boundary_exact(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        assert in_projected_path(Point2D(x=5.0, y=footprint.envelope_left), VehicleState(), footprint, DEFAULT_SETTINGS) is True
        just_outside = footprint.envelope_left + 0.001
        assert in_projected_path(Point2D(x=5.0, y=just_outside), VehicleState(), footprint, DEFAULT_SETTINGS) is False

    def test_independent_of_vehicle_speed(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        stationary = in_projected_path(Point2D(x=5.0, y=0.0), VehicleState(speed_mps=0.0), footprint, DEFAULT_SETTINGS)
        moving = in_projected_path(Point2D(x=5.0, y=0.0), VehicleState(speed_mps=10.0), footprint, DEFAULT_SETTINGS)
        assert stationary == moving is True

    def test_rotates_with_heading(self):
        footprint = vehicle_footprint(DEFAULT_SETTINGS)
        state = VehicleState(pose=VehiclePose(heading=90.0))
        # "ahead" for a vehicle facing +y is world (0, 5), not (5, 0).
        assert in_projected_path(Point2D(x=0.0, y=5.0), state, footprint, DEFAULT_SETTINGS) is True
        assert in_projected_path(Point2D(x=5.0, y=0.0), state, footprint, DEFAULT_SETTINGS) is False

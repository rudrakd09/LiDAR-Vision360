"""Tests for collision.prediction.simulate_collision: the discrete footprint-intersection
simulation, and specifically its ability to correctly distinguish a laterally-crossing object
that does vs. does not actually enter the vehicle's path -- see docs/collision.md "Collision
prediction" and the 07_moving_crossing scenario discussion."""

import pytest

from collision.geometry import vehicle_footprint
from collision.prediction import simulate_collision
from common.config import Settings
from models.collision import VehicleState
from models.mapping import VehiclePose
from models.objects import Point2D, Velocity2D

DEFAULT_SETTINGS = Settings(_env_file=None)
FOOTPRINT = vehicle_footprint(DEFAULT_SETTINGS)


class TestApproachingObjectPredictsCollision:
    def test_direct_approach_is_predicted(self):
        predicted, t, position = simulate_collision(
            Point2D(x=10.0, y=0.0), Velocity2D(vx=-5.0, vy=0.0), VehicleState(), FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS,
        )
        assert predicted is True
        assert t is not None and t > 0.0
        assert position is not None

    def test_predicted_time_is_consistent_with_hand_computation(self):
        # Object at 10m, closing at 5 m/s -> reaches the (envelope_front + object_half) contact
        # gap at approximately (10 - envelope_front - 0.5) / 5 seconds, within one sim step.
        predicted, t, _position = simulate_collision(
            Point2D(x=10.0, y=0.0), Velocity2D(vx=-5.0, vy=0.0), VehicleState(), FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS,
        )
        expected = (10.0 - FOOTPRINT.envelope_front - 0.5) / 5.0
        assert t == pytest.approx(expected, abs=DEFAULT_SETTINGS.collision_simulation_step_s * 1.5)


class TestNonApproachingObjectDoesNotPredictCollision:
    def test_stationary_object_out_of_reach(self):
        predicted, t, position = simulate_collision(
            Point2D(x=20.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), VehicleState(), FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS,
        )
        assert predicted is False
        assert t is None
        assert position is None

    def test_receding_object_never_predicted(self):
        predicted, _t, _position = simulate_collision(
            Point2D(x=5.0, y=0.0), Velocity2D(vx=5.0, vy=0.0), VehicleState(), FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS,
        )
        assert predicted is False


class TestCrossingObject:
    def test_crossing_into_the_lane_is_predicted_when_vehicle_also_closes_longitudinal_gap(self):
        # Object well ahead and to the side, moving laterally toward the vehicle's lane; vehicle
        # closing the longitudinal gap at the same time -- footprints should overlap somewhere
        # within the horizon.
        vehicle = VehicleState(speed_mps=2.0)
        predicted, t, position = simulate_collision(
            Point2D(x=6.0, y=-6.0), Velocity2D(vx=0.0, vy=1.5), vehicle, FOOTPRINT, 0.5, 0.5, DEFAULT_SETTINGS,
        )
        assert predicted is True
        assert t is not None
        assert position is not None

    def test_crossing_away_from_the_lane_is_never_predicted(self):
        vehicle = VehicleState(speed_mps=2.0)
        predicted, _t, _position = simulate_collision(
            Point2D(x=6.0, y=-6.0), Velocity2D(vx=0.0, vy=-1.5), vehicle, FOOTPRINT, 0.5, 0.5, DEFAULT_SETTINGS,
        )
        assert predicted is False

    def test_crossing_far_ahead_out_of_vehicle_reach_is_not_predicted(self):
        # Stationary vehicle -- the longitudinal gap never closes, so a purely lateral crossing
        # far ahead must never be flagged, no matter how directly it crosses y=0.
        vehicle = VehicleState(speed_mps=0.0)
        predicted, _t, _position = simulate_collision(
            Point2D(x=6.0, y=-6.0), Velocity2D(vx=0.0, vy=1.5), vehicle, FOOTPRINT, 0.5, 0.5, DEFAULT_SETTINGS,
        )
        assert predicted is False


class TestPredictionHorizonRespected:
    def test_collision_just_beyond_horizon_is_not_predicted(self):
        settings = Settings(_env_file=None, collision_prediction_horizon_s=1.0, collision_simulation_step_s=0.05)
        footprint = vehicle_footprint(settings)
        # Very slow approach that would only reach contact well after the 1s horizon.
        predicted, _t, _position = simulate_collision(
            Point2D(x=50.0, y=0.0), Velocity2D(vx=-1.0, vy=0.0), VehicleState(), footprint, 1.0, 1.0, settings,
        )
        assert predicted is False

    def test_collision_just_within_horizon_is_predicted(self):
        settings = Settings(_env_file=None, collision_prediction_horizon_s=5.0, collision_simulation_step_s=0.05)
        footprint = vehicle_footprint(settings)
        predicted, t, _position = simulate_collision(
            Point2D(x=10.0, y=0.0), Velocity2D(vx=-3.0, vy=0.0), VehicleState(), footprint, 1.0, 1.0, settings,
        )
        assert predicted is True
        assert t <= 5.0


class TestNoCrashOnDegenerateInputs:
    def test_zero_step_setting_does_not_infinite_loop(self):
        settings = Settings(_env_file=None, collision_simulation_step_s=0.0, collision_prediction_horizon_s=2.0)
        footprint = vehicle_footprint(settings)
        # Must terminate (defense-in-depth fallback inside simulate_collision), not hang.
        simulate_collision(Point2D(x=5.0, y=0.0), Velocity2D(vx=-1.0, vy=0.0), VehicleState(), footprint, 1.0, 1.0, settings)

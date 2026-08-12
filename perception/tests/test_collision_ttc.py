"""Tests for collision.ttc: closed-form longitudinal TTC and closing_speed_along.

Covers the spec's explicit TTC requirements: known analytic scenarios, zero/negative relative
velocity, already-overlapping footprints, and the noise-floor gate -- see docs/collision.md
"TTC calculation"."""

import pytest

from collision.geometry import vehicle_footprint
from collision.ttc import closing_speed_along, compute_ttc
from common.config import Settings
from models.objects import Point2D, Velocity2D

DEFAULT_SETTINGS = Settings(_env_file=None)
FOOTPRINT = vehicle_footprint(DEFAULT_SETTINGS)


class TestKnownAnalyticScenario:
    def test_head_on_approach_matches_hand_computed_value(self):
        # Object 20m ahead, closing at 5 m/s (vehicle stationary, object approaching at -5 m/s).
        # gap = 20 - envelope_front - object_half_extent(0.5) [object 1x1m -> half=0.5]
        position = Point2D(x=20.0, y=0.0)
        velocity = Velocity2D(vx=-5.0, vy=0.0)
        ttc = compute_ttc(position, velocity, 0.0, FOOTPRINT, object_width=1.0, object_depth=1.0, settings=DEFAULT_SETTINGS)
        expected_gap = 20.0 - FOOTPRINT.envelope_front - 0.5
        assert ttc == pytest.approx(expected_gap / 5.0)

    def test_rear_approach_matches_hand_computed_value(self):
        # Object 10m behind, closing at 3 m/s (object approaching from behind at +3 m/s).
        position = Point2D(x=-10.0, y=0.0)
        velocity = Velocity2D(vx=3.0, vy=0.0)
        ttc = compute_ttc(position, velocity, 0.0, FOOTPRINT, object_width=1.0, object_depth=1.0, settings=DEFAULT_SETTINGS)
        expected_gap = 10.0 - FOOTPRINT.envelope_rear - 0.5
        assert ttc == pytest.approx(expected_gap / 3.0)


class TestZeroRelativeVelocity:
    def test_zero_velocity_gives_undefined_ttc(self):
        ttc = compute_ttc(Point2D(x=10.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc is None


class TestNegativeClosingVelocity:
    def test_moving_away_gives_none_not_negative_ttc(self):
        ttc = compute_ttc(Point2D(x=10.0, y=0.0), Velocity2D(vx=5.0, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc is None


class TestAlreadyOverlapping:
    def test_footprints_already_touching_gives_zero(self):
        ttc = compute_ttc(Point2D(x=0.5, y=0.0), Velocity2D(vx=-1.0, vy=0.0), 0.0, FOOTPRINT, 0.1, 0.1, DEFAULT_SETTINGS)
        assert ttc == 0.0

    def test_object_at_vehicle_center_gives_zero(self):
        ttc = compute_ttc(Point2D(x=0.0, y=0.0), Velocity2D(vx=-1.0, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc == 0.0


class TestNoiseFloor:
    def test_closing_speed_at_or_below_minimum_is_undefined(self):
        tiny_speed = DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 0.5
        ttc = compute_ttc(Point2D(x=10.0, y=0.0), Velocity2D(vx=-tiny_speed, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc is None

    def test_closing_speed_comfortably_above_minimum_is_defined(self):
        ttc = compute_ttc(Point2D(x=10.0, y=0.0), Velocity2D(vx=-1.0, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc is not None


class TestNoDivisionByZero:
    def test_never_raises_for_a_grid_of_positions_and_velocities(self):
        for x in (-20.0, -1.0, 0.0, 1.0, 20.0):
            for vx in (-5.0, 0.0, 5.0):
                compute_ttc(Point2D(x=x, y=0.0), Velocity2D(vx=vx, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)  # must not raise


class TestClosingSpeedAlong:
    def test_positive_when_approaching_from_ahead(self):
        speed = closing_speed_along(Point2D(x=10.0, y=0.0), Velocity2D(vx=-3.0, vy=0.0), 0.0)
        assert speed == pytest.approx(3.0)

    def test_negative_when_receding_ahead(self):
        speed = closing_speed_along(Point2D(x=10.0, y=0.0), Velocity2D(vx=3.0, vy=0.0), 0.0)
        assert speed == pytest.approx(-3.0)

    def test_positive_when_approaching_from_behind(self):
        speed = closing_speed_along(Point2D(x=-10.0, y=0.0), Velocity2D(vx=3.0, vy=0.0), 0.0)
        assert speed == pytest.approx(3.0)

    def test_zero_when_stationary(self):
        speed = closing_speed_along(Point2D(x=10.0, y=0.0), Velocity2D(vx=0.0, vy=0.0), 0.0)
        assert speed == pytest.approx(0.0)


class TestTTCBeyondPredictionHorizon:
    def test_ttc_itself_has_no_horizon_cap(self):
        # A very slow approach (large TTC, likely beyond collision_prediction_horizon_s) is still
        # a well-defined, real number from compute_ttc -- the horizon only bounds the discrete
        # simulation (collision.prediction), not this closed-form estimate. See docs/collision.md
        # "Clear separation of concepts".
        slow_speed = DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 1.5
        ttc = compute_ttc(Point2D(x=50.0, y=0.0), Velocity2D(vx=-slow_speed, vy=0.0), 0.0, FOOTPRINT, 1.0, 1.0, DEFAULT_SETTINGS)
        assert ttc is not None
        assert ttc > DEFAULT_SETTINGS.collision_prediction_horizon_s

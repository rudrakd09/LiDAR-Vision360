"""Tests for tracking.kalman.KalmanFilter2D: the constant-velocity model in isolation, with no
dependency on models/Track/ObjectTracker."""

import math

import pytest

from tracking.kalman import KalmanFilter2D

DEFAULTS = dict(
    process_noise_std=0.5,
    measurement_noise_std_m=0.15,
    initial_position_uncertainty_m=1.0,
    initial_velocity_uncertainty_mps=5.0,
)


def _filter(x=0.0, y=0.0, vx=0.0, vy=0.0) -> KalmanFilter2D:
    return KalmanFilter2D(x=x, y=y, vx=vx, vy=vy, **DEFAULTS)


class TestConstruction:
    def test_initial_state_exposed_via_properties(self):
        kf = _filter(x=1.0, y=2.0, vx=0.5, vy=-0.5)
        assert kf.x == 1.0
        assert kf.y == 2.0
        assert kf.vx == 0.5
        assert kf.vy == -0.5

    def test_default_velocity_is_zero(self):
        kf = _filter(x=1.0, y=2.0)
        assert kf.vx == 0.0
        assert kf.vy == 0.0


class TestPredict:
    def test_predict_advances_position_by_velocity_times_dt(self):
        kf = _filter(x=0.0, y=0.0, vx=2.0, vy=-1.0)
        x, y = kf.predict(dt=1.0)
        assert x == pytest.approx(2.0)
        assert y == pytest.approx(-1.0)
        assert kf.x == pytest.approx(2.0)
        assert kf.y == pytest.approx(-1.0)

    def test_predict_does_not_change_velocity(self):
        kf = _filter(x=0.0, y=0.0, vx=2.0, vy=-1.0)
        kf.predict(dt=0.5)
        assert kf.vx == pytest.approx(2.0)
        assert kf.vy == pytest.approx(-1.0)

    def test_predict_with_zero_dt_is_a_no_op_on_the_mean(self):
        kf = _filter(x=3.0, y=4.0, vx=1.0, vy=1.0)
        x, y = kf.predict(dt=0.0)
        assert (x, y) == pytest.approx((3.0, 4.0))

    def test_predict_grows_covariance(self):
        kf = _filter()
        trace_before = kf.P.trace()
        kf.predict(dt=0.1)
        assert kf.P.trace() > trace_before

    def test_repeated_predict_without_update_compounds_displacement(self):
        kf = _filter(x=0.0, y=0.0, vx=1.0, vy=0.0)
        for _ in range(10):
            kf.predict(dt=0.1)
        assert kf.x == pytest.approx(1.0, abs=1e-9)


class TestUpdate:
    def test_update_moves_estimate_toward_measurement(self):
        kf = _filter(x=0.0, y=0.0)
        kf.predict(dt=0.1)  # still at (0, 0), covariance grown
        kf.update(5.0, 5.0)
        # The corrected estimate must land strictly between the pre-update prediction (0, 0) and
        # the measurement (5, 5) -- a basic Kalman-correctness sanity check.
        assert 0.0 < kf.x < 5.0
        assert 0.0 < kf.y < 5.0

    def test_update_shrinks_covariance(self):
        kf = _filter()
        kf.predict(dt=0.1)
        trace_before = kf.P.trace()
        kf.update(0.0, 0.0)
        assert kf.P.trace() < trace_before

    def test_repeated_identical_measurements_converge_position_and_zero_velocity(self):
        kf = _filter(x=10.0, y=10.0, vx=3.0, vy=3.0)  # deliberately wrong initial velocity
        for _ in range(30):
            kf.predict(dt=0.1)
            kf.update(0.0, 0.0)
        assert kf.x == pytest.approx(0.0, abs=0.05)
        assert kf.y == pytest.approx(0.0, abs=0.05)
        assert kf.vx == pytest.approx(0.0, abs=0.2)
        assert kf.vy == pytest.approx(0.0, abs=0.2)


class TestConvergesToTrueConstantVelocity:
    @pytest.mark.parametrize("vx_true,vy_true", [(1.5, 0.0), (0.0, -2.0), (1.0, 1.0)])
    def test_velocity_estimate_converges(self, vx_true, vy_true):
        dt = 0.1
        x, y = 0.0, 0.0
        kf = _filter(x=x, y=y)
        for _ in range(40):
            x += vx_true * dt
            y += vy_true * dt
            kf.predict(dt=dt)
            kf.update(x, y)
        assert kf.vx == pytest.approx(vx_true, abs=0.05)
        assert kf.vy == pytest.approx(vy_true, abs=0.05)


class TestPeekPredict:
    def test_peek_predict_does_not_mutate_state(self):
        kf = _filter(x=1.0, y=2.0, vx=1.0, vy=1.0)
        state_before = kf.state.copy()
        p_before = kf.P.copy()
        kf.peek_predict(dt=5.0)
        assert (kf.state == state_before).all()
        assert (kf.P == p_before).all()

    def test_peek_predict_matches_what_predict_would_produce(self):
        kf_a = _filter(x=1.0, y=2.0, vx=0.5, vy=-0.5)
        kf_b = _filter(x=1.0, y=2.0, vx=0.5, vy=-0.5)
        peeked = kf_a.peek_predict(dt=0.3)
        advanced = kf_b.predict(dt=0.3)
        assert peeked == pytest.approx(advanced)

    def test_peek_predict_with_negative_dt_is_clamped_to_zero(self):
        kf = _filter(x=1.0, y=2.0, vx=1.0, vy=1.0)
        x, y = kf.peek_predict(dt=-1.0)
        assert (x, y) == pytest.approx((1.0, 2.0))

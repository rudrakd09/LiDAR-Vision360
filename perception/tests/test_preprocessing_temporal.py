"""Tests for preprocessing.temporal.TemporalFilter."""

import pytest

from models.lidar import LiDARPoint
from preprocessing.temporal import TemporalFilter


def _scan(distance: float, angle: float = 0.0) -> list[LiDARPoint]:
    return [LiDARPoint(angle=angle, distance=distance, timestamp=0.0)]


class TestBasicBehavior:
    def test_first_observation_passes_through_unfiltered(self):
        result = TemporalFilter(alpha=0.5).apply(_scan(5.0))
        assert result[0].distance == 5.0

    def test_matches_the_exponential_formula_exactly(self):
        alpha = 0.3
        temporal_filter = TemporalFilter(alpha=alpha)
        first = temporal_filter.apply(_scan(10.0))[0].distance
        second = temporal_filter.apply(_scan(0.0))[0].distance
        expected_second = alpha * 0.0 + (1.0 - alpha) * first
        assert second == pytest.approx(expected_second, abs=1e-6)

    def test_constant_input_stays_constant(self):
        temporal_filter = TemporalFilter(alpha=0.4)
        for _ in range(10):
            result = temporal_filter.apply(_scan(7.0))
        assert result[0].distance == pytest.approx(7.0, abs=1e-6)

    def test_reset_clears_history(self):
        temporal_filter = TemporalFilter(alpha=0.5)
        temporal_filter.apply(_scan(10.0))
        temporal_filter.reset()
        result = temporal_filter.apply(_scan(3.0))
        assert result[0].distance == 3.0  # treated as a fresh first observation


class TestPerAngleIsolation:
    def test_different_angles_are_smoothed_independently(self):
        temporal_filter = TemporalFilter(alpha=0.5)
        temporal_filter.apply([LiDARPoint(angle=0.0, distance=10.0, timestamp=0.0), LiDARPoint(angle=90.0, distance=2.0, timestamp=0.0)])
        result = temporal_filter.apply([LiDARPoint(angle=0.0, distance=0.0, timestamp=1.0), LiDARPoint(angle=90.0, distance=2.0, timestamp=1.0)])
        by_angle = {p.angle: p.distance for p in result}
        assert by_angle[0.0] == pytest.approx(5.0, abs=1e-6)  # 0.5*0 + 0.5*10
        assert by_angle[90.0] == pytest.approx(2.0, abs=1e-6)  # unchanged input -> unchanged output


class TestEnableDisable:
    def test_disabled_by_default_in_settings(self):
        from common.config import Settings

        assert Settings(_env_file=None).preprocessing_temporal_filter_enabled is False


class TestMovingObstacleLag:
    def test_lag_stays_bounded_for_a_constant_velocity_approach(self):
        # 08_approaching_obstacle-style signal: distance shrinks by 0.2m per scan (2 m/s @ 10Hz).
        alpha = 0.5
        temporal_filter = TemporalFilter(alpha=alpha)
        raw_distances = [10.0 - 0.2 * i for i in range(20)]

        filtered_last = None
        for d in raw_distances:
            filtered_last = temporal_filter.apply(_scan(d))[0].distance

        # For a constant-slope ramp, an exponential filter's steady-state lag is a fixed offset,
        # with the filter trailing *above* the (decreasing) raw signal:
        # lag = (1 - alpha) / alpha * step_size. For alpha=0.5, step=0.2m -> lag = +0.2m.
        raw_last = raw_distances[-1]
        expected_lag = (1.0 - alpha) / alpha * 0.2
        assert filtered_last - raw_last == pytest.approx(expected_lag, abs=0.01)
        assert abs(filtered_last - raw_last) < 1.0  # sanity bound: nowhere near a full scan behind

    def test_higher_alpha_reduces_lag_magnitude(self):
        raw_distances = [10.0 - 0.2 * i for i in range(20)]

        def final_lag_magnitude(alpha: float) -> float:
            temporal_filter = TemporalFilter(alpha=alpha)
            filtered_last = None
            for d in raw_distances:
                filtered_last = temporal_filter.apply(_scan(d))[0].distance
            return abs(filtered_last - raw_distances[-1])

        assert final_lag_magnitude(alpha=0.8) < final_lag_magnitude(alpha=0.3)

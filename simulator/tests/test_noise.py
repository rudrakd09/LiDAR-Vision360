"""Tests for simulator.noise.NoiseModel: Gaussian noise, outliers, missing measurements, and
deterministic behavior under a fixed random seed."""

import pytest

from simulator.noise import NoiseConfig, NoiseModel

RANGE_MIN, RANGE_MAX = 0.05, 12.0


class TestNoReturnHandling:
    def test_none_true_distance_reports_max_range(self):
        model = NoiseModel(NoiseConfig())
        distance, valid = model.apply(None, RANGE_MIN, RANGE_MAX)
        assert valid is True
        assert distance == pytest.approx(RANGE_MAX)


class TestGaussianNoise:
    def test_zero_std_is_exact(self):
        model = NoiseModel(NoiseConfig(distance_noise_std_m=0.0, seed=1))
        distance, valid = model.apply(5.0, RANGE_MIN, RANGE_MAX)
        assert valid is True
        assert distance == pytest.approx(5.0)

    def test_nonzero_std_perturbs_distance_within_bounds(self):
        model = NoiseModel(NoiseConfig(distance_noise_std_m=0.1, seed=1))
        distance, valid = model.apply(5.0, RANGE_MIN, RANGE_MAX)
        assert valid is True
        # 5 standard deviations is an astronomically unlikely miss for a Gaussian; this bound
        # is about sanity-checking the noise is applied and clamped, not exact reproduction.
        assert abs(distance - 5.0) < 0.5


class TestMissingMeasurements:
    def test_probability_one_always_reports_missing(self):
        model = NoiseModel(NoiseConfig(missing_probability=1.0, seed=1))
        for _ in range(20):
            distance, valid = model.apply(5.0, RANGE_MIN, RANGE_MAX)
            assert valid is False
            assert distance == 0.0

    def test_probability_zero_never_reports_missing(self):
        model = NoiseModel(NoiseConfig(missing_probability=0.0, seed=1))
        for _ in range(50):
            _, valid = model.apply(5.0, RANGE_MIN, RANGE_MAX)
            assert valid is True


class TestOutliers:
    def test_probability_one_produces_value_within_range(self):
        model = NoiseModel(NoiseConfig(outlier_probability=1.0, distance_noise_std_m=0.0, seed=2))
        results = [model.apply(0.5, RANGE_MIN, RANGE_MAX)[0] for _ in range(20)]
        assert all(RANGE_MIN <= d <= RANGE_MAX for d in results)
        # An outlier replacing a true distance of 0.5m should, virtually always, land far away.
        assert any(abs(d - 0.5) > 0.5 for d in results)

    def test_probability_zero_never_produces_outliers_beyond_noise(self):
        model = NoiseModel(NoiseConfig(outlier_probability=0.0, distance_noise_std_m=0.0, seed=2))
        for _ in range(50):
            distance, _ = model.apply(5.0, RANGE_MIN, RANGE_MAX)
            assert distance == pytest.approx(5.0)


class TestDeterminism:
    def test_same_seed_produces_identical_sequence(self):
        true_distances = [5.0, None, 3.2, 0.0, 8.8, 11.9, 1.0] * 3
        model_a = NoiseModel(NoiseConfig(distance_noise_std_m=0.1, outlier_probability=0.1, missing_probability=0.1, seed=99))
        model_b = NoiseModel(NoiseConfig(distance_noise_std_m=0.1, outlier_probability=0.1, missing_probability=0.1, seed=99))

        results_a = [model_a.apply(d, RANGE_MIN, RANGE_MAX) for d in true_distances]
        results_b = [model_b.apply(d, RANGE_MIN, RANGE_MAX) for d in true_distances]

        assert results_a == results_b

    def test_different_seeds_diverge(self):
        true_distances = [5.0] * 10
        model_a = NoiseModel(NoiseConfig(distance_noise_std_m=0.2, seed=1))
        model_b = NoiseModel(NoiseConfig(distance_noise_std_m=0.2, seed=2))

        results_a = [model_a.apply(d, RANGE_MIN, RANGE_MAX) for d in true_distances]
        results_b = [model_b.apply(d, RANGE_MIN, RANGE_MAX) for d in true_distances]

        assert results_a != results_b

"""Tests for collision.metrics: collision_prediction_accuracy, ttc_error, risk_level_accuracy."""

import pytest

from collision.metrics import collision_prediction_accuracy, risk_level_accuracy, ttc_error


class TestCollisionPredictionAccuracy:
    def test_perfect_match(self):
        result = collision_prediction_accuracy([True, False, True], [True, False, True])
        assert result["accuracy"] == 1.0
        assert result["false_positive_rate"] == 0.0
        assert result["false_negative_rate"] == 0.0

    def test_false_positive(self):
        # predicted=[True, False], ground_truth=[False, False] -> 1 false positive, 1 true
        # negative -> fpr = fp / (fp + tn) = 1 / 2.
        result = collision_prediction_accuracy([True, False], [False, False])
        assert result["false_positives"] == 1
        assert result["false_positive_rate"] == 0.5

    def test_false_negative(self):
        # predicted=[False, False], ground_truth=[True, False] -> 1 false negative, 1 true
        # negative -> fnr = fn / (fn + tp) = 1 / 1 (tp=0).
        result = collision_prediction_accuracy([False, False], [True, False])
        assert result["false_negatives"] == 1
        assert result["false_negative_rate"] == 1.0

    def test_empty_input_returns_zero_not_error(self):
        result = collision_prediction_accuracy([], [])
        assert result["accuracy"] == 0.0

    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError):
            collision_prediction_accuracy([True], [True, False])


class TestTTCError:
    def test_exact_match_zero_error(self):
        result = ttc_error([2.0, 3.0], [2.0, 3.0])
        assert result["mean_error_s"] == 0.0
        assert result["n"] == 2

    def test_known_error(self):
        result = ttc_error([2.0], [3.0])
        assert result["mean_error_s"] == pytest.approx(1.0)
        assert result["max_error_s"] == pytest.approx(1.0)

    def test_none_pairs_excluded_not_treated_as_error(self):
        result = ttc_error([None, 2.0], [None, 2.0])
        assert result["n"] == 1
        assert result["mean_error_s"] == 0.0

    def test_all_none_returns_zero_not_error(self):
        result = ttc_error([None, None], [None, None])
        assert result["n"] == 0
        assert result["mean_error_s"] == 0.0

    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError):
            ttc_error([1.0], [1.0, 2.0])


class TestRiskLevelAccuracy:
    def test_perfect_match(self):
        result = risk_level_accuracy(["safe", "warning", "critical"], ["safe", "warning", "critical"])
        assert result["accuracy"] == 1.0

    def test_confusion_counts(self):
        result = risk_level_accuracy(["warning", "safe"], ["critical", "safe"])
        assert result["confusion"]["critical"]["warning"] == 1
        assert result["confusion"]["safe"]["safe"] == 1

    def test_empty_input(self):
        result = risk_level_accuracy([], [])
        assert result["accuracy"] == 0.0

    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError):
            risk_level_accuracy(["safe"], ["safe", "warning"])

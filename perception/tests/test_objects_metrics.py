"""Tests for objects.metrics: generic accuracy/precision/recall/F1/confusion-matrix computation."""

import pytest

from objects.metrics import classification_metrics, confusion_matrix


class TestConfusionMatrix:
    def test_perfect_predictions(self):
        matrix = confusion_matrix(["wall", "pole", "wall"], ["wall", "pole", "wall"])
        assert matrix["wall"]["wall"] == 2
        assert matrix["pole"]["pole"] == 1
        assert matrix["wall"]["pole"] == 0

    def test_includes_labels_seen_only_in_predictions(self):
        matrix = confusion_matrix(["wall"], ["pole"])
        assert matrix["wall"]["pole"] == 1
        assert matrix["pole"]["wall"] == 0  # pole never appeared as truth, still present at 0


class TestClassificationMetrics:
    def test_all_correct_gives_perfect_scores(self):
        y_true = ["wall", "wall", "pole", "pole"]
        y_pred = ["wall", "wall", "pole", "pole"]
        result = classification_metrics(y_true, y_pred)
        assert result["accuracy"] == pytest.approx(1.0)
        assert result["macro_precision"] == pytest.approx(1.0)
        assert result["macro_recall"] == pytest.approx(1.0)
        assert result["macro_f1"] == pytest.approx(1.0)

    def test_all_wrong_gives_zero_accuracy(self):
        y_true = ["wall", "wall"]
        y_pred = ["pole", "pole"]
        result = classification_metrics(y_true, y_pred)
        assert result["accuracy"] == 0.0

    def test_known_precision_recall_values(self):
        # wall: 2 true, predicted 3 times (2 correct, 1 false positive from a pole) ->
        # precision = 2/3, recall = 2/2 = 1.0
        y_true = ["wall", "wall", "pole"]
        y_pred = ["wall", "wall", "wall"]
        result = classification_metrics(y_true, y_pred)
        assert result["per_class"]["wall"]["precision"] == pytest.approx(2 / 3)
        assert result["per_class"]["wall"]["recall"] == pytest.approx(1.0)
        assert result["per_class"]["pole"]["precision"] == pytest.approx(0.0)
        assert result["per_class"]["pole"]["recall"] == pytest.approx(0.0)

    def test_f1_is_harmonic_mean_of_precision_and_recall(self):
        y_true = ["wall", "wall", "wall", "pole"]
        y_pred = ["wall", "wall", "pole", "pole"]
        result = classification_metrics(y_true, y_pred)
        p, r = result["per_class"]["wall"]["precision"], result["per_class"]["wall"]["recall"]
        expected_f1 = 2 * p * r / (p + r) if (p + r) else 0.0
        assert result["per_class"]["wall"]["f1"] == pytest.approx(expected_f1)

    def test_class_with_no_support_and_no_predictions_scores_zero_not_error(self):
        y_true = ["wall"]
        y_pred = ["pole"]
        result = classification_metrics(y_true, y_pred)
        # "wall" was never predicted -> precision undefined -> 0.0, not a ZeroDivisionError
        assert result["per_class"]["wall"]["precision"] == 0.0
        assert result["per_class"]["pole"]["recall"] == 0.0  # pole never true

    def test_empty_input_does_not_crash(self):
        result = classification_metrics([], [])
        assert result["accuracy"] == 0.0
        assert result["confusion_matrix"] == {}

    def test_mismatched_lengths_raises(self):
        with pytest.raises(ValueError):
            classification_metrics(["wall"], ["wall", "pole"])

    def test_support_counts_are_correct(self):
        y_true = ["wall", "wall", "pole"]
        y_pred = ["wall", "pole", "pole"]
        result = classification_metrics(y_true, y_pred)
        assert result["per_class"]["wall"]["support"] == 2
        assert result["per_class"]["pole"]["support"] == 1

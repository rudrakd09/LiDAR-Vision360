"""Tests for mapping.metrics: occupancy_confusion_counts, occupancy_accuracy_metrics."""

import numpy as np
import pytest

from mapping.metrics import occupancy_accuracy_metrics, occupancy_confusion_counts
from models.mapping import CellState

U, F, O = CellState.UNKNOWN, CellState.FREE, CellState.OCCUPIED


class TestOccupancyConfusionCounts:
    def test_perfect_match(self):
        predicted = np.array([[O, F], [F, O]], dtype=np.int8)
        ground_truth = np.array([[O, F], [F, O]], dtype=np.int8)
        counts = occupancy_confusion_counts(predicted, ground_truth)
        assert counts == {"true_positive": 2, "false_positive": 0, "false_negative": 0, "true_negative": 2}

    def test_false_positive(self):
        predicted = np.array([[O, F]], dtype=np.int8)
        ground_truth = np.array([[F, F]], dtype=np.int8)
        counts = occupancy_confusion_counts(predicted, ground_truth)
        assert counts["false_positive"] == 1
        assert counts["true_positive"] == 0

    def test_false_negative(self):
        predicted = np.array([[F, F]], dtype=np.int8)
        ground_truth = np.array([[O, F]], dtype=np.int8)
        counts = occupancy_confusion_counts(predicted, ground_truth)
        assert counts["false_negative"] == 1

    def test_unknown_on_predicted_side_excluded(self):
        predicted = np.array([[U, F]], dtype=np.int8)
        ground_truth = np.array([[O, F]], dtype=np.int8)
        counts = occupancy_confusion_counts(predicted, ground_truth)
        # The UNKNOWN cell must not count as a false negative (predicted-UNKNOWN != predicted-FREE).
        assert counts["false_negative"] == 0
        assert sum(counts.values()) == 1  # only the second cell is evaluated

    def test_unknown_on_ground_truth_side_excluded(self):
        predicted = np.array([[O, F]], dtype=np.int8)
        ground_truth = np.array([[U, F]], dtype=np.int8)
        counts = occupancy_confusion_counts(predicted, ground_truth)
        assert counts["false_positive"] == 0
        assert sum(counts.values()) == 1

    def test_shape_mismatch_raises(self):
        with pytest.raises(ValueError):
            occupancy_confusion_counts(np.zeros((2, 2), dtype=np.int8), np.zeros((3, 3), dtype=np.int8))


class TestOccupancyAccuracyMetrics:
    def test_perfect_prediction(self):
        predicted = np.array([[O, F], [F, O]], dtype=np.int8)
        ground_truth = predicted.copy()
        metrics = occupancy_accuracy_metrics(predicted, ground_truth)
        assert metrics["precision"] == pytest.approx(1.0)
        assert metrics["recall"] == pytest.approx(1.0)
        assert metrics["iou"] == pytest.approx(1.0)
        assert metrics["false_occupied_cells"] == 0
        assert metrics["false_free_cells"] == 0

    def test_no_occupied_cells_anywhere_returns_zero_not_error(self):
        predicted = np.full((3, 3), F, dtype=np.int8)
        ground_truth = np.full((3, 3), F, dtype=np.int8)
        metrics = occupancy_accuracy_metrics(predicted, ground_truth)
        assert metrics["precision"] == 0.0
        assert metrics["recall"] == 0.0
        assert metrics["iou"] == 0.0

    def test_known_precision_recall(self):
        # 2 predicted occupied (1 correct, 1 wrong); 2 true occupied (1 found, 1 missed).
        predicted = np.array([O, O, F, F], dtype=np.int8)
        ground_truth = np.array([O, F, O, F], dtype=np.int8)
        metrics = occupancy_accuracy_metrics(predicted, ground_truth)
        assert metrics["precision"] == pytest.approx(0.5)
        assert metrics["recall"] == pytest.approx(0.5)
        assert metrics["false_occupied_cells"] == 1
        assert metrics["false_free_cells"] == 1

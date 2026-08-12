"""Generic occupancy-grid ground-truth evaluation: precision/recall/IoU for the OCCUPIED class,
plus false-occupied/false-free cell counts, given two same-shaped `CellState`-coded grid arrays.

Deliberately has no dependency on `simulator` -- rasterizing a scenario's known obstacle geometry
into a comparable ground-truth grid is scenario-specific knowledge that belongs in the evaluation
script (`scripts/evaluate_mapping.py`, which already depends on both `simulator` and
`perception`). Mirrors the exact same separation `objects.metrics` (Phase 6) and
`tracking.metrics` (Phase 7) established for classification/tracking. See docs/mapping.md
"Ground-truth evaluation."
"""

from __future__ import annotations

import numpy as np

from models.mapping import CellState


def occupancy_confusion_counts(predicted: np.ndarray, ground_truth: np.ndarray) -> dict:
    """`predicted`/`ground_truth`: same-shape arrays of `CellState`-coded ints.

    Returns `{"true_positive", "false_positive", "false_negative", "true_negative"}` cell counts
    for the binary "is this cell OCCUPIED" question. A cell that is `UNKNOWN` on *either* side is
    excluded from every count -- there is no ground truth to check against for a cell outside the
    ground-truth rasterization's own knowledge, and a predicted-`UNKNOWN` cell is an honest "not
    yet observed," not a wrong answer, so it is not penalized as if it had guessed FREE. Raises
    `ValueError` on a shape mismatch.
    """
    if predicted.shape != ground_truth.shape:
        raise ValueError(f"predicted and ground_truth must have the same shape, got {predicted.shape} and {ground_truth.shape}")

    evaluated = (predicted != CellState.UNKNOWN) & (ground_truth != CellState.UNKNOWN)
    predicted_occupied = (predicted == CellState.OCCUPIED) & evaluated
    truth_occupied = (ground_truth == CellState.OCCUPIED) & evaluated

    true_positive = int(np.count_nonzero(predicted_occupied & truth_occupied))
    false_positive = int(np.count_nonzero(predicted_occupied & ~truth_occupied))
    false_negative = int(np.count_nonzero(~predicted_occupied & truth_occupied))
    true_negative = int(np.count_nonzero(~predicted_occupied & ~truth_occupied & evaluated))

    return {
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "true_negative": true_negative,
    }


def occupancy_accuracy_metrics(predicted: np.ndarray, ground_truth: np.ndarray) -> dict:
    """Precision/recall/IoU/false-occupied/false-free for the OCCUPIED class, from
    `occupancy_confusion_counts`. Every ratio is `0.0` (not a division error) when its
    denominator is `0` -- e.g. a scene with no ground-truth-occupied cells at all.
    """
    counts = occupancy_confusion_counts(predicted, ground_truth)
    tp, fp, fn, tn = counts["true_positive"], counts["false_positive"], counts["false_negative"], counts["true_negative"]

    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    iou = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0

    return {
        "precision": precision,
        "recall": recall,
        "iou": iou,
        "false_occupied_cells": fp,  # predicted OCCUPIED, ground truth says otherwise
        "false_free_cells": fn,  # predicted FREE (UNKNOWN already excluded), ground truth says OCCUPIED
        "true_positive_cells": tp,
        "true_negative_cells": tn,
        "cells_evaluated": tp + fp + fn + tn,
    }

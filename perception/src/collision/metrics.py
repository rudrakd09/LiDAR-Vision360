"""Generic collision-risk ground-truth evaluation: collision-prediction accuracy (false positive/
negative rate), TTC error, and risk-level classification accuracy, given parallel predicted and
ground-truth sequences.

Deliberately has no dependency on `simulator` -- deriving ground truth (e.g. "does the simulator's
own known constant-velocity obstacle motion actually reach the vehicle's footprint, and when") is
scenario-specific knowledge that belongs in the evaluation script (`scripts/evaluate_collision.py`,
which already depends on both `simulator` and `perception`), mirroring the exact separation
`objects.metrics` (Phase 6), `tracking.metrics` (Phase 7), and `mapping.metrics` (Phase 8) each
already established. See docs/collision.md "Ground-truth evaluation".
"""

from __future__ import annotations

import statistics
from typing import Sequence


def collision_prediction_accuracy(predicted: Sequence[bool], ground_truth: Sequence[bool]) -> dict:
    """Accuracy, false-positive rate, and false-negative rate for the binary
    `collision_predicted` decision, from parallel predicted/ground-truth boolean sequences.
    Raises `ValueError` on a length mismatch; returns all-zero for empty input."""
    if len(predicted) != len(ground_truth):
        raise ValueError(f"predicted and ground_truth must be the same length, got {len(predicted)} and {len(ground_truth)}")

    n = len(predicted)
    if n == 0:
        return {
            "accuracy": 0.0, "false_positive_rate": 0.0, "false_negative_rate": 0.0,
            "true_positives": 0, "false_positives": 0, "true_negatives": 0, "false_negatives": 0,
        }

    true_positives = sum(1 for p, g in zip(predicted, ground_truth) if p and g)
    false_positives = sum(1 for p, g in zip(predicted, ground_truth) if p and not g)
    true_negatives = sum(1 for p, g in zip(predicted, ground_truth) if not p and not g)
    false_negatives = sum(1 for p, g in zip(predicted, ground_truth) if not p and g)

    return {
        "accuracy": (true_positives + true_negatives) / n,
        "false_positive_rate": false_positives / (false_positives + true_negatives) if (false_positives + true_negatives) else 0.0,
        "false_negative_rate": false_negatives / (false_negatives + true_positives) if (false_negatives + true_positives) else 0.0,
        "true_positives": true_positives, "false_positives": false_positives,
        "true_negatives": true_negatives, "false_negatives": false_negatives,
    }


def ttc_error(predicted_ttc: Sequence[float | None], ground_truth_ttc: Sequence[float | None]) -> dict:
    """Mean/max absolute TTC error (seconds), over pairs where *both* sides report a finite TTC.
    A `None` on either side (categorically "not approaching"/"undefined," not a number) is
    excluded from this specific numeric-error metric rather than treated as an error of some
    magnitude -- see `docs/collision.md "Ground-truth evaluation"` for how a `None`-vs-finite
    *disagreement* is instead captured by `collision_prediction_accuracy`. Raises `ValueError` on
    a length mismatch; returns all-zero (with `n=0`) if no pair has both sides finite."""
    if len(predicted_ttc) != len(ground_truth_ttc):
        raise ValueError(f"predicted_ttc and ground_truth_ttc must be the same length, got {len(predicted_ttc)} and {len(ground_truth_ttc)}")

    errors = [abs(p - g) for p, g in zip(predicted_ttc, ground_truth_ttc) if p is not None and g is not None]
    if not errors:
        return {"mean_error_s": 0.0, "max_error_s": 0.0, "n": 0}
    return {"mean_error_s": statistics.fmean(errors), "max_error_s": max(errors), "n": len(errors)}


def risk_level_accuracy(predicted: Sequence[str], ground_truth: Sequence[str]) -> dict:
    """Exact-match accuracy plus a confusion-count table for the 3-class SAFE/WARNING/CRITICAL
    decision, from parallel label sequences. A lighter-weight cousin of `objects.metrics.
    classification_metrics` -- 3 ordered classes don't need a full per-class precision/recall/F1
    breakdown to see whether errors are near-misses (SAFE vs WARNING) or gross (SAFE vs
    CRITICAL); the confusion table already shows that directly. Raises `ValueError` on a length
    mismatch; returns all-zero for empty input."""
    if len(predicted) != len(ground_truth):
        raise ValueError(f"predicted and ground_truth must be the same length, got {len(predicted)} and {len(ground_truth)}")

    n = len(predicted)
    if n == 0:
        return {"accuracy": 0.0, "confusion": {}}

    correct = sum(1 for p, g in zip(predicted, ground_truth) if p == g)
    labels = sorted(set(predicted) | set(ground_truth))
    confusion = {true_label: {pred_label: 0 for pred_label in labels} for true_label in labels}
    for predicted_label, true_label in zip(predicted, ground_truth):
        confusion[true_label][predicted_label] += 1

    return {"accuracy": correct / n, "confusion": confusion}

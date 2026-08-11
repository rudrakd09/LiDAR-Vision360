"""Generic classification-quality metrics: accuracy, precision, recall, F1, and a confusion
matrix, computed from parallel lists of true/predicted labels.

Deliberately has no dependency on `simulator` or on any notion of "which scenario has which
ground truth" -- that mapping is scenario-specific knowledge that belongs in the evaluation
script (`scripts/evaluate_classification.py`), which already depends on both `simulator` and
`perception`. This module is the reusable, generic metric computation underneath it, and is
directly unit-testable with synthetic label lists. See docs/object-classification.md
"Classification quality metrics".
"""

from __future__ import annotations

from collections import Counter
from typing import Sequence


def confusion_matrix(y_true: Sequence[str], y_pred: Sequence[str]) -> dict[str, dict[str, int]]:
    """`{true_label: {predicted_label: count}}`, including every label seen in either sequence
    (so an all-correct or all-wrong class still appears with its zero counts)."""
    labels = sorted(set(y_true) | set(y_pred))
    matrix = {t: {p: 0 for p in labels} for t in labels}
    for t, p in zip(y_true, y_pred):
        matrix[t][p] += 1
    return matrix


def classification_metrics(y_true: Sequence[str], y_pred: Sequence[str]) -> dict:
    """Accuracy, and per-class + macro-averaged precision/recall/F1, from parallel label lists.

    Returns `{"accuracy": float, "per_class": {label: {"precision", "recall", "f1", "support"}},
    "macro_precision": float, "macro_recall": float, "macro_f1": float,
    "confusion_matrix": {...}}`. All metrics are `0.0` (not a division error) for classes with
    no support. Empty input returns all-zero metrics rather than raising.
    """
    n = len(y_true)
    if n != len(y_pred):
        raise ValueError(f"y_true and y_pred must be the same length, got {n} and {len(y_pred)}")

    matrix = confusion_matrix(y_true, y_pred)
    labels = sorted(set(y_true) | set(y_pred))

    correct = sum(1 for t, p in zip(y_true, y_pred) if t == p)
    accuracy = correct / n if n else 0.0

    true_counts = Counter(y_true)
    pred_counts = Counter(y_pred)

    per_class = {}
    for label in labels:
        true_positive = matrix[label][label]
        support = true_counts[label]
        predicted = pred_counts[label]

        precision = true_positive / predicted if predicted else 0.0
        recall = true_positive / support if support else 0.0
        f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) else 0.0

        per_class[label] = {"precision": precision, "recall": recall, "f1": f1, "support": support}

    macro_precision = sum(m["precision"] for m in per_class.values()) / len(labels) if labels else 0.0
    macro_recall = sum(m["recall"] for m in per_class.values()) / len(labels) if labels else 0.0
    macro_f1 = sum(m["f1"] for m in per_class.values()) / len(labels) if labels else 0.0

    return {
        "accuracy": accuracy,
        "per_class": per_class,
        "macro_precision": macro_precision,
        "macro_recall": macro_recall,
        "macro_f1": macro_f1,
        "confusion_matrix": matrix,
    }

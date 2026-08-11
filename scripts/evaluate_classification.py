#!/usr/bin/env python
"""Phase 6 evaluation script: run the classifier against simulator scenarios with *known* ground
truth (since the simulator built each scenario, we know what's actually in it) and report
accuracy/precision/recall/F1/confusion matrix.

IMPORTANT (per this phase's spec): these metrics are only meaningful for the scenarios/classes
they're actually computed over -- printed explicitly below. A class never present in these
scenarios (e.g. PERSON_LIKE and LARGE_OBSTACLE, and UNKNOWN as a *true* label) has no ground
truth here and is not evaluated; see docs/object-classification.md "Classification quality
metrics" for why (the simulator has no person-shaped or ambiguous-by-design obstacle scenario).

Ground truth is assigned per detected object by matching its centroid against each scenario's
*known* obstacle positions (since this script -- not `perception` -- is the one place allowed to
depend on both `simulator` and `perception`'s classifier, mirroring every earlier phase's
evaluation/comparison scripts). A detected object that doesn't confidently match any known
obstacle (typically a small stray fragment) is excluded from the metrics and reported separately,
not silently dropped.

Usage:
    python scripts/evaluate_classification.py
"""

from __future__ import annotations

import math
from typing import Callable, Union

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from models.objects import DetectedObject
from objects import GeometricClassifier, classification_metrics
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def _match_multiple_obstacles(obj: DetectedObject) -> str | None:
    """Ground truth for `05_multiple_obstacles`, from the scenario's own known obstacle
    positions (simulator/scenarios/05_multiple_obstacles.json): two poles, one rotated
    vehicle-like rectangle, and a background wall."""
    cx, cy = obj.centroid.x, obj.centroid.y
    known_points = [((0.0, 3.0), "pole_like"), ((4.0, 1.0), "pole_like"), ((5.0, -2.0), "vehicle_like")]
    for (kx, ky), label in known_points:
        if math.hypot(cx - kx, cy - ky) < 1.5:
            return label
    if cx > 7.0:  # the background wall sits at x=9; occlusion can fragment it (see docs/clustering.md)
        return "wall"
    return None


# Each scenario's ground truth: either a single label every detected object should be (the
# common case -- the scenario contains exactly one kind of obstacle), or a per-object matcher
# function for scenarios with multiple different obstacle types.
GROUND_TRUTH: dict[str, Union[str, Callable[[DetectedObject], "str | None"]]] = {
    "02_wall_in_front": "wall",
    "03_pole_left": "pole_like",
    "04_vehicle_ahead": "vehicle_like",
    "05_multiple_obstacles": _match_multiple_obstacles,
    "06_narrow_corridor": "wall",
    "07_moving_crossing": "pole_like",
    "08_approaching_obstacle": "vehicle_like",
    "09_noisy_lidar": "wall",
    "10_missing_outliers": "wall",
}


def _ground_truth_for(scenario_id: str, obj: DetectedObject) -> str | None:
    entry = GROUND_TRUTH[scenario_id]
    return entry if isinstance(entry, str) else entry(obj)


def evaluate() -> None:
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()

    all_true: list[str] = []
    all_pred: list[str] = []
    unmatched_total = 0

    print("Per-scenario results (ground truth from the simulator's own known scenario composition):\n")
    for scenario_id, ground_truth in GROUND_TRUTH.items():
        source = make_data_source(scenario_id)
        with source:
            raw = source.read_scan()
        clean = preprocessor.process(raw)
        cartesian = transformer.transform(clean)
        clustered = clusterer.cluster(cartesian)
        classified = classifier.classify(clustered)

        scenario_true, scenario_pred, unmatched = [], [], 0
        for obj in classified.objects:
            true_label = _ground_truth_for(scenario_id, obj)
            if true_label is None:
                unmatched += 1
                continue
            scenario_true.append(true_label)
            scenario_pred.append(obj.classification.value)

        all_true.extend(scenario_true)
        all_pred.extend(scenario_pred)
        unmatched_total += unmatched

        correct = sum(1 for t, p in zip(scenario_true, scenario_pred) if t == p)
        print(f"  {scenario_id:<24} {len(scenario_true)} evaluated, {correct} correct, {unmatched} unmatched (no ground truth)")
        for t, p in zip(scenario_true, scenario_pred):
            marker = "OK " if t == p else "ERR"
            print(f"      [{marker}] true={t:<14} predicted={p}")

    print(f"\nTotal evaluated objects: {len(all_true)} (unmatched/excluded: {unmatched_total})")
    print(f"Classes with ground truth in this evaluation: {sorted(set(all_true))}")
    print("NOT evaluated here (no ground-truth scenario available): person_like, large_obstacle, unknown-as-true-label\n")

    if not all_true:
        print("No evaluable objects -- nothing to report.")
        return

    metrics = classification_metrics(all_true, all_pred)
    print(f"Accuracy: {metrics['accuracy']:.3f}")
    print(f"Macro precision: {metrics['macro_precision']:.3f}")
    print(f"Macro recall: {metrics['macro_recall']:.3f}")
    print(f"Macro F1: {metrics['macro_f1']:.3f}\n")

    print("Per-class:")
    for label, m in sorted(metrics["per_class"].items()):
        print(f"  {label:<14} precision={m['precision']:.3f} recall={m['recall']:.3f} f1={m['f1']:.3f} support={m['support']}")

    print("\nConfusion matrix (rows=true, columns=predicted):")
    labels = sorted(metrics["confusion_matrix"].keys())
    header = " " * 16 + " ".join(f"{lbl[:10]:>10}" for lbl in labels)
    print(header)
    for true_label in labels:
        row = " ".join(f"{metrics['confusion_matrix'][true_label][pred_label]:>10}" for pred_label in labels)
        print(f"  {true_label:<14} {row}")


if __name__ == "__main__":
    setup_logging()
    evaluate()

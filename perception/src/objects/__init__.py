"""Geometry-based shape classification (Phase 6): scores each `ObstacleCluster` in a
`ClusteredScan` against a handful of explainable geometric rules and reports the best-supported
category (or `UNKNOWN` if nothing clears the confidence threshold), never claiming general-purpose
object recognition -- see docs/object-classification.md.

Status: **implemented**. Independent of `simulator` and of every later pipeline stage (tracking,
mapping, collision, clearance, Unity, cloud) -- see docs/architecture.md.

Typical usage:

    from objects import GeometricClassifier

    classifier = GeometricClassifier()  # reads defaults from common.config.Settings
    classified_scan = classifier.classify(clustered_scan)
"""

from .classifier import GeometricClassifier, classify_scan
from .features import extract_features
from .metrics import classification_metrics, confusion_matrix
from .scoring import score_large_obstacle, score_person, score_pole, score_vehicle, score_wall

__all__ = [
    "GeometricClassifier",
    "classify_scan",
    "extract_features",
    "score_wall",
    "score_pole",
    "score_vehicle",
    "score_person",
    "score_large_obstacle",
    "classification_metrics",
    "confusion_matrix",
]

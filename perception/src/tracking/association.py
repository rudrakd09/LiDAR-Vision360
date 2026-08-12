"""Nearest-neighbour object association: matches this scan's classified detections against
existing tracks' Kalman-predicted positions.

Greedy nearest-neighbour, not the Hungarian/optimal assignment algorithm -- a deliberate, simple
first cut per this phase's spec ("initially implement a simple robust method"). See
docs/tracking.md "Association algorithm" for why this is adequate for the scenarios this phase
targets (few, well-separated obstacles) and what a future upgrade (`scipy.optimize.
linear_sum_assignment`, already available transitively via `scikit-learn`) would change.
"""

from __future__ import annotations

import math

from common.config import Settings
from models.objects import DetectedObject, ObjectClassification

from .track import Track


def _association_cost(track: Track, detection: DetectedObject, settings: Settings) -> float | None:
    """Cost of associating `track` with `detection`, or `None` if the pair is infeasible (outside
    `tracking_max_association_distance_m`) -- the only *hard* gate. Dimension and classification
    differences are soft penalties layered on top, never exclusions on their own: a cluster's
    width/depth estimate is noisy scan-to-scan, and classification can legitimately flip near
    `objects.classification_min_confidence` (see docs/object-classification.md) -- per this
    phase's spec, classification must never be mandatory for association.
    """
    predicted_x, predicted_y = track.kf.x, track.kf.y
    distance = math.hypot(detection.centroid.x - predicted_x, detection.centroid.y - predicted_y)
    if distance > settings.tracking_max_association_distance_m:
        return None

    cost = distance

    dimension_diff = abs(detection.width - track.width) + abs(detection.depth - track.depth)
    cost += settings.tracking_dimension_cost_weight * min(dimension_diff, settings.tracking_max_dimension_diff_m)

    if (
        track.classification != ObjectClassification.UNKNOWN
        and detection.classification != ObjectClassification.UNKNOWN
        and track.classification != detection.classification
    ):
        cost += settings.tracking_classification_mismatch_penalty

    return cost


def associate(
    tracks: list[Track], detections: list[DetectedObject], settings: Settings
) -> tuple[dict[str, int], list[int], list[str]]:
    """Associate `detections` (this scan's `ClassifiedScan.objects`) against `tracks` (already
    `predict()`-ed to this scan's timestamp).

    Returns `(matches, unmatched_detection_indices, unmatched_track_ids)`:
    - `matches`: `{track_id: detection_index}`, each track and each detection index used at
      most once.
    - `unmatched_detection_indices`: indices into `detections` that started no match (candidates
      for new-track creation).
    - `unmatched_track_ids`: `track_id`s that matched no detection (candidates for `mark_missed`).

    Every feasible (track, detection) pair -- cost is not `None`, see `_association_cost` -- is
    considered in ascending-cost order and greedily assigned if both sides are still free.
    """
    candidates: list[tuple[float, str, int]] = []
    for track in tracks:
        for detection_index, detection in enumerate(detections):
            cost = _association_cost(track, detection, settings)
            if cost is not None:
                candidates.append((cost, track.track_id, detection_index))
    candidates.sort(key=lambda candidate: candidate[0])

    matches: dict[str, int] = {}
    matched_detection_indices: set[int] = set()
    matched_track_ids: set[str] = set()
    for _cost, track_id, detection_index in candidates:
        if track_id in matched_track_ids or detection_index in matched_detection_indices:
            continue
        matches[track_id] = detection_index
        matched_track_ids.add(track_id)
        matched_detection_indices.add(detection_index)

    unmatched_detections = [i for i in range(len(detections)) if i not in matched_detection_indices]
    unmatched_track_ids = [track.track_id for track in tracks if track.track_id not in matched_track_ids]
    return matches, unmatched_detections, unmatched_track_ids

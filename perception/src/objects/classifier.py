"""The classification pipeline: `ClusteredScan` -> `ClassifiedScan`.

    ClusteredScan
         |
    for each ObstacleCluster:
        extract_features()           -> ShapeFeatures
        score_wall / score_pole /
        score_vehicle / score_person  -> five (score, reasons) pairs
        score_large_obstacle()
        pick the best score
        best >= classification_min_confidence ?  -> that category
                                       :  -> UNKNOWN (confidence = best score anyway, for
                                                       transparency about how close it came)
        map ObstacleCluster + ShapeFeatures + classification onto a DetectedObject
         |
    ClassifiedScan

This module has no dependency on `simulator` or any later pipeline stage (tracking, mapping,
collision, clearance, Unity, cloud) -- same architecture boundary as every prior phase, see
docs/architecture.md.
"""

from __future__ import annotations

from common.config import Settings, get_settings
from common.logging import get_logger
from models.classification import ClassifiedScan
from models.clustering import ClusteredScan, ObstacleCluster
from models.objects import BoundingBox, DetectedObject, ObjectClassification, Point2D, ShapeFeatures

from .features import extract_features
from .scoring import score_large_obstacle, score_person, score_pole, score_vehicle, score_wall

logger = get_logger(__name__)

_TIE_EPSILON = 0.02  # scores within this of each other are treated as a tie -- see priority order below


class GeometricClassifier:
    """Scores each cluster in a `ClusteredScan` against explainable geometric rules and reports
    the best-supported category, or `UNKNOWN` if nothing clears the confidence threshold.

    **Not general-purpose object recognition.** This is geometry-based shape classification from
    a 2D 360° LiDAR scan -- see docs/object-classification.md for exactly what that does and does
    not support.
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()

    def classify(self, scan: ClusteredScan) -> ClassifiedScan:
        objects = [self.classify_cluster(cluster) for cluster in scan.clusters]
        return ClassifiedScan(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            objects=objects,
            noise_points=scan.noise_points,
            object_count=len(objects),
            noise_count=scan.noise_count,
        )

    def classify_cluster(self, cluster: ObstacleCluster) -> DetectedObject:
        features = extract_features(cluster)
        settings = self.settings

        wall_score, wall_reasons = score_wall(features, settings)
        pole_score, pole_reasons = score_pole(features, settings)
        vehicle_score, vehicle_reasons = score_vehicle(features, settings)
        person_score, person_reasons = score_person(features, settings)
        best_specific = max(wall_score, pole_score, vehicle_score, person_score)
        large_score, large_reasons = score_large_obstacle(features, settings, best_specific)

        # Priority order used only to break near-ties (within _TIE_EPSILON): WALL and POLE_LIKE
        # require only shape evidence (linearity/circularity + a size gate) with no size *range*
        # assumption, so they're preferred over VEHICLE_LIKE/LARGE_OBSTACLE/PERSON_LIKE, which
        # additionally assume a specific configured size envelope actually matches this sensor
        # class of obstacle -- a more specific (and so more fragile) claim. Documented, not
        # incidental: relying on raw floating-point score equality to decide ties is fragile and
        # was observed to flip unpredictably between near-identical scores in testing -- see
        # docs/object-classification.md "Parameter selection".
        priority = [
            (ObjectClassification.WALL, wall_score, wall_reasons),
            (ObjectClassification.POLE_LIKE, pole_score, pole_reasons),
            (ObjectClassification.VEHICLE_LIKE, vehicle_score, vehicle_reasons),
            (ObjectClassification.LARGE_OBSTACLE, large_score, large_reasons),
            (ObjectClassification.PERSON_LIKE, person_score, person_reasons),
        ]
        best_label, best_score, best_reasons = priority[0]
        for label, score, reasons in priority[1:]:
            if score > best_score + _TIE_EPSILON:
                best_label, best_score, best_reasons = label, score, reasons

        if best_score >= settings.classification_min_confidence:
            classification = best_label
            confidence = best_score
            reason = [f"{classification.value.upper()} (score {best_score:.2f}):"] + [f"- {r}" for r in best_reasons]
        else:
            classification = ObjectClassification.UNKNOWN
            confidence = best_score
            reason = [
                f"UNKNOWN: best candidate was {best_label.value.upper()} at score {best_score:.2f}, "
                f"below the {settings.classification_min_confidence:.2f} confidence threshold.",
            ] + [f"- {r}" for r in best_reasons]

        return _to_detected_object(cluster, features, classification, confidence, reason)


def _to_detected_object(
    cluster: ObstacleCluster,
    features: ShapeFeatures,
    classification: ObjectClassification,
    confidence: float,
    reason: list[str],
) -> DetectedObject:
    """Map an `ObstacleCluster` onto a `DetectedObject`, explicitly reconciling the two models'
    different width/depth axis conventions (see docs/clustering.md "Cluster representation" and
    docs/data-model.md) rather than copying the values across unchanged:
    `DetectedObject.width` is the *lateral* (Y) extent, `.depth` is the *radial* (X) extent;
    `ObstacleCluster.width` is the X extent, `.depth` is the Y extent -- the opposite mapping.
    """
    return DetectedObject(
        object_id=str(cluster.cluster_id),
        centroid=Point2D(x=cluster.centroid_x, y=cluster.centroid_y),
        width=cluster.depth,  # cluster's Y-extent -> DetectedObject's lateral extent
        depth=cluster.width,  # cluster's X-extent -> DetectedObject's radial extent
        distance=cluster.centroid_distance,
        classification=classification,
        confidence=round(confidence, 4),
        bounding_box=BoundingBox(min_x=cluster.min_x, max_x=cluster.max_x, min_y=cluster.min_y, max_y=cluster.max_y),
        point_count=cluster.point_count,
        min_distance=cluster.min_distance,
        angular_width=cluster.angular_width,
        shape_features=features,
        classification_reason=reason,
        timestamp=cluster.timestamp,
    )


def classify_scan(scan: ClusteredScan) -> ClassifiedScan:
    """One-off convenience wrapper around a throwaway `GeometricClassifier`."""
    return GeometricClassifier().classify(scan)

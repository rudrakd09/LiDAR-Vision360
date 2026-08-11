"""Tests for objects.classifier.GeometricClassifier: the spec's synthetic Wall/Pole/Vehicle/
Large-obstacle/Ambiguous cases, plus model-field correctness, confidence bounds, explainability,
and determinism."""

import math

import pytest

from clustering.geometry import circular_angular_extent
from common.config import Settings
from models.clustering import ClusteredScan, ObstacleCluster
from models.lidar import CartesianPoint
from models.objects import ObjectClassification
from objects.classifier import GeometricClassifier, classify_scan

DEFAULT_SETTINGS = Settings(_env_file=None)


def _cluster(xy_pairs: list[tuple[float, float]], cluster_id: int = 0, timestamp: float = 1000.0) -> ObstacleCluster:
    points = []
    for x, y in xy_pairs:
        angle = math.degrees(math.atan2(y, x)) % 360.0
        distance = math.hypot(x, y)
        points.append(CartesianPoint(angle=round(angle, 6), distance=round(distance, 6), timestamp=timestamp, x=x, y=y))

    xs = [p.x for p in points]
    ys = [p.y for p in points]
    distances = [p.distance for p in points]
    angles = [p.angle for p in points]
    centroid_x, centroid_y = sum(xs) / len(xs), sum(ys) / len(ys)
    min_angle, max_angle, angular_width = circular_angular_extent(angles)

    return ObstacleCluster(
        cluster_id=cluster_id, points=points, point_count=len(points),
        centroid_x=centroid_x, centroid_y=centroid_y,
        min_x=min(xs), max_x=max(xs), min_y=min(ys), max_y=max(ys),
        width=max(xs) - min(xs), depth=max(ys) - min(ys),
        min_distance=min(distances), max_distance=max(distances),
        centroid_distance=math.hypot(centroid_x, centroid_y),
        min_angle=min_angle, max_angle=max_angle, angular_width=angular_width,
        timestamp=timestamp,
    )


def _classifier() -> GeometricClassifier:
    return GeometricClassifier(settings=DEFAULT_SETTINGS)


# --- The spec's synthetic test clusters -----------------------------------------------------

def _wall_cluster() -> ObstacleCluster:
    # 5m away, 6m long, flat -- an unambiguous wall.
    return _cluster([(5.0, y / 10.0) for y in range(-30, 31)])


def _pole_cluster() -> ObstacleCluster:
    # A small (0.15m radius) circular arc, 3m away -- an unambiguous pole.
    radius = 0.15
    center = (3.0, 0.0)
    return _cluster([(center[0] + radius * math.cos(t), center[1] + radius * math.sin(t)) for t in [i * 2 * math.pi / 10 for i in range(10)]])


def _vehicle_cluster() -> ObstacleCluster:
    # A flat ~1.8m face, 4m away, decent point count -- matches a real 2D-LiDAR view of a
    # vehicle's near side (see docs/object-classification.md "Known failure cases").
    return _cluster([(4.0, y / 10.0) for y in range(-9, 10)])


def _large_irregular_cluster() -> ObstacleCluster:
    # Big in both axes, scattered (not linear, not circular) -- deliberately doesn't match any
    # specific category well.
    xy = []
    for i in range(30):
        x = 6.0 + (i % 6) * 0.9
        y = -2.5 + (i % 5) * 1.3
        xy.append((x, y))
    return _cluster(xy)


def _ambiguous_cluster() -> ObstacleCluster:
    # Small, sparse, and geometrically inconclusive (neither linear nor circular enough, too
    # small for wall/vehicle/large-obstacle size gates).
    return _cluster([(2.0, 0.0), (2.05, 0.2), (1.95, -0.15)])


class TestSyntheticWall:
    def test_wall_cluster_classifies_as_wall(self):
        obj = _classifier().classify_cluster(_wall_cluster())
        assert obj.classification == ObjectClassification.WALL
        assert obj.confidence >= DEFAULT_SETTINGS.classification_min_confidence

    def test_wall_has_high_linearity_feature(self):
        obj = _classifier().classify_cluster(_wall_cluster())
        assert obj.shape_features.linearity_score > 0.95


class TestSyntheticPole:
    def test_pole_cluster_classifies_as_pole_like(self):
        obj = _classifier().classify_cluster(_pole_cluster())
        assert obj.classification == ObjectClassification.POLE_LIKE
        assert obj.confidence >= DEFAULT_SETTINGS.classification_min_confidence

    def test_pole_has_high_circularity_feature(self):
        obj = _classifier().classify_cluster(_pole_cluster())
        assert obj.shape_features.circularity_score > 0.9


class TestSyntheticVehicle:
    def test_vehicle_cluster_classifies_as_vehicle_like(self):
        obj = _classifier().classify_cluster(_vehicle_cluster())
        assert obj.classification == ObjectClassification.VEHICLE_LIKE
        assert obj.confidence >= DEFAULT_SETTINGS.classification_min_confidence

    def test_vehicle_confidence_is_reasonable_not_perfect_certainty(self):
        obj = _classifier().classify_cluster(_vehicle_cluster())
        assert 0.5 <= obj.confidence <= 1.0


class TestSyntheticLargeIrregular:
    def test_large_irregular_cluster_is_large_obstacle_or_unknown(self):
        obj = _classifier().classify_cluster(_large_irregular_cluster())
        assert obj.classification in (ObjectClassification.LARGE_OBSTACLE, ObjectClassification.UNKNOWN)


class TestSyntheticAmbiguous:
    def test_ambiguous_cluster_is_unknown_not_forced(self):
        obj = _classifier().classify_cluster(_ambiguous_cluster())
        assert obj.classification == ObjectClassification.UNKNOWN

    def test_unknown_confidence_reflects_best_candidate_not_hidden(self):
        obj = _classifier().classify_cluster(_ambiguous_cluster())
        # Even though UNKNOWN, confidence should be a real number (the best candidate's score),
        # not a placeholder -- part of this phase's explainability requirement.
        assert 0.0 <= obj.confidence < DEFAULT_SETTINGS.classification_min_confidence


class TestNeverForcesACategoryBelowThreshold:
    @pytest.mark.parametrize("cluster_factory", [_wall_cluster, _pole_cluster, _vehicle_cluster, _large_irregular_cluster, _ambiguous_cluster])
    def test_confidence_below_threshold_always_means_unknown(self, cluster_factory):
        obj = _classifier().classify_cluster(cluster_factory())
        if obj.confidence < DEFAULT_SETTINGS.classification_min_confidence:
            assert obj.classification == ObjectClassification.UNKNOWN


class TestExplainability:
    def test_every_object_has_a_nonempty_reason(self):
        for factory in (_wall_cluster, _pole_cluster, _vehicle_cluster, _ambiguous_cluster):
            obj = _classifier().classify_cluster(factory())
            assert obj.classification_reason
            assert all(isinstance(line, str) and line for line in obj.classification_reason)

    def test_reason_mentions_the_classification(self):
        obj = _classifier().classify_cluster(_wall_cluster())
        assert "WALL" in obj.classification_reason[0]

    def test_shape_features_are_attached(self):
        obj = _classifier().classify_cluster(_pole_cluster())
        assert obj.shape_features is not None
        assert obj.shape_features.point_count == 10


class TestDetectedObjectFieldMapping:
    def test_axis_convention_is_reconciled_not_copied_unchanged(self):
        # cluster.width (X-extent) -> DetectedObject.depth (radial); cluster.depth (Y-extent) ->
        # DetectedObject.width (lateral) -- see docs/clustering.md and docs/data-model.md.
        cluster = _wall_cluster()
        obj = _classifier().classify_cluster(cluster)
        assert obj.width == pytest.approx(cluster.depth)
        assert obj.depth == pytest.approx(cluster.width)

    def test_min_distance_and_angular_width_populated(self):
        cluster = _wall_cluster()
        obj = _classifier().classify_cluster(cluster)
        assert obj.min_distance == pytest.approx(cluster.min_distance)
        assert obj.angular_width == pytest.approx(cluster.angular_width, abs=1e-6)

    def test_object_id_matches_cluster_id(self):
        cluster = _wall_cluster()
        obj = _classifier().classify_cluster(cluster)
        assert obj.object_id == str(cluster.cluster_id)

    def test_bounding_box_matches_cluster_extent(self):
        cluster = _wall_cluster()
        obj = _classifier().classify_cluster(cluster)
        assert obj.bounding_box.min_x == pytest.approx(cluster.min_x)
        assert obj.bounding_box.max_x == pytest.approx(cluster.max_x)

    def test_confidence_is_within_valid_bounds(self):
        for factory in (_wall_cluster, _pole_cluster, _vehicle_cluster, _large_irregular_cluster, _ambiguous_cluster):
            obj = _classifier().classify_cluster(factory())
            assert 0.0 <= obj.confidence <= 1.0


class TestScanLevelClassification:
    def test_classify_scan_produces_one_object_per_cluster(self):
        scan = ClusteredScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=1000.0,
            clusters=[_wall_cluster(), _pole_cluster()], noise_points=[],
            cluster_count=2, noise_count=0, total_points=41, clustered_points=41, noise_percentage=0.0,
            average_cluster_size=20.5, largest_cluster_size=31, smallest_cluster_size=10,
        )
        result = _classifier().classify(scan)
        assert result.object_count == 2
        assert {o.classification for o in result.objects} == {ObjectClassification.WALL, ObjectClassification.POLE_LIKE}

    def test_noise_points_pass_through_unchanged(self):
        noise = [CartesianPoint(angle=10.0, distance=12.0, timestamp=1000.0, x=11.8, y=2.1)]
        scan = ClusteredScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=1000.0,
            clusters=[], noise_points=noise, cluster_count=0, noise_count=1,
            total_points=1, clustered_points=0, noise_percentage=100.0,
        )
        result = _classifier().classify(scan)
        assert result.noise_points == noise
        assert result.object_count == 0

    def test_empty_scan_produces_empty_result(self):
        scan = ClusteredScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=1000.0,
            clusters=[], noise_points=[], cluster_count=0, noise_count=0,
            total_points=0, clustered_points=0, noise_percentage=0.0,
        )
        result = _classifier().classify(scan)
        assert result.object_count == 0
        assert result.objects == []


class TestDeterminism:
    def test_identical_cluster_produces_identical_classification(self):
        cluster = _wall_cluster()
        obj_a = _classifier().classify_cluster(cluster)
        obj_b = _classifier().classify_cluster(cluster)
        assert obj_a == obj_b

    def test_classify_scan_convenience_function_matches_classifier(self):
        scan = ClusteredScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=1000.0,
            clusters=[_pole_cluster()], noise_points=[], cluster_count=1, noise_count=0,
            total_points=10, clustered_points=10, noise_percentage=0.0,
            average_cluster_size=10.0, largest_cluster_size=10, smallest_cluster_size=10,
        )
        assert classify_scan(scan) == GeometricClassifier().classify(scan)

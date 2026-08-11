"""Tests for objects.features.extract_features: ObstacleCluster -> ShapeFeatures."""

import math

import pytest

from clustering.geometry import circular_angular_extent
from models.clustering import ObstacleCluster
from models.lidar import CartesianPoint
from objects.features import extract_features


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


class TestBasicDimensions:
    def test_point_count_width_depth(self):
        cluster = _cluster([(5.0, -1.0), (5.0, 0.0), (5.0, 1.0)])
        f = extract_features(cluster)
        assert f.point_count == 3
        assert f.width == pytest.approx(0.0)  # constant x
        assert f.depth == pytest.approx(2.0)  # y spans -1..1

    def test_aspect_ratio_for_elongated_shape(self):
        cluster = _cluster([(5.0, -3.0), (5.0, 0.0), (5.0, 3.0)])
        f = extract_features(cluster)
        # width (x-extent) ~0, depth (y-extent) 6.0 -> aspect_ratio = 6.0 / epsilon-guarded min
        assert f.aspect_ratio > 100  # very elongated

    def test_aspect_ratio_for_square_shape(self):
        cluster = _cluster([(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)])
        f = extract_features(cluster)
        assert f.aspect_ratio == pytest.approx(1.0)


class TestDistanceFeatures:
    def test_min_max_centroid_distance_passthrough(self):
        cluster = _cluster([(3.0, 0.0), (5.0, 0.0)])
        f = extract_features(cluster)
        assert f.min_distance == pytest.approx(3.0)
        assert f.max_distance == pytest.approx(5.0)
        assert f.centroid_distance == pytest.approx(4.0)

    def test_mean_and_variance(self):
        cluster = _cluster([(2.0, 0.0), (4.0, 0.0), (6.0, 0.0)])
        f = extract_features(cluster)
        assert f.mean_distance == pytest.approx(4.0)
        assert f.distance_variance == pytest.approx(((2 - 4) ** 2 + (4 - 4) ** 2 + (6 - 4) ** 2) / 3)


class TestAngularFeatures:
    def test_matches_circular_angular_extent(self):
        cluster = _cluster([(5.0, 0.0), (5.0, 0.5), (4.98, -0.3)])
        f = extract_features(cluster)
        assert f.min_angle == cluster.min_angle
        assert f.max_angle == cluster.max_angle
        assert f.angular_width == pytest.approx(cluster.angular_width, abs=1e-6)


class TestPointDistribution:
    def test_point_density_is_points_per_meter_of_major_axis(self):
        cluster = _cluster([(5.0, -2.0), (5.0, -1.0), (5.0, 0.0), (5.0, 1.0), (5.0, 2.0)])
        f = extract_features(cluster)
        assert f.point_density == pytest.approx(5 / 4.0)  # 5 points over 4m of y-extent

    def test_spatial_variance_is_zero_for_coincident_points(self):
        cluster = _cluster([(5.0, 0.0), (5.0, 0.0), (5.0, 0.0)])
        f = extract_features(cluster)
        assert f.spatial_variance == pytest.approx(0.0, abs=1e-9)


class TestShapeScores:
    def test_line_shaped_cluster_has_high_linearity_low_circularity(self):
        cluster = _cluster([(5.0, y / 10.0) for y in range(-20, 21)])
        f = extract_features(cluster)
        assert f.linearity_score > 0.99
        assert f.circularity_score == 0.0

    def test_circle_shaped_cluster_has_high_circularity(self):
        radius = 0.15
        xy = [(3.0 + radius * math.cos(t), radius * math.sin(t)) for t in [i * 2 * math.pi / 8 for i in range(8)]]
        cluster = _cluster(xy)
        f = extract_features(cluster)
        assert f.circularity_score > 0.95

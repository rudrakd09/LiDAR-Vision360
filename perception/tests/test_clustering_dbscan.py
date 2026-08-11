"""Synthetic deterministic tests for clustering.dbscan.DBSCANClusterer.

Uses hand-built `CartesianScan`s (not the simulator) so every expected outcome is exact and
precisely traceable to specific input coordinates -- the spec's Test A-E plus core mechanics
(free-space filtering, noise bookkeeping, cluster model correctness, determinism)."""

import math

import pytest

from clustering.dbscan import DBSCANClusterer, cluster_scan
from common.config import Settings
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint

DEFAULT_SETTINGS = Settings(_env_file=None)  # eps=0.6, min_samples=3, min_cluster_points=3


def _point(x: float, y: float, timestamp: float = 1000.0) -> CartesianPoint:
    angle = math.degrees(math.atan2(y, x)) % 360.0
    distance = math.hypot(x, y)
    return CartesianPoint(angle=round(angle, 6), distance=round(distance, 6), timestamp=timestamp, x=x, y=y)


def _scan(xy_pairs: list[tuple[float, float]], scan_id: str = "scan-1", sequence_number: int = 0) -> CartesianScan:
    points = [_point(x, y) for x, y in xy_pairs]
    return CartesianScan(scan_id=scan_id, sequence_number=sequence_number, source_id="unit-test", timestamp=1000.0, points=points)


def _clusterer(**overrides) -> DBSCANClusterer:
    return DBSCANClusterer(settings=DEFAULT_SETTINGS, **overrides)


class TestA_ThreeClosePointsFormOneCluster:
    def test_three_close_points_form_one_cluster(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 3
        assert result.noise_count == 0


class TestB_TwoSeparatedGroupsFormTwoClusters:
    def test_two_separated_groups_form_two_clusters(self):
        group_a = [(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)]
        group_b = [(5.0, 5.0), (5.1, 5.0), (5.0, 5.1)]
        scan = _scan(group_a + group_b)
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 2
        sizes = sorted(c.point_count for c in result.clusters)
        assert sizes == [3, 3]
        assert result.noise_count == 0

    def test_two_groups_have_distinct_centroids(self):
        group_a = [(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)]
        group_b = [(5.0, 5.0), (5.1, 5.0), (5.0, 5.1)]
        scan = _scan(group_a + group_b)
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        centroids = sorted((c.centroid_x, c.centroid_y) for c in result.clusters)
        assert centroids[0] == pytest.approx((1.0333, 1.0333), abs=0.01)
        assert centroids[1] == pytest.approx((5.0333, 5.0333), abs=0.01)


class TestC_NoisePointFarFromAllClusters:
    def test_isolated_far_point_is_noise_not_a_cluster(self):
        group = [(1.0, 1.0), (1.1, 1.0), (1.0, 1.1), (1.05, 1.05)]
        far_point = (50.0, 50.0)
        scan = _scan(group + [far_point])
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 4
        assert result.noise_count == 1
        assert (result.noise_points[0].x, result.noise_points[0].y) == pytest.approx((50.0, 50.0))


class TestD_PointsCrossingTheAngularBoundary:
    def test_points_crossing_0_360_form_a_single_cluster(self):
        # A tight arc of points straddling angle=0 -- physically contiguous in (x, y) even
        # though their angles (358, 359, 0, 1, 2) look numerically far apart.
        radius = 5.0
        angles_deg = [358.0, 359.0, 0.0, 1.0, 2.0]
        xy = [(radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a))) for a in angles_deg]
        scan = _scan(xy)
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 5
        assert result.noise_count == 0

    def test_boundary_crossing_cluster_reports_correct_angular_extent(self):
        radius = 5.0
        angles_deg = [358.0, 359.0, 0.0, 1.0, 2.0]
        xy = [(radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a))) for a in angles_deg]
        scan = _scan(xy)
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        cluster = result.clusters[0]
        assert cluster.min_angle == pytest.approx(358.0, abs=0.01)
        assert cluster.max_angle == pytest.approx(2.0, abs=0.01)
        assert cluster.angular_width == pytest.approx(4.0, abs=0.01)


class TestE_MinSamplesConfiguration:
    def test_small_group_rejected_by_high_min_samples(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])  # 3 points
        result = _clusterer(min_samples=5, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 0
        assert result.noise_count == 3

    def test_same_group_accepted_by_low_min_samples(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])
        result = _clusterer(min_samples=2, min_cluster_points=2).cluster(scan)
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 3

    def test_min_cluster_points_independently_filters_small_dbscan_clusters(self):
        # 3 tightly-packed points satisfy min_samples=3 as their own DBSCAN cluster, but
        # min_cluster_points=5 should still demote them to noise afterward.
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])
        result = _clusterer(min_samples=3, min_cluster_points=5).cluster(scan)
        assert result.cluster_count == 0
        assert result.noise_count == 3


class TestClusterModelFields:
    def test_geometric_properties_are_correct(self):
        # A simple square of 4 points at known coordinates -- every field hand-computable.
        # eps overridden well above the square's ~2.83 diagonal so this test isolates field
        # *computation* correctness from eps tuning (covered separately elsewhere).
        scan = _scan([(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)])
        result = _clusterer(eps_m=5.0, min_samples=3, min_cluster_points=3).cluster(scan)
        cluster = result.clusters[0]

        assert cluster.point_count == 4
        assert cluster.centroid_x == pytest.approx(1.0)
        assert cluster.centroid_y == pytest.approx(1.0)
        assert cluster.min_x == pytest.approx(0.0)
        assert cluster.max_x == pytest.approx(2.0)
        assert cluster.min_y == pytest.approx(0.0)
        assert cluster.max_y == pytest.approx(2.0)
        assert cluster.width == pytest.approx(2.0)  # max_x - min_x
        assert cluster.depth == pytest.approx(2.0)  # max_y - min_y
        assert cluster.centroid_distance == pytest.approx(math.hypot(1.0, 1.0), abs=1e-4)  # code rounds to 4dp
        assert cluster.timestamp == 1000.0

    def test_min_max_distance_match_extreme_member_points(self):
        scan = _scan([(1.0, 0.0), (1.0, 0.1), (5.0, 0.0)])
        # eps=0.6 must NOT bridge (1,*) and (5,0) -- distinct groups; use a large eps deliberately
        # to force them into one cluster so min/max distance can be checked against known points.
        result = _clusterer(eps_m=10.0, min_samples=3, min_cluster_points=3).cluster(scan)
        cluster = result.clusters[0]
        assert cluster.min_distance == pytest.approx(1.0, abs=0.01)
        assert cluster.max_distance == pytest.approx(5.0, abs=0.01)

    def test_cluster_ids_are_stable_and_zero_indexed(self):
        # Both groups kept well under lidar_range_max_m so neither is caught by the free-space
        # filter (a synthetic-coordinate pitfall, not related to what this test is checking).
        group_a = [(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)]
        group_b = [(5.0, 5.0), (5.1, 5.0), (5.0, 5.1)]
        scan = _scan(group_a + group_b)
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        ids = sorted(c.cluster_id for c in result.clusters)
        assert ids == [0, 1]

    def test_points_reused_not_duplicated_representation(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert all(isinstance(p, CartesianPoint) for p in result.clusters[0].points)


class TestEmptyAndDegenerateScans:
    def test_empty_scan_returns_zero_clusters(self):
        result = _clusterer().cluster(_scan([]))
        assert result.cluster_count == 0
        assert result.noise_count == 0
        assert result.total_points == 0
        assert result.average_cluster_size is None

    def test_all_points_below_min_samples_are_all_noise(self):
        scan = _scan([(1.0, 1.0), (10.0, 10.0)])  # two isolated points, far apart
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 0
        assert result.noise_count == 2


class TestFreeSpaceFiltering:
    def test_points_at_max_range_are_treated_as_noise_not_clustered(self):
        # A ring of "no return" points at exactly the configured max range must not form a
        # false giant cluster -- see docs/clustering.md "Free-space filtering".
        max_range = DEFAULT_SETTINGS.lidar_range_max_m
        ring = [
            (max_range * math.cos(math.radians(a)), max_range * math.sin(math.radians(a)))
            for a in range(0, 360, 1)
        ]
        scan = _scan(ring)
        result = _clusterer().cluster(scan)
        assert result.cluster_count == 0
        assert result.noise_count == len(ring)

    def test_genuine_obstacle_well_within_range_is_not_filtered(self):
        scan = _scan([(5.0, 0.0), (5.1, 0.0), (5.0, 0.1)])
        result = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result.cluster_count == 1


class TestDeterminism:
    def test_identical_input_produces_identical_output(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1), (5.0, 5.0), (5.1, 5.0), (5.0, 5.1)])
        result_a = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        result_b = _clusterer(min_samples=3, min_cluster_points=3).cluster(scan)
        assert result_a == result_b

    def test_cluster_scan_convenience_function_matches_clusterer(self):
        scan = _scan([(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)])
        assert cluster_scan(scan) == DBSCANClusterer().cluster(scan)

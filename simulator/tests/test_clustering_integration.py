"""Integration tests: perception.clustering running against real simulator scenarios, chained
after perception.preprocessing and perception.coordinates.

Lives here for the same reason as the Phase 3/4 integration test files: `perception.clustering`
itself has no dependency on `simulator`, but `simulator` already depends on `perception`, so
exercising the full chain belongs on this side of the dependency graph -- see
docs/clustering.md "Architecture" and docs/architecture.md.

Expected cluster counts below were verified stable across 5 repeated runs for every scenario that
has no fixed noise seed (only 09_noisy_lidar and 10_missing_outliers pin one) before being
encoded as exact assertions -- see docs/clustering.md "Scenario results" for the full table.
"""

import pytest

from clustering import DBSCANClusterer
from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source


def _run(scenario_id: str, scans: int = 1):
    """Return a list of ClusteredScans for `scans` scans of `scenario_id`."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    source = make_data_source(scenario_id)
    results = []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            results.append(clusterer.cluster(cartesian))
    return results


class TestEmptyEnvironment:
    def test_empty_scenario_produces_zero_clusters(self):
        result = _run("01_empty")[0]
        assert result.cluster_count == 0
        assert result.noise_count == result.total_points


class TestWall:
    def test_wall_forms_exactly_one_major_cluster(self):
        result = _run("02_wall_in_front")[0]
        assert result.cluster_count == 1
        assert result.clusters[0].point_count > 50  # the wall's full hit-cone, not a fragment


class TestPole:
    def test_pole_forms_exactly_one_cluster(self):
        result = _run("03_pole_left")[0]
        assert result.cluster_count == 1
        assert 1 <= result.clusters[0].point_count <= 15  # a thin pole -- few points, not fragmented


class TestVehicle:
    def test_vehicle_ahead_forms_exactly_one_major_cluster(self):
        result = _run("04_vehicle_ahead")[0]
        assert result.cluster_count == 1
        assert result.clusters[0].point_count > 10


class TestMultipleObstacles:
    def test_multiple_distinct_clusters_are_found(self):
        result = _run("05_multiple_obstacles")[0]
        assert result.cluster_count >= 2  # spec: "multiple distinct clusters"

    def test_the_two_poles_remain_compact_and_distinct(self):
        # The scenario's two poles (radius 0.15/0.2m) are several meters apart from each other
        # and from the rectangle/wall -- their clusters (small point counts) must stay small and
        # bounded, confirming they were not merged into a neighboring larger structure. (The
        # background wall's own clusters are legitimately long/linear -- several meters -- so
        # this check is scoped to the small, pole-sized clusters specifically.)
        result = _run("05_multiple_obstacles")[0]
        pole_like_clusters = [c for c in result.clusters if c.point_count <= 10]
        assert len(pole_like_clusters) >= 2  # both poles present as their own small clusters
        for cluster in pole_like_clusters:
            assert cluster.width < 1.0
            assert cluster.depth < 1.0


class TestNarrowCorridor:
    def test_two_walls_are_detected_as_separate_structures(self):
        result = _run("06_narrow_corridor")[0]
        assert result.cluster_count == 2
        # One cluster hugging y~+1, the other y~-1 (not merged into one span across both).
        centroid_ys = sorted(c.centroid_y for c in result.clusters)
        assert centroid_ys[0] == pytest.approx(-1.0, abs=0.3)
        assert centroid_ys[1] == pytest.approx(1.0, abs=0.3)


class TestMovingCrossing:
    def test_moving_obstacle_forms_a_cluster_across_successive_scans(self):
        results = _run("07_moving_crossing", scans=5)
        for result in results:
            assert result.cluster_count == 1


class TestApproachingObstacle:
    def test_approaching_obstacle_remains_one_coherent_cluster_as_distance_changes(self):
        results = _run("08_approaching_obstacle", scans=5)
        for result in results:
            assert result.cluster_count == 1
        # The single cluster's centroid distance should shrink as it approaches.
        centroid_distances = [r.clusters[0].centroid_distance for r in results]
        assert centroid_distances[-1] < centroid_distances[0]


class TestNoisyLidar:
    def test_noise_does_not_generate_a_large_number_of_false_clusters(self):
        result = _run("09_noisy_lidar")[0]
        assert result.cluster_count <= 5  # bounded -- not dozens of spurious micro-clusters


class TestMissingAndOutliers:
    def test_missing_and_outlier_data_does_not_catastrophically_fragment_clusters(self):
        result = _run("10_missing_outliers")[0]
        assert result.cluster_count <= 8  # bounded, not one cluster per surviving point
        assert result.cluster_count >= 1  # the wall is still substantially detected
        assert max(c.point_count for c in result.clusters) > 20  # a real main cluster survives

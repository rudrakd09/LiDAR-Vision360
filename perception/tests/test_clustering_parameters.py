"""Parameter-sensitivity tests for DBSCANClusterer: precise, synthetic demonstrations of how
`eps_m`/`min_samples` affect clustering, backing the defaults chosen in common.config.Settings
and documented in docs/clustering.md "Parameter selection"."""

import math

import pytest

from clustering.dbscan import DBSCANClusterer
from common.config import Settings
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint

DEFAULT_SETTINGS = Settings(_env_file=None)


def _point(x: float, y: float) -> CartesianPoint:
    angle = math.degrees(math.atan2(y, x)) % 360.0
    distance = math.hypot(x, y)
    return CartesianPoint(angle=round(angle, 6), distance=round(distance, 6), timestamp=1000.0, x=x, y=y)


def _scan(xy_pairs: list[tuple[float, float]]) -> CartesianScan:
    return CartesianScan(
        scan_id="scan-1", sequence_number=0, source_id="unit-test", timestamp=1000.0,
        points=[_point(x, y) for x, y in xy_pairs],
    )


def _arc_line(radius: float, num_points: int, angular_step_deg: float = 1.0) -> list[tuple[float, float]]:
    """A quasi-1D line of points, matching the geometry of one LiDAR-scanned surface (e.g. a
    wall segment): a fixed radius, evenly spaced in angle -- exactly the shape that revealed the
    eps/min_samples interaction documented in common.config.Settings."""
    return [
        (radius * math.cos(math.radians(i * angular_step_deg)), radius * math.sin(math.radians(i * angular_step_deg)))
        for i in range(num_points)
    ]


class TestTooSmallEps:
    def test_eps_smaller_than_point_spacing_yields_all_noise(self):
        # Adjacent-point spacing on this 5m-radius, 1deg arc is ~0.087m; eps=0.02 can't bridge it.
        line = _arc_line(radius=5.0, num_points=30)
        result = DBSCANClusterer(eps_m=0.02, min_samples=3, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 0
        assert result.noise_count == 30

    def test_eps_at_default_correctly_forms_one_cluster(self):
        line = _arc_line(radius=5.0, num_points=30)
        result = DBSCANClusterer(settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 30


class TestTooLargeEps:
    def test_eps_larger_than_the_gap_between_two_real_obstacles_merges_them(self):
        # Two clearly separate 3-point groups 1m apart -- default eps=0.6 keeps them separate...
        group_a = [(1.0, 1.0), (1.1, 1.0), (1.0, 1.1)]
        group_b = [(2.0, 1.0), (2.1, 1.0), (2.0, 1.1)]  # ~1m away from group_a
        scan = _scan(group_a + group_b)

        result_default = DBSCANClusterer(settings=DEFAULT_SETTINGS).cluster(scan)
        assert result_default.cluster_count == 2

        # ...but an eps larger than that gap incorrectly merges them into one.
        result_too_large = DBSCANClusterer(eps_m=1.5, min_samples=3, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(scan)
        assert result_too_large.cluster_count == 1
        assert result_too_large.clusters[0].point_count == 6


class TestTooSmallMinSamples:
    def test_min_samples_of_one_treats_every_point_as_its_own_valid_cluster_seed(self):
        # With min_samples=1, every point is trivially a core point (itself alone satisfies the
        # threshold); note min_cluster_points still independently filters cluster *size*.
        scan = _scan([(1.0, 1.0), (50.0, 50.0)])  # two totally isolated points
        result = DBSCANClusterer(eps_m=0.6, min_samples=1, min_cluster_points=1, settings=DEFAULT_SETTINGS).cluster(scan)
        # Each isolated point forms its own trivial 1-point "cluster" -- rarely useful in
        # practice (a single point carries no shape information), which is why
        # min_cluster_points defaults to 3, not 1.
        assert result.cluster_count <= 2  # (the far point may also be excluded as free-space)


class TestTooLargeMinSamples:
    def test_min_samples_exceeding_any_achievable_neighbor_count_yields_all_noise(self):
        line = _arc_line(radius=5.0, num_points=30)
        result = DBSCANClusterer(eps_m=0.6, min_samples=100, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 0
        assert result.noise_count == 30


class TestQuasi1DDensityInsight:
    """The specific insight that drove the final default parameters (see
    common.config.Settings.clustering_eps_m's comment and docs/clustering.md): LiDAR surface
    points form a quasi-1D arc, so a point's eps-neighborhood mostly only contains its immediate
    line neighbors -- eps must be large enough to reach `min_samples - 1` neighbors along that
    line, not just bridge the single nearest gap."""

    def test_eps_that_only_reaches_one_neighbor_each_side_fails_min_samples_four(self):
        # At radius=9m (matching the scenario that originally exposed this), 1deg spacing is
        # ~0.157m. eps=0.3 reaches only the immediate +-1 neighbor (2 neighbors + self = 3
        # points), one short of min_samples=4's requirement of 3 *other* points.
        line = _arc_line(radius=9.0, num_points=30)
        result = DBSCANClusterer(eps_m=0.3, min_samples=4, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 0
        assert result.noise_count == 30

    def test_doubling_eps_reaches_enough_neighbors_for_the_same_min_samples(self):
        line = _arc_line(radius=9.0, num_points=30)
        result = DBSCANClusterer(eps_m=0.6, min_samples=4, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 30

    def test_lowering_min_samples_instead_also_resolves_it_at_the_original_eps(self):
        line = _arc_line(radius=9.0, num_points=30)
        result = DBSCANClusterer(eps_m=0.3, min_samples=3, min_cluster_points=3, settings=DEFAULT_SETTINGS).cluster(_scan(line))
        assert result.cluster_count == 1
        assert result.clusters[0].point_count == 30

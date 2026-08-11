"""DBSCAN-based obstacle clustering: `CartesianScan` -> `ClusteredScan`.

Why DBSCAN (see docs/clustering.md for the full write-up and parameter-sensitivity analysis):

- The number of obstacles in a scan is unknown ahead of time -- DBSCAN doesn't require it
  (unlike e.g. k-means), it discovers however many dense regions actually exist.
- Obstacles are irregular shapes (walls, poles, rectangles, and whatever a real environment
  eventually contains) -- DBSCAN makes no shape assumption (unlike e.g. fitting circles/lines
  per cluster), it groups purely by spatial density.
- LiDAR data contains noise and spurious/outlier points (even after Phase 3 filtering, some
  remain by design) -- DBSCAN has a built-in notion of noise (points that aren't part of any
  dense region) rather than being forced to assign every point to some cluster.
- Cluster sizes vary hugely (a thin pole: ~5 points; a long wall: 100+ points) -- DBSCAN doesn't
  need or assume a fixed/similar cluster size, unlike e.g. k-means.

Operating on `(x, y)` rather than `(angle, distance)` is what makes the 0/360 boundary a
non-issue for the clustering *decision* itself: Cartesian space has no discontinuity at
angle=0/360 (a wall crossing that boundary is simply a continuous line segment in `(x, y)`),
unlike raw angle values, where 359 and 1 look numerically far apart despite being physically
adjacent. Only the per-cluster *angular-extent summary statistic* needs explicit circular
handling afterward (`clustering.geometry.circular_angular_extent`) -- the clustering decision
itself needs none.
"""

from __future__ import annotations

import numpy as np
from sklearn.cluster import DBSCAN

from common.config import Settings, get_settings
from common.logging import get_logger
from models.clustering import ClusteredScan, ObstacleCluster
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint

from .geometry import circular_angular_extent
from .quality import compute_cluster_statistics

logger = get_logger(__name__)


class DBSCANClusterer:
    """Groups a `CartesianScan`'s points into `ObstacleCluster`s via DBSCAN on `(x, y)`.

    Stateless -- safe to share/reuse across scans and streams. All parameters default from
    `common.config.Settings` (see there for defaults and the reasoning behind them) but can be
    overridden per instance, e.g. for the parameter-sensitivity experiments in
    `perception/tests/test_clustering_parameters.py`.
    """

    def __init__(
        self,
        eps_m: float | None = None,
        min_samples: int | None = None,
        min_cluster_points: int | None = None,
        max_range_margin_m: float | None = None,
        settings: Settings | None = None,
    ) -> None:
        settings = settings or get_settings()
        self.eps_m = eps_m if eps_m is not None else settings.clustering_eps_m
        self.min_samples = min_samples if min_samples is not None else settings.clustering_min_samples
        self.min_cluster_points = min_cluster_points if min_cluster_points is not None else settings.clustering_min_cluster_points
        self.max_range_margin_m = max_range_margin_m if max_range_margin_m is not None else settings.clustering_max_range_margin_m
        self._max_range_m = settings.lidar_range_max_m

    def cluster(self, scan: CartesianScan) -> ClusteredScan:
        all_points = scan.points

        if not all_points:
            return _empty_result(scan)

        # Free-space / "no return" points (at or beyond the configured max range, within a small
        # margin) do not represent an obstacle surface -- see docs/clustering.md "Free-space
        # filtering". They are excluded from the spatial clustering input entirely and counted
        # as noise, rather than being fed to DBSCAN (which would otherwise happily connect an
        # entire ring of adjacent "nothing detected" points into one large false cluster).
        range_threshold = self._max_range_m - self.max_range_margin_m
        candidate_points = [p for p in all_points if p.distance < range_threshold]
        free_space_points = [p for p in all_points if p.distance >= range_threshold]

        if not candidate_points:
            return _empty_result(scan, noise_points=free_space_points)

        xy = np.array([[p.x, p.y] for p in candidate_points], dtype=np.float64)
        labels = DBSCAN(eps=self.eps_m, min_samples=self.min_samples).fit(xy).labels_

        clusters: list[ObstacleCluster] = []
        noise_points: list[CartesianPoint] = list(free_space_points)
        cluster_id = 0
        for label in sorted(set(labels)):
            member_points = [candidate_points[i] for i, point_label in enumerate(labels) if point_label == label]

            if label == -1 or len(member_points) < self.min_cluster_points:
                noise_points.extend(member_points)
                continue

            clusters.append(_build_cluster(cluster_id, member_points, scan.timestamp))
            cluster_id += 1

        logger.debug(
            "Scan %s: %d cluster(s), %d noise point(s) (%d free-space) from %d candidate point(s).",
            scan.scan_id, len(clusters), len(noise_points), len(free_space_points), len(candidate_points),
        )

        stats = compute_cluster_statistics([c.point_count for c in clusters], len(noise_points))
        return ClusteredScan(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            clusters=clusters,
            noise_points=noise_points,
            cluster_count=len(clusters),
            noise_count=len(noise_points),
            **stats,
        )


def _empty_result(scan: CartesianScan, noise_points: list[CartesianPoint] | None = None) -> ClusteredScan:
    noise_points = noise_points or []
    stats = compute_cluster_statistics([], len(noise_points))
    return ClusteredScan(
        scan_id=scan.scan_id,
        sequence_number=scan.sequence_number,
        source_id=scan.source_id,
        timestamp=scan.timestamp,
        clusters=[],
        noise_points=noise_points,
        cluster_count=0,
        noise_count=len(noise_points),
        **stats,
    )


def _build_cluster(cluster_id: int, points: list[CartesianPoint], timestamp: float) -> ObstacleCluster:
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    distances = [p.distance for p in points]
    angles = [p.angle for p in points]

    centroid_x = sum(xs) / len(xs)
    centroid_y = sum(ys) / len(ys)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_angle, max_angle, angular_width = circular_angular_extent(angles)

    return ObstacleCluster(
        cluster_id=cluster_id,
        points=points,
        point_count=len(points),
        centroid_x=round(centroid_x, 4),
        centroid_y=round(centroid_y, 4),
        min_x=round(min_x, 4),
        max_x=round(max_x, 4),
        min_y=round(min_y, 4),
        max_y=round(max_y, 4),
        width=round(max_x - min_x, 4),
        depth=round(max_y - min_y, 4),
        min_distance=round(min(distances), 4),
        max_distance=round(max(distances), 4),
        centroid_distance=round(float(np.hypot(centroid_x, centroid_y)), 4),
        min_angle=min_angle,
        max_angle=max_angle,
        angular_width=round(angular_width, 4),
        timestamp=timestamp,
    )


def cluster_scan(scan: CartesianScan) -> ClusteredScan:
    """One-off convenience wrapper around a throwaway `DBSCANClusterer`."""
    return DBSCANClusterer().cluster(scan)

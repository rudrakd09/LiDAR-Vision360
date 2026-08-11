"""DBSCAN-based obstacle clustering (Phase 5): groups `CartesianScan` points into
`ObstacleCluster`s believed to be distinct physical obstacles, without classifying what any of
them IS (that's Phase 6).

Status: **implemented**. Independent of `simulator` and of every later pipeline stage (object
classification, tracking, mapping, collision, clearance, Unity, cloud) -- see docs/clustering.md.

Typical usage:

    from clustering import DBSCANClusterer

    clusterer = DBSCANClusterer()  # reads defaults from common.config.Settings
    clustered_scan = clusterer.cluster(cartesian_scan)
"""

from .dbscan import DBSCANClusterer, cluster_scan
from .geometry import circular_angular_extent
from .quality import compute_cluster_statistics

__all__ = ["DBSCANClusterer", "cluster_scan", "circular_angular_extent", "compute_cluster_statistics"]

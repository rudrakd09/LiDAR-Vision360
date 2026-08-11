"""Cluster-level quality/summary statistics for a ClusteredScan."""

from __future__ import annotations

import statistics


def compute_cluster_statistics(cluster_sizes: list[int], noise_count: int) -> dict:
    """Return the aggregate-statistics fields of `models.clustering.ClusteredScan` as a dict,
    ready to splat into its constructor alongside `clusters`/`noise_points`/`cluster_count`.

    `cluster_sizes` is each cluster's `point_count`, in any order. Percentages/averages are
    `0.0`/`None` (never a division error) when there is nothing to compute them from.
    """
    clustered_points = sum(cluster_sizes)
    total_points = clustered_points + noise_count
    noise_percentage = (noise_count / total_points * 100.0) if total_points else 0.0

    return {
        "total_points": total_points,
        "clustered_points": clustered_points,
        "noise_percentage": round(noise_percentage, 4),
        "average_cluster_size": round(statistics.fmean(cluster_sizes), 4) if cluster_sizes else None,
        "largest_cluster_size": max(cluster_sizes) if cluster_sizes else None,
        "smallest_cluster_size": min(cluster_sizes) if cluster_sizes else None,
    }

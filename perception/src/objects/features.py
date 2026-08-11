"""Geometric feature extraction: `ObstacleCluster` -> `models.objects.ShapeFeatures`.

Computes every feature the classifier (`objects.scoring`) scores against, plus a few kept purely
for explainability (exposed on `DetectedObject.shape_features`). See
docs/object-classification.md "Feature definitions" for what each one means, and "Investigated
but not implemented" for shape metrics considered and deliberately left out (convex hull area,
perimeter, a dedicated oriented rectangle-fit residual) with the reasoning why.
"""

from __future__ import annotations

import statistics

from models.clustering import ObstacleCluster
from models.objects import ShapeFeatures

from .shape_fitting import circle_fit_circularity, line_fit_linearity

_EPSILON = 1e-9


def extract_features(cluster: ObstacleCluster) -> ShapeFeatures:
    xs = [p.x for p in cluster.points]
    ys = [p.y for p in cluster.points]
    distances = [p.distance for p in cluster.points]

    major_axis_extent = max(cluster.width, cluster.depth)
    minor_axis_extent = max(min(cluster.width, cluster.depth), _EPSILON)
    # A fully degenerate cluster (all points coincident, major_axis_extent == 0 too) has no
    # meaningful elongation -- report the "square"/undefined case as 1.0 (ShapeFeatures'
    # `ge=1.0` constraint) rather than 0.0, which would otherwise fail validation.
    aspect_ratio = major_axis_extent / minor_axis_extent if major_axis_extent > _EPSILON else 1.0

    mean_distance = statistics.fmean(distances)
    distance_variance = statistics.pvariance(distances) if len(distances) > 1 else 0.0

    spatial_variance = statistics.fmean(
        (x - cluster.centroid_x) ** 2 + (y - cluster.centroid_y) ** 2 for x, y in zip(xs, ys)
    )

    point_density = cluster.point_count / max(major_axis_extent, _EPSILON)

    linearity_score = line_fit_linearity(xs, ys)
    circularity_score = circle_fit_circularity(xs, ys)

    return ShapeFeatures(
        point_count=cluster.point_count,
        width=cluster.width,
        depth=cluster.depth,
        aspect_ratio=round(aspect_ratio, 4),
        min_distance=cluster.min_distance,
        max_distance=cluster.max_distance,
        centroid_distance=cluster.centroid_distance,
        min_angle=cluster.min_angle,
        max_angle=cluster.max_angle,
        angular_width=cluster.angular_width,
        mean_distance=round(mean_distance, 4),
        distance_variance=round(distance_variance, 6),
        spatial_variance=round(spatial_variance, 6),
        point_density=round(point_density, 4),
        linearity_score=round(linearity_score, 4),
        circularity_score=round(circularity_score, 4),
    )

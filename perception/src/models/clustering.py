"""Canonical models for the obstacle-clustering stage (Phase 5).

Groups `CartesianScan` points into `ObstacleCluster`s believed to belong to distinct physical
obstacles, without attempting to classify what any of them IS (that's Phase 6). See
docs/clustering.md for the full write-up.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint


class ObstacleCluster(BaseModel):
    """A group of spatially-adjacent `CartesianPoint`s believed to belong to one obstacle.

    Field-naming note: `width`/`depth` here are defined per this phase's explicit spec as
    `width = max_x - min_x` (extent along the vehicle-forward axis) and
    `depth = max_y - min_y` (extent along the vehicle-lateral axis) -- the *opposite* axis
    mapping from `models.objects.DetectedObject.width`/`.depth` (lateral/radial, established
    Phase 0). This is a deliberate, documented inconsistency between the two phases' specs, not
    a bug -- see docs/clustering.md "Known limitations" for the full explanation and a
    recommendation for Phase 6 (classification), which will likely populate a `DetectedObject`
    from an `ObstacleCluster` and needs to reconcile the two conventions explicitly rather than
    copy the values across unchanged.
    """

    cluster_id: int = Field(..., ge=0, description="Stable ID within this scan only (0, 1, 2, ...). Not a persistent cross-scan track ID -- that's Phase 7.")
    points: list[CartesianPoint] = Field(..., description="Member points, in their original scan order.")
    point_count: int = Field(..., ge=1)

    centroid_x: float = Field(..., description="mean(x) of member points.")
    centroid_y: float = Field(..., description="mean(y) of member points.")

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: float = Field(..., ge=0.0, description="max_x - min_x. See field-naming note above.")
    depth: float = Field(..., ge=0.0, description="max_y - min_y. See field-naming note above.")

    min_distance: float = Field(..., ge=0.0, description="Minimum per-point polar distance among member points.")
    max_distance: float = Field(..., ge=0.0, description="Maximum per-point polar distance among member points.")
    centroid_distance: float = Field(..., ge=0.0, description="Distance from the LiDAR origin to (centroid_x, centroid_y) -- computed fresh, not a per-point measurement.")

    min_angle: float = Field(..., ge=0.0, lt=360.0)
    max_angle: float = Field(..., ge=0.0, lt=360.0)
    angular_width: float = Field(..., ge=0.0, le=360.0, description="Shortest arc (degrees) containing every member point's angle, correctly handling the 0/360 wrap -- see clustering.geometry.circular_angular_extent.")

    timestamp: float


class ClusteredScan(BaseModel):
    """Output of `clustering.DBSCANClusterer.cluster()`: a `CartesianScan` grouped into
    obstacle-candidate clusters, plus everything that wasn't part of one (DBSCAN-labeled noise,
    sub-minimum-size clusters, and free-space/no-return points -- see docs/clustering.md)."""

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float

    clusters: list[ObstacleCluster] = Field(default_factory=list)
    noise_points: list[CartesianPoint] = Field(default_factory=list)

    cluster_count: int = Field(..., ge=0)
    noise_count: int = Field(..., ge=0)
    total_points: int = Field(..., ge=0, description="clustered_points + noise_count.")
    clustered_points: int = Field(..., ge=0, description="Sum of every cluster's point_count.")
    noise_percentage: float = Field(..., ge=0.0, le=100.0)

    average_cluster_size: float | None = Field(default=None, description="Mean cluster point_count; None if cluster_count == 0.")
    largest_cluster_size: int | None = None
    smallest_cluster_size: int | None = None

    @property
    def point_count(self) -> int:
        return self.total_points

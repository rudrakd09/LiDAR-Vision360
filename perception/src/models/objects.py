"""Canonical detected-object model.

`DetectedObject` is intentionally over-provisioned with `Optional` fields for capabilities that
are not implemented yet (classification, tracking, velocity estimation). Per
PROJECT_SPECIFICATION.md Phase 1, the data model must be defined up front and stay extensible so
later phases (clustering, classification, tracking) only need to *populate* fields, not redesign
the schema. Nothing in this module performs classification, tracking, or velocity estimation --
that logic arrives in Phases 6/7.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, Field


class Point2D(BaseModel):
    """A point in the vehicle-relative Cartesian plane (meters)."""

    x: float
    y: float


class ObjectClassification(str, Enum):
    """Geometry-based shape classes produced by Phase 6.

    Defined now (as part of the canonical data model) so `DetectedObject.classification` has a
    concrete, documented type; the classifier that assigns these values is implemented later.
    """

    WALL = "wall"
    VEHICLE_LIKE = "vehicle_like"
    POLE = "pole"
    PERSON_LIKE = "person_like"
    LARGE_OBSTACLE = "large_obstacle"
    UNKNOWN = "unknown"


class Velocity2D(BaseModel):
    """Estimated 2D velocity of a tracked object, in meters/second."""

    vx: float = 0.0
    vy: float = 0.0

    @property
    def speed(self) -> float:
        return (self.vx ** 2 + self.vy ** 2) ** 0.5


class BoundingBox(BaseModel):
    """Axis-aligned, vehicle-relative bounding box of a detected object, in meters."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float


class DetectedObject(BaseModel):
    """A single tracked/detected obstacle, as reported by the perception pipeline.

    Fields populated by Phase 5 (clustering): ``centroid``, ``width``, ``depth``, ``distance``,
    ``bounding_box``, ``point_count``.
    Fields populated by Phase 6 (classification): ``classification``, ``confidence``.
    Fields populated by Phase 7 (tracking): ``track_id``, ``velocity``, ``direction``,
    ``first_seen``, ``last_seen``.
    """

    object_id: str = Field(..., description="Stable identifier for this object within a track's lifetime.")
    centroid: Point2D
    width: float = Field(..., ge=0.0, description="Object extent (meters) along its dominant lateral axis.")
    depth: float = Field(..., ge=0.0, description="Object extent (meters) along its dominant radial axis.")
    distance: float = Field(..., ge=0.0, description="Distance from the vehicle origin to the centroid, in meters.")
    classification: ObjectClassification = Field(default=ObjectClassification.UNKNOWN)
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="Classification confidence, 0-1. 0 until Phase 6 runs.")
    velocity: Velocity2D | None = Field(default=None, description="Populated once tracking (Phase 7) is implemented.")
    direction: float | None = Field(default=None, description="Heading in degrees, populated by tracking (Phase 7).")
    bounding_box: BoundingBox | None = Field(default=None)
    point_count: int | None = Field(default=None, ge=0, description="Number of raw LiDAR points composing this object.")
    track_id: str | None = Field(default=None, description="Cross-frame track identifier, populated by tracking (Phase 7).")
    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) this observation refers to.")

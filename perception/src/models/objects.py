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
    """Geometry-based shape classes produced by Phase 6 (see docs/object-classification.md).

    `POLE_LIKE` was named `POLE` when this enum was first defined in Phase 0, anticipating a
    category the classifier hadn't been specified yet; renamed to match Phase 6's explicit spec
    once it landed. Safe, non-breaking: nothing constructed or matched on the old name (grepped
    before renaming).
    """

    WALL = "wall"
    VEHICLE_LIKE = "vehicle_like"
    POLE_LIKE = "pole_like"
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


class TrackingState(str, Enum):
    """Lifecycle state of a track, populated by tracking (Phase 7). See docs/tracking.md
    "Track lifecycle" for the full state machine and the transitions between these four states.
    """

    TENTATIVE = "tentative"  # just created; not yet seen enough times to be trusted
    CONFIRMED = "confirmed"  # seen tracking_min_hits_to_confirm+ times; a trusted, stable track
    COASTING = "coasting"  # missed this scan but within tracking_max_missed_scans/track_timeout_s
    LOST = "lost"  # exceeded the miss budget; terminal, dropped from the active track set


class MovementState(str, Enum):
    """Coarse motion classification of a track's estimated velocity, populated by tracking
    (Phase 7) once its velocity is reliable -- see docs/tracking.md "Movement classification".
    """

    STATIONARY = "stationary"  # speed below tracking_stationary_speed_threshold_mps
    MOVING = "moving"  # speed at or above the threshold
    UNKNOWN = "unknown"  # fewer than tracking_min_observations_for_velocity hits so far


class BoundingBox(BaseModel):
    """Axis-aligned, vehicle-relative bounding box of a detected object, in meters."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float


class ShapeFeatures(BaseModel):
    """The geometric features `objects.features.extract_features` computes from one
    `ObstacleCluster`, used both internally by the rule-based classifier (Phase 6) and exposed on
    `DetectedObject.shape_features` for explainability/debugging. See
    docs/object-classification.md "Feature definitions" for what each one means and why it was
    chosen (or, for features investigated and deliberately not implemented -- convex hull area,
    perimeter, a dedicated rectangle-fit residual -- why not).
    """

    point_count: int = Field(..., ge=1)
    width: float = Field(..., ge=0.0, description="Bounding-box extent along X (forward), meters.")
    depth: float = Field(..., ge=0.0, description="Bounding-box extent along Y (lateral), meters.")
    aspect_ratio: float = Field(..., ge=1.0, description="max(width, depth) / max(min(width, depth), epsilon) -- always >= 1; 1 means square/compact, large means elongated.")

    min_distance: float = Field(..., ge=0.0)
    max_distance: float = Field(..., ge=0.0)
    centroid_distance: float = Field(..., ge=0.0)

    min_angle: float = Field(..., ge=0.0, lt=360.0)
    max_angle: float = Field(..., ge=0.0, lt=360.0)
    angular_width: float = Field(..., ge=0.0, le=360.0)

    mean_distance: float = Field(..., ge=0.0)
    distance_variance: float = Field(..., ge=0.0, description="Population variance of member points' polar distance -- radial spread.")
    spatial_variance: float = Field(..., ge=0.0, description="Mean squared distance of member points from the cluster centroid, in m^2 -- overall 2D spread.")
    point_density: float = Field(..., ge=0.0, description="point_count per meter of major-axis extent (points / max(width, depth, epsilon)).")

    linearity_score: float = Field(..., ge=0.0, le=1.0, description="1.0 = points fall on a perfect line (PCA/total-least-squares fit); 0.0 = spread equally in all directions.")
    circularity_score: float = Field(..., ge=0.0, le=1.0, description="1.0 = points fall on a perfect circular arc of consistent radius (algebraic circle fit); 0.0 = poor/degenerate fit.")


class DetectedObject(BaseModel):
    """A single classified/tracked obstacle, as reported by the perception pipeline.

    Fields populated by Phase 5 (clustering) *by way of* Phase 6, which maps an `ObstacleCluster`
    onto this model rather than clustering populating it directly (see docs/clustering.md and
    docs/data-model.md): ``centroid``, ``width``, ``depth``, ``distance``, ``bounding_box``,
    ``point_count``, ``min_distance``, ``angular_width``.
    Fields populated by Phase 6 (classification): ``classification``, ``confidence``,
    ``shape_features``, ``classification_reason``.
    Fields populated by Phase 7 (tracking): ``track_id``, ``velocity``, ``direction``,
    ``predicted_position``, ``tracking_state``, ``movement_state``, ``track_age``,
    ``track_hits``, ``track_misses``.
    """

    object_id: str = Field(..., description="Stable identifier for this object within a track's lifetime.")
    centroid: Point2D
    width: float = Field(..., ge=0.0, description="Object extent (meters) along its dominant lateral axis.")
    depth: float = Field(..., ge=0.0, description="Object extent (meters) along its dominant radial axis.")
    distance: float = Field(..., ge=0.0, description="Distance from the vehicle origin to the centroid, in meters.")
    classification: ObjectClassification = Field(default=ObjectClassification.UNKNOWN)
    confidence: float = Field(default=0.0, ge=0.0, le=1.0, description="Classification confidence, 0-1. Not a calibrated statistical probability -- see docs/object-classification.md \"Confidence score\".")
    velocity: Velocity2D | None = Field(default=None, description="Populated once tracking (Phase 7) is implemented.")
    direction: float | None = Field(default=None, description="Heading in degrees, populated by tracking (Phase 7).")
    bounding_box: BoundingBox | None = Field(default=None)
    point_count: int | None = Field(default=None, ge=0, description="Number of raw LiDAR points composing this object.")
    track_id: str | None = Field(default=None, description="Cross-frame track identifier, populated by tracking (Phase 7).")

    # Added in Phase 6 -- additive, Optional, defaulting to None so every object constructed by
    # earlier code (and Phase 0's own tests) keeps working unchanged.
    min_distance: float | None = Field(default=None, ge=0.0, description="Nearest member point's polar distance (vs. `distance`, which is the centroid's).")
    angular_width: float | None = Field(default=None, ge=0.0, le=360.0, description="Shortest arc (degrees) containing the object, 0/360-safe -- see clustering.geometry.circular_angular_extent.")
    shape_features: ShapeFeatures | None = Field(default=None, description="The full extracted feature set the classifier scored, for debugging/explainability.")
    classification_reason: list[str] | None = Field(default=None, description="Human-readable bullet points explaining the classification -- see docs/object-classification.md \"Explainability\".")

    # Added in Phase 7 -- additive, Optional, same pattern as Phase 6's additions above. Per
    # docs/data-model.md "Extensibility rule": tracking has no genuinely new *object* concept
    # (a tracked object is still just a DetectedObject with its Phase-7-reserved fields --
    # `track_id`, `velocity`, `direction`, above -- plus these few new ones -- populated), so no
    # parallel "TrackedObject" model was introduced; see docs/tracking.md "Track data model".
    predicted_position: Point2D | None = Field(default=None, description="Kalman filter's one-step-ahead position estimate (next scan's expected centroid), populated by tracking (Phase 7).")
    tracking_state: TrackingState | None = Field(default=None, description="Track lifecycle state (TENTATIVE/CONFIRMED/COASTING/LOST), populated by tracking (Phase 7).")
    movement_state: MovementState | None = Field(default=None, description="STATIONARY/MOVING/UNKNOWN, populated by tracking (Phase 7) -- UNKNOWN until tracking_min_observations_for_velocity is reached.")
    track_age: int | None = Field(default=None, ge=0, description="Number of scans since this track was created (including this one), populated by tracking (Phase 7). Named `track_age` rather than `age` to avoid ambiguity with any future notion of an object's real-world age.")
    track_hits: int | None = Field(default=None, ge=0, description="Number of scans in which this track was successfully associated with a detection (including this one, if matched), populated by tracking (Phase 7).")
    track_misses: int | None = Field(default=None, ge=0, description="Current consecutive-miss streak; 0 while actively detected, populated by tracking (Phase 7).")

    # Added in Phase 9 (sensor fusion, fusion.FusionEngine) -- additive, same "populate a few new
    # Optional fields on the one existing DetectedObject, no parallel model" pattern as Phase 6/7's
    # additions above. `sensor_sources` defaults to `["lidar"]` (not `[]`) since every object that
    # existed before Phase 9 -- and every one FusionEngine passes through untouched in LiDAR-only
    # mode -- came from LiDAR alone; this default keeps that true without every caller needing to
    # set it explicitly. See docs/fusion.md.
    sensor_sources: list[str] = Field(default_factory=lambda: ["lidar"], description='Which sensor(s) contributed to this object -- "lidar", "radar", or both, populated by fusion.FusionEngine.')
    radar_target_id: str | None = Field(default=None, description="The radar's own target/track identifier matched to this object, if fusion.FusionEngine matched one (see RadarTarget.target_id).")
    radar_confidence: float | None = Field(default=None, ge=0.0, le=1.0, description="Radar-reported detection confidence for the matched/contributing radar target, if it reported one.")
    radar_range_m: float | None = Field(default=None, ge=0.0, description="Radar-reported range to the matched target, meters, if fusion contributed one -- kept alongside `distance` (LiDAR-derived) rather than overwriting it, so both sources stay individually inspectable.")

    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) this observation refers to.")

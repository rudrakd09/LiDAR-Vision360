"""Canonical models for the collision/risk-assessment stage (Phase 9).

Reuses `VehiclePose` (Phase 8, `models.mapping`) for vehicle position/heading rather than
duplicating it -- `VehicleState` here only adds what Phase 8 didn't need: a scalar forward speed.
Reuses `Point2D`/`Velocity2D`/`ObjectClassification` (Phase 0/6, `models.objects`) for relative
position/velocity/classification rather than inventing parallel types -- per docs/data-model.md
"Extensibility rule", the only genuinely new concepts this phase introduces are the risk
classification itself and the per-object/per-scan result containers. See docs/collision.md.

**This is a prototype collision-awareness system, not a certified automotive safety system** --
see docs/collision.md "Status" for the full disclaimer.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, Field

from .mapping import VehiclePose
from .objects import ObjectClassification, Point2D, Velocity2D


class RiskLevel(str, Enum):
    """Discrete collision-risk classification -- see docs/collision.md "Risk classification"."""

    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"


class VehicleState(BaseModel):
    """The ego vehicle's pose and speed, as supplied to `collision.CollisionRiskEngine.evaluate()`.

    Reuses `VehiclePose` (`x`, `y`, `heading`) rather than duplicating it. `speed_mps` is the
    vehicle's forward speed along `pose.heading` (positive = moving forward, matching this
    project's `angle`/`heading` CCW-from-+x convention) -- a single scalar, since nothing in this
    project models independent lateral/steering velocity. See docs/collision.md "Vehicle
    velocity assumption" for how this is obtained when the caller doesn't supply a real value
    (defaults to `0.0`, i.e. stationary -- the only assumption that never *overstates* risk).
    """

    pose: VehiclePose = Field(default_factory=VehiclePose)
    speed_mps: float = Field(default=0.0, description="Forward speed along pose.heading, m/s. 0.0 (stationary) unless explicitly supplied.")


class CollisionRiskResult(BaseModel):
    """Per-object collision-risk assessment, produced by `collision.CollisionRiskEngine.
    evaluate_object()`. See docs/collision.md "Collision risk result model" for the full field-
    by-field derivation."""

    track_id: str | None = Field(default=None, description="From the source DetectedObject.track_id.")
    classification: ObjectClassification

    distance: float = Field(..., ge=0.0, description="Euclidean distance from the vehicle's current position to the object's centroid -- NOT footprint-edge clearance (Phase 10's concern). Recomputed fresh from vehicle_state, not reused from DetectedObject.distance, which assumes the vehicle is always at the world origin. See docs/collision.md \"Clear separation of concepts\".")
    relative_position: Point2D = Field(..., description="Object position minus vehicle position, world frame.")
    relative_velocity: Velocity2D = Field(..., description="Object velocity minus vehicle velocity vector, world frame.")
    relative_speed: float = Field(..., ge=0.0, description="magnitude(relative_velocity) -- full 2D relative speed, not just the closing-speed component used internally for TTC.")

    in_projected_path: bool = Field(..., description="Purely geometric: is the object within the vehicle's heading-aligned corridor (body + side margins), independent of speed/TTC. See docs/collision.md \"Projected path\".")

    ttc: float | None = Field(default=None, ge=0.0, description="Closed-form longitudinal time-to-collision, seconds. None (rendered 'N/A') whenever the object is not genuinely approaching -- moving away, stationary, or below the closing-speed noise floor -- checked before proximity, so a stationary object that already overlaps the footprint is still None, not 0.0. 0.0 is reserved for an object that IS closing and whose footprint already overlaps (contact now). Also None for a centroid within LIDAR_MIN_VALID_DISTANCE_M of the sensor (self return).")
    collision_predicted: bool = Field(..., description="From the discrete footprint-intersection simulation over collision_prediction_horizon_s -- a distinct, cross-checking computation from `ttc` (correctly handles a laterally-crossing object). See docs/collision.md \"Collision prediction\".")
    predicted_collision_time: float | None = Field(default=None, ge=0.0, description="Seconds from now, from the simulation above; None if collision_predicted is False.")
    predicted_collision_position: Point2D | None = Field(default=None, description="World-frame position at predicted_collision_time; None if collision_predicted is False.")

    risk_level: RiskLevel
    risk_score: float | None = Field(default=None, ge=0.0, le=1.0, description="Continuous [0,1] companion to risk_level, derived from the same configured thresholds -- NOT itself safety-authoritative; risk_level is. See docs/collision.md \"Risk score\".")

    reason: list[str] = Field(..., description="Human-readable bullets explaining the risk_level decision -- see docs/collision.md \"Explainability\".")
    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) this assessment refers to.")


class CollisionAssessment(BaseModel):
    """Output of `collision.CollisionRiskEngine.evaluate()`: one `CollisionRiskResult` per object
    in the source `TrackedScan`, plus the vehicle-level summary."""

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float

    results: list[CollisionRiskResult] = Field(default_factory=list)
    object_count: int = Field(..., ge=0)

    overall_risk: RiskLevel = Field(..., description="The single highest risk_level among `results`; SAFE if `results` is empty.")
    most_critical_object: CollisionRiskResult | None = Field(default=None, description="The result driving overall_risk (ties broken by lowest TTC, then shortest distance); None if `results` is empty.")

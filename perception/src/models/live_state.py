"""Canonical `LiveState` model -- the Edge computer's single, standardized representation of
"everything perception currently knows about this scan and this tracking session."

**This is an aggregate, not a new computation.** Every field is either copied verbatim from an
existing canonical model this project's real pipeline stages already produce (`TrackedScan`,
`CollisionAssessment`, `ClearanceAssessment`, `PreprocessedScan.quality_statistics`) or is
bookkeeping the Edge process itself legitimately owns (session identity, per-track history,
event transitions, measured performance) -- nothing here classifies, tracks, or assesses risk.
See `pipeline.live_state.LiveStateBuilder` for the (stateful, but computation-free) assembly code,
and docs/architecture.md "LiveState" for the full rationale: this is what makes the Edge computer
the single source of truth for every downstream consumer (cloud backend, dashboard, Unity) --
each of them renders this object; none of them (re)computes any part of it.

**Null over invention**: every `| None` field below is `None` when the source pipeline genuinely
has nothing to report (e.g. `ttc` for an object that isn't approaching, `risk` for a track the
collision stage never evaluated) -- never a fabricated placeholder value. See each field's own
docstring for exactly when it is `None` and why.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .clearance import ClearanceAssessment
from .collision import CollisionAssessment, RiskLevel
from .objects import DetectedObject, MovementState, ObjectClassification, TrackingState, Velocity2D


class TrajectoryPoint(BaseModel):
    """One historical position sample for a track, recorded the scan it was observed -- copied
    verbatim from that scan's own `DetectedObject`, never re-derived/interpolated."""

    frame_id: int = Field(..., description="The TrackedScan.sequence_number this point was observed on.")
    timestamp: float
    x: float
    y: float
    vx: float | None = Field(default=None, description="None until tracking.ObjectTracker reports a reliable velocity for this track (see DetectedObject.velocity).")
    vy: float | None = None
    distance: float
    classification: ObjectClassification
    tracking_state: TrackingState | None = None


class SensorChannelStatus(BaseModel):
    """One sensor modality's real, measured status for this scan. `None` fields mean "not
    computable from what this scan actually produced" -- never an invented placeholder. A
    modality this system has no hardware/data source for at all (see `LiveState.sensor_status`) is
    represented by the *key being absent*, not by a fabricated all-null entry."""

    connected: bool
    point_count: int | None = Field(default=None, description="Retained (post-preprocessing) point count this scan.")
    valid_percentage: float | None = Field(default=None, ge=0.0, le=100.0, description="From PreprocessedScan.quality_statistics -- real measured value, not assumed.")
    mean_distance_m: float | None = Field(default=None, ge=0.0)


class TrackedObjectState(BaseModel):
    """One tracked object's full LiveState view: the real `DetectedObject` this scan, joined by
    `track_id` with this same scan's `CollisionRiskResult` (for `ttc`/`risk`) and this track's own
    Edge-side history (`first_seen`/`last_seen`/`frames_tracked`/`trajectory`) -- see
    `pipeline.live_state.LiveStateBuilder._join_track` for the exact join, and
    `tracking.history.TrackHistory` for where the history itself comes from (driven directly by
    `tracking.ObjectTracker`'s own per-scan output, not re-derived downstream).
    """

    track_id: str
    classification: ObjectClassification
    confidence: float
    x: float
    y: float
    distance: float
    velocity: Velocity2D | None = Field(default=None, description="None until tracking reports a reliable estimate -- see DetectedObject.velocity.")

    # Exactly one sensor modality exists in this system today (2D LiDAR) -- see docs/README.md
    # "Important sensor limitation". A literal, structurally-true label, not a fabricated
    # measurement; becomes meaningful once a second modality (e.g. radar) is actually integrated.
    sensor_source: str = "lidar"

    first_seen: float | None = Field(default=None, description="Unix timestamp this track_id was first observed this session -- from tracking.history.TrackHistory, None only if the join itself failed (should not happen for a track present in tracked_objects).")
    last_seen: float | None = Field(default=None, description="Unix timestamp of the most recent sighting (this scan's own timestamp when the track is live).")
    frames_tracked: int | None = Field(default=None, description="DetectedObject.track_hits verbatim -- the real tracker's own count of scans this track was successfully associated with a detection. NOT re-derived by counting frames downstream.")
    trajectory: list[TrajectoryPoint] = Field(default_factory=list, description="Bounded (Settings.live_state_trajectory_length) position history for this track, oldest first.")

    ttc: float | None = Field(default=None, description="This object's own CollisionRiskResult.ttc, joined by track_id -- see module docstring 'TTC must belong to the correct tracked object'. None if not approaching/undefined, or if the collision stage didn't evaluate this track.")
    risk: RiskLevel | None = Field(default=None, description="This object's own CollisionRiskResult.risk_level, joined by track_id. None if the collision stage didn't evaluate this track.")

    tracking_state: TrackingState | None = None
    movement_state: MovementState | None = None


class LiveStateEvent(BaseModel):
    """One collision-risk or clearance-status transition, detected at the Edge the scan it
    happened -- the same "only write on actual change" rule `cloud/backend/src/backend/
    ingestion.py` already applied downstream; recorded here instead so the Edge, not a downstream
    consumer, is the authority on when a transition occurred."""

    event_type: str = Field(..., description='"collision" | "clearance".')
    sequence_number: int
    timestamp: float
    track_id: str | None = Field(default=None, description="Set for a collision event when it was the most-critical object driving overall_risk; None for a clearance event (clearance has no per-object identity) or if overall_risk changed with no most_critical_object.")
    previous_value: str | None
    new_value: str
    summary: str


class PerformanceMetrics(BaseModel):
    """Real, directly-measured values only -- no fabricated throughput/rate figure this process
    doesn't actually measure (same rule `cloud/backend/src/backend/routes/metrics.py` already
    documents and follows on the backend side)."""

    pipeline_processing_ms: float | None = Field(default=None, description="Wall-clock time this scan's own preprocessing-through-clearance pipeline run took, measured by the caller (scripts/serve_unity_bridge.py) around its own stage calls. None for the very first scan of a session before any timing is available... never applicable in practice (measured every scan), kept Optional for a caller that doesn't supply it.")
    measured_scan_interval_s: float | None = Field(default=None, description="Actual wall-clock time since the previous scan was processed. None for the first scan of a session (nothing to measure against yet).")
    measured_scan_rate_hz: float | None = Field(default=None, description="1 / measured_scan_interval_s. None under the same condition.")
    scans_processed: int = Field(..., ge=0, description="Total scans this LiveStateBuilder has assembled this session, including this one.")


class LiveState(BaseModel):
    """The Edge computer's single canonical snapshot of one scan plus this session's own
    bookkeeping. See module docstring."""

    session_id: str = Field(..., description="Minted once per Edge process run (see pipeline.live_state.LiveStateBuilder) -- the Edge's own session identity, not assigned by a downstream consumer.")
    source_id: str
    timestamp: float
    sequence_number: int

    sensor_status: dict[str, SensorChannelStatus | None] = Field(
        default_factory=dict,
        description='Keyed by modality ("lidar", ...). A modality this deployment has no sensor for at all is represented by an explicit `null` value under its own key (see SensorChannelStatus docstring) -- e.g. {"lidar": {...real...}, "radar": null} for this project\'s current 2D-LiDAR-only scope, never a fabricated radar reading.',
    )

    objects: list[DetectedObject] = Field(default_factory=list, description="This scan's own DetectedObject list, verbatim from TrackedScan.objects -- the real tracker's per-scan output.")
    tracked_objects: list[TrackedObjectState] = Field(default_factory=list, description="The richer, history-joined view of the same objects -- see TrackedObjectState.")

    clearance: ClearanceAssessment | None = Field(default=None, description="Verbatim from clearance.ClearanceEngine.evaluate() -- the Edge engine, never recomputed by a consumer.")
    risk: CollisionAssessment | None = Field(default=None, description="Verbatim from collision.CollisionRiskEngine.evaluate() -- the Edge engine, never recomputed by a consumer.")

    events: list[LiveStateEvent] = Field(default_factory=list, description="Bounded (Settings.live_state_event_max_count), most-recent-first transition log for this session.")
    performance_metrics: PerformanceMetrics

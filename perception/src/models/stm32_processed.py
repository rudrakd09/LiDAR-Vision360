"""Protocol-independent contract for perception results produced by the STM32 in HARDWARE MODE.

In the target hardware architecture the STM32F103C8T6 is the **primary perception-processing
node**: it ingests A3M1 LiDAR (UART) and R121 radar (CAN) and runs acquisition -> preprocessing
-> LiDAR/radar processing -> sensor fusion -> object detection -> classification -> tracking ->
TTC -> clearance -> collision-risk *itself*, then emits the finished result. That result travels
two independent ways:

    STM32 --CAN--> Vehicle ECU              (safety data for the vehicle -- Phase 8)
    STM32 --> ESP32 --Wi-Fi--> Edge PC      (monitoring/visualisation/storage -- Phase 3+)

`STM32ProcessedFrame` (this module) is the **Edge PC's internal representation** of one such
finished result frame, after the ESP32 link has delivered it and the (replaceable) deserializer
has decoded it. It is deliberately *protocol-independent*:

* It says **nothing** about byte layout, field offsets, CAN IDs, CAN bitrate, UART baud rate,
  CRC parameters, or the ESP32 Wi-Fi transport. Those are the hardware team's to specify and
  live only in `datasources.stm32.processed.serialization` (swappable) and `common.config.
  Settings` (see docs/hardware-integration.md "Hardware integration checklist").
* Fields whose *exact numeric representation* depends on the hardware team's specification are
  marked ``[HW-SPEC DEPENDENT]`` in their description and listed in
  `HARDWARE_SPEC_DEPENDENT_FIELDS` below.

**Architecture rule this model enforces by existing at all:** in hardware mode the Edge PC must
*not* re-run LiDAR/radar preprocessing, fusion, detection, tracking, TTC, clearance, or risk on
this data -- it is already processed. `STM32ProcessedFrame` carries results, never raw
measurements; there is no `points: list[LiDARPoint]` field here on purpose.

**Simulation mode is unaffected.** It keeps using ``Simulator -> existing perception pipeline
-> LiveState``. This contract is the hardware-mode counterpart:
``STM32ProcessedFrame -> (Phase 3 ESP32Source) -> LiveState``. Both converge on the *same*
`models.live_state.LiveState`, which alone feeds the Dashboard, Unity, and PostgreSQL -- see
docs/stm32-processed-contract.md "Mapping to LiveState".

**Reuse over reinvention** (per docs/data-model.md "Extensibility rule"): this contract reuses
`ObjectClassification`, `Point2D`, `Velocity2D` (`models.objects`), `RiskLevel`, `VehicleState`
(`models.collision`), and `ClearanceState` / `ClearanceDirection` (`models.clearance`) verbatim
rather than defining parallel enums/types. It introduces new models only for the genuinely new
concept -- "a single frame of already-processed perception output as one embedded node chose to
report it".

**Versioning:** `metadata.protocol_version` is a semantic-version string (``"MAJOR.MINOR.PATCH"``).
The MAJOR component is the compatibility axis -- a consumer accepts a frame only if its MAJOR is
one this build knows (`datasources.stm32.processed.version.SUPPORTED_MAJOR_VERSIONS`). MINOR/PATCH
increments must stay backward compatible (new optional fields only), which is why the models here
use `extra="ignore"`: an older Edge build silently ignores fields a newer firmware minor-bump
adds, rather than rejecting the frame.

**Validation:** constructing these models directly (test fixtures, trusted Edge-internal code)
gets only pydantic's field-level guarantees. Untrusted input (anything off the wire) must go
through `datasources.stm32.processed.parse_processed_frame` / a
`ProcessedFrameDeserializer`, which additionally runs
`datasources.stm32.processed.validate_processed_frame` and raises a single
`STM32ProcessedFrameError` (never a bare `pydantic.ValidationError`) with a clear message for
every rejection.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, ConfigDict, Field

from .clearance import ClearanceDirection, ClearanceState
from .collision import RiskLevel, VehicleState
from .objects import ObjectClassification, Point2D, Velocity2D

# Canonical current contract version. The MAJOR component gates compatibility; see this module's
# docstring and `datasources.stm32.processed.version`. Kept here (in `models`, which depends on
# nothing hardware-side) so `STM32FrameMetadata.protocol_version` can default to it; `version.py`
# re-exports this and adds the parsing/classification helpers.
PROCESSED_CONTRACT_VERSION = "1.0.0"


class STM32SourceChannel(str, Enum):
    """Which sensing modality (as the STM32's own fusion stage attributed it) produced an object
    or contributed to a fused one. Maps 1:1 onto the existing `DetectedObject.sensor_sources`
    list convention: ``LIDAR`` -> ``["lidar"]``, ``RADAR`` -> ``["radar"]``, ``FUSED`` ->
    ``["lidar", "radar"]`` (see docs/fusion.md)."""

    LIDAR = "lidar"
    RADAR = "radar"
    FUSED = "fused"


class STM32FrameMetadata(BaseModel):
    """Identity/bookkeeping for one processed frame -- everything a consumer needs to order,
    de-duplicate, time-align, and version-check the frame before looking at its contents."""

    model_config = ConfigDict(extra="ignore")

    protocol_version: str = Field(
        default=PROCESSED_CONTRACT_VERSION,
        description='Semantic version ("MAJOR.MINOR.PATCH") of THIS contract as the firmware '
        "implements it. MAJOR is the compatibility axis -- see the module docstring.",
    )
    frame_id: int = Field(
        ..., ge=0,
        description="Monotonically increasing per-frame identifier assigned by the STM32 -- the "
        "hardware-mode analogue of `tracking.ObjectTracker`'s sequence counter in simulation "
        "mode. Used as `LiveState.sequence_number` on the Edge.",
    )
    sequence_number: int = Field(
        ..., ge=0,
        description="[HW-SPEC DEPENDENT] Transport/link-level sequence counter. MAY be identical "
        "to `frame_id`, or an independent counter that also increments on non-frame messages, or "
        "a wrapping uint (see `Settings.stm32_sequence_modulus`) -- the firmware spec decides. "
        "Used by ESP32Source (Phase 3) for dropped-/duplicate-frame detection.",
    )
    timestamp: float = Field(
        ...,
        description="[HW-SPEC DEPENDENT] Frame capture/emit time as Unix epoch SECONDS (float). "
        "If the firmware sends milliseconds, device-uptime, or another epoch "
        "(`Settings.stm32_timestamp_format`), the deserializer converts to Unix-epoch-seconds "
        "before this model is populated -- this field is always Unix-epoch-seconds by the time a "
        "consumer sees it.",
    )
    source_id: str = Field(
        default="stm32_hardware",
        description="Stable identity for the emitting hardware rig -- the hardware-mode analogue "
        'of simulation\'s `"simulated:<scenario>"`. Defaults to `Settings.hardware_source_id`. '
        "Carried onto `LiveState.source_id` unchanged.",
    )
    active_object_count: int = Field(
        ..., ge=0,
        description="Number of active objects the STM32 reports for this frame. MUST equal "
        "`len(objects)` -- `validate_processed_frame` rejects a mismatch rather than trusting "
        "one over the other.",
    )
    emitter: str = Field(
        default="stm32",
        description="Which node assembled this frame. Informational; `\"stm32\"` in the target "
        "architecture (the ESP32 is a transparent gateway, not a producer).",
    )


class STM32ChannelHealth(BaseModel):
    """Health/liveness of one physical sensor channel, as the STM32 reports it. Distinct from
    `models.live_state.SensorChannelStatus` (which additionally carries per-scan point-quality
    numbers the STM32 does not send) -- the Edge maps this onto that; see
    docs/stm32-processed-contract.md."""

    model_config = ConfigDict(extra="ignore")

    connected: bool = Field(..., description="Is this channel currently delivering data to the STM32?")
    ok: bool | None = Field(
        default=None,
        description="Channel self-reported health flag, if the firmware provides one (e.g. A3M1 "
        "motor/health bit, R121 fault line). `None` = not reported, never assumed healthy.",
    )
    frames_received: int | None = Field(default=None, ge=0, description="Cumulative frames the STM32 has received on this channel, if reported.")
    frames_dropped: int | None = Field(default=None, ge=0, description="Cumulative frames the STM32 detected as dropped on this channel, if reported.")
    last_update_timestamp: float | None = Field(
        default=None,
        description="[HW-SPEC DEPENDENT] Unix-epoch-seconds of the STM32's most recent successful "
        "read on this channel, if reported (same epoch normalisation as `metadata.timestamp`).",
    )
    detail: str | None = Field(
        default=None,
        description="[HW-SPEC DEPENDENT] Free-text or coded fault detail. The firmware's fault-code "
        "vocabulary is not yet specified; treated as an opaque string here.",
    )


class STM32SensorStatus(BaseModel):
    """Per-channel sensor health for the whole frame. Mirrors `LiveState.sensor_status`'s
    "absent/`null` key = no such sensor in this deployment, never a fabricated all-false entry"
    convention."""

    model_config = ConfigDict(extra="ignore")

    lidar: STM32ChannelHealth = Field(..., description="A3M1 LiDAR channel -- always present (it is the primary sensor).")
    radar: STM32ChannelHealth | None = Field(
        default=None,
        description="R121 radar channel. `None` when no R121 is wired to this rig, or the "
        "firmware does not report radar health -- not a fabricated entry.",
    )
    fusion_active: bool | None = Field(
        default=None,
        description="Whether the STM32's fusion stage actually combined radar into THIS frame's "
        "results (vs. running LiDAR-only because radar was absent/stale). `None` = not reported.",
    )


class STM32TrackedObject(BaseModel):
    """One already-detected, already-classified, already-tracked, already-risk-assessed object,
    exactly as the STM32 reports it. The Edge does not re-derive any of these fields.

    Reuses `ObjectClassification` / `Point2D` / `Velocity2D` / `RiskLevel` verbatim. Geometry
    the STM32 does not transmit (bounding box, shape features, width/depth) is simply absent --
    the Edge leaves the corresponding `DetectedObject` fields `None`, never guessed.
    """

    model_config = ConfigDict(extra="ignore")

    track_id: str = Field(
        ..., min_length=1,
        description="Stable cross-frame identity assigned by the STM32's tracker. Non-empty; "
        "unique within a frame (`validate_processed_frame` rejects duplicates).",
    )
    object_type: ObjectClassification = Field(
        default=ObjectClassification.UNKNOWN,
        description="Classification from the STM32. Same enum the simulation pipeline produces.",
    )
    position: Point2D = Field(
        ...,
        description="[HW-SPEC DEPENDENT] Vehicle-relative position in METERS, project convention "
        "(forward = +x, left = +y; see docs/coordinates.md). If the firmware transmits polar "
        "(range, bearing) instead, the deserializer converts to (x, y) before populating this.",
    )
    distance_m: float = Field(
        ..., ge=0.0,
        description="Range from the vehicle origin to the object, meters. Kept explicitly (not "
        "only derivable from `position`) because the firmware may report it directly.",
    )
    relative_velocity: Velocity2D | None = Field(
        default=None,
        description="[HW-SPEC DEPENDENT] Object-minus-ego velocity, m/s, vehicle frame (same "
        "convention as `CollisionRiskResult.relative_velocity`). `None` when the STM32 does not "
        "estimate motion for this object (e.g. a brand-new track).",
    )
    confidence: float = Field(
        default=0.0, ge=0.0, le=1.0,
        description="STM32-reported detection/classification confidence, 0-1. Not a calibrated "
        "probability -- same caveat as `DetectedObject.confidence`.",
    )
    ttc_s: float | None = Field(
        default=None, ge=0.0,
        description="Time-to-collision for this object, seconds. `None` = not approaching / "
        "undefined; `0.0` = footprints already overlapping (identical semantics to "
        "`CollisionRiskResult.ttc`).",
    )
    risk: RiskLevel = Field(
        default=RiskLevel.SAFE,
        description="Per-object risk level from the STM32 (safe/warning/critical).",
    )
    source: STM32SourceChannel = Field(
        default=STM32SourceChannel.LIDAR,
        description="Which modality produced this object, per the STM32's fusion attribution.",
    )
    in_projected_path: bool | None = Field(
        default=None,
        description="[HW-SPEC DEPENDENT] Whether the STM32 places this object inside the ego "
        "vehicle's projected-path corridor. `None` when the firmware does not report it -- the "
        "Edge then leaves the corresponding `CollisionRiskResult` field at its conservative "
        "default rather than inventing a value.",
    )
    radar_range_m: float | None = Field(
        default=None, ge=0.0,
        description="Radar-reported range to the matched target, meters, if this object was fused "
        "with radar -- kept alongside `distance_m` so both sources stay inspectable (mirrors "
        "`DetectedObject.radar_range_m`).",
    )
    radar_target_id: str | None = Field(
        default=None,
        description="The radar's own target identifier for the matched target, if reported "
        "(mirrors `DetectedObject.radar_target_id`).",
    )


class STM32Clearance(BaseModel):
    """Directional clearance for the whole frame, as computed on the STM32. A lean projection of
    `models.clearance.ClearanceAssessment` -- reuses `ClearanceDirection` / `ClearanceState`, but
    carries only the four distances + the summary (not per-direction `nearest_point`, which the
    STM32 does not transmit). The Edge expands this back into a full `ClearanceAssessment` for
    `LiveState.clearance`; see docs/stm32-processed-contract.md."""

    model_config = ConfigDict(extra="ignore")

    front_m: float = Field(..., ge=0.0, description="Clearance ahead of the vehicle's safety envelope edge, meters.")
    rear_m: float = Field(..., ge=0.0, description="Clearance behind, meters.")
    left_m: float = Field(..., ge=0.0, description="Clearance to the left, meters.")
    right_m: float = Field(..., ge=0.0, description="Clearance to the right, meters.")
    min_clearance_m: float = Field(
        ..., ge=0.0,
        description="Smallest of the four directional values. `validate_processed_frame` checks "
        "it agrees with min(front, rear, left, right) within a small tolerance.",
    )
    min_direction: ClearanceDirection = Field(..., description="Which direction produced `min_clearance_m` (checked for consistency).")
    status: ClearanceState = Field(..., description="Overall clearance classification (safe/caution/low_clearance/critical).")
    corridor_width_m: float | None = Field(
        default=None, ge=0.0,
        description="Total lateral gap available, meters, if the STM32 reports it (see "
        "`ClearanceAssessment.corridor_width_m`). `None` = not reported.",
    )


class STM32Risk(BaseModel):
    """Vehicle-level collision-risk summary for the frame, as computed on the STM32. Reuses
    `RiskLevel`. Per-object risk lives on each `STM32TrackedObject.risk`; this is the aggregate."""

    model_config = ConfigDict(extra="ignore")

    overall_risk: RiskLevel = Field(..., description="Highest risk among all objects / the frame's single risk verdict.")
    most_critical_track_id: str | None = Field(
        default=None,
        description="`track_id` of the object driving `overall_risk`. If set, it MUST match an "
        "object in `objects[]` (`validate_processed_frame` rejects a dangling reference). `None` "
        "when no object drives the risk (e.g. an empty frame -> SAFE).",
    )
    collision_predicted: bool = Field(
        default=False,
        description="Whether the STM32's prediction stage expects a collision within its horizon.",
    )
    predicted_collision_time_s: float | None = Field(
        default=None, ge=0.0,
        description="Seconds until the predicted collision, if `collision_predicted`; else `None`.",
    )
    reason: list[str] = Field(
        default_factory=list,
        description="Human-readable bullets explaining the risk verdict, if the firmware sends "
        "them (mirrors `CollisionAssessment` explainability). May be empty.",
    )


class STM32SystemStatus(BaseModel):
    """The STM32's own health/telemetry for this frame -- the hardware-mode analogue of
    `LiveState.performance_metrics` plus a node-health summary. All timing/load fields are
    optional: the firmware sends what it measures, and the Edge reports `None` for the rest
    rather than fabricating a figure (same rule `routes/metrics.py` already documents)."""

    model_config = ConfigDict(extra="ignore")

    stm32_ok: bool = Field(default=True, description="STM32 self-reported overall health for this frame.")
    processing_load: float | None = Field(default=None, ge=0.0, le=1.0, description="STM32 CPU load 0-1 while producing this frame, if measured.")
    processing_time_ms: float | None = Field(
        default=None, ge=0.0,
        description="Wall-clock milliseconds the STM32 spent producing this frame "
        "(acquisition -> risk), if measured. Maps to `PerformanceMetrics.pipeline_processing_ms`.",
    )
    can_tx_ok: bool | None = Field(
        default=None,
        description="Whether the STM32 -> Vehicle-ECU CAN transmit path (Phase 8) is healthy, if "
        "reported. Informational only -- the Edge never drives that bus.",
    )
    fault_codes: list[str] = Field(
        default_factory=list,
        description="[HW-SPEC DEPENDENT] STM32-reported fault codes. The code vocabulary is not "
        "yet specified; opaque strings here. Empty = no faults reported.",
    )
    notes: str | None = Field(default=None, description="Free-text status note, if any.")


class STM32ProcessedFrame(BaseModel):
    """One complete frame of processed perception output from the STM32, ready for the Edge to
    turn into a `models.live_state.LiveState` (Phase 3/4) -- WITHOUT re-running any perception
    stage. See the module docstring for the architecture rationale and
    docs/stm32-processed-contract.md for the full field-by-field mapping to `LiveState`.
    """

    model_config = ConfigDict(extra="ignore")

    metadata: STM32FrameMetadata
    sensor_status: STM32SensorStatus
    vehicle_state: VehicleState | None = Field(
        default=None,
        description="Ego pose + forward speed for this frame (reuses `models.collision."
        "VehicleState`). `None` when the rig has no ego-motion source -- the Edge then treats "
        "the vehicle as stationary at the origin (the only assumption that never overstates "
        "risk), exactly as `collision_default_vehicle_speed_mps` already does.",
    )
    objects: list[STM32TrackedObject] = Field(
        default_factory=list,
        description="Active tracked objects this frame. May be empty (a valid 'nothing detected' "
        "frame). Length MUST equal `metadata.active_object_count`.",
    )
    clearance: STM32Clearance | None = Field(
        default=None,
        description="Directional clearance summary. `None` when the firmware does not run/transmit "
        "clearance -- the Edge then leaves `LiveState.clearance` `None`, never fabricated.",
    )
    risk: STM32Risk
    system_status: STM32SystemStatus


# Fields whose exact on-wire representation depends on specifications the hardware team has not
# yet provided. Every one is decoded/normalised by the (replaceable) deserializer; the model
# above always presents them in the normalised form documented in each field's description. Kept
# as data so docs and tests can assert this list stays non-empty and in sync with the
# ``[HW-SPEC DEPENDENT]`` markers.
HARDWARE_SPEC_DEPENDENT_FIELDS: tuple[str, ...] = (
    "metadata.sequence_number",       # counter semantics / modulus
    "metadata.timestamp",             # epoch + units (s / ms / device-uptime)
    "STM32ChannelHealth.last_update_timestamp",
    "STM32ChannelHealth.detail",      # fault-code vocabulary
    "STM32TrackedObject.position",    # cartesian vs. polar on the wire
    "STM32TrackedObject.relative_velocity",  # vector vs. scalar closing speed
    "STM32TrackedObject.in_projected_path",  # whether reported at all
    "STM32SystemStatus.fault_codes",  # code vocabulary
)


__all__ = [
    "PROCESSED_CONTRACT_VERSION",
    "HARDWARE_SPEC_DEPENDENT_FIELDS",
    "STM32SourceChannel",
    "STM32FrameMetadata",
    "STM32ChannelHealth",
    "STM32SensorStatus",
    "STM32TrackedObject",
    "STM32Clearance",
    "STM32Risk",
    "STM32SystemStatus",
    "STM32ProcessedFrame",
]

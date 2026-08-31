"""STM32 processed-perception contract -- the HARDWARE-MODE counterpart of the simulation
pipeline's output.

    SIMULATION:  Simulator --> existing perception pipeline --> LiveState
    HARDWARE:    STM32ProcessedFrame --> (Phase 3 ESP32Source) --> LiveState

Both converge on the same `models.live_state.LiveState`, which alone feeds Dashboard / Unity /
PostgreSQL. In hardware mode the Edge PC does **not** re-run preprocessing, fusion, detection,
tracking, TTC, clearance, or risk -- the STM32 already did. This package is:

* `models.stm32_processed.STM32ProcessedFrame` (imported here for convenience) -- the
  protocol-independent data contract: ``metadata`` / ``sensor_status`` / ``vehicle_state`` /
  ``objects[]`` / ``clearance`` / ``risk`` / ``system_status``.
* `version` -- semantic-version gating (`SUPPORTED_MAJOR_VERSIONS`, `require_supported_version`).
* `validation.validate_processed_frame` -- semantic/cross-field/plausibility checks; raises
  `STM32ProcessedFrameError` with a precise message, never a silent/partial result.
* `serialization` -- `ProcessedFrameSerializer` / `ProcessedFrameDeserializer` interfaces +
  `JsonProcessedFrameCodec` (a *reference* codec; the real STM32<->ESP32 wire format is not yet
  specified) + `parse_processed_frame` (transport-agnostic decode+validate).

Nothing here opens a socket, reads Wi-Fi, or knows a byte layout -- that is ESP32Source (Phase 3)
plus a real codec, both still to come.

------------------------------------------------------------------------------------------------
Mapping to `models.live_state.LiveState` (implemented by the Phase 3/4 adapter, NOT in this
package -- documented here and in docs/stm32-processed-contract.md so it is fixed now):

    LiveState.session_id          <- assigned by ESP32Source per hardware session (uuid4);
                                     the Edge owns session identity, exactly as in simulation.
    LiveState.source_id           <- metadata.source_id
    LiveState.timestamp           <- metadata.timestamp
    LiveState.sequence_number     <- metadata.frame_id
    LiveState.sensor_status       <- {"lidar": SensorChannelStatus(connected=lidar.connected,
                                       point_count=None, valid_percentage=None,
                                       mean_distance_m=None),   # STM32 sends no raw point stats
                                       "radar": SensorChannelStatus(...) or None}
    LiveState.objects[]           <- objects[] -> DetectedObject(object_id=track_id,
                                       track_id=track_id, centroid=position, distance=distance_m,
                                       classification=object_type, confidence=confidence,
                                       velocity=relative_velocity,
                                       sensor_sources=<per STM32SourceChannel>,
                                       radar_range_m, radar_target_id,
                                       timestamp=metadata.timestamp)
                                     -- width/depth/shape_features/bounding_box left None.
    LiveState.tracked_objects[]   <- objects[] joined with risk -> TrackedObjectState(track_id,
                                       classification=object_type, confidence, x/y=position,
                                       distance=distance_m, velocity=relative_velocity,
                                       sensor_source=source.value, ttc=ttc_s, risk=object.risk,
                                       tracking_state=None, movement_state=None,
                                       first_seen/last_seen/frames_tracked/trajectory <- an
                                       Edge-side TrackHistory keyed on track_id, same component
                                       simulation already uses).
    LiveState.clearance           <- clearance -> ClearanceAssessment(front/rear/left/right=
                                       DirectionalClearance(distance_m=..., nearest_point=None),
                                       min_clearance_m, min_direction, corridor_width_m,
                                       overall_status=status, reason=[...],
                                       scan_id/sequence_number/source_id/timestamp from metadata).
                                     None when frame.clearance is None.
    LiveState.risk                <- risk + objects -> CollisionAssessment(results=[one
                                       CollisionRiskResult per object: track_id, classification,
                                       distance, ttc, risk_level=object.risk,
                                       in_projected_path=object.in_projected_path or False,
                                       relative_position=position, relative_velocity=..., ...],
                                       overall_risk=risk.overall_risk,
                                       most_critical_object=<by risk.most_critical_track_id>,
                                       object_count, collision_predicted, ...).
                                     None when frame.risk carries nothing meaningful is NOT a
                                     case -- risk is a required section; overall SAFE + empty
                                     results is the "nothing to worry about" frame.
    LiveState.events              <- detected at the Edge from transitions in the received
                                     stream (risk change / clearance change / track created /
                                     track lost), the SAME "only on actual change" logic
                                     `pipeline.LiveStateBuilder` already runs -- never a
                                     re-computation of perception.
    LiveState.performance_metrics <- ESP32Source-measured inter-frame wall-clock interval +
                                     system_status.processing_time_ms as pipeline_processing_ms.

Fields with no STM32 source (object width/depth/shape, per-object tracking_state/movement_state,
CollisionRiskResult.relative_speed default, etc.) become `None`/model defaults -- never
fabricated. If the hardware team later adds them, extend this contract with a MINOR version bump
and widen the adapter.
"""

from __future__ import annotations

from models.stm32_processed import (
    HARDWARE_SPEC_DEPENDENT_FIELDS,
    PROCESSED_CONTRACT_VERSION,
    STM32ChannelHealth,
    STM32Clearance,
    STM32FrameMetadata,
    STM32ProcessedFrame,
    STM32Risk,
    STM32SensorStatus,
    STM32SourceChannel,
    STM32SystemStatus,
    STM32TrackedObject,
)

from .errors import STM32ContractVersionError, STM32ProcessedFrameError
from .serialization import (
    JsonProcessedFrameCodec,
    ProcessedFrameCodec,
    ProcessedFrameDeserializer,
    ProcessedFrameSerializer,
    parse_processed_frame,
)
from .validation import validate_processed_frame
from .version import (
    SUPPORTED_MAJOR_VERSIONS,
    ContractVersionStatus,
    classify_contract_version,
    parse_semver,
    require_supported_version,
)

__all__ = [
    # contract model
    "STM32ProcessedFrame",
    "STM32FrameMetadata",
    "STM32ChannelHealth",
    "STM32SensorStatus",
    "STM32TrackedObject",
    "STM32Clearance",
    "STM32Risk",
    "STM32SystemStatus",
    "STM32SourceChannel",
    "PROCESSED_CONTRACT_VERSION",
    "HARDWARE_SPEC_DEPENDENT_FIELDS",
    # versioning
    "SUPPORTED_MAJOR_VERSIONS",
    "ContractVersionStatus",
    "parse_semver",
    "classify_contract_version",
    "require_supported_version",
    # validation
    "validate_processed_frame",
    # serialization
    "ProcessedFrameSerializer",
    "ProcessedFrameDeserializer",
    "ProcessedFrameCodec",
    "JsonProcessedFrameCodec",
    "parse_processed_frame",
    # errors
    "STM32ProcessedFrameError",
    "STM32ContractVersionError",
]

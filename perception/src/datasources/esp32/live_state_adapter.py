"""Adapter: `STM32ProcessedFrame` (Phase 2) -> `models.live_state.LiveState`.

This is the join point where hardware mode meets the existing single-source-of-truth:

    SIMULATION:  Simulator -> perception pipeline -> LiveStateBuilder -> LiveState
    HARDWARE:    STM32ProcessedFrame -> ProcessedFrameToLiveState -> LiveState

**No perception happens here.** The adapter re-uses `pipeline.LiveStateBuilder` verbatim for
everything the Edge legitimately owns (session identity, per-track history, event transitions,
measured inter-frame rate) and only *re-shapes* the STM32's already-computed results into the
canonical `TrackedScan` / `CollisionAssessment` / `ClearanceAssessment` those consumers expect.
Fields the STM32 does not transmit (object width/depth/shape, per-object tracking_state, ...)
stay `None`/default -- never fabricated. See docs/stm32-processed-contract.md "Mapping to
LiveState" for the field-by-field table.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from common.config import Settings, get_settings
from models.clearance import ClearanceAssessment, ClearanceDirection, DirectionalClearance
from models.collision import CollisionAssessment, CollisionRiskResult, VehicleState
from models.live_state import LiveState, SensorChannelStatus
from models.objects import DetectedObject, Point2D, Velocity2D
from models.stm32_processed import STM32ProcessedFrame, STM32SourceChannel, STM32TrackedObject
from models.tracking import TrackedScan
from pipeline.live_state import LiveStateBuilder

_SOURCE_TO_SENSOR_SOURCES: dict[STM32SourceChannel, list[str]] = {
    STM32SourceChannel.LIDAR: ["lidar"],
    STM32SourceChannel.RADAR: ["radar"],
    STM32SourceChannel.FUSED: ["lidar", "radar"],
}


@dataclass
class AdaptedFrame:
    """Everything the wire-message builder (`streaming.protocol.build_perception_frame_message`)
    needs, all derived from one `STM32ProcessedFrame`."""

    live_state: LiveState
    tracked_scan: TrackedScan
    collision_assessment: CollisionAssessment
    clearance_assessment: ClearanceAssessment | None
    vehicle_state: VehicleState | None


def _sensor_sources(source: STM32SourceChannel) -> list[str]:
    return list(_SOURCE_TO_SENSOR_SOURCES.get(source, ["lidar"]))


def processed_frame_to_tracked_scan(frame: STM32ProcessedFrame) -> TrackedScan:
    """One `TrackedScan` whose `DetectedObject`s carry the STM32's already-tracked objects.
    Geometry the STM32 does not send (`width`/`depth`/`shape_features`/`bounding_box`) is left at
    `0.0`/`None` -- not guessed."""
    objects: list[DetectedObject] = []
    for o in frame.objects:
        objects.append(
            DetectedObject(
                object_id=o.track_id,
                track_id=o.track_id,
                centroid=Point2D(x=o.position.x, y=o.position.y),
                width=0.0,
                depth=0.0,
                distance=o.distance_m,
                classification=o.object_type,
                confidence=o.confidence,
                velocity=o.relative_velocity,
                sensor_sources=_sensor_sources(o.source),
                radar_target_id=o.radar_target_id,
                radar_range_m=o.radar_range_m,
                timestamp=frame.metadata.timestamp,
            )
        )
    return TrackedScan(
        scan_id=f"esp32-{frame.metadata.frame_id}",
        sequence_number=frame.metadata.frame_id,
        source_id=frame.metadata.source_id,
        timestamp=frame.metadata.timestamp,
        objects=objects,
        object_count=len(objects),
        noise_count=0,
        new_track_count=0,
        lost_track_count=0,
        coasting_track_count=0,
    )


def _risk_result(frame: STM32ProcessedFrame, o: STM32TrackedObject) -> CollisionRiskResult:
    rv = o.relative_velocity if o.relative_velocity is not None else Velocity2D(vx=0.0, vy=0.0)
    is_critical = frame.risk.most_critical_track_id == o.track_id
    predicted = frame.risk.collision_predicted and is_critical
    return CollisionRiskResult(
        track_id=o.track_id,
        classification=o.object_type,
        distance=o.distance_m,
        relative_position=Point2D(x=o.position.x, y=o.position.y),
        relative_velocity=rv,
        relative_speed=math.hypot(rv.vx, rv.vy),
        in_projected_path=bool(o.in_projected_path) if o.in_projected_path is not None else False,
        ttc=o.ttc_s,
        collision_predicted=predicted,
        predicted_collision_time=frame.risk.predicted_collision_time_s if predicted else None,
        predicted_collision_position=None,
        risk_level=o.risk,
        risk_score=None,
        reason=list(frame.risk.reason) if (is_critical and frame.risk.reason) else ["Per-object risk from STM32 processed frame."],
        timestamp=frame.metadata.timestamp,
    )


def processed_frame_to_collision_assessment(frame: STM32ProcessedFrame) -> CollisionAssessment:
    """`overall_risk` is taken from `frame.risk.overall_risk` verbatim (the STM32 is
    authoritative -- it may weigh factors beyond the per-object maxima), not recomputed from the
    per-object results."""
    results = [_risk_result(frame, o) for o in frame.objects]
    most_critical = next((r for r in results if r.track_id == frame.risk.most_critical_track_id), None)
    return CollisionAssessment(
        scan_id=f"esp32-{frame.metadata.frame_id}",
        sequence_number=frame.metadata.frame_id,
        source_id=frame.metadata.source_id,
        timestamp=frame.metadata.timestamp,
        results=results,
        object_count=len(results),
        overall_risk=frame.risk.overall_risk,
        most_critical_object=most_critical,
    )


def processed_frame_to_clearance_assessment(frame: STM32ProcessedFrame) -> ClearanceAssessment | None:
    c = frame.clearance
    if c is None:
        return None

    def _d(direction: ClearanceDirection, dist: float) -> DirectionalClearance:
        return DirectionalClearance(direction=direction, distance_m=dist, nearest_point=None)

    corridor = c.corridor_width_m if c.corridor_width_m is not None else (c.left_m + c.right_m)
    return ClearanceAssessment(
        scan_id=f"esp32-{frame.metadata.frame_id}",
        sequence_number=frame.metadata.frame_id,
        source_id=frame.metadata.source_id,
        timestamp=frame.metadata.timestamp,
        front=_d(ClearanceDirection.FRONT, c.front_m),
        rear=_d(ClearanceDirection.REAR, c.rear_m),
        left=_d(ClearanceDirection.LEFT, c.left_m),
        right=_d(ClearanceDirection.RIGHT, c.right_m),
        min_clearance_m=c.min_clearance_m,
        min_direction=c.min_direction,
        corridor_width_m=corridor,
        overall_status=c.status,
        reason=["Directional clearance from STM32 processed frame."],
    )


def processed_frame_sensor_status(frame: STM32ProcessedFrame) -> dict[str, SensorChannelStatus | None]:
    """`LiveState.sensor_status` for a hardware frame. `point_count`/`valid_percentage`/
    `mean_distance_m` are `None` -- the STM32 sends per-channel connectivity, not raw
    point-quality statistics. `radar` is `None` (key present, value null) when no R121 channel is
    reported -- mirrors `LiveState.sensor_status`'s own convention, and unlike simulation it can
    be a real reading here."""
    def _ch(h) -> SensorChannelStatus:
        return SensorChannelStatus(connected=h.connected, point_count=None, valid_percentage=None, mean_distance_m=None)

    return {
        "lidar": _ch(frame.sensor_status.lidar),
        "radar": _ch(frame.sensor_status.radar) if frame.sensor_status.radar is not None else None,
    }


class ProcessedFrameToLiveState:
    """Stateful across a hardware session (like `LiveStateBuilder` itself): construct once, call
    `build()` per received frame, in order. Call `reset(session_id=...)` when `ESP32Source`
    reports a new session (reconnect / new stream) so track history, the event log, and the
    last-risk/clearance memory are all cleared -- old objects/risk/TTC never linger."""

    def __init__(self, settings: Settings | None = None, *, session_id: str | None = None) -> None:
        self._settings = settings or get_settings()
        self._builder = LiveStateBuilder(settings=self._settings, session_id=session_id)

    @property
    def session_id(self) -> str:
        return self._builder.session_id

    def reset(self, *, session_id: str | None = None) -> None:
        self._builder = LiveStateBuilder(settings=self._settings, session_id=session_id)

    def build(self, frame: STM32ProcessedFrame) -> AdaptedFrame:
        tracked_scan = processed_frame_to_tracked_scan(frame)
        collision = processed_frame_to_collision_assessment(frame)
        clearance = processed_frame_to_clearance_assessment(frame)
        processing_s = (
            frame.system_status.processing_time_ms / 1000.0
            if frame.system_status.processing_time_ms is not None
            else None
        )
        live_state = self._builder.build(
            tracked_scan=tracked_scan,
            preprocessed_scan=None,
            collision_assessment=collision,
            clearance_assessment=clearance,
            pipeline_processing_s=processing_s,
        )
        # LiveStateBuilder cannot know the STM32's per-channel sensor status (it has no
        # PreprocessedScan) -- overlay it here without touching the builder.
        live_state = live_state.model_copy(update={"sensor_status": processed_frame_sensor_status(frame)})
        return AdaptedFrame(
            live_state=live_state,
            tracked_scan=tracked_scan,
            collision_assessment=collision,
            clearance_assessment=clearance,
            vehicle_state=frame.vehicle_state,
        )


__all__ = [
    "AdaptedFrame",
    "ProcessedFrameToLiveState",
    "processed_frame_to_tracked_scan",
    "processed_frame_to_collision_assessment",
    "processed_frame_to_clearance_assessment",
    "processed_frame_sensor_status",
]

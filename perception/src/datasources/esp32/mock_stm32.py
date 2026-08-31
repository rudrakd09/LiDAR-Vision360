"""MOCK STM32 HARDWARE OUTPUT -- a scripted generator of Phase-2 `STM32ProcessedFrame`s.

*** THIS IS NOT REAL HARDWARE DATA. NOTHING HERE IS A PHYSICAL MEASUREMENT. ***

It stands in for the *already-processed* perception frames the STM32F103C8T6 firmware will emit
once the physical ``A3M1 + R121 -> STM32 -> ESP32`` chain exists, so the complete Edge software
path

    (mock) STM32 -> (mock) ESP32 transport -> ESP32Source -> ProcessedFrameToLiveState
        -> LiveState -> streaming server -> {Dashboard, Unity, PostgreSQL}

can be exercised end to end and proven hardware-ready with **no devices attached** (Phase 10).

What this is NOT
---------------
It does **not** re-simulate LiDAR/radar perception -- there is no point cloud, no clustering, no
classification here. That is the simulator's job (`data_source="simulation"`), and it is left
untouched. This module models only the *output* of the STM32's on-device
fusion + tracking + TTC + clearance + risk stage.

Faithful thresholds
-------------------
Every risk / TTC / clearance value a frame carries is produced by the project's **own** rules,
run over scripted object kinematics:

* ``collision.ttc.compute_ttc`` / ``collision.ttc.closing_speed_along``  (TTC, closing speed)
* ``collision.geometry.in_projected_path``                              (projected-path membership)
* ``collision.risk.assess_risk``                                        (SAFE/WARNING/CRITICAL)
* ``clearance.risk.assess_clearance``                                   (clearance status)

-- so the boundaries come from ``Settings.collision_*`` / ``Settings.clearance_*``, the same
constants the simulation pipeline uses. Nothing is hand-picked. (The mock does not run the 2D
swept-footprint predictor ``collision.prediction.simulate_collision``; ``collision_predicted``
is left ``False`` -- an STM32 build may or may not include that stage, and ``assess_risk`` is
fully defined without it.)

Named scenes
-----------
Selected by ``Settings.esp32_mock_scenario`` when ``esp32_transport == "mock"``:

* ``"approaching_vehicle"`` (default) -- unchanged legacy behaviour
  (`transport.build_scripted_approaching_vehicle_frames`).
* ``"realtime_arc"``  -- the Phase-10 six-phase arc:
  empty -> object @10 m (SAFE) -> @7 m (SAFE) -> @5 m (WARNING) -> @3 m (CRITICAL) -> receding.
* ``"multi_object"`` -- vehicle + pole + wall, three persistent, distinct track IDs.
* ``"tracking"``     -- one object, one track ID, 10 m -> 7 m -> 5 m (track-persistence check).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from clearance.risk import assess_clearance
from collision.geometry import VehicleFootprint, in_projected_path, vehicle_footprint
from collision.risk import assess_risk
from collision.ttc import closing_speed_along, compute_ttc
from common.config import Settings, get_settings
from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel, VehicleState
from models.objects import ObjectClassification, Point2D, Velocity2D
from models.stm32_processed import (
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

_RISK_RANK: dict[RiskLevel, int] = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}

MOCK_STM32_SCENARIOS: tuple[str, ...] = (
    "approaching_vehicle",
    "realtime_arc",
    "multi_object",
    "tracking",
)


@dataclass
class MockObjectState:
    """One object's state *in a single frame* -- scripted, not sensed."""

    track_id: str
    object_type: ObjectClassification
    position: Point2D
    relative_velocity: Velocity2D | None
    confidence: float = 0.9
    source: STM32SourceChannel = STM32SourceChannel.FUSED
    radar_range_m: float | None = None
    radar_target_id: str | None = None


@dataclass
class _MockChannels:
    lidar_ok: bool = True
    radar_ok: bool = True
    fusion_active: bool = True
    fault_codes: list[str] = field(default_factory=list)


# --------------------------------------------------------------------------------------------
# core: one scripted frame  -> one validated-shape STM32ProcessedFrame
# --------------------------------------------------------------------------------------------

def build_mock_processed_frame(
    object_states: list[MockObjectState],
    *,
    frame_index: int,
    sequence_number: int | None = None,
    timestamp: float,
    settings: Settings | None = None,
    source_id: str | None = None,
    channels: _MockChannels | None = None,
    vehicle_state: VehicleState | None = None,
) -> STM32ProcessedFrame:
    """Assemble one `STM32ProcessedFrame` from a scripted per-object state list, deriving
    TTC / projected-path / per-object risk / overall risk / directional clearance with the
    project's own functions and thresholds. Pure; no wall-clock, no I/O."""
    settings = settings or get_settings()
    channels = channels or _MockChannels()
    ego = vehicle_state or VehicleState(speed_mps=0.0)
    footprint = vehicle_footprint(settings)
    heading = ego.pose.heading

    objects: list[STM32TrackedObject] = []
    for st in object_states:
        rel_vel = st.relative_velocity or Velocity2D(vx=0.0, vy=0.0)
        distance_m = math.hypot(st.position.x, st.position.y)
        in_path = in_projected_path(st.position, ego, footprint, settings)
        ttc = compute_ttc(st.position, rel_vel, heading, footprint, 0.0, 0.0, settings)
        closing = closing_speed_along(st.position, rel_vel, heading)
        risk, reasons = assess_risk(
            in_path=in_path,
            distance=distance_m,
            ttc=ttc,
            closing_speed=closing,
            collision_predicted=False,
            predicted_collision_time=None,
            settings=settings,
        )
        del reasons  # per-object risk reasons are not part of the Phase-2 contract
        objects.append(
            STM32TrackedObject(
                track_id=st.track_id,
                object_type=st.object_type,
                position=Point2D(x=round(st.position.x, 4), y=round(st.position.y, 4)),
                distance_m=round(distance_m, 4),
                relative_velocity=st.relative_velocity,
                confidence=st.confidence,
                ttc_s=None if ttc is None else round(ttc, 4),
                risk=risk,
                source=st.source,
                in_projected_path=in_path,
                radar_range_m=st.radar_range_m,
                radar_target_id=st.radar_target_id,
            )
        )

    overall_risk, most_critical_id, crit_reasons = _aggregate_risk(objects)
    clearance = _directional_clearance(object_states, settings, footprint)

    return STM32ProcessedFrame(
        metadata=STM32FrameMetadata(
            frame_id=frame_index,
            sequence_number=frame_index if sequence_number is None else sequence_number,
            timestamp=round(timestamp, 6),
            source_id=source_id or settings.hardware_source_id,
            active_object_count=len(objects),
        ),
        sensor_status=STM32SensorStatus(
            lidar=STM32ChannelHealth(connected=channels.lidar_ok, ok=channels.lidar_ok),
            radar=STM32ChannelHealth(connected=channels.radar_ok, ok=channels.radar_ok),
            fusion_active=channels.fusion_active,
        ),
        vehicle_state=ego,
        objects=objects,
        clearance=clearance,
        risk=STM32Risk(
            overall_risk=overall_risk,
            most_critical_track_id=most_critical_id,
            collision_predicted=overall_risk is RiskLevel.CRITICAL,
            predicted_collision_time_s=(
                _critical_ttc(objects, most_critical_id) if overall_risk is RiskLevel.CRITICAL else None
            ),
            reason=crit_reasons,
        ),
        system_status=STM32SystemStatus(
            stm32_ok=True,
            processing_time_ms=None,  # a MOCK -- no real measurement to report
            can_tx_ok=True,
            fault_codes=list(channels.fault_codes),
            notes="MOCK STM32 HARDWARE OUTPUT -- scripted, not sensed.",
        ),
    )


def _aggregate_risk(objects: list[STM32TrackedObject]) -> tuple[RiskLevel, str | None, list[str]]:
    if not objects:
        return RiskLevel.SAFE, None, ["No objects reported -- SAFE."]
    worst = max(objects, key=lambda o: (_RISK_RANK[o.risk], -(o.ttc_s if o.ttc_s is not None else 1e9), -o.distance_m))
    reason = [
        f"Overall risk {worst.risk.value.upper()} driven by track {worst.track_id} "
        f"at {worst.distance_m:.2f} m"
        + (f", TTC {worst.ttc_s:.2f} s" if worst.ttc_s is not None else ", TTC n/a")
        + "."
    ]
    driver = worst.track_id if worst.risk is not RiskLevel.SAFE else None
    return worst.risk, driver, reason


def _critical_ttc(objects: list[STM32TrackedObject], track_id: str | None) -> float | None:
    for o in objects:
        if o.track_id == track_id and o.ttc_s is not None:
            return o.ttc_s
    return None


def _directional_clearance(
    object_states: list[MockObjectState],
    settings: Settings,
    footprint: VehicleFootprint,
) -> STM32Clearance:
    """Directional (front/rear/left/right) clearance from the scripted object positions:
    for each direction, the smallest gap between the vehicle's safety-envelope edge and an
    object that lies in that sector. No object in a direction -> the sensor's own
    ``lidar_range_max_m`` (i.e. "clear as far as we can see"). ``assess_clearance`` (project
    function, project thresholds) then classifies the closest of the four."""
    obj_r = settings.collision_minimum_object_radius_m
    half_len = settings.vehicle_length_m / 2.0
    far = float(settings.lidar_range_max_m)

    gaps = {d: far for d in ClearanceDirection}
    for st in object_states:
        x, y = st.position.x, st.position.y
        if x > 0 and abs(y) <= footprint.envelope_left:
            gaps[ClearanceDirection.FRONT] = min(gaps[ClearanceDirection.FRONT], max(0.0, x - footprint.envelope_front - obj_r))
        if x < 0 and abs(y) <= footprint.envelope_left:
            gaps[ClearanceDirection.REAR] = min(gaps[ClearanceDirection.REAR], max(0.0, -x - footprint.envelope_rear - obj_r))
        if y > 0 and abs(x) <= half_len:
            gaps[ClearanceDirection.LEFT] = min(gaps[ClearanceDirection.LEFT], max(0.0, y - footprint.envelope_left - obj_r))
        if y < 0 and abs(x) <= half_len:
            gaps[ClearanceDirection.RIGHT] = min(gaps[ClearanceDirection.RIGHT], max(0.0, -y - footprint.envelope_right - obj_r))

    gaps = {d: round(v, 4) for d, v in gaps.items()}
    min_direction = min(gaps, key=lambda d: gaps[d])
    min_clearance = gaps[min_direction]
    status, _ = assess_clearance(min_clearance, min_direction, settings)
    corridor = round(
        gaps[ClearanceDirection.LEFT] + footprint.envelope_left + footprint.envelope_right + gaps[ClearanceDirection.RIGHT],
        4,
    )
    return STM32Clearance(
        front_m=gaps[ClearanceDirection.FRONT],
        rear_m=gaps[ClearanceDirection.REAR],
        left_m=gaps[ClearanceDirection.LEFT],
        right_m=gaps[ClearanceDirection.RIGHT],
        min_clearance_m=min_clearance,
        min_direction=min_direction,
        status=status,
        corridor_width_m=corridor,
    )


# --------------------------------------------------------------------------------------------
# named scenes
# --------------------------------------------------------------------------------------------

# The real-time arc as (checkpoint distance in metres, reported closing speed vx in m/s).
# Phase 1 is the empty frame. vx is the STM32 tracker's (filtered) closing-speed estimate --
# not necessarily the exact frame-to-frame position derivative, exactly as a real Kalman-filtered
# tracker's velocity output lags and smooths position. Risk/TTC below are the project's own
# assess_risk / compute_ttc over (checkpoint distance, vx).
_ARC_PHASES: tuple[tuple[float, float], ...] = (
    (10.0, -0.6),   # 2: far, closing slowly            -> SAFE
    (7.0, -0.8),    # 3: closer, still SAFE
    (5.0, -0.7),    # 4: at the warning distance        -> WARNING
    (3.0, -1.0),    # 5: inside the safety envelope     -> CRITICAL (TTC 0.0)
    (6.0, +1.5),    # 6: receding                       -> SAFE (TTC undefined)
)


def _arc_object(x: float, vx: float) -> MockObjectState:
    return MockObjectState(
        track_id="mock-obj-1",
        object_type=ObjectClassification.VEHICLE_LIKE,
        position=Point2D(x=round(x, 4), y=0.0),
        relative_velocity=Velocity2D(vx=vx, vy=0.0),
        source=STM32SourceChannel.FUSED,
        radar_range_m=round(abs(x), 3),
        radar_target_id="r121-tgt-1",
    )


def build_realtime_arc_frames(
    settings: Settings | None = None,
    *,
    live: bool = False,
    empty_frames: int = 1,
    scan_period_s: float = 0.1,
    start_timestamp: float | None = None,
    source_id: str | None = None,
) -> list[STM32ProcessedFrame]:
    """The Phase-10 real-time arc.

    ``live=False`` (default) -> EXACTLY the six frames the brief describes: one empty frame then
    one frame per :data:`_ARC_PHASES` checkpoint. ``live=True`` -> the same arc with each phase
    expanded into a velocity-consistent run of frames (``x`` advances by ``vx * scan_period_s``
    per frame from the previous checkpoint to this one), for a continuously-moving demo.

    Overall risk at the checkpoints: SAFE, SAFE, SAFE, WARNING, CRITICAL, SAFE.
    """
    settings = settings or get_settings()
    t0 = time.time() if start_timestamp is None else start_timestamp

    per_frame_states: list[list[MockObjectState]] = [[] for _ in range(max(0, empty_frames))]
    x_prev = _ARC_PHASES[0][0]
    for pi, (x_target, vx) in enumerate(_ARC_PHASES):
        if not live:
            per_frame_states.append([_arc_object(x_target, vx)])
            x_prev = x_target
            continue
        step = abs(vx) * scan_period_s
        n = 1 if step <= 0 else max(1, round(abs(x_target - x_prev) / step))
        for k in range(1, n + 1):
            x = x_prev + (x_target - x_prev) * k / n
            per_frame_states.append([_arc_object(x, vx)])
        x_prev = x_target

    frames: list[STM32ProcessedFrame] = []
    for i, states in enumerate(per_frame_states):
        frames.append(
            build_mock_processed_frame(
                states, frame_index=i, timestamp=t0 + i * scan_period_s,
                settings=settings, source_id=source_id,
            )
        )
    return frames


def build_multi_object_frames(
    settings: Settings | None = None,
    *,
    frame_count: int = 24,
    vehicle_start_x: float = 9.0,
    vehicle_min_x: float = 3.0,
    vehicle_vx: float = -1.0,
    scan_period_s: float = 0.1,
    start_timestamp: float | None = None,
    source_id: str | None = None,
) -> list[STM32ProcessedFrame]:
    """Vehicle + pole + wall, each with its own stable track ID for the whole run. The vehicle
    closes head-on at a constant ``vehicle_vx`` (position is the integral of velocity -- ``x``
    steps by ``vehicle_vx * scan_period_s`` each frame, floored at ``vehicle_min_x``), so front
    clearance and risk change frame to frame. The pole (left) and wall (right) are static and
    outside the projected path -> SAFE, steady lateral clearance -- the same distinct track IDs
    every frame."""
    settings = settings or get_settings()
    t0 = time.time() if start_timestamp is None else start_timestamp
    n = max(2, frame_count)

    frames: list[STM32ProcessedFrame] = []
    for i in range(n):
        veh_x = max(vehicle_min_x, vehicle_start_x + vehicle_vx * scan_period_s * i)
        moving = veh_x > vehicle_min_x
        states = [
            MockObjectState(
                track_id="mock-veh-1",
                object_type=ObjectClassification.VEHICLE_LIKE,
                position=Point2D(x=round(veh_x, 4), y=0.0),
                relative_velocity=Velocity2D(vx=vehicle_vx if moving else 0.0, vy=0.0),
                source=STM32SourceChannel.FUSED,
                radar_range_m=round(veh_x, 3),
                radar_target_id="r121-tgt-veh",
            ),
            MockObjectState(
                track_id="mock-pole-1",
                object_type=ObjectClassification.POLE_LIKE,
                position=Point2D(x=1.5, y=3.0),
                relative_velocity=Velocity2D(vx=0.0, vy=0.0),
                source=STM32SourceChannel.LIDAR,
                confidence=0.8,
            ),
            MockObjectState(
                track_id="mock-wall-1",
                object_type=ObjectClassification.WALL,
                position=Point2D(x=2.0, y=-3.6),
                relative_velocity=Velocity2D(vx=0.0, vy=0.0),
                source=STM32SourceChannel.LIDAR,
                confidence=0.85,
            ),
        ]
        frames.append(
            build_mock_processed_frame(
                states, frame_index=i, timestamp=t0 + i * scan_period_s,
                settings=settings, source_id=source_id,
            )
        )
    return frames


def build_tracking_frames(
    settings: Settings | None = None,
    *,
    start_x: float = 10.0,
    end_x: float = 5.0,
    closing_mps: float = 1.2,
    scan_period_s: float = 0.1,
    start_timestamp: float | None = None,
    source_id: str | None = None,
) -> list[STM32ProcessedFrame]:
    """One object, one track ID (``mock-obj-1``), closing head-on from ``start_x`` to ``end_x``
    at a constant ``closing_mps`` (``x`` steps by ``closing_mps * scan_period_s`` per frame --
    position is velocity-consistent). Proves the Edge shows ONE continuously-tracked object with
    a monotonically decreasing distance, not a new object every frame."""
    settings = settings or get_settings()
    t0 = time.time() if start_timestamp is None else start_timestamp
    step = max(1e-6, closing_mps * scan_period_s)
    n = max(2, int(math.ceil((start_x - end_x) / step)) + 1)

    frames: list[STM32ProcessedFrame] = []
    for i in range(n):
        x = max(end_x, start_x - step * i)
        frames.append(
            build_mock_processed_frame(
                [_arc_object(x, -closing_mps)],
                frame_index=i, timestamp=t0 + i * scan_period_s,
                settings=settings, source_id=source_id,
            )
        )
    return frames


def build_mock_stm32_frames(
    scenario: str,
    settings: Settings | None = None,
    *,
    start_timestamp: float | None = None,
    source_id: str | None = None,
    live: bool = False,
) -> list[STM32ProcessedFrame]:
    """Dispatch by scene name. ``live=True`` picks the interpolated (smooth-motion) variants for
    a continuous transport; ``live=False`` picks the minimal canonical variants for tests.
    ``"approaching_vehicle"`` is handled by the caller (legacy builder) -- included here only so
    the name validates."""
    settings = settings or get_settings()
    name = (scenario or "").strip().lower()
    if name in ("", "approaching_vehicle"):
        from .transport import build_scripted_approaching_vehicle_frames

        return build_scripted_approaching_vehicle_frames(
            max(1, settings.esp32_mock_frame_count),
            source_id=source_id or settings.hardware_source_id,
            start_timestamp=start_timestamp,
        )
    if name == "realtime_arc":
        return build_realtime_arc_frames(
            settings, live=live, empty_frames=15 if live else 1,
            start_timestamp=start_timestamp, source_id=source_id,
        )
    if name == "multi_object":
        return build_multi_object_frames(
            settings, frame_count=70 if live else 24,
            start_timestamp=start_timestamp, source_id=source_id,
        )
    if name == "tracking":
        return build_tracking_frames(
            settings, start_timestamp=start_timestamp, source_id=source_id,
        )
    raise ValueError(
        f"unknown mock STM32 scenario {scenario!r}; known: {', '.join(MOCK_STM32_SCENARIOS)}"
    )


__all__ = [
    "MOCK_STM32_SCENARIOS",
    "MockObjectState",
    "build_mock_processed_frame",
    "build_mock_stm32_frames",
    "build_realtime_arc_frames",
    "build_multi_object_frames",
    "build_tracking_frames",
]

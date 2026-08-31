"""`STM32ProcessedFrame` (Phase-2 ProcessedPerceptionData) -> semantic CAN payloads, + validation.

Pure extraction: every value is copied from the already-validated processed frame or is a
mechanical derivation (enum -> ordinal, seconds -> ms, a stable hash of `track_id`). No
perception. The three payload types line up with the proposed message catalogue
(`message_spec.build_default_message_catalog`): `PerceptionHeaderPayload` / `ObjectStatePayload`
(one per tracked object) / `SafetyStatePayload`.

`validate_*` re-checks each signal at CAN granularity (a subtly-out-of-range value must not be
silently transmitted -- Phase-5 requirement 6) and raises `CANValidationError` naming the field.
"""

from __future__ import annotations

import math
import zlib
from dataclasses import dataclass, field

from models.collision import RiskLevel
from models.stm32_processed import STM32ProcessedFrame, STM32SourceChannel, STM32TrackedObject

from .errors import CANValidationError
from .message_spec import (
    clearance_direction_code,
    clearance_state_code,
    object_type_code,
    risk_code,
)

# Edge-side plausibility bounds (NOT CAN spec -- signal ranges come from the DBC once it exists;
# these only catch a corrupt value before it reaches the bus). Mirrors can_output/validation of
# the Phase-2 contract's own limits.
MAX_PLAUSIBLE_DISTANCE_M = 1000.0
MAX_PLAUSIBLE_TTC_S = 3600.0
MAX_PLAUSIBLE_SPEED_MPS = 200.0

_SOURCE_CODE = {STM32SourceChannel.LIDAR: 0, STM32SourceChannel.RADAR: 1, STM32SourceChannel.FUSED: 2}


def track_id_hash(track_id: str) -> int:
    """A stable, deterministic 16-bit hash of a string track id -- a CAN signal cannot carry an
    arbitrary-length string, so the numeric identity is this hash. Collisions are possible but
    astronomically unlikely for the handful of live tracks; the full string id stays available on
    the ESP32 / LiveState path. `crc32` (from zlib, already a dependency-free stdlib) truncated to
    16 bits."""
    return zlib.crc32(track_id.encode("utf-8")) & 0xFFFF


# --- payload models (plain dataclasses -- these never cross a wire, they feed the encoder) ---


@dataclass
class PerceptionHeaderPayload:
    sequence_number: int
    frame_id: int
    timestamp_ms: int
    active_object_count: int
    lidar_ok: bool
    radar_ok: bool
    fusion_active: bool
    stm32_ok: bool


@dataclass
class ObjectStatePayload:
    object_index: int
    track_id: str
    track_id_hash: int
    object_type: str
    object_type_code: int
    distance_m: float
    relative_velocity_mps: float
    relative_speed_mps: float
    confidence: float
    ttc_s: float | None
    risk: str
    risk_code: int
    source: str
    source_code: int
    sequence_number: int
    timestamp_ms: int


@dataclass
class SafetyStatePayload:
    overall_risk: str
    overall_risk_code: int
    collision_predicted: bool
    most_critical_track_id: str | None
    most_critical_ttc_s: float | None
    min_clearance_m: float | None
    min_clearance_direction: str | None
    min_clearance_direction_code: int | None
    clearance_state: str | None
    clearance_state_code: int | None
    front_m: float | None
    rear_m: float | None
    left_m: float | None
    right_m: float | None
    sequence_number: int
    timestamp_ms: int


@dataclass
class ProcessedPerceptionPayloads:
    header: PerceptionHeaderPayload
    safety: SafetyStatePayload
    objects: list[ObjectStatePayload] = field(default_factory=list)


# --- extraction -------------------------------------------------------------------------------


def _rel_speed(o: STM32TrackedObject) -> tuple[float, float]:
    if o.relative_velocity is None:
        return 0.0, 0.0
    v = o.relative_velocity
    return v.vx, math.hypot(v.vx, v.vy)


def extract_payloads(frame: STM32ProcessedFrame, *, max_objects: int | None = None) -> ProcessedPerceptionPayloads:
    """Turn one `STM32ProcessedFrame` into the semantic CAN payloads. `max_objects` bounds how
    many OBJECT_STATE instances are produced (bus-load cap; the rest are simply not emitted --
    `active_object_count` in the header still reports the true total)."""
    m = frame.metadata
    ts_ms = int(round(m.timestamp * 1000.0))
    ss = frame.sensor_status

    header = PerceptionHeaderPayload(
        sequence_number=m.sequence_number,
        frame_id=m.frame_id,
        timestamp_ms=ts_ms,
        active_object_count=m.active_object_count,
        lidar_ok=bool(ss.lidar.connected and (ss.lidar.ok is not False)),
        radar_ok=bool(ss.radar is not None and ss.radar.connected and (ss.radar.ok is not False)),
        fusion_active=bool(ss.fusion_active),
        stm32_ok=bool(frame.system_status.stm32_ok),
    )

    objects: list[ObjectStatePayload] = []
    for i, o in enumerate(frame.objects):
        if max_objects is not None and i >= max_objects:
            break
        vx, speed = _rel_speed(o)
        objects.append(
            ObjectStatePayload(
                object_index=i,
                track_id=o.track_id,
                track_id_hash=track_id_hash(o.track_id),
                object_type=o.object_type.value,
                object_type_code=object_type_code(o.object_type),
                distance_m=o.distance_m,
                relative_velocity_mps=vx,
                relative_speed_mps=speed,
                confidence=o.confidence,
                ttc_s=o.ttc_s,
                risk=o.risk.value,
                risk_code=risk_code(o.risk),
                source=o.source.value,
                source_code=_SOURCE_CODE.get(o.source, 0),
                sequence_number=m.sequence_number,
                timestamp_ms=ts_ms,
            )
        )

    c = frame.clearance
    safety = SafetyStatePayload(
        overall_risk=frame.risk.overall_risk.value,
        overall_risk_code=risk_code(frame.risk.overall_risk),
        collision_predicted=bool(frame.risk.collision_predicted),
        most_critical_track_id=frame.risk.most_critical_track_id,
        most_critical_ttc_s=_most_critical_ttc(frame),
        min_clearance_m=c.min_clearance_m if c is not None else None,
        min_clearance_direction=c.min_direction.value if c is not None else None,
        min_clearance_direction_code=clearance_direction_code(c.min_direction) if c is not None else None,
        clearance_state=c.status.value if c is not None else None,
        clearance_state_code=clearance_state_code(c.status) if c is not None else None,
        front_m=c.front_m if c is not None else None,
        rear_m=c.rear_m if c is not None else None,
        left_m=c.left_m if c is not None else None,
        right_m=c.right_m if c is not None else None,
        sequence_number=m.sequence_number,
        timestamp_ms=ts_ms,
    )
    return ProcessedPerceptionPayloads(header=header, safety=safety, objects=objects)


def _most_critical_ttc(frame: STM32ProcessedFrame) -> float | None:
    tid = frame.risk.most_critical_track_id
    if tid is None:
        return None
    return next((o.ttc_s for o in frame.objects if o.track_id == tid), None)


# --- validation (requirement 6) -------------------------------------------------------------


def _finite_nonneg(value: float, field_name: str, *, ceiling: float | None = None) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise CANValidationError(f"{field_name} must be a finite number, got {value!r}.")
    if value < 0:
        raise CANValidationError(f"{field_name} must be >= 0, got {value}.")
    if ceiling is not None and value > ceiling:
        raise CANValidationError(f"{field_name} {value} exceeds the plausible ceiling {ceiling}.")


def _finite(value: float, field_name: str, *, limit: float | None = None) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise CANValidationError(f"{field_name} must be a finite number, got {value!r}.")
    if limit is not None and abs(value) > limit:
        raise CANValidationError(f"{field_name} magnitude {value} exceeds the plausible limit {limit}.")


def _non_negative_int(value: object, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CANValidationError(f"{field_name} must be a non-negative integer, got {value!r}.")


def validate_header(p: PerceptionHeaderPayload) -> None:
    _non_negative_int(p.sequence_number, "header.sequence_number")
    _non_negative_int(p.frame_id, "header.frame_id")
    _non_negative_int(p.timestamp_ms, "header.timestamp_ms")
    _non_negative_int(p.active_object_count, "header.active_object_count")
    if p.timestamp_ms < 1_000_000_000_000:  # ~2001 in ms
        raise CANValidationError(f"header.timestamp_ms {p.timestamp_ms} is not a plausible Unix-epoch-ms value.")


def validate_object_state(p: ObjectStatePayload) -> None:
    if not (isinstance(p.track_id, str) and p.track_id.strip()):
        raise CANValidationError(f"object[{p.object_index}].track_id must be a non-empty string.")
    _non_negative_int(p.object_index, f"object[{p.object_index}].object_index")
    _non_negative_int(p.sequence_number, f"object[{p.object_index}].sequence_number")
    if p.object_type_code not in range(0, 64):
        raise CANValidationError(f"object[{p.object_index}].object_type_code {p.object_type_code} out of range.")
    _finite_nonneg(p.distance_m, f"object[{p.object_index}].distance_m", ceiling=MAX_PLAUSIBLE_DISTANCE_M)
    _finite(p.relative_velocity_mps, f"object[{p.object_index}].relative_velocity_mps", limit=MAX_PLAUSIBLE_SPEED_MPS)
    _finite_nonneg(p.relative_speed_mps, f"object[{p.object_index}].relative_speed_mps", ceiling=MAX_PLAUSIBLE_SPEED_MPS)
    if not (math.isfinite(p.confidence) and 0.0 <= p.confidence <= 1.0):
        raise CANValidationError(f"object[{p.object_index}].confidence {p.confidence} outside [0, 1].")
    if p.ttc_s is not None:
        _finite_nonneg(p.ttc_s, f"object[{p.object_index}].ttc_s", ceiling=MAX_PLAUSIBLE_TTC_S)
    if p.risk not in {r.value for r in RiskLevel}:
        raise CANValidationError(f"object[{p.object_index}].risk {p.risk!r} is not a known RiskLevel.")


def validate_safety_state(p: SafetyStatePayload) -> None:
    _non_negative_int(p.sequence_number, "safety.sequence_number")
    if p.overall_risk not in {r.value for r in RiskLevel}:
        raise CANValidationError(f"safety.overall_risk {p.overall_risk!r} is not a known RiskLevel.")
    if p.most_critical_ttc_s is not None:
        _finite_nonneg(p.most_critical_ttc_s, "safety.most_critical_ttc_s", ceiling=MAX_PLAUSIBLE_TTC_S)
    for name, value in (("min_clearance_m", p.min_clearance_m), ("front_m", p.front_m),
                        ("rear_m", p.rear_m), ("left_m", p.left_m), ("right_m", p.right_m)):
        if value is not None:
            _finite_nonneg(value, f"safety.{name}", ceiling=MAX_PLAUSIBLE_DISTANCE_M)
    if p.min_clearance_m is not None and all(v is not None for v in (p.front_m, p.rear_m, p.left_m, p.right_m)):
        true_min = min(p.front_m, p.rear_m, p.left_m, p.right_m)
        if abs(p.min_clearance_m - true_min) > 0.05:
            raise CANValidationError(
                f"safety.min_clearance_m ({p.min_clearance_m}) disagrees with min(front,rear,left,right)={true_min}."
            )


__all__ = [
    "track_id_hash",
    "PerceptionHeaderPayload", "ObjectStatePayload", "SafetyStatePayload", "ProcessedPerceptionPayloads",
    "extract_payloads",
    "validate_header", "validate_object_state", "validate_safety_state",
    "MAX_PLAUSIBLE_DISTANCE_M", "MAX_PLAUSIBLE_TTC_S", "MAX_PLAUSIBLE_SPEED_MPS",
]

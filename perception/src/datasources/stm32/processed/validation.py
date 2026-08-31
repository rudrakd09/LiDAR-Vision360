"""Semantic / cross-field validation for a decoded `STM32ProcessedFrame`.

Pydantic already enforces field presence, types, enum membership, and per-field numeric bounds
(``ge``/``le``) at construction. This module adds the checks pydantic *cannot* express on its
own, and is what a `ProcessedFrameDeserializer` runs after a successful structural parse:

* finiteness -- reject NaN / +-inf on any numeric field (a plain ``float`` field accepts them);
* timestamp plausibility -- a real Unix-epoch-seconds value, not absurdly old or in the future;
* frame/sequence sanity -- non-negative, not a bool, and ``active_object_count == len(objects)``;
* object identity -- every ``track_id`` non-empty and unique within the frame;
* numeric plausibility -- distances/TTC within generous Edge-side sanity limits (NOT hardware
  specs -- see the module constants);
* clearance internal consistency -- ``min_clearance_m`` / ``min_direction`` agree with the four
  directional values;
* referential integrity -- ``risk.most_critical_track_id`` names an object that is present.

Every failure raises `STM32ProcessedFrameError` (or `STM32ContractVersionError` for the version
check) with a message that names the offending field and the reason -- never a silent drop, never
a half-populated object. The limits below are **Edge-side sanity bounds, not hardware
specifications**; they are overridable per call so a future caller can wire them to `Settings`.
"""

from __future__ import annotations

import math
import time
from collections.abc import Iterable

from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel
from models.objects import ObjectClassification
from models.stm32_processed import STM32ProcessedFrame, STM32SourceChannel

from .errors import STM32ProcessedFrameError
from .version import require_supported_version

# --- Edge-side sanity limits (NOT hardware specs; override per call if ever needed) -----------

#: Oldest timestamp treated as plausibly "real": 2001-09-09. Anything below this is almost
#: certainly a device-uptime value that was not converted to Unix epoch, or a zeroed field.
MIN_PLAUSIBLE_EPOCH_S: float = 1_000_000_000.0

#: How far past "now" a frame timestamp may sit before it is rejected as clock-skew/corruption.
DEFAULT_MAX_CLOCK_SKEW_S: float = 60.0

#: Generous upper bound on any reported range/clearance. R121-class radar tops out well under
#: this; it exists only to catch a garbled field, not to model the sensor.
DEFAULT_MAX_DISTANCE_M: float = 1000.0

#: Upper bound on a reported TTC before it is treated as noise rather than a real prediction.
DEFAULT_MAX_TTC_S: float = 3600.0

#: Tolerance when checking that ``min_clearance_m`` equals the smallest directional value.
DEFAULT_CLEARANCE_MIN_TOLERANCE_M: float = 0.05


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise STM32ProcessedFrameError(message)


def _finite(value: float | None, field: str) -> None:
    if value is None:
        return
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise STM32ProcessedFrameError(f"{field} must be a real number, got {value!r}.")
    if not math.isfinite(float(value)):
        raise STM32ProcessedFrameError(f"{field} must be finite, got {value!r}.")


def _non_negative_int(value: object, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise STM32ProcessedFrameError(f"{field} must be a non-negative integer, got {value!r}.")
    if value < 0:
        raise STM32ProcessedFrameError(f"{field} must be >= 0, got {value}.")


def validate_processed_frame(
    frame: STM32ProcessedFrame,
    *,
    now: float | None = None,
    max_clock_skew_s: float = DEFAULT_MAX_CLOCK_SKEW_S,
    min_epoch_s: float = MIN_PLAUSIBLE_EPOCH_S,
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
    max_ttc_s: float = DEFAULT_MAX_TTC_S,
    clearance_min_tolerance_m: float = DEFAULT_CLEARANCE_MIN_TOLERANCE_M,
) -> None:
    """Raise `STM32ProcessedFrameError` / `STM32ContractVersionError` if `frame` is not a
    coherent, plausible processed-perception frame. Returns `None` on success. Pure and
    stateless -- dropped-/duplicate-sequence detection across frames is ESP32Source's job
    (Phase 3), not this function's."""
    now = time.time() if now is None else now

    _validate_metadata(frame, now=now, max_clock_skew_s=max_clock_skew_s, min_epoch_s=min_epoch_s)
    _validate_sensor_status(frame)
    if frame.vehicle_state is not None:
        _validate_vehicle_state(frame)
    track_ids = _validate_objects(frame, max_distance_m=max_distance_m, max_ttc_s=max_ttc_s)
    _validate_risk(frame, track_ids)
    if frame.clearance is not None:
        _validate_clearance(frame, max_distance_m=max_distance_m, tolerance_m=clearance_min_tolerance_m)
    _validate_system_status(frame)


# --- section validators ----------------------------------------------------------------------


def _validate_metadata(frame: STM32ProcessedFrame, *, now: float, max_clock_skew_s: float, min_epoch_s: float) -> None:
    m = frame.metadata

    require_supported_version(m.protocol_version)

    _non_negative_int(m.frame_id, "metadata.frame_id")
    _non_negative_int(m.sequence_number, "metadata.sequence_number")
    _non_negative_int(m.active_object_count, "metadata.active_object_count")

    _require(bool(m.source_id and m.source_id.strip()), "metadata.source_id must be a non-empty string.")

    _finite(m.timestamp, "metadata.timestamp")
    _require(
        m.timestamp >= min_epoch_s,
        f"metadata.timestamp {m.timestamp!r} is not a plausible Unix-epoch-seconds value "
        f"(< {min_epoch_s:.0f}); a device-uptime value was likely not converted.",
    )
    _require(
        m.timestamp <= now + max_clock_skew_s,
        f"metadata.timestamp {m.timestamp!r} is more than {max_clock_skew_s:.0f}s in the future "
        f"(now={now:.3f}) -- rejecting as clock skew / corruption.",
    )

    _require(
        m.active_object_count == len(frame.objects),
        f"metadata.active_object_count ({m.active_object_count}) does not match the number of "
        f"objects in the frame ({len(frame.objects)}).",
    )


def _validate_sensor_status(frame: STM32ProcessedFrame) -> None:
    s = frame.sensor_status
    for name, channel in (("lidar", s.lidar), ("radar", s.radar)):
        if channel is None:
            continue
        _non_negative_int_or_none(channel.frames_received, f"sensor_status.{name}.frames_received")
        _non_negative_int_or_none(channel.frames_dropped, f"sensor_status.{name}.frames_dropped")
        _finite(channel.last_update_timestamp, f"sensor_status.{name}.last_update_timestamp")


def _non_negative_int_or_none(value: object, field: str) -> None:
    if value is None:
        return
    _non_negative_int(value, field)


def _validate_vehicle_state(frame: STM32ProcessedFrame) -> None:
    vs = frame.vehicle_state
    assert vs is not None
    _finite(vs.pose.x, "vehicle_state.pose.x")
    _finite(vs.pose.y, "vehicle_state.pose.y")
    _finite(vs.pose.heading, "vehicle_state.pose.heading")
    _finite(vs.speed_mps, "vehicle_state.speed_mps")


def _validate_objects(frame: STM32ProcessedFrame, *, max_distance_m: float, max_ttc_s: float) -> set[str]:
    seen: set[str] = set()
    for i, obj in enumerate(frame.objects):
        where = f"objects[{i}] (track_id={obj.track_id!r})"

        _require(bool(obj.track_id and obj.track_id.strip()), f"{where}: track_id must be a non-empty string.")
        _require(obj.track_id not in seen, f"objects[{i}]: duplicate track_id {obj.track_id!r} within the frame.")
        seen.add(obj.track_id)

        _require(
            isinstance(obj.object_type, ObjectClassification),
            f"{where}: object_type {obj.object_type!r} is not a known ObjectClassification.",
        )
        _require(
            isinstance(obj.source, STM32SourceChannel),
            f"{where}: source {obj.source!r} is not a known STM32SourceChannel.",
        )
        _require(
            isinstance(obj.risk, RiskLevel),
            f"{where}: risk {obj.risk!r} is not a known RiskLevel.",
        )

        _finite(obj.position.x, f"{where}: position.x")
        _finite(obj.position.y, f"{where}: position.y")

        _finite(obj.distance_m, f"{where}: distance_m")
        _require(0.0 <= obj.distance_m <= max_distance_m, f"{where}: distance_m {obj.distance_m} outside [0, {max_distance_m}].")

        _finite(obj.confidence, f"{where}: confidence")
        _require(0.0 <= obj.confidence <= 1.0, f"{where}: confidence {obj.confidence} outside [0, 1].")

        if obj.relative_velocity is not None:
            _finite(obj.relative_velocity.vx, f"{where}: relative_velocity.vx")
            _finite(obj.relative_velocity.vy, f"{where}: relative_velocity.vy")

        if obj.ttc_s is not None:
            _finite(obj.ttc_s, f"{where}: ttc_s")
            _require(0.0 <= obj.ttc_s <= max_ttc_s, f"{where}: ttc_s {obj.ttc_s} outside [0, {max_ttc_s}].")

        if obj.radar_range_m is not None:
            _finite(obj.radar_range_m, f"{where}: radar_range_m")
            _require(0.0 <= obj.radar_range_m <= max_distance_m, f"{where}: radar_range_m {obj.radar_range_m} outside [0, {max_distance_m}].")

    return seen


def _validate_risk(frame: STM32ProcessedFrame, track_ids: set[str]) -> None:
    r = frame.risk
    _require(isinstance(r.overall_risk, RiskLevel), f"risk.overall_risk {r.overall_risk!r} is not a known RiskLevel.")

    if r.most_critical_track_id is not None:
        _require(
            r.most_critical_track_id in track_ids,
            f"risk.most_critical_track_id {r.most_critical_track_id!r} does not name any object in the frame.",
        )

    if r.predicted_collision_time_s is not None:
        _finite(r.predicted_collision_time_s, "risk.predicted_collision_time_s")
        _require(r.predicted_collision_time_s >= 0.0, "risk.predicted_collision_time_s must be >= 0.")

    if r.collision_predicted:
        _require(
            r.predicted_collision_time_s is not None,
            "risk.collision_predicted is true but risk.predicted_collision_time_s is null.",
        )


def _validate_clearance(frame: STM32ProcessedFrame, *, max_distance_m: float, tolerance_m: float) -> None:
    c = frame.clearance
    assert c is not None

    directions = {
        ClearanceDirection.FRONT: c.front_m,
        ClearanceDirection.REAR: c.rear_m,
        ClearanceDirection.LEFT: c.left_m,
        ClearanceDirection.RIGHT: c.right_m,
    }
    for direction, value in directions.items():
        _finite(value, f"clearance.{direction.value}_m")
        _require(0.0 <= value <= max_distance_m, f"clearance.{direction.value}_m {value} outside [0, {max_distance_m}].")

    _finite(c.min_clearance_m, "clearance.min_clearance_m")
    _require(isinstance(c.status, ClearanceState), f"clearance.status {c.status!r} is not a known ClearanceState.")
    _require(isinstance(c.min_direction, ClearanceDirection), f"clearance.min_direction {c.min_direction!r} is not a known ClearanceDirection.")

    true_min = min(directions.values())
    _require(
        abs(c.min_clearance_m - true_min) <= tolerance_m,
        f"clearance.min_clearance_m ({c.min_clearance_m}) disagrees with min(front,rear,left,right) "
        f"= {true_min} by more than {tolerance_m}m.",
    )
    _require(
        abs(directions[c.min_direction] - true_min) <= tolerance_m,
        f"clearance.min_direction ({c.min_direction.value}) is not the smallest directional "
        f"clearance (that is {min(directions, key=directions.get).value} at {true_min}m).",
    )

    if c.corridor_width_m is not None:
        _finite(c.corridor_width_m, "clearance.corridor_width_m")
        _require(c.corridor_width_m >= 0.0, "clearance.corridor_width_m must be >= 0.")


def _validate_system_status(frame: STM32ProcessedFrame) -> None:
    ss = frame.system_status
    if ss.processing_load is not None:
        _finite(ss.processing_load, "system_status.processing_load")
        _require(0.0 <= ss.processing_load <= 1.0, f"system_status.processing_load {ss.processing_load} outside [0, 1].")
    if ss.processing_time_ms is not None:
        _finite(ss.processing_time_ms, "system_status.processing_time_ms")
        _require(ss.processing_time_ms >= 0.0, "system_status.processing_time_ms must be >= 0.")


def _iter_required_paths() -> Iterable[str]:  # pragma: no cover - documentation helper
    """The always-required top-level sections, for docs/tests. A frame missing any of these
    fails structural (pydantic) parsing, wrapped as `STM32ProcessedFrameError` by the
    deserializer -- see test case 12."""
    yield from ("metadata", "sensor_status", "risk", "system_status")


__all__ = [
    "validate_processed_frame",
    "MIN_PLAUSIBLE_EPOCH_S",
    "DEFAULT_MAX_CLOCK_SKEW_S",
    "DEFAULT_MAX_DISTANCE_M",
    "DEFAULT_MAX_TTC_S",
    "DEFAULT_CLEARANCE_MIN_TOLERANCE_M",
]

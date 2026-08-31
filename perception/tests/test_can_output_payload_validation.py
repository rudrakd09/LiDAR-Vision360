"""Phase 5 -- `STM32ProcessedFrame` -> semantic CAN payloads + per-signal validation
(`can_output.payload`). Covers required cases 5-8 (invalid object data / TTC / clearance / risk)
and the field mapping.
"""

from __future__ import annotations

import dataclasses

import pytest

from can_output import (
    CANValidationError,
    extract_payloads,
    track_id_hash,
    validate_header,
    validate_object_state,
    validate_safety_state,
)
from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel, VehicleState
from models.mapping import VehiclePose
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

TS = 1_700_000_000.0


def _obj(tid="t1", **o):
    base = dict(track_id=tid, object_type=ObjectClassification.VEHICLE_LIKE, position=Point2D(x=6.0, y=0.0),
                distance_m=6.0, relative_velocity=Velocity2D(vx=-1.5, vy=0.0), confidence=0.8, ttc_s=4.0,
                risk=RiskLevel.WARNING, source=STM32SourceChannel.FUSED, in_projected_path=True)
    base.update(o)
    return STM32TrackedObject(**base)


def _frame(objects=None, *, clearance=True, overall=RiskLevel.WARNING):
    objs = [_obj()] if objects is None else objects
    clr = STM32Clearance(front_m=1.5, rear_m=4.0, left_m=2.5, right_m=2.5, min_clearance_m=1.5,
                         min_direction=ClearanceDirection.FRONT, status=ClearanceState.CAUTION,
                         corridor_width_m=6.8) if clearance else None
    return STM32ProcessedFrame(
        metadata=STM32FrameMetadata(frame_id=7, sequence_number=7, timestamp=TS, source_id="stm32_hardware",
                                    active_object_count=len(objs)),
        sensor_status=STM32SensorStatus(lidar=STM32ChannelHealth(connected=True, ok=True),
                                        radar=STM32ChannelHealth(connected=True, ok=True), fusion_active=True),
        vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0),
        objects=objs, clearance=clr,
        risk=STM32Risk(overall_risk=overall, most_critical_track_id=objs[0].track_id if objs else None,
                       collision_predicted=overall is RiskLevel.CRITICAL,
                       predicted_collision_time_s=1.1 if overall is RiskLevel.CRITICAL else None),
        system_status=STM32SystemStatus(stm32_ok=True, processing_time_ms=5.0),
    )


class TestExtraction:
    def test_header_fields(self):
        p = extract_payloads(_frame()).header
        assert p.sequence_number == 7 and p.frame_id == 7
        assert p.timestamp_ms == int(TS * 1000)
        assert p.active_object_count == 1
        assert p.lidar_ok is True and p.radar_ok is True and p.fusion_active is True and p.stm32_ok is True

    def test_object_fields_and_codes(self):
        p = extract_payloads(_frame(objects=[_obj("track-9", object_type=ObjectClassification.POLE_LIKE, distance_m=9.0, ttc_s=None, risk=RiskLevel.SAFE)]))
        o = p.objects[0]
        assert o.track_id == "track-9"
        assert o.track_id_hash == track_id_hash("track-9")
        assert o.object_type == "pole_like" and o.object_type_code == 2
        assert o.distance_m == 9.0
        assert o.ttc_s is None
        assert o.risk == "safe" and o.risk_code == 0
        assert o.source == "fused" and o.source_code == 2

    def test_safety_fields(self):
        p = extract_payloads(_frame(overall=RiskLevel.CRITICAL)).safety
        assert p.overall_risk == "critical" and p.overall_risk_code == 2
        assert p.collision_predicted is True
        assert p.min_clearance_m == 1.5
        assert p.min_clearance_direction == "front" and p.min_clearance_direction_code == 0
        assert p.front_m == 1.5 and p.rear_m == 4.0

    def test_multiple_objects_indexed(self):
        p = extract_payloads(_frame(objects=[_obj("a"), _obj("b"), _obj("c")]))
        assert [o.object_index for o in p.objects] == [0, 1, 2]
        assert [o.track_id for o in p.objects] == ["a", "b", "c"]

    def test_no_clearance_yields_none_not_fabricated(self):
        p = extract_payloads(_frame(clearance=False)).safety
        assert p.min_clearance_m is None and p.front_m is None and p.clearance_state is None

    def test_max_objects_cap(self):
        p = extract_payloads(_frame(objects=[_obj(f"t{i}") for i in range(10)]), max_objects=4)
        assert len(p.objects) == 4
        assert p.header.active_object_count == 10  # true total still reported


class TestValidation:
    def _obj_payload(self, **o):
        base = extract_payloads(_frame()).objects[0]
        return dataclasses.replace(base, **o)

    def _safety_payload(self, **o):
        base = extract_payloads(_frame()).safety
        return dataclasses.replace(base, **o)

    # 5. invalid object data
    def test_empty_track_id_rejected(self):
        with pytest.raises(CANValidationError, match="track_id"):
            validate_object_state(self._obj_payload(track_id=""))

    def test_negative_distance_rejected(self):
        with pytest.raises(CANValidationError, match="distance_m"):
            validate_object_state(self._obj_payload(distance_m=-1.0))

    def test_absurd_distance_rejected(self):
        with pytest.raises(CANValidationError, match="distance_m"):
            validate_object_state(self._obj_payload(distance_m=50_000.0))

    def test_non_finite_velocity_rejected(self):
        with pytest.raises(CANValidationError, match="relative_velocity"):
            validate_object_state(self._obj_payload(relative_velocity_mps=float("nan")))

    def test_confidence_out_of_range_rejected(self):
        with pytest.raises(CANValidationError, match="confidence"):
            validate_object_state(self._obj_payload(confidence=1.4))

    # 6. invalid TTC
    def test_negative_ttc_rejected(self):
        with pytest.raises(CANValidationError, match="ttc_s"):
            validate_object_state(self._obj_payload(ttc_s=-0.5))

    def test_nan_ttc_rejected(self):
        with pytest.raises(CANValidationError, match="ttc_s"):
            validate_object_state(self._obj_payload(ttc_s=float("nan")))

    def test_none_ttc_is_valid(self):
        validate_object_state(self._obj_payload(ttc_s=None))  # not-closing -> valid

    # 7. invalid clearance
    def test_negative_clearance_rejected(self):
        with pytest.raises(CANValidationError, match="front_m"):
            validate_safety_state(self._safety_payload(front_m=-0.3))

    def test_min_clearance_inconsistent_rejected(self):
        with pytest.raises(CANValidationError, match="min_clearance_m"):
            validate_safety_state(self._safety_payload(min_clearance_m=0.1, front_m=1.5, rear_m=4.0, left_m=2.5, right_m=2.5))

    # 8. invalid risk
    def test_unknown_object_risk_rejected(self):
        with pytest.raises(CANValidationError, match="RiskLevel"):
            validate_object_state(self._obj_payload(risk="meltdown"))

    def test_unknown_overall_risk_rejected(self):
        with pytest.raises(CANValidationError, match="RiskLevel"):
            validate_safety_state(self._safety_payload(overall_risk="extreme"))

    # sequence
    def test_negative_sequence_rejected(self):
        with pytest.raises(CANValidationError, match="sequence_number"):
            validate_object_state(self._obj_payload(sequence_number=-1))

    def test_valid_payloads_pass(self):
        p = extract_payloads(_frame(objects=[_obj("a"), _obj("b")]))
        validate_header(p.header)
        for o in p.objects:
            validate_object_state(o)
        validate_safety_state(p.safety)

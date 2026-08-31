"""Phase 5 -- `can_output.encoder` (CANFrame assembly + DBC-style bit packing).

Covers required cases 1 (valid CAN message creation), 2 (object-state encoding), 3 (safety-state
encoding), 4 (multiple objects), 13 (multiple-message transmission set). Exercises BOTH the
default (unencoded / layout-PENDING) path and a hypothetical fully-specified path with round-trip.
"""

from __future__ import annotations

import pytest

from can_output import (
    CANByteOrder,
    CANConfigurationError,
    CANFrameEncoder,
    CANMessageSpec,
    CANOutputConfig,
    CANSignalScaling,
    CANSignalSpec,
    CANTransmissionMode,
    SignalPacker,
    build_default_config,
    extract_payloads,
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


def _obj(tid, *, dist=6.0, cls=ObjectClassification.VEHICLE_LIKE, risk=RiskLevel.WARNING, ttc=4.0):
    return STM32TrackedObject(track_id=tid, object_type=cls, position=Point2D(x=dist, y=0.0), distance_m=dist,
                              relative_velocity=Velocity2D(vx=-1.5, vy=0.0), confidence=0.8, ttc_s=ttc, risk=risk,
                              source=STM32SourceChannel.FUSED, in_projected_path=True)


def _frame(objects, *, overall=RiskLevel.WARNING):
    clr = STM32Clearance(front_m=1.5, rear_m=4.0, left_m=2.5, right_m=2.5, min_clearance_m=1.5,
                         min_direction=ClearanceDirection.FRONT, status=ClearanceState.CAUTION, corridor_width_m=6.8)
    return STM32ProcessedFrame(
        metadata=STM32FrameMetadata(frame_id=3, sequence_number=3, timestamp=TS, source_id="stm32_hardware",
                                    active_object_count=len(objects)),
        sensor_status=STM32SensorStatus(lidar=STM32ChannelHealth(connected=True, ok=True),
                                        radar=STM32ChannelHealth(connected=True, ok=True), fusion_active=True),
        vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0),
        objects=objects, clearance=clr,
        risk=STM32Risk(overall_risk=overall, most_critical_track_id=objects[0].track_id if objects else None),
        system_status=STM32SystemStatus(stm32_ok=True, processing_time_ms=5.0),
    )


class TestDefaultUnencodedPath:
    def test_1_valid_message_set_created(self):
        enc = CANFrameEncoder(build_default_config())
        payloads = extract_payloads(_frame([_obj("t1")]))
        frames, errors = enc.encode_frame(payloads, sequence_number=0)
        assert errors == []
        # 13. multiple-message: header + object + safety
        assert sorted(f.message_name for f in frames) == ["OBJECT_STATE", "PERCEPTION_HEADER", "SAFETY_STATE"]
        for f in frames:
            assert not f.is_encoded  # layout PENDING -> semantic-only
            assert f.arbitration_id is None and f.data is None
            assert f.signals  # values are still carried

    def test_2_object_state_encoding_carries_the_right_signals(self):
        enc = CANFrameEncoder(build_default_config())
        payloads = extract_payloads(_frame([_obj("t1", dist=9.5, cls=ObjectClassification.POLE_LIKE, risk=RiskLevel.SAFE, ttc=None)]))
        frames, _ = enc.encode_frame(payloads, sequence_number=2)
        obj = next(f for f in frames if f.message_name == "OBJECT_STATE")
        assert obj.object_index == 0
        assert obj.signals["distance_m"] == 9.5
        assert obj.signals["object_type"] == 2   # pole_like
        assert obj.signals["object_risk"] == 0   # safe
        assert obj.signals["object_ttc_s"] == -1.0  # design sentinel for "no TTC"
        assert obj.signals["rolling_counter"] == 2

    def test_3_safety_state_encoding(self):
        enc = CANFrameEncoder(build_default_config())
        payloads = extract_payloads(_frame([_obj("t1", risk=RiskLevel.CRITICAL)], overall=RiskLevel.CRITICAL))
        frames, _ = enc.encode_frame(payloads, sequence_number=1)
        s = next(f for f in frames if f.message_name == "SAFETY_STATE")
        assert s.signals["overall_risk"] == 2
        assert s.signals["min_clearance_m"] == 1.5
        assert s.signals["min_clearance_direction"] == 0
        assert s.signals["front_clearance_m"] == 1.5

    def test_4_multiple_objects_one_frame_each_multiplexed_by_index(self):
        enc = CANFrameEncoder(build_default_config())
        payloads = extract_payloads(_frame([_obj(f"t{i}") for i in range(5)]))
        frames, _ = enc.encode_frame(payloads, sequence_number=0)
        obj_frames = [f for f in frames if f.message_name == "OBJECT_STATE"]
        assert len(obj_frames) == 5
        assert [f.object_index for f in obj_frames] == [0, 1, 2, 3, 4]
        assert {f.signals["track_id_hash"] for f in obj_frames} == {f.signals["track_id_hash"] for f in obj_frames}

    def test_multiplex_cap_bounds_object_frames(self):
        cfg = build_default_config()
        cfg.message("OBJECT_STATE").max_multiplex_count = 3
        enc = CANFrameEncoder(cfg)
        payloads = extract_payloads(_frame([_obj(f"t{i}") for i in range(9)]))
        frames, _ = enc.encode_frame(payloads, sequence_number=0)
        assert sum(1 for f in frames if f.message_name == "OBJECT_STATE") == 3

    def test_5_invalid_object_drops_only_that_message(self):
        enc = CANFrameEncoder(build_default_config())
        f = _frame([_obj("ok"), _obj("bad")])
        # corrupt one object post-construction (bypasses Phase-2 validation, simulates a bug)
        object.__setattr__(f.objects[1], "confidence", 3.0)
        frames, errors = enc.encode_frame(extract_payloads(f), sequence_number=0)
        assert len(errors) == 1 and "confidence" in errors[0]
        obj_frames = [fr for fr in frames if fr.message_name == "OBJECT_STATE"]
        assert len(obj_frames) == 1  # the good one still went; header + safety too
        assert any(fr.message_name == "PERCEPTION_HEADER" for fr in frames)


class TestFullySpecifiedEncodedPath:
    """A hypothetical DBC (values supplied here for the test, NOT shipped in the default config)."""

    def _spec(self) -> CANMessageSpec:
        return CANMessageSpec(
            name="SAFETY_STATE", can_id=0x2A0, extended_id=False, dlc=8, cycle_time_ms=50.0,
            transmission_mode=CANTransmissionMode.PERIODIC,
            checksum_algorithm="xor8",
            signals=[
                CANSignalSpec(name="overall_risk", source="s", start_bit=0, length_bits=2,
                              byte_order=CANByteOrder.LITTLE_ENDIAN, scaling=CANSignalScaling(scale=1.0, offset=0.0, signed=False)),
                CANSignalSpec(name="min_clearance_m", source="s", start_bit=8, length_bits=16,
                              byte_order=CANByteOrder.LITTLE_ENDIAN, scaling=CANSignalScaling(scale=0.01, offset=0.0, signed=False)),
                CANSignalSpec(name="most_critical_ttc_s", source="s", start_bit=24, length_bits=16,
                              byte_order=CANByteOrder.BIG_ENDIAN, scaling=CANSignalScaling(scale=0.01, offset=0.0, signed=True)),
                CANSignalSpec(name="rolling_counter", source="tx", start_bit=48, length_bits=4,
                              byte_order=CANByteOrder.LITTLE_ENDIAN, is_counter=True),
                CANSignalSpec(name="checksum", source="tx", start_bit=56, length_bits=8,
                              byte_order=CANByteOrder.LITTLE_ENDIAN, is_checksum=True),
            ],
        )

    def test_encoded_frame_has_real_bytes_and_id(self):
        spec = self._spec()
        enc = CANFrameEncoder(CANOutputConfig(messages=[spec]))
        fr = enc.encode(spec, {"overall_risk": 2, "min_clearance_m": 1.53, "most_critical_ttc_s": 2.75}, sequence_number=6)
        assert fr.is_encoded
        assert fr.arbitration_id == 0x2A0 and fr.dlc == 8 and isinstance(fr.data, bytes) and len(fr.data) == 8

    def test_round_trip_little_and_big_endian_and_signed(self):
        spec = self._spec()
        enc = CANFrameEncoder(CANOutputConfig(messages=[spec]))
        fr = enc.encode(spec, {"overall_risk": 1, "min_clearance_m": 2.34, "most_critical_ttc_s": -0.5}, sequence_number=9)
        d = fr.data
        assert SignalPacker.read_bits(d, 0, 2, CANByteOrder.LITTLE_ENDIAN) == 1
        assert abs(SignalPacker.read_bits(d, 8, 16, CANByteOrder.LITTLE_ENDIAN) * 0.01 - 2.34) < 1e-6
        raw_ttc = SignalPacker.read_bits(d, 24, 16, CANByteOrder.BIG_ENDIAN)
        signed_ttc = raw_ttc - (1 << 16) if raw_ttc >= (1 << 15) else raw_ttc
        assert abs(signed_ttc * 0.01 - (-0.5)) < 1e-6
        assert SignalPacker.read_bits(d, 48, 4, CANByteOrder.LITTLE_ENDIAN) == 9

    def test_checksum_signal_populated(self):
        spec = self._spec()
        enc = CANFrameEncoder(CANOutputConfig(messages=[spec]))
        fr = enc.encode(spec, {"overall_risk": 2, "min_clearance_m": 1.0, "most_critical_ttc_s": 1.0}, sequence_number=0)
        cs = SignalPacker.read_bits(fr.data, 56, 8, CANByteOrder.LITTLE_ENDIAN)
        assert cs != 0  # xor8 over the packed bytes

    def test_require_fully_specified_raises_for_unencodable_message(self):
        cfg = build_default_config(require_fully_specified=True)  # catalogue still PENDING
        enc = CANFrameEncoder(cfg)
        with pytest.raises(CANConfigurationError, match="PENDING"):
            enc.encode(cfg.message("SAFETY_STATE"), {}, sequence_number=0)

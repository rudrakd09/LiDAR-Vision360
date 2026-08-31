"""Phase 2 -- STM32 processed-perception contract (`models.stm32_processed` +
`datasources.stm32.processed`).

Covers the 13 required cases from the Phase 2 brief:

  1. valid frame                     8. invalid TTC
  2. empty-object frame              9. invalid clearance
  3. single-object frame            10. invalid risk state
  4. multiple-object frame          11. unknown protocol version
  5. invalid timestamp              12. missing required field
  6. invalid sequence               13. serialization/deserialization round trip
  7. invalid object data

No hardware is touched -- every "wire" here is `JsonProcessedFrameCodec` (the reference codec).
The real STM32<->ESP32 byte layout is deliberately not exercised because it is not specified yet.
"""

from __future__ import annotations

import time

import pytest

from datasources.stm32.processed import (
    HARDWARE_SPEC_DEPENDENT_FIELDS,
    PROCESSED_CONTRACT_VERSION,
    JsonProcessedFrameCodec,
    STM32ChannelHealth,
    STM32Clearance,
    STM32ContractVersionError,
    STM32FrameMetadata,
    STM32ProcessedFrame,
    STM32ProcessedFrameError,
    STM32Risk,
    STM32SensorStatus,
    STM32SourceChannel,
    STM32SystemStatus,
    STM32TrackedObject,
    classify_contract_version,
    parse_processed_frame,
    validate_processed_frame,
)
from datasources.stm32.processed.version import ContractVersionStatus
from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel, VehicleState
from models.mapping import VehiclePose
from models.objects import ObjectClassification, Point2D, Velocity2D


# --------------------------------------------------------------------------------------------
# builders -- a valid frame by default; each test perturbs exactly one thing
# --------------------------------------------------------------------------------------------

def _metadata(**over) -> STM32FrameMetadata:
    base = dict(frame_id=7, sequence_number=7, timestamp=time.time(), active_object_count=0, source_id="stm32_hardware")
    base.update(over)
    return STM32FrameMetadata(**base)


def _sensor_status(**over) -> STM32SensorStatus:
    base = dict(
        lidar=STM32ChannelHealth(connected=True, ok=True, frames_received=1200, frames_dropped=3),
        radar=STM32ChannelHealth(connected=True, ok=True),
        fusion_active=True,
    )
    base.update(over)
    return STM32SensorStatus(**base)


def _object(track_id: str = "track-1", **over) -> STM32TrackedObject:
    base = dict(
        track_id=track_id,
        object_type=ObjectClassification.VEHICLE_LIKE,
        position=Point2D(x=6.0, y=0.2),
        distance_m=6.0,
        relative_velocity=Velocity2D(vx=-1.8, vy=0.0),
        confidence=0.86,
        ttc_s=2.8,
        risk=RiskLevel.WARNING,
        source=STM32SourceChannel.FUSED,
        in_projected_path=True,
        radar_range_m=6.1,
        radar_target_id="r-42",
    )
    base.update(over)
    return STM32TrackedObject(**base)


def _clearance(**over) -> STM32Clearance:
    base = dict(
        front_m=1.2, rear_m=4.0, left_m=2.1, right_m=2.4,
        min_clearance_m=1.2, min_direction=ClearanceDirection.FRONT,
        status=ClearanceState.CAUTION, corridor_width_m=6.9,
    )
    base.update(over)
    return STM32Clearance(**base)


def _risk(**over) -> STM32Risk:
    base = dict(overall_risk=RiskLevel.SAFE, reason=["nominal"])
    base.update(over)
    return STM32Risk(**base)


def _system_status(**over) -> STM32SystemStatus:
    base = dict(stm32_ok=True, processing_load=0.42, processing_time_ms=7.5, can_tx_ok=True)
    base.update(over)
    return STM32SystemStatus(**base)


def _frame(objects: list[STM32TrackedObject] | None = None, *, metadata=None, clearance=..., risk=None,
           sensor_status=None, vehicle_state=..., system_status=None) -> STM32ProcessedFrame:
    objs = [] if objects is None else objects
    meta = metadata if metadata is not None else _metadata(active_object_count=len(objs))
    return STM32ProcessedFrame(
        metadata=meta,
        sensor_status=sensor_status or _sensor_status(),
        vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0) if vehicle_state is ... else vehicle_state,
        objects=objs,
        clearance=_clearance() if clearance is ... else clearance,
        risk=risk or _risk(),
        system_status=system_status or _system_status(),
    )


CODEC = JsonProcessedFrameCodec()


def _roundtrip_dict(frame: STM32ProcessedFrame) -> dict:
    return frame.model_dump(mode="json")


# --------------------------------------------------------------------------------------------
# 1-4  valid frames of varying object counts
# --------------------------------------------------------------------------------------------

class TestValidFrames:
    def test_1_valid_frame(self):
        frame = _frame(
            objects=[_object("track-1"), _object("track-2", risk=RiskLevel.CRITICAL, ttc_s=1.1)],
            risk=_risk(overall_risk=RiskLevel.CRITICAL, most_critical_track_id="track-2", collision_predicted=True,
                       predicted_collision_time_s=1.1),
        )
        validate_processed_frame(frame)  # must not raise
        assert CODEC.decode(CODEC.encode(frame)) == frame

    def test_2_empty_object_frame(self):
        frame = _frame(objects=[], risk=_risk(overall_risk=RiskLevel.SAFE))
        validate_processed_frame(frame)
        assert frame.metadata.active_object_count == 0
        assert CODEC.decode(CODEC.encode(frame)) == frame

    def test_3_single_object_frame(self):
        frame = _frame(objects=[_object("track-1")])
        validate_processed_frame(frame)
        assert frame.metadata.active_object_count == 1

    def test_4_multiple_object_frame(self):
        objs = [_object(f"track-{i}") for i in range(5)]
        frame = _frame(objects=objs)
        validate_processed_frame(frame)
        assert len(frame.objects) == 5
        assert {o.track_id for o in frame.objects} == {f"track-{i}" for i in range(5)}

    def test_optional_sections_may_be_absent(self):
        frame = _frame(objects=[_object("track-1", in_projected_path=None, relative_velocity=None)],
                       clearance=None, vehicle_state=None,
                       sensor_status=_sensor_status(radar=None, fusion_active=None))
        validate_processed_frame(frame)
        assert frame.clearance is None and frame.vehicle_state is None
        assert frame.sensor_status.radar is None
        assert CODEC.decode(CODEC.encode(frame)) == frame


# --------------------------------------------------------------------------------------------
# 5  invalid timestamp
# --------------------------------------------------------------------------------------------

class TestInvalidTimestamp:
    def test_zero_timestamp_rejected(self):
        frame = _frame(metadata=_metadata(timestamp=0.0))
        with pytest.raises(STM32ProcessedFrameError, match="timestamp"):
            validate_processed_frame(frame)

    def test_negative_timestamp_rejected(self):
        frame = _frame(metadata=_metadata(timestamp=-1.0))
        with pytest.raises(STM32ProcessedFrameError, match="timestamp"):
            validate_processed_frame(frame)

    def test_far_future_timestamp_rejected(self):
        frame = _frame(metadata=_metadata(timestamp=time.time() + 3600))
        with pytest.raises(STM32ProcessedFrameError, match="future"):
            validate_processed_frame(frame)

    def test_nan_timestamp_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["timestamp"] = float("nan")
        with pytest.raises(STM32ProcessedFrameError, match="finite"):
            parse_processed_frame(payload)

    def test_non_numeric_timestamp_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["timestamp"] = "soon"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)


# --------------------------------------------------------------------------------------------
# 6  invalid sequence / frame id / count
# --------------------------------------------------------------------------------------------

class TestInvalidSequence:
    def test_negative_sequence_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["sequence_number"] = -1
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_negative_frame_id_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["frame_id"] = -3
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_active_object_count_mismatch_rejected(self):
        frame = _frame(objects=[_object("track-1")], metadata=_metadata(active_object_count=4))
        with pytest.raises(STM32ProcessedFrameError, match="active_object_count"):
            validate_processed_frame(frame)

    def test_non_numeric_sequence_rejected(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["sequence_number"] = "seven"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)


# --------------------------------------------------------------------------------------------
# 7  invalid object data
# --------------------------------------------------------------------------------------------

class TestInvalidObjectData:
    def test_confidence_out_of_range_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["confidence"] = 1.5
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_negative_distance_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["distance_m"] = -2.0
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_empty_track_id_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["track_id"] = ""
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_duplicate_track_id_rejected(self):
        frame = _frame(objects=[_object("dup"), _object("dup")])
        with pytest.raises(STM32ProcessedFrameError, match="duplicate track_id"):
            validate_processed_frame(frame)

    def test_unknown_classification_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["object_type"] = "spaceship"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_non_finite_position_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["position"]["x"] = float("inf")
        with pytest.raises(STM32ProcessedFrameError, match="finite"):
            parse_processed_frame(payload)

    def test_absurd_distance_rejected(self):
        frame = _frame(objects=[_object("track-1", distance_m=50_000.0)])
        with pytest.raises(STM32ProcessedFrameError, match="distance_m"):
            validate_processed_frame(frame)


# --------------------------------------------------------------------------------------------
# 8  invalid TTC
# --------------------------------------------------------------------------------------------

class TestInvalidTTC:
    def test_negative_ttc_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["ttc_s"] = -0.5
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_nan_ttc_rejected_via_codec(self):
        # NaN fails the model's own `ge=0.0` bound (nan >= 0 is False); inf is caught by the
        # finiteness check in validate_processed_frame. Either way -> STM32ProcessedFrameError.
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["ttc_s"] = float("nan")
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_inf_ttc_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["ttc_s"] = float("inf")
        with pytest.raises(STM32ProcessedFrameError, match="finite"):
            parse_processed_frame(payload)

    def test_absurd_ttc_rejected(self):
        frame = _frame(objects=[_object("track-1", ttc_s=100_000.0)])
        with pytest.raises(STM32ProcessedFrameError, match="ttc_s"):
            validate_processed_frame(frame)

    def test_zero_ttc_is_valid_overlap(self):
        frame = _frame(objects=[_object("track-1", ttc_s=0.0, risk=RiskLevel.CRITICAL)],
                       risk=_risk(overall_risk=RiskLevel.CRITICAL, most_critical_track_id="track-1"))
        validate_processed_frame(frame)  # 0.0 == "already overlapping", allowed

    def test_none_ttc_is_valid_not_approaching(self):
        frame = _frame(objects=[_object("track-1", ttc_s=None)])
        validate_processed_frame(frame)


# --------------------------------------------------------------------------------------------
# 9  invalid clearance
# --------------------------------------------------------------------------------------------

class TestInvalidClearance:
    def test_min_clearance_disagrees_with_directions(self):
        frame = _frame(clearance=_clearance(front_m=1.2, rear_m=4.0, left_m=2.1, right_m=2.4, min_clearance_m=0.1))
        with pytest.raises(STM32ProcessedFrameError, match="min_clearance_m"):
            validate_processed_frame(frame)

    def test_min_direction_inconsistent(self):
        frame = _frame(clearance=_clearance(front_m=1.2, rear_m=4.0, left_m=2.1, right_m=2.4,
                                            min_clearance_m=1.2, min_direction=ClearanceDirection.REAR))
        with pytest.raises(STM32ProcessedFrameError, match="min_direction"):
            validate_processed_frame(frame)

    def test_negative_direction_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["clearance"]["left_m"] = -0.3
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_unknown_status_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["clearance"]["status"] = "doomed"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_non_finite_clearance_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["clearance"]["front_m"] = float("inf")
        with pytest.raises(STM32ProcessedFrameError, match="finite"):
            parse_processed_frame(payload)


# --------------------------------------------------------------------------------------------
# 10  invalid risk state
# --------------------------------------------------------------------------------------------

class TestInvalidRiskState:
    def test_unknown_overall_risk_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame())
        payload["risk"]["overall_risk"] = "extreme"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_unknown_object_risk_rejected_via_codec(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["risk"] = "meltdown"
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_dangling_most_critical_track_id_rejected(self):
        frame = _frame(objects=[_object("track-1")], risk=_risk(most_critical_track_id="ghost"))
        with pytest.raises(STM32ProcessedFrameError, match="most_critical_track_id"):
            validate_processed_frame(frame)

    def test_collision_predicted_without_time_rejected(self):
        frame = _frame(risk=_risk(collision_predicted=True, predicted_collision_time_s=None))
        with pytest.raises(STM32ProcessedFrameError, match="predicted_collision_time_s"):
            validate_processed_frame(frame)


# --------------------------------------------------------------------------------------------
# 11  unknown protocol version
# --------------------------------------------------------------------------------------------

class TestProtocolVersion:
    def test_current_version_is_supported(self):
        assert classify_contract_version(PROCESSED_CONTRACT_VERSION) is ContractVersionStatus.OK

    def test_unknown_major_rejected(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["protocol_version"] = "2.0.0"
        with pytest.raises(STM32ContractVersionError, match="major version"):
            parse_processed_frame(payload)

    def test_older_major_rejected(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["protocol_version"] = "0.9.0"
        with pytest.raises(STM32ContractVersionError):
            parse_processed_frame(payload)

    def test_unparseable_version_rejected(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["protocol_version"] = "banana"
        with pytest.raises(STM32ContractVersionError, match="semantic version"):
            parse_processed_frame(payload)

    def test_contract_version_error_is_a_processed_frame_error(self):
        assert issubclass(STM32ContractVersionError, STM32ProcessedFrameError)

    def test_newer_minor_same_major_is_accepted(self):
        payload = _roundtrip_dict(_frame())
        payload["metadata"]["protocol_version"] = "1.99.0"
        payload["metadata"]["a_field_from_the_future"] = {"nested": 1}  # extra="ignore" tolerates it
        frame = parse_processed_frame(payload)
        assert frame.metadata.protocol_version == "1.99.0"


# --------------------------------------------------------------------------------------------
# 12  missing required field
# --------------------------------------------------------------------------------------------

class TestMissingRequiredField:
    def test_missing_metadata_section(self):
        payload = _roundtrip_dict(_frame())
        del payload["metadata"]
        with pytest.raises(STM32ProcessedFrameError, match="metadata.protocol_version"):
            parse_processed_frame(payload)

    def test_missing_protocol_version(self):
        payload = _roundtrip_dict(_frame())
        del payload["metadata"]["protocol_version"]
        with pytest.raises(STM32ProcessedFrameError, match="protocol_version"):
            parse_processed_frame(payload)

    def test_missing_sensor_status(self):
        payload = _roundtrip_dict(_frame())
        del payload["sensor_status"]
        with pytest.raises(STM32ProcessedFrameError, match="schema validation"):
            parse_processed_frame(payload)

    def test_missing_risk_section(self):
        payload = _roundtrip_dict(_frame())
        del payload["risk"]
        with pytest.raises(STM32ProcessedFrameError, match="schema validation"):
            parse_processed_frame(payload)

    def test_missing_metadata_timestamp(self):
        payload = _roundtrip_dict(_frame())
        del payload["metadata"]["timestamp"]
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_missing_object_position(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        del payload["objects"][0]["position"]
        with pytest.raises(STM32ProcessedFrameError):
            parse_processed_frame(payload)

    def test_non_object_payload_rejected(self):
        with pytest.raises(STM32ProcessedFrameError, match="JSON object"):
            parse_processed_frame("[1, 2, 3]")

    def test_not_json_rejected(self):
        with pytest.raises(STM32ProcessedFrameError, match="not valid JSON"):
            parse_processed_frame("<not json>")


# --------------------------------------------------------------------------------------------
# 13  serialization / deserialization round trip
# --------------------------------------------------------------------------------------------

class TestRoundTrip:
    def _frames(self) -> list[STM32ProcessedFrame]:
        return [
            _frame(objects=[]),
            _frame(objects=[_object("track-1")]),
            _frame(
                objects=[_object("track-1"), _object("track-2", source=STM32SourceChannel.RADAR,
                                                     object_type=ObjectClassification.UNKNOWN)],
                risk=_risk(overall_risk=RiskLevel.WARNING, most_critical_track_id="track-1"),
            ),
            _frame(objects=[_object("track-1", relative_velocity=None, ttc_s=None, in_projected_path=None)],
                   clearance=None, vehicle_state=None,
                   sensor_status=_sensor_status(radar=None, fusion_active=None)),
        ]

    def test_bytes_round_trip_equal(self):
        for frame in self._frames():
            assert CODEC.decode(CODEC.encode(frame)) == frame

    def test_dict_round_trip_equal(self):
        for frame in self._frames():
            assert parse_processed_frame(frame.model_dump(mode="json")) == frame

    def test_json_string_round_trip_equal(self):
        for frame in self._frames():
            assert parse_processed_frame(frame.model_dump_json()) == frame

    def test_encode_returns_bytes(self):
        assert isinstance(CODEC.encode(_frame()), bytes)

    def test_decode_runs_validation_by_default(self):
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["objects"][0]["ttc_s"] = -9.0  # semantically invalid
        with pytest.raises(STM32ProcessedFrameError):
            CODEC.decode(_json_bytes(payload))

    def test_decode_can_skip_semantic_validation(self):
        # active_object_count mismatch is a SEMANTIC check -> skipped with validate=False,
        # but structural parsing + version gating still run.
        payload = _roundtrip_dict(_frame(objects=[_object("track-1")]))
        payload["metadata"]["active_object_count"] = 9
        frame = CODEC.decode(_json_bytes(payload), validate=False)
        assert frame.metadata.active_object_count == 9
        with pytest.raises(STM32ProcessedFrameError):
            CODEC.decode(_json_bytes(payload))  # default: validate=True

    def test_encode_rejects_wrong_type(self):
        with pytest.raises(STM32ProcessedFrameError):
            CODEC.encode({"not": "a frame"})  # type: ignore[arg-type]


# --------------------------------------------------------------------------------------------
# contract-hygiene sanity
# --------------------------------------------------------------------------------------------

class TestContractHygiene:
    def test_version_is_semver(self):
        parts = PROCESSED_CONTRACT_VERSION.split(".")
        assert len(parts) == 3 and all(p.isdigit() for p in parts)

    def test_hardware_spec_dependent_fields_documented(self):
        assert len(HARDWARE_SPEC_DEPENDENT_FIELDS) >= 1
        # every entry must be a dotted path string
        assert all(isinstance(p, str) and "." in p for p in HARDWARE_SPEC_DEPENDENT_FIELDS)

    def test_reused_enums_are_the_canonical_ones(self):
        # the contract must not fork parallel enums -- it reuses the pipeline's own
        obj = _object("track-1")
        assert isinstance(obj.risk, RiskLevel)
        assert isinstance(obj.object_type, ObjectClassification)
        assert isinstance(_clearance().status, ClearanceState)

    def test_no_raw_points_field_on_contract(self):
        # hardware mode must not re-interpret raw LiDAR -- the contract carries results only
        assert "points" not in STM32ProcessedFrame.model_fields


def _json_bytes(payload: dict) -> bytes:
    import json
    return json.dumps(payload).encode("utf-8")

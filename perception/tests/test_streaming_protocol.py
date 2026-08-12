"""Tests for streaming.protocol: message envelope builders and frame_id classification."""

import math

import pytest

from common.config import Settings
from models.tracking import TrackedScan
from streaming.protocol import (
    PROTOCOL_VERSION,
    FrameIdStatus,
    MessageType,
    build_error_message,
    build_heartbeat_message,
    build_perception_frame_message,
    build_system_status_message,
    classify_frame_id,
    is_frame_id_acceptable,
)

DEFAULT_SETTINGS = Settings(_env_file=None)


def _scan(seq: int = 5) -> TrackedScan:
    return TrackedScan(
        scan_id="scan-1", sequence_number=seq, source_id="unit-test", timestamp=1000.5,
        objects=[], noise_points=[], object_count=0, noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


class TestEnvelopeCommonFields:
    def test_perception_frame_envelope_shape(self):
        message = build_perception_frame_message(_scan(), settings=DEFAULT_SETTINGS)
        assert message["protocol_version"] == PROTOCOL_VERSION
        assert message["message_type"] == MessageType.PERCEPTION_FRAME.value
        assert message["frame_id"] == 5
        assert message["timestamp"] == pytest.approx(1000.5)
        assert "transmission_timestamp" in message
        assert isinstance(message["data"], dict)

    def test_frame_id_matches_scan_sequence_number_not_a_separate_counter(self):
        message = build_perception_frame_message(_scan(seq=42), settings=DEFAULT_SETTINGS)
        assert message["frame_id"] == 42

    def test_data_holds_the_payload_fields(self):
        message = build_perception_frame_message(_scan(), settings=DEFAULT_SETTINGS)
        assert message["data"]["scan_id"] == "scan-1"
        assert message["data"]["source_id"] == "unit-test"
        assert "protocol_version" not in message["data"]  # lives at the envelope level only


class TestHeartbeatMessage:
    def test_shape(self):
        message = build_heartbeat_message(uptime_s=12.345, frames_sent=99, clients_connected=2)
        assert message["message_type"] == MessageType.HEARTBEAT.value
        assert message["frame_id"] is None
        assert message["data"] == {"uptime_s": 12.345, "frames_sent": 99, "clients_connected": 2}

    def test_protocol_version_present(self):
        message = build_heartbeat_message(0.0, 0, 0)
        assert message["protocol_version"] == PROTOCOL_VERSION


class TestSystemStatusMessage:
    def test_shape(self):
        message = build_system_status_message(status="running", source_id="simulated:08", scan_rate_hz=10.0)
        assert message["message_type"] == MessageType.SYSTEM_STATUS.value
        assert message["data"]["status"] == "running"
        assert message["data"]["source_id"] == "simulated:08"
        assert message["data"]["scan_rate_hz"] == 10.0


class TestErrorMessage:
    def test_shape(self):
        message = build_error_message(code="PIPELINE_ERROR", message="something went wrong")
        assert message["message_type"] == MessageType.ERROR.value
        assert message["data"] == {"code": "PIPELINE_ERROR", "message": "something went wrong"}


class TestPointMode:
    def _scan_and_points(self):
        from models.lidar import CartesianPoint

        points = [CartesianPoint(angle=0.0, distance=5.0, timestamp=0.0, x=5.0, y=0.0)]
        return _scan(), points

    def test_none_mode_sends_no_points(self):
        scan, points = self._scan_and_points()
        message = build_perception_frame_message(scan, point_mode="none", raw_points=points, settings=DEFAULT_SETTINGS)
        assert message["data"]["points"] is None

    def test_polar_mode_has_angle_and_distance(self):
        scan, points = self._scan_and_points()
        message = build_perception_frame_message(scan, point_mode="polar", raw_points=points, settings=DEFAULT_SETTINGS)
        entry = message["data"]["points"][0]
        assert entry["angle"] == 0.0
        assert entry["distance"] == 5.0
        assert "x" not in entry

    def test_cartesian_mode_has_x_and_y(self):
        scan, points = self._scan_and_points()
        message = build_perception_frame_message(scan, point_mode="cartesian", raw_points=points, settings=DEFAULT_SETTINGS)
        entry = message["data"]["points"][0]
        assert entry["x"] == pytest.approx(5.0, abs=1e-3)
        assert entry["y"] == pytest.approx(0.0, abs=1e-3)
        assert "angle" not in entry

    def test_both_mode_has_everything(self):
        scan, points = self._scan_and_points()
        message = build_perception_frame_message(scan, point_mode="both", raw_points=points, settings=DEFAULT_SETTINGS)
        entry = message["data"]["points"][0]
        assert entry["x"] == pytest.approx(5.0, abs=1e-3)
        assert entry["angle"] == 0.0
        assert entry["distance"] == 5.0

    def test_cartesian_matches_polar_to_cartesian_formula(self):
        scan, _ = self._scan_and_points()
        from models.lidar import CartesianPoint

        points = [CartesianPoint(angle=37.0, distance=4.0, timestamp=0.0, x=0.0, y=0.0)]
        message = build_perception_frame_message(scan, point_mode="cartesian", raw_points=points, settings=DEFAULT_SETTINGS)
        entry = message["data"]["points"][0]
        expected_x = 4.0 * math.cos(math.radians(37.0))
        expected_y = 4.0 * math.sin(math.radians(37.0))
        assert entry["x"] == pytest.approx(expected_x, abs=1e-3)
        assert entry["y"] == pytest.approx(expected_y, abs=1e-3)


class TestClassifyFrameId:
    def test_first_frame_is_accepted(self):
        assert classify_frame_id(None, 0) == FrameIdStatus.ACCEPT

    def test_immediate_next_is_accepted(self):
        assert classify_frame_id(5, 6) == FrameIdStatus.ACCEPT

    def test_same_id_is_duplicate(self):
        assert classify_frame_id(5, 5) == FrameIdStatus.DUPLICATE

    def test_lower_id_is_out_of_order(self):
        assert classify_frame_id(5, 3) == FrameIdStatus.OUT_OF_ORDER

    def test_skipped_ids_is_gap(self):
        assert classify_frame_id(5, 9) == FrameIdStatus.GAP

    def test_gap_of_exactly_two(self):
        assert classify_frame_id(5, 7) == FrameIdStatus.GAP


class TestIsFrameIdAcceptable:
    @pytest.mark.parametrize("status,expected", [
        (FrameIdStatus.ACCEPT, True),
        (FrameIdStatus.GAP, True),
        (FrameIdStatus.DUPLICATE, False),
        (FrameIdStatus.OUT_OF_ORDER, False),
    ])
    def test_acceptability(self, status, expected):
        assert is_frame_id_acceptable(status) is expected

"""Tests for serialization.unity_protocol: the Python -> Unity frame message builder."""

import base64
import json

import numpy as np
import pytest

from models.clearance import ClearanceAssessment, ClearanceDirection, ClearanceState, DirectionalClearance
from models.collision import CollisionAssessment, CollisionRiskResult, RiskLevel, VehicleState
from models.mapping import CellState, OccupancyGrid, VehiclePose
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.tracking import TrackedScan
from common.config import Settings
from serialization import build_clearance_payload, build_config_payload, build_frame_message, pack_occupancy_grid
from serialization.unity_protocol import build_object_payload, build_risk_payload, build_vehicle_payload


def _object(track_id="track-1", has_velocity=True) -> DetectedObject:
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=5.0, y=1.0), width=1.8, depth=1.8,
        distance=5.1, classification=ObjectClassification.VEHICLE_LIKE, confidence=0.8642,
        velocity=Velocity2D(vx=-1.5, vy=0.0) if has_velocity else None,
        predicted_position=Point2D(x=4.85, y=1.0) if has_velocity else None,
        tracking_state=TrackingState.CONFIRMED, movement_state=MovementState.MOVING if has_velocity else MovementState.UNKNOWN,
        track_age=10, track_hits=10, track_misses=0, timestamp=1000.0,
    )


def _tracked_scan(objects=None) -> TrackedScan:
    objects = objects if objects is not None else [_object()]
    return TrackedScan(
        scan_id="scan-abc", sequence_number=7, source_id="unit-test", timestamp=1000.5,
        objects=objects, noise_points=[], object_count=len(objects), noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _collision_result(track_id="track-1", risk=RiskLevel.WARNING) -> CollisionRiskResult:
    return CollisionRiskResult(
        track_id=track_id, classification=ObjectClassification.VEHICLE_LIKE, distance=5.1,
        relative_position=Point2D(x=5.0, y=1.0), relative_velocity=Velocity2D(vx=-1.5, vy=0.0),
        relative_speed=1.5, in_projected_path=True, ttc=3.2, collision_predicted=True,
        predicted_collision_time=3.1, predicted_collision_position=Point2D(x=0.5, y=0.9),
        risk_level=risk, risk_score=0.4, reason=["Estimated TTC = 3.20s"], timestamp=1000.5,
    )


def _assessment() -> CollisionAssessment:
    result = _collision_result()
    return CollisionAssessment(
        scan_id="scan-abc", sequence_number=7, source_id="unit-test", timestamp=1000.5,
        results=[result], object_count=1, overall_risk=RiskLevel.WARNING, most_critical_object=result,
    )


def _clearance_assessment() -> ClearanceAssessment:
    def _directional(direction, distance, point=None):
        return DirectionalClearance(direction=direction, distance_m=distance, nearest_point=point)

    return ClearanceAssessment(
        scan_id="scan-abc", sequence_number=7, source_id="unit-test", timestamp=1000.5,
        front=_directional(ClearanceDirection.FRONT, 4.0, Point2D(x=4.0, y=0.0)),
        rear=_directional(ClearanceDirection.REAR, 9.0),
        left=_directional(ClearanceDirection.LEFT, 3.0, Point2D(x=0.0, y=3.0)),
        right=_directional(ClearanceDirection.RIGHT, 6.0),
        min_clearance_m=3.0, min_direction=ClearanceDirection.LEFT,
        corridor_width_m=11.4, overall_status=ClearanceState.SAFE, reason=["Closest clearance is 3.00m, left."],
    )


def _grid(width=8, height=6) -> OccupancyGrid:
    cell_states = np.zeros((height, width), dtype=np.int8)
    cell_states[0, 0] = CellState.OCCUPIED
    cell_states[1, 1] = CellState.FREE
    return OccupancyGrid(
        width_cells=width, height_cells=height, resolution_m=0.1, origin_x_m=-4.0, origin_y_m=-3.0,
        max_range_m=12.0, timestamp=1000.5, scan_count=5, cell_states=cell_states, log_odds=np.zeros((height, width)),
    )


class TestBuildObjectPayload:
    def test_includes_expected_fields(self):
        payload = build_object_payload(_object())
        assert payload["track_id"] == "track-1"
        assert payload["classification"] == "vehicle_like"
        assert payload["velocity"] == {"vx": -1.5, "vy": 0.0}
        assert payload["predicted_position"] == {"x": 4.85, "y": 1.0}
        assert payload["tracking_state"] == "confirmed"

    def test_missing_velocity_is_null_not_absent_or_crashing(self):
        payload = build_object_payload(_object(has_velocity=False))
        assert payload["velocity"] is None
        assert payload["predicted_position"] is None

    def test_json_serializable(self):
        payload = build_object_payload(_object())
        json.dumps(payload)  # must not raise

    def test_omits_debugging_only_fields(self):
        payload = build_object_payload(_object())
        assert "shape_features" not in payload
        assert "classification_reason" not in payload


class TestBuildRiskPayload:
    def test_none_assessment_returns_none(self):
        assert build_risk_payload(None) is None

    def test_includes_overall_and_most_critical(self):
        payload = build_risk_payload(_assessment())
        assert payload["overall_risk"] == "warning"
        assert payload["most_critical"]["track_id"] == "track-1"
        assert len(payload["results"]) == 1

    def test_no_most_critical_when_no_results(self):
        empty = CollisionAssessment(
            scan_id="s", sequence_number=0, source_id="t", timestamp=0.0,
            results=[], object_count=0, overall_risk=RiskLevel.SAFE, most_critical_object=None,
        )
        payload = build_risk_payload(empty)
        assert payload["most_critical"] is None
        assert payload["results"] == []

    def test_json_serializable(self):
        json.dumps(build_risk_payload(_assessment()))


class TestBuildClearancePayload:
    def test_none_assessment_returns_none(self):
        assert build_clearance_payload(None) is None

    def test_includes_all_four_directions_and_summary(self):
        payload = build_clearance_payload(_clearance_assessment())
        assert payload["front"]["distance_m"] == 4.0
        assert payload["front"]["nearest_point"] == {"x": 4.0, "y": 0.0}
        assert payload["rear"]["nearest_point"] is None
        assert payload["min_direction"] == "left"
        assert payload["min_clearance_m"] == 3.0
        assert payload["corridor_width_m"] == 11.4
        assert payload["overall_status"] == "safe"

    def test_json_serializable(self):
        json.dumps(build_clearance_payload(_clearance_assessment()))


class TestPackOccupancyGrid:
    def test_none_grid_returns_none(self):
        assert pack_occupancy_grid(None) is None

    def test_full_resolution_round_trips(self):
        grid = _grid()
        packed = pack_occupancy_grid(grid, downsample=1)
        assert packed["width_cells"] == grid.width_cells
        assert packed["height_cells"] == grid.height_cells
        decoded = np.frombuffer(base64.b64decode(packed["cells_base64"]), dtype=np.uint8).reshape(grid.height_cells, grid.width_cells)
        assert np.array_equal(decoded, grid.cell_states)

    def test_downsampled_grid_is_smaller(self):
        grid = _grid(width=8, height=8)
        packed = pack_occupancy_grid(grid, downsample=2)
        assert packed["width_cells"] == 4
        assert packed["height_cells"] == 4
        assert packed["resolution_m"] == pytest.approx(0.2)

    def test_downsample_zero_or_negative_treated_as_one(self):
        grid = _grid()
        packed = pack_occupancy_grid(grid, downsample=0)
        assert packed["width_cells"] == grid.width_cells

    def test_json_serializable(self):
        json.dumps(pack_occupancy_grid(_grid(), downsample=1))


class TestBuildVehiclePayload:
    def test_none_defaults_to_identity(self):
        payload = build_vehicle_payload(None)
        assert payload == {"x": 0.0, "y": 0.0, "heading": 0.0, "speed_mps": 0.0}

    def test_explicit_state(self):
        state = VehicleState(pose=VehiclePose(x=1.0, y=2.0, heading=90.0), speed_mps=3.0)
        payload = build_vehicle_payload(state)
        assert payload == {"x": 1.0, "y": 2.0, "heading": 90.0, "speed_mps": 3.0}


class TestBuildConfigPayload:
    def test_reflects_settings_not_hard_coded(self):
        settings = Settings(_env_file=None, collision_critical_distance_m=1.23, vehicle_length_m=9.9)
        payload = build_config_payload(settings)
        assert payload["collision_critical_distance_m"] == pytest.approx(1.23)
        assert payload["vehicle_length_m"] == pytest.approx(9.9)

    def test_defaults_to_get_settings_when_none(self):
        payload = build_config_payload(None)
        assert "vehicle_length_m" in payload
        assert "collision_warning_ttc_s" in payload

    def test_json_serializable(self):
        json.dumps(build_config_payload(Settings(_env_file=None)))


class TestBuildFrameMessage:
    def test_minimal_message_has_no_risk_map_or_points(self):
        message = build_frame_message(_tracked_scan())
        assert message["risk"] is None
        assert message["map"] is None
        assert message["points"] is None
        assert "protocol_version" not in message  # lives on the envelope (streaming.protocol) since Phase 12, not this payload

    def test_clearance_defaults_to_none_when_not_supplied(self):
        # Phase 10 (clearance engine) is implemented (see perception/src/clearance/), but
        # build_frame_message still defaults to None for a caller that doesn't pass an assessment
        # -- e.g. an older/minimal script not wired up to that stage. See TestBuildClearancePayload
        # below for the real-data path.
        message = build_frame_message(_tracked_scan())
        assert message["clearance"] is None

    def test_clearance_populated_when_assessment_supplied(self):
        message = build_frame_message(_tracked_scan(), clearance_assessment=_clearance_assessment())
        assert message["clearance"]["overall_status"] == "safe"
        assert message["clearance"]["front"]["direction"] == "front"

    def test_config_always_present(self):
        message = build_frame_message(_tracked_scan())
        assert "vehicle_length_m" in message["config"]
        assert "collision_critical_distance_m" in message["config"]

    def test_full_message_includes_everything_requested(self):
        message = build_frame_message(
            _tracked_scan(), collision_assessment=_assessment(), occupancy_grid=_grid(),
            vehicle_state=VehicleState(speed_mps=2.0), include_map=True, include_points=False,
        )
        assert message["risk"]["overall_risk"] == "warning"
        assert message["map"] is not None
        assert message["vehicle"]["speed_mps"] == 2.0

    def test_scan_identity_fields_passthrough(self):
        message = build_frame_message(_tracked_scan())
        assert message["scan_id"] == "scan-abc"
        assert message["sequence_number"] == 7
        assert message["source_id"] == "unit-test"

    def test_empty_objects_list_is_valid(self):
        message = build_frame_message(_tracked_scan(objects=[]))
        assert message["objects"] == []

    def test_full_message_is_json_serializable(self):
        message = build_frame_message(
            _tracked_scan(), collision_assessment=_assessment(), occupancy_grid=_grid(),
            vehicle_state=VehicleState(speed_mps=2.0), include_map=True,
        )
        encoded = json.dumps(message)
        decoded = json.loads(encoded)
        assert decoded["scan_id"] == "scan-abc"

    def test_include_points_true_with_no_points_given_is_none(self):
        message = build_frame_message(_tracked_scan(), include_points=True, raw_points=None)
        assert message["points"] is None

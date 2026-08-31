"""Phase 4 -- Dashboard/Unity state consistency (§17, §19).

The Dashboard and Unity are independent TCP clients of the SAME
`streaming.PerceptionStreamServer.publish()` output. Neither runs any perception. So "Dashboard
track_id == Unity track_id", "... object type ...", "... distance ...", "... TTC ...", "... risk
...", "... clearance ..." all hold **by construction**: there is exactly one wire message, and
this test asserts that message carries ONE self-consistent value per field across the three
places a consumer might read it -- `data.objects[]` (raw), `data.tracked_objects[]` (Edge
joined view, what the dashboard's Detected/Tracking panels + Unity's in-world labels use), and
`data.risk.results[]` (per-object risk).

Exercised on the hardware (ESP32) path, which is the Phase-4 focus; the simulation path is
covered the same way by `simulator/tests/test_phase4_dashboard_livestate.py`.
"""

from __future__ import annotations

import time

from common.config import Settings
from datasources.esp32 import (
    ESP32Source,
    MockESP32Transport,
    ProcessedFrameToLiveState,
    build_scripted_approaching_vehicle_frames,
)
from streaming.protocol import build_perception_frame_message

BASE_TS = 1_786_602_600.0


def _wire_data(distance_m: float = 6.0):
    settings = Settings(data_source="hardware", esp32_transport="mock")
    tr = MockESP32Transport()
    clk = [BASE_TS]
    src = ESP32Source(settings, transport=tr, now_fn=lambda: clk[0])
    src.connect()
    [frame] = build_scripted_approaching_vehicle_frames(1, start_distance_m=distance_m, start_timestamp=BASE_TS)
    tr.push_frame(frame)
    clk[0] += 0.1
    f = src.poll()
    adapter = ProcessedFrameToLiveState(settings, session_id="hw-consistency")
    adapted = adapter.build(f)
    msg = build_perception_frame_message(
        adapted.tracked_scan, collision_assessment=adapted.collision_assessment,
        vehicle_state=adapted.vehicle_state, clearance_assessment=adapted.clearance_assessment,
        settings=settings, session_id=adapter.session_id, live_state=adapted.live_state,
    )
    return msg["data"], frame


class TestWireConsistency:
    def test_one_value_per_field_across_objects_tracked_and_results(self):
        data, src_frame = _wire_data(distance_m=3.0)  # close -> critical, non-null TTC
        obj = data["objects"][0]
        tracked = data["tracked_objects"][0]
        result = data["risk"]["results"][0]

        # identity
        assert obj["track_id"] == tracked["track_id"] == result["track_id"] == "stm32-track-1"
        # object type
        assert obj["classification"] == tracked["classification"] == result["classification"]
        # distance
        assert obj["distance"] == tracked["distance"] == result["distance"] == src_frame.objects[0].distance_m
        # TTC
        assert tracked["ttc"] == result["ttc"] == src_frame.objects[0].ttc_s
        # risk
        assert tracked["risk"] == result["risk_level"] == data["risk"]["overall_risk"]
        assert data["risk"]["overall_risk"] == src_frame.risk.overall_risk.value
        # most-critical points at the same track
        assert data["risk"]["most_critical"]["track_id"] == tracked["track_id"]

    def test_clearance_is_a_single_shared_object(self):
        data, src_frame = _wire_data()
        c = data["clearance"]
        assert c["front"]["distance_m"] == src_frame.clearance.front_m
        assert c["rear"]["distance_m"] == src_frame.clearance.rear_m
        assert c["left"]["distance_m"] == src_frame.clearance.left_m
        assert c["right"]["distance_m"] == src_frame.clearance.right_m
        assert c["min_clearance_m"] == src_frame.clearance.min_clearance_m
        assert c["min_direction"] == src_frame.clearance.min_direction.value
        assert c["overall_status"] == src_frame.clearance.status.value

    def test_sequence_frame_id_and_source_are_consistent(self):
        data, src_frame = _wire_data()
        assert data["sequence_number"] == src_frame.metadata.frame_id
        assert data["source_id"] == src_frame.metadata.source_id == "stm32_hardware"
        assert data["config"]["data_source"] == "hardware"

    def test_no_geometry_is_fabricated_for_a_hardware_object(self):
        data, _ = _wire_data()
        obj = data["objects"][0]
        # the STM32 does not transmit width/depth/shape -> the wire carries 0.0 / null, not a guess
        assert obj["width"] == 0.0 and obj["depth"] == 0.0
        assert obj["point_count"] is None and obj["aspect_ratio"] is None

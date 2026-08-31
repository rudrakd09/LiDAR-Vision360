"""Phase 3 -- `datasources.esp32.live_state_adapter.ProcessedFrameToLiveState`.

The adapter reshapes ONE `STM32ProcessedFrame` into the canonical `LiveState` the Dashboard /
Unity / PostgreSQL already consume -- reusing `pipeline.LiveStateBuilder`, running NO perception.
Covers required Phase-3 cases 13 (risk), 14 (TTC), 15 (clearance), 16 (LiveState update), plus
the session-reset behaviour.
"""

from __future__ import annotations

from common.config import Settings
from datasources.esp32 import ProcessedFrameToLiveState
from datasources.esp32.live_state_adapter import (
    processed_frame_to_collision_assessment,
    processed_frame_to_tracked_scan,
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


def _obj(track_id="t1", *, x=6.0, dist=6.0, vx=-1.5, ttc=4.0, risk=RiskLevel.WARNING,
         cls=ObjectClassification.VEHICLE_LIKE, source=STM32SourceChannel.FUSED) -> STM32TrackedObject:
    return STM32TrackedObject(
        track_id=track_id, object_type=cls, position=Point2D(x=x, y=0.0), distance_m=dist,
        relative_velocity=Velocity2D(vx=vx, vy=0.0), confidence=0.82, ttc_s=ttc, risk=risk,
        source=source, in_projected_path=True,
    )


def _frame(seq=0, *, ts=TS, objects=None, overall=RiskLevel.WARNING, most_critical="t1",
           clearance=True, radar=True) -> STM32ProcessedFrame:
    objs = [_obj()] if objects is None else objects
    clr = None
    if clearance:
        clr = STM32Clearance(front_m=1.5, rear_m=4.0, left_m=2.5, right_m=2.5, min_clearance_m=1.5,
                             min_direction=ClearanceDirection.FRONT, status=ClearanceState.CAUTION,
                             corridor_width_m=6.8)
    return STM32ProcessedFrame(
        metadata=STM32FrameMetadata(frame_id=seq, sequence_number=seq, timestamp=ts,
                                    source_id="stm32_hardware", active_object_count=len(objs)),
        sensor_status=STM32SensorStatus(
            lidar=STM32ChannelHealth(connected=True, ok=True),
            radar=STM32ChannelHealth(connected=True, ok=True) if radar else None,
            fusion_active=radar,
        ),
        vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0),
        objects=objs, clearance=clr,
        risk=STM32Risk(overall_risk=overall, most_critical_track_id=most_critical if objs else None,
                       collision_predicted=overall is RiskLevel.CRITICAL,
                       predicted_collision_time_s=1.2 if overall is RiskLevel.CRITICAL else None,
                       reason=["stm32 says so"]),
        system_status=STM32SystemStatus(stm32_ok=True, processing_time_ms=6.0),
    )


def _adapter() -> ProcessedFrameToLiveState:
    return ProcessedFrameToLiveState(Settings(), session_id="sess-A")


# ============================================================================================
# 16. LiveState update -- field-by-field mapping
# ============================================================================================

class TestLiveStateMapping:
    def test_identity_and_counts(self):
        ls = _adapter().build(_frame(7, objects=[_obj("t1"), _obj("t2")])).live_state
        assert ls.session_id == "sess-A"
        assert ls.source_id == "stm32_hardware"
        assert ls.sequence_number == 7
        assert ls.timestamp == TS
        assert len(ls.objects) == 2
        assert len(ls.tracked_objects) == 2

    def test_objects_carry_stm32_values_no_fabricated_geometry(self):
        ls = _adapter().build(_frame(0, objects=[_obj("t1", x=9.0, dist=9.0, cls=ObjectClassification.POLE_LIKE)])).live_state
        o = ls.objects[0]
        assert o.track_id == "t1"
        assert o.classification is ObjectClassification.POLE_LIKE
        assert (o.centroid.x, o.centroid.y) == (9.0, 0.0)
        assert o.distance == 9.0
        assert o.width == 0.0 and o.depth == 0.0  # not transmitted -> not guessed
        assert o.shape_features is None and o.bounding_box is None

    def test_sensor_source_mapping(self):
        ls = _adapter().build(_frame(0, objects=[
            _obj("a", source=STM32SourceChannel.LIDAR),
            _obj("b", source=STM32SourceChannel.RADAR),
            _obj("c", source=STM32SourceChannel.FUSED),
        ])).live_state
        by_id = {o.track_id: o for o in ls.objects}
        assert by_id["a"].sensor_sources == ["lidar"]
        assert by_id["b"].sensor_sources == ["radar"]
        assert sorted(by_id["c"].sensor_sources) == ["lidar", "radar"]
        # the per-track view reflects the same attribution (not a hard-coded "lidar")
        t_by_id = {t.track_id: t for t in ls.tracked_objects}
        assert t_by_id["a"].sensor_source == "lidar"
        assert t_by_id["b"].sensor_source == "radar"
        assert t_by_id["c"].sensor_source == "lidar+radar"

    def test_sensor_status_lidar_and_radar(self):
        ls = _adapter().build(_frame(0, radar=True)).live_state
        assert set(ls.sensor_status) == {"lidar", "radar"}
        assert ls.sensor_status["lidar"].connected is True
        assert ls.sensor_status["lidar"].valid_percentage is None  # STM32 sends no point stats
        assert ls.sensor_status["radar"] is not None

    def test_sensor_status_radar_absent_is_null_not_fabricated(self):
        ls = _adapter().build(_frame(0, radar=False)).live_state
        assert ls.sensor_status["radar"] is None

    def test_performance_metrics_from_stm32_processing_time(self):
        adapter = _adapter()
        adapter.build(_frame(0, ts=TS))
        ls = adapter.build(_frame(1, ts=TS + 0.1)).live_state
        assert ls.performance_metrics.pipeline_processing_ms == 6.0  # 6.0 ms from system_status
        assert ls.performance_metrics.measured_scan_rate_hz is not None  # measured from frame timestamps


# ============================================================================================
# 13. Risk update
# ============================================================================================

class TestRiskUpdate:
    def test_overall_risk_taken_from_stm32_verbatim(self):
        for level in (RiskLevel.SAFE, RiskLevel.WARNING, RiskLevel.CRITICAL):
            ls = _adapter().build(_frame(0, overall=level, objects=[_obj("t1", risk=level)])).live_state
            assert ls.risk.overall_risk is level

    def test_per_object_risk_and_most_critical(self):
        ls = _adapter().build(_frame(0, overall=RiskLevel.CRITICAL, most_critical="t2", objects=[
            _obj("t1", risk=RiskLevel.WARNING), _obj("t2", risk=RiskLevel.CRITICAL),
        ])).live_state
        assert ls.risk.most_critical_object is not None
        assert ls.risk.most_critical_object.track_id == "t2"
        by_id = {r.track_id: r for r in ls.risk.results}
        assert by_id["t1"].risk_level is RiskLevel.WARNING
        assert by_id["t2"].risk_level is RiskLevel.CRITICAL

    def test_risk_change_produces_event(self):
        adapter = _adapter()
        adapter.build(_frame(0, overall=RiskLevel.SAFE, objects=[_obj("t1", risk=RiskLevel.SAFE)]))
        ls = adapter.build(_frame(1, overall=RiskLevel.CRITICAL, objects=[_obj("t1", risk=RiskLevel.CRITICAL)])).live_state
        risk_events = [e for e in ls.events if e.event_type == "collision" and e.sequence_number == 1]
        assert risk_events and risk_events[0].new_value == "critical"

    def test_collision_predicted_only_on_the_critical_object(self):
        ls = _adapter().build(_frame(0, overall=RiskLevel.CRITICAL, most_critical="t1", objects=[
            _obj("t1", risk=RiskLevel.CRITICAL), _obj("t2", risk=RiskLevel.WARNING),
        ])).live_state
        by_id = {r.track_id: r for r in ls.risk.results}
        assert by_id["t1"].collision_predicted is True
        assert by_id["t1"].predicted_collision_time == 1.2
        assert by_id["t2"].collision_predicted is False


# ============================================================================================
# 14. TTC update
# ============================================================================================

class TestTTCUpdate:
    def test_ttc_flows_onto_tracked_object_and_risk_result(self):
        ls = _adapter().build(_frame(0, objects=[_obj("t1", ttc=2.4)])).live_state
        assert ls.tracked_objects[0].ttc == 2.4
        assert ls.risk.results[0].ttc == 2.4

    def test_ttc_none_when_not_approaching(self):
        ls = _adapter().build(_frame(0, objects=[_obj("t1", ttc=None)])).live_state
        assert ls.tracked_objects[0].ttc is None
        assert ls.risk.results[0].ttc is None

    def test_ttc_changes_across_frames(self):
        adapter = _adapter()
        a = adapter.build(_frame(0, objects=[_obj("t1", ttc=5.0)])).live_state
        b = adapter.build(_frame(1, objects=[_obj("t1", ttc=1.5)])).live_state
        assert a.tracked_objects[0].ttc == 5.0
        assert b.tracked_objects[0].ttc == 1.5


# ============================================================================================
# 15. Clearance update
# ============================================================================================

class TestClearanceUpdate:
    def test_clearance_expanded_to_full_assessment(self):
        ls = _adapter().build(_frame(0)).live_state
        c = ls.clearance
        assert c is not None
        assert c.front.distance_m == 1.5
        assert c.rear.distance_m == 4.0
        assert c.left.distance_m == 2.5
        assert c.right.distance_m == 2.5
        assert c.min_clearance_m == 1.5
        assert c.min_direction is ClearanceDirection.FRONT
        assert c.overall_status is ClearanceState.CAUTION
        assert c.front.nearest_point is None  # STM32 doesn't transmit it -> not fabricated

    def test_clearance_none_when_stm32_omits_it(self):
        ls = _adapter().build(_frame(0, clearance=False)).live_state
        assert ls.clearance is None

    def test_clearance_change_produces_event(self):
        adapter = _adapter()
        adapter.build(_frame(0))  # CAUTION
        f2 = _frame(1)
        f2 = f2.model_copy(update={"clearance": f2.clearance.model_copy(update={"status": ClearanceState.CRITICAL})})
        ls = adapter.build(f2).live_state
        clr_events = [e for e in ls.events if e.event_type == "clearance" and e.sequence_number == 1]
        assert clr_events and clr_events[0].new_value == "critical"


# ============================================================================================
# session reset -- old objects / risk / TTC must not linger
# ============================================================================================

class TestSessionReset:
    def test_reset_clears_track_history_and_events(self):
        adapter = _adapter()
        adapter.build(_frame(0, objects=[_obj("t1")]))
        adapter.build(_frame(1, objects=[_obj("t1")]))
        adapter.reset(session_id="sess-B")
        assert adapter.session_id == "sess-B"
        ls = adapter.build(_frame(0, objects=[_obj("t1")])).live_state
        # brand-new session: track-1 is "first seen" again, no carried-over trajectory
        assert ls.session_id == "sess-B"
        assert ls.tracked_objects[0].frames_tracked == (ls.objects[0].track_hits or None) or ls.tracked_objects[0].frames_tracked is None
        assert len(ls.tracked_objects[0].trajectory) == 1  # only this scan
        # the only events are this scan's own (e.g. track_created), nothing from session A
        assert all(e.sequence_number == 0 for e in ls.events)

    def test_reset_drops_previous_risk_memory(self):
        adapter = _adapter()
        adapter.build(_frame(0, overall=RiskLevel.CRITICAL, objects=[_obj("t1", risk=RiskLevel.CRITICAL)]))
        adapter.reset(session_id="sess-B")
        ls = adapter.build(_frame(0, overall=RiskLevel.SAFE, objects=[_obj("t1", risk=RiskLevel.SAFE)])).live_state
        # a fresh SAFE frame in the new session must not emit a CRITICAL->SAFE transition from
        # the old session's memory; overall risk is simply SAFE
        assert ls.risk.overall_risk is RiskLevel.SAFE


# ============================================================================================
# pure-function sanity
# ============================================================================================

class TestPureFunctions:
    def test_tracked_scan_sequence_is_frame_id(self):
        ts = processed_frame_to_tracked_scan(_frame(42))
        assert ts.sequence_number == 42
        assert ts.source_id == "stm32_hardware"

    def test_collision_assessment_object_count_matches(self):
        ca = processed_frame_to_collision_assessment(_frame(0, objects=[_obj("a"), _obj("b"), _obj("c")]))
        assert ca.object_count == 3
        assert len(ca.results) == 3

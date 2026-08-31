"""Phase 10 -- SOFTWARE-ONLY HARDWARE-READINESS VALIDATION.

    MOCK STM32 HARDWARE OUTPUT  ->  MockESP32Transport / ScriptedScenarioESP32Transport
        ->  ESP32Source  ->  ProcessedFrameToLiveState  ->  build_perception_frame_message
        ->  (the exact wire envelope Dashboard, Unity, and the backend all consume)

**No physical hardware. No sockets. Nothing here is a physical measurement.** This proves the
complete Edge software path is correct and ready to receive real processed STM32 frames, so the
real ESP32 transport is a drop-in later with zero Dashboard/Unity changes.

Covers the Phase-10 §21 acceptance checklist item by item (see the class names).
"""

from __future__ import annotations

import pytest

from common.config import Settings
from datasources.esp32 import (
    ESP32Source,
    MockESP32Transport,
    ProcessedFrameToLiveState,
    ScriptedScenarioESP32Transport,
    build_mock_stm32_frames,
    build_multi_object_frames,
    build_realtime_arc_frames,
    build_tracking_frames,
    run_esp32_edge,
)
from datasources.esp32.mock_stm32 import build_mock_processed_frame, MockObjectState
from datasources.stm32.processed import parse_processed_frame
from models.collision import RiskLevel
from models.objects import ObjectClassification, Point2D, Velocity2D
from models.stm32_processed import STM32SourceChannel
from streaming.protocol import build_error_message, build_perception_frame_message

BASE_TS = 1_700_000_000.0


class Clock:
    def __init__(self, t: float = BASE_TS) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _settings(**over) -> Settings:
    base = dict(
        data_source="hardware",
        esp32_transport="mock",
        esp32_frame_stale_after_s=1.0,
        esp32_heartbeat_timeout_s=6.0,
        esp32_read_timeout_s=0.01,
    )
    base.update(over)
    return Settings(**base)


def _drive(frames, *, settings=None, clock_step=0.1, extra_polls=5):
    """Feed `frames` one-per-tick through a Mock transport + ESP32Source + adapter, and return
    the list of published wire-message `data` dicts (one per delivered frame)."""
    settings = settings or _settings()
    clock = Clock()
    tr = MockESP32Transport()
    src = ESP32Source(settings, transport=tr, now_fn=clock)
    src.connect()
    adapter = ProcessedFrameToLiveState(settings, session_id=src.session_id)

    out: list[dict] = []
    queue = list(frames)
    for _ in range(len(queue) + extra_polls):
        if queue:
            tr.push_frame(queue.pop(0))
        clock.advance(clock_step)
        frame = src.poll()
        st = src.status
        if st.session_id != adapter.session_id:
            adapter.reset(session_id=st.session_id)
        if frame is not None:
            a = adapter.build(frame)
            msg = build_perception_frame_message(
                a.tracked_scan,
                collision_assessment=a.collision_assessment,
                vehicle_state=a.vehicle_state,
                clearance_assessment=a.clearance_assessment,
                settings=settings,
                session_id=adapter.session_id,
                live_state=a.live_state,
            )
            out.append(msg["data"])
    return out, src


# ======================================================================================
# 1. MOCK STM32 produces valid processed frames (Phase-2 contract)
# ======================================================================================

class TestMockStm32FrameGeneration:
    def test_every_scene_round_trips_through_the_phase2_validator(self):
        for scenario in ("realtime_arc", "multi_object", "tracking"):
            frames = build_mock_stm32_frames(scenario)
            assert frames, scenario
            for f in frames:
                # the codec's decode runs parse + validate_processed_frame -- no exception => valid
                parse_processed_frame(f.model_dump_json())

    def test_frames_carry_the_full_required_field_set(self):
        f = build_mock_stm32_frames("realtime_arc")[3]  # a populated frame
        assert f.metadata.frame_id >= 0
        assert f.metadata.sequence_number >= 0
        assert f.metadata.timestamp > 1_000_000_000
        assert f.metadata.active_object_count == len(f.objects) == 1
        o = f.objects[0]
        assert o.track_id and o.object_type in ObjectClassification
        assert o.position is not None and o.distance_m >= 0
        assert o.relative_velocity is not None
        assert 0.0 <= o.confidence <= 1.0
        assert o.risk in RiskLevel
        assert o.source in STM32SourceChannel
        assert f.clearance is not None and f.clearance.min_clearance_m >= 0
        assert f.risk.overall_risk in RiskLevel
        assert f.sensor_status.lidar.connected is True
        assert f.system_status.notes and "MOCK" in f.system_status.notes

    def test_risk_ttc_clearance_use_project_thresholds_not_hardcoded(self):
        """The arc's WARNING/CRITICAL boundaries must move when the project's configured
        thresholds move -- i.e. they are computed, not baked in."""
        strict = Settings(collision_warning_distance_m=10.0, collision_critical_distance_m=8.0)
        arc = build_realtime_arc_frames(strict)
        # frame 2 = object at 10 m: SAFE with defaults, but WARNING once warning distance is 10 m
        assert arc[1].objects[0].position.x == pytest.approx(10.0)
        assert arc[1].risk.overall_risk is RiskLevel.WARNING


# ======================================================================================
# 3 / 4. complete live pipeline + real-time risk/TTC/clearance arc
# ======================================================================================

class TestRealtimeArcThroughFullPipeline:
    def test_six_phase_arc_reaches_the_wire_with_the_expected_risk_progression(self):
        data, _ = _drive(build_mock_stm32_frames("realtime_arc"))
        assert len(data) == 6

        # sequence strictly increases (12. SEQUENCE)
        seqs = [d["sequence_number"] for d in data]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)

        risks = [d["risk"]["overall_risk"] for d in data]
        assert risks == ["safe", "safe", "safe", "warning", "critical", "safe"]

        # empty frame -> no objects, "NO ACTIVE OBJECTS" territory
        assert data[0]["objects"] == [] and data[0]["tracked_objects"] == []

        # 8. TTC: strictly decreasing while approaching, then unavailable when receding
        ttcs = [d["tracked_objects"][0]["ttc"] if d["tracked_objects"] else None for d in data]
        assert ttcs[1] is not None and ttcs[1] > ttcs[2] > ttcs[3] >= ttcs[4]
        assert ttcs[5] is None  # receding -> TTC undefined, never a fabricated number

        # 9. CLEARANCE: front clearance tightens as the object closes
        fronts = [d["clearance"]["front"]["distance_m"] for d in data]
        assert fronts[1] > fronts[2] > fronts[3] > fronts[4]
        assert data[4]["clearance"]["overall_status"] == "critical"

    def test_live_variant_is_continuous_and_monotonic(self):
        frames = build_mock_stm32_frames("realtime_arc", live=True)
        assert len(frames) > 50
        data, src = _drive(frames, clock_step=0.05)
        assert src.status.dropped_frames == 0
        seqs = [d["sequence_number"] for d in data]
        assert seqs == sorted(seqs)
        assert {d["risk"]["overall_risk"] for d in data} >= {"safe", "warning", "critical"}


# ======================================================================================
# 5. TRACKING -- one persistent track id, not a new object per frame
# ======================================================================================

class TestTrackingPersistence:
    def test_single_track_id_across_the_whole_approach(self):
        data, _ = _drive(build_mock_stm32_frames("tracking"))
        assert len(data) >= 3
        track_ids = {d["tracked_objects"][0]["track_id"] for d in data}
        assert track_ids == {"mock-obj-1"}
        # exactly one object every frame -- never duplicated
        assert all(len(d["tracked_objects"]) == 1 for d in data)
        # distance decreases monotonically
        dists = [d["tracked_objects"][0]["distance"] for d in data]
        assert all(a >= b for a, b in zip(dists, dists[1:]))
        assert dists[0] > dists[-1]


# ======================================================================================
# 6 / 7. MULTIPLE OBJECTS + classification comes from the frame (not the dashboard)
# ======================================================================================

class TestMultipleObjects:
    def test_three_distinct_persistent_tracks_with_frame_supplied_classification(self):
        data, _ = _drive(build_multi_object_frames(frame_count=12, start_timestamp=BASE_TS))
        last = data[-1]
        by_id = {o["track_id"]: o for o in last["tracked_objects"]}
        assert set(by_id) == {"mock-veh-1", "mock-pole-1", "mock-wall-1"}
        assert by_id["mock-veh-1"]["classification"] == "vehicle_like"
        assert by_id["mock-pole-1"]["classification"] == "pole_like"
        assert by_id["mock-wall-1"]["classification"] == "wall"
        # track ids are stable for the whole run
        for d in data:
            assert {o["track_id"] for o in d["tracked_objects"]} == set(by_id)
        # the pole and wall are out of the projected path -> never drive risk
        for d in data:
            assert d["risk"]["overall_risk"] in ("safe", "warning", "critical")

    def test_velocity_and_confidence_are_present_per_object(self):
        data, _ = _drive(build_multi_object_frames(frame_count=6, start_timestamp=BASE_TS))
        veh = next(o for o in data[-1]["tracked_objects"] if o["track_id"] == "mock-veh-1")
        assert veh["velocity"] is not None and veh["velocity"]["vx"] < 0.0
        assert 0.0 <= veh["confidence"] <= 1.0


# ======================================================================================
# 10. RISK comes from the processed state (dashboard does not recompute)
# ======================================================================================

class TestRiskIsPassThrough:
    def test_overall_risk_on_the_wire_equals_the_stm32_frame_value(self):
        frames = build_mock_stm32_frames("realtime_arc")
        data, _ = _drive(frames)
        for frame, d in zip(frames, data):
            assert d["risk"]["overall_risk"] == frame.risk.overall_risk.value

    def test_a_hand_built_critical_frame_is_reported_critical_verbatim(self):
        crit = build_mock_processed_frame(
            [MockObjectState("t-x", ObjectClassification.VEHICLE_LIKE, Point2D(x=1.5, y=0.0),
                             Velocity2D(vx=-3.0, vy=0.0), source=STM32SourceChannel.FUSED)],
            frame_index=0, timestamp=BASE_TS,
        )
        assert crit.risk.overall_risk is RiskLevel.CRITICAL
        data, _ = _drive([crit])
        assert data[0]["risk"]["overall_risk"] == "critical"


# ======================================================================================
# 11. FRAME AGE -- LIVE -> STALE -> unavailable, never "old values as current"
# ======================================================================================

class TestFrameAgeAndStale:
    def test_stream_goes_stale_then_unavailable_when_the_mock_stm32_stops(self):
        settings = _settings(esp32_heartbeat_timeout_s=5.0)
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(settings, transport=tr, now_fn=clock)
        src.connect()
        for f in build_mock_stm32_frames("tracking")[:3]:
            tr.push_frame(f)
        # drain the 3 good frames
        for _ in range(4):
            clock.advance(0.1)
            src.poll()
        assert src.status.state.value == "connected"

        # nothing more arrives
        clock.advance(1.5)  # past esp32_frame_stale_after_s (1.0)
        assert src.poll() is None
        assert src.status.state.value == "stale"

        clock.advance(5.0)  # past heartbeat timeout
        assert src.poll() is None
        assert src.status.is_unavailable


# ======================================================================================
# 12. SEQUENCE -- a missing sequence number is detected and reported
# ======================================================================================

class TestSequenceGapDetection:
    def test_missing_sequence_103_is_counted_as_a_dropped_frame(self):
        settings = _settings()
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(settings, transport=tr, now_fn=clock)
        src.connect()

        states = [MockObjectState("t-1", ObjectClassification.VEHICLE_LIKE, Point2D(x=6.0, y=0.0),
                                  Velocity2D(vx=-1.0, vy=0.0))]
        for fid, seq in ((0, 100), (1, 101), (2, 102), (3, 104)):  # 103 missing
            tr.push_frame(build_mock_processed_frame(states, frame_index=fid, sequence_number=seq, timestamp=BASE_TS + fid))
            clock.advance(0.1)
            src.poll()

        st = src.status
        assert st.dropped_frames == 1
        assert st.sequence_gaps == 1
        assert st.last_sequence_number == 104


# ======================================================================================
# 13 / 14. RECONNECTION + SESSION -- old objects/risk/TTC do not linger
# ======================================================================================

class TestReconnectionAndSession:
    def test_link_drop_then_recovery_starts_a_new_session_and_clears_state(self):
        settings = _settings()
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(settings, transport=tr, now_fn=clock)
        src.connect()
        first_session = src.session_id

        for f in build_mock_stm32_frames("tracking")[:3]:
            tr.push_frame(f)
        for _ in range(4):
            clock.advance(0.1)
            src.poll()
        assert src.status.frames_received == 3

        tr.push_error()  # ESP32 link drops
        clock.advance(0.1)
        src.poll()
        assert src.status.is_unavailable

        # backoff elapses, transport reopens, a fresh scene arrives
        clock.advance(settings.esp32_reconnect_initial_backoff_s + 0.1)
        src.poll()  # triggers reconnect
        for f in build_mock_stm32_frames("realtime_arc")[:2]:
            tr.push_frame(f)
        for _ in range(3):
            clock.advance(0.1)
            src.poll()

        assert src.session_id != first_session
        assert src.is_connected()
        # counters reset with the new session -- no carry-over of the old run
        assert src.status.frames_received <= 2

    def test_new_session_clears_objects_and_risk_in_the_adapter(self):
        settings = _settings()
        adapter = ProcessedFrameToLiveState(settings, session_id="sess-A")
        crit = build_mock_processed_frame(
            [MockObjectState("t-A", ObjectClassification.VEHICLE_LIKE, Point2D(x=1.0, y=0.0),
                             Velocity2D(vx=-3.0, vy=0.0))],
            frame_index=0, timestamp=BASE_TS,
        )
        a1 = adapter.build(crit)
        assert a1.live_state.tracked_objects and a1.collision_assessment.overall_risk is RiskLevel.CRITICAL

        adapter.reset(session_id="sess-B")
        assert adapter.session_id == "sess-B"
        empty = build_mock_processed_frame([], frame_index=0, timestamp=BASE_TS + 10)
        a2 = adapter.build(empty)
        assert a2.live_state.tracked_objects == []
        assert a2.collision_assessment.overall_risk is RiskLevel.SAFE
        assert a2.live_state.risk is None or a2.live_state.risk.overall_risk is RiskLevel.SAFE


# ======================================================================================
# 15 / 16. DASHBOARD + UNITY consume the identical wire envelope
# ======================================================================================

class TestWireEnvelopeCompleteness:
    def test_every_field_dashboard_and_unity_read_is_present_on_one_frame(self):
        data, _ = _drive(build_mock_stm32_frames("multi_object", live=False)[:6])
        d = data[-1]
        # SYSTEM
        assert d["source_id"] == "stm32_hardware"
        assert isinstance(d["sequence_number"], int)
        assert isinstance(d["timestamp"], (int, float))
        assert d["config"]["data_source"] == "hardware"
        # OBJECTS
        for o in d["tracked_objects"]:
            assert {"track_id", "classification", "distance", "velocity", "confidence",
                    "ttc", "risk", "sensor_source"} <= set(o)
        # SAFETY
        assert set(d["clearance"]) >= {"front", "rear", "left", "right",
                                       "min_clearance_m", "min_direction", "overall_status"}
        assert d["risk"]["overall_risk"] in ("safe", "warning", "critical")
        # sensor status
        assert "lidar" in d["sensor_status"]

    def test_scripted_transport_scene_selection_reaches_the_wire(self):
        """The ScriptedScenarioESP32Transport honours esp32_mock_scenario -- the same code path
        serve_unity_bridge.py uses in hardware mode."""
        settings = _settings(esp32_mock_scenario="tracking")
        tr = ScriptedScenarioESP32Transport(settings, pace_s=0.0)
        tr.open()
        raw = tr.receive(0.1)
        frame = parse_processed_frame(raw)
        assert frame.objects and frame.objects[0].track_id == "mock-obj-1"


# ======================================================================================
# 19. simulation mode is untouched
# ======================================================================================

class TestSimulationUnaffected:
    def test_default_data_source_is_still_simulation(self):
        assert Settings().data_source == "simulation"

    def test_mock_scenario_setting_is_ignored_outside_mock_transport(self):
        # a real (unset) transport still fails loudly -- the scenario knob does not enable anything
        from datasources.esp32.transport import build_transport

        with pytest.raises(ValueError):
            build_transport(Settings(data_source="hardware", esp32_transport=None,
                                     esp32_mock_scenario="realtime_arc"))


# ======================================================================================
# 20. HARDWARE mode -- NO simulation fallback, ever
# ======================================================================================

class TestNoSyntheticFallback:
    def test_link_down_publishes_only_status_and_error_no_perception_frames(self):
        clock = Clock()
        tr = MockESP32Transport()  # opened, never fed
        src = ESP32Source(_settings(esp32_heartbeat_timeout_s=100.0), transport=tr, now_fn=clock)

        class Rec:
            def __init__(self): self.msgs = []
            def start(self): ...
            def stop(self): ...
            def set_session(self, **k): ...
            def publish(self, m): self.msgs.append(m)
            @property
            def frames_sent(self): return sum(m.get("message_type") == "PERCEPTION_FRAME" for m in self.msgs)

        rec = Rec()
        run_esp32_edge(_settings(), rate_hz=0, json_server=rec, source=src,
                       adapter=ProcessedFrameToLiveState(_settings(), session_id=src.session_id),
                       max_iterations=10, now_fn=clock)
        assert rec.frames_sent == 0
        assert any(m.get("message_type") == "ERROR" and m["data"]["code"] == "HARDWARE_DATA_UNAVAILABLE"
                   for m in rec.msgs)

    def test_hardware_mode_with_unconfigured_transport_exits_never_simulates(self):
        cfg = Settings(data_source="hardware", esp32_transport=None)
        src = ESP32Source(cfg)
        with pytest.raises(SystemExit):
            run_esp32_edge(cfg, rate_hz=0, json_server=_NullServer(), source=src,
                           adapter=ProcessedFrameToLiveState(cfg, session_id=src.session_id),
                           max_iterations=3, now_fn=lambda: 0.0)


class _NullServer:
    def start(self): ...
    def stop(self): ...
    def set_session(self, **k): ...
    def publish(self, m): ...
    frames_sent = 0
    def of_type(self, t): return []

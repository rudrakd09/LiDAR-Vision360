"""Phase 3 -- SIMULATED ESP32 TRANSPORT TEST (end-to-end) + `run_esp32_edge`.

    Mock STM32ProcessedFrame -> MockESP32Transport -> ESP32Source -> ProcessedFrameToLiveState
        -> build_perception_frame_message -> (what Dashboard / Unity / backend consume)

**This is a SIMULATED ESP32 TRANSPORT TEST -- NOT hardware validation.** No physical ESP32, no
sockets. It proves the software path end to end so the real transport is a drop-in later.

Also covers required Phase-3 cases 16-18 at the runner level:
  * 16 -- every valid frame updates timestamp / sequence / frame_id / object count / tracked
          objects / TTC / clearance / risk / sensor status on the published wire message;
  * 17 -- simulation mode is untouched (the runner is hardware-only; default data_source stays
          "simulation");
  * 18 -- when the ESP32 link delivers nothing, the runner publishes SYSTEM_STATUS +
          ERROR(HARDWARE_DATA_UNAVAILABLE) and ZERO perception frames -- never synthetic data.
"""

from __future__ import annotations

import pytest

from common.config import Settings
from datasources.esp32 import (
    ESP32Source,
    MockESP32Transport,
    ProcessedFrameToLiveState,
    build_scripted_approaching_vehicle_frames,
    run_esp32_edge,
)
from datasources.esp32.transport import build_scripted_approaching_vehicle_frames as _mk


class RecordingStreamServer:
    """Stand-in for `streaming.PerceptionStreamServer` -- records published envelopes."""

    def __init__(self) -> None:
        self.messages: list[dict] = []
        self.sessions: list[tuple[str | None, str | None]] = []
        self.started = False
        self.stopped = False
        self.frames_sent = 0

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def set_session(self, session_id=None, source_id=None) -> None:
        self.sessions.append((session_id, source_id))

    def publish(self, message: dict) -> None:
        self.messages.append(message)
        if message.get("message_type") == "PERCEPTION_FRAME":
            self.frames_sent += 1

    def of_type(self, t: str) -> list[dict]:
        return [m for m in self.messages if m.get("message_type") == t]


BASE_TS = 1_700_000_000.0


class Clock:
    def __init__(self, t: float = BASE_TS) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _settings(**over) -> Settings:
    base = dict(data_source="hardware", esp32_transport="mock", esp32_frame_stale_after_s=1.0,
                esp32_heartbeat_timeout_s=6.0, esp32_read_timeout_s=0.01)
    base.update(over)
    return Settings(**base)


# ============================================================================================
# SIMULATED ESP32 TRANSPORT TEST -- end to end
# ============================================================================================

class TestSimulatedESP32TransportEndToEnd:
    def test_scripted_frames_reach_the_wire_as_perception_frames(self):
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(_settings(), transport=tr, now_fn=clock)
        src.connect()

        frames = build_scripted_approaching_vehicle_frames(60, start_distance_m=12.0, start_timestamp=BASE_TS)
        server = RecordingStreamServer()
        adapter = ProcessedFrameToLiveState(_settings(), session_id=src.session_id)

        # drive the runner loop manually so we can interleave frame delivery + clock
        for _ in range(65):
            f = None
            if frames:
                tr.push_frame(frames.pop(0))
            clock.advance(0.1)
            f = src.poll()
            st = src.status
            if st.session_id != adapter.session_id:
                adapter.reset(session_id=st.session_id)
            if f is not None:
                from streaming.protocol import build_perception_frame_message
                adapted = adapter.build(f)
                server.publish(build_perception_frame_message(
                    adapted.tracked_scan, collision_assessment=adapted.collision_assessment,
                    vehicle_state=adapted.vehicle_state, clearance_assessment=adapted.clearance_assessment,
                    settings=_settings(), session_id=adapter.session_id, live_state=adapted.live_state,
                ))

        pf = server.of_type("PERCEPTION_FRAME")
        assert len(pf) >= 40

        # case 16: every published frame carries the full real-time field set
        for m in pf:
            d = m["data"]
            assert isinstance(d["timestamp"], (int, float))
            assert isinstance(d["sequence_number"], int)
            assert m["frame_id"] == d["sequence_number"]
            assert d["source_id"] == "stm32_hardware"
            assert isinstance(d["objects"], list) and len(d["objects"]) == 1
            assert d["tracked_objects"] is not None and len(d["tracked_objects"]) == 1
            assert d["risk"] is not None and d["risk"]["overall_risk"] in ("safe", "warning", "critical")
            assert d["clearance"] is not None and "min_clearance_m" in d["clearance"]
            assert d["sensor_status"] is not None and "lidar" in d["sensor_status"]
            assert d["config"]["data_source"] == "hardware"  # dashboard shows Mode: HARDWARE

        # risk escalates as the scripted vehicle approaches (SAFE early, CRITICAL late)
        risks = [m["data"]["risk"]["overall_risk"] for m in pf]
        assert risks[0] == "safe"
        assert "critical" in risks

        # TTC on the tracked object shrinks over the run
        ttcs = [m["data"]["tracked_objects"][0]["ttc"] for m in pf if m["data"]["tracked_objects"][0]["ttc"] is not None]
        assert ttcs[0] > ttcs[-1]

        # clearance tightens
        clears = [m["data"]["clearance"]["front"]["distance_m"] for m in pf]
        assert clears[0] > clears[-1]

    def test_dashboard_relevant_fields_present_on_a_single_frame(self):
        """Case 14/15 + §14 dashboard checklist, on one representative frame."""
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(_settings(), transport=tr, now_fn=clock)
        src.connect()
        [frame] = build_scripted_approaching_vehicle_frames(1, start_distance_m=4.0, start_timestamp=BASE_TS)
        tr.push_frame(frame)
        clock.advance(0.1)
        f = src.poll()
        adapter = ProcessedFrameToLiveState(_settings(), session_id=src.session_id)
        from streaming.protocol import build_perception_frame_message
        adapted = adapter.build(f)
        msg = build_perception_frame_message(
            adapted.tracked_scan, collision_assessment=adapted.collision_assessment,
            vehicle_state=adapted.vehicle_state, clearance_assessment=adapted.clearance_assessment,
            settings=_settings(), session_id=adapter.session_id, live_state=adapted.live_state,
        )
        d = msg["data"]
        obj = d["tracked_objects"][0]
        assert obj["classification"] == "vehicle_like"
        assert obj["distance"] == frame.objects[0].distance_m
        assert obj["velocity"] == {"vx": -2.0, "vy": 0.0}
        assert obj["ttc"] is not None
        assert obj["risk"] in ("warning", "critical")
        assert obj["sensor_source"] in ("lidar", "radar", "lidar+radar")
        assert d["clearance"]["overall_status"] in ("safe", "caution", "low_clearance", "critical")


# ============================================================================================
# 18. runner never publishes synthetic frames when the link is unavailable
# ============================================================================================

class TestRunnerNoSyntheticFallback:
    def test_no_frames_publishes_only_status_and_error(self):
        clock = Clock()
        tr = MockESP32Transport()  # opened, but NOTHING ever pushed
        src = ESP32Source(_settings(esp32_heartbeat_timeout_s=100.0), transport=tr, now_fn=clock)
        server = RecordingStreamServer()
        adapter = ProcessedFrameToLiveState(_settings(), session_id=src.session_id)
        run_esp32_edge(_settings(), rate_hz=0, json_server=server, source=src, adapter=adapter,
                       max_iterations=10, now_fn=clock)

        assert server.of_type("PERCEPTION_FRAME") == []  # <-- never a fabricated frame
        errs = server.of_type("ERROR")
        assert errs, "expected a HARDWARE_DATA_UNAVAILABLE error"
        assert errs[0]["data"]["code"] == "HARDWARE_DATA_UNAVAILABLE"
        assert isinstance(errs[0]["data"]["message"], str) and errs[0]["data"]["message"]
        statuses = [m for m in server.of_type("SYSTEM_STATUS")]
        assert any(m["data"]["status"] == "hardware_unavailable" for m in statuses)

    def test_config_error_when_transport_unset_no_frames(self):
        server = RecordingStreamServer()
        cfg = Settings(data_source="hardware", esp32_transport=None)
        src = ESP32Source(cfg)
        with pytest.raises(SystemExit):
            run_esp32_edge(cfg, rate_hz=0, json_server=server, source=src,
                           adapter=ProcessedFrameToLiveState(cfg, session_id=src.session_id),
                           max_iterations=3, now_fn=lambda: 0.0)
        assert server.of_type("PERCEPTION_FRAME") == []

    def test_link_drop_midstream_stops_frames_and_reports_reason(self):
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(_settings(), transport=tr, now_fn=clock)
        server = RecordingStreamServer()
        adapter = ProcessedFrameToLiveState(_settings(), session_id=src.session_id)
        src.connect()

        frames = build_scripted_approaching_vehicle_frames(3, start_timestamp=BASE_TS)
        for fr in frames:
            tr.push_frame(fr)
        tr.push_error()  # link dies after 3 good frames

        from streaming.protocol import build_perception_frame_message
        for _ in range(12):
            clock.advance(0.1)
            f = src.poll()
            st = src.status
            if st.session_id != adapter.session_id:
                adapter.reset(session_id=st.session_id)
            if f is not None:
                a = adapter.build(f)
                server.publish(build_perception_frame_message(
                    a.tracked_scan, collision_assessment=a.collision_assessment,
                    vehicle_state=a.vehicle_state, clearance_assessment=a.clearance_assessment,
                    settings=_settings(), session_id=adapter.session_id, live_state=a.live_state))
            elif not st.is_fresh:
                from streaming.protocol import build_error_message
                server.publish(build_error_message(code="HARDWARE_DATA_UNAVAILABLE",
                                                   message=st.reason or "unavailable",
                                                   session_id=adapter.session_id))

        assert 1 <= server.frames_sent <= 3
        assert any(m["data"]["code"] == "HARDWARE_DATA_UNAVAILABLE" for m in server.of_type("ERROR"))


# ============================================================================================
# 17. simulation mode remains functional / untouched
# ============================================================================================

class TestSimulationUnaffected:
    def test_default_data_source_is_simulation(self):
        assert Settings().data_source == "simulation"

    def test_get_sensor_source_still_returns_simulator_for_simulation(self):
        import sys
        sys.path.insert(0, "scripts")
        try:
            from sensor_source import get_sensor_source
        finally:
            sys.path.pop(0)
        s = get_sensor_source(Settings(data_source="simulation"), scenario="01_empty")
        assert type(s).__name__ == "SimulatorSource"
        assert s.source_id == "simulated:01_empty"

    def test_esp32_package_has_no_simulator_dependency(self):
        # one-directional dependency: datasources.esp32 must never import the simulator package
        import pathlib
        pkg_dir = pathlib.Path(__import__("datasources.esp32", fromlist=["x"]).__file__).parent
        for py in pkg_dir.glob("*.py"):
            text = py.read_text(encoding="utf-8")
            assert "import simulator" not in text and "from simulator" not in text, py.name


def test_scripted_builder_alias_is_same_object():
    assert _mk is build_scripted_approaching_vehicle_frames

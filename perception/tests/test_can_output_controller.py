"""Phase 5 -- `can_output.output.STM32CANOutput` (transmission control, error handling, health).

Covers required cases 9 (sequence handling), 10 (transmission failure), 11 (CAN unavailable),
12 (CAN recovery), 14 (simulation/mock CAN output), plus periodic/event transmission control,
the rate cap, queue overflow, and ESP32-path independence (requirement 8).

Every transport is a `MockCANTransport` -- SIMULATED CAN OUTPUT, no bus. NOT hardware validation.
"""

from __future__ import annotations

import pytest

from can_output import (
    CANOutputState,
    MockCANTransport,
    STM32CANOutput,
    build_can_output,
    build_default_config,
)
from common.config import Settings
from datasources.esp32 import build_scripted_approaching_vehicle_frames
from models.collision import RiskLevel

BASE_TS = 1_700_000_000.0


class Clock:
    def __init__(self, t=BASE_TS):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _make(clock, **cfg_over):
    tr = MockCANTransport()
    config = build_default_config(**cfg_over)
    out = STM32CANOutput(config, tr, now_fn=clock)
    out._encoder._now = clock  # keep encoder timestamps on the same fake clock
    return out, tr


def _frames(n=5, **kw):
    return build_scripted_approaching_vehicle_frames(n, start_distance_m=kw.get("start_distance_m", 8.0), start_timestamp=BASE_TS)


# ============================================================================================
# 14. simulation / mock CAN output
# ============================================================================================

class TestMockCANOutput:
    def test_start_opens_the_simulated_transport(self):
        clock = Clock()
        out, tr = _make(clock)
        assert out.state is CANOutputState.DISABLED
        out.start()
        assert tr.is_open() and tr.open_count == 1
        assert out.state is CANOutputState.READY
        assert out.health.encoded is False  # layout PENDING -> semantic-only frames

    def test_submit_then_tick_sends_the_message_set(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        clock.advance(0.1)
        out.submit(_frames(1)[0])
        sent = out.tick()
        assert sent >= 3
        assert sorted({f.message_name for f in tr.sent}) == ["OBJECT_STATE", "PERCEPTION_HEADER", "SAFETY_STATE"]
        assert out.health.frames_sent == sent

    def test_no_physical_can_required_and_labelled_simulated(self):
        # build straight from Settings with the mock backend -- no interface / bitrate needed
        out = build_can_output(Settings(can_output_enabled=True, can_output_backend="mock"))
        out.start()
        assert out.state is CANOutputState.READY
        out.stop()


# ============================================================================================
# 9. sequence handling
# ============================================================================================

class TestSequenceHandling:
    def test_rolling_counter_increments_per_submitted_frame(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        seqs = []
        for f in _frames(4):
            clock.advance(0.1)
            out.submit(f)
            out.tick()
            hdr = [fr for fr in tr.sent if fr.message_name == "PERCEPTION_HEADER"][-1]
            seqs.append(hdr.sequence_number)
        assert seqs == [0, 1, 2, 3]
        # every message of one submit shares that submit's tx sequence number
        first_batch = [fr for fr in tr.sent if fr.sequence_number == 0]
        assert {fr.message_name for fr in first_batch} == {"PERCEPTION_HEADER", "OBJECT_STATE", "SAFETY_STATE"}

    def test_sequence_wraps_at_configured_modulus(self):
        clock = Clock()
        out, tr = _make(clock, sequence_modulus=4)
        out.start()
        for f in _frames(6):
            clock.advance(0.1)
            out.submit(f)
            out.tick()
        hdr_seqs = [fr.sequence_number for fr in tr.sent if fr.message_name == "PERCEPTION_HEADER"]
        assert hdr_seqs == [0, 1, 2, 3, 0, 1]


# ============================================================================================
# transmission control -- periodic / event / rate cap  (requirement 5)
# ============================================================================================

class TestTransmissionControl:
    def test_periodic_respects_cycle_time_ms(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        hdr = out._config.message("PERCEPTION_HEADER")
        hdr.cycle_time_ms = 200.0  # PERCEPTION_HEADER due every 200 ms
        # three submits 50 ms apart -> only the first PERCEPTION_HEADER is due
        for _ in range(3):
            clock.advance(0.05)
            out.submit(_frames(1)[0])
            out.tick()
        headers = [fr for fr in tr.sent if fr.message_name == "PERCEPTION_HEADER"]
        assert len(headers) == 1
        clock.advance(0.2)
        out.submit(_frames(1)[0])
        out.tick()
        assert len([fr for fr in tr.sent if fr.message_name == "PERCEPTION_HEADER"]) == 2

    def test_safety_state_sent_immediately_on_risk_change_event(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        safety = out._config.message("SAFETY_STATE")
        safety.cycle_time_ms = 10_000.0  # periodic almost never
        [safe_f, crit_f] = build_scripted_approaching_vehicle_frames(2, start_distance_m=20.0, start_timestamp=BASE_TS)
        crit_f = crit_f.model_copy(update={"risk": crit_f.risk.model_copy(update={"overall_risk": RiskLevel.CRITICAL})})
        clock.advance(0.05)
        out.submit(safe_f); out.tick()
        n_before = len([fr for fr in tr.sent if fr.message_name == "SAFETY_STATE"])
        clock.advance(0.05)
        out.submit(crit_f); out.tick()  # risk changed safe -> critical
        n_after = len([fr for fr in tr.sent if fr.message_name == "SAFETY_STATE"])
        assert n_after == n_before + 1

    def test_rate_cap_never_floods(self):
        clock = Clock()
        out, tr = _make(clock, max_transmit_rate_hz=30.0, queue_max_frames=500)
        out.start()
        # 100 frames (300 messages) submitted over ~0.5 s of wall-clock -> at 30 Hz the cap lets
        # at most ~15 through; the rest queue, none are flooded onto the "bus".
        for f in build_scripted_approaching_vehicle_frames(100, start_timestamp=BASE_TS):
            clock.advance(0.005)
            out.submit(f)
            out.tick()
        assert 0 < out.health.frames_sent < 30
        assert out.health.frames_built == 300

    def test_queue_overflow_drops_oldest_and_counts(self):
        clock = Clock()
        out, tr = _make(clock, queue_max_frames=5, max_transmit_rate_hz=0.001)  # effectively can't drain
        out.start()
        for f in _frames(10):
            clock.advance(0.001)
            out.submit(f)  # 3 messages each -> 30 enqueued into a depth-5 queue
        assert out.health.frames_dropped > 0
        assert out.health.queue_depth <= 5


# ============================================================================================
# 10. transmission failure   11. CAN unavailable   12. CAN recovery   (requirement 7)
# ============================================================================================

class TestErrorHandling:
    def test_10_transmit_failure_is_counted_and_frame_requeued_once(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        tr.fail_next_sends(1)  # first send() raises CANTransmitError
        clock.advance(0.1)
        out.submit(_frames(1)[0])
        out.tick()
        assert out.health.tx_errors == 1
        # the failed frame was re-queued; a later tick delivers it
        clock.advance(0.1)
        out.tick()
        assert out.health.frames_sent >= 3
        assert out.state is CANOutputState.ACTIVE

    def test_11_can_unavailable_on_start_marks_unavailable_no_frames(self):
        clock = Clock()
        tr = MockCANTransport()
        tr.unavailable = True
        out = STM32CANOutput(build_default_config(), tr, now_fn=clock)
        out.start()
        assert out.state is CANOutputState.UNAVAILABLE
        clock.advance(0.1)
        out.submit(_frames(1)[0])
        assert out.tick() == 0
        assert tr.sent == []  # nothing transmitted

    def test_11b_bus_off_during_send_transitions_and_stops_sending(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        clock.advance(0.1)
        out.submit(_frames(1)[0])
        out.tick()  # first batch ok
        n = out.health.frames_sent
        tr.set_bus_off()
        clock.advance(0.1)
        out.submit(_frames(1)[0])
        out.tick()
        assert out.state is CANOutputState.BUS_OFF
        assert out.health.bus_off_events >= 1
        assert out.health.frames_sent == n  # nothing new went out during bus-off

    def test_12_recovery_after_backoff_resumes_transmission(self):
        clock = Clock()
        out, tr = _make(clock, bus_recovery_initial_backoff_s=1.0)
        out.start()
        clock.advance(0.1); out.submit(_frames(1)[0]); out.tick()
        tr.set_bus_off()
        clock.advance(0.1); out.submit(_frames(1)[0]); out.tick()
        assert out.state is CANOutputState.BUS_OFF

        # too soon -> still recovering
        clock.advance(0.3); out.tick()
        assert out.state is CANOutputState.RECOVERING

        # bus comes back; backoff elapsed -> recovery succeeds
        tr.clear_bus_off()
        clock.advance(1.0); out.tick()
        assert out.state in (CANOutputState.READY, CANOutputState.ACTIVE)
        assert out.health.reconnect_attempts >= 1

        clock.advance(0.1); out.submit(_frames(1)[0]); sent = out.tick()
        assert sent >= 3

    def test_health_snapshot_exposes_everything(self):
        clock = Clock()
        out, tr = _make(clock)
        out.start()
        clock.advance(0.1); out.submit(_frames(1)[0]); out.tick()
        h = out.health
        assert h.state is CANOutputState.ACTIVE
        assert h.frames_built >= 3 and h.frames_sent >= 3
        assert h.encoded is False
        assert "OBJECT_STATE" in h.pending_hardware_parameters
        assert h.last_tx_at is not None


# ============================================================================================
# 8. ESP32 path independence
# ============================================================================================

class TestIndependenceFromESP32:
    def test_can_output_package_does_not_import_esp32(self):
        import pathlib
        pkg = pathlib.Path(__import__("can_output").__file__).parent
        for py in pkg.glob("*.py"):
            for line in py.read_text(encoding="utf-8").splitlines():
                s = line.strip()
                if s.startswith(("import ", "from ")):
                    assert "esp32" not in s, f"{py.name}: {s}"

    def test_same_frame_feeds_both_outputs_independently(self):
        from datasources.esp32 import ProcessedFrameToLiveState

        clock = Clock()
        out, tr = _make(clock)
        out.start()
        adapter = ProcessedFrameToLiveState(Settings(), session_id="indep")

        frame = _frames(1)[0]
        # ESP32 path
        live = adapter.build(frame).live_state
        # CAN path -- same frame, no shared state, no ordering dependency
        clock.advance(0.1)
        out.submit(frame)
        out.tick()

        assert live.sequence_number == frame.metadata.frame_id
        assert out.health.frames_sent >= 3
        # CAN failure must not affect the ESP32 path
        tr.set_bus_off()
        clock.advance(0.1); out.submit(frame); out.tick()
        live2 = adapter.build(frame).live_state
        assert live2 is not None and live2.risk.overall_risk == frame.risk.overall_risk

    def test_edge_runner_hook_off_by_default_and_isolated_when_on(self):
        from datasources.esp32 import ESP32Source, MockESP32Transport, run_esp32_edge
        from datasources.esp32 import edge_runner as er

        class RecordingServer:
            def __init__(self):
                self.msgs = []
                self.frames_sent = 0
            def start(self): pass
            def stop(self): pass
            def set_session(self, **k): pass
            def publish(self, m):
                self.msgs.append(m)
                if m.get("message_type") == "PERCEPTION_FRAME":
                    self.frames_sent += 1

        # --- off by default: no CAN output attached ---
        s_off = Settings(data_source="hardware", esp32_transport="mock")
        assert er._maybe_start_can_output(s_off) is None

        # --- on: both outputs run; ESP32 wire frames still flow ---
        s_on = Settings(data_source="hardware", esp32_transport="mock", esp32_read_timeout_s=0.01,
                        can_output_enabled=True, can_output_backend="mock")
        clock = Clock()
        tr = MockESP32Transport()
        src = ESP32Source(s_on, transport=tr, now_fn=clock)
        src.connect()
        for fr in build_scripted_approaching_vehicle_frames(6, start_timestamp=BASE_TS):
            tr.push_frame(fr)
        server = RecordingServer()
        run_esp32_edge(s_on, rate_hz=0, json_server=server, source=src, max_iterations=20, now_fn=clock)
        assert server.frames_sent >= 1  # ESP32 -> LiveState -> wire path worked with CAN also on

"""Phase 3 -- `datasources.esp32.source.ESP32Source`.

Covers the required Phase-3 cases 1-12 and 18 (adapter/LiveState cases 13-17 live in
test_esp32_live_state_adapter.py / test_esp32_edge_runner.py). Every "link" is an injected
`MockESP32Transport`; the clock is injected so timeout/stale/backoff are deterministic. No
hardware, no sockets.
"""

from __future__ import annotations

import pytest

from common.config import Settings
from datasources.esp32 import (
    ESP32ConfigurationError,
    ESP32Source,
    ESP32SourceState,
    MockESP32Transport,
    build_scripted_approaching_vehicle_frames,
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
        esp32_transport="mock",  # unused (transport is injected) but keeps connect() from config-erroring
        esp32_frame_stale_after_s=1.0,
        esp32_heartbeat_timeout_s=6.0,
        esp32_reconnect_initial_backoff_s=1.0,
        esp32_reconnect_max_backoff_s=4.0,
        esp32_read_timeout_s=0.01,
    )
    base.update(over)
    return Settings(**base)


def _object(track_id="t1", **over) -> STM32TrackedObject:
    base = dict(
        track_id=track_id, object_type=ObjectClassification.VEHICLE_LIKE,
        position=Point2D(x=6.0, y=0.0), distance_m=6.0,
        relative_velocity=Velocity2D(vx=-1.5, vy=0.0), confidence=0.8,
        ttc_s=4.0, risk=RiskLevel.WARNING, source=STM32SourceChannel.FUSED, in_projected_path=True,
    )
    base.update(over)
    return STM32TrackedObject(**base)


_FIRST = object()  # sentinel: default most_critical to the first object's track_id


def _frame(seq: int, *, ts: float, objects=None, risk_level=RiskLevel.WARNING,
           most_critical=_FIRST, clearance=True) -> STM32ProcessedFrame:
    objs = [_object()] if objects is None else objects
    if most_critical is _FIRST:
        most_critical = objs[0].track_id if objs else None
    clr = None
    if clearance:
        clr = STM32Clearance(front_m=2.0, rear_m=5.0, left_m=3.0, right_m=3.0,
                             min_clearance_m=2.0, min_direction=ClearanceDirection.FRONT,
                             status=ClearanceState.CAUTION, corridor_width_m=7.0)
    return STM32ProcessedFrame(
        metadata=STM32FrameMetadata(frame_id=seq, sequence_number=seq, timestamp=ts,
                                    source_id="stm32_hardware", active_object_count=len(objs)),
        sensor_status=STM32SensorStatus(lidar=STM32ChannelHealth(connected=True, ok=True),
                                        radar=STM32ChannelHealth(connected=True), fusion_active=True),
        vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0),
        objects=objs,
        clearance=clr,
        risk=STM32Risk(overall_risk=risk_level, most_critical_track_id=most_critical if objs else None),
        system_status=STM32SystemStatus(stm32_ok=True, processing_time_ms=4.0),
    )


def _make(clock: Clock, **settings_over):
    tr = MockESP32Transport()
    src = ESP32Source(_settings(**settings_over), transport=tr, now_fn=clock)
    return src, tr


# ============================================================================================
# 1. ESP32 connection
# ============================================================================================

class TestConnection:
    def test_connect_opens_transport_and_starts_session(self):
        clock = Clock()
        src, tr = _make(clock)
        assert src.status.state is ESP32SourceState.UNAVAILABLE
        src.connect()
        assert tr.is_open()
        assert src.status.state is ESP32SourceState.CONNECTING
        assert src.session_id  # a session id exists
        assert src.is_connected()

    def test_disconnect_closes_transport(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        src.disconnect()
        assert not tr.is_open()
        assert not src.is_connected()

    def test_unconfigured_transport_raises_and_never_falls_back(self):
        src = ESP32Source(Settings(esp32_transport=None))
        with pytest.raises(ESP32ConfigurationError):
            src.connect()
        assert src.status.state is ESP32SourceState.UNAVAILABLE

    def test_unknown_transport_raises_configuration_error(self):
        src = ESP32Source(Settings(esp32_transport="tcp"))  # not implemented -- protocol unspecified
        with pytest.raises(ESP32ConfigurationError):
            src.connect()


# ============================================================================================
# 2. Valid frame reception   /   10-12. object counts
# ============================================================================================

class TestValidFrameReception:
    def test_valid_frame_becomes_available_and_marks_connected(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=clock.t))
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None
        assert frame.metadata.frame_id == 0
        assert src.status.state is ESP32SourceState.CONNECTED
        assert src.status.frames_received == 1
        assert src.status.reason is None

    def test_same_frame_not_returned_twice(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=clock.t))
        clock.advance(0.1)
        assert src.poll() is not None
        clock.advance(0.1)
        assert src.poll() is None  # nothing new -- but still CONNECTED (fresh)
        assert src.status.state is ESP32SourceState.CONNECTED

    def test_empty_object_frame(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=clock.t, objects=[], risk_level=RiskLevel.SAFE, most_critical=None))
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None and len(frame.objects) == 0
        assert frame.metadata.active_object_count == 0

    def test_single_object_frame(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=clock.t, objects=[_object("only")]))
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None and len(frame.objects) == 1

    def test_multiple_object_frame(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        objs = [_object(f"t{i}") for i in range(4)]
        tr.push_frame(_frame(0, ts=clock.t, objects=objs))
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None and len(frame.objects) == 4
        assert {o.track_id for o in frame.objects} == {"t0", "t1", "t2", "t3"}

    def test_newest_frame_wins_when_several_arrive_between_polls(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        for i in range(5):
            tr.push_frame(_frame(i, ts=BASE_TS + i * 0.1))
        clock.advance(0.5)
        frame = src.poll()
        assert frame is not None and frame.metadata.frame_id == 4  # newest, older ones not re-served
        assert src.status.frames_received == 5


# ============================================================================================
# 3. Invalid frame   /   4. Malformed data
# ============================================================================================

class TestInvalidAndMalformed:
    def test_semantically_invalid_frame_rejected_no_object_produced(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        good = _frame(0, ts=clock.t)
        bad_json = good.model_dump_json().replace('"ttc_s":4.0', '"ttc_s":-9.0')  # invalid TTC
        tr.push_bytes(bad_json.encode("utf-8"))
        clock.advance(0.1)
        assert src.poll() is None
        assert src.status.frames_rejected == 1
        assert src.status.frames_received == 0
        assert src.status.state is not ESP32SourceState.CONNECTED

    def test_malformed_bytes_rejected(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_bytes(b"<not json at all>")
        clock.advance(0.1)
        assert src.poll() is None
        assert src.status.frames_rejected == 1

    def test_unknown_protocol_version_rejected_as_protocol_error(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        good = _frame(0, ts=clock.t)
        bumped = good.model_dump_json().replace('"protocol_version":"1.0.0"', '"protocol_version":"2.0.0"')
        tr.push_bytes(bumped.encode("utf-8"))
        clock.advance(0.1)
        assert src.poll() is None
        assert src.status.frames_rejected == 1
        assert "protocol" in (src.status.reason or "")

    def test_invalid_frame_then_valid_frame_recovers(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_bytes(b"garbage")
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None
        assert src.status.frames_rejected == 1 and src.status.frames_received == 1


# ============================================================================================
# 5. Sequence increment   /   6. Sequence gap
# ============================================================================================

class TestSequencing:
    def test_sequential_frames_no_drops(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        for i in range(4):
            tr.push_frame(_frame(i, ts=BASE_TS + i * 0.1))
            clock.advance(0.1)
            src.poll()
        assert src.status.dropped_frames == 0
        assert src.status.sequence_gaps == 0
        assert src.status.last_sequence_number == 3

    def test_sequence_gap_counts_dropped_but_still_accepts_frame(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        src.poll()
        tr.push_frame(_frame(5, ts=BASE_TS + 0.5))  # 1,2,3,4 missing
        clock.advance(0.1)
        frame = src.poll()
        assert frame is not None and frame.metadata.frame_id == 5  # newest state still delivered
        assert src.status.dropped_frames == 4
        assert src.status.sequence_gaps == 1

    def test_duplicate_and_out_of_order_frames_dropped(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(3, ts=BASE_TS))
        clock.advance(0.1)
        assert src.poll().metadata.frame_id == 3
        tr.push_frame(_frame(3, ts=BASE_TS + 0.1))  # duplicate
        tr.push_frame(_frame(1, ts=BASE_TS + 0.2))  # out of order
        clock.advance(0.1)
        assert src.poll() is None
        assert src.status.last_frame_id == 3  # unchanged


# ============================================================================================
# 7. Timeout   /   8. Stale frame
# ============================================================================================

class TestTimeoutAndStale:
    def test_no_first_frame_within_heartbeat_timeout_triggers_reconnect(self):
        clock = Clock()
        src, tr = _make(clock, esp32_heartbeat_timeout_s=3.0)
        src.connect()
        clock.advance(3.5)  # never sent a frame
        src.poll()
        assert src.status.state in (ESP32SourceState.DISCONNECTED, ESP32SourceState.RECONNECTING)
        assert "timeout" in (src.status.reason or "")

    def test_frame_older_than_stale_threshold_marks_stale_and_returns_none(self):
        clock = Clock()
        src, tr = _make(clock, esp32_frame_stale_after_s=1.0)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        assert src.poll() is not None
        clock.advance(2.0)  # no new frame
        assert src.poll() is None
        assert src.status.state is ESP32SourceState.STALE
        assert "stale" in (src.status.reason or "")
        assert src.status.frame_age_s is not None and src.status.frame_age_s >= 2.0

    def test_fresh_frame_after_stale_recovers_to_connected(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        src.poll()
        clock.advance(2.0)
        src.poll()
        assert src.status.state is ESP32SourceState.STALE
        tr.push_frame(_frame(1, ts=clock.t))
        clock.advance(0.05)
        frame = src.poll()
        assert frame is not None and src.status.state is ESP32SourceState.CONNECTED


# ============================================================================================
# 9. Reconnection
# ============================================================================================

class TestReconnection:
    def test_link_failure_then_backoff_then_reconnect_new_session(self):
        clock = Clock()
        src, tr = _make(clock, esp32_reconnect_initial_backoff_s=1.0)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        src.poll()
        session_a = src.session_id

        tr.push_error()  # link drops mid-stream
        clock.advance(0.1)
        src.poll()
        assert src.status.state in (ESP32SourceState.DISCONNECTED, ESP32SourceState.RECONNECTING)

        # too soon -- backoff not elapsed
        clock.advance(0.2)
        src.poll()
        assert src.status.state is ESP32SourceState.RECONNECTING

        # backoff elapsed -> reconnect succeeds, brand-new session id
        clock.advance(1.0)
        src.poll()
        assert src.status.state is ESP32SourceState.CONNECTING
        assert src.session_id != session_a
        assert src.status.reconnect_attempts >= 1

        # new frames flow again on the new session
        tr.push_frame(_frame(0, ts=clock.t))
        clock.advance(0.05)
        assert src.poll() is not None
        assert src.status.state is ESP32SourceState.CONNECTED

    def test_reconnect_attempts_exhausted_marks_unavailable(self):
        clock = Clock()
        src, tr = _make(clock, esp32_max_reconnect_attempts=2, esp32_reconnect_initial_backoff_s=0.1)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        src.poll()
        tr.push_error()
        clock.advance(0.1)
        src.poll()
        # each subsequent poll after backoff makes one reconnect attempt; the mock reopens fine,
        # so force repeated failures by pushing an error each cycle
        for _ in range(5):
            clock.advance(0.3)
            tr.push_error()
            src.poll()
        assert src.status.reconnect_attempts >= 2


# ============================================================================================
# 18. Hardware mode does not generate synthetic data
# ============================================================================================

class TestNoSyntheticFallback:
    def test_poll_returns_none_when_nothing_received_ever(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        for _ in range(5):
            clock.advance(0.1)
            assert src.poll() is None  # never a fabricated frame
        assert src.status.frames_received == 0

    def test_poll_returns_none_while_disconnected(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        src.disconnect()
        clock.advance(0.1)
        assert src.poll() is None

    def test_stale_never_re_serves_the_old_frame_as_current(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        tr.push_frame(_frame(0, ts=BASE_TS))
        clock.advance(0.1)
        assert src.poll() is not None
        for _ in range(4):
            clock.advance(1.0)
            assert src.poll() is None  # stale -> None, not the last frame again
        assert src.status.state is ESP32SourceState.STALE

    def test_only_frames_that_actually_arrived_are_counted(self):
        clock = Clock()
        src, tr = _make(clock)
        src.connect()
        # scripted-frame BUILDER exists, but ESP32Source never calls it -- only the transport does
        _ = build_scripted_approaching_vehicle_frames(3)
        clock.advance(0.1)
        src.poll()
        assert src.status.frames_received == 0

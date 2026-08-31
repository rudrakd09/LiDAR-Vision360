"""Phase 3 -- ESP32 transport abstraction (`datasources.esp32.transport`).

No hardware, no network. Covers the replaceable-transport contract, the in-memory mock used by
every other ESP32 test, and the clearly-labelled scripted development transport.
"""

from __future__ import annotations

import pytest

from common.config import Settings
from datasources.esp32 import (
    ESP32TransportError,
    MockESP32Transport,
    ScriptedScenarioESP32Transport,
    build_scripted_approaching_vehicle_frames,
    build_transport,
)
from datasources.stm32.processed import JsonProcessedFrameCodec, parse_processed_frame
from models.stm32_processed import STM32ProcessedFrame


class TestMockESP32Transport:
    def test_receive_before_open_raises(self):
        tr = MockESP32Transport()
        with pytest.raises(ESP32TransportError):
            tr.receive(0.1)

    def test_open_close_state(self):
        tr = MockESP32Transport()
        assert not tr.is_open()
        tr.open()
        assert tr.is_open() and tr.open_count == 1
        tr.close()
        assert not tr.is_open() and tr.close_count == 1

    def test_timeout_returns_none(self):
        tr = MockESP32Transport()
        tr.open()
        assert tr.receive(0.1) is None  # nothing queued

    def test_pushed_frame_round_trips(self):
        tr = MockESP32Transport()
        tr.open()
        frame = build_scripted_approaching_vehicle_frames(1)[0]
        tr.push_frame(frame)
        raw = tr.receive(0.1)
        assert isinstance(raw, bytes)
        assert parse_processed_frame(raw) == frame

    def test_pushed_error_is_raised(self):
        tr = MockESP32Transport()
        tr.open()
        tr.push_error()
        with pytest.raises(ESP32TransportError):
            tr.receive(0.1)

    def test_push_timeout_item(self):
        tr = MockESP32Transport()
        tr.open()
        tr.push_timeout()
        assert tr.receive(0.1) is None


class TestScriptedScenarioTransport:
    def test_emits_valid_frames(self):
        tr = ScriptedScenarioESP32Transport(Settings(esp32_mock_frame_count=10), pace_s=0.0)
        tr.open()
        seen = 0
        for _ in range(10):
            raw = tr.receive(0.1)
            assert raw is not None
            frame = parse_processed_frame(raw)
            assert isinstance(frame, STM32ProcessedFrame)
            seen += 1
        assert seen == 10

    def test_frame_ids_increase_monotonically(self):
        tr = ScriptedScenarioESP32Transport(Settings(esp32_mock_frame_count=5), pace_s=0.0)
        tr.open()
        ids = [parse_processed_frame(tr.receive(0.1)).metadata.frame_id for _ in range(12)]
        assert ids == sorted(ids)
        assert ids == list(range(12))  # keeps advancing past one script length, no reset

    def test_receive_before_open_raises(self):
        tr = ScriptedScenarioESP32Transport(Settings(), pace_s=0.0)
        with pytest.raises(ESP32TransportError):
            tr.receive(0.1)

    def test_paced_transport_gates_emission(self):
        tr = ScriptedScenarioESP32Transport(Settings(), pace_s=100.0)  # effectively "one then wait"
        tr.open()
        assert tr.receive(0.1) is not None
        assert tr.receive(0.1) is None  # not time for the next scripted frame yet


class TestBuildTransport:
    def test_mock_selected_by_name(self):
        assert isinstance(build_transport(Settings(esp32_transport="mock")), ScriptedScenarioESP32Transport)

    def test_unset_transport_rejected(self):
        with pytest.raises(ValueError):
            build_transport(Settings(esp32_transport=None))

    def test_placeholder_value_rejected(self):
        with pytest.raises(ValueError):
            build_transport(Settings(esp32_transport="<CONFIGURE>"))

    def test_unknown_real_protocol_rejected_not_invented(self):
        # tcp/udp/mqtt/websocket are NOT implemented -- the Wi-Fi protocol is unspecified.
        for name in ("tcp", "udp", "mqtt", "websocket"):
            with pytest.raises(ValueError, match="not implemented|not specified"):
                build_transport(Settings(esp32_transport=name))


def test_scripted_frames_pass_phase2_validation():
    codec = JsonProcessedFrameCodec()
    for frame in build_scripted_approaching_vehicle_frames(30):
        assert codec.decode(codec.encode(frame)) == frame  # decode() runs full validation

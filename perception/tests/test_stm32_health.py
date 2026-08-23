"""Unit tests for `datasources.stm32.health.HealthMonitor`/`ChannelHealth`. PARSER/UNIT TESTS
ONLY -- not hardware validation.
"""

import time

from datasources.stm32.health import HealthMonitor


class TestConnectionState:
    def test_starts_disconnected(self):
        h = HealthMonitor()
        assert h.connected is False
        assert h.connected_at is None

    def test_mark_connected_sets_timestamp(self):
        h = HealthMonitor()
        h.mark_connected()
        assert h.connected is True
        assert h.connected_at is not None
        assert h.disconnected_at is None

    def test_mark_disconnected_marks_all_channels_disconnected(self):
        h = HealthMonitor()
        h.mark_connected()
        h.record_frame("lidar")
        assert h.channel("lidar").connected is True
        h.mark_disconnected()
        assert h.connected is False
        assert h.channel("lidar").connected is False


class TestChannelTracking:
    def test_record_frame_increments_counts(self):
        h = HealthMonitor()
        h.record_frame("lidar")
        h.record_frame("lidar", dropped=2)
        ch = h.channel("lidar")
        assert ch.frames_received == 2
        assert ch.frames_dropped == 2
        assert ch.connected is True
        assert ch.last_received_at is not None

    def test_record_frame_duplicate_flag(self):
        h = HealthMonitor()
        h.record_frame("radar", duplicate_or_out_of_order=True)
        assert h.channel("radar").duplicate_or_out_of_order == 1

    def test_channels_are_independent(self):
        h = HealthMonitor()
        h.record_frame("lidar")
        assert h.channel("radar").frames_received == 0

    def test_is_stale_when_never_received(self):
        h = HealthMonitor()
        assert h.channel("lidar").is_stale(1.0) is True

    def test_is_stale_after_timeout(self):
        h = HealthMonitor()
        h.record_frame("lidar")
        assert h.channel("lidar").is_stale(0.0, now=time.time() + 5) is True
        assert h.channel("lidar").is_stale(10.0, now=time.time() + 1) is False

    def test_record_protocol_error(self):
        h = HealthMonitor()
        h.record_protocol_error("lidar", "checksum mismatch")
        ch = h.channel("lidar")
        assert ch.protocol_errors == 1
        assert ch.last_error == "checksum mismatch"


class TestReconnectTracking:
    def test_record_reconnect_attempt(self):
        h = HealthMonitor()
        h.record_reconnect_attempt("timeout")
        h.record_reconnect_attempt("timeout")
        assert h.reconnect_attempts == 2
        assert h.last_reconnect_error == "timeout"


class TestSnapshot:
    def test_snapshot_shape(self):
        h = HealthMonitor()
        h.mark_connected()
        h.record_frame("lidar")
        snap = h.snapshot()
        assert snap["connected"] is True
        assert "lidar" in snap["channels"]
        assert snap["channels"]["lidar"]["frames_received"] == 1

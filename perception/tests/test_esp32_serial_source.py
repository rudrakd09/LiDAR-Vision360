"""Tests for `datasources.esp32_serial.source.ESP32SerialSource` and its reader.

The reader is exercised against a fake `serial` module injected into `sys.modules`, so the whole
link -- open, read, disconnect, reconnect, backpressure -- is testable with no COM port and no
hardware attached. Nothing here talks to a real device.

The most important assertions in this file are the negative ones: a source with no hardware
behind it must raise, and must NEVER return synthesised points.
"""

from __future__ import annotations

import sys
import time
import types

import pytest

from common.config import Settings
from datasources.esp32_serial import (
    ESP32SerialConnectionError,
    ESP32SerialSource,
    ESP32SerialTimeoutError,
)
from datasources.esp32_serial.reader import SerialLineReader


# ---------------------------------------------------------------------------------------------
# A fake pyserial, injected as `serial` so `reader` imports it unchanged.
# ---------------------------------------------------------------------------------------------

class FakeSerial:
    """Serves a scripted list of byte chunks, then optionally raises to simulate an unplug."""

    instances: list["FakeSerial"] = []
    chunks: list[bytes] = []
    raise_on_open: Exception | None = None
    raise_after_chunks: Exception | None = None

    def __init__(self, port: str, baudrate: int, timeout: float) -> None:
        if FakeSerial.raise_on_open is not None:
            raise FakeSerial.raise_on_open
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.closed = False
        self._queue = list(FakeSerial.chunks)
        FakeSerial.instances.append(self)

    @property
    def in_waiting(self) -> int:
        return len(self._queue[0]) if self._queue else 0

    def read(self, _size: int) -> bytes:
        if self._queue:
            return self._queue.pop(0)
        if FakeSerial.raise_after_chunks is not None:
            raise FakeSerial.raise_after_chunks
        time.sleep(0.005)  # emulate pyserial's read timeout on a quiet port
        return b""

    def close(self) -> None:
        self.closed = True


class FakeSerialException(Exception):
    """Stands in for `serial.SerialException`."""


@pytest.fixture(autouse=True)
def fake_serial_module(monkeypatch: pytest.MonkeyPatch):
    FakeSerial.instances = []
    FakeSerial.chunks = []
    FakeSerial.raise_on_open = None
    FakeSerial.raise_after_chunks = None

    module = types.ModuleType("serial")
    module.Serial = FakeSerial
    module.SerialException = FakeSerialException
    monkeypatch.setitem(sys.modules, "serial", module)
    yield module


def wait_until(predicate, timeout_s: float = 2.0) -> bool:
    """Polls `predicate` until true or the timeout expires. Returns whether it became true."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def settings_for(**overrides) -> Settings:
    base = {
        "data_source": "esp32_serial",
        "esp32_serial_port": "COM_TEST",
        "esp32_serial_baudrate": 115200,
        "esp32_serial_min_points_per_scan": 3,
        "esp32_serial_scan_timeout_s": 1.0,
        "esp32_serial_reconnect_initial_backoff_s": 0.01,
        "esp32_serial_reconnect_max_backoff_s": 0.02,
        "esp32_serial_log_every_n_scans": 0,
    }
    base.update(overrides)
    return Settings(_env_file=None, **base)


# ---------------------------------------------------------------------------------------------
# SerialLineReader
# ---------------------------------------------------------------------------------------------

class TestSerialLineReader:
    def test_reads_complete_lines_and_reports_connected(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:850\nA:1 , D:842\n"]
        reader = SerialLineReader(port="COM_TEST", baudrate=115200)

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.lines_read >= 2)
            assert reader.is_connected
            assert reader.read_lines() == ["A:0 , D:850", "A:1 , D:842"]
        finally:
            reader.stop()

    def test_a_line_split_across_reads_is_reassembled(self) -> None:
        """TCP-style partial reads are normal on a serial port; a split line must not be lost."""
        FakeSerial.chunks = [b"A:45 , D:12", b"00\nA:46 , D:1201\n"]
        reader = SerialLineReader(port="COM_TEST", baudrate=115200)

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.lines_read >= 2)
            assert reader.read_lines() == ["A:45 , D:1200", "A:46 , D:1201"]
        finally:
            reader.stop()

    def test_crlf_terminators_are_stripped(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:850\r\nA:1 , D:842\r\n"]
        reader = SerialLineReader(port="COM_TEST", baudrate=115200)

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.lines_read >= 2)
            assert reader.read_lines() == ["A:0 , D:850", "A:1 , D:842"]
        finally:
            reader.stop()

    def test_a_failed_open_does_not_raise_and_keeps_retrying(self) -> None:
        """The ESP32 may be plugged in after the Edge starts -- that must not be fatal."""
        FakeSerial.raise_on_open = FakeSerialException("could not open port COM_TEST")
        reader = SerialLineReader(
            port="COM_TEST", baudrate=115200,
            reconnect_initial_backoff_s=0.01, reconnect_max_backoff_s=0.02,
        )

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.connect_attempts >= 2)
            assert not reader.is_connected
            assert reader.is_running  # still trying, not dead
            assert "could not open port" in (reader.stats.last_error or "")
        finally:
            reader.stop()

    def test_max_reconnect_attempts_eventually_gives_up(self) -> None:
        FakeSerial.raise_on_open = FakeSerialException("nope")
        reader = SerialLineReader(
            port="COM_TEST", baudrate=115200,
            reconnect_initial_backoff_s=0.001, reconnect_max_backoff_s=0.002,
            max_reconnect_attempts=3,
        )

        reader.start()
        try:
            assert wait_until(lambda: not reader.is_running)
            assert reader.stats.connect_attempts == 3
        finally:
            reader.stop()

    def test_a_mid_stream_disconnect_reconnects_automatically(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:850\n"]
        FakeSerial.raise_after_chunks = FakeSerialException("device disconnected")
        reader = SerialLineReader(
            port="COM_TEST", baudrate=115200,
            reconnect_initial_backoff_s=0.01, reconnect_max_backoff_s=0.02,
        )

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.reconnect_count >= 1)
            assert len(FakeSerial.instances) >= 2  # the port really was reopened
            assert FakeSerial.instances[0].closed
        finally:
            reader.stop()

    def test_backpressure_drops_the_oldest_lines_and_counts_them(self) -> None:
        """A bounded queue keeps the pipeline on FRESH data; drops must be visible, not silent."""
        FakeSerial.chunks = [b"".join(f"A:{i} , D:1000\n".encode() for i in range(10))]
        reader = SerialLineReader(port="COM_TEST", baudrate=115200, max_buffered_lines=4)

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.lines_dropped_backpressure > 0)
            buffered = reader.read_lines()
            assert len(buffered) <= 4
            # The newest lines survived; the oldest were evicted.
            assert buffered[-1] == "A:9 , D:1000"
        finally:
            reader.stop()

    def test_a_torrent_with_no_terminator_is_discarded_not_buffered_forever(self) -> None:
        """The classic wrong-baud-rate symptom -- must not grow one pseudo-line without bound."""
        FakeSerial.chunks = [b"\xff" * 3000, b"\xfe" * 3000]
        reader = SerialLineReader(port="COM_TEST", baudrate=9600)

        reader.start()
        try:
            assert wait_until(lambda: reader.stats.partial_lines_discarded >= 1)
            assert reader.stats.lines_read == 0
        finally:
            reader.stop()

    def test_stop_is_idempotent_and_closes_the_port(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:850\n"]
        reader = SerialLineReader(port="COM_TEST", baudrate=115200)
        reader.start()
        assert wait_until(lambda: reader.is_connected)

        reader.stop()
        reader.stop()

        assert not reader.is_running
        assert FakeSerial.instances[0].closed


# ---------------------------------------------------------------------------------------------
# ESP32SerialSource
# ---------------------------------------------------------------------------------------------

class TestESP32SerialSource:
    def test_read_scan_before_connect_raises(self) -> None:
        source = ESP32SerialSource(settings=settings_for())

        with pytest.raises(ESP32SerialConnectionError, match="before connect"):
            source.read_scan()

    def test_assembles_a_real_scan_from_the_wire_format(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:1000\nA:90 , D:2000\nA:270 , D:3000\nA:0 , D:1000\n"]
        source = ESP32SerialSource(settings=settings_for())

        source.connect()
        try:
            frame = source.read_scan()
        finally:
            source.disconnect()

        assert frame.point_count == 3
        assert frame.source_id == "esp32_serial"
        assert [p.angle for p in frame.points] == [0.0, 90.0, 270.0]
        # Millimetres on the wire -> metres in the frame.
        assert [p.distance for p in frame.points] == pytest.approx([1.0, 2.0, 3.0])

    def test_malformed_lines_are_skipped_without_breaking_the_scan(self) -> None:
        FakeSerial.chunks = [
            b"ESP32 booting...\nA:0 , D:1000\nA:abc,D:5\nA:90 , D:2000\n"
            b"A:400,D:1\nA:270 , D:3000\nA:0 , D:1000\n"
        ]
        source = ESP32SerialSource(settings=settings_for())

        source.connect()
        try:
            frame = source.read_scan()
        finally:
            source.disconnect()

        assert frame.point_count == 3  # only the three valid measurements
        assert source.stats["parser"]["rejected"] == 3

    def test_timeout_raises_and_returns_no_synthesised_points(self) -> None:
        """The core "NO FAKE DATA" guarantee: silence produces an error, never invented points."""
        FakeSerial.chunks = []
        source = ESP32SerialSource(settings=settings_for(esp32_serial_scan_timeout_s=0.2))

        source.connect()
        try:
            with pytest.raises(ESP32SerialTimeoutError):
                source.read_scan()
        finally:
            source.disconnect()

    def test_timeout_message_explains_a_port_that_never_opened(self) -> None:
        FakeSerial.raise_on_open = FakeSerialException("Access is denied")
        source = ESP32SerialSource(settings=settings_for(esp32_serial_scan_timeout_s=0.2))

        source.connect()
        try:
            with pytest.raises(ESP32SerialTimeoutError) as excinfo:
                source.read_scan()
        finally:
            source.disconnect()

        message = str(excinfo.value)
        assert "not open" in message
        assert "Access is denied" in message
        assert "sniff_esp32.py --list" in message  # actionable, not generic

    def test_timeout_message_explains_a_wrong_baud_rate(self) -> None:
        """Lines arrive but none parse -- the message must say so, not just "timed out"."""
        FakeSerial.chunks = [b"\x00garbage\n" * 20]
        source = ESP32SerialSource(settings=settings_for(esp32_serial_scan_timeout_s=0.3))

        source.connect()
        try:
            with pytest.raises(ESP32SerialTimeoutError) as excinfo:
                source.read_scan()
        finally:
            source.disconnect()

        assert "none parsed" in str(excinfo.value)
        assert "baud rate" in str(excinfo.value)

    def test_is_connected_stays_true_across_a_transient_unplug(self) -> None:
        """`stream()` must keep polling through a reconnect rather than ending the run."""
        FakeSerial.chunks = [b"A:0 , D:1000\n"]
        FakeSerial.raise_after_chunks = FakeSerialException("device disconnected")
        source = ESP32SerialSource(settings=settings_for())

        source.connect()
        try:
            assert wait_until(lambda: source._reader.stats.reconnect_count >= 1)
            assert source.is_connected()  # the SOURCE is alive...
            assert source.stats["reader"]["connected"] in (True, False)  # ...the LINK may flap
        finally:
            source.disconnect()

    def test_disconnect_stops_the_reader_thread(self) -> None:
        source = ESP32SerialSource(settings=settings_for())
        source.connect()

        source.disconnect()

        assert not source.is_connected()

    def test_context_manager_connects_and_disconnects(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:1000\nA:90 , D:2000\nA:270 , D:3000\nA:0 , D:1000\n"]
        source = ESP32SerialSource(settings=settings_for())

        with source:
            assert source.is_connected()
            assert source.read_scan().point_count == 3

        assert not source.is_connected()

    def test_stats_report_measured_values_only(self) -> None:
        FakeSerial.chunks = [b"A:0 , D:1000\nA:90 , D:2000\nA:270 , D:3000\nA:0 , D:1000\n"]
        source = ESP32SerialSource(settings=settings_for())

        source.connect()
        try:
            source.read_scan()
            stats = source.stats
        finally:
            source.disconnect()

        assert stats["parser"]["accepted"] == 4
        assert stats["frames"]["scans_completed"] == 1
        # One scan gives no interval to measure a rate from -- `None`, not a fabricated default.
        assert stats["frames"]["measured_scan_rate_hz"] is None
        assert source.measured_scan_rate_hz is None

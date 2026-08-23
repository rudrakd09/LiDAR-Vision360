"""Tests for `datasources.STM32Source` -- the STM32 hardware adapter.

PARSER/UNIT/ARCHITECTURE TESTS ONLY, all against an in-memory fake `Transport` and a synthetic,
hypothetical frame layout invented for this test file. NOT hardware validation -- no real STM32,
A3M1, or R121 is involved anywhere here, and nothing in this file should be read as a claim about
the actual (still unknown) wire protocol.
"""

from __future__ import annotations

import struct

import pytest

from common.config import Settings
from datasources import STM32Source, SensorSource
from datasources.stm32.errors import STM32ConfigurationError, STM32ConnectionError
from datasources.stm32.parsers import LiDARMessageParser, RadarMessageParser
from datasources.stm32.transport import Transport
from models.lidar import LiDARPoint
from models.radar import RadarReading, RadarTarget
from models.scan import ScanFrame


# --- Test fixtures: a synthetic frame layout + parsers + fake transport, invented for this file
# only. Layout: <AA 55><msg_type:1><seq:2 LE><payload...><crc:1><0D 0A>.

_START = "AA55"
_END = "0D0A"
_LIDAR_TYPE = 1
_RADAR_TYPE = 2


def _full_settings(**overrides) -> Settings:
    values = dict(
        stm32_uart_port="COM7",
        stm32_uart_baudrate=115200,
        stm32_frame_start_marker=_START,
        stm32_frame_end_marker=_END,
        stm32_byte_order="little",
        stm32_message_type_offset=2,
        stm32_message_type_width=1,
        stm32_sequence_number_offset=3,
        stm32_sequence_number_width=2,
        stm32_lidar_message_type=_LIDAR_TYPE,
        stm32_radar_message_type=_RADAR_TYPE,
        stm32_crc_algorithm="xor8",
        stm32_timestamp_format="none",
        stm32_read_timeout_s=0.05,
        stm32_reconnect_initial_backoff_s=0.001,
        stm32_reconnect_max_backoff_s=0.001,
    )
    values.update(overrides)
    return Settings(_env_file=None, **values)


def _build_frame(msg_type: int, seq: int, payload: bytes, *, corrupt_crc: bool = False) -> bytes:
    header = bytes([msg_type]) + seq.to_bytes(2, "little")
    # The checksum covers everything from the start marker up to (not including) the checksum
    # byte itself -- matching STM32Source._verify_crc's own span (start marker included; see its
    # `_trailing_marker_length` handling for the end marker).
    protected = bytes.fromhex(_START) + header + payload
    crc = 0
    for b in protected:
        crc ^= b
    if corrupt_crc:
        crc ^= 0xFF
    return protected + bytes([crc]) + bytes.fromhex(_END)


class _ExampleLiDARParser(LiDARMessageParser):
    """<AA55><type><seq LE><uint16 angle_centideg><uint16 distance_mm>...<crc><0D0A> -- test fixture only."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int) -> list[LiDARPoint]:
        payload = frame[5:-3]  # strip start(2)+type(1)+seq(2) and crc(1)+end(2)
        points = []
        for i in range(0, len(payload), 4):
            angle_centideg, distance_mm = struct.unpack_from("<HH", payload, i)
            points.append(LiDARPoint(angle=angle_centideg / 100.0, distance=distance_mm / 1000.0, timestamp=timestamp, valid=True))
        return points


class _ExampleRadarParser(RadarMessageParser):
    """<AA55><type><seq LE><uint16 range_cm><int16 velocity_cms><crc><0D0A> -- test fixture only."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int, source_id: str) -> RadarReading:
        payload = frame[5:-3]
        range_cm, velocity_cms = struct.unpack_from("<Hh", payload, 0)
        target = RadarTarget(range_m=range_cm / 100.0, velocity_mps=velocity_cms / 100.0)
        return RadarReading(source_id=source_id, sequence_number=sequence_number, timestamp=timestamp, targets=[target])


class _ScriptedTransport(Transport):
    """Fake transport that yields a pre-scripted sequence of `read()` results (bytes, or an
    Exception instance to raise), then `b""` forever once the script is exhausted."""

    def __init__(self, script: list):
        self._script = list(script)
        self._open = False
        self.open_calls = 0
        self.written: list[bytes] = []

    def open(self) -> None:
        self.open_calls += 1
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def read(self, size: int) -> bytes:
        if not self._script:
            return b""
        item = self._script.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)


class _StallThenRecoverTransport(Transport):
    """Fake transport returning no data at all until it has been reopened once (simulating a
    disconnect the connection manager must notice via the read-timeout watchdog and recover
    from via `reconnect()`), after which it serves `frame_bytes` once."""

    def __init__(self, frame_bytes: bytes):
        self._open = False
        self.open_calls = 0
        self._frame = frame_bytes
        self._served = False

    def open(self) -> None:
        self.open_calls += 1
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def read(self, size: int) -> bytes:
        if self.open_calls > 1 and not self._served:
            self._served = True
            return self._frame
        return b""

    def write(self, data: bytes) -> int:
        return len(data)


# --- Basic contract -------------------------------------------------------------------------


class TestBasicContract:
    def test_is_a_sensor_source(self):
        assert issubclass(STM32Source, SensorSource)

    def test_defaults_come_from_settings(self):
        settings = Settings(_env_file=None, stm32_uart_port="COM7", stm32_uart_baudrate=9600)
        source = STM32Source(settings=settings)
        assert source.port == "COM7"
        assert source.baudrate == 9600

    def test_explicit_args_override_settings(self):
        settings = Settings(_env_file=None, stm32_uart_port="COM7", stm32_uart_baudrate=9600)
        source = STM32Source(settings=settings, port="COM9", baudrate=57600)
        assert source.port == "COM9"
        assert source.baudrate == 57600

    def test_is_connected_false_by_default(self):
        source = STM32Source(settings=Settings(_env_file=None))
        assert source.is_connected() is False

    def test_source_id_from_settings(self):
        settings = Settings(_env_file=None, hardware_source_id="my_rig")
        source = STM32Source(settings=settings)
        assert source.source_id == "my_rig"


# --- Configuration validation -- the core "never guess, never fall back" contract ------------


class TestConfigurationValidation:
    def test_connect_raises_configuration_error_when_totally_unconfigured(self):
        source = STM32Source(settings=Settings(_env_file=None))
        with pytest.raises(STM32ConfigurationError) as exc_info:
            source.connect()
        assert "STM32 protocol configuration incomplete" in str(exc_info.value)

    def test_error_lists_every_missing_field(self):
        source = STM32Source(settings=Settings(_env_file=None))
        with pytest.raises(STM32ConfigurationError) as exc_info:
            source.connect()
        message = str(exc_info.value)
        for expected in ["frame format", "byte order", "message-type field offset", "sequence-number field offset", "CRC/checksum algorithm", "timestamp format"]:
            assert expected in message

    def test_connect_never_opens_a_port_when_unconfigured(self):
        # If this were to actually attempt a serial open, it would raise STM32ConnectionError
        # (COM7 doesn't exist on the test machine) instead of STM32ConfigurationError -- getting
        # the configuration error proves the port was never touched.
        source = STM32Source(settings=Settings(_env_file=None, stm32_uart_port="COM7"))
        with pytest.raises(STM32ConfigurationError):
            source.connect()

    def test_partially_configured_still_raises(self):
        settings = _full_settings()
        # Remove just one required field -- CRC algorithm.
        settings = Settings(_env_file=None, **{**settings.model_dump(), "stm32_crc_algorithm": None})
        source = STM32Source(settings=settings)
        with pytest.raises(STM32ConfigurationError) as exc_info:
            source.connect()
        assert "CRC/checksum algorithm" in str(exc_info.value)

    def test_read_scan_before_connect_raises_runtime_error(self):
        source = STM32Source(settings=_full_settings())
        with pytest.raises(RuntimeError):
            source.read_scan()


# --- Full read_scan() flow against a fake transport -------------------------------------------


class TestReadScanEndToEnd:
    def _connected_source(self, script: list) -> STM32Source:
        settings = _full_settings()
        transport = _ScriptedTransport(script)
        source = STM32Source(
            settings=settings, lidar_parser=_ExampleLiDARParser(), radar_parser=_ExampleRadarParser(), transport=transport
        )
        source.connect()
        return source

    def test_returns_a_scan_frame_for_a_lidar_message(self):
        payload = struct.pack("<HH", 9000, 5000)  # angle 90.00 deg, distance 5.000 m
        frame_bytes = _build_frame(_LIDAR_TYPE, seq=1, payload=payload)
        source = self._connected_source([frame_bytes])

        scan = source.read_scan()
        assert isinstance(scan, ScanFrame)
        assert scan.source_id == source.source_id
        assert scan.point_count == 1
        assert scan.points[0].angle == pytest.approx(90.0)
        assert scan.points[0].distance == pytest.approx(5.0)

    def test_outgoing_sequence_number_increments(self):
        payload = struct.pack("<HH", 0, 1000)
        frames = _build_frame(_LIDAR_TYPE, 1, payload) + _build_frame(_LIDAR_TYPE, 2, payload)
        source = self._connected_source([frames])
        first = source.read_scan()
        second = source.read_scan()
        assert second.sequence_number == first.sequence_number + 1

    def test_radar_frame_updates_latest_radar_reading_and_is_not_returned_as_a_scan(self):
        radar_payload = struct.pack("<Hh", 1200, -150)
        lidar_payload = struct.pack("<HH", 0, 1000)
        script = [_build_frame(_RADAR_TYPE, 1, radar_payload) + _build_frame(_LIDAR_TYPE, 1, lidar_payload)]
        source = self._connected_source(script)

        assert source.latest_radar_reading is None
        scan = source.read_scan()  # radar frame is consumed internally; only the LiDAR one returns
        assert isinstance(scan, ScanFrame)
        assert source.latest_radar_reading is not None
        assert source.latest_radar_reading.targets[0].range_m == pytest.approx(12.0)

    def test_corrupt_crc_frame_is_skipped_not_fatal(self):
        good_payload = struct.pack("<HH", 0, 1000)
        bad = _build_frame(_LIDAR_TYPE, 1, good_payload, corrupt_crc=True)
        good = _build_frame(_LIDAR_TYPE, 2, good_payload)
        source = self._connected_source([bad + good])

        scan = source.read_scan()  # skips the corrupt one, returns the valid one
        assert isinstance(scan, ScanFrame)
        assert source.health.channel("frame").protocol_errors == 1

    def test_unknown_message_type_is_skipped_not_fatal(self):
        good_payload = struct.pack("<HH", 0, 1000)
        unknown = _build_frame(99, 1, good_payload)
        good = _build_frame(_LIDAR_TYPE, 1, good_payload)
        source = self._connected_source([unknown + good])

        scan = source.read_scan()
        assert isinstance(scan, ScanFrame)

    def test_dropped_frame_is_reflected_in_health(self):
        payload = struct.pack("<HH", 0, 1000)
        frames = _build_frame(_LIDAR_TYPE, 1, payload) + _build_frame(_LIDAR_TYPE, 5, payload)  # seq 2,3,4 missing
        source = self._connected_source([frames])
        source.read_scan()
        source.read_scan()
        assert source.health.channel("lidar").frames_dropped == 3


# --- Timeout + reconnection --------------------------------------------------------------------


class TestTimeoutAndReconnection:
    def test_no_data_triggers_reconnect_and_then_recovers(self):
        payload = struct.pack("<HH", 0, 2000)
        frame_bytes = _build_frame(_LIDAR_TYPE, 1, payload)
        settings = _full_settings(stm32_read_timeout_s=0.01)
        transport = _StallThenRecoverTransport(frame_bytes)
        source = STM32Source(settings=settings, lidar_parser=_ExampleLiDARParser(), radar_parser=_ExampleRadarParser(), transport=transport)
        source.connect()
        assert transport.open_calls == 1

        scan = source.read_scan()  # must stall, notice the timeout, reconnect, then succeed
        assert isinstance(scan, ScanFrame)
        assert transport.open_calls == 2  # one reconnect happened
        assert source.health.reconnect_attempts == 0  # this reconnect succeeded on the first attempt, no failed attempts recorded

    def test_reconnect_exhaustion_propagates_connection_error(self):
        settings = _full_settings(stm32_read_timeout_s=0.01, stm32_max_reconnect_attempts=2)
        # A transport whose read() never returns data and whose open() always fails after the
        # first (successful) connect() call -- forces reconnect() to exhaust its attempts.
        class _NeverRecoversTransport(Transport):
            def __init__(self):
                self.open_calls = 0
                self._open = False

            def open(self):
                self.open_calls += 1
                if self.open_calls == 1:
                    self._open = True
                    return
                raise STM32ConnectionError("still down")

            def close(self):
                self._open = False

            def is_open(self):
                return self._open

            def read(self, size):
                return b""

            def write(self, data):
                return len(data)

        transport = _NeverRecoversTransport()
        source = STM32Source(settings=settings, transport=transport)
        source.connect()
        with pytest.raises(STM32ConnectionError):
            source.read_scan()

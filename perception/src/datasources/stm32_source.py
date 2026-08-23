"""STM32 hardware LiDAR (+ radar) data-source adapter (Phase 8 architecture; real protocol
values are Phase 16, once the hardware team's specification is known).

`STM32Source` is the concrete `SensorSource` selected by `Settings.data_source == "hardware"`
(`DATA_SOURCE`/`LIDAR_DATA_SOURCE` -- see `common.config.Settings` and `scripts/sensor_source.py`).
It composes the pieces in `datasources.stm32` (transport, connection manager, framing, CRC,
sequence validation, health, parsers) into one `LiDARDataSource`:

    A3M1 --UART--> STM32F103C8T6 --(STM32-to-Edge link)--> STM32Source --SensorFrame--> pipeline

`connect()` validates every piece of required wire-protocol configuration up front and raises
`STM32ConfigurationError` (never opens a port, never falls back to simulated/fabricated data) if
anything is still unset -- see `_validate_protocol_config` below and
docs/hardware-integration.md "Hardware integration checklist" for exactly what the hardware team
must still supply. `read_scan()` reads bytes, extracts frames, validates CRC and sequence number,
dispatches LiDAR frames to `lidar_parser` (producing the returned `ScanFrame`) and radar frames to
`radar_parser` (updating `latest_radar_reading`, exposed for future use -- NOT fed into the
existing LiDAR-only detection/clustering/tracking pipeline; see `models.radar`'s own docstring for
why), and transparently reconnects (with backoff) on a read timeout or transport error.
"""

from __future__ import annotations

import time
import uuid

from common.config import Settings, get_settings
from models.lidar import LiDARPoint
from models.radar import RadarReading
from models.scan import ScanFrame

from .serial_source import SerialLiDARDataSource
from .stm32 import (
    ChannelHealth,
    FrameCodec,
    HealthMonitor,
    LiDARMessageParser,
    RadarMessageParser,
    SequenceValidator,
    SerialTransport,
    STM32ConfigurationError,
    STM32ConnectionError,
    STM32ConnectionManager,
    STM32ProtocolError,
    STM32TimeoutError,
    UnconfiguredLiDARParser,
    UnconfiguredRadarParser,
    checksum_length_bytes,
    decode_header_field,
    verify_checksum,
)

_LIDAR_CHANNEL = "lidar"
_RADAR_CHANNEL = "radar"


def _validate_protocol_config(settings: Settings) -> None:
    """Raises `STM32ConfigurationError`, listing every still-unset piece of required wire-protocol
    configuration, if any is missing. Called at the START of `connect()`, before any serial port
    is opened -- see module docstring."""
    missing: list[str] = []

    if not settings.stm32_uart_port:
        missing.append("UART port")
    if not settings.stm32_uart_baudrate:
        missing.append("baud rate")
    if not settings.stm32_frame_start_marker and not settings.stm32_frame_length_bytes:
        missing.append("frame format (start/end marker or fixed frame length)")
    if settings.stm32_byte_order not in ("little", "big"):
        missing.append("byte order")
    if settings.stm32_message_type_offset is None:
        missing.append("message-type field offset")
    if settings.stm32_lidar_message_type is None and settings.stm32_radar_message_type is None:
        missing.append("message type IDs")
    if settings.stm32_sequence_number_offset is None:
        missing.append("sequence-number field offset")
    if not settings.stm32_crc_algorithm:
        missing.append("CRC/checksum algorithm")
    if not settings.stm32_timestamp_format:
        missing.append("timestamp format")

    if missing:
        raise STM32ConfigurationError("STM32 protocol configuration incomplete: " + ", ".join(missing) + " required.")


def _resolve_timestamp(settings: Settings, frame: bytes) -> float:
    """Per `Settings.stm32_timestamp_format`. Always returns the Edge's own receive time for now
    -- there is no configured field offset/width for an on-wire timestamp yet (only message-type
    and sequence-number have dedicated offset/width settings so far), so `"unix_epoch_ms"` /
    `"unix_epoch_s"` / `"device_uptime_ms"` cannot actually be decoded from `frame` yet even
    though the format name is already selectable in config. This is always a safe, honest value
    (never fabricated) even where it's not the sensor's own exact capture time -- see the
    hardware integration checklist for the follow-up (a timestamp field offset/width) needed to
    decode a real on-wire timestamp instead."""
    return time.time()


class STM32Source(SerialLiDARDataSource):
    """Real (once configured) STM32 hardware adapter. See module docstring.

    Args:
        settings: source of hardware/protocol config. Defaults to `common.config.get_settings()`.
        port / baudrate: override `settings.stm32_uart_port` / `stm32_uart_baudrate`.
        lidar_parser / radar_parser: override the default `Unconfigured*Parser` (which always
            raises `STM32ConfigurationError`) with a real implementation once the payload layout
            is known. Everything else in this class works unchanged once that's supplied.
        transport: override the real `SerialTransport` -- used by tests to inject a fake
            in-memory transport instead of opening an actual serial port.
    """

    def __init__(
        self,
        settings: Settings | None = None,
        port: str | None = None,
        baudrate: int | None = None,
        *,
        lidar_parser: LiDARMessageParser | None = None,
        radar_parser: RadarMessageParser | None = None,
        transport: SerialTransport | None = None,
    ) -> None:
        self.settings = settings or get_settings()
        resolved_port = port if port is not None else self.settings.stm32_uart_port
        resolved_baudrate = baudrate if baudrate is not None else self.settings.stm32_uart_baudrate
        super().__init__(port=resolved_port, baudrate=resolved_baudrate)
        # A stable, externalized identity for every hardware run -- see docs/architecture.md
        # "Session and sequence management": `source_id = "stm32_hardware"` (not per-scenario,
        # unlike SimulatorSource -- one real hardware rig has no scenario variant to encode).
        self.source_id = self.settings.hardware_source_id

        self.lidar_parser: LiDARMessageParser = lidar_parser or UnconfiguredLiDARParser()
        self.radar_parser: RadarMessageParser = radar_parser or UnconfiguredRadarParser()
        self._injected_transport = transport

        self._connection_manager: STM32ConnectionManager | None = None
        self._frame_codec: FrameCodec | None = None
        self._trailing_marker_length = 0
        self._pending_frames: list[bytes] = []
        self._lidar_sequence = SequenceValidator(modulus=self.settings.stm32_sequence_modulus)
        self._radar_sequence = SequenceValidator(modulus=self.settings.stm32_sequence_modulus)
        self.health = HealthMonitor()
        self._sequence_number = 0  # outgoing ScanFrame sequence -- independent of the wire's own
        self._last_data_at: float | None = None
        self.latest_radar_reading: RadarReading | None = None

    # --- LiDARDataSource interface -----------------------------------------------------------

    def connect(self) -> None:
        _validate_protocol_config(self.settings)

        transport = self._injected_transport or SerialTransport(
            port=self.port, baudrate=self.baudrate, timeout_s=self.settings.stm32_read_timeout_s
        )
        self._connection_manager = STM32ConnectionManager(
            transport,
            connect_timeout_s=self.settings.stm32_connect_timeout_s,
            reconnect_initial_backoff_s=self.settings.stm32_reconnect_initial_backoff_s,
            reconnect_max_backoff_s=self.settings.stm32_reconnect_max_backoff_s,
            max_reconnect_attempts=self.settings.stm32_max_reconnect_attempts,
        )
        self._connection_manager.connect()

        start_marker = bytes.fromhex(self.settings.stm32_frame_start_marker) if self.settings.stm32_frame_start_marker else None
        end_marker = bytes.fromhex(self.settings.stm32_frame_end_marker) if self.settings.stm32_frame_end_marker else None
        self._frame_codec = FrameCodec(
            start_marker=start_marker, end_marker=end_marker, frame_length=self.settings.stm32_frame_length_bytes
        )
        # A delimited frame's raw bytes (FrameCodec.extract_frames) include the trailing end
        # marker itself -- the checksum sits BEFORE it, not in the frame's last N bytes. Fixed-
        # length/marker-only framing has no such trailer, so this is 0 there.
        self._trailing_marker_length = len(end_marker) if end_marker is not None else 0

        self._connected = True
        self._last_data_at = None
        self._pending_frames = []
        self.health.mark_connected()

    def disconnect(self) -> None:
        if self._connection_manager is not None:
            self._connection_manager.disconnect()
        self._connected = False
        self.health.mark_disconnected()

    def is_connected(self) -> bool:
        return self._connected and self._connection_manager is not None and self._connection_manager.is_connected()

    def read_scan(self) -> ScanFrame:
        if not self.is_connected() or self._connection_manager is None or self._frame_codec is None:
            raise RuntimeError("STM32Source.read_scan() called before connect().")

        connected_at = self.health.connected_at or time.time()

        while True:
            try:
                chunk = self._connection_manager.read(4096)
            except STM32ConnectionError as e:
                self._reconnect(str(e))
                continue

            now = time.time()
            if chunk:
                self._frame_codec.feed(chunk)
                self._last_data_at = now
            else:
                reference = self._last_data_at or connected_at
                if now - reference > self.settings.stm32_read_timeout_s:
                    self._reconnect(f"No data received from STM32 within {self.settings.stm32_read_timeout_s}s.")
                    connected_at = self.health.connected_at or time.time()
                    continue

            # `extract_frames()` can return several complete frames in one call (e.g. a chunk
            # containing a radar frame and two LiDAR frames back to back) -- `read_scan()` only
            # returns ONE `ScanFrame` per call, so anything beyond the first LiDAR frame found is
            # queued in `_pending_frames` rather than discarded, and gets drained (no new
            # transport read needed) on the next call(s).
            self._pending_frames.extend(self._frame_codec.extract_frames())
            while self._pending_frames:
                raw_frame = self._pending_frames.pop(0)
                lidar_scan = self._handle_frame(raw_frame)
                if lidar_scan is not None:
                    return lidar_scan

    def _reconnect(self, reason: str) -> None:
        assert self._connection_manager is not None
        self.health.mark_disconnected()
        self._connected = False
        try:
            self._connection_manager.reconnect(on_attempt=lambda n, err: self.health.record_reconnect_attempt(str(err)))
        except STM32ConnectionError:
            self._connected = False
            raise
        self._connected = True
        self._last_data_at = None
        self.health.mark_connected()

    # --- Frame handling ------------------------------------------------------------------------

    def _handle_frame(self, raw_frame: bytes) -> ScanFrame | None:
        try:
            self._verify_crc(raw_frame)
            message_type = decode_header_field(
                raw_frame,
                offset=self.settings.stm32_message_type_offset,
                width=self.settings.stm32_message_type_width,
                byte_order=self.settings.stm32_byte_order,
            )
            sequence_number = decode_header_field(
                raw_frame,
                offset=self.settings.stm32_sequence_number_offset,
                width=self.settings.stm32_sequence_number_width,
                byte_order=self.settings.stm32_byte_order,
            )
        except (STM32ProtocolError, ValueError) as e:
            self.health.record_protocol_error("frame", str(e))
            return None

        timestamp = _resolve_timestamp(self.settings, raw_frame)

        if message_type == self.settings.stm32_lidar_message_type:
            return self._handle_lidar_frame(raw_frame, sequence_number=sequence_number, timestamp=timestamp)
        if message_type == self.settings.stm32_radar_message_type:
            self._handle_radar_frame(raw_frame, sequence_number=sequence_number, timestamp=timestamp)
            return None

        self.health.record_protocol_error("frame", f"Unknown message type {message_type!r}.")
        return None

    def _verify_crc(self, raw_frame: bytes) -> None:
        algorithm = self.settings.stm32_crc_algorithm
        n = checksum_length_bytes(algorithm)
        if n == 0:
            return
        trailer = self._trailing_marker_length
        if len(raw_frame) < n + trailer:
            raise STM32ProtocolError(f"Frame too short ({len(raw_frame)} bytes) to contain a {n}-byte checksum.")
        # Checksum bytes sit immediately before any trailing end marker (see `connect()`'s
        # `_trailing_marker_length` comment) -- not necessarily the frame's literal last N bytes.
        end = len(raw_frame) - trailer
        payload, checksum_bytes = raw_frame[: end - n], raw_frame[end - n : end]
        expected = int.from_bytes(checksum_bytes, byteorder=self.settings.stm32_byte_order)  # type: ignore[arg-type]
        if not verify_checksum(algorithm, payload, expected):
            raise STM32ProtocolError(f"Checksum mismatch (algorithm={algorithm}).")

    def _handle_lidar_frame(self, raw_frame: bytes, *, sequence_number: int, timestamp: float) -> ScanFrame:
        result = self._lidar_sequence.validate(sequence_number)
        self.health.record_frame(_LIDAR_CHANNEL, dropped=result.dropped_count, duplicate_or_out_of_order=result.is_duplicate_or_out_of_order)

        points: list[LiDARPoint] = self.lidar_parser.parse(raw_frame, timestamp=timestamp, sequence_number=sequence_number)

        frame = ScanFrame(
            scan_id=str(uuid.uuid4()),
            sequence_number=self._sequence_number,
            source_id=self.source_id,
            timestamp=timestamp,
            points=points,
        )
        self._sequence_number += 1
        return frame

    def _handle_radar_frame(self, raw_frame: bytes, *, sequence_number: int, timestamp: float) -> None:
        result = self._radar_sequence.validate(sequence_number)
        self.health.record_frame(_RADAR_CHANNEL, dropped=result.dropped_count, duplicate_or_out_of_order=result.is_duplicate_or_out_of_order)

        self.latest_radar_reading = self.radar_parser.parse(
            raw_frame, timestamp=timestamp, sequence_number=sequence_number, source_id=self.source_id
        )

"""`ESP32SerialSource` -- the real hardware `SensorSource` for the USB-serial topology.

    LiDAR -> STM32 -> ESP32 -> USB serial -> [ SerialLineReader -> MeasurementParser
                                               -> ScanFrameBuilder ] -> ScanFrame

Binds the three single-purpose pieces of this package into one object satisfying
`datasources.SensorSource`, so `scripts/serve_unity_bridge.py` runs the FULL existing perception
pipeline (preprocessing -> coordinates -> clustering -> classification -> tracking -> collision /
clearance / mapping -> LiveState) against real measurements with no pipeline changes at all. That
is the entire point of the `SensorSource` seam: the stages downstream cannot tell a scan from
this class apart from a simulated one.

RELATIONSHIP TO THE OTHER TWO HARDWARE ADAPTERS
-----------------------------------------------
* `datasources.stm32_source.STM32Source` -- binary UART framing/CRC/sequence scaffolding whose
  payload parser is deliberately unimplemented, because that wire format is unknown. Untouched.
* `datasources.esp32.ESP32Source` -- the Wi-Fi link carrying ALREADY-PROCESSED frames, where the
  Edge runs no perception. Untouched.
* **This class** -- the ASCII USB-serial link carrying RAW measurements, where the Edge runs all
  the perception. The format is known and fixed, so nothing here is guessed.

NO FABRICATED DATA
------------------
There is no synthetic fallback anywhere in this class. If the port is closed, or no complete scan
arrives inside `esp32_serial_scan_timeout_s`, `read_scan()` raises -- it never returns invented
points to keep the dashboard looking busy. Simulation lives in a completely separate source
(`simulator.SimulatorSource`, selected by `DATA_SOURCE=simulation`) and cannot be reached from
this code path.
"""

from __future__ import annotations

import time
from collections import deque

from common.config import Settings, get_settings
from common.logging import get_logger
from datasources.base import LiDARDataSource
from models.scan import ScanFrame

from .errors import ESP32SerialConnectionError, ESP32SerialTimeoutError
from .frame_builder import ScanFrameBuilder
from .parser import MeasurementParser
from .reader import SerialLineReader

logger = get_logger(__name__)

# How long `read_scan()` sleeps when the line queue is momentarily empty. Long enough not to spin
# a core, far shorter than one revolution so it adds no meaningful latency.
_IDLE_POLL_S = 0.002


class ESP32SerialSource(LiDARDataSource):
    """A `SensorSource` producing real 360 degree `ScanFrame`s from the ESP32's USB serial link."""

    def __init__(self, settings: Settings | None = None) -> None:
        self._settings = settings or get_settings()
        s = self._settings

        self.source_id = s.esp32_serial_source_id
        self._scan_timeout_s = s.esp32_serial_scan_timeout_s
        self._log_every_n_scans = s.esp32_serial_log_every_n_scans

        self._reader = SerialLineReader(
            port=s.esp32_serial_port,
            baudrate=s.esp32_serial_baudrate,
            read_timeout_s=s.esp32_serial_read_timeout_s,
            max_buffered_lines=s.esp32_serial_max_buffered_lines,
            reconnect_initial_backoff_s=s.esp32_serial_reconnect_initial_backoff_s,
            reconnect_max_backoff_s=s.esp32_serial_reconnect_max_backoff_s,
            max_reconnect_attempts=s.esp32_serial_max_reconnect_attempts,
        )
        self._parser = MeasurementParser()
        self._builder = ScanFrameBuilder(
            source_id=self.source_id,
            wrap_threshold_deg=s.esp32_serial_wrap_threshold_deg,
            min_points_per_scan=s.esp32_serial_min_points_per_scan,
            max_points_per_scan=s.esp32_serial_max_points_per_scan,
            max_scan_duration_s=s.esp32_serial_max_scan_duration_s,
            duplicate_angle_policy=s.esp32_serial_duplicate_angle_policy,
        )

        self._started = False
        # Tracks link transitions so a reconnect can drop the half-accumulated scan that spans it.
        self._was_connected = False
        # Lines drained from the reader but not yet consumed. `read_scan()` returns the moment a
        # revolution completes, and one drain can easily contain several revolutions' worth of
        # measurements (the reader batches whatever the OS buffered). Without this backlog those
        # surplus lines would be dropped on the floor by the early return -- silently losing every
        # scan after the first in each batch.
        self._line_backlog: deque[str] = deque()

    # -- SensorSource contract ------------------------------------------------------------

    def connect(self) -> None:
        """Starts the background reader.

        Returns as soon as the thread is running -- deliberately WITHOUT waiting for the port to
        open. The ESP32 may legitimately be plugged in after the Edge starts, and the reader's own
        backoff loop handles that; blocking here would mean the Edge could not start at all until
        hardware appeared. `is_connected()` and `stats` report the real link state throughout.
        """
        self._reader.start()
        self._started = True
        logger.info(
            "[SERIAL] ESP32 serial source started (port=%s, baud=%d). Waiting for measurements.",
            self._settings.esp32_serial_port, self._settings.esp32_serial_baudrate,
        )

    def disconnect(self) -> None:
        self._reader.stop()
        self._started = False

    def is_connected(self) -> bool:
        """Whether the source can currently produce scans.

        True while the reader thread is running -- including while it is between reconnect
        attempts -- so `stream()` keeps polling across a transient unplug instead of ending the
        run. Use `stats["reader"]["connected"]` for the underlying port state itself.
        """
        return self._started and self._reader.is_running

    def read_scan(self) -> ScanFrame:
        """Blocks until one complete revolution has been assembled, and returns it.

        Raises `ESP32SerialTimeoutError` if no complete scan arrives within
        `esp32_serial_scan_timeout_s`, and `ESP32SerialConnectionError` if called before
        `connect()`. Never returns a partial or synthesised frame.
        """
        if not self._started:
            raise ESP32SerialConnectionError(
                "ESP32SerialSource.read_scan() called before connect()."
            )

        deadline = time.monotonic() + self._scan_timeout_s

        while True:
            self._handle_link_transitions()

            if not self._line_backlog:
                self._line_backlog.extend(self._reader.read_lines())

            # Consume the backlog one line at a time, so returning mid-batch leaves the remainder
            # queued for the next call rather than discarding it.
            while self._line_backlog:
                measurement = self._parser.parse_line(self._line_backlog.popleft())
                if measurement is None:
                    continue
                frame = self._builder.add(measurement)
                if frame is not None:
                    self._log_scan(frame)
                    return frame

            # Nothing new: give a stalled sweep a chance to force-complete rather than hanging on
            # a sensor that stopped mid-revolution.
            stale_frame = self._builder.flush_if_stale()
            if stale_frame is not None:
                logger.warning(
                    "[SCAN] Force-completed a stalled scan after %.1fs with %d point(s) -- "
                    "is the LiDAR still spinning?",
                    self._settings.esp32_serial_max_scan_duration_s, stale_frame.point_count,
                )
                self._log_scan(stale_frame)
                return stale_frame

            if time.monotonic() >= deadline:
                raise ESP32SerialTimeoutError(self._timeout_message())

            time.sleep(_IDLE_POLL_S)

    # -- diagnostics ----------------------------------------------------------------------

    @property
    def stats(self) -> dict[str, object]:
        """Real, measured link/parser/frame health -- every value counted, none estimated."""
        return {
            "reader": self._reader.stats.snapshot(),
            "parser": self._parser.stats.snapshot(),
            "frames": self._builder.stats.snapshot(),
            "buffered_lines": self._reader.buffered_line_count + len(self._line_backlog),
        }

    @property
    def measured_scan_rate_hz(self) -> float | None:
        """Scan rate measured from real inter-scan intervals, or `None` before the second scan."""
        return self._builder.stats.measured_scan_rate_hz

    # -- internals ------------------------------------------------------------------------

    def _handle_link_transitions(self) -> None:
        """Drops a scan straddling a reconnect, so two half-revolutions are never merged."""
        connected_now = self._reader.is_connected
        if connected_now and not self._was_connected:
            self._builder.reset()
            self._line_backlog.clear()
        self._was_connected = connected_now

    def _log_scan(self, frame: ScanFrame) -> None:
        """Periodic (never per-scan) health summary -- see requirement 19 on log volume."""
        if self._log_every_n_scans <= 0:
            return
        stats = self._builder.stats
        if stats.scans_completed % self._log_every_n_scans != 0:
            return

        rate = stats.measured_scan_rate_hz
        parser_stats = self._parser.stats
        logger.info(
            "[SCAN] Frame completed: %d points | rate: %s | scans: %d | parser accepted %d / "
            "rejected %d %s",
            frame.point_count,
            f"{rate:.1f} Hz" if rate is not None else "--",
            stats.scans_completed,
            parser_stats.accepted,
            parser_stats.rejected,
            dict(parser_stats.rejections_by_reason) or "",
        )

    def _timeout_message(self) -> str:
        """Explains WHY no scan arrived, using only measured counters -- not a generic message."""
        reader = self._reader.stats
        parser = self._parser.stats
        builder = self._builder.stats

        if not reader.connected:
            reason = (
                f"the serial port {reader.port} is not open"
                + (f" (last error: {reader.last_error})" if reader.last_error else "")
                + ". Is the ESP32 plugged in, is the port correct "
                "(`python scripts/sniff_esp32.py --list`), and is another program "
                "(Arduino Serial Monitor, PuTTY) holding the port?"
            )
        elif reader.lines_read == 0:
            reason = (
                f"the port {reader.port} is open but no data has arrived. Is the ESP32 "
                f"transmitting, and is the baud rate correct (configured: {reader.baudrate})?"
            )
        elif parser.accepted == 0:
            reason = (
                f"{reader.lines_read} line(s) arrived but none parsed as 'A:<deg> , D:<mm>' "
                f"(rejections: {dict(parser.rejections_by_reason)}). Check the baud rate and the "
                "firmware's output format."
            )
        elif builder.scans_completed == 0 and builder.scans_discarded_too_few_points > 0:
            reason = (
                f"{builder.scans_discarded_too_few_points} scan boundary/boundaries were detected "
                f"but each had fewer than the required minimum points. Lower "
                "LIDAR_ESP32_SERIAL_MIN_POINTS_PER_SCAN if your sensor emits few points per "
                "revolution."
            )
        else:
            reason = (
                f"measurements are arriving ({parser.accepted} accepted) but no full revolution "
                "completed in time. Is the LiDAR spinning?"
            )

        return f"No complete scan within {self._scan_timeout_s:.1f}s: {reason}"


__all__ = ["ESP32SerialSource"]

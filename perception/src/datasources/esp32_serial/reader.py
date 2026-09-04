"""Background serial-line reader for the ESP32 USB link.

Owns exactly one responsibility: get complete text lines off the COM port and into an in-memory
queue, forever, without ever blocking or crashing the perception pipeline that consumes it.

WHY A THREAD
------------
`pyserial` reads are blocking, and the Edge loop must not stall on a quiet port. A dedicated
daemon thread does the blocking reads and hands lines to the pipeline through a bounded queue, so
acquisition and processing are decoupled: a slow scan (clustering, classification) never causes
missed bytes, and a quiet sensor never freezes the pipeline (see PROJECT requirement 17,
"Do not let frontend rendering block serial acquisition").

BACKPRESSURE POLICY
-------------------
The queue is bounded (`max_buffered_lines`). When it fills -- the consumer has fallen behind the
sensor -- the OLDEST lines are dropped, not the newest. This is the correct choice for a
real-time system: stale measurements describe where the world *was*, and an unbounded queue would
grow until the process died while the dashboard fell further and further behind live. Drops are
counted (`ReaderStats.lines_dropped_backpressure`) and warned about, never silent.

RECONNECTION
------------
Unplugging the ESP32 raises from the read call. The thread closes the handle, reports
DISCONNECTED with the real reason, sleeps an exponentially-growing backoff (capped), and retries
-- indefinitely by default. The perception process stays alive and the dashboard shows the link
as down; nothing is fabricated to fill the gap.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass

from common.logging import get_logger

from .errors import ESP32SerialConfigurationError

logger = get_logger(__name__)

# Bytes accumulated without ever seeing a line terminator before the partial-line buffer is
# discarded. Protects against a wrong baud rate (which yields a torrent of bytes containing no
# newline) growing one "line" without bound.
_MAX_PARTIAL_LINE_BYTES = 4096


@dataclass
class ReaderStats:
    """Live link health, surfaced to the dashboard through `ESP32SerialSource.stats`."""

    connected: bool = False
    port: str | None = None
    baudrate: int | None = None
    connected_at: float | None = None
    last_line_at: float | None = None
    lines_read: int = 0
    bytes_read: int = 0
    lines_dropped_backpressure: int = 0
    partial_lines_discarded: int = 0
    connect_attempts: int = 0
    reconnect_count: int = 0
    last_error: str | None = None

    def snapshot(self) -> dict[str, object]:
        return {
            "connected": self.connected,
            "port": self.port,
            "baudrate": self.baudrate,
            "connected_at": self.connected_at,
            "last_line_at": self.last_line_at,
            "lines_read": self.lines_read,
            "bytes_read": self.bytes_read,
            "lines_dropped_backpressure": self.lines_dropped_backpressure,
            "partial_lines_discarded": self.partial_lines_discarded,
            "connect_attempts": self.connect_attempts,
            "reconnect_count": self.reconnect_count,
            "last_error": self.last_error,
        }


class SerialLineReader:
    """Reads newline-delimited text from a serial port on a background daemon thread.

    Thread-safe: `read_lines()` and `stats` may be called from the consumer thread while the
    reader thread is running.
    """

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        read_timeout_s: float = 0.1,
        max_buffered_lines: int = 20000,
        reconnect_initial_backoff_s: float = 1.0,
        reconnect_max_backoff_s: float = 10.0,
        max_reconnect_attempts: int | None = None,
        encoding: str = "ascii",
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._read_timeout_s = read_timeout_s
        self._reconnect_initial_backoff_s = reconnect_initial_backoff_s
        self._reconnect_max_backoff_s = reconnect_max_backoff_s
        self._max_reconnect_attempts = max_reconnect_attempts
        self._encoding = encoding

        self._lines: deque[str] = deque(maxlen=max_buffered_lines)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None  # pyserial Serial handle, owned solely by the reader thread

        self.stats = ReaderStats(port=port, baudrate=baudrate)

    # -- lifecycle ------------------------------------------------------------------------

    def start(self) -> None:
        """Starts the reader thread.

        Raises `ESP32SerialConfigurationError` immediately if `pyserial` is unavailable -- a
        missing dependency is a configuration fault that no amount of retrying can fix. Failure to
        *open* the port, by contrast, is a normal transient condition handled by the reconnect
        loop (the ESP32 may simply not be plugged in yet).
        """
        try:
            import serial  # noqa: F401  -- imported for availability check only
        except ImportError as e:
            raise ESP32SerialConfigurationError(
                "pyserial is not installed, so the ESP32 USB serial link cannot be opened. "
                "Install it with:  pip install pyserial"
            ) from e

        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="esp32-serial-reader", daemon=True)
        self._thread.start()

    def stop(self, join_timeout_s: float = 2.0) -> None:
        """Signals the reader thread to stop and waits briefly for it to exit.

        Cooperative cancellation via an Event -- never `Thread.abort()`-style termination, which
        can leave the port handle in an undefined state.
        """
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=join_timeout_s)
        self._thread = None

    @property
    def is_connected(self) -> bool:
        return self.stats.connected

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -- consumer API ---------------------------------------------------------------------

    def read_lines(self, max_lines: int | None = None) -> list[str]:
        """Drains up to `max_lines` buffered lines (all of them when `None`). Never blocks."""
        with self._lock:
            if max_lines is None or max_lines >= len(self._lines):
                drained = list(self._lines)
                self._lines.clear()
                return drained
            return [self._lines.popleft() for _ in range(max_lines)]

    @property
    def buffered_line_count(self) -> int:
        with self._lock:
            return len(self._lines)

    # -- reader thread --------------------------------------------------------------------

    def _run(self) -> None:
        backoff_s = self._reconnect_initial_backoff_s
        attempts = 0

        while not self._stop_event.is_set():
            if not self._open_port():
                attempts += 1
                if self._max_reconnect_attempts is not None and attempts >= self._max_reconnect_attempts:
                    logger.error(
                        "[SERIAL] Giving up on %s after %d failed connection attempt(s): %s",
                        self._port, attempts, self.stats.last_error,
                    )
                    return
                # Interruptible sleep -- stop() must not have to wait out a 10s backoff.
                self._stop_event.wait(backoff_s)
                backoff_s = min(backoff_s * 2.0, self._reconnect_max_backoff_s)
                continue

            # Connected: reset the backoff so the *next* unrelated drop retries quickly again.
            backoff_s = self._reconnect_initial_backoff_s
            attempts = 0
            self._pump()  # returns on disconnect or stop

        self._close_port()

    def _open_port(self) -> bool:
        import serial

        self.stats.connect_attempts += 1
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._read_timeout_s,
            )
        except Exception as e:  # noqa: BLE001 -- pyserial raises SerialException and OSError variants
            self.stats.connected = False
            self.stats.last_error = f"{type(e).__name__}: {e}"
            # Debug, not error: "not plugged in yet" is an expected steady state while waiting,
            # and logging it at ERROR every backoff would flood the terminal (requirement 19).
            logger.debug("[SERIAL] Could not open %s @ %d: %s", self._port, self._baudrate, e)
            return False

        if self.stats.connect_attempts > 1:
            self.stats.reconnect_count += 1
        self.stats.connected = True
        self.stats.connected_at = time.time()
        self.stats.last_error = None
        logger.info("[SERIAL] Connected %s @ %d baud.", self._port, self._baudrate)
        return True

    def _pump(self) -> None:
        """Blocking read loop for one connected session. Returns when the link drops or we stop."""
        buffer = bytearray()

        while not self._stop_event.is_set():
            try:
                # `in_waiting` lets one syscall collect a whole burst; `or 1` makes the call block
                # up to `read_timeout_s` when the port is quiet instead of spinning the CPU.
                pending = self._serial.in_waiting
                chunk = self._serial.read(pending or 1)
            except Exception as e:  # noqa: BLE001 -- device unplugged mid-read, port vanished, etc.
                self.stats.connected = False
                self.stats.last_error = f"{type(e).__name__}: {e}"
                logger.warning("[SERIAL] Link lost on %s: %s -- reconnecting.", self._port, e)
                self._close_port()
                return

            if not chunk:
                continue  # read timeout with no data -- normal on a quiet port

            self.stats.bytes_read += len(chunk)
            buffer.extend(chunk)

            if b"\n" not in buffer:
                if len(buffer) > _MAX_PARTIAL_LINE_BYTES:
                    # No terminator in 4 KB: almost always a baud-rate mismatch. Drop the garbage
                    # rather than letting one pseudo-line grow without bound.
                    self.stats.partial_lines_discarded += 1
                    logger.warning(
                        "[SERIAL] Discarded %d buffered bytes with no line terminator on %s -- "
                        "is the baud rate correct? (configured: %d)",
                        len(buffer), self._port, self._baudrate,
                    )
                    buffer.clear()
                continue

            # Split on the terminator, keeping any trailing partial line for the next read.
            *complete, remainder = buffer.split(b"\n")
            buffer = bytearray(remainder)

            decoded = [
                raw.decode(self._encoding, errors="replace").strip("\r").strip()
                for raw in complete
            ]
            self._enqueue(decoded)

    def _enqueue(self, lines: list[str]) -> None:
        if not lines:
            return
        with self._lock:
            overflow = len(self._lines) + len(lines) - self._lines.maxlen
            if overflow > 0:
                # deque(maxlen=...) already evicts from the left; count it so the drop is visible.
                self.stats.lines_dropped_backpressure += overflow
            self._lines.extend(lines)
        self.stats.lines_read += len(lines)
        self.stats.last_line_at = time.time()

    def _close_port(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 -- closing a already-vanished handle must not raise
                pass
            self._serial = None
        self.stats.connected = False


__all__ = ["SerialLineReader", "ReaderStats"]

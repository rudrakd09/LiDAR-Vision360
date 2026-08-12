"""The real-time Python -> Unity streaming servers (Phase 12).

    Perception Engine
         |
    PerceptionStreamServer.publish(message)      -- never blocks the caller
         |
    per-client LatestFrameQueue                   -- bounded, drop-oldest (see .latest_frame_queue)
         |
    per-client sender thread                        -- the ONLY code path that ever calls socket.sendall()
         |
    TCP
         |
    Unity

Conceptually: `server = PerceptionStreamServer(settings); server.start(); server.publish(message);
server.stop()` (see docs/communication.md "Python API"). Not tightly coupled to any one
perception module -- `publish()` takes an already-built message dict (see `streaming.protocol`),
so a caller assembles whatever message it wants from whatever pipeline stages it has wired up;
this class only owns connection lifecycle, per-client queueing, and sending.

**Non-blocking design** (see docs/communication.md "Non-blocking design"): `publish()` only ever
enqueues into each connected client's own `LatestFrameQueue` (a `deque.append`, microseconds) and
returns -- it never calls a blocking socket operation itself. Each client's own dedicated sender
thread is the only code path that ever blocks on `sendall()`, so one slow client can never stall
another client, let alone the perception pipeline calling `publish()`.

`RawLidarStreamServer` serves the legacy `<START>`/`<END>` raw protocol on its own port with the
exact same non-blocking-publish / bounded-per-client-queue / dedicated-sender-thread design --
Phase 11's own first cut of this (`scripts/serve_unity_bridge.py`'s `ClientBroadcaster`) called
`sendall()` synchronously from the main loop, which had the same slow-client-blocks-everything
risk this phase explicitly warns against; this supersedes it.
"""

from __future__ import annotations

import socket
import threading
import time

from common.config import Settings, get_settings
from common.logging import get_logger

from .framing import encode_message
from .latest_frame_queue import LatestFrameQueue
from .protocol import MessageType, build_heartbeat_message

logger = get_logger(__name__)


class PerceptionStreamServer:
    """Serves the structured, versioned JSON protocol (`Settings.streaming_json_port`)."""

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()
        self._server_socket: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._running = False
        self._start_time: float | None = None

        self._clients_lock = threading.Lock()
        self._clients: dict[socket.socket, dict] = {}

        self._frames_sent = 0

    @property
    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    @property
    def frames_sent(self) -> int:
        return self._frames_sent

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.settings.streaming_host, self.settings.streaming_json_port))
        self._server_socket.listen(8)
        self._running = True
        self._start_time = time.time()

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        if self.settings.streaming_heartbeat_interval_s > 0:
            self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._heartbeat_thread.start()

        logger.info("[STREAM] JSON server listening on %s:%d", self.settings.streaming_host, self.settings.streaming_json_port)

    def stop(self) -> None:
        self._running = False
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self._clients.keys())
        for conn in clients:
            self._disconnect_client(conn)
        logger.info("[STREAM] JSON server stopped (%d frame(s) sent).", self._frames_sent)

    def publish(self, message: dict) -> None:
        """Enqueue `message` for every currently-connected client. Never blocks. A message that
        arrives while a specific client's queue is already full evicts *that client's own*
        oldest still-pending message (see `LatestFrameQueue`) -- other clients are unaffected,
        and the perception pipeline calling this is never affected either way."""
        max_bytes = self.settings.streaming_max_message_bytes
        if max_bytes > 0:
            try:
                encoded_size = len(encode_message(message))
            except (TypeError, ValueError) as e:
                logger.warning("[STREAM] Message failed to serialize, dropping: %s", e)
                return
            if encoded_size > max_bytes:
                logger.warning("[STREAM] Message too large (%d bytes > %d limit), dropping.", encoded_size, max_bytes)
                return

        with self._clients_lock:
            clients = list(self._clients.values())
        for client in clients:
            client["queue"].put(message)

        if message.get("message_type") == MessageType.PERCEPTION_FRAME.value:
            self._frames_sent += 1

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, address = self._server_socket.accept()
            except OSError:
                return  # socket closed -- normal shutdown
            self._add_client(conn, address)

    def _add_client(self, conn: socket.socket, address) -> None:
        client_queue = LatestFrameQueue(maxsize=self.settings.streaming_max_outgoing_queue)
        sender_thread = threading.Thread(target=self._sender_loop, args=(conn, client_queue), daemon=True)

        with self._clients_lock:
            self._clients[conn] = {"queue": client_queue, "thread": sender_thread, "address": address}

        sender_thread.start()
        logger.info("[STREAM] Client connected: %s (%d total).", address, self.client_count)

    def _sender_loop(self, conn: socket.socket, client_queue: LatestFrameQueue) -> None:
        while self._running:
            message = client_queue.get(timeout=1.0)
            if message is None:
                continue
            try:
                conn.sendall(encode_message(message))
            except OSError:
                break
        self._disconnect_client(conn)

    def _disconnect_client(self, conn: socket.socket) -> None:
        with self._clients_lock:
            info = self._clients.pop(conn, None)
        if info is None:
            return  # already removed via another path (e.g. stop() and the sender thread's own error handler racing)
        try:
            conn.close()
        except OSError:
            pass
        logger.info(
            "[STREAM] Client disconnected: %s (%d total remaining, %d message(s) dropped for this client).",
            info["address"], self.client_count, info["queue"].dropped_count,
        )

    def _heartbeat_loop(self) -> None:
        interval = self.settings.streaming_heartbeat_interval_s
        while self._running:
            time.sleep(interval)
            if not self._running:
                return
            uptime = (time.time() - self._start_time) if self._start_time else 0.0
            self.publish(build_heartbeat_message(uptime, self._frames_sent, self.client_count))


class RawLidarStreamServer:
    """Serves the legacy `<START>`/`<END>` raw protocol (`Settings.streaming_raw_port`),
    byte-compatible with the existing, unmodified `LidarTCPClient.cs`/`LidarSerialReader.cs`.
    Same non-blocking-publish / bounded-queue / dedicated-sender-thread design as
    `PerceptionStreamServer`, applied to raw scans instead of JSON messages.
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()
        self._server_socket: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._running = False

        self._clients_lock = threading.Lock()
        self._clients: dict[socket.socket, dict] = {}

    @property
    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def start(self) -> None:
        if self._running:
            return
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.settings.streaming_host, self.settings.streaming_raw_port))
        self._server_socket.listen(8)
        self._running = True

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        logger.info("[STREAM] Raw server listening on %s:%d", self.settings.streaming_host, self.settings.streaming_raw_port)

    def stop(self) -> None:
        self._running = False
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self._clients.keys())
        for conn in clients:
            self._disconnect_client(conn)
        logger.info("[STREAM] Raw server stopped.")

    def publish_scan(self, points) -> None:
        """Enqueue one scan's raw points for every connected client -- never blocks."""
        with self._clients_lock:
            clients = list(self._clients.values())
        if not clients:
            return
        payload = format_raw_scan(points)
        for client in clients:
            client["queue"].put(payload)

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, address = self._server_socket.accept()
            except OSError:
                return
            self._add_client(conn, address)

    def _add_client(self, conn: socket.socket, address) -> None:
        client_queue = LatestFrameQueue(maxsize=self.settings.streaming_max_outgoing_queue)
        sender_thread = threading.Thread(target=self._sender_loop, args=(conn, client_queue), daemon=True)
        with self._clients_lock:
            self._clients[conn] = {"queue": client_queue, "thread": sender_thread, "address": address}
        sender_thread.start()
        logger.info("[STREAM] Raw client connected: %s (%d total).", address, self.client_count)

    def _sender_loop(self, conn: socket.socket, client_queue: LatestFrameQueue) -> None:
        while self._running:
            payload = client_queue.get(timeout=1.0)
            if payload is None:
                continue
            try:
                conn.sendall(payload)
            except OSError:
                break
        self._disconnect_client(conn)

    def _disconnect_client(self, conn: socket.socket) -> None:
        with self._clients_lock:
            info = self._clients.pop(conn, None)
        if info is None:
            return
        try:
            conn.close()
        except OSError:
            pass
        logger.info("[STREAM] Raw client disconnected: %s (%d total remaining).", info["address"], self.client_count)


def format_raw_scan(points) -> bytes:
    """`<START>` / `angle,distance` / `<END>`, one line each, matching the existing
    `LidarTCPClient.cs`/`LidarSerialReader.cs` framing exactly.

    **Unit contract** (see docs/unity.md "Legacy raw protocol" for the full derivation): the
    existing `LidarCubes.cs` recovers meters via `distance / 10`, i.e. the wire value is meters
    x 10 (decimeters) -- *not* centimeters, despite that script's own comment saying "cm". This
    sender matches the code Unity actually runs, not the comment.
    """
    lines = ["<START>"]
    for point in points:
        lines.append(f"{point.angle:.2f},{point.distance * 10.0:.2f}")
    lines.append("<END>")
    return ("\n".join(lines) + "\n").encode("ascii")

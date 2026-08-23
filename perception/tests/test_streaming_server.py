"""Integration tests for streaming.server: PerceptionStreamServer / RawLidarStreamServer against
real TCP sockets -- connect/disconnect, publish/receive, heartbeat, non-blocking publish under a
stalled client, oversized-message rejection, and multiple simultaneous clients.
"""

import json
import socket
import time

import pytest

from common.config import Settings
from models.lidar import CartesianPoint
from models.tracking import TrackedScan
from streaming.protocol import MessageType, build_heartbeat_message, build_perception_frame_message
from streaming.server import PerceptionStreamServer, RawLidarStreamServer, format_raw_scan


def _free_port() -> int:
    """Ask the OS for an available port -- avoids hard-coded port collisions between tests
    running in the same process/machine."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _settings(**overrides) -> Settings:
    defaults = dict(
        streaming_json_port=_free_port(), streaming_raw_port=_free_port(),
        streaming_heartbeat_interval_s=0.0,  # off by default in tests -- explicit tests enable it
        streaming_max_outgoing_queue=2,
    )
    defaults.update(overrides)
    return Settings(_env_file=None, **defaults)


def _scan(seq: int = 1) -> TrackedScan:
    return TrackedScan(
        scan_id="s", sequence_number=seq, source_id="unit-test", timestamp=time.time(),
        objects=[], noise_points=[], object_count=0, noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _recv_lines(sock: socket.socket, count: int, timeout: float = 3.0) -> list[dict]:
    sock.settimeout(timeout)
    buf = b""
    messages = []
    deadline = time.time() + timeout
    while len(messages) < count and time.time() < deadline:
        try:
            chunk = sock.recv(1 << 20)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if line.strip():
                messages.append(json.loads(line))
    return messages


@pytest.fixture
def json_server():
    settings = _settings()
    server = PerceptionStreamServer(settings=settings)
    server.start()
    yield server, settings
    server.stop()


@pytest.fixture
def raw_server():
    settings = _settings()
    server = RawLidarStreamServer(settings=settings)
    server.start()
    yield server, settings
    server.stop()


class TestClientConnectDisconnect:
    def test_client_connect_is_counted(self, json_server):
        server, settings = json_server
        assert server.client_count == 0
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        assert server.client_count == 1
        sock.close()

    def test_client_disconnect_is_detected_after_a_publish(self, json_server):
        server, settings = json_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        sock.close()
        # detection happens on the next send attempt -- publish once to trigger it
        for _ in range(20):
            server.publish(build_perception_frame_message(_scan(), settings=settings))
            if server.client_count == 0:
                break
            time.sleep(0.1)
        assert server.client_count == 0

    def test_multiple_simultaneous_clients(self, json_server):
        server, settings = json_server
        socks = [socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3) for _ in range(3)]
        time.sleep(0.2)
        assert server.client_count == 3
        for s in socks:
            s.close()


class TestPublishAndReceive:
    def test_published_frame_is_received(self, json_server):
        server, settings = json_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)

        server.publish(build_perception_frame_message(_scan(seq=7), settings=settings))
        messages = _recv_lines(sock, count=1)

        assert len(messages) == 1
        assert messages[0]["message_type"] == MessageType.PERCEPTION_FRAME.value
        assert messages[0]["frame_id"] == 7
        sock.close()

    def test_frames_sent_counter_increments(self, json_server):
        server, settings = json_server
        for i in range(5):
            server.publish(build_perception_frame_message(_scan(seq=i), settings=settings))
        assert server.frames_sent == 5

    def test_publish_with_no_clients_does_not_raise(self, json_server):
        server, settings = json_server
        server.publish(build_perception_frame_message(_scan(), settings=settings))  # must not raise


class TestHeartbeat:
    def test_heartbeat_sent_on_interval(self):
        settings = _settings(streaming_heartbeat_interval_s=0.3)
        server = PerceptionStreamServer(settings=settings)
        server.start()
        try:
            sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
            time.sleep(0.2)
            messages = _recv_lines(sock, count=2, timeout=3.0)
            assert any(m["message_type"] == MessageType.HEARTBEAT.value for m in messages)
            sock.close()
        finally:
            server.stop()

    def test_no_heartbeat_when_interval_is_zero(self, json_server):
        server, settings = json_server  # streaming_heartbeat_interval_s=0.0 by default in this fixture
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.5)
        sock.settimeout(0.3)
        with pytest.raises(socket.timeout):
            sock.recv(1024)
        sock.close()


class TestNonBlockingPublish:
    def test_publish_stays_fast_with_a_stalled_non_reading_client(self, json_server):
        server, settings = json_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        # deliberately never read from `sock` -- simulates a stalled/slow Unity client

        start = time.perf_counter()
        for i in range(200):
            server.publish(build_perception_frame_message(_scan(seq=i), settings=settings))
        elapsed = time.perf_counter() - start

        assert elapsed < 1.0  # 200 publishes against a stalled client must not block the caller
        sock.close()

    def test_stale_messages_are_dropped_not_queued_forever(self, json_server):
        server, settings = json_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)

        for i in range(50):
            server.publish(build_perception_frame_message(_scan(seq=i), settings=settings))

        # Only ever a handful of messages could possibly be delivered/queued -- most were dropped.
        messages = _recv_lines(sock, count=50, timeout=1.0)
        assert len(messages) < 50
        sock.close()


class TestOversizedMessageRejection:
    def test_message_exceeding_max_bytes_is_dropped(self):
        settings = _settings(streaming_max_message_bytes=100)
        server = PerceptionStreamServer(settings=settings)
        server.start()
        try:
            sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
            time.sleep(0.2)

            huge_message = build_perception_frame_message(_scan(), settings=settings)
            huge_message["data"]["padding"] = "x" * 10_000  # comfortably over the 100-byte limit
            server.publish(huge_message)

            sock.settimeout(0.5)
            with pytest.raises(socket.timeout):
                sock.recv(1024)  # nothing should have been sent
            sock.close()
        finally:
            server.stop()

    def test_message_within_limit_is_sent(self):
        settings = _settings(streaming_max_message_bytes=1_000_000)
        server = PerceptionStreamServer(settings=settings)
        server.start()
        try:
            sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
            time.sleep(0.2)
            server.publish(build_perception_frame_message(_scan(), settings=settings))
            messages = _recv_lines(sock, count=1)
            assert len(messages) == 1
            sock.close()
        finally:
            server.stop()


class TestRawLidarServer:
    def test_raw_scan_received_in_legacy_format(self, raw_server):
        server, settings = raw_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_raw_port), timeout=3)
        time.sleep(0.2)

        points = [CartesianPoint(angle=float(i), distance=5.0, timestamp=0.0, x=1.0, y=1.0) for i in range(3)]
        server.publish_scan(points)

        sock.settimeout(2.0)
        data = sock.recv(65536).decode("ascii")
        assert data.startswith("<START>\n")
        assert data.rstrip().endswith("<END>")
        assert "0.00,50.00" in data  # 5.0m * 10 = 50.0 decimeters
        sock.close()

    def test_publish_with_no_clients_does_not_raise(self, raw_server):
        server, settings = raw_server
        server.publish_scan([])  # must not raise


class TestFailureScenarios:
    """See docs/communication.md "Failure testing" -- each of these mirrors one of that
    section's numbered scenarios not already covered by a more specific test class above."""

    def test_processing_continues_with_no_client_ever_connected(self, json_server):
        # "Unity starts after Python" / "Python processing continues while Unity is unavailable."
        server, settings = json_server
        for i in range(20):
            server.publish(build_perception_frame_message(_scan(seq=i), settings=settings))
        assert server.frames_sent == 20  # every scan was still "processed" (published) despite zero clients

    def test_late_connecting_client_receives_subsequent_frames(self, json_server):
        # "Unity starts after Python": frames published before any client connects are simply not
        # delivered (nobody was listening) -- but the *next* one, after a client connects, must
        # arrive normally. No crash, no special-casing needed on the publisher's side.
        server, settings = json_server
        server.publish(build_perception_frame_message(_scan(seq=1), settings=settings))  # nobody listening yet

        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        server.publish(build_perception_frame_message(_scan(seq=2), settings=settings))

        messages = _recv_lines(sock, count=1)
        assert len(messages) == 1
        assert messages[0]["frame_id"] == 2
        sock.close()

    def test_reconnect_after_disconnect_receives_new_frames(self, json_server):
        # "Network interruption" / client reconnecting after a drop.
        server, settings = json_server

        first = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        first.close()
        for _ in range(20):
            server.publish(build_perception_frame_message(_scan(seq=100), settings=settings))
            if server.client_count == 0:
                break
            time.sleep(0.1)
        assert server.client_count == 0

        second = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        assert server.client_count == 1
        server.publish(build_perception_frame_message(_scan(seq=101), settings=settings))
        messages = _recv_lines(second, count=1)
        assert len(messages) == 1
        assert messages[0]["frame_id"] == 101
        second.close()

    def test_server_stop_while_client_connected_does_not_raise(self, json_server):
        # "Python disconnects" from Unity's perspective -- the server side must shut down cleanly
        # even with an active client.
        server, settings = json_server
        sock = socket.create_connection(("127.0.0.1", settings.streaming_json_port), timeout=3)
        time.sleep(0.2)
        server.stop()  # must not raise
        sock.close()


class TestStartStopIdempotency:
    def test_double_start_is_safe(self):
        settings = _settings()
        server = PerceptionStreamServer(settings=settings)
        server.start()
        try:
            server.start()  # must not raise or open a second listener
            assert server.is_running
        finally:
            server.stop()

    def test_stop_before_start_is_safe(self):
        server = PerceptionStreamServer(settings=_settings())
        server.stop()  # must not raise


class TestPortExclusivity:
    """Regression test for the "dashboard silently stuck talking to a stale bridge process"
    bug: a second, genuinely-separate `PerceptionStreamServer` instance (standing in for a second
    `serve_unity_bridge.py` process someone forgot to stop) must fail loudly on the same port a
    first one is still actively listening on, not silently coexist -- see `streaming.server.
    _bind_exclusive`'s own docstring for why plain `SO_REUSEADDR` didn't provide this on Windows.
    """

    def test_second_instance_on_same_port_raises(self):
        settings = _settings()
        first = PerceptionStreamServer(settings=settings)
        first.start()
        try:
            second = PerceptionStreamServer(settings=settings)  # same json port as `first`
            with pytest.raises(OSError):
                second.start()
            assert not second.is_running
        finally:
            first.stop()

    def test_port_is_free_again_after_stop(self):
        """Not just "raises when taken" -- confirms `stop()` actually releases the port so a
        legitimate next run (the whole point of the demo workflow: stop scenario A, start
        scenario B) isn't itself blocked by this same exclusivity."""
        settings = _settings()
        first = PerceptionStreamServer(settings=settings)
        first.start()
        first.stop()

        second = PerceptionStreamServer(settings=settings)
        second.start()  # must not raise -- the port was genuinely released
        try:
            assert second.is_running
        finally:
            second.stop()

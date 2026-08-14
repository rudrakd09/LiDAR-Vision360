"""Tests for backend.ingestion.PerceptionIngestor: connect to a fake TCP source (standing in for
`PerceptionStreamServer`), consume real newline-delimited JSON messages, verify state updates and
DB event persistence -- the backend-side mirror of the Unity-emulator harness already used to
validate Phase 12 streaming end-to-end.
"""

import asyncio
import json
import socket
import threading
import time

import pytest

from backend.config import Settings
from backend.db import Database
from backend.ingestion import PerceptionIngestor
from backend.models_db import CollisionEvent
from backend.state import LatestState
from backend.ws import LiveBroadcastHub
from sqlalchemy import select


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _encode(msg: dict) -> bytes:
    return (json.dumps(msg) + "\n").encode("utf-8")


def _frame_message(frame_id: int, risk_level: str = "safe") -> dict:
    return {
        "protocol_version": "2.0.0", "message_type": "PERCEPTION_FRAME", "frame_id": frame_id,
        "timestamp": 1000.0 + frame_id, "transmission_timestamp": time.time(),
        "data": {
            "timestamp": 1000.0 + frame_id, "scan_id": "s", "sequence_number": frame_id, "source_id": "test",
            "objects": [{"track_id": "t1", "classification": "vehicle_like", "confidence": 0.9,
                         "centroid": {"x": 1.0, "y": 0.0}, "width": 1.0, "depth": 1.0, "distance": 1.0,
                         "velocity": None, "direction": None, "predicted_position": None,
                         "tracking_state": "confirmed", "movement_state": "unknown",
                         "track_age": 1, "track_hits": 1, "track_misses": 0}],
            "risk": {"overall_risk": risk_level, "most_critical": {"track_id": "t1", "classification": "vehicle_like",
                     "distance": 1.0, "relative_speed": 0.0, "in_projected_path": True, "ttc": None,
                     "collision_predicted": False, "predicted_collision_time": None, "predicted_collision_position": None,
                     "risk_level": risk_level, "risk_score": 0.0, "reason": ["test"]}, "results": []},
            "clearance": None, "vehicle": {"x": 0.0, "y": 0.0, "heading": 0.0, "speed_mps": 0.0},
            "config": {}, "map": None, "points": None,
        },
    }


def _run_fake_server(port_holder, payloads, ready_event, hold_open_s=0.5):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    port_holder.append(srv.getsockname()[1])
    srv.listen(1)
    ready_event.set()
    conn, _ = srv.accept()
    for p in payloads:
        conn.sendall(_encode(p))
        time.sleep(0.02)
    time.sleep(hold_open_s)
    conn.close()
    srv.close()


@pytest.fixture
def event_loop_for_ingestor():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)


@pytest.fixture
def db(tmp_path):
    settings = Settings(_env_file=None, database_url=f"sqlite:///{(tmp_path / 'test.db').as_posix()}")
    database = Database(settings)
    database.create_all()
    return database


class TestIngestionAgainstFakeServer:
    def test_frames_update_latest_state(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()
        payloads = [_frame_message(1), _frame_message(2), _frame_message(3)]
        server_thread = threading.Thread(target=_run_fake_server, args=(port_holder, payloads, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 3:
            time.sleep(0.05)

        assert state.connection.frames_received == 3
        assert state.connection.last_frame_id == 3
        assert state.latest_frame["objects"][0]["track_id"] == "t1"

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_risk_transition_persists_collision_event(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()
        payloads = [_frame_message(1, "safe"), _frame_message(2, "safe"), _frame_message(3, "critical")]
        server_thread = threading.Thread(target=_run_fake_server, args=(port_holder, payloads, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 3:
            time.sleep(0.05)

        # Give the DB write (happens synchronously right after state update, same thread) a moment.
        time.sleep(0.2)
        with db.session() as session:
            events = session.execute(select(CollisionEvent).order_by(CollisionEvent.frame_id)).scalars().all()

        # Two transitions, not one: the very first observation (None -> "safe" on frame 1) is
        # itself a legitimate transition worth recording (e.g. "session started already in
        # WARNING"), plus the "safe" -> "critical" one on frame 3. Frame 2 (still "safe") records
        # nothing, since nothing changed.
        assert len(events) == 2
        assert events[0].risk_level == "safe" and events[0].previous_risk_level is None
        assert events[1].risk_level == "critical" and events[1].previous_risk_level == "safe"

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_duplicate_frame_id_does_not_advance_state(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()
        payloads = [_frame_message(5), _frame_message(5)]  # exact duplicate
        server_thread = threading.Thread(target=_run_fake_server, args=(port_holder, payloads, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        time.sleep(1.0)
        assert state.connection.frames_received == 1
        assert state.connection.duplicate_or_out_of_order_dropped == 1

        ingestor.stop()
        server_thread.join(timeout=3)


class TestSourceIdFromFrameData:
    """Regression test for the "Session: no active session" dashboard bug: source_id must come
    from every PERCEPTION_FRAME's own `data.source_id` (always present), not only from the
    SYSTEM_STATUS message -- which is published once per bridge *run*, so a client connecting
    after it already went out (any late-connecting backend, not a hypothetical -- this is exactly
    what happened) would otherwise never learn the session's source_id despite frames actively
    flowing."""

    def test_source_id_populated_without_any_system_status_message(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()
        # Deliberately NO SYSTEM_STATUS message in this payload list -- only PERCEPTION_FRAMEs,
        # simulating a client that connected after SYSTEM_STATUS already went out.
        payloads = [_frame_message(1), _frame_message(2), _frame_message(3)]
        server_thread = threading.Thread(target=_run_fake_server, args=(port_holder, payloads, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 3:
            time.sleep(0.05)

        assert state.connection.source_id == "test"  # from _frame_message's own data.source_id

        ingestor.stop()
        server_thread.join(timeout=3)


def _frame_message_with_source(frame_id: int, source_id: str, risk_level: str = "safe") -> dict:
    msg = _frame_message(frame_id, risk_level)
    msg["data"]["source_id"] = source_id
    return msg


class TestReconnectToNewProducerResetsFrameIdTracking:
    """Regression test for the "dashboard stuck on the previous scenario" bug: every new TCP
    connection is a brand new producer (a fresh `scripts/serve_unity_bridge.py` run -- a
    different scenario, or the same one restarted after a crash), whose own `frame_id`/
    `sequence_number` sequence always starts back at 0 (`datasources.simulated.
    SimulatedDataSource.__init__`), completely unrelated to whatever frame_id the *previous*
    connection's producer last reached. `_last_accepted_frame_id` (and `_last_risk_level`/
    `_last_clearance_status`) must be reset on every fresh connection -- otherwise
    `classify_frame_id` sees the new producer's low frame_ids as OUT_OF_ORDER against the old
    high-water mark and `_handle_frame` silently drops every one of them, leaving
    `state.latest_frame`/`source_id` frozen on the old producer's last frame forever (or until the
    new producer's counter happens to climb back past the old one)."""

    def test_new_connection_with_lower_frame_ids_is_not_dropped_as_out_of_order(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()

        def _two_producers_in_sequence(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()

            # First "bridge run": frame_ids climb well past what a fresh run would start at.
            conn, _ = srv.accept()
            for fid in (500, 501, 502):
                conn.sendall(_encode(_frame_message_with_source(fid, "simulated:scenario_a")))
                time.sleep(0.02)
            conn.close()  # simulates the first bridge process being stopped

            # Second "bridge run" (a different scenario): brand new process, frame_ids restart at 0.
            conn2, _ = srv.accept()
            for fid in (0, 1, 2):
                conn2.sendall(_encode(_frame_message_with_source(fid, "simulated:scenario_b")))
                time.sleep(0.02)
            time.sleep(0.3)
            conn2.close()
            srv.close()

        server_thread = threading.Thread(target=_two_producers_in_sequence, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 6
        while time.time() < deadline and state.connection.frames_received < 6:
            time.sleep(0.05)

        assert state.connection.source_id == "simulated:scenario_b", (
            "backend stayed stuck on the previous producer's source_id -- the new (lower) "
            "frame_ids from the second connection were dropped as out-of-order"
        )
        assert state.connection.last_frame_id == 2
        assert state.latest_frame["objects"][0]["track_id"] == "t1"
        # None of the second producer's 3 frames should have been misclassified as out-of-order.
        assert state.connection.duplicate_or_out_of_order_dropped == 0

        ingestor.stop()
        server_thread.join(timeout=3)


class TestIngestionSurvivesBadMessages:
    """Regression test for the silent-thread-death bug: a message that causes an exception deep
    in dispatch (e.g. a field with an unexpected type) must be logged and skipped, not kill the
    background thread -- which would otherwise freeze every value the dashboard shows (connection
    state stays "connected" forever, but no further frame is ever processed) with no visible
    indication anything went wrong."""

    def test_one_malformed_frame_does_not_stop_subsequent_good_frames(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()

        good_frame_1 = _frame_message(1)
        malformed = _frame_message(2)
        malformed["data"]["risk"] = "not-a-dict-this-will-raise-attributeerror"  # .get() on a str raises
        good_frame_2 = _frame_message(3)
        good_frame_3 = _frame_message(4)

        payloads = [good_frame_1, malformed, good_frame_2, good_frame_3]
        server_thread = threading.Thread(target=_run_fake_server, args=(port_holder, payloads, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        # If the bug regresses, frames_received will get stuck at 1 (frame 1 processed, frame 2
        # kills the thread, frames 3/4 are never read) -- this deadline distinguishes "reached 4"
        # from "stuck forever", not just "eventually reached at least something".
        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 4:
            time.sleep(0.05)

        assert state.connection.frames_received == 4, "ingestion thread stopped processing after the malformed message -- the silent-death bug regressed"
        assert state.connection.last_frame_id == 4
        assert state.connection.state == "connected"

        ingestor.stop()
        server_thread.join(timeout=3)

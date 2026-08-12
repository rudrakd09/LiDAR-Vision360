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

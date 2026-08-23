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


def _collision_event(frame_id: int, new_value: str, previous_value: str | None, track_id: str = "t1") -> dict:
    """A `LiveState.events` "collision" entry, exactly as `pipeline.LiveStateBuilder` would build
    it -- see `perception/src/pipeline/live_state.py::_record_events`. Ingestion now persists
    ONLY from this real, Edge-computed signal (see `PerceptionIngestor._persist_events_from_frame`),
    not by re-deriving transitions itself, so tests exercising DB persistence must supply it
    explicitly, matching what a real frame would carry."""
    return {
        "event_type": "collision", "sequence_number": frame_id, "timestamp": 1000.0 + frame_id,
        "track_id": track_id, "previous_value": previous_value, "new_value": new_value,
        "summary": f"Risk: {previous_value or 'unknown'} -> {new_value}",
    }


def _frame_message(frame_id: int, risk_level: str = "safe", events: list | None = None) -> dict:
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
            "tracked_objects": [],
            "events": events or [],
            "risk": {"overall_risk": risk_level, "most_critical": {"track_id": "t1", "classification": "vehicle_like",
                     "distance": 1.0, "relative_speed": 0.0, "in_projected_path": True, "ttc": None,
                     "collision_predicted": False, "predicted_collision_time": None, "predicted_collision_position": None,
                     "risk_level": risk_level, "risk_score": 0.0, "reason": ["test"]}, "results": []},
            "clearance": None, "vehicle": {"x": 0.0, "y": 0.0, "heading": 0.0, "speed_mps": 0.0},
            "config": {}, "map": None, "points": None,
        },
    }


def _tracked_object(track_id: str, ttc=None, risk="safe", frames_tracked=1, first_seen=1000.0, last_seen=1000.0) -> dict:
    return {
        "track_id": track_id, "classification": "vehicle_like", "confidence": 0.9, "x": 1.0, "y": 0.0,
        "distance": 1.0, "velocity": None, "sensor_source": "lidar", "first_seen": first_seen,
        "last_seen": last_seen, "frames_tracked": frames_tracked, "trajectory": [],
        "ttc": ttc, "risk": risk, "tracking_state": "confirmed", "movement_state": "unknown",
    }


def _ttc_event(frame_id: int, track_id: str, new_value: str, previous_value: str | None) -> dict:
    return {
        "event_type": "ttc_change", "sequence_number": frame_id, "timestamp": 1000.0 + frame_id,
        "track_id": track_id, "previous_value": previous_value, "new_value": new_value,
        "summary": f"Track {track_id}: {new_value}",
    }


def _track_created_event(frame_id: int, track_id: str) -> dict:
    return {
        "event_type": "track_created", "sequence_number": frame_id, "timestamp": 1000.0 + frame_id,
        "track_id": track_id, "previous_value": None, "new_value": "confirmed",
        "summary": f"Track {track_id} created (vehicle_like).",
    }


def _track_lost_event(frame_id: int, track_id: str) -> dict:
    return {
        "event_type": "track_lost", "sequence_number": frame_id, "timestamp": 1000.0 + frame_id,
        "track_id": track_id, "previous_value": "confirmed", "new_value": "lost",
        "summary": f"Track {track_id} lost.",
    }


def _sensor_event(frame_id: int, new_value: str, previous_value: str) -> dict:
    return {
        "event_type": "sensor", "sequence_number": frame_id, "timestamp": 1000.0 + frame_id,
        "track_id": None, "previous_value": previous_value, "new_value": new_value,
        "summary": f"LiDAR: {previous_value} -> {new_value} (92.0% valid).",
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
        payloads = [
            _frame_message(1, "safe", events=[_collision_event(1, "safe", None)]),
            _frame_message(2, "safe"),  # unchanged -- Edge itself would not have emitted a new event this scan
            _frame_message(3, "critical", events=[_collision_event(3, "critical", "safe")]),
        ]
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


class TestNewPersistedRecordTypes:
    """`TrackRecord`/`TTCEvent`/`SensorEvent` -- all sourced directly from the Edge's own
    `data["events"]`/`data["tracked_objects"]`, never re-derived (see docs/architecture.md
    "PostgreSQL as history, WebSocket as live transport")."""

    def test_track_created_then_lost_persists_lifecycle(self, db, event_loop_for_ingestor):
        from backend.models_db import TrackRecord

        port_holder, ready = [], threading.Event()

        def _server(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()
            conn, _ = srv.accept()

            frame1 = _frame_message(1, events=[_track_created_event(1, "t1")])
            frame1["data"]["tracked_objects"] = [_tracked_object("t1", frames_tracked=1)]
            conn.sendall(_encode(frame1))
            time.sleep(0.02)

            frame2 = _frame_message(2, events=[])
            frame2["data"]["tracked_objects"] = [_tracked_object("t1", frames_tracked=2)]
            conn.sendall(_encode(frame2))
            time.sleep(0.02)

            frame3 = _frame_message(3, events=[_track_lost_event(3, "t1")])
            frame3["data"]["tracked_objects"] = []  # t1 no longer present -- lost
            conn.sendall(_encode(frame3))
            time.sleep(0.3)
            conn.close()
            srv.close()

        server_thread = threading.Thread(target=_server, args=(port_holder, ready), daemon=True)
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
        time.sleep(0.2)

        with db.session() as session:
            records = session.execute(select(TrackRecord).where(TrackRecord.track_id == "t1")).scalars().all()

        assert len(records) == 1, "one row per track_id, updated in place -- never one row per frame"
        assert records[0].status == "lost"
        assert records[0].frames_tracked == 2
        assert records[0].classification == "vehicle_like"

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_ttc_event_persists_with_real_ttc_value(self, db, event_loop_for_ingestor):
        from backend.models_db import TTCEvent

        port_holder, ready = [], threading.Event()

        def _server(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()
            conn, _ = srv.accept()

            frame = _frame_message(1, events=[_ttc_event(1, "t1", "approaching", "not_approaching")])
            frame["data"]["risk"]["results"] = [
                {"track_id": "t1", "classification": "vehicle_like", "distance": 4.5, "relative_speed": 2.0,
                 "in_projected_path": True, "ttc": 1.8, "collision_predicted": False, "predicted_collision_time": None,
                 "predicted_collision_position": None, "risk_level": "warning", "risk_score": 0.5, "reason": ["approaching"]},
            ]
            conn.sendall(_encode(frame))
            time.sleep(0.3)
            conn.close()
            srv.close()

        server_thread = threading.Thread(target=_server, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 1:
            time.sleep(0.05)
        time.sleep(0.2)

        with db.session() as session:
            records = session.execute(select(TTCEvent)).scalars().all()

        assert len(records) == 1
        assert records[0].track_id == "t1"
        assert records[0].previous_value == "not_approaching"
        assert records[0].new_value == "approaching"
        assert records[0].ttc_s == pytest.approx(1.8)  # the REAL ttc value, joined from risk.results[] by track_id

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_sensor_event_persists_with_real_valid_percentage(self, db, event_loop_for_ingestor):
        from backend.models_db import SensorEvent

        port_holder, ready = [], threading.Event()

        def _server(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()
            conn, _ = srv.accept()

            frame = _frame_message(1, events=[_sensor_event(1, "degraded", "ok")])
            frame["data"]["sensor_status"] = {"lidar": {"connected": True, "point_count": 340, "valid_percentage": 92.0, "mean_distance_m": 6.0}, "radar": None}
            conn.sendall(_encode(frame))
            time.sleep(0.3)
            conn.close()
            srv.close()

        server_thread = threading.Thread(target=_server, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.frames_received < 1:
            time.sleep(0.05)
        time.sleep(0.2)

        with db.session() as session:
            records = session.execute(select(SensorEvent)).scalars().all()

        assert len(records) == 1
        assert records[0].sensor_type == "lidar"
        assert records[0].previous_status == "ok"
        assert records[0].new_status == "degraded"
        assert records[0].valid_percentage == pytest.approx(92.0)

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_two_sessions_produce_two_separate_session_records(self, db, event_loop_for_ingestor):
        """Sessions must be separated -- one SessionRecord row per ingested TCP connection, never
        merged into one."""
        from backend.models_db import SessionRecord

        port_holder, ready = [], threading.Event()

        def _two_connections(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()

            conn1, _ = srv.accept()
            conn1.sendall(_encode(_frame_message(0)))
            time.sleep(0.05)
            conn1.close()

            conn2, _ = srv.accept()
            conn2.sendall(_encode(_frame_message(0)))
            time.sleep(0.3)
            conn2.close()
            srv.close()

        server_thread = threading.Thread(target=_two_connections, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        time.sleep(1.5)

        with db.session() as session:
            records = session.execute(select(SessionRecord)).scalars().all()

        assert len(records) == 2
        assert records[0].id != records[1].id

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


def _frame_message_with_session(frame_id: int, session_id: str, source_id: str, track_id: str = "t1", risk_level: str = "safe") -> dict:
    """Envelope-level `session_id`/`source_id` -- matches the real wire shape
    `streaming.protocol._envelope` now builds (see docs/architecture.md "Session and sequence
    management"), distinct from `_frame_message_with_source`'s older, `data`-only source_id."""
    msg = _frame_message(frame_id, risk_level)
    msg["session_id"] = session_id
    msg["source_id"] = source_id
    msg["data"]["source_id"] = source_id
    msg["data"]["objects"][0]["track_id"] = track_id
    msg["data"]["risk"]["most_critical"]["track_id"] = track_id
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


class TestSessionBoundaryWithoutReconnect:
    """See docs/architecture.md "Session and sequence management": a `session_id` change is
    detected and fully resets session state even without a TCP reconnect (a defensive backstop
    beyond `TestReconnectToNewProducerResetsFrameIdTracking`'s own reconnect-triggered reset) --
    and a stale message claiming an already-superseded `session_id` is rejected outright, never
    applied ("old sessions must never overwrite new sessions")."""

    def test_new_session_id_resets_state_mid_connection(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()

        def _one_connection_two_sessions(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()

            conn, _ = srv.accept()
            for fid in (0, 1, 2):
                conn.sendall(_encode(_frame_message_with_session(fid, "session-A", "simulated:scenario_a", track_id="a-track")))
                time.sleep(0.02)
            for fid in (0, 1):  # new session's own producer restarts its frame_id counter too
                conn.sendall(_encode(_frame_message_with_session(fid, "session-B", "simulated:scenario_b", track_id="b-track")))
                time.sleep(0.02)
            time.sleep(0.3)
            conn.close()
            srv.close()

        server_thread = threading.Thread(target=_one_connection_two_sessions, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.session_id != "session-B":
            time.sleep(0.05)
        time.sleep(0.2)  # let session-B's remaining frames land

        assert state.connection.session_id == "session-B"
        assert state.connection.source_id == "simulated:scenario_b"
        assert state.latest_frame["objects"][0]["track_id"] == "b-track"
        # session-A's track must not still be reachable through session-B's state.
        assert all(t["track_id"] != "a-track" for t in state.tracks_roster())
        assert state.track_history("a-track") is None
        # session_frames_received reflects ONLY session-B's own 2 frames, not the 5 total ever
        # received on this one TCP connection (frames_received, unaffected, stays cumulative).
        assert state.connection.session_frames_received == 2
        assert state.connection.frames_received == 5

        ingestor.stop()
        server_thread.join(timeout=3)

    def test_stale_message_from_superseded_session_is_rejected(self, db, event_loop_for_ingestor):
        port_holder, ready = [], threading.Event()

        def _one_connection_two_sessions_then_stale(port_holder, ready_event):
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", 0))
            port_holder.append(srv.getsockname()[1])
            srv.listen(1)
            ready_event.set()

            conn, _ = srv.accept()
            conn.sendall(_encode(_frame_message_with_session(0, "session-A", "simulated:scenario_a", track_id="a-track")))
            time.sleep(0.02)
            conn.sendall(_encode(_frame_message_with_session(0, "session-B", "simulated:scenario_b", track_id="b-track")))
            time.sleep(0.02)
            # A stale/out-of-order message somehow still carrying the now-superseded session-A id
            # -- must be rejected outright, not treated as "session-A is current again."
            conn.sendall(_encode(_frame_message_with_session(1, "session-A", "simulated:scenario_a", track_id="a-track-2")))
            time.sleep(0.02)
            time.sleep(0.3)
            conn.close()
            srv.close()

        server_thread = threading.Thread(target=_one_connection_two_sessions_then_stale, args=(port_holder, ready), daemon=True)
        server_thread.start()
        ready.wait(timeout=3)

        settings = Settings(_env_file=None, streaming_host="127.0.0.1", streaming_json_port=port_holder[0], streaming_reconnect_interval_s=0.2)
        state = LatestState(ring_buffer_size=50, track_grace_period_s=2.0)
        hub = LiveBroadcastHub()
        ingestor = PerceptionIngestor(settings, state, db, hub, event_loop_for_ingestor)
        ingestor.start()

        deadline = time.time() + 5
        while time.time() < deadline and state.connection.session_id != "session-B":
            time.sleep(0.05)
        time.sleep(0.3)  # let the stale session-A message arrive and (correctly) be dropped

        assert state.connection.session_id == "session-B", "a stale message from a superseded session must never overwrite the current session"
        assert state.latest_frame["objects"][0]["track_id"] == "b-track"
        assert state.connection.session_frames_received == 1, "the stale session-A message must not have been applied"

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

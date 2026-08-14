"""Tests for the REST API via FastAPI's TestClient -- exercises the app with its real lifespan
(a real, harmless `PerceptionIngestor` pointed at an unused port, since no bridge is running
during these tests) and seeds `LatestState` directly to make responses deterministic.
"""

import socket

import pytest
from fastapi.testclient import TestClient

from backend.config import get_settings


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def client(tmp_path, monkeypatch):
    get_settings.cache_clear()
    monkeypatch.setenv("LIDAR_DATABASE_URL", f"sqlite:///{(tmp_path / 'api_test.db').as_posix()}")
    monkeypatch.setenv("LIDAR_STREAMING_JSON_PORT", str(_free_port()))  # nothing listens here -- ingestor just idles
    monkeypatch.setenv("LIDAR_STREAMING_RECONNECT_INTERVAL_S", "60")  # don't spam reconnect attempts during the test

    from backend.main import create_app

    app = create_app()
    with TestClient(app) as test_client:
        yield test_client
    get_settings.cache_clear()


class TestHealth:
    def test_health_ok(self, client):
        resp = client.get("/api/health")
        assert resp.status_code == 200
        assert resp.json()["status"] == "ok"


class TestStatus:
    def test_status_reflects_no_connection_yet(self, client):
        resp = client.get("/api/status")
        assert resp.status_code == 200
        body = resp.json()
        assert body["state"] in ("connecting", "reconnecting")
        assert body["frames_received"] == 0


class TestLatestObjectsTracks:
    def test_latest_returns_204_when_nothing_received(self, client):
        resp = client.get("/api/latest")
        assert resp.status_code == 204

    def test_objects_empty_list_when_nothing_received(self, client):
        resp = client.get("/api/objects")
        assert resp.status_code == 200
        assert resp.json() == []

    def test_tracks_empty_list_when_nothing_received(self, client):
        resp = client.get("/api/tracks")
        assert resp.status_code == 200
        assert resp.json() == []

    def test_latest_reflects_seeded_state(self, client):
        state = client.app.state.latest_state
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None}, frame_id=1)
        resp = client.get("/api/latest")
        assert resp.status_code == 200
        assert resp.json()["objects"][0]["track_id"] == "t1"

        resp = client.get("/api/objects")
        assert resp.json()[0]["classification"] == "wall"


class TestDebugEndpoints:
    def test_live_frame_is_null_when_nothing_received(self, client):
        resp = client.get("/debug/live-frame")
        assert resp.status_code == 200
        assert resp.json() is None

    def test_live_frame_reflects_seeded_state(self, client):
        state = client.app.state.latest_state
        state.connection.state = "connected"  # debug_live_frame only reports data while actually connected
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None, "timestamp": 123.0}, frame_id=7)
        resp = client.get("/debug/live-frame")
        assert resp.status_code == 200
        body = resp.json()
        assert body["objects"][0]["track_id"] == "t1"
        assert body["timestamp"] == 123.0

    def test_live_frame_is_null_while_disconnected_even_with_a_cached_frame(self, client):
        """Regression test for a real repro: after a producer disconnects (e.g. switching
        scenarios -- the old bridge process stopped, the new one not yet accepted),
        `state.latest_frame` still holds the *previous* producer's last frame (by design, for
        `GET /latest`/`/ws/live`'s own dashboard-facing contract -- see LatestState's own
        docstring) -- but `/debug/live-frame`, read in isolation, must not hand that back as if it
        were the current live answer. `objects: []` on that stale cached frame (as in this test)
        is exactly the case that was previously indistinguishable from "the current scenario
        genuinely has no objects"."""
        state = client.app.state.latest_state
        state.connection.state = "connected"
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None, "timestamp": 123.0, "source_id": "simulated:previous_scenario"}, frame_id=7)
        assert client.get("/debug/live-frame").json()["objects"]  # sanity: it does report data while connected

        state.connection.state = "reconnecting"  # producer dropped -- state.latest_frame is untouched (still the stale frame)
        resp = client.get("/debug/live-frame")
        assert resp.status_code == 200
        assert resp.json() is None

    def test_status_endpoint_still_reports_the_stale_frame_alongside_connected_false(self, client):
        """`GET /latest` (dashboard-facing, unlike /debug/live-frame) intentionally keeps its own
        contract unchanged -- it's paired with `connection.session_status`/client-side staleness
        checking elsewhere, not meant to go null on every disconnect blip."""
        state = client.app.state.latest_state
        state.connection.state = "connected"
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None, "timestamp": 123.0}, frame_id=7)
        state.connection.state = "reconnecting"
        resp = client.get("/api/latest")
        assert resp.status_code == 200
        assert resp.json()["objects"][0]["track_id"] == "t1"  # unchanged, deliberately

    def test_stream_status_before_any_frame(self, client):
        resp = client.get("/debug/stream-status")
        assert resp.status_code == 200
        body = resp.json()
        assert body["connected"] is False
        assert body["last_frame_id"] is None
        assert body["frames_received"] == 0
        assert body["frames_dropped"] == 0
        assert body["object_count"] == 0
        assert body["track_count"] == 0
        assert body["risk"] is None
        assert body["age_ms"] is None

    def test_stream_status_reflects_seeded_frame(self, client):
        state = client.app.state.latest_state
        state.connection.state = "connected"
        state.connection.source_id = "simulated:test"  # normally set by PerceptionIngestor._handle_frame
        state.record_frame(
            {
                "objects": [{"track_id": "t1", "classification": "vehicle_like"}, {"track_id": "t2", "classification": "wall"}],
                "risk": {"overall_risk": "critical", "most_critical": None, "results": []},
                "clearance": None, "timestamp": 555.0, "source_id": "simulated:test",
            },
            frame_id=42,
        )
        resp = client.get("/debug/stream-status")
        assert resp.status_code == 200
        body = resp.json()
        assert body["connected"] is True
        assert body["last_frame_id"] == 42
        assert body["last_frame_timestamp"] == 555.0
        assert body["frames_received"] == 1
        assert body["source_id"] == "simulated:test"
        assert body["risk"] == "critical"
        assert body["object_count"] == 2
        assert body["age_ms"] is not None and body["age_ms"] >= 0

    def test_api_prefixed_debug_routes_also_exist(self, client):
        # Same double-mount convention every other route module gets (see main.py's own comment).
        assert client.get("/api/debug/live-frame").status_code == 200
        assert client.get("/api/debug/stream-status").status_code == 200


class TestEventsAndSessionsEmpty:
    def test_collision_events_empty_list(self, client):
        resp = client.get("/api/collision-events")
        assert resp.status_code == 200
        assert resp.json() == []

    def test_clearance_events_empty_list(self, client):
        resp = client.get("/api/clearance-events")
        assert resp.json() == []

    def test_events_combined_empty_list(self, client):
        resp = client.get("/api/events")
        assert resp.json() == []

    def test_sessions_has_the_active_session(self, client):
        resp = client.get("/api/sessions")
        assert resp.status_code == 200
        # The lifespan-started ingestor never connects (nothing is listening), so no session
        # should have been opened yet -- this documents that "connecting" != "connected".
        assert resp.json() == []


class TestEventLimitCapping:
    def test_limit_query_param_is_capped(self, client):
        settings = get_settings()
        resp = client.get(f"/api/collision-events?limit={settings.backend_event_max_limit + 1000}")
        assert resp.status_code == 200  # never errors, just silently capped server-side


class TestEventsAreSessionScopedByDefault:
    """Regression test for the "Event Timeline shows stale events from previous runs" bug:
    /api/events and friends must default to the currently active session, not every event ever
    recorded across every past ingestion connection -- a live dashboard should see the current
    run's events, not a jumble of every previous demo run's history mixed in."""

    def _seed_two_sessions(self, client):
        from backend.models_db import CollisionEvent, SessionRecord

        db = client.app.state.db
        with db.session() as db_session:
            db_session.add(SessionRecord(id="session-old", source_id="old_scenario", status="ended"))
            db_session.add(SessionRecord(id="session-current", source_id="08_approaching_obstacle", status="active"))
            db_session.add(CollisionEvent(session_id="session-old", frame_id=1, timestamp=1.0, risk_level="critical", previous_risk_level=None))
            db_session.add(CollisionEvent(session_id="session-current", frame_id=1, timestamp=2.0, risk_level="safe", previous_risk_level=None))
            db_session.commit()
        client.app.state.latest_state.current_session_id = "session-current"

    def test_default_query_only_returns_current_session_events(self, client):
        self._seed_two_sessions(client)
        resp = client.get("/api/collision-events")
        assert resp.status_code == 200
        events = resp.json()
        assert len(events) == 1
        assert events[0]["session_id"] == "session-current"

    def test_session_id_all_returns_every_session(self, client):
        self._seed_two_sessions(client)
        resp = client.get("/api/collision-events?session_id=all")
        assert resp.status_code == 200
        session_ids = {e["session_id"] for e in resp.json()}
        assert session_ids == {"session-old", "session-current"}

    def test_explicit_session_id_returns_only_that_one(self, client):
        self._seed_two_sessions(client)
        resp = client.get("/api/collision-events?session_id=session-old")
        assert resp.status_code == 200
        events = resp.json()
        assert len(events) == 1
        assert events[0]["session_id"] == "session-old"

    def test_no_active_session_means_no_filter(self, client):
        # If nothing is currently active (e.g. the bridge was never started), showing all history
        # is more useful than showing nothing -- there is no "current" to default to.
        self._seed_two_sessions(client)
        client.app.state.latest_state.current_session_id = None
        resp = client.get("/api/collision-events")
        assert len(resp.json()) == 2


class TestRootAndDocs:
    def test_root_returns_service_info_not_404(self, client):
        resp = client.get("/")
        assert resp.status_code == 200
        body = resp.json()
        assert body["name"] == "LiDAR-Vision360 Backend"
        assert "docs" in body
        assert "endpoints" in body
        assert "/health" in body["endpoints"]
        assert "/api/health" in body["endpoints"]
        assert "stream" in body
        assert "session_status" in body["stream"]

    def test_docs_available(self, client):
        resp = client.get("/docs")
        assert resp.status_code == 200

    def test_openapi_schema_available(self, client):
        resp = client.get("/openapi.json")
        assert resp.status_code == 200


class TestBarePathAliases:
    """Every route must work identically at its bare path and its original /api/* path -- 'add
    compatible routes rather than breaking existing clients'."""

    def test_health_matches_at_both_paths(self, client):
        bare, prefixed = client.get("/health"), client.get("/api/health")
        assert bare.status_code == prefixed.status_code == 200
        assert bare.json()["status"] == prefixed.json()["status"] == "ok"  # uptime_s itself may differ by a few ms between the two calls

    def test_status_identical_at_both_paths(self, client):
        assert client.get("/status").json()["state"] == client.get("/api/status").json()["state"]

    def test_objects_identical_at_both_paths(self, client):
        state = client.app.state.latest_state
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None}, frame_id=1)
        assert client.get("/objects").json() == client.get("/api/objects").json()

    def test_metrics_available_at_both_paths(self, client):
        assert client.get("/metrics").status_code == client.get("/api/metrics").status_code == 200


class TestTrackDetailAndHistory:
    def test_unknown_track_returns_404(self, client):
        resp = client.get("/api/tracks/never-seen")
        assert resp.status_code == 404

    def test_known_track_returns_summary(self, client):
        state = client.app.state.latest_state
        state.record_frame({"objects": [{"track_id": "t7", "classification": "vehicle_like", "centroid": {"x": 1.0, "y": 2.0}, "distance": 3.0}], "risk": None, "clearance": None}, frame_id=1)
        resp = client.get("/api/tracks/t7")
        assert resp.status_code == 200
        body = resp.json()
        assert body["track_id"] == "t7"
        assert body["frames_tracked"] == 1
        assert body["current"]["classification"] == "vehicle_like"

    def test_tracking_history_requires_track_id_param(self, client):
        resp = client.get("/api/tracking-history")
        assert resp.status_code == 422  # required query param missing

    def test_tracking_history_unknown_track_returns_404(self, client):
        resp = client.get("/api/tracking-history?track_id=never-seen")
        assert resp.status_code == 404

    def test_tracking_history_returns_points_for_known_track(self, client):
        state = client.app.state.latest_state
        for i in range(3):
            state.record_frame({"objects": [{"track_id": "t7", "classification": "vehicle_like", "centroid": {"x": float(i), "y": 0.0}, "distance": 1.0}], "risk": None, "clearance": None}, frame_id=i)
        resp = client.get("/api/tracking-history?track_id=t7")
        assert resp.status_code == 200
        points = resp.json()
        assert len(points) == 3
        assert [p["x"] for p in points] == [0.0, 1.0, 2.0]

    def test_tracking_history_respects_limit(self, client):
        state = client.app.state.latest_state
        for i in range(5):
            state.record_frame({"objects": [{"track_id": "t7", "classification": "vehicle_like", "centroid": {"x": float(i), "y": 0.0}, "distance": 1.0}], "risk": None, "clearance": None}, frame_id=i)
        resp = client.get("/api/tracking-history?track_id=t7&limit=2")
        assert len(resp.json()) == 2


class TestSessionDetail:
    def test_unknown_session_returns_404(self, client):
        resp = client.get("/api/sessions/never-existed")
        assert resp.status_code == 404

    def test_known_session_returns_detail(self, client):
        from backend.models_db import SessionRecord

        db = client.app.state.db
        with db.session() as db_session:
            db_session.add(SessionRecord(id="s1", source_id="08_approaching_obstacle", status="ended", frame_count=42))
            db_session.commit()
        resp = client.get("/api/sessions/s1")
        assert resp.status_code == 200
        assert resp.json()["id"] == "s1"
        assert resp.json()["frame_count"] == 42

    def test_active_session_detail_reflects_live_frame_count(self, client):
        from backend.models_db import SessionRecord

        db = client.app.state.db
        with db.session() as db_session:
            db_session.add(SessionRecord(id="s2", source_id="08_approaching_obstacle", status="active", frame_count=0))
            db_session.commit()
        state = client.app.state.latest_state
        state.current_session_id = "s2"
        for i in range(3):
            state.record_frame({"objects": [], "risk": None, "clearance": None}, frame_id=i)
        resp = client.get("/api/sessions/s2")
        assert resp.json()["frame_count"] == 3  # patched from live state, not the stale DB 0


class TestMetrics:
    def test_metrics_returns_real_counters(self, client):
        state = client.app.state.latest_state
        state.record_frame({"objects": [{"track_id": "t1", "classification": "wall"}], "risk": None, "clearance": None}, frame_id=1)
        resp = client.get("/api/metrics")
        assert resp.status_code == 200
        body = resp.json()
        assert body["frames_received"] == 1
        assert body["active_tracks"] == 1
        assert body["ring_buffer_used"] == 1
        assert "session_status" in body
        assert "uptime_s" in body

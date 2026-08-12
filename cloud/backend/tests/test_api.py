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

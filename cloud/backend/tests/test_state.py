"""Tests for backend.state.LatestState."""

import time

from backend.state import ConnectionInfo, LatestState, compute_session_status


def _frame(objects=None, timestamp=1000.0):
    return {"objects": objects or [], "risk": None, "clearance": None, "timestamp": timestamp}


def _object(track_id="t1", x=1.0, y=2.0, vx=0.5, vy=-0.5, distance=3.0, classification="vehicle_like", tracking_state="confirmed"):
    return {
        "track_id": track_id, "classification": classification, "distance": distance,
        "centroid": {"x": x, "y": y}, "velocity": {"vx": vx, "vy": vy} if vx is not None else None,
        "tracking_state": tracking_state,
    }


class TestRecordFrame:
    def test_latest_frame_reflects_most_recent(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(), frame_id=1)
        state.record_frame(_frame(), frame_id=2)
        assert state.connection.last_frame_id == 2

    def test_ring_buffer_is_bounded(self):
        state = LatestState(ring_buffer_size=3, track_grace_period_s=2.0)
        for i in range(10):
            state.record_frame(_frame(), frame_id=i)
        assert len(state.recent_frames()) == 3

    def test_frames_received_counter_increments(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        for i in range(5):
            state.record_frame(_frame(), frame_id=i)
        assert state.connection.frames_received == 5


class TestTracksRoster:
    def test_track_appears_after_being_seen(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(objects=[{"track_id": "t1", "classification": "vehicle_like"}]), frame_id=1)
        roster = state.tracks_roster()
        assert any(t["track_id"] == "t1" for t in roster)

    def test_track_pruned_after_grace_period(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=0.05)
        state.record_frame(_frame(objects=[{"track_id": "t1", "classification": "vehicle_like"}]), frame_id=1)
        time.sleep(0.1)
        roster = state.tracks_roster()
        assert not any(t["track_id"] == "t1" for t in roster)

    def test_object_without_track_id_is_not_indexed(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(objects=[{"track_id": None, "classification": "unknown"}]), frame_id=1)
        assert state.tracks_roster() == []


class TestTrackHistory:
    """Regression coverage for the 'VERY IMPORTANT' real-tracking-history requirement: bounded
    per-scan position/velocity history per track, not a static/single-snapshot list."""

    def test_history_accumulates_across_frames(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0, track_history_length=50)
        for i in range(5):
            state.record_frame(_frame(objects=[_object(x=float(i))], timestamp=1000.0 + i), frame_id=i)
        history = state.track_history("t1")
        assert history is not None
        assert len(history) == 5
        assert [p["x"] for p in history] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert [p["frame_id"] for p in history] == [0, 1, 2, 3, 4]

    def test_history_is_bounded_oldest_dropped_first(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0, track_history_length=3)
        for i in range(10):
            state.record_frame(_frame(objects=[_object(x=float(i))]), frame_id=i)
        history = state.track_history("t1")
        assert len(history) == 3
        assert [p["x"] for p in history] == [7.0, 8.0, 9.0]  # oldest (0..6) dropped, newest 3 kept

    def test_history_limit_param_returns_most_recent_n(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0, track_history_length=50)
        for i in range(10):
            state.record_frame(_frame(objects=[_object(x=float(i))]), frame_id=i)
        history = state.track_history("t1", limit=2)
        assert [p["x"] for p in history] == [8.0, 9.0]

    def test_unknown_track_id_returns_none_not_empty_list(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        assert state.track_history("never-seen") is None

    def test_history_cleared_when_track_pruned(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=0.05)
        state.record_frame(_frame(objects=[_object()]), frame_id=1)
        assert state.track_history("t1") is not None
        time.sleep(0.1)
        state.tracks_roster()  # triggers prune
        assert state.track_history("t1") is None

    def test_velocity_and_distance_recorded_per_point(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(objects=[_object(vx=1.5, vy=-2.5, distance=6.2)]), frame_id=1)
        point = state.track_history("t1")[0]
        assert point["vx"] == 1.5
        assert point["vy"] == -2.5
        assert point["distance"] == 6.2
        assert point["classification"] == "vehicle_like"
        assert point["tracking_state"] == "confirmed"


class TestTrackSummary:
    def test_unknown_track_returns_none(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        assert state.track_summary("never-seen") is None

    def test_known_track_has_first_seen_last_seen_and_frame_count(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        for i in range(3):
            state.record_frame(_frame(objects=[_object()]), frame_id=i)
        summary = state.track_summary("t1")
        assert summary is not None
        assert summary["track_id"] == "t1"
        assert summary["frames_tracked"] == 3
        assert summary["first_seen_at"] is not None
        assert summary["last_seen_at"] is not None
        assert summary["first_seen_at"] <= summary["last_seen_at"]
        assert summary["current"]["track_id"] == "t1"

    def test_tracks_roster_includes_summary_fields(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        for i in range(2):
            state.record_frame(_frame(objects=[_object()]), frame_id=i)
        roster = state.tracks_roster()
        assert roster[0]["frames_tracked"] == 2
        assert roster[0]["first_seen_at"] is not None


class TestSessionStatus:
    def test_disconnected_when_not_connected(self):
        info = ConnectionInfo(state="reconnecting")
        assert compute_session_status(info, now=1000.0, stale_threshold_s=3.0) == "disconnected"

    def test_active_when_connected_and_no_message_yet(self):
        info = ConnectionInfo(state="connected", last_message_at=None)
        assert compute_session_status(info, now=1000.0, stale_threshold_s=3.0) == "active"

    def test_active_when_recent_message(self):
        info = ConnectionInfo(state="connected", last_message_at=998.0)
        assert compute_session_status(info, now=1000.0, stale_threshold_s=3.0) == "active"

    def test_stale_when_message_older_than_threshold(self):
        info = ConnectionInfo(state="connected", last_message_at=990.0)
        assert compute_session_status(info, now=1000.0, stale_threshold_s=3.0) == "stale"

    def test_latest_state_exposes_session_status(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        assert state.session_status(stale_threshold_s=3.0) == "disconnected"  # never connected


class TestSnapshotForNewClient:
    def test_includes_connection_and_latest_frame(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(), frame_id=1)
        snapshot = state.snapshot_for_new_client(stale_threshold_s=3.0)
        assert "connection" in snapshot
        assert snapshot["latest_frame"] is not None

    def test_empty_state_snapshot_does_not_crash(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        snapshot = state.snapshot_for_new_client(stale_threshold_s=3.0)
        assert snapshot["latest_frame"] is None

    def test_connection_includes_session_status(self):
        """Regression test: the WS snapshot's `connection` dict must carry `session_status`
        (previously it silently didn't -- `_connection_dict` never included it -- so a dashboard
        that only ever read the one-time snapshot showed the Session badge as permanently
        INACTIVE regardless of the real, live session state)."""
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.connection.state = "connected"
        snapshot = state.snapshot_for_new_client(stale_threshold_s=3.0)
        assert snapshot["connection"]["session_status"] == "active"

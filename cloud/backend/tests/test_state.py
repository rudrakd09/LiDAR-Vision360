"""Tests for backend.state.LatestState."""

import time

from backend.state import LatestState


def _frame(objects=None):
    return {"objects": objects or [], "risk": None, "clearance": None}


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


class TestSnapshotForNewClient:
    def test_includes_connection_and_latest_frame(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        state.record_frame(_frame(), frame_id=1)
        snapshot = state.snapshot_for_new_client()
        assert "connection" in snapshot
        assert snapshot["latest_frame"] is not None

    def test_empty_state_snapshot_does_not_crash(self):
        state = LatestState(ring_buffer_size=10, track_grace_period_s=2.0)
        snapshot = state.snapshot_for_new_client()
        assert snapshot["latest_frame"] is None

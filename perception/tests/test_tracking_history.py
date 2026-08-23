"""Tests for `tracking.TrackHistory` -- Edge-side per-track first-seen/last-seen/trajectory
bookkeeping, driven entirely by `TrackedScan.objects` (i.e. what `ObjectTracker` already produced,
never a second tracking computation)."""

from __future__ import annotations

from models.objects import DetectedObject, ObjectClassification, Point2D, TrackingState
from models.tracking import TrackedScan
from tracking.history import TrackHistory


def _obj(track_id: str, x: float, timestamp: float) -> DetectedObject:
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=x, y=0.0),
        width=1.0, depth=1.0, distance=x, classification=ObjectClassification.VEHICLE_LIKE,
        confidence=0.9, tracking_state=TrackingState.CONFIRMED, timestamp=timestamp,
    )


def _scan(sequence_number: int, timestamp: float, objects: list[DetectedObject]) -> TrackedScan:
    return TrackedScan(
        scan_id=f"scan-{sequence_number}", sequence_number=sequence_number, source_id="test",
        timestamp=timestamp, objects=objects, object_count=len(objects), noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


class TestTrackHistory:
    def test_unknown_track_returns_none_and_empty(self):
        history = TrackHistory()
        assert history.first_seen("track-1") is None
        assert history.last_seen("track-1") is None
        assert history.trajectory("track-1") == []

    def test_first_seen_set_once_last_seen_updates(self):
        history = TrackHistory()
        history.update(_scan(0, 100.0, [_obj("track-1", 5.0, 100.0)]))
        history.update(_scan(1, 100.1, [_obj("track-1", 4.9, 100.1)]))

        assert history.first_seen("track-1") == 100.0
        assert history.last_seen("track-1") == 100.1

    def test_trajectory_grows_and_records_real_positions(self):
        history = TrackHistory()
        history.update(_scan(0, 100.0, [_obj("track-1", 5.0, 100.0)]))
        history.update(_scan(1, 100.1, [_obj("track-1", 4.9, 100.1)]))

        traj = history.trajectory("track-1")
        assert [p.x for p in traj] == [5.0, 4.9]
        assert [p.frame_id for p in traj] == [0, 1]

    def test_trajectory_bounded(self):
        history = TrackHistory(max_trajectory_points=3)
        for i in range(10):
            history.update(_scan(i, 100.0 + i, [_obj("track-1", float(i), 100.0 + i)]))
        assert len(history.trajectory("track-1")) == 3
        assert [p.x for p in history.trajectory("track-1")] == [7.0, 8.0, 9.0]

    def test_lost_track_is_pruned_immediately(self):
        history = TrackHistory()
        history.update(_scan(0, 100.0, [_obj("track-1", 5.0, 100.0)]))
        history.update(_scan(1, 100.1, []))  # track-1 no longer present -- LOST and dropped upstream

        assert history.first_seen("track-1") is None
        assert history.last_seen("track-1") is None
        assert history.trajectory("track-1") == []

    def test_update_returns_created_and_lost_track_ids(self):
        history = TrackHistory()
        created, lost = history.update(_scan(0, 100.0, [_obj("track-1", 5.0, 100.0)]))
        assert created == {"track-1"}
        assert lost == set()

        created, lost = history.update(_scan(1, 100.1, [_obj("track-1", 4.9, 100.1), _obj("track-2", 8.0, 100.1)]))
        assert created == {"track-2"}  # track-1 already known -- not "created" again
        assert lost == set()

        created, lost = history.update(_scan(2, 100.2, [_obj("track-2", 7.9, 100.2)]))  # track-1 dropped
        assert created == set()
        assert lost == {"track-1"}

    def test_independent_tracks_do_not_interfere(self):
        history = TrackHistory()
        history.update(_scan(0, 100.0, [_obj("track-1", 5.0, 100.0), _obj("track-2", 8.0, 100.0)]))
        history.update(_scan(1, 100.1, [_obj("track-1", 4.9, 100.1)]))  # track-2 dropped

        assert history.first_seen("track-1") == 100.0
        assert history.first_seen("track-2") is None  # pruned
        assert len(history.trajectory("track-1")) == 2

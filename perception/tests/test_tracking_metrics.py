"""Tests for tracking.metrics: track_summary, track_id_consistency, position_error, velocity_error."""

import pytest

from models.classification import ClassifiedScan
from models.objects import ObjectClassification, Point2D
from models.tracking import TrackedScan
from tracking.metrics import position_error, track_id_consistency, track_summary, velocity_error
from tracking.tracker import ObjectTracker


def _classified_scan(seq: int, detections) -> ClassifiedScan:
    return ClassifiedScan(
        scan_id=f"s{seq}", sequence_number=seq, source_id="unit-test", timestamp=seq * 0.1,
        objects=detections, noise_points=[], object_count=len(detections), noise_count=0,
    )


def _detection(x, y):
    from models.objects import DetectedObject

    return DetectedObject(
        object_id="d", centroid=Point2D(x=x, y=y), width=0.3, depth=0.3,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=ObjectClassification.POLE_LIKE, confidence=0.9, timestamp=0.0,
    )


def _run_tracker(positions: list[list[tuple]]) -> list[TrackedScan]:
    tracker = ObjectTracker()
    return [tracker.update(_classified_scan(i, [_detection(x, y) for x, y in p])) for i, p in enumerate(positions)]


class TestTrackSummary:
    def test_empty_input_returns_all_zero_summary(self):
        summary = track_summary([])
        assert summary["total_tracks_created"] == 0
        assert summary["active_tracks_at_end"] == 0
        assert summary["total_tracks_lost"] == 0
        assert summary["mean_track_age"] == 0.0

    def test_counts_a_single_persistent_track(self):
        scans = _run_tracker([[(5.0, 2.0)] for _ in range(5)])
        summary = track_summary(scans)
        assert summary["total_tracks_created"] == 1
        assert summary["active_tracks_at_end"] == 1
        assert summary["mean_track_age"] == 5

    def test_counts_a_lost_track(self):
        from common.config import Settings

        settings = Settings(_env_file=None, tracking_max_missed_scans=1, tracking_track_timeout_s=999.0)
        tracker = ObjectTracker(settings=settings)
        scans = [
            tracker.update(_classified_scan(0, [_detection(5.0, 2.0)])),
            tracker.update(_classified_scan(1, [])),
            tracker.update(_classified_scan(2, [])),
            tracker.update(_classified_scan(3, [])),
        ]
        summary = track_summary(scans)
        assert summary["total_tracks_created"] == 1
        assert summary["active_tracks_at_end"] == 0
        assert summary["total_tracks_lost"] == 1


class TestTrackIdConsistency:
    def test_empty_input_is_vacuously_consistent(self):
        assert track_id_consistency([]) == 1.0

    def test_perfectly_stable_sequence_is_fully_consistent(self):
        assert track_id_consistency([["a", "a", "a", "a"]]) == 1.0

    def test_a_single_reassignment_reduces_the_score(self):
        # 3 steps total, 1 broken (a -> b)
        score = track_id_consistency([["a", "a", "b", "b"]])
        assert score == pytest.approx(2 / 3)

    def test_none_to_none_steps_are_not_counted(self):
        assert track_id_consistency([[None, None, None]]) == 1.0

    def test_multiple_sequences_are_pooled(self):
        score = track_id_consistency([["a", "a"], ["b", "c"]])
        assert score == pytest.approx(0.5)


class TestPositionError:
    def test_empty_input_returns_zero(self):
        result = position_error([], [])
        assert result == {"mean_error_m": 0.0, "max_error_m": 0.0}

    def test_perfect_estimate_has_zero_error(self):
        result = position_error([(1.0, 1.0), (2.0, 2.0)], [(1.0, 1.0), (2.0, 2.0)])
        assert result["mean_error_m"] == pytest.approx(0.0)
        assert result["max_error_m"] == pytest.approx(0.0)

    def test_known_offset_computed_correctly(self):
        result = position_error([(3.0, 4.0)], [(0.0, 0.0)])
        assert result["mean_error_m"] == pytest.approx(5.0)
        assert result["max_error_m"] == pytest.approx(5.0)

    def test_mismatched_lengths_raises(self):
        with pytest.raises(ValueError):
            position_error([(1.0, 1.0)], [(1.0, 1.0), (2.0, 2.0)])


class TestVelocityError:
    def test_empty_input_returns_zero(self):
        result = velocity_error([], [])
        assert result == {"mean_error_m_s": 0.0, "max_error_m_s": 0.0}

    def test_known_offset_computed_correctly(self):
        result = velocity_error([(1.0, 0.0)], [(0.0, 0.0)])
        assert result["mean_error_m_s"] == pytest.approx(1.0)

    def test_mismatched_lengths_raises(self):
        with pytest.raises(ValueError):
            velocity_error([(1.0, 1.0)], [])

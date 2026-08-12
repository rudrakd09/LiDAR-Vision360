"""Tests for tracking.track.Track: lifecycle state machine (TENTATIVE/CONFIRMED/COASTING/LOST),
Kalman integration, and the DetectedObject projection in `to_detected_object`."""

import pytest

from common.config import Settings
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState
from tracking.track import Track

DEFAULT_SETTINGS = Settings(_env_file=None)


def _detection(x=5.0, y=2.0, classification=ObjectClassification.POLE_LIKE, confidence=0.9, width=0.3, depth=0.3) -> DetectedObject:
    return DetectedObject(
        object_id="d", centroid=Point2D(x=x, y=y), width=width, depth=depth,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=classification, confidence=confidence,
        point_count=8, min_distance=(x ** 2 + y ** 2) ** 0.5 - 0.1, angular_width=5.0, timestamp=0.0,
    )


class TestConstruction:
    def test_new_track_starts_tentative_with_one_hit(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        assert track.state == TrackingState.TENTATIVE
        assert track.age == 1
        assert track.hits == 1
        assert track.misses == 0

    def test_new_track_initializes_kalman_position_from_detection(self):
        track = Track("t1", _detection(x=3.0, y=-1.0), timestamp=0.0, settings=DEFAULT_SETTINGS)
        assert track.kf.x == pytest.approx(3.0)
        assert track.kf.y == pytest.approx(-1.0)
        assert track.kf.vx == pytest.approx(0.0)
        assert track.kf.vy == pytest.approx(0.0)


class TestConfirmation:
    def test_tentative_until_min_hits_reached(self):
        settings = Settings(_env_file=None, tracking_min_hits_to_confirm=3)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        assert track.state == TrackingState.TENTATIVE  # hit 1
        track.predict()
        track.update(_detection(), timestamp=0.1)
        assert track.state == TrackingState.TENTATIVE  # hit 2
        track.predict()
        track.update(_detection(), timestamp=0.2)
        assert track.state == TrackingState.CONFIRMED  # hit 3 == min_hits_to_confirm


class TestMissesAndCoasting:
    def test_single_miss_goes_coasting_not_lost(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        track.predict()
        track.mark_missed(timestamp=0.1)
        assert track.state == TrackingState.COASTING
        assert track.misses == 1

    def test_reappearing_after_coasting_returns_to_confirmed(self):
        settings = Settings(_env_file=None, tracking_min_hits_to_confirm=1)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        assert track.state == TrackingState.CONFIRMED  # min_hits_to_confirm=1, confirmed immediately
        track.predict()
        track.mark_missed(timestamp=0.1)
        assert track.state == TrackingState.COASTING
        track.predict()
        track.update(_detection(), timestamp=0.2)
        assert track.state == TrackingState.CONFIRMED
        assert track.misses == 0

    def test_misses_reset_to_zero_on_a_real_update(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        track.predict()
        track.mark_missed(timestamp=0.1)
        track.predict()
        track.mark_missed(timestamp=0.2)
        assert track.misses == 2
        track.predict()
        track.update(_detection(), timestamp=0.3)
        assert track.misses == 0


class TestTimeout:
    def test_exceeding_max_missed_scans_marks_lost(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=2, tracking_track_timeout_s=999.0)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        for i in range(1, 4):
            track.predict()
            track.mark_missed(timestamp=float(i) * 0.1)
        # 3 consecutive misses > max_missed_scans(2) -> LOST
        assert track.state == TrackingState.LOST

    def test_exceeding_track_timeout_marks_lost_even_within_miss_budget(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=999, tracking_track_timeout_s=0.5)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        track.predict()
        track.mark_missed(timestamp=1.0)  # only 1 miss, but 1.0s > 0.5s timeout
        assert track.state == TrackingState.LOST

    def test_within_both_budgets_stays_coasting(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=5, tracking_track_timeout_s=1.0)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        track.predict()
        track.mark_missed(timestamp=0.1)
        assert track.state == TrackingState.COASTING


class TestVelocityReliability:
    def test_velocity_none_before_min_observations(self):
        settings = Settings(_env_file=None, tracking_min_observations_for_velocity=3)
        track = Track("t1", _detection(), timestamp=0.0, settings=settings)
        obj = track.to_detected_object(timestamp=0.0)
        assert obj.velocity is None
        assert obj.movement_state == MovementState.UNKNOWN

        track.predict()
        track.update(_detection(x=5.1, y=2.0), timestamp=0.1)
        obj = track.to_detected_object(timestamp=0.1)
        assert obj.velocity is None  # only 2 hits so far

    def test_velocity_reported_once_min_observations_reached(self):
        settings = Settings(_env_file=None, tracking_min_observations_for_velocity=3)
        track = Track("t1", _detection(x=5.0, y=2.0), timestamp=0.0, settings=settings)
        for i in range(1, 3):
            track.predict()
            track.update(_detection(x=5.0 - i * 0.1, y=2.0), timestamp=float(i) * 0.1)
        obj = track.to_detected_object(timestamp=0.3)
        assert obj.velocity is not None
        assert obj.direction is not None


class TestToDetectedObject:
    def test_carries_track_id_and_classification(self):
        track = Track("track-42", _detection(classification=ObjectClassification.WALL, confidence=0.8), timestamp=0.0, settings=DEFAULT_SETTINGS)
        obj = track.to_detected_object(timestamp=0.0)
        assert obj.track_id == "track-42"
        assert obj.object_id == "track-42"
        assert obj.classification == ObjectClassification.WALL
        assert obj.confidence == pytest.approx(0.8)

    def test_predicted_position_is_populated(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        obj = track.to_detected_object(timestamp=0.0)
        assert obj.predicted_position is not None

    def test_tracking_state_and_counters_populated(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        obj = track.to_detected_object(timestamp=0.0)
        assert obj.tracking_state == TrackingState.TENTATIVE
        assert obj.track_age == 1
        assert obj.track_hits == 1
        assert obj.track_misses == 0

    def test_shape_fields_carried_from_last_detection(self):
        track = Track("t1", _detection(), timestamp=0.0, settings=DEFAULT_SETTINGS)
        obj = track.to_detected_object(timestamp=0.0)
        assert obj.point_count == 8
        assert obj.angular_width == pytest.approx(5.0)

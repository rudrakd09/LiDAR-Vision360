"""Tests for models.tracking.TrackedScan and the Phase 7 additive fields on models.objects.
DetectedObject (predicted_position, tracking_state, movement_state, track_age, track_hits,
track_misses)."""

import pytest
from pydantic import ValidationError

from models.lidar import CartesianPoint
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState
from models.tracking import TrackedScan


def _object(track_id="track-1", point_count=5) -> DetectedObject:
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=1.0, y=2.0), width=0.5, depth=0.5,
        distance=2.24, point_count=point_count, timestamp=0.0,
    )


class TestDetectedObjectPhase7Fields:
    def test_defaults_are_none_for_objects_built_by_earlier_phases(self):
        obj = _object()
        assert obj.predicted_position is None
        assert obj.tracking_state is None
        assert obj.movement_state is None
        assert obj.track_age is None
        assert obj.track_hits is None
        assert obj.track_misses is None

    def test_fields_can_be_populated(self):
        obj = DetectedObject(
            object_id="track-1", track_id="track-1", centroid=Point2D(x=1.0, y=2.0), width=0.5, depth=0.5,
            distance=2.24, timestamp=0.0,
            predicted_position=Point2D(x=1.1, y=2.1),
            tracking_state=TrackingState.CONFIRMED,
            movement_state=MovementState.MOVING,
            track_age=10, track_hits=8, track_misses=0,
        )
        assert obj.predicted_position == Point2D(x=1.1, y=2.1)
        assert obj.tracking_state == TrackingState.CONFIRMED
        assert obj.movement_state == MovementState.MOVING
        assert obj.track_age == 10
        assert obj.track_hits == 8
        assert obj.track_misses == 0

    def test_negative_track_counters_rejected(self):
        with pytest.raises(ValidationError):
            DetectedObject(
                object_id="t", centroid=Point2D(x=0.0, y=0.0), width=0.1, depth=0.1, distance=0.0,
                timestamp=0.0, track_age=-1,
            )


class TestTrackingStateEnum:
    def test_values(self):
        assert TrackingState.TENTATIVE.value == "tentative"
        assert TrackingState.CONFIRMED.value == "confirmed"
        assert TrackingState.COASTING.value == "coasting"
        assert TrackingState.LOST.value == "lost"


class TestMovementStateEnum:
    def test_values(self):
        assert MovementState.STATIONARY.value == "stationary"
        assert MovementState.MOVING.value == "moving"
        assert MovementState.UNKNOWN.value == "unknown"


class TestTrackedScan:
    def test_point_count_sums_objects_and_noise(self):
        scan = TrackedScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=0.0,
            objects=[_object(point_count=5), _object(track_id="track-2", point_count=3)],
            noise_points=[CartesianPoint(angle=10.0, distance=12.0, timestamp=0.0, x=11.8, y=2.1)],
            object_count=2, noise_count=1, new_track_count=0, lost_track_count=0, coasting_track_count=0,
        )
        assert scan.point_count == 5 + 3 + 1

    def test_empty_scan_is_valid(self):
        scan = TrackedScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=0.0,
            objects=[], noise_points=[], object_count=0, noise_count=0,
            new_track_count=0, lost_track_count=0, coasting_track_count=0,
        )
        assert scan.point_count == 0

    def test_counts_are_independent_fields(self):
        scan = TrackedScan(
            scan_id="s", sequence_number=1, source_id="unit-test", timestamp=1.0,
            objects=[_object()], noise_points=[], object_count=1, noise_count=0,
            new_track_count=1, lost_track_count=2, coasting_track_count=0,
        )
        assert scan.new_track_count == 1
        assert scan.lost_track_count == 2

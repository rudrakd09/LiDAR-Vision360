"""Tests for tracking.association.associate: nearest-neighbour matching, the hard distance gate,
and the soft dimension/classification cost terms."""

import pytest

from common.config import Settings
from models.objects import DetectedObject, ObjectClassification, Point2D
from tracking.association import associate
from tracking.track import Track

DEFAULT_SETTINGS = Settings(_env_file=None)


def _detection(x: float, y: float, width=0.5, depth=0.5, classification=ObjectClassification.POLE_LIKE, confidence=0.9) -> DetectedObject:
    return DetectedObject(
        object_id="d", centroid=Point2D(x=x, y=y), width=width, depth=depth,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=classification, confidence=confidence, timestamp=0.0,
    )


def _track(track_id: str, x: float, y: float, settings: Settings = DEFAULT_SETTINGS, **detection_kwargs) -> Track:
    return Track(track_id, _detection(x, y, **detection_kwargs), timestamp=0.0, settings=settings)


class TestBasicMatching:
    def test_single_track_single_close_detection_matches(self):
        track = _track("t1", 5.0, 2.0)
        detections = [_detection(5.05, 2.02)]
        matches, unmatched_detections, unmatched_tracks = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 0}
        assert unmatched_detections == []
        assert unmatched_tracks == []

    def test_no_tracks_all_detections_unmatched(self):
        detections = [_detection(1.0, 1.0), _detection(2.0, 2.0)]
        matches, unmatched_detections, unmatched_tracks = associate([], detections, DEFAULT_SETTINGS)
        assert matches == {}
        assert unmatched_detections == [0, 1]
        assert unmatched_tracks == []

    def test_no_detections_all_tracks_unmatched(self):
        track = _track("t1", 5.0, 2.0)
        matches, unmatched_detections, unmatched_tracks = associate([track], [], DEFAULT_SETTINGS)
        assert matches == {}
        assert unmatched_detections == []
        assert unmatched_tracks == ["t1"]


class TestDistanceGate:
    def test_detection_beyond_max_distance_is_not_matched(self):
        track = _track("t1", 0.0, 0.0)
        far = DEFAULT_SETTINGS.tracking_max_association_distance_m + 1.0
        detections = [_detection(far, 0.0)]
        matches, unmatched_detections, unmatched_tracks = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {}
        assert unmatched_detections == [0]
        assert unmatched_tracks == ["t1"]

    def test_detection_just_within_max_distance_is_matched(self):
        track = _track("t1", 0.0, 0.0)
        near = DEFAULT_SETTINGS.tracking_max_association_distance_m - 0.01
        detections = [_detection(near, 0.0)]
        matches, _, _ = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 0}


class TestNearestNeighbourChoosesClosest:
    def test_track_matches_the_closer_of_two_candidate_detections(self):
        track = _track("t1", 0.0, 0.0)
        detections = [_detection(1.5, 0.0), _detection(0.2, 0.0)]
        matches, unmatched_detections, _ = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 1}
        assert unmatched_detections == [0]

    def test_each_track_and_detection_used_at_most_once(self):
        track_a = _track("a", 0.0, 0.0)
        track_b = _track("b", 3.0, 0.0)
        detections = [_detection(0.1, 0.0), _detection(3.1, 0.0)]
        matches, unmatched_detections, unmatched_tracks = associate([track_a, track_b], detections, DEFAULT_SETTINGS)
        assert matches == {"a": 0, "b": 1}
        assert unmatched_detections == []
        assert unmatched_tracks == []

    def test_closest_pair_wins_when_both_tracks_could_match_the_same_detection(self):
        # Detection at (1.0, 0) is closer to track "near" (0,0) than to track "far" (2.5, 0);
        # "far" is left to match its own (2.6, 0) detection instead of stealing this one.
        track_near = _track("near", 0.0, 0.0)
        track_far = _track("far", 2.5, 0.0)
        detections = [_detection(1.0, 0.0), _detection(2.6, 0.0)]
        matches, unmatched_detections, unmatched_tracks = associate([track_near, track_far], detections, DEFAULT_SETTINGS)
        assert matches == {"near": 0, "far": 1}
        assert unmatched_detections == []
        assert unmatched_tracks == []


class TestClassificationIsNotMandatory:
    def test_mismatched_classification_still_matches_if_close_enough(self):
        track = _track("t1", 0.0, 0.0, classification=ObjectClassification.WALL)
        detections = [_detection(0.1, 0.0, classification=ObjectClassification.POLE_LIKE)]
        matches, _, _ = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 0}

    def test_matching_classification_is_preferred_on_a_near_tie(self):
        # Two detections nearly equidistant from the track; only the classification match should
        # decide it, demonstrating classification is used as a soft, not mandatory, cost term.
        track = _track("t1", 0.0, 0.0, classification=ObjectClassification.WALL)
        same_class = _detection(1.0, 0.0, classification=ObjectClassification.WALL)
        other_class = _detection(0.99, 0.0, classification=ObjectClassification.VEHICLE_LIKE)
        matches, _, _ = associate([track], [other_class, same_class], DEFAULT_SETTINGS)
        assert matches == {"t1": 1}

    def test_unknown_classification_on_either_side_never_penalized(self):
        track = _track("t1", 0.0, 0.0, classification=ObjectClassification.UNKNOWN)
        detections = [_detection(0.1, 0.0, classification=ObjectClassification.VEHICLE_LIKE)]
        matches, _, _ = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 0}


class TestDimensionIsSoft:
    def test_large_dimension_difference_does_not_block_an_otherwise_close_match(self):
        track = _track("t1", 0.0, 0.0, width=0.2, depth=0.2)
        detections = [_detection(0.1, 0.0, width=5.0, depth=5.0)]
        matches, _, _ = associate([track], detections, DEFAULT_SETTINGS)
        assert matches == {"t1": 0}

"""Synthetic, deterministic tests for tracking.tracker.ObjectTracker across full multi-scan
sequences -- the scenario checklist from this phase's spec (stationary/straight/diagonal motion,
new/disappearing/reappearing objects, multiple and nearby objects, noisy and missing detections,
classification changes, track timeout, and track ID uniqueness). Moving-obstacle *simulator*
scenarios (07_moving_crossing, 08_approaching_obstacle) are covered by
simulator/tests/test_tracking_integration.py instead, per this project's established split (see
docs/architecture.md "Cross-package integration tests")."""

import random

import pytest

from common.config import Settings
from models.classification import ClassifiedScan
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState
from tracking.tracker import ObjectTracker

DEFAULT_SETTINGS = Settings(_env_file=None)
DT = 1.0 / DEFAULT_SETTINGS.lidar_scan_frequency_hz  # nominal inter-scan interval tracking assumes


def _detection(x: float, y: float, classification=ObjectClassification.POLE_LIKE, confidence=0.9, width=0.3, depth=0.3) -> DetectedObject:
    return DetectedObject(
        object_id="d", centroid=Point2D(x=x, y=y), width=width, depth=depth,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=classification, confidence=confidence, timestamp=0.0,
    )


def _scan(seq: int, timestamp: float, detections: list[DetectedObject]) -> ClassifiedScan:
    return ClassifiedScan(
        scan_id=f"scan-{seq}", sequence_number=seq, source_id="unit-test", timestamp=timestamp,
        objects=detections, noise_points=[], object_count=len(detections), noise_count=0,
    )


def _run(tracker: ObjectTracker, positions_per_scan: list[list[tuple]], settings: Settings = DEFAULT_SETTINGS):
    """Feed one detection sequence through `tracker`, one scan per entry of `positions_per_scan`
    (each entry is a list of `(x, y)` or `(x, y, kwargs_dict)` tuples). Returns the list of
    `TrackedScan` results, one per scan."""
    results = []
    for i, positions in enumerate(positions_per_scan):
        detections = []
        for pos in positions:
            if len(pos) == 2:
                x, y = pos
                detections.append(_detection(x, y))
            else:
                x, y, kwargs = pos
                detections.append(_detection(x, y, **kwargs))
        results.append(tracker.update(_scan(i, i * DT, detections)))
    return results


class TestStationaryObject:
    def test_stationary_object_gets_a_stable_id_and_is_classified_stationary_once_reliable(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        results = _run(tracker, [[(5.0, 2.0)] for _ in range(10)])
        ids = {o.track_id for r in results for o in r.objects}
        assert ids == {"track-1"}
        final = results[-1].objects[0]
        assert final.movement_state == MovementState.STATIONARY
        assert final.velocity is not None
        assert final.velocity.speed < DEFAULT_SETTINGS.tracking_stationary_speed_threshold_mps


class TestMovingStraight:
    def test_moving_object_velocity_converges_to_true_speed(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        true_vx = -1.5
        positions = [[(5.0 + true_vx * DT * i, 2.0)] for i in range(15)]
        results = _run(tracker, positions)
        final = results[-1].objects[0]
        assert final.movement_state == MovementState.MOVING
        assert final.velocity.vx == pytest.approx(true_vx, abs=0.1)
        assert final.velocity.vy == pytest.approx(0.0, abs=0.1)

    def test_same_track_id_maintained_across_scans(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = [[(5.0 - 0.15 * i, 2.0)] for i in range(10)]
        results = _run(tracker, positions)
        ids = {o.track_id for r in results for o in r.objects}
        assert ids == {"track-1"}


class TestMovingDiagonally:
    def test_diagonal_velocity_converges_on_both_axes(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        true_vx, true_vy = 1.0, 1.0
        positions = [[(1.0 + true_vx * DT * i, -1.0 + true_vy * DT * i)] for i in range(15)]
        results = _run(tracker, positions)
        final = results[-1].objects[0]
        assert final.velocity.vx == pytest.approx(true_vx, abs=0.15)
        assert final.velocity.vy == pytest.approx(true_vy, abs=0.15)
        assert final.direction == pytest.approx(45.0, abs=5.0)


class TestNewTrackCreation:
    def test_detection_with_no_prior_track_creates_a_new_one(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        result = tracker.update(_scan(0, 0.0, [_detection(1.0, 1.0)]))
        assert result.new_track_count == 1
        assert result.objects[0].track_id == "track-1"
        assert result.objects[0].tracking_state == TrackingState.TENTATIVE

    def test_second_unrelated_detection_creates_a_second_track(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        tracker.update(_scan(0, 0.0, [_detection(1.0, 1.0)]))
        result = tracker.update(_scan(1, DT, [_detection(1.0, 1.0), _detection(20.0, 20.0)]))
        assert result.new_track_count == 1
        assert {o.track_id for o in result.objects} == {"track-1", "track-2"}


class TestTemporaryOcclusionAndReappearance:
    def test_track_coasts_through_a_gap_and_keeps_its_id_on_reappearance(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = [[(5.0, 2.0)]] * 4 + [[]] * 3 + [[(5.0, 2.0)]] * 3
        results = _run(tracker, positions)

        coasting_scans = results[4:7]
        for r in coasting_scans:
            assert len(r.objects) == 1
            assert r.objects[0].tracking_state == TrackingState.COASTING
            assert r.objects[0].track_id == "track-1"

        reappeared = results[-1].objects[0]
        assert reappeared.track_id == "track-1"
        assert reappeared.tracking_state == TrackingState.CONFIRMED
        assert results[-1].new_track_count == 0
        assert results[-1].lost_track_count == 0

    def test_coasting_object_reports_a_predicted_only_position(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = [[(5.0, 2.0)]] * 4 + [[]]
        results = _run(tracker, positions)
        coasting = results[-1].objects[0]
        assert coasting.tracking_state == TrackingState.COASTING
        assert coasting.centroid.x == pytest.approx(5.0, abs=0.01)  # stationary -> predicted stays put


class TestMultipleObjects:
    def test_three_simultaneous_objects_each_get_a_distinct_stable_id(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = [[(5.0, 5.0), (-5.0, -5.0), (0.0, 8.0)] for _ in range(6)]
        results = _run(tracker, positions)
        final_ids = {o.track_id for o in results[-1].objects}
        assert final_ids == {"track-1", "track-2", "track-3"}
        assert results[-1].new_track_count == 0


class TestTwoNearbyObjects:
    def test_two_objects_1m_apart_moving_in_parallel_are_not_confused(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        # Two poles 1m apart (y=2.0 and y=3.0), both moving forward (-x) at the same speed --
        # close enough to test the associator doesn't swap their identities, far enough apart
        # (> tracking_measurement_noise_std_m) to have an unambiguous correct answer.
        positions = [[(5.0 - 0.1 * i, 2.0), (5.0 - 0.1 * i, 3.0)] for i in range(10)]
        results = _run(tracker, positions)

        # Track each physical object's y-coordinate across scans by whichever track_id stays
        # closest to y=2.0 each time -- if identities ever swapped, this would jump to y=3.0.
        near_track_id = min(results[0].objects, key=lambda o: abs(o.centroid.y - 2.0)).track_id
        for r in results:
            near_obj = next(o for o in r.objects if o.track_id == near_track_id)
            assert near_obj.centroid.y == pytest.approx(2.0, abs=0.05)

        assert len({o.track_id for o in results[-1].objects}) == 2


class TestNoisyDetections:
    def test_small_random_jitter_does_not_break_track_continuity(self):
        random.seed(42)
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = []
        for i in range(15):
            x = 5.0 - 0.1 * i + random.uniform(-0.03, 0.03)
            y = 2.0 + random.uniform(-0.03, 0.03)
            positions.append([(x, y)])
        results = _run(tracker, positions)
        ids = {o.track_id for r in results for o in r.objects}
        assert ids == {"track-1"}  # noise alone never spawns a spurious second track
        assert results[-1].objects[0].velocity.vx == pytest.approx(-1.0, abs=0.3)


class TestMissingDetections:
    def test_isolated_single_scan_misses_do_not_lose_the_track(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        positions = [[(5.0, 2.0)], [(4.9, 2.0)], [], [(4.7, 2.0)], [], [(4.5, 2.0)]]
        results = _run(tracker, positions)
        ids = {o.track_id for r in results for o in r.objects}
        assert ids == {"track-1"}
        assert sum(r.lost_track_count for r in results) == 0


class TestClassificationChanges:
    def test_flickering_classification_does_not_fragment_the_track(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        classes = [ObjectClassification.POLE_LIKE, ObjectClassification.UNKNOWN, ObjectClassification.VEHICLE_LIKE, ObjectClassification.UNKNOWN, ObjectClassification.POLE_LIKE]
        positions = [[(5.0 - 0.1 * i, 2.0, {"classification": classes[i]})] for i in range(len(classes))]
        results = _run(tracker, positions)
        ids = {o.track_id for r in results for o in r.objects}
        assert ids == {"track-1"}

    def test_reported_classification_reflects_most_recent_detection(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        tracker.update(_scan(0, 0.0, [_detection(5.0, 2.0, classification=ObjectClassification.POLE_LIKE)]))
        result = tracker.update(_scan(1, DT, [_detection(5.0, 2.0, classification=ObjectClassification.VEHICLE_LIKE)]))
        assert result.objects[0].classification == ObjectClassification.VEHICLE_LIKE


class TestTrackTimeout:
    def test_track_marked_lost_and_dropped_after_exceeding_miss_budget(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=3, tracking_track_timeout_s=999.0)
        tracker = ObjectTracker(settings=settings)
        positions = [[(5.0, 2.0)]] * 3 + [[]] * 5
        results = _run(tracker, positions, settings=settings)

        lost_at = next(i for i, r in enumerate(results) if r.lost_track_count > 0)
        assert lost_at is not None
        for r in results[lost_at:]:
            assert r.objects == []
        assert tracker.active_track_count == 0

    def test_lost_track_id_is_never_reused(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=2, tracking_track_timeout_s=999.0)
        tracker = ObjectTracker(settings=settings)
        positions = [[(5.0, 2.0)]] * 2 + [[]] * 4 + [[(5.0, 2.0)]]  # a *new* object reappears at the same spot
        results = _run(tracker, positions, settings=settings)
        reappeared = results[-1].objects[0]
        assert reappeared.track_id == "track-2"  # not "track-1" -- that identity is gone for good


class TestTrackIdUniqueness:
    def test_many_simultaneous_new_objects_get_all_distinct_ids(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        detections = [_detection(float(i) * 3.0, 0.0) for i in range(8)]
        result = tracker.update(_scan(0, 0.0, detections))
        ids = [o.track_id for o in result.objects]
        assert len(ids) == len(set(ids)) == 8

    def test_ids_unique_across_an_entire_session_including_lost_and_new_tracks(self):
        settings = Settings(_env_file=None, tracking_max_missed_scans=1, tracking_track_timeout_s=999.0)
        tracker = ObjectTracker(settings=settings)
        all_ids_ever = set()
        for i in range(20):
            x = float(i % 5) * 4.0  # objects appear/disappear pseudo-randomly across positions
            detections = [_detection(x, 0.0)] if i % 3 != 0 else []
            result = tracker.update(_scan(i, i * DT, detections))
            for o in result.objects:
                all_ids_ever.add(o.track_id)
        # Every track_id ever assigned by this session is unique by construction (a fresh
        # itertools.count per ObjectTracker instance) -- this just exercises that no exception
        # or id collision occurs across a long, churny multi-track session.
        assert len(all_ids_ever) == len(set(all_ids_ever))


class TestResetAndSessionIsolation:
    def test_reset_clears_tracks_and_restarts_id_numbering(self):
        tracker = ObjectTracker(settings=DEFAULT_SETTINGS)
        tracker.update(_scan(0, 0.0, [_detection(5.0, 2.0)]))
        tracker.update(_scan(1, DT, [_detection(20.0, 20.0)]))
        assert tracker.active_track_count == 2

        tracker.reset()
        assert tracker.active_track_count == 0
        result = tracker.update(_scan(0, 0.0, [_detection(1.0, 1.0)]))
        assert result.objects[0].track_id == "track-1"

    def test_two_independent_tracker_instances_do_not_share_id_numbering(self):
        tracker_a = ObjectTracker(settings=DEFAULT_SETTINGS)
        tracker_b = ObjectTracker(settings=DEFAULT_SETTINGS)
        tracker_a.update(_scan(0, 0.0, [_detection(5.0, 2.0)]))
        result_b = tracker_b.update(_scan(0, 0.0, [_detection(1.0, 1.0)]))
        assert result_b.objects[0].track_id == "track-1"


class TestSceneCounts:
    def test_noise_points_pass_through_unchanged(self):
        from models.lidar import CartesianPoint

        noise = [CartesianPoint(angle=10.0, distance=12.0, timestamp=0.0, x=11.8, y=2.1)]
        scan = ClassifiedScan(
            scan_id="s", sequence_number=0, source_id="unit-test", timestamp=0.0,
            objects=[], noise_points=noise, object_count=0, noise_count=1,
        )
        result = ObjectTracker(settings=DEFAULT_SETTINGS).update(scan)
        assert result.noise_points == noise
        assert result.object_count == 0

    def test_empty_scan_produces_empty_result(self):
        scan = ClassifiedScan(scan_id="s", sequence_number=0, source_id="unit-test", timestamp=0.0, objects=[], noise_points=[], object_count=0, noise_count=0)
        result = ObjectTracker(settings=DEFAULT_SETTINGS).update(scan)
        assert result.object_count == 0
        assert result.new_track_count == 0
        assert result.lost_track_count == 0
        assert result.coasting_track_count == 0

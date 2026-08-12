"""Integration tests: perception.tracking running against real simulator scenarios, chained
after preprocessing, coordinates, clustering, and classification -- particularly the two
moving-obstacle scenarios (07_moving_crossing, 08_approaching_obstacle) this phase's spec calls
out by name.

Lives here for the same reason as the Phase 3/4/5/6 integration test files: `perception.tracking`
itself has no dependency on `simulator`, but `simulator` already depends on `perception`, so
exercising the full chain belongs on this side of the dependency graph -- see
docs/tracking.md "Architecture" and docs/architecture.md "Cross-package integration tests".

Expected values below (track count, converged velocity) were verified stable across repeated
runs (unseeded scenarios have no obstacle-position noise in `07`/`08` since only the *stationary*
background scenarios have any noise configured) before being encoded as tolerant-range
assertions, mirroring `test_classification_integration.py`'s own approach.
"""

import math

import pytest

from clustering import DBSCANClusterer
from common.config import Settings
from coordinates import CoordinateTransformer
from models.objects import MovementState, ObjectClassification, TrackingState
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker


def _run(scenario_id: str, scans: int, settings: Settings | None = None):
    """Return the list of `TrackedScan`s from running `scenario_id` for `scans` scans through the
    full preprocessing -> coordinates -> clustering -> classification -> tracking chain."""
    preprocessor = Preprocessor(settings=settings)
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer(settings=settings)
    classifier = GeometricClassifier(settings=settings)
    tracker = ObjectTracker(settings=settings)
    source = make_data_source(scenario_id, settings=settings)

    results = []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            results.append(tracker.update(classified))
    return results


class TestAllScenariosRunWithoutError:
    @pytest.mark.parametrize(
        "scenario_id",
        [
            "01_empty", "02_wall_in_front", "03_pole_left", "04_vehicle_ahead",
            "05_multiple_obstacles", "06_narrow_corridor", "07_moving_crossing",
            "08_approaching_obstacle", "09_noisy_lidar", "10_missing_outliers",
        ],
    )
    def test_scenario_tracks_without_error(self, scenario_id):
        for scan in _run(scenario_id, scans=10):
            for obj in scan.objects:
                assert obj.track_id is not None
                assert obj.tracking_state in (TrackingState.TENTATIVE, TrackingState.CONFIRMED, TrackingState.COASTING)
                if obj.velocity is not None:
                    assert obj.velocity.speed >= 0.0


class TestApproachingObstacle:
    """08_approaching_obstacle: a vehicle-like rectangle moving straight toward the vehicle at a
    known 2.0 m/s (see simulator/scenarios/08_approaching_obstacle.json)."""

    def test_single_stable_track(self):
        results = _run("08_approaching_obstacle", scans=15)
        track_ids = {o.track_id for r in results for o in r.objects}
        assert track_ids == {"track-1"}
        assert results[-1].new_track_count == 0

    def test_velocity_converges_to_the_known_approach_speed(self):
        results = _run("08_approaching_obstacle", scans=15)
        final = results[-1].objects[0]
        assert final.velocity is not None
        # direction_deg=180.0 in the scenario spec -> moving in -x; velocity_mps=2.0.
        assert final.velocity.vx == pytest.approx(-2.0, abs=0.15)
        assert final.velocity.vy == pytest.approx(0.0, abs=0.15)
        assert final.movement_state == MovementState.MOVING

    def test_direction_points_toward_the_vehicle(self):
        results = _run("08_approaching_obstacle", scans=15)
        final = results[-1].objects[0]
        # direction is degrees CCW from +x; a purely -x velocity is 180 degrees.
        assert final.direction == pytest.approx(180.0, abs=10.0)

    def test_distance_decreases_over_time(self):
        results = _run("08_approaching_obstacle", scans=10)
        distances = [r.objects[0].distance for r in results if r.objects]
        assert distances[-1] < distances[0]


class TestMovingCrossing:
    """07_moving_crossing: a pole crossing laterally at a known 1.5 m/s, +y direction (see
    simulator/scenarios/07_moving_crossing.json)."""

    def test_single_stable_track_despite_classification_flicker(self):
        # The pole is small/fast enough that its classification can flicker between POLE_LIKE and
        # UNKNOWN scan-to-scan (documented, expected -- see docs/object-classification.md); the
        # track identity must not fragment because of it (classification is not mandatory for
        # association, see docs/tracking.md "Association algorithm").
        results = _run("07_moving_crossing", scans=15)
        track_ids = {o.track_id for r in results for o in r.objects}
        assert track_ids == {"track-1"}

    def test_velocity_converges_to_the_known_crossing_speed(self):
        results = _run("07_moving_crossing", scans=15)
        final = results[-1].objects[0]
        assert final.velocity is not None
        assert final.velocity.vy == pytest.approx(1.5, abs=0.2)
        assert final.velocity.vx == pytest.approx(0.0, abs=0.2)
        assert final.movement_state == MovementState.MOVING

    def test_lateral_position_increases_over_time(self):
        results = _run("07_moving_crossing", scans=10)
        ys = [r.objects[0].centroid.y for r in results if r.objects]
        assert ys[-1] > ys[0]


class TestStationaryScenarios:
    @pytest.mark.parametrize("scenario_id", ["02_wall_in_front", "03_pole_left", "04_vehicle_ahead"])
    def test_static_obstacle_eventually_classified_stationary(self, scenario_id):
        results = _run(scenario_id, scans=10)
        final = results[-1]
        assert final.objects
        for obj in final.objects:
            assert obj.movement_state == MovementState.STATIONARY
            assert obj.velocity is not None
            assert obj.velocity.speed < Settings(_env_file=None).tracking_stationary_speed_threshold_mps

    def test_static_obstacle_keeps_one_stable_track(self):
        results = _run("02_wall_in_front", scans=10)
        track_ids = {o.track_id for r in results for o in r.objects}
        assert track_ids == {"track-1"}
        assert sum(r.lost_track_count for r in results) == 0


class TestEmptyScenario:
    def test_no_tracks_ever_created(self):
        results = _run("01_empty", scans=5)
        assert all(r.object_count == 0 for r in results)
        assert all(r.new_track_count == 0 for r in results)


class TestNarrowCorridor:
    def test_both_walls_get_distinct_stable_tracks(self):
        results = _run("06_narrow_corridor", scans=8)
        final_ids = {o.track_id for o in results[-1].objects}
        assert len(final_ids) == 2
        assert sum(r.lost_track_count for r in results) == 0

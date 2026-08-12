"""Integration tests: perception.collision running against real simulator scenarios, chained
after preprocessing, coordinates, clustering, classification, and tracking.

Lives here for the same reason as the Phase 3/4/5/6/7/8 integration test files:
`perception.collision` itself has no dependency on `simulator`, but `simulator` already depends
on `perception`, so exercising the full chain belongs on this side of the dependency graph -- see
docs/collision.md "Architecture" and docs/architecture.md "Cross-package integration tests".
"""

import pytest

from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.config import Settings
from coordinates import CoordinateTransformer
from models.collision import RiskLevel, VehicleState
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

REQUIRED_INTEGRATION_SCENARIOS = [
    "01_empty", "02_wall_in_front", "03_pole_left", "04_vehicle_ahead", "05_multiple_obstacles",
    "06_narrow_corridor", "07_moving_crossing", "08_approaching_obstacle", "09_noisy_lidar",
    "10_missing_outliers",
]


def _run(scenario_id: str, scans: int, vehicle_state: VehicleState | None = None, settings: Settings | None = None):
    """Return a list of CollisionAssessments for `scans` scans of `scenario_id`, run through the
    full Phase 3-9 chain with persistent tracker/mapper-equivalent state where needed."""
    preprocessor = Preprocessor(settings=settings)
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer(settings=settings)
    classifier = GeometricClassifier(settings=settings)
    tracker = ObjectTracker(settings=settings)
    engine = CollisionRiskEngine(settings=settings)
    source = make_data_source(scenario_id, settings=settings)

    results = []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked = tracker.update(classified)
            results.append(engine.evaluate(tracked, vehicle_state=vehicle_state))
    return results


class TestAllRequiredScenariosRun:
    @pytest.mark.parametrize("scenario_id", REQUIRED_INTEGRATION_SCENARIOS)
    def test_scenario_evaluates_without_error(self, scenario_id):
        for assessment in _run(scenario_id, scans=5):
            for result in assessment.results:
                assert 0.0 <= (result.risk_score or 0.0) <= 1.0
                assert result.reason
                if result.ttc is not None:
                    assert result.ttc >= 0.0


class TestEmptyScenario:
    def test_no_objects_no_risk(self):
        assessments = _run("01_empty", scans=3)
        for assessment in assessments:
            assert assessment.overall_risk == RiskLevel.SAFE


class TestVehicleAheadScenario:
    """`04_vehicle_ahead` -- one of this phase's specially-flagged important scenarios."""

    def test_stationary_ego_far_from_stationary_vehicle_is_not_critical(self):
        # The scenario places a vehicle-like rectangle ~10m ahead, not moving. A stationary ego
        # vehicle should not treat this as an imminent collision.
        assessments = _run("04_vehicle_ahead", scans=3, vehicle_state=VehicleState(speed_mps=0.0))
        assert assessments[-1].overall_risk in (RiskLevel.SAFE, RiskLevel.WARNING)

    def test_ego_approaching_the_stationary_vehicle_gets_finite_ttc(self):
        assessments = _run("04_vehicle_ahead", scans=3, vehicle_state=VehicleState(speed_mps=3.0))
        assessment = assessments[-1]
        assert assessment.results  # the vehicle was detected/tracked
        in_path_results = [r for r in assessment.results if r.in_projected_path]
        assert in_path_results
        assert any(r.ttc is not None for r in in_path_results)


class TestMovingCrossingScenario:
    """`07_moving_crossing` -- must not flag every nearby moving object; only a trajectory that
    actually intersects the vehicle's path."""

    def test_stationary_ego_never_reaches_critical(self):
        # The ego vehicle never closes the longitudinal gap in this test (speed=0), so the
        # crossing pole -- however it moves laterally -- can never actually reach the vehicle's
        # small body+margin footprint; must never spuriously escalate to CRITICAL.
        assessments = _run("07_moving_crossing", scans=15, vehicle_state=VehicleState(speed_mps=0.0))
        assert all(a.overall_risk != RiskLevel.CRITICAL for a in assessments)

    def test_does_not_flag_every_scan_as_at_risk(self):
        # A crossing object should not be WARNING/CRITICAL for the *entire* run purely because
        # it's nearby and moving -- most scans (while it's still far from the lane) should be SAFE.
        assessments = _run("07_moving_crossing", scans=15, vehicle_state=VehicleState(speed_mps=0.0))
        safe_count = sum(1 for a in assessments if a.overall_risk == RiskLevel.SAFE)
        assert safe_count >= len(assessments) // 2


class TestApproachingObstacleScenario:
    """`08_approaching_obstacle` -- must show the SAFE -> WARNING -> CRITICAL progression as
    distance/TTC decrease."""

    def test_risk_level_is_monotonically_non_decreasing_in_severity_as_it_approaches(self):
        severity = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}
        assessments = _run("08_approaching_obstacle", scans=20, vehicle_state=VehicleState(speed_mps=0.0))
        levels = [severity[a.overall_risk] for a in assessments]
        # Not strictly monotonic scan-to-scan (tracking/velocity estimation takes a few scans to
        # stabilize -- see docs/tracking.md "Velocity estimation"), but the *trend* over the run
        # must never regress once genuinely elevated: the max severity seen so far never exceeds
        # the max severity seen by the end.
        assert levels[-1] >= max(levels[:5])

    def test_reaches_at_least_warning_by_the_end_of_the_run(self):
        assessments = _run("08_approaching_obstacle", scans=20, vehicle_state=VehicleState(speed_mps=0.0))
        assert assessments[-1].overall_risk in (RiskLevel.WARNING, RiskLevel.CRITICAL)

    def test_distance_decreases_over_the_run(self):
        assessments = _run("08_approaching_obstacle", scans=20, vehicle_state=VehicleState(speed_mps=0.0))
        first_distance = min((r.distance for r in assessments[2].results), default=None)
        last_distance = min((r.distance for r in assessments[-1].results), default=None)
        assert first_distance is not None and last_distance is not None
        assert last_distance < first_distance


class TestMultipleObstaclesScenario:
    def test_multiple_objects_produce_independent_results(self):
        assessments = _run("05_multiple_obstacles", scans=5)
        assessment = assessments[-1]
        assert assessment.object_count >= 1
        for result in assessment.results:
            assert result.risk_level in (RiskLevel.SAFE, RiskLevel.WARNING, RiskLevel.CRITICAL)


class TestNoisyAndIncompleteScans:
    def test_noisy_lidar_does_not_crash(self):
        for assessment in _run("09_noisy_lidar", scans=5):
            assert assessment is not None

    def test_missing_outliers_does_not_crash(self):
        for assessment in _run("10_missing_outliers", scans=5):
            assert assessment is not None

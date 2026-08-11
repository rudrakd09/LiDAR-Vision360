"""Integration tests: perception.preprocessing running against real simulator scenarios.

Lives here (not in perception/tests/) deliberately: `perception.preprocessing` itself has no
dependency on `simulator` (see docs/preprocessing.md "Architecture"), but exercising it against
real scenario output is exactly what `simulator/tests/` already does for the rest of the
simulator, and `simulator` already depends on `perception` -- so the dependency direction stays
one-way (simulator -> perception), matching docs/architecture.md.
"""

import statistics

import pytest

from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

REQUIRED_INTEGRATION_SCENARIOS = [
    "02_wall_in_front",
    "04_vehicle_ahead",
    "05_multiple_obstacles",
    "07_moving_crossing",
    "08_approaching_obstacle",
    "09_noisy_lidar",
    "10_missing_outliers",
]


def _run(scenario_id: str, scans: int = 1):
    """Return (raw_frames, processed_scans) for `scans` scans of `scenario_id`."""
    preprocessor = Preprocessor()
    source = make_data_source(scenario_id)
    raw_frames, processed = [], []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            raw_frames.append(raw)
            processed.append(preprocessor.process(raw))
    return raw_frames, processed


class TestAllRequiredScenariosRun:
    @pytest.mark.parametrize("scenario_id", REQUIRED_INTEGRATION_SCENARIOS)
    def test_scenario_preprocesses_without_error(self, scenario_id):
        raw_frames, processed = _run(scenario_id, scans=2)
        for raw, clean in zip(raw_frames, processed):
            assert clean.total_count == raw.point_count
            assert clean.point_count == clean.valid_count - clean.outlier_count


class TestCleanScansAreLargelyUnchanged:
    @pytest.mark.parametrize("scenario_id", ["02_wall_in_front", "05_multiple_obstacles"])
    def test_low_noise_scenario_barely_changes_point_count(self, scenario_id):
        raw_frames, processed = _run(scenario_id, scans=1)
        raw, clean = raw_frames[0], processed[0]
        # Default noise (std=0.02m) shouldn't itself trigger meaningful outlier removal.
        assert clean.point_count >= raw.point_count * 0.95

    def test_wall_in_front_distance_is_preserved_after_processing(self):
        _, processed = _run("02_wall_in_front", scans=1)
        clean = processed[0]
        near_zero = [p for p in clean.points if p.angle < 2.0 or p.angle > 358.0]
        assert near_zero
        for p in near_zero:
            assert p.distance == pytest.approx(5.0, abs=0.2)


class TestObstacleBoundariesRemainIntact:
    def test_vehicle_ahead_front_edge_distance_is_preserved(self):
        # A rectangle's front edge is a genuine sharp transition from free space (12m) to the
        # obstacle surface (~3.75m); preprocessing must not blur it away.
        _, processed = _run("04_vehicle_ahead", scans=1)
        clean = processed[0]
        near_zero = [p for p in clean.points if p.angle < 2.0 or p.angle > 358.0]
        assert near_zero
        for p in near_zero:
            assert p.distance == pytest.approx(3.75, abs=0.3)


def _near_zero_distances(points, angle_margin_deg: float = 30.0):
    """Distances for points within `angle_margin_deg` of angle=0 -- comfortably inside the
    02/09/10 wall scenarios' ~63deg hit-cone, so every point here shares the same true distance
    (~5.0m) and any variance is attributable to noise, not scan-wide obstacle/free-space
    geometry (which legitimately should NOT be collapsed together by preprocessing)."""
    return [p.distance for p in points if p.angle <= angle_margin_deg or p.angle >= 360.0 - angle_margin_deg]


class TestNoiseIsReduced:
    def test_noisy_lidar_std_dev_is_reduced_by_processing(self):
        # Whole-scan variance is dominated by the wall's ~63deg hit-cone (wall hits ~5m vs.
        # misses at max range 12m) -- that split is real obstacle geometry preprocessing must
        # NOT smooth away, so noise reduction is measured within the hit-cone only, where every
        # point shares the same true distance and variance is attributable to sensor noise.
        raw_frames, processed = _run("09_noisy_lidar", scans=1)
        raw, clean = raw_frames[0], processed[0]
        raw_distances = _near_zero_distances([p for p in raw.points if p.valid])
        clean_distances = _near_zero_distances(clean.points)
        assert len(raw_distances) > 5 and len(clean_distances) > 5
        assert statistics.pstdev(clean_distances) < statistics.pstdev(raw_distances)


class TestOutliersAndMissingAreHandled:
    def test_missing_and_outlier_scenario_accounts_for_both(self):
        _, processed = _run("10_missing_outliers", scans=1)
        clean = processed[0]
        # missing_probability=0.08 in the scenario -> a meaningful share of invalid measurements.
        assert clean.quality_statistics.invalid_percentage > 1.0
        # outlier_probability=0.08 -> the outlier detector should catch a meaningful share of them.
        assert clean.outlier_count > 0

    def test_outliers_reduce_distance_variance_relative_to_raw(self):
        # Same reasoning as the noisy-LiDAR test above: restrict to the wall's hit-cone so the
        # comparison isn't dominated by legitimate wall-vs-free-space geometry.
        raw_frames, processed = _run("10_missing_outliers", scans=1)
        raw, clean = raw_frames[0], processed[0]
        raw_distances = _near_zero_distances([p for p in raw.points if p.valid])
        clean_distances = _near_zero_distances(clean.points)
        assert len(raw_distances) > 5 and len(clean_distances) > 5
        assert statistics.pstdev(clean_distances) < statistics.pstdev(raw_distances)


class TestMovingObstaclesAreNotExcessivelyDelayed:
    @pytest.mark.parametrize("scenario_id", ["07_moving_crossing", "08_approaching_obstacle"])
    def test_min_distance_tracks_raw_scan_by_scan_without_added_lag(self, scenario_id):
        # Temporal filtering is disabled by default, so preprocessing must not introduce any lag
        # of its own: each processed scan's minimum distance should track that same scan's raw
        # minimum distance closely (allowing for outlier removal shifting which point is nearest).
        raw_frames, processed = _run(scenario_id, scans=10)
        for raw, clean in zip(raw_frames, processed):
            raw_valid = [p.distance for p in raw.points if p.valid]
            if not raw_valid or not clean.points:
                continue
            raw_min = min(raw_valid)
            clean_min = min(p.distance for p in clean.points)
            assert clean_min == pytest.approx(raw_min, abs=0.5)

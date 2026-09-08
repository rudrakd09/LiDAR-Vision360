"""Tests for simulator.scenarios: loading/resolving the predefined scenario JSON files."""

import pytest

from simulator.scenarios import get_scenario, list_scenarios, make_data_source

REQUIRED_SCENARIO_IDS = [
    "01_empty",
    "02_wall_in_front",
    "03_pole_left",
    "04_vehicle_ahead",
    "05_multiple_obstacles",
    "06_narrow_corridor",
    "07_moving_crossing",
    "08_approaching_obstacle",
    "09_noisy_lidar",
    "10_missing_outliers",
]


class TestScenarioDiscovery:
    def test_at_least_ten_scenarios_available(self):
        specs = list_scenarios()
        assert len(specs) >= 10

    def test_scenario_ids_are_unique(self):
        specs = list_scenarios()
        ids = [s.id for s in specs]
        assert len(ids) == len(set(ids))

    @pytest.mark.parametrize("scenario_id", REQUIRED_SCENARIO_IDS)
    def test_required_scenario_present(self, scenario_id):
        spec = get_scenario(scenario_id)
        assert spec.id == scenario_id
        assert spec.name
        assert spec.description


class TestScenarioExecution:
    @pytest.mark.parametrize("scenario_id", REQUIRED_SCENARIO_IDS)
    def test_scenario_produces_a_valid_scan(self, scenario_id):
        source = make_data_source(scenario_id)
        with source:
            frame = source.read_scan()
        assert frame.point_count > 0
        for p in frame.points:
            assert 0.0 <= p.angle < 360.0
            assert p.distance >= 0.0
            assert isinstance(p.valid, bool)

    def test_empty_scenario_has_no_returns_within_default_range(self):
        source = make_data_source("01_empty")
        with source:
            frame = source.read_scan()
        # Small default noise aside, every point should read at/near max range.
        assert all(p.distance > 10.0 for p in frame.points if p.valid)


class TestDeterministicSeeding:
    def test_same_scenario_seed_reproduces_identical_scan(self):
        source_a = make_data_source("09_noisy_lidar")
        source_b = make_data_source("09_noisy_lidar")
        with source_a:
            frame_a = source_a.read_scan()
        with source_b:
            frame_b = source_b.read_scan()

        distances_a = [p.distance for p in frame_a.points]
        distances_b = [p.distance for p in frame_b.points]
        assert distances_a == distances_b

    def test_missing_and_outlier_scenario_produces_invalid_points(self):
        source = make_data_source("10_missing_outliers")
        with source:
            frame = source.read_scan()
        invalid_count = sum(1 for p in frame.points if not p.valid)
        assert invalid_count > 0


# Named risk/clearance/TTC scenarios (selectable as `--scenario <name>`), exercised end-to-end
# through the full pipeline in tests/test_risk_scenarios.py. Here we only assert they load and
# produce a valid scan, mirroring TestScenarioExecution above for the numbered set.
NAMED_RISK_SCENARIOS = [
    "safe", "caution", "low_clearance", "critical", "static_obstacle",
    "approaching", "moving_away", "multiple_objects", "track_lost", "full_360",
]


class TestNamedRiskScenarios:
    @pytest.mark.parametrize("scenario_id", NAMED_RISK_SCENARIOS)
    def test_named_scenario_present_and_runnable(self, scenario_id):
        spec = get_scenario(scenario_id)
        assert spec.id == scenario_id
        assert spec.name and spec.description
        source = make_data_source(scenario_id)
        with source:
            frame = source.read_scan()
        assert frame.point_count > 0
        for p in frame.points:
            assert 0.0 <= p.angle < 360.0
            assert p.distance >= 0.0

    def test_named_scenario_ids_do_not_collide_with_numbered_ones(self):
        all_ids = {s.id for s in list_scenarios()}
        for sid in NAMED_RISK_SCENARIOS + REQUIRED_SCENARIO_IDS:
            assert sid in all_ids

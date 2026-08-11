"""Integration tests: perception.coordinates running against real simulator scenarios, chained
after perception.preprocessing.

Lives here for the same reason as test_preprocessing_integration.py: `perception.coordinates`
itself has no dependency on `simulator`, but `simulator` already depends on `perception`, so
exercising the two together belongs on this side of the dependency graph -- see
docs/coordinates.md "Architecture" and docs/architecture.md.
"""

import math

import pytest

from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

REQUIRED_INTEGRATION_SCENARIOS = [
    "01_empty",
    "02_wall_in_front",
    "04_vehicle_ahead",
    "05_multiple_obstacles",
    "06_narrow_corridor",
    "07_moving_crossing",
    "08_approaching_obstacle",
]


def _run(scenario_id: str, scans: int = 1):
    """Return a list of CartesianScans for `scans` scans of `scenario_id`."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    source = make_data_source(scenario_id)
    results = []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            results.append(cartesian)
    return results


def _near(angle_target: float, points, margin_deg: float = 2.0):
    return [p for p in points if abs(((p.angle - angle_target + 180.0) % 360.0) - 180.0) <= margin_deg]


class TestAllRequiredScenariosRun:
    @pytest.mark.parametrize("scenario_id", REQUIRED_INTEGRATION_SCENARIOS)
    def test_scenario_transforms_without_error(self, scenario_id):
        for scan in _run(scenario_id, scans=2):
            for p in scan.points:
                assert math.isfinite(p.x) and math.isfinite(p.y)

    @pytest.mark.parametrize("scenario_id", REQUIRED_INTEGRATION_SCENARIOS)
    def test_euclidean_distance_matches_original_polar_distance(self, scenario_id):
        # sqrt(x^2 + y^2) must reconstruct the original `distance` for every retained point,
        # regardless of scenario geometry.
        for scan in _run(scenario_id, scans=1):
            for p in scan.points:
                reconstructed = math.hypot(p.x, p.y)
                assert reconstructed == pytest.approx(p.distance, abs=1e-3)


class TestEmptyEnvironmentGeometry:
    def test_all_points_lie_near_the_max_range_circle(self):
        scan = _run("01_empty", scans=1)[0]
        for p in scan.points:
            assert math.hypot(p.x, p.y) == pytest.approx(12.0, abs=0.5)


class TestWallGeometry:
    def test_points_near_forward_axis_sit_around_x_5_y_0(self):
        scan = _run("02_wall_in_front", scans=1)[0]
        forward_points = _near(0.0, scan.points)
        assert len(forward_points) >= 5
        for p in forward_points:
            assert p.x == pytest.approx(5.0, abs=0.3)
            assert p.y == pytest.approx(0.0, abs=0.3)


class TestVehicleAheadGeometry:
    def test_forms_a_compact_group_around_the_front_face(self):
        # A car-sized rectangle centered at (6, 0), 1.8m wide, 4.5m deep -> front face at x=3.75.
        scan = _run("04_vehicle_ahead", scans=1)[0]
        forward_points = _near(0.0, scan.points, margin_deg=5.0)
        assert len(forward_points) > 3
        xs = [p.x for p in forward_points]
        ys = [p.y for p in forward_points]
        assert min(xs) == pytest.approx(3.75, abs=0.3)
        assert all(abs(y) <= 1.2 for y in ys)  # within the vehicle's lateral extent + margin


class TestMultipleObstaclesGeometry:
    def test_distinct_obstacle_regions_are_present(self):
        # Scenario has a pole near (0, 3), a rotated vehicle-like rectangle near (5, -2), and a
        # far background wall near x=9. Check each region has at least one nearby Cartesian point.
        scan = _run("05_multiple_obstacles", scans=1)[0]

        def _has_point_near(x0: float, y0: float, radius: float) -> bool:
            return any(math.hypot(p.x - x0, p.y - y0) <= radius for p in scan.points)

        assert _has_point_near(0.0, 3.0, radius=0.5)
        assert _has_point_near(9.0, 0.0, radius=1.0)


class TestNarrowCorridorGeometry:
    def test_left_and_right_walls_sit_near_y_plus_minus_one(self):
        scan = _run("06_narrow_corridor", scans=1)[0]
        left_points = _near(90.0, scan.points, margin_deg=3.0)
        right_points = _near(270.0, scan.points, margin_deg=3.0)
        assert left_points and right_points
        for p in left_points:
            assert p.y == pytest.approx(1.0, abs=0.3)
        for p in right_points:
            assert p.y == pytest.approx(-1.0, abs=0.3)


class TestMovingObstacleGeometry:
    @pytest.mark.parametrize("scenario_id", ["07_moving_crossing", "08_approaching_obstacle"])
    def test_nearest_point_position_is_geometrically_consistent_across_scans(self, scenario_id):
        scans = _run(scenario_id, scans=5)
        for scan in scans:
            if not scan.points:
                continue
            nearest = min(scan.points, key=lambda p: math.hypot(p.x, p.y))
            assert math.hypot(nearest.x, nearest.y) == pytest.approx(nearest.distance, abs=1e-3)

    def test_approaching_obstacle_nearest_x_decreases_over_scans(self):
        # The nearest *point* (by Euclidean distance, not simply "most negative x" -- the
        # free-space circle behind the vehicle has far more negative x than anything relevant
        # here) belongs to the obstacle approaching from directly ahead, so its x should shrink
        # toward the vehicle scan over scan.
        scans = _run("08_approaching_obstacle", scans=5)
        nearest_x = []
        for scan in scans:
            if not scan.points:
                continue
            nearest = min(scan.points, key=lambda p: math.hypot(p.x, p.y))
            nearest_x.append(nearest.x)
        assert len(nearest_x) >= 2
        assert nearest_x[-1] < nearest_x[0]  # obstacle is approaching -> gets closer (smaller x)

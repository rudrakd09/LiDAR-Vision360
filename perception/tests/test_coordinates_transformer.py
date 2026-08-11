"""Tests for coordinates.transformer: polar_to_cartesian (pure, vectorized) and
CoordinateTransformer (PreprocessedScan -> CartesianScan)."""

import math

import numpy as np
import pytest

from coordinates.transformer import CoordinateTransformer, polar_to_cartesian, transform_scan
from models.lidar import LiDARPoint
from models.preprocessing import PreprocessedScan
from preprocessing.quality import compute_quality_statistics

ABS_TOL = 1e-6


def _preprocessed_scan(points: list[LiDARPoint], scan_id: str = "scan-1", sequence_number: int = 0) -> PreprocessedScan:
    """Build a `PreprocessedScan` directly (bypassing `Preprocessor`) so these are pure,
    isolated unit tests of the coordinate transformer -- not affected by Phase 3's
    window-based outlier/noise filtering (which, correctly, behaves differently on the tiny
    2-4 point scans used below than it would on a realistic 360-point scan)."""
    stats = compute_quality_statistics(len(points), len(points), 0, 0, points)
    return PreprocessedScan(
        scan_id=scan_id, sequence_number=sequence_number, source_id="unit-test", timestamp=1000.0,
        points=points, total_count=len(points), valid_count=len(points), invalid_count=0, outlier_count=0,
        quality_statistics=stats,
    )


def _point(angle: float, distance: float, timestamp: float = 1000.0) -> LiDARPoint:
    return LiDARPoint(angle=angle, distance=distance, timestamp=timestamp)


class TestCardinalAngles:
    """The spec's exact worked examples."""

    def test_zero_degrees(self):
        x, y = polar_to_cartesian(np.array([0.0]), np.array([5.0]))
        assert x[0] == pytest.approx(5.0, abs=ABS_TOL)
        assert y[0] == pytest.approx(0.0, abs=ABS_TOL)

    def test_ninety_degrees(self):
        x, y = polar_to_cartesian(np.array([90.0]), np.array([3.0]))
        assert x[0] == pytest.approx(0.0, abs=ABS_TOL)
        assert y[0] == pytest.approx(3.0, abs=ABS_TOL)

    def test_one_eighty_degrees(self):
        x, y = polar_to_cartesian(np.array([180.0]), np.array([4.0]))
        assert x[0] == pytest.approx(-4.0, abs=ABS_TOL)
        assert y[0] == pytest.approx(0.0, abs=ABS_TOL)

    def test_two_seventy_degrees(self):
        x, y = polar_to_cartesian(np.array([270.0]), np.array([2.0]))
        assert x[0] == pytest.approx(0.0, abs=ABS_TOL)
        assert y[0] == pytest.approx(-2.0, abs=ABS_TOL)

    def test_forty_five_degrees(self):
        x, y = polar_to_cartesian(np.array([45.0]), np.array([1.0]))
        expected = 1.0 / math.sqrt(2)
        assert x[0] == pytest.approx(expected, abs=ABS_TOL)
        assert y[0] == pytest.approx(expected, abs=ABS_TOL)


class TestAngleHandling:
    def test_negative_angle_matches_its_positive_equivalent(self):
        # -90 deg and 270 deg point the same direction; trig functions are periodic, so no
        # explicit normalization is needed for correct geometry.
        x_neg, y_neg = polar_to_cartesian(np.array([-90.0]), np.array([2.0]))
        x_pos, y_pos = polar_to_cartesian(np.array([270.0]), np.array([2.0]))
        assert x_neg[0] == pytest.approx(x_pos[0], abs=ABS_TOL)
        assert y_neg[0] == pytest.approx(y_pos[0], abs=ABS_TOL)

    def test_360_degrees_matches_0_degrees(self):
        x_360, y_360 = polar_to_cartesian(np.array([360.0]), np.array([5.0]))
        x_0, y_0 = polar_to_cartesian(np.array([0.0]), np.array([5.0]))
        assert x_360[0] == pytest.approx(x_0[0], abs=ABS_TOL)
        assert y_360[0] == pytest.approx(y_0[0], abs=ABS_TOL)

    def test_angle_greater_than_360_wraps_correctly(self):
        x_450, y_450 = polar_to_cartesian(np.array([450.0]), np.array([2.0]))  # 450 = 360 + 90
        x_90, y_90 = polar_to_cartesian(np.array([90.0]), np.array([2.0]))
        assert x_450[0] == pytest.approx(x_90[0], abs=ABS_TOL)
        assert y_450[0] == pytest.approx(y_90[0], abs=ABS_TOL)

    def test_original_angle_is_preserved_unchanged_in_output(self):
        scan = _preprocessed_scan([_point(angle=137.5, distance=3.0)])
        result = CoordinateTransformer().transform(scan)
        assert result.points[0].angle == 137.5


class TestMultiplePointsAndOrdering:
    def test_multiple_points_all_converted(self):
        points = [_point(angle=0.0, distance=5.0), _point(angle=1.0, distance=5.1), _point(angle=2.0, distance=5.2)]
        result = CoordinateTransformer().transform(_preprocessed_scan(points))
        assert result.point_count == 3

    def test_output_ordering_matches_input_ordering(self):
        # Deliberately unsorted input -- the transformer must not reorder anything.
        points = [_point(angle=300.0, distance=5.0), _point(angle=10.0, distance=5.0), _point(angle=150.0, distance=5.0)]
        result = CoordinateTransformer().transform(_preprocessed_scan(points))
        assert [p.angle for p in result.points] == [300.0, 10.0, 150.0]

    def test_angle_distance_xy_correspondence(self):
        points = [_point(angle=0.0, distance=5.0), _point(angle=90.0, distance=3.0), _point(angle=180.0, distance=4.0)]
        result = CoordinateTransformer().transform(_preprocessed_scan(points))
        by_angle = {p.angle: (p.distance, p.x, p.y) for p in result.points}
        assert by_angle[0.0] == pytest.approx((5.0, 5.0, 0.0), abs=ABS_TOL)
        assert by_angle[90.0] == pytest.approx((3.0, 0.0, 3.0), abs=ABS_TOL)
        assert by_angle[180.0] == pytest.approx((4.0, -4.0, 0.0), abs=ABS_TOL)


class TestEmptyAndDegenerateScans:
    def test_empty_scan_returns_empty_cartesian_scan(self):
        result = CoordinateTransformer().transform(_preprocessed_scan([]))
        assert result.point_count == 0
        assert result.points == []

    def test_empty_scan_preserves_metadata(self):
        result = CoordinateTransformer().transform(_preprocessed_scan([], scan_id="abc", sequence_number=9))
        assert result.scan_id == "abc"
        assert result.sequence_number == 9

    def test_zero_distance_converts_to_origin(self):
        result = CoordinateTransformer().transform(_preprocessed_scan([_point(angle=45.0, distance=0.0)]))
        assert result.points[0].x == pytest.approx(0.0, abs=ABS_TOL)
        assert result.points[0].y == pytest.approx(0.0, abs=ABS_TOL)

    def test_near_zero_distance(self):
        result = CoordinateTransformer().transform(_preprocessed_scan([_point(angle=0.0, distance=0.001)]))
        assert result.points[0].x == pytest.approx(0.001, abs=ABS_TOL)
        assert result.points[0].y == pytest.approx(0.0, abs=ABS_TOL)

    def test_large_distance_value(self):
        result = CoordinateTransformer().transform(_preprocessed_scan([_point(angle=0.0, distance=100_000.0)]))
        assert result.points[0].x == pytest.approx(100_000.0, abs=1e-3)
        assert result.points[0].y == pytest.approx(0.0, abs=1e-3)


class TestInvalidMeasurementHandling:
    def test_nan_point_reaching_this_stage_is_dropped_not_crashed(self):
        # Simulates a malformed point that bypassed Phase 3 validation somehow (see
        # perception/tests/test_preprocessing_validation.py for the same pattern) -- the
        # transformer must not crash, and must not emit a NaN coordinate.
        good = _point(angle=0.0, distance=5.0)
        bad = LiDARPoint.model_construct(angle=math.nan, distance=math.nan, timestamp=1000.0, valid=True, intensity=None)
        result = CoordinateTransformer().transform(_preprocessed_scan([good, bad]))
        assert result.point_count == 1
        assert result.points[0].angle == 0.0
        assert all(math.isfinite(p.x) and math.isfinite(p.y) for p in result.points)

    def test_infinite_distance_reaching_this_stage_is_dropped_not_crashed(self):
        good = _point(angle=0.0, distance=5.0)
        bad = LiDARPoint.model_construct(angle=10.0, distance=math.inf, timestamp=1000.0, valid=True, intensity=None)
        result = CoordinateTransformer().transform(_preprocessed_scan([good, bad]))
        assert result.point_count == 1
        assert all(math.isfinite(p.x) and math.isfinite(p.y) for p in result.points)

    def test_out_of_range_angle_reaching_this_stage_is_dropped_not_crashed(self):
        # angle=400 fails CartesianPoint's own [0, 360) constraint at construction time --
        # covered here as the final defense-in-depth layer, not just polar_to_cartesian's math.
        good = _point(angle=0.0, distance=5.0)
        bad = LiDARPoint.model_construct(angle=400.0, distance=5.0, timestamp=1000.0, valid=True, intensity=None)
        result = CoordinateTransformer().transform(_preprocessed_scan([good, bad]))
        assert result.point_count == 1

    def test_all_points_malformed_returns_empty_without_crashing(self):
        bad = LiDARPoint.model_construct(angle=math.nan, distance=5.0, timestamp=1000.0, valid=True, intensity=None)
        result = CoordinateTransformer().transform(_preprocessed_scan([bad]))
        assert result.point_count == 0


class TestQualityStatisticsPassthrough:
    def test_quality_statistics_carried_through_unchanged(self):
        scan = _preprocessed_scan([_point(angle=0.0, distance=5.0)])
        result = CoordinateTransformer().transform(scan)
        assert result.quality_statistics == scan.quality_statistics


class TestDeterminism:
    def test_identical_input_produces_identical_output(self):
        points = [_point(angle=float(i), distance=5.0 + i * 0.01) for i in range(20)]
        scan = _preprocessed_scan(points)
        result_a = CoordinateTransformer().transform(scan)
        result_b = CoordinateTransformer().transform(scan)
        assert result_a == result_b

    def test_transform_scan_convenience_function_matches_transformer(self):
        scan = _preprocessed_scan([_point(angle=0.0, distance=5.0)])
        assert transform_scan(scan) == CoordinateTransformer().transform(scan)


class TestNumericalAccuracyAgainstManualTrig:
    @pytest.mark.parametrize("angle,distance", [(17.0, 6.3), (203.0, 1.1), (359.0, 9.9), (89.9, 0.5)])
    def test_matches_hand_computed_trig(self, angle, distance):
        x, y = polar_to_cartesian(np.array([angle]), np.array([distance]))
        expected_x = distance * math.cos(math.radians(angle))
        expected_y = distance * math.sin(math.radians(angle))
        assert x[0] == pytest.approx(expected_x, abs=ABS_TOL)
        assert y[0] == pytest.approx(expected_y, abs=ABS_TOL)

    def test_pythagorean_identity_holds(self):
        # x^2 + y^2 should equal distance^2 for any angle.
        angles = np.linspace(0, 359, 60)
        distances = np.full_like(angles, 7.5)
        x, y = polar_to_cartesian(angles, distances)
        recovered_distance = np.sqrt(x**2 + y**2)
        assert np.allclose(recovered_distance, distances, atol=ABS_TOL)

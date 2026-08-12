"""Tests for mapping.raytrace.bresenham_line: horizontal/vertical/diagonal/negative/short/long
rays, matching this phase's explicit ray-traversal correctness requirements."""

import pytest

from mapping.raytrace import bresenham_line


class TestHorizontal:
    def test_positive_direction(self):
        assert bresenham_line(0, 0, 0, 5) == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5)]

    def test_negative_direction(self):
        assert bresenham_line(0, 5, 0, 0) == [(0, 5), (0, 4), (0, 3), (0, 2), (0, 1), (0, 0)]


class TestVertical:
    def test_positive_direction(self):
        assert bresenham_line(0, 0, 5, 0) == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]

    def test_negative_direction(self):
        assert bresenham_line(5, 0, 0, 0) == [(5, 0), (4, 0), (3, 0), (2, 0), (1, 0), (0, 0)]


class TestDiagonal:
    def test_45_degrees_positive(self):
        assert bresenham_line(0, 0, 5, 5) == [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5)]

    def test_45_degrees_negative(self):
        assert bresenham_line(5, 5, 0, 0) == [(5, 5), (4, 4), (3, 3), (2, 2), (1, 1), (0, 0)]

    def test_anti_diagonal(self):
        assert bresenham_line(0, 5, 5, 0) == [(0, 5), (1, 4), (2, 3), (3, 2), (4, 1), (5, 0)]


class TestArbitraryAngle:
    def test_shallow_slope_visits_every_column_once(self):
        cells = bresenham_line(0, 0, 2, 10)
        columns = [c for _, c in cells]
        assert columns == list(range(11))  # every column 0..10 visited exactly once

    def test_steep_slope_visits_every_row_once(self):
        cells = bresenham_line(0, 0, 10, 2)
        rows = [r for r, _ in cells]
        assert rows == list(range(11))


class TestShortAndDegenerateRays:
    def test_single_cell_ray(self):
        assert bresenham_line(3, 3, 3, 3) == [(3, 3)]

    def test_two_adjacent_cells(self):
        assert bresenham_line(0, 0, 0, 1) == [(0, 0), (0, 1)]


class TestLongRay:
    def test_long_diagonal_ray_has_correct_length_and_endpoints(self):
        cells = bresenham_line(0, 0, 399, 399)
        assert len(cells) == 400
        assert cells[0] == (0, 0)
        assert cells[-1] == (399, 399)

    def test_long_ray_contains_no_duplicate_or_skipped_cells_on_axis(self):
        cells = bresenham_line(0, 0, 0, 399)
        assert len(cells) == 400
        assert len(set(cells)) == 400  # no duplicates
        assert [c for _, c in cells] == list(range(400))  # no gaps


class TestNegativeCoordinates:
    def test_negative_indices_supported(self):
        cells = bresenham_line(-5, -5, 5, 5)
        assert cells[0] == (-5, -5)
        assert cells[-1] == (5, 5)
        assert len(cells) == 11

    def test_crossing_zero(self):
        cells = bresenham_line(-2, 0, 2, 0)
        assert cells == [(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0)]


class TestEndpointsAlwaysIncluded:
    @pytest.mark.parametrize(
        "r0,c0,r1,c1",
        [(0, 0, 0, 5), (0, 0, 5, 0), (0, 0, 5, 5), (0, 0, 3, 7), (0, 0, 7, 3), (2, 2, 2, 2), (-3, 4, 6, -2)],
    )
    def test_first_and_last_cell_match_arguments(self, r0, c0, r1, c1):
        cells = bresenham_line(r0, c0, r1, c1)
        assert cells[0] == (r0, c0)
        assert cells[-1] == (r1, c1)

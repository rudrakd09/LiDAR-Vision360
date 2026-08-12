"""Tests for mapping.coordinate_transform: world <-> grid conversion, negative coordinates, and
different resolutions."""

import pytest

from mapping.coordinate_transform import grid_to_world, world_to_grid


class TestWorldToGrid:
    def test_origin_maps_to_cell_zero_zero(self):
        assert world_to_grid(0.0, 0.0, 0.0, 0.0, 0.1, 100, 100) == (0, 0)

    def test_positive_x_increases_column(self):
        assert world_to_grid(0.5, 0.0, 0.0, 0.0, 0.1, 100, 100) == (0, 5)

    def test_positive_y_increases_row(self):
        assert world_to_grid(0.0, 0.5, 0.0, 0.0, 0.1, 100, 100) == (5, 0)

    def test_out_of_bounds_negative_returns_none(self):
        assert world_to_grid(-1.0, -1.0, 0.0, 0.0, 0.1, 100, 100) is None

    def test_out_of_bounds_beyond_extent_returns_none(self):
        assert world_to_grid(20.0, 0.0, 0.0, 0.0, 0.1, 100, 100) is None

    def test_exactly_on_upper_boundary_is_out_of_bounds(self):
        # width=100 cells at 0.1m -> valid x in [0.0, 10.0); x=10.0 is the first out-of-bounds value.
        assert world_to_grid(10.0, 0.0, 0.0, 0.0, 0.1, 100, 100) is None
        assert world_to_grid(9.99, 0.0, 0.0, 0.0, 0.1, 100, 100) is not None

    def test_negative_origin_centers_correctly(self):
        # A grid whose origin is (-5, -5) puts world (0, 0) at its center cell.
        assert world_to_grid(0.0, 0.0, -5.0, -5.0, 0.1, 100, 100) == (50, 50)

    def test_negative_world_coordinates_with_negative_origin(self):
        assert world_to_grid(-4.9, -4.9, -5.0, -5.0, 0.1, 100, 100) == (0, 0)

    @pytest.mark.parametrize("resolution", [0.01, 0.05, 0.1, 0.5, 1.0])
    def test_different_resolutions_place_the_same_world_point_consistently(self, resolution):
        width_cells = int(20.0 / resolution)
        cell = world_to_grid(1.0, 1.0, -10.0, -10.0, resolution, width_cells, width_cells)
        assert cell is not None
        expected = int((1.0 - -10.0) / resolution)
        assert cell == (expected, expected)


class TestGridToWorld:
    def test_cell_zero_zero_is_origin_plus_half_resolution(self):
        assert grid_to_world(0, 0, 0.0, 0.0, 0.1) == pytest.approx((0.05, 0.05))

    def test_round_trips_back_into_the_same_cell(self):
        origin_x, origin_y, resolution = -5.0, -5.0, 0.1
        row, col = 37, 62
        x, y = grid_to_world(row, col, origin_x, origin_y, resolution)
        assert world_to_grid(x, y, origin_x, origin_y, resolution, 100, 100) == (row, col)

    def test_negative_origin(self):
        x, y = grid_to_world(0, 0, -5.0, -5.0, 0.1)
        assert (x, y) == pytest.approx((-4.95, -4.95))

    @pytest.mark.parametrize("resolution", [0.01, 0.05, 0.1, 0.5, 1.0])
    def test_different_resolutions(self, resolution):
        x, y = grid_to_world(0, 0, 0.0, 0.0, resolution)
        assert (x, y) == pytest.approx((resolution / 2.0, resolution / 2.0))

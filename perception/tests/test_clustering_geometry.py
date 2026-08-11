"""Tests for clustering.geometry.circular_angular_extent: the 0/360 boundary handling for a
cluster's angular-extent summary statistic."""

import pytest

from clustering.geometry import circular_angular_extent


class TestNonWrappingCases:
    def test_simple_contiguous_range(self):
        min_a, max_a, width = circular_angular_extent([10.0, 15.0, 20.0])
        assert min_a == 10.0
        assert max_a == 20.0
        assert width == pytest.approx(10.0)

    def test_single_angle(self):
        min_a, max_a, width = circular_angular_extent([42.0])
        assert min_a == max_a == 42.0
        assert width == 0.0

    def test_repeated_single_angle(self):
        min_a, max_a, width = circular_angular_extent([42.0, 42.0, 42.0])
        assert min_a == max_a == 42.0
        assert width == 0.0

    def test_empty_list(self):
        assert circular_angular_extent([]) == (0.0, 0.0, 0.0)


class TestWrappingCases:
    def test_spec_example_358_359_0_1_2(self):
        # The spec's exact example: these must be treated as one contiguous arc, not split.
        min_a, max_a, width = circular_angular_extent([358.0, 359.0, 0.0, 1.0, 2.0])
        assert min_a == 358.0
        assert max_a == 2.0
        assert width == pytest.approx(4.0)

    def test_min_greater_than_max_signals_a_wrap(self):
        min_a, max_a, _ = circular_angular_extent([355.0, 0.0, 5.0])
        assert min_a > max_a  # the documented signal that this arc crosses 0/360

    def test_wide_wrapping_arc(self):
        min_a, max_a, width = circular_angular_extent([300.0, 330.0, 0.0, 30.0, 60.0])
        assert min_a == 300.0
        assert max_a == 60.0
        assert width == pytest.approx(120.0)

    def test_single_point_exactly_at_zero(self):
        min_a, max_a, width = circular_angular_extent([0.0])
        assert min_a == max_a == 0.0
        assert width == 0.0


class TestSymmetricEdgeCase:
    def test_two_antipodal_points_pick_one_of_the_two_equally_valid_180deg_arcs(self):
        # Genuinely ambiguous (both directions are a valid shortest-arc answer); documented
        # behavior rather than asserting a specific winner matters here -- just that it's
        # self-consistent and doesn't crash.
        min_a, max_a, width = circular_angular_extent([0.0, 180.0])
        assert width == pytest.approx(180.0)

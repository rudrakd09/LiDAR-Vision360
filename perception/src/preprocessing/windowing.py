"""Shared circular-window helper for angle-ordered sequences.

Both outlier detection and the median noise filter need "the N measurements angularly nearest to
this one," wrapping correctly at the 0/360 boundary (358, 359, 0, 1, 2 must be treated as
neighbors). Both operate on a sequence already sorted by ascending angle, so "nearest in angle"
reduces to "nearest by array index, modulo the array length."
"""

from __future__ import annotations

from typing import Sequence, TypeVar

T = TypeVar("T")


def circular_window(values: Sequence[T], center: int, window_size: int) -> list[T]:
    """The `window_size` values angularly nearest to `values[center]`, including itself.

    Wraps around both ends of `values` (index `-1` is the last element, index `len(values)` is
    the first) so callers never need special-case the 0/360 boundary themselves.

    `window_size` is interpreted as an approximate total window width: the actual window is
    `2 * (window_size // 2) + 1` values (always odd, centered on `center`) -- e.g. both `4` and
    `5` produce a radius-2 (5-value) window. If `window_size >= len(values)`, every value is
    returned (the whole ring). Returns `list(values)` unchanged if there's nothing to window
    over (`len(values) <= 1`).
    """
    n = len(values)
    if n <= 1 or window_size >= n:
        return list(values)

    radius = window_size // 2
    return [values[(center + offset) % n] for offset in range(-radius, radius + 1)]

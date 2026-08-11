"""Circular-aware geometric helpers for cluster angular extent.

A cluster spanning the 0/360 boundary (e.g. a wall directly ahead, straddling angle=0) must not
be reported as spanning nearly the entire circle just because its raw angle values look
numerically far apart (359 and 1 differ by 358 numerically but are 2 degrees apart physically).
This module finds the shortest arc containing every angle, which handles that correctly with no
special-casing needed by callers.

Note this is a different concern from the clustering *decision* itself (which operates on (x, y)
and has no 0/360 discontinuity at all, see `clustering.dbscan`) -- this module is only for the
angular-extent *summary statistic* computed per cluster afterward.
"""

from __future__ import annotations


def circular_angular_extent(angles: list[float]) -> tuple[float, float, float]:
    """Return `(min_angle, max_angle, angular_width)`: the shortest arc (degrees) containing
    every angle in `angles`, treating the angle domain as circular (`[0, 360)` wrapping to
    itself).

    `min_angle`/`max_angle` are the arc's start/end going counter-clockwise; if the arc crosses
    the 0/360 boundary, `min_angle > max_angle` (e.g. `min_angle=355, max_angle=5` for a
    10-degree arc straddling 0). `angular_width` is always the true shortest-arc span regardless
    of whether it crosses the boundary.

    `angles=[]` returns `(0.0, 0.0, 0.0)`. A single distinct angle (possibly repeated) returns
    `(angle, angle, 0.0)`.
    """
    if not angles:
        return 0.0, 0.0, 0.0

    unique_sorted = sorted(set(angles))
    n = len(unique_sorted)
    if n == 1:
        return unique_sorted[0], unique_sorted[0], 0.0

    # The gap after each angle, wrapping the last back to the first (mod 360 handles that wrap
    # gap uniformly, no separate branch needed).
    gaps = [(unique_sorted[(i + 1) % n] - unique_sorted[i]) % 360.0 for i in range(n)]
    max_gap = max(gaps)
    max_gap_idx = gaps.index(max_gap)

    # Removing the single largest empty gap leaves the shortest arc covering every point; that
    # arc starts right after the gap and ends right before it.
    min_angle = unique_sorted[(max_gap_idx + 1) % n]
    max_angle = unique_sorted[max_gap_idx]
    angular_width = 360.0 - max_gap
    return min_angle, max_angle, angular_width

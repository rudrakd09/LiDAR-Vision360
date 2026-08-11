"""Local outlier detection for angle-ordered LiDAR measurements.

Algorithm: a Hampel-style local-median identifier. For each point, take the `window_size`
measurements angularly nearest to it (including itself, wrapping at 0/360 via
`windowing.circular_window`), compute their median distance, and flag the point as an outlier if
its own distance deviates from that local median by more than `threshold_m`.

Why this method: it directly implements the spec's definition of an outlier -- "isolated values
that are significantly different from their local surroundings" -- and, because the window is
dominated by whichever value is locally in the majority, a genuine step edge (several consecutive
points at a new, real distance) shifts the local median *with* the edge rather than flagging it:
a run of >= `window_size // 2 + 1` consecutive similar values is never flagged, only truly
isolated spikes are. See docs/preprocessing.md "Outlier Detection" for the worked examples this
was validated against (a single isolated spike vs. a multi-point boundary transition), and
`perception/tests/test_preprocessing_outliers.py` for both as automated tests.

Limitation: a genuine object narrower than roughly `window_size // 2` consecutive points can
still be attenuated/flagged, the same way any median filter blurs features narrower than its
window -- keep `outlier_window_size` small (default 5) relative to the angular width of the
smallest real object you need to preserve. Documented, not silently hidden.
"""

from __future__ import annotations

import statistics

from models.lidar import LiDARPoint

from .windowing import circular_window


def detect_outliers(points: list[LiDARPoint], threshold_m: float, window_size: int) -> tuple[list[LiDARPoint], int]:
    """Return `(points_with_outliers_removed, outlier_count)`.

    `points` must already be sorted by ascending angle (the pipeline guarantees this before
    calling here) so that array-adjacency matches angular adjacency. `window_size < 3` disables
    outlier detection entirely (no meaningful neighborhood to compare against) and returns every
    point unchanged.
    """
    n = len(points)
    if n == 0 or window_size < 3:
        return list(points), 0

    distances = [p.distance for p in points]
    kept: list[LiDARPoint] = []
    outlier_count = 0

    for i in range(n):
        local_median = statistics.median(circular_window(distances, i, window_size))
        if abs(distances[i] - local_median) > threshold_m:
            outlier_count += 1
        else:
            kept.append(points[i])

    return kept, outlier_count

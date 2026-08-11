"""Spatial noise filtering: a circular median filter over angle-ordered measurements.

Why a median filter (over moving average / exponential smoothing) as the baseline: it removes
small-amplitude jitter (sensor Gaussian noise) about as well as a moving average, but -- unlike a
mean-based filter -- it does not blur genuine step edges between an obstacle and free space,
because the median of a window straddling a step is pulled to whichever side has the majority of
points rather than being dragged toward the average of both sides. This matches the project's
"do not excessively smooth genuine obstacle boundaries" requirement directly. Exponential
smoothing has the same edge-blurring problem as a moving average and, being asymmetric (depends
on scan order rather than true angular neighbors), is a worse fit for a spatial filter -- it is
used instead for cross-scan *temporal* smoothing (`temporal.py`), where "previous" has a natural,
unambiguous meaning.
"""

from __future__ import annotations

import statistics

from models.lidar import LiDARPoint

from .windowing import circular_window


def median_filter(points: list[LiDARPoint], window_size: int) -> list[LiDARPoint]:
    """Return a new list with each point's `distance` replaced by its local circular median.

    `points` must already be sorted by ascending angle. `window_size <= 1` disables filtering
    (returns `points` unchanged) -- there is no window to take a median over.
    """
    n = len(points)
    if n == 0 or window_size <= 1:
        return list(points)

    distances = [p.distance for p in points]
    filtered: list[LiDARPoint] = []
    for i in range(n):
        local_median = statistics.median(circular_window(distances, i, window_size))
        filtered.append(points[i].model_copy(update={"distance": round(local_median, 4)}))

    return filtered

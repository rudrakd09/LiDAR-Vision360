"""A simple 2D top-down matplotlib debug view comparing a raw `ScanFrame` against its
`PreprocessedScan`. Debugging/validation tool only -- not the future Unity digital twin.

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily so
the rest of `preprocessing` has no import-time dependency on it, mirroring
`simulator.visualize`'s pattern.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from models.preprocessing import PreprocessedScan
from models.scan import ScanFrame

if TYPE_CHECKING:
    from matplotlib.axes import Axes


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def _to_xy(angle_deg: float, distance: float) -> tuple[float, float]:
    rad = math.radians(angle_deg)
    return distance * math.cos(rad), distance * math.sin(rad)


def plot_raw_vs_processed(raw: ScanFrame, processed: PreprocessedScan, ax: "Axes | None" = None):
    """Plot `raw`'s points and `processed`'s retained points on the same 2D axes for comparison.

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(
        f"scan #{processed.sequence_number}  raw={raw.point_count} pts  "
        f"processed={processed.point_count} pts  ({processed.invalid_count} invalid, "
        f"{processed.outlier_count} outliers)"
    )
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")

    ax.plot(0.0, 0.0, marker="+", color="black", markersize=12, label="vehicle / LiDAR origin")

    raw_xy = [_to_xy(p.angle, p.distance) for p in raw.points if p.valid]
    if raw_xy:
        ax.scatter(*zip(*raw_xy), s=10, color="lightcoral", alpha=0.6, label="raw points")

    processed_xy = [_to_xy(p.angle, p.distance) for p in processed.points]
    if processed_xy:
        ax.scatter(*zip(*processed_xy), s=8, color="tab:blue", label="processed points")

    ax.legend(loc="upper right", fontsize="small")
    return ax

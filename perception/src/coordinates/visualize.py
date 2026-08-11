"""A simple 2D top-down matplotlib debug view of a `CartesianScan`. Debugging/algorithm-validation
tool only -- not the future Unity digital twin.

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring `simulator.visualize` and `preprocessing.visualize`. Unlike those two, no on-the-fly
angle/distance -> x/y conversion is needed here -- `CartesianScan.points` already carry `x`/`y`
directly, which is the entire point of this pipeline stage.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from models.coordinates import CartesianScan

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


def plot_cartesian_scan(scan: CartesianScan, ax: "Axes | None" = None, distance_scale_m: float | None = None):
    """Plot a `CartesianScan`'s points in the vehicle-relative top-down frame (+x forward, +y
    left), with the LiDAR/vehicle origin marked and a labeled distance-scale ring.

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()
    from matplotlib.patches import Circle

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(f"scan #{scan.sequence_number}  ({scan.point_count} Cartesian points)")
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")

    ax.plot(0.0, 0.0, marker="+", color="black", markersize=12, label="LiDAR / vehicle origin")

    xs = [p.x for p in scan.points]
    ys = [p.y for p in scan.points]
    if xs:
        ax.scatter(xs, ys, s=8, color="tab:blue", label="Cartesian points")

    # A labeled distance-scale ring, sized to the data (or an explicit override).
    if distance_scale_m is None:
        max_r = max((p.distance for p in scan.points), default=1.0)
        distance_scale_m = max(1.0, round(max_r / 2.0, 1)) if max_r else 1.0
    for radius in (distance_scale_m, distance_scale_m * 2):
        ax.add_patch(Circle((0, 0), radius, fill=False, linestyle="--", color="lightgray", linewidth=0.8))
        ax.annotate(f"{radius:g}m", (radius, 0), fontsize="x-small", color="gray")

    ax.legend(loc="upper right", fontsize="small")
    return ax

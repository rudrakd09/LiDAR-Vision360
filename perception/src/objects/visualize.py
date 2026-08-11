"""A simple 2D top-down matplotlib debug view of a `ClassifiedScan`: each object's bounding box,
centroid, classification label, and confidence score; noise points shown separately.
Debugging/algorithm-validation tool only -- this is what will later map onto the Unity digital
twin (Phase 11), not a replacement for it.

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring every earlier phase's `visualize.py`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from models.classification import ClassifiedScan

if TYPE_CHECKING:
    from matplotlib.axes import Axes

_CLASS_COLORS = {
    "wall": "tab:gray",
    "vehicle_like": "tab:blue",
    "pole_like": "tab:orange",
    "person_like": "tab:red",
    "large_obstacle": "tab:purple",
    "unknown": "black",
}


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def plot_classified_scan(scan: ClassifiedScan, ax: "Axes | None" = None):
    """Plot every classified object (color-coded by category, with its bounding box, centroid,
    and a `LABEL confidence` text label) and every noise point (gray 'x').

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()
    from matplotlib.patches import Rectangle

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(f"scan #{scan.sequence_number}  {scan.object_count} object(s), {scan.noise_count} noise pt(s)")
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")
    ax.plot(0.0, 0.0, marker="+", color="black", markersize=12, label="LiDAR / vehicle origin")

    if scan.noise_points:
        ax.scatter(
            [p.x for p in scan.noise_points], [p.y for p in scan.noise_points],
            s=6, marker="x", color="lightgray", alpha=0.6, label="noise",
        )

    for obj in scan.objects:
        color = _CLASS_COLORS.get(obj.classification.value, "black")
        bb = obj.bounding_box
        if bb is not None:
            ax.add_patch(
                Rectangle(
                    (bb.min_x, bb.min_y), bb.max_x - bb.min_x, bb.max_y - bb.min_y,
                    fill=False, edgecolor=color, linewidth=1.4,
                )
            )
        ax.plot(obj.centroid.x, obj.centroid.y, marker="*", color=color, markersize=10)
        ax.annotate(
            f"#{obj.object_id} {obj.classification.value}\n{obj.confidence:.2f}",
            (obj.centroid.x, obj.centroid.y),
            fontsize="x-small", color=color, xytext=(4, 4), textcoords="offset points",
        )

    ax.legend(loc="upper right", fontsize="small")
    return ax

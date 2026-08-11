"""A simple 2D top-down matplotlib debug view of a `ClusteredScan`: distinct clusters (each with
a bounding box, centroid, ID, and distance label), and noise points shown separately.
Debugging/algorithm-validation tool only -- not the future Unity digital twin.

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring `simulator.visualize`, `preprocessing.visualize`, and `coordinates.visualize`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from models.clustering import ClusteredScan

if TYPE_CHECKING:
    from matplotlib.axes import Axes

_CLUSTER_COLORS = [
    "tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple",
    "tab:brown", "tab:pink", "tab:olive", "tab:cyan",
]


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def plot_clustered_scan(scan: ClusteredScan, ax: "Axes | None" = None):
    """Plot every cluster (distinct color, bounding box, centroid marker, ID + distance label)
    and every noise point (gray 'x') in the vehicle-relative top-down frame.

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()
    from matplotlib.patches import Rectangle

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(f"scan #{scan.sequence_number}  {scan.cluster_count} cluster(s), {scan.noise_count} noise pt(s)")
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")
    ax.plot(0.0, 0.0, marker="+", color="black", markersize=12, label="LiDAR / vehicle origin")

    if scan.noise_points:
        ax.scatter(
            [p.x for p in scan.noise_points], [p.y for p in scan.noise_points],
            s=6, marker="x", color="gray", alpha=0.5, label="noise",
        )

    for cluster in scan.clusters:
        color = _CLUSTER_COLORS[cluster.cluster_id % len(_CLUSTER_COLORS)]
        ax.scatter([p.x for p in cluster.points], [p.y for p in cluster.points], s=14, color=color)
        ax.add_patch(
            Rectangle(
                (cluster.min_x, cluster.min_y), cluster.width, cluster.depth,
                fill=False, edgecolor=color, linewidth=1.2,
            )
        )
        ax.plot(cluster.centroid_x, cluster.centroid_y, marker="*", color=color, markersize=10)
        ax.annotate(
            f"#{cluster.cluster_id} ({cluster.point_count}pt, {cluster.centroid_distance:.1f}m)",
            (cluster.centroid_x, cluster.centroid_y),
            fontsize="x-small", color=color, xytext=(4, 4), textcoords="offset points",
        )

    ax.legend(loc="upper right", fontsize="small")
    return ax

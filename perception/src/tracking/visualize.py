"""A simple 2D top-down matplotlib debug view of a `TrackedScan`: each tracked object's current
position (marker shape encodes tracking state), a velocity vector arrow (only once velocity is
reliable), a faint marker at the Kalman-predicted next position, and a
`#track_id classification (state)` text label; noise points shown separately. Debugging/
algorithm-validation tool only -- this is what will later map onto the Unity digital twin's
trajectory visualization (Phase 11), not a replacement for it. See docs/tracking.md
"Track visualization".

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring every earlier phase's `visualize.py` (`preprocessing`, `coordinates`, `clustering`,
`objects`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from models.tracking import TrackedScan

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

_STATE_MARKERS = {
    "tentative": "o",
    "confirmed": "*",
    "coasting": "^",
    "lost": "x",
}


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def plot_tracked_scan(scan: TrackedScan, ax: "Axes | None" = None):
    """Plot every tracked object and every noise point in `scan`.

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(
        f"scan #{scan.sequence_number}  {scan.object_count} track(s) "
        f"(+{scan.new_track_count} new, {scan.coasting_track_count} coasting, -{scan.lost_track_count} lost)"
    )
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
        state = obj.tracking_state.value if obj.tracking_state is not None else "tentative"
        marker = _STATE_MARKERS.get(state, "o")

        ax.plot(obj.centroid.x, obj.centroid.y, marker=marker, color=color, markersize=10)

        if obj.velocity is not None and (obj.velocity.vx or obj.velocity.vy):
            ax.annotate(
                "",
                xy=(obj.centroid.x + obj.velocity.vx, obj.centroid.y + obj.velocity.vy),
                xytext=(obj.centroid.x, obj.centroid.y),
                arrowprops=dict(arrowstyle="->", color=color, linewidth=1.6),
            )

        if obj.predicted_position is not None:
            ax.plot(obj.predicted_position.x, obj.predicted_position.y, marker="+", color=color, markersize=8, alpha=0.5)

        speed_str = f"{obj.velocity.speed:.1f}m/s" if obj.velocity is not None else "..."
        ax.annotate(
            f"#{obj.track_id} {obj.classification.value} ({state})\n{speed_str}",
            (obj.centroid.x, obj.centroid.y),
            fontsize="x-small", color=color, xytext=(4, 4), textcoords="offset points",
        )

    ax.legend(loc="upper right", fontsize="small")
    return ax

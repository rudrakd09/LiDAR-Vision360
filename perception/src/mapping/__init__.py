"""2D occupancy-grid mapping (Phase 8): a persistent FREE/OCCUPIED/UNKNOWN environment
representation built from successive `CartesianScan`s via ray-based log-odds updates -- see
docs/mapping.md.

**This is a 2D LiDAR occupancy map, not a true 3D map** -- the underlying sensor is a 2D 360°
LiDAR (see docs/architecture.md "Sensor limitation"); Unity may later render a 3D digital-twin
*representation* of it, but no 3D reconstruction happens here.

Status: **implemented**. Independent of `simulator` and of every later pipeline stage (collision,
clearance, Unity, cloud, database, STM32) -- see docs/architecture.md.

Typical usage:

    from mapping import OccupancyGridMapper

    mapper = OccupancyGridMapper()  # reads defaults from common.config.Settings
    # MUST be reused scan-to-scan -- the map persists across calls, it is not rebuilt each time.
    for cartesian_scan in cartesian_scans:
        grid = mapper.update(cartesian_scan)

    stats = mapper.get_statistics()
"""

from .grid import OccupancyGridMapper
from .metrics import occupancy_accuracy_metrics, occupancy_confusion_counts
from .raytrace import bresenham_line
from .statistics import compute_map_statistics

__all__ = [
    "OccupancyGridMapper",
    "bresenham_line",
    "compute_map_statistics",
    "occupancy_confusion_counts",
    "occupancy_accuracy_metrics",
]

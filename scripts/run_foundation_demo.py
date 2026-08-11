#!/usr/bin/env python
"""Phase 0 foundation smoke test.

Exercises the full (currently implemented) chain: configuration -> logging -> a
`SimulatedLiDARDataSource` -> one `ScanFrame` of validated `LiDARPoint`s -- proving the
data models, config system, logging, and data-source abstraction all work together before any
filtering/clustering/tracking logic exists.

Run from the repository root, with the perception package installed (see perception/README.md):

    pip install -e "./perception[dev]"
    python scripts/run_foundation_demo.py
"""

from __future__ import annotations

from common.config import get_settings
from common.logging import get_logger, setup_logging
from datasources import SimulatedLiDARDataSource

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    settings = get_settings()

    logger.info("LiDAR-Vision360 foundation demo starting (environment=%s)", settings.environment)
    logger.info(
        "LiDAR config: %d points, range %.2f-%.2fm, %.1fHz",
        settings.lidar_num_points,
        settings.lidar_range_min_m,
        settings.lidar_range_max_m,
        settings.lidar_scan_frequency_hz,
    )

    source = SimulatedLiDARDataSource(settings=settings)
    with source:
        frame = source.read_scan()

    distances = [p.distance for p in frame.points]
    logger.info("Captured scan %s (sequence #%d) from source '%s'", frame.scan_id, frame.sequence_number, frame.source_id)
    logger.info("Point count: %d", frame.point_count)
    logger.info("Distance range: %.3fm - %.3fm", min(distances), max(distances))
    logger.info("First point: angle=%.2f deg, distance=%.3fm", frame.points[0].angle, frame.points[0].distance)
    logger.info("Foundation demo complete: models, config, logging, and data-source abstraction are all working.")


if __name__ == "__main__":
    main()

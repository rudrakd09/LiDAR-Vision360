"""Polar (angle, distance) -> Cartesian (x, y) coordinate transformation.

    PreprocessedScan
         |
    CoordinateTransformer.transform()
         |
    CartesianScan

Coordinate convention (see docs/coordinates.md and docs/data-model.md for the full write-up):
LiDAR/vehicle at the origin, `+x` forward, `+y` left, `angle` in degrees measured
counter-clockwise from `+x`:

    x = distance * cos(radians(angle))
    y = distance * sin(radians(angle))

This is the exact convention already used by `models.lidar.CartesianPoint`'s field docs and by
`simulator.geometry.ray_direction` -- Phase 4 implements it, it does not introduce a new one.

This module has no dependency on `simulator` or on any later pipeline stage, matching
`preprocessing`'s architecture boundary (see docs/architecture.md).
"""

from __future__ import annotations

import numpy as np
from pydantic import ValidationError

from common.logging import get_logger
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint, LiDARPoint
from models.preprocessing import PreprocessedScan

logger = get_logger(__name__)


def polar_to_cartesian(angles_deg: np.ndarray, distances: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized polar -> Cartesian conversion. `angles_deg` and `distances` must be the same
    shape. Trigonometric functions are periodic, so this is correct for any real angle value --
    negative, `>= 360`, or exactly `360` -- with no normalization needed; angle values are never
    modified, only read.
    """
    radians = np.radians(angles_deg)
    x = distances * np.cos(radians)
    y = distances * np.sin(radians)
    return x, y


class CoordinateTransformer:
    """Converts a `PreprocessedScan` into a `CartesianScan`. Stateless -- safe to share/reuse
    across scans and streams."""

    def transform(self, scan: PreprocessedScan) -> CartesianScan:
        points = scan.points

        if not points:
            return CartesianScan(
                scan_id=scan.scan_id,
                sequence_number=scan.sequence_number,
                source_id=scan.source_id,
                timestamp=scan.timestamp,
                points=[],
                quality_statistics=scan.quality_statistics,
            )

        angles = np.array([p.angle for p in points], dtype=np.float64)
        distances = np.array([p.distance for p in points], dtype=np.float64)

        # Preprocessing (Phase 3) already validated every point in `scan.points` -- this is a
        # defense-in-depth check, not a re-validation, per docs/coordinates.md "Invalid
        # measurements": it must not crash or silently emit NaN/inf coordinates if an invalid
        # value somehow reaches here anyway.
        finite = np.isfinite(angles) & np.isfinite(distances)

        xs, ys = polar_to_cartesian(angles, distances)
        finite &= np.isfinite(xs) & np.isfinite(ys)

        cartesian_points: list[CartesianPoint] = []
        skipped = 0
        for i, point in enumerate(points):
            if not finite[i]:
                skipped += 1
                continue
            try:
                cartesian_points.append(_to_cartesian_point(point, float(xs[i]), float(ys[i])))
            except ValidationError:
                # e.g. an angle outside [0, 360) somehow reached this stage -- CartesianPoint
                # inherits LiDARPoint's own range constraint, so construction itself is the
                # final defense-in-depth check.
                skipped += 1

        if skipped:
            logger.warning(
                "CoordinateTransformer: scan %s had %d/%d point(s) that were invalid or "
                "malformed reaching this stage unexpectedly (preprocessing should have removed "
                "these); dropping them rather than emitting bad coordinates.",
                scan.scan_id, skipped, len(points),
            )

        return CartesianScan(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            points=cartesian_points,
            quality_statistics=scan.quality_statistics,
        )


def _to_cartesian_point(point: LiDARPoint, x: float, y: float) -> CartesianPoint:
    return CartesianPoint(
        angle=point.angle,
        distance=point.distance,
        timestamp=point.timestamp,
        valid=point.valid,
        intensity=point.intensity,
        x=round(x, 4),
        y=round(y, 4),
    )


def transform_scan(scan: PreprocessedScan) -> CartesianScan:
    """One-off convenience wrapper around a throwaway `CoordinateTransformer`."""
    return CoordinateTransformer().transform(scan)

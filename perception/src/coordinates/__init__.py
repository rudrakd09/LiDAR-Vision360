"""Polar-to-Cartesian coordinate conversion (x = r*cos(theta), y = r*sin(theta)).

Status: **implemented (Phase 4)**. Turns a `models.preprocessing.PreprocessedScan` into a
`models.coordinates.CartesianScan`. Independent of `simulator` and of every later pipeline stage
(clustering, tracking, collision, Unity, cloud) -- see docs/coordinates.md.

Typical usage:

    from coordinates import CoordinateTransformer

    transformer = CoordinateTransformer()
    cartesian_scan = transformer.transform(preprocessed_scan)
"""

from .transformer import CoordinateTransformer, polar_to_cartesian, transform_scan

__all__ = ["CoordinateTransformer", "polar_to_cartesian", "transform_scan"]

"""Canonical data models shared across the entire perception pipeline.

Every stage of the pipeline (preprocessing, coordinates, clustering, objects, tracking, mapping,
collision, clearance, pipeline) reads and/or writes these models, so a LiDAR data source -
simulated or real hardware - only needs to speak this schema to be usable by the rest of the
system. See docs/data-model.md for the full write-up and coordinate convention.
"""

from .lidar import CartesianPoint, LiDARPoint
from .objects import BoundingBox, DetectedObject, ObjectClassification, Point2D, Velocity2D
from .preprocessing import PreprocessedScan, ScanQualityStatistics
from .scan import ScanFrame

__all__ = [
    "LiDARPoint",
    "CartesianPoint",
    "Point2D",
    "BoundingBox",
    "Velocity2D",
    "ObjectClassification",
    "DetectedObject",
    "ScanFrame",
    "PreprocessedScan",
    "ScanQualityStatistics",
]

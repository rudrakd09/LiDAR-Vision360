"""Canonical data models shared across the entire perception pipeline.

Every stage of the pipeline (preprocessing, coordinates, clustering, objects, tracking, mapping,
collision, clearance, pipeline) reads and/or writes these models, so a LiDAR data source -
simulated or real hardware - only needs to speak this schema to be usable by the rest of the
system. See docs/data-model.md for the full write-up and coordinate convention.
"""

from .classification import ClassifiedScan
from .clearance import ClearanceAssessment, ClearanceDirection, ClearanceState, DirectionalClearance
from .clustering import ClusteredScan, ObstacleCluster
from .collision import CollisionAssessment, CollisionRiskResult, RiskLevel, VehicleState
from .coordinates import CartesianScan
from .lidar import CartesianPoint, LiDARPoint
from .mapping import CellState, MapStatistics, OccupancyGrid, VehiclePose
from .objects import (
    BoundingBox,
    DetectedObject,
    MovementState,
    ObjectClassification,
    Point2D,
    ShapeFeatures,
    TrackingState,
    Velocity2D,
)
from .live_state import (
    LiveState,
    LiveStateEvent,
    PerformanceMetrics,
    SensorChannelStatus,
    TrackedObjectState,
    TrajectoryPoint,
)
from .preprocessing import PreprocessedScan, ScanQualityStatistics
from .radar import RadarReading, RadarTarget
from .scan import ScanFrame
from .tracking import TrackedScan

__all__ = [
    "LiDARPoint",
    "CartesianPoint",
    "Point2D",
    "BoundingBox",
    "Velocity2D",
    "ObjectClassification",
    "TrackingState",
    "MovementState",
    "ShapeFeatures",
    "DetectedObject",
    "ScanFrame",
    "PreprocessedScan",
    "ScanQualityStatistics",
    "CartesianScan",
    "ObstacleCluster",
    "ClusteredScan",
    "ClassifiedScan",
    "TrackedScan",
    "CellState",
    "VehiclePose",
    "MapStatistics",
    "OccupancyGrid",
    "RiskLevel",
    "VehicleState",
    "CollisionRiskResult",
    "CollisionAssessment",
    "ClearanceDirection",
    "ClearanceState",
    "DirectionalClearance",
    "ClearanceAssessment",
    "LiveState",
    "LiveStateEvent",
    "PerformanceMetrics",
    "SensorChannelStatus",
    "TrackedObjectState",
    "TrajectoryPoint",
    "RadarTarget",
    "RadarReading",
]

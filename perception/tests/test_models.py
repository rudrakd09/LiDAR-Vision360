"""Tests for the canonical data models (models.lidar, models.objects, models.scan)."""

import pytest
from pydantic import ValidationError

from models import (
    CartesianPoint,
    DetectedObject,
    LiDARPoint,
    ObjectClassification,
    Point2D,
    ScanFrame,
)


class TestLiDARPoint:
    def test_valid_point_constructs(self):
        p = LiDARPoint(angle=90.0, distance=3.5, timestamp=1_700_000_000.0)
        assert p.angle == 90.0
        assert p.distance == 3.5
        assert p.valid is True
        assert p.intensity is None

    @pytest.mark.parametrize("angle", [-1.0, 360.0, 720.0])
    def test_angle_out_of_range_rejected(self, angle):
        with pytest.raises(ValidationError):
            LiDARPoint(angle=angle, distance=1.0, timestamp=0.0)

    def test_negative_distance_rejected(self):
        with pytest.raises(ValidationError):
            LiDARPoint(angle=0.0, distance=-0.1, timestamp=0.0)

    def test_zero_distance_allowed_as_no_return(self):
        p = LiDARPoint(angle=0.0, distance=0.0, timestamp=0.0)
        assert p.distance == 0.0


class TestCartesianPoint:
    def test_extends_lidar_point_with_xy(self):
        cp = CartesianPoint(angle=0.0, distance=2.0, timestamp=0.0, x=2.0, y=0.0)
        assert cp.x == 2.0
        assert cp.y == 0.0
        # Still a LiDARPoint under the hood.
        assert isinstance(cp, LiDARPoint)


class TestDetectedObject:
    def test_defaults_are_unknown_and_unconfident(self):
        obj = DetectedObject(
            object_id="obj-1",
            centroid=Point2D(x=1.0, y=2.0),
            width=0.5,
            depth=0.5,
            distance=2.24,
            timestamp=0.0,
        )
        assert obj.classification == ObjectClassification.UNKNOWN
        assert obj.confidence == 0.0
        assert obj.velocity is None
        assert obj.track_id is None

    def test_confidence_bounds_enforced(self):
        with pytest.raises(ValidationError):
            DetectedObject(
                object_id="obj-2",
                centroid=Point2D(x=0.0, y=0.0),
                width=0.1,
                depth=0.1,
                distance=0.0,
                confidence=1.5,
                timestamp=0.0,
            )


class TestScanFrame:
    def test_point_count_matches_points(self):
        points = [LiDARPoint(angle=float(i), distance=1.0, timestamp=0.0) for i in range(10)]
        frame = ScanFrame(
            scan_id="scan-1",
            sequence_number=0,
            source_id="unit-test",
            timestamp=0.0,
            points=points,
        )
        assert frame.point_count == 10
        assert frame.cartesian_points is None
        assert frame.objects is None

    def test_empty_frame_is_valid(self):
        frame = ScanFrame(scan_id="scan-2", sequence_number=0, source_id="unit-test", timestamp=0.0)
        assert frame.point_count == 0

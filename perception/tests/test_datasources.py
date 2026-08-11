"""Tests for the LiDARDataSource abstraction and its simulated/serial implementations."""

import pytest

from common.config import Settings
from datasources import LiDARDataSource, SerialLiDARDataSource, SimulatedLiDARDataSource
from models import LiDARPoint, ScanFrame


class TestSimulatedLiDARDataSource:
    def _make_source(self, num_points: int = 36) -> SimulatedLiDARDataSource:
        settings = Settings(_env_file=None, lidar_num_points=num_points)
        return SimulatedLiDARDataSource(settings=settings, seed=42)

    def test_is_a_lidar_data_source(self):
        assert issubclass(SimulatedLiDARDataSource, LiDARDataSource)

    def test_read_scan_requires_connect(self):
        source = self._make_source()
        with pytest.raises(RuntimeError):
            source.read_scan()

    def test_read_scan_returns_expected_point_count(self):
        source = self._make_source(num_points=36)
        source.connect()
        try:
            frame = source.read_scan()
        finally:
            source.disconnect()

        assert isinstance(frame, ScanFrame)
        assert frame.point_count == 36
        assert all(isinstance(p, LiDARPoint) for p in frame.points)
        assert all(0.0 <= p.angle < 360.0 for p in frame.points)
        assert all(p.distance >= 0.0 for p in frame.points)

    def test_sequence_number_increments(self):
        source = self._make_source()
        with source:
            first = source.read_scan()
            second = source.read_scan()
        assert second.sequence_number == first.sequence_number + 1

    def test_context_manager_connects_and_disconnects(self):
        source = self._make_source()
        assert not source.is_connected()
        with source:
            assert source.is_connected()
        assert not source.is_connected()

    def test_stream_yields_scan_frames(self):
        source = self._make_source()
        source.connect()
        seen = []
        for frame in source.stream():
            seen.append(frame)
            if len(seen) == 3:
                source.disconnect()
        assert len(seen) == 3
        assert all(isinstance(f, ScanFrame) for f in seen)


class TestSerialLiDARDataSource:
    def test_is_a_lidar_data_source(self):
        assert issubclass(SerialLiDARDataSource, LiDARDataSource)

    def test_connect_raises_not_implemented(self):
        source = SerialLiDARDataSource(port="COM3")
        with pytest.raises(NotImplementedError):
            source.connect()

    def test_read_scan_raises_not_implemented(self):
        source = SerialLiDARDataSource(port="COM3")
        with pytest.raises(NotImplementedError):
            source.read_scan()

    def test_is_connected_false_by_default(self):
        source = SerialLiDARDataSource(port="COM3")
        assert source.is_connected() is False

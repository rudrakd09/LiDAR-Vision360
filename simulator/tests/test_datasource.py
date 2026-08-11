"""Tests for simulator.datasource.SimulatedLiDARDataSource: end-to-end scan generation."""

import time

import pytest

from datasources import LiDARDataSource
from models import ScanFrame
from simulator.datasource import SimulatedLiDARDataSource
from simulator.environment import Environment
from simulator.geometry import Vec2
from simulator.lidar_model import LiDARModel
from simulator.noise import NoiseConfig, NoiseModel
from simulator.obstacles import PoleObstacle, WallObstacle, velocity_from_heading
from simulator.vehicle import VehicleConfig


def _make_source(obstacles, angular_resolution_deg=10.0, scan_frequency_hz=10.0, noise=None):
    env = Environment(obstacles)
    vehicle = VehicleConfig(width_m=1.8, length_m=4.5)
    lidar_model = LiDARModel(
        range_min_m=0.05, range_max_m=12.0, angular_resolution_deg=angular_resolution_deg, scan_frequency_hz=scan_frequency_hz
    )
    noise_model = NoiseModel(noise or NoiseConfig())
    return SimulatedLiDARDataSource(env, vehicle, lidar_model, noise_model)


class TestBasicBehavior:
    def test_is_a_lidar_data_source(self):
        assert issubclass(SimulatedLiDARDataSource, LiDARDataSource)

    def test_read_scan_before_connect_raises(self):
        source = _make_source([])
        with pytest.raises(RuntimeError):
            source.read_scan()

    def test_point_count_matches_lidar_model(self):
        source = _make_source([], angular_resolution_deg=10.0)  # 36 points
        with source:
            frame = source.read_scan()
        assert isinstance(frame, ScanFrame)
        assert frame.point_count == 36

    def test_all_angles_within_valid_range(self):
        source = _make_source([], angular_resolution_deg=10.0)
        with source:
            frame = source.read_scan()
        assert all(0.0 <= p.angle < 360.0 for p in frame.points)

    def test_sequence_number_increments(self):
        source = _make_source([], angular_resolution_deg=90.0)
        with source:
            first = source.read_scan()
            second = source.read_scan()
        assert second.sequence_number == first.sequence_number + 1


class TestWallDistance:
    def test_exact_distance_with_zero_noise(self):
        wall = WallObstacle(Vec2(5.0, -10.0), Vec2(5.0, 10.0))
        source = _make_source([wall], angular_resolution_deg=10.0)  # angle 0.0 is sampled exactly
        with source:
            frame = source.read_scan()
        point_at_zero = next(p for p in frame.points if p.angle == 0.0)
        assert point_at_zero.valid is True
        assert point_at_zero.distance == pytest.approx(5.0, abs=1e-6)

    def test_no_obstacle_reports_max_range(self):
        source = _make_source([], angular_resolution_deg=10.0)
        with source:
            frame = source.read_scan()
        point_at_zero = next(p for p in frame.points if p.angle == 0.0)
        assert point_at_zero.distance == pytest.approx(12.0, abs=1e-6)


class TestMovingObstacle:
    def test_distance_changes_consistently_with_velocity_between_scans(self):
        # Pole at (0, 6) (i.e. along the angle=90 ray) approaching the vehicle at 2 m/s in -y;
        # scan_frequency_hz=10 -> dt=0.1s -> 0.2m closer per scan.
        pole = PoleObstacle(Vec2(0.0, 6.0), radius=0.1, velocity=velocity_from_heading(2.0, 270.0))
        source = _make_source([pole], angular_resolution_deg=10.0, scan_frequency_hz=10.0)  # angle 90.0 sampled exactly
        with source:
            first = source.read_scan()
            second = source.read_scan()

        d1 = next(p for p in first.points if p.angle == 90.0).distance
        d2 = next(p for p in second.points if p.angle == 90.0).distance
        assert d1 - d2 == pytest.approx(0.2, abs=1e-6)


class TestRealTimePacing:
    def test_real_time_mode_paces_between_scans(self):
        source = _make_source([], angular_resolution_deg=90.0, scan_frequency_hz=20.0)  # dt=0.05s
        source.real_time = True
        with source:
            source.read_scan()
            start = time.time()
            source.read_scan()
            elapsed = time.time() - start
        # Generous lower bound to avoid flakiness while still confirming pacing happened.
        assert elapsed >= 0.02

"""Tests for common.geometry.EgoFootprint -- the geometric ego-vehicle self-return mask."""

import math

from common.config import Settings
from common.geometry import EgoFootprint

DEFAULT = Settings(_env_file=None)


class TestFromSettings:
    def test_built_from_vehicle_dimensions_and_mount_pose(self):
        ef = EgoFootprint.from_settings(DEFAULT)
        assert ef is not None
        assert ef.half_length_m == DEFAULT.vehicle_length_m / 2.0
        assert ef.half_width_m == DEFAULT.vehicle_width_m / 2.0
        # centre-mounted by default -> body centre at the sensor origin
        assert ef.center_x_m == 0.0 and ef.center_y_m == 0.0
        assert ef.margin_m == DEFAULT.ego_footprint_margin_m

    def test_returns_none_when_disabled(self):
        assert EgoFootprint.from_settings(Settings(_env_file=None, ego_footprint_filter_enabled=False)) is None

    def test_returns_none_for_a_degenerate_vehicle(self):
        assert EgoFootprint.from_settings(Settings(_env_file=None, vehicle_width_m=0.0)) is None


class TestContains:
    def test_centre_mounted_body_masks_the_whole_near_field(self):
        ef = EgoFootprint.from_settings(DEFAULT)
        assert ef.contains(0.0, 0.0)          # the origin itself
        assert ef.contains(0.4, 0.0)          # the live-rig track-1 distance, dead ahead
        assert ef.contains(-0.25, 0.42)       # a rear-left self-return arc point
        assert ef.contains(2.2, 0.85)         # just inside the body corner

    def test_centre_mounted_body_does_not_mask_real_obstacles(self):
        ef = EgoFootprint.from_settings(DEFAULT)
        assert not ef.contains(3.0, 0.0)      # 3 m ahead
        assert not ef.contains(0.0, 1.5)      # 1.5 m to the left (outside the 0.9 m half-width)
        assert not ef.contains(-3.0, 0.0)     # 3 m behind
        assert not ef.contains(6.0, 6.0)

    def test_margin_inflates_the_rectangle(self):
        base = EgoFootprint.from_settings(Settings(_env_file=None, ego_footprint_margin_m=0.0))
        wide = EgoFootprint.from_settings(Settings(_env_file=None, ego_footprint_margin_m=0.30))
        x = DEFAULT.vehicle_width_m / 2.0 + 0.15   # 0.15 m beyond the bare body edge, laterally
        assert not base.contains(0.0, x)
        assert wide.contains(0.0, x)

    def test_non_finite_coordinate_is_never_inside(self):
        ef = EgoFootprint.from_settings(DEFAULT)
        assert not ef.contains(math.nan, 0.0)
        assert not ef.contains(0.0, math.inf)

    def test_contains_polar_matches_the_cartesian_transform(self):
        ef = EgoFootprint.from_settings(DEFAULT)
        for angle, dist in [(0.0, 0.4), (135.0, 0.5), (250.0, 0.45), (30.0, 8.0)]:
            rad = math.radians(angle)
            assert ef.contains_polar(angle, dist) == ef.contains(dist * math.cos(rad), dist * math.sin(rad))


class TestBumperMount:
    """`lidar_mount_x_m > 0` places the body BEHIND the sensor: a genuine obstacle just ahead is
    then outside the footprint and must NOT be masked (TASK 6 -- preserve valid close objects)."""

    def test_real_obstacle_ahead_of_a_bumper_sensor_is_kept(self):
        ef = EgoFootprint.from_settings(Settings(_env_file=None, lidar_mount_x_m=2.2))
        assert ef.center_x_m == -2.2
        assert not ef.contains(0.4, 0.0)      # 0.4 m dead ahead of the bumper -> real obstacle
        assert not ef.contains(1.0, 0.0)
        assert ef.contains(-1.0, 0.0)         # 1 m behind the sensor -> into the body -> self return

    def test_orientation_rotates_the_body_rectangle(self):
        # Sensor yawed 90deg relative to the body: the body's long axis now runs along sensor +y.
        ef = EgoFootprint.from_settings(Settings(_env_file=None, lidar_mount_orientation_deg=90.0))
        assert ef.contains(0.0, 2.0)          # 2 m along sensor +y is within the (rotated) 2.25 m half-length
        assert not ef.contains(2.0, 0.0)      # 2 m along sensor +x now exceeds the (rotated) 0.9 m half-width

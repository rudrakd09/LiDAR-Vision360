"""Tests for common.config.Settings / get_settings."""

from common.config import Settings, get_settings


class TestSettings:
    def test_defaults(self):
        settings = Settings(_env_file=None)
        assert settings.lidar_num_points == 360
        assert settings.lidar_range_min_m == 0.05
        assert settings.lidar_range_max_m == 12.0
        # Near-field self-return cutoff: a separate, larger threshold than lidar_range_min_m,
        # at/above the RPLIDAR A3M1's 0.20 m rated minimum range with headroom for the
        # near-floor noise band and a centre-mounted unit's ego/self return.
        assert settings.min_valid_distance_m == 0.3
        assert settings.min_valid_distance_m > settings.lidar_range_min_m
        # Geometric ego-vehicle self-return mask -- on by default, small perimeter margin.
        assert settings.ego_footprint_filter_enabled is True
        assert settings.ego_footprint_margin_m == 0.05
        assert settings.backend_port == 8000

    def test_min_valid_distance_can_be_overridden_via_env(self, monkeypatch):
        monkeypatch.setenv("LIDAR_MIN_VALID_DISTANCE_M", "0.45")
        assert Settings(_env_file=None).min_valid_distance_m == 0.45

    def test_ego_footprint_mask_can_be_disabled_and_tuned_via_env(self, monkeypatch):
        monkeypatch.setenv("LIDAR_EGO_FOOTPRINT_FILTER_ENABLED", "false")
        monkeypatch.setenv("LIDAR_EGO_FOOTPRINT_MARGIN_M", "0.2")
        s = Settings(_env_file=None)
        assert s.ego_footprint_filter_enabled is False
        assert s.ego_footprint_margin_m == 0.2

    def test_env_prefix_override(self, monkeypatch):
        monkeypatch.setenv("LIDAR_LIDAR_NUM_POINTS", "720")
        monkeypatch.setenv("LIDAR_LOG_LEVEL", "DEBUG")
        settings = Settings(_env_file=None)
        assert settings.lidar_num_points == 720
        assert settings.log_level == "DEBUG"

    def test_get_settings_is_cached(self):
        assert get_settings() is get_settings()

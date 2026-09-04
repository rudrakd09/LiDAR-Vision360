"""Tests for 360 degree frame assembly (`datasources.esp32_serial.frame_builder`).

The stream carries no scan delimiter, so every scan boundary is inferred from the angle wrapping.
These tests pin down that inference against the messy input real firmware actually produces:
missing angles, duplicates, jitter, stalls, and a sweep that never reaches 0 at all.
"""

from __future__ import annotations

import pytest

from datasources.esp32_serial.frame_builder import ScanFrameBuilder
from datasources.esp32_serial.parser import RawMeasurement


def measurement(angle: float, distance_m: float = 1.0) -> RawMeasurement:
    return RawMeasurement(angle_deg=angle, distance_m=distance_m)


def build(**overrides) -> ScanFrameBuilder:
    kwargs = {"source_id": "test", "min_points_per_scan": 3}
    kwargs.update(overrides)
    return ScanFrameBuilder(**kwargs)


def feed(builder: ScanFrameBuilder, angles, start_t: float = 1000.0, dt: float = 0.001):
    """Feeds `angles` in order and returns every frame that completed."""
    frames = []
    for i, angle in enumerate(angles):
        frame = builder.add(measurement(angle), received_at=start_t + i * dt)
        if frame is not None:
            frames.append(frame)
    return frames


class TestWrapDetection:
    def test_359_to_0_completes_a_scan(self) -> None:
        builder = build()

        frames = feed(builder, [0, 1, 2, 357, 358, 359, 0, 1])

        assert len(frames) == 1
        # The wrapping measurement (the second 0) starts the NEXT scan, so it is not in this one.
        assert [p.angle for p in frames[0].points] == [0, 1, 2, 357, 358, 359]

    def test_a_scan_that_never_emits_angle_zero_is_still_detected(self) -> None:
        """A naive `angle == 0` test would never fire here; the backward-jump test does."""
        builder = build()

        frames = feed(builder, [3, 90, 180, 357, 2, 91])

        assert len(frames) == 1
        assert [p.angle for p in frames[0].points] == [3, 90, 180, 357]

    def test_missing_angles_do_not_split_a_scan(self) -> None:
        builder = build()

        frames = feed(builder, [0, 45, 120, 300, 359, 10])

        assert len(frames) == 1
        assert frames[0].point_count == 5

    def test_small_backward_jitter_does_not_split_a_scan(self) -> None:
        """Out-of-order arrival within a revolution must not be mistaken for a wrap."""
        builder = build()

        frames = feed(builder, [10, 12, 11.5, 13, 14])

        assert frames == []

    @pytest.mark.parametrize("threshold", [90.0, 180.0, 270.0])
    def test_wrap_threshold_is_configurable(self, threshold: float) -> None:
        builder = build(wrap_threshold_deg=threshold)

        # A backward jump of exactly 100 deg: a wrap only under the 90 deg threshold.
        frames = feed(builder, [0, 50, 150, 50, 60, 70])

        assert len(frames) == (1 if threshold == 90.0 else 0)

    def test_sequence_numbers_increment_across_scans(self) -> None:
        builder = build()

        frames = feed(builder, [0, 1, 2, 359, 0, 1, 2, 359, 0, 1, 2, 359, 0])

        assert [f.sequence_number for f in frames] == [0, 1, 2]

    def test_every_frame_gets_a_unique_scan_id(self) -> None:
        builder = build()

        frames = feed(builder, [0, 1, 2, 359, 0, 1, 2, 359, 0, 1, 2, 359])

        assert len({f.scan_id for f in frames}) == len(frames)


class TestDuplicateAngles:
    def test_last_policy_keeps_the_newest_measurement_for_a_bearing(self) -> None:
        builder = build(duplicate_angle_policy="last")

        builder.add(measurement(10, 1.0))
        builder.add(measurement(10, 2.0))
        builder.add(measurement(20, 3.0))
        frames = feed(builder, [359, 0], start_t=2000.0)

        points = {p.angle: p.distance for p in frames[0].points}
        assert points[10] == pytest.approx(2.0)
        assert builder.stats.duplicate_angles_resolved == 1

    def test_first_policy_keeps_the_original_measurement(self) -> None:
        builder = build(duplicate_angle_policy="first")

        builder.add(measurement(10, 1.0))
        builder.add(measurement(10, 2.0))
        builder.add(measurement(20, 3.0))
        frames = feed(builder, [359, 0], start_t=2000.0)

        points = {p.angle: p.distance for p in frames[0].points}
        assert points[10] == pytest.approx(1.0)

    def test_keep_all_policy_retains_every_measurement(self) -> None:
        builder = build(duplicate_angle_policy="keep_all")

        builder.add(measurement(10, 1.0))
        builder.add(measurement(10, 2.0))
        builder.add(measurement(20, 3.0))
        frames = feed(builder, [359, 0], start_t=2000.0)

        assert frames[0].point_count == 4  # 10, 10, 20, 359

    def test_an_unknown_policy_is_rejected_at_construction(self) -> None:
        with pytest.raises(ValueError, match="duplicate_angle_policy"):
            build(duplicate_angle_policy="whatever")


class TestGuards:
    def test_a_runt_scan_is_discarded_not_published(self) -> None:
        """Two stray points are not a scan; publishing them would drive clustering on noise."""
        builder = build(min_points_per_scan=10)

        frames = feed(builder, [300, 359, 0, 1])

        assert frames == []
        assert builder.stats.scans_discarded_too_few_points == 1

    def test_a_never_wrapping_stream_is_capped_not_unbounded(self) -> None:
        builder = build(max_points_per_scan=5, duplicate_angle_policy="keep_all")

        frames = feed(builder, [10] * 12)

        assert len(frames) >= 2
        assert all(f.point_count <= 5 for f in frames)
        assert builder.stats.scans_force_completed_max_points >= 2

    def test_a_stalled_scan_force_completes_after_the_timeout(self) -> None:
        builder = build(max_scan_duration_s=1.0)
        feed(builder, [0, 1, 2], start_t=500.0)

        assert builder.flush_if_stale(now=500.5) is None  # not stale yet
        frame = builder.flush_if_stale(now=502.0)

        assert frame is not None
        assert frame.point_count == 3
        assert builder.stats.scans_force_completed_timeout == 1

    def test_flush_is_a_no_op_when_nothing_is_pending(self) -> None:
        assert build().flush_if_stale(now=999.0) is None

    def test_reset_discards_a_scan_straddling_a_reconnect(self) -> None:
        builder = build()
        feed(builder, [0, 1, 2, 3])

        builder.reset()
        frames = feed(builder, [5, 6, 7, 359, 0], start_t=3000.0)

        # Only the post-reset revolution is published -- the pre-drop half is gone, not stitched on.
        assert len(frames) == 1
        assert [p.angle for p in frames[0].points] == [5, 6, 7, 359]

    def test_an_out_of_range_wrap_threshold_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="wrap_threshold_deg"):
            build(wrap_threshold_deg=0.0)


class TestFrameContents:
    def test_points_are_sorted_by_bearing_regardless_of_arrival_order(self) -> None:
        builder = build()

        frames = feed(builder, [40, 10, 30, 20, 359, 0])

        angles = [p.angle for p in frames[0].points]
        assert angles == sorted(angles)

    def test_frame_timestamp_is_the_completion_time(self) -> None:
        builder = build()

        frames = feed(builder, [0, 1, 2, 359, 0], start_t=1000.0, dt=0.01)

        # Points at t=1000.00, 1000.01, 1000.02, 1000.03; the 5th starts the next scan.
        assert frames[0].timestamp == pytest.approx(1000.03)

    def test_scan_duration_is_measured_not_assumed(self) -> None:
        builder = build()

        feed(builder, [0, 1, 2, 359, 0], start_t=1000.0, dt=0.01)

        assert builder.stats.last_scan_duration_s == pytest.approx(0.03)

    def test_scan_rate_is_measured_from_real_intervals(self) -> None:
        builder = build()
        # Two complete scans whose completions are 0.1 s apart -> 10 Hz.
        builder.add(measurement(0), received_at=1000.00)
        builder.add(measurement(90), received_at=1000.01)
        builder.add(measurement(270), received_at=1000.02)
        builder.add(measurement(0), received_at=1000.03)   # completes scan 1 at t=1000.02
        builder.add(measurement(90), received_at=1000.10)
        builder.add(measurement(270), received_at=1000.12)
        builder.add(measurement(0), received_at=1000.13)   # completes scan 2 at t=1000.12

        assert builder.stats.measured_scan_rate_hz == pytest.approx(10.0, rel=1e-6)

    def test_scan_rate_is_none_before_a_second_scan(self) -> None:
        """`None`, never a fabricated default -- one scan gives no interval to measure."""
        builder = build()

        feed(builder, [0, 1, 2, 359, 0])

        assert builder.stats.measured_scan_rate_hz is None

    def test_source_id_is_carried_onto_every_frame(self) -> None:
        builder = build(source_id="esp32_serial")

        frames = feed(builder, [0, 1, 2, 359, 0])

        assert frames[0].source_id == "esp32_serial"

    def test_distances_are_carried_through_in_metres(self) -> None:
        builder = build()
        builder.add(measurement(0, 1.2), received_at=1.0)
        builder.add(measurement(90, 0.85), received_at=1.001)
        builder.add(measurement(270, 2.5), received_at=1.002)
        frame = builder.add(measurement(0, 1.0), received_at=1.003)

        assert frame is not None
        assert [p.distance for p in frame.points] == pytest.approx([1.2, 0.85, 2.5])

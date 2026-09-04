"""Assembles the ESP32's per-measurement serial stream into complete 360 degree `ScanFrame`s.

The link delivers one measurement at a time with no scan delimiter, so scan boundaries have to be
*inferred*. The only reliable signal available is the angle wrapping back around::

    ... A:357 A:358 A:359 | A:0 A:1 A:2 ...
                          ^ new scan starts here

WHAT THIS HANDLES, AND WHY
--------------------------
Real firmware output is not a clean 0..359 ramp, so a naive "angle == 0" test is not usable:

* **Missing angles** -- a scan may jump 357 -> 2. Detection is therefore "the angle went
  *backwards* by more than `wrap_threshold_deg`", not "the angle equals zero". Any large
  backward step is a wrap; a scan that never emits angle 0 at all is still detected.
* **Duplicate angles** -- the same bearing arriving twice in one revolution is resolved by
  `duplicate_angle_policy` ("last" by default: the newest measurement for a bearing wins).
* **Irregular sampling / jitter** -- small backward steps (sensor jitter, out-of-order arrival)
  are below the threshold and do not split a scan.
* **Never-wrapping input** -- if the angle stops advancing (sensor stalled, motor stopped) the
  builder force-completes the scan after `max_scan_duration_s` rather than growing without bound,
  and hard-caps at `max_points_per_scan`.
* **Runt scans** -- a boundary that yields fewer than `min_points_per_scan` points is discarded
  rather than published, so a burst of noise right after connect cannot masquerade as a scan and
  drive clustering/risk downstream. Discards are counted, never silently ignored.

TIMESTAMPS
----------
Each `LiDARPoint` carries the wall-clock time its line was read. The `ScanFrame.timestamp` is the
time the scan *completed* (its most recent measurement) -- the freshest instant the frame
describes, and the value the Edge's end-to-end latency measurement is meaningful against. Scan
rate is measured from the real interval between consecutive completions, never assumed from a
configured target.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field

from models.lidar import LiDARPoint
from models.scan import ScanFrame

from .parser import RawMeasurement

_DUPLICATE_ANGLE_POLICIES = ("last", "first", "keep_all")


@dataclass
class FrameBuilderStats:
    """Running frame-assembly counts, surfaced through `ESP32SerialSource.stats`."""

    scans_completed: int = 0
    scans_discarded_too_few_points: int = 0
    scans_force_completed_timeout: int = 0
    scans_force_completed_max_points: int = 0
    duplicate_angles_resolved: int = 0
    points_accepted: int = 0
    last_scan_point_count: int | None = None
    last_scan_duration_s: float | None = None
    measured_scan_rate_hz: float | None = None

    def snapshot(self) -> dict[str, object]:
        return {
            "scans_completed": self.scans_completed,
            "scans_discarded_too_few_points": self.scans_discarded_too_few_points,
            "scans_force_completed_timeout": self.scans_force_completed_timeout,
            "scans_force_completed_max_points": self.scans_force_completed_max_points,
            "duplicate_angles_resolved": self.duplicate_angles_resolved,
            "points_accepted": self.points_accepted,
            "last_scan_point_count": self.last_scan_point_count,
            "last_scan_duration_s": self.last_scan_duration_s,
            "measured_scan_rate_hz": self.measured_scan_rate_hz,
        }


@dataclass
class _PendingScan:
    """The scan currently being accumulated."""

    # Insertion-ordered mapping bearing -> point, so "last"/"first" duplicate resolution is a
    # dict write and the emitted point order still follows arrival order. "keep_all" bypasses it.
    by_angle: dict[float, LiDARPoint] = field(default_factory=dict)
    all_points: list[LiDARPoint] = field(default_factory=list)
    started_at: float | None = None
    last_angle: float | None = None
    last_point_at: float | None = None

    @property
    def count(self) -> int:
        return len(self.all_points)


class ScanFrameBuilder:
    """Feeds on `RawMeasurement`s and emits a `ScanFrame` each time a revolution completes.

    Usage is a single call per measurement::

        frame = builder.add(measurement, received_at=time.time())
        if frame is not None:
            ...  # one complete 360 degree scan

    `flush_if_stale()` must be polled by the caller when no measurement arrives, so a stalled
    sensor still force-completes rather than the pending scan growing forever.
    """

    def __init__(
        self,
        *,
        source_id: str,
        wrap_threshold_deg: float = 180.0,
        min_points_per_scan: int = 10,
        max_points_per_scan: int = 5000,
        max_scan_duration_s: float = 2.0,
        duplicate_angle_policy: str = "last",
    ) -> None:
        if duplicate_angle_policy not in _DUPLICATE_ANGLE_POLICIES:
            raise ValueError(
                f"duplicate_angle_policy must be one of {_DUPLICATE_ANGLE_POLICIES}, "
                f"got {duplicate_angle_policy!r}."
            )
        if not (0.0 < wrap_threshold_deg < 360.0):
            raise ValueError(f"wrap_threshold_deg must be in (0, 360), got {wrap_threshold_deg!r}.")

        self.source_id = source_id
        self._wrap_threshold_deg = wrap_threshold_deg
        self._min_points = min_points_per_scan
        self._max_points = max_points_per_scan
        self._max_scan_duration_s = max_scan_duration_s
        self._duplicate_policy = duplicate_angle_policy

        self._pending = _PendingScan()
        self._sequence_number = 0
        self._previous_completion_at: float | None = None
        self.stats = FrameBuilderStats()

    # -- accumulation ---------------------------------------------------------------------

    def add(self, measurement: RawMeasurement, received_at: float | None = None) -> ScanFrame | None:
        """Adds one measurement; returns a `ScanFrame` if it completed the current revolution.

        The wrap is detected *before* the measurement is stored, so the wrapping measurement
        becomes the first point of the NEXT scan rather than the last point of the finished one.
        """
        now = received_at if received_at is not None else time.time()
        pending = self._pending

        completed: ScanFrame | None = None
        if pending.last_angle is not None and self._is_wrap(pending.last_angle, measurement.angle_deg):
            completed = self._complete_scan()

        self._store(measurement, now)

        # A scan that never wraps must still be bounded. Checked after storing so the cap is on
        # the real accumulated size.
        if completed is None and self._pending.count >= self._max_points:
            self.stats.scans_force_completed_max_points += 1
            completed = self._complete_scan()

        return completed

    def flush_if_stale(self, now: float | None = None) -> ScanFrame | None:
        """Force-completes a pending scan that has been open longer than `max_scan_duration_s`.

        Returns `None` when there is nothing pending or it is not yet stale. Call this from the
        caller's idle path (no measurement available) so a stalled motor is noticed.
        """
        pending = self._pending
        if pending.started_at is None or pending.count == 0:
            return None

        now = now if now is not None else time.time()
        if (now - pending.started_at) < self._max_scan_duration_s:
            return None

        self.stats.scans_force_completed_timeout += 1
        return self._complete_scan()

    def reset(self) -> None:
        """Discards any partially-accumulated scan.

        Called on reconnect: measurements from before a link drop and after it belong to different
        revolutions, and stitching them together would emit one physically meaningless frame.
        """
        self._pending = _PendingScan()

    # -- internals ------------------------------------------------------------------------

    def _is_wrap(self, last_angle: float, angle: float) -> bool:
        """True when the bearing jumped backwards far enough to mean a new revolution.

        359 -> 0 is a backwards step of 359 deg; ordinary jitter (12.4 -> 12.1) is a fraction of a
        degree. Anything above `wrap_threshold_deg` (default 180, i.e. half a revolution) is a
        wrap.
        """
        return (last_angle - angle) > self._wrap_threshold_deg

    def _store(self, measurement: RawMeasurement, now: float) -> None:
        pending = self._pending
        point = LiDARPoint(
            angle=measurement.angle_deg,
            distance=measurement.distance_m,
            timestamp=now,
            valid=True,
        )

        if pending.started_at is None:
            pending.started_at = now

        if self._duplicate_policy == "keep_all":
            pending.all_points.append(point)
        else:
            existing = pending.by_angle.get(measurement.angle_deg)
            if existing is None:
                pending.by_angle[measurement.angle_deg] = point
            else:
                self.stats.duplicate_angles_resolved += 1
                if self._duplicate_policy == "last":
                    pending.by_angle[measurement.angle_deg] = point
                # "first": keep what is already stored.
            pending.all_points = list(pending.by_angle.values())

        pending.last_angle = measurement.angle_deg
        pending.last_point_at = now
        self.stats.points_accepted += 1

    def _complete_scan(self) -> ScanFrame | None:
        """Closes the pending scan and starts a fresh one. Returns `None` for a runt scan."""
        pending = self._pending
        self._pending = _PendingScan()

        points = pending.all_points
        if len(points) < self._min_points:
            self.stats.scans_discarded_too_few_points += 1
            return None

        completed_at = pending.last_point_at if pending.last_point_at is not None else time.time()
        duration_s = (
            completed_at - pending.started_at if pending.started_at is not None else None
        )

        if self._previous_completion_at is not None:
            interval_s = completed_at - self._previous_completion_at
            # Guard against a zero/negative interval (two completions inside one clock tick) --
            # reporting `inf` Hz downstream would be worse than reporting nothing.
            self.stats.measured_scan_rate_hz = (1.0 / interval_s) if interval_s > 0 else None
        self._previous_completion_at = completed_at

        frame = ScanFrame(
            scan_id=str(uuid.uuid4()),
            sequence_number=self._sequence_number,
            source_id=self.source_id,
            timestamp=completed_at,
            # Sorted by bearing so downstream stages that assume angular ordering (the
            # preprocessor sorts defensively, but neighbour-based filters read better on ordered
            # input) see a clean sweep regardless of arrival order.
            points=sorted(points, key=lambda p: p.angle),
        )
        self._sequence_number += 1
        self.stats.scans_completed += 1
        self.stats.last_scan_point_count = len(points)
        self.stats.last_scan_duration_s = duration_s
        return frame


__all__ = ["ScanFrameBuilder", "FrameBuilderStats"]

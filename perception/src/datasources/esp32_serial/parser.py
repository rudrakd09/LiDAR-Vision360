"""ASCII measurement parser for the ESP32 USB-serial link.

WIRE FORMAT (fixed, supplied by the hardware -- not guessed here)
----------------------------------------------------------------
The ESP32 emits exactly one measurement per line::

    A:45 , D:1200

* ``A`` -- angle in **degrees**.
* ``D`` -- distance in **millimetres**.

Whitespace around the separators varies between firmware builds and is not significant, so all
of these are accepted as the same measurement::

    A:45,D:1200        A:45 ,D:1200        A:45, D:1200        A:45 , D:1200

This module is the ONLY place in the project that encodes that format. Everything downstream
consumes `models.lidar.LiDARPoint` (degrees + **metres**), so a future firmware change is a
one-file change here.

VALIDATION
----------
A measurement is accepted only when it is structurally well-formed *and* physically meaningful:

* ``0 <= angle <= 360`` -- 360 is accepted and normalised to 0.0, because `LiDARPoint.angle` is
  defined on the half-open interval ``[0, 360)`` (see `models.lidar`) and 360 deg is the same
  bearing as 0 deg. Anything outside that closed range is rejected.
* ``distance > 0`` -- a zero or negative distance is not a measurement. (Note: 0 mm is rejected
  *here* as malformed rather than being turned into a `valid=False` dropout point, because this
  firmware does not use 0 as a documented no-return sentinel; inventing that meaning would be
  guessing hardware behaviour.)

Range filtering (`lidar_range_min_m` / `lidar_range_max_m`) is deliberately NOT done here -- that
is `preprocessing`'s job and is separately configurable. The parser's only job is
"is this line a real measurement, yes or no".

Rejected lines are counted by reason (`ParserStats`) and dropped. Nothing is ever substituted,
interpolated, or fabricated for a bad line -- per this project's "never silently create fake
values" rule.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

# One measurement per line. Case-insensitive keys, optional sign, integer or decimal magnitude,
# and arbitrary (including zero) whitespace around ':' and ','. Anchored at both ends so a line
# carrying trailing junk is rejected rather than half-read.
#
# Signs are accepted by the *pattern* (rather than excluded syntactically) so that a genuinely
# negative value reaches the range checks below and is counted under a precise rejection reason
# ("distance_not_positive" / "angle_out_of_range") instead of the useless "malformed".
_MEASUREMENT_RE = re.compile(
    r"""^\s*
        A \s* : \s* (?P<angle>    [+-]? (?: \d+\.?\d* | \.\d+ ) )
        \s* , \s*
        D \s* : \s* (?P<distance> [+-]? (?: \d+\.?\d* | \.\d+ ) )
        \s*$
    """,
    re.IGNORECASE | re.VERBOSE,
)

_MM_PER_M = 1000.0


@dataclass(frozen=True)
class RawMeasurement:
    """One validated `A:<deg> , D:<mm>` line, in the project's canonical units.

    `angle_deg` is already normalised into ``[0, 360)`` and `distance_m` is already converted
    from the wire's millimetres to metres, so callers never repeat either conversion.
    """

    angle_deg: float
    distance_m: float

    @property
    def distance_mm(self) -> float:
        """The original on-wire value, for logging/diagnostics."""
        return self.distance_m * _MM_PER_M


@dataclass
class ParserStats:
    """Running counts, so a live run can report *why* lines are being dropped.

    Exposed through `ESP32SerialSource.stats` and logged periodically -- a silently-high
    `malformed` count is the single most useful signal that the baud rate is wrong or the firmware
    format has changed.
    """

    lines_seen: int = 0
    accepted: int = 0
    rejected: int = 0
    rejections_by_reason: dict[str, int] = field(default_factory=dict)

    def _reject(self, reason: str) -> None:
        self.rejected += 1
        self.rejections_by_reason[reason] = self.rejections_by_reason.get(reason, 0) + 1

    @property
    def accept_ratio(self) -> float | None:
        """Fraction of non-blank lines accepted, or `None` before any line has been seen."""
        return (self.accepted / self.lines_seen) if self.lines_seen else None

    def snapshot(self) -> dict[str, object]:
        return {
            "lines_seen": self.lines_seen,
            "accepted": self.accepted,
            "rejected": self.rejected,
            "accept_ratio": self.accept_ratio,
            "rejections_by_reason": dict(self.rejections_by_reason),
        }


class MeasurementParser:
    """Turns raw serial lines into `RawMeasurement`s, counting and discarding the rest.

    Stateless apart from `stats`; safe to reuse for the lifetime of a run.
    """

    def __init__(self) -> None:
        self.stats = ParserStats()

    def parse_line(self, line: str) -> RawMeasurement | None:
        """Returns the measurement this line carries, or `None` if it is not a valid one.

        Blank/whitespace-only lines are ignored entirely (not counted as rejections) -- they are
        normal on a serial link that terminates with ``\r\n``, and counting them would drown the
        genuinely-useful `malformed` signal.
        """
        if not line or not line.strip():
            return None

        self.stats.lines_seen += 1

        match = _MEASUREMENT_RE.match(line)
        if match is None:
            # Boot banners, firmware log lines, and bytes garbled by a wrong baud rate all land
            # here. Expected in small numbers right after reset; a sustained high rate is a fault.
            self.stats._reject("malformed")
            return None

        try:
            angle = float(match.group("angle"))
            distance_mm = float(match.group("distance"))
        except ValueError:  # pragma: no cover -- the regex already guarantees float-parseable text
            self.stats._reject("malformed")
            return None

        if not (0.0 <= angle <= 360.0):
            self.stats._reject("angle_out_of_range")
            return None

        if distance_mm <= 0.0:
            self.stats._reject("distance_not_positive")
            return None

        self.stats.accepted += 1
        # 360 deg == 0 deg; `LiDARPoint.angle` is defined on [0, 360). `% 360.0` also collapses
        # the exact-360 case without a special branch.
        return RawMeasurement(angle_deg=angle % 360.0, distance_m=distance_mm / _MM_PER_M)


__all__ = ["MeasurementParser", "ParserStats", "RawMeasurement"]

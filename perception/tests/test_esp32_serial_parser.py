"""Tests for the ESP32 ASCII measurement parser (`datasources.esp32_serial.parser`).

Covers exactly the contract the hardware fixes: `A:<degrees> , D:<millimetres>`, one measurement
per line, whitespace-insensitive -- plus the rejection rules, which matter just as much: a bad
line must be dropped and counted, never turned into a fabricated measurement.
"""

from __future__ import annotations

import pytest

from datasources.esp32_serial.parser import MeasurementParser


@pytest.fixture()
def parser() -> MeasurementParser:
    return MeasurementParser()


class TestCanonicalFormat:
    def test_parses_the_documented_example(self, parser: MeasurementParser) -> None:
        measurement = parser.parse_line("A:45 , D:1200")

        assert measurement is not None
        assert measurement.angle_deg == pytest.approx(45.0)
        # Millimetres on the wire -> metres internally, converted once, here.
        assert measurement.distance_m == pytest.approx(1.2)
        assert measurement.distance_mm == pytest.approx(1200.0)

    @pytest.mark.parametrize(
        "line",
        [
            "A:45,D:1200",
            "A:45 ,D:1200",
            "A:45, D:1200",
            "A:45 , D:1200",
            "  A:45   ,   D:1200  ",
            "A : 45 , D : 1200",
            "a:45,d:1200",  # firmware may not be consistent about case
            "A:45,D:1200\r",  # a stray CR the line splitter did not strip
        ],
    )
    def test_whitespace_and_case_variants_are_the_same_measurement(
        self, parser: MeasurementParser, line: str
    ) -> None:
        measurement = parser.parse_line(line)

        assert measurement is not None, f"{line!r} should parse"
        assert measurement.angle_deg == pytest.approx(45.0)
        assert measurement.distance_m == pytest.approx(1.2)

    @pytest.mark.parametrize(
        ("line", "angle", "distance_m"),
        [
            ("A:0 , D:850", 0.0, 0.850),
            ("A:1 , D:842", 1.0, 0.842),
            ("A:180 , D:2500", 180.0, 2.5),
            ("A:359 , D:1100", 359.0, 1.1),
            ("A:45.5 , D:1200.5", 45.5, 1.2005),  # decimals, should the firmware ever send them
        ],
    )
    def test_representative_real_measurements(
        self, parser: MeasurementParser, line: str, angle: float, distance_m: float
    ) -> None:
        measurement = parser.parse_line(line)

        assert measurement is not None
        assert measurement.angle_deg == pytest.approx(angle)
        assert measurement.distance_m == pytest.approx(distance_m)

    def test_angle_360_normalises_to_0(self, parser: MeasurementParser) -> None:
        """`LiDARPoint.angle` is defined on [0, 360), and 360 deg is the same bearing as 0 deg."""
        measurement = parser.parse_line("A:360 , D:1000")

        assert measurement is not None
        assert measurement.angle_deg == pytest.approx(0.0)


class TestRejection:
    @pytest.mark.parametrize(
        ("line", "reason"),
        [
            ("A:abc,D:1200", "malformed"),
            ("A:45,D:xyz", "malformed"),
            ("garbage", "malformed"),
            ("A:45", "malformed"),  # missing the distance field entirely
            ("D:1200,A:45", "malformed"),  # fields in the wrong order
            ("A:45,D:1200,X:9", "malformed"),  # trailing junk -- anchored regex rejects it
            ("A:45;D:1200", "malformed"),  # wrong separator
            ("ESP32 booting...", "malformed"),  # a real firmware banner line
            ("A:45,D:-10", "distance_not_positive"),
            ("A:45,D:0", "distance_not_positive"),
            ("A:400,D:1200", "angle_out_of_range"),
            ("A:-5,D:1200", "angle_out_of_range"),
        ],
    )
    def test_invalid_lines_are_rejected_and_counted_by_reason(
        self, parser: MeasurementParser, line: str, reason: str
    ) -> None:
        assert parser.parse_line(line) is None
        assert parser.stats.rejected == 1
        assert parser.stats.rejections_by_reason == {reason: 1}
        assert parser.stats.accepted == 0

    @pytest.mark.parametrize("line", ["", "   ", "\r", "\n", "\t"])
    def test_blank_lines_are_ignored_entirely(self, parser: MeasurementParser, line: str) -> None:
        """Not counted as rejections -- they are normal CRLF artefacts, and counting them would
        drown the genuinely-useful `malformed` signal."""
        assert parser.parse_line(line) is None
        assert parser.stats.lines_seen == 0
        assert parser.stats.rejected == 0

    def test_a_bad_line_never_yields_a_substituted_value(self, parser: MeasurementParser) -> None:
        """The project's "never silently create fake values" rule, asserted directly."""
        for line in ("A:abc,D:1200", "A:45,D:-10", "A:400,D:1200", "nonsense"):
            assert parser.parse_line(line) is None
        assert parser.stats.accepted == 0


class TestStats:
    def test_counts_and_accept_ratio_track_a_mixed_stream(self, parser: MeasurementParser) -> None:
        lines = ["A:0,D:100", "junk", "A:1,D:200", "A:999,D:5", "A:2,D:300"]
        for line in lines:
            parser.parse_line(line)

        assert parser.stats.lines_seen == 5
        assert parser.stats.accepted == 3
        assert parser.stats.rejected == 2
        assert parser.stats.accept_ratio == pytest.approx(3 / 5)
        assert parser.stats.rejections_by_reason == {"malformed": 1, "angle_out_of_range": 1}

    def test_accept_ratio_is_none_before_any_line(self, parser: MeasurementParser) -> None:
        """`None`, not 0.0 -- "nothing measured yet" and "everything rejected" are different."""
        assert parser.stats.accept_ratio is None

    def test_snapshot_is_a_plain_serialisable_dict(self, parser: MeasurementParser) -> None:
        parser.parse_line("A:45,D:1200")
        snapshot = parser.stats.snapshot()

        assert snapshot["accepted"] == 1
        assert isinstance(snapshot["rejections_by_reason"], dict)

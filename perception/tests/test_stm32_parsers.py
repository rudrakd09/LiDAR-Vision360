"""Unit tests for `datasources.stm32.parsers`. PARSER/UNIT TESTS ONLY -- not hardware validation.

`ExampleLiDARParser`/`ExampleRadarParser` below are TEST FIXTURES ONLY, defined in this test file
to exercise the `LiDARMessageParser`/`RadarMessageParser` interface mechanics end-to-end against a
synthetic, hypothetical payload layout this test file made up for its own purposes. They are NOT
shipped in `datasources/stm32/parsers.py` and must never be mistaken for a real implementation of
the actual (still unknown) STM32/A3M1/R121 wire format.
"""

import struct

import pytest

from datasources.stm32.errors import STM32ConfigurationError
from datasources.stm32.parsers import (
    UnconfiguredLiDARParser,
    UnconfiguredRadarParser,
    decode_header_field,
    LiDARMessageParser,
    RadarMessageParser,
)
from models.lidar import LiDARPoint
from models.radar import RadarReading


class TestDecodeHeaderField:
    def test_little_endian_one_byte(self):
        assert decode_header_field(b"\x2a\x00\x00", offset=0, width=1, byte_order="little") == 0x2A

    def test_little_endian_two_bytes(self):
        assert decode_header_field(b"\x34\x12", offset=0, width=2, byte_order="little") == 0x1234

    def test_big_endian_two_bytes(self):
        assert decode_header_field(b"\x12\x34", offset=0, width=2, byte_order="big") == 0x1234

    def test_offset_into_frame(self):
        assert decode_header_field(b"\xff\xff\x07\x00", offset=2, width=2, byte_order="little") == 0x0007

    def test_field_out_of_bounds_raises(self):
        with pytest.raises(ValueError):
            decode_header_field(b"\x01\x02", offset=1, width=4, byte_order="little")


class TestUnconfiguredParsersAlwaysRaise:
    def test_lidar_parser_raises_configuration_error(self):
        parser = UnconfiguredLiDARParser()
        with pytest.raises(STM32ConfigurationError):
            parser.parse(b"\x00" * 10, timestamp=0.0, sequence_number=0)

    def test_radar_parser_raises_configuration_error(self):
        parser = UnconfiguredRadarParser()
        with pytest.raises(STM32ConfigurationError):
            parser.parse(b"\x00" * 10, timestamp=0.0, sequence_number=0, source_id="stm32_hardware")

    def test_lidar_parser_is_the_interface(self):
        assert issubclass(UnconfiguredLiDARParser, LiDARMessageParser)

    def test_radar_parser_is_the_interface(self):
        assert issubclass(UnconfiguredRadarParser, RadarMessageParser)


# --- Interface-mechanics tests against a hypothetical example layout (test fixture only) -------


class ExampleLiDARParser(LiDARMessageParser):
    """TEST FIXTURE ONLY: `payload = <uint16 angle_centideg><uint16 distance_mm>` repeated, all
    little-endian (centidegrees, not millidegrees, so a full 0-360 deg range fits a uint16).
    Invented purely to prove `LiDARMessageParser`'s contract works end-to-end; not a claim about
    the real A3M1/STM32 payload shape."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int) -> list[LiDARPoint]:
        points = []
        for i in range(0, len(frame), 4):
            angle_centideg, distance_mm = struct.unpack_from("<HH", frame, i)
            points.append(LiDARPoint(angle=angle_centideg / 100.0, distance=distance_mm / 1000.0, timestamp=timestamp, valid=True))
        return points


class ExampleRadarParser(RadarMessageParser):
    """TEST FIXTURE ONLY: `payload = <uint16 range_cm><int16 velocity_cms>` -- invented for the
    same reason as `ExampleLiDARParser` above."""

    def parse(self, frame: bytes, *, timestamp: float, sequence_number: int, source_id: str) -> RadarReading:
        from models.radar import RadarTarget

        range_cm, velocity_cms = struct.unpack_from("<Hh", frame, 0)
        target = RadarTarget(range_m=range_cm / 100.0, velocity_mps=velocity_cms / 100.0)
        return RadarReading(source_id=source_id, sequence_number=sequence_number, timestamp=timestamp, targets=[target])


class TestExampleLiDARParserFixture:
    def test_parses_two_points(self):
        # angle=90.00 deg (9000 centideg), distance=5.000 m (5000 mm)
        # angle=180.00 deg, distance=2.500 m
        payload = struct.pack("<HHHH", 9000, 5000, 18000, 2500)
        points = ExampleLiDARParser().parse(payload, timestamp=123.0, sequence_number=7)
        assert len(points) == 2
        assert points[0].angle == pytest.approx(90.0)
        assert points[0].distance == pytest.approx(5.0)
        assert points[1].angle == pytest.approx(180.0)
        assert points[1].distance == pytest.approx(2.5)
        assert all(p.timestamp == 123.0 for p in points)


class TestExampleRadarParserFixture:
    def test_parses_one_target(self):
        payload = struct.pack("<Hh", 1200, -150)  # 12.00 m range, -1.50 m/s velocity
        reading = ExampleRadarParser().parse(payload, timestamp=42.0, sequence_number=3, source_id="stm32_hardware")
        assert reading.source_id == "stm32_hardware"
        assert reading.sequence_number == 3
        assert len(reading.targets) == 1
        assert reading.targets[0].range_m == pytest.approx(12.0)
        assert reading.targets[0].velocity_mps == pytest.approx(-1.5)

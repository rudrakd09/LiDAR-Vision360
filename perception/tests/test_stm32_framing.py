"""Unit tests for `datasources.stm32.framing.FrameCodec` against synthetic byte streams built
from EXAMPLE/hypothetical configurations. PARSER/UNIT TESTS ONLY -- not hardware validation. The
marker bytes and frame lengths here are illustrative test fixtures chosen for this test file, not
real values from the actual STM32 firmware (which is still unknown).
"""

import pytest

from datasources.stm32.errors import STM32ConfigurationError
from datasources.stm32.framing import FrameCodec


class TestConstructionValidation:
    def test_requires_a_framing_mode(self):
        with pytest.raises(STM32ConfigurationError):
            FrameCodec()

    def test_rejects_both_modes_at_once(self):
        with pytest.raises(STM32ConfigurationError):
            FrameCodec(start_marker=b"\xaa\x55", frame_length=10)

    def test_rejects_non_positive_frame_length(self):
        with pytest.raises(STM32ConfigurationError):
            FrameCodec(frame_length=0)


class TestFixedLengthFraming:
    def _codec(self) -> FrameCodec:
        return FrameCodec(frame_length=4)

    def test_extracts_nothing_until_a_full_frame_arrives(self):
        codec = self._codec()
        codec.feed(b"\x01\x02")
        assert codec.extract_frames() == []

    def test_extracts_one_complete_frame(self):
        codec = self._codec()
        codec.feed(b"\x01\x02\x03\x04")
        assert codec.extract_frames() == [b"\x01\x02\x03\x04"]

    def test_extracts_multiple_frames_fed_at_once(self):
        codec = self._codec()
        codec.feed(b"\x01\x02\x03\x04" + b"\x05\x06\x07\x08")
        assert codec.extract_frames() == [b"\x01\x02\x03\x04", b"\x05\x06\x07\x08"]

    def test_retains_partial_trailing_frame_across_calls(self):
        codec = self._codec()
        codec.feed(b"\x01\x02\x03\x04\x05\x06")  # one full frame + 2 leftover bytes
        assert codec.extract_frames() == [b"\x01\x02\x03\x04"]
        assert codec.buffered_byte_count() == 2
        codec.feed(b"\x07\x08")
        assert codec.extract_frames() == [b"\x05\x06\x07\x08"]


class TestDelimitedFramingWithEndMarker:
    def _codec(self) -> FrameCodec:
        return FrameCodec(start_marker=b"\xaa\x55", end_marker=b"\x0d\x0a")

    def test_extracts_one_frame(self):
        codec = self._codec()
        codec.feed(b"\xaa\x55\x01\x02\x03\x0d\x0a")
        assert codec.extract_frames() == [b"\xaa\x55\x01\x02\x03\x0d\x0a"]

    def test_discards_noise_before_first_start_marker(self):
        codec = self._codec()
        codec.feed(b"\xff\xff\xaa\x55\x01\x0d\x0a")
        assert codec.extract_frames() == [b"\xaa\x55\x01\x0d\x0a"]

    def test_incomplete_frame_stays_buffered(self):
        codec = self._codec()
        codec.feed(b"\xaa\x55\x01\x02")
        assert codec.extract_frames() == []
        codec.feed(b"\x03\x0d\x0a")
        assert codec.extract_frames() == [b"\xaa\x55\x01\x02\x03\x0d\x0a"]

    def test_extracts_consecutive_frames(self):
        codec = self._codec()
        codec.feed(b"\xaa\x55\x01\x0d\x0a" + b"\xaa\x55\x02\x0d\x0a")
        assert codec.extract_frames() == [b"\xaa\x55\x01\x0d\x0a", b"\xaa\x55\x02\x0d\x0a"]


class TestDelimitedFramingStartMarkerOnly:
    def _codec(self) -> FrameCodec:
        return FrameCodec(start_marker=b"\xaa\x55")

    def test_frame_runs_until_next_start_marker(self):
        codec = self._codec()
        codec.feed(b"\xaa\x55\x01\x02\x03" + b"\xaa\x55\x04\x05")
        # First frame is only known complete once the second start marker appears.
        assert codec.extract_frames() == [b"\xaa\x55\x01\x02\x03"]

    def test_last_frame_stays_buffered_until_a_following_marker_arrives(self):
        codec = self._codec()
        codec.feed(b"\xaa\x55\x01\x02\x03")
        assert codec.extract_frames() == []
        codec.feed(b"\xaa\x55\x04")
        assert codec.extract_frames() == [b"\xaa\x55\x01\x02\x03"]

"""Unit tests for `datasources.stm32.crc` -- standard checksum algorithms against hand-computed
synthetic byte sequences. PARSER/UNIT TESTS ONLY -- not hardware validation; no real STM32
data is involved anywhere in this file.
"""

import pytest

from datasources.stm32.crc import ChecksumAlgorithm, checksum_length_bytes, compute_checksum, verify_checksum


class TestChecksumLength:
    def test_none_is_zero_bytes(self):
        assert checksum_length_bytes("none") == 0

    def test_xor8_and_sum8_and_crc8_are_one_byte(self):
        assert checksum_length_bytes("xor8") == 1
        assert checksum_length_bytes("sum8") == 1
        assert checksum_length_bytes("crc8") == 1

    def test_crc16_ccitt_is_two_bytes(self):
        assert checksum_length_bytes("crc16_ccitt") == 2

    def test_unknown_algorithm_raises(self):
        with pytest.raises(ValueError):
            checksum_length_bytes("not-a-real-algorithm")


class TestNoneAlgorithm:
    def test_always_verifies(self):
        assert verify_checksum("none", b"\x01\x02\x03", expected=0xFF) is True


class TestXor8:
    def test_computed_value(self):
        # 0x01 ^ 0x02 ^ 0x03 == 0x00
        assert compute_checksum(ChecksumAlgorithm.XOR8, bytes([0x01, 0x02, 0x03])) == 0x00
        # 0xDE ^ 0xAD == 0x73
        assert compute_checksum("xor8", bytes([0xDE, 0xAD])) == 0x73

    def test_verify_roundtrip(self):
        data = bytes([0x10, 0x20, 0x30, 0x40])
        checksum = compute_checksum("xor8", data)
        assert verify_checksum("xor8", data, checksum) is True
        assert verify_checksum("xor8", data, checksum ^ 0x01) is False


class TestSum8:
    def test_computed_value(self):
        assert compute_checksum("sum8", bytes([0x01, 0x02, 0x03])) == 0x06
        # wraps modulo 256
        assert compute_checksum("sum8", bytes([0xFF, 0x02])) == 0x01

    def test_verify_roundtrip(self):
        data = bytes(range(20))
        checksum = compute_checksum("sum8", data)
        assert verify_checksum("sum8", data, checksum) is True
        assert verify_checksum("sum8", data, (checksum + 1) & 0xFF) is False


class TestCrc8:
    def test_empty_input_is_zero(self):
        assert compute_checksum("crc8", b"") == 0x00

    def test_known_vector(self):
        # CRC-8/SMBUS (poly 0x07, init 0x00) of ASCII "123456789" is a widely published test
        # vector: 0xF4.
        assert compute_checksum("crc8", b"123456789") == 0xF4

    def test_verify_roundtrip(self):
        data = bytes([0xAA, 0x55, 0x01, 0x02, 0x03])
        checksum = compute_checksum("crc8", data)
        assert verify_checksum("crc8", data, checksum) is True
        assert verify_checksum("crc8", data, checksum ^ 0xFF) is False


class TestCrc16Ccitt:
    def test_known_vector(self):
        # CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of ASCII "123456789" is a widely
        # published test vector: 0x29B1.
        assert compute_checksum("crc16_ccitt", b"123456789") == 0x29B1

    def test_verify_roundtrip(self):
        data = bytes([0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
        checksum = compute_checksum("crc16_ccitt", data)
        assert verify_checksum("crc16_ccitt", data, checksum) is True
        assert verify_checksum("crc16_ccitt", data, checksum ^ 0xFFFF) is False

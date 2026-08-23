"""CRC/checksum algorithms for validating STM32 frames.

Every algorithm here is a standard, well-defined one -- which of them the real firmware actually
uses is what's unknown (`Settings.stm32_crc_algorithm`, unset by default), never the algorithms
themselves. This module does not invent a project-specific checksum; it only implements common,
documented ones and lets configuration pick.
"""

from __future__ import annotations

from enum import Enum


class ChecksumAlgorithm(str, Enum):
    NONE = "none"
    XOR8 = "xor8"
    SUM8 = "sum8"
    CRC8 = "crc8"  # CRC-8/SMBUS: poly 0x07, init 0x00, no reflect, no xorout
    CRC16_CCITT = "crc16_ccitt"  # CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout


_LENGTHS: dict[ChecksumAlgorithm, int] = {
    ChecksumAlgorithm.NONE: 0,
    ChecksumAlgorithm.XOR8: 1,
    ChecksumAlgorithm.SUM8: 1,
    ChecksumAlgorithm.CRC8: 1,
    ChecksumAlgorithm.CRC16_CCITT: 2,
}


def _resolve(algorithm: str | ChecksumAlgorithm) -> ChecksumAlgorithm:
    if isinstance(algorithm, ChecksumAlgorithm):
        return algorithm
    try:
        return ChecksumAlgorithm(algorithm)
    except ValueError as e:
        valid = ", ".join(a.value for a in ChecksumAlgorithm)
        raise ValueError(f"Unknown checksum algorithm {algorithm!r}. Expected one of: {valid}.") from e


def checksum_length_bytes(algorithm: str | ChecksumAlgorithm) -> int:
    """How many trailing bytes `algorithm` produces (0 for `"none"`)."""
    return _LENGTHS[_resolve(algorithm)]


def compute_checksum(algorithm: str | ChecksumAlgorithm, data: bytes) -> int:
    """Computes `algorithm`'s checksum over `data`. `data` is exactly the bytes the checksum
    protects (the caller decides what that span is -- typically the frame minus its own trailing
    checksum bytes; see `datasources.stm32.framing`)."""
    algo = _resolve(algorithm)

    if algo is ChecksumAlgorithm.NONE:
        return 0

    if algo is ChecksumAlgorithm.XOR8:
        value = 0
        for b in data:
            value ^= b
        return value

    if algo is ChecksumAlgorithm.SUM8:
        return sum(data) & 0xFF

    if algo is ChecksumAlgorithm.CRC8:
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc

    if algo is ChecksumAlgorithm.CRC16_CCITT:
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        return crc

    raise AssertionError(f"unhandled ChecksumAlgorithm member {algo!r}")  # pragma: no cover


def verify_checksum(algorithm: str | ChecksumAlgorithm, data: bytes, expected: int) -> bool:
    """`True` iff `compute_checksum(algorithm, data) == expected`. `"none"` always verifies True
    (no checksum configured means nothing to check -- the caller decides separately whether that's
    an acceptable configuration for a real hardware run)."""
    if _resolve(algorithm) is ChecksumAlgorithm.NONE:
        return True
    return compute_checksum(algorithm, data) == expected

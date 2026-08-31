"""Contract/schema versioning for `models.stm32_processed.STM32ProcessedFrame`.

`metadata.protocol_version` is a semantic version string (``"MAJOR.MINOR.PATCH"``). The MAJOR
component is the compatibility axis:

* **Same MAJOR, any MINOR/PATCH** -> accepted. MINOR/PATCH increments are backward compatible by
  contract (new *optional* fields only), and `STM32ProcessedFrame`'s models use ``extra="ignore"``
  so an older Edge build silently drops fields a newer firmware minor-bump adds.
* **Unknown MAJOR** -> rejected with `STM32ContractVersionError`. A breaking firmware change
  (removed/renamed/reshaped field) bumps MAJOR; the Edge must be upgraded (and
  `SUPPORTED_MAJOR_VERSIONS` extended, plus a new deserializer branch if the shape changed)
  before it can accept those frames. It must never silently mis-parse them.

This mirrors the exact policy `streaming.protocol.PROTOCOL_VERSION` /
`FrameIdValidator`-style version handling already use for the Edge->Unity/Dashboard wire -- one
rule, applied one layer upstream at the STM32 boundary.
"""

from __future__ import annotations

from enum import Enum

from models.stm32_processed import PROCESSED_CONTRACT_VERSION

from .errors import STM32ContractVersionError

# MAJOR versions this Edge build can decode and map to LiveState. Extend this (and add a
# deserializer branch if the payload SHAPE changed, not just its field set) when adopting a new
# breaking firmware contract.
SUPPORTED_MAJOR_VERSIONS: frozenset[int] = frozenset({1})


class ContractVersionStatus(str, Enum):
    """Result of classifying a frame's ``metadata.protocol_version`` -- the processed-contract
    analogue of `streaming.protocol.FrameIdStatus`."""

    OK = "ok"  # MAJOR is supported -- decode normally
    UNSUPPORTED_MAJOR = "unsupported_major"  # parseable, but MAJOR not in SUPPORTED_MAJOR_VERSIONS
    UNPARSEABLE = "unparseable"  # not a "int.int.int" string at all


def parse_semver(version: str) -> tuple[int, int, int]:
    """``"1.4.2"`` -> ``(1, 4, 2)``. Raises `STM32ContractVersionError` for anything that is not
    exactly three dot-separated non-negative integers (no pre-release/build suffixes -- this
    contract does not use them)."""
    if not isinstance(version, str):
        raise STM32ContractVersionError(f"protocol_version must be a string, got {type(version).__name__}.")
    parts = version.strip().split(".")
    if len(parts) != 3:
        raise STM32ContractVersionError(
            f"protocol_version {version!r} is not a 'MAJOR.MINOR.PATCH' semantic version."
        )
    try:
        major, minor, patch = (int(p) for p in parts)
    except ValueError as e:
        raise STM32ContractVersionError(
            f"protocol_version {version!r} has a non-integer component."
        ) from e
    if major < 0 or minor < 0 or patch < 0:
        raise STM32ContractVersionError(f"protocol_version {version!r} has a negative component.")
    return major, minor, patch


def classify_contract_version(version: str) -> ContractVersionStatus:
    """Non-raising classification, for a caller that wants to log/branch rather than reject
    outright (e.g. a future 'accept unknown MINOR but warn' policy)."""
    try:
        major, _minor, _patch = parse_semver(version)
    except STM32ContractVersionError:
        return ContractVersionStatus.UNPARSEABLE
    return ContractVersionStatus.OK if major in SUPPORTED_MAJOR_VERSIONS else ContractVersionStatus.UNSUPPORTED_MAJOR


def require_supported_version(version: str) -> None:
    """Raise `STM32ContractVersionError` unless `version` parses and its MAJOR is supported.
    Called first by every deserializer, before any structural parse -- so an unknown-MAJOR frame
    (whose shape may have changed entirely) gets a crisp version error, not a pile of
    missing-field errors."""
    status = classify_contract_version(version)
    if status is ContractVersionStatus.UNPARSEABLE:
        raise STM32ContractVersionError(
            f"protocol_version {version!r} is not a parseable semantic version."
        )
    if status is ContractVersionStatus.UNSUPPORTED_MAJOR:
        major = parse_semver(version)[0]
        raise STM32ContractVersionError(
            f"STM32 processed-frame protocol major version {major} (from {version!r}) is not "
            f"supported by this Edge build (supported: "
            f"{sorted(SUPPORTED_MAJOR_VERSIONS)}). Upgrade the Edge software."
        )


__all__ = [
    "PROCESSED_CONTRACT_VERSION",
    "SUPPORTED_MAJOR_VERSIONS",
    "ContractVersionStatus",
    "parse_semver",
    "classify_contract_version",
    "require_supported_version",
]

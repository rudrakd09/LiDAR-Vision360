"""Exceptions for the STM32 processed-perception contract (`models.stm32_processed`).

Both extend `STM32Error` (`datasources.stm32.errors`), so a caller can already catch every
STM32-side failure -- raw-protocol *and* processed-contract -- with one `except STM32Error`,
the same broad tolerance `scripts/serve_unity_bridge.py` applies elsewhere. A processed frame
that fails validation is skipped, logged, and surfaced as an `ERROR` on the wire (Phase 3+) --
never allowed to silently produce partial/fabricated objects.
"""

from __future__ import annotations

from ..errors import STM32Error


class STM32ProcessedFrameError(STM32Error):
    """One received STM32 processed-perception frame failed schema or semantic validation.

    Raised (with a message naming the exact field and why) instead of ever returning a
    half-populated `STM32ProcessedFrame` or fabricating a missing value. Every
    `ProcessedFrameDeserializer` converts a bare `pydantic.ValidationError` into this so callers
    have a single exception type to handle.
    """


class STM32ContractVersionError(STM32ProcessedFrameError):
    """The frame's ``metadata.protocol_version`` is unparseable, or its MAJOR component is not one
    this Edge build supports (`datasources.stm32.processed.version.SUPPORTED_MAJOR_VERSIONS`).

    A distinct subtype because the correct operator response differs: a version mismatch means
    "this Edge build is older than the firmware -- upgrade the Edge", not "the firmware sent a
    corrupt frame". Retrying cannot fix it.
    """


__all__ = ["STM32ProcessedFrameError", "STM32ContractVersionError"]

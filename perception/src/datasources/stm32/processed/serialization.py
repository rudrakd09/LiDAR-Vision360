"""Serializer / deserializer interfaces for `STM32ProcessedFrame`, plus a reference JSON codec.

    STM32ProcessedFrame --Serializer.encode()--> bytes
                        ... ESP32 / Wi-Fi transport (NOT defined here) ...
    bytes --Deserializer.decode()--> STM32ProcessedFrame   (validated, or STM32ProcessedFrameError)

**The real STM32<->ESP32<->Edge wire format is not yet specified** (framing, field layout,
endianness, CRC, compression, Wi-Fi transport -- all pending the hardware team; see
docs/hardware-integration.md). So this module defines only the *seam*:

* `ProcessedFrameSerializer` / `ProcessedFrameDeserializer` -- the abstract contract every codec
  implements. ESP32Source (Phase 3) will depend on these interfaces, never on a concrete codec.
* `JsonProcessedFrameCodec` -- a concrete, dependency-free reference implementation (UTF-8 JSON
  via pydantic). It is explicitly **not** the hardware wire format; it exists for development,
  tests, fixtures, logging, and the Phase 11 simulation-driven bring-up of the hardware path.
  Swap it for a real codec once the spec lands -- `STM32ProcessedFrame`, the validator, and
  every consumer stay unchanged.

Every deserializer MUST:
  1. version-gate first (`require_supported_version`) so an unknown-MAJOR frame yields a crisp
     `STM32ContractVersionError`, not a pile of missing-field errors;
  2. convert a `pydantic.ValidationError` into `STM32ProcessedFrameError` (callers handle one
     exception type);
  3. run `validate_processed_frame` before returning (unless the caller explicitly opts out for
     already-trusted data).
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from typing import Any

from pydantic import ValidationError

from models.stm32_processed import STM32ProcessedFrame

from .errors import STM32ContractVersionError, STM32ProcessedFrameError
from .validation import validate_processed_frame
from .version import require_supported_version


class ProcessedFrameSerializer(ABC):
    """Encodes an `STM32ProcessedFrame` for transmission over the (future) ESP32 link."""

    @abstractmethod
    def encode(self, frame: STM32ProcessedFrame) -> bytes:
        """Return the wire bytes for `frame`. Must round-trip through this codec's own
        `decode` (see `ProcessedFrameDeserializer`)."""


class ProcessedFrameDeserializer(ABC):
    """Decodes wire bytes from the ESP32 link back into a validated `STM32ProcessedFrame`."""

    @abstractmethod
    def decode(self, raw: bytes, *, validate: bool = True) -> STM32ProcessedFrame:
        """Parse `raw` into an `STM32ProcessedFrame`.

        Raises `STM32ContractVersionError` for an unparseable/unsupported ``protocol_version``,
        and `STM32ProcessedFrameError` for any other structural or semantic problem -- never a
        bare `pydantic.ValidationError`, and never a partially-populated frame.

        `validate=False` skips `validate_processed_frame` (version gating and structural parsing
        still run) -- for already-trusted, Edge-internal re-hydration only.
        """


class ProcessedFrameCodec(ProcessedFrameSerializer, ProcessedFrameDeserializer, ABC):
    """Convenience base for a codec that does both directions."""


def parse_processed_frame(
    data: bytes | bytearray | str | dict[str, Any],
    *,
    validate: bool = True,
    **validation_limits: Any,
) -> STM32ProcessedFrame:
    """Transport-agnostic entry point: turn already-deserialized JSON text / bytes / a plain
    ``dict`` into a validated `STM32ProcessedFrame`. This is the shared core every JSON-family
    codec (and tests) call; a binary codec would have its own byte-level parse but the same
    version-gate -> wrap -> validate flow.

    `**validation_limits` are forwarded to `validate_processed_frame` (e.g. ``now=``,
    ``max_distance_m=``) so a caller can wire them to `Settings` later.
    """
    if isinstance(data, (bytes, bytearray)):
        try:
            data = data.decode("utf-8")
        except UnicodeDecodeError as e:
            raise STM32ProcessedFrameError(f"processed frame is not valid UTF-8: {e}") from e

    if isinstance(data, str):
        try:
            payload = json.loads(data)
        except json.JSONDecodeError as e:
            raise STM32ProcessedFrameError(f"processed frame is not valid JSON: {e}") from e
    elif isinstance(data, dict):
        payload = data
    else:
        raise STM32ProcessedFrameError(
            f"processed frame must be bytes, str, or dict -- got {type(data).__name__}."
        )

    if not isinstance(payload, dict):
        raise STM32ProcessedFrameError(f"processed frame must decode to a JSON object, got {type(payload).__name__}.")

    # 1. Version gate FIRST -- before structural parsing, so a future breaking-MAJOR frame (whose
    #    shape may differ entirely) fails with a clear version error rather than field errors.
    metadata = payload.get("metadata")
    if not isinstance(metadata, dict) or "protocol_version" not in metadata:
        raise STM32ProcessedFrameError("processed frame is missing required field metadata.protocol_version.")
    require_supported_version(metadata["protocol_version"])

    # 2. Structural parse -- wrap pydantic's error as our single exception type.
    try:
        frame = STM32ProcessedFrame.model_validate(payload)
    except ValidationError as e:
        raise STM32ProcessedFrameError(f"processed frame failed schema validation: {e}") from e

    # 3. Semantic / cross-field / plausibility validation.
    if validate:
        validate_processed_frame(frame, **validation_limits)
    return frame


class JsonProcessedFrameCodec(ProcessedFrameCodec):
    """Reference codec: UTF-8 JSON, one object per `encode()` call, via pydantic's own
    ``model_dump_json`` / ``model_validate``.

    NOT the hardware wire format (see module docstring). Deterministic and dependency-free, so it
    is safe for fixtures and golden-file tests.
    """

    def __init__(self, *, default_validate: bool = True, **validation_limits: Any) -> None:
        """`default_validate` / `validation_limits` set the defaults for `decode`; an individual
        `decode` call can still override `validate`."""
        self._default_validate = default_validate
        self._validation_limits = validation_limits

    def encode(self, frame: STM32ProcessedFrame) -> bytes:
        if not isinstance(frame, STM32ProcessedFrame):
            raise STM32ProcessedFrameError(f"encode() expects an STM32ProcessedFrame, got {type(frame).__name__}.")
        return frame.model_dump_json().encode("utf-8")

    def decode(self, raw: bytes | bytearray | str | dict[str, Any], *, validate: bool | None = None) -> STM32ProcessedFrame:
        do_validate = self._default_validate if validate is None else validate
        return parse_processed_frame(raw, validate=do_validate, **self._validation_limits)


__all__ = [
    "ProcessedFrameSerializer",
    "ProcessedFrameDeserializer",
    "ProcessedFrameCodec",
    "JsonProcessedFrameCodec",
    "parse_processed_frame",
    "STM32ProcessedFrameError",
    "STM32ContractVersionError",
]

"""`CANFrameEncoder` -- semantic CAN payloads -> `CANFrame`s.

Two responsibilities:

1. **Message assembly** -- turn one frame's `ProcessedPerceptionPayloads` into the set of
   `CANFrame`s the configured message catalogue calls for (PERCEPTION_HEADER once, OBJECT_STATE
   once per object up to the multiplex cap, SAFETY_STATE once), validating every signal first
   (`CANValidationError` -> that message is skipped, not the frame).

2. **Bit packing** -- `SignalPacker` implements the standard DBC start-bit / length / byte-order /
   scale / offset / signed mechanism. It is a *mechanism*, not an invented layout: it produces
   bytes only when the `CANMessageSpec` is fully specified. Until then, `encode()` returns an
   **unencoded** `CANFrame` (semantic signals only) so the pipeline runs end to end today.
"""

from __future__ import annotations

import time

from datasources.stm32.crc import checksum_length_bytes, compute_checksum

from .errors import CANConfigurationError, CANValidationError
from .frame import CANFrame
from .message_spec import CANByteOrder, CANMessageSpec, CANOutputConfig, CANSignalSpec
from .payload import (
    ObjectStatePayload,
    PerceptionHeaderPayload,
    ProcessedPerceptionPayloads,
    SafetyStatePayload,
    validate_header,
    validate_object_state,
    validate_safety_state,
)


class SignalPacker:
    """Standard DBC-style bit packing. Only usable when every signal has start_bit / length_bits
    / byte_order (and, for value signals, scale/signed)."""

    @staticmethod
    def _raw_from_physical(value: float, spec: CANSignalSpec) -> int:
        sc = spec.scaling
        scale = sc.scale if sc.scale not in (None, 0) else 1.0
        offset = sc.offset or 0.0
        raw = round((value - offset) / scale)
        bits = spec.length_bits or 0
        if sc.signed:
            lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
            raw = max(lo, min(hi, raw))
            if raw < 0:
                raw += 1 << bits
        else:
            raw = max(0, min((1 << bits) - 1, raw))
        return raw & ((1 << bits) - 1)

    @classmethod
    def pack(cls, spec: CANMessageSpec, values: dict[str, float | int | bool]) -> bytes:
        if spec.dlc is None:
            raise CANConfigurationError(f"{spec.name}.dlc is unspecified -- cannot pack.")
        buf = bytearray(spec.dlc)
        for sig in spec.signals:
            if sig.is_checksum:
                continue  # computed last, over the packed bytes
            if not sig.is_layout_specified:
                raise CANConfigurationError(f"{spec.name}.{sig.name} layout is unspecified -- cannot pack.")
            raw = cls._raw_from_physical(float(values.get(sig.name, 0) or 0), sig) if not sig.is_counter \
                else int(values.get(sig.name, 0)) & ((1 << (sig.length_bits or 0)) - 1)
            cls._write_bits(buf, sig.start_bit, sig.length_bits, sig.byte_order, raw)

        cs_sig = next((s for s in spec.signals if s.is_checksum), None)
        if cs_sig is not None and spec.checksum_algorithm:
            n = checksum_length_bytes(spec.checksum_algorithm)
            if n:
                cs = compute_checksum(spec.checksum_algorithm, bytes(buf))
                cls._write_bits(buf, cs_sig.start_bit, cs_sig.length_bits, cs_sig.byte_order, cs & ((1 << (cs_sig.length_bits or (8 * n))) - 1))
        return bytes(buf)

    @staticmethod
    def _write_bits(buf: bytearray, start_bit: int, length: int, order: CANByteOrder, raw: int) -> None:
        if order is CANByteOrder.LITTLE_ENDIAN:
            for i in range(length):
                bit = (raw >> i) & 1
                pos = start_bit + i
                buf[pos // 8] |= bit << (pos % 8)
        else:  # big-endian / Motorola sawtooth
            for i in range(length):
                bit = (raw >> (length - 1 - i)) & 1
                pos = start_bit + i
                buf[pos // 8] |= bit << (7 - (pos % 8))

    @staticmethod
    def read_bits(data: bytes, start_bit: int, length: int, order: CANByteOrder) -> int:
        raw = 0
        if order is CANByteOrder.LITTLE_ENDIAN:
            for i in range(length):
                pos = start_bit + i
                raw |= ((data[pos // 8] >> (pos % 8)) & 1) << i
        else:
            for i in range(length):
                pos = start_bit + i
                raw |= ((data[pos // 8] >> (7 - (pos % 8))) & 1) << (length - 1 - i)
        return raw


class CANFrameEncoder:
    def __init__(self, config: CANOutputConfig, *, now_fn=time.time) -> None:
        self._config = config
        self._now = now_fn

    # --- one message ---------------------------------------------------------------------------

    def encode(self, spec: CANMessageSpec, signal_values: dict, *, sequence_number: int, object_index: int | None = None) -> CANFrame:
        """Build one `CANFrame` for `spec`. Encoded (real bytes) iff `spec.is_fully_specified`;
        otherwise an unencoded semantic-only frame -- unless `config.require_fully_specified`,
        which makes an unspecified spec a `CANConfigurationError`."""
        now = self._now()
        values = dict(signal_values)
        values["rolling_counter"] = sequence_number

        if not spec.is_fully_specified:
            if self._config.require_fully_specified:
                raise CANConfigurationError(
                    f"CAN message {spec.name!r} is not fully specified; still PENDING: "
                    + ", ".join(spec.missing_hardware_fields())
                )
            return CANFrame(
                message_name=spec.name, signals=values,
                sequence_number=sequence_number, timestamp=now, object_index=object_index,
            )

        data = SignalPacker.pack(spec, values)
        return CANFrame(
            message_name=spec.name, arbitration_id=spec.can_id, extended=spec.extended_id,
            dlc=spec.dlc, data=data, signals=values,
            sequence_number=sequence_number, timestamp=now, object_index=object_index,
        )

    # --- one processed frame -> all its messages ---------------------------------------------

    def encode_frame(self, payloads: ProcessedPerceptionPayloads, *, sequence_number: int) -> tuple[list[CANFrame], list[str]]:
        """Returns `(frames, validation_errors)`. A message whose signals fail validation is
        omitted from `frames` and its reason appended to `validation_errors` -- never silently
        transmitted, never aborting the other messages."""
        frames: list[CANFrame] = []
        errors: list[str] = []

        header_spec = self._config.message("PERCEPTION_HEADER")
        if header_spec is not None:
            try:
                validate_header(payloads.header)
                frames.append(self.encode(header_spec, _header_signals(payloads.header), sequence_number=sequence_number))
            except CANValidationError as e:
                errors.append(str(e))

        obj_spec = self._config.message("OBJECT_STATE")
        if obj_spec is not None:
            cap = obj_spec.max_multiplex_count if obj_spec.multiplexed else len(payloads.objects)
            for i, obj in enumerate(payloads.objects[:cap]):
                try:
                    validate_object_state(obj)
                    frames.append(self.encode(obj_spec, _object_signals(obj), sequence_number=sequence_number, object_index=i))
                except CANValidationError as e:
                    errors.append(str(e))

        safety_spec = self._config.message("SAFETY_STATE")
        if safety_spec is not None:
            try:
                validate_safety_state(payloads.safety)
                frames.append(self.encode(safety_spec, _safety_signals(payloads.safety), sequence_number=sequence_number))
            except CANValidationError as e:
                errors.append(str(e))

        return frames, errors


def _header_signals(p: PerceptionHeaderPayload) -> dict:
    return {
        "sequence_number": p.sequence_number, "frame_id": p.frame_id, "timestamp_ms": p.timestamp_ms,
        "active_object_count": p.active_object_count, "lidar_ok": int(p.lidar_ok), "radar_ok": int(p.radar_ok),
        "fusion_active": int(p.fusion_active), "stm32_ok": int(p.stm32_ok),
    }


def _object_signals(p: ObjectStatePayload) -> dict:
    return {
        "object_index": p.object_index, "track_id_hash": p.track_id_hash, "object_type": p.object_type_code,
        "distance_m": p.distance_m, "relative_velocity_mps": p.relative_velocity_mps, "confidence": p.confidence,
        # -1.0 is the design's "no TTC / not closing" sentinel; the DBC's real sentinel replaces it once known.
        "object_ttc_s": p.ttc_s if p.ttc_s is not None else -1.0,
        "object_risk": p.risk_code, "fusion_source": p.source_code,
    }


def _safety_signals(p: SafetyStatePayload) -> dict:
    return {
        "overall_risk": p.overall_risk_code, "collision_predicted": int(p.collision_predicted),
        "most_critical_ttc_s": p.most_critical_ttc_s if p.most_critical_ttc_s is not None else -1.0,
        "min_clearance_m": p.min_clearance_m if p.min_clearance_m is not None else -1.0,
        "min_clearance_direction": p.min_clearance_direction_code if p.min_clearance_direction_code is not None else 0,
        "clearance_state": p.clearance_state_code if p.clearance_state_code is not None else 0,
        "front_clearance_m": p.front_m if p.front_m is not None else -1.0,
        "rear_clearance_m": p.rear_m if p.rear_m is not None else -1.0,
        "left_clearance_m": p.left_m if p.left_m is not None else -1.0,
        "right_clearance_m": p.right_m if p.right_m is not None else -1.0,
    }


__all__ = ["SignalPacker", "CANFrameEncoder"]

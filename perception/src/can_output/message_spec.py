"""Configurable CAN message / signal definitions for the STM32 -> Vehicle-ECU output.

**Nothing here is a real CAN specification.** Every value the hardware team must provide --
bitrate, CAN ID, standard/extended identifier, DLC, per-signal start bit / length / byte order /
scale / offset / signed-ness, checksum algorithm, transmission period -- defaults to `None` and
is reported by `missing_hardware_fields()` / `CANOutputConfig.pending_hardware_parameters()`.
What *is* defined here is the *software design*: which processed-perception fields each
conceptual message carries, and the mechanism for packing them once the layout is known.

The `*_code` ordinal mappings (`OBJECT_TYPE_CODES`, `RISK_CODES`, `CLEARANCE_DIRECTION_CODES`)
are a **PROPOSED** stable encoding of this project's enums -- offered as a starting point for the
ECU DBC, not a fixed wire fact. Remap them to match the real DBC when it exists; a single edit
here is all that takes.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, Field

from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel
from models.objects import ObjectClassification

# --- PROPOSED enum -> integer encodings (confirm / remap against the real ECU DBC) -----------

#: PROPOSED. Ordinals for `ObjectClassification` on the wire. `unknown` == 0 deliberately, so an
#: all-zero / defaulted signal reads as "unknown", never as a specific type.
OBJECT_TYPE_CODES: dict[ObjectClassification, int] = {
    ObjectClassification.UNKNOWN: 0,
    ObjectClassification.WALL: 1,
    ObjectClassification.POLE_LIKE: 2,
    ObjectClassification.VEHICLE_LIKE: 3,
    ObjectClassification.PERSON_LIKE: 4,
    ObjectClassification.LARGE_OBSTACLE: 5,
}

#: PROPOSED. Ordinals for `RiskLevel`. Ascending == more severe (a common ECU convention), so a
#: consumer can compare numerically; confirm against the DBC.
RISK_CODES: dict[RiskLevel, int] = {
    RiskLevel.SAFE: 0,
    RiskLevel.WARNING: 1,
    RiskLevel.CRITICAL: 2,
}

#: PROPOSED. Ordinals for `ClearanceDirection`.
CLEARANCE_DIRECTION_CODES: dict[ClearanceDirection, int] = {
    ClearanceDirection.FRONT: 0,
    ClearanceDirection.REAR: 1,
    ClearanceDirection.LEFT: 2,
    ClearanceDirection.RIGHT: 3,
}

#: PROPOSED. Ordinals for `ClearanceState` (four-state, distinct from RiskLevel's three).
CLEARANCE_STATE_CODES: dict[ClearanceState, int] = {
    ClearanceState.SAFE: 0,
    ClearanceState.CAUTION: 1,
    ClearanceState.LOW_CLEARANCE: 2,
    ClearanceState.CRITICAL: 3,
}


def object_type_code(c: ObjectClassification) -> int:
    return OBJECT_TYPE_CODES.get(c, 0)


def risk_code(r: RiskLevel) -> int:
    return RISK_CODES.get(r, 0)


def clearance_direction_code(d: ClearanceDirection) -> int:
    return CLEARANCE_DIRECTION_CODES.get(d, 0)


def clearance_state_code(s: ClearanceState) -> int:
    return CLEARANCE_STATE_CODES.get(s, 0)


# --- transmission mode (a DESIGN choice we can make -- not a hardware fact) ------------------


class CANTransmissionMode(str, Enum):
    PERIODIC = "periodic"                      # sent every cycle_time_ms
    EVENT = "event"                            # sent only when its trigger fires (e.g. risk change)
    PERIODIC_AND_EVENT = "periodic_and_event"  # both -- periodic heartbeat plus immediate on change


class CANByteOrder(str, Enum):
    """The two standard CAN signal layouts. Which one the ECU uses is PENDING -- this enum only
    names the mechanisms (same pattern as `datasources.stm32.crc.ChecksumAlgorithm`)."""

    LITTLE_ENDIAN = "little_endian"  # "Intel"
    BIG_ENDIAN = "big_endian"        # "Motorola" (sawtooth / MSB-first)


# --- configurable message + signal specs ---------------------------------------------------


class CANSignalScaling(BaseModel):
    """`physical = raw * scale + offset`. Every field `None` until the DBC provides it."""

    scale: float | None = Field(default=None, description="PENDING hardware spec.")
    offset: float | None = Field(default=None, description="PENDING hardware spec.")
    signed: bool | None = Field(default=None, description="PENDING hardware spec (two's-complement raw).")
    min_value: float | None = Field(default=None, description="PENDING -- physical-range floor; a value below this fails validation.")
    max_value: float | None = Field(default=None, description="PENDING -- physical-range ceiling.")

    @property
    def is_specified(self) -> bool:
        return self.scale is not None and self.signed is not None


class CANSignalSpec(BaseModel):
    """One signal within a CAN message. `name`/`source` are the software design (fixed here);
    `start_bit`/`length_bits`/`byte_order`/`scaling` are PENDING the DBC."""

    name: str = Field(..., description="Semantic signal name (this project's design, stable).")
    source: str = Field(..., description="Which processed-perception field feeds it, e.g. 'object.distance_m'.")
    unit: str | None = Field(default=None, description="Physical unit of the decoded value (design intent).")

    start_bit: int | None = Field(default=None, description="PENDING hardware spec -- bit position in the frame.")
    length_bits: int | None = Field(default=None, description="PENDING hardware spec.")
    byte_order: CANByteOrder | None = Field(default=None, description="PENDING hardware spec.")
    scaling: CANSignalScaling = Field(default_factory=CANSignalScaling)

    is_counter: bool = Field(default=False, description="This signal carries the rolling message counter.")
    is_checksum: bool = Field(default=False, description="This signal carries the message checksum.")

    @property
    def is_layout_specified(self) -> bool:
        return (
            self.start_bit is not None
            and self.length_bits is not None
            and self.byte_order is not None
            and (self.is_checksum or self.is_counter or self.scaling.is_specified)
        )

    def missing_hardware_fields(self) -> list[str]:
        missing: list[str] = []
        if self.start_bit is None:
            missing.append(f"{self.name}.start_bit")
        if self.length_bits is None:
            missing.append(f"{self.name}.length_bits")
        if self.byte_order is None:
            missing.append(f"{self.name}.byte_order")
        if not (self.is_checksum or self.is_counter) and not self.scaling.is_specified:
            missing.append(f"{self.name}.scaling(scale/signed)")
        return missing


class CANMessageSpec(BaseModel):
    """One CAN message. `name` + `signals[*].name/source` + `transmission_mode` +
    `multiplexed` are the software design. `can_id`/`extended_id`/`dlc`/`cycle_time_ms`/
    `checksum_algorithm` + every signal's layout are PENDING the DBC."""

    name: str = Field(..., description="Semantic message name (this project's design), e.g. 'OBJECT_STATE'.")
    description: str = ""

    can_id: int | None = Field(default=None, description="PENDING hardware spec -- DO NOT invent.")
    extended_id: bool | None = Field(default=None, description="PENDING -- 11-bit (standard) vs 29-bit (extended).")
    dlc: int | None = Field(default=None, ge=0, le=64, description="PENDING -- data length code (0-8 classic CAN, up to 64 CAN-FD).")
    cycle_time_ms: float | None = Field(default=None, gt=0, description="PENDING -- nominal transmission period.")

    transmission_mode: CANTransmissionMode = Field(
        default=CANTransmissionMode.PERIODIC,
        description="Design choice: periodic / event / both. Not a hardware fact.",
    )
    multiplexed: bool = Field(
        default=False,
        description="Design choice: one instance per tracked object (multiplexed by an object index signal).",
    )
    max_multiplex_count: int = Field(
        default=16, ge=1,
        description="Design cap on how many multiplexed object instances are emitted per frame -- bounds bus load.",
    )

    checksum_algorithm: str | None = Field(
        default=None,
        description="PENDING -- one of datasources.stm32.crc.ChecksumAlgorithm ('none'|'xor8'|'sum8'|'crc8'|'crc16_ccitt'). Which one is unknown; the algorithms are reused, not invented.",
    )

    signals: list[CANSignalSpec] = Field(default_factory=list)

    def signal(self, name: str) -> CANSignalSpec | None:
        return next((s for s in self.signals if s.name == name), None)

    @property
    def is_fully_specified(self) -> bool:
        return (
            self.can_id is not None
            and self.extended_id is not None
            and self.dlc is not None
            and all(s.is_layout_specified for s in self.signals)
        )

    def missing_hardware_fields(self) -> list[str]:
        missing: list[str] = []
        if self.can_id is None:
            missing.append(f"{self.name}.can_id")
        if self.extended_id is None:
            missing.append(f"{self.name}.extended_id")
        if self.dlc is None:
            missing.append(f"{self.name}.dlc")
        if self.cycle_time_ms is None and self.transmission_mode is not CANTransmissionMode.EVENT:
            missing.append(f"{self.name}.cycle_time_ms")
        for s in self.signals:
            missing.extend(s.missing_hardware_fields())
        return missing


class CANOutputConfig(BaseModel):
    """The whole STM32->ECU CAN output configuration. Hardware values are PENDING; the Edge-side
    operational guards below are NOT hardware facts and have sensible, overridable defaults."""

    # --- PENDING hardware / DBC spec ---
    bitrate_bps: int | None = Field(default=None, description="PENDING -- e.g. 250000 / 500000. DO NOT invent.")
    interface: str | None = Field(default=None, description="PENDING -- e.g. 'can0', 'PCAN_USBBUS1'.")
    sequence_modulus: int | None = Field(default=None, description="PENDING -- rolling-counter wrap (e.g. 16 for a 4-bit counter). None = don't wrap.")
    default_cycle_time_ms: float | None = Field(default=None, gt=0, description="PENDING -- fallback transmission period for a message whose own cycle_time_ms is unset.")

    messages: list[CANMessageSpec] = Field(default_factory=list)

    # --- Edge-side operational guards (design, not hardware) ---
    max_transmit_rate_hz: float = Field(
        default=100.0, gt=0,
        description="Hard cap on total frames/sec the output will ever emit -- so the model never 'transmits indefinitely at an arbitrary rate'. Not a bus parameter.",
    )
    queue_max_frames: int = Field(default=64, ge=1, description="Bounded outgoing queue depth; overflow drops the oldest and is counted.")
    require_fully_specified: bool = Field(
        default=False,
        description="True -> STM32CANOutput.start() raises CANConfigurationError until every PENDING field is filled. False (default) -> unencoded (semantic-only) frames are produced so the pipeline works now.",
    )
    bus_recovery_initial_backoff_s: float = Field(default=1.0, gt=0)
    bus_recovery_max_backoff_s: float = Field(default=10.0, gt=0)

    def message(self, name: str) -> CANMessageSpec | None:
        return next((m for m in self.messages if m.name == name), None)

    @property
    def is_ready_for_hardware(self) -> bool:
        return (
            self.bitrate_bps is not None
            and bool(self.messages)
            and all(m.is_fully_specified for m in self.messages)
        )

    def pending_hardware_parameters(self) -> dict[str, list[str]]:
        """Every value still required from the hardware team, grouped by area."""
        pending: dict[str, list[str]] = {"bus": []}
        if self.bitrate_bps is None:
            pending["bus"].append("bitrate_bps")
        if self.interface is None:
            pending["bus"].append("interface")
        for m in self.messages:
            miss = m.missing_hardware_fields()
            if miss:
                pending[m.name] = miss
        return {k: v for k, v in pending.items() if v}


# --- the PROPOSED (not final) conceptual message catalogue ---------------------------------


def build_default_message_catalog() -> list[CANMessageSpec]:
    """The conceptual message set the software is built around -- **not** the final CAN messages.
    Semantic names/sources are fixed; every CAN-layout field is `None` (PENDING). Rework freely
    to match the real ECU message set: nothing downstream assumes these exact messages, only the
    `CANMessageSpec` shape."""
    counter = CANSignalSpec(name="rolling_counter", source="tx.counter", is_counter=True)
    checksum = CANSignalSpec(name="checksum", source="tx.checksum", is_checksum=True)

    header = CANMessageSpec(
        name="PERCEPTION_HEADER",
        description="Per-frame identity + sensor/system health.",
        transmission_mode=CANTransmissionMode.PERIODIC,
        signals=[
            CANSignalSpec(name="sequence_number", source="header.sequence_number", unit="count"),
            CANSignalSpec(name="frame_id", source="header.frame_id", unit="count"),
            CANSignalSpec(name="timestamp_ms", source="header.timestamp_ms", unit="ms"),
            CANSignalSpec(name="active_object_count", source="header.active_object_count", unit="count"),
            CANSignalSpec(name="lidar_ok", source="header.lidar_ok", unit="bool"),
            CANSignalSpec(name="radar_ok", source="header.radar_ok", unit="bool"),
            CANSignalSpec(name="fusion_active", source="header.fusion_active", unit="bool"),
            CANSignalSpec(name="stm32_ok", source="header.stm32_ok", unit="bool"),
            counter.model_copy(), checksum.model_copy(),
        ],
    )
    object_state = CANMessageSpec(
        name="OBJECT_STATE",
        description="Per-tracked-object state (one instance per object, multiplexed by object_index).",
        transmission_mode=CANTransmissionMode.PERIODIC,
        multiplexed=True,
        signals=[
            CANSignalSpec(name="object_index", source="object.index", unit="count"),
            CANSignalSpec(name="track_id_hash", source="object.track_id_hash", unit="hash"),
            CANSignalSpec(name="object_type", source="object.object_type_code", unit="enum(OBJECT_TYPE_CODES)"),
            CANSignalSpec(name="distance_m", source="object.distance_m", unit="m"),
            CANSignalSpec(name="relative_velocity_mps", source="object.relative_velocity_mps", unit="m/s"),
            CANSignalSpec(name="confidence", source="object.confidence", unit="0..1"),
            CANSignalSpec(name="object_ttc_s", source="object.ttc_s", unit="s"),
            CANSignalSpec(name="object_risk", source="object.risk_code", unit="enum(RISK_CODES)"),
            CANSignalSpec(name="fusion_source", source="object.source", unit="enum(lidar/radar/fused)"),
            counter.model_copy(), checksum.model_copy(),
        ],
    )
    safety_state = CANMessageSpec(
        name="SAFETY_STATE",
        description="Frame-level safety summary: risk, TTC, directional clearance.",
        transmission_mode=CANTransmissionMode.PERIODIC_AND_EVENT,  # heartbeat + immediate on risk change
        signals=[
            CANSignalSpec(name="overall_risk", source="safety.overall_risk_code", unit="enum(RISK_CODES)"),
            CANSignalSpec(name="collision_predicted", source="safety.collision_predicted", unit="bool"),
            CANSignalSpec(name="most_critical_ttc_s", source="safety.most_critical_ttc_s", unit="s"),
            CANSignalSpec(name="min_clearance_m", source="safety.min_clearance_m", unit="m"),
            CANSignalSpec(name="min_clearance_direction", source="safety.min_clearance_direction_code", unit="enum(CLEARANCE_DIRECTION_CODES)"),
            CANSignalSpec(name="clearance_state", source="safety.clearance_state_code", unit="enum(CLEARANCE_STATE_CODES)"),
            CANSignalSpec(name="front_clearance_m", source="safety.front_m", unit="m"),
            CANSignalSpec(name="rear_clearance_m", source="safety.rear_m", unit="m"),
            CANSignalSpec(name="left_clearance_m", source="safety.left_m", unit="m"),
            CANSignalSpec(name="right_clearance_m", source="safety.right_m", unit="m"),
            counter.model_copy(), checksum.model_copy(),
        ],
    )
    return [header, object_state, safety_state]


def build_default_config(**overrides) -> CANOutputConfig:
    """A `CANOutputConfig` with the proposed catalogue and every hardware value still PENDING.
    `overrides` set operational guards (e.g. `max_transmit_rate_hz=`) or, once known, hardware
    values."""
    base = dict(messages=build_default_message_catalog())
    base.update(overrides)
    return CANOutputConfig(**base)


__all__ = [
    "OBJECT_TYPE_CODES", "RISK_CODES", "CLEARANCE_DIRECTION_CODES", "CLEARANCE_STATE_CODES",
    "object_type_code", "risk_code", "clearance_direction_code", "clearance_state_code",
    "CANTransmissionMode", "CANByteOrder",
    "CANSignalScaling", "CANSignalSpec", "CANMessageSpec", "CANOutputConfig",
    "build_default_message_catalog", "build_default_config",
]

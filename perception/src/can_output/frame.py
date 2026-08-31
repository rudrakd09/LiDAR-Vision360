"""`CANFrame` -- one message ready for (or already handed to) a `CANTransport`.

A frame is either **encoded** (the message spec is fully specified -> `arbitration_id`/`data`/
`dlc` are real bytes ready for a controller) or **unencoded** (the CAN layout is still PENDING
-> `arbitration_id`/`data` are `None`, and `signals` carries the semantic values). Both forms
flow through the whole pipeline identically, so nothing downstream has to wait for the DBC --
requirement 11 ("once the spec is given, only the mapping needs completing").
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class CANFrame(BaseModel):
    model_config = {"arbitrary_types_allowed": True}

    message_name: str = Field(..., description="Semantic message name, e.g. 'OBJECT_STATE'.")
    arbitration_id: int | None = Field(default=None, description="CAN ID -- None until the message spec provides one (PENDING).")
    extended: bool | None = Field(default=None, description="29-bit extended identifier? None until specified.")
    dlc: int | None = Field(default=None, description="Data length -- None until specified.")
    data: bytes | None = Field(default=None, description="Packed payload bytes -- None for an unencoded (semantic-only) frame.")

    signals: dict[str, float | int | str | bool | None] = Field(
        default_factory=dict,
        description="The semantic signal values that were (or would be) packed. Always populated.",
    )

    sequence_number: int = Field(..., ge=0, description="The output stage's own rolling tx counter for this frame.")
    timestamp: float = Field(..., description="Unix-epoch seconds this frame was built.")
    object_index: int | None = Field(default=None, description="For a multiplexed OBJECT_STATE frame -- which tracked object it carries.")

    @property
    def is_encoded(self) -> bool:
        return self.data is not None and self.arbitration_id is not None

    def __str__(self) -> str:  # for [SIM-CAN] log lines
        if self.is_encoded:
            return f"{self.message_name} id=0x{self.arbitration_id:X} dlc={self.dlc} data={self.data.hex()} seq={self.sequence_number}"
        return f"{self.message_name} <unencoded, layout PENDING> signals={self.signals} seq={self.sequence_number}"


__all__ = ["CANFrame"]

"""Exceptions for the STM32 -> Vehicle-ECU CAN output stage (`can_output`).

This is a SOFTWARE MODEL of the CAN-transmit stage the STM32 firmware performs in the real
vehicle -- see `can_output/__init__.py`. The exception hierarchy mirrors the failure modes a real
CAN controller surfaces so the same handling code carries over once a real backend is dropped in.

* `CANConfigurationError` -- a required CAN/DBC parameter (bitrate, CAN ID, DLC, byte layout,
  scaling, ...) is missing or invalid. Raised only when the caller explicitly requires a
  fully-specified configuration; never invented around. Not retried.
* `CANValidationError` -- one signal value failed validation. The offending message is dropped and
  counted; it is never silently transmitted.
* `CANBusError` -- the bus is unavailable / bus-off / the link dropped. Recoverable: the output
  controller marks itself BUS_OFF and retries with backoff.
* `CANTransmitError` -- a single frame's transmission failed (arbitration loss, tx-queue full on
  the controller, ...). Recoverable per-frame.
"""

from __future__ import annotations


class CANOutputError(RuntimeError):
    """Base class for every error raised by `can_output`."""


class CANConfigurationError(CANOutputError):
    """A required CAN message / signal / bus parameter is unspecified or invalid.

    Raised (listing exactly which parameters are still PENDING the hardware team's CAN/DBC
    specification) rather than ever guessing a CAN ID, bitrate, DLC, byte offset, scaling factor,
    or transmission period.
    """


class CANValidationError(CANOutputError):
    """A processed-perception signal value failed validation and must not be transmitted.

    The message carrying it is dropped and recorded in `CANOutputHealth.validation_failures`;
    other messages for the same frame are unaffected.
    """


class CANBusError(CANOutputError):
    """The CAN bus is unavailable, bus-off, or the transport link failed.

    The `STM32CANOutput` controller catches this, transitions to `BUS_OFF`, and attempts
    recovery with exponential backoff.
    """


class CANTransmitError(CANOutputError):
    """A single frame failed to transmit (recoverable). Counted; the frame may be re-queued once."""


__all__ = [
    "CANOutputError",
    "CANConfigurationError",
    "CANValidationError",
    "CANBusError",
    "CANTransmitError",
]

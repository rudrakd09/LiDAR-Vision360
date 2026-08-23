# STM32 Firmware

Firmware itself is not part of this repository/phase. This directory documents the Edge-side
expectations `perception/src/datasources/stm32_source.py` was built against, so firmware
development and Edge development can proceed in parallel once the wire protocol is fixed.

See [docs/hardware-integration.md](../../docs/hardware-integration.md) for the full checklist of
exactly which protocol fields the firmware needs to fix (framing, byte order, message-type IDs,
CRC, scaling, timestamps) and the STM32-to-Edge architecture diagram.

## Expected shape (not yet fixed)

The Edge side assumes -- pending confirmation, see the checklist -- a simple
marker + message-type + sequence-number + payload + checksum framing convention, with LiDAR
(A3M1) and radar (R121, via CAN) data distinguished by message-type on the same UART link. None of
the actual byte values (markers, type IDs, checksum algorithm, field offsets/scaling) are assumed;
only that structural shape is scaffolded on the Edge side, ready to be configured once real values
exist.

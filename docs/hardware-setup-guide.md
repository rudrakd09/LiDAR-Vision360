# Hardware Setup Guide (wiring & power)

Physical bring-up reference for the LiDAR-Vision360 rig. Pair this with
`docs/hardware-validation-procedure.md` (the per-signal specification checklist + the TEST 1→9
electrical/decode runbook) and `docs/final-demonstration-package.md` (the demo procedure).

> **Every numeric value in this guide marked `⟨PENDING SPEC⟩` is unknown in this repository.**
> Fill it from the device datasheet / STM32 firmware / vehicle DBC before wiring, and verify it
> against the datasheet again after wiring. Nothing here is invented — placeholders stay
> placeholders until a real spec replaces them. No hardware, firmware, or spec is present in this
> environment (`serial.tools.list_ports.comports()` → NONE).

---

## 1. Target topology

```
        A3M1 LiDAR ──UART──►┐
                            ├──► STM32F103C8T6 ──CAN──► Vehicle ECU
        R121 Radar ──CAN───►┘        │
                                     └──(STM32→ESP32 link)──► ESP32 ──Wi-Fi──► Edge PC
                                                                                  │
                                              ┌───────────────────────────────────┤
                                              ▼                 ▼                 ▼
                                          Dashboard           Unity          PostgreSQL
```

The STM32 runs the **full** perception + fusion pipeline on-device. The Edge PC does **no**
perception in hardware mode — it consumes the STM32's already-processed frames via `ESP32Source`.

---

## 2. Power

| Rail | Feeds | Voltage / current | Notes |
|---|---|---|---|
| LiDAR supply | A3M1 (logic + motor) | ⟨PENDING SPEC — A3M1 datasheet⟩ | The motor draws a current spike at spin-up; size the supply for peak, not idle. Separate the motor rail from logic if the datasheet says so. |
| Radar supply | R121 | ⟨PENDING SPEC — R121 datasheet⟩ | — |
| STM32 board | STM32F103C8T6 + CAN transceiver(s) | 3.3 V logic (5 V in via USB/regulator) | The BluePill's on-board regulator is weak; power the CAN transceivers from a dedicated 3.3 V/5 V rail per the transceiver datasheet. |
| ESP32 | ESP32 module | 3.3 V regulated; 5 V via USB | Wi-Fi TX current spikes ~ hundreds of mA — use a supply with margin or the module browns out mid-transmit. |

**Common ground:** tie the grounds of A3M1, R121, STM32, ESP32, and (where isolation allows) the
CAN buses together. A floating ground is the most common cause of intermittent UART/CAN errors.

---

## 3. A3M1 LiDAR → STM32 (UART)

| Item | Value |
|---|---|
| Interface | UART (asynchronous serial) |
| Baud rate | ⟨PENDING SPEC — `LIDAR_STM32_UART_BAUDRATE`; the `115200` in config is an **unconfirmed** placeholder⟩ |
| Frame format | ⟨PENDING SPEC — 8N1 assumed, confirm⟩ |
| Level | ⟨PENDING SPEC — confirm A3M1 UART is 3.3 V TTL; add a level shifter if it is 5 V⟩ |
| Motor control | ⟨PENDING SPEC — A3M1 may need a PWM/enable line or external motor driver; confirm from datasheet⟩ |

Wiring:

| A3M1 pin | → | STM32 pin | Note |
|---|---|---|---|
| LiDAR TX | → | STM32 USART RX | data from sensor |
| LiDAR RX | → | STM32 USART TX | commands to sensor (start scan / motor) |
| GND | → | STM32 GND | common ground |
| Motor / PWM (if present) | → | STM32 timer PWM or motor driver | ⟨PENDING SPEC⟩ |

Decode payload layout (angle/distance/quality offsets, scaling, scan-boundary marker, CRC) per
`docs/hardware-validation-procedure.md` §"A3M1 → STM32 UART specification". A concrete
`datasources.stm32.parsers.LiDARMessageParser` subclass is required on the Edge side only if the
Edge ever parses raw STM32 serial (legacy path); in the target architecture the STM32 firmware
owns this decode.

---

## 4. R121 Radar → STM32 (CAN)

| Item | Value |
|---|---|
| Interface | CAN 2.0 |
| Bitrate | ⟨PENDING SPEC — R121 datasheet⟩ |
| CAN ID(s) | ⟨PENDING SPEC — target list, DLC, byte layout⟩ |
| Signal scaling / offset / sign | ⟨PENDING SPEC — per signal: target ID, range, angle, relative velocity, confidence/status, timestamp/seq⟩ |
| Transceiver | ⟨PENDING SPEC — 3.3 V-logic CAN transceiver on the STM32 side (e.g. SN65HVD230-class); confirm⟩ |

Wiring:

| R121 | → | STM32 CAN transceiver | Note |
|---|---|---|---|
| CANH | → | CANH | twisted pair with CANL |
| CANL | → | CANL | — |
| GND | → | transceiver GND / common | — |
| Termination | — | **120 Ω across CANH–CANL at each physical end of the bus** | two resistors total on the segment, not one per node |

Radar is fused **on the STM32** per the target architecture. The Edge does not re-fuse in
hardware mode.

---

## 5. STM32 → Vehicle ECU (CAN)

| Item | Value |
|---|---|
| Interface | CAN 2.0 (separate bus from the R121 bus unless the vehicle design says otherwise) |
| Bitrate | ⟨PENDING SPEC — vehicle DBC; `LIDAR_CAN_OUTPUT_BITRATE_BPS`⟩ |
| Message catalogue | ⟨PENDING SPEC — confirm or replace the proposed `PERCEPTION_HEADER` / `OBJECT_STATE` / `SAFETY_STATE`; see `docs/can-output.md`⟩ |
| Per message | CAN ID, standard/extended, DLC, transmission period ⟨PENDING SPEC⟩ |
| Per signal | start bit, length, byte order, scale, offset, signed-ness ⟨PENDING SPEC⟩ |
| Checksum + rolling counter | algorithm + bit position; counter width ⟨PENDING SPEC — `LIDAR_CAN_OUTPUT_CHECKSUM_ALGORITHM` / `_SEQUENCE_MODULUS`⟩ |

Wiring: CANH/CANL twisted pair to the ECU bus; **120 Ω at both physical ends**; common ground
where the bus is not galvanically isolated. Put a CAN analyser (or bench ECU) on the bus for
TEST 4.

The Edge-side `can_output` model is configuration-only and needs a real
`can_output.transport.CANTransport` subclass (e.g. wrapping `python-can`) **only** if the Edge PC
is used to transmit or observe this bus for cross-checking; `python-can` is not a project
dependency.

---

## 6. STM32 → ESP32 (local link)

| Item | Value |
|---|---|
| Physical link | ⟨PENDING SPEC — UART or SPI between STM32 and ESP32; confirm from firmware⟩ |
| Baud / mode | ⟨PENDING SPEC⟩ |
| Framing | ⟨PENDING SPEC — length prefix / delimiter / one message per frame⟩ |
| Payload encoding | ⟨PENDING SPEC — JSON assumed by `JsonProcessedFrameCodec`; if CBOR/protobuf/binary, a `datasources.stm32.processed.ProcessedFrameDeserializer` subclass is required⟩ |
| CRC / auth | ⟨PENDING SPEC⟩ |

Wiring (if UART): STM32 TX→ESP32 RX, STM32 RX→ESP32 TX, GND↔GND, both at 3.3 V logic.

---

## 7. ESP32 → Edge PC (Wi-Fi)

| Item | Value |
|---|---|
| Transport | ⟨PENDING SPEC — TCP / UDP / WebSocket / MQTT / raw framed; `LIDAR_ESP32_TRANSPORT`⟩ |
| Host / port | ⟨PENDING SPEC — `LIDAR_ESP32_HOST` / `LIDAR_ESP32_PORT`; static IP or discovery⟩ |
| Wi-Fi network | ESP32 firmware SSID/credentials; the Edge PC on the **same subnet** |
| Auth / encryption | ⟨PENDING SPEC — `LIDAR_ESP32_PROTOCOL` / `LIDAR_ESP32_AUTH_TOKEN`⟩ |
| Sequence semantics | ⟨PENDING SPEC — `metadata.sequence_number` wrap → `LIDAR_ESP32_SEQUENCE_MODULUS`⟩ |
| Heartbeat / silence | ⟨PENDING SPEC — does the ESP32 emit explicit "no detections" frames or go silent?⟩ |

Edge-side: a concrete `datasources.esp32.transport.ESP32Transport` subclass registered in
`build_transport()`. Without it, `LIDAR_DATA_SOURCE=hardware` with an unset/placeholder
`LIDAR_ESP32_TRANSPORT` makes `scripts/serve_unity_bridge.py` **exit 2** — by design, never a
simulation fallback.

Network checklist: `ping ⟨ESP32 host⟩` succeeds from the Edge PC; the Edge firewall allows inbound/
outbound on `⟨ESP32 port⟩`; no client-isolation on the AP.

---

## 8. Edge PC → Dashboard / Unity / PostgreSQL

No wiring — all software on the Edge PC / LAN.

| Consumer | Reaches data via | Default |
|---|---|---|
| Dashboard | backend REST + `/ws/live` WebSocket | backend `:8000`, dashboard dev server `:5173` |
| Unity | streaming JSON envelope, TCP | `127.0.0.1:5006` (`LidarInputManager.mode = StructuredJsonTcp`) |
| PostgreSQL | backend SQLAlchemy | `LIDAR_DATABASE_URL`; falls back to `cloud/backend/data/lidar_vision360.db` (SQLite) if unreachable |

---

## 9. Configuration — put every value in `.env`, never in code

Copy `.env.example` to `.env` and fill the hardware blocks. Keys (all currently unset / `None`):

```
LIDAR_DATA_SOURCE=hardware

# ESP32 → Edge
LIDAR_ESP32_TRANSPORT=⟨PENDING SPEC⟩
LIDAR_ESP32_HOST=⟨PENDING SPEC⟩
LIDAR_ESP32_PORT=⟨PENDING SPEC⟩
LIDAR_ESP32_PROTOCOL=⟨PENDING SPEC⟩
LIDAR_ESP32_AUTH_TOKEN=⟨PENDING SPEC, if any⟩
LIDAR_ESP32_SEQUENCE_MODULUS=⟨PENDING SPEC⟩
LIDAR_ESP32_FRAME_STALE_AFTER_S=1.0
LIDAR_ESP32_HEARTBEAT_TIMEOUT_S=6.0

# STM32 raw-serial parse (legacy Edge path only)
LIDAR_STM32_UART_BAUDRATE=⟨PENDING SPEC⟩
LIDAR_STM32_BYTE_ORDER=⟨PENDING SPEC⟩
LIDAR_STM32_FRAME_START_MARKER=⟨PENDING SPEC⟩
LIDAR_STM32_FRAME_END_MARKER=⟨PENDING SPEC⟩
# ... see docs/hardware-validation-procedure.md for the full STM32_* list

# STM32 → ECU CAN
LIDAR_CAN_OUTPUT_ENABLED=true
LIDAR_CAN_OUTPUT_BACKEND=⟨real backend; "mock" for bench⟩
LIDAR_CAN_OUTPUT_BITRATE_BPS=⟨PENDING SPEC⟩
LIDAR_CAN_OUTPUT_INTERFACE=⟨PENDING SPEC⟩
LIDAR_CAN_OUTPUT_CHECKSUM_ALGORITHM=⟨PENDING SPEC⟩
LIDAR_CAN_OUTPUT_SEQUENCE_MODULUS=⟨PENDING SPEC⟩
```

Then follow `docs/hardware-validation-procedure.md` TEST 1 → TEST 9 in order, stopping at the
first failure, and record results into `docs/final-demonstration-package.md`.

---

## 10. Bring-up order (electrical)

1. Grounds bonded; supplies verified at the connector with a meter **before** connecting a device.
2. A3M1 alone: power, confirm motor spin, confirm UART idle level.
3. R121 alone: power, confirm CAN idle (recessive) level.
4. STM32 alone: flash firmware, confirm boot.
5. A3M1 ↔ STM32 UART → TEST 1.
6. R121 ↔ STM32 CAN (termination in place) → TEST 2.
7. STM32 processing sanity → TEST 3.
8. STM32 ↔ ECU CAN (termination, analyser) → TEST 4.
9. STM32 ↔ ESP32 link → TEST 5.
10. ESP32 ↔ Wi-Fi ↔ Edge PC → TEST 6.
11. Edge → Dashboard / Unity / PostgreSQL → TEST 7–9.

Troubleshooting for each stage: `docs/troubleshooting.md`.

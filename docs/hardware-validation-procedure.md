# Hardware Validation Procedure (Phase 6 runbook)

## Status

**BLOCKED — HARDWARE AND SPECIFICATIONS NOT PRESENT.** No physical A3M1 / R121 / STM32 / ESP32 is
connected to this environment, no STM32 firmware exists in the repository, and no hardware
specification (UART baud rate, framing, byte order, scaling, CRC; R121 CAN IDs / DLC / layout;
STM32→ECU CAN IDs / DLC / periods / checksum; ESP32 transport / IP / port / protocol) has been
supplied. See "Prerequisites" below for the exact list. None of the tests in this runbook can be
executed until every prerequisite is met. **Nothing here is invented.**

This document is the procedure to follow **once** the hardware and specs arrive. Each stage names
the exact existing config field / endpoint / command; every hardware value it needs is a
`Settings` field that today is `None` / a labelled placeholder.

## Prerequisites (all currently MISSING)

### Hardware
- [ ] SLAMTEC RPLIDAR **A3M1** unit, wired to the STM32 UART.
- [ ] **R121** radar module, wired to the STM32 CAN.
- [ ] **STM32F103C8T6** board **with firmware** that performs the Phase-2 perception/fusion
      pipeline and emits (a) the STM32→ECU CAN safety messages and (b) the STM32→ESP32 processed
      frames. *No firmware is in this repository (`embedded/stm32/` is a README only).*
- [ ] **ESP32** module bridging STM32 ↔ Wi-Fi.
- [ ] A **Vehicle ECU** (or CAN analyser / bench ECU) on the STM32→ECU bus.
- [ ] The **Edge PC** on the same network as the ESP32.
- [ ] A **USB-CAN adapter** on the Edge PC only if the Edge is to observe the STM32→ECU bus for
      TEST 4 (otherwise a CAN analyser suffices).

*Checked on this machine: `serial.tools.list_ports.comports()` → NONE. No serial/CAN device present.*

### A3M1 → STM32 UART specification (`docs/hardware-integration.md` checklist rows 1–3, 8–9, 11)
- [ ] Actual **UART baud rate** (config placeholder is `115200` — unconfirmed) → `LIDAR_STM32_UART_BAUDRATE`
- [ ] UART **port** convention on the Edge (fixed `COM#`? VID/PID auto-detect?) → `LIDAR_STM32_UART_PORT`
- [ ] **Frame boundaries**: delimited (start/end marker hex bytes) or fixed length →
      `LIDAR_STM32_FRAME_START_MARKER` / `LIDAR_STM32_FRAME_END_MARKER` / `LIDAR_STM32_FRAME_LENGTH_BYTES`
- [ ] **Byte order** (`little` | `big`) → `LIDAR_STM32_BYTE_ORDER`
- [ ] **Message-type field** offset/width and the **LiDAR message-type ID** →
      `LIDAR_STM32_MESSAGE_TYPE_OFFSET` / `_WIDTH` / `LIDAR_STM32_LIDAR_MESSAGE_TYPE`
- [ ] **Sequence-number field** offset/width, wrap modulus →
      `LIDAR_STM32_SEQUENCE_NUMBER_OFFSET` / `_WIDTH` / `LIDAR_STM32_SEQUENCE_MODULUS`
- [ ] **CRC/checksum** algorithm + position → `LIDAR_STM32_CRC_ALGORITHM`
- [ ] **LiDAR payload layout**: angle & distance field offsets/widths/scaling, quality/intensity
      field if present, scan/frame-boundary marker →
      `LIDAR_STM32_LIDAR_DISTANCE_SCALE` / `LIDAR_STM32_LIDAR_ANGLE_SCALE` **plus a concrete
      `datasources.stm32.parsers.LiDARMessageParser` subclass** (the shipped
      `UnconfiguredLiDARParser` raises by design).
- [ ] **Timestamp** format & field → `LIDAR_STM32_TIMESTAMP_FORMAT`

### R121 → STM32 CAN specification (checklist row 10)
- [ ] R121 **CAN ID(s)**, **DLC**, **byte layout** as forwarded by the STM32.
- [ ] Signal **scaling / offsets / signed-ness** for: target ID, range, angle, relative velocity,
      confidence/status, timestamp/sequence.
- [ ] The **radar message-type ID** on the STM32→Edge link → `LIDAR_STM32_RADAR_MESSAGE_TYPE`,
      `LIDAR_STM32_RADAR_RANGE_SCALE`, `LIDAR_STM32_RADAR_VELOCITY_SCALE` **plus a concrete
      `datasources.stm32.parsers.RadarMessageParser` subclass**.
- [ ] Confirmation that radar is fused **on the STM32** (per the target architecture) — the Edge
      does not re-fuse in hardware mode.

### STM32 → ECU CAN specification (`docs/can-output.md` "Hardware parameters still required")
- [ ] CAN **bitrate**, **interface** → `LIDAR_CAN_OUTPUT_BITRATE_BPS` / `LIDAR_CAN_OUTPUT_INTERFACE`
- [ ] Per message: **CAN ID**, standard/extended, **DLC**, **transmission period**.
- [ ] Per signal: **start bit**, **length**, **byte order**, **scale**, **offset**, **signed-ness**.
- [ ] **Checksum** algorithm + position → `LIDAR_CAN_OUTPUT_CHECKSUM_ALGORITHM`
- [ ] Rolling-counter width → `LIDAR_CAN_OUTPUT_SEQUENCE_MODULUS`
- [ ] Confirmation of (or replacement for) the proposed `OBJECT_STATE` / `SAFETY_STATE` /
      `PERCEPTION_HEADER` catalogue and the proposed enum→code maps.
- [ ] A real `can_output.transport.CANTransport` subclass (e.g. `python-can`) if the Edge is to
      transmit; `python-can` is **not** a dependency.

### STM32 → ESP32 → Edge specification (`docs/esp32-integration.md` "Hardware specifications still required")
- [ ] ESP32 **transport** (TCP / UDP / WebSocket / MQTT / raw framed) → `LIDAR_ESP32_TRANSPORT`
- [ ] ESP32 **host / port** (or discovery) → `LIDAR_ESP32_HOST` / `LIDAR_ESP32_PORT`
- [ ] **Message framing** (length prefix / delimiter / one datagram per frame / WS message).
- [ ] **Payload encoding** (JSON / CBOR / protobuf / fixed binary). If not JSON, a concrete
      `datasources.stm32.processed.ProcessedFrameDeserializer` subclass.
- [ ] Per-message **CRC** + **auth/encryption**, if any → `LIDAR_ESP32_PROTOCOL` / `LIDAR_ESP32_AUTH_TOKEN`
- [ ] `metadata.timestamp` epoch/units; `metadata.sequence_number` semantics + wrap →
      `LIDAR_ESP32_SEQUENCE_MODULUS`
- [ ] Heartbeat rate; whether the ESP32 emits explicit "no detections" frames or goes silent.

Until **every** box above is ticked and the config filled in (in `.env`, never in code), the
procedure below cannot run and no result may be claimed.

---

## Diagnostic procedure — run in order, STOP on the first failure

Each test lists **REQUIRES**, **STEPS**, **PASS CRITERIA**, and **ON FAILURE**. Do not start
TEST *n+1* until TEST *n* passes.

### TEST 1 — A3M1 → STM32 (UART)

- **REQUIRES:** A3M1 wired to STM32 UART; STM32 firmware running; A3M1 UART baud/format spec.
- **STEPS:**
  1. Power the A3M1; confirm the motor spins and the firmware's LiDAR-RX status flag is set.
  2. On the STM32 (debugger / firmware log), inspect the raw A3M1 frames: confirm distance,
     angle, quality/intensity (if the A3M1 model provides it), and scan-boundary markers decode
     to sane values against a known static scene (e.g. a wall at a measured distance).
  3. Confirm the STM32's per-scan point count and scan rate.
- **PASS CRITERIA:** STM32 reports a stable stream of `(angle, distance[, quality])` covering a
  full sweep, updating at the A3M1's rated scan rate, with distances matching a tape-measured
  reference to within the sensor's spec.
- **ON FAILURE:** STOP. Record: baud rate tried, wiring, firmware LiDAR-RX error counters.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 2 — R121 → STM32 (CAN)

- **REQUIRES:** R121 wired to STM32 CAN; R121 CAN spec (IDs / DLC / layout / scaling).
- **STEPS:**
  1. Confirm the CAN bus terminates correctly and the STM32 CAN peripheral is at the R121's bitrate.
  2. With a CAN analyser on the same bus, capture R121 frames; confirm the STM32 decodes target
     ID, range, angle, relative velocity, confidence/status per the supplied byte layout &
     scaling — compare against the analyser's own DBC decode.
  3. Move a metal target radially; confirm range and relative-velocity signs/magnitudes track it.
- **PASS CRITERIA:** STM32-decoded R121 targets match the analyser decode field-for-field; range
  matches a measured reference; velocity sign convention confirmed.
- **ON FAILURE:** STOP. Record: bitrate, analyser capture, decode mismatch per field.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 3 — STM32 processing / fusion

- **REQUIRES:** TESTs 1–2 pass; STM32 firmware perception+fusion stage.
- **STEPS:**
  1. With a single static obstacle in view of both sensors, confirm the STM32 pipeline produces:
     one detected+classified+tracked object with a stable `track_id`, a plausible position/
     distance, `sensor_source = fused` (or `lidar`/`radar` as appropriate), a per-object TTC
     (null when not closing), per-object risk, and the frame-level clearance + overall risk.
  2. Confirm the processed frame carries every field the Phase-2 contract defines
     (`models.stm32_processed.STM32ProcessedFrame`) with `active_object_count == len(objects)`.
- **PASS CRITERIA:** The STM32's processed frame validates against
  `datasources.stm32.processed.validate_processed_frame` (feed a captured frame to it offline),
  and the object count / risk / clearance match the physical scene.
- **ON FAILURE:** STOP. Record: which stage (detection/tracking/TTC/clearance/risk) is wrong.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 4 — STM32 → ECU (CAN)

- **REQUIRES:** TEST 3 passes; STM32→ECU CAN spec (IDs / DLC / periods / scaling / checksum);
  ECU or CAN analyser on the bus.
- **STEPS:**
  1. Fill the real values into a `can_output.CANOutputConfig` (per-message `can_id`,
     `extended_id`, `dlc`, `cycle_time_ms`, every signal's `start_bit`/`length_bits`/
     `byte_order`/`scaling`, `checksum_algorithm`) — configuration only.
  2. With a CAN analyser, capture the STM32's safety frames; decode them with the DBC and confirm
     object id/type/distance/relative-velocity/TTC/clearance/risk/confidence match the STM32's
     own processed state and the physical scene.
  3. Confirm transmission period, DLC, and checksum against the spec.
  4. *(Optional Edge cross-check)* run `can_output` with the same filled-in config and a real
     transport; compare the Edge-encoded bytes to the analyser capture byte-for-byte.
- **PASS CRITERIA:** analyser decode == STM32 processed state, at the specified rate/DLC, checksum valid.
- **ON FAILURE:** STOP. Record: CAN ID, DLC, raw bytes, decode diff, checksum result.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 5 — STM32 → ESP32

- **REQUIRES:** TEST 3 passes; STM32↔ESP32 link spec.
- **STEPS:**
  1. On the ESP32, confirm it receives complete processed frames from the STM32 (framing intact,
     CRC valid if used) and forwards them onto Wi-Fi without modifying the payload semantics.
  2. Confirm sequence numbers are monotonic and the ESP32 does not buffer/reorder.
- **PASS CRITERIA:** every STM32 processed frame appears once on the ESP32's Wi-Fi output, in order.
- **ON FAILURE:** STOP. Record: ESP32 RX error counters, drop counters.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 6 — ESP32 → Edge PC (`ESP32Source`)

- **REQUIRES:** TEST 5 passes; ESP32 transport/host/port/encoding spec; a concrete
  `datasources.stm32.processed.ProcessedFrameDeserializer` (if non-JSON) and, if the transport is
  not `mock`, a concrete `datasources.esp32.transport.ESP32Transport` registered in
  `build_transport()`.
- **STEPS:**
  1. Set `LIDAR_DATA_SOURCE=hardware` and the real `LIDAR_ESP32_*` values in `.env`.
  2. Run `python scripts/serve_unity_bridge.py --rate <hz>` — it dispatches to `run_esp32_edge`.
  3. Watch the `[ESP32]` logs: `connected`, `frame received` (frame_id / seq / objects / risk),
     no `sequence gap` / `stale` / `rejected` warnings under nominal conditions.
  4. Inspect `ESP32SourceStatus` (via `GET /debug/stream-status`): `frames_received` rising,
     `frames_rejected` 0, `dropped_frames` 0, `frame_age_s` small, `state = CONNECTED`.
- **PASS CRITERIA:** `ESP32Source` state `CONNECTED`, sequence continuous, frame age within
  `esp32_frame_stale_after_s`, reconnect not triggered.
- **ON FAILURE:** STOP. Record: `ESP32SourceStatus` snapshot, `[ESP32]` log excerpt.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 7 — Edge → Dashboard

- **REQUIRES:** TEST 6 passes; backend + dashboard running (`uvicorn backend.main:app …`, `npm run dev`).
- **STEPS:**
  1. `Invoke-RestMethod http://localhost:8000/debug/live-frame` — confirm `source_id ==
     "stm32_hardware"`, `config.data_source == "hardware"`, `timestamp` changing, `sequence_number`
     increasing, `objects` / `tracked_objects` reflecting the physical scene, `risk` / `clearance`
     / per-object `ttc` present and real.
  2. Open `http://localhost:5173` — confirm `Mode: HARDWARE`, `Source ID: stm32_hardware`,
     `ESP32: LIVE`, and the Detected Objects / Safety / Tracking / Environment Map / Event Timeline
     panels update within one frame (no page refresh). Confirm Scan Rate is measured, Frame Age
     small, latencies real.
  3. Cross-check: the `/debug/live-frame` values equal the Dashboard values.
- **PASS CRITERIA:** Dashboard shows hardware source + all fields from the STM32 processed data,
  updating in real time; `/debug/live-frame` == Dashboard.
- **ON FAILURE:** STOP. Record: `/debug/live-frame` body, `/debug/stream-status` body, screenshot.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 8 — Edge → Unity

- **REQUIRES:** TEST 6 passes; Unity Editor with `unity/LiDARVision360`,
  `LidarInputManager.mode = StructuredJsonTcp`.
- **STEPS:**
  1. Press Play; Unity connects to `127.0.0.1:5006` (the same stream the backend consumes).
  2. Compare, for the same live frame: Unity track id == Dashboard track id; object type;
     distance; TTC; clearance; overall risk. Confirm Unity's HUD "SYSTEM" line shows Connected and
     the session id matches.
  3. Confirm Unity runs **no** perception (documented — no clustering/filter/threshold in
     `Assets/Scripts/`); it only visualises.
- **PASS CRITERIA:** every compared field is identical between Dashboard and Unity for the same
  `session_id` + `sequence_number`.
- **ON FAILURE:** STOP. Record: side-by-side field table, Unity Console excerpt.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

### TEST 9 — Edge → PostgreSQL

- **REQUIRES:** TEST 6 passes; PostgreSQL reachable (or the SQLite fallback).
- **STEPS:**
  1. Let a hardware session run for a few minutes with obstacles moving.
  2. Query `sessions` — the new row has `source_id = "stm32_hardware"`, an `edge_session_id`
     (the Edge-minted uuid), `started_at`/`ended_at`, `frame_count`.
  3. Query `tracks`, `collision_events`, `clearance_events`, `ttc_events`, `sensor_events` for
     that `session_id` — confirm each row carries `session_id` + `source_id = "stm32_hardware"` +
     `timestamp` + the relevant track id / object type / distance / velocity / TTC / clearance /
     risk, and that there is **one** `tracks` row per `track_id` (not per frame) and **one**
     event row per transition.
  4. Confirm a simulation session in the same DB is distinguishable by `source_id`
     (`"simulated:<scenario>"`) — hardware and simulation never mixed without source/session id.
- **PASS CRITERIA:** hardware rows present, correctly source-tagged, no per-frame duplication.
- **ON FAILURE:** STOP. Record: `SELECT` outputs, row counts.
- **STATUS: NOT TESTED — HARDWARE/CONFIGURATION REQUIRED.**

---

## Real-time controlled test (Phase 6 §12) — after TESTs 1–9 pass

Record actual measurements at each step (do not estimate):

| Step | Expected observable | Dashboard | Unity | `/debug/live-frame` |
|---|---|---|---|---|
| No obstacle | `objects = 0`, `tracks = 0`, `risk = safe` | | | |
| Obstacle placed in front | object appears, `track_created` event, distance ≈ measured | | | |
| Obstacle moved closer | distance decreases, TTC appears/decreases (if closing), clearance decreases, risk escalates | | | |
| Obstacle moved away | distance increases, TTC → N/A, clearance increases, risk de-escalates | | | |

Also record: measured update rate (`performance_metrics.measured_scan_rate_hz`), end-to-end
latency (`/debug/stream-status.sensor_ingestion_latency_ms`), dropped frames
(`duplicate_or_out_of_order_dropped` + `ESP32SourceStatus.dropped_frames`).

## Failure tests (Phase 6 §13) — after the real-time test

For each: induce the fault, record the reported state, confirm **no** synthetic/simulation data
appears.

| Fault | Expected reported state |
|---|---|
| A3M1 unplugged | STM32 `sensor_status.lidar.connected = false` / `ok = false`; propagated to `LiveState.sensor_status`; Dashboard LiDAR badge red |
| R121 unplugged | STM32 `sensor_status.radar` reports disconnected; `fusion_active = false`; objects fall back to `lidar` source |
| STM32 powered off | ESP32 stops receiving → `ESP32Source` → `STALE` then `DISCONNECTED`; `HARDWARE_DATA_UNAVAILABLE` on the wire; Dashboard banner; **no sim fallback** |
| ESP32 unplugged / Wi-Fi lost | `ESP32Source` transport error → `RECONNECTING` with backoff; Dashboard `ESP32: UNAVAILABLE` + reason; timeline "Connection lost" |
| Stale frame (STM32 hangs) | `ESP32Source` → `STALE` after `esp32_frame_stale_after_s`; `poll()` returns `None`; last values shown but marked `STALE DATA`, not `LIVE` |
| Packet loss | `ESP32Source.sequence_gaps` / `dropped_frames` increment; newest frame still delivered; `/debug/stream-status.frames_dropped` rises |
| Invalid frame | `ESP32Source.frames_rejected` increments; `reason = "invalid frame"` / `"protocol error"`; frame not applied |
| CAN comms failure (STM32→ECU) | `STM32SystemStatus.can_tx_ok = false` in the processed frame; if the Edge `can_output` model is attached, its `CANOutputHealth.state = BUS_OFF` with recovery; **the ESP32 path is unaffected** |

**Every fault must be reported as a real state, never masked by simulation data.**

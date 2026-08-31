# Final Demonstration Package (Phase 9)

**Purpose:** everything needed to run the final **physical hardware demonstration** of
LiDAR-Vision360, plus the evidence slots to fill *during* that demonstration.

**Current state — read first.** No physical A3M1 / R121 / STM32 / ESP32 / Vehicle ECU is
connected to this environment, no STM32 firmware is in the repository, and no hardware / DBC
specification has been supplied. Therefore **every "PHYSICAL HARDWARE" row below is `NOT
PERFORMED`**, and this document is the *procedure and evidence template* to execute once the rig
and specifications exist. The "SIMULATION" and "MOCK HARDWARE" numbers quoted are **real
measurements taken in software this session** (see `docs/final-demonstration.md`, the README
"Current Status", and the Phase 7/8 reports) — they are not physical results and are never
presented as such.

Related docs: `docs/hardware-validation-procedure.md` (stage-by-stage bring-up + the full
per-field specification checklist), `docs/hardware-setup-guide.md` (wiring/power),
`docs/final-demonstration.md` (16-step software demo), `docs/troubleshooting.md`,
`docs/esp32-integration.md`, `docs/can-output.md`, `docs/stm32-processed-contract.md`.

---

## 0. Result categories — never combined

| Tag | Meaning |
|---|---|
| **SW-UNIT** | `pytest` / `tsc` / Vitest against mocks & fixtures |
| **SIM** | live backend + `serve_unity_bridge.py` in `DATA_SOURCE=simulation` (10 scenarios) |
| **MOCK** | live backend + bridge in `DATA_SOURCE=hardware` + `ESP32_TRANSPORT=mock` (scripted SIMULATED ESP32 transport) |
| **PHYS** | real A3M1/R121/STM32/ESP32/ECU — **NOT PERFORMED** |

---

## 1. FINAL HARDWARE CHECKLIST (per link)

Run **top to bottom; do not continue past a failed link.** Each row: what to check, how,
pass criterion, and where the evidence goes. Full per-signal specification list:
`docs/hardware-validation-procedure.md` §Prerequisites.

| # | Link | Check | Pass criterion | Spec needed (all currently MISSING) | PHYS result | Evidence |
|---|---|---|---|---|---|---|
| 1 | **A3M1 → STM32** (UART) | On the STM32 (debugger/firmware log), raw A3M1 frames decode to sane `(angle, distance[, quality])`; full sweep; scan-boundary markers; rate = A3M1 rating. Compare a wall to a tape measure. | Stable full-sweep stream at rated Hz; distances within sensor spec of the reference | UART baud, framing (start/end marker or fixed length), byte order, LiDAR payload offsets/widths, distance/angle scaling, quality field, timestamp format | ☐ NOT PERFORMED | _(STM32 log / scope capture)_ |
| 2 | **R121 → STM32** (CAN) | STM32 CAN peripheral at the R121 bitrate; a CAN analyser on the bus; STM32 decode of target ID / range / angle / relative velocity / confidence/status matches the analyser DBC decode. Move a metal target radially. | STM32 decode == analyser decode field-for-field; range matches reference; velocity sign confirmed | R121 CAN ID(s), DLC, byte layout, scaling/offset/sign per signal, bitrate | ☐ NOT PERFORMED | _(analyser .blf/.asc + STM32 log)_ |
| 3 | **STM32 → ECU** (CAN) | Fill the real DBC into `can_output.CANOutputConfig` (per-message `can_id`/`extended_id`/`dlc`/`cycle_time_ms`, per-signal `start_bit`/`length_bits`/`byte_order`/`scaling`, `checksum_algorithm`). Capture the STM32's safety frames with an analyser; DBC-decode; compare to the STM32's own processed state and the physical scene. Check period, DLC, checksum. | Analyser decode == STM32 processed state; correct rate/DLC; checksum valid | CAN bitrate, interface, per-message ID/DLC/period, per-signal layout/scaling/sign, checksum algorithm + position, rolling-counter width | ☐ NOT PERFORMED | _(analyser capture; `CANOutputHealth` if Edge cross-check used)_ |
| 4 | **STM32 → ESP32** | ESP32 receives complete processed frames (framing intact, CRC valid if used) and forwards them onto Wi-Fi without changing payload semantics; sequence monotonic; no reorder/buffer. | Every STM32 processed frame appears once on the ESP32 output, in order | STM32↔ESP32 link framing + encoding + CRC | ☐ NOT PERFORMED | _(ESP32 RX/drop counters; serial log)_ |
| 5 | **ESP32 → Edge PC** (`ESP32Source`) | `.env`: `LIDAR_DATA_SOURCE=hardware`, real `LIDAR_ESP32_TRANSPORT`/`HOST`/`PORT`; a real `ESP32Transport` subclass registered in `build_transport()` and (if non-JSON) a `ProcessedFrameDeserializer`. Run `python scripts/serve_unity_bridge.py --rate <hz>`. Watch `[ESP32]` logs and `GET /debug/stream-status`. | `ESP32Source` state `CONNECTED`; sequence continuous; `frames_rejected=0`, `dropped_frames=0`; `frame_age_s` < `esp32_frame_stale_after_s` | ESP32 transport name/host/port/encoding/auth; sequence semantics + wrap | ☐ NOT PERFORMED | _(`/debug/stream-status` JSON; `[ESP32]` log excerpt)_ |
| 6 | **Edge → LiveState** | `GET /debug/live-frame` shows `source_id=stm32_hardware`, `config.data_source=hardware`, `timestamp` changing, `sequence_number` increasing, `objects`/`tracked_objects`/`risk`/`clearance`/per-object `ttc` real for the scene. **`null` when the link is down — no synthetic frame.** | Live frame reflects the physical scene; matches `/debug/stream-status` | — (uses whatever Steps 1–5 delivered) | ☐ NOT PERFORMED | _(`/debug/live-frame` JSON, 3 samples showing change)_ |
| 7 | **Edge → Dashboard** | Backend + `npm run dev` running. `http://localhost:5173`: `Mode: HARDWARE`, `Source ID: stm32_hardware`, `ESP32: LIVE`; Detected Objects / Safety / Tracking / Environment Map / Event Timeline update within one frame, no refresh; Scan Rate measured, Frame Age small, latencies real. | Dashboard == `/debug/live-frame`; updates in real time | — | ☐ NOT PERFORMED | _(screen recording + `/debug/stream-status` at capture time)_ |
| 8 | **Edge → Unity** | Unity Editor, `LidarInputManager.mode = StructuredJsonTcp`, Play. For the same live frame compare: track ID, object type, position, distance, TTC, clearance, overall risk — Dashboard vs Unity. Confirm Unity runs no perception. | Every compared field identical for the same `session_id`+`sequence_number` | — | ☐ NOT PERFORMED | _(side-by-side screenshot + field table)_ |
| 9 | **Edge → PostgreSQL** | Run a hardware session a few minutes with obstacles moving. Query `sessions` (`source_id=stm32_hardware`, `edge_session_id`, `frame_count`), `tracks` (one row per `track_id`), `collision_events`/`clearance_events`/`ttc_events` (one per transition, all `source_id=stm32_hardware`). Confirm a simulation session is distinguishable by `source_id`. | Hardware rows present, source-tagged, no per-frame duplication | — | ☐ NOT PERFORMED | _(`SELECT` outputs / `GET /api/sessions`, `/api/collision-events`)_ |

---

## 2. PRE-DEMO CHECKLIST

Tick every box **before** starting Section 3.

**Power & sensors**
- ☐ A3M1 supplied at its rated voltage/current; motor spins freely.
- ☐ R121 supplied at its rated voltage; fault line (if any) clear.
- ☐ STM32 board powered; status LED nominal; firmware flashed (the perception + dual-output build).
- ☐ ESP32 powered; on the demo Wi-Fi network (SSID/creds set in firmware).
- ☐ Common ground between A3M1, R121, STM32, ESP32.

**Buses / links**
- ☐ A3M1 ↔ STM32 UART wired (TX↔RX, RX↔TX, GND); level shifting correct if needed.
- ☐ R121 ↔ STM32 CAN wired (CANH/CANL); **120 Ω termination at both ends**.
- ☐ STM32 ↔ ECU CAN wired; termination correct; a CAN analyser or bench ECU on the bus.
- ☐ STM32 ↔ ESP32 link wired per the firmware (UART/SPI/…); baud/mode matched.

**Edge PC**
- ☐ On the same subnet as the ESP32; can `ping` the ESP32 host; firewall allows the ESP32 port.
- ☐ Repo checked out; `.venv` created; `pip install -e "./perception[dev]" "./simulator[dev]" "./cloud/backend[dev]"`.
- ☐ `cloud/dashboard` deps installed (`npm install`).
- ☐ **`.env` filled** with the real `LIDAR_DATA_SOURCE=hardware` + `LIDAR_ESP32_*` + `LIDAR_STM32_*` + `LIDAR_CAN_OUTPUT_*` values from the spec (see `docs/hardware-validation-procedure.md`). No value guessed.
- ☐ Concrete driver subclasses present: `LiDARMessageParser`, `RadarMessageParser`, `ESP32Transport` (+ `ProcessedFrameDeserializer` if non-JSON), `CANTransport`; registered where the docs say.
- ☐ `python -m pytest perception/tests simulator/tests -q` and `cd cloud/dashboard && npm test` pass (regression baseline).

**Services**
- ☐ Ports 8000 (backend) and 5006 (streaming JSON) free — `scripts/stop_demo.ps1` first if unsure.
- ☐ PostgreSQL reachable at `LIDAR_DATABASE_URL`, **or** accept the automatic SQLite fallback (`cloud/backend/data/lidar_vision360.db`) and note it in the evidence.
- ☐ Backend starts clean (`[BACKEND] Ready on 0.0.0.0:8000`).
- ☐ Dashboard dev server serving `http://localhost:5173`.
- ☐ Unity project opens; Built-in Render Pipeline; `LidarInputManager.mode = StructuredJsonTcp`.

---

## 3. STEP-BY-STEP DEMONSTRATION

> Do not advance a step until the previous one passes. Record every measurement — never estimate.

| STEP | Action | Verify | PHYS result |
|---|---|---|---|
| 1 | Power the sensors and STM32 | A3M1 motor spins; STM32 firmware boots; ESP32 associates to Wi-Fi | ☐ NOT PERFORMED |
| 2 | Verify A3M1 → STM32 | STM32 shows a full-sweep `(angle, distance[, quality])` stream at the rated rate; a tape-measured wall matches | ☐ NOT PERFORMED |
| 3 | Verify R121 → STM32 | STM32 decodes R121 targets (id/range/angle/velocity/confidence) matching a CAN-analyser DBC decode | ☐ NOT PERFORMED |
| 4 | Verify STM32 processing | STM32 produces one detected+classified+tracked object with a stable `track_id`, TTC (null if not closing), per-object risk, frame-level clearance + overall risk; frame validates against `datasources.stm32.processed.validate_processed_frame` (offline) | ☐ NOT PERFORMED |
| 5 | Verify STM32 → ECU CAN | CAN analyser: the configured safety messages appear at the configured CAN ID / DLC / period; DBC-decoded values == STM32 processed state; checksum valid | ☐ NOT PERFORMED |
| 6 | Verify STM32 → ESP32 | ESP32 log/counters: every processed frame received once, framing/CRC valid, sequence monotonic | ☐ NOT PERFORMED |
| 7 | Verify ESP32 → Edge PC | Edge: `python scripts/serve_unity_bridge.py --rate <hz>`; `[ESP32] connected`; frames arriving; no `sequence gap` / `stale` / `rejected` under nominal conditions | ☐ NOT PERFORMED |
| 8 | Verify `ESP32Source` receives data | `GET /debug/stream-status`: `edge_status=connected`, `frames_received` rising, `frames_dropped=0`, `frame_age_ms` small, `last_error_code=null` | ☐ NOT PERFORMED |
| 9 | Verify `LiveState` updates | `GET /debug/live-frame` (3×, spaced): `sequence_number` increases, `timestamp` changes, `objects`/`risk`/`clearance` reflect the scene; `source_id=stm32_hardware` | ☐ NOT PERFORMED |
| 10 | Open Dashboard | `http://localhost:5173`: `Mode HARDWARE`, `ESP32 LIVE`, all panels live, no refresh | ☐ NOT PERFORMED |
| 11 | Open Unity | Press Play; Unity connects to `:5006`; in-world objects/labels appear; HUD "SYSTEM: Connected" | ☐ NOT PERFORMED |
| 12 | Verify PostgreSQL logging | `GET /api/sessions` shows the hardware session; `GET /api/collision-events` / `/api/clearance-events` accumulate | ☐ NOT PERFORMED |

---

## 4. CONTROLLED CONDITION TESTS

Perform each with a real object; record what the Dashboard **and** Unity **and**
`/debug/live-frame` show. (Simulation equivalents already verified — see §8.)

| Test | Setup | Expected | PHYS observation |
|---|---|---|---|
| **1 — Empty** | Clear field of view | `objects=0`, `tracks=0`, risk **SAFE**, `NO ACTIVE OBJECTS` in table, large clearances | ☐ NOT PERFORMED |
| **2 — Static object** | One object ~3–5 m ahead, stationary | 1 object; a classification; a `track_id`; a real distance; clearance panel shows the reduced direction | ☐ NOT PERFORMED |
| **3 — Moving object** | Move the object laterally across the FOV | same `track_id` persists; position/distance change frame-to-frame; velocity updates once tracking is confident | ☐ NOT PERFORMED |
| **4 — Approaching object** | Move an object steadily toward the sensor | distance **decreases**; TTC becomes finite and **decreases**; front clearance **decreases**; risk escalates SAFE→WARNING→CRITICAL | ☐ NOT PERFORMED |
| **5 — Multiple objects** | 2–3 objects at different positions | ≥2 objects; **distinct** `track_id`s; per-object type/distance/velocity/TTC/risk in the table | ☐ NOT PERFORMED |

---

## 5. DASHBOARD EVIDENCE (fill during the demo)

| Field | Where on the Dashboard | Test 1 | Test 2 | Test 3 | Test 4 | Test 5 |
|---|---|---|---|---|---|---|
| Connected hardware | System Status → `ESP32: LIVE`, `Edge: CONNECTED` | | | | | |
| Real-time frame age | Live Data → Frame Age | | | | | |
| Update rate | Live Data → Scan Rate | | | | | |
| Object count | Objects → Object Count | | | | | |
| Tracked objects | Detected Objects table row count | | | | | |
| Classification | Detected Objects → Classification | | | | | |
| Distance | Detected Objects → Distance | | | | | |
| Velocity | Detected Objects → Velocity | | | | | |
| TTC | Safety → Min TTC + table TTC | | | | | |
| Clearance | Safety → Front/Rear/Left/Right + Min | | | | | |
| SAFE/WARNING/CRITICAL | Safety → Risk tile | | | | | |

Attach: screen recording of each test; a `GET /debug/stream-status` JSON captured at the same
instant as each screenshot.

---

## 6. UNITY EVIDENCE (fill during the demo)

For one representative live frame in each of Tests 2–5, record **Dashboard value vs Unity value**:

| Field | Dashboard | Unity | Match? |
|---|---|---|---|
| Track ID | | | ☐ |
| Object type | | | ☐ |
| Position (x, y) | | | ☐ |
| Distance | | | ☐ |
| TTC | | | ☐ |
| Clearance (min + direction) | | | ☐ |
| Risk (overall) | | | ☐ |
| `session_id` + `sequence_number` | | | ☐ |

Unity must run **no** independent perception (verified structurally: no clustering/filter/
threshold code under `unity/.../Assets/Scripts/`). Attach a side-by-side screenshot.

---

## 7. DATABASE EVIDENCE (fill during the demo)

Run after ~2–3 minutes of live hardware with moving obstacles:

```powershell
Invoke-RestMethod "http://localhost:8000/api/sessions?limit=5"
Invoke-RestMethod "http://localhost:8000/api/tracks"
Invoke-RestMethod "http://localhost:8000/api/collision-events?limit=20"
Invoke-RestMethod "http://localhost:8000/api/clearance-events?limit=20"
Invoke-RestMethod "http://localhost:8000/api/ttc-events?limit=20"
```

| Check | Expected | PHYS result |
|---|---|---|
| `sessions` row | `source_id = stm32_hardware`, non-null `edge_session_id`, rising `frame_count`, `status` `active`→`ended` | ☐ NOT PERFORMED |
| `tracks` | one row per unique `track_id` this session (never one per frame); `source_id = stm32_hardware`; `classification`/`sensor_source` populated | ☐ NOT PERFORMED |
| `collision_events` / `clearance_events` / `ttc_events` | one row per transition; each carries `session_id` + `source_id` + `timestamp` + track/distance/TTC/clearance/risk | ☐ NOT PERFORMED |
| Simulation vs hardware | a `simulated:<scenario>` session in the same DB is separable by `source_id` | ☐ NOT PERFORMED |
| Historical retrieval | the `GET /api/*` endpoints above return the stored rows | ☐ NOT PERFORMED |

---

## 8. PERFORMANCE EVIDENCE

**Measured this session (SIM and MOCK only — not physical):**

| Metric | SIM (10 scenarios, live) | MOCK HARDWARE (live) | PHYS |
|---|---|---|---|
| Sensor / stream update rate (`measured_scan_rate_hz`) | 9.93 – 9.99 Hz, mean **9.96** (target 10) | 9.85 – 9.95 Hz, mean **9.91** | ☐ NOT MEASURED |
| STM32 processing time | n/a (no STM32) | model field only, not a real device | ☐ NOT MEASURED |
| ESP32 transmission rate | n/a | = stream rate (mock transport, 0 gaps) | ☐ NOT MEASURED |
| Edge reception rate | = stream rate, 0 gaps | = stream rate, 0 gaps | ☐ NOT MEASURED |
| Wire latency (`latency_ms`) | 0.0 – 9.5 ms, mean **3.7** | 1.0 – 5.0 ms, mean **2.6** | ☐ NOT MEASURED |
| End-to-end sensor→backend (`sensor_ingestion_latency_ms`) | 32.7 – 115.3 ms, mean **63.7** (incl. full pipeline) | 3.0 – 27.9 ms, mean **8.1** (adapter only) | ☐ NOT MEASURED |
| Frame age at sampling | small; > 6.5 s → STALE after link loss | small | ☐ NOT MEASURED |
| Dropped / duplicate packets | **0** across all 10 scenarios | **0** | ☐ NOT MEASURED |
| Sequence gaps / packet loss | 0 | 0 | ☐ NOT MEASURED |
| Active tracks (`/debug/stream-status.tracks`) | 0 (`01_empty`) → 5 (`05_multiple`) → 13 (`09_noisy` roster) | 1 | ☐ NOT MEASURED |
| Detection count (objects/frame) | 0 → 5 as scene dictates | 1 | ☐ NOT MEASURED |

**PHYS column instructions:** capture `GET /debug/stream-status` once per second for ≥ 60 s
during a stable scene and once during each controlled test; report min / mean / max for each row
and the raw JSON as evidence. Also record the A3M1 rated scan rate and the R121 rated update
rate from their datasheets and compare to what the STM32 actually ingests.

---

## 9. FAILURE DEMONSTRATION

Only if safe and practical. For each: induce the fault, record the reported state, confirm **no
synthetic/simulation data appears**.

| Fault | Expected reported state | SW/MOCK verified | PHYS result |
|---|---|---|---|
| A3M1 unplugged | STM32 `sensor_status.lidar.connected=false`/`ok=false` → `LiveState.sensor_status`; Dashboard LiDAR badge red | `test_esp32_live_state_adapter` (sensor_status mapping) | ☐ NOT PERFORMED |
| R121 unplugged | STM32 `sensor_status.radar` disconnected; `fusion_active=false`; objects fall back to `lidar` source | SW-UNIT (adapter) | ☐ NOT PERFORMED |
| STM32 powered off | ESP32 stops → `ESP32Source` → `STALE` then `DISCONNECTED`; `HARDWARE_DATA_UNAVAILABLE` on the wire; Dashboard banner; `/debug/live-frame` → `null`; **no sim fallback** | **MOCK verified** (bridge kill → `edge_status=connecting`, live-frame `null`, `no_synthetic_switch=true`) | ☐ NOT PERFORMED |
| ESP32 unplugged / Wi-Fi lost | `ESP32Source` transport error → `RECONNECTING` with backoff; Dashboard `ESP32: UNAVAILABLE` + reason; timeline "Connection lost" | SW-UNIT `test_esp32_source` (39); MOCK recovery test | ☐ NOT PERFORMED |
| Stale frame (STM32 hangs) | `ESP32Source` → `STALE` after `esp32_frame_stale_after_s`; `poll()` → `None`; last values shown but `System: STALE DATA` (not LIVE) | SW-UNIT `test_esp32_source`; MOCK | ☐ NOT PERFORMED |
| Packet loss | `ESP32Source.sequence_gaps` / `dropped_frames` increment; newest frame still delivered | SW-UNIT `test_esp32_source` | ☐ NOT PERFORMED |
| CAN comms failure (STM32→ECU) | `STM32SystemStatus.can_tx_ok=false` in the processed frame; Edge `can_output` model (if attached) → `CANOutputHealth.state=BUS_OFF` + recovery; **ESP32 path unaffected** | SW-UNIT `test_can_output_controller` (17) | ☐ NOT PERFORMED |
| Config missing (`LIDAR_ESP32_TRANSPORT` unset) | bridge exits **2**; log: *"NEVER falls back to simulation data"* + names the missing var | **MOCK verified** (exit 2) | ☐ NOT PERFORMED |

---

## 10. FINAL EVIDENCE TABLE

| Component | Test | SW-UNIT | SIM | MOCK | **PHYS** | Evidence (PHYS) |
|---|---|---|---|---|---|---|
| **A3M1** | STM32 reception | — | — | — | **NOT PERFORMED** | _(STM32 log / scope)_ |
| **R121** | STM32 reception | — | — | — | **NOT PERFORMED** | _(CAN analyser + STM32 log)_ |
| **STM32** | Processing / fusion | contract validation 61 tests | — | scripted frames validate | **NOT PERFORMED** | _(processed-frame capture)_ |
| **CAN** | ECU output | `can_output` 73 tests | — | mock transport frames | **NOT PERFORMED** | _(CAN analyser capture)_ |
| **ESP32** | Data reception | `esp32` 70 tests | — | scripted transport → `CONNECTED` | **NOT PERFORMED** | _(`/debug/stream-status`, `[ESP32]` log)_ |
| **Edge** | Data reception / LiveState | 1056 perception tests | 10/10 scenarios, `/debug/*` consistent | `source_id=stm32_hardware`, seq↑, 0 dropped | **NOT PERFORMED** | _(`/debug/live-frame` ×3)_ |
| **Dashboard** | Visualization | 53 Vitest, `tsc` clean | 10/10 render correct | `Mode HARDWARE`, all panels live | **NOT PERFORMED** | _(screen recording + `/debug/stream-status`)_ |
| **Unity** | Visualization | structural consistency 4 tests | — | same wire message | **NOT PERFORMED** | _(side-by-side screenshot + field table)_ |
| **PostgreSQL** | Storage | 41 backend tests | events persisted per scenario | session row + 1 track row/track + events, source-tagged | **NOT PERFORMED** | _(`SELECT` / `GET /api/*` outputs)_ |

---

## 11. Deliverables index (this package)

| Deliverable | Location |
|---|---|
| Final README (verified architecture) | `README.md` |
| Hardware setup guide (wiring/power) | `docs/hardware-setup-guide.md` |
| Software startup guide | `docs/final-demonstration.md` STEPS 1–5, and README "Running the Project" |
| Hardware validation checklist (per-field spec + stage runbook) | `docs/hardware-validation-procedure.md` |
| Demonstration procedure (this document + the 16-step) | `docs/final-demonstration-package.md`, `docs/final-demonstration.md` |
| Troubleshooting guide | `docs/troubleshooting.md` |
| Performance measurement table | §8 above |
| Final test report | Phase 7 & Phase 8 reports; §10 above; README "Current Status" |
| Subsystem design docs | `docs/stm32-processed-contract.md`, `docs/esp32-integration.md`, `docs/can-output.md`, `docs/hardware-integration.md`, `docs/architecture.md`, `docs/communication.md` |

---

## FINAL DEMO STATUS

Legend: **READY** = the software seam is implemented, unit/SIM/MOCK-tested, and needs only its
specification + a physical device to run. **NOT READY** = a physical device and/or its
specification is absent, so the demonstration step cannot execute.

| Item | Status | Why |
|---|---|---|
| **A3M1** | **NOT READY** | no unit connected; no UART/framing/scaling spec; no `LiDARMessageParser` implementation |
| **R121** | **NOT READY** | no unit connected; no CAN ID/DLC/layout/scaling spec; no `RadarMessageParser` implementation |
| **STM32** | **NOT READY** | no board with firmware in this repo (`embedded/stm32/` is a README); Edge-side seams (`datasources.stm32`, `models.stm32_processed`) are READY |
| **CAN → ECU** | **NOT READY** | no bus, no ECU, no bitrate/IDs/DLC/DBC; `can_output` software model is READY (configurable, mock-tested) |
| **ESP32** | **NOT READY** | no module; no transport/host/port/encoding spec; no real `ESP32Transport` subclass. `ESP32Source` + mock transport are READY |
| **Edge** | **READY (software)** | `ESP32Source → ProcessedFrameToLiveState → LiveState` verified end-to-end with the mock transport; no synthetic fallback; needs only a real transport + device |
| **Dashboard** | **READY** | hardware-mode display, `HARDWARE DATA UNAVAILABLE` surface, real-time updates — verified in MOCK; `tsc` clean, 53 Vitest pass |
| **Unity** | **READY (software) / NOT VALIDATED (runtime)** | consumes the identical wire message, no independent calculation; not compiled/run (no Editor here) |
| **PostgreSQL** | **READY** | session/track/event persistence verified (SQLite fallback) in MOCK; one row per track/transition; source-tagged; history retrievable |
| **END-TO-END** | **NOT READY** | blocked on: physical hardware (A3M1, R121, STM32+firmware, ESP32, ECU) + the four specification sets + the concrete driver subclasses. Runbook: `docs/hardware-validation-procedure.md` TEST 1 → TEST 9 |

**PHYSICAL HARDWARE VALIDATION NOT PERFORMED. No physical results are claimed. No hardware
parameters are invented.**

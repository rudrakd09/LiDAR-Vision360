# Troubleshooting Guide

Consolidated software + hardware bring-up troubleshooting for LiDAR-Vision360. Ordered by where
in the pipeline the symptom shows. Hardware sections reference
`docs/hardware-validation-procedure.md` (TEST 1→9) and `docs/hardware-setup-guide.md`.

Legend: **[SW]** reproducible without hardware · **[HW]** needs the physical rig.

---

## 1. Backend won't start

| Symptom | Likely cause | Fix |
|---|---|---|
| `Address already in use` on `:8000` | a previous backend / `run_demo.ps1` still running | `.\scripts\stop_demo.ps1`; or find the PID with `Get-NetTCPConnection -LocalPort 8000` and stop it |
| `sqlalchemy ... OperationalError` / `no such column` on startup **[SW]** | stale SQLite fallback DB after a schema change | delete `cloud/backend/data/lidar_vision360.db` (gitignored runtime state) and restart — it is rebuilt |
| Logs `PostgreSQL unreachable, falling back to SQLite` | `LIDAR_DATABASE_URL` unset or DB down | expected if you have no PostgreSQL; set `LIDAR_DATABASE_URL` to use Postgres. Note the fallback in any evidence you capture |
| `ModuleNotFoundError: backend` | wrong `--app-dir` | run `uvicorn backend.main:app --app-dir cloud/backend/src ...` from the repo root |

---

## 2. Edge / bridge (`scripts/serve_unity_bridge.py`)

| Symptom | Likely cause | Fix |
|---|---|---|
| Exits with code **2**, log: *"NEVER falls back to simulation data"* **[SW]** | `LIDAR_DATA_SOURCE=hardware` but `LIDAR_ESP32_TRANSPORT` unset / placeholder | **by design.** Set a real transport (or `mock` for a bench run). The log names the missing variable |
| `Address already in use` on `:5006` | orphaned bridge from an earlier run holding the port | kill it: `taskkill /F /T /PID <pid>` (find via `Get-NetTCPConnection -LocalPort 5006`). The Phase-7/8 harnesses use `kill_tree` + a socket probe to avoid this |
| Scenario data looks wrong / frozen on one scenario **[SW]** | a second `serve_unity_bridge.py` (different `--scenario`) still bound to `:5006` | ensure only one bridge runs; `stop_demo.ps1`; confirm `/debug/live-frame.source_id == simulated:<scenario>` matches what you launched |
| `CoordinateTransformer() takes no arguments` in a custom harness **[SW]** | constructing pipeline stages with `settings=` where they take none | mirror `serve_unity_bridge.py`: most stages are no-arg; only `FusionEngine(settings=settings)` and `LiveStateBuilder(settings=settings)` take it |
| Bridge runs but backend shows `edge_status = connecting` | backend ingestion thread can't reach `127.0.0.1:5006` | start the bridge **before** or shortly after the backend; check no firewall rule blocks loopback `:5006` |

---

## 3. `ESP32Source` / hardware data path

| Symptom | Likely cause | Fix |
|---|---|---|
| `/debug/stream-status` → `edge_status: connecting`, never `connected` **[HW]** | ESP32 not reachable | `ping ⟨ESP32 host⟩`; check same subnet, AP client-isolation off, firewall allows `⟨ESP32 port⟩`; confirm `LIDAR_ESP32_HOST`/`PORT` |
| `frames_rejected` climbing **[HW]** | payload fails schema/version validation | inspect a raw frame; check `metadata.protocol_version` major is in `SUPPORTED_MAJOR_VERSIONS`; check encoding matches the deserializer (JSON vs binary → needs a `ProcessedFrameDeserializer`) |
| `sequence_gaps` / `dropped_frames` climbing **[HW]** | packet loss on Wi-Fi, or wrong `LIDAR_ESP32_SEQUENCE_MODULUS` (false wrap) | verify the modulus against the STM32/ESP32 sequence width; check Wi-Fi RSSI; newest frame is still delivered, so this is a quality signal, not a stop |
| State flips to `STALE` then `poll()` returns `None` **[HW]** | no fresh frame within `esp32_frame_stale_after_s` (STM32 hung / link dropped) | expected protective behaviour — **no synthetic frame is emitted**. Fix the upstream link; Dashboard shows `STALE DATA`, not `LIVE` |
| State `RECONNECTING` with growing backoff **[HW]** | transport open/receive raising | check ESP32 power (Wi-Fi TX brownout), AP association; backoff is `esp32_reconnect_*_backoff_s`, capped |
| `/debug/live-frame` → `null` in hardware mode | Edge not `connected` | **correct** when the link is down. Not a bug — there is no fallback frame |
| `build_transport()` raises "not implemented" **[HW]** | `LIDAR_ESP32_TRANSPORT` set to a name with no registered subclass | implement + register a concrete `ESP32Transport`; only `mock` ships |

---

## 4. STM32 → ECU CAN output (`can_output`)

| Symptom | Likely cause | Fix |
|---|---|---|
| `CANOutputConfig` validation error: fields required **[SW/HW]** | `can_output_require_fully_specified=True` with unset spec fields | fill every per-message/per-signal field from the vehicle DBC, or set `require_fully_specified=False` for a bench run with the mock backend |
| `CANOutputHealth.state = BUS_OFF` **[HW]** | bus wiring / termination / bitrate mismatch, or no other node ACKing | verify **120 Ω at both ends**, `LIDAR_CAN_OUTPUT_BITRATE_BPS` matches the ECU, at least one other node present; recovery uses `bus_recovery_*_backoff_s` |
| Queue overflow / frames dropped from the TX queue **[SW/HW]** | producing faster than `can_output_max_transmit_rate_hz` or the bus drains | raise the rate cap only if the DBC period allows; check `can_output_queue_max_frames`; confirm downstream isn't stalled |
| Analyser decodes wrong values **[HW]** | signal `start_bit` / `length_bits` / `byte_order` / `scale` / `offset` / sign wrong | re-check each against the DBC; confirm checksum algorithm + position and rolling-counter width |
| No frames on the bus at all **[HW]** | `LIDAR_CAN_OUTPUT_ENABLED` false, or no real `CANTransport` subclass | enable it; provide a `can_output.transport.CANTransport` implementation (e.g. `python-can`) — not a project dependency |
| ESP32 path breaks when CAN fails, or vice versa **[SW]** | should never happen — they are independent | if observed, it's a regression: `run_esp32_edge` wraps the CAN call in its own `try/except`; file it |

---

## 5. Dashboard

| Symptom | Likely cause | Fix |
|---|---|---|
| Panels blank / "DISCONNECTED" **[SW]** | backend down, or WebSocket not connecting | check `http://localhost:8000/health`; check the browser console for `/ws/live` errors; only one dashboard tab (multiple tabs are fine but each opens a socket) |
| "HARDWARE DATA UNAVAILABLE" banner **[HW]** | hardware mode + Edge not delivering fresh frames | **correct** display when the link is down — see §3. Not shown in simulation mode |
| Scan Rate shows a value with no stream, or looks stuck at 10 Hz | fallback chain in `SystemPanel` | the panel prefers server `measured_scan_rate_hz`, then client-measured, then configured. A real measured value only appears with a live stream |
| Objects from a previous scenario linger | session change not applied | on a scenario/session switch the table must clear; if not, capture `/debug/live-frame` + the WS message and file it — Phase 4 added `sessionOrdering` + reset handling |
| `TTC: N/A` for a clearly closing object **[HW/SW]** | object not classified as closing, or TTC genuinely unavailable | verify against `(/debug/live-frame).risk.results[].ttc`; the frontend never fabricates a TTC number |
| `npm run dev` fails / `tsc` errors after a pull **[SW]** | stale deps | `cd cloud/dashboard && npm install`; `npm test` for the Vitest suite |

---

## 6. Unity

| Symptom | Likely cause | Fix |
|---|---|---|
| HUD "SYSTEM: Disconnected" **[SW]** | bridge not serving `:5006`, or wrong input mode | start the bridge; set `LidarInputManager.mode = StructuredJsonTcp`; Unity connects to `127.0.0.1:5006` independently of the backend |
| Unity values differ from the Dashboard **[HW/SW]** | should be identical — both consume the same wire frame | compare for the **same** `session_id` + `sequence_number`; Unity does no perception of its own. A mismatch on matched frames is a regression |
| Nothing renders but connection is up | envelope version / message type mismatch | check the bridge is emitting `PERCEPTION_FRAME` on the versioned envelope (port 5006), not the legacy raw `<START>/<END>` (port 5005) |

---

## 7. PostgreSQL / persistence

| Symptom | Likely cause | Fix |
|---|---|---|
| No `sessions` row with `source_id` set after a run | session still open, or queried mid-session | the backend sets `source_id` on session **close**; stop the bridge and wait a few seconds before querying. The Phase-7 `p7_db_check.py` waits 8 s |
| `tracks` table empty right after a fresh restart **[SW]** | "event-history cold-start gap" — a `track_created` at frame 0 can be missed | fixed in Phase 8 via `_backfill_missing_tracks` in `ingestion.py` (persists any `track_id` seen in `tracked_objects` with no cached `db_id`). If still empty, confirm that fix is present |
| Duplicate rows on repeated WebSocket messages | idempotency broken | `_persist_track_created` early-returns when `db_id` is set; events are one-per-transition. Duplicates = regression, capture the frame sequence |
| Hardware and simulation rows indistinguishable | `source_id` not tagged | hardware rows carry `source_id = stm32_hardware`; simulation `simulated:<scenario>`. Query `WHERE source_id LIKE 'stm32_hardware%'` |
| `sqlite3.OperationalError: database is locked` **[SW]** | backend + a manual `sqlite3` connection writing at once | query read-only, or stop the backend first |

---

## 8. Demo scripts (`run_demo.ps1` / `stop_demo.ps1`)

| Symptom | Likely cause | Fix |
|---|---|---|
| `stop_demo.ps1` leaves a process running | process identity changed / child not in the tree | it uses identity-based detection; if one lingers, `Get-NetTCPConnection -LocalPort 8000,5006,5173` → stop the owning PIDs |
| `run_demo.ps1` reports a component unhealthy | slow start on first `npm install` / cold venv | re-run; increase the verify wait; run `verify_demo.py` standalone to see which check fails |
| Two `python.exe` per launched component in Task Manager **[SW]** | expected on this box — `.venv` launcher spawns a child | not a leak; `stop_demo.ps1` handles the tree |

---

## 9. Hardware bring-up (per stage — see `docs/hardware-validation-procedure.md`)

| Stage | Common failure | First checks |
|---|---|---|
| **TEST 1 — A3M1→STM32 UART** | garbage / no bytes | baud rate (`LIDAR_STM32_UART_BAUDRATE` is an unconfirmed `115200` placeholder); TX/RX not swapped; 3.3 V vs 5 V level; common ground; A3M1 motor actually spinning |
| **TEST 2 — R121→STM32 CAN** | no frames / error-passive | bitrate match; **120 Ω at both ends**; CANH/CANL not swapped; transceiver powered; analyser sees R121 traffic at all |
| **TEST 3 — STM32 processing** | frame fails `validate_processed_frame` offline | check `metadata` fields (protocol_version, sequence_number ≥ 0, timestamp epoch); `most_critical_track_id` names a present object; TTC/distance within configured limits |
| **TEST 4 — STM32→ECU CAN** | wrong decode / wrong period | signal layout vs DBC; checksum algo + position; DLC; transmission period vs `cycle_time_ms`; bus has an ACKing node |
| **TEST 5 — STM32→ESP32** | ESP32 sees partial / no frames | link baud/mode; framing (length prefix vs delimiter); CRC; 3.3 V both sides |
| **TEST 6 — ESP32→Edge** | see §3 above | transport/host/port; subnet; firewall; a registered `ESP32Transport` subclass |
| **TEST 7–9 — Edge→Dashboard/Unity/DB** | see §5–§7 above | mode shows HARDWARE; `source_id = stm32_hardware`; compare matched `sequence_number` |

**Any missing spec value → STOP and record what is missing.** Do not substitute a guess.
No hardware parameter in this repository is confirmed against a datasheet.

---

## 10. Escalation checklist

Before filing an issue, capture:

1. `GET /health` and `GET /debug/stream-status` (full JSON).
2. `GET /debug/live-frame` ×3, a few seconds apart.
3. The bridge console output (mode, `[ESP32]` / `[BRIDGE]` lines, exit code).
4. Which mode: `simulation` / `mock hardware` / `physical hardware`.
5. For hardware: which TEST number in `docs/hardware-validation-procedure.md` fails, and the
   per-field decode mismatch or electrical reading.
6. `git rev-parse HEAD` and whether `.env` is filled.

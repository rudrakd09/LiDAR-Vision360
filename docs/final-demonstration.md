# Final Demonstration Procedure

A single, repeatable walkthrough of LiDAR-Vision360. Steps 1–15 run in **simulation** (or
**mock hardware**) and need no physical device. Step 16 is the physical-sensor flow, to be run
only when the rig and specifications from `docs/hardware-validation-procedure.md` are available.

All shell blocks are PowerShell from the repository root. The Python venv must be active
(`.venv\Scripts\Activate.ps1`) or substitute `.venv\Scripts\python.exe` for `python`.

---

## STEP 1 — Start the backend

```powershell
python -m uvicorn backend.main:app --app-dir cloud/backend/src --host 0.0.0.0 --port 8000
```
Wait for `[BACKEND] Ready on 0.0.0.0:8000`. PostgreSQL is used if reachable; otherwise the
backend logs a warning and falls back to `cloud/backend/data/lidar_vision360.db` (SQLite).

## STEP 2 — Start the Edge / perception source

**Simulation** (one scenario; the demo cycles scenarios in later steps):
```powershell
python scripts/serve_unity_bridge.py --scenario 01_empty --rate 10
```

**Mock hardware** (SIMULATED ESP32 transport — scripted approaching vehicle, clearly labelled):
```powershell
$env:LIDAR_DATA_SOURCE = "hardware"; $env:LIDAR_ESP32_TRANSPORT = "mock"
python scripts/serve_unity_bridge.py --rate 10
```

The backend's ingestion thread connects automatically to `127.0.0.1:5006`.

## STEP 3 — Start the Dashboard

```powershell
cd cloud/dashboard; npm install   # first run only
npm run dev
```
Open `http://localhost:5173`.

## STEP 4 — Start Unity (optional; no Editor in CI)

Open `unity/LiDARVision360` in the Unity Editor, set `LidarInputManager.mode =
StructuredJsonTcp`, press **Play**. Unity connects to `127.0.0.1:5006` independently — the same
stream the backend consumes.

## STEP 5 — Verify health

```powershell
Invoke-RestMethod http://localhost:8000/health
```
Expect `status = ok`, a rising `uptime_s`.

```powershell
Invoke-RestMethod http://localhost:8000/debug/stream-status
```
Expect `connected = true`, `edge_status = connected`, `backend_status = ok`,
`websocket_status` = `connected` (dashboard open) or `no_clients`.

## STEP 6 — Verify the live frame

```powershell
Invoke-RestMethod http://localhost:8000/debug/live-frame | ConvertTo-Json -Depth 4
```
Confirm `source_id`, `sequence_number` (increases on each call), `timestamp` (changes),
`objects` / `tracked_objects`, `risk`, `clearance`. In **mock hardware** mode: `source_id =
stm32_hardware`, `config.data_source = hardware`. If it returns `null`, the Edge is not connected
(no synthetic fallback — that is correct behaviour).

## STEP 7 — Empty scenario (`01_empty`)

```powershell
python scripts/serve_unity_bridge.py --scenario 01_empty --rate 10
```
Dashboard: **Object Count 0**, **Track Count 0**, **Risk SAFE**, `NO ACTIVE OBJECTS` in the
Detected Objects table, clearance panel showing large distances. `/debug/live-frame`:
`objects: []`, `risk.overall_risk: "safe"`.

## STEP 8 — Vehicle scenario (`04_vehicle_ahead`)

```powershell
python scripts/serve_unity_bridge.py --scenario 04_vehicle_ahead --rate 10
```
Dashboard: **1 object**, classification **vehicle like**, a real **distance**, a **track ID**
(`track-1`), **Risk WARNING**, clearance panel showing a reduced front clearance.

## STEP 9 — Multiple-object scenario (`05_multiple_obstacles`)

```powershell
python scripts/serve_unity_bridge.py --scenario 05_multiple_obstacles --rate 10
```
Dashboard: **multiple objects** (≈5), **multiple distinct track IDs** (`track-1 … track-5`),
mixed classifications (wall / pole_like / vehicle_like / unknown), **Risk CRITICAL**.

## STEP 10 — Approaching-object scenario (`08_approaching_obstacle`)

```powershell
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --rate 10
```
Watch over ~10 s: **distance decreases**, **TTC decreases** (finite, e.g. 1.8 s → 0.7 s → 0.0 s),
**Risk** escalates toward **CRITICAL**, front clearance shrinks. The Dashboard updates every
frame — no page refresh.

## STEP 11 — Show TTC

Dashboard → **Safety** panel → **Min TTC** tile (the most-critical object's TTC); and the
**Detected Objects** table's **TTC** column (per-object). `N/A` when an object is not closing —
never a fabricated number. Cross-check: `(/debug/live-frame).risk.results[].ttc`.

## STEP 12 — Show clearance

Dashboard → **Safety** panel → **Front / Rear / Left / Right** cells + **Min Clearance** +
direction + **Corridor Width**. Environment Map draws the nearest-clearance ray. Path is
`ClearanceEngine → LiveState.clearance → /ws/live → ClearancePanel` (and Unity `HUDController`);
nothing is computed in React. The historical "Clearance Engine module missing" issue does **not**
exist — `perception/src/clearance/engine.py` is present and wired in both modes.

## STEP 13 — Show risk

Dashboard → **Safety** panel → **Risk** tile showing **SAFE / WARNING / CRITICAL** and the
**Critical Object** breakdown (track, distance, TTC, risk badge). The value comes verbatim from
`LiveState.risk.overall_risk` (simulation: the collision engine; hardware: the STM32 processed
frame). No distance-based risk logic in the frontend.

## STEP 14 — Show tracking history

Dashboard → **Tracking** panel → select a track ID → trajectory SVG + recorded-frames table +
first-seen / last-seen / frames-tracked / current & previous position / velocity. On a scenario
switch the history resets (a reused `track-1` in a new session does not splice the old
trajectory).

## STEP 15 — Show PostgreSQL

```powershell
Invoke-RestMethod http://localhost:8000/api/sessions?limit=5
Invoke-RestMethod http://localhost:8000/api/collision-events?limit=10
Invoke-RestMethod http://localhost:8000/api/clearance-events?limit=10
Invoke-RestMethod "http://localhost:8000/api/tracks"
```
Each session row carries `source_id` (`simulated:<scenario>` or `stm32_hardware`) and
`edge_session_id`. Event rows carry `session_id` + `source_id` + `timestamp` + the relevant
track/distance/TTC/clearance/risk. One `tracks` row per `track_id` (never one per frame); one
event row per transition. Hardware and simulation rows are distinguishable by `source_id`.

## STEP 16 — Physical sensor flow (hardware only)

**Prerequisites:** the rig and all four specification sets in
`docs/hardware-validation-procedure.md` "Prerequisites"; the three concrete subclasses
(`LiDARMessageParser`, `RadarMessageParser`, `ESP32Transport`/`ProcessedFrameDeserializer`,
`CANTransport`); a filled-in `.env`.

Then run `docs/hardware-validation-procedure.md` **TEST 1 → TEST 9 in order**, stopping on the
first failure:

1. A3M1 → STM32 (UART) 2. R121 → STM32 (CAN) 3. STM32 processing/fusion 4. STM32 → ECU (CAN)
5. STM32 → ESP32 6. ESP32 → Edge (`ESP32Source`) 7. Edge → Dashboard 8. Edge → Unity
9. Edge → PostgreSQL — then the controlled physical test (§12) and failure matrix (§13).

```powershell
$env:LIDAR_DATA_SOURCE = "hardware"
$env:LIDAR_ESP32_TRANSPORT = "<real transport name>"
$env:LIDAR_ESP32_HOST = "<ESP32 host>"; $env:LIDAR_ESP32_PORT = "<ESP32 port>"
python scripts/serve_unity_bridge.py --rate 10
```
With `LIDAR_ESP32_TRANSPORT` unset this exits **2** with *"NEVER falls back to simulation"* — by
design.

---

## Failure demonstration (any mode)

| Induce | Expected Dashboard / API response |
|---|---|
| Stop the bridge (`Ctrl+C` / `scripts/stop_demo.ps1`) | `edge_status → connecting`; `System: DISCONNECTED`; `/debug/live-frame → null` (not a stale frame). **No simulation fallback.** |
| No fresh frame for > 3 s (bridge paused) | `System: STALE DATA`; last values shown but not claimed LIVE; frame age grows |
| Restart the bridge | `edge_status → connected`; new session; Dashboard resets old objects/risk/TTC |
| Hardware mode, `LIDAR_ESP32_TRANSPORT` unset | bridge exits 2; log names the missing variable; **never** starts simulation |
| Hardware mode, ESP32/Wi-Fi lost | `ESP32Source` → `RECONNECTING` (backoff); Dashboard `ESP32: UNAVAILABLE` + `HARDWARE DATA UNAVAILABLE — <reason>`; timeline "Connection lost" |
| Malformed / stale / duplicate frame | `ESP32Source.frames_rejected` / `dropped_frames` / `sequence_gaps` increment; newest frame still delivered; frame not applied on a stale/duplicate |

---

## One-command simulation demo

```powershell
.\scripts\run_demo.ps1 -Scenario 08_approaching_obstacle -Rate 10
```
Starts perception bridge + backend + dashboard in their own windows with process-identity
startup/shutdown and end-to-end verification. Stop with `.\scripts\stop_demo.ps1`.

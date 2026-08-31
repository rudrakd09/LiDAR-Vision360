# MOCK STM32 HARDWARE OUTPUT (Phase 10)

A scripted generator of Phase-2 `STM32ProcessedFrame`s that stands in for the *already-processed*
perception frames the STM32F103C8T6 firmware will emit once the physical
`A3M1 + R121 -> STM32 -> ESP32` chain exists. It lets the **complete Edge software path** be run
and proven hardware-ready with **no devices attached**:

```
MOCK STM32 HARDWARE OUTPUT
   -> ScriptedScenarioESP32Transport   (SIMULATED ESP32 transport, LIDAR_ESP32_TRANSPORT=mock)
   -> ESP32Source                      (connection / stale / sequence-gap / reconnect handling)
   -> ProcessedFrameToLiveState        (adapter -> pipeline.LiveStateBuilder; no perception re-run)
   -> LiveState
   -> streaming.PerceptionStreamServer (:5006)
        -> backend  -> WebSocket -> Dashboard
        -> Unity
        -> PostgreSQL
```

**Nothing here is a physical measurement.** It never claims to be hardware data — every frame's
`system_status.notes` says `"MOCK STM32 HARDWARE OUTPUT -- scripted, not sensed."` and the
transport logs `SIMULATED ESP32 TRANSPORT ... NOT REAL HARDWARE` on every `open()`.

Module: [`perception/src/datasources/esp32/mock_stm32.py`](../perception/src/datasources/esp32/mock_stm32.py).

---

## What it is *not*

It does **not** re-simulate LiDAR/radar perception — no point cloud, no clustering, no
classification. That is the simulator's job (`data_source="simulation"`), left untouched. This
module models only the **output** of the STM32's on-device fusion + tracking + TTC + clearance +
risk stage.

## Faithful thresholds — nothing hand-picked

Every risk / TTC / clearance value a frame carries is produced by the project's **own** functions
run over scripted object kinematics:

| Concern | Function used | Config it reads |
|---|---|---|
| TTC, closing speed | `collision.ttc.compute_ttc`, `closing_speed_along` | `collision_minimum_closing_speed_mps`, vehicle geometry/margins |
| Projected-path membership | `collision.geometry.in_projected_path` | `vehicle_*_m`, `*_safety_margin_m`, `lidar_range_max_m` |
| SAFE / WARNING / CRITICAL | `collision.risk.assess_risk` | `collision_warning/critical_distance_m`, `collision_warning/critical_ttc_s` |
| Clearance status | `clearance.risk.assess_clearance` | `clearance_caution/low/critical_distance_m` |

So the WARNING/CRITICAL boundaries move if you move the configured thresholds — verified by
`test_risk_ttc_clearance_use_project_thresholds_not_hardcoded`. (The mock does **not** run the 2D
swept-footprint predictor `collision.prediction.simulate_collision`; `collision_predicted` stays
`False` and `assess_risk` is fully defined without it. `system_status.processing_time_ms` is
`None` — a mock has no real timing to report.)

---

## Scenes

Selected by `LIDAR_ESP32_MOCK_SCENARIO` when `LIDAR_ESP32_TRANSPORT=mock` (or `scenario=` /
`build_mock_stm32_frames(name)` in code). `live=True` expands each into a smooth, velocity-
consistent run for a continuous stream; `live=False` yields the minimal canonical form used by
tests.

| Scenario | Shape | Canonical form | Purpose |
|---|---|---|---|
| `approaching_vehicle` *(default, unset)* | legacy `build_scripted_approaching_vehicle_frames` | — | unchanged Phase-3 behaviour |
| `realtime_arc` | empty → 10 m → 7 m → 5 m → 3 m → receding, one `vehicle_like` track | exactly 6 frames | real-time risk/TTC/clearance arc |
| `multi_object` | `vehicle_like` closing head-on + static `pole_like` (left) + static `wall` (right), 3 persistent distinct track IDs | 24 frames | multi-object display + track persistence + classification pass-through |
| `tracking` | one `vehicle_like` track, `mock-obj-1`, closing 10 m → 5 m at constant speed | ~42 frames | one continuously-tracked object, monotonically decreasing distance |

### `realtime_arc` canonical risk progression (project thresholds, defaults)

| Frame | Object | Reported closing speed | TTC | Overall risk |
|---|---|---|---|---|
| 1 | none | — | — | **SAFE** |
| 2 | 10.0 m ahead | 0.6 m/s | 11.0 s | **SAFE** (dist > 5 m, TTC > 4 s) |
| 3 | 7.0 m ahead | 0.8 m/s | 4.5 s | **SAFE** (TTC still > 4 s) |
| 4 | 5.0 m ahead | 0.7 m/s | 2.29 s | **WARNING** (dist ≤ 5 m) |
| 5 | 3.0 m ahead | 1.0 m/s | 0.0 s | **CRITICAL** (inside the safety envelope → TTC 0) |
| 6 | 6.0 m ahead, receding | +1.5 m/s | undefined | **SAFE** (moving away) |

(The object position is scripted at each checkpoint; the reported closing speed is the STM32
tracker's *filtered* estimate, exactly as a real Kalman-filtered tracker's velocity output
smooths/lags position — it is not required to be the exact frame-to-frame position derivative.
`live=True` *is* velocity-consistent frame to frame.)

---

## Running it live

```powershell
# backend
python -m uvicorn backend.main:app --app-dir cloud/backend/src --port 8000

# edge, hardware mode, mock STM32 -> mock ESP32
$env:LIDAR_DATA_SOURCE   = "hardware"
$env:LIDAR_ESP32_TRANSPORT = "mock"
$env:LIDAR_ESP32_MOCK_SCENARIO = "realtime_arc"   # or multi_object | tracking
python scripts/serve_unity_bridge.py --rate 10

# dashboard
cd cloud/dashboard; npm run dev        # http://localhost:5173  -> Mode: HARDWARE
```

`GET /debug/live-frame` shows `source_id = stm32_hardware`, `config.data_source = hardware`,
`sequence_number` increasing, and the arc's risk/TTC/clearance changing frame to frame. With
`LIDAR_ESP32_TRANSPORT` unset, `serve_unity_bridge.py` exits **2** — there is no simulation
fallback in hardware mode.

## Using it in code / tests

```python
from datasources.esp32 import (
    build_mock_stm32_frames, MockESP32Transport, ESP32Source, ProcessedFrameToLiveState,
)

frames = build_mock_stm32_frames("realtime_arc")          # list[STM32ProcessedFrame]
tr = MockESP32Transport(); tr.push_frames(frames)
src = ESP32Source(settings, transport=tr); src.connect()
adapter = ProcessedFrameToLiveState(settings, session_id=src.session_id)
# ... src.poll() -> adapter.build(frame) -> build_perception_frame_message(...)
```

`build_mock_processed_frame([MockObjectState(...)], frame_index=…, timestamp=…)` builds one
arbitrary frame directly (used by the sequence-gap and pass-through tests).

Tests: [`perception/tests/test_phase10_mock_stm32_pipeline.py`](../perception/tests/test_phase10_mock_stm32_pipeline.py)
(20 cases, mapped to the Phase-10 acceptance checklist).

---

## What still needs the real hardware specification

The mock proves the **software** is complete and correct. It cannot substitute for:

* the real ESP32 Wi-Fi transport (a `datasources.esp32.transport.ESP32Transport` subclass) —
  `LIDAR_ESP32_TRANSPORT`, host/port/framing/encoding are still `⟨PENDING SPEC⟩`;
* a non-JSON wire codec (`ProcessedFrameDeserializer`) if the STM32↔ESP32 link is not JSON;
* the STM32 firmware itself (A3M1 UART decode, R121 CAN decode, on-device fusion/tracking/risk);
* real timing, real sensor-health flags, real sequence-wrap semantics.

See [`hardware-validation-procedure.md`](hardware-validation-procedure.md) for the per-signal
checklist and the physical bring-up runbook.

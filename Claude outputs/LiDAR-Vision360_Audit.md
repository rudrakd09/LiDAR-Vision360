# LiDAR-Vision360 — Initial Code Audit

**Scope:** read-only inspection. No files were modified. This covers `PROJECT_SPECIFICATION.md`, `README.md`, `.env.example`, `.gitignore`, all 26 files in `docs/`, and the core Python (`perception/`), C# (`unity/`), and cloud (`cloud/backend`, `cloud/dashboard`) source relevant to UART ingestion, parsing, the 360° pipeline, and collision logic. Facts below are cited from the repository; anything I couldn't verify directly is labeled as such.

---

## 1. Project Purpose

LiDAR-Vision360 turns ~360 `(angle, distance)` measurements from a 2D 360° LiDAR into structured environmental understanding: obstacle detection, shape classification, persistent object tracking, time-to-collision (TTC), directional clearance, occupancy mapping, and an overall SAFE/WARNING/CRITICAL risk assessment. Results are meant to be surfaced identically through a Unity 3D "digital twin," a real-time web dashboard, and a PostgreSQL-backed event history. Optional radar fusion and a CAN output stage (toward a vehicle ECU) exist as software models. It is explicitly a **prototype**, not a certified automotive safety system, and explicitly **2D-only** (Unity's 3D view is a visualization of 2D-derived data, not true 3D reconstruction).

The project was built "hardware-independent first": every stage was developed and validated against a configurable 2D LiDAR **simulator** before any physical sensor was involved. A second, parallel framework exists for real hardware, built around a different target architecture than a naive "PC parses raw LiDAR bytes" design — see §2 and §5.

---

## 2. Current Architecture — Data Flow

There are **three distinct, independent data paths** in this repository. This is the single most important architectural fact for your stated goal (you have real UART data now) — read this section before doing anything else.

### Path A — Simulation (the only path that is fully working end-to-end today)

```
simulator.SimulatorSource (10 scenario JSON files, ray-casting)
  → ScanFrame (raw polar points)
  → preprocessing.Preprocessor          (validate → sort by angle → outlier removal → median filter → optional temporal filter)
  → coordinates.CoordinateTransformer   (polar → Cartesian, vectorized numpy)
  → clustering.DBSCANClusterer          (obstacle clustering on x,y)
  → objects.GeometricClassifier         (wall/vehicle_like/pole_like/person_like/large_obstacle/unknown)
  → tracking.ObjectTracker              (nearest-neighbor association + per-track Kalman filter, track_id lifecycle)
  → fusion.FusionEngine                 (merges synthetic RadarReading if present; LiDAR-only passthrough otherwise)
  → collision.CollisionRiskEngine       (TTC, projected-path check, SAFE/WARNING/CRITICAL + hysteresis)
  → clearance.ClearanceEngine           (front/rear/left/right clearance, independently, off the Cartesian scan)
  → mapping.OccupancyGridMapper         (2D log-odds occupancy grid, branches off the Cartesian scan in parallel)
  → pipeline.LiveStateBuilder           (assembles one LiveState per scan: joins objects+risk by track_id, event log, timing)
  → streaming.PerceptionStreamServer    (TCP :5006, versioned JSON envelope) + streaming.RawLidarStreamServer (TCP :5005, legacy <START>/<END> text)
  → Unity (PerceptionTCPClient.cs) and cloud/backend (FastAPI) as two independent TCP clients of the same broadcast
  → cloud/backend persists to PostgreSQL (or local SQLite fallback) and re-broadcasts over /ws/live to cloud/dashboard (React)
```

This is orchestrated by `scripts/serve_unity_bridge.py --scenario <name>`. It is the only path the README claims has been end-to-end verified (10/10 scenarios, live).

### Path B — "Hardware" processed-frame path (`ESP32Source`) — the *intended* real-hardware path, but not what you're describing

The target architecture puts the **STM32 itself** as the primary perception node: STM32 does UART LiDAR acquisition, preprocessing, clustering, classification, tracking, TTC, clearance, and risk **on-device**, then ships an already-fully-processed `STM32ProcessedFrame` (JSON, by default) to an ESP32, which relays it over Wi-Fi to the PC. The PC (`datasources.esp32.ESP32Source` → `ProcessedFrameToLiveState`) does **no** perception in this mode — it just reshapes an already-finished result into the same `LiveState` the simulator produces. This only runs today against a scripted `mock` transport (`LIDAR_ESP32_TRANSPORT=mock`) — there is no real ESP32 Wi-Fi transport implemented, and the wire protocol (host/port/framing/encoding) is entirely unspecified.

**This path cannot consume raw `(D, A)` LiDAR measurements at all** — its input contract is pre-computed objects/risk/clearance, not raw points.

### Path C — Raw-serial "STM32Source" path — architecturally present, but its parser is a stub that always raises

This is the one that structurally matches "PC reads raw UART bytes and extracts distance/angle": `perception/src/datasources/stm32_source.py` + `perception/src/datasources/stm32/` (`transport.py` — real `pyserial`-backed serial I/O; `framing.py` — delimited or fixed-length frame extraction; `crc.py` — none/xor8/sum8/crc8/crc16_ccitt; `sequence.py` — dropped-frame detection; `health.py`; `parsers.py`).

**Critical fact:** `STM32Source.connect()` calls `_validate_protocol_config()` first and raises `STM32ConfigurationError` unless ~10 protocol fields are set (frame markers or fixed length, byte order, message-type offset/width/IDs, sequence-number offset/width, CRC algorithm, timestamp format) — **all of these are unset by design** in `.env.example`. Even if you set them all, the actual byte→`LiDARPoint` decode (`LiDARMessageParser.parse()`) is implemented only as `UnconfiguredLiDARParser`, whose `.parse()` **always raises `STM32ConfigurationError`** (`perception/src/datasources/stm32/parsers.py`). No concrete subclass exists anywhere in the repo. This is intentional — the project's stated rule is "never invent a wire protocol ahead of a real hardware spec" — but the practical consequence is: **this path cannot currently turn any real UART bytes into `LiDARPoint`s, no matter how you configure `.env`.** Someone has to write a `LiDARMessageParser` subclass that knows your sensor's actual byte layout.

### Path D — Unity's own direct serial reader (`LidarSerialReader.cs`) — a fourth, separate, pre-existing path

There is also a **completely independent, self-contained C# script** that opens a COM port directly from Unity and reads a simple text protocol:

```
LidarSerialReader.cs (Unity, System.IO.Ports.SerialPort, default COM3 @ 115200)
  reads lines: "<START>" ... "angle,distance" (comma-separated floats) ... "<END>"
  → LidarCubes.cs (UpdateScan) — divides distance by 10 ("cm → meters" per the comment, but
    the code and the matching Python sender both treat it as decimeters, i.e. meters × 10 on the wire)
  → renders a bare, uncolored-by-risk point cloud only
```

This is documented in `docs/communication.md`/`docs/unity.md` as "the original, completely unmodified point-cloud + audio path," selectable via `LidarInputManager.mode = LegacySerial`, and explicitly **does not feed the Python perception pipeline at all** — no clustering, classification, tracking, TTC, clearance, or risk. It only drives a bare cube visualizer (`LidarCubes.cs`) and a beep (`LidarBeep.cs`).

### Which of these matches "I already have real UART data with D/A"?

Given what exists in the repo, **Path D is the only one that can currently ingest a live serial stream with no further code changes**, and only if your hardware's line protocol already happens to match `<START>` / `angle,distance` / `<END>` with distance on the wire in **decimeters** (meters × 10) — not centimeters despite the misleading comment, not raw meters, not millimeters. Paths B and C cannot currently consume your data without new code (a real `LiDARMessageParser`, or a real `ESP32Transport` + non-JSON deserializer). I did not find any indication of what your actual device's serial line format is; you'll need to tell me that before anything downstream can be wired up correctly. **I have not invented or assumed a protocol** — this matches the project's own explicit rule.

---

## 3. Repository Structure

| Path | Contents |
|---|---|
| `PROJECT_SPECIFICATION.md` | Original, frozen spec (26 objectives, 18 phases). Not updated for status — `README.md` is. |
| `README.md` | Living status document; very detailed, cross-referenced, and (per my reading) accurate against the source. |
| `.env.example` | Every `LIDAR_*` setting with defaults/placeholders; hardware fields deliberately left unset. |
| `perception/` | The Python perception engine (installable package). `src/` splits into `common` (config/logging), `models` (canonical pydantic data models), `datasources` (simulator/STM32/ESP32 adapters), `preprocessing`, `coordinates`, `clustering`, `objects` (classification), `tracking`, `mapping`, `collision`, `clearance`, `fusion`, `can_output`, `serialization`, `streaming`, `pipeline` (`LiveStateBuilder`). `tests/` — reported ~1056 tests. |
| `simulator/` | Standalone 2D LiDAR simulator (ray-casting, 10 scenario JSON files under `scenarios/`, noise/obstacle models). Depends on `perception`; `perception` never depends on it. |
| `unity/LiDARVision360/` | Unity project. `Scripts/Input/` (both the legacy serial/raw-TCP readers and the new `PerceptionTCPClient`), `Core/` (coordinate conversion, session/frame-id validation), `Visualization/`, `Objects/`, `Safety/`, `Mapping/`, `Audio/`, `UI/`, `Tests/EditMode/`. |
| `cloud/backend/` | FastAPI + SQLAlchemy (PostgreSQL, SQLite fallback). Ingests the same TCP stream Unity consumes, persists sessions/tracks/events, serves REST + `/ws/live`. |
| `cloud/dashboard/` | React + TypeScript + Vite. Renders `LiveState` only — no client-side recomputation of risk/tracking. |
| `embedded/stm32/` | **README only.** No firmware in this repository. Documents the *assumed* (not confirmed) wire-shape the Edge side was built against. |
| `docs/` | 26 markdown files, extremely thorough and internally consistent — architecture, per-stage design docs, communication protocol, hardware integration checklist, hardware setup/wiring guide, hardware validation runbook (TEST 1→9), troubleshooting, final-demo procedures. |
| `scripts/` | `serve_unity_bridge.py` (the main Edge process), `sensor_source.py` (simulation/hardware source factory), demo/benchmark/visualize/evaluate scripts per phase, `run_demo.ps1`/`stop_demo.ps1`. |
| `docker/` | README only — no Dockerfile yet. |

---

## 4. Current Implementation Status

**Fully implemented and (per the repo's own reported test runs) passing, against simulation/synthetic data only:**
Preprocessing, polar→Cartesian, DBSCAN clustering, geometric shape classification, nearest-neighbor + Kalman tracking, 2D occupancy grid mapping, collision/TTC engine with risk hysteresis, directional clearance engine, sensor fusion engine (LiDAR + synthetic radar), the STM32→ECU CAN output *software model*, the STM32-processed-frame contract + validator + JSON reference codec, the real-time streaming protocol (both legacy raw and versioned JSON), Unity's structured-JSON visualization suite (written, but **never compiled/run in an actual Unity Editor** — no Editor was available while it was built), the FastAPI backend, and the React dashboard.

**Explicitly scaffolded but not functional against real hardware:**
- `STM32Source` (raw-serial path): connection management, framing, CRC, sequence validation, health tracking, and reconnect-with-backoff are real and tested against synthetic byte streams — but the LiDAR/radar payload parser is a stub that always raises (see §2, Path C).
- `ESP32Source` (processed-frame path): connection/timeout/reconnect/stale-detection state machine is real — but only ever exercised against a scripted mock transport; no real Wi-Fi transport exists.
- `can_output`: fully modeled (message/signal specs, DBC-style bit-packer, health/recovery) but no real `CANTransport` and no DBC spec.

**Not implemented at all:**
- Any STM32 firmware (the directory is a README).
- Any concrete `LiDARMessageParser`/`RadarMessageParser`.
- Any real ESP32 Wi-Fi transport or non-JSON codec.
- Authentication, production deployment, alerting beyond the dashboard's own event timeline, true 3D perception (out of scope by design).

**What "already tested" means in this repo, concretely:** unit tests (pytest/Vitest) plus live runs of all 10 simulator scenarios and the mock-hardware path. The README is explicit and repeated, in multiple places, that **no physical A3M1/R121/STM32/ESP32 has ever been connected**, and that `serial.tools.list_ports.comports()` returned **none** in the environment this was developed in. That statement predates this conversation — if you now have a sensor physically wired up and emitting data, that is new since the repo's own documentation was last updated, and nothing in the repo has been validated against it yet.

---

## 5. Potential Bugs / Risks

### Confirmed (verified directly in source, not assumption)

1. **Unit-mismatch risk between real hardware and the legacy Unity visualizer.** `LidarCubes.cs` does `float dist = data.distance / 10;` with the comment `// cm → meters`. That comment is wrong: dividing by 10 recovers meters from **decimeters** (meters × 10), not centimeters (which would need `/100`). This matches what the Python `RawLidarStreamServer.format_raw_scan()` intentionally sends (`point.distance * 10.0`), so the simulation path round-trips correctly *despite* the misleading comment. But `LidarSerialReader.cs` feeds the exact same `/10` conversion directly from **your raw hardware serial line** — if your sensor's actual UART "distance" field is in meters, millimeters, or centimeters (not decimeters), the point cloud will be off by a wrong, silent scale factor. This is the single most likely "why do my points look wrong" bug if you wire your real device into the legacy Unity path as-is.
2. **`STM32Source`'s LiDAR/radar parsers always raise (`UnconfiguredLiDARParser`/`UnconfiguredRadarParser`).** Confirmed in `perception/src/datasources/stm32/parsers.py` — there is no code path today that turns real UART bytes into `LiDARPoint`s through this class. This is a deliberate, documented placeholder, not an oversight, but it means the "rich" Python pipeline (clustering/tracking/collision/mapping) currently has **no way to receive your real sensor's data** without a new parser class being written.
3. **`SerialLiDARDataSource.connect()`/`.read_scan()` are `NotImplementedError` stubs** (`perception/src/datasources/serial_source.py`) — the plain placeholder `LiDARDataSource` the spec calls for Phase 16 is not implemented either.
4. **`LidarSerialReader.cs` has no reconnect, no malformed-line recovery beyond a bare `try { } catch { }` that silently discards the failing line, and calls `Thread.Abort()` on quit** — `Thread.Abort()` is deprecated/unreliable on modern .NET runtimes and can leave the serial port thread in an undefined state on exit. This script also never validates `angle`/`distance` ranges before use (no bounds check), unlike every stage of the Python pipeline, which validates inputs explicitly.
5. **`embedded/stm32/` contains no firmware** — confirmed by directory listing (single `README.md`). Any assumption about what the STM32 (or your actual UART source) sends is, by the repo's own admission, unconfirmed against a datasheet.
6. **Two of the reported test suites are pre-existing known failures, not fixed:** `cloud/backend/tests/test_api.py` hangs under pytest (not run in CI); `simulator/tests::TestPole::test_pole_classifies_as_pole_like` is documented as noise-driven flaky (~1/12 runs). Neither blocks the rest of the suite, but neither is "1056 passing" territory — the README's own numbers already net these out honestly.

### Plausible / worth checking, not yet confirmed by direct evidence

7. **Sequence-number wraparound heuristic in `SequenceValidator._gap_forward`** treats any apparent gap larger than `modulus // 2` as "backward/duplicate" rather than "a huge number of dropped frames." This is a standard, reasonable heuristic, but if your real device's sequence counter width/modulus is smaller than assumed (e.g. an 8-bit counter wrapping at 256) and `LIDAR_STM32_SEQUENCE_MODULUS` / `LIDAR_ESP32_SEQUENCE_MODULUS` is left unset, drop/duplicate accounting could be silently wrong. I did not find evidence either way since no modulus is configured yet.
8. **`STM32Source._resolve_timestamp()` always returns Edge receive time**, never a real on-wire timestamp, even once `stm32_timestamp_format` is configured — there is no field offset/width defined for it yet. If your device timestamps its own samples and you need that value downstream (rather than PC receive time), this needs an additional config field + decode, not just a settings value.
9. **DBSCAN clustering runs on a full `(x, y)` array with no spatial index** (`sklearn.cluster.DBSCAN` default) — fine at ~360 points/scan (the design target), but if your real sensor has a much higher point density (e.g. a modern 2D LiDAR at 5–10k points/rev instead of the simulator's 360), clustering/preprocessing latency should be re-benchmarked; the documented performance figures (§ "Performance" in `docs/communication.md`) were all measured at the simulator's own point density.
10. **The occupancy grid mapper (`mapping/grid.py`) is a single dense NumPy array (`(height_cells, width_cells)` at the configured resolution)** with no sparsity — fine at the documented 400×400@0.1m default, but worth knowing if you later widen `mapping_width_m`/`mapping_height_m` or tighten resolution substantially.

### Explicitly *not* bugs (documented, deliberate design, so you don't waste time on them)

- Hysteresis holding a risk level for one extra scan during de-escalation — intentional debounce (`collision/hysteresis.py`), not lag/latency.
- `sensor_status["radar"]` always `null` in simulation mode — intentional ("never fabricate a reading").
- `/debug/live-frame` returning `null` when hardware mode has no fresh frame — intentional, "no synthetic fallback" rule, not a connectivity bug.
- Unity C# throughout: **written and reviewed, never compiled/run in a live Unity Editor** anywhere in this project's history (repo-wide constraint, not specific to any one script) — treat every Unity script as unverified-in-Editor even though the code itself reads carefully.

---

## 6. Specification Compliance

Against `PROJECT_SPECIFICATION.md` §3's 26 objectives:

| Status | Objectives |
|---|---|
| **Complete** (simulation-validated) | Simulated 360° generation; preprocessing; noise/outlier filtering; polar→Cartesian; obstacle clustering; segmentation; geometric feature extraction; shape classification; object IDs; tracking; velocity estimation; occupancy grid mapping; collision-risk estimation; TTC; low-clearance detection; vehicle safety zones; Unity visualization; Unity digital twin; cloud telemetry; real-time dashboard; automated tests; documentation. |
| **Partially complete** | *Real LiDAR data integration via hardware adapter* — architecture (framing/CRC/sequence/health/reconnect) is implemented and tested against synthetic bytes, but the actual byte→point decode is an intentional stub (§5.2 above); nothing has processed a real measurement. *Historical analytics* — bounded event history exists in PostgreSQL; no analytics UI beyond the dashboard's own timeline. *Safety alerts* — risk/clearance levels are computed and shown; there's no separate alerting/notification channel (email/SMS/push). |
| **Missing** | Real hardware validation of any kind (explicitly, repeatedly stated in the README as not performed). STM32 firmware. A concrete `LiDARMessageParser` for your actual sensor. |
| **Potentially incorrect (needs your input, not a code fix)** | The `.env.example`/hardware-setup-guide's assumed STM32-side framing convention (marker + message-type + sequence-number + payload + checksum) is explicitly labeled "expected shape (not yet fixed)" in `embedded/stm32/README.md` — it may not match your actual device's protocol at all, especially if your device isn't going through an STM32/A3M1 the way the target architecture assumes. |

Two things worth flagging as scope drift from the original spec, not defects: (a) the spec's Phase-16 hardware adapter was meant to be a **thin STM32→PC raw serial link with perception running on the PC**; the project has since pivoted its *target* hardware architecture to put perception **on the STM32 itself** (ESP32Source path), with the raw-serial PC-side path (STM32Source) demoted to "legacy, still tested, not the target." If your actual setup is "a LiDAR sends raw D/A over UART to a PC, and the PC does the perception," that is the **original spec's** intent and Path C, not the currently-preferred Path B — worth deciding explicitly which target you want, since the codebase has partially moved away from Path C as "the" hardware path. (b) Radar/sensor fusion was originally "explicitly out of scope for this prototype" per the spec's own sensor-limitation section, then later reversed and implemented anyway (documented as a deliberate, approved reversal in `docs/architecture.md`).

---

## 7. Technical Debt / Architecture Issues

- **Four independent "hardware entry points" (§2) is real complexity, not incidental.** It's individually well-justified (each documented at length), but it means "wire up my real LiDAR" is not a single obvious place to plug in — the first real decision needed is *which* of paths B/C/D your setup should extend, given your actual device and whether perception should run on the PC or on a microcontroller.
- **`embedded/stm32/` firmware genuinely doesn't exist.** If your real hardware setup does *not* include an STM32 at all (e.g. LiDAR → USB-serial adapter → PC directly, or LiDAR → Arduino/ESP32 relaying raw D/A rather than processed frames), neither Path B nor Path C match your topology as designed, and the "STM32 does the perception" target architecture may not apply to you at all — this is a fork in the road worth resolving explicitly before writing any parser code.
- **`perception/src/pipeline/` predates `LiveStateBuilder` in name only** — `docs/architecture.md` still describes it as "empty, documented placeholder" in one place while `pipeline/live_state.py` (with real, substantial code) clearly exists; a stale doc paragraph, not a code problem, but worth a docs cleanup pass later.
- **Config surface is large** (`common/config.py` is ~1400 lines / 49KB) — appropriate given "never hard-code thresholds" is a hard project rule, but it does mean onboarding to "what does X control" has real ramp-up cost. Not urgent.
- **No Dockerfile yet** (`docker/` is a README) despite the spec listing containerization as a goal — low priority given the project's current single-machine, prototype scope.
- **Two known-flaky/hanging tests** (§5.6) are a small amount of accumulated debt worth resolving before you rely on `pytest` output as a regression gate.

---

## 8. Recommended Next Steps

**I have not made any of these changes. This is a prioritized menu for you to choose from before I touch any code.**

### Critical (blocks "get my real UART data flowing through the real pipeline")
1. **Decide which architecture your physical setup actually matches**: (a) LiDAR → STM32 → PC raw serial (Path C — perception on PC), (b) LiDAR → STM32 → ESP32 → Wi-Fi, already-processed frames (Path B — perception on STM32, not applicable unless you have STM32 firmware doing full perception), or (c) something simpler that doesn't involve an STM32 at all. This determines everything else.
2. **Tell me your device's actual wire format**: what exactly arrives on the UART line — ASCII text (e.g. `"D:452,A:118\n"`), a fixed binary struct, framed with markers, one point per line vs. one full scan per message, units (mm/cm/m) for distance, degrees vs. some other angle encoding, baud rate, and whether there's any checksum. I will not guess this, per the project's own explicit rule and your instructions.
3. Only once (1) and (2) are known: implement a concrete parser (a real `LiDARMessageParser` subclass for Path C, or a new lightweight `SensorSource` if your setup doesn't fit the STM32-shaped abstraction at all) and the matching `.env` fields.

### High priority
4. Fix the `LidarCubes.cs` unit-conversion comment/assumption (§5.1) before trusting any visual output from the legacy serial path with real hardware — confirm your device's actual distance units against the `/10` decimeter assumption.
5. Resolve the two known test issues (`test_api.py` hang, the flaky pole test) so `pytest` becomes a trustworthy regression gate again.
6. Decide, and document, whether fusion/CAN-output are still in scope for your near-term goal, or purely future work — both are fully built but irrelevant until real radar/CAN hardware exists.

### Medium priority
7. Re-benchmark clustering/preprocessing/mapping if your real sensor's point density per scan differs meaningfully from the simulator's 360 points/rev.
8. Replace `Thread.Abort()` in `LidarSerialReader.cs` with a cooperative-cancellation pattern if you intend to keep using/extending that script.
9. Update `docs/architecture.md`'s stale "`pipeline/` is an empty placeholder" line now that `LiveStateBuilder` exists.

### Optional / future
10. Add a Dockerfile once the local dev workflow stabilizes.
11. Build out real alerting beyond the dashboard's event timeline, and historical-analytics views, once the hardware path is proven.

---

## My understanding of your current system (please verify)

- This is not a small hobby script — it's a mature, ~1000-plus-test, multi-language system (Python perception engine, Unity digital twin, FastAPI+PostgreSQL backend, React dashboard) that has been built and validated almost entirely against a **simulator**, not physical hardware.
- The rich pipeline (preprocessing → clustering → classification → tracking → TTC/clearance/risk → occupancy mapping) is real, tested, and works — but only ever against simulated or scripted-mock data.
- There is **no functioning code path today that turns real UART bytes into the pipeline's `LiDARPoint` format** — the two Python-side hardware adapters (`STM32Source`, `ESP32Source`) are both deliberately gated behind configuration/parsers that don't exist yet, by explicit project policy ("never guess a hardware protocol").
- There **is** a separate, older, self-contained Unity script (`LidarSerialReader.cs`) that opens a COM port directly and can display a raw point cloud from a simple `angle,distance` text line format — but it bypasses the entire Python perception pipeline (no clustering/tracking/collision) and assumes a specific, possibly-wrong distance unit (decimeters).
- I could not determine, from the repository alone, what your actual sensor's UART output format is, or which of the repo's several "hardware" code paths (if any) your physical setup is meant to match — I need that from you before writing any integration code, per both the project's own rule and your explicit "do not invent a protocol" instruction.
- The repository's own documentation (README, as last updated) states plainly and repeatedly that no physical sensor has ever been connected to this codebase and that hardware auto-detection returned nothing in that environment — if you now have a LiDAR physically talking over UART, that is new ground relative to everything documented here, and nothing downstream of "raw bytes" has been exercised against it yet.
- I found one concrete, likely-relevant bug candidate (the `/10` "cm → meters" comment/assumption in `LidarCubes.cs`, actually decimeters) that could produce a silently wrong scale if your real device's units don't match what that script assumes.
- No code, config, or docs were changed during this pass.

# ESP32 USB-Serial Integration (`DATA_SOURCE=esp32_serial`)

The in-service hardware path for LiDAR Vision 360. This document covers the wire format, the Edge
pipeline it feeds, every tunable, and how to diagnose a link that is not producing scans.

> **This is not the same thing as `DATA_SOURCE=hardware`.** See
> [Which hardware path is which](#which-hardware-path-is-which) below — picking the wrong one is
> the single most likely setup mistake.

---

## 1. Topology

```
LiDAR ──▶ STM32 ──▶ ESP32 ──▶ USB serial ──▶ Windows Edge PC ──▶ Dashboard
                               (COM port)      ALL perception
```

The STM32 and ESP32 firmware are unchanged and are not part of this repository. The ESP32's only
job on this link is to emit measurements; **every algorithm runs on the PC.**

Three processes make up the running system:

| Process | Command | Port |
|---|---|---|
| Edge pipeline | `python edge/main.py --mode live` | serves 5006 (JSON), 5005 (legacy raw) |
| Backend | `uvicorn backend.main:app --app-dir cloud/backend/src` | 8000 (REST + `/ws/live`) |
| Dashboard | `npm run dev` in `cloud/dashboard` | 5173 |

---

## 2. Wire format

One measurement per line:

```
A:45 , D:1200
```

* `A` — angle in **degrees**
* `D` — distance in **millimetres**

Whitespace around `:` and `,` is not significant. All of these are the same measurement:

```
A:45,D:1200      A:45 ,D:1200      A:45, D:1200      A:45 , D:1200
```

Parsing is case-insensitive and tolerates a trailing `\r` (CRLF line endings).

This format is encoded in exactly one file — [`perception/src/datasources/esp32_serial/parser.py`](../perception/src/datasources/esp32_serial/parser.py).
A firmware format change is a one-file change there.

### Validation

A line becomes a measurement only if it is well-formed **and** physically meaningful:

| Rule | Rejected as |
|---|---|
| Matches the `A:<num> , D:<num>` grammar exactly | `malformed` |
| `0 ≤ angle ≤ 360` (360 normalises to 0) | `angle_out_of_range` |
| `distance > 0` | `distance_not_positive` |

Rejected lines are **counted by reason and dropped**. Nothing is interpolated, substituted, or
invented for a bad line. Blank lines are ignored entirely and not counted (they are normal CRLF
artefacts, and counting them would drown the useful `malformed` signal).

Range filtering (`lidar_range_min_m` / `lidar_range_max_m`) is *not* done here — that is
preprocessing's job, and separately configurable. The parser's only question is "is this line a
real measurement, yes or no".

---

## 3. Coordinate convention

Identical in the parser, the pipeline, the wire protocol, the dashboard, and Unity:

```
distance_m = distance_mm / 1000
angle_rad  = radians(angle)

x = distance_m * cos(angle_rad)
y = distance_m * sin(angle_rad)
```

| Axis | Direction | Angle |
|---|---|---|
| **+X** | FRONT (vehicle forward) | 0° |
| **+Y** | LEFT | 90° |
| **−X** | REAR | 180° |
| **−Y** | RIGHT | 270° |

Angles are degrees in `[0, 360)`, measured **counter-clockwise** from forward. Distances are
**metres** internally — the millimetre→metre conversion happens once, in the parser, and never
again.

Worked example from the specification: `A:45 , D:1200` → `distance = 1.2 m`,
`x = 1.2·cos(45°) ≈ 0.8485`, `y = 1.2·sin(45°) ≈ 0.8485`. Asserted in
[`test_esp32_serial_pipeline.py`](../perception/tests/test_esp32_serial_pipeline.py).

---

## 4. Pipeline

```
ESP32 USB serial
   ↓  SerialLineReader      background thread, bounded queue, auto-reconnect
   ↓  MeasurementParser     "A:45 , D:1200" → (45.0°, 1.2 m), validated
   ↓  ScanFrameBuilder      angle-wrap detection → one complete ScanFrame per revolution
   ├──────────── everything below is the project's PRE-EXISTING pipeline, unchanged ────────────
   ↓  Preprocessor          range filtering, outlier removal, median/temporal filtering
   ↓  CoordinateTransformer polar → Cartesian (vectorised numpy)
   ↓  DBSCANClusterer       clustering on (x, y) + cluster feature extraction
   ↓  GeometricClassifier   shape/size/density → classification + confidence
   ↓  ObjectTracker         nearest-neighbour association + per-track Kalman → stable IDs, velocity
   ↓  OccupancyGridMapper   2D log-odds occupancy grid
   ↓  CollisionRiskEngine   TTC, projected path, SAFE/WARNING/CRITICAL + hysteresis
   ↓  ClearanceEngine       front/rear/left/right clearance, corridor width
   ↓  LiveStateBuilder      joins everything by track_id, records events, mints session identity
   ↓  PerceptionStreamServer  TCP :5006, newline-delimited versioned JSON
   ↓  cloud/backend           persists + rebroadcasts
   ↓  /ws/live → React dashboard
```

The three new stages sit **above** the `SensorSource` seam. Nothing downstream of `ScanFrame`
knows this link exists — which is exactly why LIVE and SIMULATION run byte-for-byte identical
processing.

---

## 5. Scan-boundary detection

The stream carries no scan delimiter, so a revolution boundary is *inferred* from the angle
wrapping (`359 → 0`). A naive `angle == 0` test is not usable against real firmware output, so
the rule is **"the angle jumped backwards by more than `wrap_threshold_deg`"**.

That single rule handles all of:

| Situation | Behaviour |
|---|---|
| Missing angles (`357 → 2`) | Still detected — any large backward step is a wrap |
| A scan that never emits angle 0 | Still detected |
| Jitter / out-of-order (`12.4 → 12.1`) | Below threshold — does **not** split the scan |
| Duplicate bearings in one revolution | Resolved by `duplicate_angle_policy` |
| Angle stops advancing (stalled motor) | Force-completed after `max_scan_duration_s`, with a warning |
| Wrap never arrives at all | Hard-capped at `max_points_per_scan` |
| Fewer points than `min_points_per_scan` | Discarded, not published — and counted |
| Reconnect mid-revolution | Pending scan dropped, so two half-sweeps are never merged |

**Scan rate is measured**, from the real wall-clock interval between consecutive completions —
never assumed from a configured target. Before the second scan it is `null`, not a fabricated
default.

`ScanFrame.timestamp` is the time the scan *completed* (its most recent measurement); each point
additionally carries the time its own line was read.

---

## 6. Configuration

Everything below is set in `.env` (prefix `LIDAR_`) or overridden on the command line. See
`.env.example` for the annotated block.

| Setting | Default | Meaning |
|---|---|---|
| `LIDAR_DATA_SOURCE` | `simulation` | Set to `esp32_serial` for this path |
| `LIDAR_ESP32_SERIAL_PORT` | `COM5` | **The one value you must set for your machine** |
| `LIDAR_ESP32_SERIAL_BAUDRATE` | `115200` | Must match the firmware |
| `LIDAR_ESP32_SERIAL_SOURCE_ID` | `esp32_serial` | Session/source label on every frame |
| `LIDAR_ESP32_SERIAL_READ_TIMEOUT_S` | `0.1` | Reader-thread blocking-read granularity |
| `LIDAR_ESP32_SERIAL_SCAN_TIMEOUT_S` | `5.0` | Wait for a complete revolution before reporting unavailable |
| `LIDAR_ESP32_SERIAL_RECONNECT_INITIAL_BACKOFF_S` | `1.0` | First retry delay |
| `LIDAR_ESP32_SERIAL_RECONNECT_MAX_BACKOFF_S` | `10.0` | Backoff ceiling |
| `LIDAR_ESP32_SERIAL_MAX_RECONNECT_ATTEMPTS` | unset | Unset = retry indefinitely |
| `LIDAR_ESP32_SERIAL_MAX_BUFFERED_LINES` | `20000` | Reader→pipeline queue depth |
| `LIDAR_ESP32_SERIAL_WRAP_THRESHOLD_DEG` | `180.0` | Backward jump that means "new revolution" |
| `LIDAR_ESP32_SERIAL_MIN_POINTS_PER_SCAN` | `10` | Below this, a boundary is a runt and is discarded |
| `LIDAR_ESP32_SERIAL_MAX_POINTS_PER_SCAN` | `5000` | Hard cap if a wrap never arrives |
| `LIDAR_ESP32_SERIAL_MAX_SCAN_DURATION_S` | `2.0` | Force-complete a stalled sweep |
| `LIDAR_ESP32_SERIAL_DUPLICATE_ANGLE_POLICY` | `last` | `last` \| `first` \| `keep_all` |
| `LIDAR_ESP32_SERIAL_LOG_EVERY_N_SCANS` | `20` | Health summary cadence (0 disables) |
| `LIDAR_STREAMING_POINT_MODE` | `polar` | `polar` \| `cartesian` \| `both` \| `none` — the map's point cloud needs one of the first three |

**Why 180° for the wrap threshold:** half a revolution — far above any plausible sampling jitter,
far below a full sweep. It detects a wrap even when the scan skips angle 0 entirely, and never
mistakes out-of-order arrival for one.

**Why `min_points_per_scan = 10`:** a handful of noise points is not a scan, and feeding one to
DBSCAN would produce meaningless clusters, objects, and risk. Lower it if your firmware genuinely
emits very few points per revolution.

---

## 7. Backpressure and threading

```
[reader thread]  blocking serial reads → bounded deque
                                            ↓
[pipeline thread]  drain → parse → assemble → full perception → publish
```

`pyserial` reads block, and the Edge loop must not stall on a quiet port — so a dedicated daemon
thread does the blocking reads and hands lines over through a bounded queue. Acquisition and
processing are decoupled: a slow scan never causes missed bytes, and a quiet sensor never freezes
the pipeline.

When the queue fills — the pipeline has fallen behind the sensor — the **oldest** lines are
dropped, not the newest. Stale measurements describe where the world *was*, and an unbounded
queue would grow until the process died while the dashboard fell further behind live. Drops are
counted in `ReaderStats.lines_dropped_backpressure` and warned about.

In LIVE mode the Edge's rate limiter is **disabled**: `read_scan()` returns exactly when a
revolution completes, so the sensor sets the pace. Sleeping on top of that would only add latency
and let the queue back up. `--rate` therefore applies to simulation only.

---

## 8. No fabricated data

There is no synthetic fallback anywhere on this path.

* If the port will not open, or no complete scan arrives within `scan_timeout_s`, `read_scan()`
  **raises**. The Edge logs a one-line warning, publishes `HARDWARE_DATA_UNAVAILABLE` (which the
  dashboard renders as an explicit "no hardware data" state), and keeps running.
* `get_sensor_source()` can only construct a `SimulatorSource` when `data_source == "simulation"`.
  There is no code path from `esp32_serial` to the simulator — the simulator package is not even
  imported on that branch.
* Scan rate, clearance, risk, TTC, object count and confidence are all computed from measurements.
  Where a value genuinely cannot be measured yet (e.g. scan rate before the second scan) the wire
  carries `null`, never a placeholder number.

---

## 9. Diagnostics

`ESP32SerialSource.stats` reports only counted values:

```python
{
  "reader": {"connected", "port", "baudrate", "lines_read", "bytes_read",
             "lines_dropped_backpressure", "partial_lines_discarded",
             "connect_attempts", "reconnect_count", "last_error", ...},
  "parser": {"lines_seen", "accepted", "rejected", "accept_ratio", "rejections_by_reason"},
  "frames": {"scans_completed", "scans_discarded_too_few_points",
             "scans_force_completed_timeout", "scans_force_completed_max_points",
             "duplicate_angles_resolved", "last_scan_point_count",
             "last_scan_duration_s", "measured_scan_rate_hz"},
  "buffered_lines": int,
}
```

The scan-timeout message is written from these counters, so it says *why* nothing arrived rather
than just "timed out":

| Symptom | Message says | Likely cause |
|---|---|---|
| Port never opened | `the serial port COM5 is not open (last error: ...)` | Not plugged in, wrong port, or another program (Arduino Serial Monitor, PuTTY) holds it |
| Port open, zero bytes | `the port is open but no data has arrived` | ESP32 not transmitting, or wrong baud rate |
| Lines arrive, none parse | `N line(s) arrived but none parsed as 'A:<deg> , D:<mm>'` | Wrong baud rate, or the firmware format changed |
| Boundaries found, all runts | `scan boundary/boundaries were detected but each had fewer than the required minimum points` | Lower `MIN_POINTS_PER_SCAN` |
| Measurements arrive, no revolution | `no full revolution completed in time` | LiDAR not spinning |

Also: `[SERIAL] Discarded N buffered bytes with no line terminator` is the classic
**wrong-baud-rate** signature — a torrent of bytes containing no newline.

---

## 10. Which hardware path is which

Three hardware adapters exist. Only one matches this topology.

| `DATA_SOURCE` | Adapter | Wire | Perception runs on | Status |
|---|---|---|---|---|
| **`esp32_serial`** | `datasources.esp32_serial` | ASCII `A:… , D:…` over **USB serial** | **the PC** | **Working — this document** |
| `hardware` | `datasources.esp32` | already-**processed** frames over Wi-Fi | the STM32 | Scaffolding; only a scripted mock transport exists |
| `hardware` (legacy) | `datasources.stm32` | binary UART frames | the PC | Framing/CRC/sequence implemented; the payload parser is a deliberate stub because that format is unknown |

`hardware` routes to the processed-frame edge loop and **bypasses the perception pipeline
entirely** — it expects finished objects/risk/clearance, not raw points. Selecting it for this
ESP32 will not work. The two paths were kept separate rather than merged precisely because they
are opposite architectures; the existing tests for both still pass unchanged.

---

## 11. Commands

```bash
# Find the COM port
python scripts/sniff_esp32.py --list

# LIVE
python edge/main.py --mode live
python edge/main.py --mode live --port COM7 --baud 115200
python edge/main.py --mode live --log-level DEBUG

# SIMULATION (no hardware)
python edge/main.py --mode simulation --scenario 08_approaching_obstacle

# Tests for this path
python -m pytest perception/tests/test_esp32_serial_parser.py \
                 perception/tests/test_esp32_serial_frame_builder.py \
                 perception/tests/test_esp32_serial_source.py \
                 perception/tests/test_esp32_serial_pipeline.py -v
```

---

## 12. Performance notes

* The pipeline was benchmarked at the simulator's ~360 points/revolution. If your firmware emits
  substantially more (several thousand per revolution), re-benchmark clustering and preprocessing:
  `sklearn`'s DBSCAN runs without a spatial index, which is comfortable at 360 points but scales
  super-linearly.
* Raise `LIDAR_STREAMING_MAP_EVERY_N_SCANS` (or set `LIDAR_STREAMING_POINT_MODE=none`) to shrink
  per-frame payloads if the WebSocket becomes the bottleneck.
* `performance_metrics.pipeline_processing_ms` on every frame is real measured stage-compute time,
  excluding network I/O — use it, not guesswork, to find the expensive stage.

# LiDAR Simulator

## Status

**Phase 2 — implemented.** Package: `simulator/` (installable as `lidar-vision360-simulator`,
depends on `perception` being installed first). Implements `datasources.LiDARDataSource` from
the perception foundation, so it is a drop-in stand-in for real hardware everywhere the pipeline
consumes a data source.

The minimal `perception.datasources.SimulatedLiDARDataSource` from Phase 0 still exists unchanged
(random distances, no geometry) -- it's a lightweight fallback with no dependency on `simulator/`.
Everything described below is the full, ray-casting simulator built in this phase.

## How it works, end to end

```
ScenarioSpec (JSON)  --resolve_scenario()-->  Environment + VehicleConfig + LiDARModel + NoiseModel
                                                        |
                                          SimulatedLiDARDataSource.read_scan()
                                                        |
                              for each sampled angle: ray-cast Environment -> true distance
                                          -> NoiseModel.apply() -> LiDARPoint
                                                        |
                                                    ScanFrame
```

1. A **scenario** (`simulator/scenarios/*.json`) declares obstacles and optional overrides for
   vehicle/LiDAR/noise parameters.
2. `resolve_scenario()` merges those overrides onto the global `common.config.Settings` defaults
   and builds concrete `Environment`, `VehicleConfig`, `LiDARModel`, `NoiseModel` objects.
3. `SimulatedLiDARDataSource.read_scan()` samples every angle in `LiDARModel.angles_deg()`,
   ray-casts it through `Environment` to find the nearest true obstacle distance, runs that
   through `NoiseModel`, and packages the result as a `models.scan.ScanFrame` of
   `models.lidar.LiDARPoint`s -- the exact same types every other data source produces.
4. Between calls to `read_scan()`, `Environment.step(dt)` advances any moving obstacles by
   `dt = 1 / scan_frequency_hz`.

## Coordinate system

Same convention as `docs/data-model.md`: the vehicle is fixed at the world origin `(0, 0)`,
facing `+x`. `+y` is to the vehicle's left. Angles are degrees, `[0, 360)`, measured
counter-clockwise from `+x`. All obstacle coordinates in scenario files are in this same
vehicle-relative frame, in meters.

The vehicle itself never moves in this simulator -- only obstacles do. Vehicle motion is out of
scope for this prototype (see Limitations).

## LiDAR model

`simulator.lidar_model.LiDARModel` turns a configured **angular resolution** (not a hard-coded
point count) into the sampled angle list:

```
num_points = round(360 / angular_resolution_deg)
angles = [i * 360/num_points for i in range(num_points)]
```

`angular_resolution_deg=1.0` (the default) gives the traditional 360 points; `0.5` gives 720;
any value works. Range limits (`range_min_m`, `range_max_m`) and `scan_frequency_hz` are also
part of this model. All of it comes from `common.config.Settings`
(`lidar_angular_resolution_deg`, `lidar_range_min_m`, `lidar_range_max_m`,
`lidar_scan_frequency_hz`) unless a scenario overrides it -- see Configuration below.

## Vehicle and LiDAR mounting

`simulator.vehicle.VehicleConfig` holds `width_m`, `length_m`, and the LiDAR's mount offset
(`lidar_x_m`, `lidar_y_m`) and mounting rotation (`lidar_orientation_deg`) relative to the
vehicle's forward axis. Defaults to mounted dead-center with no rotation
(`common.config.Settings.lidar_mount_x_m/_y_m/_orientation_deg`, all `0.0`). A ray sampled at
sensor angle `a` is cast in world direction `a + lidar_orientation_deg`; the *reported* point
still carries the raw sensor angle `a`, matching what a real sensor would report regardless of
how it's mounted.

## Ray-casting / obstacle geometry (`simulator.geometry`, `simulator.obstacles`)

Every obstacle answers one question: "what's the distance from this ray origin/direction to my
nearest surface, if any?" `Environment.true_distance()` asks every obstacle and keeps the
minimum -- this is a real geometric nearest-hit computation, not a random number.

| Obstacle | Shape | Method |
|---|---|---|
| `WallObstacle` | line segment (`p1`-`p2`) | ray/segment intersection (parametric, closed form) |
| `PoleObstacle` | circle (`center`, `radius`) | ray/circle intersection (quadratic, closed form) |
| `RectangleObstacle` | rectangle (`center`, `width`, `depth`, `rotation_deg`) | 4 edges as segments, nearest hit |

`width`/`depth` follow the same convention as `models.objects.DetectedObject`: `depth` is the
extent along the obstacle's own forward axis, `width` along its lateral axis, then the whole
rectangle is rotated `rotation_deg` (CCW) around its center. A "vehicle-like" obstacle is just a
`RectangleObstacle` with car-ish dimensions and an optional `tag` for human documentation --
no shape classification happens here (that's Phase 6).

`PoleObstacle` and `RectangleObstacle` optionally carry a constant `velocity` (m/s, `Vec2`);
`Environment.step(dt)` moves them. `WallObstacle` is always static.

## Noise model (`simulator.noise`)

`NoiseModel.apply(true_distance, range_min, range_max)` returns `(measured_distance, valid)`:

1. **Missing measurement**: with probability `missing_probability`, returns `(0.0, False)` --
   modeling a dropped/no return, distinct from "nothing in range."
2. **No obstacle in range** (`true_distance is None`): reported as `range_max_m`, `valid=True` --
   matching how real LiDARs typically report free space (max range), not "no data."
3. **Gaussian noise**: `distance += N(0, distance_noise_std_m)`.
4. **Outlier**: with probability `outlier_probability`, distance is replaced by a uniform random
   value in `[range_min, range_max]` (a spurious reflection).
5. Final distance is clamped to `[0, range_max_m]`.

All randomness for one `NoiseModel` instance flows through a single seeded `random.Random`, so a
full sequence of `.apply()` calls is exactly reproducible given the same `seed` --
`docs/testing.md` / `tests/test_noise.py::TestDeterminism` verify this directly, and
`tests/test_scenarios.py::TestDeterministicSeeding` verifies it end-to-end through a scenario.

## Moving obstacles

Configured per-obstacle in scenario JSON via a `moving` object: `velocity_mps` + `direction_deg`
(same angle convention as everything else). `SimulatedLiDARDataSource` advances the environment
by `dt = 1 / scan_frequency_hz` between scans (not on the very first scan, so scan #0 always
reflects the scenario's stated initial positions). This `dt` is the model's *nominal* interval --
it does not depend on wall-clock time, so behavior is identical whether or not `real_time=True`.

## Scenarios

Ten predefined scenarios live in `simulator/scenarios/*.json`, loaded via
`simulator.scenarios.list_scenarios()` / `get_scenario(id)`:

| id | Demonstrates |
|---|---|
| `01_empty` | No obstacles; every ray reports max range |
| `02_wall_in_front` | Straight wall, exact-distance ray casting |
| `03_pole_left` | Cylindrical obstacle detection |
| `04_vehicle_ahead` | Vehicle-like rectangular obstacle |
| `05_multiple_obstacles` | Several obstacle types/angles at once, nearest-hit logic |
| `06_narrow_corridor` | Two close parallel walls (tight clearance) |
| `07_moving_crossing` | A moving obstacle crossing the vehicle's path |
| `08_approaching_obstacle` | A moving obstacle approaching head-on (future TTC input) |
| `09_noisy_lidar` | Elevated Gaussian noise, fixed seed |
| `10_missing_outliers` | Elevated missing-measurement and outlier rates, fixed seed |

Each is a small JSON file (see any file in `simulator/scenarios/` for the exact shape): `id`,
`name`, `description`, optional `vehicle`/`lidar`/`noise` override blocks (any field left out
falls back to the global `Settings` default), and an `obstacles` list of `wall`/`pole`/`rectangle`
objects (see `simulator/src/simulator/scenario_schema.py` for the full pydantic schema).

Obstacle **layouts** live in these versioned JSON files rather than environment variables --
they're inherently spatial/structured data, not flat operational parameters. Every *operational*
parameter (range, noise, vehicle size, seed, ...) still flows through `common.config.Settings`
and can be overridden per-scenario or globally via `.env` -- nothing is hard-coded in the obstacle
geometry code itself.

## Data source abstraction

`simulator.datasource.SimulatedLiDARDataSource` implements `datasources.LiDARDataSource` (same
interface as the Phase 0 foundation). Build one directly, or use a scenario:

```python
from simulator import make_data_source

with make_data_source("02_wall_in_front") as source:
    frame = source.read_scan()
```

Nothing downstream needs to know this isn't hardware -- `RecordedLiDARDataSource` (below) and the
future `SerialLiDARDataSource` (Phase 16 placeholder, `perception/src/datasources/serial_source.py`,
still deliberately unimplemented) satisfy the exact same interface.

## Real-time mode

`SimulatedLiDARDataSource(..., real_time=True)` sleeps as needed inside `read_scan()` to pace
calls at `lidar_model.scan_frequency_hz`. Default is `real_time=False` (as-fast-as-possible),
appropriate for tests and batch recording. Obstacle motion (`dt`) is unaffected either way --
see "Moving obstacles" above.

## Recording

Format: **JSON Lines** (`.jsonl`) -- one line per `ScanFrame`, serialized with pydantic's own
`model_dump_json()` (`simulator.recording.save_scan_jsonl` / `save_scans_jsonl`). No separate
hand-rolled schema to keep in sync; every `ScanFrame`/`LiDARPoint` field is preserved (`scan_id`,
`sequence_number`, `source_id`, `timestamp`, and each point's `angle`, `distance`, `timestamp`,
`valid`, `intensity`), which is a superset of the minimum the spec asked for (timestamp, scan ID,
angle, distance).

Example line (pretty-printed here; actual file has one compact line per scan):

```json
{"scan_id": "8b145b48-...", "sequence_number": 0, "source_id": "simulated:08_approaching_obstacle",
 "timestamp": 1786455386.168, "points": [
   {"angle": 0.0, "distance": 7.7684, "timestamp": 1786455386.168, "valid": true, "intensity": null},
   ...
 ], "cartesian_points": null, "objects": null}
```

## Replay

`simulator.recording.RecordedLiDARDataSource(path, loop=False)` also implements
`LiDARDataSource`: `connect()` loads the whole file, `read_scan()` returns frames in original
order, `is_connected()` goes `False` once exhausted (unless `loop=True`, which wraps around
indefinitely). Because it satisfies the same interface, any code written against
`SimulatedLiDARDataSource` works unchanged against a replayed recording.

## Configuration

Global defaults live on `common.config.Settings` (see `.env.example`): `lidar_range_min_m`,
`lidar_range_max_m`, `lidar_angular_resolution_deg`, `lidar_scan_frequency_hz`,
`lidar_distance_noise_std_m`, `lidar_outlier_probability`, `lidar_missing_probability`,
`lidar_random_seed`, `vehicle_width_m`, `vehicle_length_m`, `lidar_mount_x_m`/`_y_m`/`_orientation_deg`.
Any scenario JSON file can override any of these for its own run without touching global config.
Nothing about range, resolution, noise, vehicle size, or obstacle placement is hard-coded in
Python.

## How to run it

Install (see root [README.md](../README.md) Quickstart) then:

```bash
# List scenarios
python -m simulator.cli list

# Run a scenario for N scans
python -m simulator.cli run --scenario 02_wall_in_front --scans 5

# Run in real time, paced at the scenario's scan frequency
python -m simulator.cli run --scenario 07_moving_crossing --scans 30 --real-time

# Show a live 2D debug plot (requires: pip install -e "./simulator[viz]")
python -m simulator.cli run --scenario 05_multiple_obstacles --scans 20 --visualize
```

Also installed as a console script: `lidar-simulate` (same subcommands).

## Recording and replaying data

```bash
# Record 50 scans of a scenario to a .jsonl file
python -m simulator.cli run --scenario 08_approaching_obstacle --scans 50 --record out.jsonl

# Replay it back (prints a summary per scan; --loop to repeat indefinitely)
python -m simulator.cli replay --file out.jsonl
```

## Visualization / debugging tool

`simulator.visualize` (optional `matplotlib` dependency, `pip install -e "./simulator[viz]"`)
draws a 2D top-down view: the vehicle as a rectangle at the origin, obstacles (walls as lines,
poles as circles, rectangles as polygons), and the current frame's LiDAR points converted to
Cartesian **locally within this plotting function only** (not the perception pipeline's Phase 4
coordinate-conversion stage, which does not exist yet). This is a debugging aid for validating
the simulator and, later, perception algorithms -- it does not replace the Phase 11 Unity digital
twin.

## Testing

`simulator/tests/`, run with `pytest simulator/tests` (or `pytest` from inside `simulator/`):

- `test_geometry.py` -- ray/segment and ray/circle intersection math, rectangle corner geometry
- `test_obstacles.py` -- per-obstacle-type distance calculation and motion
- `test_environment.py` -- empty environment, nearest-obstacle-wins, range limits, stepping
- `test_noise.py` -- Gaussian noise, outliers, missing measurements, determinism under a fixed seed
- `test_datasource.py` -- end-to-end scan generation, exact wall distance, moving-obstacle deltas
  between scans, real-time pacing
- `test_scenarios.py` -- all 10 scenarios load and produce valid scans; deterministic reproduction
  under a fixed seed; the missing/outlier scenario actually produces invalid points
- `test_recording.py` -- save/load/iterate `.jsonl`, replay in order, loop behavior

77 tests, run independently from `perception/tests`'s 24 (each subproject has its own
`pyproject.toml`/`testpaths`; running both directories in a single `pytest` invocation from the
repo root currently collides on the shared `tests` package name -- run them as two separate
`pytest` invocations, as shown above and in the root README).

## Assumptions and limitations

- The vehicle is always stationary at the world origin; only obstacles move. Vehicle motion
  (ego-motion) is out of scope for this prototype.
- One `ScanFrame`'s points all share a single timestamp (the moment `read_scan()` was called),
  i.e. a full 360° sweep is treated as instantaneous rather than modeling per-point acquisition
  time. Acceptable simplification for a 2D LiDAR at these scan rates; noted here for when
  Phase 16 hardware data may need finer-grained timestamps.
- Obstacles move at constant velocity with no collision response (they can pass through each
  other or through the vehicle's footprint); this simulator only produces sensor data; it does
  not enforce physical plausibility.
- "No obstacle within range" is reported as `range_max_m` with `valid=True`. This is a design
  choice (matching common real-sensor behavior) documented here, not an artifact.
- The 2D-only, no-elevation limitation from `PROJECT_SPECIFICATION.md` applies throughout: this
  simulates a 2D 360° LiDAR, not a 3D sensor.

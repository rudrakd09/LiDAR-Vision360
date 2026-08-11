# simulator

A configurable 2D 360° LiDAR scenario simulator for LiDAR-Vision360 (**Phase 2 — implemented**).
Ray-casts a configurable environment of obstacles (walls, poles, rectangles, moving obstacles)
and produces `ScanFrame`s in exactly the format the perception pipeline expects -- implementing
`perception`'s `LiDARDataSource` interface, so it's a drop-in stand-in for real hardware.

Full write-up: [`../docs/simulation.md`](../docs/simulation.md).

## Status

| Capability | Status |
|---|---|
| Configurable angular resolution (arbitrary, not hard-coded to 360) | ✅ |
| Ray-cast environment (walls, poles, rectangles/vehicle-like obstacles) | ✅ |
| Configurable vehicle + LiDAR mount pose | ✅ |
| Gaussian noise, outliers, missing measurements, seeded determinism | ✅ |
| Moving obstacles (constant velocity) | ✅ |
| 10 predefined scenarios (`simulator/scenarios/*.json`) | ✅ |
| `LiDARDataSource` implementation | ✅ |
| Real-time (paced) mode | ✅ |
| Recording to `.jsonl` | ✅ |
| Replay from `.jsonl` (`RecordedLiDARDataSource`) | ✅ |
| 2D debug visualization (optional matplotlib) | ✅ |
| CLI (`python -m simulator.cli` / `lidar-simulate`) | ✅ |

## Setup

Requires `perception` to be installed first (editable) -- the simulator imports its `models`,
`common`, and `datasources` packages directly. From the repository root:

```bash
pip install -e "./perception[dev]"
pip install -e "./simulator[dev]"        # add the `viz` extra for the debug plot:
pip install -e "./simulator[dev,viz]"
```

## Quick start

```bash
python -m simulator.cli list
python -m simulator.cli run --scenario 02_wall_in_front --scans 5
python -m simulator.cli run --scenario 07_moving_crossing --scans 20 --real-time --visualize
python -m simulator.cli run --scenario 08_approaching_obstacle --scans 20 --record out.jsonl
python -m simulator.cli replay --file out.jsonl
```

Or from Python:

```python
from simulator import make_data_source

with make_data_source("02_wall_in_front") as source:
    frame = source.read_scan()
    print(frame.point_count, min(p.distance for p in frame.points))
```

## Tests

```bash
pytest simulator/tests
```

77 tests covering geometry, obstacles, environment ray-casting, noise/determinism, end-to-end
scan generation, all 10 scenarios, and recording/replay. See
[`../docs/testing.md`](../docs/testing.md).

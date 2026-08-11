# PROJECT SPECIFICATION — LiDAR-Vision360

**Real-Time LiDAR-Based Vehicle Environment Perception, Digital Twin, Collision-Awareness and Cloud Monitoring System.**

This document is the canonical specification for the project. It is preserved as originally defined so
that every phase of implementation can be traced back to an explicit requirement. Implementation status
is tracked in [README.md](README.md) and in `docs/`; this file itself is not updated to reflect progress
(see "Status" markers there instead).

---

## 1. Core Project Concept

The final physical architecture will eventually be:

```
2D 360° LiDAR
    ↓
  STM32
    ↓
  UART
    ↓
PC / Edge Computer
    ↓
LiDAR Perception Engine
    ↓
    ├──────────────┬──────────────┐
    │              │              │
 Unity          Cloud          Alerts
 Digital Twin    Dashboard
    │              │
Environment    Historical
Visualization   Analytics
```

The LiDAR provides approximately 360 distance measurements corresponding to angular positions. Each
measurement can conceptually be represented as `(angle, distance)`. The system transforms these
measurements into meaningful environmental information.

## 2. Important Sensor Limitation

The current target sensor is a **2D 360° LiDAR**. Therefore:

- The system does **not** perform true 3D LiDAR reconstruction.
- Unity provides a **3D visualization / digital-twin representation of a 2D LiDAR-derived environment**.
- True 3D perception (e.g. via a 3D LiDAR or sensor fusion) is a **future extension**, out of scope for
  this prototype.

## 3. Main Objectives

The final prototype should support:

1. Simulated 360° LiDAR data generation.
2. Real LiDAR data integration through a future hardware adapter.
3. LiDAR data preprocessing.
4. Noise/outlier filtering.
5. Polar-to-Cartesian conversion.
6. Obstacle clustering.
7. Object/obstacle segmentation.
8. Geometric feature extraction.
9. Shape-based object classification.
10. Object IDs.
11. Object tracking.
12. Velocity estimation.
13. Occupancy grid mapping.
14. Collision-risk estimation.
15. Time-to-Collision estimation.
16. Low-clearance detection.
17. Vehicle safety zones.
18. Real-time Unity visualization.
19. Unity digital twin.
20. Cloud telemetry.
21. Real-time dashboard.
22. Historical analytics.
23. Safety alerts.
24. Sensor/system health monitoring.
25. Automated tests.
26. Documentation.

## 4. Development Principle

Build the system in stages. Do **not** begin with deep learning. First build a deterministic
geometry-based perception pipeline. Progression:

```
Simulated LiDAR → Filtering → Coordinate conversion → Clustering → Object properties →
Shape classification → Object tracking → Occupancy mapping → Collision detection →
Clearance analysis → Unity visualization → Cloud backend → Dashboard →
Hardware integration → Advanced ML / sensor fusion
```

## 5. Technology Stack

**Perception / simulation:** Python 3, NumPy, SciPy, scikit-learn, OpenCV (where useful), FastAPI,
Pydantic, pytest.

**Visualization:** Unity, C#.

**Cloud/backend:** Python FastAPI.

**Real-time communication:** WebSocket initially; MQTT can be added later as a hardware/IoT
communication layer.

**Database:** PostgreSQL.

**Dashboard:** React + TypeScript, with a suitable React charting library.

**Containerization:** Docker where useful.

**Hardware integration (later):** STM32, C/C++, UART.

## 6. Repository Architecture

```
LiDAR-Vision360/
├── perception/
│   ├── src/
│   │   ├── models/
│   │   ├── preprocessing/
│   │   ├── coordinates/
│   │   ├── clustering/
│   │   ├── objects/
│   │   ├── tracking/
│   │   ├── mapping/
│   │   ├── collision/
│   │   ├── clearance/
│   │   └── pipeline/
│   ├── tests/
│   ├── requirements.txt
│   └── README.md
│
├── simulator/
│   ├── src/
│   ├── scenarios/
│   └── tests/
│
├── unity/
│   └── LiDARVision360/
│
├── cloud/
│   ├── backend/
│   └── dashboard/
│
├── embedded/
│   └── stm32/
│
├── docs/
├── scripts/
├── docker/
├── .gitignore
├── README.md
└── PROJECT_SPECIFICATION.md
```

This structure is adapted with technically-justified additions as implementation proceeds; deviations
and their rationale are recorded in [docs/architecture.md](docs/architecture.md).

## 7. Phased Delivery Plan

- **Phase 0 — Foundation:** repo structure, docs skeleton, config, logging, canonical data models,
  data-source abstraction, base tests.
- **Phase 1 — Data model layer:** see Phase 0 (implemented together as the foundation).
- **Phase 2 — LiDAR simulator:** realistic simulated 360° scans, configurable environments/noise.
- **Phase 3 — Preprocessing:** range validation, outlier/noise filtering, temporal filtering.
- **Phase 4 — Coordinate transformation:** polar → Cartesian, documented convention.
- **Phase 5 — Clustering:** DBSCAN-based obstacle clustering with per-cluster geometric features.
- **Phase 6 — Object classification:** geometry-based shape classification with confidence scores.
- **Phase 7 — Object tracking:** nearest-neighbour association, then Kalman filtering.
- **Phase 8 — Occupancy grid:** FREE / OCCUPIED / UNKNOWN 2D grid, continuously updated.
- **Phase 9 — Collision engine:** minimum distance, safety zone, TTC, SAFE/WARNING/CRITICAL risk.
- **Phase 10 — Clearance engine:** front/rear/left/right clearance, corridor width, clearance states.
- **Phase 11 — Unity digital twin:** 3D visualization of the 2D-LiDAR-derived environment.
- **Phase 12 — Cloud backend:** FastAPI telemetry, REST + WebSocket, validation, health monitoring.
- **Phase 13 — Database:** PostgreSQL schema for vehicles, scans, objects, events, health.
- **Phase 14 — Dashboard:** React + TypeScript real-time and historical views.
- **Phase 15 — Alerts:** configurable collision/clearance/sensor/comms alerts.
- **Phase 16 — Hardware adapter:** `LiDARDataSource` abstraction; `SerialLiDARDataSource` placeholder
  for the future STM32 UART protocol (protocol intentionally not invented ahead of hardware spec).
- **Phase 17 — Testing:** unit + integration tests across all of the above.
- **Phase 18 — Documentation:** architecture, perception, data model, simulation, clustering,
  tracking, collision, unity, cloud, hardware-integration, testing docs.

## 8. Quality Requirements

The project must be modular, maintainable, testable, documented, configurable, extensible, and
hardware-independent during initial development.

Do **not** hard-code: file paths, vehicle dimensions, LiDAR range, safety thresholds, network
addresses, ports, or database credentials. Use environment/configuration files. Never commit secrets.

## 9. Development Rule

Work incrementally, one phase at a time. After each phase: run tests, run the relevant application,
verify the result, review the code, fix errors, update documentation, report what was completed and
what remains, and commit a Git checkpoint if appropriate. Do not advance to the next major phase if
the current phase is fundamentally broken.

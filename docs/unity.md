# Unity Digital Twin

## Status

**Implemented (Phase 11, extended in Phase 12)**, in `unity/LiDARVision360/`. Builds on the
Python perception pipeline through Phase 9 (Collision/Risk Engine); Phase 10 (clearance engine) is
also now implemented, and the HUD's clearance line shows real front/rear/left/right data -- see
docs/collision.md "Directional clearance" for the engine and `PerceptionDataTypes.ClearanceData`
for the Unity-side type. **Phase 12 replaced Phase 11's un-enveloped JSON messages with a versioned, structured envelope**
(`message_type`/`frame_id`/heartbeats/frame-ordering validation) -- see docs/communication.md for
the full protocol reference; this document covers Unity-side architecture and setup only.

**Important environment caveat**: this phase (like Phase 11) was implemented without access to a
Unity Editor or a C# compiler (no `.NET` SDK was available in the build environment either).
Every C# script was written carefully by hand, cross-checked for consistent types/references
across files, and brace/structure-verified, but **none of it has been compiled or run inside the
actual Unity Editor**. Treat this as thoroughly-reviewed source, not verified-working software,
until you've opened it in Unity yourself -- see "Troubleshooting" for what's most likely to need
a small fix on first import.

## Objective

Evolve the existing 360°-point-cloud Unity visualization into a 3D digital twin that displays the
*output* of the Python perception pipeline (classification, tracking, occupancy mapping,
collision risk) -- without ever running any perception algorithm inside Unity itself.

## Unity architecture

### Existing components (retained, unmodified)

| Script | Folder | Role |
|---|---|---|
| `LidarSerialReader.cs` | `Input/` | Reads `<START>`/`<END>`-framed scans from a serial COM port (115200 baud), background thread. |
| `LidarTCPClient.cs` | `Input/` | Reads the same framing from a raw TCP socket (`127.0.0.1:5005`). |
| `LidarCubes.cs` | `Visualization/` | Pre-instantiates 360 cubes once, repositions/recolors them per scan by distance. |
| `LidarBeep.cs` | `Audio/` | Red-zone continuous beep / yellow-zone variable-interval beep from raw minimum distance. |

**Copied byte-for-byte** into the new folder structure from the four `.cs` files supplied for
this phase -- not one line changed. They continue to work exactly as before, wired to each other
via the same public `LidarCubes`/`LidarBeep` field references they always used.

### New components

| Script | Folder | Role |
|---|---|---|
| `CoordinateConverter.cs` | `Core/` | The one centralized Python↔Unity coordinate transform -- see "Coordinate transformation". |
| `VehiclePoseSync.cs` | `Core/` | Applies Python's `vehicle` pose to the origin transform (currently a no-op -- see its own docstring). |
| `ILidarDataSource.cs` | `Input/` | Common scan-source abstraction (`LidarScanPoint`, `ConnectionState`, `ILidarDataSource`). |
| `PerceptionDataTypes.cs` | `Input/` | Newtonsoft.Json-serializable DTOs mirroring the Python JSON schema exactly. |
| `PerceptionTCPClient.cs` | `Input/` | Connects to the structured JSON protocol port, implements `ILidarDataSource`. |
| `LidarInputManager.cs` | `Input/` | Top-level mode selector: Legacy Serial / Legacy Raw TCP / Structured JSON TCP. |
| `LidarPointCloudRenderer.cs` | `Visualization/` | Improved, pooled, configurable point-cloud renderer (parallel to, not a replacement for, `LidarCubes.cs`). |
| `CoordinateAxesGizmo.cs` | `Visualization/` | Debug-mode coordinate axes. |
| `TrackedObjectView.cs` | `Objects/` | One tracked object's visual (shape/label/velocity vector/trajectory). |
| `TrackedObjectVisualizer.cs` | `Objects/` | Manages one `TrackedObjectView` per persistent Python `track_id`. |
| `TrajectoryRenderer.cs` | `Objects/` | Draws the current→predicted-position line for one object. |
| `OccupancyMapRenderer.cs` | `Mapping/` | Texture-based occupancy-grid rendering (not per-cell GameObjects). |
| `SafetyZoneRenderer.cs` | `Safety/` | Vehicle footprint / warning envelope / projected-path outlines, from Python's own config values. |
| `CollisionRiskIndicator.cs` | `Safety/` | Vehicle-level SAFE/WARNING/CRITICAL visual indicator. |
| `HUDController.cs` | `UI/` | Main on-screen HUD. |
| `DebugTogglesPanel.cs` | `UI/` | Debug-mode layer visibility toggles. |
| `CameraModeController.cs` | `CameraControl/` | Top-down / third-person camera switching. |
| `RiskAudioController.cs` | `Audio/` | Risk-driven audio for the structured pipeline (parallel to, not a replacement for, `LidarBeep.cs`). |
| `FrameIdValidator.cs` (Phase 12) | `Core/` | Duplicate/out-of-order/gap classification for incoming `frame_id`s -- mirrors `streaming.protocol.classify_frame_id` exactly. |

**Phase 12 also substantially rewrote** `PerceptionDataTypes.cs` (added `MessageEnvelope`,
`HeartbeatData`, `SystemStatusData`, `ErrorData`; `PerceptionFrameData` now represents the
envelope's `data` payload rather than a whole top-level message) and `PerceptionTCPClient.cs`
(envelope parsing/dispatch by `message_type`, frame-ID rejection via `FrameIdValidator`, staleness
detection) -- see docs/communication.md "Protocol" for the schema these implement. Every other
consumer script (`TrackedObjectVisualizer`, `OccupancyMapRenderer`, `SafetyZoneRenderer`,
`CollisionRiskIndicator`, `RiskAudioController`, `VehiclePoseSync`) needed **zero changes**: they
all subscribe to the same `OnPerceptionFrameReceived(PerceptionFrameData)` event signature as
Phase 11, which `PerceptionTCPClient` still fires with the unwrapped, already-validated payload --
only `HUDController.cs` gained a small addition (the new `Stale` connection state, and an
`OnErrorReceived` subscription).

### Data flow

```
                        ┌─ LegacySerial ──> LidarSerialReader ──┐
LidarInputManager ──────┼─ LegacyRawTcp ──> LidarTCPClient ─────┼──> LidarCubes ──> LidarBeep
 (mode select)          │                                       │    (unmodified, always available)
                        └─ StructuredJsonTcp ──> PerceptionTCPClient
                                                       │ OnScanReceived         OnPerceptionFrameReceived
                                                       ├──> LidarPointCloudRenderer
                                                       ├──> TrackedObjectVisualizer ──> TrackedObjectView (x N, by track_id)
                                                       │                                    └─> TrajectoryRenderer
                                                       ├──> OccupancyMapRenderer
                                                       ├──> SafetyZoneRenderer
                                                       ├──> CollisionRiskIndicator
                                                       ├──> HUDController
                                                       ├──> DebugTogglesPanel (controls the above's visibility)
                                                       ├──> RiskAudioController
                                                       └──> VehiclePoseSync
```

Python side: `scripts/serve_unity_bridge.py` runs the full pipeline (preprocessing → coordinates
→ clustering → classification → tracking → mapping → collision) against a simulator scenario and
serves two independent TCP servers -- the legacy raw port and the new structured JSON port -- see
"Python → Unity protocol".

## Important design decision (why Unity has no perception logic)

Unity never runs DBSCAN, Kalman filtering, TTC, or occupancy-grid math. Every visualization
component here is a thin renderer of values Python already computed and sent over the wire --
verified by construction: search every `.cs` file in this project and there is no clustering
loop, no filter/prediction step, no risk-threshold comparison against a *Unity-side* constant
(`SafetyZoneRenderer`/`CollisionRiskIndicator`/`HUDController` all read `risk_level`/`ttc`/
threshold *values* directly off the incoming frame, never compare against a number written into a
`.cs` file).

## Coordinate transformation

**The single source of truth is `Core/CoordinateConverter.cs`** -- every new script goes through
it; nothing else re-derives its own trig.

Python convention (`docs/coordinates.md`): `+X` forward, `+Y` left, degrees CCW from `+X`, meters.

This project's *existing* Unity convention, derived directly from `LidarCubes.cs`'s own working
code (`x = dist*cos(angleRad); z = dist*sin(angleRad)`), read line-for-line against Python's own
`x = dist*cos(angle); y = dist*sin(angle)`, is a **direct, unrotated relabeling**:

```
Python X (forward)  ->  Unity X
Python Y (left)      ->  Unity Z
Python has no Z       ->  Unity Y   (fixed height only -- 2D LiDAR, no elevation)
```

This was derived from the existing code, not assumed from "Unity convention usually has +Z
forward" -- per this phase's explicit "do not blindly change this" instruction. One consequence:
any vehicle/camera prefab built for this scene should visually face world `+X`, not Unity's
native `transform.forward` (`+Z`).

**Worked example**: a Python point at `angle=90°, distance=3m` (i.e. 3m to the vehicle's left) is
`x=0, y=3` in Python. `CoordinateConverter.PerceptionToUnity(0, 3, origin)` returns
`origin.TransformPoint(0, 0.25, 3)` -- 3m along the origin's local Z axis. With the origin
unrotated, that is Unity world `(0, 0.25, 3)`.

**Heading/yaw** (`CoordinateConverter.PerceptionHeadingToUnityYaw`): `yaw = -heading`, assuming
the target GameObject's own unrotated mesh visually points along local `+X`. **This assumption
could not be visually verified without a live Editor** -- if your prefab's authored "front" points
along a different axis, the resulting rotation will be off by a fixed amount; every script using
it exposes an adjustable offset. Verify by rotating one object in Play mode and comparing to the
Scene view.

**Occupancy map orientation** (`OccupancyMapRenderer`) has the same kind of unverified assumption
-- `flipRows`/`flipColumns` Inspector toggles are provided as a one-click fix if the rendered map
appears mirrored relative to the point cloud once you run both together.

## Python → Unity protocol

**See [docs/communication.md](communication.md) for the full, authoritative protocol reference**
(message envelope, all four message types, the complete perception-frame schema, framing,
versioning, heartbeat/staleness, frame-ID ordering, latency measurement) -- introduced in Phase
12 and not duplicated here. In short: two independent TCP ports, `streaming_raw_port` (default
`5005`) serving the unchanged legacy `<START>`/`<END>` protocol byte-compatible with
`LidarTCPClient.cs`/`LidarSerialReader.cs`, and `streaming_json_port` (default `5006`) serving a
versioned, enveloped, newline-delimited JSON protocol
(`PERCEPTION_FRAME`/`HEARTBEAT`/`SYSTEM_STATUS`/`ERROR`) built by
`perception/src/streaming/protocol.py` and parsed by `PerceptionTCPClient.cs` +
`PerceptionDataTypes.cs`.

## Vehicle model / scene contents

A scene should contain: the vehicle (simple primitives are fine -- this phase explicitly says not
to spend time on a detailed model), a LiDAR-mount reference point (can be the same transform as
the vehicle), a ground plane, the point cloud, tracked-object visuals, the occupancy map overlay,
safety-zone outlines, and the HUD. See "Setup" below for exact manual construction steps.

## Improved LiDAR point visualization

`LidarPointCloudRenderer.cs`: pooled points (sized once, like `LidarCubes.cs`'s own 360-cube
pool -- never `Instantiate`/`Destroy` per frame), configurable point size, a distance `Gradient`,
maximum display range, invalid-point filtering, optional persistence with configurable fade
duration. Colors via `MaterialPropertyBlock`, not `Renderer.material` (the latter silently
allocates a unique material instance per renderer on first access -- a real Unity performance
pitfall this script specifically avoids).

## Live scan update / input abstraction

`ILidarDataSource` (event `OnScanReceived`, property `State`) is the common abstraction, but the
existing `LidarSerialReader`/`LidarTCPClient` are **not** retrofitted to implement it -- both
hard-reference the concrete `LidarCubes`/`LidarBeep` classes in their own public fields, and
changing those field types would risk any Inspector wiring already set up. Instead,
`LidarInputManager` selects, at the top level, between the untouched legacy pipeline and the new
structured pipeline (`PerceptionTCPClient`, the one real `ILidarDataSource` implementation today,
ready for a future WebSocket/cloud source per this phase's own "allows future data sources" goal).

**Why Serial mode only ever drives the legacy point cloud**: the structured protocol's
objects/tracks/risk data is inherently a Python-perception-pipeline output -- raw serial hardware
alone was never going to produce it. `LidarInputManager.InputMode.LegacySerial` therefore only
ever routes to `LidarCubes`/`LidarBeep`, honestly reflecting what real hardware can actually
supply.

## Perception object visualization

`TrackedObjectView.cs` builds a primitive shape at runtime by classification (**visualization
proxies, not object reconstruction**):

| Classification | Shape |
|---|---|
| `vehicle_like` | Box (1.8 × 0.8 × 4.2) |
| `pole_like` | Cylinder (0.3 × 0.9 × 0.3) |
| `wall` | Elongated box (2.0 × 0.6 × 0.2) |
| `person_like` | Capsule |
| `large_obstacle` | Box (1.5 × 1.0 × 1.5) |
| `unknown` (or anything unrecognized) | Sphere |

Plus an in-world `TextMesh` label (track ID / classification / confidence / distance / speed),
built-in Unity component, no TextMeshPro dependency. Velocity vector and predicted-trajectory
line both come directly from Python's `velocity`/`predicted_position` fields -- **Unity never
computes velocity or a second prediction**.

## Persistent track IDs

`TrackedObjectVisualizer` keeps a `Dictionary<string track_id, TrackedObjectView>` -- the *same*
Unity GameObject is updated every scan a given `track_id` reappears in, never recreated, never
assigned a Unity-generated ID. A `track_id` absent from the stream for longer than
`removeAfterSeconds` (default `2s`) has its view destroyed -- Python's own tracker (see
docs/tracking.md) already handles the TENTATIVE/CONFIRMED/COASTING/LOST lifecycle; "missing from
the stream" is a faithful, sufficient proxy for "the track is gone" without Unity needing to
special-case the state name.

## Object movement / predicted trajectory

Every position/velocity/prediction value comes directly from the incoming frame; nothing is
computed in Unity. `TrajectoryRenderer` draws exactly the two points it's given (current,
predicted) -- configurable enable/color/width via the Inspector; no "duration" concept is needed
since it's redrawn fresh from real data every scan rather than animated.

## Occupancy map visualization / performance

`OccupancyMapRenderer.cs` renders the grid as **one textured quad**, not per-cell GameObjects
(which would mean up to 160,000 objects at full resolution -- explicitly forbidden by this
phase). Python downsamples before sending (`--map-downsample`, default 4× → 100×100 = 10,000
cells ≈ 13KB base64 per update) and only includes the map periodically (`--map-every-n-scans`,
default every 5th scan) rather than every frame; Unity keeps showing the last received map in the
scans between updates. The `Texture2D` instance itself is reused across updates (never
reallocated unless the map's own dimensions change).

## Collision risk / TTC / safety zones

`CollisionRiskIndicator.cs` recolors the vehicle body (green/orange/red) and optionally pulses a
`Light` on CRITICAL -- reading `risk.overall_risk` directly. `SafetyZoneRenderer.cs` draws the
vehicle body, the margin-expanded warning envelope, and the projected-path corridor as outlines,
sized entirely from the `config` field's vehicle-geometry/threshold values -- **no dimension or
threshold is hard-coded in any `.cs` file**. There is deliberately no separate static "critical
zone" shape: unlike the warning envelope, CRITICAL isn't a fixed rectangle in Python's own model
(it depends on TTC and the 2D collision simulation, not just static distance) -- see
docs/collision.md "Risk classification" -- so drawing one would imply a geometry Python doesn't
actually have. `HUDController.ttcText` shows the most critical object's TTC (`"N/A"` if `null`,
never a fabricated number).

## Audio

`LidarBeep.cs` is unmodified and keeps driving audio for the legacy Serial/raw-TCP pipeline
exactly as before. `RiskAudioController.cs` is a new, independent script for the structured
pipeline: SAFE → silent, WARNING → periodic beep, CRITICAL → continuous tone, reading Python's
`risk.overall_risk` directly. If the structured stream is connected but `risk` happens to be
`null` (collision stage not wired into a particular bridge run), it falls back to the same kind
of raw-minimum-distance zone logic `LidarBeep.cs` uses, rather than staying silent.

## User interface

`HUDController.cs` -- connection status, LiDAR scan rate (measured from frame arrival timing, not
reported by Python), object count, overall risk (colored), most-critical-object TTC/summary, and a
clearance line showing front/back/left/right distances, min clearance + direction, and corridor
width (colored by `overall_status`) once the Phase 10 clearance engine is wired in -- falls back to
`"CLEARANCE: N/A"` only if `clearance` is actually `null` on the wire (a caller not running that
stage). Uses built-in `UnityEngine.UI.Text` (no TextMeshPro dependency, matching `TrackedObjectView`'s own
choice for in-world labels).

## Camera

`CameraModeController.cs` switches between a top-down `Camera` and a third-person/isometric one
(`Tab` to cycle by default) -- both pre-positioned in the Editor; this script only enables one at
a time.

## Debug mode

`DebugTogglesPanel.cs` -- one `Toggle` per layer (raw points, classified objects, track ID
labels, velocity vectors, predicted trajectories, occupancy map, safety zones, clearance,
coordinate axes). Each toggle calls `GameObject.SetActive` on the relevant layer (not just the
component's `enabled` flag, which would stop updates but leave already-rendered visuals stuck on
screen).

## Data validation

Handled throughout, not bolted on afterward: `PerceptionTCPClient` catches and logs (never
throws past) a malformed JSON line and continues reading the next one; `TrackedObjectVisualizer`
skips any object with a missing `track_id`; `LidarPointCloudRenderer` filters `NaN`/`Infinity`
angle or distance values and out-of-range points; `OccupancyMapRenderer` validates the decoded
byte-array length against the declared cell count before touching the texture; every renderer
treats a `null` optional field (`velocity`, `risk`, `map`, `clearance`, `predicted_position`) as
"nothing to show right now," never a crash.

## Connection status / threading

`PerceptionTCPClient.State` (`Disconnected`/`Connecting`/`Connected`/`Reconnecting`) drives
`HUDController`'s system-status line. Both `PerceptionTCPClient` and the existing
`LidarSerialReader` read on a background thread and only touch Unity APIs / fire events from
`Update()` (main thread) -- `PerceptionTCPClient` auto-retries on disconnect
(`reconnectDelaySeconds`, default 2s) rather than giving up.

## Setup

### 1. Install Unity and create the project

This repository does not (and, per this phase's own scope, should not) include Unity's own
generated project files (`ProjectSettings/`, `Packages/manifest.json`, `.unity` scenes) --
building those by hand without a live Editor to validate them would be far riskier than having
Unity itself generate them. Install **Unity 2021 LTS or newer** (2022 LTS recommended) via Unity
Hub, then:

1. Unity Hub → New Project → **3D (Built-In Render Pipeline)** template (the shaders referenced
   below -- `Sprites/Default`, `Unlit/Transparent` -- are Built-in-RP shader names; if you use
   URP/HDRP instead, swap them for the URP/HDRP-unlit equivalents in `SafetyZoneRenderer.cs`/
   `OccupancyMapRenderer.cs`).
2. Name it `LiDARVision360` and create it **at** `unity/LiDARVision360/` in this repo (so the
   `Assets/Scripts/` folder already checked in lands inside Unity's own `Assets/` folder).
3. If Unity created its own `Assets/Scripts/` placeholder, merge/overwrite with the one from this
   repo (they should be the same path).

### 2. Install Newtonsoft.Json

Required by `Input/PerceptionDataTypes.cs` and `PerceptionTCPClient.cs` (Unity's built-in
`JsonUtility` can't represent this schema's many optional/nullable fields or top-level arrays).

Window → Package Manager → `+` → **Add package by name** → `com.unity.nuget.newtonsoft-json` →
Add. (This is Unity's own official redistribution of Newtonsoft.Json, not a third-party asset.)

### 3. Scene construction

Create a new Scene (`Assets/Scenes/Main.unity`). Build this hierarchy (empty GameObjects unless
noted):

```
Main (Scene)
├── Vehicle                        <- empty GameObject; this is CoordinateConverter's "origin"
│   ├── VehicleBody (Cube primitive, scale ~1.8 x 0.8 x 4.5, matching config.vehicle_width_m/length_m)
│   │   └── (add CollisionRiskIndicator here, targetRenderer = this Cube's Renderer)
│   └── (add VehiclePoseSync here, client = LidarInputManager/StructuredPerception/PerceptionTCPClient)
├── Ground (Plane primitive, scaled up, e.g. 50x50)
├── LidarInputManager (empty GO)
│   └── add LidarInputManager component
├── LegacySerialRoot (empty GO, initially inactive)
│   └── add LidarSerialReader component
├── LegacyTcpRoot (empty GO, initially inactive)
│   └── add LidarTCPClient component
├── LegacyVisualizationRoot (empty GO)
│   ├── add LidarCubes component (cubePrefab = a small Cube prefab, center = Vehicle transform)
│   └── add LidarBeep component (audioSource = an AudioSource on this or a child)
├── StructuredPerceptionRoot (empty GO)
│   ├── add PerceptionTCPClient component
│   ├── PointCloud (empty GO)
│   │   └── add LidarPointCloudRenderer (dataSourceBehaviour = PerceptionTCPClient above,
│   │       origin = Vehicle, pointPrefab = a small Sphere/Cube prefab)
│   ├── TrackedObjects (empty GO)
│   │   └── add TrackedObjectVisualizer (client = PerceptionTCPClient, origin = Vehicle,
│   │       trackedObjectViewPrefab = see Prefabs below)
│   ├── OccupancyMap (Quad primitive, rotated 90° on X so it lies flat)
│   │   └── add OccupancyMapRenderer (client = PerceptionTCPClient, origin = Vehicle)
│   ├── SafetyZones (empty GO)
│   │   └── add SafetyZoneRenderer (client = PerceptionTCPClient, origin = Vehicle)
│   ├── RiskAudio (empty GO)
│   │   └── add RiskAudioController (client = PerceptionTCPClient, audioSource = an AudioSource)
│   └── add CollisionRiskIndicator's client field pointing here too (if not already wired above)
├── Cameras (empty GO)
│   ├── TopDownCamera (Camera, positioned high above Vehicle looking straight down)
│   ├── ThirdPersonCamera (Camera, positioned behind/above Vehicle)
│   └── add CameraModeController on this or another GO (topDownCamera / thirdPersonCamera wired)
├── CoordinateAxes (empty GO)
│   └── add CoordinateAxesGizmo (origin = Vehicle)
└── HUD (Canvas, Screen Space - Overlay)
    ├── SystemStatusText, LidarInfoText, ObjectsText, RiskText, TtcText, MostCriticalText,
    │   ClearanceText (UI > Text elements, positioned as desired)
    ├── DebugTogglesPanel (empty GO under Canvas, with UI > Toggle children)
    │   └── add DebugTogglesPanel component, wire targets + toggles
    └── add HUDController component (client = PerceptionTCPClient, wire each Text field)
```

Wire `LidarInputManager`'s `legacySerialRoot`/`legacyTcpRoot`/`legacyVisualizationRoot`/
`structuredPerceptionRoot` fields to the matching GameObjects above.

### 4. Prefabs

- **Point prefab** (for `LidarCubes.cubePrefab` and `LidarPointCloudRenderer.pointPrefab`): a
  small Cube or Sphere primitive with its `Collider` removed (optional but recommended --
  visualization only), dragged from the Hierarchy into `Assets/Prefabs/`.
- **Tracked object view prefab** (for `TrackedObjectVisualizer.trackedObjectViewPrefab`): an
  empty GameObject with a `TrackedObjectView` component and a `LineRenderer` component (the
  `[RequireComponent(typeof(LineRenderer))]` attribute adds the latter automatically the moment
  you add `TrackedObjectView`) -- optionally also add a child `TrajectoryRenderer` GameObject
  (also needs its own `LineRenderer`) and wire `TrackedObjectView.trajectoryRenderer` to it.
  Drag into `Assets/Prefabs/`.

### 5. Materials

Not strictly required -- `OccupancyMapRenderer` and `SafetyZoneRenderer` both create their own
`Material` at runtime (`Shader.Find(...)`, see "Setup" step 1's render-pipeline caveat). If you
want a specific look for `LidarCubes`/`LidarPointCloudRenderer` points or the vehicle body, create
standard `Assets/Materials/*.mat` assets and assign them to the relevant prefabs/renderers as you
would for any Unity project.

### 6. TCP setup

Default: `127.0.0.1:5005` (legacy raw, matches the existing `LidarTCPClient.cs` hard-coded
value) and `127.0.0.1:5006` (structured JSON, `PerceptionTCPClient.port`). Both configurable via
their respective component's Inspector fields, and via `scripts/serve_unity_bridge.py --raw-port
--json-port` on the Python side -- keep both sides matching.

### 7. Serial setup

`LidarSerialReader.portName` (default `COM3`) / `.baudRate` (default `115200`) -- set to match
your actual hardware's COM port in the Inspector, same as before this phase.

### 8. Demo mode (no real hardware required)

```bash
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --vehicle-speed 0 --rate 10
```

Then in Unity: set `LidarInputManager.mode = StructuredJsonTcp`, press Play. See "End-to-end
demo" in the completion report for the full walkthrough including verification steps.

### 9. Running the EditMode tests (Phase 12)

`Assets/Tests/EditMode/` contains `FrameIdValidatorTests.cs` and
`PerceptionProtocolParsingTests.cs` (NUnit, Unity Test Framework -- see docs/communication.md
"Testing"). Requires the `com.unity.test-framework` package (included by default in the 3D
template; Package Manager → confirm "Test Framework" is present if not). Window → General → Test
Runner → EditMode tab → Run All. **Not run in this environment** -- see "Status".

## Troubleshooting

- **Scripts don't compile**: the most likely first-import issues are (a) the Newtonsoft.Json
  package not yet installed (step 2 above) -- `PerceptionDataTypes.cs`/`PerceptionTCPClient.cs`
  won't compile without it; or (b) the `LiDARVision360.Runtime.asmdef` under `Assets/Scripts/`
  (added Phase 12, so `Assets/Tests/EditMode/`'s own asmdef has something concrete to reference)
  failing to resolve `Newtonsoft.Json` as an assembly reference -- confirm the package installed
  correctly first. Everything else uses only core `UnityEngine`/`UnityEngine.UI`.
- **EditMode tests don't appear in Test Runner**: confirm `com.unity.test-framework` is installed
  and both `.asmdef` files under `Assets/Scripts/` and `Assets/Tests/EditMode/` imported without
  errors (check the Console).
- **Map or safety zones look mirrored**: toggle `OccupancyMapRenderer.flipRows`/`flipColumns`, or
  compare object positions against `SafetyZoneRenderer`'s outlines directly -- see "Coordinate
  transformation"'s own caveat about unverified orientation assumptions.
- **A rotated/oriented object (heading arrow, vehicle) points the wrong way**: adjust the
  relevant script's yaw-offset handling per `CoordinateConverter.PerceptionHeadingToUnityYaw`'s
  own documented assumption.
- **Nothing appears, HUD shows "Disconnected"**: confirm `scripts/serve_unity_bridge.py` is
  running and the host/port match; check the Unity Console for `PerceptionTCPClient` warnings.
- **HUD shows "Stale (no data)"**: the socket is open but no frame or heartbeat arrived within
  `connectionTimeoutSeconds` -- check the Python process is still running and not stuck (an
  `ERROR` message, if one arrived, appears appended to the status line for 10s).
- **Shader not found / pink materials**: you're likely using URP/HDRP instead of the Built-in
  Render Pipeline -- see "Setup" step 1.
- **HUD clearance line stays "CLEARANCE: N/A"**: confirm the bridge process is actually running
  `ClearanceEngine` (default in `scripts/serve_unity_bridge.py` since Phase 10 landed -- check the
  Python process wasn't started from an older checkout); a real, connected stream should never
  send a `null` `clearance` field today. If distances look implausible (e.g. permanently near-zero
  in every direction), double check `VehiclePoseSync`/the vehicle's identity pose assumption -- see
  "Known limitations."

## Performance

Not measured in a live Editor (no Unity install in the build environment) -- every allocation-
avoidance requirement from this phase's spec was followed by construction (pooled points, pooled
per-track-id objects, `MaterialPropertyBlock` instead of `Renderer.material`, a single reused
`Texture2D` for the map, no `Instantiate`/`Destroy` in any per-frame code path), but actual
FPS/memory numbers require running this in the Editor. The **network/serialization side** of
performance (message size, serialization time, throughput, dropped frames, end-to-end latency
excluding Unity's own render step) *is* measured -- see docs/communication.md "Performance" for
the real numbers.

## Known limitations

- **Not compiled/run in a live Unity Editor** -- see "Status" above. Review before treating as
  verified-working.
- **Coordinate/texture orientation assumptions** (heading yaw, occupancy-map row/column order)
  could not be visually verified -- adjustable escape hatches are provided (see "Troubleshooting").
- **Vehicle pose is always identity** in this project's current scope -- `VehiclePoseSync` is
  wired and ready but has nothing non-trivial to apply yet.
- **Fixed height, 2D data only** -- consistent with the whole project's 2D-LiDAR scope (see
  docs/architecture.md "Sensor limitation"); no elevation is ever rendered.
- **Render-pipeline-specific shader names** (`Sprites/Default`, `Unlit/Transparent`) assume the
  Built-in Render Pipeline; swap for URP/HDRP equivalents if your project uses one of those.

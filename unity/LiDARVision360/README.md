# Unity LiDAR-Vision360 Digital Twin

**Phase 11 -- implemented.** See [docs/unity.md](../../docs/unity.md) for the full architecture,
protocol reference, and step-by-step setup instructions, and
[PROJECT_SPECIFICATION.md](../../PROJECT_SPECIFICATION.md) for the overall project plan.

## What's here

`Assets/Scripts/` contains every C# script for this phase, organized by concern (`Core/`,
`Input/`, `Visualization/`, `Objects/`, `Mapping/`, `Safety/`, `UI/`, `CameraControl/`, `Audio/`)
-- including the four pre-existing scripts (`LidarSerialReader.cs`, `LidarTCPClient.cs`,
`LidarCubes.cs`, `LidarBeep.cs`), copied in unmodified.

**The Unity project itself (scenes, prefabs, materials, `ProjectSettings/`,
`Packages/manifest.json`) is not included here** -- generating those by hand without a live Unity
Editor to validate them would be far riskier than letting Unity itself create them. See
[docs/unity.md](../../docs/unity.md) "Setup" for exact steps: create a new Unity 3D project at
this path, install the `com.unity.nuget.newtonsoft-json` package, then build the scene hierarchy
described there and wire up the components already written in `Assets/Scripts/`.

## Quick start (once the Unity project exists)

```bash
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --vehicle-speed 0 --rate 10
```

Then in Unity, set `LidarInputManager.mode = StructuredJsonTcp` and press Play.

## Important caveat

This phase's C# was written and reviewed carefully but **has not been compiled or run inside an
actual Unity Editor** (none was available in the environment it was built in). See
[docs/unity.md](../../docs/unity.md) "Status" and "Troubleshooting" before treating it as
verified-working.

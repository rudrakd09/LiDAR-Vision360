# LiDAR Simulator

## Status

Not yet implemented. Planned for **Phase 2**.

A minimal placeholder, `perception.datasources.SimulatedLiDARDataSource`, exists today (Phase 0)
purely to make the `LiDARDataSource` abstraction runnable — it emits evenly-spaced angles with
uniform-random distance and light Gaussian noise, nothing configurable beyond point count.

The real simulator (`simulator/`) will support configurable environments (walls, poles,
rectangular obstacles, vehicle-like objects, multiple/moving obstacles, narrow corridors),
configurable LiDAR range, point count, noise, outliers, missing measurements, and scan frequency,
producing output in the exact `ScanFrame`/`LiDARPoint` format the perception engine expects. This
document will be filled in when Phase 2 starts.

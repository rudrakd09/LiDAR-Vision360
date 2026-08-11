# LiDAR Simulator

Not yet implemented. Planned for **Phase 2**: configurable 360° scenario generation (walls,
poles, rectangular obstacles, vehicle-like objects, multiple/moving obstacles, narrow corridors)
with configurable range, point count, noise, outliers, missing measurements, and scan frequency —
producing output in the same `ScanFrame`/`LiDARPoint` shape as `perception/src/models`.

See [docs/simulation.md](../docs/simulation.md). In the meantime, a minimal placeholder,
`perception.datasources.SimulatedLiDARDataSource`, is available for foundation-level development.

# Unity Digital Twin

## Status

Not yet implemented. Planned for **Phase 11** (`unity/LiDARVision360/`).

**Reminder of the sensor limitation:** the source sensor is a 2D 360° LiDAR. Unity will render a
3D visualization/digital-twin *representation* of a 2D-LiDAR-derived environment — this is not
true 3D reconstruction.

Will document: how Unity consumes perception output (initially simulated, later real
hardware-derived data, same schema either way per `docs/data-model.md`); the communication
mechanism (WebSocket, per the project's real-time-communication choice); and what is rendered
(vehicle, LiDAR points, obstacles, object IDs/classification/confidence/distance, bounding boxes,
occupancy map, safety zones, velocity vectors, trajectories, collision warnings, clearance
status).

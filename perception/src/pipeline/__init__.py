"""Top-level pipeline orchestration: wires a `LiDARDataSource` through preprocessing, coordinates,
clustering, objects, tracking, mapping, collision, and clearance into a single per-frame call.

Status: not yet implemented. Will be assembled incrementally as each stage above lands, and is
what `cloud/backend` and the Unity bridge will ultimately drive. See PROJECT_SPECIFICATION.md.
"""

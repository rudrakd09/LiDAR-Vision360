# Perception Engine

## Status

Foundation only (Phase 0): package structure, data models, config/logging, and the
`LiDARDataSource` abstraction exist. The actual perception logic (preprocessing through
clearance) has not been implemented yet.

This document will describe the end-to-end perception pipeline (preprocessing → coordinates →
clustering → objects → tracking → mapping → collision → clearance) once those phases (3-10) are
implemented. See [PROJECT_SPECIFICATION.md](../PROJECT_SPECIFICATION.md) for the planned scope of
each stage, and [architecture.md](architecture.md) for the current package layout.

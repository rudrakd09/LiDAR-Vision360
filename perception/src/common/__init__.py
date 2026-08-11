"""Cross-cutting infrastructure shared by every pipeline stage: configuration and logging.

This package is a technically-justified addition to the repository layout proposed in
PROJECT_SPECIFICATION.md (which lists models/preprocessing/.../pipeline but not a shared
infrastructure package). Config and logging are needed by every stage from Phase 0 onward, so
they live here rather than being duplicated or bolted onto an unrelated subsystem. See
docs/architecture.md for the rationale.
"""

"""Top-level pipeline orchestration: wires a `LiDARDataSource` through preprocessing, coordinates,
clustering, objects, tracking, mapping, collision, and clearance into a single per-frame call.

Status: **stage-execution orchestration itself is not yet implemented** -- each stage is still
wired up directly by its own caller (`scripts/serve_unity_bridge.py`); see docs/architecture.md
"Repository layout" for why that remains reasonable today.

`live_state.py` is a **related but distinct** concern that *is* implemented: aggregating each
stage's already-computed output into one canonical `models.live_state.LiveState` object (the
Edge's single source of truth -- see docs/architecture.md "LiveState"). It does not drive stage
execution and runs no perception algorithm of its own -- see `LiveStateBuilder`'s own docstring.
"""

from .live_state import LiveStateBuilder

__all__ = ["LiveStateBuilder"]

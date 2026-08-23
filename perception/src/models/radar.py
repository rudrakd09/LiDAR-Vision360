"""Radar reading models -- for the R121 CAN radar's data as received via the STM32 link.

Deliberately NOT part of `ScanFrame`/`SensorFrame` and NOT consumed anywhere in the existing
perception pipeline (preprocessing/clustering/tracking/collision/clearance all remain
LiDAR-only). PROJECT_SPECIFICATION.md's "Important Sensor Limitation" section and this codebase's
own `pipeline.live_state._sensor_status` are both explicit that sensor fusion (radar/camera) is
"a future extension, out of scope for this prototype" -- see that function's `"radar": None`
comment. This module exists so `STM32Source` has somewhere correctly-typed to put a parsed R121
reading (satisfying "radar message parser interface" / "conversion into SensorFrame" from the
Phase 8 architecture ask) without silently smuggling radar detections into the LiDAR-only
detection/clustering/tracking path. `STM32Source.latest_radar_reading` exposes the most recent one
for future use (e.g. richer `sensor_status` reporting) -- see docs/hardware-integration.md
"Hardware integration checklist" for the explicit follow-up decision needed before any fusion is
implemented.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class RadarTarget(BaseModel):
    """One detected radar target. Field set is intentionally minimal/generic (range, optional
    velocity, angle, confidence, target ID) -- the R121's actual CAN payload layout is not yet
    known; once it is, this model is extended (or a mapping layer added) to carry whatever it
    actually reports, without needing to change anything downstream of
    `STM32Source.latest_radar_reading` or `fusion.FusionEngine` (Phase 9), both of which already
    treat every field but `range_m` as optional/"when available".

    `confidence`/`target_id` added in Phase 9 (`fusion.FusionEngine` requirement: radar must be
    able to contribute range/angle/velocity/confidence/target ID *when those fields are
    available*) -- additive, `None` by default, same pattern as every other placeholder field on
    this model.
    """

    range_m: float = Field(..., ge=0.0, description="Target range in meters.")
    velocity_mps: float | None = Field(
        default=None,
        description="Range-rate (d(range)/dt), if the radar reports it. Convention this project "
        "defines and uses internally (not yet confirmed against real hardware): negative = "
        "closing (range decreasing), positive = receding (range increasing) -- see "
        "fusion.association's radial-velocity handling, which depends on this sign convention.",
    )
    angle_deg: float | None = Field(default=None, ge=0.0, lt=360.0, description="Bearing angle in degrees, same convention as LiDARPoint.angle (CCW from vehicle forward axis), if the radar reports it.")
    confidence: float | None = Field(default=None, ge=0.0, le=1.0, description="Radar-reported detection confidence, 0-1, if it reports one.")
    target_id: str | None = Field(default=None, description="The radar's own persistent target/track identifier, if it reports one -- lets fusion.FusionEngine preserve identity for a radar-only detection across scans. Without it, a radar-only detection gets a fresh identity every scan (no cross-frame continuity is possible from range/angle alone).")


class RadarReading(BaseModel):
    """One R121 report as received via the STM32 link -- zero or more targets."""

    source_id: str
    sequence_number: int
    timestamp: float
    targets: list[RadarTarget] = Field(default_factory=list)

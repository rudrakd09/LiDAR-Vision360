"""Canonical raw and Cartesian LiDAR point models.

These are the lowest-level data structures in the perception pipeline. Every LiDAR data source
(simulated or real) must ultimately produce :class:`LiDARPoint` instances so that everything
downstream (preprocessing, coordinate conversion, clustering, ...) is hardware-independent.

Coordinate / angle convention (see docs/data-model.md and docs/coordinates.md for the full
write-up once Phase 4 lands):

- ``angle`` is in **degrees**, measured counter-clockwise from the vehicle's forward axis,
  in the range ``[0, 360)``.
- ``distance`` is in **meters**.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class LiDARPoint(BaseModel):
    """A single raw polar measurement from a 360° LiDAR scan.

    This is the format every :class:`~datasources.base.LiDARDataSource` implementation
    (simulated or real) must emit, so the rest of the pipeline never depends on where the
    data actually came from.
    """

    model_config = ConfigDict(frozen=False)

    angle: float = Field(
        ..., ge=0.0, lt=360.0, description="Angle in degrees, [0, 360), CCW from vehicle forward axis."
    )
    distance: float = Field(..., ge=0.0, description="Measured distance in meters. 0 means no return.")
    timestamp: float = Field(..., description="Unix epoch timestamp (seconds, float) of the measurement.")
    valid: bool = Field(default=True, description="False if this measurement is flagged invalid (out of range, dropout, ...).")
    intensity: float | None = Field(default=None, ge=0.0, description="Optional return-signal intensity, if the sensor provides it.")


class CartesianPoint(LiDARPoint):
    """A :class:`LiDARPoint` after polar → Cartesian conversion (Phase 4).

    Retains the original polar fields (``angle``, ``distance``) alongside the derived
    Cartesian coordinates so downstream consumers never need to re-derive one from the other.
    """

    x: float = Field(..., description="Cartesian X in meters, x = distance * cos(radians(angle)).")
    y: float = Field(..., description="Cartesian Y in meters, y = distance * sin(radians(angle)).")

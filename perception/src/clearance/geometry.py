"""Directional-quadrant clearance geometry: classify a point into front/rear/left/right relative
to the vehicle's own heading, and compute the gap from the vehicle's safety-margin envelope edge
to the nearest return in each quadrant.

Reuses `collision.geometry.to_vehicle_frame`/`vehicle_footprint` (the exact same vehicle-local
`(along, lateral)` frame and footprint/margin envelope `collision.CollisionRiskEngine` already
established) rather than duplicating them -- this phase's four directions are literally that same
frame's four quadrants, so there is exactly one vehicle-frame transform and one footprint model in
this project, not two independently-derived ones. See docs/collision.md "Directional clearance".
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from collision.geometry import VehicleFootprint, to_vehicle_frame
from common.config import Settings
from models.clearance import ClearanceDirection
from models.collision import VehicleState
from models.coordinates import CartesianScan
from models.objects import Point2D


def classify_quadrant(along: float, lateral: float) -> ClearanceDirection:
    """Four non-overlapping 90-degree quadrants centered on the vehicle's own local axes
    (front=0deg, left=90deg, rear=+-180deg, right=-90deg, measured CCW in the vehicle-local
    frame), boundaries at the 45-degree diagonals -- every point belongs to exactly one quadrant,
    no gaps and no overlap."""
    angle = math.degrees(math.atan2(lateral, along))  # (-180, 180]
    if -45.0 <= angle < 45.0:
        return ClearanceDirection.FRONT
    if 45.0 <= angle < 135.0:
        return ClearanceDirection.LEFT
    if -135.0 <= angle < -45.0:
        return ClearanceDirection.RIGHT
    return ClearanceDirection.REAR


@dataclass(frozen=True)
class QuadrantReading:
    distance_m: float
    world_point: Point2D | None


def _envelope_offset(direction: ClearanceDirection, footprint: VehicleFootprint) -> float:
    return {
        ClearanceDirection.FRONT: footprint.envelope_front,
        ClearanceDirection.REAR: footprint.envelope_rear,
        ClearanceDirection.LEFT: footprint.envelope_left,
        ClearanceDirection.RIGHT: footprint.envelope_right,
    }[direction]


def _extent_along_direction(direction: ClearanceDirection, along: float, lateral: float) -> float:
    """The vehicle-local coordinate component relevant to `direction`, always increasing away from
    the vehicle in that direction (e.g. `along` is negative behind the vehicle, so REAR uses
    `-along` to get a positive "how far behind" value)."""
    if direction == ClearanceDirection.FRONT:
        return along
    if direction == ClearanceDirection.REAR:
        return -along
    if direction == ClearanceDirection.LEFT:
        return lateral
    return -lateral  # RIGHT


def nearest_in_each_quadrant(
    scan: CartesianScan, vehicle_state: VehicleState, footprint: VehicleFootprint, settings: Settings,
) -> dict[ClearanceDirection, QuadrantReading]:
    """One pass over `scan.points`: for every valid point within `lidar_range_max_m`, classify its
    quadrant and track the minimum envelope-edge-to-point gap seen so far in that quadrant.

    A quadrant with no qualifying point defaults to `lidar_range_max_m` minus that direction's own
    envelope offset (floored at 0) -- "no return" means "clear at least out to sensor range", the
    same convention preprocessing/mapping already use elsewhere for a missing return, not a new
    rule invented here (see docs/mapping.md "no return" handling). "No valid measurement" is
    therefore NEVER reported as zero clearance -- a genuinely blank sector reads as clear-to-range.

    Points closer than `Settings.min_valid_distance_m` are skipped here as well as in preprocessing
    (defense-in-depth): a near-zero self return would otherwise drive `raw_extent -
    envelope_offset` hugely negative in its quadrant, clamp to 0.0 below, and force a false
    "0.00 m" REAR/LEFT/RIGHT clearance. The primary self-return removal -- including a self-return
    *arc* off the ego body a few tens of cm out -- is the ego-footprint mask in
    `preprocessing.validation` (`common.geometry.EgoFootprint`); by the time a `CartesianScan`
    reaches here those points are already gone, so a genuinely blank sector reads clear-to-range,
    never zero.
    """
    min_valid_distance_m = settings.min_valid_distance_m
    best: dict[ClearanceDirection, QuadrantReading] = {
        d: QuadrantReading(distance_m=max(0.0, settings.lidar_range_max_m - _envelope_offset(d, footprint)), world_point=None)
        for d in ClearanceDirection
    }

    for point in scan.points:
        if not point.valid or point.distance <= 0.0 or point.distance > settings.lidar_range_max_m:
            continue
        if point.distance < min_valid_distance_m:
            continue  # near-zero self return -- not a real obstacle surface

        along, lateral = to_vehicle_frame(point.x, point.y, vehicle_state)
        direction = classify_quadrant(along, lateral)

        raw_extent = _extent_along_direction(direction, along, lateral)
        gap = raw_extent - _envelope_offset(direction, footprint)
        if gap < 0.0:
            gap = 0.0  # already inside/overlapping the safety envelope -- report contact, not negative clearance

        if gap < best[direction].distance_m:
            best[direction] = QuadrantReading(distance_m=round(gap, 4), world_point=Point2D(x=point.x, y=point.y))

    return best

"""2D grid ray traversal: which cells does a straight line between two grid cells pass through.

**Algorithm choice: Bresenham's line algorithm**, applied to already-quantized `(row, col)`
integer endpoints (the origin cell and the hit/no-return cell -- both already computed via
`coordinate_transform.world_to_grid`), rather than Amanatides & Woo's voxel traversal (which
walks the *continuous* line through fractional cell boundaries). Bresenham was chosen because:

- It operates entirely in integers -- no floating-point accumulation drift over a long ray, and
  no extra step-size/boundary-crossing bookkeeping.
- It is a single, well-known, easily-verified algorithm that -- by construction, not by cases
  handled specially -- covers every octant, so horizontal, vertical, diagonal, negative-direction,
  single-cell, and long rays all fall out of the same four lines of update logic (see the tests in
  `perception/tests/test_mapping_raytrace.py`).
- At this project's default resolution (0.1m/cell) the difference between "which cells does the
  exact continuous line pass through" (Amanatides & Woo) and "which cells does the nearest
  integer-quantized line pass through" (Bresenham on already-quantized endpoints) is at most a
  fraction of a cell -- immaterial for this phase's purpose (building a FREE/OCCUPIED/UNKNOWN
  belief grid, not a sub-cell-precision reconstruction).

Amanatides & Woo remains a documented future upgrade if a later phase ever needs precise
fractional ray-cell overlap (e.g. weighting a cell's update by exactly how much of the ray's
length crossed it) -- see docs/mapping.md "Ray traversal".
"""

from __future__ import annotations


def bresenham_line(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    """Every grid cell `(row, col)` from `(r0, c0)` to `(r1, c1)` inclusive, in traversal order
    (origin first, endpoint last). Handles equal endpoints (`(r0, c0) == (r1, c1)`, a single-cell
    result -- e.g. a measurement landing in the same cell as the sensor) and every direction
    (horizontal, vertical, diagonal, and any octant in between, including negative row/col deltas
    -- grid indices are always `>= 0` in practice here, but the algorithm itself places no such
    restriction).
    """
    cells: list[tuple[int, int]] = []

    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    step_r = 1 if r0 < r1 else -1
    step_c = 1 if c0 < c1 else -1
    error = dr - dc

    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        doubled_error = 2 * error
        if doubled_error > -dc:
            error -= dc
            r += step_r
        if doubled_error < dr:
            error += dr
            c += step_c

    return cells

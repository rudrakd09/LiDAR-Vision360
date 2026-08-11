"""Closed-form (non-iterative) line and circle fitting for a cluster's `(x, y)` points.

Why not RANSAC (which this phase's spec explicitly asks to investigate): RANSAC earns its keep
when a dataset has significant outlier *contamination* that a robust fit must ignore. By the time
points reach this stage, Phase 3 (preprocessing) has already removed sensor outliers and Phase 5
(DBSCAN) has already separated noise from dense clusters -- a cluster's member points are already
a curated, low-contamination set. For that case, a direct closed-form fit (PCA/total-least-squares
for lines, an algebraic Kasa fit for circles) gives the same practical result as RANSAC without
RANSAC's randomness or iteration cost, and -- being deterministic -- is naturally easier to test
and explain. See docs/object-classification.md "Shape fitting" for the full write-up.
"""

from __future__ import annotations

import math

_EPSILON = 1e-9


def line_fit_linearity(xs: list[float], ys: list[float]) -> float:
    """Fit the best (total-least-squares) line through the points via PCA on their covariance
    matrix, and return a `[0, 1]` linearity score: `1 - minor_eigenvalue / major_eigenvalue`.

    The major eigenvalue is the point spread *along* the best-fit line; the minor eigenvalue is
    the spread *perpendicular* to it (the line-fit residual variance). A score near 1 means the
    points are essentially collinear; a score near 0 means they spread out equally in every
    direction (no dominant line direction at all -- e.g. a compact blob or circle).

    Requires >= 2 distinct points to be meaningful; returns `0.0` for fewer (a single point has
    no direction to be "linear" in).
    """
    n = len(xs)
    if n < 2:
        return 0.0

    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    dx = [x - mean_x for x in xs]
    dy = [y - mean_y for y in ys]

    cov_xx = sum(v * v for v in dx) / n
    cov_yy = sum(v * v for v in dy) / n
    cov_xy = sum(a * b for a, b in zip(dx, dy)) / n

    # Eigenvalues of the symmetric 2x2 covariance matrix [[cov_xx, cov_xy], [cov_xy, cov_yy]].
    trace = cov_xx + cov_yy
    det = cov_xx * cov_yy - cov_xy * cov_xy
    discriminant = max(trace * trace - 4.0 * det, 0.0)  # clamp: tiny negative from fp error
    sqrt_disc = math.sqrt(discriminant)
    major = (trace + sqrt_disc) / 2.0
    minor = (trace - sqrt_disc) / 2.0

    if major < _EPSILON:
        return 0.0  # all points coincident -- no direction, not meaningfully "linear"

    return max(0.0, min(1.0, 1.0 - minor / major))


def circle_fit_circularity(
    xs: list[float], ys: list[float], max_radius_m: float = 2.0, residual_scale: float = 0.15
) -> float:
    """Fit a circle through the points via the Kasa algebraic method, and return a `[0, 1]`
    circularity score based on how tightly the points hug that circle's edge.

    Kasa's method solves the linear least-squares system for `x^2 + y^2 + D*x + E*y + F = 0`
    (a circle's equation multiplied out), giving center `(-D/2, -E/2)` and radius from `D, E, F`
    directly -- closed-form, no iteration.

    Guards against the degenerate case a nearly-flat set of points produces: Kasa's fit still
    "succeeds" numerically but returns a wildly oversized radius (mathematically, a line is a
    circle of infinite radius). `max_radius_m` is an **absolute** cap (meters), not scaled to the
    point cloud's own extent -- an earlier version of this function scaled the cap to the
    cluster's own extent and was fooled by *short* flat segments (e.g. a vehicle's front face,
    only ~1.7m across) that still produced a large-but-not-*that*-large-relative-to-their-own-
    extent radius; measured directly against real scenario data (see
    docs/object-classification.md "Parameter selection"), a flat wall's fit radius came out at
    ~60m, a flat vehicle face's at ~7m, and a genuine small pole's at ~0.15-0.4m -- an absolute
    cap comfortably above the real poles and comfortably below both flat cases separates them
    cleanly, which a per-cluster relative ratio did not.

    `residual_scale`: the RMS radial residual, as a fraction of the fitted radius, at which the
    score reaches `0.0` (linear in between `0` and this value). `0.15` (15%) was chosen the same
    way -- see docs/object-classification.md "Parameter selection".
    """
    n = len(xs)
    if n < 3:
        return 0.0  # a circle needs at least 3 points to be fit at all

    extent = max(max(xs) - min(xs), max(ys) - min(ys))
    if extent < _EPSILON:
        return 0.0  # all points coincident

    # Kasa linear least squares: solve the normal equations for [D, E, F].
    sum_x = sum(xs)
    sum_y = sum(ys)
    sum_xx = sum(x * x for x in xs)
    sum_yy = sum(y * y for y in ys)
    sum_xy = sum(x * y for x, y in zip(xs, ys))
    sum_xz = sum(x * (x * x + y * y) for x, y in zip(xs, ys))
    sum_yz = sum(y * (x * x + y * y) for x, y in zip(xs, ys))
    sum_z = sum(x * x + y * y for x, y in zip(xs, ys))

    # Normal-equation matrix for [D, E, F]:
    #   [sum_xx  sum_xy  sum_x] [D]   [-sum_xz]
    #   [sum_xy  sum_yy  sum_y] [E] = [-sum_yz]
    #   [sum_x   sum_y   n    ] [F]   [-sum_z ]
    a11, a12, a13 = sum_xx, sum_xy, sum_x
    a21, a22, a23 = sum_xy, sum_yy, sum_y
    a31, a32, a33 = sum_x, sum_y, float(n)
    b1, b2, b3 = -sum_xz, -sum_yz, -sum_z

    det = (
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31)
    )
    if abs(det) < _EPSILON:
        return 0.0  # singular system (e.g. all points exactly collinear) -- can't fit a circle

    def _cramer(col: int) -> float:
        m = [[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]]
        rhs = [b1, b2, b3]
        for row in range(3):
            m[row][col] = rhs[row]
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        ) / det

    d, e, f = _cramer(0), _cramer(1), _cramer(2)
    center_x, center_y = -d / 2.0, -e / 2.0
    radius_sq = center_x * center_x + center_y * center_y - f
    if radius_sq <= 0:
        return 0.0
    radius = math.sqrt(radius_sq)

    if radius > max_radius_m:
        return 0.0  # degenerate "line masquerading as a huge circle" case

    residuals = [abs(math.hypot(x - center_x, y - center_y) - radius) for x, y in zip(xs, ys)]
    rms_residual = math.sqrt(sum(r * r for r in residuals) / n)
    normalized_residual = rms_residual / radius

    return max(0.0, min(1.0, 1.0 - normalized_residual / residual_scale))

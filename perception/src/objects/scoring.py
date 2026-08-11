"""Explainable rule-based scoring for each candidate object category.

Each `score_*` function returns `(score, reasons)`: a `[0, 1]`-ish score derived from measurable
`ShapeFeatures`, and a list of human-readable strings explaining how it was reached. No black-box
model, no learned weights -- every number below is a documented, configurable threshold. See
docs/object-classification.md "Classification method" and "Parameter selection" for the full
reasoning behind each formula and every constant's value.
"""

from __future__ import annotations

from common.config import Settings
from models.objects import ShapeFeatures

Score = tuple[float, list[str]]


def _clip01(x: float) -> float:
    return max(0.0, min(1.0, x))


def _range_membership(value: float, min_v: float, max_v: float, margin: float) -> float:
    """1.0 inside `[min_v, max_v]`, ramping down to 0.0 over `margin` outside either edge."""
    if min_v <= value <= max_v:
        return 1.0
    if value < min_v:
        return _clip01(1.0 - (min_v - value) / margin)
    return _clip01(1.0 - (value - max_v) / margin)


def score_wall(f: ShapeFeatures, settings: Settings) -> Score:
    extent = max(f.width, f.depth)
    thickness = min(f.width, f.depth)
    min_length = settings.classification_wall_min_length_m
    # 0 credit right at the minimum length, full credit by 2x the minimum -- a short linear
    # fragment barely past the gate isn't as confidently "a wall" as a long one.
    extent_factor = _clip01((extent - min_length) / min_length)
    # A wall is (near-)flat: its perpendicular thickness, as seen by the LiDAR, should be small.
    # Without this, a genuinely boxy/2-faced object (e.g. a vehicle seen at an angle, showing a
    # corner) can still have high linearity along its dominant axis and enough extent to pass the
    # check above -- verified against real scenario data, where a rotated-rectangle cluster
    # (visible width AND depth both several meters) scored as confidently "wall" until this gate
    # was added. See docs/object-classification.md "Parameter selection".
    flatness_factor = _clip01(1.0 - thickness / settings.classification_wall_max_thickness_m)
    score = f.linearity_score * extent_factor * flatness_factor

    reasons = [
        f"linearity {f.linearity_score:.2f} (total-least-squares line fit)",
        f"extent {extent:.2f}m ({'>=' if extent >= min_length else '<'} {min_length:.2f}m minimum wall length)",
        f"thickness {thickness:.2f}m ({'<=' if thickness <= settings.classification_wall_max_thickness_m else '>'} {settings.classification_wall_max_thickness_m:.2f}m maximum wall thickness -- rules out boxy/2-faced clusters)",
    ]
    return score, reasons


def score_pole(f: ShapeFeatures, settings: Settings) -> Score:
    extent = max(f.width, f.depth)
    max_extent = settings.classification_pole_max_extent_m
    compact_factor = _clip01(1.0 - extent / max_extent)
    score = f.circularity_score * compact_factor

    reasons = [
        f"circularity {f.circularity_score:.2f} (algebraic circle fit)",
        f"extent {extent:.2f}m ({'<=' if extent <= max_extent else '>'} {max_extent:.2f}m maximum pole extent)",
    ]
    return score, reasons


def score_vehicle(f: ShapeFeatures, settings: Settings) -> Score:
    # Deliberately does not require *both* dimensions to fall in a car-sized range: a vehicle
    # viewed flat-on presents only its near face to a 2D LiDAR (its true depth/length is
    # occluded by its own body), so `min(width, depth)` here is often near zero and that must
    # not disqualify it -- see docs/object-classification.md "Known failure cases: viewing angle".
    #
    # The visible-face size check uses only the configured *width* range (not a union with the
    # depth range) -- a union was tried and tested against real scenario data (see
    # docs/object-classification.md "Parameter selection"): it let occluded wall *fragments* in
    # the 3-5m range score as confidently vehicle-like as an actual car, since a wall fragment's
    # length can coincidentally fall inside a car's *length* range even though it is nothing like
    # a car's *width*. Checking against the (narrower) width range only still correctly matches a
    # vehicle's near face (whichever physical dimension it happens to be -- width or length -- a
    # flat face at driving distance is usually within a car's width order of magnitude) while
    # giving wall-length fragments much less unearned credit.
    primary_extent = max(f.width, f.depth)
    secondary_extent = min(f.width, f.depth)

    # Checked against *both* the primary (larger) and secondary (smaller) extent, taking
    # whichever matches better: a flat-face view puts the meaningful dimension in `primary`
    # (secondary is near zero); a cornered/boxy view can inflate `primary` past a real vehicle's
    # width via axis-aligned-bounding-box-of-a-rotated-shape effects (see docs/clustering.md
    # "Known limitations"), in which case `secondary` is the more reliable of the two. Verified
    # against real scenario data -- see docs/object-classification.md "Parameter selection".
    size_factor = max(
        _range_membership(primary_extent, settings.classification_vehicle_width_min_m, settings.classification_vehicle_width_max_m, margin=0.5),
        _range_membership(secondary_extent, settings.classification_vehicle_width_min_m, settings.classification_vehicle_width_max_m, margin=0.5),
    )
    # Only penalize the secondary dimension if it's itself implausibly large (suggesting this is
    # actually part of something bigger, e.g. a wall) -- small/near-zero secondary is expected.
    secondary_factor = _clip01(1.0 - max(0.0, secondary_extent - settings.classification_vehicle_depth_max_m) / 1.0)
    point_count_factor = _clip01(f.point_count / settings.classification_vehicle_min_points)
    shape_factor = 1.0 - f.circularity_score  # a genuinely circular fit is a pole, not a vehicle

    # `size_factor` is a multiplicative *gate*, not just one weighted term among several: a
    # cluster whose size doesn't remotely resemble a vehicle must not still reach a passable
    # score purely from generic, weakly-discriminative evidence (decent point count, low
    # circularity -- true of almost anything that isn't a pole, including a long wall). Verified
    # directly against real scenario data: without this gate, a long noisy wall (extent ~19m,
    # size_factor 0) still scored ~0.60 -- comfortably over the confidence threshold -- purely
    # from the other three terms. See docs/object-classification.md "Parameter selection".
    quality = 0.4 + 0.2 * secondary_factor + 0.2 * point_count_factor + 0.2 * shape_factor
    score = size_factor * quality

    reasons = [
        f"visible extent {primary_extent:.2f}m within configured vehicle-width range [{settings.classification_vehicle_width_min_m:.1f}, {settings.classification_vehicle_width_max_m:.1f}]m" if size_factor > 0.5 else f"visible extent {primary_extent:.2f}m outside the configured vehicle-width range",
        f"point count {f.point_count} (density factor {point_count_factor:.2f})",
        f"low circularity {f.circularity_score:.2f} (not pole-shaped)",
    ]
    return score, reasons


def score_person(f: ShapeFeatures, settings: Settings) -> Score:
    """Deliberately conservative -- see docs/object-classification.md "Person-like
    classification". A single 2D LiDAR scan cannot reliably identify a human; this category
    exists but its score is hard-capped well below full confidence."""
    extent = max(f.width, f.depth)
    size_factor = _range_membership(
        extent, settings.classification_person_extent_min_m, settings.classification_person_extent_max_m, margin=0.1
    )
    # Neither a thin pole (high circularity) nor a flat surface (high linearity) -- a person's
    # cross-section is irregular, so *both* must be unremarkable, not merely similar to each
    # other. (An earlier version used `1 - abs(circularity - linearity)`, which rewards a cluster
    # where *both* happen to be high together -- exactly what small (~5-point) clusters tend to
    # produce regardless of true shape, since a line fit trivially "explains" a handful of points
    # about as well as a circle fit does. Requiring the *minimum* of the two complements avoids
    # that false positive -- see docs/object-classification.md "Parameter selection".)
    irregularity_factor = min(1.0 - f.circularity_score, 1.0 - f.linearity_score)
    raw_score = size_factor * _clip01(irregularity_factor)
    score = min(raw_score, settings.classification_person_max_confidence)

    reasons = [
        f"extent {extent:.2f}m within the narrow configured person-like range [{settings.classification_person_extent_min_m:.2f}, {settings.classification_person_extent_max_m:.2f}]m" if size_factor > 0.5 else f"extent {extent:.2f}m outside the narrow person-like range",
        "neither strongly linear nor strongly circular (irregular cross-section)",
        f"confidence capped at {settings.classification_person_max_confidence:.2f} -- a single 2D LiDAR scan cannot reliably identify a person",
    ]
    return score, reasons


def score_large_obstacle(f: ShapeFeatures, settings: Settings, best_other_score: float) -> Score:
    """Generic catch-all: clearly substantial, but not a confident match for anything specific.
    `best_other_score` is the best of wall/pole/vehicle/person's own scores -- this category only
    activates where those don't already explain the cluster well."""
    extent = max(f.width, f.depth)
    min_extent = settings.classification_large_min_extent_m
    extent_factor = _clip01((extent - min_extent) / min_extent)
    unexplained_factor = 1.0 - best_other_score
    score = extent_factor * unexplained_factor

    reasons = [
        f"extent {extent:.2f}m ({'>=' if extent >= min_extent else '<'} {min_extent:.2f}m minimum substantial size)",
        f"point count {f.point_count}",
        f"does not confidently match a more specific category (best specific score {best_other_score:.2f})",
    ]
    return score, reasons

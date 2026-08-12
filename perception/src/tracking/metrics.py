"""Generic tracking-quality metrics, computed from a sequence of `TrackedScan`s (and, optionally,
parallel ground-truth positions/velocities for position/velocity error).

Deliberately has no dependency on `simulator` or on any notion of "which scenario has which
ground truth" -- that mapping is scenario-specific knowledge that belongs in the evaluation
script (`scripts/evaluate_tracking.py`, which already depends on both `simulator` and
`perception`). This module is the reusable, generic metric computation underneath it, directly
unit-testable with synthetic data -- the same separation `objects.metrics` established for
classification (Phase 6). See docs/tracking.md "Tracking quality metrics".
"""

from __future__ import annotations

import math
import statistics
from typing import Sequence

from models.tracking import TrackedScan


def track_summary(scans: Sequence[TrackedScan]) -> dict:
    """Summary statistics over a completed tracking run.

    `total_tracks_created` counts every distinct `track_id` that ever appeared in `objects`
    across `scans` (a track lost and never reassigned that id counts once). `active_tracks_at_end`
    / age / hits / misses reflect only the final scan's still-active tracks -- a track's own
    counters are already cumulative up to that point (see `models.objects.DetectedObject.
    track_age`/`track_hits`/`track_misses`). Empty input returns an all-zero summary rather than
    raising.
    """
    seen_track_ids: set[str] = set()
    total_lost = 0
    for scan in scans:
        seen_track_ids.update(o.track_id for o in scan.objects if o.track_id)
        total_lost += scan.lost_track_count

    final_objects = scans[-1].objects if scans else []
    ages = [o.track_age for o in final_objects if o.track_age is not None]
    hits = [o.track_hits for o in final_objects if o.track_hits is not None]
    misses = [o.track_misses for o in final_objects if o.track_misses is not None]

    return {
        "total_tracks_created": len(seen_track_ids),
        "active_tracks_at_end": len(final_objects),
        "total_tracks_lost": total_lost,
        "mean_track_age": statistics.fmean(ages) if ages else 0.0,
        "max_track_age": max(ages) if ages else 0,
        "mean_track_hits": statistics.fmean(hits) if hits else 0.0,
        "mean_track_misses": statistics.fmean(misses) if misses else 0.0,
    }


def track_id_consistency(id_sequences: Sequence[Sequence[str | None]]) -> float:
    """Fraction of consecutive-scan steps, across one or more per-object track-ID sequences, that
    kept the same `track_id` -- `1.0` means perfect continuity (an object's assigned ID never
    changed once assigned), lower means the tracker reassigned/lost it partway through.

    Each element of `id_sequences` is one physical object's `track_id` across scans, in scan
    order (`None` for a scan it had no live track in at all). A `None -> None` step (never
    tracked, still not tracked) is not counted either way -- it is neither consistent nor
    inconsistent, just absent. Empty input, or input with fewer than 2 scans of history for every
    sequence, returns `1.0` (vacuously consistent -- nothing to have broken).
    """
    total_steps = 0
    consistent_steps = 0
    for sequence in id_sequences:
        for previous_id, current_id in zip(sequence, sequence[1:]):
            if previous_id is None and current_id is None:
                continue
            total_steps += 1
            if previous_id == current_id:
                consistent_steps += 1
    return consistent_steps / total_steps if total_steps else 1.0


def position_error(estimated: Sequence[tuple[float, float]], ground_truth: Sequence[tuple[float, float]]) -> dict:
    """Mean/max Euclidean position error (meters) between parallel estimated `(x, y)` and
    ground-truth `(x, y)` sequences. Raises `ValueError` on a length mismatch; returns all-zero
    for empty input rather than raising."""
    if len(estimated) != len(ground_truth):
        raise ValueError(f"estimated and ground_truth must be the same length, got {len(estimated)} and {len(ground_truth)}")
    if not estimated:
        return {"mean_error_m": 0.0, "max_error_m": 0.0}
    errors = [math.hypot(ex - gx, ey - gy) for (ex, ey), (gx, gy) in zip(estimated, ground_truth)]
    return {"mean_error_m": statistics.fmean(errors), "max_error_m": max(errors)}


def velocity_error(estimated: Sequence[tuple[float, float]], ground_truth: Sequence[tuple[float, float]]) -> dict:
    """Mean/max Euclidean velocity error (m/s) between parallel estimated `(vx, vy)` and
    ground-truth `(vx, vy)` sequences. Same shape/empty-input behavior as `position_error`."""
    if len(estimated) != len(ground_truth):
        raise ValueError(f"estimated and ground_truth must be the same length, got {len(estimated)} and {len(ground_truth)}")
    if not estimated:
        return {"mean_error_m_s": 0.0, "max_error_m_s": 0.0}
    errors = [math.hypot(ex - gx, ey - gy) for (ex, ey), (gx, gy) in zip(estimated, ground_truth)]
    return {"mean_error_m_s": statistics.fmean(errors), "max_error_m_s": max(errors)}

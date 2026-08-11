"""Optional cross-scan exponential temporal filter.

`filtered_t = alpha * current_t + (1 - alpha) * previous_filtered_t`, matched per angle bin so a
measurement is only smoothed against its own history, not its neighbors' (that's the spatial
median filter's job). Lower `alpha` means more smoothing but more lag on genuinely moving
obstacles; higher `alpha` tracks changes faster but smooths less. Disabled by default
(`Settings.preprocessing_temporal_filter_enabled = False`) for exactly that reason -- see
docs/preprocessing.md "Temporal Filtering" for the lag-vs-scan-count analysis this default was
chosen against, validated using the `07_moving_crossing` / `08_approaching_obstacle` scenarios.

This filter is inherently stateful across calls (it needs the *previous* filtered value per
angle), so it is a class owned by one `Preprocessor` instance, not a pure function like the
spatial filters. Use one instance per independent scan stream; reuse across sources will blend
unrelated histories together.
"""

from __future__ import annotations

from models.lidar import LiDARPoint


class TemporalFilter:
    def __init__(self, alpha: float, angle_precision: int = 2) -> None:
        """`alpha`: weight given to the current measurement, in `(0, 1]`. `angle_precision`:
        decimal places used to key state by angle -- tolerates float jitter between scans while
        still treating measurements as "the same angle bin" for a fixed angular sampling grid."""
        self.alpha = alpha
        self._angle_precision = angle_precision
        self._state: dict[float, float] = {}

    def _key(self, angle: float) -> float:
        return round(angle, self._angle_precision)

    def apply(self, points: list[LiDARPoint]) -> list[LiDARPoint]:
        """Smooth each point's `distance` against this filter's running per-angle history.

        An angle seen for the first time (or not seen in the immediately preceding call) is
        passed through unfiltered and becomes that angle bin's new starting point -- there is no
        prior value to blend with. An angle bin missing from a given scan simply keeps its last
        stored value untouched until it reappears.
        """
        result: list[LiDARPoint] = []
        for point in points:
            key = self._key(point.angle)
            previous = self._state.get(key)
            filtered_distance = point.distance if previous is None else (
                self.alpha * point.distance + (1.0 - self.alpha) * previous
            )
            self._state[key] = filtered_distance
            result.append(point.model_copy(update={"distance": round(filtered_distance, 4)}))
        return result

    def reset(self) -> None:
        """Clear all per-angle history, e.g. when reusing a `Preprocessor` for a new stream."""
        self._state.clear()

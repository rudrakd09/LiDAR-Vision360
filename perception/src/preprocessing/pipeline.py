"""The preprocessing pipeline: `ScanFrame` (raw) -> `PreprocessedScan` (clean).

    Raw ScanFrame
         |
    Validation           (validation.validate_points)
         |
    Range Filtering      (folded into validation -- see docs/preprocessing.md)
         |
    [sort by angle]      (canonical order; required for correct circular neighbor handling)
         |
    Outlier Detection    (outliers.detect_outliers)
         |
    Noise Filtering      (denoise.median_filter)
         |
    Temporal Filtering   (temporal.TemporalFilter -- optional)
         |
    PreprocessedScan

This module has no import-time dependency on `simulator` or any concrete `LiDARDataSource`
implementation -- it only consumes `models.scan.ScanFrame`, so it processes simulated and (once
Phase 16 lands) real hardware scans identically. It also does not import or get imported by
anything Unity/cloud/tracking-related; see docs/preprocessing.md "Architecture" for the
intentional pipeline boundary this phase stops at.
"""

from __future__ import annotations

from dataclasses import dataclass

from common.config import Settings, get_settings
from common.logging import get_logger
from models.preprocessing import PreprocessedScan
from models.scan import ScanFrame

from .denoise import median_filter
from .outliers import detect_outliers
from .quality import compute_quality_statistics
from .temporal import TemporalFilter
from .validation import validate_points

logger = get_logger(__name__)


@dataclass(frozen=True)
class PreprocessingConfig:
    """A snapshot of the preprocessing parameters used by one `Preprocessor` instance, resolved
    from `Settings` once at construction time -- so behavior is stable for the life of that
    instance even if global settings are changed elsewhere mid-run."""

    min_range_m: float
    max_range_m: float
    outlier_threshold_m: float
    outlier_window_size: int
    median_filter_window: int
    temporal_filter_enabled: bool
    temporal_filter_alpha: float

    @classmethod
    def from_settings(cls, settings: Settings) -> "PreprocessingConfig":
        return cls(
            min_range_m=settings.lidar_range_min_m,
            max_range_m=settings.lidar_range_max_m,
            outlier_threshold_m=settings.preprocessing_outlier_threshold_m,
            outlier_window_size=settings.preprocessing_outlier_window_size,
            median_filter_window=settings.preprocessing_median_filter_window,
            temporal_filter_enabled=settings.preprocessing_temporal_filter_enabled,
            temporal_filter_alpha=settings.preprocessing_temporal_filter_alpha,
        )


class Preprocessor:
    """Turns raw `ScanFrame`s into `PreprocessedScan`s. Stateless except for the optional
    temporal filter's per-angle history -- construct one instance per independent scan stream
    (e.g. one per data source) and reuse it across calls to `process()`."""

    def __init__(self, settings: Settings | None = None, config: PreprocessingConfig | None = None) -> None:
        self.config = config or PreprocessingConfig.from_settings(settings or get_settings())
        self._temporal_filter = (
            TemporalFilter(self.config.temporal_filter_alpha) if self.config.temporal_filter_enabled else None
        )

    def process(self, scan: ScanFrame) -> PreprocessedScan:
        total_count = len(scan.points)

        valid_points, invalid_count, reason_counts = validate_points(
            scan.points, self.config.min_range_m, self.config.max_range_m
        )
        if reason_counts:
            logger.debug("Scan %s: %d/%d measurements invalid (%s)", scan.scan_id, invalid_count, total_count, dict(reason_counts))

        # Canonical angle order is required for correct circular-neighbor handling below, and is
        # a useful invariant for every downstream consumer regardless of raw scan order.
        valid_points = sorted(valid_points, key=lambda p: p.angle)

        clean_points, outlier_count = detect_outliers(
            valid_points, self.config.outlier_threshold_m, self.config.outlier_window_size
        )
        clean_points = median_filter(clean_points, self.config.median_filter_window)

        if self._temporal_filter is not None:
            clean_points = self._temporal_filter.apply(clean_points)

        quality_statistics = compute_quality_statistics(
            total_count, len(valid_points), invalid_count, outlier_count, clean_points
        )

        return PreprocessedScan(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            points=clean_points,
            total_count=total_count,
            valid_count=len(valid_points),
            invalid_count=invalid_count,
            outlier_count=outlier_count,
            quality_statistics=quality_statistics,
        )

    def reset_temporal_state(self) -> None:
        """Clear temporal-filter history, e.g. before reusing this instance for a new stream."""
        if self._temporal_filter is not None:
            self._temporal_filter.reset()


def preprocess_scan(scan: ScanFrame, settings: Settings | None = None) -> PreprocessedScan:
    """One-off convenience wrapper around a throwaway `Preprocessor`.

    For temporal filtering to have any effect, construct and reuse a `Preprocessor` instance
    directly across multiple scans instead -- a fresh instance here has no prior-scan history.
    """
    return Preprocessor(settings=settings).process(scan)

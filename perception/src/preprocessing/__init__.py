"""LiDAR preprocessing: validation, range filtering, outlier detection, noise filtering, and
optional temporal filtering. Turns a raw `models.scan.ScanFrame` into a clean
`models.preprocessing.PreprocessedScan`.

Status: **implemented (Phase 3)**. Independent of `simulator` and of every later pipeline stage
(coordinates, clustering, tracking, collision, Unity, cloud) -- see docs/preprocessing.md.

Typical usage:

    from preprocessing import Preprocessor

    preprocessor = Preprocessor()  # reads defaults from common.config.Settings
    clean_scan = preprocessor.process(raw_scan_frame)
"""

from .outliers import detect_outliers
from .denoise import median_filter
from .pipeline import PreprocessingConfig, Preprocessor, preprocess_scan
from .quality import compute_quality_statistics
from .temporal import TemporalFilter
from .validation import InvalidReason, classify_point, validate_points

__all__ = [
    "Preprocessor",
    "PreprocessingConfig",
    "preprocess_scan",
    "validate_points",
    "classify_point",
    "InvalidReason",
    "detect_outliers",
    "median_filter",
    "TemporalFilter",
    "compute_quality_statistics",
]

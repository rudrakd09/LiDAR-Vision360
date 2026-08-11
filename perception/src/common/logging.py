"""Process-wide logging setup.

Call `setup_logging()` once near the start of any entry point (script, API server, test session
plugin); every other module just does `logger = get_logger(__name__)`, which will lazily set up
logging with defaults if nothing has configured it yet.
"""

from __future__ import annotations

import logging
import sys

from .config import get_settings

_CONFIGURED = False


def setup_logging(level: str | None = None) -> None:
    """Configure the root logger once. Subsequent calls are no-ops.

    Args:
        level: Optional override (e.g. "DEBUG"). Defaults to `Settings.log_level`.
    """
    global _CONFIGURED
    if _CONFIGURED:
        return

    settings = get_settings()
    resolved_level = (level or settings.log_level).upper()

    logging.basicConfig(
        level=resolved_level,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout,
    )
    _CONFIGURED = True


def get_logger(name: str) -> logging.Logger:
    """Return a module-level logger, configuring root logging with defaults if needed."""
    setup_logging()
    return logging.getLogger(name)

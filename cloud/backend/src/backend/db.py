"""Database engine setup: tries `Settings.database_url` (PostgreSQL by default); on any connection
failure at startup (no local server, wrong credentials, driver not installed), falls back to a
local SQLite file at `Settings.backend_sqlite_fallback_path` and logs a warning rather than failing
to start -- per this project's explicit "local development first... SQLite fallback if PostgreSQL
is not configured" requirement. See docs/cloud.md "Database".

Synchronous SQLAlchemy (not the async engine) -- deliberately: this is a local, single-instance
demo backend with modest write volume (occasional session/event rows, not high-frequency), and
FastAPI already runs sync `def` route handlers in a threadpool automatically, so there is no need
for the added complexity of `asyncpg`/`aiosqlite` here. The one genuinely real-time path
(`/ws/live`) never touches the database at all -- it broadcasts from `state.LatestState`, which is
already in-memory and thread-safe.
"""

from __future__ import annotations

from pathlib import Path

from sqlalchemy import create_engine, text
from sqlalchemy.orm import Session, sessionmaker

from common.logging import get_logger

from .config import Settings

logger = get_logger(__name__)


def _try_connect(url: str, connect_timeout_s: int = 3) -> bool:
    """Best-effort connectivity probe -- `SELECT 1` against `url`, `connect_args` timeout applied
    where the driver supports it (Postgres; SQLite ignores it, harmlessly). Returns False on *any*
    failure (missing driver, connection refused, auth failure, ...) rather than letting a specific
    exception type leak into the caller -- this function's only job is "can I use this URL right
    now," not diagnosing why not (that goes to the log)."""
    try:
        connect_args = {"connect_timeout": connect_timeout_s} if url.startswith("postgresql") else {}
        probe_engine = create_engine(url, connect_args=connect_args, pool_pre_ping=True)
        with probe_engine.connect() as conn:
            conn.execute(text("SELECT 1"))
        probe_engine.dispose()
        return True
    except Exception as e:  # noqa: BLE001 -- deliberately broad, see docstring
        logger.warning("[DB] Could not connect to %s (%s: %s).", _redact(url), type(e).__name__, e)
        return False


def _redact(url: str) -> str:
    """Never log a real password -- redacts the credentials portion of a URL for the warning
    message above."""
    if "@" not in url:
        return url
    scheme_and_creds, rest = url.rsplit("@", 1)
    scheme = scheme_and_creds.split("://", 1)[0] if "://" in scheme_and_creds else scheme_and_creds
    return f"{scheme}://***:***@{rest}"


def resolve_database_url(settings: Settings) -> str:
    """The URL actually used: `settings.database_url` if reachable, otherwise the local SQLite
    fallback (its parent directory is created if missing)."""
    if settings.database_url.startswith("sqlite"):
        _ensure_sqlite_parent_dir(settings.database_url)
        return settings.database_url

    if _try_connect(settings.database_url):
        logger.info("[DB] Connected to configured database (%s).", _redact(settings.database_url))
        return settings.database_url

    fallback_path = Path(settings.backend_sqlite_fallback_path)
    fallback_path.parent.mkdir(parents=True, exist_ok=True)
    fallback_url = f"sqlite:///{fallback_path.as_posix()}"
    logger.warning("[DB] Falling back to local SQLite at %s.", fallback_path)
    return fallback_url


def _ensure_sqlite_parent_dir(sqlite_url: str) -> None:
    # sqlite:///relative/path.db or sqlite:////absolute/path.db
    raw_path = sqlite_url.split("sqlite:///", 1)[-1]
    if raw_path and raw_path != ":memory:":
        Path(raw_path).parent.mkdir(parents=True, exist_ok=True)


class Database:
    """Owns the engine + sessionmaker for the process lifetime. Constructed once in `main.py`'s
    lifespan, passed to routes/ingestion via FastAPI dependency injection (`get_session`)."""

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or Settings()
        self.url = resolve_database_url(self.settings)
        connect_args = {"check_same_thread": False} if self.url.startswith("sqlite") else {}
        self.engine = create_engine(self.url, connect_args=connect_args, pool_pre_ping=True)
        self.SessionLocal = sessionmaker(bind=self.engine, autoflush=False, autocommit=False)

    def create_all(self) -> None:
        from .models_db import Base

        Base.metadata.create_all(self.engine)

    def session(self) -> Session:
        return self.SessionLocal()

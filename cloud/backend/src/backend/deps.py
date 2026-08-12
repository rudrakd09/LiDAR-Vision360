"""FastAPI dependency getters -- pull the app-level singletons (`LatestState`, `Database`,
`LiveBroadcastHub`, `PerceptionIngestor`, start time) off `request.app.state`, where `main.py`'s
lifespan puts them. Keeps every route a plain function of its declared dependencies, not a module
reaching into a global.
"""

from __future__ import annotations

from fastapi import Request

from .db import Database
from .ingestion import PerceptionIngestor
from .state import LatestState
from .ws import LiveBroadcastHub


def get_state(request: Request) -> LatestState:
    return request.app.state.latest_state


def get_db(request: Request) -> Database:
    return request.app.state.db


def get_hub(request: Request) -> LiveBroadcastHub:
    return request.app.state.hub


def get_ingestor(request: Request) -> PerceptionIngestor:
    return request.app.state.ingestor


def get_start_time(request: Request) -> float:
    return request.app.state.start_time

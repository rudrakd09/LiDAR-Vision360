"""FastAPI application entrypoint: `uvicorn backend.main:app --host <host> --port <port>`.

Lifespan starts `PerceptionIngestor` (a background thread connecting to the SAME
`streaming_json_port` Unity connects to) on startup and stops it on shutdown -- the backend never
runs its own scenario/simulator/pipeline; every value it serves comes from that one ingested
stream. See docs/cloud.md "Architecture".
"""

from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from common.logging import get_logger, setup_logging

from .config import get_settings
from .db import Database
from .ingestion import PerceptionIngestor
from .routes import core, debug, events, frames, metrics
from .state import LatestState
from .ws import LiveBroadcastHub

# Every route module below is defined with NO baked-in prefix (see each router's own module
# docstring) specifically so it can be mounted at both a bare path (`/health`) and the original
# `/api/*` path (`/api/health`) by including it twice -- "if the project already uses different
# route names, preserve the existing naming where possible and add compatible routes rather than
# breaking existing clients." Both mounts are the exact same route functions/objects, not two
# implementations to keep in sync.
_ROUTE_MODULES = (core, frames, events, metrics, debug)

logger = get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    setup_logging()
    settings = get_settings()

    app.state.start_time = time.time()
    app.state.latest_state = LatestState(
        ring_buffer_size=settings.backend_ring_buffer_size,
        track_grace_period_s=settings.backend_track_grace_period_s,
        track_history_length=settings.backend_track_history_length,
    )
    app.state.db = Database(settings)
    app.state.db.create_all()
    app.state.hub = LiveBroadcastHub()

    loop = asyncio.get_running_loop()
    app.state.ingestor = PerceptionIngestor(settings, app.state.latest_state, app.state.db, app.state.hub, loop)
    app.state.ingestor.start()

    logger.info("[BACKEND] Ready on %s:%d (ingesting from %s:%d).", settings.backend_host, settings.backend_port, settings.streaming_host, settings.streaming_json_port)
    try:
        yield
    finally:
        app.state.ingestor.stop()
        logger.info("[BACKEND] Shut down.")


def create_app() -> FastAPI:
    settings = get_settings()
    app = FastAPI(title="LiDAR-Vision360 Backend", version="0.1.0", lifespan=lifespan)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.backend_cors_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    for module in _ROUTE_MODULES:
        app.include_router(module.router)  # bare path, e.g. GET /health
        app.include_router(module.router, prefix="/api")  # original path, e.g. GET /api/health

    @app.get("/", tags=["core"])
    def root() -> dict:
        """Service info -- who this is, whether it's up, and where everything else lives.
        Real values only: `stream` below is `state.session_status`/`connection`, the same fields
        `GET /status` returns, not a separate hard-coded "ok"."""
        s = get_settings()
        latest_state: LatestState = app.state.latest_state
        c = latest_state.connection
        return {
            "name": "LiDAR-Vision360 Backend",
            "version": app.version,
            "status": "ok",
            "docs": "/docs",
            "websocket": "/ws/live",
            # `app.routes` doesn't flatten `include_router`-mounted routes into simple objects
            # with their own `.path` in every FastAPI version -- the OpenAPI schema is already
            # the canonical, always-accurate list of every actual path this app serves, so build
            # this from that instead of re-deriving it a second, potentially-incomplete way.
            "endpoints": sorted(p for p in app.openapi()["paths"] if p not in ("/", "/openapi.json")),
            "stream": {
                "connection_state": c.state,
                "session_status": latest_state.session_status(s.backend_session_stale_threshold_s),
                "source_id": c.source_id,
                "last_frame_id": c.last_frame_id,
                "frames_received": c.frames_received,
            },
        }

    @app.websocket("/ws/live")
    async def ws_live(websocket: WebSocket) -> None:
        hub: LiveBroadcastHub = get_hub_from_app(websocket)
        state: LatestState = get_state_from_app(websocket)

        await hub.connect(websocket)
        try:
            # Send the current snapshot immediately so a newly-connected dashboard doesn't wait
            # for the next frame to render anything -- see LatestState.snapshot_for_new_client.
            settings = get_settings()
            await websocket.send_json({"type": "snapshot", "data": state.snapshot_for_new_client(settings.backend_session_stale_threshold_s)})
            while True:
                # This connection is broadcast-only (server -> client); we still need to await
                # something so a client disconnect is detected promptly rather than only on the
                # next broadcast's failed send.
                await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            await hub.disconnect(websocket)

    return app


def get_hub_from_app(websocket: WebSocket) -> LiveBroadcastHub:
    return websocket.app.state.hub


def get_state_from_app(websocket: WebSocket) -> LatestState:
    return websocket.app.state.latest_state


app = create_app()

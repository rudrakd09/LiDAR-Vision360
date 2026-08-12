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
from .routes import core, events, frames
from .state import LatestState
from .ws import LiveBroadcastHub

logger = get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    setup_logging()
    settings = get_settings()

    app.state.start_time = time.time()
    app.state.latest_state = LatestState(
        ring_buffer_size=settings.backend_ring_buffer_size,
        track_grace_period_s=settings.backend_track_grace_period_s,
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

    app.include_router(core.router)
    app.include_router(frames.router)
    app.include_router(events.router)

    @app.websocket("/ws/live")
    async def ws_live(websocket: WebSocket) -> None:
        hub: LiveBroadcastHub = get_hub_from_app(websocket)
        state: LatestState = get_state_from_app(websocket)

        await hub.connect(websocket)
        try:
            # Send the current snapshot immediately so a newly-connected dashboard doesn't wait
            # for the next frame to render anything -- see LatestState.snapshot_for_new_client.
            await websocket.send_json({"type": "snapshot", "data": state.snapshot_for_new_client()})
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

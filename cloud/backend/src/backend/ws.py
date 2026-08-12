"""`/ws/live` broadcast hub: a set of connected WebSocket clients, fed by `ingestion.
PerceptionIngestor` every time a new frame/heartbeat/status message arrives.

Same "one slow client must never stall the others" principle `perception.streaming.
PerceptionStreamServer` already established for Python->Unity (bounded per-client queue there);
here, simpler for a local demo's realistic client count, a per-send timeout plus dropping any
client whose send fails/hangs -- broadcasting is fire-and-forget, never something the ingestion
thread blocks on.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import WebSocket

from common.logging import get_logger

logger = get_logger(__name__)

_SEND_TIMEOUT_S = 2.0


class LiveBroadcastHub:
    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._clients.add(ws)
        logger.info("[WS] Dashboard client connected (%d total).", len(self._clients))

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)
        logger.info("[WS] Dashboard client disconnected (%d total).", len(self._clients))

    async def broadcast(self, message: dict[str, Any]) -> None:
        async with self._lock:
            clients = list(self._clients)
        if not clients:
            return
        payload = json.dumps(message)
        stale: list[WebSocket] = []
        for client in clients:
            try:
                await asyncio.wait_for(client.send_text(payload), timeout=_SEND_TIMEOUT_S)
            except Exception:  # noqa: BLE001 -- a broken/slow client must never affect the others
                stale.append(client)
        if stale:
            async with self._lock:
                for client in stale:
                    self._clients.discard(client)

    @property
    def client_count(self) -> int:
        return len(self._clients)

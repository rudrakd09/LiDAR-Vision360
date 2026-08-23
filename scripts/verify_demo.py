#!/usr/bin/env python
"""Demo verification helper for `scripts/run_demo.ps1` (and for manually checking one running
scenario end-to-end): a handful of small, independent checks against the already-running
backend/bridge, each callable on its own, plus a `full-check` that runs all of them and prints one
JSON summary.

Deliberately dependency-free beyond what's already in `.venv` (`urllib`/`socket`/`json` from the
stdlib, `websockets` -- already a backend dependency, see `cloud/backend`) -- no `requests`
needed. Every subcommand prints one JSON object to stdout and exits 0 on success / 1 on failure,
so `run_demo.ps1` can call it with `& $VenvPython scripts/verify_demo.py <cmd> ...` and check
`$LASTEXITCODE` the same way it checks any other external command.

Usage:
    python scripts/verify_demo.py health --base-url http://localhost:8000
    python scripts/verify_demo.py live-frame --base-url http://localhost:8000 --timeout 15
    python scripts/verify_demo.py source-id --base-url http://localhost:8000 --expect simulated:08_approaching_obstacle
    python scripts/verify_demo.py websocket --ws-url ws://localhost:8000/ws/live --timeout 10
    python scripts/verify_demo.py frames-changing --base-url http://localhost:8000 --samples 3 --interval 1.0
    python scripts/verify_demo.py full-check --base-url http://localhost:8000 --ws-url ws://localhost:8000/ws/live --scenario 08_approaching_obstacle
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import urllib.error
import urllib.request


def _get_json(url: str, timeout: float = 5.0):
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read()
    if not body:
        return None
    return json.loads(body)


def _fail(msg: str, **extra) -> None:
    print(json.dumps({"ok": False, "error": msg, **extra}, default=str))
    sys.exit(1)


def _succeed(**fields) -> None:
    print(json.dumps({"ok": True, **fields}, default=str))
    sys.exit(0)


# --- individual checks (each also usable as a plain Python function by full-check) ---


def check_health(base_url: str, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    last_err = None
    while time.monotonic() < deadline:
        try:
            data = _get_json(f"{base_url}/health", timeout=3.0)
            if data and data.get("status") == "ok":
                return data
        except (urllib.error.URLError, OSError) as e:
            last_err = e
        time.sleep(0.5)
    raise RuntimeError(f"/health did not report ok within {timeout}s (last error: {last_err})")


def check_live_frame(base_url: str, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    last_err = None
    while time.monotonic() < deadline:
        try:
            data = _get_json(f"{base_url}/debug/live-frame", timeout=3.0)
            if data:
                return data
        except (urllib.error.URLError, OSError) as e:
            last_err = e
        time.sleep(0.5)
    raise RuntimeError(f"/debug/live-frame stayed null/unreachable for {timeout}s (last error: {last_err})")


def check_source_id(base_url: str, expect: str | None, timeout: float) -> dict:
    frame = check_live_frame(base_url, timeout)
    source_id = frame.get("source_id")
    if not source_id:
        raise RuntimeError(f"live frame has no source_id: {frame}")
    if expect is not None and source_id != expect:
        raise RuntimeError(f"source_id {source_id!r} != expected {expect!r}")
    return {"source_id": source_id, "session_id": frame.get("session_id")}


def check_websocket(ws_url: str, timeout: float) -> dict:
    import asyncio

    import websockets

    async def _run():
        async with websockets.connect(ws_url, open_timeout=timeout, close_timeout=3) as ws:
            raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
            msg = json.loads(raw)
            return msg

    try:
        msg = asyncio.run(_run())
    except Exception as e:  # noqa: BLE001 -- surfaced as a plain failure reason, not a traceback
        raise RuntimeError(f"WebSocket connect/recv to {ws_url} failed: {type(e).__name__}: {e}") from e

    if msg.get("type") != "snapshot":
        raise RuntimeError(f"expected first WebSocket message type 'snapshot', got {msg.get('type')!r}")
    return {"first_message_type": msg.get("type")}


def check_frames_changing(base_url: str, samples: int, interval: float) -> dict:
    seqs = []
    for i in range(samples):
        frame = check_live_frame(base_url, timeout=10.0)
        seqs.append(frame.get("sequence_number"))
        if i < samples - 1:
            time.sleep(interval)
    distinct = len(set(seqs))
    if distinct < 2:
        raise RuntimeError(f"sequence_number did not change across {samples} samples spaced {interval}s apart: {seqs}")
    return {"sequence_numbers": seqs}


def check_raw_port(host: str, port: int, timeout: float) -> dict:
    """Transport-level sanity check for the legacy raw port (5005) Unity's `LidarTCPClient.cs`
    connects to -- not a substitute for actually running the Unity Editor (this environment has
    no Unity Editor / GUI available), but proves the port accepts a TCP client and streams
    well-framed `<START>...<END>` scans, which is everything on the Python side of that
    connection this script can verify."""
    deadline = time.monotonic() + timeout
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        buf = b""
        while time.monotonic() < deadline:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            if b"<START>" in buf and b"<END>" in buf:
                return {"raw_frame_seen": True}
    raise RuntimeError(f"no complete <START>...<END> raw frame seen on {host}:{port} within {timeout}s")


def check_events(base_url: str, timeout: float, session_id: str | None = None) -> dict:
    """Defaults to the backend's own "current active session" scoping (`?session_id` omitted).
    A caller that already knows the exact session_id it cares about (e.g. `full-check`, right
    after a fresh bridge start) should pass it explicitly instead -- immediately after startup,
    "current session" resolution and this call can otherwise race the DB commit that opens the
    new `SessionRecord` row, undercounting events that are really there just a moment later."""
    url = f"{base_url}/events"
    if session_id:
        url += f"?session_id={session_id}"
    data = _get_json(url, timeout=timeout)
    if data is None:
        raise RuntimeError("/events returned null")
    return {"event_count": len(data), "event_types": sorted({e.get("event_type") for e in data})}


def check_stream_status(base_url: str, timeout: float) -> dict:
    data = _get_json(f"{base_url}/debug/stream-status", timeout=timeout)
    if data is None:
        raise RuntimeError("/debug/stream-status returned null")
    return data


# --- CLI plumbing ---


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("health")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--timeout", type=float, default=15.0)

    p = sub.add_parser("live-frame")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--timeout", type=float, default=15.0)

    p = sub.add_parser("source-id")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--expect", default=None)
    p.add_argument("--timeout", type=float, default=15.0)

    p = sub.add_parser("websocket")
    p.add_argument("--ws-url", default="ws://localhost:8000/ws/live")
    p.add_argument("--timeout", type=float, default=10.0)

    p = sub.add_parser("frames-changing")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--samples", type=int, default=3)
    p.add_argument("--interval", type=float, default=1.0)

    p = sub.add_parser("raw-port")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--timeout", type=float, default=10.0)

    p = sub.add_parser("events")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--session-id", default=None)

    p = sub.add_parser("full-check")
    p.add_argument("--base-url", default="http://localhost:8000")
    p.add_argument("--ws-url", default="ws://localhost:8000/ws/live")
    p.add_argument("--scenario", required=True)
    p.add_argument("--raw-host", default="127.0.0.1")
    p.add_argument("--raw-port", type=int, default=5005)
    p.add_argument("--timeout", type=float, default=20.0)

    args = parser.parse_args()

    try:
        if args.command == "health":
            result = check_health(args.base_url, args.timeout)
        elif args.command == "live-frame":
            result = check_live_frame(args.base_url, args.timeout)
        elif args.command == "source-id":
            result = check_source_id(args.base_url, args.expect, args.timeout)
        elif args.command == "websocket":
            result = check_websocket(args.ws_url, args.timeout)
        elif args.command == "frames-changing":
            result = check_frames_changing(args.base_url, args.samples, args.interval)
        elif args.command == "raw-port":
            result = check_raw_port(args.host, args.port, args.timeout)
        elif args.command == "events":
            result = check_events(args.base_url, args.timeout, args.session_id)
        elif args.command == "full-check":
            expected_source = f"simulated:{args.scenario}"
            summary: dict = {"scenario": args.scenario, "expected_source_id": expected_source}

            health = check_health(args.base_url, args.timeout)
            summary["health"] = health

            src = check_source_id(args.base_url, expected_source, args.timeout)
            summary["source_id"] = src["source_id"]
            summary["session_id"] = src["session_id"]

            frames = check_frames_changing(args.base_url, samples=3, interval=1.2)
            summary["sequence_numbers"] = frames["sequence_numbers"]
            summary["frames_changing"] = True

            frame = check_live_frame(args.base_url, timeout=5.0)
            objects = frame.get("objects") or []
            summary["object_count"] = len(objects)
            summary["track_ids"] = sorted({o.get("track_id") for o in objects if o.get("track_id") is not None})
            summary["classifications"] = sorted({o.get("classification") for o in objects if o.get("classification")})

            risk = frame.get("risk") or {}
            summary["overall_risk"] = risk.get("overall_risk")
            ttc_values = [r.get("ttc") for r in (risk.get("results") or []) if r.get("ttc") is not None]
            summary["ttc_values"] = ttc_values
            summary["ttc_present"] = len(ttc_values) > 0

            clearance = frame.get("clearance") or {}
            summary["clearance_overall_status"] = clearance.get("overall_status")
            summary["min_clearance_m"] = clearance.get("min_clearance_m")

            try:
                ws = check_websocket(args.ws_url, timeout=min(args.timeout, 10.0))
                summary["websocket_ok"] = True
                summary["websocket_first_message_type"] = ws["first_message_type"]
            except Exception as e:  # noqa: BLE001
                summary["websocket_ok"] = False
                summary["websocket_error"] = str(e)

            try:
                check_raw_port(args.raw_host, args.raw_port, timeout=min(args.timeout, 10.0))
                summary["raw_port_ok"] = True
            except Exception as e:  # noqa: BLE001
                summary["raw_port_ok"] = False
                summary["raw_port_error"] = str(e)

            events = check_events(args.base_url, timeout=5.0, session_id=summary["session_id"])
            summary["event_count"] = events["event_count"]
            summary["event_types"] = events["event_types"]

            stream_status = check_stream_status(args.base_url, timeout=5.0)
            summary["measured_scan_rate_hz"] = stream_status.get("measured_scan_rate_hz")
            summary["dashboard_clients_connected"] = stream_status.get("dashboard_clients_connected")

            result = summary
        else:  # pragma: no cover - argparse restricts choices
            _fail(f"unknown command {args.command!r}")
            return
    except Exception as e:  # noqa: BLE001 -- deliberately broad: this is a CLI, report and exit 1
        _fail(str(e), command=args.command)
        return

    _succeed(**result)


if __name__ == "__main__":
    main()

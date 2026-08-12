"""LiDAR-Vision360 local cloud backend.

Ingests the exact same structured JSON perception stream (`streaming_json_port`, default 5006)
Unity's `PerceptionTCPClient.cs` consumes -- as an ordinary second TCP client, not a relay behind
Unity -- and serves a REST + WebSocket API for `cloud/dashboard`. See docs/cloud.md.

**Single source of truth**: this package runs no perception logic of its own and starts no second
scenario/simulator instance -- every value it serves traces back to the one
`scripts/serve_unity_bridge.py` run currently streaming, via `ingestion.PerceptionIngestor`.
"""

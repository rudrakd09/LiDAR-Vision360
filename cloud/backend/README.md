# Cloud Backend

FastAPI + SQLAlchemy backend that ingests the same structured Python perception stream Unity
consumes (port 5006 by default) and serves it to `cloud/dashboard` over REST + WebSocket. See
[docs/cloud.md](../../docs/cloud.md) for the full architecture/API reference.

## Install

```bash
# from the repo root, after perception is already installed editable (see ../../README.md)
pip install -e "./cloud/backend[dev]"
```

## Run

```bash
# in one terminal: the perception pipeline + streaming servers
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --rate 10

# in another: this backend
uvicorn backend.main:app --app-dir cloud/backend/src --host 0.0.0.0 --port 8000
```

Then `curl http://localhost:8000/api/health`, or connect to `ws://localhost:8000/ws/live`.

## Test

```bash
pytest cloud/backend/tests
```

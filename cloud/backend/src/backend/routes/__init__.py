"""REST route modules -- each takes the shared app-level singletons (`LatestState`, `Database`,
`LiveBroadcastHub`) via FastAPI dependency injection (see `backend.main`'s `Depends` wiring), never
its own global state.
"""

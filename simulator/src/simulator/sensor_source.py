"""`SimulatorSource`: the concrete `SensorSource` selected by `Settings.data_source ==
"simulation"` (the default) -- see `common.config.Settings.data_source`,
`scripts/sensor_source.py`, and docs/architecture.md "Sensor source abstraction".

A thin composition/delegation adapter around `scenarios.make_data_source(...)` +
`SimulatedLiDARDataSource` (both pre-existing, unchanged): given only a scenario id/name, it
builds the full ray-casting simulated source and forwards every `datasources.SensorSource` method
to it. **No new frame-construction logic** -- `SimulatedLiDARDataSource` already implements
`datasources.LiDARDataSource` (== `SensorSource`) and already produces real
`datasources.SensorFrame` (== `ScanFrame`) instances; this class exists only so a caller composing
the pipeline (`scripts/sensor_source.py`'s `DATA_SOURCE` switch) can construct "the simulation
source for scenario X" and "the hardware source" through the exact same two-line shape, without
knowing anything about scenario files, `Environment`, `LiDARModel`, or `NoiseModel`.

Typical usage (identical to `simulator.make_data_source`, just addressed via the `SensorSource`
vocabulary):

    from simulator import SimulatorSource

    with SimulatorSource(scenario="08_approaching_obstacle") as source:
        frame = source.read_scan()   # a SensorFrame, ready for preprocessing.Preprocessor.process()
"""

from __future__ import annotations

from pathlib import Path

from common.config import Settings, get_settings
from datasources import SensorFrame, SensorSource

from .scenarios import SCENARIOS_DIR, make_data_source


class SimulatorSource(SensorSource):
    """Wraps a scenario-driven `SimulatedLiDARDataSource` behind the `SensorSource` interface.

    Args:
        scenario: scenario id or filename (e.g. `"08_approaching_obstacle"`) -- see
            `simulator.scenarios.get_scenario` for resolution rules.
        settings: resolved against scenario overrides the same way `make_data_source` already
            does. Defaults to `common.config.get_settings()`.
        real_time: forwarded to the underlying `SimulatedLiDARDataSource` -- see its own
            docstring (`False`/default = as-fast-as-possible, appropriate for tests/batch use;
            `True` paces `read_scan()` at the scenario's own scan frequency).
        directory: scenario JSON directory, defaults to `simulator/scenarios/`.
    """

    def __init__(
        self,
        scenario: str,
        settings: Settings | None = None,
        real_time: bool = False,
        directory: str | Path = SCENARIOS_DIR,
    ) -> None:
        self.scenario = scenario
        self.settings = settings or get_settings()
        # Delegate, don't re-implement: `make_data_source` already resolves the scenario spec
        # against `self.settings` and builds a ready-to-use `SimulatedLiDARDataSource` -- exactly
        # the same object `scripts/serve_unity_bridge.py` used directly before this abstraction
        # existed (see docs/architecture.md "Sensor source abstraction" for what changed and why).
        self._delegate = make_data_source(scenario, settings=self.settings, real_time=real_time, directory=directory)

    @property
    def source_id(self) -> str:
        """`"simulated:<scenario-id>"` -- forwarded from the underlying source unchanged (see
        `ScanFrame.source_id`, and docs/communication.md's own use of this exact value)."""
        return self._delegate.source_id

    def connect(self) -> None:
        self._delegate.connect()

    def disconnect(self) -> None:
        self._delegate.disconnect()

    def is_connected(self) -> bool:
        return self._delegate.is_connected()

    def read_scan(self) -> SensorFrame:
        return self._delegate.read_scan()

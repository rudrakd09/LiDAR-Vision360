"""Composition-root factory for the sensor input abstraction: builds the `SensorSource` selected
by `Settings.data_source` (`DATA_SOURCE` / `LIDAR_DATA_SOURCE` -- see `common.config.Settings`,
`.env.example`) -- `simulator.SimulatorSource` for `"simulation"` (the default), `datasources.
STM32Source` for `"hardware"`.

Both concrete sources already conform to `datasources.SensorSource` and already produce
`datasources.SensorFrame` (== `ScanFrame`) instances, so `scripts/serve_unity_bridge.py` (and
anything else wiring up the pipeline) depends on neither `simulator` nor STM32 specifics directly
-- only on this one switch and the resulting `SensorFrame`s.

This module lives in `scripts/`, not `perception/`, because it is the one place in this project
allowed to depend on *both* `perception` and `simulator`: `perception` must never import
`simulator` (see docs/architecture.md "Why `perception` and `simulator` are separate top-level
projects" -- the dependency is one-directional, `simulator` -> `perception`, never the reverse),
and `STM32Source` lives in `perception/datasources` (hardware has nothing to do with the simulator
package), so only a caller sitting above both -- a script, the existing composition-root pattern
this project already uses for wiring pipeline stages together -- can choose between them. The
`simulator` import below is deferred into the branch that actually needs it so that a
pure-hardware deployment is never required to have the `simulator` package installed at all.
"""

from __future__ import annotations

from common.config import Settings, get_settings
from datasources import STM32Source, SensorSource

_VALID_DATA_SOURCES = ("simulation", "hardware")


def get_sensor_source(
    settings: Settings | None = None,
    *,
    scenario: str | None = None,
    real_time: bool = False,
) -> SensorSource:
    """Returns the `SensorSource` for `settings.data_source`:

    - `"simulation"` (default): `simulator.SimulatorSource(scenario, settings, real_time)` --
      `scenario` is required (there is no simulated data without a scenario to run).
    - `"hardware"`: `datasources.STM32Source(settings)` -- still a placeholder; connecting raises
      `NotImplementedError` until the STM32 UART protocol is defined (see that class's own
      docstring). `scenario`/`real_time` are not applicable and ignored.

    Raises `ValueError` for any other `settings.data_source` value -- never silently falls back
    to one or the other.
    """
    settings = settings or get_settings()

    if settings.data_source == "simulation":
        if not scenario:
            raise ValueError(
                "Settings.data_source == 'simulation' requires a scenario id "
                "(e.g. --scenario 08_approaching_obstacle; see `python -m simulator.cli list`)."
            )
        from simulator import SimulatorSource  # deferred -- see module docstring

        return SimulatorSource(scenario=scenario, settings=settings, real_time=real_time)

    if settings.data_source == "hardware":
        return STM32Source(settings=settings)

    raise ValueError(
        f"Unknown Settings.data_source {settings.data_source!r} -- expected one of {_VALID_DATA_SOURCES}."
    )

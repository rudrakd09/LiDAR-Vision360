"""LIVE mode is exclusively real ESP32 data -- there is no automatic fallback to simulation.

`python edge/main.py --mode live --port COMx --baud 115200` must select the ESP32 serial source
and nothing else: if the hardware is absent or silent the pipeline reports the link as down and
publishes HARDWARE_DATA_UNAVAILABLE, it never substitutes generated points. Simulation is only
reachable by asking for it explicitly (`--mode simulation`).

This test drives the actual composition root (`scripts/sensor_source.get_sensor_source`) and the
actual launcher argument mapping (`edge/main.py`), not a re-implementation.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "scripts"))

from common.config import Settings  # noqa: E402
from datasources.esp32_serial import ESP32SerialSource  # noqa: E402
from sensor_source import get_sensor_source  # noqa: E402


def _load_edge_main():
    spec = importlib.util.spec_from_file_location("edge_main", _REPO_ROOT / "edge" / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestLiveModeSelectsRealHardwareOnly:
    def test_live_mode_maps_to_esp32_serial_data_source(self):
        edge_main = _load_edge_main()
        assert edge_main._MODE_TO_DATA_SOURCE["live"] == "esp32_serial"
        assert edge_main._MODE_TO_DATA_SOURCE["simulation"] == "simulation"
        # 'live' can only ever mean esp32_serial -- no third mapping sneaks simulation in.
        assert set(edge_main._MODE_TO_DATA_SOURCE.values()) == {"esp32_serial", "simulation"}

    def test_live_overrides_force_esp32_serial_even_if_env_said_simulation(self):
        edge_main = _load_edge_main()
        parser = edge_main.build_parser()
        args = parser.parse_args(["--mode", "live", "--port", "COM7", "--baud", "115200"])
        base = Settings(_env_file=None, data_source="simulation")
        resolved = edge_main._apply_overrides(args, base)
        assert resolved.data_source == "esp32_serial"
        assert resolved.esp32_serial_port == "COM7"
        assert resolved.esp32_serial_baudrate == 115200

    def test_get_sensor_source_for_esp32_serial_is_the_real_serial_source(self):
        settings = Settings(_env_file=None, data_source="esp32_serial", esp32_serial_port="COM_TEST")
        # No scenario passed -- a live source must not need one.
        source = get_sensor_source(settings, scenario=None)
        assert isinstance(source, ESP32SerialSource)

    def test_get_sensor_source_never_falls_back_to_simulation_on_the_live_branch(self, monkeypatch):
        # If anything on the esp32_serial branch tried to build a SimulatorSource, importing
        # `simulator` would be required -- make that explode and prove it is never reached.
        import builtins

        real_import = builtins.__import__

        def _boom(name, *a, **k):
            if name == "simulator" or name.startswith("simulator."):
                raise AssertionError("LIVE mode must never import the simulator package")
            return real_import(name, *a, **k)

        monkeypatch.setattr(builtins, "__import__", _boom)
        settings = Settings(_env_file=None, data_source="esp32_serial", esp32_serial_port="COM_TEST")
        source = get_sensor_source(settings, scenario="safe")  # scenario arg ignored on this branch
        assert isinstance(source, ESP32SerialSource)

    def test_simulation_still_requires_an_explicit_scenario(self):
        settings = Settings(_env_file=None, data_source="simulation")
        with pytest.raises(ValueError, match="requires a scenario"):
            get_sensor_source(settings, scenario=None)

    def test_unknown_data_source_raises_rather_than_guessing(self):
        settings = Settings(_env_file=None, data_source="bogus")
        with pytest.raises(ValueError):
            get_sensor_source(settings, scenario="safe")

"""Tests for `simulator.SimulatorSource` -- the `SensorSource` adapter around scenario-driven
`SimulatedLiDARDataSource`. Confirms the simulator no longer needs to be addressed through its own
concrete type to produce a valid `SensorFrame`: any of the 10 predefined scenarios must work
through this same adapter, exactly as they do through `make_data_source` directly.
"""

import pytest

from datasources import SensorFrame, SensorSource
from models import LiDARPoint, ScanFrame

from simulator import SimulatorSource
from simulator.scenarios import list_scenarios


class TestSimulatorSource:
    def test_is_a_sensor_source(self):
        assert issubclass(SimulatorSource, SensorSource)

    def test_source_id_reflects_scenario(self):
        source = SimulatorSource(scenario="01_empty")
        assert source.source_id == "simulated:01_empty"

    def test_context_manager_connects_and_disconnects(self):
        source = SimulatorSource(scenario="01_empty")
        assert not source.is_connected()
        with source:
            assert source.is_connected()
        assert not source.is_connected()

    def test_read_scan_returns_a_sensor_frame(self):
        with SimulatorSource(scenario="01_empty") as source:
            frame = source.read_scan()
        assert isinstance(frame, SensorFrame)
        assert isinstance(frame, ScanFrame)  # SensorFrame is ScanFrame -- see datasources/base.py
        assert frame.point_count > 0
        assert all(isinstance(p, LiDARPoint) for p in frame.points)

    def test_sequence_number_increments(self):
        with SimulatorSource(scenario="01_empty") as source:
            first = source.read_scan()
            second = source.read_scan()
        assert second.sequence_number == first.sequence_number + 1

    @pytest.mark.parametrize("scenario_id", [s.id for s in list_scenarios()])
    def test_every_predefined_scenario_produces_sensor_frames(self, scenario_id):
        """All 10 scenarios must keep working through the SensorSource abstraction -- this is
        the direct regression test for "the existing 10 scenarios MUST continue to work"."""
        with SimulatorSource(scenario=scenario_id) as source:
            frame = source.read_scan()
        assert isinstance(frame, SensorFrame)
        assert frame.source_id == f"simulated:{scenario_id}"
        assert frame.point_count > 0

# Hardware Integration

## Status

Placeholder only. Planned for **Phase 16** (`perception/src/datasources/serial_source.py`,
`embedded/stm32/`).

## What exists today

- `LiDARDataSource` (`perception/src/datasources/base.py`): the abstract interface every data
  source implements, so the perception engine never depends on physical hardware directly.
- `SerialLiDARDataSource` (`perception/src/datasources/serial_source.py`): a stub subclass whose
  `connect()`/`read_scan()` raise `NotImplementedError` with an explanatory message. It exists to
  show where the future integration plugs in, not to define behavior.

## What is intentionally not defined yet

The STM32 <-> PC UART packet/framing protocol (baud rate is a placeholder default only, frame
structure, checksums, angle/distance encoding on the wire, error signaling) is **not invented
here**, per `PROJECT_SPECIFICATION.md`. It will be defined once real hardware/firmware
requirements are known, then implemented in `SerialLiDARDataSource` and documented in this file.

Until then, `perception` develops entirely against `SimulatedLiDARDataSource`
(`docs/simulation.md`), which produces data in the identical `ScanFrame`/`LiDARPoint` shape.

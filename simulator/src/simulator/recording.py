"""Record simulated scans to disk and replay them back through the `LiDARDataSource` interface.

Format: **JSON Lines** (`.jsonl`) -- one JSON object per line, one line per `ScanFrame`, using
`ScanFrame`'s own pydantic serialization directly (no separate hand-rolled schema to drift out of
sync). This trivially satisfies "record scan_id, timestamp, angle, distance per scan" while also
preserving `valid`/`intensity` and every other model field. See docs/simulation.md for the format
and an example line.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Iterator

from datasources.base import LiDARDataSource
from models.scan import ScanFrame


def save_scan_jsonl(frame: ScanFrame, path: str | Path, append: bool = True) -> None:
    """Append a single `ScanFrame` as one line to a `.jsonl` file (created if missing)."""
    mode = "a" if append else "w"
    with open(path, mode, encoding="utf-8") as f:
        f.write(frame.model_dump_json())
        f.write("\n")


def save_scans_jsonl(frames: Iterable[ScanFrame], path: str | Path) -> int:
    """Write a full sequence of `ScanFrame`s to `path`, overwriting any existing file.

    Returns the number of frames written.
    """
    count = 0
    with open(path, "w", encoding="utf-8") as f:
        for frame in frames:
            f.write(frame.model_dump_json())
            f.write("\n")
            count += 1
    return count


def load_scans_jsonl(path: str | Path) -> list[ScanFrame]:
    """Read every `ScanFrame` recorded in a `.jsonl` file, in original order."""
    frames: list[ScanFrame] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(ScanFrame.model_validate_json(line))
    return frames


def iter_scans_jsonl(path: str | Path) -> Iterator[ScanFrame]:
    """Stream `ScanFrame`s from a `.jsonl` file one at a time, without loading it all into memory."""
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield ScanFrame.model_validate_json(line)


class RecordedLiDARDataSource(LiDARDataSource):
    """Replays a previously recorded `.jsonl` file of `ScanFrame`s as a `LiDARDataSource`.

    Loads the entire file into memory on `connect()` (recordings are expected to be
    scenario-sized, not unbounded logs). Set `loop=True` to replay indefinitely.
    """

    def __init__(self, path: str | Path, loop: bool = False) -> None:
        self.path = Path(path)
        self.loop = loop
        self._frames: list[ScanFrame] = []
        self._index = 0
        self._connected = False

    def connect(self) -> None:
        self._frames = load_scans_jsonl(self.path)
        self._index = 0
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        if not self._connected:
            return False
        return self.loop or self._index < len(self._frames)

    def read_scan(self) -> ScanFrame:
        if not self._connected:
            raise RuntimeError("RecordedLiDARDataSource.read_scan() called before connect().")
        if not self._frames:
            raise RuntimeError(f"No scans found in recording: {self.path}")
        if self._index >= len(self._frames):
            if self.loop:
                self._index = 0
            else:
                raise StopIteration(f"No more recorded scans in {self.path}")
        frame = self._frames[self._index]
        self._index += 1
        return frame

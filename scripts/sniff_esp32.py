"""Live capture + wire-format analyser for the ESP32 USB serial link.

WHY THIS EXISTS
---------------
The Edge project deliberately does not encode the STM32 wire format anywhere: every
`Settings.stm32_*` protocol field is `None`, and `datasources.stm32.parsers.UnconfiguredLiDARParser`
raises on every call, precisely so nothing is ever guessed (see docs/hardware-integration.md,
"What NOT to guess"). The format therefore exists only in the STM32/ESP32 firmware and on the wire.

This script reads the real bytes off the real port and *derives* the format from them, so the
LiDAR parser and the `LIDAR_STM32_*` configuration can be written from measured evidence rather
than assumption. It is a diagnostic tool: it never writes to the serial port, never touches the
firmware, and is not part of the runtime data path.

USAGE
-----
    python scripts/sniff_esp32.py --list
    python scripts/sniff_esp32.py --port COM7 --baud 115200 --seconds 3
    python scripts/sniff_esp32.py --port COM7 --scan-baud
    python scripts/sniff_esp32.py --port COM7 --baud 115200 --raw-out capture.bin

`--list` also prints VID:PID and description for each port, which is how you identify which COM
number the ESP32 enumerated as (see "How to identify the ESP32 port" in the output).
"""

from __future__ import annotations

import argparse
import collections
import sys
import time

# Baud rates tried by --scan-baud, commonly used on ESP32 USB-serial bridges.
CANDIDATE_BAUDS = (115200, 230400, 250000, 256000, 460800, 500000, 921600, 1000000, 9600, 57600)

PRINTABLE = set(range(0x20, 0x7F)) | {0x09, 0x0A, 0x0D}


# --------------------------------------------------------------------------------------------
# Port discovery
# --------------------------------------------------------------------------------------------

def list_ports() -> int:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial is not installed. Run:  pip install pyserial", file=sys.stderr)
        return 1

    ports = sorted(lp.comports(), key=lambda p: p.device)
    if not ports:
        print("No serial ports found. Is the ESP32 plugged in?")
        return 1

    print(f"{'PORT':<14} {'VID:PID':<12} DESCRIPTION")
    print("-" * 72)
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None and p.pid is not None else "-"
        print(f"{p.device:<14} {vidpid:<12} {p.description}")

    print(
        "\nESP32 boards use a USB-serial bridge; the common ones are:\n"
        "  10C4:EA60  Silicon Labs CP2102 / CP2104\n"
        "  1A86:7523  CH340 / CH341\n"
        "  1A86:55D4  CH9102\n"
        "  0403:6001  FTDI FT232\n"
        "  303A:1001  Espressif native USB (ESP32-S2/S3/C3)\n"
        "Match one of those VID:PIDs, or unplug the ESP32, re-run --list, and see which port "
        "disappeared."
    )
    return 0


# --------------------------------------------------------------------------------------------
# Capture
# --------------------------------------------------------------------------------------------

def capture(port: str, baud: float, seconds: float) -> bytes:
    import serial

    buf = bytearray()
    with serial.Serial(port=port, baudrate=int(baud), timeout=0.2) as ser:
        # Discard whatever was already sitting in the driver buffer: it may be a partial frame,
        # or stale bytes from before this process started, either of which would skew the
        # structural analysis below.
        ser.reset_input_buffer()
        time.sleep(0.1)
        ser.reset_input_buffer()

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)
    return bytes(buf)


# --------------------------------------------------------------------------------------------
# Structural analysis -- all measurements, no assumptions
# --------------------------------------------------------------------------------------------

def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    return sum(1 for b in data if b in PRINTABLE) / len(data)


def constant_offsets(data: bytes, period: int, min_share: float = 0.90) -> list[tuple[int, int, float]]:
    """For a hypothesised fixed record length `period`, report which byte positions within the
    record hold (almost) the same value every time. A real fixed-length format with a start marker
    or a constant message-type byte shows up as one or more such positions; unstructured data
    shows none. Returns [(offset, modal_byte, share), ...]."""
    records = len(data) // period
    if records < 8:
        return []

    out: list[tuple[int, int, float]] = []
    for off in range(period):
        counter = collections.Counter(data[off + i * period] for i in range(records))
        value, count = counter.most_common(1)[0]
        share = count / records
        if share >= min_share:
            out.append((off, value, share))
    return out


def score_periods(data: bytes, max_period: int = 64) -> list[tuple[int, int, list[tuple[int, int, float]]]]:
    """Rank candidate fixed record lengths by how many constant positions they produce.
    Returns [(period, n_constant_offsets, offsets), ...] best first."""
    scored = []
    for period in range(2, max_period + 1):
        offs = constant_offsets(data, period)
        if offs:
            scored.append((period, len(offs), offs))
    # Prefer more constant positions; break ties toward the SHORTER period, since any multiple of
    # a true period also scores (a 5-byte record trivially also looks constant at period 10).
    scored.sort(key=lambda t: (-t[1], t[0]))
    return scored[:6]


def repeated_markers(data: bytes, width: int, top: int = 5) -> list[tuple[bytes, int, list[int]]]:
    """Most frequent `width`-byte sequences, with the gaps between successive occurrences. A real
    start marker recurs at a near-constant stride; a coincidence does not."""
    counts: collections.Counter[bytes] = collections.Counter()
    for i in range(len(data) - width + 1):
        counts[data[i : i + width]] += 1

    results = []
    for seq, count in counts.most_common(40):
        if count < 8:
            continue
        positions = []
        start = 0
        while True:
            idx = data.find(seq, start)
            if idx < 0:
                break
            positions.append(idx)
            start = idx + 1
        gaps = [b - a for a, b in zip(positions, positions[1:])]
        if not gaps:
            continue
        modal_gap, modal_count = collections.Counter(gaps).most_common(1)[0]
        if modal_count / len(gaps) >= 0.80:  # consistent stride
            results.append((seq, modal_gap, gaps[:8]))
        if len(results) >= top:
            break
    return results


def looks_like_rplidar_nodes(data: bytes) -> tuple[bool, float, int]:
    """Test one specific, checkable hypothesis: that the stream is raw RPLIDAR standard-scan
    5-byte nodes passed straight through. Those have two self-validating bits -- byte0 bit0 (S)
    must be the inverse of byte0 bit1 (!S), and byte1 bit0 (the check bit) must be 1. Random or
    differently-structured data satisfies both at only ~25% per record.

    Returns (verdict, best_pass_rate, best_phase)."""
    best_rate, best_phase = 0.0, 0
    for phase in range(5):
        body = data[phase:]
        records = len(body) // 5
        if records < 20:
            continue
        ok = 0
        for i in range(records):
            b0 = body[i * 5]
            b1 = body[i * 5 + 1]
            if (b0 & 0x01) != ((b0 >> 1) & 0x01) and (b1 & 0x01):
                ok += 1
        rate = ok / records
        if rate > best_rate:
            best_rate, best_phase = rate, phase
    return best_rate >= 0.95, best_rate, best_phase


def hexdump(data: bytes, limit: int = 256) -> str:
    lines = []
    for off in range(0, min(len(data), limit), 16):
        row = data[off : off + 16]
        hexpart = " ".join(f"{b:02X}" for b in row).ljust(47)
        asciipart = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in row)
        lines.append(f"{off:06X}  {hexpart}  |{asciipart}|")
    return "\n".join(lines)


# --------------------------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------------------------

def analyse(data: bytes, *, port: str, baud: float, seconds: float) -> None:
    rate = len(data) / seconds if seconds else 0.0
    print("=" * 78)
    print(f"CAPTURE  port={port}  baud={int(baud)}  duration={seconds:.1f}s")
    print(f"         {len(data)} bytes  ({rate:,.0f} B/s = {rate * 10:,.0f} bit/s incl. framing)")
    print("=" * 78)

    if not data:
        print(
            "\nNO BYTES RECEIVED.\n"
            "  - Wrong port? Run with --list.\n"
            "  - Wrong baud? Run with --scan-baud.\n"
            "  - Another program may already hold the port (a serial monitor, the Arduino IDE).\n"
            "  - Some ESP32 boards need DTR/RTS deasserted; try closing any other monitor first."
        )
        return

    theoretical = baud / 10.0
    print(f"\nLink utilisation: {rate / theoretical:.1%} of {int(baud)} baud "
          f"({theoretical:,.0f} B/s theoretical max at 8N1).")
    if rate / theoretical > 0.85:
        print("  WARNING: >85% utilisation. The link is close to saturation; bytes may already be")
        print("  dropping upstream. Consider a higher baud rate on the STM32->ESP32 link.")

    print("\n--- FIRST BYTES ---")
    print(hexdump(data))

    ratio = printable_ratio(data)
    print(f"\n--- ENCODING ---")
    print(f"Printable-ASCII ratio: {ratio:.1%}")

    if ratio > 0.95:
        print("VERDICT: ASCII / text.")
        text = data.decode("ascii", errors="replace")
        lines = text.splitlines()
        print(f"\nLine count in capture: {len(lines)}")
        print("First 15 complete lines:")
        for line in lines[1:16]:  # skip lines[0], which is probably a partial line
            print(f"  {line!r}")
        if len(lines) > 3:
            sample = lines[1:200]
            for delim in (",", ";", "\t", " ", ":"):
                counts = collections.Counter(ln.count(delim) for ln in sample if ln)
                value, n = counts.most_common(1)[0]
                if value > 0 and n / max(1, len([ln for ln in sample if ln])) > 0.9:
                    print(f"\nConsistent delimiter {delim!r}: {value} per line "
                          f"=> {value + 1} fields per record.")
                    break
        print("\nNEXT STEP: the field order and units are now readable directly above. Map them")
        print("to LiDARPoint(angle_degrees, distance_metres, intensity) in a new")
        print("datasources/stm32/parsers_v1.py.")
        return

    print("VERDICT: binary.")

    is_rp, rp_rate, rp_phase = looks_like_rplidar_nodes(data)
    print(f"\n--- HYPOTHESIS: raw RPLIDAR 5-byte scan nodes passed through unmodified ---")
    print(f"Self-validating bits (S != !S, and check bit set) pass rate: {rp_rate:.1%} "
          f"at phase {rp_phase}")
    if is_rp:
        print("VERDICT: MATCH. The stream is raw RPLIDAR standard-scan nodes.")
        print("  byte0: [quality:6][!S][S]   byte1: [angle_q6 low 7][check=1]")
        print("  byte2: [angle_q6 high 8]    byte3/4: distance_q2 little-endian")
        print("  angle_deg = ((b2 << 7) | (b1 >> 1)) / 64      distance_mm = ((b4 << 8) | b3) / 4")
        body = data[rp_phase:]
        print("\n  First 10 decoded measurements from this capture:")
        for i in range(min(10, len(body) // 5)):
            b0, b1, b2, b3, b4 = body[i * 5 : i * 5 + 5]
            angle = (((b2 << 7) | (b1 >> 1)) / 64.0) % 360.0
            dist_mm = ((b4 << 8) | b3) / 4.0
            flag = "START" if (b0 & 0x01) else "     "
            print(f"    {flag} angle={angle:7.2f}deg  distance={dist_mm / 1000.0:6.3f} m  "
                  f"quality={b0 >> 2:3d}")
        return
    print("VERDICT: no match -- this is a custom format, analysed below.")

    print("\n--- FIXED-LENGTH RECORD CANDIDATES ---")
    print("(byte positions holding a near-constant value across every record of that length --")
    print(" a start marker or a constant message-type byte looks like this)")
    periods = score_periods(data)
    if not periods:
        print("None found. The format is probably delimited rather than fixed-length.")
    for period, n_const, offs in periods:
        detail = ", ".join(f"[{o}]=0x{v:02X} ({s:.0%})" for o, v, s in offs[:8])
        print(f"  record length {period:3d}:  {n_const} constant position(s)  {detail}")

    print("\n--- RECURRING MARKER CANDIDATES ---")
    print("(byte sequences that repeat at a consistent stride -- a real start marker does)")
    found_any = False
    for width in (2, 3, 4):
        for seq, stride, gaps in repeated_markers(data, width):
            found_any = True
            print(f"  {seq.hex().upper():<10} width={width}  stride={stride:3d} bytes  "
                  f"first gaps={gaps}")
    if not found_any:
        print("None found.")

    print("\nNEXT STEP: cross-check the record length and marker above against the STM32 transmit")
    print("code, then fill in LIDAR_STM32_FRAME_START_MARKER / _FRAME_LENGTH_BYTES / _BYTE_ORDER /")
    print("_MESSAGE_TYPE_OFFSET / _SEQUENCE_NUMBER_OFFSET / _CRC_ALGORITHM in .env and write the")
    print("matching LiDARMessageParser subclass. Nothing here should be guessed -- if the analysis")
    print("above is ambiguous, the transmit code is the authority.")


def scan_baud(port: str, seconds: float) -> None:
    print("Trying candidate baud rates. The correct one usually shows a high printable ratio")
    print("(ASCII) or a strong fixed-length structure (binary); wrong ones look like noise.\n")
    print(f"{'BAUD':>9}  {'BYTES/s':>9}  {'PRINTABLE':>9}  {'BEST RECORD LEN':>16}  RPLIDAR?")
    print("-" * 72)
    for baud in CANDIDATE_BAUDS:
        try:
            data = capture(port, baud, seconds)
        except Exception as e:  # noqa: BLE001 -- report and continue to the next baud
            print(f"{baud:>9}  error: {e}")
            continue
        if not data:
            print(f"{baud:>9}  {0:>9}  {'-':>9}  {'-':>16}  -")
            continue
        periods = score_periods(data)
        best = f"{periods[0][0]} ({periods[0][1]} const)" if periods else "-"
        is_rp, rp_rate, _ = looks_like_rplidar_nodes(data)
        print(f"{baud:>9}  {len(data) / seconds:>9,.0f}  {printable_ratio(data):>8.1%}  "
              f"{best:>16}  {'YES' if is_rp else f'{rp_rate:.0%}'}")
    print("\nRe-run with --baud <the winner> for the full analysis.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture and analyse the live ESP32 USB serial stream to derive its packet format."
    )
    parser.add_argument("--list", action="store_true", help="List serial ports with VID:PID and exit.")
    parser.add_argument("--port", help="Serial port, e.g. COM7 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200).")
    parser.add_argument("--seconds", type=float, default=3.0, help="Capture duration (default 3.0).")
    parser.add_argument("--scan-baud", action="store_true", help="Try every candidate baud rate and compare.")
    parser.add_argument("--raw-out", help="Also write the raw captured bytes to this file.")
    args = parser.parse_args()

    if args.list:
        return list_ports()

    if not args.port:
        print("--port is required (or use --list). Example: --port COM7", file=sys.stderr)
        return 2

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial is not installed. Run:  pip install pyserial", file=sys.stderr)
        return 1

    if args.scan_baud:
        scan_baud(args.port, min(args.seconds, 1.5))
        return 0

    try:
        data = capture(args.port, args.baud, args.seconds)
    except Exception as e:  # noqa: BLE001 -- a serial failure here is a user-facing message, not a traceback
        print(f"Could not read {args.port} at {args.baud} baud: {e}", file=sys.stderr)
        return 1

    if args.raw_out:
        with open(args.raw_out, "wb") as fh:
            fh.write(data)
        print(f"Wrote {len(data)} raw bytes to {args.raw_out}\n")

    analyse(data, port=args.port, baud=args.baud, seconds=args.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

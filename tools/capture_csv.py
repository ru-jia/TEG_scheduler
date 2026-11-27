#!/usr/bin/env python3
"""Save USB CSV rows. Requires pyserial; close other serial monitors first."""

import argparse
import csv
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="USB serial device")
    parser.add_argument("output", nargs="?", type=Path,
                        help="New CSV file to create (required unless --list is used)")
    parser.add_argument("--list", action="store_true", help="List serial devices")
    parser.add_argument("--duration", type=float, default=0,
                        help="Seconds to record after opening the port; 0 means until Ctrl-C")
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("--duration must be nonnegative")
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.exit(1, "Install pyserial in your Python environment first.\n")
    if args.list:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return
    if not args.port or not args.output:
        parser.error("port and output file are required unless --list is used")
    output_path = args.output
    count = 0
    header = None
    started = time.monotonic()
    # Exclusive creation prevents accidental replacement of an earlier trial.
    with output_path.open("x", newline="") as output:
        writer = csv.writer(output)
        try:
            with serial.Serial(args.port, 115200, timeout=1, exclusive=True) as device:
                device.dtr = True
                print(f"Recording to {output_path}. Press Ctrl-C to stop.", file=sys.stderr)
                while not args.duration or time.monotonic() - started < args.duration:
                    line = device.readline().decode("ascii", errors="replace").strip()
                    if not line or line.startswith("#"):
                        continue
                    row = next(csv.reader([line]))
                    if row[0] == "time_s":
                        if "vst_voltage_V" not in row or "vout_voltage_V" not in row:
                            raise ValueError("Firmware CSV does not contain both voltages")
                        if header is None:
                            header = row
                            writer.writerow(header)
                            output.flush()
                        elif row != header:
                            raise ValueError("CSV schema changed during capture")
                        continue
                    if header is None or len(row) != len(header):
                        continue
                    try:
                        for value in row:
                            float(value)
                    except ValueError:
                        continue
                    writer.writerow(row)
                    output.flush()
                    count += 1
        except KeyboardInterrupt:
            pass
    print(f"Saved {count} rows to {output_path}", file=sys.stderr)
    if count == 0:
        raise SystemExit("No measurements received; check firmware, port and USB connection")


if __name__ == "__main__":
    main()

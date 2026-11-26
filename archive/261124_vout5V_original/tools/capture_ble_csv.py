#!/usr/bin/env python3
"""Save the XIAO's BLE measurement stream as CSV using Bleak."""

import argparse
import asyncio
import csv
import math
import sys
from pathlib import Path


NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
COLUMNS = (
    "time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
    "vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
    "temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status"
).split(",")
CONTROLLER_COLUMNS = (
    "cap_energy_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,"
    "battery_start_mWh"
).split(",")


async def capture(output_path, full=False, battery_mwh=None):
    from bleak import BleakClient, BleakScanner

    print("Scanning for TEG_XIAO…", file=sys.stderr, flush=True)
    device = await BleakScanner.find_device_by_name(
        "TEG_XIAO", timeout=15, service_uuids=[NUS_SERVICE]
    )
    if device is None:
        raise RuntimeError("TEG_XIAO was not found. Check power and disconnect other BLE clients.")

    loop = asyncio.get_running_loop()
    finished = loop.create_future()
    columns = COLUMNS + CONTROLLER_COLUMNS if full else COLUMNS

    def on_disconnect(_client):
        if not finished.done():
            finished.set_result(None)

    count = 0
    pending = b""

    def first_row_timeout():
        if count == 0 and not finished.done():
            message = "No measurements received over BLE."
            if full:
                message += " Check that the XIAO is running firmware with full BLE CSV support."
            finished.set_exception(RuntimeError(message))

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        with output_path.open("x", newline="", encoding="ascii") as output:
            writer = csv.writer(output)
            writer.writerow(columns)
            output.flush()

            def on_notification(_characteristic, data):
                nonlocal pending, count
                if finished.done():
                    return
                # NUS can split a newline-terminated row across multiple notifications.
                lines = (pending + data).split(b"\n")
                pending = lines.pop()
                for line in lines:
                    try:
                        fields = line.decode("ascii").strip().split(",")
                        if len(fields) != len(columns):
                            continue
                        values = [float(field) for field in fields]
                    except ValueError:
                        continue
                    if math.isfinite(values[0]) and values[0] >= 0:
                        try:
                            writer.writerow(fields)
                            output.flush()
                        except OSError as error:
                            finished.set_exception(error)
                            return
                        count += 1
                        if count == 1:
                            print(f"Recording to {output_path}. Press Ctrl-C to stop.",
                                  file=sys.stderr, flush=True)

            print("Connected. Waiting for the first measurement…", file=sys.stderr, flush=True)
            if full:
                command = b"csv" if battery_mwh is None else f"csv {battery_mwh:.9g}".encode("ascii")
                await client.write_gatt_char(NUS_RX, command, response=True)
            await client.start_notify(NUS_TX, on_notification)
            first_row_timer = loop.call_later(45, first_row_timeout)
            try:
                await finished
            finally:
                first_row_timer.cancel()
                if client.is_connected:
                    await client.stop_notify(NUS_TX)
                print(f"Saved {count} rows to {output_path}", file=sys.stderr)
    raise RuntimeError("TEG_XIAO disconnected.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Path to a new CSV file")
    parser.add_argument("--full", action="store_true", help="Save all measurement, controller, and battery columns")
    parser.add_argument("--battery-mwh", type=float, help="Starting battery energy for this run (requires --full)")
    args = parser.parse_args()
    if args.battery_mwh is not None:
        if not args.full or not math.isfinite(args.battery_mwh) or args.battery_mwh < 0.0:
            parser.error("--battery-mwh requires --full and a finite, nonnegative value.")
    if args.output.exists():
        parser.error(f"{args.output} already exists. Choose a new filename.")
    try:
        from bleak.exc import BleakError
    except ImportError:
        parser.exit(1, "Install Bleak in this Python environment with python -m pip install bleak.\n")
    try:
        asyncio.run(capture(args.output, args.full, args.battery_mwh))
    except KeyboardInterrupt:
        pass
    except (BleakError, OSError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()

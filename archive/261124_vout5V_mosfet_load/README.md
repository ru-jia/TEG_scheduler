# TEG scheduler

XIAO nRF52840 firmware on nRF Connect SDK 3.3.4 / Zephyr. Reads D0 VOUT and D1 VSTORE,
ADS1115 VIN, and SHT40 temperature/humidity; sends CSV over USB and BLE.
USB readings and phone BLE notifications work. The capacitor PID adjusts
the shared sensing and BLE interval for bench tests. Direct TEG/LTC power
could not start XIAO.

## Wiring

Use USB power for XIAO and share ground with LTC and both sensors.

| Connection | Wiring |
| --- | --- |
| VSTORE measurement | `VSTORE -> 1 Mohm -> D1/A1 -> 330 kohm -> GND` |
| VOUT measurement | `VOUT -> 1 Mohm -> D0/A0 -> 330 kohm -> GND` |
| Sensor power | XIAO 3V3 to ADS1115 VDD and SHT40 VIN |
| I2C | Both SDA to D4; both SCL to D5 |
| ADS1115 | ADDR to GND (`0x48`); A0 to LTC VIN/TEG positive |
| SHT40 | Address `0x44` |

ADS1115 reads A0 against GND in single-shot mode. Keep its input positive
and below the configured 256 mV full scale. D0 measures VOUT and D1 measures VSTORE. Do not join the divider midpoints.
The 0.1 F storage capacitor stays between VSTORE and GND. For a 5 V
LTC3108 output, connect both VS1 and VS2 to VAUX, not XIAO 3V3.

## Readings and calibration

Each cycle averages 20 samples per
XIAO channel and reads both sensors. The first conversion after selecting each
channel is discarded. The commanded cycle period includes acquisition and BLE.
The baseline targets 1 second. Adaptive cycles target 1–30 seconds, and slow
reads or BLE output can extend a cycle. Channels are read sequentially.
The timestamp is board uptime at the start of acquisition, not experiment time zero.
Opening USB serial at 115200 baud prints the header and continuous CSV:

```text
time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status,cap_energy_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,battery_start_mWh,load_pulse_uJ,pid_p_Hz,pid_i_Hz,pid_d_Hz
```

`vst_voltage_V` is calibrated in firmware using averaged `vst_adc_raw` times
`VST_CAL_GAIN_V_PER_COUNT` plus `VST_CAL_OFFSET_V` in `src/main.c`. The fit
maps counts directly to capacitor volts and includes the divider. The November 22,
2026 charge-run calibration uses 209 time-matched reference pairs across nine
captures, weighted equally by capture. The reference pairs span 0.608 to 5.180 V.
The capture-weighted RMS residual is 13 mV, and the RMS error when holding out
each capture in turn is 17 mV. Readings outside 162.4 to 1464.8 D1 counts
are extrapolations.
`d1_voltage_V` reports voltage at the ADC pin.

`vout_voltage_V` uses a one-point estimate for the 5 V setting. It scales the
averaged D0 count by 5.09 V / 1411.3 counts, using medians from the same
measurement window. Accuracy away from 5 V is unverified. To refine it,
set `VOUT_CAL_VOLTAGE` and `VOUT_CAL_ADC` to a simultaneous multimeter and
averaged D0 pair. Setting both to zero restores the nominal 1.33 / 0.33
divider multiplier. `vin_mV` uses the ADS1115's ADC conversion.

The divider's source resistance is about 248 kohm (1 Mohm parallel with
330 kohm). Both channels use 40 us acquisition, within the nRF52840's
800 kohm source-resistance limit at that setting. No additional ADC filter
capacitors are required for this configuration. The 0.1 F storage capacitor
remains on VSTORE. Compare settled readings at multiple voltages against a
multimeter; resistor tolerance, input leakage and breadboard wiring can still
cause errors. At 5 V, each divider draws about 3.76 uA (18.8 uW), which loads
the harvested supply and reduces its measured net charging rate.

### Save USB data

Install `pyserial` in your Python environment, then close other serial monitors.
Run from this project directory:

```sh
python3 tools/capture_csv.py --list
python3 tools/capture_csv.py /dev/cu.usbmodemXXXX --duration 600
python3 tools/capture_csv.py /dev/cu.usbmodemXXXX --battery-mwh 750 --duration 600
```

By default, the script creates a timestamped CSV under
`261119_energyBudgetModeler/261122_capCharging` in the TEG workspace. Pass an
explicit CSV path after the serial port to override the default.
Omit `--duration` to record until Ctrl-C. The script excludes comment lines,
flushes each row to disk and refuses to overwrite an existing file. `nan` values
and device error statuses remain in the CSV. To enter starting battery
energy manually, send `battery_mwh 750` followed by Enter over the USB
serial console. Replace 750 with the run's starting energy in mWh. The
firmware acknowledges the value at the next cycle, up to about 30 s later.
The capture script waits for that reply when `--battery-mwh` is supplied.
The value stays in RAM until reset or until you enter a new one. It is run
metadata for USB capture, independent of the capacitor PID.

For capacitor charging, start
capture before applying the thermal stimulus. Plot `vst_voltage_V` against
`time_s` for rows with `vst_status = 0`. Subtract the board uptime at stimulus
onset to set the experiment time origin. The slope over a selected interval
is the change in VSTORE divided by elapsed time.

Status: `0` = success, `-19` = device unavailable at startup,
`-34` = ADS1115 full scale. Failed values appear as `nan`.
Check power, wiring, and addresses; reset after fixing startup failures.

## Phone BLE test

1. In Nordic nRF Connect for Mobile, scan and connect to `TEG_XIAO`.
2. Open Nordic UART Service (`6e400001…`); enable TX notifications (`6e400003…`).
3. Select UTF-8/text to see the measurement CSV. It includes the columns
   through `sht_status`. The Mac logger can request the PID trace over BLE.

With a small MTU, join notification fragments through the newline to
reconstruct a row. Advertising restarts on disconnect. Reconnection still
needs testing.

### Save battery-powered measurements over BLE

With the XIAO powered by the LiPo and USB disconnected, disconnect any phone
or ESP32 BLE receiver. The firmware supports one BLE connection at a time.
On the Mac, enable Bluetooth and run from this directory:

```sh
python -m pip install bleak
python tools/capture_ble_csv.py lipo-run-01.csv
```

Allow Bluetooth access for Terminal if macOS asks. Start the experiment when
the script reports that it is recording. Press Ctrl-C to finish. Use a new
filename for each run. The default CSV contains the BLE measurement columns
through `sht_status`.

Build and flash the updated XIAO firmware once to capture the controller
columns over BLE. Unplug USB-C for the run and use `--full`:

```sh
python tools/capture_ble_csv.py lipo-full-01.csv --full
python tools/capture_ble_csv.py lipo-full-02.csv --full --battery-mwh 750
```

With `--full`, the logger saves all 26 columns in the USB CSV order, from
`time_s` through `pid_d_Hz`. `--battery-mwh` sends the starting
battery energy in milliwatt-hours to the XIAO for this BLE connection. If
omitted, the firmware uses its USB-entered value if present, or reports
`nan`. Adaptive cycles with an invalid capacitor energy reading skip BLE
output. `ble_sent` is `1` in received BLE rows.

## Capacitor energy PID test

The nRF runs from the LiPo, so the firmware gives each event a real cost at the
capacitor. Every cycle, XIAO D2 switches a MOSFET on for `LOAD_PULSE_MS`, and
current flows from VSTORE through `LOAD_RESISTOR_OHM` to ground. The TEG fills
the capacitor, events drain it, and the PID sets how often events happen.
Wiring: `261124 TEG sensor-only battery build`, Section 5.

The controller output is an event rate in Hz; `event_interval_ms` is 1000 / rate,
clamped to 1–30 s. The error is `cap_energy_J - cap_target_J`, so a positive
error means surplus energy and a faster rate. (Earlier builds logged
target − energy.) The target is fixed at `ENERGY_TARGET_FRACTION` (0.8) times
the first valid capacitor energy of the run and repeats on every row. Start
each run with VSTORE near full. Starting gains: `PID_KP 40` Hz/J,
`PID_KI 0.15` Hz/(J·s), `PID_KD 0`, `RATE_BIAS_HZ 0.2`. Change one gain per
build and compare runs at the same thermal condition.

| Setting | Effect |
| --- | --- |
| `PID_ENABLED 0` | Fixed `BASELINE_INTERVAL_MS`, load still pulses: shows the drain at that rate |
| `LOAD_ENABLED 0` | No pulses: the capacitor only charges, as in earlier builds |
| `LOAD_PULSE_MS`, `LOAD_RESISTOR_OHM` | Event cost, about V² / R × t |

New columns: `load_pulse_uJ` (estimated energy of this cycle's pulse),
`pid_p_Hz`, `pid_i_Hz` and `pid_d_Hz` (the three PID terms). `cap_net_power_uW`
is the smoothed slope of capacitor energy: harvest minus pulses and divider
loss. If the energy stays above target at the 1 s limit, the harvest exceeds
the load the fastest rate can spend. Increase `LOAD_PULSE_MS`.
If it stays below target at 30 s, increase the harvest or shorten the pulse.

## Build and flash

Run in this directory from an nRF Connect SDK 3.3.4 terminal:

```sh
west build -b xiao_ble/nrf52840 -d build-adc --sysbuild .
```

Double-press RESET; copy `build-adc/TEG_scheduler/zephyr/zephyr.uf2`
to the XIAO bootloader drive. Firmware runs automatically after flashing.

## Files

| File | Purpose |
| --- | --- |
| `src/main.c` | Measurements, conversion, capacitor load pulse, energy PID, USB output |
| `src/ble_output.c`, `.h` | BLE advertising, connections, NUS output |
| `tools/capture_ble_csv.py` | Record the BLE measurement stream to a CSV on the host |
| `prj.conf` | Zephyr features |
| `boards/xiao_ble_nrf52840.overlay` | ADC pin, gain, resolution |
| `CMakeLists.txt` | Build definition |

## Reflection on power integration

I compared TEG configurations, tested an LTC3108 boost circuit and charging of
a 0.1 F capacitor, and built a USB-powered XIAO node that reads voltage,
temperature and humidity and sends data to a phone over BLE.

When I used the TEG and LTC to power XIAO directly, the board did not start.
Even with hot and cold surfaces at 66.6°C and 41.7°C, the output dropped from
5.07 V without XIAO to about 1.05 V with it connected. Reaching 5 V without
a load was not enough to run the node.

Personal environmental sensing remains the goal. I am now considering a
battery-powered node with TEG harvesting to reduce battery use. The next
question is whether the harvested energy can measurably extend battery life,
and how transmission frequency can balance responsiveness and energy use.

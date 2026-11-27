# TEG scheduler

XIAO nRF52840 firmware on nRF Connect SDK 3.3.4 / Zephyr. Reads D0 VOUT and D1 VSTORE,
ADS1115 VIN, and SHT40 temperature/humidity; sends CSV over USB and BLE.
**Current hardware:** LTC3108 VOUT set to 3.3 V powers the SHT40 and ADS1115; the
XIAO runs from USB. Wiring: `261124 TEG harvester-powered sensors build`.
Earlier 5 V firmware is kept in `archive/` (see `archive/README.md`).
USB readings and phone BLE notifications work. The capacitor PID adjusts
the shared sensing and BLE interval for bench tests. Direct TEG/LTC power
could not start XIAO.

## Wiring

Use USB power for XIAO and share ground with LTC and both sensors.

| Connection | Wiring |
| --- | --- |
| VSTORE measurement | `VSTORE -> 1 Mohm -> D1/A1 -> 330 kohm -> GND` |
| VOUT measurement | `VOUT -> 1 Mohm -> D0/A0 -> 330 kohm -> GND` |
| Sensor power | LTC3108 VOUT (3.3 V: VS1 to VAUX, VS2 to GND) to ADS1115 VDD and SHT40 VIN |
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
The baseline targets 1 second. Adaptive cycles target 0.5–30 seconds, and slow
reads or BLE output can extend a cycle. Channels are read sequentially.
The timestamp is board uptime at the start of acquisition, not experiment time zero.
Opening USB serial at 115200 baud prints the header and continuous CSV:

```text
time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status,cap_energy_J,cap_energy_filt_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,pid_p_Hz,pid_i_Hz,pid_d_Hz
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
python3 tools/capture_csv.py /dev/cu.usbmodemXXXX run-01.csv --duration 600
```

Give the CSV path after the serial port; a relative path is relative to the
folder you run the command from.
Omit `--duration` to record until Ctrl-C. The script excludes comment lines,
flushes each row to disk and refuses to overwrite an existing file. `nan` values
and device error statuses remain in the CSV.

For capacitor charging, start
capture before applying the thermal stimulus. Plot `vst_voltage_V` against
`time_s` for rows with `vst_status = 0`. Subtract the board uptime at stimulus
onset to set the experiment time origin. The slope over a selected interval
is the change in VSTORE divided by elapsed time.

Status: `0` = success, `-11` = sensors skipped because VOUT < 3.0 V,
`-19` = device unavailable, `-34` = ADS1115 full scale, `-77` = SHT40 checksum
error, other negatives = I2C errors. Failed values appear as `nan`.

## Phone BLE test

1. In Nordic nRF Connect for Mobile, scan and connect to `TEG_XIAO`.
2. Open Nordic UART Service (`6e400001…`); enable TX notifications (`6e400003…`).
3. Select UTF-8/text to see the measurement CSV. It includes the columns
   through `sht_status`. The Mac logger can request the PID trace over BLE.

With a small MTU, join notification fragments through the newline to
reconstruct a row. Advertising restarts on disconnect. Reconnection still
needs testing.

### Save measurements over BLE

Disconnect any phone
or ESP32 BLE receiver. The firmware supports one BLE connection at a time.
On the Mac, enable Bluetooth and run from this directory:

```sh
python -m pip install bleak
python tools/capture_ble_csv.py ble-run-01.csv
```

Allow Bluetooth access for Terminal if macOS asks. Start the experiment when
the script reports that it is recording. Press Ctrl-C to finish. Use a new
filename for each run. The default CSV contains the BLE measurement columns
through `sht_status`.

To also capture the controller columns, use `--full`:

```sh
python tools/capture_ble_csv.py ble-full-01.csv --full
```

With `--full`, the logger writes `csv` to the XIAO, which then sends all 25
columns for that connection, in the USB CSV order from `time_s` through
`pid_d_Hz`. `ble_sent` is `1` in received BLE rows.

## Capacitor energy PID test

The SHT40 and ADS1115 run from LTC3108 VOUT (3.3 V), so every reading is paid
for from the harvester, and the 0.1 F capacitor on VSTORE is the reserve. The
PID sets how often to read so the reserve stays at its target. The sensors are
read directly over I²C (no Zephyr sensor drivers) and only while VOUT ≥
`VOUT_MIN_FOR_SENSORS_V` (3.0 V). Otherwise `sht_status` and `ads_status` are
`-11` and the read is retried next cycle.

The controller output is an event rate in Hz; `event_interval_ms` is 1000 / rate,
clamped to 0.5–30 s. The controller works on `cap_energy_filt_J`, a 10 s running
average of `cap_energy_J`, because ADC noise (about 1 mJ) is larger than one
event (tens of µJ). `cap_error_J` = filtered energy − target; positive means
surplus and a faster rate. The target is `ENERGY_TARGET_FRACTION` (1.0) times
the first valid energy of the run: holding the starting level keeps net power at
zero (energy-neutral sensing) without slow drift. Start below the 5.2 V ceiling. Default gains assume about 20 µJ per
event: `PID_KP 500`, `PID_KI 1.0`, `PID_KD 0`, `RATE_BIAS_HZ 0.5`.

### What to expect

The controller can hold the target only when harvest lies between the fixed
loads (dividers, sensor idle: about 10–20 µW) and the most the sensors can spend
at 0.5 s. Above that, the capacitor stays full at the fastest rate; below it,
the rate sits at 30 s and the reserve drains. A fraction below 1.0 is a step
test, and a slow one: each event is cheap, so draining 20% takes hours.

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
| `src/main.c` | Measurements, direct I²C sensor reads, energy PID, USB output |
| `src/ble_output.c`, `.h` | BLE advertising, connections, NUS output |
| `tools/capture_csv.py` | Record the USB CSV stream to a file on the host |
| `tools/capture_ble_csv.py` | Record the BLE CSV stream to a file on the host |
| `prj.conf` | Zephyr features |
| `boards/xiao_ble_nrf52840.overlay` | ADC pins, gain, resolution; I²C bus |
| `archive/` | Earlier 5 V firmware snapshots, not built |
| `CMakeLists.txt` | Build definition |

## Reflection on power integration

I compared TEG configurations, tested an LTC3108 boost circuit and charging of
a 0.1 F capacitor, and built a USB-powered XIAO node that reads voltage,
temperature and humidity and sends data to a phone over BLE.

When I used the TEG and LTC to power XIAO directly, the board did not start.
Even with hot and cold surfaces at 66.6°C and 41.7°C, the output dropped from
5.07 V without XIAO to about 1.05 V with it connected. Reaching 5 V without
a load was not enough to run the node.

So the node is now split by power need. The XIAO and BLE run from USB. The
SHT40 and ADS1115 run from LTC3108 VOUT at 3.3 V, so every sensor reading is
paid for with harvested energy, and the 0.1 F capacitor on VSTORE is the
reserve that shows how much is left. An earlier version charged harvest to the
capacitor but drew the sensing cost from USB or the battery, so the controller was
balancing two separate energy stores; this setup fixes that.

Personal environmental sensing remains the goal. The next question is how
often harvested energy alone can pay for sensing: first measuring the sensors'
idle power and the energy of one reading, then letting the controller find the
energy-neutral interval and seeing how it changes with the temperature
difference across the TEG.

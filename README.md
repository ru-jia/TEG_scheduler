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
time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status,cap_energy_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,battery_start_mWh
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
   through `sht_status`. The PID trace is available over USB.

Keep USB power connected; the serial monitor can stay closed. With a small
MTU, join notification fragments through the newline to reconstruct a row.
Advertising restarts on disconnect; reconnection still needs testing.

## Capacitor PID test

Set `PID_ENABLED` to `0` in `src/main.c` for the fixed one-second sense-and-send
baseline. For tracking, set it to `1`, build, and flash. The first valid
VSTORE reading after boot sets the reference to half that reading's capacitor
energy. Begin each gain trial at the same VSTORE and with the same phone
connection and thermal conditions. The PID uses the 0.1 F storage capacitor
and adjusts the interval between complete sensing and sending cycles. Each
cycle updates the PID using the actual elapsed time since the preceding sample
and attempts to send the row when BLE is available.

Start with `PID_KP` and `PID_KI = PID_KD = 0`. Change one gain per build.
Plot `cap_energy_J` and `cap_target_J` against `time_s`, then compare
`cap_error_J`, `event_interval_ms`, and `ble_sent` between runs. The
interval is the target time from this cycle's start to the next. Compare
successive `time_s` values to see the actual cycle spacing and use
`ble_sent` to identify successful send calls. The first sample has no
slope estimate. A failed adaptive VSTORE read skips BLE and retries on
the next cycle at the 30 s interval.

`cap_net_power_uW` is the filtered slope of measured capacitor energy. It
includes charging and any load seen at VSTORE. If the capacitor keeps gaining
energy even with a connected phone receiving every second, the scheduler
cannot lower it toward the reference under those conditions. The 1 s baseline
can show whether the reference is reachable before comparing gains.

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
| `src/main.c` | Measurements, conversion, capacitor PID, USB output |
| `src/ble_output.c`, `.h` | BLE advertising, connections, NUS output |
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

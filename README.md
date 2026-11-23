# TEG scheduler

XIAO nRF52840 firmware on nRF Connect SDK 3.3.4 / Zephyr. Reads D0 VOUT and D1 VSTORE,
ADS1115 VIN, and SHT40 temperature/humidity; sends CSV over USB and BLE.
USB readings and phone BLE notifications work. Energy-aware scheduling is
pending, and direct TEG/LTC power could not start XIAO.

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
channel is discarded. Cycles target 1 second including acquisition; slow reads
or BLE output can extend this. Channels are read sequentially, not simultaneously.
The timestamp is board uptime at the start of acquisition, not experiment time zero.
Opening USB serial at 115200 baud prints the header and continuous CSV:

```text
time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status
```

`vst_voltage_V` is calibrated in firmware using averaged `vst_adc_raw` times
`VST_CAL_GAIN_V_PER_COUNT` plus `VST_CAL_OFFSET_V` in `src/main.c`. The fit
maps counts directly to capacitor volts and includes the divider. It uses nine
time-matched multimeter medians from the November 20–21 charging run, spanning
0.818 to 2.480 V. The fit is preliminary, with a 71 mV RMS residual across
those medians and a 144 mV RMS error when holding out each reference in turn.
Readings outside 181.4 to 555.8 D1 counts are extrapolations.
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
```

By default, the script creates a timestamped CSV under
`261119_energyBudgetModeler/261122_capCharging` in the TEG workspace. Pass an
explicit CSV path after the serial port to override the default.
Omit `--duration` to record until Ctrl-C. The script excludes comment lines,
flushes each row to disk and refuses to overwrite an existing file. `nan` values
and device error statuses remain in the CSV. For capacitor charging, start
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
3. Select UTF-8/text to see CSV about every 5 seconds, using the columns above.

Keep USB power connected; the serial monitor can stay closed. With a small
MTU, join notification fragments through the newline to reconstruct a row.
Advertising restarts on disconnect; reconnection still needs testing.

`BLE_TEST_INTERVAL_MS = 5000` controls the current test interval.
`INITIAL_TX_INTERVAL_MS = 300000` is reserved for later scheduling.

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
| `src/main.c` | Measurements, conversion, USB output |
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

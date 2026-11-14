# TEG scheduler

XIAO nRF52840 firmware on nRF Connect SDK 3.3.4 / Zephyr. Reads D0 voltage,
ADS1115 VIN, and SHT40 temperature/humidity; sends CSV over USB and BLE.
USB readings and phone BLE notifications work. Voltage calibration and
energy-aware scheduling are pending; direct TEG/LTC power could not start XIAO.

## Wiring

Use USB power for XIAO and share ground with LTC and both sensors.

| Connection | Wiring |
| --- | --- |
| VOUT measurement | `VOUT -> 1 Mohm -> D0/A0 -> 330 kohm -> GND` |
| Sensor power | XIAO 3V3 to ADS1115 VDD and SHT40 VIN |
| I2C | Both SDA to D4; both SCL to D5 |
| ADS1115 | ADDR to GND (`0x48`); A0 to LTC VIN/TEG positive |
| SHT40 | Address `0x44` |

ADS1115 reads A0 against GND in single-shot mode. Keep its input positive
and below the configured 256 mV full scale. D0 measures VOUT, not VSTORE.

## Readings and calibration

Measurements and BLE start at boot. Each cycle averages 20 D0 samples,
reads both sensors, then waits 1 second. Opening USB serial at 115200 baud
prints the header and continuous CSV:

```text
time_s,adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,adc_status,ads_status,sht_status
```

In `src/main.c`, set `CAL_VOLTAGE` to multimeter VOUT and `CAL_ADC` to the
simultaneous averaged raw ADC reading. Leaving both zero uses the nominal
divider ratio. Voltage trends have been checked; accuracy is not yet calibrated.

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

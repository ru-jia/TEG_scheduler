# TEG scheduler

XIAO nRF52840 firmware using nRF Connect SDK 3.3.4 and Zephyr.
Current functionality: single-channel ADC measurement with USB CSV output.
ADS1115, SHT40, BLE, and scheduling are not implemented yet.
The initial BLE interval is set aside as `INITIAL_TX_INTERVAL_MS = 300000`
(300 seconds). The current ADC test still waits 1 second between readings.

## Wiring and output

Power the XIAO over USB and share ground with the LTC3108 board.
Connect `VOUT -> 1 Mohm -> D0/A0 -> 330 kohm -> GND`.
The firmware waits for the USB serial connection, averages 20 samples,
prints one CSV row, and waits 1 second before repeating.

```text
time_s,adc_raw,d0_voltage_V,vout_voltage_V
```

Set `CAL_VOLTAGE` and `CAL_ADC` in `src/main.c` using a paired multimeter
and ADC reading. With both values zero, output uses the nominal divider ratio.
Only agreement in voltage trends has been verified; calibration is pending.

## Source files

- `src/main.c`: ADC acquisition, conversion, and USB output.
- `prj.conf`: enabled Zephyr features.
- `boards/xiao_ble_nrf52840.overlay`: ADC pin, gain, and resolution.
- `CMakeLists.txt`: application build definition.

## Build and flash

From an nRF Connect SDK 3.3.4 terminal in this directory:

```sh
west build -b xiao_ble/nrf52840 -d build-adc --sysbuild .
```

Double-press RESET and copy
`build-adc/TEG_scheduler/zephyr/zephyr.uf2` to the XIAO bootloader drive.
Open its USB serial port at 115200 baud to start measurements.

# TEG scheduler

XIAO nRF52840 firmware using nRF Connect SDK 3.3.4 and Zephyr.
Current functionality: D0 ADC, ADS1115 VIN, and SHT40 temperature/humidity
measurements with USB CSV output and Nordic UART Service BLE notifications.
Energy-aware scheduling is not implemented yet.
The initial BLE interval is set aside as `INITIAL_TX_INTERVAL_MS = 300000`
(300 seconds). Phone tests use `BLE_TEST_INTERVAL_MS = 5000` (5 seconds).
The ADC test still waits 1 second between readings.

## Wiring and output

Power the XIAO over USB and share ground with the LTC3108 board.
Connect `VOUT -> 1 Mohm -> D0/A0 -> 330 kohm -> GND`.
Power ADS1115 VDD and SHT40 VIN from XIAO 3V3, with all grounds shared.
Connect both SDA pins to D4 and both SCL pins to D5.
Connect ADS1115 ADDR to GND (0x48), and A0 to LTC VIN/TEG positive.
SHT40 uses address 0x44. ADS1115 measures A0 against GND in single-shot
mode, with a 256 mV positive full scale. Keep VIN positive and below this
range for valid measurements. D0 remains labeled VOUT for calibration.
The firmware starts BLE and measurements at boot. It averages 20 D0 samples,
reads ADS1115 and SHT40, and waits 1 second before repeating. Opening the USB
serial connection prints the CSV header and enables continuous USB output.

```text
time_s,adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,temperature_C,humidity_pct,adc_status,ads_status,sht_status
```

Set `CAL_VOLTAGE` and `CAL_ADC` in `src/main.c` using a paired multimeter
and ADC reading. With both values zero, output uses the nominal divider ratio.
Only agreement in voltage trends has been verified; calibration is pending.
Status 0 means a successful read. Failed measurements are `nan` with a
negative status code; -19 means the device was not ready at startup,
and -34 means ADS1115 reached full scale. Check power, wiring and addresses,
then reset after fixing startup failures. Sensor readings have been observed
on USB; voltage accuracy still needs multimeter validation.

## Phone BLE test

Install Nordic nRF Connect for Mobile, enable Bluetooth, and scan in the app.
Connect to `TEG_XIAO`, open Nordic UART Service (UUID starts with `6e400001`),
and enable notifications on TX (UUID starts with `6e400003`). Choose UTF-8/text
display to read CSV rows in the same column order shown above. Notifications
start on the next measurement and repeat approximately every 5 seconds.
Small negotiated MTUs split a row across notifications; concatenate the text
through its newline. A larger MTU can display each row in one notification.
Disconnecting the phone restarts advertising so it can reconnect.

Keep XIAO and the sensor boards USB-powered for this integration test.
BLE does not require the USB serial monitor to be open. The firmware builds
successfully, and a phone has discovered `TEG_XIAO` after flashing.
Notification reception and disconnect/reconnect behavior remain to be verified.

## Source files

- `src/main.c`: ADC acquisition, conversion, and USB output.
- `src/ble_output.c` and `.h`: BLE advertising, connection handling, and NUS output.
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
Open its USB serial port at 115200 baud to view measurements.

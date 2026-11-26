# Archived firmware snapshots

These copies are not built (`CMakeLists.txt` only compiles `src/`). To go back to one,
copy its `src/`, `boards/`, `prj.conf` and `tools/capture_ble_csv.py` over the
project's files, then rebuild.

| Folder | What it is | Hardware it expects |
| --- | --- | --- |
| `261124_vout5V_original` | Firmware as of Nov 24, 2026 before the energy-loop work: Nov 22 VST calibration, VOUT one-point 5 V calibration, capacitor PID off (`PID_ENABLED 0`), 1 s baseline | LTC3108 VOUT = 5 V (VS1 and VS2 to VAUX); ADS1115 and SHT40 on XIAO 3V3. Wiring: `261119 TEG breadboard rebuild` |
| `261124_vout5V_mosfet_load` | Same calibration, plus the emulated event load: D2 pulses a MOSFET and 1 kΩ across VSTORE, PID on capacitor energy | Same as above, plus Section 5 of `261124 TEG sensor-only battery build` |

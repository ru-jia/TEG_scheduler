# TEG harvester-powered sensors build

Updates: The **SHT40 and ADS1115 now run from the LTC3108's VOUT set to 3.3 V**. The XIAO nRF52840 and BLE still run from the USB. The 0.1 F capacitor on VSTORE is the LTC3108's reserve: the TEG fills it, and it covers the sensors whenever they draw more than the TEG supplies. Every sensor reading is now paid for with harvested energy, and the energy PID sets how often to read so that reserve stays at its target.

This is the LTC3108's intended use. The datasheet's first figure is a wireless sensor powered from a Peltier cell, with sensors on VOUT and a 0.1 F capacitor on VSTORE. [LTC3108 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/3108fc.pdf)

**To go back to 5 V:** use [[261119 TEG breadboard rebuild]] and the firmware in `TEG_scheduler/archive/261124_vout5V_original`. That means VS2 back to VAUX (8h → 14h) and sensor power back to T+#42 / T+#48.

## 1. Connections

Same orientation and hole notation as [[261119 TEG breadboard rebuild]]: row **1 at left**, **63 at right**. From top to bottom: **T−, T+, j–f, center gap, e–a, B−, B+**. Both negative rails are common ground once bridged. Both positive rails carry XIAO 3V3 once bridged. **The sensors no longer use the 3V3 rails.** Rows 26 f–j now carry LTC3108 VOUT (3.3 V) and power the sensors. **Changed rows are in bold.**

FM means female at the component and male at the board. MM means male at both ends. FF means female at both ends. Resistors and the capacitor use their own leads. Every module stays off-board.

| Section                          | Component pin or starting point | Board hole or destination                      | Connection                                | Color               |
| -------------------------------- | ------------------------------- | ---------------------------------------------- | ----------------------------------------- | ------------------- |
| Rails                            | T−#32                           | B−#32                                          | MM bridge                                 | Black               |
| Rails                            | T+#36                           | B+#36                                          | MM bridge                                 | Red                 |
| TEG / LTC3108 · 5–14             | TEG + male lead                 | 5h                                             | Existing male lead                        | Existing lead color |
| TEG / LTC3108                    | TEG − male lead                 | LTC3108 GND                                    | FF                                        | Black               |
| TEG / LTC3108                    | LTC3108 VIN                     | 5j                                             | FM                                        | White               |
| TEG / LTC3108                    | LTC3108 GND, second connection  | T−#5                                           | FM                                        | Black               |
| TEG / LTC3108                    | LTC3108 VAUX                    | 8j                                             | FM                                        | Grey                |
| TEG / LTC3108                    | LTC3108 VS1                     | 11j                                            | FM                                        | Yellow              |
| TEG / LTC3108                    | LTC3108 VS2                     | 14j                                            | FM                                        | Beige               |
| TEG / LTC3108                    | VAUX at 8g                      | 11g (VS1)                                      | MM                                        | Yellow              |
| **TEG / LTC3108**                | **VS2 at 14h**                  | **T−#14**                                      | **MM (replaces the old 8h ↔ 14h jumper)** | **Black**           |
| Capacitor · 18–20                | Capacitor + at 20a              | 26d                                            | MM                                        | Purple              |
| Capacitor                        | Capacitor +                     | 20e                                            | Direct lead                               | —                   |
| Capacitor                        | Capacitor −                     | 18e                                            | Direct lead                               | —                   |
| Capacitor                        | Capacitor − at 18a              | B−#13                                          | MM                                        | Black               |
| VOUT divider · 26–34, f–j        | LTC3108 VOUT                    | 26j                                            | FM                                        | Orange              |
| VOUT divider                     | 1 MΩ resistor                   | 26f ↔ 30f                                      | Direct leads                              | —                   |
| VOUT divider                     | 330 kΩ resistor                 | 30g ↔ 34g                                      | Direct leads                              | —                   |
| VOUT divider                     | Low side at 34j                 | T−#27                                          | MM                                        | Black               |
| VST divider · 26–34, a–e         | LTC3108 VST                     | 26e                                            | FM                                        | Purple              |
| VST divider                      | 1 MΩ resistor                   | 26a ↔ 30a                                      | Direct leads                              | —                   |
| VST divider                      | 330 kΩ resistor                 | 30b ↔ 34b                                      | Direct leads                              | —                   |
| VST divider                      | Low side at 34a                 | B−#27                                          | MM                                        | Black               |
| XIAO · off-board near rail feeds | XIAO D0                         | 30j (VOUT divider midpoint)                    | FM                                        | Orange              |
| XIAO                             | XIAO D1                         | 30e (VST divider midpoint)                     | FM                                        | Purple              |
| XIAO                             | XIAO GND                        | B−#34                                          | FM                                        | Black               |
| XIAO                             | XIAO 3V3                        | B+#34                                          | FM                                        | Red                 |
| XIAO                             | XIAO 5V / VUSB pin              | Nothing                                        | Leave the pin bare                        | —                   |
| XIAO                             | XIAO USB-C                      | Laptop, only for flashing, USB CSV or charging | USB cable                                 | —                   |
| I²C sensors · 52–56              | XIAO SDA                        | 52j                                            | FM                                        | Blue                |
| I²C sensors                      | ADS1115 SDA                     | 52h                                            | FM                                        | Blue                |
| I²C sensors                      | SHT40 SDA                       | 52f                                            | FM                                        | Blue                |
| I²C sensors                      | XIAO SCL                        | 56j                                            | FM                                        | Green               |
| I²C sensors                      | ADS1115 SCL                     | 56h                                            | FM                                        | Green               |
| I²C sensors                      | SHT40 SCL                       | 56f                                            | FM                                        | Green               |
| **I²C sensors**                  | **ADS1115 VDD**                 | **26i (VOUT)**                                 | **FM, moved from T+#42**                  | **Orange**          |
| I²C sensors                      | ADS1115 GND                     | T−#42                                          | FM                                        | Black               |
| I²C sensors                      | ADS1115 ADDR                    | T−#45                                          | FM                                        | Black               |
| I²C sensors                      | ADS1115 A0                      | 5f (TEG+)                                      | FM                                        | White               |
| **I²C sensors**                  | **SHT40 VIN**                   | **26g (VOUT)**                                 | **FM, moved from T+#48**                  | **Orange**          |
| I²C sensors                      | SHT40 GND                       | T−#48                                          | FM                                        | Black               |

- **VOUT = 3.33 V.** The LTC3108 sets VOUT from VS1 and VS2: VS1 = VAUX with VS2 = GND gives 3.3 V (3.234–3.36 V). The old build tied both to VAUX for 5 V. 3.3 V is within both sensors' supply ranges and matches the XIAO's I²C logic level.
- **Sensor board idle current decides whether this works.** Each breakout's own parts draw from VOUT all the time, not just the sensor chip: a power LED, and on many boards a 3.3 V regulator. A lit LED alone is often 0.1–1 mA, which is 0.3–3 mW and far more than the 50–120 µW harvest. If a board's LED cannot be disabled, measure it first (step 2 of Section 2) before moving anything:
  - **Board idle ≤ about 5 µA:** fine on VOUT.
  - **Board idle well above the harvest:** that board cannot run from VOUT. Leave it on the 3V3 rail (battery), and put only the low-draw board on VOUT.
- **I²C pull-ups now sit on VOUT.** The breakout boards' pull-ups move with their power pin, and the XIAO only ever pulls SDA/SCL low. So when VOUT is down, the bus is unpowered rather than back-feeding the sensors from the battery. Step 6 below checks this.
- **Constant loads on the harvest:** each divider draws about V²/1.33 MΩ, roughly 8 µW on VOUT at 3.3 V and 9–20 µW on VSTORE. These were already in the Nov 22 charge-curve slope, so the curve reflects them.

## 3. Multimeter probe points

Set the meter to DC voltage. Leave the **black clip** at **B−#50** using **black MM**. Move the **red clip** through the points below.

| Check                    | Breadboard or rail probe hole | measured    | Probe jumper color | Expected                                                | build |
| ------------------------ | ----------------------------- | ----------- | ------------------ | ------------------------------------------------------- | ----- |
| XIAO 3V3                 | B+#50, then T+#50             | 3.30V       | Red                | About 3.3 V. Not the battery voltage                    | kept  |
| VIN / TEG+               | 5g                            | Around 44mV | White              | Raw TEG voltage in mV                                   | kept  |
| VAUX                     | 8f                            | 3.35V       | Grey               | Reference for VS1                                       |       |
| VS1                      | 11f                           | 3.35V       | Yellow             | Same voltage as VAUX                                    |       |
| **VS2**                  | **14f**                       | 0           | **Beige**          | **0 V (tied to ground)**                                |       |
| **VOUT / sensor supply** | **26h**                       | 3.33V       | **Orange**         | **About 3.3 V (3.234–3.36 V) once regulated**           | kept  |
| **VOUT at D0**           | **30h**                       | 0.8V        | **Orange**         | **About 0.248 × VOUT, about 0.82 V**                    |       |
| VST / capacitor positive | 26c                           | charging    | Purple             | Capacitor voltage                                       | kept  |
| VST at D1                | 30c                           |             | Purple             | About 0.248 × VST                                       |       |
| SDA idle                 | 52g                           | 3.27V       | Blue               | About 3.3 V with VOUT up; about 0 V with VOUT unplugged |       |
| SCL idle                 | 56g                           | 3.27V       | Green              | About 3.3 V with VOUT up; about 0 V with VOUT unplugged |       |

## 4. Energy PID runs

**When the controller has something to do.** One sensing event (one SHT40 plus one ADS1115 reading) is estimated at about 20 µJ; measure it in step 1. At the fastest rate (0.5 s), the sensors spend roughly 40 µW, plus about 10–20 µW of fixed loads. The controller can hold the target only when harvest falls between the fixed loads and that maximum:

- **Harvest above the maximum spend** (likely at ΔT ≈ 40 °C, 70–120 µW): the capacitor stays full and the rate sits at the fastest limit. That is the correct "energy is plentiful" behavior, but the controller is not exercised. Lower ΔT to about 10–20 °C, or raise `SHT_READS_PER_EVENT` so each event does more sensing and costs more.
- **Harvest below the fixed loads:** no rate can hold the target. The rate sits at 30 s and the capacitor slowly drains. That is a real limit worth logging.

**1. Measure the event energy (two baselines).** Charge VSTORE to about 4–4.5 V, below the 5.2 V ceiling so the LTC3108 is not clamping it, and keep the thermal condition steady. Build with `PID_ENABLED 0` and `BASELINE_INTERVAL_MS 500`, log 20 minutes, then repeat with `BASELINE_INTERVAL_MS 5000`. Fit the slope of `cap_energy_J` against `time_s` for each run:

  E_event = (P_5000 − P_500) / (2.0 Hz − 0.2 Hz)

Then set the gains as the comments in `main.c` explain: Kp = 1 / (E_event × 100 s) and Ki = E_event × Kp² / 4.

**2. Energy-neutral run (net power = 0).** Charge VSTORE to about 4–4.5 V, below the 5.2 V ceiling. At the ceiling the LTC3108 stops charging and the extra harvest cannot be seen. Build with `PID_ENABLED 1` and `ENERGY_TARGET_FRACTION 1.0`, the default. Record with `tools/capture_ble_csv.py run.csv --full`, or USB `capture_csv.py`. The first reading fixes the target at the starting energy, and the target repeats on every row. Holding that level means the sensors spend what the TEG harvests: `cap_net_power_uW` should hover around 0, and `event_interval_ms` settles at the energy-neutral interval for that ΔT. A simulation at 30 µW harvest settled within about 5 minutes and held within about ±2 mJ at about 1 s.

Why hold the level instead of controlling dE/dt directly: a controller that only looks at dE/dt never corrects small errors that add up, so the capacitor would slowly drift until it was full or empty. Holding the level is net-power control with that drift corrected. Change one gain per run and compare `cap_energy_filt_J` against `cap_target_J`, `cap_net_power_uW`, `event_interval_ms` and the `pid_p_Hz` / `pid_i_Hz` terms.

**3. Disturbance test.** Once it is holding, change ΔT (warm or cool the TEG). Net power swings away from 0, then the interval moves to the new energy-neutral value and net power returns to 0. This is the result that shows the controller doing its job. The interval against ΔT is also a direct measurement of how often harvested energy can pay for sensing.

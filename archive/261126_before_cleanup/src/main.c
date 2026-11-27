/* SPDX-License-Identifier: Apache-2.0 */

/*
 * TEG energy scheduler for the XIAO nRF52840: harvester-powered sensors.
 *
 * THE HARDWARE IDEA
 *   The LTC3108 output VOUT is set to 3.3 V and powers the SHT40 and the ADS1115.
 *   The 0.1 F capacitor on VSTORE is the LTC3108's reserve: the TEG fills it, and
 *   whenever the sensors draw more than the TEG supplies, the capacitor makes up the
 *   difference. So every sensor reading is paid for by harvested energy, and the
 *   capacitor's energy tells us how much reserve is left.
 *   The nRF52840 and BLE still run from the LiPo (or USB), because they need far
 *   more power than the TEG produces.
 *
 * WHAT THIS PROGRAM DOES, ONE CYCLE AT A TIME
 *   1. Read the capacitor voltage (VSTORE, pin D1) and turn it into energy in joules.
 *   2. Ask the controller how long to wait before the next cycle.
 *   3. Read VOUT (pin D0). If VOUT is high enough, the sensors are powered, so read
 *      the SHT40 (temperature, humidity) and the ADS1115 (TEG voltage VIN).
 *      If VOUT is too low, skip them this cycle and try again next cycle.
 *   4. Send one CSV row over BLE (and print it on USB if a terminal is open).
 *   5. Sleep until the next cycle is due.
 *
 * THE CONTROL IDEA
 *   The capacitor energy changes as   dE/dt = P_harvest - P_sensor_idle - E_event * f
 *   where f is the event rate (sensor readings per second, f = 1 / interval) and
 *   E_event is the energy of one round of sensor readings.
 *   The goal is net power zero: f chosen so  E_event * f = P_harvest - P_sensor_idle.
 *   The controller holds the capacitor at its starting energy, which keeps dE/dt at 0.
 *   - More energy stored than the target -> raise f  (measure more often).
 *   - Less energy stored than the target -> lower f  (measure less often).
 *   The PID calculates f; the CSV shows the interval, 1000 / f milliseconds.
 *   We control the rate rather than the interval because energy used per second is
 *   proportional to the rate, so the same gains behave the same at 1 s or at 20 s.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ble_output.h"

/* ------------------------------------------------------------------------------------
 * Measurement settings
 * ------------------------------------------------------------------------------------ */

/* Both voltages go through a divider before the ADC pin:
 *   source -- 1 Mohm -- ADC pin -- 330 kohm -- GND
 * D0/A0 measures VOUT; D1/A1 measures VSTORE (the capacitor). */
static const double R_TOP_OHM = 1000000.0;
static const double R_BOTTOM_OHM = 330000.0;

/* Nov 22, 2026 VSTORE calibration: capacitor volts = averaged D1 count * gain + offset.
 * Checked against the multimeter from 0.608 to 5.180 V. Unchanged by the 3.3 V VOUT
 * setting, because the VSTORE divider and wiring are the same. */
static const double VST_CAL_GAIN_V_PER_COUNT = 0.003490639465;
static const double VST_CAL_OFFSET_V = 0.047304725;

/* VOUT one-point calibration, made at the 5 V setting (IMG_5676). It scales counts
 * to volts, so it is still roughly right at 3.3 V. Check it against the multimeter
 * at 3.3 V and update these two numbers if needed. Set both to zero to use the
 * plain divider ratio instead. */
static const double VOUT_CAL_VOLTAGE = 5.09;
static const double VOUT_CAL_ADC = 1411.3;

/* Each D0/D1 reading throws away one conversion, then averages 20, 5 ms apart. */
#define SAMPLES 20
#define SAMPLE_SPACING_MS 5

/* Sensors are read only when VOUT is at least this high. Below it the LTC3108 is
 * not holding 3.3 V and the sensors may be unpowered or browning out. */
#define VOUT_MIN_FOR_SENSORS_V 3.0

/* I2C addresses (ADS1115 ADDR pin to GND; SHT40 fixed). */
#define ADS1115_ADDR 0x48
#define SHT40_ADDR 0x44

/* Number of SHT40 readings averaged per event. More readings give smoother data and
 * cost more harvested energy per event; it is a real accuracy-versus-energy knob. */
#define SHT_READS_PER_EVENT 1

/* ------------------------------------------------------------------------------------
 * Experiment settings: these are the numbers you change between runs
 * ------------------------------------------------------------------------------------ */

/* 1 = the PID picks the interval.  0 = baseline: fixed BASELINE_INTERVAL_MS.
 * Two baseline runs at different intervals (for example 500 ms and 5000 ms) give
 * the energy of one event; see README "Measure the event energy". */
#define PID_ENABLED 1
#define BASELINE_INTERVAL_MS 1000

/* Capacitor and target: ENERGY NEUTRALITY (net power = 0).
 * The goal is that the sensors spend exactly what the TEG harvests, so the net power
 * into the capacitor, dE/dt, is zero. We get that by holding the capacitor at the
 * energy it had when the run started:
 *     target = ENERGY_TARGET_FRACTION * (capacitor energy at the first reading)
 * with the fraction at 1.0. Why hold the level instead of controlling dE/dt directly?
 *   - If the level stays constant, dE/dt is zero on average, which is what we want.
 *   - A controller that only looks at dE/dt never notices small errors adding up,
 *     so the capacitor would slowly drift until it is full or empty. Holding the
 *     level corrects that drift. In PID terms: P and I on energy here are the same
 *     as I and "double I" on net power.
 * Start each run with VSTORE well below the 5.2 V ceiling (about 4-4.5 V). At the
 * ceiling the LTC3108 stops charging, so extra harvest is thrown away and the
 * capacitor can no longer show it. Set the fraction below 1.0 only for a step test. */
#define CAPACITANCE_F 0.1
#define ENERGY_TARGET_FRACTION 1.0

/* The raw capacitor energy wobbles by about 1 mJ from ADC noise, which is bigger
 * than one event (tens of uJ). The controller therefore works on a smoothed energy
 * (a running average with this time constant). The raw value is still logged. */
#define ENERGY_FILTER_S 10.0

/* PID gains. The controller output is an event RATE in Hz (events per second).
 *   error = smoothed energy - target            (joules; positive = surplus)
 *   rate  = RATE_BIAS_HZ + Kp*error + Ki*(sum of error*dt) + Kd*(d energy / dt)
 * Units: Kp in Hz per J, Ki in Hz per (J*s), Kd in Hz per W.
 *
 * How to choose them once you know E_event (energy of one event, in J):
 *   - With P only, the error shrinks with time constant  tau = 1 / (E_event * Kp).
 *     Pick tau around 60-120 s:  Kp = 1 / (E_event * tau).
 *   - For a response that settles without overshoot:  Ki = E_event * Kp^2 / 4.
 * The defaults assume E_event = 20 uJ (an estimate until you measure it):
 *   Kp = 500 -> tau = 100 s;  Ki = 20e-6 * 500^2 / 4 = 1.25 -> use 1.0.
 * Keep Kd = 0 at first: the energy slope is noisy. */
#define PID_KP 500.0
#define PID_KI 1.0
#define PID_KD 0.0

/* The rate the controller uses when the error is zero and the integral is empty.
 * 0.5 Hz = one event every 2 s. */
#define RATE_BIAS_HZ 0.5

/* Fastest and slowest allowed cycle. One cycle takes about 0.25 s of measuring,
 * so 0.5 s is close to the fastest this firmware can run. */
#define EVENT_INTERVAL_MIN_S 0.5
#define EVENT_INTERVAL_MAX_S 30.0

/* Smoothing time constant for the logged energy slope (dE/dt). */
#define PID_SLOPE_FILTER_S 30.0

/* ------------------------------------------------------------------------------------
 * CSV layout
 * ------------------------------------------------------------------------------------ */

static const char measurement_columns[] =
	"time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
	"vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
	"temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status";
static const char controller_columns[] =
	"cap_energy_J,cap_energy_filt_J,cap_target_J,cap_error_J,cap_net_power_uW,"
	"event_interval_ms,ble_sent,battery_start_mWh,pid_p_Hz,pid_i_Hz,pid_d_Hz";

/* ------------------------------------------------------------------------------------
 * Hardware handles. Zephyr builds these from boards/xiao_ble_nrf52840.overlay.
 * ------------------------------------------------------------------------------------ */

static const struct adc_dt_spec vst_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vst);
static const struct adc_dt_spec vout_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vout);
/* The I2C bus on XIAO D4 (SDA) and D5 (SCL). The sensors are talked to directly on
 * this bus instead of through Zephyr's sensor drivers: those drivers check the
 * sensor once at boot and give up for good if it is unpowered then. Talking to it
 * directly means an unpowered sensor simply fails this cycle and works again
 * once VOUT comes back. */
static const struct device *const i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* ------------------------------------------------------------------------------------
 * Optional run note: starting battery energy, typed over USB as "battery_mwh 1850".
 * It is only written into the CSV for your records; the controller does not use it.
 * ------------------------------------------------------------------------------------ */

struct battery_run_input {
	double start_mwh;
	char line[48];
	size_t length;
	bool discard;
};

static void read_battery_run_input(const struct device *console, struct battery_run_input *input)
{
	static const char prefix[] = "battery_mwh ";
	unsigned char byte;

	/* Only look at characters that have already arrived, so typing never pauses
	 * the measurement loop. Characters are collected until Enter is pressed. */
	for (size_t i = 0; i < sizeof(input->line) && uart_poll_in(console, &byte) == 0; i++) {
		if (byte != '\r' && byte != '\n') {
			if (!input->discard && byte != '\0' && input->length < sizeof(input->line) - 1) {
				input->line[input->length++] = byte;
			} else {
				input->discard = true; /* Line too long: ignore it. */
			}
			continue;
		}
		if (input->length > 0 || input->discard) {
			input->line[input->length] = '\0';
			double energy_mwh = NAN;
			if (!input->discard && strncmp(input->line, prefix, sizeof(prefix) - 1) == 0) {
				const char *value = input->line + sizeof(prefix) - 1;
				char *end;
				double parsed = strtod(value, &end);

				if (end != value && *end == '\0') {
					energy_mwh = parsed;
				}
			}
			if (isfinite(energy_mwh) && energy_mwh >= 0.0) {
				input->start_mwh = energy_mwh;
				printf("# battery_start_mWh=%.9g\n", input->start_mwh);
			} else {
				printf("# Use battery_mwh followed by a nonnegative energy in mWh.\n");
			}
		}
		input->length = 0;
		input->discard = false;
	}
}

/* ------------------------------------------------------------------------------------
 * The controller
 * ------------------------------------------------------------------------------------ */

/* Everything the controller has to remember from one cycle to the next. */
struct energy_pid {
	double target_j;          /* Fixed after the first valid reading. NAN until then. */
	double energy_filt_j;     /* Smoothed capacitor energy; the controller uses this. */
	double previous_filt_j;   /* Last cycle's smoothed energy, to compute the slope. */
	int64_t previous_ms;      /* Last cycle's time stamp. */
	bool has_previous;        /* False on the first cycle and after a failed reading. */
	double net_power_w;       /* Smoothed dE/dt: harvest minus everything the store feeds. */
	double integral_hz;       /* The I term, already multiplied by Ki (in Hz). */
	/* The three terms from the latest update, kept only so they can be logged. */
	double p_hz;
	double i_hz;
	double d_hz;
};

/* Run one controller step and return the wait until the next cycle, in milliseconds. */
static int update_event_interval(struct energy_pid *pid, double energy_j, int64_t now_ms)
{
	const double rate_min_hz = 1.0 / EVENT_INTERVAL_MAX_S;
	const double rate_max_hz = 1.0 / EVENT_INTERVAL_MIN_S;

	/* A failed capacitor reading: we do not know the energy, so be cautious and wait
	 * the longest interval. Keep the integral and the smoothed energy, but do not
	 * compute a slope across the gap. */
	if (!isfinite(energy_j)) {
		pid->has_previous = false;
		pid->net_power_w = NAN;
		pid->p_hz = NAN;
		pid->d_hz = NAN;
		return PID_ENABLED ? (int)(EVENT_INTERVAL_MAX_S * 1000.0) : BASELINE_INTERVAL_MS;
	}

	/* The first valid reading fixes the target for the whole run. */
	if (!isfinite(pid->target_j)) {
		pid->target_j = ENERGY_TARGET_FRACTION * energy_j;
		pid->energy_filt_j = energy_j;
		printf("# Target fixed at %.6f J (%.0f%% of first reading %.6f J).\n",
		       pid->target_j, ENERGY_TARGET_FRACTION * 100.0, energy_j);
	}

	double dt_s = (now_ms - pid->previous_ms) / 1000.0;
	double integral_step_hz = 0.0;

	if (pid->has_previous && dt_s > 0.0) {
		/* Running average of the energy (a first-order low-pass filter). alpha is
		 * how much the newest reading counts; a longer filter time means less. */
		double alpha_e = dt_s / (ENERGY_FILTER_S + dt_s);

		pid->energy_filt_j += alpha_e * (energy_j - pid->energy_filt_j);

		/* Slope of the smoothed energy since the last cycle, in watts (J/s),
		 * smoothed once more for logging and for the D term. */
		double slope_w = (pid->energy_filt_j - pid->previous_filt_j) / dt_s;
		double alpha_s = dt_s / (PID_SLOPE_FILTER_S + dt_s);

		pid->net_power_w = isfinite(pid->net_power_w) ?
			pid->net_power_w + alpha_s * (slope_w - pid->net_power_w) :
			slope_w;
	}

	/* Positive error = more energy than the target = we can afford to measure faster. */
	double error_j = pid->energy_filt_j - pid->target_j;

	if (pid->has_previous && dt_s > 0.0) {
		/* The I term adds up error x time. */
		integral_step_hz = PID_KI * error_j * dt_s;
	}
	pid->previous_filt_j = pid->energy_filt_j;
	pid->previous_ms = now_ms;
	pid->has_previous = true;

	/* P term: react to how far we are from the target right now. */
	pid->p_hz = PID_KP * error_j;
	/* D term: react to how fast the energy is moving. The target is constant,
	 * so d(error)/dt is the same as dE/dt. Energy rising -> measure faster. */
	pid->d_hz = isfinite(pid->net_power_w) ? PID_KD * pid->net_power_w : 0.0;

	double rate_hz = RATE_BIAS_HZ + pid->p_hz + pid->integral_hz + pid->d_hz;
	double candidate_hz = rate_hz + integral_step_hz;

	/* Anti-windup. When the rate is already pinned at the fastest or slowest limit,
	 * do not let the integral keep growing in that same direction; otherwise it
	 * builds up a large "debt" and the controller overshoots badly once the error
	 * changes sign. The integral is always allowed to move back toward the range. */
	if ((candidate_hz <= rate_max_hz || integral_step_hz < 0.0) &&
	    (candidate_hz >= rate_min_hz || integral_step_hz > 0.0)) {
		pid->integral_hz += integral_step_hz;
		rate_hz = candidate_hz;
	}
	pid->i_hz = pid->integral_hz;

	if (!PID_ENABLED) {
		return BASELINE_INTERVAL_MS;
	}
	rate_hz = fmin(fmax(rate_hz, rate_min_hz), rate_max_hz);
	return (int)(1000.0 / rate_hz + 0.5);
}

/* ------------------------------------------------------------------------------------
 * Sensors on the I2C bus, powered from LTC3108 VOUT
 * ------------------------------------------------------------------------------------ */

/* One ADS1115 reading of the TEG voltage on input A0.
 * The ADS1115 is set by writing a 16-bit "config" word to register 0x01:
 *   0xCB83 = start one conversion, A0 against GND, +/-0.256 V range,
 *            single-shot (sleep afterwards), 128 samples/s, comparator off.
 * One conversion takes about 7.8 ms; then the result is read from register 0x00.
 * One count is 0.256 V / 32768 = 7.8125 uV, the same scale as before.
 * Returns 0 on success or a negative error code (I2C failure, or -ERANGE). */
static int read_vin(double *raw, double *millivolts)
{
	const uint8_t start[3] = {0x01, 0xCB, 0x83};
	const uint8_t result_register = 0x00;
	uint8_t data[2];
	int ret = i2c_write(i2c_bus, start, sizeof(start), ADS1115_ADDR);

	if (ret < 0) {
		return ret;
	}
	k_msleep(10);
	ret = i2c_write_read(i2c_bus, ADS1115_ADDR, &result_register, 1, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}
	int16_t code = (int16_t)((data[0] << 8) | data[1]);

	/* The largest possible code means "256 mV or more", so report it as out of range. */
	if (code == INT16_MAX) {
		return -ERANGE;
	}
	*raw = code;
	*millivolts = code * (256.0 / 32768.0);
	return 0;
}

/* The SHT40 adds a small checksum (CRC-8) after every two data bytes, so we can tell
 * if a reading was corrupted, for example while VOUT was sagging. */
static uint8_t sht40_crc(const uint8_t *bytes)
{
	uint8_t crc = 0xFF;

	for (int i = 0; i < 2; i++) {
		crc ^= bytes[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/* One SHT40 measurement: send command 0xFD (high repeatability), wait 10 ms, read
 * 6 bytes = temperature (2) + checksum (1) + humidity (2) + checksum (1).
 * The formulas are from the SHT40 datasheet. */
static int read_sht40_once(double *temperature, double *humidity)
{
	const uint8_t command = 0xFD;
	uint8_t data[6];
	int ret = i2c_write(i2c_bus, &command, 1, SHT40_ADDR);

	if (ret < 0) {
		return ret;
	}
	k_msleep(10);
	ret = i2c_read(i2c_bus, data, sizeof(data), SHT40_ADDR);
	if (ret < 0) {
		return ret;
	}
	if (sht40_crc(&data[0]) != data[2] || sht40_crc(&data[3]) != data[5]) {
		return -EBADMSG;
	}
	uint16_t t_ticks = (uint16_t)((data[0] << 8) | data[1]);
	uint16_t rh_ticks = (uint16_t)((data[3] << 8) | data[4]);

	*temperature = -45.0 + 175.0 * t_ticks / 65535.0;
	*humidity = fmin(fmax(-6.0 + 125.0 * rh_ticks / 65535.0, 0.0), 100.0);
	return 0;
}

/* Average SHT_READS_PER_EVENT measurements. Any failed read fails the event. */
static int read_environment(double *temperature, double *humidity)
{
	double t_sum = 0.0;
	double rh_sum = 0.0;

	for (int i = 0; i < SHT_READS_PER_EVENT; i++) {
		double t;
		double rh;
		int ret = read_sht40_once(&t, &rh);

		if (ret < 0) {
			return ret;
		}
		t_sum += t;
		rh_sum += rh;
	}
	*temperature = t_sum / SHT_READS_PER_EVENT;
	*humidity = rh_sum / SHT_READS_PER_EVENT;
	return 0;
}

/* ------------------------------------------------------------------------------------
 * Onboard ADC (powered by the LiPo/USB side, unchanged)
 * ------------------------------------------------------------------------------------ */

/* Average SAMPLES conversions on one onboard ADC pin. Gives back the average raw
 * count (used by the calibration) and the average pin voltage (for logging). */
static int read_adc(const struct adc_dt_spec *input,
		    double *raw_average, double *pin_voltage)
{
	int16_t sample;
	int32_t raw_total = 0;
	int64_t microvolt_total = 0;
	struct adc_sequence sequence = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};
	int ret = adc_sequence_init_dt(input, &sequence);

	if (ret < 0) {
		return ret;
	}

	/* The first conversion after switching pins can be off, so it is thrown away. */
	ret = adc_read_dt(input, &sequence);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < SAMPLES; i++) {
		ret = adc_read_dt(input, &sequence);
		if (ret < 0) {
			return ret;
		}
		raw_total += sample;
		int32_t microvolts = sample;

		ret = adc_raw_to_microvolts_dt(input, &microvolts);
		if (ret < 0) {
			return ret;
		}
		microvolt_total += microvolts;
		k_msleep(SAMPLE_SPACING_MS);
	}

	*raw_average = (double)raw_total / SAMPLES;
	*pin_voltage = (double)microvolt_total / SAMPLES / 1000000.0;
	return 0;
}

/* VOUT: use the one-point calibration if it is set, otherwise the divider ratio. */
static double source_voltage(double raw, double pin, double cal_v, double cal_adc)
{
	if (cal_v > 0.0 && cal_adc > 0.0) {
		return raw * cal_v / cal_adc;
	}
	return pin * (R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;
}

/* ------------------------------------------------------------------------------------
 * Main loop
 * ------------------------------------------------------------------------------------ */

int main(void)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int ret;

	/* The console and both onboard voltage pins must work, or nothing useful runs. */
	if (!device_is_ready(console) || !adc_is_ready_dt(&vst_input) ||
	    !adc_is_ready_dt(&vout_input)) {
		return 0;
	}

	ret = adc_channel_setup_dt(&vst_input);
	if (ret == 0) {
		ret = adc_channel_setup_dt(&vout_input);
	}
	if (ret < 0) {
		printf("# ADC setup failed: %d\n", ret);
		return 0;
	}

	/* The I2C controller is inside the nRF, so it is ready even while the sensors
	 * themselves are unpowered. */
	bool i2c_ready = device_is_ready(i2c_bus);
	int ble_status = ble_output_init();
	struct energy_pid pid = {
		.target_j = NAN,
		.energy_filt_j = NAN,
		.net_power_w = NAN,
		.p_hz = NAN,
		.i_hz = NAN,
		.d_hz = NAN,
	};
	struct battery_run_input battery = {.start_mwh = NAN};
	bool usb_was_open = false;

	printf("# D0=VOUT (3.3 V setting), D1=VSTORE; each uses 1M/330k.\n");
	printf("# SHT40 0x44 and ADS1115 0x48 powered from VOUT; I2C ready=%d.\n", i2c_ready);

	while (1) {
		/* Board uptime in ms: the row's time stamp and the start of this cycle. */
		int64_t cycle_start = k_uptime_get();
		uint32_t dtr = 0;

		/* "DTR" is set while a program on the laptop has the USB serial port open
		 * (for example capture_csv.py). When one opens, print the settings and the
		 * column header. On battery with no USB, this is simply skipped. */
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		if (dtr && !usb_was_open) {
			printf("# BLE status=%d; PID=%d; Kp=%.4g Hz/J, Ki=%.4g Hz/(J*s), "
			       "Kd=%.4g Hz/W, bias=%.3g Hz\n",
			       ble_status, PID_ENABLED, PID_KP, PID_KI, PID_KD, RATE_BIAS_HZ);
			printf("# Interval %.1f-%.1f s; target fraction %.2f; energy filter %.0f s; "
			       "sensors need VOUT >= %.2f V; SHT reads/event %d\n",
			       EVENT_INTERVAL_MIN_S, EVENT_INTERVAL_MAX_S, ENERGY_TARGET_FRACTION,
			       ENERGY_FILTER_S, VOUT_MIN_FOR_SENSORS_V, SHT_READS_PER_EVENT);
			printf("%s,%s\n", measurement_columns, controller_columns);
		}
		if (dtr) {
			read_battery_run_input(console, &battery);
		} else {
			battery.length = 0;
			battery.discard = false;
		}
		usb_was_open = dtr != 0;

		/* Every value starts as "not a number" (nan) each cycle, so a failed read
		 * shows up as nan in the CSV instead of silently repeating an old value. */
		double vst_raw = NAN, vst_pin_voltage = NAN, vst_voltage = NAN;
		double vout_raw = NAN, vout_pin_voltage = NAN, vout_voltage = NAN;
		double vin_raw = NAN, vin_mv = NAN;
		double temperature = NAN, humidity = NAN;

		/* Step 1: capacitor energy. */
		int vst_status = read_adc(&vst_input, &vst_raw, &vst_pin_voltage);
		if (vst_status == 0) {
			vst_voltage = vst_raw * VST_CAL_GAIN_V_PER_COUNT + VST_CAL_OFFSET_V;
		}
		double cap_energy_j = NAN;
		if (vst_status == 0 && isfinite(vst_voltage) && vst_voltage > 0.0) {
			cap_energy_j = 0.5 * CAPACITANCE_F * vst_voltage * vst_voltage;
		}

		/* Step 2: controller picks the time until the next cycle. */
		int event_interval_ms = update_event_interval(&pid, cap_energy_j, cycle_start);

		/* Step 3: VOUT, then the sensors if VOUT can power them.
		 * Status codes: 0 = ok, -11 (EAGAIN) = skipped because VOUT was too low,
		 * -19 (ENODEV) = I2C bus not available, other negatives = I2C/data errors. */
		int vout_status = read_adc(&vout_input, &vout_raw, &vout_pin_voltage);
		if (vout_status == 0) {
			vout_voltage = source_voltage(vout_raw, vout_pin_voltage,
				VOUT_CAL_VOLTAGE, VOUT_CAL_ADC);
		}
		int ads_status = -ENODEV;
		int sht_status = -ENODEV;

		if (i2c_ready) {
			if (vout_status == 0 && vout_voltage >= VOUT_MIN_FOR_SENSORS_V) {
				sht_status = read_environment(&temperature, &humidity);
				ads_status = read_vin(&vin_raw, &vin_mv);
			} else {
				sht_status = -EAGAIN;
				ads_status = -EAGAIN;
			}
		}

		/* Step 4: build the row and send it. */
		char line[256];
		int length = snprintf(line, sizeof(line),
			"%.3f,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.0f,%.3f,%.2f,%.2f,%d,%d,%d,%d\n",
			cycle_start / 1000.0, vst_raw, vst_pin_voltage, vst_voltage,
			vout_raw, vout_pin_voltage, vout_voltage, vin_raw, vin_mv,
			temperature, humidity, vst_status, vout_status, ads_status, sht_status);

		bool ble_sent = false;
		if (length < 0 || (size_t)length >= sizeof(line)) {
			printf("# CSV formatting failed.\n");
		} else {
			const struct ble_csv_controller controller = {
				.energy_j = cap_energy_j,
				.energy_filt_j = pid.energy_filt_j,
				.target_j = pid.target_j,
				.net_power_uw = pid.net_power_w * 1000000.0,
				.interval_ms = event_interval_ms,
				.battery_start_mwh = battery.start_mwh,
				.p_hz = pid.p_hz,
				.i_hz = pid.i_hz,
				.d_hz = pid.d_hz,
			};

			if (ble_status == 0) {
				ret = ble_output_send(line, length, &controller);
				if (ret == 0) {
					ble_sent = true;
				} else if (ret != -ENOTCONN) {
					printf("# BLE send failed: %d\n", ret);
				}
			}
			if (dtr) {
				printf("%.*s,%.6f,%.6f,%.6f,%.6f,%.2f,%d,%d,%.9g,%.5f,%.5f,%.5f\n",
				       length - 1, line, cap_energy_j, pid.energy_filt_j, pid.target_j,
				       pid.energy_filt_j - pid.target_j, pid.net_power_w * 1000000.0,
				       event_interval_ms, ble_sent, battery.start_mwh,
				       pid.p_hz, pid.i_hz, pid.d_hz);
			}
		}

		/* Step 5: sleep for whatever is left of the interval. The interval runs from
		 * the start of one cycle to the start of the next, so the time spent
		 * measuring and sending is already included. */
		int64_t remaining = event_interval_ms - (k_uptime_get() - cycle_start);
		if (remaining > 0) {
			k_msleep(remaining);
		}
	}
	return 0;
}

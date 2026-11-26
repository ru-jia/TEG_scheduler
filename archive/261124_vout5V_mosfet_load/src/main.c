/* SPDX-License-Identifier: Apache-2.0 */

/*
 * TEG energy scheduler for the XIAO nRF52840.
 *
 * WHAT THIS PROGRAM DOES, ONE CYCLE AT A TIME
 *   1. Read the capacitor voltage (VSTORE, pin D1) and turn it into energy in joules.
 *   2. Ask the controller how long to wait before the next cycle.
 *   3. Read the other sensors: VOUT (D0), TEG voltage VIN (ADS1115), temperature and
 *      humidity (SHT40).
 *   4. "Spend" one event's worth of energy from the capacitor: switch the MOSFET on for
 *      a short pulse so current flows from the capacitor through the load resistor.
 *   5. Send one CSV row over BLE (and print it on USB if a terminal is open).
 *   6. Sleep until the next cycle is due.
 *
 * WHY THE MOSFET PULSE EXISTS
 *   The nRF itself runs from the LiPo, because the capacitor cannot start it. Without the
 *   pulse, the TEG would only ever fill the capacitor and nothing would empty it, so the
 *   wait interval would have no effect on the energy we measure. The pulse makes each
 *   sensing event take a fixed amount of energy out of the SAME capacitor the TEG charges.
 *   Then:  energy in = TEG harvest,  energy out = pulses,  control input = how often
 *   we pulse. That is a real feedback loop.
 *
 * THE CONTROL IDEA
 *   The capacitor energy changes as   dE/dt = P_harvest - E_pulse * f
 *   where f is the event rate (events per second, f = 1 / interval).
 *   - More energy stored than the target -> raise f  (send more often).
 *   - Less energy stored than the target -> lower f  (send less often).
 *   The PID calculates f; the interval printed in the CSV is just 1000 / f milliseconds.
 *   We control the rate rather than the interval because the energy used per second is
 *   proportional to the rate, so the same gains behave the same at 2 s or at 20 s.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ble_output.h"

/* ------------------------------------------------------------------------------------
 * Measurement settings (unchanged from the calibrated version)
 * ------------------------------------------------------------------------------------ */

/* Both voltages go through a divider before the ADC pin:
 *   source -- 1 Mohm -- ADC pin -- 330 kohm -- GND
 * D0/A0 measures VOUT; D1/A1 measures VSTORE (the capacitor). */
static const double R_TOP_OHM = 1000000.0;
static const double R_BOTTOM_OHM = 330000.0;

/* Nov 22, 2026 VSTORE calibration: capacitor volts = averaged D1 count * gain + offset.
 * Checked against the multimeter from 0.608 to 5.180 V. */
static const double VST_CAL_GAIN_V_PER_COUNT = 0.003490639465;
static const double VST_CAL_OFFSET_V = 0.047304725;

/* VOUT one-point calibration from the 5 V recording (IMG_5676).
 * Set both to zero to use the plain divider ratio instead. */
static const double VOUT_CAL_VOLTAGE = 5.09;
static const double VOUT_CAL_ADC = 1411.3;

/* Each D0/D1 reading throws away one conversion, then averages 20, 5 ms apart. */
#define SAMPLES 20
#define SAMPLE_SPACING_MS 5

/* ------------------------------------------------------------------------------------
 * Experiment settings: these are the numbers you change between runs
 * ------------------------------------------------------------------------------------ */

/* 1 = the PID picks the interval.  0 = baseline: fixed BASELINE_INTERVAL_MS.
 * In both modes the MOSFET pulse still happens every cycle if LOAD_ENABLED is 1,
 * so a 1 s baseline shows how fast the capacitor drains at the fastest rate. */
#define PID_ENABLED 1
#define BASELINE_INTERVAL_MS 1000

/* 1 = pulse the load every cycle.  0 = never pulse (the capacitor only charges). */
#define LOAD_ENABLED 1

/* Load hardware: XIAO D2 drives the MOSFET; the resistor sits between VSTORE and the
 * MOSFET. One pulse removes about  V^2 / R * pulse_time  joules from the capacitor.
 * Example: 4.6 V, 1000 ohm, 20 ms -> about 0.42 mJ per event. */
#define LOAD_RESISTOR_OHM 1000.0
#define LOAD_PULSE_MS 20

/* Capacitor and target.
 * The target is fixed once, at the first valid reading:
 *     target = ENERGY_TARGET_FRACTION * (capacitor energy at the first reading)
 * and then stays the same for every row of the run. Start each run with the capacitor
 * charged close to full (about 5.2 V) so 80% of it is a sensible operating point. */
#define CAPACITANCE_F 0.1
#define ENERGY_TARGET_FRACTION 0.8

/* PID gains. The controller output is an event RATE in Hz (events per second).
 *   error = stored energy - target            (joules; positive = surplus)
 *   rate  = RATE_BIAS_HZ + Kp*error + Ki*(sum of error*dt) + Kd*(d error / dt)
 * Units: Kp in Hz per J, Ki in Hz per (J*s), Kd in Hz per W.
 *
 * Where the starting values come from: one event removes about E_pulse = 0.42 mJ.
 * With P only, the error shrinks with a time constant of  1 / (E_pulse * Kp).
 * Kp = 40 gives about 60 s, slow enough to ignore ADC noise and fast enough to watch.
 * Ki = 0.15 removes the steady offset that P alone leaves when RATE_BIAS_HZ is not
 * exactly the balance rate. Keep Kd = 0 at first: the energy slope is noisy. */
#define PID_KP 40.0
#define PID_KI 0.15
#define PID_KD 0.0

/* The rate the controller uses when the error is zero and the integral is empty.
 * 0.2 Hz = one event every 5 s. */
#define RATE_BIAS_HZ 0.2

/* Fastest and slowest allowed cycle. The controller is clamped to this range. */
#define EVENT_INTERVAL_MIN_S 1.0
#define EVENT_INTERVAL_MAX_S 30.0

/* The energy slope (dE/dt) is smoothed with this time constant before it is logged
 * or used for the D term. A longer value is smoother but reacts later. */
#define PID_SLOPE_FILTER_S 30.0

/* ------------------------------------------------------------------------------------
 * CSV layout
 * ------------------------------------------------------------------------------------ */

static const char measurement_columns[] =
	"time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
	"vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
	"temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status";
static const char controller_columns[] =
	"cap_energy_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,"
	"battery_start_mWh,load_pulse_uJ,pid_p_Hz,pid_i_Hz,pid_d_Hz";

/* ------------------------------------------------------------------------------------
 * Hardware handles. Zephyr builds these from boards/xiao_ble_nrf52840.overlay.
 * ------------------------------------------------------------------------------------ */

static const struct adc_dt_spec vst_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vst);
static const struct adc_dt_spec vout_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vout);
static const struct adc_dt_spec vin_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vin);
static const struct device *const sht40 = DEVICE_DT_GET(DT_NODELABEL(sht40));
/* The MOSFET gate pin (XIAO D2). "load-gpios" in the overlay. */
static const struct gpio_dt_spec load_switch =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), load_gpios);

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
	double previous_energy_j; /* Last cycle's energy, to compute the slope. */
	int64_t previous_ms;      /* Last cycle's time stamp. */
	bool has_previous;        /* False on the first cycle and after a failed reading. */
	double net_power_w;       /* Smoothed dE/dt: harvest minus pulses minus losses. */
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
	 * the longest interval. Keep the integral, but do not compute a slope across
	 * the gap. */
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
		printf("# Target fixed at %.6f J (%.0f%% of first reading %.6f J).\n",
		       pid->target_j, ENERGY_TARGET_FRACTION * 100.0, energy_j);
	}

	/* Positive error = more energy than the target = we can afford to send faster. */
	double error_j = energy_j - pid->target_j;
	double dt_s = (now_ms - pid->previous_ms) / 1000.0;
	double integral_step_hz = 0.0;

	if (pid->has_previous && dt_s > 0.0) {
		/* Slope of stored energy since the last cycle, in watts (J/s). One reading
		 * is noisy, so blend each new slope into a running average
		 * (a first-order low-pass filter). alpha is how much the new value counts. */
		double slope_w = (energy_j - pid->previous_energy_j) / dt_s;
		double alpha = dt_s / (PID_SLOPE_FILTER_S + dt_s);

		pid->net_power_w = isfinite(pid->net_power_w) ?
			pid->net_power_w + alpha * (slope_w - pid->net_power_w) :
			slope_w;
		/* The I term adds up error x time. */
		integral_step_hz = PID_KI * error_j * dt_s;
	}
	pid->previous_energy_j = energy_j;
	pid->previous_ms = now_ms;
	pid->has_previous = true;

	/* P term: react to how far we are from the target right now. */
	pid->p_hz = PID_KP * error_j;
	/* D term: react to how fast the energy is moving. The target is constant,
	 * so d(error)/dt is the same as dE/dt. Energy rising -> send faster. */
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
 * The event "cost": one pulse through the load resistor
 * ------------------------------------------------------------------------------------ */

/* Switch the MOSFET on for LOAD_PULSE_MS and return an estimate of the energy removed
 * from the capacitor in microjoules (V^2 / R * t). The voltage barely changes during
 * one pulse (about 1 mV), so the reading from the start of the cycle is used. */
static double pulse_load(bool load_ready, double vst_voltage)
{
	if (!LOAD_ENABLED || !load_ready || !isfinite(vst_voltage)) {
		return NAN;
	}
	gpio_pin_set_dt(&load_switch, 1);
	k_msleep(LOAD_PULSE_MS);
	gpio_pin_set_dt(&load_switch, 0);
	return vst_voltage * vst_voltage / LOAD_RESISTOR_OHM * (LOAD_PULSE_MS / 1000.0) * 1e6;
}

/* ------------------------------------------------------------------------------------
 * Sensor reads (unchanged logic)
 * ------------------------------------------------------------------------------------ */

/* One ADS1115 reading of the TEG voltage. Returns 0 on success, or a negative error
 * code. On success writes the raw count and the voltage in mV. */
static int read_vin(double *raw, double *millivolts)
{
	int16_t sample;
	struct adc_sequence sequence = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};
	int ret = adc_sequence_init_dt(&vin_input, &sequence);

	if (ret < 0) {
		return ret;
	}
	ret = adc_read_dt(&vin_input, &sequence);
	if (ret < 0) {
		return ret;
	}

	/* Zephyr converts the copy to microvolts in place; sample keeps the raw count. */
	int32_t microvolts = sample;

	ret = adc_raw_to_microvolts_dt(&vin_input, &microvolts);
	if (ret < 0) {
		return ret;
	}
	/* The largest possible code means "256 mV or more", so report it as out of range. */
	if (sample == INT16_MAX) {
		return -ERANGE;
	}
	*raw = sample;
	*millivolts = microvolts / 1000.0;
	return 0;
}

/* One SHT40 reading. Zephyr stores each value as whole + fractional parts;
 * sensor_value_to_double turns that into a normal number. */
static int read_environment(double *temperature, double *humidity)
{
	struct sensor_value temp;
	struct sensor_value rh;
	int ret = sensor_sample_fetch(sht40);

	if (ret < 0) {
		return ret;
	}
	ret = sensor_channel_get(sht40, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_channel_get(sht40, SENSOR_CHAN_HUMIDITY, &rh);
	if (ret < 0) {
		return ret;
	}
	*temperature = sensor_value_to_double(&temp);
	*humidity = sensor_value_to_double(&rh);
	return 0;
}

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

	/* Start the MOSFET pin OFF so the capacitor is never drained by accident. */
	bool load_ready = gpio_is_ready_dt(&load_switch) &&
		gpio_pin_configure_dt(&load_switch, GPIO_OUTPUT_INACTIVE) == 0;

	/* The sensors are optional: if one is missing, the others keep running and
	 * its status column shows the error. */
	int ads_setup_status = adc_is_ready_dt(&vin_input) ?
		adc_channel_setup_dt(&vin_input) : -ENODEV;
	int sht_setup_status = device_is_ready(sht40) ? 0 : -ENODEV;
	int ble_status = ble_output_init();
	struct energy_pid pid = {
		.target_j = NAN,
		.net_power_w = NAN,
		.p_hz = NAN,
		.i_hz = NAN,
		.d_hz = NAN,
	};
	struct battery_run_input battery = {.start_mwh = NAN};
	bool usb_was_open = false;

	printf("# D0=VOUT, D1=VSTORE; each uses 1M/330k.\n");
	printf("# ADS1115 A0: VIN, 0x48, 0..256 mV; SHT40: 0x44. Startup ADS=%d, SHT=%d.\n",
	       ads_setup_status, sht_setup_status);

	while (1) {
		/* Board uptime in ms: the row's time stamp and the start of this cycle. */
		int64_t cycle_start = k_uptime_get();
		uint32_t dtr = 0;

		/* "DTR" is set while a program on the laptop has the USB serial port open
		 * (for example capture_csv.py). When one opens, print the settings and the
		 * column header. On battery with no USB, this is simply skipped. */
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		if (dtr && !usb_was_open) {
			printf("# BLE status=%d; PID=%d; load=%d (ready=%d), %.0f ohm, %d ms\n",
			       ble_status, PID_ENABLED, LOAD_ENABLED, load_ready,
			       LOAD_RESISTOR_OHM, LOAD_PULSE_MS);
			printf("# Kp=%.4g Hz/J, Ki=%.4g Hz/(J*s), Kd=%.4g Hz/W, bias=%.3g Hz, "
			       "interval %.1f-%.1f s, target fraction %.2f\n",
			       PID_KP, PID_KI, PID_KD, RATE_BIAS_HZ,
			       EVENT_INTERVAL_MIN_S, EVENT_INTERVAL_MAX_S, ENERGY_TARGET_FRACTION);
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

		/* Step 1: capacitor first, before this cycle's pulse. This reading therefore
		 * includes the harvest and the previous pulse since the last cycle. */
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

		/* Step 3: the other measurements. */
		int vout_status = read_adc(&vout_input, &vout_raw, &vout_pin_voltage);
		if (vout_status == 0) {
			vout_voltage = source_voltage(vout_raw, vout_pin_voltage,
				VOUT_CAL_VOLTAGE, VOUT_CAL_ADC);
		}
		int ads_status = ads_setup_status;
		int sht_status = sht_setup_status;

		if (ads_status == 0) {
			ads_status = read_vin(&vin_raw, &vin_mv);
		}
		if (sht_status == 0) {
			sht_status = read_environment(&temperature, &humidity);
		}

		/* Step 4: pay this event's energy out of the capacitor. */
		double load_pulse_uj = pulse_load(load_ready, vst_voltage);

		/* Step 5: build the row and send it. */
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
				.target_j = pid.target_j,
				.net_power_uw = pid.net_power_w * 1000000.0,
				.interval_ms = event_interval_ms,
				.battery_start_mwh = battery.start_mwh,
				.load_pulse_uj = load_pulse_uj,
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
				printf("%.*s,%.6f,%.6f,%.6f,%.2f,%d,%d,%.9g,%.1f,%.5f,%.5f,%.5f\n",
				       length - 1, line, cap_energy_j, pid.target_j,
				       cap_energy_j - pid.target_j, pid.net_power_w * 1000000.0,
				       event_interval_ms, ble_sent, battery.start_mwh,
				       load_pulse_uj, pid.p_hz, pid.i_hz, pid.d_hz);
			}
		}

		/* Step 6: sleep for whatever is left of the interval. The interval runs from
		 * the start of one cycle to the start of the next, so the time spent
		 * measuring and sending is already included. */
		int64_t remaining = event_interval_ms - (k_uptime_get() - cycle_start);
		if (remaining > 0) {
			k_msleep(remaining);
		}
	}
	return 0;
}

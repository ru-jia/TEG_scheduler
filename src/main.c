/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ble_output.h"

/* Each input: source -- 1 Mohm -- ADC -- 330 kohm -- GND.
 * No external ADC filter capacitor is required for this divider at 40 us.
 * D0/A0 measures VOUT; D1/A1 measures VSTORE. */
static const double R_TOP_OHM = 1000000.0;
static const double R_BOTTOM_OHM = 330000.0;

/* The 2026-11-22 VSTORE fit maps averaged D1 counts directly to capacitor volts.
 * Matched multimeter readings span 0.608 to 5.180 V. */
static const double VST_CAL_GAIN_V_PER_COUNT = 0.003490639465;
static const double VST_CAL_OFFSET_V = 0.047304725;

/* VOUT one-point scale from the time-matched 5 V recording (IMG_5676).
 * Leave both at zero to use the nominal divider ratio. */
static const double VOUT_CAL_VOLTAGE = 5.09;
static const double VOUT_CAL_ADC = 1411.3;

/* Each onboard ADC channel gets one discarded conversion and 20 retained
 * conversions. The main loop's measurement period is a separate setting. */
#define SAMPLES 20
#define SAMPLE_SPACING_MS 5
/* Fixed period for the baseline that senses and sends at 1 Hz. */
#define ADC_TEST_INTERVAL_MS 1000

/* Tune these for the capacitor tracking experiment. Set PID_ENABLED to 0
 * for a baseline: sample and try to send once per second. */
#define PID_ENABLED 1
#define CAPACITANCE_F 0.1
#define ENERGY_TARGET_FRACTION 0.5
#define PID_KP 0.1             /* s/J */
#define PID_KI 0.0              /* 1/J */
#define PID_KD 0.0              /* s^2/J */
#define PID_SLOPE_FILTER_S 10.0 /* Low-pass time constant for dE/dt. */
#define EVENT_INTERVAL_MIN_S 1.0
#define EVENT_INTERVAL_BIAS_S 5.0 /* Interval at zero error with I and D at zero. */
#define EVENT_INTERVAL_MAX_S 30.0

/* USB and opt-in BLE add controller fields after the measurement columns. */
static const char measurement_columns[] =
	"time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
	"vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
	"temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status";
static const char controller_columns[] =
	"cap_energy_J,cap_target_J,cap_error_J,cap_net_power_uW,event_interval_ms,ble_sent,"
	"battery_start_mWh";

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

	/* Read only buffered bytes. USB input never delays a sensing cycle. */
	for (size_t i = 0; i < sizeof(input->line) && uart_poll_in(console, &byte) == 0; i++) {
		if (byte != '\r' && byte != '\n') {
			if (!input->discard && byte != '\0' && input->length < sizeof(input->line) - 1) {
				input->line[input->length++] = byte;
			} else {
				input->discard = true;
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

/* Zephyr resolves these names from the board overlay at build time.
 * vst and vout use the XIAO ADC. vin and sht40 are on the I2C bus. */
static const struct adc_dt_spec vst_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vst);
static const struct adc_dt_spec vout_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vout);
static const struct adc_dt_spec vin_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vin);
static const struct device *const sht40 = DEVICE_DT_GET(DT_NODELABEL(sht40));

/* State kept across sampling cycles. Gains and units are above. */
struct energy_pid {
	double target_j;
	double previous_energy_j;
	double net_power_w;
	double integral_s; /* Integral contribution already scaled by Ki. */
	int64_t previous_ms;
	bool has_target;
	bool has_previous;
};

static int update_event_interval(struct energy_pid *pid, double energy_j, int64_t now_ms)
{
	if (!isfinite(energy_j)) {
		/* Preserve the reference and integral. Avoid differentiating or
		 * integrating across a gap in measurements. */
		pid->has_previous = false;
		pid->net_power_w = NAN;
		return PID_ENABLED ? (int)(EVENT_INTERVAL_MAX_S * 1000.0) :
			ADC_TEST_INTERVAL_MS;
	}

	if (!pid->has_target) {
		/* Capture the first valid capacitor energy once per boot. */
		pid->target_j = ENERGY_TARGET_FRACTION * energy_j;
		pid->has_target = true;
	}

	/* Positive error means the capacitor is below target. A positive
	 * correction lengthens the shared sense-and-send interval. */
	double error_j = pid->target_j - energy_j;
	double integral_step_s = 0.0;
	double derivative_s = 0.0;
	double dt_s = (now_ms - pid->previous_ms) / 1000.0;

	if (pid->has_previous && dt_s > 0.0) {
		double slope_w = (energy_j - pid->previous_energy_j) / dt_s;
		/* This slope already includes all charging and loads seen by
		 * VSTORE. Filter it before using it for the derivative term. */
		double alpha = dt_s / (PID_SLOPE_FILTER_S + dt_s);

		pid->net_power_w = isfinite(pid->net_power_w) ?
			pid->net_power_w + alpha * (slope_w - pid->net_power_w) :
			alpha * slope_w;
		integral_step_s = PID_KI * error_j * dt_s;
		/* The reference is fixed, so d(error)/dt is -d(energy)/dt. */
		derivative_s = -PID_KD * pid->net_power_w;
	}

	pid->previous_energy_j = energy_j;
	pid->previous_ms = now_ms;
	pid->has_previous = true;
	if (!PID_ENABLED) {
		return ADC_TEST_INTERVAL_MS;
	}

	double wait_s = EVENT_INTERVAL_BIAS_S + PID_KP * error_j +
			pid->integral_s + derivative_s;
	double candidate_s = wait_s + integral_step_s;

	/* Freeze the integral when its next step would push farther beyond
	 * a limit. Allow it to unwind when the error changes direction. */
	if ((candidate_s <= EVENT_INTERVAL_MAX_S || integral_step_s < 0.0) &&
	    (candidate_s >= EVENT_INTERVAL_MIN_S || integral_step_s > 0.0)) {
		pid->integral_s += integral_step_s;
		wait_s = candidate_s;
	}
	wait_s = fmin(fmax(wait_s, EVENT_INTERVAL_MIN_S), EVENT_INTERVAL_MAX_S);
	return (int)(wait_s * 1000.0 + 0.5);
}

/* Take one ADS1115 reading. Return a status code and, on success, write
 * the ADC count and the converted input voltage into the caller's variables. */
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

	/* Zephyr converts this copy in place. sample keeps the original count. */
	int32_t microvolts = sample;

	ret = adc_raw_to_microvolts_dt(&vin_input, &microvolts);
	if (ret < 0) {
		return ret;
	}
	/* A full-scale code cannot distinguish 256 mV from overrange. */
	if (sample == INT16_MAX) {
		return -ERANGE;
	}
	*raw = sample;
	*millivolts = microvolts / 1000.0;
	return 0;
}

/* Fetch one SHT40 sample, then extract temperature and humidity from it.
 * Zephyr sensor_value stores the whole and fractional parts separately. */
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

/* Average one divided-voltage channel. The caller gets the mean count
 * for calibration and the mean ADC-pin voltage for logging. */
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

	/* The sequence reuses the same buffer for one conversion at a time. */
	/* Discard the first conversion after switching channels. */
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
		/* This sleep follows each retained conversion, including the last.
		 * The ADC's 40 us acquisition window is configured separately. */
		k_msleep(SAMPLE_SPACING_MS);
	}

	*raw_average = (double)raw_total / SAMPLES;
	*pin_voltage = (double)microvolt_total / SAMPLES / 1000000.0;
	return 0;
}

/* Use the VOUT calibration when both reference values are positive.
 * Otherwise multiply the measured pin voltage by the nominal divider ratio's inverse. */
static double source_voltage(double raw, double pin, double cal_v, double cal_adc)
{
	if (cal_v > 0.0 && cal_adc > 0.0) {
		return raw * cal_v / cal_adc;
	}
	return pin * (R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;
}

int main(void)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int ret;

	/* The console and both onboard voltage channels are required at startup. */
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
	/* Keep each optional device's startup status so the other channels
	 * can keep running if one device is unavailable. */
	int ads_setup_status = adc_is_ready_dt(&vin_input) ?
		adc_channel_setup_dt(&vin_input) : -ENODEV;
	int sht_setup_status = device_is_ready(sht40) ? 0 : -ENODEV;
	int ble_status = ble_output_init();
	struct energy_pid pid = {.net_power_w = NAN};
	/* Per-run battery metadata lives in RAM and stays separate from the
	 * capacitor energy used by the PID. USB can change it without a rebuild. */
	struct battery_run_input battery = {.start_mwh = NAN};
	bool usb_was_open = false;

	printf("# D0=VOUT, D1=VSTORE; each uses 1M/330k; no external ADC filter capacitor.\n");
	printf("# ADS1115 A0: VIN, address 0x48, range 0..256 mV; SHT40: 0x44.\n");
	printf("# Startup status: ADS=%d, SHT=%d (0=ready).\n",
	       ads_setup_status, sht_setup_status);

	while (1) {
		/* Uptime in ms anchors the row timestamp and the cycle sleep. */
		int64_t cycle_start = k_uptime_get();
		uint32_t dtr = 0;

		/* DTR follows the host's USB serial session. Print the header
		 * when it opens. Sampling continues with the session closed. */
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		if (dtr && !usb_was_open) {
			printf("# BLE TEG_XIAO: startup status=%d; PID enabled=%d; Kp=%.4g, Ki=%.4g, Kd=%.4g\n",
			       ble_status, PID_ENABLED, PID_KP, PID_KI, PID_KD);
			printf("# VST calibration: raw * %.12f + %.9f V. Reference medians: 0.608-5.180 V.\n",
			       VST_CAL_GAIN_V_PER_COUNT, VST_CAL_OFFSET_V);
			if (VOUT_CAL_VOLTAGE > 0.0 && VOUT_CAL_ADC > 0.0) {
				printf("# VOUT calibration: raw * %.3f V / %.1f counts (5 V setting).\n",
				       VOUT_CAL_VOLTAGE, VOUT_CAL_ADC);
			}
			printf("%s,%s\n", measurement_columns, controller_columns);
		}
		if (dtr) {
			read_battery_run_input(console, &battery);
		} else {
			/* Discard any partial command if the host closes the port. */
			battery.length = 0;
			battery.discard = false;
		}
		usb_was_open = dtr != 0;

		/* Start with missing values each cycle. The status columns
		 * explain failures, and a failed read cannot reuse old data. */
		double vst_raw = NAN, vst_pin_voltage = NAN, vst_voltage = NAN;
		double vout_raw = NAN, vout_pin_voltage = NAN, vout_voltage = NAN;
		double vin_raw = NAN, vin_mv = NAN;
		double temperature = NAN, humidity = NAN;
		int vst_status = read_adc(&vst_input, &vst_raw, &vst_pin_voltage);
		int vout_status = read_adc(&vout_input, &vout_raw, &vout_pin_voltage);

		/* Apply the separate calibrations to the two voltage rails. */
		if (vst_status == 0) {
			vst_voltage = vst_raw * VST_CAL_GAIN_V_PER_COUNT + VST_CAL_OFFSET_V;
		}
		if (vout_status == 0) {
			vout_voltage = source_voltage(vout_raw, vout_pin_voltage,
				VOUT_CAL_VOLTAGE, VOUT_CAL_ADC);
		}
		/* Report the startup error if the device was unavailable.
		 * Otherwise report the result of this cycle's read. */
		int ads_status = ads_setup_status;
		int sht_status = sht_setup_status;

		if (ads_status == 0) {
			ads_status = read_vin(&vin_raw, &vin_mv);
		}
		if (sht_status == 0) {
			sht_status = read_environment(&temperature, &humidity);
		}

		/* VSTORE describes the capacitor. Convert volts to joules before
		 * comparing against the reference or estimating charging power. */
		double cap_energy_j = NAN;
		if (vst_status == 0 && isfinite(vst_voltage) && vst_voltage > 0.0) {
			cap_energy_j = 0.5 * CAPACITANCE_F * vst_voltage * vst_voltage;
		}
		int event_interval_ms = update_event_interval(&pid, cap_energy_j, cycle_start);

		/* The measurement row is also the default BLE format. */
		char line[256];
		int length = snprintf(line, sizeof(line),
		       "%.3f,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.0f,%.3f,%.2f,%.2f,%d,%d,%d,%d\n",
			       cycle_start / 1000.0, vst_raw, vst_pin_voltage, vst_voltage,
			       vout_raw, vout_pin_voltage, vout_voltage, vin_raw, vin_mv,
		       temperature, humidity, vst_status, vout_status, ads_status, sht_status);

		double target_j = pid.has_target ? pid.target_j : (double)NAN;
		char controller_fields[112];
		int controller_length = snprintf(controller_fields, sizeof(controller_fields),
						 "%.6f,%.6f,%.6f,%.2f,%d", cap_energy_j, target_j,
						 target_j - cap_energy_j, pid.net_power_w * 1000000.0,
						 event_interval_ms);
		bool ble_sent = false;
		if (length < 0 || (size_t)length >= sizeof(line) || controller_length < 0 ||
		    (size_t)controller_length >= sizeof(controller_fields)) {
			printf("# CSV formatting failed.\n");
		} else {
			/* One cycle is one sensing event and one BLE send attempt.
			 * A failed adaptive VSTORE read skips the send this cycle. */
			if (ble_status == 0 && (!PID_ENABLED || isfinite(cap_energy_j))) {
				ret = ble_output_send(line, length, controller_fields, battery.start_mwh);
				if (ret == 0) {
					ble_sent = true;
				} else if (ret != -ENOTCONN) {
					printf("# BLE send failed: %d\n", ret);
				}
			}
			if (dtr) {
				printf("%.*s,%s,%d,%.9g\n", length - 1, line, controller_fields,
				       ble_sent, battery.start_mwh);
			}
		}

		/* The interval is start to start and includes sensing and sending.
		 * The next VSTORE reading and PID update happen in the next cycle. */
		int64_t remaining = event_interval_ms - (k_uptime_get() - cycle_start);
		if (remaining > 0) {
			k_msleep(remaining);
		}
	}
	return 0;
}

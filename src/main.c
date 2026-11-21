/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
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

/* VSTORE fit to nine time-matched multimeter medians from 2026-11-20/21.
 * Reference medians span 0.818 to 2.480 V. The fit maps D1 counts to source volts. */
static const double VST_CAL_GAIN_V_PER_COUNT = 0.004209365651919832;
static const double VST_CAL_OFFSET_V = 0.12394891661442003;

/* VOUT one-point scale from the time-matched 5 V recording (IMG_5676).
 * Leave both at zero to use the nominal divider ratio. */
static const double VOUT_CAL_VOLTAGE = 5.09;
static const double VOUT_CAL_ADC = 1411.3;

#define SAMPLES 20
#define SAMPLE_SPACING_MS 5
#define ADC_TEST_INTERVAL_MS 1000

/* Short interval for phone tests; use INITIAL_TX_INTERVAL_MS later. */
#define BLE_TEST_INTERVAL_MS 5000
#define INITIAL_TX_INTERVAL_MS (300 * 1000)

static const char csv_header[] =
	"time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
	"vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
	"temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status\n";

static const struct adc_dt_spec vst_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vst);
static const struct adc_dt_spec vout_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vout);
static const struct adc_dt_spec vin_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vin);
static const struct device *const sht40 = DEVICE_DT_GET(DT_NODELABEL(sht40));

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
		k_msleep(SAMPLE_SPACING_MS);
	}

	*raw_average = (double)raw_total / SAMPLES;
	*pin_voltage = (double)microvolt_total / SAMPLES / 1000000.0;
	return 0;
}

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
	int ads_setup_status = adc_is_ready_dt(&vin_input) ?
		adc_channel_setup_dt(&vin_input) : -ENODEV;
	int sht_setup_status = device_is_ready(sht40) ? 0 : -ENODEV;
	int ble_status = ble_output_init();
	int64_t next_ble_time = 0;
	bool usb_was_open = false;

	printf("# D0=VOUT, D1=VSTORE; each uses 1M/330k; no external ADC filter capacitor.\n");
	printf("# ADS1115 A0: VIN, address 0x48, range 0..256 mV; SHT40: 0x44.\n");
	printf("# Startup status: ADS=%d, SHT=%d (0=ready).\n",
	       ads_setup_status, sht_setup_status);

	while (1) {
		int64_t cycle_start = k_uptime_get();
		uint32_t dtr = 0;

		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		if (dtr && !usb_was_open) {
			printf("# BLE TEG_XIAO: startup status=%d; interval=%d ms.\n",
			       ble_status, BLE_TEST_INTERVAL_MS);
			printf("# VST calibration: raw * %.9f + %.6f V. Reference medians: 0.818-2.480 V.\n",
			       VST_CAL_GAIN_V_PER_COUNT, VST_CAL_OFFSET_V);
			if (VOUT_CAL_VOLTAGE > 0.0 && VOUT_CAL_ADC > 0.0) {
				printf("# VOUT calibration: raw * %.3f V / %.1f counts (5 V setting).\n",
				       VOUT_CAL_VOLTAGE, VOUT_CAL_ADC);
			}
			printf("%s", csv_header);
		}
		usb_was_open = dtr != 0;

		double vst_raw = NAN, vst_pin_voltage = NAN, vst_voltage = NAN;
		double vout_raw = NAN, vout_pin_voltage = NAN, vout_voltage = NAN;
		double vin_raw = NAN, vin_mv = NAN;
		double temperature = NAN, humidity = NAN;
		int vst_status = read_adc(&vst_input, &vst_raw, &vst_pin_voltage);
		int vout_status = read_adc(&vout_input, &vout_raw, &vout_pin_voltage);

		if (vst_status == 0) {
			vst_voltage = vst_raw * VST_CAL_GAIN_V_PER_COUNT + VST_CAL_OFFSET_V;
		}
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
		char line[256];
		int length = snprintf(line, sizeof(line),
		       "%.3f,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.0f,%.3f,%.2f,%.2f,%d,%d,%d,%d\n",
			       cycle_start / 1000.0, vst_raw, vst_pin_voltage, vst_voltage,
			       vout_raw, vout_pin_voltage, vout_voltage, vin_raw, vin_mv,
		       temperature, humidity, vst_status, vout_status, ads_status, sht_status);

		if (length < 0 || length >= sizeof(line)) {
			printf("# CSV formatting failed.\n");
			k_msleep(ADC_TEST_INTERVAL_MS);
			continue;
		}
		if (dtr) {
			printf("%s", line);
		}
		if (ble_status == 0 && k_uptime_get() >= next_ble_time) {
			ret = ble_output_send(line, length);
			if (ret == 0) {
				next_ble_time = k_uptime_get() + BLE_TEST_INTERVAL_MS;
			} else if (ret != -ENOTCONN) {
				printf("# BLE send failed: %d\n", ret);
			}
		}

		/* Target one row per second, including acquisition time. */
		int64_t remaining = ADC_TEST_INTERVAL_MS - (k_uptime_get() - cycle_start);
		if (remaining > 0) {
			k_msleep(remaining);
		}
	}
	return 0;
}

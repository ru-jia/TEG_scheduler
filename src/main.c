/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/* VOUT -- 1 Mohm -- D0/A0 -- 330 kohm -- GND. */
static const double R_TOP_OHM = 1000000.0;
static const double R_BOTTOM_OHM = 330000.0;

/*
 * Optional end-to-end calibration. Leave both at zero initially.
 * At a stable output, measure VOUT-to-GND with a multimeter and record
 * adc_raw from the same operating point. Put that pair here and rebuild.
 * CAL_VOLTAGE is VOUT, in volts; CAL_ADC is the averaged raw ADC count.
 * This calibration already includes the resistor divider ratio.
 */
static const double CAL_VOLTAGE = 0.0;
static const double CAL_ADC = 0.0;

#define SAMPLES 20
#define SAMPLE_SPACING_MS 5
#define INTERVAL_MS 1000

static const struct adc_dt_spec adc_input =
	ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int read_adc(double *raw_average, double *pin_voltage)
{
	int16_t sample;
	int32_t raw_total = 0;
	int64_t microvolt_total = 0;
	struct adc_sequence sequence = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};
	int ret = adc_sequence_init_dt(&adc_input, &sequence);

	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < SAMPLES; i++) {
		ret = adc_read_dt(&adc_input, &sequence);
		if (ret < 0) {
			return ret;
		}

		raw_total += sample;
		int32_t microvolts = sample;

		ret = adc_raw_to_microvolts_dt(&adc_input, &microvolts);
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

int main(void)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int ret;

	if (!device_is_ready(console) || !adc_is_ready_dt(&adc_input)) {
		return 0;
	}

	/* USB CDC is initialized at boot by the Zephyr USB stack. */
	uint32_t dtr = 0;

	while (!dtr) {
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		k_msleep(100);
	}

	ret = adc_channel_setup_dt(&adc_input);
	if (ret < 0) {
		printf("# ADC setup failed: %d\n", ret);
		return 0;
	}

	printf("# D0/A0 voltage test; VOUT uses the 1M/330k divider.\n");
	printf("# VOUT calibration: %s\n",
	       CAL_ADC > 0.0 && CAL_VOLTAGE > 0.0 ? "enabled" : "not yet calibrated");
	printf("time_s,adc_raw,d0_voltage_V,vout_voltage_V\n");

	while (1) {
		double adc_raw;
		double d0_voltage;

		ret = read_adc(&adc_raw, &d0_voltage);
		if (ret < 0) {
			printf("# ADC read failed: %d\n", ret);
		} else {
			double vout_voltage = d0_voltage *
				(R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;

			if (CAL_ADC > 0.0 && CAL_VOLTAGE > 0.0) {
				vout_voltage = adc_raw * CAL_VOLTAGE / CAL_ADC;
			}

			printf("%.3f,%.1f,%.4f,%.4f\n",
			       k_uptime_get() / 1000.0, adc_raw, d0_voltage, vout_voltage);
		}

		/* As in the original sketch, wait after each averaged reading. */
		k_msleep(INTERVAL_MS);
	}
	return 0;
}

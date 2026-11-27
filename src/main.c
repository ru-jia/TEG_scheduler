/*
 * TEG energy scheduler for the XIAO nRF52840: harvester-powered sensors.
 *
 * THE HARDWARE
 *   The TEG feeds the LTC3108. The LTC3108 makes VOUT = 3.3 V, which powers the
 *   SHT40 and the ADS1115, and puts any extra harvest into the 0.1 F capacitor on
 *   VSTORE. When the sensors need more than the TEG supplies, the LTC3108 takes the
 *   difference from that capacitor. So every sensor reading is paid for with
 *   harvested energy, and the capacitor's energy shows how much reserve is left.
 *   The nRF52840 itself (and BLE) runs from USB, because it needs far more power
 *   than the TEG produces.
 *
 * ONE CYCLE OF THE MAIN LOOP
 *   1. Read the capacitor voltage (VSTORE, pin D1) and turn it into energy in joules.
 *   2. Ask the controller how long to wait before the next cycle.
 *   3. Read VOUT (pin D0). If VOUT is high enough, the sensors have power, so read
 *      the SHT40 (temperature, humidity) and the ADS1115 (TEG voltage VIN).
 *      If VOUT is too low, skip the sensors this cycle and try again next cycle.
 *   4. Send one CSV row over BLE, and print it on USB if a terminal is open.
 *   5. Sleep until the next cycle is due.
 *
 * THE CONTROL IDEA
 *   The capacitor energy E changes as
 *       dE/dt = P_harvest - P_idle - E_event * f
 *   P_idle   everything that drains the capacitor all the time (sensor boards at
 *            rest, voltage dividers, the LTC3108 itself, capacitor leakage),
 *   E_event  the extra energy of one round of sensor readings,
 *   f        the event rate: readings per second, f = 1 / interval.
 *   This equation is to give a sense of the system's energy balance. The code
 *   never uses P_harvest, P_idle or E_event: the controller only compares the
 *   measured capacitor energy with the target, and feedback does the rest.
 *   The goal is net power zero: spend exactly what is harvested. The controller
 *   does that by holding the capacitor at the energy it had at the start:
 *   - more energy than the target -> raise f (measure more often),
 *   - less energy than the target -> lower f (measure less often).
 *   The controller computes a rate f, and the CSV shows the interval 1000 / f in ms.
 *   It works on the rate rather than the interval because energy spent per second
 *   is proportional to the rate, so the same gains behave the same at 1 s or 20 s.
 */

#include <errno.h> //contains named error codes. used in status column
#include <math.h> // math functions. used: NAN for no reading, isfinite(), fmin(), fmax()
#include <stdbool.h> //bool types
#include <stdio.h> // provides print functions
#include <zephyr/drivers/adc.h> // XIAO's built'in ADC function
#include <zephyr/drivers/i2c.h> // send and receive bytes on I2C bus
#include <zephyr/drivers/uart.h> // check if there is a USB plugged in
#include <zephyr/kernel.h> // uptime, sleep time, check if device is ready, etc

#include "ble_output.h" // 3 bluetooth functions for start, send, and check for requests

/* ====================================================================================
 * Settings you change between experiments
 * ==================================================================================== */

/* 1 = the controller picks the interval.
 * 0 = fixed interval of BASELINE_INTERVAL_MS (for measuring P_idle and E_event). */
#define PID_ENABLED 0
#define BASELINE_INTERVAL_MS 500

/* Number of SHT40 readings averaged per event. More readings give smoother data and
 * cost more harvested energy per event. */
#define SHT_READS_PER_EVENT 1 //read_environment() loops this many times

/* Storage capacitor on VSTORE, in farads. Stored energy E = 1/2 * C * V^2. */
#define CAPACITANCE_F 0.1

/* Target energy = ENERGY_TARGET_FRACTION * (capacitor energy at the first reading).
 * 1.0 holds the starting level, which means net power zero. Use a value below 1.0
 * only for a step test. 
 *    -> Say you set 0.8. The target becomes 80% of the starting energy, 
 * 		so at the first reading the setpoint jumps down.
 * 		can use this to check response time and gain settings.
 * 		also 0.8 might be too big, bc total energy stored in the current CAP is 1.35J at max,
 * 		and 80% of it requires about 48min to drain it to 80% with TEG disconnected 
 * 		before any refill happens. if try, try 0.99.
 * Start each run with VSTORE below about 5 V: near 5.2 V the LTC3108 stops
 * charging the capacitor, so extra harvest is thrown away and cannot be seen. */
#define ENERGY_TARGET_FRACTION 1.0

/* The raw energy reading wobbles by about 1 mJ from ADC noise, more than one event
 * (tens of uJ). The controller therefore uses a smoothed energy: a running average
 * with this time constant, in seconds. The raw energy is still logged. 
 * 		an example to understand this:
 * 			cap_energy_J:       1.0120  1.0105  1.0131  1.0098  ...   (jumps by ~1 mJ)
 * 			cap_energy_filt_J:  1.0120  1.0119  1.0120  1.0119  ...   (barely moves)
 * 		and ENERGY_FILTER_S is how far back the new reading counts to calculate an average.
 * 		every older reading still counts, but its weight shrinks by a factor of e for evey 10s.
 * */
#define ENERGY_FILTER_S 10.0

/* PID gains. The controller output is an event rate in Hz:
 *     error = smoothed energy - target                 (J; positive = surplus)
 *     rate  = RATE_BIAS_HZ + Kp * error + Ki * (sum of error * dt) + Kd * dE/dt
 * Units: Kp in Hz/J, Ki in Hz/(J*s), Kd in Hz/W.
 * 
 * an example to get some intuition:
 * 		Say the capacitor is 1 mJ above target. or 0.001J.
 * 		The P term adds 500 × 0.001 = 0.5 Hz of extra readings.
 * 		Those cost 0.5 × 20 µJ = 10 µW extra. we assume roughly each event takes 20µJ.
 * 		At 10 µW, spending that 1 mJ takes 1 mJ / 10 µW = 100 s. That's tau.
 * then we run experiments to measure E_event, 
 * and plug it in for the equation below to get actual Kp and Ki
 *
 * Choosing them once E_event (J per event) is measured:
 *   - With P only, the error shrinks with time constant tau = 1 / (E_event * Kp).
 *     For tau of about 100 s:  Kp = 1 / (E_event * 100).
 * 		-> *100 is to react slower than noise
 * 
 *   - To settle without overshoot:  Ki = E_event * Kp^2 / 4.
 * 		-> / 4 is to make the CAP energy don't overshoot relative to target
 * 		-> / 4 is like critically damped
 * 		-> P is like damping and I is like the spring
 * 
 * The values below assume E_event = 20 uJ, an estimate until it is measured:
 *   Kp = 1 / (20e-6 * 100) = 500;  Ki = 20e-6 * 500^2 / 4 = 1.25, rounded to 1.0.
 * 
 * Keep Kd = 0 at first: the energy slope is noisy. */
#define PID_KP 500.0
#define PID_KI 1.0
#define PID_KD 0.0

/* The rate used when the error is zero and the integral is empty: one event every 2 s. 
* rate = RATE_BIAS_HZ + P + I + D, so if PID adds to 0, this is the rate the system runs on
* ideally this is the erngy-neutral rate after experiments
*/
#define RATE_BIAS_HZ 0.5

/* Fastest and slowest allowed cycle, in seconds. One cycle spends about 0.25 s
 * measuring, so 0.5 s is close to the fastest this firmware can run. */
#define EVENT_INTERVAL_MIN_S 0.5
#define EVENT_INTERVAL_MAX_S 30.0

/* Smoothing time constant, in seconds, for the logged net power (dE/dt). 
this only matters if the kd is not zero.*/
#define NET_POWER_FILTER_S 30.0

/* ====================================================================================
 * Measurement constants (change only when the hardware or calibration changes)
 * ==================================================================================== */

/* Both voltages reach the ADC pins through the same kind of divider:
 *     source -- 1 Mohm -- ADC pin -- 330 kohm -- GND
 * D0/A0 measures VOUT; D1/A1 measures VSTORE (the capacitor). */
#define DIVIDER_TOP_OHM 1000000.0 // the divier resistance we used
#define DIVIDER_BOTTOM_OHM 330000.0

/* VSTORE calibration from the Nov 22, 2026 charge runs (multimeter, 0.608-5.180 V):
 *     capacitor volts = averaged D1 count * gain + offset
 * 		RMS error of about 13 mV
 * The fit already includes the divider. */
#define VST_CAL_GAIN_V_PER_COUNT 0.003490639465
#define VST_CAL_OFFSET_V 0.047304725

/* VOUT one-point calibration, made at the old 5 V setting: 5.09 V read 1411.3 counts.
 * It scales counts to volts, so it is still roughly right at 3.3 V. Update both
 * numbers with a multimeter reading at 3.3 V if needed. Setting both to 0 uses the
 * plain divider ratio instead. VOUT only decides whether the sensors are read. */
#define VOUT_CAL_VOLTAGE 5.09
#define VOUT_CAL_ADC 1411.3

/* Each D0/D1 reading throws away one conversion, then averages 20, 5 ms apart. */
#define ADC_SAMPLES 20
#define ADC_SAMPLE_SPACING_MS 5

/* The sensors are read only when VOUT is at least this high. Below it the LTC3108
 * is not holding 3.3 V, and the sensors may be unpowered or browning out. */
#define VOUT_MIN_FOR_SENSORS_V 3.0

/* I2C addresses: ADS1115 with its ADDR pin to GND; the SHT40 address is fixed. */
#define ADS1115_ADDR 0x48
#define SHT40_ADDR 0x44

/* ====================================================================================
 * CSV columns
 * ==================================================================================== */

/* The measurement columns are always sent. The controller columns follow them on
 * USB, and over BLE when the receiver asks for the full row. */
static const char measurement_columns[] =
	"time_s,vst_adc_raw,d1_voltage_V,vst_voltage_V,"
	"vout_adc_raw,d0_voltage_V,vout_voltage_V,vin_adc_raw,vin_mV,"
	"temperature_C,humidity_pct,vst_status,vout_status,ads_status,sht_status";
static const char controller_columns[] =
	"cap_energy_J,cap_energy_filt_J,cap_target_J,cap_error_J,cap_net_power_uW,"
	"event_interval_ms,ble_sent,pid_p_Hz,pid_i_Hz,pid_d_Hz";

/* ====================================================================================
 * Hardware handles, built by Zephyr from boards/xiao_ble_nrf52840.overlay
 * ==================================================================================== */

 //macros Zephyr uses to find pins and stuff, and packs it in a struct
static const struct adc_dt_spec vst_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vst);
static const struct adc_dt_spec vout_input =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vout);

// "static": these handles are private to main.c
// "const" : they never change after the build.
// "const struct device *const": both the device it points to 
// 								 and the pointer itself are read-only.

/* The I2C bus on XIAO D4 (SDA) and D5 (SCL). The sensors are read directly on this
 * bus instead of through Zephyr's sensor drivers: those drivers check each sensor
 * once at boot and give up for good if it has no power then. Reading directly means
 * an unpowered sensor just fails this cycle and works again once VOUT is back. */
static const struct device *const i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* ====================================================================================
 * Onboard ADC: VSTORE and VOUT through their dividers
 * ==================================================================================== */

/* Average ADC_SAMPLES conversions on one onboard ADC pin. Returns the average raw
 * count (used by the calibrations) and the average pin voltage (logged). */
static int read_adc(const struct adc_dt_spec *input, double *raw_average, double *pin_voltage)
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

	/* The first conversion after switching pins can be off, so throw it away. */
	ret = adc_read_dt(input, &sequence);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < ADC_SAMPLES; i++) {
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
		k_msleep(ADC_SAMPLE_SPACING_MS);
	}

	*raw_average = (double)raw_total / ADC_SAMPLES;
	*pin_voltage = (double)microvolt_total / ADC_SAMPLES / 1000000.0;
	return 0;
}

/* VOUT in volts: the one-point calibration if it is set, otherwise the divider ratio. */
static double vout_from_adc(double raw, double pin_voltage)
{
	if (VOUT_CAL_VOLTAGE > 0.0 && VOUT_CAL_ADC > 0.0) {
		return raw * VOUT_CAL_VOLTAGE / VOUT_CAL_ADC;
	}
	return pin_voltage * (DIVIDER_TOP_OHM + DIVIDER_BOTTOM_OHM) / DIVIDER_BOTTOM_OHM;
}

/* ====================================================================================
 * Sensors on the I2C bus, powered from LTC3108 VOUT
 * ==================================================================================== */

/* One ADS1115 reading of the TEG voltage on input A0.
 * Writing the 16-bit config word 0xCB83 to register 0x01 means:
 *   start one conversion, A0 against GND, +/-0.256 V range,
 *   single-shot (sleep afterwards), 128 samples/s, comparator off.
 * A conversion takes about 7.8 ms; then the result is read from register 0x00.
 * One count is 0.256 V / 32768 = 7.8125 uV.
 * Returns 0, an I2C error, or -ERANGE when the input is at or above full scale. */
static int read_vin(double *raw, double *millivolts)
{
	const uint8_t start_conversion[3] = {0x01, 0xCB, 0x83};
	const uint8_t result_register = 0x00;
	uint8_t data[2];
	int ret = i2c_write(i2c_bus, start_conversion, sizeof(start_conversion), ADS1115_ADDR);

	if (ret < 0) {
		return ret;
	}
	k_msleep(10);
	ret = i2c_write_read(i2c_bus, ADS1115_ADDR, &result_register, 1, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	int16_t code = (int16_t)((data[0] << 8) | data[1]);

	/* The largest code means "256 mV or more", so it is reported as out of range. */
	if (code == INT16_MAX) {
		return -ERANGE;
	}
	*raw = code;
	*millivolts = code * (256.0 / 32768.0);
	return 0;
}

/* The SHT40 sends a checksum (CRC-8) after every two data bytes, so a reading
 * corrupted on the bus (for example while VOUT sags) can be detected. */
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
 * The conversion formulas are from the SHT40 datasheet. */
static int read_sht40_once(double *temperature, double *humidity)
{
	const uint8_t measure_command = 0xFD;
	uint8_t data[6];
	int ret = i2c_write(i2c_bus, &measure_command, 1, SHT40_ADDR);

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

/* Average SHT_READS_PER_EVENT SHT40 measurements. Any failed read fails the event. */
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

/* ====================================================================================
 * The controller
 * ==================================================================================== */

/* What the controller remembers from one cycle to the next. */
struct energy_pid {
	double target_j;        /* Set at the first valid reading, then fixed. NAN before. */
	double energy_filt_j;   /* Smoothed capacitor energy; the controller uses this. */
	double previous_filt_j; /* Last cycle's smoothed energy, for the slope. */
	int64_t previous_ms;    /* Last cycle's time stamp. */
	bool has_previous;      /* False on the first cycle and after a failed reading. */
	double net_power_w;     /* Smoothed dE/dt = harvest minus everything drawn. */
	double integral_hz;     /* The I term, already multiplied by Ki. */
	double p_hz;            /* The P and D terms of the latest step, for the log. */
	double d_hz;
};

/* Run one controller step and return the time until the next cycle, in ms. */
static int update_event_interval(struct energy_pid *pid, double energy_j, int64_t now_ms)
{
	const double rate_min_hz = 1.0 / EVENT_INTERVAL_MAX_S;
	const double rate_max_hz = 1.0 / EVENT_INTERVAL_MIN_S;

	/* Failed capacitor reading: the energy is unknown, so wait the longest interval.
	 * Keep the integral and the smoothed energy, but do not compute a slope across
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
		pid->energy_filt_j = energy_j;
		printf("# Target fixed at %.6f J (%.0f%% of first reading %.6f J).\n",
		       pid->target_j, ENERGY_TARGET_FRACTION * 100.0, energy_j);
	}

	/* Seconds since the last cycle; 0 when there is no usable previous cycle. */
	double dt_s = pid->has_previous ? (now_ms - pid->previous_ms) / 1000.0 : 0.0;

	if (dt_s > 0.0) {
		/* Smooth the energy (first-order low-pass filter). alpha is how much the
		 * newest reading counts; a longer filter time makes it count less. */
		double alpha_energy = dt_s / (ENERGY_FILTER_S + dt_s);

		pid->energy_filt_j += alpha_energy * (energy_j - pid->energy_filt_j);

		/* Slope of the smoothed energy (W = J/s), smoothed once more. */
		double slope_w = (pid->energy_filt_j - pid->previous_filt_j) / dt_s;
		double alpha_power = dt_s / (NET_POWER_FILTER_S + dt_s);

		pid->net_power_w = isfinite(pid->net_power_w) ?
			pid->net_power_w + alpha_power * (slope_w - pid->net_power_w) :
			slope_w;
	}
	pid->previous_filt_j = pid->energy_filt_j;
	pid->previous_ms = now_ms;
	pid->has_previous = true;

	/* Positive error = more energy than the target = can afford to measure faster. */
	double error_j = pid->energy_filt_j - pid->target_j;

	/* P reacts to the error now. D reacts to how fast the energy moves; the target
	 * is constant, so d(error)/dt equals dE/dt. I adds up error x time. */
	pid->p_hz = PID_KP * error_j;
	pid->d_hz = isfinite(pid->net_power_w) ? PID_KD * pid->net_power_w : 0.0;
	double integral_step_hz = PID_KI * error_j * dt_s;

	double rate_hz = RATE_BIAS_HZ + pid->p_hz + pid->integral_hz + pid->d_hz;
	double rate_with_step_hz = rate_hz + integral_step_hz;

	/* Anti-windup: while the rate is pinned at a limit, do not let the integral keep
	 * growing past that limit. Otherwise it builds up a large "debt" and overshoots
	 * once the error changes sign. A step back toward the allowed range is always
	 * taken. */
	bool too_fast = rate_with_step_hz > rate_max_hz && integral_step_hz > 0.0;
	bool too_slow = rate_with_step_hz < rate_min_hz && integral_step_hz < 0.0;

	if (!too_fast && !too_slow) {
		pid->integral_hz += integral_step_hz;
		rate_hz = rate_with_step_hz;
	}

	if (!PID_ENABLED) {
		return BASELINE_INTERVAL_MS;
	}
	rate_hz = fmin(fmax(rate_hz, rate_min_hz), rate_max_hz);
	return (int)(1000.0 / rate_hz + 0.5);
}

/* ====================================================================================
 * One cycle's readings and the CSV row
 * ==================================================================================== */

/* Everything measured in one cycle. Values start as NAN, so a failed read shows up
 * as "nan" in the CSV instead of silently repeating an old value.
 * Status codes: 0 = ok, -11 (EAGAIN) = sensors skipped because VOUT was too low,
 * -19 (ENODEV) = I2C bus not available, -34 (ERANGE) = ADS1115 at full scale,
 * -77 (EBADMSG) = SHT40 checksum error, other negatives = ADC or I2C errors. */
struct cycle_readings {
	double time_s;
	double vst_raw, vst_pin_voltage, vst_voltage;
	double vout_raw, vout_pin_voltage, vout_voltage;
	double vin_raw, vin_mv;
	double temperature, humidity;
	int vst_status, vout_status, ads_status, sht_status;
	double cap_energy_j;
	int event_interval_ms;
};

/* Write one CSV row with a trailing newline. With include_controller false, only
 * the measurement columns are written. Returns the length, or a negative value if
 * the row did not fit. */
static int format_row(char *row, size_t size, const struct cycle_readings *r,
		      const struct energy_pid *pid, bool include_controller, bool ble_sent)
{
	int length = snprintf(row, size,
		"%.3f,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.0f,%.3f,%.2f,%.2f,%d,%d,%d,%d",
		r->time_s, r->vst_raw, r->vst_pin_voltage, r->vst_voltage,
		r->vout_raw, r->vout_pin_voltage, r->vout_voltage, r->vin_raw, r->vin_mv,
		r->temperature, r->humidity,
		r->vst_status, r->vout_status, r->ads_status, r->sht_status);

	if (length >= 0 && (size_t)length < size && include_controller) {
		length += snprintf(row + length, size - length,
			",%.6f,%.6f,%.6f,%.6f,%.2f,%d,%d,%.5f,%.5f,%.5f",
			r->cap_energy_j, pid->energy_filt_j, pid->target_j,
			pid->energy_filt_j - pid->target_j, pid->net_power_w * 1000000.0,
			r->event_interval_ms, ble_sent, pid->p_hz, pid->integral_hz, pid->d_hz);
	}
	if (length >= 0 && (size_t)length < size) {
		length += snprintf(row + length, size - length, "\n");
	}
	if (length < 0 || (size_t)length >= size) {
		return -EMSGSIZE;
	}
	return length;
}

/* ====================================================================================
 * Main loop
 * ==================================================================================== */

int main(void)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	/* The console and both onboard voltage pins must work, or nothing useful runs. */
	if (!device_is_ready(console) || !adc_is_ready_dt(&vst_input) ||
	    !adc_is_ready_dt(&vout_input)) {
		return 0;
	}

	int ret = adc_channel_setup_dt(&vst_input);

	if (ret == 0) {
		ret = adc_channel_setup_dt(&vout_input);
	}
	if (ret < 0) {
		printf("# ADC setup failed: %d\n", ret);
		return 0;
	}

	/* The I2C controller is inside the nRF, so it is ready even while the sensors
	 * themselves have no power. */
	bool i2c_ready = device_is_ready(i2c_bus);
	int ble_status = ble_output_init();
	struct energy_pid pid = {
		.target_j = NAN,
		.energy_filt_j = NAN,
		.net_power_w = NAN,
		.p_hz = NAN,
		.d_hz = NAN,
	};
	bool usb_was_open = false;

	/* Rows are built in static buffers to keep them off the small main stack. */
	static char usb_row[384];
	static char ble_row[384];

	while (1) {
		int64_t cycle_start_ms = k_uptime_get();
		struct cycle_readings r = {
			.time_s = cycle_start_ms / 1000.0,
			.vst_raw = NAN, .vst_pin_voltage = NAN, .vst_voltage = NAN,
			.vout_raw = NAN, .vout_pin_voltage = NAN, .vout_voltage = NAN,
			.vin_raw = NAN, .vin_mv = NAN,
			.temperature = NAN, .humidity = NAN,
			.ads_status = -ENODEV, .sht_status = -ENODEV,
			.cap_energy_j = NAN,
		}; // a bundle holding everything form one cycle

		/* DTR is set while a program on the laptop has the USB serial port open
		 * (for example capture_csv.py). When one opens, print the settings and the
		 * column header once. */
		uint32_t dtr = 0;

		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		bool usb_open = dtr != 0;

		if (usb_open && !usb_was_open) {
			printf("# SHT40 0x44 and ADS1115 0x48 powered from VOUT; I2C ready=%d; "
			       "BLE status=%d\n", i2c_ready, ble_status);
			printf("# PID=%d; Kp=%.4g Hz/J, Ki=%.4g Hz/(J*s), Kd=%.4g Hz/W, "
			       "bias=%.3g Hz; baseline %d ms\n",
			       PID_ENABLED, PID_KP, PID_KI, PID_KD, RATE_BIAS_HZ,
			       BASELINE_INTERVAL_MS);
			printf("# Interval %.1f-%.1f s; target fraction %.2f; energy filter %.0f s; "
			       "sensors need VOUT >= %.2f V; SHT reads/event %d\n",
			       EVENT_INTERVAL_MIN_S, EVENT_INTERVAL_MAX_S, ENERGY_TARGET_FRACTION,
			       ENERGY_FILTER_S, VOUT_MIN_FOR_SENSORS_V, SHT_READS_PER_EVENT);
			printf("%s,%s\n", measurement_columns, controller_columns);
		}
		usb_was_open = usb_open;

		/* Step 1: capacitor voltage and energy. */
		r.vst_status = read_adc(&vst_input, &r.vst_raw, &r.vst_pin_voltage);
		if (r.vst_status == 0) {
			r.vst_voltage = r.vst_raw * VST_CAL_GAIN_V_PER_COUNT + VST_CAL_OFFSET_V;
			if (r.vst_voltage > 0.0) {
				r.cap_energy_j = 0.5 * CAPACITANCE_F * r.vst_voltage * r.vst_voltage;
			}
		}

		/* Step 2: the controller picks the time until the next cycle. */
		r.event_interval_ms = update_event_interval(&pid, r.cap_energy_j, cycle_start_ms);

		/* Step 3: VOUT, then the sensors if VOUT can power them. */
		r.vout_status = read_adc(&vout_input, &r.vout_raw, &r.vout_pin_voltage);
		if (r.vout_status == 0) {
			r.vout_voltage = vout_from_adc(r.vout_raw, r.vout_pin_voltage);
		}
		if (i2c_ready) {
			if (r.vout_status == 0 && r.vout_voltage >= VOUT_MIN_FOR_SENSORS_V) {
				r.sht_status = read_environment(&r.temperature, &r.humidity);
				r.ads_status = read_vin(&r.vin_raw, &r.vin_mv);
			} else {
				r.sht_status = -EAGAIN;
				r.ads_status = -EAGAIN;
			}
		}

		/* Step 4: send the row over BLE, then print it on USB.
		 * A row received over BLE was by definition sent, so its ble_sent is 1. */
		bool ble_sent = false;

		if (ble_status == 0) {
			int length = format_row(ble_row, sizeof(ble_row), &r, &pid,
						ble_output_full_row_requested(), true);

			if (length > 0) {
				ret = ble_output_send(ble_row, length);
				if (ret == 0) {
					ble_sent = true;
				} else if (ret != -ENOTCONN) {
					printf("# BLE send failed: %d\n", ret);
				}
			}
		}
		if (usb_open) {
			int length = format_row(usb_row, sizeof(usb_row), &r, &pid, true, ble_sent);

			if (length > 0) {
				printf("%s", usb_row);
			} else {
				printf("# CSV row did not fit in the buffer.\n");
			}
		}

		/* Step 5: sleep for what is left of the interval. The interval runs from the
		 * start of one cycle to the start of the next, so the time spent measuring
		 * and sending is already included. */
		int64_t remaining_ms = r.event_interval_ms - (k_uptime_get() - cycle_start_ms);

		if (remaining_ms > 0) {
			k_msleep(remaining_ms);
		}
	}
	return 0;
}

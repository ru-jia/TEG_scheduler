/* SPDX-License-Identifier: Apache-2.0 */

#ifndef TEG_BLE_OUTPUT_H
#define TEG_BLE_OUTPUT_H

#include <stddef.h>

struct ble_csv_controller {
	double energy_j;
	double target_j;
	double net_power_uw;
	int interval_ms;
	double battery_start_mwh;
};

int ble_output_init(void);
int ble_output_send(const char *data, size_t length, const struct ble_csv_controller *controller);

#endif

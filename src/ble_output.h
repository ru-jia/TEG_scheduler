/* SPDX-License-Identifier: Apache-2.0 */

#ifndef TEG_BLE_OUTPUT_H
#define TEG_BLE_OUTPUT_H

#include <stddef.h>

int ble_output_init(void);
int ble_output_send(const char *data, size_t length);

#endif

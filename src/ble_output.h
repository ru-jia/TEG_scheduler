/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BLE output over the Nordic UART Service (NUS).
 *
 * The XIAO advertises as TEG_XIAO and accepts one connection. Each CSV row is sent
 * as notifications on the NUS TX characteristic. By default a row has only the
 * measurement columns. A receiver that writes "csv" to the NUS RX characteristic
 * gets the full row (measurement + controller columns) for the rest of that
 * connection; tools/capture_ble_csv.py --full does this.
 */

#ifndef TEG_BLE_OUTPUT_H
#define TEG_BLE_OUTPUT_H

#include <stdbool.h>
#include <stddef.h>

/* Start Bluetooth and advertising. Returns 0 or a negative error code. */
int ble_output_init(void);

/* True while a connected receiver has asked for the full row. */
bool ble_output_full_row_requested(void);

/* Send one newline-terminated CSV row. Returns 0, -ENOTCONN when no receiver is
 * subscribed, or another negative error code. */
int ble_output_send(const char *row, size_t length);

#endif

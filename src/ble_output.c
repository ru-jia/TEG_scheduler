/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_output.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

static struct bt_conn *phone;
static bool full_csv;
static double connection_battery_mwh;
static atomic_t notifications_enabled;
K_MUTEX_DEFINE(phone_lock);

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
static const struct bt_data scan_response[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
		advertising_data, ARRAY_SIZE(advertising_data),
		scan_response, ARRAY_SIZE(scan_response));
}

static void restart_advertising(struct k_work *work)
{
	ARG_UNUSED(work);
	int ret = start_advertising();

	if (ret < 0) {
		printf("# BLE advertising failed: %d\n", ret);
	}
}

K_WORK_DEFINE(advertising_work, restart_advertising);

static void connected(struct bt_conn *conn, uint8_t error)
{
	if (error) {
		return;
	}
	k_mutex_lock(&phone_lock, K_FOREVER);
	phone = bt_conn_ref(conn);
	full_csv = false;
	connection_battery_mwh = NAN;
	k_mutex_unlock(&phone_lock);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);
	atomic_clear(&notifications_enabled);
	k_mutex_lock(&phone_lock, K_FOREVER);
	if (phone == conn) {
		bt_conn_unref(phone);
		phone = NULL;
	}
	k_mutex_unlock(&phone_lock);
}

static void recycled(void)
{
	/* Restart only after the previous connection object has been released. */
	k_work_submit(&advertising_work);
}

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
};

static void subscription_changed(enum bt_nus_send_status status)
{
	atomic_set(&notifications_enabled, status == BT_NUS_SEND_STATUS_ENABLED);
}

/* A single NUS RX write of "csv" or "csv <mWh>" selects the full CSV. */
static void receive_command(struct bt_conn *conn, const uint8_t *data, uint16_t length)
{
	double battery_mwh = NAN;
	char command[32];

	if (length == 3 && memcmp(data, "csv", 3) == 0) {
		/* Use the firmware's existing battery metadata, if set. */
	} else if (length > 4 && length < sizeof(command) && memcmp(data, "csv ", 4) == 0) {
		memcpy(command, data, length);
		command[length] = '\0';
		char *end;

		battery_mwh = strtod(command + 4, &end);
		if (end != command + length || !isfinite(battery_mwh) || battery_mwh < 0.0) {
			return;
		}
	} else {
		return;
	}

	k_mutex_lock(&phone_lock, K_FOREVER);
	if (phone == conn) {
		full_csv = true;
		connection_battery_mwh = battery_mwh;
	}
	k_mutex_unlock(&phone_lock);
}

int ble_output_init(void)
{
	static struct bt_nus_cb callbacks = {
		.received = receive_command,
		.send_enabled = subscription_changed,
	};
	int ret = bt_nus_init(&callbacks);

	if (ret < 0) {
		return ret;
	}
	ret = bt_enable(NULL);
	if (ret < 0) {
		return ret;
	}
	return start_advertising();
}

int ble_output_send(const char *data, size_t length, const struct ble_csv_controller *controller)
{
	if (!atomic_get(&notifications_enabled)) {
		return -ENOTCONN;
	}
	k_mutex_lock(&phone_lock, K_FOREVER);
	struct bt_conn *conn = phone ? bt_conn_ref(phone) : NULL;
	bool send_full_csv = full_csv;
	double ble_battery_mwh = connection_battery_mwh;
	k_mutex_unlock(&phone_lock);
	if (conn == NULL) {
		return -ENOTCONN;
	}

	if (send_full_csv) {
		/* Only the main sensing loop calls this function. */
		static char full_row[448];
		double battery_mwh = isfinite(ble_battery_mwh) ? ble_battery_mwh :
				     controller->battery_start_mwh;

		/* A complete row received over BLE implies ble_sent=1. */
		int size = snprintf(full_row, sizeof(full_row),
				    "%.*s,%.6f,%.6f,%.6f,%.6f,%.2f,%d,1,%.9g,%.5f,%.5f,%.5f\n",
				    (int)length - 1, data, controller->energy_j,
				    controller->energy_filt_j, controller->target_j,
				    controller->energy_filt_j - controller->target_j,
				    controller->net_power_uw, controller->interval_ms, battery_mwh,
				    controller->p_hz, controller->i_hz, controller->d_hz);
		if (size < 0 || (size_t)size >= sizeof(full_row)) {
			bt_conn_unref(conn);
			return -EMSGSIZE;
		}
		data = full_row;
		length = size;
	}

	/* Fit notifications to the negotiated MTU, including the 20-byte default. */
	size_t payload_size = bt_nus_get_mtu(conn);
	int ret = 0;

	while (length > 0) {
		size_t chunk_size = MIN(length, payload_size);

		ret = bt_nus_send(conn, (const uint8_t *)data, chunk_size);
		if (ret < 0) {
			break;
		}
		data += chunk_size;
		length -= chunk_size;
	}
	bt_conn_unref(conn);
	return ret;
}

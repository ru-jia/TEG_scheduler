/* SPDX-License-Identifier: Apache-2.0 */

#include "ble_output.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/* The connected receiver, or NULL. phone and full_row are shared with the Bluetooth
 * callbacks, so they are only touched while holding phone_lock. */
static struct bt_conn *phone;
static bool full_row;
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
	full_row = false; /* Each new connection starts with the short row. */
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
	/* Advertise again only after the previous connection has been released. */
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

/* A write of exactly "csv" to NUS RX switches this connection to the full row. */
static void receive_command(struct bt_conn *conn, const uint8_t *data, uint16_t length)
{
	if (length != 3 || memcmp(data, "csv", 3) != 0) {
		return;
	}
	k_mutex_lock(&phone_lock, K_FOREVER);
	if (phone == conn) {
		full_row = true;
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

bool ble_output_full_row_requested(void)
{
	k_mutex_lock(&phone_lock, K_FOREVER);
	bool requested = phone != NULL && full_row;
	k_mutex_unlock(&phone_lock);
	return requested;
}

int ble_output_send(const char *row, size_t length)
{
	if (!atomic_get(&notifications_enabled)) {
		return -ENOTCONN;
	}
	k_mutex_lock(&phone_lock, K_FOREVER);
	struct bt_conn *conn = phone ? bt_conn_ref(phone) : NULL;
	k_mutex_unlock(&phone_lock);
	if (conn == NULL) {
		return -ENOTCONN;
	}

	/* A notification can carry at most one MTU of data (20 bytes by default), so
	 * longer rows go out in pieces. The receiver joins them up to the newline. */
	size_t chunk_limit = bt_nus_get_mtu(conn);
	int ret = 0;

	while (length > 0) {
		size_t chunk_size = MIN(length, chunk_limit);

		ret = bt_nus_send(conn, (const uint8_t *)row, chunk_size);
		if (ret < 0) {
			break;
		}
		row += chunk_size;
		length -= chunk_size;
	}
	bt_conn_unref(conn);
	return ret;
}

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <settings/settings.h>

#include "ble_conn_mgr.h"
#include "ble_codec.h"
#include "cJSON.h"
#include "ble.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_conn_mgr, CONFIG_LOG_DEFAULT_LEVEL);

static int num_connected;
static struct ble_device_conn connected_ble_devices[CONFIG_BT_MAX_CONN];

static struct desired_conn desired_connections[CONFIG_BT_MAX_CONN];

#define CONN_MGR_STACK_SIZE 3072
#define CONN_MGR_PRIORITY 1


static void process_connection(int i)
{
	int err;
	struct ble_device_conn *dev = &connected_ble_devices[i];

	if (dev->free) {
		return;
	}
	/* Add devices to whitelist */
	if (!dev->added_to_whitelist) {
		ble_add_to_whitelist(dev->addr);
		dev->added_to_whitelist = true;
		update_shadow(dev->addr, true, false);
		dev->shadow_updated = true;
		LOG_INF("Device added to whitelist.");
	}

	/* Connected. Do discovering if not discovered or currently
	 * discovering.
	 */
	if (dev->connected && !dev->discovered && !dev->discovering) {

		err = ble_discover(dev->addr);

		if (!err) {
			LOG_DBG("ble_discover(%s) failed: %d",
				log_strdup(dev->addr), err);
		}
	}

	/* Discovering done. Encode and send. */
	if (dev->connected && dev->encode_discovered) {
		/* I don't know why this is needed in a work thread.
		 * TODO: FIX
		 */
		uint32_t lock = irq_lock();

		dev->encode_discovered = false;
		device_discovery_send(&connected_ble_devices[i]);

		irq_unlock(lock);
	}
}

void connection_manager(int unused1, int unused2, int unused3)
{
	int i;

	ble_conn_mgr_init();
	while (1) {

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			/* Manager is busy. Do nothing. */
			if (connected_ble_devices[i].discovering) {
				LOG_DBG("Connection work busy.");
				goto end;
			}
		}

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			process_connection(i);
		}

end:
		k_sleep(K_MSEC(500));
	}
}

K_THREAD_DEFINE(conn_mgr_thread, CONN_MGR_STACK_SIZE,
		connection_manager, NULL, NULL, NULL,
		CONN_MGR_PRIORITY, 0, 0);

static void ble_conn_mgr_conn_reset(struct ble_device_conn
					*dev)
{
	if (!dev->free) {
		if (num_connected) {
			num_connected--;
		}
		dev->free = true;
	}
	dev->connected = false;
	dev->disconnect = false;
	dev->discovering = false;
	dev->discovered = false;
	dev->added_to_whitelist = false;
	dev->encode_discovered = false;
	dev->shadow_updated = false;
	dev->num_pairs = 0;
	LOG_INF("Conn Removed to %s", log_strdup(dev->addr));
}

void ble_conn_mgr_update_connections(void)
{
	int i;

	for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct ble_device_conn *dev = &connected_ble_devices[i];

		if (dev->connected || dev->added_to_whitelist) {
			dev->disconnect = true;

			for (int j = 0; j < CONFIG_BT_MAX_CONN; j++) {
				if (desired_connections[j].active) {
					if (!strcmp(desired_connections[j].addr,
						    dev->addr)) {
						/* If in the list then don't
						 * disconnect.
						 */
						dev->disconnect = false;
						break;
					}
				}
			}

			if (dev->disconnect) {
				LOG_INF("cloud: disconnect device %s",
					log_strdup(dev->addr));
				ble_remove_from_whitelist(dev->addr);
				disconnect_device_by_addr(dev->addr);
				ble_conn_mgr_conn_reset(dev);
				if (IS_ENABLED(CONFIG_SETTINGS)) {
					LOG_INF("Saving settings");
					settings_save();
				}
			}
		}
	}
}

void ble_conn_mgr_change_desired(char *addr, uint8_t index,
				 bool active, bool manual)
{
	if (index <= CONFIG_BT_MAX_CONN) {
		strncpy(desired_connections[index].addr, addr, DEVICE_ADDR_LEN);
		desired_connections[index].active = active;
		desired_connections[index].manual = manual;

		LOG_INF("Desired Connection %s: %s %s",
			active ? "Added" : "Removed",
			log_strdup(addr),
			manual ? "(manual)" : "");
	}
}

struct desired_conn *get_desired_array(int *array_size)
{
	if (array_size == NULL) {
		return NULL;
	}
	*array_size = ARRAY_SIZE(desired_connections);
	return desired_connections;
}

static int find_desired_connection(char *addr,
				   struct desired_conn **pcon)
{
	struct desired_conn *con;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		con = &desired_connections[i];
		if ((strncmp(addr, con->addr, sizeof(con->addr)) == 0) &&
		    (addr[0] != '\0')) {
			if (pcon) {
				*pcon = con;
			}
			return i;
		}
	}
	if (pcon) {
		*pcon = NULL;
	}
	return -EINVAL;
}

void ble_conn_mgr_update_desired(char *addr, uint8_t index)
{
	ble_conn_mgr_change_desired(addr, index, true, false);
}

int ble_conn_mgr_add_desired(char *addr, bool manual)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!desired_connections[i].active) {
			ble_conn_mgr_change_desired(addr, i, true, manual);
			return 0;
		}
	}
	return -EINVAL;
}

int ble_conn_mgr_rem_desired(char *addr, bool manual)
{
	if (addr == NULL) {
		return -EINVAL;
	}

	int i = find_desired_connection(addr, NULL);

	if (i >= 0) {
		ble_conn_mgr_change_desired(addr, i, false, manual);
		return 0;
	}
	return -EINVAL;
}

void ble_conn_mgr_clear_desired(bool all)
{
	LOG_INF("Desired Connections Cleared.");

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!desired_connections[i].manual || all) {
			desired_connections[i].active = false;
		}
	}
}

bool ble_conn_mgr_enabled(char *addr)
{
	struct desired_conn *con;

	if (addr == NULL) {
		return false;
	}

	find_desired_connection(addr, &con);

	if (con != NULL) {
		return !con->manual;
	}

	return true;
}

int ble_conn_mgr_generate_path(struct ble_device_conn *conn_ptr, uint16_t handle,
				char *path, bool ccc)
{
	int err = 0;
	char path_str[BT_MAX_PATH_LEN];
	char service_uuid[BT_UUID_STR_LEN];
	char ccc_uuid[BT_UUID_STR_LEN];
	char chrc_uuid[BT_UUID_STR_LEN];
	uint8_t path_depth = 0;
	bool found = false;

	path_str[0] = '\0';

	LOG_DBG("Num Pairs: %d", conn_ptr->num_pairs);

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		struct uuid_handle_pair *uuid_handle =
			&conn_ptr->uuid_handle_pairs[i];

		if (handle != uuid_handle->handle) {
			continue;
		}

		found = true;
		path_depth = uuid_handle->path_depth;
		LOG_DBG("Path Depth %d", path_depth);

		bt_uuid_get_str(&uuid_handle->uuid_128.uuid,
				chrc_uuid, BT_MAX_UUID_LEN);
		uuid_handle++;
		bt_uuid_get_str(&uuid_handle->uuid_128.uuid,
				ccc_uuid, BT_MAX_UUID_LEN);

		for (int j = i; j >= 0; j--) {
			uuid_handle = &conn_ptr->uuid_handle_pairs[j];
			if (uuid_handle->is_service) {
				bt_uuid_get_str(&uuid_handle->uuid_128.uuid,
						service_uuid, BT_MAX_UUID_LEN);
				LOG_DBG("service uuid in path %s",
					log_strdup(service_uuid));
				break;
			}
		}

		snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s",
			 service_uuid, chrc_uuid);

		if (ccc) {
			snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s/%s",
				 service_uuid, chrc_uuid, ccc_uuid);
		}
	}

	if (!found) {
		LOG_ERR("path not generated; handle %u not found for addr %s",
			handle, log_strdup(conn_ptr->addr));
		err = -ENXIO;
	}

	bt_to_upper(path_str, strlen(path_str));
	memset(path, 0, BT_MAX_PATH_LEN);
	memcpy(path, path_str, strlen(path_str));

	LOG_DBG("Generated Path: %s", log_strdup(path_str));
	return err;
}

int ble_conn_mgr_add_conn(char *addr)
{
	int err = 0;
	struct ble_device_conn *connected_ble_ptr;

	/* Check if already added */
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_ble_devices[i].free == false) {
			if (!strcmp(addr, connected_ble_devices[i].addr)) {
				LOG_DBG("Connection already exsists");
				return 1;
			}
		}
	}

	err = ble_conn_mgr_get_free_conn(&connected_ble_ptr);

	if (err) {
		LOG_ERR("No free connections");
		return err;
	}

	memcpy(connected_ble_ptr->addr, addr, DEVICE_ADDR_LEN);
	connected_ble_ptr->free = false;
	num_connected++;
	LOG_INF("BLE conn to %s added to manager", log_strdup(addr));
	return err;
}

int ble_conn_set_connected(char *addr, bool connected)
{
	int err = 0;
	struct ble_device_conn *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Conn %s not found", log_strdup(addr));
		return err;
	}

	if (connected) {
		connected_ble_ptr->connected = true;
	} else {
		connected_ble_ptr->connected = false;
	}
	LOG_INF("Conn updated: connected=%u", connected);
	return err;
}

int ble_conn_set_disconnected(char *addr)
{
	int err = 0;
	struct ble_device_conn *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Can't find conn to disconnect");
		return err;
	}

	connected_ble_ptr->connected = false;
	connected_ble_ptr->shadow_updated = false;
	LOG_INF("Conn %s Disconnected", log_strdup(addr));
	return err;
}

int ble_conn_mgr_rediscover(char *addr)
{
	int err = 0;
	struct ble_device_conn *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Can't find conn to disconnect");
		return err;
	}

	if (!connected_ble_ptr->discovering) {
		connected_ble_ptr->discovered = false;
		connected_ble_ptr->num_pairs = 0;
	}

	return err;
}

int ble_conn_mgr_remove_conn(char *addr)
{
	int err = 0;
	struct ble_device_conn *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);
	if (err) {
		LOG_ERR("Can't find conn %s to remove", log_strdup(addr));
		return err;
	}

	ble_conn_mgr_conn_reset(connected_ble_ptr);
	return err;
}


int ble_conn_mgr_get_free_conn(struct ble_device_conn **conn_ptr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_ble_devices[i].free == true) {
			*conn_ptr = &connected_ble_devices[i];
			LOG_DBG("Found Free connection: %d", i);
			return 0;
		}
	}
	return 1;
}


int ble_conn_mgr_get_conn_by_addr(char *addr, struct ble_device_conn **conn_ptr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!strcmp(addr, connected_ble_devices[i].addr)) {
			*conn_ptr = &connected_ble_devices[i];
			LOG_DBG("Conn Found");
			return 0;
		}
	}
	LOG_ERR("No Conn Found for addr %s", log_strdup(addr));
	return 1;

}

int ble_conn_mgr_set_subscribed(uint16_t handle, uint8_t sub_index,
				 struct ble_device_conn *conn_ptr)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pairs[i].handle) {
			conn_ptr->uuid_handle_pairs[i].sub_enabled = true;
			conn_ptr->uuid_handle_pairs[i].sub_index = sub_index;
			return 0;
		}
	}

	return 1;
}


int ble_conn_mgr_remove_subscribed(uint16_t handle,
				    struct ble_device_conn *conn_ptr)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pairs[i].handle) {
			conn_ptr->uuid_handle_pairs[i].sub_enabled = false;
			return 0;
		}
	}
	return 1;
}


int ble_conn_mgr_get_subscribed(uint16_t handle, struct ble_device_conn *conn_ptr,
				 bool *status, uint8_t *sub_index)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pairs[i].handle) {
			*status = conn_ptr->uuid_handle_pairs[i].sub_enabled;
			*sub_index = conn_ptr->uuid_handle_pairs[i].sub_index;
			return 0;
		}
	}

	return 1;
}

int ble_conn_mgr_get_uuid_by_handle(uint16_t handle, char *uuid,
				     struct ble_device_conn *conn_ptr)
{
	char uuid_str[BT_UUID_STR_LEN];

	memset(uuid, 0, BT_UUID_STR_LEN);

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		struct uuid_handle_pair *uuid_handle = 
			&conn_ptr->uuid_handle_pairs[i];

		if (handle == uuid_handle->handle) {
			bt_uuid_get_str(&uuid_handle->uuid_128.uuid, uuid_str,
					BT_MAX_UUID_LEN);
			bt_to_upper(uuid_str, strlen(uuid_str));
			memcpy(uuid, uuid_str, strlen(uuid_str));
			LOG_DBG("Found UUID: %s For Handle: %d",
				log_strdup(uuid_str),
				handle);
			return 0;
		}
	}

	LOG_ERR("Handle %u on addr %s not found; num pairs: %d", handle,
		log_strdup(conn_ptr->addr),
		(int)conn_ptr->num_pairs);
	return 1;
}


int ble_conn_mgr_get_handle_by_uuid(uint16_t *handle, char *uuid,
				     struct ble_device_conn *conn_ptr)
{
	char str[BT_UUID_STR_LEN];

	LOG_DBG("Num Pairs: %d", conn_ptr->num_pairs);

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		struct uuid_handle_pair *uuid_handle =
			&conn_ptr->uuid_handle_pairs[i];

		bt_uuid_get_str(&uuid_handle->uuid_16.uuid, str, sizeof(str));
		bt_to_upper(str, strlen(str));
		LOG_DBG("UUID IN: %s UUID FOUND: %s", log_strdup(uuid),
			log_strdup(str));

		if (!strcmp(uuid, str)) {
			*handle = uuid_handle->handle;
			LOG_DBG("16 Bit UUID Found");
			return 0;
		}

		bt_uuid_get_str(&uuid_handle->uuid_128.uuid, str, sizeof(str));
		bt_to_upper(str, strlen(str));
		LOG_DBG("UUID IN: %s UUID FOUND: %s", log_strdup(uuid),
			log_strdup(str));
		if (!strcmp(uuid, str)) {
			*handle = uuid_handle->handle;
			LOG_DBG("128 Bit UUID Found");
			return 0;
		}
	}

	LOG_ERR("Handle Not Found");
	return 1;
}

int ble_conn_mgr_add_uuid_pair(const struct bt_uuid *uuid, uint16_t handle,
				uint8_t path_depth, uint8_t properties,
				uint8_t attr_type,
				struct ble_device_conn *conn_ptr,
				bool is_service)
{
	int err = 0;
	char str[BT_UUID_STR_LEN];

	if (!conn_ptr) {
		LOG_ERR("no connection ptr!");
		return 1;
	}

	if (conn_ptr->num_pairs >= MAX_UUID_PAIRS) {
		LOG_ERR("Max uuid pair limit reached on %s",
			log_strdup(conn_ptr->addr));
		return 1;
	}

	LOG_DBG("Handle Added: %d", handle);

	if (!uuid) {
		return 0;
	}

	struct uuid_handle_pair *uuid_handle;

	uuid_handle = &conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs];

	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		memcpy(&uuid_handle->uuid_16, BT_UUID_16(uuid),
		       sizeof(struct bt_uuid_16));
		uuid_handle->uuid_type = BT_UUID_TYPE_16;
		bt_uuid_get_str(&uuid_handle->uuid_16.uuid, str, sizeof(str));

		LOG_DBG("\tCONN MGR Characteristic: 0x%s", log_strdup(str));
		break;
	case BT_UUID_TYPE_128:
		memcpy(&uuid_handle->uuid_128, BT_UUID_128(uuid),
		       sizeof(struct bt_uuid_128));
		uuid_handle->uuid_type = BT_UUID_TYPE_128;
		bt_uuid_get_str(&uuid_handle->uuid_128.uuid, str, sizeof(str));

		LOG_DBG("\tCONN MGR Characteristic: 0x%s", log_strdup(str));
		break;
	default:
		return 0;
	}

	conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs].properties = properties;
	conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs].attr_type = attr_type;
	conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs].path_depth = path_depth;
	conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs].is_service = is_service;
	conn_ptr->uuid_handle_pairs[conn_ptr->num_pairs].handle = handle;
	LOG_INF("%d. Handle Added: %d", conn_ptr->num_pairs, handle);

	conn_ptr->num_pairs++;

	return err;
}

struct ble_device_conn *get_connected_device(unsigned int i)
{
	if (i < CONFIG_BT_MAX_CONN) {
		if (!connected_ble_devices[i].free) {
			return &connected_ble_devices[i];
		}
	}
	return NULL;
}

int get_num_connected(void)
{
	return num_connected;
}

void ble_conn_mgr_init()
{
	num_connected = 0;
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		connected_ble_devices[i].connected = false;
		connected_ble_devices[i].discovering = false;
		connected_ble_devices[i].free = true;
		connected_ble_devices[i].discovered = false;
		connected_ble_devices[i].encode_discovered = false;
		connected_ble_devices[i].added_to_whitelist = false;
		connected_ble_devices[i].shadow_updated = false;
		connected_ble_devices[i].disconnect = false;
	}
}

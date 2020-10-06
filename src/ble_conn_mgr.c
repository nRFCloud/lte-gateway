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

static connected_ble_devices connected_ble_device[CONFIG_BT_MAX_CONN];

typedef struct {
	char addr[DEVICE_ADDR_LEN];
	bool active;
} desired_conns;

static desired_conns desired_connection[CONFIG_BT_MAX_CONN];

#define CONN_MGR_STACK_SIZE 3072
#define CONN_MGR_PRIORITY 1


static void process_connection(int i)
{
	int err;
	connected_ble_devices *dev = &connected_ble_device[i];

	if (dev->free) {
		return;
	}
	/* Add devices to whitelist */
	if (!dev->added_to_whitelist) {
		ble_add_to_whitelist(dev->addr);
		dev->added_to_whitelist = true;
		update_shadow(dev->addr, true, false);
		LOG_INF("Device added to whitelist.");
	}

	/* Not connected. Update the shadow. */
	if (dev->connected && !dev->shadow_updated) {
		/* TODO: remove? Maybe not the right spot to send this. IDK. */
		/* device_shadow_data_encode(dev->addr, false, true); */
		/* dev->shadow_updated = true; */
	}

	/* Connected. Do discovering if not discovered or currently
	 * discovering.
	 */
	if (dev->connected && !dev->discovered && !dev->discovering) {

		err = ble_discover(dev->addr);

		if (!err) {
			dev->discovered = true;
		}
	}

	/* Discoverying done. Encode and send. */
	if (dev->connected && dev->encode_discovered) {
		/* I don't know why this is needed in a work thread.
		 * TODO: FIX
		 */
		uint32_t lock = irq_lock();

		dev->encode_discovered = false;
		device_discovery_send(&connected_ble_device[i]);

		irq_unlock(lock);

		/* Maybe not the right spot to send this. IDK. */
		/* device_shadow_data_encode(dev->addr, false, true); */
		/* update_shadow(dev->addr, false, true); */
	}
}

void connection_manager(int unused1, int unused2, int unused3)
{
	int i;

	ble_conn_mgr_init();
	while (1) {

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			/* Manager is busy. Do nothing. */
			if (connected_ble_device[i].discovering) {
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


static void ble_conn_mgr_conn_reset(uint8_t conn)
{
	connected_ble_device[conn].free = true;
	connected_ble_device[conn].added_to_whitelist = false;
	connected_ble_device[conn].connected = false;
	connected_ble_device[conn].discovered = false;
	connected_ble_device[conn].encode_discovered = false;
	connected_ble_device[conn].disconnect = false;
}

void ble_conn_mgr_update_connections()
{
	int i;

	for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		connected_ble_devices *dev = &connected_ble_device[i];

		if (dev->connected) {
			dev->disconnect = true;

			for (int j = 0; j < CONFIG_BT_MAX_CONN; j++) {
				if (desired_connection[j].active) {
					if (!strcmp(desired_connection[j].addr,
						    dev->addr)) {
						/* If in the list then don't
						 * disconnect.
						 */
						dev->disconnect = false;
					}
				}
			}

			if (dev->disconnect) {
				LOG_INF("cloud: disconnect device");
				ble_remove_from_whitelist(dev->addr);
				disconnect_device_by_addr(dev->addr);
				ble_conn_mgr_conn_reset(i);
				if (IS_ENABLED(CONFIG_SETTINGS)) {
					LOG_INF("Saving settings");
					settings_save();
				}
			}
		}
	}
}


void ble_conn_mgr_update_desired(char *addr, uint8_t index)
{
	if (index <= CONFIG_BT_MAX_CONN) {
		memcpy(desired_connection[index].addr, addr, strlen(addr));
		desired_connection[index].active = true;

		LOG_INF("Desired Connection Added: %s",
			desired_connection[index].addr);
	}
}

void ble_conn_mgr_clear_desired()
{
	LOG_INF("Desired Connections Cleared.");

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		desired_connection[i].active = false;
	}
}

int ble_conn_mgr_generate_path(connected_ble_devices *conn_ptr, uint16_t handle,
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
		uuid_handle_pairs *uuid_handle = &conn_ptr->uuid_handle_pair[i];

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
			uuid_handle = &conn_ptr->uuid_handle_pair[j];
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
	connected_ble_devices *connected_ble_ptr;

	/* Check if already added */
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_ble_device[i].free == false) {
			if (!strcmp(addr, connected_ble_device[i].addr)) {
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
	LOG_INF("Ble conn added to manager");
	return err;
}

int ble_conn_set_connected(char *addr, bool connected)
{
	int err = 0;
	connected_ble_devices *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Conn not found");
		return err;
	}

	if (connected) {
		connected_ble_ptr->connected = true;
	} else {
		connected_ble_ptr->connected = false;
	}
	LOG_INF("Conn updated");
	return err;
}

int ble_conn_set_disconnected(char *addr)
{
	int err = 0;
	connected_ble_devices *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Can't find conn to disconnect");
		return err;
	}

	connected_ble_ptr->connected = false;
	/* Should we need to discover again on reconnect? */
	connected_ble_ptr->discovered = false;
	connected_ble_ptr->num_pairs = 0;
	connected_ble_ptr->shadow_updated = false;
	/* connected_ble_ptr->added_to_whitelist = false; */
	LOG_INF("Conn Disconnected");
	return err;
}

int ble_conn_mgr_rediscover(char *addr)
{
	int err = 0;
	connected_ble_devices *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Can't find conn to disconnect");
		return err;
	}

	if (!connected_ble_ptr->discovering) {
		/* Should we need to discover again on reconnect? */
		connected_ble_ptr->discovered = false;
		connected_ble_ptr->num_pairs = 0;
	}

	return err;
}

int ble_conn_mgr_remove_conn(char *addr)
{
	int err = 0;
	connected_ble_devices *connected_ble_ptr;

	err = ble_conn_mgr_get_conn_by_addr(addr, &connected_ble_ptr);

	if (err) {
		LOG_ERR("Can't find conn to remove");
		return err;
	}

	memset(connected_ble_ptr, 0, sizeof(connected_ble_devices));

	connected_ble_ptr->free = true;
	connected_ble_ptr->connected = false;
	connected_ble_ptr->discovered = false;
	LOG_INF("Conn Removed");
	return err;
}


int ble_conn_mgr_get_free_conn(connected_ble_devices **conn_ptr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_ble_device[i].free == true) {
			*conn_ptr = &connected_ble_device[i];
			LOG_DBG("Found Free connection: %d", i);
			return 0;
		}
	}
	return 1;
}


int ble_conn_mgr_get_conn_by_addr(char *addr, connected_ble_devices **conn_ptr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!strcmp(addr, connected_ble_device[i].addr)) {
			*conn_ptr = &connected_ble_device[i];
			LOG_DBG("Conn Found");
			return 0;
		}
	}
	LOG_ERR("No Conn Found");
	return 1;

}

int ble_conn_mgr_set_subscribed(uint16_t handle, uint8_t sub_index,
				 connected_ble_devices *conn_ptr)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pair[i].handle) {
			conn_ptr->uuid_handle_pair[i].sub_enabled = true;
			conn_ptr->uuid_handle_pair[i].sub_index = sub_index;
			return 0;
		}
	}

	return 1;
}


int ble_conn_mgr_remove_subscribed(uint16_t handle,
				    connected_ble_devices *conn_ptr)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pair[i].handle) {
			conn_ptr->uuid_handle_pair[i].sub_enabled = false;
			return 0;
		}
	}
	return 1;
}


int ble_conn_mgr_get_subscribed(uint16_t handle, connected_ble_devices *conn_ptr,
				 bool *status, uint8_t *sub_index)
{
	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		if (handle == conn_ptr->uuid_handle_pair[i].handle) {
			*status = conn_ptr->uuid_handle_pair[i].sub_enabled;
			*sub_index = conn_ptr->uuid_handle_pair[i].sub_index;
			return 0;
		}
	}

	return 1;
}

int ble_conn_mgr_get_uuid_by_handle(uint16_t handle, char *uuid,
				     connected_ble_devices *conn_ptr)
{
	char uuid_str[BT_UUID_STR_LEN];

	memset(uuid, 0, BT_UUID_STR_LEN);

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		uuid_handle_pairs *uuid_handle = &conn_ptr->uuid_handle_pair[i];

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

	LOG_ERR("Handle Not Found");
	return 1;
}


int ble_conn_mgr_get_handle_by_uuid(uint16_t *handle, char *uuid,
				     connected_ble_devices *conn_ptr)
{
	char str[BT_UUID_STR_LEN];

	LOG_DBG("Num Pairs: %d", conn_ptr->num_pairs);

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		uuid_handle_pairs *uuid_handle = &conn_ptr->uuid_handle_pair[i];

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
				connected_ble_devices *conn_ptr,
				bool is_service)
{
	int err = 0;
	char str[BT_UUID_STR_LEN];

	if (!conn_ptr) {
		LOG_ERR("no connection ptr!");
		return 1;
	}

	if (conn_ptr->num_pairs >= MAX_UUID_PAIRS) {
		LOG_ERR("Max uuid pair limit reached");
		return 1;
	}

	LOG_INF("Handle Added: %d", handle);

	if (!uuid) {
		return 0;
	}

	uuid_handle_pairs *uuid_handle;

	uuid_handle = &conn_ptr->uuid_handle_pair[conn_ptr->num_pairs];

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

	conn_ptr->uuid_handle_pair[conn_ptr->num_pairs].properties = properties;
	conn_ptr->uuid_handle_pair[conn_ptr->num_pairs].attr_type = attr_type;
	conn_ptr->uuid_handle_pair[conn_ptr->num_pairs].path_depth = path_depth;
	conn_ptr->uuid_handle_pair[conn_ptr->num_pairs].is_service = is_service;
	conn_ptr->uuid_handle_pair[conn_ptr->num_pairs].handle = handle;

	conn_ptr->num_pairs++;

	return err;
}

void ble_conn_mgr_init()
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		connected_ble_device[i].free = true;
		connected_ble_device[i].added_to_whitelist = false;
		connected_ble_device[i].connected = false;
		connected_ble_device[i].discovered = false;
		connected_ble_device[i].encode_discovered = false;
		connected_ble_device[i].disconnect = false;
	}
}

#ifndef _BLE_CONN_MGR_H_
#define _BLE_CONN_MGR_H_

#include "ble.h"
#include <bluetooth/uuid.h>

#define MAX_UUID_PAIRS 68
#define DEVICE_ADDR_LEN 18

#define BT_MAX_UUID_LEN 37
#define BT_MAX_PATH_LEN 111

#define SMALL_UUID_HANDLE_PAIR_SIZE (sizeof(struct uuid_handle_pair) - \
				     sizeof(struct bt_uuid_128) + \
				     sizeof(struct bt_uuid_16))
#define LARGE_UUID_HANDLE_PAIR_SIZE (sizeof(struct uuid_handle_pair))

struct uuid_handle_pair {
	uint16_t handle;
	uint8_t uuid_type;
	uint8_t attr_type;
	uint8_t path_depth;
	uint8_t properties;
	bool is_service;
	bool sub_enabled;
	uint8_t sub_index;
	union {
		struct bt_uuid_16 uuid_16;
		struct bt_uuid_128 uuid_128;
	};
};

struct ble_device_conn {
	char addr[DEVICE_ADDR_LEN];
	struct uuid_handle_pair *uuid_handle_pairs[MAX_UUID_PAIRS];
	uint8_t num_pairs;
	bool connected;
	bool discovering;
	bool free;
	bool discovered;
	bool encode_discovered;
	bool added_to_allowlist;
	bool shadow_updated;
	bool disconnect;
};

struct desired_conn {
	char addr[DEVICE_ADDR_LEN];
	bool active;
	bool manual;
};

int ble_conn_mgr_add_conn(char *addr);
int ble_conn_mgr_generate_path(struct ble_device_conn *conn_ptr,
			       uint16_t handle,
				char *path, bool ccc);
int ble_conn_mgr_remove_conn(char *addr);
int ble_conn_mgr_get_free_conn(struct ble_device_conn **conn_ptr);
int ble_conn_mgr_get_conn_by_addr(char *addr,
				  struct ble_device_conn **conn_ptr);
int ble_conn_mgr_add_uuid_pair(const struct bt_uuid *uuid, uint16_t handle,
			       uint8_t path_depth, uint8_t properties,
			       uint8_t attr_type,
			       struct ble_device_conn *conn_ptr,
			       bool is_service);
int ble_conn_mgr_get_uuid_by_handle(uint16_t handle, char *uuid,
				    struct ble_device_conn *conn_ptr);
int ble_conn_mgr_get_handle_by_uuid(uint16_t *handle, char *uuid,
				    struct ble_device_conn *conn_ptr);
void ble_conn_mgr_init();
int ble_conn_set_connected(char *addr, bool connected);
int ble_conn_set_disconnected(char *addr);
int ble_conn_mgr_set_subscribed(uint16_t handle, uint8_t sub_index,
				struct ble_device_conn *conn_ptr);
int ble_conn_mgr_remove_subscribed(uint16_t handle,
				   struct ble_device_conn *conn_ptr);
int ble_conn_mgr_get_subscribed(uint16_t handle,
				struct ble_device_conn *conn_ptr,
				bool *status, uint8_t *sub_index);
void ble_conn_mgr_update_desired(char *addr, uint8_t index);
int ble_conn_mgr_add_desired(char *addr, bool manual);
int ble_conn_mgr_rem_desired(char *addr, bool manual);
void ble_conn_mgr_clear_desired(bool all);
bool ble_conn_mgr_enabled(char *addr);
void ble_conn_mgr_update_connections(void);
int ble_conn_mgr_rediscover(char *addr);
struct ble_device_conn *get_connected_device(unsigned int i);
int get_num_connected(void);
struct desired_conn *get_desired_array(int *array_size);
bool ble_conn_mgr_is_addr_connected(char *addr);
void ble_conn_mgr_print_mem(void);
int ble_conn_mgr_find_related_addr(char *old_addr, char *new_addr, int len);
int ble_conn_mgr_force_dfu_rediscover(char *addr);

#endif

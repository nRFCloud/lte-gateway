#ifndef _BLE_CONN_MGR_H_
#define _BLE_CONN_MGR_H_

#include "ble.h"
#include <bluetooth/uuid.h>

#define MAX_UUID_PAIRS 100
#define DEVICE_ADDR_LEN 18
#define DEVICE_ADDR_TYPE_LEN 7

#define BT_MAX_UUID_LEN 37
#define BT_MAX_PATH_LEN 111

typedef struct uuid_handle_pairs
{
    u16_t handle;
    u8_t uuid_type;
    u8_t attr_type;
    u8_t path_depth;
    u8_t properties;
    bool is_service;
    bool sub_enabled;
    u8_t sub_index;
    union{
      struct bt_uuid_16 uuid_16;
      struct bt_uuid_128 uuid_128;
    };

} uuid_handle_pairs;

typedef struct connected_ble_devices
{
  char addr[DEVICE_ADDR_LEN];
  char addr_type[DEVICE_ADDR_TYPE_LEN];
  uuid_handle_pairs uuid_handle_pair[MAX_UUID_PAIRS];
  u8_t num_pairs;
  bool connected;
  bool discovering;
  bool free;
  bool discovered;
  bool encode_discovered;
  bool added_to_whitelist;
  bool shadow_updated;
  bool disconnect;

} connected_ble_devices;


u8_t ble_conn_mgr_add_conn(char* addr, char* addr_type);
void ble_conn_mgr_generate_path(connected_ble_devices* conn_ptr, u16_t handle, char* path, bool ccc);
u8_t ble_conn_mgr_remove_conn(char* addr);
u8_t ble_conn_mgr_get_free_conn(connected_ble_devices** conn_ptr);
u8_t ble_conn_mgr_get_conn_by_addr(char* addr, connected_ble_devices** conn_ptr);
u8_t ble_conn_mgr_add_uuid_pair(struct bt_uuid *uuid, u16_t handle, u8_t path_depth, u8_t properties, u8_t attr_type, connected_ble_devices* conn_ptr, bool is_service);
u8_t ble_conn_mgr_get_uuid_by_handle(u16_t handle, char* uuid , connected_ble_devices* conn_ptr);
u8_t ble_conn_mgr_get_handle_by_uuid(u16_t* handle, char* uuid , connected_ble_devices* conn_ptr);
void ble_conn_mgr_init();
u8_t ble_conn_set_connected(char* addr, bool connected);
u8_t ble_conn_set_disconnected(char* addr);
u8_t ble_conn_mgr_set_subscribed(u16_t handle, u8_t sub_index, connected_ble_devices* conn_ptr);
u8_t ble_conn_mgr_remove_subscribed(u16_t handle, connected_ble_devices* conn_ptr);
u8_t ble_conn_mgr_get_subscribed(u16_t handle, connected_ble_devices* conn_ptr, bool* status, u8_t* sub_index);
void ble_conn_mgr_clear_desired();
void ble_conn_mgr_update_connections();
u8_t ble_conn_mgr_rediscover(char* addr);
#endif
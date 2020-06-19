#ifndef BLE_CODEC_H_
#define BLE_CODEC_H_

//#include "ble.h"
#include "cJSON.h"
#include "ble_conn_mgr.h"

u8_t device_found_encode(u8_t num_devices_found);
u8_t device_connect_result_encode(char* ble_address, bool conn_status);
u8_t device_discovery_result_encode(char* ble_address, bool conn_status, char *discovered_json);
u8_t device_value_changed_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length);
u8_t device_chrc_read_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length);
u8_t create_device_wrapper(char* ble_address, bool conn_status);
u8_t device_discovery_add_attr(char *discovered_json, bool last_attr);
u8_t device_discovery_send();
u8_t device_discovery_encode(connected_ble_devices* conn_ptr);
u8_t device_value_write_result_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length);
u8_t device_descriptor_value_changed_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length);
u8_t device_error_encode(char* ble_address, char* error_msg);
u8_t device_shadow_delete_encode();
u8_t device_disconnect_result_encode(char* ble_address, bool conn_status);
int device_shadow_data_encode(char* ble_address, bool connecting, bool connected);
#endif
#ifndef BLE_CODEC_H_
#define BLE_CODEC_H_

//#include "ble.h"
#include "cJSON.h"
#include "ble_conn_mgr.h"

int device_found_encode(u8_t num_devices_found);
int device_connect_result_encode(char* ble_address, bool conn_status);
int device_discovery_result_encode(char* ble_address, bool conn_status,
				   char *discovered_json);
int device_value_changed_encode(char* ble_address, char* uuid, char* path,
				char* value, u16_t value_length);
int device_chrc_read_encode(char* ble_address, char* uuid, char* path,
			    char* value, u16_t value_length);
int device_discovery_add_attr(char *discovered_json, bool last_attr);
int device_discovery_send();
int device_discovery_encode(connected_ble_devices* conn_ptr);
int device_value_write_result_encode(char* ble_address, char* uuid, char* path,
				     char* value, u16_t value_length);
int device_descriptor_value_changed_encode(char* ble_address, char* uuid,
					   char* path, char* value,
					   u16_t value_length);
int device_error_encode(char* ble_address, char* error_msg);
int device_shadow_delete_encode();
int device_disconnect_result_encode(char* ble_address, bool conn_status);
int device_shadow_data_encode(char* ble_address, bool connecting,
			      bool connected);
#endif
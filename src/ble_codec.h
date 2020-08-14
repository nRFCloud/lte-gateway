#ifndef BLE_CODEC_H_
#define BLE_CODEC_H_

#include "cJSON.h"
#include "ble_conn_mgr.h"

struct ble_msg {
	char *buf;
	size_t len;
};

int device_found_encode(u8_t num_devices_found, struct ble_msg *msg);
int device_connect_result_encode(char *ble_address, bool conn_status,
				 struct ble_msg *msg);
int device_value_changed_encode(char *ble_address, char *uuid, char *path,
				char *value, u16_t value_length,
				struct ble_msg *msg);
int device_chrc_read_encode(char *ble_address, char *uuid, char *path,
			    char *value, u16_t value_length,
			    struct ble_msg *msg);
int device_discovery_add_attr(char *discovered_json, bool last_attr,
			      struct ble_msg *msg);
int device_discovery_encode(connected_ble_devices *conn_ptr,
			    struct ble_msg *msg);
int device_value_write_result_encode(char *ble_address, char *uuid, char *path,
				     char *value, u16_t value_length,
				     struct ble_msg *msg);
int device_descriptor_value_changed_encode(char *ble_address, char *uuid,
					   char *path, char *value,
					   u16_t value_length,
					   struct ble_msg *msg);
int device_error_encode(char *ble_address, char *error_msg,
			struct ble_msg *msg);
int device_disconnect_result_encode(char *ble_address, bool conn_status,
				    struct ble_msg *msg);
int device_shadow_data_encode(char *ble_address, bool connecting,
			      bool connected, struct ble_msg *msg);
#endif

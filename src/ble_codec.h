#ifndef BLE_CODEC_H_
#define BLE_CODEC_H_

#include "cJSON.h"
#include "ble_conn_mgr.h"

struct ble_msg {
	struct k_mutex lock;
	char *buf;
	size_t len;
};

int device_found_encode(uint8_t num_devices_found, struct ble_msg *msg);
int device_connect_result_encode(char *ble_address, bool conn_status,
				 struct ble_msg *msg);
int device_value_changed_encode(char *ble_address, char *uuid, char *path,
				char *value, uint16_t value_length,
				struct ble_msg *msg);
int device_chrc_read_encode(char *ble_address, char *uuid, char *path,
			    char *value, uint16_t value_length,
			    struct ble_msg *msg);
int device_discovery_add_attr(char *discovered_json, bool last_attr,
			      struct ble_msg *msg);
int device_discovery_encode(struct ble_device_conn *conn_ptr,
			    struct ble_msg *msg);
int device_value_write_result_encode(char *ble_address, char *uuid, char *path,
				     char *value, uint16_t value_length,
				     struct ble_msg *msg);
int device_descriptor_value_encode(char *ble_address, char *uuid,
				   char *path, char *value,
				   uint16_t value_length,
				   struct ble_msg *msg, bool changed);
int device_error_encode(char *ble_address, char *error_msg,
			struct ble_msg *msg);
int device_disconnect_result_encode(char *ble_address, bool conn_status,
				    struct ble_msg *msg);
int gateway_shadow_data_encode(void *modem, char *buf, size_t len);
int device_shadow_data_encode(char *ble_address, bool connecting,
			      bool connected, struct ble_msg *msg);
void get_uuid_str(struct uuid_handle_pair *uuid_handle, char *str, size_t len);
int nrf_cloud_update_gateway_state(struct desired_conn *desired,
					  int num_desired);
char *get_time_str(char *dst, size_t len);
void ble_codec_init(void);

#endif

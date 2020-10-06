/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef _BLE_H_
#define _BLE_H_

#include <bluetooth/uuid.h>

#define MAX_SCAN_RESULTS 25
#define BT_ADDR_LE_DEVICE_LEN 17
#define BT_ADDR_LE_DEVICE_LEN_SHIFT BT_ADDR_LE_DEVICE_LEN+2
#define BT_ADDR_LE_TYPE_LEN 6

#define BT_ATTR_SERVICE 0
#define BT_ATTR_CHRC 1
#define BT_ATTR_CCC 3

#define NAME_LEN 30
#define UUID_STR_LEN 37
#define BT_MAX_SUBSCRIBES 25

typedef struct ble_scanned_devices {
	int rssi;
	char type[7];
	char name[NAME_LEN];
	char addr[18];
} ble_scanned_devices;

struct connected_ble_devices;

void ble_init(void);
void ble_add_to_whitelist(char *addr_str);
void scan_start(void);
void ble_subscribe(char *ble_addr, char *chrc_uuid, uint8_t value_type);
void gatt_read(char *ble_addr, char *chrc_uuid);
void gatt_write(char *ble_addr, char *chrc_uuid, uint8_t *data, uint16_t data_len);
uint8_t ble_discover(char *ble_addr);
void bt_uuid_get_str(const struct bt_uuid *uuid, char *str, size_t len);
void bt_to_upper(char *addr, uint8_t addr_len);
uint8_t disconnect_device_by_addr(char *ble_addr);
void ble_remove_from_whitelist(char *addr_str);
void ble_clear_discover_inprogress();
int device_discovery_send(struct connected_ble_devices *conn_ptr);
void update_shadow(char *ble_address, bool connecting, bool connected);

#endif /* _BLE_H_ */

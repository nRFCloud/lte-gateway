/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef _BLE_H_
#define _BLE_H_

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

#define READ_BUF_SIZE 256

typedef struct ble_scanned_devices
{
  int rssi;
  char type[7];
  char name[30];
  char addr[18];

} ble_scanned_devices;

void ble_init(void);
void ble_add_to_whitelist(char* addr_str, char* conn_type);
void scan_start(void);
void ble_subscribe(char* ble_addr, char* chrc_uuid, u8_t value_type);
void gatt_read(char* ble_addr, char* chrc_uuid);
void gatt_write(char* ble_addr, char* chrc_uuid, u8_t* data, u16_t data_len);
u8_t ble_discover(char* ble_addr, char* type);
void bt_uuid_get_str(const struct bt_uuid *uuid, char *str, size_t len);
void bt_to_upper(char* addr, u8_t addr_len);
u8_t disconnect_device_by_addr(char* ble_addr, char* type);
void ble_remove_from_whitelist(char* addr_str, char* conn_type);
void ble_clear_discover_inprogress();
#endif /* _BLE_H_ */

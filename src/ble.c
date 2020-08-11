/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>

#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <dk_buttons_and_leds.h>

#include "net/nrf_cloud.h"
#include "ble.h"

#include "ble_codec.h"
#include "ctype.h"
#include "nrf_cloud_transport.h"
#include "ble_conn_mgr.h"

#define SEND_NOTIFY_STACK_SIZE 2048
#define SEND_NOTIFY_PRIORITY 9

#define SUBSCRIPTION_LIMIT 4

#include <logging/log.h>
LOG_MODULE_REGISTER(ble, CONFIG_APRICITY_GATEWAY_LOG_LEVEL);

bool discover_in_progress = false;
bool ok_to_send = true;
bool scan_waiting = false;

int num_devices_found = 0;
u8_t read_buf[READ_BUF_SIZE];

struct k_timer rec_timer;
struct k_timer scan_timer;
struct k_timer auto_conn_start_timer;

struct k_work scan_off_work;
struct k_work ble_device_encode_work;
struct k_work start_auto_conn_work;

char uuid[BT_MAX_UUID_LEN];
char path[BT_MAX_PATH_LEN];

ble_scanned_devices ble_scanned_device[MAX_SCAN_RESULTS];



struct rec_data_t
{
        void *fifo_reserved;
        struct bt_gatt_subscribe_params sub_params;
        struct bt_gatt_read_params read_params;
        char addr_trunc[BT_ADDR_STR_LEN];
        char data[256];
        bool read;
        u8_t length;
};

K_FIFO_DEFINE(rec_fifo);

/*Convert ble address string to uppcase*/
void bt_to_upper(char* addr, u8_t addr_len)
{

        for(int i = 0; i < addr_len; i++)
        {
                addr[i] = toupper(addr[i]);
        }

}

/*Get uuid string from bt_uuid object*/
void bt_uuid_get_str(const struct bt_uuid *uuid, char *str, size_t len)
{

        u32_t tmp1, tmp5;
        u16_t tmp0, tmp2, tmp3, tmp4;

        switch (uuid->type) {
        case BT_UUID_TYPE_16:
                snprintk(str, len, "%04x", BT_UUID_16(uuid)->val);
                break;
        case BT_UUID_TYPE_32:
                snprintk(str, len, "%04x", BT_UUID_32(uuid)->val);
                break;
        case BT_UUID_TYPE_128:
                memcpy(&tmp0, &BT_UUID_128(uuid)->val[0], sizeof(tmp0));
                memcpy(&tmp1, &BT_UUID_128(uuid)->val[2], sizeof(tmp1));
                memcpy(&tmp2, &BT_UUID_128(uuid)->val[6], sizeof(tmp2));
                memcpy(&tmp3, &BT_UUID_128(uuid)->val[8], sizeof(tmp3));
                memcpy(&tmp4, &BT_UUID_128(uuid)->val[10], sizeof(tmp4));
                memcpy(&tmp5, &BT_UUID_128(uuid)->val[12], sizeof(tmp5));

                snprintk(str, len, "%08x%04x%04x%04x%08x%04x",
                         tmp5, tmp4, tmp3, tmp2, tmp1, tmp0);
                break;
        default:
                (void)memset(str, 0, len);
                return;
        }

}

static void svc_attr_data_add(const struct bt_gatt_service_val *gatt_service, u16_t handle, connected_ble_devices* ble_conn_ptr)
{

        char str[UUID_STR_LEN];

        bt_uuid_get_str(gatt_service->uuid, str, sizeof(str));

        bt_to_upper(str, strlen(str));

        ble_conn_mgr_add_uuid_pair(gatt_service->uuid, handle, 0, 0, BT_ATTR_SERVICE, ble_conn_ptr, true);

}

static void chrc_attr_data_add(const struct bt_gatt_chrc *gatt_chrc,  connected_ble_devices* ble_conn_ptr)
{

        u16_t handle = gatt_chrc->value_handle;

        ble_conn_mgr_add_uuid_pair(gatt_chrc->uuid, handle, 1, gatt_chrc->properties, BT_ATTR_CHRC, ble_conn_ptr, false);

}

static void ccc_attr_data_add(const struct bt_uuid* uuid, u16_t handle, connected_ble_devices* ble_conn_ptr)
{

        LOG_DBG("\tHandle: %d",handle);

        ble_conn_mgr_add_uuid_pair(uuid, handle, 2, 0, BT_ATTR_CCC, ble_conn_ptr, false);

}

/*Add attributes to the connection manager objects*/
static void attr_add(const struct bt_gatt_dm *dm,
                     const struct bt_gatt_dm_attr *attr,
                     connected_ble_devices* ble_conn_ptr)
{
        char str[UUID_STR_LEN];
        /* PETE: NEED TO RESOLVE attr->user_data not being present now in 1.3.0 */

	// perhaps use:
	// struct bt_gatt_service_val *bt_gatt_dm_attr_service_val(const struct bt_gatt_dm_attr *attr)
	// struct bt_gatt_chrc *bt_gatt_dm_attr_chrc_val(const struct bt_gatt_dm_attr *attr)

	const struct bt_gatt_service_val *gatt_service =
		bt_gatt_dm_attr_service_val(attr);
	const struct bt_gatt_chrc *gatt_chrc =
		bt_gatt_dm_attr_chrc_val(attr);

        bt_uuid_get_str(attr->uuid, str, sizeof(str));

        bt_to_upper(str, strlen(str));

        if ((bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) == 0) ||
            (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) == 0)) {
                svc_attr_data_add(gatt_service, attr->handle, ble_conn_ptr);
        } else if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) == 0) {
                chrc_attr_data_add(gatt_chrc, ble_conn_ptr);
        } else if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC) == 0) {
                ccc_attr_data_add(attr->uuid, attr->handle, ble_conn_ptr);
        }
}

void ble_dm_data_add(struct bt_gatt_dm *dm)
{

        const struct bt_gatt_dm_attr *attr = NULL;
        char addr_trunc[BT_ADDR_STR_LEN];
        char addr[BT_ADDR_LE_STR_LEN];

        connected_ble_devices* ble_conn_ptr;

        struct bt_conn* conn_obj;
        conn_obj = bt_gatt_dm_conn_get(dm);

        bt_addr_le_to_str(bt_conn_get_dst(conn_obj), addr, sizeof(addr));

        memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
        addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

        bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

        ble_conn_mgr_get_conn_by_addr(addr_trunc, &ble_conn_ptr);

        discover_in_progress = true;

        attr = bt_gatt_dm_service_get(dm);

        attr_add(dm, attr, ble_conn_ptr);

        while (NULL != (attr = bt_gatt_dm_attr_next(dm, attr))) {
                attr_add(dm, attr, ble_conn_ptr);
        }

}

/*Thread responsible for transfering ble data over MQTT*/
void send_notify_data(int unused1, int unused2, int unused3)
{

        memset(uuid, 0, BT_MAX_UUID_LEN);
        memset(path, 0, BT_MAX_PATH_LEN);

        connected_ble_devices* connected_ptr;

        while (1)
        {
                struct rec_data_t *rx_data = k_fifo_get(&rec_fifo, K_NO_WAIT);

                if(rx_data!= NULL)
                {
                        LOG_DBG("Trunc Addr %s", rx_data->addr_trunc);

                        ble_conn_mgr_get_conn_by_addr(rx_data->addr_trunc, &connected_ptr);

                        if(rx_data->read)
                        {
                                LOG_DBG("Value Handle %d", rx_data->read_params.single.handle);

                                ble_conn_mgr_get_uuid_by_handle(rx_data->read_params.single.handle, uuid, connected_ptr);

                                ble_conn_mgr_generate_path(connected_ptr, rx_data->read_params.single.handle, path, false);

                                device_chrc_read_encode(rx_data->addr_trunc, uuid, path, ((char *)rx_data->data), rx_data->length);
                        }
                        else
                        {
                                LOG_DBG("Value Handle %d", rx_data->sub_params.value_handle);

                                ble_conn_mgr_get_uuid_by_handle(rx_data->sub_params.value_handle, uuid, connected_ptr);

                                ble_conn_mgr_generate_path(connected_ptr, rx_data->sub_params.value_handle, path, true);
                                device_value_changed_encode(rx_data->addr_trunc, uuid, path, ((char *)rx_data->data), rx_data->length);
                        }

                        k_free(rx_data);
                }
                k_sleep(K_MSEC(50));
        }

}

K_THREAD_DEFINE(rec_thread, SEND_NOTIFY_STACK_SIZE,
                send_notify_data, NULL, NULL, NULL,
                SEND_NOTIFY_PRIORITY, 0, 0);


static void discovery_completed(struct bt_gatt_dm *disc, void *ctx)
{

        LOG_DBG("Attribute count: %d", bt_gatt_dm_attr_cnt(disc));

        ble_dm_data_add(disc);

        bt_gatt_dm_data_release(disc);

        bt_gatt_dm_continue(disc, NULL);

}

/*Despite the name. This is what is called at the end of a discovery service.*/
static void discovery_service_not_found(struct bt_conn *conn, void *ctx)
{

        LOG_DBG("Service not found!");

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];
        connected_ble_devices* connected_ptr;

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
        addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

        bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

        ble_conn_mgr_get_conn_by_addr(addr_trunc, &connected_ptr);

        connected_ptr->encode_discovered = true;
        connected_ptr->discovered = true;
        connected_ptr->discovering = false;
        discover_in_progress = false;

        //check scan waiting
        if(scan_waiting)
        {
          scan_start();
        }

}

static void discovery_error_found(struct bt_conn *conn, int err, void *ctx)
{
        LOG_ERR("The discovery procedure failed, err %d", err);

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];
        connected_ble_devices* connected_ptr;

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
        addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

        bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

        ble_conn_mgr_get_conn_by_addr(addr_trunc, &connected_ptr);
        connected_ptr->num_pairs = 0;
        connected_ptr->discovering = false;
        connected_ptr->discovered = false;
        discover_in_progress = false;

        //Disconnect?
        bt_conn_disconnect(conn, 0x16);

        //check scan waiting
        if(scan_waiting)
        {
          scan_start();
        }

}

static u8_t gatt_read_callback(struct bt_conn *conn, u8_t err,
                               struct bt_gatt_read_params *params,
                               const void *data, u16_t length)
{

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];

        if(length > 0 && data != NULL)
        {
                bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

                memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
                addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

                bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

                LOG_INF("Read Addr %s", addr_trunc);

                struct rec_data_t read_data = { .length = length, .read = true};

                memcpy(&read_data.addr_trunc, addr_trunc, strlen(addr_trunc));
                memcpy(&read_data.data, data, length);
                memcpy(&read_data.read_params, params, sizeof(struct bt_gatt_read_params));

                size_t size = sizeof(struct rec_data_t);

                char *mem_ptr = k_malloc(size);


                if(mem_ptr != NULL)
                {

                        memcpy(mem_ptr, &read_data, size);

                        k_fifo_put(&rec_fifo, mem_ptr);

                }

        }

        return BT_GATT_ITER_CONTINUE;

}

void gatt_read(char* ble_addr, char* chrc_uuid)
{

        int err;

        static struct bt_gatt_read_params params;
        struct bt_conn *conn;
        bt_addr_le_t addr;
        connected_ble_devices* connected_ptr;
        u16_t handle;

        ble_conn_mgr_get_conn_by_addr(ble_addr, &connected_ptr);

        err = ble_conn_mgr_get_handle_by_uuid(&handle, chrc_uuid, connected_ptr);

        if (err) {
                LOG_ERR("Could not find handle");
                return;
        }

        params.handle_count = 1;
        params.single.handle = handle;
        params.func = gatt_read_callback;

        err = bt_addr_le_from_str(ble_addr, "random", &addr);
        if (err) {
                LOG_ERR("Address from string failed (err %d)", err);
        }

        conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &addr);
        if (conn == NULL) {
                LOG_ERR("Null Conn object (err)");
                return;
        }

        bt_gatt_read(conn, &params);

        bt_conn_unref(conn);



}

static void on_sent(struct bt_conn *conn, u8_t err,
                    struct bt_gatt_write_params *params)
{
        const void *data;
        u16_t length;

        /* Make a copy of volatile data that is required by the callback. */
        data = params->data;
        length = params->length;

        LOG_DBG("Sent Data of Length: %d", length);
}

void gatt_write(char* ble_addr, char* chrc_uuid, u8_t* data, u16_t data_len)
{

        int err;
        struct bt_conn *conn;
        bt_addr_le_t addr;
        connected_ble_devices* connected_ptr;
        u16_t handle;
        static struct bt_gatt_write_params params;

        ble_conn_mgr_get_conn_by_addr(ble_addr, &connected_ptr);

        ble_conn_mgr_get_handle_by_uuid(&handle, chrc_uuid, connected_ptr);

        err = bt_addr_le_from_str(ble_addr, "random", &addr);
        if (err) {
                LOG_ERR("Address from string failed (err %d)", err);
        }

        conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &addr);
        if (conn == NULL) {
                LOG_ERR("Null Conn object (err)\n");
                return;
        }

        for(int i =0; i< data_len; i++)
        {
                LOG_DBG("Writing: %x", data[i]);
        }

        LOG_INF("Writing to addr: %s to chrc %s with handle %d\n:", ble_addr, chrc_uuid, handle);

        params.func = on_sent;
        params.handle = handle;
        params.offset = 0;
        params.data = data;
        params.length = data_len;

        bt_gatt_write(conn, &params);

        bt_conn_unref(conn);


//TODO: Add function for write without response.
//  bt_gatt_write_without_response(conn, handle, data, data_len, false);

}


void rec_timer_handler(struct k_timer *timer)
{

        ok_to_send = true;

}
K_TIMER_DEFINE(rec_timer, rec_timer_handler, NULL);


static u8_t on_received(struct bt_conn *conn,
                        struct bt_gatt_subscribe_params *params,
                        const void *data, u16_t length)
{

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];

        if (!data)
        {
                return BT_GATT_ITER_STOP;
        }

        u32_t lock = irq_lock();

        if(length>0 && data !=NULL && ok_to_send)
        {

                bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

                memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
                addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

                bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

                LOG_DBG("Received on addr %s", addr_trunc);

                struct rec_data_t tx_data = { .length = length};

                memcpy(&tx_data.addr_trunc, addr_trunc, strlen(addr_trunc));
                memcpy(&tx_data.data, data, length);
                memcpy(&tx_data.sub_params, params, sizeof(struct bt_gatt_subscribe_params));

                size_t size = sizeof(struct rec_data_t);

                char *mem_ptr = k_malloc(size);

                if(mem_ptr != NULL)
                {

                        memcpy(mem_ptr, &tx_data, size);
                        k_fifo_put(&rec_fifo, mem_ptr);

                }

                k_timer_start(&rec_timer, K_MSEC(500), K_SECONDS(0)); //Timer to limit the amount of data we can send. Some characteristics notify faster than can be processed.
                ok_to_send = false;
        }

        irq_unlock(lock);



        return BT_GATT_ITER_CONTINUE;

}


void ble_subscribe(char* ble_addr, char* chrc_uuid, u8_t value_type)
{

        int err;
        static int index = 0;
        static u8_t curr_subs = 0;
        /* Must be statically allocated */
        static struct bt_gatt_subscribe_params param[BT_MAX_SUBSCRIBES];     //TODO: The param needs to remain the entire time the sub exists. Should probably be stored with the conn manager.
        struct bt_conn *conn;
        bt_addr_le_t addr;
        connected_ble_devices* connected_ptr;
        u16_t handle;
        bool subscribed;
        u8_t param_index;

        ble_conn_mgr_get_conn_by_addr(ble_addr, &connected_ptr);

        ble_conn_mgr_get_handle_by_uuid(&handle, chrc_uuid, connected_ptr);

        ble_conn_mgr_get_subscribed(handle, connected_ptr, &subscribed, &param_index);


        param[index].notify = on_received;

        param[index].value = BT_GATT_CCC_NOTIFY;

        if(value_type == BT_GATT_CCC_INDICATE)
        {
                param[index].value = BT_GATT_CCC_INDICATE;
        }

        param[index].value_handle = handle;
        param[index].ccc_handle = handle+1;

        LOG_INF("Subscribing Address: %s", ble_addr);
        LOG_INF("Value Handle: %d", param[index].value_handle);
        LOG_INF("CCC Handle: %d", param[index].ccc_handle);

        err = bt_addr_le_from_str(ble_addr, "random", &addr);
        if (err) {
                LOG_ERR("Address from string failed (err %d)", err);
        }

        conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &addr);
        if (conn == NULL) {
                LOG_ERR("Null Conn object (err %d)", err);
                goto end;
        }

        ble_conn_mgr_generate_path(connected_ptr, handle, path, true);

        if(!subscribed && curr_subs < SUBSCRIPTION_LIMIT)
        {
                err = bt_gatt_subscribe(conn, &param[index]);
                if (err) {
                        LOG_ERR("Subscribe failed (err %d)", err);
                        goto end;
                }

                ble_conn_mgr_set_subscribed(handle, index, connected_ptr);

                u8_t value[2] = {1,0};
                device_descriptor_value_changed_encode(ble_addr, "2902", path, value, 2);
                device_value_write_result_encode(ble_addr, "2902", path, value, 2);


end:

                LOG_INF("Subscribed to %d", handle+1);
                curr_subs++;
                index++;
        }
        else if(subscribed) //If subscribed then unsubscribe.
        {

                bt_gatt_unsubscribe(conn, &param[param_index]);
                LOG_INF("Unsubscribed to %d", handle+1);
                ble_conn_mgr_remove_subscribed(handle, connected_ptr);

                u8_t value[2] = {0,0};
                device_descriptor_value_changed_encode(ble_addr, "2902", path, value, 2);
                device_value_write_result_encode(ble_addr, "2902", path, value, 2);
                curr_subs--;
        }

        else if(curr_subs >= SUBSCRIPTION_LIMIT) //Send error when limit is reached.
        {

                device_error_encode(ble_addr, "Reached subscription limit of 4");

        }

        if (conn != NULL) {
                bt_conn_unref(conn);
        }
}

static struct bt_gatt_dm_cb discovery_cb = {
        .completed = discovery_completed,
        .service_not_found = discovery_service_not_found,
        .error_found = discovery_error_found,
};


u8_t ble_discover(char* ble_addr, char* type)
{

        int err;
        struct bt_conn *conn;
        bt_addr_le_t addr;
        connected_ble_devices* connection_ptr;


        printk("Discover Addr: %s\n", ble_addr);
        printk("Discover Type: %s\n", type);

        if(!discover_in_progress)
        {

                err = bt_addr_le_from_str(ble_addr, type, &addr);
                if (err) {
                        LOG_ERR("Address from string failed (err %d)", err);
                        return err;
                }


                conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &addr);
                if (conn == NULL) {
                        LOG_DBG("Null Conn object (err %d)", err);
                        return 1;
                }

                ble_conn_mgr_get_conn_by_addr(ble_addr, &connection_ptr);

                if(!connection_ptr->discovered)
                {
                        err = bt_gatt_dm_start(conn, NULL, &discovery_cb, NULL);
                        if (err) {
                                LOG_ERR("Could not start service discovery, err %d", err);
                                connection_ptr->discovering = false;

                                //Disconnect device.
                                LOG_INF("Disconnecting from device...");
                                bt_conn_disconnect(conn, 0x16);

                                bt_conn_unref(conn);
                                return err;
                        }
                        discover_in_progress = true;
                        connection_ptr->discovering = true;
                }
                else
                {
                        connection_ptr->encode_discovered = true;
                }
        }
        else
        {
                return 1;
        }

        bt_conn_unref(conn);
        return err;

}


u8_t disconnect_device_by_addr(char* ble_addr, char* type)
{

        int err;
        struct bt_conn *conn;
        bt_addr_le_t addr;

        err = bt_addr_le_from_str(ble_addr, type, &addr);
        if (err) {
                LOG_ERR("Address from string failed (err %d)", err);
                return err;
        }

        conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &addr);
        if (conn == NULL) {
                LOG_DBG("Null Conn object (err %d)", err);
                return 1;
        }

        //Disconnect device.
        bt_conn_disconnect(conn, 0x16);

        bt_conn_unref(conn);

        return err;

}

void auto_conn_start_work_handler(struct k_work *work)
{

        int err;

        //Restart to scanning for whitelisted devices
        struct bt_conn_le_create_param param = BT_CONN_LE_CREATE_PARAM_INIT(
                                                BT_CONN_LE_OPT_NONE,
                                                BT_GAP_SCAN_FAST_INTERVAL,
                                                BT_GAP_SCAN_FAST_WINDOW);

        err = bt_conn_le_create_auto(&param, BT_LE_CONN_PARAM_DEFAULT);

        if (err) {
                LOG_INF("Connection exists");
        } else {
                LOG_INF("Connection creation pending");
        }

}

K_WORK_DEFINE(auto_conn_start_work, auto_conn_start_work_handler);

void auto_conn_start_timer_handler(struct k_timer *timer)
{
        k_work_submit(&auto_conn_start_work);

}
K_TIMER_DEFINE(auto_conn_start_timer, auto_conn_start_timer_handler, NULL);

static void connected(struct bt_conn *conn, u8_t conn_err)
{

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];
        connected_ble_devices* connection_ptr;

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
        addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

        bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

        ble_conn_mgr_get_conn_by_addr(addr_trunc, &connection_ptr);

        if (conn_err) {

                LOG_ERR("Failed to connect to %s (%u)", addr, conn_err);
                ble_conn_set_connected(addr_trunc, false);
                bt_conn_unref(conn);
                return;
        }

        LOG_INF("Connected: %s", addr);

        device_connect_result_encode(addr_trunc, true);

        device_shadow_data_encode(addr_trunc, false, true);

        ble_conn_set_connected(addr_trunc, true);

        ble_remove_from_whitelist(addr_trunc, connection_ptr->addr_type);

        //Start the timer to begin scanning again.
        k_timer_start(&auto_conn_start_timer, K_SECONDS(3), K_SECONDS(0));

}

static void disconnected(struct bt_conn *conn, u8_t reason)
{

        char addr[BT_ADDR_LE_STR_LEN];
        char addr_trunc[BT_ADDR_STR_LEN];
        connected_ble_devices* connection_ptr;

        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        memcpy(addr_trunc, addr, BT_ADDR_LE_DEVICE_LEN);
        addr_trunc[BT_ADDR_LE_DEVICE_LEN] = 0;

        bt_to_upper(addr_trunc, BT_ADDR_LE_STR_LEN);

        ble_conn_mgr_get_conn_by_addr(addr_trunc, &connection_ptr);

        device_disconnect_result_encode(addr_trunc, false);

        device_shadow_data_encode(addr_trunc, false, false);

        ble_conn_set_disconnected(addr_trunc);

        LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

        if(!connection_ptr->free)
        {
                ble_add_to_whitelist(connection_ptr->addr, connection_ptr->addr_type);
        }

        //Start the timer to begin scanning again.
        k_timer_start(&auto_conn_start_timer, K_SECONDS(3), K_SECONDS(0));

}

static struct bt_conn_cb conn_callbacks = {
        .connected = connected,
        .disconnected = disconnected,
};

void scan_filter_match(struct bt_scan_device_info *device_info,
                       struct bt_scan_filter_match *filter_match,
                       bool connectable)
{

        char addr[BT_ADDR_LE_STR_LEN];

        bt_addr_le_to_str(device_info->addr, addr, sizeof(addr));

        LOG_INF("Device found: %s", addr);

}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
        LOG_ERR("Connection to peer failed!");
}


static bool data_cb(struct bt_data *data, void *user_data)
{

        char *name = user_data;

        switch (data->type) {
        case BT_DATA_NAME_SHORTENED:
        case BT_DATA_NAME_COMPLETE:
                memcpy(name, data->data, MIN(data->data_len, NAME_LEN - 1));
                return false;
        default:
                return true;
        }

}



void ble_device_found_enc_handler(struct k_work *work)
{

        device_found_encode(num_devices_found);

}

K_WORK_DEFINE(ble_device_encode_work, ble_device_found_enc_handler);


static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
                         struct net_buf_simple *ad)
{
        char addr_str[BT_ADDR_LE_STR_LEN];
        char name[NAME_LEN] = {0};
        bool dup_addr = false;

        (void)memset(name, 0, sizeof(name));

        bt_data_parse(ad, data_cb, name);

        /* We're only interested in connectable events */
        if (type != BT_HCI_ADV_IND && type != BT_HCI_ADV_DIRECT_IND) {
                return;
        }

        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

        memcpy(ble_scanned_device[num_devices_found].type, addr_str+BT_ADDR_LE_DEVICE_LEN_SHIFT, BT_ADDR_LE_TYPE_LEN);
        ble_scanned_device[num_devices_found].type[BT_ADDR_LE_TYPE_LEN] = 0;

        bt_to_upper(addr_str, BT_ADDR_LE_STR_LEN);

        memcpy(ble_scanned_device[num_devices_found].addr, addr_str, BT_ADDR_LE_DEVICE_LEN);
        ble_scanned_device[num_devices_found].addr[BT_ADDR_LE_DEVICE_LEN] = 0;
        ble_scanned_device[num_devices_found].rssi = (int)rssi;
        memcpy(ble_scanned_device[num_devices_found].name, name, strlen(name));

        /* Check for dupclicate addresses */
        for(int j=0; j<num_devices_found; j++)
        {
                if(!(strcmp(ble_scanned_device[num_devices_found].addr, ble_scanned_device[j].addr)))
                {
                        dup_addr = true;
                }
        }

        if((num_devices_found < MAX_SCAN_RESULTS) && (dup_addr == false))
        {
                LOG_INF("Device found: %s (RSSI %d)", ble_scanned_device[num_devices_found].addr, rssi);
                LOG_INF("Device Name: %s", ble_scanned_device[num_devices_found].name);
                LOG_INF("Type: %s", ble_scanned_device[num_devices_found].type);

                num_devices_found++;
        }
}

void scan_off_handler(struct k_work *work)
{
        int err;

        err = bt_le_scan_stop();
        if (err) {
                LOG_INF("Stopping scanning failed (err %d)", err);
        } else {
                LOG_INF("Scan successfully stopped");
        }


        //Start the timer to begin scanning again.
        k_timer_start(&auto_conn_start_timer, K_SECONDS(3), K_SECONDS(0));

        k_work_submit(&ble_device_encode_work);

}

K_WORK_DEFINE(scan_off_work, scan_off_handler);

void scan_timer_handler(struct k_timer *timer)
{

        k_work_submit(&scan_off_work);

}

K_TIMER_DEFINE(scan_timer, scan_timer_handler, NULL);

void ble_add_to_whitelist(char* addr_str, char* conn_type)
{

        int err;
        bt_addr_le_t addr;

        LOG_INF("Whitelisting Address: %s", addr_str);
        LOG_INF("Whitelisting Address Type: %s", conn_type);

        err = bt_addr_le_from_str(addr_str, conn_type, &addr);
        if (err) {
                LOG_ERR("Invalid peer address (err %d)", err);
                return;
        }

        bt_conn_create_auto_stop();

        bt_le_whitelist_add(&addr);

        //Start the timer to begin scanning again.
        k_timer_start(&auto_conn_start_timer, K_SECONDS(3), K_SECONDS(0));

}

void ble_remove_from_whitelist(char* addr_str, char* conn_type)
{

        int err;
        bt_addr_le_t addr;

        LOG_INF("Removing Whitelist Address: %s", addr_str);
        LOG_INF("Whitelisting Address Type: %s", conn_type);

        err = bt_addr_le_from_str(addr_str, conn_type, &addr);
        if (err) {
                LOG_ERR("Invalid peer address (err %d)", err);
                return;
        }

        bt_conn_create_auto_stop();

        bt_le_whitelist_rem(&addr);

        //Start the timer to begin scanning again.
        k_timer_start(&auto_conn_start_timer, K_SECONDS(3), K_SECONDS(0));

}

void scan_start(void)
{

        int err;

        num_devices_found = 0;
        memset(ble_scanned_device, 0, sizeof(ble_scanned_device));

        struct bt_le_scan_param param = {
                .type       = BT_HCI_LE_SCAN_ACTIVE,
                .filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE,
                .interval   = BT_GAP_SCAN_FAST_INTERVAL,
                .window     = BT_GAP_SCAN_FAST_WINDOW
        };

        if(!discover_in_progress)
        {

              //Stop the auto connect
              bt_conn_create_auto_stop();

              err = bt_le_scan_start(&param, device_found);
              if (err) {
                      LOG_ERR("Bluetooth set active scan failed "
                              "(err %d)\n", err);
              } else {
                      LOG_INF("Bluetooth active scan enabled");

                      k_timer_start(&scan_timer, K_SECONDS(5), K_SECONDS(0));      //TODO: Get scan timeout from scan message
              }

              scan_waiting = false;
        }
        else
        {
              LOG_INF("Scan waiting... Discover in progress");
              scan_waiting = true;

        }

}

void ble_clear_discover_inprogress()
{
    discover_in_progress = false;
}

static void ble_ready(int err)
{

        LOG_INF("Bluetooth ready");

        bt_conn_cb_register(&conn_callbacks);

}

void ble_init(void)
{
        int err;

        LOG_INF("Initializing Bluetooth..");
        err = bt_enable(ble_ready);
        if (err) {
                LOG_ERR("Bluetooth init failed (err %d)", err);
                return;
        }

        for(int i = 0; i< MAX_SCAN_RESULTS; i++)
        {
                memset(ble_scanned_device[i].name, 0, sizeof(ble_scanned_device[1].name));
        }
}

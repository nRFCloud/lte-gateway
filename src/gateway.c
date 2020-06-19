#include "nrf_cloud_codec.h"
#include "gateway.h"
#include "nrf_cloud_transport.h"
#include "nrf_cloud_mem.h"

#include <zephyr.h>
#include <stdio.h>
#include <net/mqtt.h>
#include <net/socket.h>
#include <bluetooth/gatt.h>

#include "cJSON.h"
#include "cJSON_os.h"
#include "ble.h"
#include "bluetooth/bluetooth.h"
#include "ble_codec.h"
#include <power/reboot.h>
#include <power/reboot.h>
#include "ble_conn_mgr.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(gateway, CONFIG_APRICITY_GATEWAY_LOG_LEVEL);

#define CLOUD_PROC_STACK_SIZE 2048
#define CLOUD_PROC_PRIORITY 5

u8_t value_buf[256];


struct cloud_data_t
{
        void *fifo_reserved;
        char addr[BT_ADDR_STR_LEN];
        char uuid[UUID_STR_LEN];
        bool read;
        bool sub;
        bool indicate;
};

K_FIFO_DEFINE(cloud_data_fifo);

void cloud_data_process(int unused1, int unused2, int unused3)
{

        while (1)
        {

                struct cloud_data_t *cloud_data = k_fifo_get(&cloud_data_fifo, K_NO_WAIT);

                if(cloud_data!= NULL)
                {

                        u32_t lock = irq_lock();
                        if(cloud_data->read)
                        {
                                gatt_read(cloud_data->addr, cloud_data->uuid);

                        }
                        else if(cloud_data->sub)
                        {

                                if(cloud_data->indicate)
                                {
                                        ble_subscribe(cloud_data->addr, cloud_data->uuid, BT_GATT_CCC_NOTIFY);
                                }
                                else
                                {
                                        ble_subscribe(cloud_data->addr, cloud_data->uuid, BT_GATT_CCC_INDICATE);
                                }

                        }

                        /*Writing added too much data to the fifo. There would be no memory to unsubscribe from memory intensive subscribes.*/
              //        else if(cloud_data->write)
              //        {
              //
              //          gatt_write(cloud_data->addr, cloud_data->uuid, cloud_data->data, cloud_data->data_len);
              //
              //        }

                        k_free(cloud_data);

                        irq_unlock(lock);
                }

                k_sleep(K_MSEC(1000));
        }

}

K_THREAD_DEFINE(cloud_proc_thread, CLOUD_PROC_STACK_SIZE,
                cloud_data_process, NULL, NULL, NULL,
                CLOUD_PROC_PRIORITY, 0, 0);

static cJSON *json_object_decode(cJSON *obj, const char *str)
{
        return obj ? cJSON_GetObjectItem(obj, str) : NULL;
}

static bool compare(const char *s1, const char *s2)
{
        return !strncmp(s1, s2, strlen(s2));
}

u8_t gateway_handler(const struct nct_gw_data *gw_data)
{

        cJSON *root_obj;
        cJSON *desired_obj;

        cJSON *type_obj;
        cJSON *operation_obj;

        cJSON *ble_address;

        cJSON *chrc_uuid;
        cJSON *service_uuid;
        cJSON *desc_uuid;
        cJSON *desc_arr;
        u8_t desc_buf[2] = {0};
        u8_t desc_len = 0;

        cJSON *value_arr;
        u8_t value_len = 0;

        root_obj = cJSON_Parse(gw_data->data.ptr);

        if (root_obj == NULL) {
                LOG_ERR("cJSON_Parse failed: %s",
                       log_strdup((char *)gw_data->data.ptr));
                return -ENOENT;
        }

        type_obj = json_object_decode(root_obj, "type");

        if (type_obj != NULL)
        {
                const char *type_str = type_obj->valuestring;
                if (compare(type_str, "operation")) {
                        operation_obj = json_object_decode(root_obj, "operation");
                        desired_obj = json_object_decode(operation_obj, "type");
                        if (compare(desired_obj->valuestring, "scan")) {
                                desired_obj = json_object_decode(operation_obj, "scanType");
                                switch(desired_obj->valueint)
                                {
                                case 0:
                                        // submit k_work to respond
                                        scan_start();

                                        break;
                                case 1:
                                        // submit k_work to respond
                                        //TODO: Add beacon support
                                        break;
                                default:

                                        break;
                                }
                        }

                        else if (compare(desired_obj->valuestring, "device_characteristic_value_read"))
                        {
                                ble_address = json_object_decode(operation_obj, "deviceAddress");
                                chrc_uuid = json_object_decode(operation_obj, "characteristicUUID");

                                if(ble_address != NULL && chrc_uuid != NULL)
                                {

                                        struct cloud_data_t cloud_data = { .read = true};

                                        memcpy(&cloud_data.addr, ble_address->valuestring, strlen(ble_address->valuestring));
                                        memcpy(&cloud_data.uuid, chrc_uuid->valuestring, strlen(chrc_uuid->valuestring));
                                        size_t size = sizeof(struct cloud_data_t);
                                        char *mem_ptr = k_malloc(size);


                                        if(mem_ptr != NULL)
                                        {
                                                memcpy(mem_ptr, &cloud_data, size);
                                                k_fifo_put(&cloud_data_fifo, mem_ptr);
                                        }
                                        //gatt_read(ble_address->valuestring, chrc_uuid->valuestring);
                                }
                        }

                        else if(compare(desired_obj->valuestring, "device_descriptor_value_write"))
                        {
                                ble_address = json_object_decode(operation_obj, "deviceAddress");
                                chrc_uuid = json_object_decode(operation_obj, "characteristicUUID");
                                desc_arr = json_object_decode(operation_obj, "descriptorValue");

                                desc_len = cJSON_GetArraySize(desc_arr);
                                for (int i = 0; i < desc_len; i++)
                                {
                                        cJSON * item = cJSON_GetArrayItem(desc_arr, i);
                                        desc_buf[i] = item->valueint;
                                }

                                if(ble_address != NULL && chrc_uuid != NULL)
                                {

                                        struct cloud_data_t cloud_data = { .sub = true, .indicate = true};
                                        memcpy(&cloud_data.addr, ble_address->valuestring, strlen(ble_address->valuestring));
                                        memcpy(&cloud_data.uuid, chrc_uuid->valuestring, strlen(chrc_uuid->valuestring));

                                        if(desc_buf[0] == 2)
                                        {
                                                cloud_data.indicate = false;
                                        }

                                        size_t size = sizeof(struct cloud_data_t);
                                        char *mem_ptr = k_malloc(size);

                                        if(mem_ptr != NULL)
                                        {
                                                memcpy(mem_ptr, &cloud_data, size);
                                                k_fifo_put(&cloud_data_fifo, mem_ptr);
                                        }
                                }
                        }

                        else if(compare(desired_obj->valuestring, "device_characteristic_value_write"))
                        {
                                ble_address = json_object_decode(operation_obj, "deviceAddress");
                                chrc_uuid = json_object_decode(operation_obj, "characteristicUUID");
                                service_uuid= json_object_decode(operation_obj, "serviceUUID");
                                value_arr = json_object_decode(operation_obj, "characteristicValue");

                                value_len = cJSON_GetArraySize(value_arr);

                                for (int i = 0; i < value_len; i++)
                                {
                                        cJSON * item = cJSON_GetArrayItem(value_arr, i);

                                        //TODO: add check i < max buf size
                                        value_buf[i] = item->valueint;
                                }

                                //printk("Device Write Value: %s\n", value_buf);

                                if(ble_address != NULL && chrc_uuid != NULL)
                                {

                                        //          struct cloud_data_t cloud_data = { .write = true, .data_len = value_len};
                                        //
                                        //          memcpy(&cloud_data.addr, ble_address->valuestring, strlen(ble_address->valuestring));
                                        //          memcpy(&cloud_data.uuid, chrc_uuid->valuestring, strlen(chrc_uuid->valuestring));
                                        //          memcpy(&cloud_data.data, value_buf, value_len);
                                        //          size_t size = sizeof(struct cloud_data_t);
                                        //          char *mem_ptr = k_malloc(size);
                                        //
                                        //          if(mem_ptr != NULL)
                                        //          {
                                        //            memcpy(mem_ptr, &cloud_data, size);
                                        //            k_fifo_put(&cloud_data_fifo, mem_ptr);
                                        //          }

                                        gatt_write(ble_address->valuestring, chrc_uuid->valuestring, value_buf, value_len);
                                }
                        }

                        else if(compare(desired_obj->valuestring, "device_discover"))
                        {
                                ble_address = json_object_decode(operation_obj, "deviceAddress");

                                if(ble_address != NULL)
                                {

                                          ble_conn_mgr_rediscover(ble_address->valuestring);

                                }
                        }
                }
        }

        cJSON_Delete(root_obj);
        return 0;
}
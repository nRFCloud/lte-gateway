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

#define QUEUE_CHAR_READS

/* Uncomment below to enable writing characteristics from cloud_data_process()
 * rather than from gateway_handler(). However, writing added too much data
 * to the fifo. There was no memory to unsubscribe from memory intensive
 * subscribes.
 */
/* #define QUEUE_CHAR_WRITES */

#define VALUE_BUF_SIZE 256
static uint8_t value_buf[VALUE_BUF_SIZE];

struct cloud_data_t {
	void *fifo_reserved;
	char addr[BT_ADDR_STR_LEN];
	char uuid[UUID_STR_LEN];
	bool read;
	bool sub;
	uint8_t client_char_config;
};

K_FIFO_DEFINE(cloud_data_fifo);

void cloud_data_process(int unused1, int unused2, int unused3)
{
	struct k_mutex lock;

	k_mutex_init(&lock);

	while (1) {
		struct cloud_data_t *cloud_data = k_fifo_get(&cloud_data_fifo,
							     K_NO_WAIT);

		if (cloud_data != NULL) {
			/* uint32_t lock = irq_lock(); */
			k_mutex_lock(&lock, K_FOREVER);

			if (cloud_data->sub) {
				ble_subscribe(cloud_data->addr,
					      cloud_data->uuid,
					      cloud_data->client_char_config);
			}
#if defined(QUEUE_CHAR_READS)
			else if (cloud_data->read) {
				gatt_read(cloud_data->addr, cloud_data->uuid);
			}
#endif
#if defined(QUEUE_CHAR_WRITES)
			else if (cloud_data->write) {
				gatt_write(cloud_data->addr, cloud_data->uuid,
					   cloud_data->data,
					   cloud_data->data_len);

			}
#endif
			k_free(cloud_data);
			/* irq_unlock(lock); */
			k_mutex_unlock(&lock);
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

uint8_t gateway_handler(const struct nct_gw_data *gw_data)
{
	int ret = 0;
	cJSON *root_obj;
	cJSON *desired_obj;

	cJSON *type_obj;
	cJSON *operation_obj;

	cJSON *ble_address;

	cJSON *chrc_uuid;
	cJSON *service_uuid;
	cJSON *desc_arr;
	uint8_t desc_buf[2] = {0};
	uint8_t desc_len = 0;

	cJSON *value_arr;
	uint8_t value_len = 0;

	root_obj = cJSON_Parse(gw_data->data.ptr);

	if (root_obj == NULL) {
		LOG_ERR("cJSON_Parse failed: %s",
			log_strdup((char *)gw_data->data.ptr));
		return -ENOENT;
	}

	type_obj = json_object_decode(root_obj, "type");

	if (type_obj == NULL) {
		ret = -ENOENT;
		goto exit_handler;
	}

	const char *type_str = type_obj->valuestring;

	if (!compare(type_str, "operation")) {
		ret = -EINVAL;
		goto exit_handler;
	}

	operation_obj = json_object_decode(root_obj, "operation");
	desired_obj = json_object_decode(operation_obj, "type");

	if (compare(desired_obj->valuestring, "scan")) {
		desired_obj = json_object_decode(operation_obj, "scanType");
		switch (desired_obj->valueint) {
		case 0:
			/* submit k_work to respond */
			scan_start();
			break;
		case 1:
			/* submit k_work to respond */
			/* TODO: Add beacon support */
			break;
		default:
			break;
		}

	} else if (compare(desired_obj->valuestring,
			   "device_characteristic_value_read")) {
		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		chrc_uuid = json_object_decode(operation_obj,
					       "characteristicUUID");

		if ((ble_address != NULL) && (chrc_uuid != NULL)) {
#if defined(QUEUE_CHAR_READS)
			struct cloud_data_t cloud_data = {
				.read = true
			};

			memcpy(&cloud_data.addr,
			       ble_address->valuestring,
			       strlen(ble_address->valuestring));
			memcpy(&cloud_data.uuid,
			       chrc_uuid->valuestring,
			       strlen(chrc_uuid->valuestring));

			size_t size = sizeof(struct cloud_data_t);
			char *mem_ptr = k_malloc(size);

			if (mem_ptr == NULL) {
				LOG_ERR("Out of memory!");
				ret = -ENOMEM;
				goto exit_handler;
			}

			memcpy(mem_ptr, &cloud_data, size);
			k_fifo_put(&cloud_data_fifo, mem_ptr);
#else
			gatt_read(ble_address->valuestring,
				  chrc_uuid->valuestring);
#endif
		}

	} else if (compare(desired_obj->valuestring,
			   "device_descriptor_value_write")) {
		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		chrc_uuid = json_object_decode(operation_obj,
					       "characteristicUUID");
		desc_arr = json_object_decode(operation_obj,
					      "descriptorValue");

		desc_len = cJSON_GetArraySize(desc_arr);
		for (int i = 0; i < desc_len; i++) {
			cJSON *item = cJSON_GetArrayItem(desc_arr, i);

			desc_buf[i] = item->valueint;
		}

		if ((ble_address != NULL) && (chrc_uuid != NULL)) {
			struct cloud_data_t cloud_data = {
				.sub = true,
			};

			memcpy(&cloud_data.addr,
			       ble_address->valuestring,
			       strlen(ble_address->valuestring));
			memcpy(&cloud_data.uuid,
			       chrc_uuid->valuestring,
			       strlen(chrc_uuid->valuestring));

			cloud_data.client_char_config = desc_buf[0];

			size_t size = sizeof(struct cloud_data_t);
			char *mem_ptr = k_malloc(size);

			if (mem_ptr == NULL) {
				LOG_ERR("Out of memory!");
				ret = -ENOMEM;
				goto exit_handler;
			}

			memcpy(mem_ptr, &cloud_data, size);
			k_fifo_put(&cloud_data_fifo, mem_ptr);
		}

	} else if (compare(desired_obj->valuestring,
			   "device_characteristic_value_write")) {
		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		chrc_uuid = json_object_decode(operation_obj,
					       "characteristicUUID");
		service_uuid = json_object_decode(operation_obj,
						  "serviceUUID");
		value_arr = json_object_decode(operation_obj,
					       "characteristicValue");

		value_len = MIN(cJSON_GetArraySize(value_arr), VALUE_BUF_SIZE);

		for (int i = 0; i < value_len; i++) {
			cJSON *item = cJSON_GetArrayItem(value_arr, i);
			value_buf[i] = item->valueint;
		}

		LOG_DBG("Device Write Value: %s\n", value_buf);

		if ((ble_address != NULL) && (chrc_uuid != NULL)) {
#if defined(QUEUE_CHAR_WRITES)
			struct cloud_data_t cloud_data = {
				.write = true,
				.data_len = value_len
			};

			memcpy(&cloud_data.addr,
			       ble_address->valuestring,
			       strlen(ble_address->valuestring));
			memcpy(&cloud_data.uuid,
			       chrc_uuid->valuestring,
			       strlen(chrc_uuid->valuestring));
			memcpy(&cloud_data.data,
			       value_buf,
			       value_len);

			size_t size = sizeof(struct cloud_data_t);
			char *mem_ptr = k_malloc(size);

			if (mem_ptr == NULL) {
				LOG_ERR("Out of memory!");
				ret = -ENOMEM;
				goto exit_handler;
			}

			memcpy(mem_ptr, &cloud_data, size);
			k_fifo_put(&cloud_data_fifo, mem_ptr);
#else
			gatt_write(ble_address->valuestring,
				   chrc_uuid->valuestring,
				   value_buf, value_len);
#endif
		}

	} else if (compare(desired_obj->valuestring, "device_discover")) {
		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		if (ble_address != NULL) {
			ble_conn_mgr_rediscover(ble_address->valuestring);
		}
	}

exit_handler:
	cJSON_Delete(root_obj);
	return ret;
}

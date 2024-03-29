#include <zephyr.h>
#include <stdio.h>
#include <net/mqtt.h>
#include <net/socket.h>
#include <bluetooth/gatt.h>
#include <power/reboot.h>
#include <nrf_modem.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <modem/lte_lc.h>
#include <nrf9160.h>
#include <hal/nrf_gpio.h>
#include <net/cloud.h>
#if defined(CONFIG_NRF_MODEM_LIB)
#include <nrf_socket.h>
#endif

#include "nrf_cloud_codec.h"
#include "gateway.h"
#include <net/nrf_cloud.h>
#include "nrf_cloud_mem.h"

#include "ui.h"
#include "cJSON.h"
#include "cJSON_os.h"
#include "ble.h"
#include "bluetooth/bluetooth.h"
#include "ble_codec.h"
#include "ble_conn_mgr.h"

LOG_MODULE_REGISTER(gateway, CONFIG_NRF_CLOUD_GATEWAY_LOG_LEVEL);

#define AT_CMNG_READ_LEN 97

#define CLOUD_PROC_STACK_SIZE 2048
#define CLOUD_PROC_PRIORITY 5

#define QUEUE_CHAR_READS

#define GET_PSK_ID "AT%CMNG=2,16842753,4"
#define GET_PSK_ID_LEN (sizeof(GET_PSK_ID)-1)
#define GET_PSK_ID_ERR "ERROR"

/* Uncomment below to enable writing characteristics from cloud_data_process()
 * rather than from gateway_handler(). However, writing added too much data
 * to the fifo. There was no memory to unsubscribe from memory intensive
 * subscribes.
 */
/* #define QUEUE_CHAR_WRITES */

#define VALUE_BUF_SIZE 256
static uint8_t value_buf[VALUE_BUF_SIZE];

char gateway_id[NRF_CLOUD_CLIENT_ID_LEN+1];

struct cloud_data_t {
	void *fifo_reserved;
	char addr[BT_ADDR_STR_LEN];
	char uuid[UUID_STR_LEN];
	bool read;
	bool ccc;
	bool sub;
	uint8_t client_char_config;
};

K_FIFO_DEFINE(cloud_data_fifo);

void cloud_data_process(int unused1, int unused2, int unused3)
{
	struct k_mutex lock;
	int ret;

	k_mutex_init(&lock);

	while (1) {
		struct cloud_data_t *cloud_data = k_fifo_get(&cloud_data_fifo,
							     K_NO_WAIT);

		if (cloud_data != NULL) {
			k_mutex_lock(&lock, K_FOREVER);

			if (cloud_data->sub) {
				ble_subscribe(cloud_data->addr,
					      cloud_data->uuid,
					      cloud_data->client_char_config);
			}
#if defined(QUEUE_CHAR_READS)
			else if (cloud_data->read) {
				LOG_DBG("Dequeued gatt_read request %s, %s, %u",
					log_strdup(cloud_data->addr),
					log_strdup(cloud_data->uuid),
					cloud_data->ccc);
				ret = gatt_read(cloud_data->addr,
						cloud_data->uuid,
						cloud_data->ccc);
				if (ret) {
					LOG_ERR("Error on gatt_read(%s, %s, %u): %d",
						log_strdup(cloud_data->addr),
						log_strdup(cloud_data->uuid),
						cloud_data->ccc,
						ret);
				}
			}
#endif
#if defined(QUEUE_CHAR_WRITES)
			else if (cloud_data->write) {
				ret = gatt_write(cloud_data->addr,
						 cloud_data->uuid,
						 cloud_data->data,
						 cloud_data->data_len, NULL);
				if (ret) {
					LOG_ERR("Error on gatt_write(%s, %s): %d",
						log_strdup(cloud_data->addr),
						log_strdup(cloud_data->uuid),
						ret);
				}
			}
#endif
			k_free(cloud_data);
			k_mutex_unlock(&lock);
		}
		k_sleep(K_MSEC(100));
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

int gateway_handler(const struct cloud_msg *gw_data)
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

	root_obj = cJSON_Parse(gw_data->buf);

	if (root_obj == NULL) {
		LOG_ERR("cJSON_Parse failed: %s",
			log_strdup((char *)gw_data->buf));
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
			scan_start(false);
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

		LOG_INF("Got device_characteristic_value_read: %s",
			log_strdup(ble_address->valuestring));
		if ((ble_address != NULL) && (chrc_uuid != NULL)) {
#if defined(QUEUE_CHAR_READS)
			struct cloud_data_t cloud_data = {
				.read = true,
				.ccc = false,
				.sub = false
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
			LOG_INF("Queued device_characteristic_value_read %s, %s, 0",
				log_strdup(cloud_data.addr),
				log_strdup(cloud_data.uuid));
#else
			ret = gatt_read(ble_address->valuestring,
					chrc_uuid->valuestring);
			if (ret) {
				LOG_ERR("Error on gatt_read(%s, %s, 0): %d",
					log_strdup(ble_address->valuestring),
					log_strdup(chrc_uuid->valuestring),
					ret);
			}
#endif
		}

	} else if (compare(desired_obj->valuestring,
			   "device_descriptor_value_read")) {

		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		chrc_uuid = json_object_decode(operation_obj,
					       "characteristicUUID");

		LOG_INF("Got device_descriptor_value_read: %s",
			log_strdup(ble_address->valuestring));
		if ((ble_address != NULL) && (chrc_uuid != NULL)) {
#if defined(QUEUE_CHAR_READS)
			struct cloud_data_t cloud_data = {
				.read = true,
				.ccc = true,
				.sub = false
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
			LOG_INF("Queued device_descriptor_value_read %s, %s",
				log_strdup(cloud_data.addr),
				log_strdup(cloud_data.uuid));
#else
			ret = gatt_read(ble_address->valuestring,
					chrc_uuid->valuestring, true);
			if (ret) {
				LOG_ERR("Error on gatt_read(%s, %s, 1): %d",
					log_strdup(ble_address->valuestring),
					log_strdup(chrc_uuid->valuestring),
					ret);
			}
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
				.read = false
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
			ret = gatt_write(ble_address->valuestring,
					 chrc_uuid->valuestring,
					 value_buf, value_len, NULL);
			if (ret) {
				LOG_ERR("Error on gatt_write(%s, %s): %d",
					log_strdup(ble_address->valuestring),
					log_strdup(chrc_uuid->valuestring),
					ret);
			}
#endif
		}

	} else if (compare(desired_obj->valuestring, "device_discover")) {
		ble_address = json_object_decode(operation_obj,
						 "deviceAddress");
		if (ble_address != NULL) {
			LOG_INF("Cloud requested device_discover");
			ble_conn_mgr_rediscover(ble_address->valuestring);
		}
	}

exit_handler:
	cJSON_Delete(root_obj);
	return ret;
}

void init_gateway(void)
{
	ble_codec_init();
}

int gw_client_id_query(void)
{
#if !defined(CONFIG_NRF_CLOUD_CLIENT_ID_SRC_RUNTIME)
	return nrf_cloud_client_id_get(gateway_id, sizeof(gateway_id));
#else
	return strlen(gateway_id) ? 0 : -EINVAL;
#endif
}

#if defined(CONFIG_NRF_CLOUD_CLIENT_ID_SRC_RUNTIME)
int gw_psk_id_get(char **id, size_t *id_len)
{
	char psk_buf[AT_CMNG_READ_LEN];
	int bytes_written;
	int bytes_read;
	int at_socket_fd;
	int ret = 0;
	int err;

	at_socket_fd = nrf_socket(NRF_AF_LTE, NRF_SOCK_DGRAM, NRF_PROTO_AT);
	if (at_socket_fd < 0) {
		return -EIO;
	}

	bytes_written = nrf_write(at_socket_fd, GET_PSK_ID, GET_PSK_ID_LEN);
	if (bytes_written != GET_PSK_ID_LEN) {
		ret = -EIO;
		goto cleanup;
	}
	bytes_read = nrf_read(at_socket_fd, psk_buf, AT_CMNG_READ_LEN);
	if (bytes_read != AT_CMNG_READ_LEN) {
		ret = -EIO;
		goto cleanup;
	}

	if (!strncmp(psk_buf, GET_PSK_ID_ERR, strlen(GET_PSK_ID_ERR))) {
		strcpy(gateway_id, "no-psk-ids");
		*id = gateway_id;
		*id_len = strlen(*id);
	} else {
/*
 * below, we extract the 'nrf-124578' portion as the gateway_id
 * AT%CMNG=2,16842753,4 returns:
 * %CMNG: 16842753,
 * 4,
 * "0404040404040404040404040404040404040404040404040404040404040404",
 * "nrf-124578"
 */
		int ofs;
		int i;
		int len = strlen(psk_buf);
		char *ptr = psk_buf;
		const char *delimiters = ",";

		LOG_DBG("ID is inside this: %s", log_strdup(psk_buf));
		for (i = 0; i < 3; i++) {
			ofs = strcspn(ptr, delimiters) + 1;
			ptr += ofs;
			len -= ofs;
			if (len <= 0) {
				break;
			}
		}
		if (len > 0) {
			if (*ptr == '"') {
				ptr++;
			}
			ofs = strcspn(ptr, "\"");
			memcpy(gateway_id, ptr, ofs);
			gateway_id[ofs] = 0;
			*id = gateway_id;
			*id_len = strlen(gateway_id);

		} else {
			LOG_ERR("No GWID set in PSK!");
			ret = -EINVAL;
		}
	}

cleanup:
	err = nrf_close(at_socket_fd);
	if (err != 0) {
		ret = -EIO;
	}
	return ret;
}
#endif

int g2c_send(const struct nrf_cloud_data *output)
{
	struct nrf_cloud_tx_data msg;

	msg.data.ptr = output->ptr;
	msg.data.len = output->len;
	msg.topic_type = NRF_CLOUD_TOPIC_MESSAGE;
	msg.qos = MQTT_QOS_1_AT_LEAST_ONCE;

	return nrf_cloud_send(&msg);
}

int gw_shadow_publish(const struct nrf_cloud_data *output)
{
	struct nrf_cloud_tx_data msg;

	msg.data.ptr = output->ptr;
	msg.data.len = output->len;
	msg.topic_type = NRF_CLOUD_TOPIC_STATE;
	msg.qos = MQTT_QOS_1_AT_LEAST_ONCE;

	return nrf_cloud_send(&msg);
}

void device_shutdown(bool reboot)
{
	struct cloud_backend *backend;
	int err;

	LOG_PANIC();
	if (!reboot) {
		LOG_INF("Shutting down...");
		nrf_gpio_cfg_input(CONFIG_MODEM_WAKEUP_PIN,
				   NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_sense_set(CONFIG_MODEM_WAKEUP_PIN,
				       NRF_GPIO_PIN_SENSE_LOW);
	}

	ui_led_set_pattern(UI_BLE_OFF, PWM_DEV_1);

	LOG_INF("Disconnect from cloud...");
	backend = cloud_get_binding("NRF_CLOUD");
	err = cloud_disconnect(backend);
	if (err) {
		LOG_ERR("Error closing cloud backend: %d",
			err);
	}

	LOG_INF("Power off LTE...");
	err = lte_lc_power_off();
	if (err) {
		LOG_ERR("Error powering off LTE: %d",
			err);
	}

	LOG_INF("Shutdown modem...");
	err = nrf_modem_shutdown();
	if (err) {
		LOG_ERR("Error on bsd_shutdown(): %d",
			err);
	}

	if (reboot) {
		LOG_INF("Rebooting...");
		k_sleep(K_SECONDS(1));

#if defined(CONFIG_REBOOT)
		sys_reboot(SYS_REBOOT_COLD);
#else
		LOG_ERR("sys_reboot not defined: "
			"enable CONFIG_REBOOT and rebuild");
#endif
	} else {
		LOG_INF("Power down.");
		k_sleep(K_SECONDS(1));
		NRF_REGULATORS_NS->SYSTEMOFF = 1;
	}
}


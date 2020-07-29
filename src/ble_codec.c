#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "cJSON.h"
#include "cJSON_os.h"
#include "ble_codec.h"
#include "ble_conn_mgr.h"
#include "ble.h"
#include "nrf_cloud_transport.h"

#define MAX_BUF_SIZE 11000
#define MAX_SERVICE_BUF_SIZE 300

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_codec, CONFIG_APRICITY_GATEWAY_LOG_LEVEL);

extern ble_scanned_devices ble_scanned_device[MAX_SCAN_RESULTS];
extern char gateway_id[10];

char buffer[MAX_BUF_SIZE];
char service_buffer[MAX_SERVICE_BUF_SIZE];

bool first_service = true;
bool first_chrc = true;

/* define macros to enable memory allocation error checking and
 * cleanup, and also improve readability
 */
#define CJCREATE(_a_) do { \
	(_a_) = cJSON_CreateObject(); \
	if ((_a_) == NULL) { \
		goto cleanup; \
	} \
} while(0)

#define CJADD cJSON_AddItemToObject
#define CJADDBOOL cJSON_AddBoolToObject
#define CJADDNUM cJSON_AddNumberToObject
#define CJPRINT cJSON_PrintPreallocated

#define CJADDCHK(_a_, _b_, _c_) do { \
	if ((_c_) == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToObject((_a_), (_b_), (_c_)); \
} while(0)

#define CJADDSTR(_a_, _b_, _c_) do { \
	cJSON *tmp = cJSON_CreateString(_c_); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToObject((_a_), (_b_), tmp); \
} while(0)

#define CJADDBOOLOBJ(_a_, _b_, _c_) do { \
	cJSON *tmp = cJSON_CreateBool(_c_); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToObject((_a_), (_b_), tmp); \
} while(0)

#define CJADDNULL(_a_, _b_) do { \
	cJSON *tmp = cJSON_CreateNull(); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToObject((_a_), (_b_), tmp); \
} while(0)

#define CJADDTRUE(_a_, _b_) do { \
	cJSON *tmp = cJSON_CreateTrue(); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToObject((_a_), (_b_), tmp); \
} while(0)

#define CJADDARROBJ(_a_, _b_) do { \
	(_b_) = cJSON_CreateObject(); \
	if ((_b_) == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToArray((_a_), (_b_)); \
} while(0)

#define CJADDARRNUM(_a_, _b_) do { \
	cJSON *tmp = cJSON_CreateNumber((_b_)); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToArray((_a_), tmp); \
} while(0)

#define CJADDARRSTR(_a_, _b_) do { \
	cJSON *tmp = cJSON_CreateString((_b_)); \
	if (tmp == NULL) { \
		goto cleanup; \
	} \
	cJSON_AddItemToArray((_a_), tmp); \
} while(0)


int device_error_encode(char *ble_address, char *error_msg)
{
	/* TODO: Front end doesn't handle error messages yet.
	 * This format may change.
	 */
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *error = cJSON_CreateObject();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (error == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);

	CJADD(root_obj, "event", event);
	CJADDSTR(root_obj, "timestamp", "2020-02-19T18:38:50.363Z");

	CJADDSTR(event, "type", "error");
	CJADD(event, "device", device);
	CJADD(event, "error", error);

	CJADDSTR(error, "description", error_msg);
	CJADDSTR(device, "deviceAddress", ble_address);

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);
	return 0;

cleanup:
	cJSON_Delete(error);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_found_encode(u8_t num_devices_found)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *devices = cJSON_CreateArray();
	cJSON *device = NULL;
	cJSON *address = NULL;

	if ((root_obj == NULL) || (event == NULL) || (devices == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "scan_result");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADDSTR(event, "subType", "instant");
	CJADD(event, "devices", devices);

	for (int i = 0; i < num_devices_found; i++) {
		LOG_INF("Adding device %s RSSI: %d\n",
		       ble_scanned_device[i].addr, ble_scanned_device[i].rssi);
		CJADDARROBJ(devices, device);
		/* TODO: Update for beacons */
		CJADDSTR(device, "deviceType", "BLE");
		CJCREATE(address);
		CJADD(device, "address", address);
		CJADDSTR(address, "address", ble_scanned_device[i].addr);
		CJADDSTR(address, "type", ble_scanned_device[i].type);

		if (strlen(ble_scanned_device[i].name) > 0) {
			CJADDSTR(device, "name", ble_scanned_device[i].name);
		} else {
			CJADDNULL(device, "name");
		}

		/* TODO: cJSON print numbers doesn't work? */
		CJADDNUM(device, "rssi", ble_scanned_device[i].rssi);
	}
	CJADDTRUE(event, "timeout");
	/* Figure out a messageId:
	 * cJSON_AddItemToObject(root_obj, "messageId", cJSON_CreateNumber(1));
	 */
	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(devices);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_connect_result_encode(char *ble_address, bool conn_status)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *status = cJSON_CreateObject();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (status == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "device_connect_result");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	CJADD(device, "status", status);
	/* TODO: not sure */
	CJADDBOOL(status, "connected", conn_status);

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(status);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_disconnect_result_encode(char *ble_address, bool conn_status)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *status = cJSON_CreateObject();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (status == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "device_disconnect");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	CJADD(device, "status", status);
	/* TODO: not sure */
	CJADDBOOL(status, "connected", conn_status);

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(status);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_value_changed_encode(char *ble_address, char *uuid, char *path,
				 char *value, u16_t value_length)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *chrc = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (chrc == NULL) || (value_arr == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "device_characteristic_value_changed");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	/* TODO: Get Type; */
	CJADDSTR(address, "type", "random");

	CJADD(event, "characteristic", chrc);
	CJADDSTR(chrc, "uuid", uuid);
	CJADDSTR(chrc, "path", path);
	CJADD(chrc, "value", value_arr);

	for (int i = 0; i < value_length; i++) {
		CJADDARRNUM(value_arr, value[i]);
	}

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(value_arr);
	cJSON_Delete(chrc);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_value_write_result_encode(char *ble_address, char *uuid, char *path,
				      char *value, u16_t value_length)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *desc = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (desc == NULL) || (value_arr == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "device_descriptor_value_write_result");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	/* TODO: Get Type; */
	CJADDSTR(address, "type", "random");
	CJADD(event, "descriptor", desc);
	CJADDSTR(desc, "uuid", uuid);
	CJADDSTR(desc, "path", path);
	CJADD(desc, "value", value_arr);

	for (int i = 0; i < value_length; i++) {
		CJADDARRNUM(value_arr, value[i]);
	}

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(value_arr);
	cJSON_Delete(desc);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_descriptor_value_changed_encode(char *ble_address, char *uuid,
					    char *path, char *value,
					    u16_t value_length)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *desc = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (desc == NULL) || (value_arr == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADD(root_obj, "event", event);
	CJADDNULL(root_obj, "requestId");

	CJADDSTR(event, "type", "device_descriptor_value_changed");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	/* TODO: Get Type; */
	CJADDSTR(address, "type", "random");

	CJADD(event, "descriptor", desc);
	CJADDSTR(desc, "uuid", uuid);
	CJADDSTR(desc, "path", path);
	CJADD(desc, "value", value_arr);

	for (int i = 0; i < value_length; i++) {
		CJADDARRNUM(value_arr, value[i]);
	}

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(value_arr);
	cJSON_Delete(desc);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_chrc_read_encode(char *ble_address, char *uuid, char *path,
			     char *value, u16_t value_length)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *chrc = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (chrc == NULL) || (value_arr == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADDNULL(root_obj, "requestId");
	CJADD(root_obj, "event", event);

	CJADDSTR(event, "type", "device_characteristic_value_read_result");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	/* TODO: Get Type; */
	CJADDSTR(address, "type", "random");

	CJADD(event, "characteristic", chrc);
	CJADDSTR(chrc, "uuid", uuid);
	CJADDSTR(chrc, "path", path);
	CJADD(chrc, "value", value_arr);

	for (int i = 0; i < value_length; i++) {
		CJADDARRNUM(value_arr, value[i]);
	}

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	LOG_DBG("Device JSON: %s", buffer);

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(value_arr);
	cJSON_Delete(chrc);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

static int create_device_wrapper(char *ble_address, bool conn_status)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *event = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *address = cJSON_CreateObject();
	cJSON *status = cJSON_CreateObject();
	cJSON *services = cJSON_CreateObject();

	if ((root_obj == NULL) || (event == NULL) || (device == NULL) ||
	    (address == NULL) || (status == NULL) || (services == NULL)) {
		goto cleanup;
	}

	CJADDSTR(root_obj, "type", "event");
	/* TODO: get real gateway */
	CJADDSTR(root_obj, "gatewayId", gateway_id);
	CJADDNULL(root_obj, "requestId");
	CJADD(root_obj, "event", event);

	CJADDSTR(event, "type", "device_discover_result");
	CJADDSTR(event, "timestamp", "2020-02-19T18:38:50.363Z");
	CJADD(event, "device", device);

	CJADDSTR(device, "id", ble_address);
	CJADD(device, "address", address);
	CJADDSTR(address, "address", ble_address);
	CJADD(device, "status", status);
	/* TODO: not sure */
	CJADDBOOL(status, "connected", conn_status);
	CJADD(event, "services", services);

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	cJSON_Delete(root_obj);

	buffer[strlen(buffer) - 3] = 0;

	if (buffer == NULL) {
		LOG_ERR("No memory for JSON");
		return -ENOMEM;
	}

	return 0;

cleanup:
	cJSON_Delete(services);
	cJSON_Delete(status);
	cJSON_Delete(address);
	cJSON_Delete(device);
	cJSON_Delete(event);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

int device_shadow_data_encode(char *ble_address, bool connecting,
			      bool connected)
{
	cJSON *root_obj = cJSON_CreateObject();
	cJSON *state_obj = cJSON_CreateObject();
	cJSON *reported_obj = cJSON_CreateObject();
	cJSON *status_connections = cJSON_CreateObject();
	cJSON *device = cJSON_CreateObject();
	cJSON *status = cJSON_CreateObject();

	cJSON *gateway_device = cJSON_CreateObject();
	cJSON *service_info = cJSON_CreateObject();
	cJSON *fota_arr = cJSON_CreateArray();

	if ((root_obj == NULL) || (state_obj == NULL) ||
	    (reported_obj == NULL) || (status_connections == NULL) ||
	    (device == NULL) || (status == NULL) || (gateway_device == NULL) ||
	    (service_info == NULL) || (fota_arr == NULL)) {
		goto cleanup;
	}

	CJADD(root_obj, "state", state_obj);
	CJADD(state_obj, "reported", reported_obj);

	CJADD(reported_obj, "device", gateway_device);
	CJADD(gateway_device, "serviceInfo", service_info);
	CJADD(service_info, "fota_v1", fota_arr);
	CJADDARRSTR(fota_arr, "APP");
	CJADDARRSTR(fota_arr, "MODEM");
	CJADDARRSTR(fota_arr, "BOOT");

	CJADD(reported_obj, "statusConnections", status_connections);
	CJADD(status_connections, ble_address, device);
	CJADDSTR(device, "id", ble_address);
	CJADD(device, "status", status);
	CJADDBOOLOBJ(status, "connected", connected);
	CJADDBOOLOBJ(status, "connecting", connecting);

	CJPRINT(root_obj, buffer, MAX_BUF_SIZE, 0);

	/* TODO: Move out of decode. */
	shadow_publish(buffer);

	cJSON_Delete(root_obj);

	return 0;

cleanup:
	cJSON_Delete(fota_arr);
	cJSON_Delete(service_info);
	cJSON_Delete(gateway_device);
	cJSON_Delete(status);
	cJSON_Delete(device);
	cJSON_Delete(status_connections);
	cJSON_Delete(reported_obj);
	cJSON_Delete(state_obj);
	cJSON_Delete(root_obj);
	return -ENOMEM;
}

/* Start of functions to assemble large JSON string. No room to create these
 * all at once in cJSON
 */
static int device_discover_add_service(char *discovered_json)
{
	if (first_service == false) {
		strcat(buffer, "}},");
	}

	strcat(buffer, discovered_json + 1);

	if (strlen(buffer) >= 3) {
		buffer[strlen(buffer) - 3] = 0;
	}

	LOG_INF("JSON Size: %d", strlen(buffer));

	memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);
	first_service = false;
	first_chrc = true;
	return 0;
}

static int device_discover_add_chrc(char *discovered_json)
{
	if (first_chrc == false) {
		strcat(buffer, ",");
	}

	strcat(buffer, discovered_json + 1);

	if (strlen(buffer) >= 1) {
		buffer[strlen(buffer) - 1] = 0;
	}

	LOG_DBG("JSON Size: %d", strlen(buffer));

	memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);

	first_chrc = false;
	return 0;
}

static int device_discover_add_ccc(char *discovered_json)
{
	if (strlen(buffer) >= 2) {
		buffer[strlen(buffer) - 2] = 0;
	}

	strcat(buffer, discovered_json + 1);
	strcat(buffer, "}");

	LOG_DBG("JSON Size: %d", strlen(buffer));

	memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);

	first_chrc = false;
	return 0;
}

int device_discovery_send()
{
	/* Add the remaing brackets to the JSON string that was assembled. */
	strcat(buffer, "}}}}}");

	LOG_DBG("JSON Size: %d", strlen(buffer));

	/* TODO: Move out of decode. */
	g2c_send(buffer);

	memset(buffer, 0, MAX_BUF_SIZE);
	memset(service_buffer, 0, MAX_SERVICE_BUF_SIZE);

	return 0;
}

static int svc_attr_encode(char *uuid, char *path,
			    connected_ble_devices *ble_conn_ptr)
{
	cJSON *services = cJSON_CreateObject();
	cJSON *service = cJSON_CreateObject();
	cJSON *chrcs = cJSON_CreateObject();

	if ((services == NULL) || (service == NULL) || (chrcs == NULL)) {
		goto cleanup;
	}

	CJADD(services, uuid, service);
	CJADDSTR(service, "uuid", uuid);
	CJADD(service, "characteristics", chrcs);

	/* Print services and add to wrapper */
	CJPRINT(services, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
	LOG_DBG("JSON: %s", service_buffer);
	cJSON_Delete(services);
	return device_discover_add_service(service_buffer);

cleanup:
	cJSON_Delete(chrcs);
	cJSON_Delete(service);
	cJSON_Delete(services);
	return -ENOMEM;
}

static int chrc_attr_encode(char *uuid, char *path, u8_t properties,
			     connected_ble_devices *ble_conn_ptr)
{
	cJSON *chrc = cJSON_CreateObject();
	cJSON *descriptors = cJSON_CreateObject();
	cJSON *props = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();
	cJSON *parent_chrc = cJSON_CreateObject();

	if ((chrc == NULL) || (descriptors == NULL) || (props == NULL) ||
	    (value_arr == NULL) || (parent_chrc == NULL)) {
		goto cleanup;
	}

	CJADD(parent_chrc, uuid, chrc);
	CJADDSTR(chrc, "uuid", uuid);
	CJADDSTR(chrc, "path", path);
	CJADD(chrc, "value", value_arr);

	CJADDARRNUM(value_arr, 0);

	CJADD(chrc, "properties", props);

	/* Check and add properties */
	switch (properties) {

	case BT_GATT_CHRC_READ:
		CJADDTRUE(props, "read");
		break;

	case BT_GATT_CHRC_READ + BT_GATT_CHRC_INDICATE:
		CJADDTRUE(props, "read");
		CJADDTRUE(props, "indicate");
		break;

	case BT_GATT_CHRC_READ + BT_GATT_CHRC_NOTIFY:
		CJADDTRUE(props, "read");
		CJADDTRUE(props, "notify");
		break;

	case BT_GATT_CHRC_WRITE:
		CJADDTRUE(props, "write");
		break;

	case BT_GATT_CHRC_WRITE_WITHOUT_RESP:
		CJADDTRUE(props, "writeWithoutResponse");
		break;

	case BT_GATT_CHRC_READ + BT_GATT_CHRC_WRITE:
		CJADDTRUE(props, "read");
		CJADDTRUE(props, "write");
		break;

	case BT_GATT_CHRC_READ + BT_GATT_CHRC_WRITE_WITHOUT_RESP:
		CJADDTRUE(props, "read");
		CJADDTRUE(props, "writeWithoutResponse");
		break;

	case BT_GATT_CHRC_INDICATE:
		CJADDTRUE(props, "indicate");
		break;

	case BT_GATT_CHRC_NOTIFY:
		CJADDTRUE(props, "notify");
		break;

	case BT_GATT_CHRC_INDICATE + BT_GATT_CHRC_WRITE:
		CJADDTRUE(props, "indicate");
		CJADDTRUE(props, "write");
		break;

	case BT_GATT_CHRC_INDICATE + BT_GATT_CHRC_NOTIFY:
		CJADDTRUE(props, "indicate");
		CJADDTRUE(props, "notify");
		break;

	case BT_GATT_CHRC_AUTH:
		CJADDTRUE(props, "authorizedSignedWrite");
		break;

	default:
		LOG_ERR("Unknown CHRC property: %d\n", properties);
		break;
	}

	CJADD(chrc, "descriptors", descriptors);

	/* Print parent_chrhc and add to service */
	CJPRINT(parent_chrc, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
	LOG_DBG("JSON: %s", service_buffer);
	cJSON_Delete(parent_chrc);
	return device_discover_add_chrc(service_buffer);

cleanup:
	cJSON_Delete(parent_chrc);
	cJSON_Delete(value_arr);
	cJSON_Delete(props);
	cJSON_Delete(descriptors);
	cJSON_Delete(chrc);
	return -ENOMEM;
}

static int ccc_attr_encode(char *uuid, char *path,
			    connected_ble_devices *ble_conn_ptr)
{
	cJSON *descriptor = cJSON_CreateObject();
	cJSON *value_arr = cJSON_CreateArray();
	cJSON *parent_ccc =  cJSON_CreateObject();

	if ((descriptor == NULL) || (value_arr == NULL) ||
	     (parent_ccc == NULL)) {
		goto cleanup;
	}

	CJADD(parent_ccc, uuid, descriptor);
	CJADDSTR(descriptor, "uuid", uuid);
	CJADD(descriptor, "value", value_arr);
	CJADDARRNUM(value_arr, 0);
	CJADDARRNUM(value_arr, 0);
	CJADDSTR(descriptor, "path", path);

	/* Print parent_ccc and add to service */
	CJPRINT(parent_ccc, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
	LOG_DBG("JSON: %s", service_buffer);
	cJSON_Delete(parent_ccc);
	return device_discover_add_ccc(service_buffer);

cleanup:
	cJSON_Delete(parent_ccc);
	cJSON_Delete(value_arr);
	cJSON_Delete(descriptor);
	return -ENOMEM;
}

static int attr_encode(u16_t attr_type, char* uuid_str, char* path,
			u8_t properties, connected_ble_devices* ble_conn_ptr)
{
	int ret = 0;

        bt_to_upper(uuid_str, strlen(uuid_str));
        bt_to_upper(path, strlen(path));

        if (attr_type ==  BT_ATTR_SERVICE) {
                LOG_INF("Encoding Service : UUID: %s", log_strdup(uuid_str));
                ret = svc_attr_encode(uuid_str, path, ble_conn_ptr);

        } else if (attr_type == BT_ATTR_CHRC) {
                LOG_INF("Encoding Characteristic : UUID: %s  PATH: %s",
			log_strdup(uuid_str), path);
                ret = chrc_attr_encode(uuid_str, path, properties,
				       ble_conn_ptr);

        } else if (attr_type == BT_ATTR_CCC) {
                LOG_INF("Encoding CCC : UUID: %s  PATH: %s",
			log_strdup(uuid_str), path);
                ret = ccc_attr_encode(uuid_str, path, ble_conn_ptr);
        } else {
                LOG_ERR("Unknown Attr Type");
		ret = -EINVAL;
        }
	return ret;
}

static void get_uuid_str(uuid_handle_pairs *uuid_handle, char *str)
{
	size_t len = BT_MAX_UUID_LEN;
	struct bt_uuid *uuid;

	if (uuid_handle->uuid_type == BT_UUID_TYPE_16) {
		uuid = &uuid_handle->uuid_16.uuid;
	} else if (uuid_handle->uuid_type == BT_UUID_TYPE_128) {
		uuid = &uuid_handle->uuid_128.uuid;
	} else {
		str[0] = '\0';
		return;
	}
	bt_uuid_get_str(uuid, str, len);
}

int device_discovery_encode(connected_ble_devices *conn_ptr)
{
	char uuid_str[BT_MAX_UUID_LEN];
	char service_attr_str[BT_MAX_UUID_LEN];
	char path_dep_two_str[BT_MAX_UUID_LEN];
	char path_str[BT_MAX_PATH_LEN];
	int ret = 0;
	u8_t num_encoded = 0;

	first_service = true;

	LOG_INF("Num Pairs: %d", conn_ptr->num_pairs);

	ret = create_device_wrapper(conn_ptr->addr, true);
	if (ret) {
		return ret;
	}

	for (int i = 0; i < conn_ptr->num_pairs; i++) {
		struct uuid_handle_pairs *uuid_handle;
		struct uuid_handle_pairs *uh;

		uuid_handle = &conn_ptr->uuid_handle_pair[i];

		if ((uuid_handle->uuid_type != BT_UUID_TYPE_16) &&
		    (uuid_handle->uuid_type != BT_UUID_TYPE_128)) {
			LOG_ERR("ERROR: unknown UUID type");
			ret = -EINVAL;
			continue;
		}
		get_uuid_str(uuid_handle, uuid_str);

		switch (uuid_handle->path_depth) {
		case 1:
			for (int j = i; j >= 0; j--) {
				uh = &conn_ptr->uuid_handle_pair[j];
				if (uh->is_service == true) {
					get_uuid_str(uh, service_attr_str);
					break;
				}
			}
			snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s",
				 service_attr_str, uuid_str);
			break;
		case 2:
			uh = &conn_ptr->uuid_handle_pair[i - 1];
			get_uuid_str(uh, path_dep_two_str);
			snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s/%s",
				 service_attr_str, path_dep_two_str, uuid_str);
			break;
		}

		ret = attr_encode(uuid_handle->attr_type, uuid_str, path_str,
			    uuid_handle->properties, conn_ptr);
		if (!ret) {
			num_encoded++;
		}
	}

	/* make sure we output at least one attribute, or
	 * device_discovery_send() will send malformed JSON
	 */
	if (num_encoded) {
		device_discovery_send();
		ret = 0; /* ignore invalid UUID type since others were ok */
	}
	return ret;
}
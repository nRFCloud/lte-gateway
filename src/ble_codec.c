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


u8_t device_error_encode(char* ble_address, char* error_msg)
{

        //TODO: Front end doesn't handle error messages yet. This format may change.
        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *error = cJSON_CreateObject();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("error"));
        cJSON_AddItemToObject(event, "error", error);
        cJSON_AddItemToObject(error, "description", cJSON_CreateString(error_msg));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "deviceAddress", cJSON_CreateString(ble_address));


        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_found_encode(u8_t num_devices_found)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *devices = cJSON_CreateArray();
        cJSON *device = NULL;
        cJSON *address = NULL;
        cJSON *rssi_json = NULL;

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("scan_result"));
        cJSON_AddItemToObject(event, "subType", cJSON_CreateString("instant"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "devices", devices);

        for(int i =0; i<num_devices_found; i++)
        {
                printk("Adding device %s RSSI: %d\n", ble_scanned_device[i].addr, ble_scanned_device[i].rssi);

                cJSON_AddItemToArray(devices, device = cJSON_CreateObject());
                cJSON_AddItemToObject(device, "deviceType", cJSON_CreateString("BLE")); //TODO: Update for beacons
                cJSON_AddItemToObject(device, "address", address = cJSON_CreateObject());
                cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_scanned_device[i].addr));
                cJSON_AddItemToObject(address, "type", cJSON_CreateString(ble_scanned_device[i].type));

                if(strlen(ble_scanned_device[i].name) >0)
                {
                        cJSON_AddItemToObject(device, "name", cJSON_CreateString(ble_scanned_device[i].name));
                }
                else
                {
                        cJSON_AddItemToObject(device, "name", cJSON_CreateNull());
                }

                //cJSON_AddNumberToObject(device, "rssi", ble_scanned_device[i].rssi);   TODO: cJSON print numbers doesn't work!

        }
        cJSON_AddItemToObject(event, "timeout", cJSON_CreateTrue());
        //cJSON_AddItemToObject(root_obj, "messageId", cJSON_CreateNumber(1));  //Figure out a messageId

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_connect_result_encode(char* ble_address, bool conn_status)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *status = cJSON_CreateObject();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_connect_result"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "status", status);
        cJSON_AddBoolToObject(status, "connected", conn_status); //TODO: not sure

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_disconnect_result_encode(char* ble_address, bool conn_status)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *status = cJSON_CreateObject();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_disconnect"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "status", status);
        cJSON_AddBoolToObject(status, "connected", conn_status); //TODO: not sure

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_value_changed_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *chrc = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_characteristic_value_changed"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(address, "type", cJSON_CreateString("random")); //TODO: Get Type;

        cJSON_AddItemToObject(event, "characteristic", chrc);
        cJSON_AddItemToObject(chrc, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(chrc, "path", cJSON_CreateString(path));
        cJSON_AddItemToObject(chrc, "value", value_arr);

        for(int i = 0; i<value_length; i++)
        {
                cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(value[i]));
        }

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_value_write_result_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *desc = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_descriptor_value_write_result"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(address, "type", cJSON_CreateString("random")); //TODO: Get Type;

        cJSON_AddItemToObject(event, "descriptor", desc);
        cJSON_AddItemToObject(desc, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(desc, "path", cJSON_CreateString(path));
        cJSON_AddItemToObject(desc, "value", value_arr);

        for(int i = 0; i<value_length; i++)
        {
                cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(value[i]));
        }

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_descriptor_value_changed_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *desc = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_descriptor_value_changed"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(address, "type", cJSON_CreateString("random")); //TODO: Get Type;

        cJSON_AddItemToObject(event, "descriptor", desc);
        cJSON_AddItemToObject(desc, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(desc, "path", cJSON_CreateString(path));
        cJSON_AddItemToObject(desc, "value", value_arr);

        for(int i = 0; i<value_length; i++)
        {
                cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(value[i]));
        }

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

u8_t device_chrc_read_encode(char* ble_address, char* uuid, char* path, char* value, u16_t value_length)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *chrc = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();

        if (root_obj == NULL) {
                return -ENOMEM;
        }

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_characteristic_value_read_result"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(address, "type", cJSON_CreateString("random")); //TODO: Get Type;

        cJSON_AddItemToObject(event, "characteristic", chrc);
        cJSON_AddItemToObject(chrc, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(chrc, "path", cJSON_CreateString(path));
        cJSON_AddItemToObject(chrc, "value", value_arr);

        for(int i = 0; i<value_length; i++)
        {
                cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(value[i]));
        }

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        LOG_DBG("Device JSON: %s", buffer);

        g2c_send(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}



u8_t create_device_wrapper(char* ble_address, bool conn_status)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *event = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *address = cJSON_CreateObject();
        cJSON *status = cJSON_CreateObject();
        cJSON *services = cJSON_CreateObject();

        cJSON_AddItemToObject(root_obj, "type", cJSON_CreateString("event"));
        cJSON_AddItemToObject(root_obj, "gatewayId", cJSON_CreateString(gateway_id)); //TODO: get real gateway
        cJSON_AddItemToObject(root_obj, "requestId", cJSON_CreateNull());
        cJSON_AddItemToObject(root_obj, "event", event);
        cJSON_AddItemToObject(event, "type", cJSON_CreateString("device_discover_result"));
        cJSON_AddItemToObject(event, "timestamp", cJSON_CreateString("2020-02-19T18:38:50.363Z"));
        cJSON_AddItemToObject(event, "device", device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "address", address);
        cJSON_AddItemToObject(address, "address", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "status", status);
        cJSON_AddBoolToObject(status, "connected", conn_status); //TODO: not sure
        cJSON_AddItemToObject(event, "services", services);

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        cJSON_Delete(root_obj);

        buffer[strlen(buffer)-3] = 0;

        if (buffer == NULL) {
                LOG_ERR("No memory for JSON");
                return -ENOMEM;
        }

        return 0;

}

int device_shadow_data_encode(char* ble_address, bool connecting, bool connected)
{

        cJSON *root_obj = cJSON_CreateObject();
        cJSON *state_obj = cJSON_CreateObject();
        cJSON *reported_obj = cJSON_CreateObject();
        cJSON *status_connections = cJSON_CreateObject();
        cJSON *device = cJSON_CreateObject();
        cJSON *status = cJSON_CreateObject();

        cJSON *gateway_device = cJSON_CreateObject();
        cJSON *service_info = cJSON_CreateObject();
        //cJSON *fota_version = cJSON_CreateObject();
        cJSON *fota_arr = cJSON_CreateArray();

        cJSON_AddItemToObject(root_obj, "state", state_obj);
        cJSON_AddItemToObject(state_obj, "reported", reported_obj);

        cJSON_AddItemToObject(reported_obj, "device", gateway_device);
        cJSON_AddItemToObject(gateway_device, "serviceInfo", service_info);
        cJSON_AddItemToObject(service_info, "fota_v1", fota_arr);
        cJSON_AddItemToArray(fota_arr, cJSON_CreateString("APP"));
        cJSON_AddItemToArray(fota_arr, cJSON_CreateString("MODEM"));
        cJSON_AddItemToArray(fota_arr, cJSON_CreateString("BOOT"));

        cJSON_AddItemToObject(reported_obj, "statusConnections", status_connections);
        cJSON_AddItemToObject(status_connections, ble_address, device);
        cJSON_AddItemToObject(device, "id", cJSON_CreateString(ble_address));
        cJSON_AddItemToObject(device, "status", status);
        cJSON_AddItemToObject(status, "connected", cJSON_CreateBool(connected));
        cJSON_AddItemToObject(status, "connecting", cJSON_CreateBool(connecting));

        cJSON_PrintPreallocated(root_obj, buffer, MAX_BUF_SIZE, 0);

        shadow_publish(buffer);                   //TODO: Move out of decode.

        cJSON_Delete(root_obj);

        return 0;

}

/*Start of functions to assemble large JSON string. No room to create these all at once in cJSON*/
u8_t device_discover_add_service(char *discovered_json)
{

        if(first_service == false)
        {
                strcat(buffer, "}},");
        }

        strcat(buffer, discovered_json+1);

        if(strlen(buffer) >=3)
        {
          buffer[strlen(buffer)-3] = 0;
        }

        LOG_INF("JSON Size: %d", strlen(buffer));

        memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);
        first_service = false;
        first_chrc = true;
        return 0;

}

u8_t device_discover_add_chrc(char *discovered_json)
{

        if(first_chrc == false)
        {
                strcat(buffer, ",");
        }

        strcat(buffer, discovered_json+1);

        if(strlen(buffer) >=1)
        {
          buffer[strlen(buffer)-1] = 0;
        }

        LOG_DBG("JSON Size: %d", strlen(buffer));

        memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);

        first_chrc = false;
        return 0;

}

u8_t device_discover_add_ccc(char *discovered_json)
{

        if(strlen(buffer) >=2)
        {
          buffer[strlen(buffer)-2] = 0;
        }

        strcat(buffer, discovered_json+1);

        strcat(buffer, "}");

        LOG_DBG("JSON Size: %d", strlen(buffer));

        memset(discovered_json, 0, MAX_SERVICE_BUF_SIZE);

        first_chrc = false;
        return 0;

}

u8_t device_discovery_send()
{

        //Add the remaing brackets to the JSON string that was assembled.
        strcat(buffer, "}}}}}");

        LOG_DBG("JSON Size: %d", strlen(buffer));

        g2c_send(buffer);                   //TODO: Move out of decode.

        memset(buffer, 0, MAX_BUF_SIZE);
        memset(service_buffer, 0, MAX_SERVICE_BUF_SIZE);

        return 0;

}

static void svc_attr_encode( char* uuid, char* path, connected_ble_devices* ble_conn_ptr)
{

        cJSON * services = cJSON_CreateObject();
        cJSON * service = cJSON_CreateObject();
        cJSON * chrcs = cJSON_CreateObject();

        cJSON_AddItemToObject(services, uuid, service);
        cJSON_AddItemToObject(service, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(service, "characteristics", chrcs);

        //Print services and add to wrapper
        cJSON_PrintPreallocated(services, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
        LOG_DBG("JSON: %s", service_buffer);
        cJSON_Delete(services);
        device_discover_add_service(service_buffer);

}



static void chrc_attr_encode(char* uuid, char* path, u8_t properties, connected_ble_devices* ble_conn_ptr)
{

        cJSON *chrc = cJSON_CreateObject();
        cJSON *descriptors = cJSON_CreateObject();
        cJSON *props = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();
        cJSON * parent_chrc = cJSON_CreateObject();

        cJSON_AddItemToObject(parent_chrc, uuid, chrc);
        cJSON_AddItemToObject(chrc, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(chrc, "path", cJSON_CreateString(path));
        cJSON_AddItemToObject(chrc, "value", value_arr);

        cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(0));

        cJSON_AddItemToObject(chrc, "properties", props);

        //Check and add properties
        switch(properties)
        {

        case BT_GATT_CHRC_READ:
                cJSON_AddItemToObject(props, "read", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_READ+BT_GATT_CHRC_INDICATE:
                cJSON_AddItemToObject(props, "read", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "indicate", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_READ+BT_GATT_CHRC_NOTIFY:
                cJSON_AddItemToObject(props, "read", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "notify", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_WRITE:
                cJSON_AddItemToObject(props, "write", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_WRITE_WITHOUT_RESP:
                cJSON_AddItemToObject(props, "writeWithoutResponse", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_READ+BT_GATT_CHRC_WRITE:
                cJSON_AddItemToObject(props, "read", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "write", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_READ+BT_GATT_CHRC_WRITE_WITHOUT_RESP:
                cJSON_AddItemToObject(props, "read", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "writeWithoutResponse", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_INDICATE:
                cJSON_AddItemToObject(props, "indicate", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_NOTIFY:
                cJSON_AddItemToObject(props, "notify", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_INDICATE+BT_GATT_CHRC_WRITE:
                cJSON_AddItemToObject(props, "indicate", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "write", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_INDICATE+BT_GATT_CHRC_NOTIFY:
                cJSON_AddItemToObject(props, "indicate", cJSON_CreateTrue());
                cJSON_AddItemToObject(props, "notify", cJSON_CreateTrue());
                break;

        case BT_GATT_CHRC_AUTH:
                cJSON_AddItemToObject(props, "authorizedSignedWrite", cJSON_CreateTrue());
                break;

        default:
                LOG_ERR("Unknown CHRC property: %d\n", properties);
                break;

        }

        cJSON_AddItemToObject(chrc, "descriptors", descriptors);

        //Print parent_chrhc and add to service
        cJSON_PrintPreallocated(parent_chrc, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
        LOG_DBG("JSON: %s", service_buffer);
        cJSON_Delete(parent_chrc);
        device_discover_add_chrc(service_buffer);
}



static void ccc_attr_encode(char* uuid, char* path, connected_ble_devices* ble_conn_ptr)
{

        cJSON *descriptor = cJSON_CreateObject();
        cJSON *value_arr = cJSON_CreateArray();
        cJSON *parent_ccc =  cJSON_CreateObject();

        cJSON_AddItemToObject(parent_ccc, uuid, descriptor);
        cJSON_AddItemToObject(descriptor, "uuid", cJSON_CreateString(uuid));
        cJSON_AddItemToObject(descriptor, "value", value_arr);
        cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(0));
        cJSON_AddItemToArray(value_arr, cJSON_CreateNumber(0));
        cJSON_AddItemToObject(descriptor, "path", cJSON_CreateString(path));


        //Print parent_ccc and add to service
        cJSON_PrintPreallocated(parent_ccc, service_buffer, MAX_SERVICE_BUF_SIZE, 0);
        LOG_DBG("JSON: %s", service_buffer);
        cJSON_Delete(parent_ccc);
        device_discover_add_ccc(service_buffer);

}


static void attr_encode( u16_t attr_type, char* uuid_str, char* path, u8_t properties,
                         connected_ble_devices* ble_conn_ptr )
{

        bt_to_upper(uuid_str, strlen(uuid_str));
        bt_to_upper(path, strlen(path));

        if (attr_type ==  BT_ATTR_SERVICE) {
                LOG_INF("Encoding Service : UUID: %s", uuid_str);
                svc_attr_encode(uuid_str, path, ble_conn_ptr);

        } else if (attr_type == BT_ATTR_CHRC) {
                LOG_INF("Encoding Characteristic : UUID: %s  PATH: %s", uuid_str, path);
                chrc_attr_encode(uuid_str, path, properties, ble_conn_ptr);

        } else if (attr_type == BT_ATTR_CCC) {
                LOG_INF("Encoding CCC : UUID: %s  PATH: %s", uuid_str, path);
                ccc_attr_encode(uuid_str, path, ble_conn_ptr);
        }
        else{
                LOG_ERR("Unknown Attr Type");
        }

}


u8_t device_discovery_encode(connected_ble_devices* conn_ptr)
{

        char uuid_str[BT_MAX_UUID_LEN];
        char service_attr_str[BT_MAX_UUID_LEN];
        char path_dep_two_str[BT_MAX_UUID_LEN];
        char path_str[BT_MAX_PATH_LEN];

        first_service = true;

        LOG_INF("Num Pairs: %d", conn_ptr->num_pairs);

        create_device_wrapper(conn_ptr->addr, true);

        for(int i = 0; i<conn_ptr->num_pairs; i++)
        {
                switch (conn_ptr->uuid_handle_pair[i].uuid_type) {

                case BT_UUID_TYPE_16:

                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i].uuid_16.uuid, uuid_str, BT_MAX_UUID_LEN);
                        switch(conn_ptr->uuid_handle_pair[i].path_depth)
                        {

                        case 1:

                                for(int j = i; j>=0; j--)
                                {
                                        if(conn_ptr->uuid_handle_pair[j].is_service == true)
                                        {
                                                if(conn_ptr->uuid_handle_pair[j].uuid_type == BT_UUID_TYPE_16)
                                                {
                                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[j].uuid_16.uuid, service_attr_str, BT_MAX_UUID_LEN);
                                                }
                                                else
                                                {
                                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[j].uuid_128.uuid, service_attr_str, BT_MAX_UUID_LEN);
                                                }
                                                break;
                                        }
                                }

                                snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s", service_attr_str, uuid_str);
                                break;

                        case 2:

                                if(conn_ptr->uuid_handle_pair[i-1].uuid_type == BT_UUID_TYPE_16)
                                {
                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i-1].uuid_16.uuid, path_dep_two_str, BT_MAX_UUID_LEN);
                                }
                                else
                                {
                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i-1].uuid_128.uuid, path_dep_two_str, BT_MAX_UUID_LEN);
                                }

                                snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s/%s", service_attr_str, path_dep_two_str, uuid_str);
                                break;
                        }

                        attr_encode( conn_ptr->uuid_handle_pair[i].attr_type, uuid_str, path_str, conn_ptr->uuid_handle_pair[i].properties, conn_ptr );

                        break;

                case BT_UUID_TYPE_128:

                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i].uuid_128.uuid, uuid_str, BT_MAX_UUID_LEN);

                        switch(conn_ptr->uuid_handle_pair[i].path_depth)
                        {
                        case 1:

                                for(int j = i; j>=0; j--)
                                {
                                        if(conn_ptr->uuid_handle_pair[j].is_service == true)
                                        {
                                                if(conn_ptr->uuid_handle_pair[j].uuid_type == BT_UUID_TYPE_16)
                                                {
                                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[j].uuid_16.uuid, service_attr_str, BT_MAX_UUID_LEN);
                                                }
                                                else
                                                {
                                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[j].uuid_128.uuid, service_attr_str, BT_MAX_UUID_LEN);
                                                }
                                                break;
                                        }
                                }

                                snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s", service_attr_str, uuid_str);

                                break;

                        case 2:

                                if(conn_ptr->uuid_handle_pair[i-1].uuid_type == BT_UUID_TYPE_16)
                                {
                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i-1].uuid_16.uuid, path_dep_two_str, BT_MAX_UUID_LEN);
                                }
                                else
                                {
                                        bt_uuid_get_str(&conn_ptr->uuid_handle_pair[i-1].uuid_128.uuid, path_dep_two_str, BT_MAX_UUID_LEN);
                                }

                                snprintk(path_str, BT_MAX_PATH_LEN, "%s/%s/%s", service_attr_str, path_dep_two_str, uuid_str);

                                break;
                        }

                        attr_encode( conn_ptr->uuid_handle_pair[i].attr_type, uuid_str, path_str, conn_ptr->uuid_handle_pair[i].properties, conn_ptr );

                        break;

                default:

                        LOG_ERR("ERROR: unknown UUID type");
                        break;
                }
        }

        device_discovery_send();

        return 1;

}

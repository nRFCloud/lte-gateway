/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <kernel_structs.h>
#include <stdio.h>
#include <string.h>
#include <device.h>
#include <drivers/uart.h>
#include <drivers/sensor.h>
#include <console/console.h>
#include <power/reboot.h>
#include <logging/log_ctrl.h>
#if defined(CONFIG_NRF_MODEM_LIB)
#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#endif /* CONFIG_NRF_MODEM_LIB */
#include <net/cloud.h>
#include <net/socket.h>
#include <net/nrf_cloud.h>
#undef __XSI_VISIBLE
#define __XSI_VISIBLE 1
#include <time.h>
#include <posix/time.h>
#include <fw_info.h>
#include <settings/settings.h>
#include <debug/cpu_load.h>

#if defined(CONFIG_LWM2M_CARRIER)
#include <lwm2m_carrier.h>
#endif

#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#include <dfu/mcuboot.h>
#endif


#include "ui.h"
#include <modem/at_cmd.h>
#include "nrf_cloud_transport.h"
#include "watchdog.h"
#include "ble_conn_mgr.h"
#include "ble.h"
#include "config.h"
#include "gateway.h"
#include "peripheral_dfu.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(nrf_cloud_gateway, CONFIG_NRF_CLOUD_GATEWAY_LOG_LEVEL);

#define CALIBRATION_PRESS_DURATION  K_SECONDS(5)
#define CLOUD_CONNACK_WAIT_DURATION (CONFIG_CLOUD_WAIT_DURATION * MSEC_PER_SEC)

#ifdef CONFIG_ACCEL_USE_SIM
#define FLIP_INPUT			CONFIG_FLIP_INPUT
#define CALIBRATION_INPUT		-1
#else
#define FLIP_INPUT			-1
#ifdef CONFIG_ACCEL_CALIBRATE
#define CALIBRATION_INPUT		CONFIG_CALIBRATION_INPUT
#else
#define CALIBRATION_INPUT		-1
#endif /* CONFIG_ACCEL_CALIBRATE */
#endif /* CONFIG_ACCEL_USE_SIM */

#if defined(CONFIG_NRF_MODEM_LIB) && \
!defined(CONFIG_LTE_LINK_CONTROL)
#error "Missing CONFIG_LTE_LINK_CONTROL"
#endif

#if defined(CONFIG_NRF_MODEM_LIB) && \
defined(CONFIG_LTE_AUTO_INIT_AND_CONNECT) && \
defined(CONFIG_NRF_CLOUD_PROVISION_CERTIFICATES)
#error "PROVISION_CERTIFICATES \
	requires CONFIG_LTE_AUTO_INIT_AND_CONNECT to be disabled!"
#endif

#define CLOUD_LED_ON_STR "{\"led\":\"on\"}"
#define CLOUD_LED_OFF_STR "{\"led\":\"off\"}"
#define CLOUD_LED_MSK UI_LED_1

/* Interval in milliseconds after which the device will reboot
 * if the disconnect event has not been handled.
 */
#define REBOOT_AFTER_DISCONNECT_WAIT_MS     (15 * MSEC_PER_SEC)

/* Interval in milliseconds after which the device will
 * disconnect and reconnect if association was not completed.
 */
#define CONN_CYCLE_AFTER_ASSOCIATION_REQ_MS K_MINUTES(5)

/* Wait forever for nrf52840 to reboot after user updates it over USB */
#define WAIT_BOOT_TIMEOUT_MS -1

/* uncomment below to help with diagnosing UART DTS issues */
/* #define DEBUG_UART_PINS */
#if defined(DEBUG_UART_PINS)
#define UART0 DT_NODELABEL(uart0)
#define UART0_TX DT_PROP(UART0, tx_pin)
#define UART0_RX DT_PROP(UART0, rx_pin)
#define UART0_RTS DT_PROP(UART0, rts_pin)
#define UART0_CTS DT_PROP(UART0, cts_pin)
#define UART0_SPEED DT_PROP(UART0, current_speed)
#define UART1 DT_NODELABEL(uart1)
#define UART1_TX DT_PROP(UART1, tx_pin)
#define UART1_RX DT_PROP(UART1, rx_pin)
#define UART1_RTS DT_PROP(UART1, rts_pin)
#define UART1_CTS DT_PROP(UART1, cts_pin)
#define UART1_SPEED DT_PROP(UART1, current_speed)
#endif

struct rsrp_data {
	uint16_t value;
	uint16_t offset;
};

/* Stack definition for application workqueue */
K_THREAD_STACK_DEFINE(application_stack_area,
		      CONFIG_APPLICATION_WORKQUEUE_STACK_SIZE);
static struct k_work_q application_work_q;
static struct cloud_backend *cloud_backend;

/* Sensor data */

static atomic_t carrier_requested_disconnect;
static atomic_t cloud_connect_attempts;

/* Variable to keep track of nRF cloud association state. */
enum cloud_association_state {
	CLOUD_ASSOCIATION_STATE_INIT,
	CLOUD_ASSOCIATION_STATE_REQUESTED,
	CLOUD_ASSOCIATION_STATE_PAIRED,
	CLOUD_ASSOCIATION_STATE_RECONNECT,
	CLOUD_ASSOCIATION_STATE_READY,
};
static atomic_val_t cloud_association =
	ATOMIC_INIT(CLOUD_ASSOCIATION_STATE_INIT);

/* Structures for work */
static struct k_delayed_work cloud_reboot_work;
static struct k_delayed_work cycle_cloud_connection_work;
static struct k_delayed_work cloud_connect_work;
static struct k_work no_sim_go_offline_work;

#if defined(CONFIG_AT_CMD)
#define MODEM_AT_CMD_BUFFER_LEN (CONFIG_AT_CMD_RESPONSE_MAX_LEN + 1)
#else
#define MODEM_AT_CMD_NOT_ENABLED_STR "Error: AT Command driver is not enabled"
#define MODEM_AT_CMD_BUFFER_LEN (sizeof(MODEM_AT_CMD_NOT_ENABLED_STR))
#endif
#define MODEM_AT_CMD_RESP_TOO_BIG_STR "Error: AT Command response is too large to be sent"
#define MODEM_AT_CMD_MAX_RESPONSE_LEN (2000)
static K_SEM_DEFINE(modem_at_cmd_sem, 1, 1);
static K_SEM_DEFINE(cloud_disconnected, 0, 1);
#if defined(CONFIG_LWM2M_CARRIER)
static void app_disconnect(void);
static K_SEM_DEFINE(nrf_modem_initialized, 0, 1);
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(cloud_ready_to_connect, 0, 1);
#endif
static bool cloud_connection_enabled = true;
static bool cloud_connection_status;
static bool lte_connection_status;

#ifdef CONFIG_MODEM_INFO
struct modem_param_info modem_param;
#endif

enum error_type {
	ERROR_BLE,
	ERROR_CLOUD,
	ERROR_MODEM_RECOVERABLE,
	ERROR_MODEM_IRRECOVERABLE,
	ERROR_LTE_LC,
	ERROR_SYSTEM_FAULT
};

/* Forward declaration of functions */

static void work_init(void);
static void cycle_cloud_connection(struct k_work *work);
static void connection_evt_handler(const struct cloud_event *const evt);
static void no_sim_go_offline(struct k_work *work);

bool get_lte_connection_status(void)
{
	return lte_connection_status;
}

bool get_cloud_connection_status(void)
{
	return cloud_connection_status;
}

static void shutdown_modem(void)
{
	lte_connection_status = false;
#if defined(CONFIG_LTE_LINK_CONTROL)
	/* Turn off and shutdown modem */
	LOG_ERR("LTE link disconnect");
	int err = lte_lc_power_off();

	if (err) {
		LOG_ERR("lte_lc_power_off failed: %d", err);
	}
#endif /* CONFIG_LTE_LINK_CONTROL */
#if defined(CONFIG_NRF_MODEM_LIB)
	LOG_ERR("Shutdown modem");
	nrf_modem_lib_shutdown();
#endif
}

#if defined(CONFIG_LWM2M_CARRIER)
int lwm2m_carrier_event_handler(const lwm2m_carrier_event_t *event)
{
	switch (event->type) {
	case LWM2M_CARRIER_EVENT_BSDLIB_INIT:
		LOG_INF("LWM2M_CARRIER_EVENT_BSDLIB_INIT");
		k_sem_give(&nrf_modem_initialized);
		break;
	case LWM2M_CARRIER_EVENT_CONNECTING:
		LOG_INF("LWM2M_CARRIER_EVENT_CONNECTING\n");
		break;
	case LWM2M_CARRIER_EVENT_CONNECTED:
		LOG_INF("LWM2M_CARRIER_EVENT_CONNECTED");
		k_sem_give(&lte_connected);
		break;
	case LWM2M_CARRIER_EVENT_DISCONNECTING:
		LOG_INF("LWM2M_CARRIER_EVENT_DISCONNECTING");
		break;
	case LWM2M_CARRIER_EVENT_BOOTSTRAPPED:
		LOG_INF("LWM2M_CARRIER_EVENT_BOOTSTRAPPED");
		break;
	case LWM2M_CARRIER_EVENT_DISCONNECTED:
		LOG_INF("LWM2M_CARRIER_EVENT_DISCONNECTED");
		break;
	case LWM2M_CARRIER_EVENT_READY:
		LOG_INF("LWM2M_CARRIER_EVENT_READY");
		k_sem_give(&cloud_ready_to_connect);
		break;
	case LWM2M_CARRIER_EVENT_FOTA_START:
		LOG_INF("LWM2M_CARRIER_EVENT_FOTA_START");
		/* Due to limitations in the number of secure sockets,
		 * the cloud socket has to be closed when the carrier
		 * library initiates firmware upgrade download.
		 */
		atomic_set(&carrier_requested_disconnect, 1);
		app_disconnect();
		break;
	case LWM2M_CARRIER_EVENT_REBOOT:
		LOG_INF("LWM2M_CARRIER_EVENT_REBOOT");
		break;
	case LWM2M_CARRIER_EVENT_ERROR:
		LOG_ERR("LWM2M_CARRIER_EVENT_ERROR: code %d, value %d",
			((lwm2m_carrier_event_error_t *)event->data)->code,
			((lwm2m_carrier_event_error_t *)event->data)->value);
		break;
	}

	return 0;
}

/**@brief Disconnects from cloud. First it tries using the cloud backend's
 *        disconnect() implementation. If that fails, it falls back to close the
 *        socket directly, using close().
 */
static void app_disconnect(void)
{
	int err;

	atomic_set(&cloud_association, CLOUD_ASSOCIATION_STATE_INIT);
	LOG_INF("Disconnecting from cloud.");

	err = cloud_disconnect(cloud_backend);
	if (err == 0) {
		/* Ensure that the socket is indeed closed before returning. */
		if (k_sem_take(&cloud_disconnected, K_MINUTES(1)) == 0) {
			LOG_INF("Disconnected from cloud.");
			return;
		}
	} else if (err == -ENOTCONN) {
		LOG_INF("Cloud connection was not established.");
		return;
	} else {
		LOG_ERR("Could not disconnect from cloud, err: %d", err);
	}

	LOG_INF("Closing the cloud socket directly.");

	err = close(cloud_backend->config->socket);
	if (err) {
		LOG_ERR("Failed to close socket, error: %d", err);
		return;
	}

	LOG_INF("Socket was closed successfully.");
	return;
}
#endif /* defined(CONFIG_LWM2M_CARRIER) */

void set_log_panic(void)
{
	LOG_PANIC();
}

/**@brief nRF Cloud error handler. */
void error_handler(enum error_type err_type, int err_code)
{
	atomic_set(&cloud_association, CLOUD_ASSOCIATION_STATE_INIT);

	if (err_type == ERROR_CLOUD) {
		shutdown_modem();
	}

#if !defined(CONFIG_DEBUG) && defined(CONFIG_REBOOT)
	LOG_PANIC();
	sys_reboot(0);
#else
	switch (err_type) {
	case ERROR_BLE:
		ui_led_set_pattern(UI_BLE_ERROR, PWM_DEV_1);
		LOG_ERR("Error of type ERROR_BLE: %d", err_code);
		break;
	case ERROR_CLOUD:
		/* Blinking all LEDs ON/OFF in pairs (1 and 4, 2 and 3)
		 * if there is an application error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_CLOUD, PWM_DEV_0);
		LOG_ERR("Error of type ERROR_CLOUD: %d", err_code);
		break;
	case ERROR_MODEM_RECOVERABLE:
		/* Blinking all LEDs ON/OFF in pairs (1 and 3, 2 and 4)
		 * if there is a recoverable error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_MODEM_REC, PWM_DEV_0);
		LOG_ERR("Error of type ERROR_MODEM_RECOVERABLE: %d", err_code);
		break;
	case ERROR_MODEM_IRRECOVERABLE:
		/* Blinking all LEDs ON/OFF in pairs (1 and 3, 2 and 4)
		 * if there is a recoverable error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_MODEM_IRREC, PWM_DEV_0);
		LOG_ERR("Error of type ERROR_MODEM_IRRECOVERABLE: %d",
			err_code);
		break;
	default:
		/* Blinking all LEDs ON/OFF in pairs (1 and 2, 3 and 4)
		 * undefined error.
		 */
		ui_led_set_pattern(UI_LED_ERROR_UNKNOWN, PWM_DEV_0);
		LOG_ERR("Unknown error type: %d, code: %d",
			err_type, err_code);
		break;
	}

	while (true) {
		k_cpu_idle();
	}
#endif /* CONFIG_DEBUG */
}

void k_sys_fatal_error_handler(unsigned int reason,
			       const z_arch_esf_t *esf)
{
	ARG_UNUSED(esf);

	LOG_PANIC();
	LOG_ERR("Running main.c error handler");
	error_handler(ERROR_SYSTEM_FAULT, reason);
	CODE_UNREACHABLE;
}

void cloud_error_handler(int err)
{
	error_handler(ERROR_CLOUD, err);
}

void cloud_connect_error_handler(enum cloud_connect_result err)
{
	bool reboot = true;
	char *backend_name = "invalid";

	if (err == CLOUD_CONNECT_RES_SUCCESS) {
		return;
	}

	cloud_connection_status = false;

	if (!cloud_connection_enabled) {
		LOG_WRN("Ignoring cloud error");
		return;
	}
	LOG_ERR("Failed to connect to cloud, error %d", err);

	switch (err) {
	case CLOUD_CONNECT_RES_ERR_NOT_INITD: {
		LOG_ERR("Cloud back-end has not been initialized");
		/* no need to reboot, program error */
		reboot = false;
		break;
	}
	case CLOUD_CONNECT_RES_ERR_NETWORK: {
		LOG_ERR("Network error, check cloud configuration");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_BACKEND: {
		if (cloud_backend && cloud_backend->config &&
		    cloud_backend->config->name) {
			backend_name = cloud_backend->config->name;
		}
		LOG_ERR("An error occurred specific to the cloud back-end: %s",
			log_strdup(backend_name));
		break;
	}
	case CLOUD_CONNECT_RES_ERR_PRV_KEY: {
		LOG_ERR("Ensure device has a valid private key");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_CERT: {
		LOG_ERR("Ensure device has a valid CA and client certificate");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_CERT_MISC: {
		LOG_ERR("A certificate/authorization error has occurred");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_TIMEOUT_NO_DATA: {
		LOG_ERR("Connect timeout. SIM card may be out of data");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_ALREADY_CONNECTED: {
		LOG_ERR("Connection already exists.");
		break;
	}
	case CLOUD_CONNECT_RES_ERR_MISC: {
		break;
	}
	default: {
		LOG_ERR("Unhandled connect error");
		break;
	}
	}

	if (reboot) {
		LOG_ERR("Device will reboot in %d seconds",
			CONFIG_CLOUD_CONNECT_ERR_REBOOT_S);
		k_delayed_work_submit_to_queue(
				&application_work_q, &cloud_reboot_work,
				K_SECONDS(CONFIG_CLOUD_CONNECT_ERR_REBOOT_S));
	}

	ui_led_set_pattern(UI_LED_ERROR_CLOUD, PWM_DEV_0);
	shutdown_modem();
	k_thread_suspend(k_current_get());
}

/**@brief Recoverable modem library error. */
void nrf_modem_recoverable_error_handler(uint32_t err)
{
	error_handler(ERROR_MODEM_RECOVERABLE, (int)err);
}

void connect_to_cloud(const int32_t connect_delay_s)
{
	static bool initial_connect = true;

	if (!cloud_connection_enabled) {
		LOG_WRN("Cloud disabled; not connecting");
		return;
	}

	/* Ensure no data can be sent to cloud before connection is established.
	 */
	atomic_set(&cloud_association, CLOUD_ASSOCIATION_STATE_INIT);

	if (atomic_get(&carrier_requested_disconnect)) {
		/* A disconnect was requested to free up the TLS socket
		 * used by the cloud.  If enabled, the carrier library
		 * (CONFIG_LWM2M_CARRIER) will perform FOTA updates in
		 * the background and reboot the device when complete.
		 */
		return;
	}

	atomic_inc(&cloud_connect_attempts);

	/* Check if max cloud connect retry count is exceeded. */
	if (atomic_get(&cloud_connect_attempts) >
			CONFIG_CLOUD_CONNECT_COUNT_MAX) {
		LOG_ERR("The max cloud connection attempt count exceeded.");
		cloud_error_handler(-ETIMEDOUT);
	}

	if (!initial_connect) {
		LOG_INF("Attempting reconnect in %d seconds...",
			connect_delay_s);
		k_delayed_work_cancel(&cloud_reboot_work);
	} else {
		initial_connect = false;

		char id[256];
		int ret;

		LOG_INF("Retrieve device ID...");
		ret = cloud_get_id(cloud_backend, id, sizeof(id));
		if (ret) {
			LOG_ERR("Could not retrieve ID: %d", ret);
		} else {
			LOG_INF("Device ID = %s", log_strdup(id));
			LOG_INF("Endpoint = %s", CONFIG_NRF_CLOUD_HOST_NAME);
		}
	}

	k_delayed_work_submit_to_queue(&application_work_q,
				       &cloud_connect_work,
				       K_SECONDS(connect_delay_s));
}

static void cloud_connect_work_fn(struct k_work *work)
{
	int ret;

	LOG_INF("Connecting to cloud, attempt %d of %d",
		atomic_get(&cloud_connect_attempts),
		CONFIG_CLOUD_CONNECT_COUNT_MAX);

	k_delayed_work_submit_to_queue(&application_work_q,
				       &cloud_reboot_work,
				       K_MSEC(CLOUD_CONNACK_WAIT_DURATION));

	ui_led_set_pattern(UI_CLOUD_CONNECTING, PWM_DEV_0);

	/* Attempt cloud connection */
	ret = cloud_connect(cloud_backend);
	if (ret != CLOUD_CONNECT_RES_SUCCESS) {
		k_delayed_work_cancel(&cloud_reboot_work);
		/* Will not return from this function.
		 * If the connect fails here, it is likely
		 * that user intervention is required.
		 */
		cloud_connect_error_handler(ret);
	} else {
		LOG_INF("Cloud connection request sent.");
		LOG_INF("Connection response timeout is set to %d seconds.",
			CLOUD_CONNACK_WAIT_DURATION / MSEC_PER_SEC);
		k_delayed_work_submit_to_queue(&application_work_q,
					&cloud_reboot_work,
					K_MSEC(CLOUD_CONNACK_WAIT_DURATION));
	}
}

/**@brief Reboot the device if CONNACK has not arrived. */
static void cloud_reboot_handler(struct k_work *work)
{
	error_handler(ERROR_CLOUD, -ETIMEDOUT);
}

/**@brief nRF Cloud specific callback for cloud association event. */
static void on_user_pairing_req(const struct cloud_event *evt)
{
	if (atomic_get(&cloud_association) !=
			CLOUD_ASSOCIATION_STATE_REQUESTED) {
		atomic_set(&cloud_association,
			   CLOUD_ASSOCIATION_STATE_REQUESTED);
		ui_led_set_pattern(UI_CLOUD_PAIRING, PWM_DEV_0);
		LOG_INF("Add device to cloud account.");
		LOG_INF("Waiting for cloud association...");

		/* clear sessions to ensure devices switching accounts
		 * will start on the new account with full topic
		 * subscriptions
		 */
		LOG_INF("Clearing persistent sessions...");
		save_session_state(0);

		ble_conn_mgr_init();
		ble_conn_mgr_clear_desired(true);
		ble_stop_activity();

		/* If the association is not done soon enough (< ~5 min?)
		 * a connection cycle is needed... TBD why.
		 */
		k_delayed_work_submit_to_queue(&application_work_q,
					&cycle_cloud_connection_work,
					CONN_CYCLE_AFTER_ASSOCIATION_REQ_MS);
	}
}

static void cycle_cloud_connection(struct k_work *work)
{
	int32_t reboot_wait_ms = REBOOT_AFTER_DISCONNECT_WAIT_MS;

	LOG_INF("Disconnecting from cloud...");
	cloud_connection_status = false;

	if (cloud_disconnect(cloud_backend) != 0) {
		reboot_wait_ms = 5 * MSEC_PER_SEC;
		LOG_INF("Disconnect failed. Device will reboot in %d seconds",
			(reboot_wait_ms / MSEC_PER_SEC));
	}

	/* Reboot fail-safe on disconnect */
	k_delayed_work_submit_to_queue(&application_work_q, &cloud_reboot_work,
				       K_MSEC(reboot_wait_ms));
}

/** @brief Handle procedures after successful association with nRF Cloud. */
void on_pairing_done(void)
{
	if (atomic_get(&cloud_association) ==
			CLOUD_ASSOCIATION_STATE_REQUESTED) {
		k_delayed_work_cancel(&cycle_cloud_connection_work);

		/* After successful association, the device must
		 * reconnect to the cloud.
		 */
		LOG_INF("Device associated with cloud.");
		LOG_INF("Reconnecting for cloud policy to take effect.");
		atomic_set(&cloud_association,
			   CLOUD_ASSOCIATION_STATE_RECONNECT);
		k_delayed_work_submit_to_queue(&application_work_q,
					       &cycle_cloud_connection_work,
					       K_NO_WAIT);
	} else {
		atomic_set(&cloud_association, CLOUD_ASSOCIATION_STATE_PAIRED);
	}
}

void cloud_event_handler(const struct cloud_backend *const backend,
			 const struct cloud_event *const evt,
			 void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case CLOUD_EVT_CONNECTED:
	case CLOUD_EVT_CONNECTING:
	case CLOUD_EVT_DISCONNECTED:
		connection_evt_handler(evt);
		break;
	case CLOUD_EVT_READY:
		LOG_INF("CLOUD_EVT_READY");
		ui_led_set_pattern(UI_CLOUD_CONNECTED, PWM_DEV_0);

#if defined(CONFIG_BOOTLOADER_MCUBOOT) && !defined(CONFIG_NRF_CLOUD_FOTA)
		/* Mark image as good to avoid rolling back after update */
		boot_write_img_confirmed();
#endif
		atomic_set(&cloud_association, CLOUD_ASSOCIATION_STATE_READY);

		break;
	case CLOUD_EVT_ERROR:
		LOG_INF("CLOUD_EVT_ERROR");
		break;
	case CLOUD_EVT_DATA_SENT:
		LOG_INF("CLOUD_EVT_DATA_SENT");
		break;
	case CLOUD_EVT_DATA_RECEIVED:
		LOG_INF("CLOUD_EVT_DATA_RECEIVED");
		break;
	case CLOUD_EVT_PAIR_REQUEST:
		LOG_INF("CLOUD_EVT_PAIR_REQUEST");
		on_user_pairing_req(evt);
		break;
	case CLOUD_EVT_PAIR_DONE:
		LOG_INF("CLOUD_EVT_PAIR_DONE");
		on_pairing_done();
		break;
	case CLOUD_EVT_FOTA_DONE:
		LOG_INF("CLOUD_EVT_FOTA_DONE");
		lte_connection_status = false;
		LOG_INF("Rebooting to complete FOTA...");
		k_sleep(K_SECONDS(2));
#if defined(CONFIG_LTE_LINK_CONTROL)
		lte_lc_power_off();
#endif
#if defined(CONFIG_REBOOT)
		sys_reboot(SYS_REBOOT_COLD);
#endif
		break;
	default:
		LOG_WRN("Unknown cloud event type: %d", evt->type);
		break;
	}
}

void query_modem_info(void)
{
#ifdef CONFIG_MODEM_INFO
	modem_info_init();
	modem_info_params_init(&modem_param);

	int ret = modem_info_params_get(&modem_param);

	if (ret) {
		LOG_ERR("Error getting modem info: %d", ret);
	} else {
		if (modem_param.network.date_time.value_string) {
			char *str = modem_param.network.date_time.value_string;

			_timezone = atoi(&str[18]) * 15 * 60;
			_daylight = atoi(&str[25]);
			LOG_INF("Network date/time: %s "
				"DST %d TZ %ld",
				log_strdup(modem_param.network.date_time.value_string),
				_daylight, _timezone);

			struct tm tm;
			char *rm;
			char *src = modem_param.network.date_time.value_string;
			struct timespec ts;

			/* 20/06/12,00:47:47-28 */
			rm = strptime(src, "%y/%m/%d,%H:%M:%S", &tm);
			if (rm) {
				ts.tv_sec = mktime(&tm);
				ts.tv_nsec = 0;
				LOG_INF("setting time to %lld", ts.tv_sec);

				/* tzset();
				 * char *tz = getenv("TZ");
				 * LOG_INF("TZ=%s", tz ? tz : "<unknown>");
				 */

				if (!clock_settime(CLOCK_REALTIME, &ts)) {
					LOG_INF("time set");
				} else {
					LOG_ERR("error %d on clock_settime()",
						errno);
				}
			} else {
				LOG_ERR("strptime() could not parse time");
			}
		} else {
			LOG_WRN("modem_info.network.date_time: empty");
		}
	}
#endif
}

void control_cloud_connection(bool enable)
{
	cloud_connection_enabled = enable;
}

void connection_evt_handler(const struct cloud_event *const evt)
{
	if (evt->type == CLOUD_EVT_CONNECTING) {
		cloud_connection_status = false;
		LOG_INF("CLOUD_EVT_CONNECTING");
		ui_led_set_pattern(UI_CLOUD_CONNECTING, PWM_DEV_0);
		k_delayed_work_cancel(&cloud_reboot_work);

		if (evt->data.err != CLOUD_CONNECT_RES_SUCCESS) {
			cloud_connect_error_handler(evt->data.err);
		}
		return;
	} else if (evt->type == CLOUD_EVT_CONNECTED) {
		cloud_connection_status = true;
		LOG_INF("*******************************");
		query_modem_info();
		LOG_INF("*******************************");
		LOG_INF("CLOUD_EVT_CONNECTED");
		k_delayed_work_cancel(&cloud_reboot_work);
		k_sem_take(&cloud_disconnected, K_NO_WAIT);
		atomic_set(&cloud_connect_attempts, 0);

		LOG_INF("Persistent Sessions = %u",
			evt->data.persistent_session);
		setup_gw_shadow();

	} else if (evt->type == CLOUD_EVT_DISCONNECTED) {
		int32_t connect_wait_s = CONFIG_CLOUD_CONNECT_RETRY_DELAY;

		cloud_connection_status = false;
		LOG_INF("CLOUD_EVT_DISCONNECTED: %d", evt->data.err);
		ui_led_set_pattern(UI_LTE_CONNECTED, PWM_DEV_0);

		switch (evt->data.err) {
		case CLOUD_DISCONNECT_INVALID_REQUEST:
			LOG_INF("Cloud connection closed.");
			if ((atomic_get(&cloud_connect_attempts) == 1) &&
				(atomic_get(&cloud_association) ==
						CLOUD_ASSOCIATION_STATE_INIT)) {
				LOG_INF("This can occur during initial "
					"nRF Cloud provisioning.");
#if defined(CONFIG_LWM2M_CARRIER)
#if !defined(CONFIG_DEBUG) && defined(CONFIG_REBOOT)
				/* Reconnect does not work with
				 * carrier library
				 */
				LOG_INF("Rebooting in 10 seconds...");
				k_sleep(K_SECONDS(10));
#endif
				error_handler(ERROR_CLOUD, -EIO);
				break;
#endif
				connect_wait_s = 10;
			} else {
				LOG_INF("This can occur if the device has "
					"the wrong nRF Cloud certificates.");
			}
			break;
		case CLOUD_DISCONNECT_USER_REQUEST:
			LOG_INF("CLOUD_DISCONNECT_USER_REQUEST");
			if (atomic_get(&cloud_association) ==
					CLOUD_ASSOCIATION_STATE_RECONNECT ||
				atomic_get(&cloud_association) ==
					CLOUD_ASSOCIATION_STATE_REQUESTED ||
				(atomic_get(&carrier_requested_disconnect))) {
				connect_wait_s = 10;
				LOG_INF("reconnect for association");
			} else {
				connect_wait_s = -1;
			}
			break;
		case CLOUD_DISCONNECT_CLOSED_BY_REMOTE:
			LOG_INF("Disconnected by the cloud.");
			break;
		case CLOUD_DISCONNECT_MISC:
			LOG_INF("CLOUD_DISCONNECT_MISC");
		default:
			break;
		}
		k_sem_give(&cloud_disconnected);
		if ((connect_wait_s >= 0) && cloud_connection_enabled) {
			connect_to_cloud(connect_wait_s);
		}
	}
}


/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
	k_delayed_work_init(&cloud_reboot_work, cloud_reboot_handler);
	k_delayed_work_init(&cycle_cloud_connection_work,
			    cycle_cloud_connection);
	k_delayed_work_init(&cloud_connect_work, cloud_connect_work_fn);
	k_work_init(&no_sim_go_offline_work, no_sim_go_offline);
}

static void cloud_api_init(void)
{
	int ret;

	cloud_backend = cloud_get_binding("NRF_CLOUD");
	__ASSERT(cloud_backend != NULL, "nRF Cloud backend not found");

	ret = cloud_init(cloud_backend, cloud_event_handler);
	if (ret) {
		LOG_ERR("Cloud backend could not be initialized, error: %d",
			ret);
		cloud_error_handler(ret);
	}

	ret = peripheral_dfu_init();
	if (ret) {
		LOG_ERR("Error initializing BLE DFU: %d", ret);
	}
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static int modem_configure(void)
{
#if defined(CONFIG_NRF_MODEM_LIB)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on */
		/* and connected */
		goto connected;
	}

	ui_led_set_pattern(UI_LTE_CONNECTING, PWM_DEV_0);
	LOG_INF("Connecting to LTE network.");
	LOG_INF("This may take several minutes.");

#if defined(CONFIG_LWM2M_CARRIER)
	/* Wait for the LWM2M carrier library to configure the */
	/* modem and set up the LTE connection. */
	k_sem_take(&lte_connected, K_FOREVER);
#else /* defined(CONFIG_LWM2M_CARRIER) */
	int err = lte_lc_init_and_connect();
	if (err) {
		lte_connection_status = false;
		LOG_ERR("LTE link could not be established.");
		return err;
	}
#endif /* defined(CONFIG_LWM2M_CARRIER) */

connected:
	lte_connection_status = true;
	LOG_INF("Connected to LTE network.");
	ui_led_set_pattern(UI_LTE_CONNECTED, PWM_DEV_0);

#endif /* defined(CONFIG_NRF_MODEM_LIB) */
	return 0;
}

void handle_nrf_modem_lib_init_ret(void)
{
#if defined(CONFIG_NRF_MODEM_LIB) && !defined(CONFIG_NRF_CLOUD_FOTA)
	int ret = nrf_modem_lib_get_init_ret();

	/* Handle return values relating to modem firmware update */
	switch (ret) {
	case 0:
		/* Initialization successful, no action required. */
		break;
	case MODEM_DFU_RESULT_OK:
		LOG_INF("MODEM UPDATE OK. Will run new firmware");
#if defined(CONFIG_REBOOT)
		sys_reboot(SYS_REBOOT_COLD);
#endif
		break;
	case MODEM_DFU_RESULT_UUID_ERROR:
	case MODEM_DFU_RESULT_AUTH_ERROR:
		LOG_ERR("MODEM UPDATE ERROR %d. Will run old firmware", ret);
#if defined(CONFIG_REBOOT)
		sys_reboot(SYS_REBOOT_COLD);
#endif
		break;
	case MODEM_DFU_RESULT_HARDWARE_ERROR:
	case MODEM_DFU_RESULT_INTERNAL_ERROR:
		LOG_ERR("MODEM UPDATE FATAL ERROR %d. Modem failiure", ret);
#if defined(CONFIG_REBOOT)
		sys_reboot(SYS_REBOOT_COLD);
#endif
		break;
	default:
		/* All non-zero return codes other than DFU result codes are
		 * considered irrecoverable and a reboot is needed.
		 */
		LOG_ERR("modem library initialization failed, error: %d", ret);
		error_handler(ERROR_MODEM_IRRECOVERABLE, ret);

		CODE_UNREACHABLE;
	}
#endif /* CONFIG_NRF_MODEM_LIB */
}

static void no_sim_go_offline(struct k_work *work)
{
#if defined(CONFIG_NRF_MODEM_LIB)
	lte_lc_offline();
	/* Wait for lte_lc events to be processed before printing info message */
	k_sleep(K_MSEC(100));
	LOG_INF("No SIM card detected.");
	LOG_INF("Insert SIM and reset device to run the gateway.");
	ui_led_set_pattern(UI_LED_ERROR_LTE_LC, PWM_DEV_0);
#endif /* CONFIG_NRF_MODEM_LIB */
}

static void starting_button_handler(void)
{
#if defined(CONFIG_ENTER_52840_MCUBOOT_VIA_BUTTON)
	/* the active board file need to include the functions:
	 * nrf52840_reset_to_mcuboot() and nrf52840_wait_boot_low()
	 */
	if (ui_button_is_active(1)) {
		printk("BOOT BUTTON HELD\n");
		ui_led_set_pattern(UI_BLE_BUTTON, PWM_DEV_1);
		while (ui_button_is_active(1)) {
			k_sleep(K_MSEC(500));
		}
		printk("BOOT BUTTON RELEASED\n");
		if (!is_boot_selected()) {
			printk("Boot held after reset but not long enough"
			       " to select the nRF52840 bootloader!\n");
			ui_led_set_pattern(UI_BLE_OFF, PWM_DEV_1);
		} else {
			/* User wants to update the 52840 */
			nrf52840_reset_to_mcuboot();

			/* wait forever for signal from other side so we can
			 * continue
			 */
			int err;

			err = nrf52840_wait_boot_complete(WAIT_BOOT_TIMEOUT_MS);
			if (err == 0) {
				ui_led_set_pattern(UI_BLE_OFF, PWM_DEV_1);
				k_sleep(K_SECONDS(1));
				printk("nRF52840 update complete\n");
			} else {
				/* unable to monitor; just halt */
				printk("Error waiting for nrf52840 reboot: %d\n",
				       err);
				for (;;) {
					k_sleep(K_MSEC(500));
				}
			}
		}
	}
#endif
}

static void log_uart_pins(void)
{
#if defined(DEBUG_UART_PINS)
	struct uart_config config;
	struct device *uart_0_dev = device_get_binding("UART_0");
	struct device *uart_1_dev = device_get_binding("UART_1");

	LOG_INF("UART0 tx:%d, rx:%d, rts:%d, cts:%d, speed:%d",
		UART0_TX, UART0_RX, UART0_RTS, UART0_CTS, UART0_SPEED);
	LOG_INF("UART1 tx:%d, rx:%d, rts:%d, cts:%d, speed:%d",
		UART1_TX, UART1_RX, UART1_RTS, UART1_CTS, UART1_SPEED);
	k_sleep(K_MSEC(50));

	uart_config_get(uart_0_dev, &config);
	LOG_INF("UART0 speed:%u, flow:%d", config.baudrate,
		config.flow_ctrl);
	k_sleep(K_MSEC(50));

	uart_config_get(uart_1_dev, &config);
	LOG_INF("UART1 speed:%u, flow:%d", config.baudrate,
		config.flow_ctrl);

	LOG_INF("Reset pin:%d",
		CONFIG_BOARD_NRF52840_GPIO_RESET_PIN);
#endif
}


void lg_printk(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	log_printk(fmt, args);
	va_end(args);
}

void main(void)
{
	int err;

	lg_printk("\n*************************************************\n");
	lg_printk("nRF Cloud Gateway Starting Up...\n");
	lg_printk("Ver:%s Built:%s\n", FW_REV_STRING, BUILT_STRING);
	lg_printk("*************************************************\n\n");
	log_uart_pins();

#if defined(CONFIG_USE_UI_MODULE)
	ui_init(power_button_handler);
#endif

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		LOG_INF("Loading settings");
		settings_load();
	}

	k_work_q_start(&application_work_q, application_stack_area,
		       K_THREAD_STACK_SIZEOF(application_stack_area),
		       CONFIG_APPLICATION_WORKQUEUE_PRIORITY);
	k_thread_name_set(&application_work_q.thread, "appworkq");

	starting_button_handler();

	k_sleep(K_SECONDS(2));
	cli_init();

	init_gateway();

	err = ble_init();
	if (err) {
		error_handler(ERROR_BLE, err);
	} else {
		ui_led_set_pattern(UI_BLE_OFF, PWM_DEV_1);
	}

	if (IS_ENABLED(CONFIG_WATCHDOG)) {
		watchdog_init_and_start(&application_work_q);
	}

#if defined(CONFIG_LWM2M_CARRIER)
	k_sem_take(&nrf_modem_initialized, K_FOREVER);
#else
	handle_nrf_modem_lib_init_ret();
#endif
	/* delay a bit to allow BLE logging to catch up before
	 * connecting to cloud -- otherwise it can be hard to follow
	 * what's happening when debugging
	 */
	k_sleep(K_SECONDS(5));
	cloud_api_init();

	work_init();
#if defined(CONFIG_CPU_LOAD)
	cpu_load_init();
#endif

	while (modem_configure() != 0) {
		LOG_WRN("Failed to establish LTE connection.");
		LOG_WRN("Will retry in %d seconds.",
			CONFIG_CLOUD_CONNECT_RETRY_DELAY);
		k_sleep(K_SECONDS(CONFIG_CLOUD_CONNECT_RETRY_DELAY));
	}

#if defined(CONFIG_LWM2M_CARRIER)
	LOG_INF("Waiting for LWM2M carrier to complete initialization...");
	k_sem_take(&cloud_ready_to_connect, K_FOREVER);
#endif

	connect_to_cloud(0);
}

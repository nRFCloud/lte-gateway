#include <zephyr.h>
#include <sys/printk.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <shell/shell_history.h>
#include <power/reboot.h>
#include <version.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include <fw_info.h>
#undef __XSI_VISIBLE
#define __XSI_VISIBLE 1
#include <time.h>
#include <posix/time.h>
#include <modem/modem_info.h>
#include <modem/at_cmd.h>
#include <modem/lte_lc.h>
#include <net/cloud.h>
#include <net/buf.h>
#include <net/fota_download.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_vs.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <sw_isr_table.h>
#include <modem/lte_lc.h>

#include "config.h"
#include "nrf_cloud_codec.h"
#include "nrf_cloud_transport.h"
#include "ble.h"
#include "ble_codec.h"
#include "ble_conn_mgr.h"
#include "gateway.h"

LOG_MODULE_REGISTER(cli, CONFIG_NRF_CLOUD_GATEWAY_LOG_LEVEL);

#define DEFAULT_PASSWORD "alThazar12"
/* disable for now -- breaks BLE HCI after used */
#define PRINT_CTLR_INFO_ENABLED 0

/* uncomment to list UUID paths in 'info conn' display */
/* #define INCLUDE_PATHS */

enum ble_cmd_type {
	BLE_CMD_ALL,
	BLE_CMD_MAC,
	BLE_CMD_NAME
};

extern struct modem_param_info modem_param;
static char response_buf[CONFIG_AT_CMD_RESPONSE_MAX_LEN];

static void set_at_prompt(const struct shell *shell, bool at_mode);

char *rem_eol(const char *s)
{
	if (!s) {
		return "";
	}

	char *x = log_strdup(s);
	char *p = x;

	while (*p) {
		if ((*p == '\r') || (*p == '\n')) {
			*p = '\0';
			break;
		}
		p++;
	}
	return x;
}

void print_fw_info(const struct shell *shell)
{
	/* TODO: debug this */
	/* this does not work: fw_info *info = fw_info_find(0); */
	extern struct fw_info m_firmware_info;
	struct fw_info *info = &m_firmware_info;
	static char id[64];

	if (info) {
		shell_print(shell, "HW rev: \t%s", HW_REV_STRING);
		shell_print(shell, "FW rev: \t%s", FW_REV_STRING);
		shell_print(shell, "Built:  \t%s", BUILT_STRING);
		shell_print(shell, "Ver:    \t%u", info->version);
		shell_print(shell, "Size:   \t%u", info->size);
		shell_print(shell, "Start:  \t0x%08x", info->address);
		shell_print(shell, "Boot:   \t0x%08x", info->boot_address);
		shell_print(shell, "Valid:  \t%u", info->valid);

		struct cloud_backend *backend = cloud_get_binding("NRF_CLOUD");

		if (backend) {
			cloud_get_id(backend, id, sizeof(id));
		}
		shell_print(shell, "Dev ID: \t%s", id);
	}
}

void print_modem_info(const struct shell *shell)
{
#ifdef CONFIG_MODEM_INFO
	modem_info_init();
	modem_info_params_init(&modem_param);

	int ret = modem_info_params_get(&modem_param);

	if (ret) {
		shell_error(shell, "Error getting modem info: %d", ret);
	} else {
		struct device_param *dev = &modem_param.device;
		struct network_param *net = &modem_param.network;
		struct sim_param *sim = &modem_param.sim;

		if (dev->modem_fw.type == MODEM_INFO_FW_VERSION) {
			shell_print(shell, "Modem fw: \t%s",
			  rem_eol(dev->modem_fw.value_string));
		} else {
			shell_print(shell, "Modem fw type: \t%u val %u",
				dev->modem_fw.type,
				dev->modem_fw.value);
		}
		if (dev->battery.type == MODEM_INFO_BATTERY) {
			shell_print(shell, "Battery: \t%u mV",
				    dev->battery.value);
		}
		if (dev->imei.type == MODEM_INFO_IMEI) {
			shell_print(shell, "IMEI: \t\t%s",
			      rem_eol(dev->imei.value_string));
		}
		shell_print(shell, "Board: \t\t%s",
			    rem_eol(dev->board));
		shell_print(shell, "App Name: \t%s",
			    rem_eol(dev->app_name));
		shell_print(shell, "App Ver: \t%s",
			    rem_eol(dev->app_version));

		if (sim->uicc.type == MODEM_INFO_UICC) {
			shell_print(shell, "UICC: \t\t%u",
				    sim->uicc.value);
		}
		if (sim->iccid.type == MODEM_INFO_ICCID) {
			shell_print(shell, "ICCID: \t\t%s",
				    rem_eol(sim->iccid.value_string));
		}
		if (sim->imsi.type == MODEM_INFO_IMSI) {
			shell_print(shell, "IMSI: \t\t%s",
				    rem_eol(sim->imsi.value_string));
		}

		if (net->current_band.type ==
		    MODEM_INFO_CUR_BAND) {
			shell_print(shell, "Cur band: \t%u",
				    net->current_band.value);
		}
		if (net->sup_band.type ==
		    MODEM_INFO_SUP_BAND) {
			shell_print(shell, "Sup band: \t%s",
				    rem_eol(net->sup_band.value_string));
		}
		if (net->area_code.type ==
		    MODEM_INFO_AREA_CODE) {
			shell_print(shell, "Area code: \t%s",
				 rem_eol(net->area_code.value_string));
		}
		if (net->current_operator.type ==
		    MODEM_INFO_OPERATOR) {
			shell_print(shell, "Operator: \t%s",
			     rem_eol(net->current_operator.value_string));
		}
		if (net->mcc.type ==
		    MODEM_INFO_MCC) {
			shell_print(shell, "MCC: \t\t%u",
				    net->mcc.value);
		}
		if (net->mnc.type ==
		    MODEM_INFO_MNC) {
			shell_print(shell, "MNC: \t\t%u",
				    net->mnc.value);
		}
		if (net->ip_address.type ==
		    MODEM_INFO_IP_ADDRESS) {
			shell_print(shell, "IP address: \t%s",
				   rem_eol(net->ip_address.value_string));
		}
		if (net->ue_mode.type ==
		    MODEM_INFO_UE_MODE) {
			shell_print(shell, "UE mode: \t%u",
				    net->ue_mode.value);
		}
		if (net->apn.type ==
		    MODEM_INFO_APN) {
			shell_print(shell, "Access point: \t%s",
				    rem_eol(net->apn.value_string));
		}
		if (net->lte_mode.value == 1) {
			shell_print(shell, "Mode: \t\tLTE-M");
		} else if (net->nbiot_mode.value == 1) {
			shell_print(shell, "Mode: \t\tNB-IoT");
		}
		shell_print(shell, "Cell ID: \t%ld", 
			    (long)net->cellid_dec);

		char *str = net->date_time.value_string;
		struct timespec now;
		struct tm *tm;

		shell_print(shell, "Net date/time: \t%s "
			"DST %d TZ %ld",
			log_strdup(str), _daylight, _timezone);
		clock_gettime(CLOCK_REALTIME, &now);
		tm = localtime(&now.tv_sec);
		shell_print(shell, "Time now: \t%s", asctime(tm));
	}
#endif
}

void print_connection_status(const struct shell *shell)
{
	int err;
	enum lte_lc_nw_reg_status status;
	enum lte_lc_system_mode smode;
	enum lte_lc_func_mode fmode;

	err = lte_lc_nw_reg_status_get(&status);
	if (err) {
		shell_error(shell, "lte_lc_nw_reg_status_get: %d", err);
	} else {
		shell_print(shell, "LTE reg status: \t%d\n", (int)status);
	}
	err = lte_lc_system_mode_get(&smode);
	if (err) {
		shell_error(shell, "lte_lc_system_mode_get: %d", err);
	} else {
		shell_print(shell, "LTE system mode: \t%d\n", (int)smode);
	}
	err = lte_lc_func_mode_get(&fmode);
	if (err) {
		shell_error(shell, "lte_lc_func_mode_get: %d", err);
	} else {
		shell_print(shell, "LTE functional mode: \t%d\n", (int)fmode);
	}

	shell_print(shell, "LTE connection: \t%s\n",
		    get_lte_connection_status() ?
		    "connected" : "disconnected");
	shell_print(shell, "cloud connection: \t%s\n",
		    get_cloud_connection_status() ?
		    "connected" : "disconnected");
}

static void print_cloud_info(const struct shell *shell)
{
	char stage[8];

	nct_gw_get_stage(stage, sizeof(stage));
	print_connection_status(shell);

	shell_print(shell, "nrfcloud stage: \t%s", stage);
	shell_print(shell, "nrfcloud endpoint: \t%s", CONFIG_NRF_CLOUD_HOST_NAME);
}

static void print_ctlr_info(const struct shell *shell)
{
#if PRINT_CTLR_INFO_ENABLED
	struct net_buf *rsp;
	int ret;

	ret = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_VERSION_INFO, NULL, &rsp);
	if (ret) {
		shell_error(shell, "Could not read version info: %d", ret);
	} else {
		struct bt_hci_rp_vs_read_version_info *info;

		info = (void *)rsp->data;
		shell_print(shell, "HW platform: \t0x%04x", info->hw_platform);
		shell_print(shell, "HW variant: \t0x%04x", info->hw_variant);
		shell_print(shell, "FW variant: \t0x%02x", info->fw_variant);
		shell_print(shell, "FW version: \t%u.%u",
			    info->fw_version,
			    sys_le16_to_cpu(info->fw_revision));
		shell_print(shell, "Build:    \t0x%08X", info->fw_build);
	}

	if (!IS_ENABLED(CONFIG_BT_LL_SOFTDEVICE)) {
		ret = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_BUILD_INFO,
					   NULL, &rsp);
		if (ret) {
			LOG_ERR("Could not read build info: %d", ret);
		} else {
			struct bt_hci_rp_vs_read_build_info *build;

			build = (void *)rsp->data;
			shell_print(shell, "OS build: \t%s",
				    (const char *)build->info);
		}
	}

	if (IS_ENABLED(CONFIG_BT_LL_SOFTDEVICE)) {
		ret = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_STATIC_ADDRS,
					   NULL, &rsp);
		if (ret || (rsp == NULL)) {
			LOG_ERR("Could not read static address: %d", ret);
		} else {
			struct bt_hci_rp_vs_read_static_addrs *addrs;
			uint8_t *a;

			addrs = (void *)rsp->data;
			a = &addrs->a[0].bdaddr.val[0];
			shell_print(shell, "Static addr: \t"
				    "%02X:%02X:%02X:%02X:%02X:%02X",
				    a[0], a[1], a[2], a[3], a[4], a[5]);
		}
	}
#else
	shell_print(shell, "unavailable");
#endif
}

static void print_scan_info(const struct shell *shell)
{
	unsigned int i;
	struct ble_scanned_dev *dev;

	shell_print(shell, "   MAC, type, RSSI, name");
	for (i = 0; i < MAX_SCAN_RESULTS; i++) {
		dev = get_scanned_device(i);
		if (dev == NULL) {
			shell_print(shell, "<end of list>");
			break;
		}
		shell_print(shell, "%u. %s, %s, %d, %s",
			    i + 1, dev->addr, dev->type, dev->rssi, dev->name);
	}
}

static void print_conn_info(const struct shell *shell)
{
	unsigned int i;
	unsigned int j;
	int count = 0;
	struct ble_device_conn *dev;
	struct uuid_handle_pair *up;
	char uuid_str[BT_MAX_UUID_LEN];
#if defined(INCLUDE_PATHS)
	char path[BT_MAX_PATH_LEN];
#endif
	const char *types[] = {"svc", "chr", "---", "ccc"};

	shell_print(shell, "   MAC, connected, discovered, shadow"
			   " updated, blocklist status, num UUIDs");
	for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		dev = get_connected_device(i);
		if (dev == NULL) {
			continue;
		}
		count++;
		shell_print(shell, "%u. %s, %s, %s, %s, %s, UUIDs:%u",
			    count,
			    dev->addr,
			    dev->connected ? "CONNNECTED" : "disconnected",
			    dev->discovered ? "DISCOVERED" :
			    (dev->discovering ? "DISCOVERING" : "not dscvred"),
			    dev->shadow_updated ? "SHADOW UPDATED" :
			    "shadow not set",
			    dev->added_to_whitelist ? "CONN ALLOWED" :
			    "conn blocked",
			    (unsigned int)dev->num_pairs
			   );
		shell_print(shell, "   is service, UUID, handle, type, path"
			    " depth, properties, sub index, sub enabled");
		for (j = 0; j < dev->num_pairs; j++) {
			up = &dev->uuid_handle_pairs[j];
			get_uuid_str(up, uuid_str, BT_MAX_UUID_LEN);
			shell_print(shell,
				    "   %u, %s, %s, %u, %s, %u, 0x%02X, %u, %s",
				    j + 1,
				    up->is_service ? "serv" : "char",
				    uuid_str,
				    up->handle,
				    up->attr_type < 4 ? types[up->attr_type] :
				    "unk",
				    (unsigned int)up->path_depth,
				    (unsigned int)up->properties,
				    (unsigned int)up->sub_index,
				    up->sub_enabled ? "NOTIFY ON" : "notify off"
			);
#if defined(INCLUDE_PATHS)
			ble_conn_mgr_generate_path(dev, up->handle, path,
						   up->attr_type == 3);
			shell_print(shell, "       %u, %s", up->handle, path);
#endif
		}
	}
}

static void print_irq_info(const struct shell *shell)
{
	int i;
	extern char _vector_start[];
	void **vectors = (void *)_vector_start;

	shell_print(shell, "IRQn, entry, prio, en, pend, active, arg");
	for (i = -15; i < IRQ_TABLE_SIZE; i++) {
		void *entry;
		const void *arg;

		if (i < 0) {
			entry = vectors[i + 16];
			arg = 0;
		} else {
			entry = _sw_isr_table[i].isr;
			arg = _sw_isr_table[i].arg;
		}
		if (entry == z_irq_spurious) {
			continue;
		}
		shell_print(shell, "% 3d. %010p, %d, %d, %d, %d, %010p",
			    i, entry,
			    NVIC_GetPriority(i), NVIC_GetEnableIRQ(i),
			    NVIC_GetPendingIRQ(i), NVIC_GetActive(i),
			    arg);
	}
}

static int ble_conn_save(const struct shell *shell)
{
	int num;
	struct desired_conn *desired = get_desired_array(&num);

	return nrf_cloud_update_gateway_state(desired, num);
}

static int ble_conn_mac(const struct shell *shell, char *addr)
{
	int err;

	shell_print(shell, "  Adding connection to %s...", addr);
	err = ble_conn_mgr_add_conn(addr);
	if (!err) {
		shell_print(shell, "  Adding to desired list...");
		err = ble_conn_mgr_add_desired(addr, true);
	}
	if (err) {
		shell_error(shell, "  Failed: error %d", err);
	} else {
		shell_print(shell, "  Done.");
	}

	return err;
}

static int ble_conn_all(const struct shell *shell)
{
	unsigned int i;
	struct ble_scanned_dev *dev;
	int err = 0;
	int count = 0;

	for (i = 0; i < MAX_SCAN_RESULTS; i++) {
		dev = get_scanned_device(i);
		if (dev == NULL) {
			break;
		}
		err = ble_conn_mac(shell, dev->addr);
		if (!err) {
			count++;
		}
	}

	shell_print(shell, "Connected to %d devices", count);
	return err;
}

static int ble_conn_name(const struct shell *shell, char *name)
{
	unsigned int i;
	struct ble_scanned_dev *dev;

	for (i = 0; i < MAX_SCAN_RESULTS; i++) {
		dev = get_scanned_device(i);
		if (dev == NULL) {
			break;
		}
		if (strcmp(name, dev->name) == 0) {
			shell_print(shell, "Connecting to %s", name);
			return ble_conn_mac(shell, dev->addr);
		}
	}

	shell_error(shell, "BLE device %s not found", name);
	return -EINVAL;
}

static int ble_disconn_mac(const struct shell *shell, char *addr)
{
	int err;

	shell_print(shell, "  Removing connection to %s...", addr);
	err = ble_conn_mgr_remove_conn(addr);
	if (!err) {
		shell_print(shell, "  Removing from desired list...");
		err = ble_conn_mgr_rem_desired(addr, true);
		if (!err) {
			shell_print(shell, "  Disconnecting device...");
			err = disconnect_device_by_addr(addr);
		}
	}
	if (err) {
		shell_error(shell, "  Failed: error %d", err);
	} else {
		shell_print(shell, "  Done.");
	}

	return err;
}

static int ble_disconn_all(const struct shell *shell)
{
	unsigned int i;
	struct ble_device_conn *con;
	int err = 0;
	int count = 0;

	for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		con = get_connected_device(i);
		if (con == NULL) {
			continue;
		}
		err = ble_disconn_mac(shell, con->addr);
		if (!err) {
			count++;
		}
	}

	shell_print(shell, "Disconnected %d devices", count);
	return 0;
}

static int ble_disconn_name(const struct shell *shell, char *name)
{
	unsigned int i;
	struct ble_scanned_dev *dev;

	for (i = 0; i < MAX_SCAN_RESULTS; i++) {
		dev = get_scanned_device(i);
		if (dev == NULL) {
			break;
		}
		if (strcmp(name, dev->name) == 0) {
			shell_print(shell, "Disconnecting %s", name);
			return ble_disconn_mac(shell, dev->addr);
		}
	}

	shell_print(shell, "BLE device %s not found", name);
	return -EINVAL;
}

static enum ble_cmd_type get_cmd_type(char *arg)
{
	enum ble_cmd_type ret;

	if (strcmp(arg, "all") == 0) {
		ret = BLE_CMD_ALL;
	} else if (strchr(arg, ':')) {
		ret = BLE_CMD_MAC;
	} else {
		ret = BLE_CMD_NAME;
	}
	return ret;
}

/* COMMAND HANDLERS*/

static int cmd_info_gateway(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_fw_info(shell);
	return 0;
}

static int cmd_info_modem(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_modem_info(shell);
	return 0;
}

static int cmd_info_cloud(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	print_cloud_info(shell);
	return 0;
}

static int cmd_info_ctlr(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_ctlr_info(shell);
	return 0;
}

static int cmd_info_scan(const struct shell *shell, size_t argc,
			      char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_scan_info(shell);
	return 0;
}

static int cmd_info_conn(const struct shell *shell, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_conn_info(shell);
	return 0;
}

static int cmd_info_irq(const struct shell *shell, size_t argc,
			char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_irq_info(shell);
	return 0;
}

static int cmd_ble_scan(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	scan_start();
	return 0;
}

static int cmd_ble_save(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "save connection list to cloud...");
	return ble_conn_save(shell);
}

static int cmd_ble_conn(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		return -EINVAL;
	}

	switch (get_cmd_type(argv[1])) {
	case BLE_CMD_ALL:
		shell_print(shell, "connecting all BLE devices...");
		return ble_conn_all(shell);
	case BLE_CMD_MAC:
		shell_print(shell, "connecting to MAC %s", argv[1]);
		return ble_conn_mac(shell, argv[1]);
	case BLE_CMD_NAME:
		shell_print(shell, "connecting to name %s", argv[1]);
		return ble_conn_name(shell, argv[1]);
	default:
		return -EINVAL;
	}
}

static int cmd_ble_disc(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		return -EINVAL;
	}

	switch (get_cmd_type(argv[1])) {
	case BLE_CMD_ALL:
		shell_print(shell, "disconnecting all BLE devices...");
		return ble_disconn_all(shell);
	case BLE_CMD_MAC:
		shell_print(shell, "disconnecting MAC %s", argv[1]);
		return ble_disconn_mac(shell, argv[1]);
	case BLE_CMD_NAME:
		shell_print(shell, "disconnecting name %s", argv[1]);
		return ble_disconn_name(shell, argv[1]);
	default:
		return -EINVAL;
	}
}

static int cmd_ble_notif_en(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		return -EINVAL;
	}

	switch (get_cmd_type(argv[1])) {
	case BLE_CMD_ALL:
		shell_print(shell, "enable notifications on all devices");
		struct ble_device_conn *dev;
		int i;
		int count = 0;

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			dev = get_connected_device(i);
			if (dev == NULL) {
				continue;
			}
			shell_print(shell, "enabling on MAC %s...", dev->addr);
			ble_subscribe_all(dev->addr, BT_GATT_CCC_NOTIFY);
			count++;
		}
		if (!count) {
			shell_warn(shell, "Connect a device first.");
		}
		break;
	case BLE_CMD_MAC:
		if ((argc < 3) || (strcmp(argv[2], "all") == 0)) {
			shell_print(shell, "enable all notifications on MAC %s",
				    argv[1]);
			ble_subscribe_all(argv[1], BT_GATT_CCC_NOTIFY);
			return 0;
		}
		uint16_t handle = atoi(argv[2]);

		shell_print(shell, "enable notification on MAC %s handle %u",
			    argv[1], handle);
		ble_subscribe_handle(argv[1], handle, BT_GATT_CCC_NOTIFY);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int cmd_ble_notif_dis(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		return -EINVAL;
	}

	switch (get_cmd_type(argv[1])) {
	case BLE_CMD_ALL:
		shell_print(shell, "disable notifications on all devices");
		struct ble_device_conn *dev;
		int i;

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			dev = get_connected_device(i);
			if (dev == NULL) {
				continue;
			}
			shell_print(shell, "disabling on MAC %s...",
				    dev->addr);
			ble_subscribe_all(dev->addr, 0);
		}
		break;
	case BLE_CMD_MAC:
		if ((argc < 3) || (strcmp(argv[2], "all") == 0)) {
			shell_print(shell, "disable all notifications on MAC %s",
				    argv[1]);
			ble_subscribe_all(argv[1], 0);
			return 0;
		}
		uint16_t handle = atoi(argv[2]);

		shell_print(shell, "disable notification on MAC %s handle %u",
			    argv[1], handle);
		ble_subscribe_handle(argv[1], handle, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void set_at_prompt(const struct shell *shell, bool at_mode)
{
	static bool normal_prompt = true;

	if (at_mode && normal_prompt) {
		normal_prompt = false;

		shell_print(shell, "Type 'exit' to exit AT mode");
		shell_set_unfiltered_argument(0);
		shell_prompt_change(shell, "");

		/* hack: manually turn color mode back on; from
		 * zephyr/subsys/shell/shell_ops.h:flag_use_colors_set() */
		shell->ctx->internal.flags.use_colors = false;
		//flag_use_colors_set(shell, false);

	} else if (!at_mode && !normal_prompt) {
		normal_prompt = true;

		shell_set_unfiltered_argument(CONFIG_SHELL_ARGC_MAX);
		shell_prompt_change(shell, CONFIG_SHELL_PROMPT_SECURE);

		/* hack: manually turn off select mode as is done in
		 * zephyr/subsys/shell/shell.c:alt_metakeys_handle()
		 * when it receives SHELL_VT100_ASCII_ALT_R -- there is
		 * no API for doing so
		 */
		shell_set_root_cmd(NULL);

		/* hack: manually turn color mode back on; from
		 * zephyr/subsys/shell/shell_ops.h:flag_use_colors_set() */
		shell->ctx->internal.flags.use_colors = true;
		//flag_use_colors_set(shell, true);
	}
}

static int app_cmd_at(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (strcmp(argv[1], "enable") == 0) {
		shell_set_root_cmd("at");
		set_at_prompt(shell, true);
		return 0;
	}
	else if (strcmp(argv[1], "exit") == 0) {
		set_at_prompt(shell, false);
		return 0;
	} else {
		set_at_prompt(shell, true);
	}
	int err;

	if (strcmp(argv[1], "AT+CFUN=4") == 0) {
		control_cloud_connection(false); /* disable reconnections */
	} else if (strcmp(argv[1], "AT+CFUN=1") == 0) {
		control_cloud_connection(true); /* enable reconnections */
	}

	char *c = argv[1];
	int i = 0;

	while ((c = strstr(c, "\\n")) != NULL) {
		c[0] = '\r';
		c[1] = '\n';
		i++;
	}

	if (IS_ENABLED(CONFIG_NRF_CLOUD_GATEWAY_LOG_LEVEL_DBG)) {
		if (i) {
			shell_print(shell, "replaced %d \\n with <CR><LF>", i);
		}

		for (int i = 1; i < argc; i++) {
			shell_print(shell, "argv[%d]:%s", i, argv[i]);
		}
	}

	err = at_cmd_write(argv[1], response_buf, sizeof(response_buf), NULL);
	if (err) {
		shell_error(shell, "ERROR");
		return -EINVAL;
	}

	shell_print(shell, "%sOK", response_buf);

	return 0;
}

static int app_exit(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	set_at_prompt(shell, false);
	shell_set_unfiltered_argument(CONFIG_SHELL_ARGC_MAX);
	return 0;
}

static int cmd_reboot(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	device_shutdown(true);
	return 0;
}

static int cmd_shutdown(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	device_shutdown(false);
	return 0;
}

/*
example: 
fota firmware.nrfcloud.com 3c7003c6-45a0-4a74-9023-1e006ceeb835/APP*30f6ce17*1.4.0/app_update.bin
*/
static int cmd_fota(const struct shell *shell, size_t argc, char **argv)
{
	char *host = argv[1];
	char *path = argv[2];
	int sec_tag = -1;
	char *apn = NULL;
	size_t frag = 0;

	if (argc > 3) {
		sec_tag = atoi(argv[3]);
	}
	if (argc > 4) {
		frag = atoll(argv[4]);
	}
	if (argc > 5) {
		apn = argv[5];
	}
	shell_print(shell, "starting FOTA download from host:%s, path:%s, "
			   "sec_tag:%d, apn:%s, frag_size:%zd",
			   host, path, sec_tag, apn ? apn : "<n/a>", frag);

	char *space = strstr(path, " ");

	if (space) {
		shell_print(shell, "mcuboot download detected");
	}
	int err = fota_download_start(host, path, sec_tag, apn, frag);
	if (err) {
		shell_error(shell, "Error %d starting download", err);
	}
	return 0;
}

int check_passwd(char *passwd)
{
	return strcmp(passwd, DEFAULT_PASSWORD);
}

int is_valid_passwd(char *passwd)
{
	if (strlen(passwd) < 8) {
		return -EINVAL;
	} else {
		return 0;
	}
}

int set_passwd(char *passwd)
{
	return -ENOTSUP;
}

static int cmd_login(const struct shell *shell, size_t argc, char **argv)
{
	static uint32_t attempts = 0;

	if (argc < 2) {
		shell_print(shell, "Access requires: <password>");
		return -EINVAL;
	}

	if (check_passwd(argv[1]) == 0) {
		attempts = 0;
		/* not public: shell_obscure_set(shell, false); */
		shell->ctx->internal.flags.obscure = false;
		shell_history_purge(shell->history);
		shell_set_root_cmd(NULL);
		shell_prompt_change(shell, CONFIG_SHELL_PROMPT_SECURE);
		shell_print(shell, "nRF Cloud Gateway\n");
		shell_print(shell, "Hit tab for help.\n");
		return 0;
	} else {
		shell_error(shell, "Incorrect password!");
		attempts++;
		if (attempts > 3) {
			k_sleep(K_SECONDS(attempts));
		}
		return -EINVAL;
	}
	return 0;
}

static int cmd_passwd(const struct shell *shell, size_t argc, char **argv)
{
	int err;

	err = check_passwd(argv[1]);
	if (err) {
		shell_error(shell, "Incorrect password!");
		return err;
	}

	err = is_valid_passwd(argv[2]);
	if (err) {
		shell_error(shell, "Invalid password.  Must be 8 characters or longer.");
		return err;
	}

	err = set_passwd(argv[2]);
	if (!err) {
		shell_print(shell, "Password changed.");
	} else {
		shell_error(shell, "Unable to store password.");
	}

	return err;
}

static int cmd_logout(const struct shell *shell, size_t argc, char **argv)
{
	shell_set_root_cmd("login");
	/* not public: shell_obscure_set(shell, true); */
	shell->ctx->internal.flags.obscure = true;
	shell_prompt_change(shell, CONFIG_SHELL_PROMPT_UART);
	shell_print(shell, "\n");
	return 0;
}

void cli_init(void)
{
	shell_set_root_cmd("login");
#if defined(CONFIG_STARTING_LOG_OVERRIDE)
	for (int i = 0; i < log_src_cnt_get(CONFIG_LOG_DOMAIN_ID); i++) {
		if (IS_ENABLED(CONFIG_NRF_CLOUD_GATEWAY_LOG_LEVEL_DBG)) {
			printk("%d. %s\n", i,
			       log_source_name_get(CONFIG_LOG_DOMAIN_ID, i));
		}
		log_filter_set(NULL, CONFIG_LOG_DOMAIN_ID, i,
			       CONFIG_STARTING_LOG_LEVEL);
	}
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_info,
	SHELL_CMD(gateway, NULL, "Gateway information.",
		  cmd_info_gateway),
	SHELL_CMD(modem, NULL, "Modem information.", cmd_info_modem),
	SHELL_CMD(cloud, NULL, "Cloud information.", cmd_info_cloud),
	SHELL_CMD(ctlr, NULL, "BLE controller information.",
		  cmd_info_ctlr),
	SHELL_CMD(scan, NULL, "Bluetooth scan results.",
		  cmd_info_scan),
	SHELL_CMD(conn, NULL, "Connected Bluetooth devices information.",
		  cmd_info_conn),
	SHELL_COND_CMD(CONFIG_GATEWAY_DBG_CMDS,
		       irq, NULL, "Dump IRQ table.", cmd_info_irq),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(info, &sub_info, "Informational commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble,
	SHELL_CMD(scan, NULL, "Scan for BLE devices.", cmd_ble_scan),
	SHELL_CMD(save, NULL, "Save desired connections to shadow.",
		  cmd_ble_save),
	SHELL_CMD_ARG(conn, NULL, "<all | name | MAC> Connect to BLE device(s).",
		      cmd_ble_conn, 2, 0),
	SHELL_CMD_ARG(disc, NULL, "<all | name | MAC> Disconnect BLE device(s).",
		      cmd_ble_disc, 2, 0),
	SHELL_CMD_ARG(en, NULL, "<all | MAC [all | handle]> Enable "
		      "notifications on BLE device(s).",
		      cmd_ble_notif_en, 1, 2),
	SHELL_CMD_ARG(dis, NULL, "<all | MAC [all | handle]> Disable "
		      "notifications on BLE device(s).",
		      cmd_ble_notif_dis, 1, 2),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_ARG_REGISTER(ble, &sub_ble, "Bluetooth commands", NULL, 0, 3);

SHELL_CMD_ARG_REGISTER(fota, NULL, "<host> <path> [sec_tag] [frag_size] [apn] "
				   "firmware over-the-air update.",
		       cmd_fota, 2, 3);
SHELL_CMD_ARG_REGISTER(at, NULL, "<enable | AT<cmd> | exit> Execute an AT command.  Use "
				 "<at enable> first to remain in AT command "
				 "mode until 'exit'.", app_cmd_at, 2, 0);
SHELL_CMD_REGISTER(reboot, NULL, "Reboot the gateway.", cmd_reboot);
SHELL_CMD_REGISTER(shutdown, NULL, "Shutdown the gateway.", cmd_shutdown);
SHELL_CMD_REGISTER(exit, NULL, "Exit 'select at' mode.", app_exit);
SHELL_CMD_ARG_REGISTER(login, NULL, "<password>", cmd_login, 2, 0);
SHELL_CMD_ARG_REGISTER(passwd, NULL, "<oldpw> <newpw>", cmd_passwd, 3, 0);
SHELL_CMD_REGISTER(logout, NULL, "Log out.", cmd_logout);

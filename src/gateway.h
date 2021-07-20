#ifndef GATEWAY_CLOUD_TRANSPORT__
#define GATEWAY_CLOUD_TRANSPORT__

#define NRF_CLOUD_CLIENT_ID_LEN  10
extern char gateway_id[NRF_CLOUD_CLIENT_ID_LEN+1];

struct nrf_cloud_gw_data;

void device_shutdown(bool reboot);
void control_cloud_connection(bool enable);
void cli_init(void);
bool get_lte_connection_status(void);
bool get_cloud_connection_status(void);
void init_gateway(void);
int gw_client_id_get(char **id, size_t *id_len);

#if defined(CONFIG_ENTER_52840_MCUBOOT_VIA_BUTTON)
/* functions defined in the board's .c file */
int nrf52840_reset_to_mcuboot(void);
int nrf52840_wait_boot_complete(int timeout_ms);
#endif

#endif

#ifndef PERIPHERAL_DFU_H_
#define PERIPHERAL_DFU_H_

int peripheral_dfu_init(void);
int peripheral_dfu_config(const char *addr, int size, const char *version,
			   uint32_t crc, bool init_pkt, bool use_printk);
int peripheral_dfu_start(const char *host, const char *file, int sec_tag,
			 const char *apn, size_t fragment_size);

#endif
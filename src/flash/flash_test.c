#include <zephyr.h>
#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#if (CONFIG_SPI_FLASH_W25QXXDV - 0)
/* NB: W25Q16DV is a JEDEC spi-nor device, but has a separate driver. */
#define FLASH_DEVICE CONFIG_SPI_FLASH_W25QXXDV_DRV_NAME
#define FLASH_NAME "W25QXXDV"
#elif (CONFIG_SPI_NOR - 0) || \
    DT_NODE_HAS_STATUS(DT_INST(0, jedec_spi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, jedec_spi_nor))
#define FLASH_NAME "JEDEC SPI-NOR"
#elif (CONFIG_NORDIC_QSPI_NOR - 0) || \
    DT_NODE_HAS_STATUS(DT_INST(0, nordic_qspi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, nordic_qspi_nor))
#define FLASH_NAME "JEDEC QSPI-NOR"
#else
#error Unsupported flash driver
#endif

#if defined(CONFIG_BOARD_ADAFRUIT_FEATHER_STM32F405)
#define FLASH_TEST_REGION_OFFSET 0xf000
#else
#define FLASH_TEST_REGION_OFFSET 0xff000
#endif
#define FLASH_SECTOR_SIZE 4096

#define EXT_MEM_CTRL_PIN 9

/**@brief Set the external mem control pin to high to 
  * enable access to the external memory chip. */
static int flash_test_init(struct device *dev)
{
	struct device *port;
	int err;

	printk("%s\n", __func__);

	port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!port) {
		printk("error on device_get_binding(): %d\n", errno);
		return -EIO;
	}

	err = gpio_pin_configure(port, EXT_MEM_CTRL_PIN, GPIO_OUTPUT_HIGH);
	if (err) {
		printk("error on gpio_pin_configure(): %d\n", err);
		return err;
	}

	err = gpio_pin_set(port, EXT_MEM_CTRL_PIN, 1);
	if (err) {
		printk("error on gpio_pin_set(): %d\n", err);
		return err;
	}

	printk("init complete\n");
	return err;
}

SYS_INIT(flash_test_init, POST_KERNEL, CONFIG_SPI_NOR_INIT_PRIORITY);

static int flash_test(struct device *dev)
{
	const uint8_t expected[] = {0x55, 0xaa, 0x66, 0x99};
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	struct device *flash_dev;
	int rc;
	int ret = 0;

	printk("\n" FLASH_NAME " SPI flash testing\n");
	printk("==========================\n");

	if (flash_test_init(NULL)) {
		printk("SPI Control init failed\n");
	}

	flash_dev = device_get_binding(FLASH_DEVICE);

	if (!flash_dev) {
		printk("SPI flash driver %s was not found!\n",
		       FLASH_DEVICE);
		ret = -EIO;
		goto error;
	}

	/* Write protection needs to be disabled before each write or
	    * erase, since the flash component turns on write protection
	    * automatically after completion of write and erase
	    * operations.
	    */
	printk("\nTest 1: Flash erase: ");
	flash_write_protection_set(flash_dev, false);

	rc = flash_erase(flash_dev, FLASH_TEST_REGION_OFFSET,
			 FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printk("FAIL - %d\n", rc);
	} else {
		printk("PASS\n");
	}

	printk("\nTest 2: Flash write %u bytes: ", len);
	flash_write_protection_set(flash_dev, false);

	rc = flash_write(flash_dev, FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printk("FAIL - %d\n", rc);
		ret = -EIO;
		goto error;
	} else {
		printk("PASS\n");
	}

	printk("\nTest 3: Flash read %u bytes: ", len);
	memset(buf, 0, len);
	rc = flash_read(flash_dev, FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printk("FAIL - %d\n", rc);
		ret = -EIO;
		goto error;
	} else {
		printk("PASS\n");
	}

	printk("\nTest 4: Compare %u bytes: ", len);
	if (memcmp(expected, buf, len) != 0) {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + len;

		ret= -EFAULT;
		printk("FAIL - %d\n", ret);
		while (rp < rpe) {
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
		goto error;
	} else {
		printk("PASS\n");
	}

error:
	for (;;) {
	}
	return ret;
}

SYS_INIT(flash_test, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

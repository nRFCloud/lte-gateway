/*
 * Copyright (c) 2020 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <device.h>

#define RESET_PIN CONFIG_BOARD_NRF52840_GPIO_RESET_PIN
#define BOOT_PIN CONFIG_BOARD_NRF52840_GPIO_BOOT_SELECT_PIN
#define WAIT_BOOT_INTERVAL_MS 50

static int nrf52840_reset_assert(bool boot_select)
{
	int err;
	const struct device *port;

	port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!port) {
		return -EIO;
	}

	/* Configure pin as output and initialize it to low. */
	err = gpio_pin_configure(port, RESET_PIN, GPIO_OUTPUT_LOW);
	if (err) {
		return err;
	}

	if (boot_select) {
		printk("Resetting nrf52840 in MCUboot "
		       "USB serial update mode\n");
		/* delay to ensure logging finishes before reset */
		k_sleep(K_SECONDS(2));
	}
	err = gpio_pin_configure(port, BOOT_PIN, GPIO_OUTPUT |
				 GPIO_OPEN_DRAIN | GPIO_PULL_UP);
	if (err) {
		return err;
	}

	err = gpio_pin_set(port, BOOT_PIN, boot_select ? 0 : 1);
	if (err) {
		return err;
	}

	err = gpio_pin_set(port, RESET_PIN, 1);
	return err;
}

static int nrf52840_boot_select_deassert(void)
{
	const struct device *port;
	int err;

	port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!port) {
		return -EIO;
	}

	err = gpio_pin_set(port, BOOT_PIN, 1);
	if (err) {
		return err;
	}

	k_sleep(K_MSEC(10));

	err = gpio_pin_configure(port, BOOT_PIN, GPIO_INPUT | GPIO_PULL_UP);
	return err;
}

static int nrf52840_reset_deassert(void)
{
	const struct device *port;

	port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!port) {
		return -EIO;
	}

	return gpio_pin_set(port, RESET_PIN, 0);
}

int nrf52840_wait_boot_complete(int timeout_ms)
{
	const struct device *port;
	int err;
	int total_time;

	port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!port) {
		return -EIO;
	}

	/* make the boot select pin a pulled up input, so we can
	 * read it to see when the 52840 has rebooted into its application
	 */
	err = gpio_pin_configure(port, BOOT_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		return err;
	}

	total_time = 0;
	do {
		k_sleep(K_MSEC(WAIT_BOOT_INTERVAL_MS));
		total_time += WAIT_BOOT_INTERVAL_MS;
		err = gpio_pin_get_raw(port, BOOT_PIN);
		if (err < 0) {
			return err;
		}
		if (err && (timeout_ms > 0) && (total_time > timeout_ms)) {
			return -ETIMEDOUT;
		}
	} while (err == 1);

	return 0;
}

int nrf52840_reset_to_mcuboot(void)
{
	int err;

	err = nrf52840_reset_assert(true);
	if (err) {
		return err;
	}
	k_sleep(K_MSEC(10));

	err = nrf52840_reset_deassert();
	if (err) {
		return err;
	}

	k_sleep(K_SECONDS(5));
	return nrf52840_boot_select_deassert();
}

int bt_hci_transport_setup(struct device *h4)
{
	char c;
	int err;

	/* Reset the nRF52840 and let it wait until the pin is
	 * pulled low again before running to main to ensure
	 * that it won't send any data until the H4 device
	 * is setup and ready to receive.
	 */
	err = nrf52840_reset_assert(false);
	if (err) {
		return err;
	}

	/* Wait for the nRF52840 peripheral to stop sending data.
	 *
	 * It is critical (!) to wait here, so that all bytes
	 * on the lines are received and drained correctly.
	 */
	k_sleep(K_MSEC(10));

	/* Drain bytes */
	while (uart_fifo_read(h4, &c, 1)) {
		continue;
	}

	/* We are ready, let the nRF52840 run to main */
	nrf52840_reset_deassert();

	return 0;
}


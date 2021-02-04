/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <nrf9160.h>
#include <hal/nrf_gpio.h>

#include "nrf_cloud_transport.h"
#include "gateway.h"
#include "ui.h"
#include "led_pwm.h"

LOG_MODULE_REGISTER(ui, CONFIG_UI_LOG_LEVEL);

static enum ui_led_pattern current_led_state;

static ui_callback_t callback;

static bool falling_edge = true;
static bool shutdown;
static bool held_at_startup;
#if defined(CONFIG_ENTER_52840_MCUBOOT_VIA_BUTTON)
static bool boot_select;
#endif

#if !defined(CONFIG_UI_LED_USE_PWM)
static struct k_delayed_work leds_update_work;

/**@brief Update LEDs state. */
static void leds_update(struct k_work *work)
{
	static bool led_on;
	static uint8_t current_led_on_mask;
	uint8_t led_on_mask;

	led_on_mask = UI_LED_GET_ON(current_led_state);
	led_on = !led_on;

	if (led_on) {
		led_on_mask |= UI_LED_GET_BLINK(current_led_state);
	} else {
		led_on_mask &= ~UI_LED_GET_BLINK(current_led_state);
	}

	if (led_on_mask != current_led_on_mask) {
#if defined(CONFIG_DK_LIBRARY)
		dk_set_leds(led_on_mask);
#endif
		current_led_on_mask = led_on_mask;
	}

	if (work) {
		if (led_on) {
			k_delayed_work_submit(&leds_update_work,
						UI_LED_ON_PERIOD_NORMAL);
		} else {
			k_delayed_work_submit(&leds_update_work,
						UI_LED_OFF_PERIOD_NORMAL);
		}
	}
}
#endif /* CONFIG_UI_LED_USE_PWM */

static void button_press_timer_handler(struct k_timer *timer)
{
	if (ui_button_is_active(1)) {
		ui_led_set_pattern(UI_LTE_DISCONNECTED, PWM_DEV_0);
		ui_led_set_pattern(UI_BLE_OFF, PWM_DEV_1);
		shutdown = true;
		LOG_INF("Button pressed; disconnecting!");
	}
}
K_TIMER_DEFINE(button_press_timer, button_press_timer_handler, NULL);

#if defined(CONFIG_ENTER_52840_MCUBOOT_VIA_BUTTON)
static void boot_select_timer_handler(struct k_timer *timer)
{
	if (ui_button_is_active(1)) {
		boot_select = true;
		ui_led_set_pattern(UI_BLE_UPDATE, PWM_DEV_1);
		printk("52840 mcuboot selected\n");
	}
}
K_TIMER_DEFINE(boot_select_timer, boot_select_timer_handler, NULL);

bool is_boot_selected(void)
{
	return boot_select;
}
#endif

void power_button_handler(struct ui_evt evt)
{
	if (falling_edge && ui_button_is_active(1)) {
		k_timer_start(&button_press_timer, K_SECONDS(1), K_SECONDS(0));
		falling_edge = false;
	} else if (!falling_edge && !ui_button_is_active(1)) {
		falling_edge = true;

		if (shutdown && !held_at_startup) {
			device_shutdown(false);
		}
		held_at_startup = false;
	}
}

/**@brief Callback for button events from the DK buttons and LEDs library. */
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	struct ui_evt evt;
	uint8_t btn_num;

	while (has_changed) {
		btn_num = 0;

		/* Get bit position for next button that changed state. */
		for (uint8_t i = 0; i < 32; i++) {
			if (has_changed & BIT(i)) {
				btn_num = i + 1;
				break;
			}
		}

		/* Button number has been stored, remove from bitmask. */
		has_changed &= ~(1UL << (btn_num - 1));

		evt.button = btn_num;
		evt.type = (button_states & BIT(btn_num - 1))
				? UI_EVT_BUTTON_ACTIVE
				: UI_EVT_BUTTON_INACTIVE;

		callback(evt);
	}
}



void ui_led_set_pattern(enum ui_led_pattern state, uint8_t led_num)
{
	current_led_state = state;
#ifdef CONFIG_UI_LED_USE_PWM
	ui_led_set_effect(state, led_num);
#else
	current_led_state = state;
#endif /* CONFIG_UI_LED_USE_PWM */
}

enum ui_led_pattern ui_led_get_pattern(void)
{
	return current_led_state;
}

int ui_led_set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t led_num)
{
#ifdef CONFIG_UI_LED_USE_PWM
	return ui_led_set_rgb(red, green, blue, led_num);
#else
	return -ENOTSUP;
#endif /* CONFIG_UI_LED_USE_PWM */
}

void ui_led_set_state(uint32_t led, uint8_t value)
{
#if !defined(CONFIG_UI_LED_USE_PWM)
	if (value) {
		current_led_state |= BIT(led - 1);
	} else {
		current_led_state &= ~BIT(led - 1);
	}
#endif
}

int ui_init(ui_callback_t cb)
{
	int err = 0;

#ifdef CONFIG_UI_LED_USE_PWM
	err = ui_leds_init();
	if (err) {
		LOG_ERR("Error when initializing PWM controlled LEDs");
		return err;
	}
#else
	err = dk_leds_init();
	if (err) {
		LOG_ERR("Could not initialize leds, err code: %d\n", err);
		return err;
	}

	err = dk_set_leds_state(0x00, DK_ALL_LEDS_MSK);
	if (err) {
		LOG_ERR("Could not set leds state, err code: %d\n", err);
		return err;
	}

	k_delayed_work_init(&leds_update_work, leds_update);
	k_delayed_work_submit(&leds_update_work, K_NO_WAIT);
#endif /* CONFIG_UI_LED_USE_PWM */

	if (cb) {
		callback = cb;

		err = dk_buttons_init(button_handler);
		if (err) {
			LOG_ERR("Could not initialize buttons, err code: %d\n",
				err);
			return err;
		}

		if (ui_button_is_active(1)) {
			falling_edge = false;
			held_at_startup = true;
#if defined(CONFIG_ENTER_52840_MCUBOOT_VIA_BUTTON)
			LOG_DBG("Starting boot select timer");
			k_timer_start(&boot_select_timer, K_SECONDS(4),
				      K_SECONDS(0));
#endif
		}
	}

	return err;
}

bool ui_button_is_active(uint32_t button)
{
#if defined(CONFIG_DK_LIBRARY)
	return dk_get_buttons() & BIT((button - 1));
#else
	return false;
#endif
}

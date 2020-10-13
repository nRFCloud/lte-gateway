/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <drivers/pwm.h>
#include <string.h>

#include "ui.h"
#include "led_pwm.h"
#include "led_effect.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(ui_led_pwm, CONFIG_UI_LOG_LEVEL);

struct led {
	struct device *pwm_dev;

	size_t id;
	struct led_color color;
	const struct led_effect *effect;
	uint16_t effect_step;
	uint16_t effect_substep;

	struct k_delayed_work work;
};

static const struct led_effect effect[] = {
	[UI_LTE_DISCONNECTED] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						       UI_LED_OFF_PERIOD_NORMAL,
						       UI_LTE_DISCONNECTED_COLOR),
	[UI_LTE_CONNECTING] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						     UI_LED_OFF_PERIOD_NORMAL,
						     UI_LTE_CONNECTING_COLOR),
	[UI_LTE_CONNECTED] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						    UI_LED_OFF_PERIOD_NORMAL,
						    UI_LTE_CONNECTED_COLOR),
	[UI_CLOUD_CONNECTING] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						       UI_LED_OFF_PERIOD_NORMAL,
						       UI_CLOUD_CONNECTING_COLOR),
	[UI_CLOUD_CONNECTED] = LED_EFFECT_LED_ON(UI_CLOUD_CONNECTED_COLOR),
	[UI_CLOUD_PAIRING] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						    UI_LED_OFF_PERIOD_NORMAL,
						    UI_CLOUD_PAIRING_COLOR),
	[UI_ACCEL_CALIBRATING] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
							UI_LED_OFF_PERIOD_NORMAL,
							UI_ACCEL_CALIBRATING_COLOR),
	[UI_LED_ERROR_CLOUD] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
						      UI_LED_OFF_PERIOD_ERROR,
						      UI_LED_ERROR_CLOUD_COLOR),
	[UI_LED_ERROR_BSD_REC] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
							UI_LED_OFF_PERIOD_ERROR,
							UI_LED_ERROR_BSD_REC_COLOR),
	[UI_LED_ERROR_BSD_IRREC] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
							  UI_LED_OFF_PERIOD_ERROR,
							  UI_LED_ERROR_BSD_IRREC_COLOR),
	[UI_LED_ERROR_LTE_LC] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
						       UI_LED_OFF_PERIOD_ERROR,
						       UI_LED_ERROR_LTE_LC_COLOR),
	[UI_LED_ERROR_UNKNOWN] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
							UI_LED_OFF_PERIOD_ERROR,
							UI_LED_ERROR_UNKNOWN_COLOR),
	[UI_LED_GPS_SEARCHING] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
							UI_LED_OFF_PERIOD_ERROR,
							UI_LED_GPS_SEARCHING_COLOR),
	[UI_LED_GPS_BLOCKED] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
						      UI_LED_OFF_PERIOD_ERROR,
						      UI_LED_GPS_BLOCKED_COLOR),
	[UI_LED_GPS_FIX] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_ERROR,
						  UI_LED_OFF_PERIOD_ERROR,
						  UI_LED_GPS_FIX_COLOR),
	[UI_BLE_OFF] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
					      UI_LED_OFF_PERIOD_NORMAL,
					      UI_BLE_OFF_COLOR),
	[UI_BLE_DISCONNECTED] = LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
						       UI_LED_OFF_PERIOD_NORMAL,
						       UI_BLE_DISCONNECTED_COLOR),
	[UI_BLE_CONNECTED] = LED_EFFECT_LED_ON(UI_BLE_CONNECTED_COLOR),
};

static struct led_effect custom_effect =
	LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
			       UI_LED_OFF_PERIOD_NORMAL,
			       LED_NOCOLOR());

static struct led leds_1;

static struct led leds_2;

static const size_t led_1_pins[3] = {
	CONFIG_UI_LED_1_RED_PIN,
	CONFIG_UI_LED_1_GREEN_PIN,
	CONFIG_UI_LED_1_BLUE_PIN,
};

static const size_t led_2_pins[3] = {
	CONFIG_UI_LED_2_RED_PIN,
	CONFIG_UI_LED_2_GREEN_PIN,
	CONFIG_UI_LED_2_BLUE_PIN,
};

static void pwm_out(struct led *led, struct led_color *color, uint8_t led_num)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(color->c); i++) {
		if (led_num == 0) {
			pwm_pin_set_usec(led->pwm_dev, led_1_pins[i],
					 255, color->c[i], 0);
		} else {
			pwm_pin_set_usec(led->pwm_dev, led_2_pins[i],
					 255, color->c[i], 0);
		}
	}
}

static void pwm_off(struct led *led, uint8_t led_num)
{
	struct led_color nocolor = {0};

	pwm_out(led, &nocolor, led_num);
}

static void led_1_work_handler(struct k_work *work)
{
	struct led *led = CONTAINER_OF(work, struct led, work);
	const struct led_effect_step *effect_step =
		&leds_1.effect->steps[leds_1.effect_step];
	int substeps_left = effect_step->substep_count - leds_1.effect_substep;

	for (size_t i = 0; i < ARRAY_SIZE(leds_1.color.c); i++) {
		int diff = (effect_step->color.c[i] - leds_1.color.c[i]) /
			substeps_left;
		leds_1.color.c[i] += diff;
	}

	pwm_out(led, &leds_1.color, 0);

	leds_1.effect_substep++;
	if (leds_1.effect_substep == effect_step->substep_count) {
		leds_1.effect_substep = 0;
		leds_1.effect_step++;

		if (leds_1.effect_step == leds_1.effect->step_count) {
			if (leds_1.effect->loop_forever) {
				leds_1.effect_step = 0;
			}
		} else {
			__ASSERT_NO_MSG(leds_1.effect->steps[leds_1.effect_step].substep_count > 0);
		}
	}

	if (leds_1.effect_step < leds_1.effect->step_count) {
		int32_t next_delay =
			leds_1.effect->steps[leds_1.effect_step].substep_time;

		k_delayed_work_submit(&leds_1.work, K_MSEC(next_delay));
	}
}

static void led_2_work_handler(struct k_work *work)
{
	struct led *led = CONTAINER_OF(work, struct led, work);

	const struct led_effect_step *effect_step =
		&leds_2.effect->steps[leds_2.effect_step];
	int substeps_left = effect_step->substep_count - leds_2.effect_substep;

	for (size_t i = 0; i < ARRAY_SIZE(leds_2.color.c); i++) {
		int diff = (effect_step->color.c[i] - leds_2.color.c[i]) /
			substeps_left;
		leds_2.color.c[i] += diff;
	}

	pwm_out(led, &leds_2.color, 1);

	leds_2.effect_substep++;
	if (leds_2.effect_substep == effect_step->substep_count) {
		leds_2.effect_substep = 0;
		leds_2.effect_step++;

		if (leds_2.effect_step == leds_2.effect->step_count) {
			if (leds_2.effect->loop_forever) {
				leds_2.effect_step = 0;
			}
		} else {
			__ASSERT_NO_MSG(leds_2.effect->steps[leds_2.effect_step].substep_count > 0);
		}
	}

	if (leds_2.effect_step < leds_2.effect->step_count) {
		int32_t next_delay =
			leds_2.effect->steps[leds_2.effect_step].substep_time;

		k_delayed_work_submit(&leds_2.work, K_MSEC(next_delay));
	}
}

static void led_update(struct led *led)
{
	k_delayed_work_cancel(&led->work);

	led->effect_step = 0;
	led->effect_substep = 0;

	if (!led->effect) {
		LOG_DBG("No effect set");
		return;
	}

	__ASSERT_NO_MSG(led->effect->steps);

	if (led->effect->step_count > 0) {
		int32_t next_delay =
			led->effect->steps[led->effect_step].substep_time;

		k_delayed_work_submit(&led->work, K_MSEC(next_delay));
	} else {
		LOG_DBG("LED effect with no effect");
	}
}

int ui_leds_init(void)
{
	const char *dev1_name = CONFIG_UI_LED_PWM_1_DEV_NAME;
	const char *dev2_name = CONFIG_UI_LED_PWM_2_DEV_NAME;
	int err = 0;

	leds_1.pwm_dev = device_get_binding(dev1_name);
	leds_1.id = 0;
	leds_1.effect = &effect[UI_LTE_DISCONNECTED];

	if (!leds_1.pwm_dev) {
		LOG_ERR("Could not bind to device %s", dev1_name);
		return -ENODEV;
	}

	k_delayed_work_init(&leds_1.work, led_1_work_handler);
	led_update(&leds_1);

	leds_2.pwm_dev = device_get_binding(dev2_name);
	leds_2.id = 0;
	leds_2.effect = &effect[UI_BLE_DISCONNECTED];

	if (!leds_2.pwm_dev) {
		LOG_ERR("Could not bind to device %s", dev2_name);
		return -ENODEV;
	}

	k_delayed_work_init(&leds_2.work, led_2_work_handler);
	led_update(&leds_2);

	return err;
}

void ui_leds_start(void)
{
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	int err = device_set_power_state(leds.pwm_dev,
					 DEVICE_PM_ACTIVE_STATE,
					 NULL, NULL);

	if (err) {
		LOG_ERR("PWM enable failed");
	}
#endif
	led_update(&leds_1);
	led_update(&leds_2);
}

void ui_leds_stop(void)
{
	k_delayed_work_cancel(&leds_1.work);
	k_delayed_work_cancel(&leds_2.work);

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	int err = device_set_power_state(leds.pwm_dev,
					 DEVICE_PM_SUSPEND_STATE,
					 NULL, NULL);

	if (err) {
		LOG_ERR("PWM disable failed");
	}
#endif
	pwm_off(&leds_1, 0);
	pwm_off(&leds_2, 1);
}

void ui_led_set_effect(enum ui_led_pattern state, uint8_t led_num)
{
	if (led_num == 0) {
		leds_1.effect = &effect[state];
		led_update(&leds_1);
	} else {
		leds_2.effect = &effect[state];
		led_update(&leds_2);
	}
}

int ui_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t led_num)
{
	struct led_effect effect =
		LED_EFFECT_LED_BREATHE(UI_LED_ON_PERIOD_NORMAL,
				       UI_LED_OFF_PERIOD_NORMAL,
				       LED_COLOR(red, green, blue));

	memcpy((void *)custom_effect.steps, (void *)effect.steps,
	       effect.step_count * sizeof(struct led_effect_step));

	if (led_num == 0) {
		leds_1.effect = &custom_effect;
		led_update(&leds_1);
	} else {
		leds_2.effect = &custom_effect;
		led_update(&leds_2);
	}

	return 0;
}
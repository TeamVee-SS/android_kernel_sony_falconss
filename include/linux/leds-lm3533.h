/*
 * leds-lm3533.h - platform data structure for LM3533 LEDdriver chip
 *
 * Copyright (C) 2009 Antonio Ospite <ospite@studenti.unina.it>
 * Copyright (C) 2016 Caio Oliveira <caiooliveirafarias0@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_LEDS_LM3533_H
#define __LINUX_LEDS_LM3533_H

#define LM3533_SNS 0
#define LM3533_NOTIFICATION 1
#define LM3533_BACKLIGHT 2
#define LM3533_LEDS_MAX 3

#define LM3533_CURRENT_SINK_OUTPUT_CONFIGURATION 0x10
#define LM3533_STARTUP_SHUTDOWN_RAMP_RATES 0x12
#define LM3533_CONTROL_BANK_A_PWM_CONFIGURATION 0x14
#define LM3533_CONTROL_BANK_A_BRIGHTNESS_CONFIGURATION 0x1A
#define LM3533_CONTROL_BANK_A_FULL_SCALE_CURRENT 0x21
#define LM3533_CONTROL_ENABLE 0x27
#define LM3533_PATTERN_GENERATOR_ENABLE_ALS_SCALING_CONTROL 0x28
#define LM3533_OVP_FREQUENCY_PWM_POLARITY 0x2C
#define LM3533_ALS_DOWN_DELAY_CONTROL 0x33
#define LM3533_BRIGHTNESS_REGISTER_A 0x40
#define LM3533_BRIGHTNESS_REGISTER_C 0x42
#define LM3533_BRIGHTNESS_REGISTER_F 0x45
#define LM3533_PATTERN_GENERATOR_0_DELAY 0xA0

enum lm3533_rgb_brightness {
	SNS_NO = 0,
	SNS_B = 1,
	SNS_G = 10,
	SNS_GB = 11,
	SNS_R = 100,
	SNS_RB = 101,
	SNS_RG = 110,
	SNS_RGB = 111,
};

struct lm3533_led {
	const char *name;
};

struct lm3533_platform_data {
	struct lm3533_led leds[LM3533_LEDS_MAX];
	int hwint_gpio;
	int hwint_gpio_flag;
	u8 leds_size;
};

extern void lm3533_backlight_control(unsigned long brightness);

#endif /* __LINUX_LEDS_LM3533_H */

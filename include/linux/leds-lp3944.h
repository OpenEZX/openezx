/*
 * lp3944.h - platform data structure for lp3944 led controller
 *
 * Copyright (C) 2008 Antonio Ospite <ao2@openezx.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_I2C_LP3944_H
#define __LINUX_I2C_LP3944_H

#include <linux/leds.h>

#define LP3944_LED0 0
#define LP3944_LED1 1
#define LP3944_LED2 2
#define LP3944_LED3 3
#define LP3944_LED4 4
#define LP3944_LED5 5
#define LP3944_LED6 6
#define LP3944_LED7 7
#define LP3944_LEDS_MAX 8

#define LP3944_DIM0 0
#define LP3944_DIM1 1
#define LP3944_DIMS_MAX 2

/* period in 1/10 sec */
#define LP3944_PERIOD_MIN 0
#define LP3944_PERIOD_MAX 16

/* duty cycle is a percentage */
#define LP3944_DUTY_CYCLE_MIN 0
#define LP3944_DUTY_CYCLE_MAX 100

enum lp3944_status {
	LP3944_LED_STATUS_OFF  = 0x0,
	LP3944_LED_STATUS_ON   = 0x1,
	LP3944_LED_STATUS_DIM0 = 0x2,
	LP3944_LED_STATUS_DIM1 = 0x3
};

enum lp3944_type {
	LP3944_LED_TYPE_NONE,
	LP3944_LED_TYPE_LED,
	LP3944_LED_TYPE_LED_INVERTED,
};

struct lp3944_dim {
	unsigned period;
	unsigned dutycycle;
};

struct lp3944_led {
	u8 id;
	struct i2c_client *client;
	char *name;
	struct led_classdev ldev;
	enum lp3944_type type;
	enum lp3944_status status;
};

struct lp3944_platform_data {
	struct lp3944_dim dims[LP3944_DIMS_MAX];
	struct lp3944_led leds[LP3944_LEDS_MAX];
	unsigned dims_size;
	unsigned leds_size;
};

#endif /* __LINUX_I2C_LP3944_H */

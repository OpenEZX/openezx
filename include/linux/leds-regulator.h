/*
 * leds-regulator.h - platform data structure for regulator driven LEDs.
 *
 * Copyright (C) 2009 Antonio Ospite <ospite@studenti.unina.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_LEDS_REGULATOR_H
#define __LINUX_LEDS_REGULATOR_H

struct led_regulator_platform_data {
	char *name;	/* LED name as expected by LED class */
	char *supply;
};

#endif /* __LINUX_LEDS_REGULATOR_H */

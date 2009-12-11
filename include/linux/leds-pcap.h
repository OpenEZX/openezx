/*
 * leds-pcap.h - platform data structure for leds-pcap driver
 *
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
 * Copyright (C) 2009 Antonio Ospite <ospite@studenti.unina.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_LEDS_PCAP_H
#define __LINUX_LEDS_PCAP_H

#include <linux/leds.h>
#include <linux/workqueue.h>

struct pcap_led {
	u8 type;
	char *name;
	u8 curr;
	u8 timing;
	int gpio;
	bool gpio_invert;
	int brightness;
};

struct pcap_leds_platform_data {
	int num_leds;
	struct pcap_led leds[];
};

#endif /* __LINUX_LEDS_PCAP_H */

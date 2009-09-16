/*
 * EOC VBUS sensing driver for B peripheral only devices
 * with EOC transceivers.
 * Optionally D+ pullup can be controlled by a second GPIO.
 *
 * Copyright (c) 2009 guiming <gmzhuo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/mfd/ezx-eoc.h>
struct eoc_vbus_mach_info {
	void (*mach_switch_mode)(enum eoc_transceiver_mode);
};

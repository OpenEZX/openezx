/*
 *  motoq.c - Board code for Motorola Q CDMA (Franklin)
 *
 *  Copyright (c) 2009 Timothy Meade <zt.tmzt@gmail.com>
 *  		  2009 Daniel Ribeiro <drwyrm@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/pxa27x.h>
#include <mach/pxafb.h>
#include <plat/i2c.h>
#include <mach/hardware.h>
#include <mach/pxa27x_keypad.h>

#include "devices.h"
#include "generic.h"

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 1023,
	.dft_brightness	= 1023,
	.pwm_period_ns	= 78770,
};

static struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent	= &pxa27x_device_pwm0.dev,
		.platform_data = &backlight_data,
	},
};

static struct pxafb_mode_info motoq_lcd_mode = {
	.pixclock		= 192307,
	.xres			= 240,
	.yres			= 320,
	.bpp			= 16,
	.hsync_len		= 17,
	.vsync_len		= 2,
	.left_margin		= 20,
	.right_margin		= 10,
	.upper_margin		= 1,
	.lower_margin		= 1,
};

static struct pxafb_mach_info motoq_lcd_info = {
	.modes			= &motoq_lcd_mode,
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP,
};

static unsigned int motoq_matrix_key_map[] = {
	KEY(0, 0, KEY_O),
	KEY(0, 1, KEY_Y),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_D),
	KEY(0, 4, KEY_W),
	KEY(0, 5, KEY_G),
	KEY(0, 6, KEY_J),
	KEY(0, 7, KEY_L),

	KEY(1, 0, KEY_U),
	KEY(1, 1, KEY_V),
	KEY(1, 2, KEY_DOT),
	KEY(1, 3, KEY_0),
	KEY(1, 4, KEY_LEFTALT),		/* speakerphone correct */
	KEY(1, 5, KEY_LEFTCTRL),	/* mail !!!!! */
	KEY(1, 6, KEY_8),
	KEY(1, 7, KEY_ENTER),		/* ok */

	KEY(2, 0, KEY_F),
	KEY(2, 1, KEY_O),
	KEY(2, 2, KEY_O),
	KEY(2, 3, KEY_Z),
	KEY(2, 4, KEY_ENTER),		/* wheel click correct */
	KEY(2, 5, KEY_BACKSPACE),	/* backspace (left arrow softkey) */
	KEY(2, 6, KEY_RIGHTSHIFT),	/* shift ! */
	KEY(2, 7, KEY_R),

	KEY(3, 0, KEY_T),
	KEY(3, 1, KEY_E),
	KEY(3, 2, KEY_RIGHTALT),	/* fn !?? */
	KEY(3, 3, KEY_I),
	KEY(3, 4, KEY_ESC),		/* wheel back correct */
	KEY(3, 5, KEY_B),
	KEY(3, 6, KEY_TAB),		/* right softkey correct */
	KEY(3, 7, KEY_4),

	KEY(4, 0, KEY_H),
	KEY(4, 1, KEY_LEFTCTRL),	/* mail */
	KEY(4, 2, KEY_X),
	KEY(4, 3, KEY_V),
	KEY(4, 4, KEY_LEFTCTRL),	/* left softkey correct */
	KEY(4, 5, KEY_P),
	KEY(4, 6, KEY_M),
	KEY(4, 7, KEY_UP),		/* up ? */

	KEY(5, 0, KEY_K),
	KEY(5, 1, KEY_ENTER),		/* ok (center button) */
	KEY(5, 2, KEY_N),
	KEY(5, 3, KEY_DOWN),		/* not B? */
	KEY(5, 4, KEY_LEFT),		/* left (widescreen) correct */
	KEY(5, 5, KEY_C),
	KEY(5, 6, KEY_UP),		/* up (widescreen) correct */
	KEY(5, 7, KEY_Q),

	KEY(6, 0, KEY_S),
	KEY(6, 1, KEY_SPACE),
	KEY(6, 2, KEY_ENTER),		/* return key */
	KEY(6, 3, KEY_N),
	KEY(6, 4, KEY_RIGHT),		/* right (widescreen) correct */
	KEY(6, 5, KEY_MINUS),		/* call? (green phone) !!!!! */
	KEY(6, 6, KEY_RIGHTALT),	/* cmaera correct */
	KEY(6, 7, KEY_SLASH),		/* home (house) correct */

	KEY(7, 0, KEY_LEFTBRACE),	/* alt/fn ? */
	KEY(7, 1, KEY_Y),
	KEY(7, 2, KEY_BACKSLASH),	/* backspace ? */
	KEY(7, 3, KEY_SEMICOLON),	/* backspace !? */
	KEY(7, 4, KEY_K),
	KEY(7, 5, KEY_C),
	KEY(7, 6, KEY_RIGHTALT),	/* fn correct */
	KEY(7, 7, KEY_9),
};

static struct pxa27x_keypad_platform_data motoq_keypad_info = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= motoq_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(motoq_matrix_key_map),
	.debounce_interval	= 30, /* from littleton.c */
};

static unsigned long pin_config[] = {
	GPIO18_GPIO,
	GPIO29_SSP1_SCLK,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,
	GPIO28_GPIO,

	/* BTUART */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* OHCI Connected to MSM */
	GPIO30_USB_P3_2,	/* ICL_TXENB */
	GPIO31_USB_P3_6,	/* ICL_VPOUT */
	GPIO90_USB_P3_5,	/* ICL_VPIN */
	GPIO91_USB_P3_1,	/* ICL_XRXD */
	GPIO56_USB_P3_4,	/* ICL_VMOUT */
	GPIO113_USB_P3_3,	/* ICL_VMIN */

	/* camera */
	GPIO23_CIF_MCLK,
	GPIO54_CIF_PCLK,
	GPIO85_CIF_LV,
	GPIO84_CIF_FV,
	GPIO27_CIF_DD_0,
	GPIO114_CIF_DD_1,
	GPIO51_CIF_DD_2,
	GPIO115_CIF_DD_3,
	GPIO95_CIF_DD_4,
	GPIO94_CIF_DD_5,
	GPIO17_CIF_DD_6,
	GPIO108_CIF_DD_7,
};

static struct platform_device *devices[] __initdata = {
	&backlight_device,
};

static void __init motoq_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(pin_config));
	pxa_set_i2c_info(NULL);
	set_pxa_fb_info(&motoq_lcd_info);
	pxa_set_keypad_info(&motoq_keypad_info);

	platform_add_devices(devices, ARRAY_SIZE(devices));
};

MACHINE_START(MOTOQ, "Motorola Q CDMA (Franklin)")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa27x_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= motoq_init,
MACHINE_END

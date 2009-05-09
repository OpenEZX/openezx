/*
 *  ezx.c - Common code for the EZX platform.
 *
 *  Copyright (C) 2005-2006 Harald Welte <laforge@openezx.org>,
 *		  2007-2008 Daniel Ribeiro <drwyrm@gmail.com>,
 *		  2007-2008 Stefan Schmidt <stefan@datenfreihafen.org>
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
#include <linux/gpio_keys.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/spi/mmc_spi.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/leds-pcap.h>
#include <linux/leds-lp3944.h>
#include <linux/regulator/machine.h>

#include <media/soc_camera.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>

#include <mach/pxa27x.h>
#include <mach/pxafb.h>
#include <mach/ohci.h>
#include <mach/i2c.h>
#include <mach/hardware.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>
#include <mach/mmc.h>
#include <mach/udc.h>
#include <mach/pxa27x-udc.h>
#include <mach/camera.h>

#include "devices.h"
#include "generic.h"

#define GPIO12_A780_FLIP_LID 		12
#define GPIO15_A1200_FLIP_LID 		15
#define GPIO15_A910_FLIP_LID 		15
#define GPIO12_E680_LOCK_SWITCH 	12
#define GPIO15_E6_LOCK_SWITCH 		15
#define GPIO1_PCAP_IRQ			1
#define GPIO11_MMC_DETECT		11
#define GPIO20_A910_MMC_CS		20
#define GPIO24_PCAP_CS			24
#define GPIO46_E680_LED_RED		46
#define GPIO47_E680_LED_GREEN		47

static struct platform_pwm_backlight_data ezx_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 1023,
	.dft_brightness	= 1023,
	.pwm_period_ns	= 78770,
};

static struct platform_device ezx_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent	= &pxa27x_device_pwm0.dev,
		.platform_data = &ezx_backlight_data,
	},
};

static struct pxafb_mode_info mode_ezx_old = {
	.pixclock		= 150000,
	.xres			= 240,
	.yres			= 320,
	.bpp			= 16,
	.hsync_len		= 10,
	.left_margin		= 20,
	.right_margin		= 10,
	.vsync_len		= 2,
	.upper_margin		= 3,
	.lower_margin		= 2,
	.sync			= 0,
};

static struct pxafb_mach_info ezx_fb_info_1 = {
	.modes		= &mode_ezx_old,
	.num_modes	= 1,
	.lcd_conn	= LCD_COLOR_TFT_16BPP,
};

static struct pxafb_mode_info mode_72r89803y01 = {
	.pixclock		= 192308,
	.xres			= 240,
	.yres			= 320,
	.bpp			= 32,
	.depth			= 18,
	.hsync_len		= 10,
	.left_margin		= 20,
	.right_margin		= 10,
	.vsync_len		= 2,
	.upper_margin		= 3,
	.lower_margin		= 2,
	.sync			= 0,
};

static struct pxafb_mach_info ezx_fb_info_2 = {
	.modes		= &mode_72r89803y01,
	.num_modes	= 1,
	.lcd_conn	= LCD_COLOR_TFT_18BPP,
};

/* MMC */
static int ezx_mci_init(struct device *dev,
		irqreturn_t (*detect_int)(int, void *), void *data)
{
	int err = 0;

	/* A1200 slot is not hot-plug */
	if (!machine_is_ezx_a1200()) {
		err = request_irq(gpio_to_irq(GPIO11_MMC_DETECT), detect_int,
			IRQF_DISABLED | IRQF_SAMPLE_RANDOM |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"MMC card detect", data);
	}

	return err;
}

static void ezx_mci_exit(struct device *dev, void *data)
{
	if (!machine_is_ezx_a1200())
		free_irq(gpio_to_irq(GPIO11_MMC_DETECT), data);
}

static struct pxamci_platform_data ezx_mci_platform_data = {
	.init           = ezx_mci_init,
	.exit           = ezx_mci_exit,
	.detect_delay   = 250 / (1000 / HZ),
};

static struct platform_device *devices[] __initdata = {
	&ezx_backlight_device,
};

static unsigned long ezx_pin_config[] __initdata = {
	/* PWM backlight */
	GPIO16_PWM0_OUT,

	/* BTUART */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* I2C */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,

	/* PCAP SSP */
	GPIO29_SSP1_SCLK,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,
	GPIO24_GPIO,				/* pcap chip select */
	GPIO1_GPIO | WAKEUP_ON_EDGE_RISE,	/* pcap interrupt */
	GPIO4_GPIO | MFP_LPM_DRIVE_HIGH,	/* WDI_AP */
	GPIO55_GPIO | MFP_LPM_DRIVE_HIGH,	/* SYS_RESTART */

	/* MMC */
	GPIO32_MMC_CLK,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,
	GPIO11_GPIO,				/* mmc detect */

	/* usb to external transceiver */
	GPIO34_USB_P2_2,
	GPIO35_USB_P2_1,
	GPIO36_USB_P2_4,
	GPIO39_USB_P2_6,
	GPIO40_USB_P2_5,
	GPIO53_USB_P2_3,

	/* usb to Neptune GSM chip */
	GPIO30_USB_P3_2,
	GPIO31_USB_P3_6,
	GPIO90_USB_P3_5,
	GPIO91_USB_P3_1,
	GPIO56_USB_P3_4,
	GPIO113_USB_P3_3,
};

#if defined(CONFIG_MACH_EZX_A780) || defined(CONFIG_MACH_EZX_E680)
static unsigned long gen1_pin_config[] __initdata = {
	/* flip / lockswitch */
	GPIO12_GPIO | WAKEUP_ON_EDGE_BOTH,

	/* bluetooth (bcm2035) */
	GPIO14_GPIO | WAKEUP_ON_EDGE_RISE,	/* HOSTWAKE */
	GPIO48_GPIO,				/* RESET */
	GPIO28_GPIO,				/* WAKEUP */

	/* Neptune handshake */
	GPIO0_GPIO | WAKEUP_ON_EDGE_FALL,	/* BP_RDY */
	GPIO57_GPIO | MFP_LPM_DRIVE_HIGH,	/* AP_RDY */
	GPIO13_GPIO | WAKEUP_ON_EDGE_BOTH,	/* WDI */
	GPIO3_GPIO | WAKEUP_ON_EDGE_BOTH,	/* WDI2 */
	GPIO82_GPIO | MFP_LPM_DRIVE_HIGH,	/* RESET */
	GPIO99_GPIO | MFP_LPM_DRIVE_HIGH,	/* TC_MM_EN */

	/* sound */
	GPIO52_SSP3_SCLK,
	GPIO83_SSP3_SFRM,
	GPIO81_SSP3_TXD,
	GPIO89_SSP3_RXD,

	/* ssp2 pins to in */
	GPIO22_GPIO,				/* SSP2_SCLK */
	GPIO37_GPIO,				/* SSP2_SFRM */
	GPIO38_GPIO,				/* SSP2_TXD */
	GPIO88_GPIO,				/* SSP2_RXD */

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
	GPIO50_GPIO,				/* CAM_EN */
	GPIO19_GPIO,				/* CAM_RST */

	/* EMU */
	GPIO120_GPIO,				/* EMU_MUX1 */
	GPIO119_GPIO,				/* EMU_MUX2 */
	GPIO86_GPIO,				/* SNP_INT_CTL */
	GPIO87_GPIO,				/* SNP_INT_IN */
};
#endif

#if defined(CONFIG_MACH_EZX_A1200) || defined(CONFIG_MACH_EZX_A910) || \
	defined(CONFIG_MACH_EZX_E2) || defined(CONFIG_MACH_EZX_E6)
static unsigned long gen2_pin_config[] __initdata = {
	/* flip / lockswitch */
	GPIO15_GPIO | WAKEUP_ON_EDGE_BOTH,

	/* EOC */
	GPIO10_GPIO | WAKEUP_ON_EDGE_RISE,

	/* bluetooth (bcm2045) */
	GPIO13_GPIO | WAKEUP_ON_EDGE_RISE,	/* HOSTWAKE */
	GPIO37_GPIO,				/* RESET */
	GPIO57_GPIO,				/* WAKEUP */

	/* Neptune handshake */
	GPIO0_GPIO | WAKEUP_ON_EDGE_FALL,	/* BP_RDY */
	GPIO96_GPIO | MFP_LPM_DRIVE_HIGH,	/* AP_RDY */
	GPIO3_GPIO | WAKEUP_ON_EDGE_FALL,	/* WDI */
	GPIO116_GPIO | MFP_LPM_DRIVE_HIGH,	/* RESET */
	GPIO41_GPIO,				/* BP_FLASH */

	/* sound */
	GPIO52_SSP3_SCLK,
	GPIO83_SSP3_SFRM,
	GPIO81_SSP3_TXD,
	GPIO82_SSP3_RXD,

	/* ssp2 pins to in */
	GPIO22_GPIO,				/* SSP2_SCLK */
	GPIO14_GPIO,				/* SSP2_SFRM */
	GPIO38_GPIO,				/* SSP2_TXD */
	GPIO88_GPIO,				/* SSP2_RXD */

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
	GPIO48_CIF_DD_5,
	GPIO93_CIF_DD_6,
	GPIO12_CIF_DD_7,
	GPIO50_GPIO,				/* CAM_EN */
	GPIO28_GPIO,				/* CAM_RST */
	GPIO17_GPIO,				/* CAM_FLASH */
};
#endif

#ifdef CONFIG_MACH_EZX_A780
static unsigned long a780_pin_config[] __initdata = {
	/* keypad */
	GPIO93_KP_DKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_MKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_MKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,
	GPIO107_KP_MKOUT_4,

	/* attenuate sound */
	GPIO96_GPIO,
};
#endif

#ifdef CONFIG_MACH_EZX_E680
static unsigned long e680_pin_config[] __initdata = {
	/* keypad */
	GPIO93_KP_DKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO96_KP_DKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_DKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_DKIN_5 | WAKEUP_ON_LEVEL_HIGH,
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,

	/* MIDI */
	GPIO79_GPIO,				/* VA_SEL_BUL */
	GPIO80_GPIO,				/* FLT_SEL_BUL */
	GPIO78_GPIO,				/* MIDI_RESET */
	GPIO33_GPIO,				/* MIDI_CS */
	GPIO15_GPIO,				/* MIDI_IRQ */
	GPIO49_GPIO,				/* MIDI_NPWE */
	GPIO18_GPIO,				/* MIDI_RDY */

	/* leds */
	GPIO46_GPIO,
	GPIO47_GPIO,
};
#endif

#ifdef CONFIG_MACH_EZX_A1200
static unsigned long a1200_pin_config[] __initdata = {
	/* keypad */
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_MKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_MKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,
	GPIO107_KP_MKOUT_4,
	GPIO108_KP_MKOUT_5,
};
#endif

#ifdef CONFIG_MACH_EZX_A910
static unsigned long a910_pin_config[] __initdata = {
	/* keypad */
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_MKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_MKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,
	GPIO107_KP_MKOUT_4,
	GPIO108_KP_MKOUT_5,

	/* WLAN */
	GPIO89_GPIO,				/* RESET */
	GPIO33_GPIO,				/* WAKEUP */
	GPIO94_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* HOSTWAKE */

	/* MMC CS */
	GPIO20_GPIO,
};
#endif

#ifdef CONFIG_MACH_EZX_E2
static unsigned long e2_pin_config[] __initdata = {
	/* keypad */
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_MKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_MKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,
	GPIO107_KP_MKOUT_4,
	GPIO108_KP_MKOUT_5,
};
#endif

#ifdef CONFIG_MACH_EZX_E6
static unsigned long e6_pin_config[] __initdata = {
	/* keypad */
	GPIO100_KP_MKIN_0 | WAKEUP_ON_LEVEL_HIGH,
	GPIO101_KP_MKIN_1 | WAKEUP_ON_LEVEL_HIGH,
	GPIO102_KP_MKIN_2 | WAKEUP_ON_LEVEL_HIGH,
	GPIO97_KP_MKIN_3 | WAKEUP_ON_LEVEL_HIGH,
	GPIO98_KP_MKIN_4 | WAKEUP_ON_LEVEL_HIGH,
	GPIO103_KP_MKOUT_0,
	GPIO104_KP_MKOUT_1,
	GPIO105_KP_MKOUT_2,
	GPIO106_KP_MKOUT_3,
	GPIO107_KP_MKOUT_4,
	GPIO108_KP_MKOUT_5,
};
#endif

/* KEYPAD */
#ifdef CONFIG_MACH_EZX_A780
static unsigned int a780_key_map[] = {
	KEY(0, 0, KEY_SEND),
	KEY(0, 1, KEY_BACK),
	KEY(0, 2, KEY_END),
	KEY(0, 3, KEY_PAGEUP),
	KEY(0, 4, KEY_UP),

	KEY(1, 0, KEY_NUMERIC_1),
	KEY(1, 1, KEY_NUMERIC_2),
	KEY(1, 2, KEY_NUMERIC_3),
	KEY(1, 3, KEY_SELECT),
	KEY(1, 4, KEY_KPENTER),

	KEY(2, 0, KEY_NUMERIC_4),
	KEY(2, 1, KEY_NUMERIC_5),
	KEY(2, 2, KEY_NUMERIC_6),
	KEY(2, 3, KEY_RECORD),
	KEY(2, 4, KEY_LEFT),

	KEY(3, 0, KEY_NUMERIC_7),
	KEY(3, 1, KEY_NUMERIC_8),
	KEY(3, 2, KEY_NUMERIC_9),
	KEY(3, 3, KEY_HOME),
	KEY(3, 4, KEY_RIGHT),

	KEY(4, 0, KEY_NUMERIC_STAR),
	KEY(4, 1, KEY_NUMERIC_0),
	KEY(4, 2, KEY_NUMERIC_POUND),
	KEY(4, 3, KEY_PAGEDOWN),
	KEY(4, 4, KEY_DOWN),
};

static struct pxa27x_keypad_platform_data a780_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 5,
	.matrix_key_map = a780_key_map,
	.matrix_key_map_size = ARRAY_SIZE(a780_key_map),

	.direct_key_map = { KEY_CAMERA },
	.direct_key_num = 1,

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_A780 */

#ifdef CONFIG_MACH_EZX_E680
static unsigned int e680_key_map[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_RESERVED),
	KEY(0, 3, KEY_SEND),

	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_LEFT),
	KEY(1, 2, KEY_PAGEUP),
	KEY(1, 3, KEY_PAGEDOWN),

	KEY(2, 0, KEY_RESERVED),
	KEY(2, 1, KEY_RESERVED),
	KEY(2, 2, KEY_RESERVED),
	KEY(2, 3, KEY_KPENTER),
};

static struct pxa27x_keypad_platform_data e680_keypad_platform_data = {
	.matrix_key_rows = 3,
	.matrix_key_cols = 4,
	.matrix_key_map = e680_key_map,
	.matrix_key_map_size = ARRAY_SIZE(e680_key_map),

	.direct_key_map = {
		KEY_CAMERA,
		KEY_RESERVED,
		KEY_RESERVED,
		KEY_F1,
		KEY_CANCEL,
		KEY_F2,
	},
	.direct_key_num = 6,

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_E680 */

#ifdef CONFIG_MACH_EZX_A1200
static unsigned int a1200_key_map[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_PAGEDOWN),
	KEY(0, 3, KEY_RESERVED),
	KEY(0, 4, KEY_RESERVED),
	KEY(0, 5, KEY_RESERVED),

	KEY(1, 0, KEY_RESERVED),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_CAMERA),
	KEY(1, 3, KEY_RESERVED),
	KEY(1, 4, KEY_RESERVED),
	KEY(1, 5, KEY_RESERVED),

	KEY(2, 0, KEY_RESERVED),
	KEY(2, 1, KEY_KPENTER),
	KEY(2, 2, KEY_RECORD),
	KEY(2, 3, KEY_RESERVED),
	KEY(2, 4, KEY_RESERVED),
	KEY(2, 5, KEY_SELECT),

	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_SEND),
	KEY(3, 3, KEY_RESERVED),
	KEY(3, 4, KEY_RESERVED),
	KEY(3, 5, KEY_RESERVED),

	KEY(4, 0, KEY_RESERVED),
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_PAGEUP),
	KEY(4, 3, KEY_RESERVED),
	KEY(4, 4, KEY_RESERVED),
	KEY(4, 5, KEY_RESERVED),
};

static struct pxa27x_keypad_platform_data a1200_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 6,
	.matrix_key_map = a1200_key_map,
	.matrix_key_map_size = ARRAY_SIZE(a1200_key_map),

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_A1200 */

#ifdef CONFIG_MACH_EZX_E6
static unsigned int e6_key_map[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_PAGEDOWN),
	KEY(0, 3, KEY_RESERVED),
	KEY(0, 4, KEY_RESERVED),
	KEY(0, 5, KEY_NEXTSONG),

	KEY(1, 0, KEY_RESERVED),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_PROG1),
	KEY(1, 3, KEY_RESERVED),
	KEY(1, 4, KEY_RESERVED),
	KEY(1, 5, KEY_RESERVED),

	KEY(2, 0, KEY_RESERVED),
	KEY(2, 1, KEY_ENTER),
	KEY(2, 2, KEY_CAMERA),
	KEY(2, 3, KEY_RESERVED),
	KEY(2, 4, KEY_RESERVED),
	KEY(2, 5, KEY_WWW),

	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_SEND),
	KEY(3, 3, KEY_RESERVED),
	KEY(3, 4, KEY_RESERVED),
	KEY(3, 5, KEY_PLAYPAUSE),

	KEY(4, 0, KEY_RESERVED),
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_PAGEUP),
	KEY(4, 3, KEY_RESERVED),
	KEY(4, 4, KEY_RESERVED),
	KEY(4, 5, KEY_PREVIOUSSONG),
};

static struct pxa27x_keypad_platform_data e6_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 6,
	.matrix_key_map = e6_key_map,
	.matrix_key_map_size = ARRAY_SIZE(e6_key_map),

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_E6 */

#ifdef CONFIG_MACH_EZX_A910
static unsigned int a910_key_map[] = {
	KEY(0, 0, KEY_NUMERIC_6),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_PAGEDOWN),
	KEY(0, 3, KEY_KPENTER),
	KEY(0, 4, KEY_NUMERIC_5),
	KEY(0, 5, KEY_CAMERA),

	KEY(1, 0, KEY_NUMERIC_8),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_RESERVED),
	KEY(1, 3, KEY_F1), /* Left SoftKey */
	KEY(1, 4, KEY_NUMERIC_STAR),
	KEY(1, 5, KEY_RESERVED),

	KEY(2, 0, KEY_NUMERIC_7),
	KEY(2, 1, KEY_NUMERIC_9),
	KEY(2, 2, KEY_RECORD),
	KEY(2, 3, KEY_F2), /* Right SoftKey */
	KEY(2, 4, KEY_BACK),
	KEY(2, 5, KEY_SELECT),

	KEY(3, 0, KEY_NUMERIC_2),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_SEND),
	KEY(3, 3, KEY_NUMERIC_0),
	KEY(3, 4, KEY_NUMERIC_1),
	KEY(3, 5, KEY_RECORD),

	KEY(4, 0, KEY_NUMERIC_4),
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_PAGEUP),
	KEY(4, 3, KEY_NUMERIC_POUND),
	KEY(4, 4, KEY_NUMERIC_3),
	KEY(4, 5, KEY_RESERVED),
};

static struct pxa27x_keypad_platform_data a910_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 6,
	.matrix_key_map = a910_key_map,
	.matrix_key_map_size = ARRAY_SIZE(a910_key_map),

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_A910 */

#ifdef CONFIG_MACH_EZX_E2
static unsigned int e2_key_map[] = {
	KEY(0, 0, KEY_NUMERIC_6),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_NUMERIC_9),
	KEY(0, 3, KEY_NEXTSONG),
	KEY(0, 4, KEY_NUMERIC_5),
	KEY(0, 5, KEY_F1), /* Left SoftKey */

	KEY(1, 0, KEY_NUMERIC_8),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_RESERVED),
	KEY(1, 3, KEY_PAGEUP),
	KEY(1, 4, KEY_NUMERIC_STAR),
	KEY(1, 5, KEY_F2), /* Right SoftKey */

	KEY(2, 0, KEY_NUMERIC_7),
	KEY(2, 1, KEY_KPENTER),
	KEY(2, 2, KEY_RECORD),
	KEY(2, 3, KEY_PAGEDOWN),
	KEY(2, 4, KEY_BACK),
	KEY(2, 5, KEY_NUMERIC_0),

	KEY(3, 0, KEY_NUMERIC_2),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_SEND),
	KEY(3, 3, KEY_PLAYPAUSE),
	KEY(3, 4, KEY_NUMERIC_1),
	KEY(3, 5, KEY_SOUND), /* Music SoftKey */

	KEY(4, 0, KEY_NUMERIC_4),
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_NUMERIC_POUND),
	KEY(4, 3, KEY_PREVIOUSSONG),
	KEY(4, 4, KEY_NUMERIC_3),
	KEY(4, 5, KEY_RESERVED),
};

static struct pxa27x_keypad_platform_data e2_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 6,
	.matrix_key_map = e2_key_map,
	.matrix_key_map_size = ARRAY_SIZE(e2_key_map),

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_E2 */

/* PCAP */
static void ezx_pcap_init(void)
{
	/* set SW1 sleep to keep SW1 1.3v in sync mode */
	/* SW1 active in sync mode */
/*	ezx_pcap_set_sw(SW1, SW_MODE, 0x1); */

	/* set core voltage */
/*	ezx_pcap_set_sw(SW1, SW_VOLTAGE, SW_VOLTAGE_1250); */

	/* FIXME: EMU driver */
	ezx_pcap_write(PCAP_REG_BUSCTRL,
			(PCAP_BUSCTRL_RS232ENB | PCAP_BUSCTRL_VUSB_EN));
	gpio_request(120, "EMU mux1");
	gpio_request(119, "EMU mux2");
	gpio_direction_output(120, 0);
	gpio_direction_output(119, 0);
}

static struct pcap_platform_data ezx_pcap_platform_data = {
	.irq    = gpio_to_irq(GPIO1_PCAP_IRQ),
	.config = 0,
	.init   = ezx_pcap_init,
};

static struct pxa2xx_spi_chip ezx_pcap_chip_info = {
	.tx_threshold   = 1,
	.rx_threshold   = 1,
	.dma_burst_size = 0,
	.timeout        = 100,
	.gpio_cs	= GPIO24_PCAP_CS,
};

static struct pxa2xx_spi_master ezx_spi_masterinfo = {
	.clock_enable   = CKEN_SSP1,
	.num_chipselect = 1,
	.enable_dma     = 1,
};

static struct spi_board_info ezx_spi_boardinfo[] __initdata = {
	{
		.modalias        = "ezx-pcap",
		.bus_num         = 1,
		.chip_select     = 0,
		.max_speed_hz    = 13000000,
		.platform_data   = &ezx_pcap_platform_data,
		.controller_data = &ezx_pcap_chip_info,
		.mode            = SPI_MODE_0,
	},
};

/* voltage regulators */
/* VAUX2: MMC on E680, E6, E2 */
static struct regulator_consumer_supply pcap_regulator_VAUX2_consumers[] = {
	{ .dev = &pxa_device_mci.dev, .supply = "vmmc", },
};

static struct regulator_init_data pcap_regulator_VAUX2_data = {
	.constraints = {
		.min_uV = 1875000,
		.max_uV = 3000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | 
					REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(pcap_regulator_VAUX2_consumers),
	.consumer_supplies = pcap_regulator_VAUX2_consumers,
};

static struct platform_device pcap_regulator_VAUX2_device = {
	.name = "pcap-regulator", .id = VAUX2,
	.platform_data = &pcap_regulator_VAUX2_data,
};

/* VAUX3: MMC on A780, A1200, A910 */
static struct regulator_consumer_supply pcap_regulator_VAUX3_consumers[] = {
	{ .dev = &pxa_device_mci.dev, .supply = "vmmc", },
};

static struct regulator_init_data pcap_regulator_VAUX3_data = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 3600000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | 
					REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(pcap_regulator_VAUX3_consumers),
	.consumer_supplies = pcap_regulator_VAUX3_consumers,
};

static struct platform_device pcap_regulator_VAUX3_device = {
	.name = "pcap-regulator", .id = VAUX3,
	.platform_data = &pcap_regulator_VAUX3_data,
};

/* SW1: CORE on A1200, A910, E6, E2 */
static struct regulator_consumer_supply pcap_regulator_SW1_consumers[] = {
	{ .supply = "vcc_core", },
};

static struct regulator_init_data pcap_regulator_SW1_data = {
	.constraints = {
		.min_uV = 950000,
		.max_uV = 1705000,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(pcap_regulator_SW1_consumers),
	.consumer_supplies = pcap_regulator_SW1_consumers,
};

static struct platform_device pcap_regulator_SW1_device = {
	.name = "pcap-regulator", .id = SW1,
	.platform_data = &pcap_regulator_SW1_data,
};

/* PCAP_TS */
struct platform_device pcap_ts_device = {
	.name = "pcap-ts",
	.id   = -1,
};

/* PCAP_RTC */
static struct platform_device pcap_rtc_device = {
	.name = "rtc-pcap",
	.id   = -1,
};

/* UDC */
static void ezx_udc_command(int cmd)
{
	unsigned int tmp;
	ezx_pcap_read(PCAP_REG_BUSCTRL, &tmp);
	switch (cmd) {
	case PXA2XX_UDC_CMD_DISCONNECT:
		if (machine_is_ezx_a780() || machine_is_ezx_e680())
			tmp &= ~PCAP_BUSCTRL_USB_PU;
		break;
	case PXA2XX_UDC_CMD_CONNECT:
		/* temp. set UP2OCR here until we have a transceiver driver */
		UP2OCR = UP2OCR_SEOS(2);
		if (machine_is_ezx_a780() || machine_is_ezx_e680())
			tmp |= PCAP_BUSCTRL_USB_PU;
		break;
	}
	ezx_pcap_write(PCAP_REG_BUSCTRL, tmp);
}

static struct pxa2xx_udc_mach_info ezx_udc_info = {
	.udc_command	= ezx_udc_command,
	.gpio_pullup	= -1,
	.gpio_vbus	= -1,
};

#ifdef CONFIG_MACH_EZX_A780
/* gpio_keys */
static struct gpio_keys_button a780_buttons[] = {
	[0] = {
		.code = SW_LID,
		.gpio = GPIO12_A780_FLIP_LID,
		.active_low = 0,
		.desc = "A780 flip lid",
		.type = EV_SW,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data a780_gpio_keys_platform_data = {
	.buttons  = a780_buttons,
	.nbuttons = ARRAY_SIZE(a780_buttons),
};

static struct platform_device a780_gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &a780_gpio_keys_platform_data,
	},
};

/* mtd partitions on NOR flash */
static struct resource flash_resource = {
	.start = PXA_CS0_PHYS,
	.end   = PXA_CS0_PHYS + SZ_32M - 1,
	.flags = IORESOURCE_MEM,
};

static struct mtd_partition a780_partitions[] = {
	{
		.name       = "Bootloader",
		.size       = 0x00020000,
		.offset     = 0x0,
		.mask_flags = MTD_WRITEABLE, /* force read-only */
	}, {
		.name       = "Kernel",
		.size       = 0x000e0000,
		.offset     = 0x00020000,
	} , {
		.name       = "rootfs",
		.size       = 0x018e0000,
		.offset     = 0x00120000,
	} , {
		.name       = "VFM_Filesystem",
		.size       = 0x00580000,
		.offset     = 0x01a00000,
	} , {
		.name       = "setup",
		.size       = 0x00020000,
		.offset     = 0x01fa0000,
	} , {
		.name       = "Logo",
		.size       = 0x00020000,
		.offset     = 0x01fc0000,
	},
};

static struct flash_platform_data a780_flash_data = {
	.map_name = "cfi_probe",
	.name     = "A780 NOR flash",
	.parts    = a780_partitions,
	.nr_parts = ARRAY_SIZE(a780_partitions),
};

static struct platform_device a780_flash_device = {
	.name          = "pxa2xx-flash",
	.id            = 0,
	.dev           = {
		.platform_data = &a780_flash_data,
	},
	.resource      = &flash_resource,
	.num_resources = 1,
};

/* camera */
static int a780_pxacamera_init(struct device *dev)
{
	/* 
	 * GPIO50_GPIO is CAM_EN: active low
	 * GPIO19_GPIO is CAM_RST: active high
	 */
	gpio_request(MFP_PIN_GPIO50, "nCAM_EN");
	gpio_request(MFP_PIN_GPIO19, "CAM_RST");
	gpio_direction_output(MFP_PIN_GPIO50, 0);
	gpio_direction_output(MFP_PIN_GPIO19, 1);

	return 0;
}

static int a780_pxacamera_power(struct device *dev, int on)
{
	gpio_set_value(MFP_PIN_GPIO50, on ? 0 : 1);

#if 0
	/* 
	 * This is reported to resolve the vertical line in view finder issue
	 * (LIBff11930), is this still needed?
	 *
	 * AP Kernel camera driver: set TC_MM_EN to low when camera is running
	 * and TC_MM_EN to high when camera stops.
	 *
	 * BP Software: if TC_MM_EN is low, BP do not shut off 26M clock, but
	 * BP can sleep itself.
	 */
	gpio_set_value(MFP_PIN_GPIO99, on ? 0 : 1);
#endif

	return 0;
}

static int a780_pxacamera_reset(struct device *dev)
{
	gpio_set_value(MFP_PIN_GPIO19, 0);
	msleep(10);
	gpio_set_value(MFP_PIN_GPIO19, 1);

	return 0;
}

struct pxacamera_platform_data a780_pxacamera_platform_data = {
	.init	= a780_pxacamera_init,
	.flags  = PXA_CAMERA_MASTER | PXA_CAMERA_DATAWIDTH_8 |
		PXA_CAMERA_PCLK_EN | PXA_CAMERA_MCLK_EN,
	.mclk_10khz = 5000,
};

static struct soc_camera_link a780_iclink = {
	.bus_id	= 0,
	.power = a780_pxacamera_power,
	.reset = a780_pxacamera_reset,
	.flags = SOCAM_SENSOR_INVERT_PCLK,
};

static struct i2c_board_info __initdata a780_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("mt9m111", 0x5d),
		.platform_data = &a780_iclink,
	},
};


/* pcap-leds */
static struct pcap_leds_platform_data a780_leds = {
	.leds = {
		{
			.type = PCAP_BL0,
			.name = "a780:main",
		}, {
			.type = PCAP_BL1,
			.name = "a780:aux",
		}, {
			.type = PCAP_VIB,
			.name = "a780:vibrator",
		},
	},
	.num_leds = 3,
};

struct platform_device a780_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &a780_leds,
	},
};


static void __init a780_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen1_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a780_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(a780_i2c_board_info));

	ezx_pcap_platform_data.config = PCAP_SECOND_PORT;
	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &ezx_spi_masterinfo;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));
	platform_device_register(&pcap_regulator_VAUX3_device);

	pxa_set_mci_parent(&spi_pd->dev);
	pxa_set_mci_info(&ezx_mci_platform_data);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_1);

	pxa_set_keypad_info(&a780_keypad_platform_data);

	platform_device_register(&a780_gpio_keys);
	platform_device_register(&pcap_ts_device);
	platform_device_register(&a780_leds_device);

	/* FIXME: Could this be simplified to just 2 ? */
	a780_flash_data.width = (BOOT_DEF & 1) ? 2 : 4,
	platform_device_register(&a780_flash_device);

	pxa_set_camera_info(&a780_pxacamera_platform_data);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A780, "Motorola EZX A780")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = a780_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E680
/* gpio_keys */
static struct gpio_keys_button e680_buttons[] = {
	[0] = {
		.code = KEY_SCREENLOCK,
		.gpio = GPIO12_E680_LOCK_SWITCH,
		.active_low = 0,
		.desc = "E680 lock switch",
		.type = EV_KEY,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data e680_gpio_keys_platform_data = {
	.buttons  = e680_buttons,
	.nbuttons = ARRAY_SIZE(e680_buttons),
};

static struct platform_device e680_gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &e680_gpio_keys_platform_data,
	},
};

/* pcap-leds */
static struct pcap_leds_platform_data e680_leds = {
	.leds = {
		{
			.type = PCAP_LED0,
			.name = "e680:red",
			.curr = PCAP_LED_4MA,
			.timing = 0xc,
			.gpio = GPIO46_E680_LED_RED | PCAP_LED_GPIO_EN |
							PCAP_LED_GPIO_INVERT,
		}, {
			.type = PCAP_LED0,
			.name = "e680:green",
			.curr = PCAP_LED_4MA,
			.timing = 0xc,
			.gpio = GPIO47_E680_LED_GREEN | PCAP_LED_GPIO_EN,
		}, {
			.type = PCAP_LED1,
			.name = "e680:blue",
			.curr = PCAP_LED_3MA,
			.timing = 0xc,
			.gpio = 0,
		},
	},
	.num_leds = 3,
};

struct platform_device e680_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &e680_leds,
	},
};

static struct i2c_board_info __initdata e680_i2c_board_info[] = {
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e680_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen1_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e680_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(e680_i2c_board_info));

	ezx_pcap_platform_data.config = PCAP_SECOND_PORT;
	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &ezx_spi_masterinfo;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));
	platform_device_register(&pcap_regulator_VAUX2_device);

	pxa_set_mci_parent(&spi_pd->dev);
	pxa_set_mci_info(&ezx_mci_platform_data);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_1);

	pxa_set_keypad_info(&e680_keypad_platform_data);

	platform_device_register(&e680_gpio_keys);
	platform_device_register(&pcap_ts_device);
	platform_device_register(&e680_leds_device);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E680, "Motorola EZX E680")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = e680_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_A1200
/* gpio_keys */
static struct gpio_keys_button a1200_buttons[] = {
	[0] = {
		.code = SW_LID,
		.gpio = GPIO15_A1200_FLIP_LID,
		.active_low = 0,
		.desc = "A1200 flip lid",
		.type = EV_SW,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data a1200_gpio_keys_platform_data = {
	.buttons  = a1200_buttons,
	.nbuttons = ARRAY_SIZE(a1200_buttons),
};

static struct platform_device a1200_gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &a1200_gpio_keys_platform_data,
	},
};

static struct platform_device pcap_keys_device = {
	.name = "pcap-keys",
	.id   = -1,
};

/* pcap-leds */
static struct pcap_leds_platform_data a1200_leds = {
	.leds = {
		{
			.type = PCAP_VIB,
			.name = "a1200:vibrator",
		},
	},
	.num_leds = 1,
};

struct platform_device a1200_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &a1200_leds,
	},
};

static struct i2c_board_info __initdata a1200_i2c_board_info[] = {
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init a1200_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a1200_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(a1200_i2c_board_info));

	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &ezx_spi_masterinfo;
	ezx_spi_boardinfo[0].mode |= SPI_CS_HIGH;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));
	platform_device_register(&pcap_regulator_SW1_device);
	platform_device_register(&pcap_regulator_VAUX3_device);

	pxa_set_mci_parent(&spi_pd->dev);
	pxa_set_mci_info(&ezx_mci_platform_data);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_2);

	pxa_set_keypad_info(&a1200_keypad_platform_data);

	platform_device_register(&a1200_gpio_keys);
	platform_device_register(&pcap_keys_device);
	platform_device_register(&pcap_ts_device);
	platform_device_register(&a1200_leds_device);
	platform_device_register(&pcap_rtc_device);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A1200, "Motorola EZX A1200")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = a1200_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_A910
/* gpio_keys */
static struct gpio_keys_button a910_buttons[] = {
	[0] = {
		.code = SW_LID,
		.gpio = GPIO15_A910_FLIP_LID,
		.active_low = 0,
		.desc = "A910 flip lid",
		.type = EV_SW,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data a910_gpio_keys_platform_data = {
	.buttons  = a910_buttons,
	.nbuttons = ARRAY_SIZE(a910_buttons),
};

static struct platform_device a910_gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &a910_gpio_keys_platform_data,
	},
};

/* A910 SPI/MMC */
static struct pxa2xx_spi_master a910_spi_masterinfo = {
	.clock_enable = CKEN_SSP1,
	.num_chipselect = 2,
	.enable_dma = 1,
};

static struct pxa2xx_spi_chip a910_mmcspi_chip_info = {
	.tx_threshold = 8,
	.rx_threshold = 8,
	.dma_burst_size = 8,
	.timeout = 10000,
	.gpio_cs = GPIO20_A910_MMC_CS,
};

static struct mmc_spi_platform_data a910_mci_platform_data = {
	.init           = ezx_mci_init,
	.exit           = ezx_mci_exit,
	.detect_delay   = 150 / (1000 / HZ),
};

static struct spi_board_info a910_spi_boardinfo[] __initdata = {
	{
		.modalias = "ezx-pcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 13000000,
		.platform_data = &ezx_pcap_platform_data,
		.controller_data = &ezx_pcap_chip_info,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "mmc_spi",
		.bus_num = 1,
		.chip_select = 1,
		.max_speed_hz = 13000000,
		.platform_data = &a910_mci_platform_data,
		.controller_data = &a910_mmcspi_chip_info,
		.mode = SPI_MODE_0,
	},
};

/* pcap-leds */
static struct pcap_leds_platform_data a910_leds = {
	.leds = {
		{
			.type = PCAP_VIB,
			.name = "a910:vibrator",
		},
	},
	.num_leds = 1,
};

struct platform_device a910_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &a910_leds,
	},
};

/* camera */
static int a910_pxacamera_init(struct device *dev)
{
	/* 
	 * GPIO50_GPIO is CAM_EN: active low
	 * GPIO28_GPIO is CAM_RST: active high
	 */
	gpio_request(MFP_PIN_GPIO50, "nCAM_EN");
	gpio_request(MFP_PIN_GPIO28, "CAM_RST");
	gpio_direction_output(MFP_PIN_GPIO50, 0);
	gpio_direction_output(MFP_PIN_GPIO28, 1);

	return 0;
}

static int a910_pxacamera_power(struct device *dev, int on)
{
	gpio_set_value(MFP_PIN_GPIO50, on ? 0 : 1);
	return 0;
}

static int a910_pxacamera_reset(struct device *dev)
{
	gpio_set_value(MFP_PIN_GPIO28, 0);
	msleep(10);
	gpio_set_value(MFP_PIN_GPIO28, 1);

	return 0;
}

struct pxacamera_platform_data a910_pxacamera_platform_data = {
	.init	= a910_pxacamera_init,
	.flags  = PXA_CAMERA_MASTER | PXA_CAMERA_DATAWIDTH_8 |
		PXA_CAMERA_PCLK_EN | PXA_CAMERA_MCLK_EN,
	.mclk_10khz = 5000,
};

static struct soc_camera_link a910_iclink = {
	.bus_id	= 0,
	.power = a910_pxacamera_power,
	.reset = a910_pxacamera_reset,
};

/* leds-lp3944 */
static struct lp3944_platform_data a910_lp3944_leds = {
	.dims_size = LP3944_DIMS_MAX,
	.leds_size = LP3944_LEDS_MAX,

	/* set two default dim modes, a fast one and a slow one.
	 * they will be used when brightness == {2,3}
	 */
	.dims = {
		[0] = {
			.period = 5,
			.dutycycle = 5,
		},
		[1] = {
			.period = 10,
			.dutycycle = 50,
		},
	},
	.leds = {
		[0] = {
			.name = "a910:red",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED,
		},
		[1] = {
			.name = "a910:green",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED,
		},
		[2] {
			.name = "a910:blue",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED,
		},
		/* Leds 3 and 4 are used as display power switches */
		[3] = {
			.name = "a910:cli_display",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED_INVERTED
		},
		[4] = {
			.name = "a910:main_display",
			.status = LP3944_LED_STATUS_ON,
			.type = LP3944_LED_TYPE_LED_INVERTED
		},
		[5] = { .type = LP3944_LED_TYPE_NONE },
		[6] = {
			.name = "a910:torch",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED,
		},
		[7] = {
			.name = "a910:flash",
			.status = LP3944_LED_STATUS_OFF,
			.type = LP3944_LED_TYPE_LED_INVERTED,
		},
	},
};

static struct i2c_board_info __initdata a910_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("mt9m111", 0x5d),
		.platform_data = &a910_iclink,
	}, {
		I2C_BOARD_INFO("lp3944", 0x60),
		.platform_data = &a910_lp3944_leds,
	},
};

static void __init a910_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a910_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(a910_i2c_board_info));

	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &a910_spi_masterinfo;
	ezx_spi_boardinfo[0].mode |= SPI_CS_HIGH;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(a910_spi_boardinfo));
	platform_device_register(&pcap_regulator_SW1_device);
	platform_device_register(&pcap_regulator_VAUX3_device);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_2);

	pxa_set_keypad_info(&a910_keypad_platform_data);

	platform_device_register(&a910_gpio_keys);
	platform_device_register(&pcap_keys_device);
	platform_device_register(&a910_leds_device);
	platform_device_register(&pcap_rtc_device);

	pxa_set_camera_info(&a910_pxacamera_platform_data);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A910, "Motorola EZX A910")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = a910_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E6
/* gpio_keys */
static struct gpio_keys_button e6_buttons[] = {
	[0] = {
		.code = KEY_SCREENLOCK,
		.gpio = GPIO15_E6_LOCK_SWITCH,
		.active_low = 0,
		.desc = "E6 lock switch",
		.type = EV_KEY,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data e6_gpio_keys_platform_data = {
	.buttons  = e6_buttons,
	.nbuttons = ARRAY_SIZE(e6_buttons),
};

static struct platform_device e6_gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &e6_gpio_keys_platform_data,
	},
};

/* pcap-leds */
static struct pcap_leds_platform_data e6_leds = {
	.leds = {
		{
			.type = PCAP_VIB,
			.name = "e6:vibrator",
		},
	},
	.num_leds = 1,
};

struct platform_device e6_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &e6_leds,
	},
};

static struct i2c_board_info __initdata e6_i2c_board_info[] = {
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e6_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e6_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(e6_i2c_board_info));

	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &ezx_spi_masterinfo;
	ezx_spi_boardinfo[0].mode |= SPI_CS_HIGH;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));
	platform_device_register(&pcap_regulator_SW1_device);
	platform_device_register(&pcap_regulator_VAUX2_device);

	pxa_set_mci_parent(&spi_pd->dev);
	pxa_set_mci_info(&ezx_mci_platform_data);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_2);

	pxa_set_keypad_info(&e6_keypad_platform_data);

	platform_device_register(&e6_gpio_keys);
	platform_device_register(&pcap_keys_device);
	platform_device_register(&pcap_ts_device);
	platform_device_register(&e6_leds_device);
	platform_device_register(&pcap_rtc_device);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E6, "Motorola EZX E6")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = e6_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E2
/* pcap-leds */
static struct pcap_leds_platform_data e2_leds = {
	.leds = {
		{
			.type = PCAP_VIB,
			.name = "e2:vibrator",
		},
	},
	.num_leds = 1,
};

struct platform_device e2_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &e2_leds,
	},
};

static struct i2c_board_info __initdata e2_i2c_board_info[] = {
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e2_init(void)
{
	struct platform_device *spi_pd;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e2_pin_config));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(e2_i2c_board_info));

	spi_pd = platform_device_alloc("pxa2xx-spi", 1);
	spi_pd->dev.platform_data = &ezx_spi_masterinfo;
	ezx_spi_boardinfo[0].mode |= SPI_CS_HIGH;
	platform_device_add(spi_pd);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));
	platform_device_register(&pcap_regulator_SW1_device);
	platform_device_register(&pcap_regulator_VAUX2_device);

	pxa_set_mci_parent(&spi_pd->dev);
	pxa_set_mci_info(&ezx_mci_platform_data);

	pxa_set_udc_parent(&spi_pd->dev);
	pxa_set_udc_info(&ezx_udc_info);

	set_pxa_fb_info(&ezx_fb_info_2);

	pxa_set_keypad_info(&e2_keypad_platform_data);

	platform_device_register(&pcap_keys_device);
	platform_device_register(&e2_leds_device);
	platform_device_register(&pcap_rtc_device);

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E2, "Motorola EZX E2")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = e2_init,
MACHINE_END
#endif

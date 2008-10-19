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
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/spi/mmc_spi.h>
#include <linux/irq.h>

#include <asm/setup.h>
#include <mach/pxafb.h>
#include <mach/ohci.h>
#include <mach/i2c.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>
#include <mach/mmc.h>

#include <mach/mfp-pxa27x.h>
#include <mach/pxa-regs.h>
#include <mach/pxa2xx-regs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "devices.h"
#include "generic.h"

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
		err = request_irq(gpio_to_irq(11), detect_int,
			IRQF_DISABLED | IRQF_SAMPLE_RANDOM |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"MMC card detect", data);
	}

	return err;
}

static int ezx_pcap_mmcsd_voltage(unsigned int vdd)
{
	/* 7 is the active bit in MMC_VDD_165_195 */
	int val = (vdd == 7) ? 6 : (vdd + 1) / 2 + 3;

	if (machine_is_ezx_e680() || machine_is_ezx_e6() ||
			machine_is_ezx_e2())
		return ezx_pcap_set_vreg(VAUX2, V_VAL, 3);
	else if (machine_is_ezx_a780() || machine_is_ezx_a1200() ||
			machine_is_ezx_a910())
		return ezx_pcap_set_vreg(VAUX3, V_VAL, val);
	else
		return -ENODEV;
}

static int ezx_pcap_mmcsd_power(int on)
{
	if (machine_is_ezx_e680() || machine_is_ezx_e6() ||
			machine_is_ezx_e2())
		return ezx_pcap_set_vreg(VAUX2, V_EN, on);
	else if (machine_is_ezx_a780() || machine_is_ezx_a1200() ||
			machine_is_ezx_a910())
		return ezx_pcap_set_vreg(VAUX3, V_EN, on);
	else
		return -ENODEV;
}

static void ezx_mci_setpower(struct device *dev, unsigned int vdd)
{
	ezx_pcap_mmcsd_voltage(vdd);
	ezx_pcap_mmcsd_power(1);
}

static void ezx_mci_exit(struct device *dev, void *data)
{
	ezx_pcap_mmcsd_power(0);
	if (!machine_is_ezx_a1200())
		free_irq(gpio_to_irq(11), data);
}

static struct pxamci_platform_data ezx_mci_platform_data = {
	.ocr_mask       = MMC_VDD_165_195|MMC_VDD_20_21|MMC_VDD_21_22
				|MMC_VDD_22_23|MMC_VDD_23_24|MMC_VDD_24_25
				|MMC_VDD_25_26|MMC_VDD_26_27|MMC_VDD_27_28
				|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31
				|MMC_VDD_31_32|MMC_VDD_32_33|MMC_VDD_33_34
				|MMC_VDD_34_35|MMC_VDD_35_36,
	.init           = ezx_mci_init,
	.setpower       = ezx_mci_setpower,
	.exit           = ezx_mci_exit,
	.detect_delay	= 150 / (1000 / HZ),
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

	/* STUART */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* PCAP SSP */
	GPIO29_SSP1_SCLK,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,
	GPIO24_GPIO,				/* pcap chip select */
	GPIO1_GPIO,				/* pcap interrupt */
	GPIO4_GPIO,				/* WDI_AP */
	GPIO55_GPIO,				/* SYS_RESTART */

	/* MMC */
	GPIO32_MMC_CLK,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,
	GPIO11_GPIO,				/* mmc detect */

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
	GPIO12_GPIO,

	/* bluetooth (bcm2035) */
	GPIO14_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* HOSTWAKE */
	GPIO48_GPIO,				/* RESET */
	GPIO28_GPIO,				/* WAKEUP */

	/* Neptune handshake */
	GPIO0_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* BP_RDY */
	GPIO57_GPIO,				/* AP_RDY */
	GPIO13_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* WDI */
	GPIO3_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* WDI2 */
	GPIO82_GPIO,				/* RESET */

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
	GPIO15_GPIO,

	/* EOC */
	GPIO10_GPIO,

	/* bluetooth (bcm2045) */
	GPIO13_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* HOSTWAKE */
	GPIO37_GPIO,				/* RESET */
	GPIO57_GPIO,				/* WAKEUP */

	/* Neptune handshake */
	GPIO0_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* BP_RDY */
	GPIO96_GPIO,				/* AP_RDY */
	GPIO3_GPIO | WAKEUP_ON_LEVEL_HIGH,	/* WDI */
	GPIO116_GPIO,				/* RESET */
	GPIO41_GPIO,				/* BP_FLASH */

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

};
#endif

#ifdef CONFIG_MACH_EZX_E6
static unsigned long e6_pin_config[] __initdata = {

};
#endif

/* KEYPAD */
#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)

#ifdef CONFIG_MACH_EZX_A780
static unsigned int a780_key_map[] = {
	KEY(0, 0, KEY_PHONE),
	KEY(0, 1, KEY_MENU),
	KEY(0, 2, KEY_CANCEL),
	KEY(0, 3, KEY_PAGEUP),
	KEY(0, 4, KEY_UP),

	KEY(1, 0, KEY_KP1),
	KEY(1, 1, KEY_KP2),
	KEY(1, 2, KEY_KP3),
	KEY(1, 3, KEY_ENTER),
	KEY(1, 4, KEY_KPENTER),

	KEY(2, 0, KEY_KP4),
	KEY(2, 1, KEY_KP5),
	KEY(2, 2, KEY_KP6),
	KEY(2, 3, KEY_RECORD),
	KEY(2, 4, KEY_LEFT),

	KEY(3, 0, KEY_KP7),
	KEY(3, 1, KEY_KP8),
	KEY(3, 2, KEY_KP9),
	KEY(3, 3, KEY_HOME),
	KEY(3, 4, KEY_RIGHT),

	KEY(4, 0, KEY_KPASTERISK),
	KEY(4, 1, KEY_KP0),
	KEY(4, 2, KEY_KPDOT),
	KEY(4, 3, KEY_PAGEDOWN),
	KEY(4, 4, KEY_DOWN),
};

static struct pxa27x_keypad_platform_data a780_keypad_platform_data = {
	.matrix_key_rows = 5,
	.matrix_key_cols = 5,
	.matrix_key_map = a780_key_map,
	.matrix_key_map_size = ARRAY_SIZE(a780_key_map),

	.direct_key_num = 1,
	.direct_key_map = { KEY_CAMERA, 0, 0, 0, 0, 0, 0, 0 },

	.debounce_interval = 30,
};
#endif /* CONFIG_MACH_EZX_A780 */

#ifdef CONFIG_MACH_EZX_E680
static unsigned int e680_key_map[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_RESERVED),
	KEY(0, 3, KEY_PHONE),

	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_LEFT),
	KEY(1, 2, KEY_VOLUMEUP),
	KEY(1, 3, KEY_VOLUMEDOWN),

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

	.direct_key_num = 6,
	.direct_key_map = {
		KEY_CAMERA,
		KEY_RESERVED,
		KEY_RESERVED,
		KEY_HOME,
		KEY_POWER,
		KEY_MENU,
		0,
		0,
	},

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
	KEY(2, 5, KEY_ENTER),

	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_HOME),
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

#ifdef CONFIG_MACH_EZX_A910
static unsigned int a910_key_map[] = {
	KEY(0, 0, KEY_KP6),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_PAGEDOWN),
	KEY(0, 3, KEY_KPENTER),
	KEY(0, 4, KEY_KP5),
	KEY(0, 5, KEY_CAMERA),

	KEY(1, 0, KEY_KP8),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_RESERVED),
	KEY(1, 3, KEY_F1), /* Left SoftKey */
	KEY(1, 4, KEY_KPASTERISK),
	KEY(1, 5, KEY_RESERVED),

	KEY(2, 0, KEY_KP7),
	KEY(2, 1, KEY_KP9),
	KEY(2, 2, KEY_RECORD),
	KEY(2, 3, KEY_F2), /* Right SoftKey */
	KEY(2, 4, KEY_CANCEL),
	KEY(2, 5, KEY_ENTER),

	KEY(3, 0, KEY_KP2),
	KEY(3, 1, KEY_UP),
	KEY(3, 2, KEY_PHONE),
	KEY(3, 3, KEY_KP0),
	KEY(3, 4, KEY_KP1),
	KEY(3, 5, KEY_RECORD),

	KEY(4, 0, KEY_KP4),
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_PAGEUP),
	KEY(4, 3, KEY_KPDOT),
	KEY(4, 4, KEY_KP3),
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

#endif /* CONFIG_KEYBOARD_PXA27x */

/* PCAP */
static void ezx_pcap_init(void)
{
	/* set SW1 sleep to keep SW1 1.3v in sync mode */
	/*  SW1 active in sync mode */
	ezx_pcap_set_sw(SW1, SW_MODE, 0x1);

	/*  set core voltage */
	ezx_pcap_set_sw(SW1, SW_VOLTAGE, SW_VOLTAGE_1250);
}

static struct pcap_platform_data ezx_pcap_platform_data = {
	.cs     = 24,
	.irq    = IRQ_GPIO1,
	.config = 0,
	.init   = ezx_pcap_init,
};

static void pcap_cs_control(u32 command)
{
	if (command & PXA2XX_CS_ASSERT) {
		gpio_set_value(ezx_pcap_platform_data.cs,
		 (ezx_pcap_platform_data.config & PCAP_CS_INVERTED) ? 0 : 1);
	} else {
		gpio_set_value(ezx_pcap_platform_data.cs,
		 (ezx_pcap_platform_data.config & PCAP_CS_INVERTED) ? 1 : 0);
	}
}

static struct pxa2xx_spi_chip ezx_pcap_chip_info = {
	.tx_threshold = 1,
	.rx_threshold = 1,
	.dma_burst_size = 0,
	.timeout = 100,
	.cs_control = pcap_cs_control,
};

static struct pxa2xx_spi_master ezx_spi_masterinfo = {
	.clock_enable = CKEN_SSP1,
	.num_chipselect = 1,
	.enable_dma = 1,
};

static struct spi_board_info ezx_spi_boardinfo[] __initdata = {
	{
		.modalias = "ezx-pcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 13000000,
		.platform_data = &ezx_pcap_platform_data,
		.controller_data = &ezx_pcap_chip_info,
		.mode = SPI_MODE_0,
	},
};

static void __init ezx_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
	/* We have two ram chips. First one with 32MB at 0xA0000000 and a second
	 * 16MB one at 0xAC000000
	 */
	mi->nr_banks = 2;
	mi->bank[0].start = 0xa0000000;
	mi->bank[0].node = 0;
	mi->bank[0].size = (32*1024*1024);
	mi->bank[1].start = 0xac000000;
	mi->bank[1].node = 1;
	mi->bank[1].size = (16*1024*1024);
}

#ifdef CONFIG_MACH_EZX_A780
static void __init a780_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen1_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a780_pin_config));

	pxa_set_i2c_info(NULL);

	ezx_pcap_platform_data.config |= PCAP_SECOND_PORT | PCAP_CS_INVERTED;
	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_mci_info(&ezx_mci_platform_data);

	set_pxa_fb_info(&ezx_fb_info_1);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
	pxa_set_keypad_info(&a780_keypad_platform_data);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A780, "Motorola EZX A780")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &a780_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E680
static struct i2c_board_info __initdata e680_i2c_board_info[] = {
	{ I2C_BOARD_INFO("lm4857", 0x7c) },
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e680_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen1_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e680_pin_config));

	i2c_register_board_info(0, ARRAY_AND_SIZE(e680_i2c_board_info));

	ezx_pcap_platform_data.config |= PCAP_SECOND_PORT | PCAP_CS_INVERTED;
	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_mci_info(&ezx_mci_platform_data);

	set_pxa_fb_info(&ezx_fb_info_1);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
	pxa_set_keypad_info(&e680_keypad_platform_data);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E680, "Motorola EZX E680")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &e680_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_A1200
static struct i2c_board_info __initdata a1200_i2c_board_info[] = {
	{ I2C_BOARD_INFO("ezx-eoc", 0x17) },
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init a1200_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a1200_pin_config));

	i2c_register_board_info(0, ARRAY_AND_SIZE(a1200_i2c_board_info));

	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_mci_info(&ezx_mci_platform_data);

	set_pxa_fb_info(&ezx_fb_info_2);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
	pxa_set_keypad_info(&a1200_keypad_platform_data);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A1200, "Motorola EZX A1200")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &a1200_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_A910
static struct i2c_board_info __initdata a910_i2c_board_info[] = {
	{ I2C_BOARD_INFO("ezx-eoc", 0x17) },
};

/* A910 SPI/MMC */
static void a910_mmc_cs_control(u32 command)
{
	if (command & PXA2XX_CS_ASSERT)
		gpio_set_value(20, 0);
	else {
		gpio_set_value(20, 1);
	}
}

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
	.cs_control = a910_mmc_cs_control,
};

static struct mmc_spi_platform_data a910_mci_platform_data = {
	.ocr_mask       = MMC_VDD_165_195|MMC_VDD_20_21|MMC_VDD_21_22
				|MMC_VDD_22_23|MMC_VDD_23_24|MMC_VDD_24_25
				|MMC_VDD_25_26|MMC_VDD_26_27|MMC_VDD_27_28
				|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31
				|MMC_VDD_31_32|MMC_VDD_32_33|MMC_VDD_33_34
				|MMC_VDD_34_35|MMC_VDD_35_36,
	.init           = ezx_mci_init,
	.setpower       = ezx_mci_setpower,
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

static void __init a910_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(a910_pin_config));

	i2c_register_board_info(0, ARRAY_AND_SIZE(a910_i2c_board_info));

	gpio_direction_output(20, 1);
	pxa2xx_set_spi_info(1, &a910_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(a910_spi_boardinfo));

	set_pxa_fb_info(&ezx_fb_info_2);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
	pxa_set_keypad_info(&a910_keypad_platform_data);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_A910, "Motorola EZX A910")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &a910_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E6
static struct i2c_board_info __initdata e6_i2c_board_info[] = {
	{ I2C_BOARD_INFO("ezx-eoc", 0x17) },
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e6_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e6_pin_config));

	i2c_register_board_info(0, ARRAY_AND_SIZE(e6_i2c_board_info));

	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_mci_info(&ezx_mci_platform_data);

	set_pxa_fb_info(&ezx_fb_info_2);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
#warning "Please contribute a keypad map for E6"
	/* pxa_set_keypad_info(&e6_keypad_platform_data); */
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E6, "Motorola EZX E6")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &e6_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_EZX_E2
static struct i2c_board_info __initdata e2_i2c_board_info[] = {
	{ I2C_BOARD_INFO("ezx-eoc", 0x17) },
	{ I2C_BOARD_INFO("tea5767", 0x81) },
};

static void __init e2_init(void)
{
	pxa2xx_mfp_config(ARRAY_AND_SIZE(ezx_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(gen2_pin_config));
	pxa2xx_mfp_config(ARRAY_AND_SIZE(e2_pin_config));

	i2c_register_board_info(0, ARRAY_AND_SIZE(e2_i2c_board_info));

	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_mci_info(&ezx_mci_platform_data);

	set_pxa_fb_info(&ezx_fb_info_2);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
#warning "Please contribute a keypad map for E2"
	/* pxa_set_keypad_info(&e2_keypad_platform_data); */
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(EZX_E2, "Motorola EZX E2")
	.phys_io        = 0x40000000,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.fixup			= ezx_fixup,
	.boot_params    = 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa27x_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = &e2_init,
MACHINE_END
#endif

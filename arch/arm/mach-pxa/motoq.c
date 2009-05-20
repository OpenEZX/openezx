#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/leds-pcap.h>

#include <media/soc_camera.h>

#include <asm/setup.h>
#include <mach/pxafb.h>
#include <mach/ohci.h>
#include <mach/i2c.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>
#include <mach/camera.h>
#include <mach/ezx-bp.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxa-regs.h>
#include <mach/pxa2xx-regs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>

#include "devices.h"
#include "generic.h"

#define GPIO11_MOTOQ_SD_N 11
#define GPIO18_MOTOQ_USB_VBUS 18

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

static void motoq_lcd_power(int on, struct fb_var_screeninfo *var)
{
	if(on) {
		//udelay(6000);
	}
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
	.pxafb_lcd_power	= motoq_lcd_power,
};

static unsigned int motoq_matrix_key_map[] = {

KEY(0,0,KEY_O),
KEY(0,1,KEY_Y),
KEY(0,2,KEY_A),
KEY(0,3,KEY_D),
KEY(0,4,KEY_W),
KEY(0,5,KEY_G),
KEY(0,6,KEY_J),
KEY(0,7,KEY_L),

KEY(1,0,KEY_U),
KEY(1,1,KEY_V),
KEY(1,2,KEY_DOT),
KEY(1,3,KEY_0),
KEY(1,4,KEY_LEFTALT),	/* speakerphone correct */
KEY(1,5,KEY_LEFTCTRL),	/* mail !!!!! */
KEY(1,6,KEY_8),
KEY(1,7,KEY_ENTER),	/* ok */

KEY(2,0,KEY_F),
KEY(2,1,KEY_O),
KEY(2,2,KEY_O),
KEY(2,3,KEY_Z),
KEY(2,4,KEY_ENTER),	/* wheel click correct */
KEY(2,5,KEY_BACKSPACE),	/* backspace (left arrow softkey) */
KEY(2,6,KEY_RIGHTSHIFT),	/* shift ! */
KEY(2,7,KEY_R),

KEY(3,0,KEY_T),
KEY(3,1,KEY_E),
KEY(3,2,KEY_RIGHTALT),	/* fn !?? */
KEY(3,3,KEY_I),
KEY(3,4,KEY_ESC),	/* wheel back correct */
KEY(3,5,KEY_B),
KEY(3,6,KEY_TAB),	/* right softkey correct */
KEY(3,7,KEY_4),

KEY(4,0,KEY_H),
KEY(4,1,KEY_LEFTCTRL), /* mail */
KEY(4,2,KEY_X),
KEY(4,3,KEY_V),
KEY(4,4,KEY_LEFTCTRL),	/* left softkey correct */
KEY(4,5,KEY_P),
KEY(4,6,KEY_M),
KEY(4,7,KEY_UP),	/* up ? */

KEY(5,0,KEY_K),
KEY(5,1,KEY_ENTER),	/* ok (center button) */
KEY(5,2,KEY_N),
KEY(5,3,KEY_DOWN),	/* not B? */
KEY(5,4,KEY_LEFT),	/* left (widescreen) correct */
KEY(5,5,KEY_C),
KEY(5,6,KEY_UP),	/* up (widescreen) correct */
KEY(5,7,KEY_Q),

KEY(6,0,KEY_S),
KEY(6,1,KEY_SPACE),
KEY(6,2,KEY_ENTER),	/* return key */
KEY(6,3,KEY_N),
KEY(6,4,KEY_RIGHT),	/* right (widescreen) correct */
KEY(6,5,KEY_MINUS),	/* call? (green phone) !!!!! */
KEY(6,6,KEY_RIGHTALT),	/* cmaera correct */
KEY(6,7,KEY_SLASH),	/* home (house) correct */

KEY(7,0,KEY_LEFTBRACE),	/* alt/fn ? */
KEY(7,1,KEY_Y),
KEY(7,2,KEY_BACKSLASH),	/* backspace ? */
KEY(7,3,KEY_SEMICOLON), /* backspace !? */
KEY(7,4,KEY_K),
KEY(7,5,KEY_C),
KEY(7,6,KEY_RIGHTALT), /* fn correct */
KEY(7,7,KEY_9),

};

static struct pxa27x_keypad_platform_data motoq_keypad_info = {

  .matrix_key_rows    = 8,
  .matrix_key_cols    = 8,
  .matrix_key_map     = motoq_matrix_key_map,
  .matrix_key_map_size = ARRAY_SIZE(motoq_matrix_key_map),

  .debounce_interval = 30,  /* from littleton.c */

};

/* OHCI Controller */
static int ezx_ohci_init(struct device *dev)
{
	void __iomem *iobase;

	iobase = ioremap(0x40600000,0x1000);
	__raw_writel(0x00000002, iobase+0x24);
	iounmap(iobase);

	iobase = ioremap(0x4C000000,0x1000);
	__raw_writel(__raw_readl(iobase + 0x64) & ~((1<<10)|(1<<11)|(1<<5)), iobase+0x64);
	iounmap(iobase);

	return 0;
}

static struct pxaohci_platform_data ezx_ohci_platform_data = {
	.port_mode	= PMM_NPS_MODE,
	.init		= ezx_ohci_init,
};

/* SOC Camera (based on A780) */

#if defined(CONFIG_LEDS_PCAP) || defined(CONFIG_LEDS_PCAP_MODULES)
static struct pcap_leds_platform_data motoq_leds = {
	.leds = {
		{
			.type = PCAP_BL0,
			.name = "a780:main",
		}, {
			.type = PCAP_BL1,
			.name = "motoq:keylight",
		},
	},
	.num_leds = 2,
};

struct platform_device motoq_leds_device = {
	.name           = "pcap-leds",
	.id             = -1,
	.dev = {
		.platform_data = &motoq_leds,
	},
};
#endif

static int motoq_pxacamera_init(struct device *dev)
{
	/* 
	 * GPIO50_GPIO is CAM_EN: active low
	 * GPIO19_GPIO is CAM_RST: active high
	 */
	gpio_set_value(MFP_PIN_GPIO50, 0);
	gpio_set_value(MFP_PIN_GPIO19, 1);

	return 0;
}

static int motoq_pxacamera_power(struct device *dev, int on)
{
	gpio_set_value(MFP_PIN_GPIO50, on ? 0 : 1);

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

	return 0;
}

static int motoq_pxacamera_reset(struct device *dev)
{
	gpio_set_value(MFP_PIN_GPIO19, 0);
	msleep(10);
	gpio_set_value(MFP_PIN_GPIO19, 1);

	return 0;
}

struct pxacamera_platform_data motoq_pxacamera_platform_data = {
	.init	= motoq_pxacamera_init,
	.flags  = PXA_CAMERA_MASTER | PXA_CAMERA_DATAWIDTH_8 |
		PXA_CAMERA_PCLK_EN | PXA_CAMERA_MCLK_EN | PXA_CAMERA_PCP,
	.mclk_10khz = 1000,
};

static struct soc_camera_link motoq_iclink = {
	.bus_id	= 0,
	.power = motoq_pxacamera_power,
	.reset = motoq_pxacamera_reset,
};



static void ezx_pcap_init(void)
{
	/* set SW1 sleep to keep SW1 1.3v in sync mode */
	/*  SW1 active in sync mode */
	//ezx_pcap_set_sw(SW1, SW_MODE, 0x1);

	/*  set core voltage */
	//ezx_pcap_set_sw(SW1, SW_VOLTAGE, SW_VOLTAGE_1250);

	ezx_pcap_write(PCAP_REG_BUSCTRL,
			(PCAP_BUSCTRL_RS232ENB | PCAP_BUSCTRL_VUSB_EN));

	ezx_pcap_set_vreg(V6, V_EN, 1);

	//gpio_direction_output(120, 0);
	//gpio_direction_output(119, 0);
};


static struct pcap_platform_data ezx_pcap_platform_data = {
/*	.cs	=	24,	*/
	.cs	=	28,	/* from drwyrm */
				/* rxd: 26?? */
	.irq	=	gpio_to_irq(1),
/*	.irq	=	gpio_to_irq(18),	*/
	.config =	0,
/*	.config =	PCAP_CS_INVERTED,	*/
	.init	=	ezx_pcap_init,
};

static void pcap_cs_control(u32 command)
{
	if (command & PXA2XX_CS_ASSERT) {
		gpio_set_value(ezx_pcap_platform_data.cs,
		 (ezx_pcap_platform_data.config & PCAP_CS_INVERTED) ? 0 : 1 );
	} else {
		gpio_set_value(ezx_pcap_platform_data.cs,
		 (ezx_pcap_platform_data.config & PCAP_CS_INVERTED) ? 1 : 0 );
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

/*
static void __init motoq_fixup(struct machine_desc *desc, struct tag *tags,
			char **cmdlne, struct meminfo *mi)
{
};
*/

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

	/* For A780 support (connected with Neptune GSM chip) */
	GPIO30_USB_P3_2,	/* ICL_TXENB */
	GPIO31_USB_P3_6,	/* ICL_VPOUT */
	GPIO90_USB_P3_5,	/* ICL_VPIN */
	GPIO91_USB_P3_1,	/* ICL_XRXD */
	GPIO56_USB_P3_4,	/* ICL_VMOUT */
	GPIO113_USB_P3_3,	/* /ICL_VMIN */


	/* sound */
//	GPIO52_SSP3_SCLK,
//	GPIO83_SSP3_SFRM,
//	GPIO81_SSP3_TXD,
//	GPIO89_SSP3_RXD,

//	/* ssp2 pins to in */
//	GPIO22_GPIO,				/* SSP2_SCLK */
//	GPIO37_GPIO,				/* SSP2_SFRM */
//	GPIO38_GPIO,				/* SSP2_TXD */
//	GPIO88_GPIO,				/* SSP2_RXD */

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
//	GPIO50_GPIO | MFP_DIR_OUT,		/* CAM_EN */
//	GPIO19_GPIO | MFP_DIR_OUT,		/* CAM_RST */

	/* EMU */
//	GPIO120_GPIO,				/* EMU_MUX1 */
//	GPIO119_GPIO,				/* EMU_MUX2 */
//	GPIO86_GPIO,				/* SNP_INT_CTL */
//	GPIO87_GPIO,				/* SNP_INT_IN */

};

static struct i2c_board_info __initdata motoq_i2c_board_info[] = {
/*	{ I2C_BOARD_INFO("ezx-eoc", 0x17) },	*/
	{
		I2C_BOARD_INFO("mt9m11", 0x5d),
		.platform_data = &motoq_iclink,
	},
};

static struct platform_device *devices[] __initdata = {
	&ezx_backlight_device,
};

static void __init motoq_init(void)
{
	set_pxa_fb_info(&motoq_lcd_info);

	pxa_set_keypad_info(&motoq_keypad_info);

	pxa_set_ohci_info(&ezx_ohci_platform_data);

	pxa2xx_mfp_config(ARRAY_AND_SIZE(pin_config));

	pxa2xx_set_spi_info(1, &ezx_spi_masterinfo);
	spi_register_board_info(ARRAY_AND_SIZE(ezx_spi_boardinfo));

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(motoq_i2c_board_info));

#if defined(CONFIG_LEDS_PCAP) || defined(CONFIG_LEDS_PCAP_MODULES)
	platform_device_register(&motoq_leds_device);
#endif
#if defined(CONFIG_VIDEO_PXA27x) || defined(CONFIG_VIDEO_PXA27x_MODULE)
	pxa_set_camera_info(&motoq_pxacamera_platform_data);
#endif

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

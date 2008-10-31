/*
 *  BP handshake code for Motorola EZX phones
 *
 *  Copyright (c) 2007 Daniel Ribeiro <wyrm@openezx.org>
 *
 *  Based on Motorola's a780.c Copyright (c) 2003-2005 Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>

#include <mach/pxa-regs.h>
#include <mach/pxa27x-udc.h>
#include <mach/mfp-pxa27x.h>
#include <linux/gpio.h>
#include <mach/ohci.h>


#define BP_RDY_TIMEOUT		0x000c0000

#if 1
#define DEBUGP(x, args ...)	printk(x, ##args)
#else
#define DEBUGP(x, args ...)
#endif

extern void usb_send_readurb(void);

/* check power down condition */
static inline void check_power_off(void)
{
	if (gpio_get_value(bp->bp_rdy) == 0) {
		DEBUGP("BP request poweroff!\n");
		/*
		 * It is correct to power off here, the following line is
		 * commented out because e680 lowers WDI2 when BP is in
		 * flash mode, otherwise WDI2 is used to detect low
		 * battery. You can safely uncomment this line if you are
		 * using this kernel with BP in normal mode.
		 */
#if 0 /* some versions of BP firmware doesnt honor this */
		pm_power_off();
#endif
	}
}


inline int bp_handshake_passed(void)
{
	return (bp->cur_step > bp->last_step);
}
EXPORT_SYMBOL_GPL(bp_handshake_passed);

static void handshake(void)
{
	/* step 1: check MCU_INT_SW or BP_RDY is low (now it is checked in apboot) */
	DEBUGP("bp handshake entered!\n");
	if (bp->cur_step == 1) {
		int timeout = BP_RDY_TIMEOUT;

		/* config MCU_INT_SW, BP_RDY as input */
		gpio_direction_input(bp->bp_mcu_int_sw);
		gpio_direction_input(bp->bp_rdy);

		while (timeout--) {
			if (gpio_get_value(bp->bp_mcu_int_sw) == 0
			    || gpio_get_value(bp->bp_rdy) == 0) {
				bp->cur_step++;
				break;
			}

			check_power_off();
		}
		DEBUGP("ezx-bp: handshake step 1\n");
	}

	/* step 2: wait BP_RDY is low */
	if (bp->cur_step == 2) {
		if (gpio_get_value(bp->bp_rdy) == 0) {
			/* config MCU_INT_SW as output */
			gpio_direction_output(bp->bp_mcu_int_sw, 0);
//			gpio_set_value(bp->bp_mcu_int_sw, 0);

			bp->cur_step++;
			DEBUGP("ezx-bp: handshake step 2\n");
		}
	}

	/* step 3: wait BP_RDY is high */
	else if (bp->cur_step == 3) {
		if (gpio_get_value(bp->bp_rdy)) {
			bp->cur_step++;
			DEBUGP("ezx-bp: handshake step 3 %d\n",jiffies);
			
			gpio_direction_output(bp->ap_rdy, 1);
//			gpio_set_value(bp->ap_rdy, 1);
		}
	}
}

irqreturn_t bp_wdi_handler(int irq, void *dev_id)
{
	DEBUGP("BP Lowered WDI line. This is not good :( %d\n",jiffies);
	return IRQ_HANDLED;
}

static irqreturn_t bp_rdy_handler(int irq, void *dev_id)
{

	
	if (!bp_handshake_passed()) {
		handshake();
	}
#ifdef CONFIG_TS0710_MUX_USB
	else usb_send_readurb();
#endif
	return IRQ_HANDLED;
}


static int __init ezxbp_probe(struct platform_device *pdev)
{
	int ret;
	
	bp = pdev->dev.platform_data;

	set_irq_type(gpio_to_irq(bp->bp_wdi), IRQ_TYPE_EDGE_FALLING);
	request_irq(gpio_to_irq(bp->bp_wdi), bp_wdi_handler, IRQF_DISABLED,
		    "bp wdi", bp);

	set_irq_type(gpio_to_irq(bp->bp_rdy), IRQ_TYPE_EDGE_BOTH);
	request_irq(gpio_to_irq(bp->bp_rdy), bp_rdy_handler, IRQF_DISABLED,
			"bp rdy", bp);

	/* turn on BP */
	gpio_direction_output(bp->bp_reset, 1);
//	gpio_set_value(gen2_bp_single.bp_reset, 1);

	handshake();

	return 0;
}

static int ezxbp_remove(struct platform_device *dev)
{
	return 0;

	free_irq(gpio_to_irq(bp->bp_wdi), bp);
	free_irq(gpio_to_irq(bp->bp_rdy), bp);

	return 0;
}

static int ezxbp_suspend(struct platform_device *dev, pm_message_t state)
{
	DEBUGP("bp suspend!\n");
/*	gpio_set_value(bp->bp_mcu_int_sw, 0); */
	return 0;
}

static int ezxbp_resume(struct platform_device *dev)
{
	DEBUGP("bp resume!\n");
/*	gpio_set_value(bp->bp_mcu_int_sw, 1); */
	return 0;
}
static struct platform_driver ezxbp_driver = {
	.probe		= ezxbp_probe,
	.remove		= ezxbp_remove,
#warning FIXME: missing suspend/resume support
	.suspend	= ezxbp_suspend,
	.resume		= ezxbp_resume,
	.driver		= {
		.name	= "ezx-bp",
		.owner	= THIS_MODULE,
	},
};



static struct platform_device ezxbp_device = {
	.name		= "ezx-bp",
	.dev		= {
		.platform_data	= &gen2_bp_single,
	},
	.id		= -1,
};

static struct platform_device *devices[] __initdata = {
	&ezxbp_device,
};

/* OHCI Controller */
static int ezx_ohci_init(struct device *dev)
{

	UP3OCR = 0x00000002;

	UHCHR = UHCHR & ~(UHCHR_SSEP2 | UHCHR_SSEP3 | UHCHR_SSE);

	return 0;
}

static struct pxaohci_platform_data ezx_ohci_platform_data = {
	.port_mode	= PMM_NPS_MODE,
	.init		= ezx_ohci_init,
};



int __init ezxbp_init(void)
{
	pxa_set_ohci_info(&ezx_ohci_platform_data);
	platform_add_devices(devices, 1);
	return platform_driver_register(&ezxbp_driver);
}

void ezxbp_fini(void)
{
	return platform_driver_unregister(&ezxbp_driver);
}

subsys_initcall(ezxbp_init);
module_exit(ezxbp_fini);

MODULE_DESCRIPTION("Motorola BP Control driver");
MODULE_AUTHOR("Daniel Ribeiro <wyrm@openezx.org>");
MODULE_LICENSE("GPL");


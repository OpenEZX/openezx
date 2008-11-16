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
#include <linux/delay.h>

#include <mach/pxa-regs.h>
#include <mach/pxa27x-udc.h>
#include <mach/mfp-pxa27x.h>
#include <linux/gpio.h>
#include <mach/ohci.h>
#include <mach/ezx-bp.h>

#define BP_RDY_TIMEOUT		0x000c0000

#if 1
#define DEBUGP(x, args ...)	printk(x, ##args)
#else
#define DEBUGP(x, args ...)
#endif

extern void usb_send_readurb(void);

static struct ezxbp_config *bp;
static int step;

/* check power down condition */
static inline void check_power_off(void)
{
	if (bp->bp_wdi2 >= 0 && gpio_get_value(bp->bp_wdi2) == 0) {
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

int ezx_wake_bp(void)
{
	int t = 100;

	/* bp is awake */
	if (gpio_get_value(bp->bp_rdy))
		return 0;

	/* bp is sleeping, wake it up */
	DEBUGP("wake up bp\n");
	gpio_set_value(bp->ap_rdy, 0);
	udelay(125);
	gpio_set_value(bp->ap_rdy, 1);

	while (!gpio_get_value(bp->bp_rdy) && --t)
		udelay(1);

	if (!t)
		return -ETIMEDOUT;
	return 0;
}
EXPORT_SYMBOL_GPL(ezx_wake_bp);

inline int bp_handshake_passed(void)
{
	return (step > 3);
}

static void handshake(void)
{
	DEBUGP("bp handshake entered!\n");
	/* step 1: check MCU_INT_SW or BP_RDY is low (now it is checked in apboot) */
	if (step == 1) {
		int timeout = BP_RDY_TIMEOUT;

		while (timeout--) {
			if (gpio_get_value(bp->ap_rdy) == 0
			    || gpio_get_value(bp->bp_rdy) == 0) {
				step++;
				break;
			}

			check_power_off();
		}
		DEBUGP("ezx-bp: handshake step 1\n");
	}

	/* step 2: wait BP_RDY is low */
	if (step == 2) {
		if (gpio_get_value(bp->bp_rdy) == 0) {
			/* config MCU_INT_SW as output */
			gpio_direction_output(bp->ap_rdy, 0);
			step++;
			DEBUGP("ezx-bp: handshake step 2\n");
		}
	}

	/* step 3: wait BP_RDY is high */
	else if (step == 3) {
		if (gpio_get_value(bp->bp_rdy)) {
			step++;
			gpio_direction_output(bp->ap_rdy, 1);
			DEBUGP("ezx-bp: handshake done!\n");
			if (bp->init)
				bp->init();
		}
	}
}

static irqreturn_t bp_wdi_handler(int irq, void *dev_id)
{
	DEBUGP("BP Lowered WDI line. This is not good :(\n");
	return IRQ_HANDLED;
}

static irqreturn_t bp_wdi2_handler(int irq, void *dev_id)
{
	DEBUGP("BP request power off\n");
	return IRQ_HANDLED;
}

static irqreturn_t bp_rdy_handler(int irq, void *dev_id)
{
	DEBUGP("BP rdy irq\n");

	if (!bp_handshake_passed()) {
		handshake();
		if (bp_handshake_passed() && bp->bp_wdi2 >= 0)
			disable_irq(gpio_to_irq(bp->bp_wdi2));
	}
#ifdef CONFIG_TS0710_MUX_USB
	else usb_send_readurb();
#endif
	return IRQ_HANDLED;
}

static int __init ezxbp_probe(struct platform_device *pdev)
{
	bp = pdev->dev.platform_data;
	step = bp->first_step;

	set_irq_type(gpio_to_irq(bp->bp_wdi), IRQ_TYPE_EDGE_FALLING);
	request_irq(gpio_to_irq(bp->bp_wdi), bp_wdi_handler, IRQF_DISABLED,
		    "bp wdi", bp);

	set_irq_type(gpio_to_irq(bp->bp_rdy), IRQ_TYPE_EDGE_RISING);
	request_irq(gpio_to_irq(bp->bp_rdy), bp_rdy_handler, IRQF_DISABLED,
			"bp rdy", bp);

	if (bp->bp_wdi2 >= 0) {
		set_irq_type(gpio_to_irq(bp->bp_wdi2), IRQ_TYPE_EDGE_FALLING);
		request_irq(gpio_to_irq(bp->bp_wdi2), bp_wdi2_handler,
				IRQF_DISABLED, "bp wdi2", bp);
	}

	if (bp->bp_reset >= 0)
		gpio_direction_output(bp->bp_reset, 1);

	handshake();

	return 0;
}

static int ezxbp_remove(struct platform_device *dev)
{
	return 0;

	free_irq(gpio_to_irq(bp->bp_wdi), bp);
	free_irq(gpio_to_irq(bp->bp_rdy), bp);
	if (bp->bp_wdi2 >= 0)
		free_irq(gpio_to_irq(bp->bp_wdi2), bp);

	return 0;
}

static int ezxbp_suspend(struct platform_device *dev, pm_message_t state)
{
	DEBUGP("bp suspend!\n");
	gpio_set_value(bp->ap_rdy, 0);
	return 0;
}

static int ezxbp_resume(struct platform_device *dev)
{
	DEBUGP("bp resume!\n");
	gpio_set_value(bp->ap_rdy, 1);
	return 0;
}

static struct platform_driver ezxbp_driver = {
	.probe		= ezxbp_probe,
	.remove		= ezxbp_remove,
	.suspend	= ezxbp_suspend,
	.resume		= ezxbp_resume,
	.driver		= {
		.name	= "ezx-bp",
		.owner	= THIS_MODULE,
	},
};

int __init ezxbp_init(void)
{
	return platform_driver_register(&ezxbp_driver);
}

void ezxbp_fini(void)
{
	return platform_driver_unregister(&ezxbp_driver);
}

module_init(ezxbp_init);
module_exit(ezxbp_fini);

MODULE_DESCRIPTION("Motorola BP Control driver");
MODULE_AUTHOR("Daniel Ribeiro <wyrm@openezx.org>");
MODULE_LICENSE("GPL");

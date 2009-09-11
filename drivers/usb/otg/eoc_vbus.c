/*
 * eoc-vbus.c - EOC VBUS sensing driver for B peripheral devices
 * based on gpio-vbus
 *
 * Copyright (c) 2009 guiming zhuo <gmzhuo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <linux/regulator/consumer.h>

#include <linux/usb/gadget.h>
#include <linux/usb/eoc_vbus.h>
#include <linux/usb/otg.h>
#include <linux/mfd/ezx-eoc.h>


/*
 * EOC VBUS sensing driver for B peripheral only devices
 * with eoc transceivers. It can control a D+ pullup and
 * a regulator to limit the current drawn from VBUS.
 *
 * Needs to be loaded before the UDC driver that will use it.
 */
struct eoc_vbus_data {
	struct otg_transceiver otg;
	struct device          *dev;
	struct regulator       *vbus_draw;
	int			vbus_draw_enabled;
	unsigned		mA;
	struct work_struct	work;
	struct eoc_chip	       *eoc;
	void (*mach_switch_mode)(enum eoc_transceiver_mode);
};


/* interface to regulator framework */
static void set_vbus_draw(struct eoc_vbus_data *eoc_vbus, unsigned mA)
{
	struct regulator *vbus_draw = eoc_vbus->vbus_draw;
	int enabled;

	if (!vbus_draw)
		return;

	enabled = eoc_vbus->vbus_draw_enabled;
	if (mA) {
		regulator_set_current_limit(vbus_draw, 0, 1000 * mA);
		if (!enabled) {
			regulator_enable(vbus_draw);
			eoc_vbus->vbus_draw_enabled = 1;
		}
	} else {
		if (enabled) {
			regulator_disable(vbus_draw);
			eoc_vbus->vbus_draw_enabled = 0;
		}
	}
	eoc_vbus->mA = mA;
}

static int is_vbus_powered(struct eoc_chip *eoc)
{
	int vbus = 0;
	int sense_reg;
	eoc_reg_read(eoc, EOC_REG_SENSE, &sense_reg);
	vbus = !!(sense_reg & EOC_IRQ_VBUS);
	/*should query eoc chip here,let add it later*/
	return vbus;
}

static void eoc_vbus_work(struct work_struct *work)
{
	struct eoc_vbus_data *eoc_vbus =
		container_of(work, struct eoc_vbus_data, work);
	/*
	struct eoc_vbus_mach_info *pdata = eoc_vbus->dev->platform_data;
	*/
	if (!eoc_vbus->otg.gadget)
		return;

	/* Peripheral controllers which manage the pullup themselves won't have
	 * gpio_pullup configured here.  If it's configured here, we'll do what
	 * isp1301_omap::b_peripheral() does and enable the pullup here... although
	 * that may complicate usb_gadget_{,dis}connect() support.
	 */
	if (is_vbus_powered(eoc_vbus->eoc)) {
		eoc_vbus->otg.state = OTG_STATE_B_PERIPHERAL;
		eoc_vbus->mach_switch_mode(EOC_MODE_USB_CLIENT);
		usb_gadget_vbus_connect(eoc_vbus->otg.gadget);

		/* drawing a "unit load" is *always* OK, except for OTG */
		set_vbus_draw(eoc_vbus, 100);

		/* optionally enable D+ pullup */
	} else {
		eoc_vbus->mach_switch_mode(EOC_MODE_NONE);
		set_vbus_draw(eoc_vbus, 0);

		usb_gadget_vbus_disconnect(eoc_vbus->otg.gadget);
		eoc_vbus->otg.state = OTG_STATE_B_IDLE;
	}
}

/* VBUS change IRQ handler */
static irqreturn_t eoc_vbus_irq(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct eoc_vbus_data *eoc_vbus = platform_get_drvdata(pdev);

	if (eoc_vbus->otg.gadget)
		schedule_work(&eoc_vbus->work);

	return IRQ_HANDLED;
}

/* OTG transceiver interface */

/* bind/unbind the peripheral controller */
static int eoc_vbus_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	struct eoc_vbus_data *eoc_vbus;
	struct eoc_vbus_mach_info *pdata;
	struct platform_device *pdev;
	int irq;

	eoc_vbus = container_of(otg, struct eoc_vbus_data, otg);
	pdev = to_platform_device(eoc_vbus->dev);
	pdata = eoc_vbus->dev->platform_data;
	irq = eoc_to_irq(eoc_vbus->eoc, EOC_IRQ_VBUS);

	if (!gadget) {
		dev_dbg(&pdev->dev, "unregistering gadget '%s'\n",
			otg->gadget->name);

		set_vbus_draw(eoc_vbus, 0);

		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_UNDEFINED;

		otg->gadget = NULL;
		return 0;
	}

	otg->gadget = gadget;
	dev_dbg(&pdev->dev, "registered gadget '%s'\n", gadget->name);

	return 0;
}

/* effective for B devices, ignored for A-peripheral */
static int eoc_vbus_set_power(struct otg_transceiver *otg, unsigned mA)
{
	struct eoc_vbus_data *eoc_vbus;

	eoc_vbus = container_of(otg, struct eoc_vbus_data, otg);

	if (otg->state == OTG_STATE_B_PERIPHERAL)
		set_vbus_draw(eoc_vbus, mA);
	return 0;
}

/* for non-OTG B devices: set/clear transceiver suspend mode */
static int eoc_vbus_set_suspend(struct otg_transceiver *otg, int suspend)
{
	struct eoc_vbus_data *eoc_vbus;

	eoc_vbus = container_of(otg, struct eoc_vbus_data, otg);

	/* draw max 0 mA from vbus in suspend mode; or the previously
	 * recorded amount of current if not suspended
	 *
	 * NOTE: high powered configs (mA > 100) may draw up to 2.5 mA
	 * if they're wake-enabled ... we don't handle that yet.
	 */
	return eoc_vbus_set_power(otg, suspend ? 0 : eoc_vbus->mA);
}

/* platform driver interface */

static int __init eoc_vbus_probe(struct platform_device *pdev)
{

	struct eoc_vbus_mach_info *pdata = pdev->dev.platform_data;
	struct eoc_vbus_data *eoc_vbus;
	int err, irq;

	if (!pdata)
		return -EINVAL;

	eoc_vbus = kzalloc(sizeof(struct eoc_vbus_data), GFP_KERNEL);
	if (!eoc_vbus)
		return -ENOMEM;

	platform_set_drvdata(pdev, eoc_vbus);
	eoc_vbus->eoc = dev_get_drvdata(pdev->dev.parent);
	eoc_vbus->dev = &pdev->dev;
	eoc_vbus->otg.label = "eoc-vbus";
	eoc_vbus->otg.state = OTG_STATE_UNDEFINED;
	eoc_vbus->otg.set_peripheral = eoc_vbus_set_peripheral;
	eoc_vbus->otg.set_power = eoc_vbus_set_power;
	eoc_vbus->otg.set_suspend = eoc_vbus_set_suspend;
	eoc_vbus->mach_switch_mode = pdata->mach_switch_mode;

	irq = eoc_to_irq(eoc_vbus->eoc, EOC_IRQ_VBUS);

	err = request_irq(irq, eoc_vbus_irq, 0,
		"vbus_detect", pdev);
	if (err) {
		dev_err(&pdev->dev, "can't request irq %i, err: %d\n",
			irq, err);
		goto err_irq;
	}
	INIT_WORK(&eoc_vbus->work, eoc_vbus_work);

	/* only active when a gadget is registered */
	err = otg_set_transceiver(&eoc_vbus->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		goto err_otg;
	}

	eoc_vbus->vbus_draw = regulator_get(&pdev->dev, "vbus_draw");
	if (IS_ERR(eoc_vbus->vbus_draw)) {
		dev_dbg(&pdev->dev, "can't get vbus_draw regulator, err: %ld\n",
			PTR_ERR(eoc_vbus->vbus_draw));
		eoc_vbus->vbus_draw = NULL;
	}

	return 0;
err_otg:
	free_irq(irq, &pdev->dev);
err_irq:
	platform_set_drvdata(pdev, NULL);
	return err;
}

static int __exit eoc_vbus_remove(struct platform_device *pdev)
{
	struct eoc_vbus_data *eoc_vbus = platform_get_drvdata(pdev);
	/*
	struct eoc_vbus_mach_info *pdata = pdev->dev.platform_data;
	*/
	regulator_put(eoc_vbus->vbus_draw);

	otg_set_transceiver(NULL);

	free_irq(eoc_to_irq(eoc_vbus->eoc ,EOC_IRQ_VBUS), &pdev->dev);
	platform_set_drvdata(pdev, NULL);
	kfree(eoc_vbus);

	return 0;
}

/* NOTE:  the gpio-vbus device may *NOT* be hotplugged */

MODULE_ALIAS("platform:eoc-vbus");

static struct platform_driver eoc_vbus_driver = {
	.driver = {
		.name  = "eoc-vbus",
		.owner = THIS_MODULE,
	},
	.remove  = __exit_p(eoc_vbus_remove),
};

static int __init eoc_vbus_init(void)
{
	return platform_driver_probe(&eoc_vbus_driver, eoc_vbus_probe);
}
subsys_initcall(eoc_vbus_init);

static void __exit eoc_vbus_exit(void)
{
	platform_driver_unregister(&eoc_vbus_driver);
}
module_exit(eoc_vbus_exit);

MODULE_DESCRIPTION("EOC controlled OTG transceiver driver");
MODULE_AUTHOR("guiming zhuo");
MODULE_LICENSE("GPL");

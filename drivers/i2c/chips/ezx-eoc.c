/*
 * EOC USB Transceiver driver for EZX Phones
 *
 * Copyright (C) 2007 Alex Zhang <celeber2@gmail.com>
 * Copyright (C) 2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/i2c/ezx-eoc.h>
#include <linux/usb.h>

#include <linux/regulator/consumer.h>

#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

struct eoc_vbus_data {
	struct otg_transceiver otg;
	struct device          *dev;
	struct regulator       *vbus_draw;
	int			vbus_draw_enabled;
	unsigned		mA;
	struct work_struct	work;
};


static int sense_reg, power0_reg;
static int eoc_charger_enable(struct regulator_dev *rdev);

static struct i2c_client *eoc_i2c_client;
static const struct i2c_device_id eoc_id[] = {
	{ "ezx-eoc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, eoc_id);
struct work_struct work;

int eoc_reg_read(char reg, unsigned int *val)
{
	char value[EOC_REG_DATA_SIZE];

	struct i2c_msg msgs[2] = {
		{ eoc_i2c_client->addr, 0, EOC_REG_ADDR_SIZE, &reg },
		{ eoc_i2c_client->addr, I2C_M_RD, EOC_REG_DATA_SIZE, value },
	};
	if (i2c_transfer(eoc_i2c_client->adapter, msgs, 2) != 2)
		return -EIO;
	*val = (value[2]);
	*val |= (value[1] << 8);
	*val |= (value[0] << 16);

	return 0;
}

int eoc_reg_write(char reg, unsigned int val)
{
	char value[EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE];

	value[0] = reg;
	value[1] = (char)(val >> 16);
	value[2] = (char)(val >> 8);
	value[3] = (char)val;
	if(i2c_master_send(eoc_i2c_client, value, EOC_REG_ADDR_SIZE +
		EOC_REG_DATA_SIZE) != (EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE))
		return -EIO;
	return 0;
}

int eoc_reg_write_mask(char reg, int mask, int value)
{
	unsigned int old_value;
	int ret;

	ret = eoc_reg_read(reg, &old_value);
	if (!ret)
		return ret;

	old_value &= ~mask;
	old_value |= value & mask;
	ret = eoc_reg_write(reg, old_value);
	if (!ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(eoc_reg_write_mask);

static void eoc_work(struct work_struct *_eoc)
{
	unsigned int isr, msr, i, x;
	struct eoc_vbus_data *eoc_vbus;
	eoc_vbus = i2c_get_clientdata(eoc_i2c_client);

	eoc_reg_read(EOC_REG_ISR, &isr);
	eoc_reg_read(EOC_REG_MSR, &msr);
	eoc_reg_read(EOC_REG_SENSE, &sense_reg);
	printk(KERN_INFO "Sens: %x\n", sense_reg);

	printk(KERN_INFO "EOC INTS: \n");
	for (i = (isr & ~msr), x = 0; i; x++) {
		if (!(i & (1 << x)))
			continue;
		i &= ~(1 << x);

		printk(KERN_INFO "bit %x ", (1 << x) & sense_reg);

		switch (1 << x) {
		case EOC_IRQ_VBUS_3V4:
			printk("VBUS_3V4 ");
			break;
		case EOC_IRQ_VBUS:

			printk("cable: %s\n",
				(sense_reg & EOC_IRQ_VBUS) ?
				"connected" : "disconnected"
			);
			if (eoc_vbus->otg.gadget) {
				if (sense_reg & EOC_IRQ_VBUS)
					usb_gadget_vbus_connect(
						eoc_vbus->otg.gadget);
				else
					usb_gadget_vbus_disconnect(
						eoc_vbus->otg.gadget);
			}
			break;
		case EOC_IRQ_VBUS_OV:
			printk("VBUS_OV ");
			break;
		case EOC_IRQ_RVRS_CHRG:
			printk("RVRS_CHRG ");
			break;
		case EOC_IRQ_ID:
			printk("ID ");
			break;
		case EOC_IRQ_ID_GROUND:
			printk("ID_GROUND ");
			break;
		case EOC_IRQ_SE1:
			printk("SE1 ");
			break;
		case EOC_IRQ_CC_CV:
			printk("CC_CV ");
			break;
		case EOC_IRQ_CHRG_CURR:
			printk("CHRG_CURR ");
			break;
		case EOC_IRQ_RVRS_CURR:
			printk("RVRS_CURR ");
			break;
		case EOC_IRQ_CK:
			printk("CK ");
			break;
		case EOC_IRQ_BATTPON:
			printk("BATTPON ");
			break;
		}

		printk("\n");
	}

	eoc_reg_write(EOC_REG_ISR, isr);
	eoc_reg_write(EOC_REG_POWER_CONTROL_0, power0_reg);
	eoc_reg_read(EOC_REG_POWER_CONTROL_0, &power0_reg);

	if (gpio_get_value(10))
		schedule_work(&work);

}

static irqreturn_t eoc_irq(int irq, void *arg)
{
	schedule_work(&work);
	return IRQ_HANDLED;
}

int eoc_cable_connected(int mask)
{

	if (!(sense_reg & EOC_IRQ_VBUS))
		return 0;

	return sense_reg & mask;

}

static int eoc_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	int mask;
	int charge_current, setup;
	if (max_uA < 100000)
		charge_current = 1;
	else if (max_uA > 1300000)
		charge_current = 14;
	else
		charge_current = max_uA / 100000;

	setup = (charge_current << EMU_ICHRG_SHIFT) & EMU_ICHRG_MASK;
	if (charge_current != 0)
		mask = EMU_ICHRG_MASK | EMU_ICHRG_TR_MASK;
	else
		mask = EMU_ICHRG_MASK;

	power0_reg &= ~mask;
	power0_reg |= setup & mask;

	schedule_work(&work);

	return 0;
}

static int eoc_get_current_limit(struct regulator_dev *rdev)
{
	int value = power0_reg;

	value &= EMU_ICHRG_MASK;
	value >>= EMU_ICHRG_SHIFT;
	return value * 1000;
}

static int eoc_charger_enable(struct regulator_dev *rdev)
{
	power0_reg &= ~EMU_VCHRG_MASK;
	power0_reg |= EMU_VCHRG_MASK & 7;

	printk(KERN_INFO "enable charger %x\n", power0_reg);
	schedule_work(&work);

	return 0;
}

static int eoc_charger_disable(struct regulator_dev *rdev)
{

	power0_reg &= ~EMU_VCHRG_MASK;
	schedule_work(&work);

	return 0;
}

static int eoc_charger_is_enabled(struct regulator_dev *rdev)
{
	int value = power0_reg;

	value &= EMU_VCHRG_MASK;

	return value >= 3;
}

static struct regulator_ops eoc_regulator_ops = {
	.set_current_limit = eoc_set_current_limit,
	.get_current_limit = eoc_get_current_limit,
	.enable            = eoc_charger_enable,
	.disable           = eoc_charger_disable,
	.is_enabled        = eoc_charger_is_enabled,
};

static struct regulator_desc eoc_regulator_desc = {
	.name  = "eoc_charger",
	.ops   = &eoc_regulator_ops,
	.type  = REGULATOR_CURRENT,
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

/* bind/unbind the peripheral controller */
static int eoc_vbus_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	struct eoc_vbus_data *eoc_vbus;
	eoc_vbus = container_of(otg, struct eoc_vbus_data, otg);

	if (!gadget) {
		set_vbus_draw(eoc_vbus, 0);

		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_UNDEFINED;

		otg->gadget = NULL;
		return 0;
	}

	otg->gadget = gadget;
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

static int eoc_vbus_set_suspend(struct otg_transceiver *otg, int suspend)
{
	struct eoc_vbus_data *eoc_vbus;

	eoc_vbus = container_of(otg, struct eoc_vbus_data, otg);

	return eoc_vbus_set_power(otg, suspend ? 0 : eoc_vbus->mA);
}


static int __devinit eoc_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int tmp, x, ret;
	struct eoc_vbus_data *eoc_vbus;

	eoc_i2c_client = client;

	INIT_WORK(&work, eoc_work);

	ret = eoc_reg_write(EOC_REG_MSR, 0x0); //fef);
	if (ret)
		goto ret;

	power0_reg = 0x400; /* software controlled, dual path */
	eoc_reg_write(EOC_REG_POWER_CONTROL_0, power0_reg);
	if (ret)
		goto ret;
	eoc_reg_write(EOC_REG_POWER_CONTROL_1, 0xc);
	if (ret)
		goto ret;
	eoc_reg_write(EOC_REG_CONN_CONTROL, 0x21044);
	if (ret)
		goto ret;
	for (x = EOC_REG_MSR; x <= EOC_REG_CONN_CONTROL; x++) {
		ret = eoc_reg_read(x, &tmp);
		if (ret)
			goto ret;
	}

	eoc_vbus = kzalloc(sizeof(struct eoc_vbus_data), GFP_KERNEL);
	if (!eoc_vbus)
		return -ENOMEM;
	eoc_vbus->dev = &client->dev;
	eoc_vbus->otg.label = "eoc-vbus";
	eoc_vbus->otg.state = OTG_STATE_UNDEFINED;
	eoc_vbus->otg.set_peripheral = eoc_vbus_set_peripheral;
	eoc_vbus->otg.set_power = eoc_vbus_set_power;
	eoc_vbus->otg.set_suspend = eoc_vbus_set_suspend;

	i2c_set_clientdata(client, eoc_vbus);
	otg_set_transceiver(&eoc_vbus->otg);

	ret = gpio_request(10, "EOC");
	if (ret)
		goto ret;
	gpio_direction_input(10);
	ret = request_irq(gpio_to_irq(10), eoc_irq, IRQF_TRIGGER_RISING,
								"EOC", NULL);
	eoc_reg_write(EOC_REG_ISR, 0xffffff);
	eoc_reg_read(EOC_REG_SENSE, &sense_reg);

ret:
	return ret;
}

static int __devexit eoc_remove(struct i2c_client *client)
{
	free_irq(gpio_to_irq(10), NULL);
	flush_scheduled_work();
	kfree(i2c_get_clientdata(client));

	return 0;
}

static struct i2c_driver eoc_driver = {
	.driver = {
		.name = "ezx-eoc",
		.owner = THIS_MODULE,
	},
	.probe = eoc_probe,
	.remove = __devexit_p(eoc_remove),
	.id_table = eoc_id,
};



static int eoc_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
#ifdef CONFIG_REGULATOR
	/* register regulator */
	rdev = regulator_register(&eoc_regulator_desc, &pdev->dev,
			pdev->dev.platform_data, NULL);
	if (IS_ERR(rdev)) {
		return PTR_ERR(rdev);
	}
#endif

	return 0;
}

static int eoc_regulator_remove(struct platform_device *pdev)
{
#ifdef CONFIG_REGULATOR
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
#endif
	return 0;
}


static struct platform_driver eoc_regulator_driver = {
	.probe = eoc_regulator_probe,
	.remove = eoc_regulator_remove,
	.driver		= {
		.name	= "eoc-regulator",
	},
};


static int __init eoc_init(void)
{
	platform_driver_probe(&eoc_regulator_driver, eoc_regulator_probe);
	return i2c_add_driver(&eoc_driver);
}

static void __exit eoc_exit(void)
{
	i2c_del_driver(&eoc_driver);
	platform_driver_unregister(&eoc_regulator_driver);
}

MODULE_AUTHOR("Alex Zhang <celeber2@gmail.com>");
MODULE_DESCRIPTION("ezx-eoc usb tranceiver");
MODULE_LICENSE("GPL");

subsys_initcall(eoc_init);
module_exit(eoc_exit);

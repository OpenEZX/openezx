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

#define EOC_REG_ADDR_SIZE		1
#define EOC_REG_DATA_SIZE		3
#define EOC_REG_INT_STATUS		32
#define EOC_REG_INT_MASK		33
#define EOC_REG_INT_SENSE		34
#define EOC_REG_POWER_CONTROL_0		35
#define EOC_REG_POWER_CONTROL_1		36
#define EOC_REG_CONN_CONTROL		37

#define EOC_INT_VBUS_3V4		(1 << 0)
#define EOC_INT_VBUS			(1 << 1)
#define EOC_INT_VBUS_OV			(1 << 2)
#define EOC_INT_RVRS_CHRG		(1 << 3)
#define EOC_INT_ID			(1 << 4)
#define EOC_INT_ID_GROUND		(1 << 5)
#define EOC_INT_SE1			(1 << 6)
#define EOC_INT_CC_CV			(1 << 7)
#define EOC_INT_CHRG_CURR		(1 << 8)
#define EOC_INT_RVRS_CURR		(1 << 9)
#define EOC_INT_CK			(1 << 10)
#define EOC_INT_BATTPON			(1 << 11)

/* 21044  2, 6(70K_PD), 12(XCVR), 17(MODE(3)) */
#define EOC_CONN_USB_SUSPEND		(1 << 1)
#define EOC_CONN_DPLUS_150K_PU		(1 << 5)
#define EOC_CONN_VBUS_70K_PD		(1 << 6)
#define EOC_CONN_XCVR			(1 << 12)
#define EOC_CONN_MODE(x)		((x & 7) << 14)
#define EOC_CONN_ID_PD			(1 << 20)
#define EOC_CONN_ID_PU			(1 << 22)

/* c 2, 3 */
#define EOC_POWER1_INPUT_SOURCE(x)	((x & 3) << 0)
#define EOC_POWER1_OUTPUT_VOLTAGE	(1 << 2)
#define EOC_POWER1_VUSB			(1 << 3)

/* c00 10, 11 */
#define EOC_POWER0_VBUS_5K_PD		(1 << 19)
#define EOC_POWER0_REVERSE_MODE		(1 << 13)

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

	printk(KERN_INFO "EOC: read %d: %08x\n", reg, *val);
	return 0;
}

int eoc_reg_write(char reg, unsigned int val)
{
	char value[EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE];

	value[0] = reg;
	value[1] = (char)(val >> 16);
	value[2] = (char)(val >> 8);
	value[3] = (char)val;
	printk(KERN_INFO "EOC: write %d: %08x\n", reg, val);
	if(i2c_master_send(eoc_i2c_client, value, EOC_REG_ADDR_SIZE +
		EOC_REG_DATA_SIZE) != (EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE))
		return -EIO;
	return 0;
}

static void eoc_work(struct work_struct *_eoc)
{
	unsigned int isr, msr, i, x;
	eoc_reg_read(EOC_REG_INT_STATUS, &isr);
	eoc_reg_read(EOC_REG_INT_MASK, &msr);
	printk(KERN_INFO "EOC INTS: ");
	for (i = (isr & ~msr), x = 0; i; x++) {
		if (!(i & (1 << x)))
			continue;
		i &= ~(1 << x);
		switch (1 << x) {
		case EOC_INT_VBUS_3V4:
			printk("VBUS_3V4 ");
			break;
		case EOC_INT_VBUS:
			printk("VBUS ");
			break;
		case EOC_INT_VBUS_OV:
			printk("VBUS_OV ");
			break;
		case EOC_INT_RVRS_CHRG:
			printk("RVRS_CHRG ");
			break;
		case EOC_INT_ID:
			printk("ID ");
			break;
		case EOC_INT_ID_GROUND:
			printk("ID_GROUND ");
			break;
		case EOC_INT_SE1:
			printk("SE1 ");
			break;
		case EOC_INT_CC_CV:
			printk("CC_CV ");
			break;
		case EOC_INT_CHRG_CURR:
			printk("CHRG_CURR ");
			break;
		case EOC_INT_RVRS_CURR:
			printk("RVRS_CURR ");
			break;
		case EOC_INT_CK:
			printk("CK ");
			break;
		case EOC_INT_BATTPON:
			printk("BATTPON ");
			break;
		}
	}

	printk("\n");
	eoc_reg_write(EOC_REG_INT_STATUS, isr);
}

static irqreturn_t eoc_irq(int irq, void *arg)
{
	schedule_work(&work);
	return IRQ_HANDLED;
}

static int __devinit eoc_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int tmp, x, ret;

	eoc_i2c_client = client;
	INIT_WORK(&work, eoc_work);

	ret = eoc_reg_write(EOC_REG_INT_MASK, 0x0); //fef);
	if (ret)
		goto ret;
	eoc_reg_write(EOC_REG_POWER_CONTROL_0, 0xc00);
	if (ret)
		goto ret;
	eoc_reg_write(EOC_REG_POWER_CONTROL_1, 0xc);
	if (ret)
		goto ret;
	eoc_reg_write(EOC_REG_CONN_CONTROL, 0x21044);
	if (ret)
		goto ret;
	for (x = EOC_REG_INT_MASK; x <= EOC_REG_CONN_CONTROL; x++) {
		ret = eoc_reg_read(x, &tmp);
		if (ret)
			goto ret;
	}
	ret = gpio_request(10, "EOC");
	if (ret)
		goto ret;
	gpio_direction_input(10);
	ret = request_irq(gpio_to_irq(10), eoc_irq, IRQF_TRIGGER_RISING,
								"EOC", NULL);
	eoc_reg_write(EOC_REG_INT_STATUS, 0xffffff);
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

static int __init eoc_init(void)
{
	return i2c_add_driver(&eoc_driver);
}

static void __exit eoc_exit(void)
{
	i2c_del_driver(&eoc_driver);
}

MODULE_AUTHOR("Alex Zhang <celeber2@gmail.com>");
MODULE_DESCRIPTION("ezx-eoc usb tranceiver");
MODULE_LICENSE("GPL");

module_init(eoc_init);
module_exit(eoc_exit);

/*
 * EOC USB Transceiver driver for EZX Phones
 *
 * Copyright (C) 2007 Alex Zhang <celeber2@gmail.com>
 * Copyright (C) 2008 Daniel Ribeiro <drwyrm@gmail.com>
 * Copyright (C) 2009 Ilya Petrov <ilya.muromec@gmail.com>
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
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/mfd/ezx-eoc.h>

static const struct i2c_device_id eoc_id[] = {
	{ "ezx-eoc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, eoc_id);



int eoc_reg_read(struct eoc_chip *eoc, char reg, unsigned int *val)
{
	char value[EOC_REG_DATA_SIZE];

	struct i2c_msg msgs[2] = {
		{ eoc->client->addr, 0, EOC_REG_ADDR_SIZE, &reg },
		{ eoc->client->addr, I2C_M_RD, EOC_REG_DATA_SIZE, value },
	};
	if (i2c_transfer(eoc->client->adapter, msgs, 2) != 2)
		return -EIO;
	*val = (value[2]);
	*val |= (value[1] << 8);
	*val |= (value[0] << 16);

	return 0;
}
EXPORT_SYMBOL_GPL(eoc_reg_read);

int eoc_reg_write(struct eoc_chip *eoc, char reg, unsigned int val)
{
	char value[EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE];

	value[0] = reg;
	value[1] = (char)(val >> 16);
	value[2] = (char)(val >> 8);
	value[3] = (char)val;
	if(i2c_master_send(eoc->client, value, EOC_REG_ADDR_SIZE +
		EOC_REG_DATA_SIZE) != (EOC_REG_ADDR_SIZE + EOC_REG_DATA_SIZE))
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(eoc_reg_write);

int eoc_reg_write_mask(struct eoc_chip *eoc, char reg, int mask, int value)
{
	unsigned int old_value;
	int ret;

	ret = eoc_reg_read(eoc, reg, &old_value);
	if (!ret)
		return ret;

	old_value &= ~mask;
	old_value |= value & mask;
	ret = eoc_reg_write(eoc, reg, old_value);
	if (!ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(eoc_reg_write_mask);

int eoc_switch_mode(struct eoc_chip *eoc, enum eoc_transceiver_mode mode)
{
	if (eoc->mach_switch_mode)
		eoc->mach_switch_mode(mode);

	switch (mode) {
	case EOC_MODE_NONE:
	case EOC_MODE_USB_CLIENT:
	case EOC_MODE_USB_HOST:
	case EOC_MODE_UART:
		break;
	}
}
EXPORT_SYMBOL_GPL(eoc_switch_mode);

static void eoc_isr_work(struct work_struct *work)
{

	struct eoc_chip *eoc = container_of(work, struct eoc_chip, isr_work);

	unsigned int isr, msr, i, x;
	int irq;
	eoc_reg_read(eoc, EOC_REG_ISR, &isr);
	eoc_reg_read(eoc ,EOC_REG_MSR, &msr);
	eoc_reg_read(eoc ,EOC_REG_SENSE, &eoc->sense);

	local_irq_disable();
	for (i = (isr & ~msr), x = 0; i; x++) {
		if (!(i & (1 << x)))
			continue;
		i &= ~(1 << x);

		irq = eoc->irq_base + x;

		struct irq_desc *desc = irq_to_desc(irq);

		if (WARN(!desc, KERN_WARNING
					"Invalid EOC IRQ %d\n", irq))
			break;

		if (desc->status & IRQ_DISABLED)
			note_interrupt(irq, desc, IRQ_NONE);
		else
			desc->handle_irq(irq, desc);

	}
	local_irq_enable();

	eoc_reg_write(eoc, EOC_REG_ISR, isr);

	if (gpio_get_value(10))
		schedule_work(&eoc->isr_work);

}

static void eoc_msr_work(struct work_struct *work)
{

	struct eoc_chip *eoc = container_of(work, struct eoc_chip, msr_work);

	eoc_reg_write(eoc, EOC_REG_MSR, eoc->msr);
}

static void eoc_power0_work(struct work_struct *work)
{

	struct eoc_chip *eoc = container_of(work, struct eoc_chip, power0_work);

	int write = eoc->power0;
	eoc_reg_write(eoc, EOC_REG_POWER_CONTROL_0, eoc->power0);
	eoc_reg_read(eoc, EOC_REG_POWER_CONTROL_0, &eoc->power0);

	printk("power0: write %x, read %x\n",
			write, eoc->power0 );
}

static irqreturn_t eoc_irq(int irq, void *_eoc)
{
	struct eoc_chip *eoc = _eoc;
	schedule_work(&eoc->isr_work);

	return IRQ_HANDLED;
}

static struct platform_device eoc_vbus = {
	.name = "eoc-vbus",
	.id   = -1,
};

static struct platform_device *eoc_sub_devices[] = {
	&eoc_vbus,
};

static inline struct device *add_child(struct eoc_chip *eoc, const char *name,
		int id,
		void *pdata, unsigned pdata_len,
		bool can_wakeup)
{

	struct platform_device	*pdev;
	int			status;

	pdev = platform_device_alloc(name, id);
	if (!pdev) {
		dev_dbg(&eoc->client->dev, "can't alloc dev\n");
		status = -ENOMEM;
		goto err;
	}

	device_init_wakeup(&pdev->dev, can_wakeup);
	pdev->dev.parent = &eoc->client->dev;

	if (pdata) {
		status = platform_device_add_data(pdev, pdata, pdata_len);
		if (status < 0) {
			dev_dbg(&pdev->dev, "can't add platform_data\n");
			goto err;
		}
	}

	platform_set_drvdata(pdev, eoc);

	status = platform_device_add(pdev);

err:
	if (status < 0) {
		platform_device_put(pdev);
		dev_err(&eoc->client->dev, "can't add %s dev\n", name);
		return ERR_PTR(status);
	}
	return &pdev->dev;

}

static int
add_children(struct eoc_chip *eoc)
{

	struct device   *charger;

	charger = add_child(eoc, "eoc_charger", -1, NULL, 0, false);

	if (IS_ERR(charger))
		return PTR_ERR(charger);

	static struct regulator_consumer_supply charge_consumer_c[] = {
		[0] = {
			.supply = "ac_draw",
		},
		[1] = {
			.supply = "vbus_draw",
		},
	};

	static struct regulator_consumer_supply charge_consumer_v = {
		.supply = "ac_voltage", 
	};


	charge_consumer_c[0].dev = charger;
	charge_consumer_c[0].dev = &eoc_vbus.dev;
	charge_consumer_v.dev = charger;
	
	struct regulator_init_data charge_data = {
		.num_consumer_supplies = 2,
		.consumer_supplies = charge_consumer_c,
                .constraints = {
			.max_uA = 1300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_CURRENT
			| REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_DRMS
		},
	};

	struct regulator_init_data charge_data_voltage = {
		.num_consumer_supplies = 1,
		.consumer_supplies = &charge_consumer_v,
                .constraints = {
			.max_uV = 4500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_DRMS
		},
	};


	add_child(eoc, "eoc_reg", 0, &charge_data,
			sizeof(charge_data), false);

        add_child(eoc, "eoc_reg", 1, &charge_data_voltage,
			sizeof(charge_data_voltage), false);

	platform_add_devices(eoc_sub_devices, 1);
	return 0;
}

static int __devinit eoc_add_subdev(struct eoc_chip *eoc,
						struct eoc_subdev *subdev)
{
	struct platform_device *pdev;

	pdev = platform_device_alloc(subdev->name, subdev->id);
	pdev->dev.parent = &eoc->client->dev;
	pdev->dev.platform_data = subdev->platform_data;
	platform_set_drvdata(pdev, eoc);

	return platform_device_add(pdev);
}

/* subdevs */
static int eoc_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}



/* IRQ */
int irq_to_eoc(struct eoc_chip *eoc, int irq)
{
	return irq - eoc->irq_base;
}
EXPORT_SYMBOL_GPL(irq_to_eoc);

int eoc_to_irq(struct eoc_chip *eoc, int irq)
{
	return eoc->irq_base + irq;
}
EXPORT_SYMBOL_GPL(eoc_to_irq);

static void eoc_mask_irq(unsigned int irq)
{
	struct eoc_chip *eoc = get_irq_chip_data(irq);

	eoc->msr |= 1 << irq_to_eoc(eoc, irq);
	schedule_work(&eoc->msr_work);
}

static void eoc_unmask_irq(unsigned int irq)
{
	struct eoc_chip *eoc = get_irq_chip_data(irq);

	eoc->msr &= ~(1 << irq_to_eoc(eoc, irq));
	schedule_work(&eoc->msr_work);
}

static struct irq_chip eoc_irq_chip = {
	.name	= "pcap",
	.mask	= eoc_mask_irq,
	.unmask	= eoc_unmask_irq,
};

static int __devinit eoc_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int tmp, x, ret;
	int i;
        struct eoc_platform_data *pdata = client->dev.platform_data;
	struct eoc_chip *eoc;

	eoc = kzalloc(sizeof(*eoc), GFP_KERNEL);
	if (!eoc) {
		ret = -ENOMEM;
		goto ret;
	}

	eoc->client = client;
	INIT_WORK(&eoc->isr_work, eoc_isr_work);
	INIT_WORK(&eoc->msr_work, eoc_msr_work);
	INIT_WORK(&eoc->power0_work, eoc_power0_work);

	eoc->msr = 0xFFF;
	ret = eoc_reg_write(eoc, EOC_REG_MSR, eoc->msr);
	if (ret)
		goto ret;

	eoc->power0 = 0x400;
	eoc_reg_write(eoc, EOC_REG_POWER_CONTROL_0, eoc->power0);
	if (ret)
		goto ret;
	eoc_reg_write(eoc, EOC_REG_POWER_CONTROL_1, 0xc);
	if (ret)
		goto ret;
	eoc_reg_write(eoc, EOC_REG_CONN_CONTROL, 0x21044);
	if (ret)
		goto ret;
	for (x = EOC_REG_MSR; x <= EOC_REG_CONN_CONTROL; x++) {
		ret = eoc_reg_read(eoc, x, &tmp);
		if (ret)
			goto ret;
	}
	ret = gpio_request(10, "EOC");
	if (ret)
		goto ret;
	gpio_direction_input(10);
	ret = request_irq(gpio_to_irq(10), eoc_irq, IRQF_TRIGGER_RISING,
		"EOC", eoc);

	eoc_reg_write(eoc, EOC_REG_ISR, 0xffffff);
	eoc_reg_read(eoc, EOC_REG_SENSE, &eoc->sense);

	eoc->irq_base = pdata->irq_base;
	printk("eoc irq base: %d\n",eoc->irq_base);

	/* setup irq chip */
	for (x = eoc->irq_base; x < (eoc->irq_base + EOC_NIRQS); x++) {
		set_irq_chip_and_handler(x, &eoc_irq_chip, handle_simple_irq);

		set_irq_chip_data(x, eoc);
#ifdef CONFIG_ARM
		set_irq_flags(x, IRQF_VALID);
#else
		set_irq_noprobe(x);
#endif
	}

        ret = add_children(eoc);
	if (ret)
		goto remove_subdevs;
	/* setup subdevs */
	for (i = 0; i < pdata->num_subdevs; i++) {
		ret = eoc_add_subdev(eoc, &pdata->subdevs[i]);
		if (ret)
			goto remove_subdevs;
	}
	return 0;
remove_subdevs:
	device_for_each_child(&client->dev, NULL, eoc_remove_subdev);
	for (i = eoc->irq_base; i < (eoc->irq_base + EOC_NIRQS); i++)
		set_irq_chip_and_handler(i, NULL, NULL);
	kfree(eoc);
ret:
	return ret;
}

static int __devexit eoc_remove(struct i2c_client *client)
{
	/* remove all registered subdevs */
	device_for_each_child(&client->dev, NULL, eoc_remove_subdev);

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

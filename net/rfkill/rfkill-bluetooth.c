/*
 * Regulator consumer driver for bluetooth rfkill
 *
 * Copyright (C) 2009 gmzhuo <gmzhuo@gmail.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/rfkill.h>
#include <linux/regulator/consumer.h>

struct rfkill_bluetooth_info {
	struct rfkill *rf_kill;
	char rf_kill_name[60];
	struct regulator *vcc;
};
static struct rfkill_bluetooth_info rfkill_info;

static int reg_bt_set_block(void *data, bool blocked)
{
	struct rfkill_bluetooth_info *pinfo = data;
	if (blocked)
		regulator_disable(pinfo->vcc);
	else
		regulator_enable(pinfo->vcc);
	return 0;
}

struct rfkill_ops reg_bt_ops = {
	.set_block = reg_bt_set_block,
};

static int __devinit reg_rfkill_probe(struct platform_device *pdev)
{
	int ret;
	struct regulator *reg = regulator_get(&pdev->dev, "vbluetooth");
	if (!reg)
		return -ENODEV;
	rfkill_info.vcc = reg;

	rfkill_info.rf_kill = rfkill_alloc("bluetooth:rfkill", &pdev->dev,
					   RFKILL_TYPE_BLUETOOTH,
					   &reg_bt_ops, &rfkill_info);
	if (!rfkill_info.rf_kill) {
		regulator_put(reg);
		return -ENOMEM;
	}

	ret = rfkill_register(rfkill_info.rf_kill);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, &rfkill_info);

	return 0;
}

static int __devexit reg_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill_bluetooth_info *pinfo = platform_get_drvdata(pdev);
	struct rfkill *rf_kill = pinfo->rf_kill;
	rfkill_unregister(rf_kill);
	regulator_put(pinfo->vcc);
	pinfo->vcc = 0;
	return 0;
}

static struct platform_driver bluetooth_rfkill_driver = {
	.probe = reg_rfkill_probe,
	.remove = __devexit_p(reg_rfkill_remove),
	.driver = {
		   .name = "rfkill_bluetooth",
		   .owner = THIS_MODULE,
		   },
};

static int __init reg_bluetooth_rfkill_init(void)
{
	return platform_driver_register(&bluetooth_rfkill_driver);
}

static void __exit reg_bluetooth_rfkill_exit(void)
{
	platform_driver_unregister(&bluetooth_rfkill_driver);
}

module_init(reg_bluetooth_rfkill_init);
module_exit(reg_bluetooth_rfkill_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("gm zhuo");
MODULE_DESCRIPTION("regulator rfkill for bluetooth");

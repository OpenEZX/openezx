/*
 * Driver for Motorola PCAP2 as present in EZX phones
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

static int reg_bt_toggle_radio(void *data, enum rfkill_state state)
{

	struct rfkill_bluetooth_info *pinfo = data;
	switch (state) {
	case RFKILL_STATE_SOFT_BLOCKED:
		regulator_disable(pinfo->vcc);
		return 0;
	case RFKILL_STATE_UNBLOCKED:
		regulator_enable(pinfo->vcc);
		return 0;
	default:
		return -EINVAL;
	}
}

static int reg_bt_getstate(void *data, enum rfkill_state *state)
{
	struct rfkill_bluetooth_info *pinfo = data;
	if (regulator_is_enabled(pinfo->vcc))
		*state = RFKILL_STATE_UNBLOCKED;
	else
		*state = RFKILL_STATE_SOFT_BLOCKED;

	return 0;
}

static int __devinit reg_rfkill_probe(struct platform_device *pdev)
{
	struct regulator *reg = regulator_get(&pdev->dev, "vbluetooth");
	if (!reg)
		return -ENODEV;
	rfkill_info.vcc = reg;

	rfkill_info.rf_kill = rfkill_allocate(&pdev->dev,
					      RFKILL_TYPE_BLUETOOTH);
	if (!rfkill_info.rf_kill) {
		regulator_put(reg);
		return -ENOMEM;
	}

	snprintf(rfkill_info.rf_kill_name, sizeof(rfkill_info.rf_kill_name),
		 "bluetooth:rfkill");
	rfkill_info.rf_kill->name = rfkill_info.rf_kill_name;
	rfkill_info.rf_kill->data = &rfkill_info;
	rfkill_info.rf_kill->toggle_radio = reg_bt_toggle_radio;
	rfkill_info.rf_kill->get_state = reg_bt_getstate;
	rfkill_info.rf_kill->state = RFKILL_STATE_SOFT_BLOCKED;
	rfkill_info.rf_kill->user_claim_unsupported = 1;

	rfkill_register(rfkill_info.rf_kill);

	return 0;
}

static int __devexit reg_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rf_kill = platform_get_drvdata(pdev);
	struct rfkill_bluetooth_info *pinfo;
	pinfo = rf_kill->data;
	regulator_put(pinfo->vcc);
	rfkill_unregister(rf_kill);

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

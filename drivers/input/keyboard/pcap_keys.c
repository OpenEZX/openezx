/*
 *  Input driver for PCAP events:
 *   * Power key
 *   * Jack plug/unplug
 *
 *  Copyright (c) 2008,2009 Ilya Petrov <ilya.muromec@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/ezx-pcap.h>

static struct input_dev *pcap_input;

/* PCAP2 interrupts us on keypress */
static irqreturn_t pcap_pwrkey_event(u32 event, void *unused)
{
	u32 pstat;
	ezx_pcap_read(PCAP_REG_PSTAT, &pstat);
	pstat &= PCAP_IRQ_ONOFF;

	input_report_key(pcap_input, KEY_POWER, (pstat ? 0 : 1));
	input_sync(pcap_input);

	return IRQ_HANDLED;
}

/* PCAP2 interrupts us on plug/unplug */
static irqreturn_t pcap_jack_event(u32 event, void *unused)
{
	u32 pstat;
	ezx_pcap_read(PCAP_REG_PSTAT, &pstat);
	pstat &= PCAP_IRQ_A1 ;

	input_report_switch(pcap_input, SW_HEADPHONE_INSERT, (pstat  ? 0 : 1));
	input_sync(pcap_input);

	return IRQ_HANDLED;
}

static int __init pcap_keys_probe(struct platform_device *pdev)
{
	int err;

	ezx_pcap_register_event(PCAP_IRQ_ONOFF,
			pcap_pwrkey_event, NULL, "Power key");
	ezx_pcap_register_event(PCAP_IRQ_A1, pcap_jack_event, NULL, "HP/MIC");

	pcap_input = input_allocate_device();
	if (!pcap_input) {
		printk(KERN_ERR "pcap_keys: No mem\n");
		err = -ENOMEM;
		goto fail_irq;
	}

	pcap_input->name = pdev->name;
	pcap_input->phys = "pcap-keys/input0";
	pcap_input->dev.parent = &pdev->dev;

	printk(KERN_INFO "pcap events initializing\n");
	pcap_input->evbit[0] = BIT_MASK(EV_KEY)  | BIT_MASK(EV_SW);
	pcap_input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	set_bit(SW_HEADPHONE_INSERT, pcap_input->swbit);

	err =  input_register_device(pcap_input);
	if (err) {
		printk(KERN_ERR "pcap_keys: Failed to register device\n");
		goto fail_dev;
	}
	return 0;

fail_dev:
	input_free_device(pcap_input);

fail_irq:
	ezx_pcap_unregister_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1 | PCAP_IRQ_ONOFF);

	return err;
}

static int pcap_keys_remove(struct platform_device *pdev)
{
	ezx_pcap_unregister_event(PCAP_IRQ_MB2 | PCAP_IRQ_A1 | PCAP_IRQ_ONOFF);

	input_unregister_device(pcap_input);

	return 0;
}

static struct platform_driver pcap_keys_device_driver = {
	.probe		= pcap_keys_probe,
	.remove		= pcap_keys_remove,
	.driver		= {
		.name	= "pcap-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init pcap_keys_init(void)
{
	return platform_driver_register(&pcap_keys_device_driver);
};

static void __exit pcap_keys_exit(void)
{
	platform_driver_unregister(&pcap_keys_device_driver);
};

module_init(pcap_keys_init);
module_exit(pcap_keys_exit);

MODULE_DESCRIPTION("Motorola PCAP2 input events driver");
MODULE_AUTHOR("Ilya Petrov <ilya.muromec@gmail.com>");
MODULE_LICENSE("GPL");

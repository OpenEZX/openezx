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
#include <mach/irqs.h>

/* PCAP2 interrupts us on keypress */
static irqreturn_t pcap_pwrkey_handler(int irq, void *pcap_input)
{
	u32 pstat;
	ezx_pcap_read(PCAP_REG_PSTAT, &pstat);
	pstat &= PCAP_IRQ_ONOFF;

	input_report_key(pcap_input, KEY_POWER, (pstat ? 0 : 1));
	input_sync(pcap_input);

	return IRQ_HANDLED;
}

/* PCAP2 interrupts us on plug/unplug */
static irqreturn_t pcap_jack_handler(int irq, void *pcap_input)
{
	u32 pstat;
	ezx_pcap_read(PCAP_REG_PSTAT, &pstat);
	pstat &= PCAP_IRQ_HS ;

	input_report_switch(pcap_input, SW_HEADPHONE_INSERT, (pstat  ? 0 : 1));
	input_sync(pcap_input);

	return IRQ_HANDLED;
}

static int __init pcap_keys_probe(struct platform_device *pdev)
{
	int err = -ENOMEM;
	struct input_dev *pcap_input;

	pcap_input = input_allocate_device();
	if (!pcap_input)
		goto fail;

	platform_set_drvdata(pdev, pcap_input);
	pcap_input->name = pdev->name;
	pcap_input->phys = "pcap-keys/input0";
	pcap_input->dev.parent = &pdev->dev;

	pcap_input->evbit[0] = BIT_MASK(EV_KEY)  | BIT_MASK(EV_SW);
	pcap_input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	set_bit(SW_HEADPHONE_INSERT, pcap_input->swbit);

	err = request_irq(PCAP_IRQ_ONOFF, pcap_pwrkey_handler, 0,
				"Power key", pcap_input);
	if (err)
		goto fail_dev;

	err = request_irq(PCAP_IRQ_HS, pcap_jack_handler, 0,
				"HP/MIC", pcap_input);
	if (err)
		goto fail_pwrkey;

	err =  input_register_device(pcap_input);
	if (err)
		goto fail_jack;

	return 0;

fail_jack:
	free_irq(PCAP_IRQ_HS, pcap_input);
fail_pwrkey:
	free_irq(PCAP_IRQ_ONOFF, pcap_input);
fail_dev:
	input_free_device(pcap_input);
fail:
	return err;
}

static int pcap_keys_remove(struct platform_device *pdev)
{
	struct input_dev *pcap_input = platform_get_drvdata(pdev);

	free_irq(PCAP_IRQ_ONOFF, pcap_input);
	free_irq(PCAP_IRQ_HS, pcap_input);

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

/*
 *  Input driver for PCAP events:
 *   * Power key
 *   * Jack plug/unplug
 *   * Headphone button
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

struct pcap_keys {
	struct pcap_chip *pcap;
	struct input_dev *input;
};

/* PCAP2 interrupts us on keypress */
static irqreturn_t pcap_keys_handler(int irq, void *_pcap_keys)
{
	struct pcap_keys *pcap_keys = _pcap_keys;
	int pirq = irq_to_pcap(pcap_keys->pcap, irq);
	u32 pstat;

	ezx_pcap_read(pcap_keys->pcap, PCAP_REG_PSTAT, &pstat);
	pstat &= 1 << pirq;

	switch (pirq) {
	case PCAP_IRQ_ONOFF:
		input_report_key(pcap_keys->input, KEY_POWER, !pstat);
		break;
	case PCAP_IRQ_HS:
		input_report_switch(pcap_keys->input,
				SW_HEADPHONE_INSERT, !pstat);
		break;
	case PCAP_IRQ_MIC:
		input_report_key(pcap_keys->input, KEY_HP, !pstat);
		break;
	}

	input_sync(pcap_keys->input);

	return IRQ_HANDLED;
}

static int __init pcap_keys_probe(struct platform_device *pdev)
{
	int err = -ENOMEM;
	struct pcap_keys *pcap_keys;

	pcap_keys = kmalloc(sizeof(struct pcap_keys), GFP_KERNEL);
	if (!pcap_keys)
		return err;

	pcap_keys->pcap = platform_get_drvdata(pdev);

	pcap_keys->input = input_allocate_device();
	if (!pcap_keys->input)
		goto fail;

	platform_set_drvdata(pdev, pcap_keys);
	pcap_keys->input->name = pdev->name;
	pcap_keys->input->phys = "pcap-keys/input0";
	pcap_keys->input->dev.parent = &pdev->dev;

	pcap_keys->input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SW);
	set_bit(KEY_POWER, pcap_keys->input->keybit);
	set_bit(SW_HEADPHONE_INSERT, pcap_keys->input->swbit);
	set_bit(KEY_HP, pcap_keys->input->keybit);

	err = request_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_ONOFF),
			pcap_keys_handler, 0, "Power key", pcap_keys);
	if (err)
		goto fail_dev;

	err = request_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_HS),
			pcap_keys_handler, 0, "Headphone jack", pcap_keys);
	if (err)
		goto fail_pwrkey;

	err = request_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_MIC),
			pcap_keys_handler, 0, "MIC jack/button", pcap_keys);
	if (err)
		goto fail_jack;

	err = input_register_device(pcap_keys->input);
	if (err)
		goto fail_mic;

	return 0;

fail_mic:
	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_MIC), pcap_keys);
fail_jack:
	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_HS), pcap_keys);
fail_pwrkey:
	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_ONOFF), pcap_keys);
fail_dev:
	input_free_device(pcap_keys->input);
fail:
	kfree(pcap_keys);
	return err;
}

static int pcap_keys_remove(struct platform_device *pdev)
{
	struct pcap_keys *pcap_keys = platform_get_drvdata(pdev);

	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_ONOFF), pcap_keys);
	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_HS), pcap_keys);
	free_irq(pcap_to_irq(pcap_keys->pcap, PCAP_IRQ_MIC), pcap_keys);

	input_unregister_device(pcap_keys->input);
	kfree(pcap_keys);

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

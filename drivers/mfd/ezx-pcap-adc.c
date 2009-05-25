/*
 *  pcap ADC code for Motorola EZX phones
 *
 *  Copyright (c) 2009 Daniel Ribeiro <drwyrm@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/ezx-pcap.h>

#define PCAP_ADC_MAXQ		8

struct pcap_adc_request {
	u8 bank;
	u8 ch[2];
	u32 flags;

	void (*callback)(void *, u16[]);
	void *data;
};

struct pcap_adc_sync_request {
	u16 res[2];
	struct completion completion;
};

struct pcap_adc {
	struct pcap_adc_request *queue[PCAP_ADC_MAXQ];
	u8 head;
	u8 tail;
	struct mutex mutex;
};
struct pcap_adc adc;

static void pcap_disable_adc(void)
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADEN|PCAP_ADC_BATT_I_ADC | PCAP_ADC_BATT_I_POLARITY);
	tmp |= (PCAP_ADC_TS_M_STANDBY << PCAP_ADC_TS_M_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
}

static void pcap_adc_trigger(void)
{
	u32 tmp;
	u8 head;

	mutex_lock(&adc.mutex);
	head = adc.head;
	if (!adc.queue[head]) {
		/* queue is empty */
		pcap_disable_adc();
		mutex_unlock(&adc.mutex);
		return;
	}
	mutex_unlock(&adc.mutex);

	/* start conversion on requested bank */
	tmp = adc.queue[head]->flags | PCAP_ADC_ADEN;

	if (adc.queue[head]->bank == PCAP_ADC_BANK_1)
		tmp |= PCAP_ADC_AD_SEL1;

	ezx_pcap_write(PCAP_REG_ADC, tmp);
	ezx_pcap_write(PCAP_REG_ADR, PCAP_ADR_ASC);
}

static int pcap_adc_enqueue(struct pcap_adc_request *req)
{
	mutex_lock(&adc.mutex);
	if (adc.queue[adc.tail]) {
		mutex_unlock(&adc.mutex);
		return -EBUSY;
	}
	adc.queue[adc.tail] = req;
	adc.tail = (adc.tail + 1) & (PCAP_ADC_MAXQ - 1);
	mutex_unlock(&adc.mutex);
	pcap_adc_trigger();
	
	return 0;
}

static irqreturn_t pcap_adc_irq(int irq, void *unused)
{
	struct pcap_adc_request *req;
	u16 res[2];
	u32 tmp;

	mutex_lock(&adc.mutex);
	req = adc.queue[adc.head];

	if (!req) { /* huh? */
		printk("%s: spurious adc irq???\n", __func__);
		return IRQ_HANDLED;
	}

	/* read requested channels results */
	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADA1_MASK | PCAP_ADC_ADA2_MASK);
	tmp |= (req->ch[0] << PCAP_ADC_ADA1_SHIFT);
	tmp |= (req->ch[1] << PCAP_ADC_ADA2_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	ezx_pcap_read(PCAP_REG_ADR, &tmp);
	res[0] = (tmp & PCAP_ADR_ADD1_MASK) >> PCAP_ADR_ADD1_SHIFT;
	res[1] = (tmp & PCAP_ADR_ADD2_MASK) >> PCAP_ADR_ADD2_SHIFT;

	adc.queue[adc.head] = NULL;
	adc.head = (adc.head + 1) & (PCAP_ADC_MAXQ - 1);
	mutex_unlock(&adc.mutex);

	req->callback(req->data, res);
	kfree(req);
	pcap_adc_trigger();

	return IRQ_HANDLED;
}

int pcap_adc_async(u8 bank, u32 flags, u8 ch[], void *callback, void *data)
{
	struct pcap_adc_request *req;
	int ret = -ENOMEM;

	req = kmalloc(sizeof(struct pcap_adc_request), GFP_KERNEL);
	if (!req)
		return ret;

	req->bank = bank;
	req->flags = flags;
	req->ch[0] = ch[0];
	req->ch[1] = ch[1];
	req->callback = callback;
	req->data = data;


	ret = pcap_adc_enqueue(req);
	if (ret)
		kfree(req);
	return ret;
}
EXPORT_SYMBOL_GPL(pcap_adc_async);

static void pcap_adc_sync_cb(void *param, u16 res[])
{
	struct pcap_adc_sync_request *req = param;

	req->res[0] = res[0];
	req->res[1] = res[1];
	complete(&req->completion);
}

int pcap_adc_sync(u8 bank, u32 flags, u8 ch[], u16 res[])
{
	struct pcap_adc_sync_request sync_data;
	int ret;

	init_completion(&sync_data.completion);
	ret = pcap_adc_async(bank, flags, ch, pcap_adc_sync_cb, &sync_data);
	if (ret)
		return ret;
	wait_for_completion(&sync_data.completion);
	res[0] = sync_data.res[0];
	res[1] = sync_data.res[1];

	return 0;
}
EXPORT_SYMBOL_GPL(pcap_adc_sync);

static int __devinit pcap_adc_probe(struct platform_device *pdev)
{
	mutex_init(&adc.mutex);

	request_irq(pcap_irq(PCAP_IRQ_ADCDONE2), pcap_adc_irq, 0, "ADC", NULL);
	request_irq(pcap_irq(PCAP_IRQ_ADCDONE), pcap_adc_irq, 0, "ADC", NULL);

	return 0;
}

static int __devexit pcap_adc_remove(struct platform_device *pdev)
{
	int i;

	free_irq(pcap_irq(PCAP_IRQ_ADCDONE2), NULL);
	free_irq(pcap_irq(PCAP_IRQ_ADCDONE), NULL);

	mutex_lock(&adc.mutex);
	for (i = 0; i < PCAP_ADC_MAXQ; i++)
		kfree(adc.queue[i]);
	mutex_unlock(&adc.mutex);

	return 0;
}

static struct platform_driver pcap_adc_driver = {
	.probe = pcap_adc_probe,
	.remove = __devexit_p(pcap_adc_remove),
	.driver = {
		.name  = "pcap-adc",
	},
};

static int __init pcap_adc_init(void)
{
	return platform_driver_register(&pcap_adc_driver);
}

static void __exit pcap_adc_exit(void)
{
	platform_driver_unregister(&pcap_adc_driver);
}

module_init(pcap_adc_init);
module_exit(pcap_adc_exit);

MODULE_DESCRIPTION("Motorola EZX pcap ADC driver");
MODULE_AUTHOR("Daniel Ribeiro <drwyrm@gmail.com>");
MODULE_LICENSE("GPL");

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

static DEFINE_MUTEX(adc_lock);
static void (* ezx_pcap_adc_done)(void *);
static void *ezx_pcap_adc_data;

static irqreturn_t ezx_pcap_adc_irq(int irq, void *unused)
{
	void (*adc_done)(void *);
	void *adc_data;

	printk(KERN_DEBUG "ezx_pcap_adc_irq: called!\n");

	if (!ezx_pcap_adc_done)
		return IRQ_HANDLED;

	adc_done = ezx_pcap_adc_done;
	adc_data = ezx_pcap_adc_data;
	ezx_pcap_adc_done = ezx_pcap_adc_data = NULL;

	/* let caller get the results */
	adc_done(adc_data);

	return IRQ_HANDLED;
}

void ezx_pcap_start_adc(u8 bank, u8 time, u32 flags,
		void *adc_done, void *adc_data)
{
	u32 adc;
	u32 adr;

	printk(KERN_DEBUG "ezx_pcap_start_adc: called!\n");

	mutex_lock(&adc_lock);

	adc = flags | PCAP_ADC_ADEN;

	if (bank == PCAP_ADC_BANK_1)
		adc |= PCAP_ADC_AD_SEL1;

	ezx_pcap_write(PCAP_REG_ADC, adc);

	ezx_pcap_adc_done = adc_done;
	ezx_pcap_adc_data = adc_data;

	if (time == PCAP_ADC_T_NOW) {
		ezx_pcap_read(PCAP_REG_ADR, &adr);
		adr = PCAP_ADR_ASC;
		ezx_pcap_write(PCAP_REG_ADR, adr);
		return;
	}

	if (time == PCAP_ADC_T_IN_BURST)
		adc |= (PCAP_ADC_ATO_IN_BURST << PCAP_ADC_ATO_SHIFT);

	ezx_pcap_write(PCAP_REG_ADC, adc);

	ezx_pcap_read(PCAP_REG_ADR, &adr);
	adr &= ~PCAP_ADR_ONESHOT;
	ezx_pcap_write(PCAP_REG_ADR, adr);
	adr |= PCAP_ADR_ONESHOT;
	ezx_pcap_write(PCAP_REG_ADR, adr);
}
EXPORT_SYMBOL_GPL(ezx_pcap_start_adc);

void ezx_pcap_get_adc_channel_result(u8 ch1, u8 ch2, u32 res[])
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADA1_MASK | PCAP_ADC_ADA2_MASK);
	tmp |= (ch1 << PCAP_ADC_ADA1_SHIFT) | (ch2 << PCAP_ADC_ADA2_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	ezx_pcap_read(PCAP_REG_ADR, &tmp);
	res[0] = (tmp & PCAP_ADR_ADD1_MASK) >> PCAP_ADR_ADD1_SHIFT;
	res[1] = (tmp & PCAP_ADR_ADD2_MASK) >> PCAP_ADR_ADD2_SHIFT;
}
EXPORT_SYMBOL_GPL(ezx_pcap_get_adc_channel_result);

void ezx_pcap_disable_adc()
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADEN|PCAP_ADC_BATT_I_ADC | PCAP_ADC_BATT_I_POLARITY);
	tmp |= (PCAP_ADC_TS_M_STANDBY << PCAP_ADC_TS_M_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	mutex_unlock(&adc_lock);
}
EXPORT_SYMBOL_GPL(ezx_pcap_disable_adc);


static int __devinit ezx_pcap_adc_probe(struct platform_device *plat_dev)
{
	struct pcap_platform_data *ppdata = (plat_dev->dev.parent)->platform_data;

	if (ppdata->config & PCAP_SECOND_PORT)
		request_irq(PCAP_IRQ_ADCDONE2, ezx_pcap_adc_irq, 0, "ADC", NULL);
	else
		request_irq(PCAP_IRQ_ADCDONE, ezx_pcap_adc_irq, 0, "ADC", NULL);

	return 0;
}

static int __devexit ezx_pcap_adc_remove(struct platform_device *plat_dev)
{
	struct pcap_platform_data *ppdata = (plat_dev->dev.parent)->platform_data;

	if (ppdata->config & PCAP_SECOND_PORT)
		free_irq(PCAP_IRQ_ADCDONE2, NULL);
	else
		free_irq(PCAP_IRQ_ADCDONE, NULL);

	return 0;
}

static struct platform_driver ezx_pcap_adc_driver = {
	.probe = ezx_pcap_adc_probe,
	.remove = __devexit_p(ezx_pcap_adc_remove),
	.driver = {
		.name  = "pcap-adc",
	},
};

static int __init ezx_pcap_adc_init(void)
{
	return platform_driver_register(&ezx_pcap_adc_driver);
}

static void __exit ezx_pcap_adc_exit(void)
{
	platform_driver_unregister(&ezx_pcap_adc_driver);
}

module_init(ezx_pcap_adc_init);
module_exit(ezx_pcap_adc_exit);

MODULE_DESCRIPTION("Motorola EzX pcap ADC driver");
MODULE_AUTHOR("Daniel Ribeiro <drwyrm@gmail.com>");
MODULE_LICENSE("GPL");

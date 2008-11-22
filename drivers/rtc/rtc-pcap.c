/*
 *  pcap rtc code for Motorola EZX phones
 *
 *  Copyright (c) 2008 guiming zhuo <gmzhuo@gmail.com>
 *  Copyright (c) 2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 *  Based on Motorola's rtc.c Copyright (c) 2003-2005 Motorola
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

static void pcap_rtc_irq(u32 events, void *data)
{
	unsigned long rtc_events = 0;
	struct rtc_device *rtc = data;

	if (events & PCAP_IRQ_1HZ)
		rtc_events |= RTC_IRQF | RTC_UF;
	if (events & PCAP_IRQ_TODA)
		rtc_events |= RTC_IRQF | RTC_AF;

	rtc_update_irq(rtc, 1, rtc_events);
	return;
}

static int pcap_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	unsigned long secs;
	u32 value;

	ezx_pcap_read(PCAP_REG_RTC_TODA, &value);
	value &= PCAP_RTC_TOD_MASK;
	secs = value;

	ezx_pcap_read(PCAP_REG_RTC_DAYA, &value);
	value &= PCAP_RTC_DAY_MASK;
	secs += value * SEC_PER_DAY;

	rtc_time_to_tm(secs, tm);

	return 0;
}

static int pcap_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	unsigned long secs;
	u32 value;

	rtc_tm_to_time(tm, &secs);

	ezx_pcap_read(PCAP_REG_RTC_TODA, &value);
	value &= ~PCAP_RTC_TOD_MASK;
	value |= secs % SEC_PER_DAY;
	ezx_pcap_write(PCAP_REG_RTC_TODA, value);

	ezx_pcap_read(PCAP_REG_RTC_DAYA, &value);
	value &= ~PCAP_RTC_DAY_MASK;
	value |= secs / SEC_PER_DAY;
	ezx_pcap_write(PCAP_REG_RTC_DAYA, value);

	return 0;
}

static int pcap_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long secs;
	u32 value;

	ezx_pcap_read(PCAP_REG_RTC_TOD, &value);
	value &= PCAP_RTC_TOD_MASK;
	secs = value;

	ezx_pcap_read(PCAP_REG_RTC_DAY, &value);
	value &= PCAP_RTC_DAY_MASK;
	secs += value * SEC_PER_DAY;

	rtc_time_to_tm(secs, tm);

	return rtc_valid_tm(tm);
}

static int pcap_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	u32 value;
	
	ezx_pcap_read(PCAP_REG_RTC_TOD, &value);
	value &= ~PCAP_RTC_TOD_MASK;
	value |= secs % SEC_PER_DAY;
	ezx_pcap_write(PCAP_REG_RTC_TOD, value);

	ezx_pcap_read(PCAP_REG_RTC_DAY, &value);
	value &= ~PCAP_RTC_DAY_MASK;
	value |= secs / SEC_PER_DAY;
	ezx_pcap_write(PCAP_REG_RTC_DAY, value);

	return 0;
}

static int pcap_rtc_alarm_irq_enable(struct device *dev, unsigned int en)
{
	if (en)
		ezx_pcap_unmask_event(PCAP_IRQ_TODA);
	else
		ezx_pcap_mask_event(PCAP_IRQ_TODA);

	return 0;
}

static int pcap_rtc_update_irq_enable(struct device *dev, unsigned int en)
{
	if (en)
		ezx_pcap_unmask_event(PCAP_IRQ_1HZ);
	else
		ezx_pcap_mask_event(PCAP_IRQ_1HZ);

	return 0;
}

static const struct rtc_class_ops pcap_rtc_ops = {
	.read_time = pcap_rtc_read_time,
	.read_alarm = pcap_rtc_read_alarm,
	.set_alarm = pcap_rtc_set_alarm,
	.set_mmss = pcap_rtc_set_mmss,
	.alarm_irq_enable = pcap_rtc_alarm_irq_enable,
	.update_irq_enable = pcap_rtc_update_irq_enable,
};

static int __init pcap_rtc_probe(struct platform_device *plat_dev)
{
	struct rtc_device *rtc;

	rtc = rtc_device_register("pcap", &plat_dev->dev,
				  &pcap_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(plat_dev, rtc);

	ezx_pcap_register_event(PCAP_IRQ_1HZ, pcap_rtc_irq, rtc, "RTC Timer");
	ezx_pcap_register_event(PCAP_IRQ_TODA, pcap_rtc_irq, rtc, "RTC Alarm");

	return 0;
}

static int __exit pcap_rtc_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);

	ezx_pcap_unregister_event(PCAP_IRQ_1HZ | PCAP_IRQ_TODA);
	rtc_device_unregister(rtc);
	return 0;
}

static struct platform_driver pcap_rtc_driver = {
	.remove = __exit_p(pcap_rtc_remove),
	.driver = {
		.name  = "rtc-pcap",
		.owner = THIS_MODULE,
	},
};

static int __init rtc_pcap_init(void)
{
	return platform_driver_probe(&pcap_rtc_driver, pcap_rtc_probe);
}

static void __exit rtc_pcap_exit(void)
{
	platform_driver_unregister(&pcap_rtc_driver);
}

module_init(rtc_pcap_init);
module_exit(rtc_pcap_exit);

MODULE_DESCRIPTION("Motorola pcap rtc driver");
MODULE_AUTHOR("guiming zhuo <gmzhuo@gmail.com>");
MODULE_LICENSE("GPL");

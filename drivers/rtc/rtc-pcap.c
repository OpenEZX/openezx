/*
 *  pcap rtc code for Motorola EZX phones
 *
 *  Copyright (c) 2008 guiming zhuo <gmzhuo@gmail.com>
 *  Copyright (c) 2009 Daniel Ribeiro <drwyrm@gmail.com>
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

static irqreturn_t pcap_rtc_irq_timer(int irq, void *rtc)
{
	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_UF);
	return IRQ_HANDLED;
}

static irqreturn_t pcap_rtc_irq_alarm(int irq, void *rtc)
{
	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
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

static inline int pcap_rtc_irq_enable(int irq, unsigned int en)
{
	if (en)
		enable_irq(irq);
	else
		disable_irq(irq);

	return 0;
}

static int pcap_rtc_alarm_irq_enable(struct device *dev, unsigned int en)
{
	struct platform_device *pdev = to_platform_device(dev);
	return pcap_rtc_irq_enable(platform_get_irq(pdev, 1), en);
}

static int pcap_rtc_update_irq_enable(struct device *dev, unsigned int en)
{
	struct platform_device *pdev = to_platform_device(dev);
	return pcap_rtc_irq_enable(platform_get_irq(pdev, 0), en);
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
	int irq;

	rtc = rtc_device_register("pcap", &plat_dev->dev,
				  &pcap_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(plat_dev, rtc);

	irq = platform_get_irq(plat_dev, 0);
	if (irq > 0)
		request_irq(irq, pcap_rtc_irq_timer, 0, "RTC Timer", rtc);

	irq = platform_get_irq(plat_dev, 1);
	if (irq > 0)
		request_irq(irq, pcap_rtc_irq_alarm, 0, "RTC Alarm", rtc);

	return 0;
}

static int __exit pcap_rtc_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);

	free_irq(platform_get_irq(plat_dev, 0), rtc);
	free_irq(platform_get_irq(plat_dev, 1), rtc);
	rtc_device_unregister(rtc);

	return 0;
}

static struct platform_driver pcap_rtc_driver = {
	.remove = __exit_p(pcap_rtc_remove),
	.driver = {
		.name  = "pcap_rtc",
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

/*
 *  pcap rtc code for Motorola EZX phones
 *
 *  Copyright (c) 2008 guiming zhuo <gmzhuo@gmail.com>
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
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#define POWER_IC_POWER_CUT_BIT             14
#define POWER_IC_POWER_CUT_NUM_BITS         4
#define POWER_IC_TIME_REG_BIT               0
#define POWER_IC_TOD_NUM_BITS              17
#define POWER_IC_DAY_NUM_BITS              15
#define POWER_IC_NUM_SEC_PER_DAY        86400

#define RTC_TOD_REG                     POWER_IC_REG_PCAP_RTC_TOD
#define RTC_DAY_REG                     POWER_IC_REG_PCAP_RTC_DAY
#define RTC_TODA_REG                    POWER_IC_REG_PCAP_RTC_TODA
#define RTC_DAYA_REG                    POWER_IC_REG_PCAP_RTC_DAYA
#define RTC_POWER_CUT_REG               POWER_IC_REG_PCAP_RTC_DAY
#define RTC_TODA_EVENT                  POWER_IC_EVENT_PCAP_TODAI

static struct rtc_device *rtc;

int power_ic_rtc_set_time(struct timeval *power_ic_time)
{
	int err = 0;
	unsigned int value;
	unsigned int mask;
	if (power_ic_time->tv_usec > 500000) {
		power_ic_time->tv_sec += 1;
	}

	ezx_pcap_read(PCAP_REG_RTC_TOD, &value);
	mask = 1;
	mask <<= 17;
	mask -= 1;
	value &= (~mask);
	value |= ((power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY) & mask);
	ezx_pcap_write(PCAP_REG_RTC_TOD, value);

	ezx_pcap_read(PCAP_REG_RTC_DAY, &value);
	mask = 1;
	mask <<= 15;
	mask -= 1;
	value &= (~mask);
	value |= ((power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY) & mask);
	ezx_pcap_write(PCAP_REG_RTC_DAY, value);

	return err;
}

/*!
 * @brief Gets the RTC time
 *
 * This function retrieves the value in the RTC_TOD and RTC_DAY registers.
 * Those values are converted into the number of seconds that have elapsed
 * since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_get_time(struct timeval *power_ic_time)
{
	int err = 0;

	unsigned int value;
	unsigned int mask;

	ezx_pcap_read(PCAP_REG_RTC_TOD, &value);
	mask = 1;
	mask <<= 17;
	mask -= 1;
	value &= mask;
	power_ic_time->tv_sec = value;

	ezx_pcap_read(PCAP_REG_RTC_DAY, &value);
	mask = 1;
	mask <<= 15;
	mask -= 1;
	value &= mask;
	power_ic_time->tv_sec += value * POWER_IC_NUM_SEC_PER_DAY;

	return err;
}

/*!
 * @brief Sets the RTC alarm time
 *
 * This function sets the value in the RTC_TODA and RTC_DAYA registers based on the
 * number of seconds that have passed since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_set_time_alarm(struct timeval *power_ic_time)
{

	int err = 0;
	unsigned int value;
	unsigned int mask;

	if (power_ic_time->tv_usec > 500000) {
		power_ic_time->tv_sec += 1;
	}

	ezx_pcap_read(PCAP_REG_RTC_TODA, &value);
	mask = 1;
	mask <<= 17;
	mask -= 1;
	value &= ~mask;
	value |= ((power_ic_time->tv_sec % POWER_IC_NUM_SEC_PER_DAY) & mask);
	ezx_pcap_write(PCAP_REG_RTC_TODA, value);

	ezx_pcap_read(PCAP_REG_RTC_DAYA, &value);
	mask = 1;
	mask <<= 15;
	mask -= 1;
	value &= ~mask;
	value |= ((power_ic_time->tv_sec / POWER_IC_NUM_SEC_PER_DAY) & mask);
	ezx_pcap_write(PCAP_REG_RTC_DAYA, value);

//    err |= power_ic_event_unmask(RTC_TODA_EVENT);

	return err;
}

/*!
 * @brief Gets the RTC alarm time
 *
 * This function retrieves the value in the RTC_TODA and RTC_DAYA registers.
 * Those values are converted into the number of seconds that have elapsed
 * since January, 1 1970 00:00:00 UTC.
 *
 * @param power_ic_time pointer to the time in seconds stored in memory
 *
 * @return 0 if successful
 */

int power_ic_rtc_get_time_alarm(struct timeval *power_ic_time)
{
	int err = 0;

	unsigned int value;
	unsigned int mask;

	ezx_pcap_read(PCAP_REG_RTC_TODA, &value);
	mask = 1;
	mask <<= 17;
	mask -= 1;
	value &= mask;
	power_ic_time->tv_sec = value;

	ezx_pcap_read(PCAP_REG_RTC_DAYA, &value);
	mask = 1;
	mask <<= 15;
	mask -= 1;
	value &= mask;

	power_ic_time->tv_sec += value * POWER_IC_NUM_SEC_PER_DAY;

	return err;
}

/*!
 * @brief Gets the value of the power cut counter
 *
 * This function reads the power cut counter from the power IC.
 *
 * @param power_cuts pointer to location in which to store the power cut counter value
 *
 * @return 0 if successful
 */

int power_ic_get_num_power_cuts(int *power_cuts)
{

	//err = power_ic_get_reg_value(RTC_POWER_CUT_REG, POWER_IC_POWER_CUT_BIT,
	//                           power_cuts, POWER_IC_POWER_CUT_NUM_BITS);
	return (1000);
}

static irqreturn_t pcap_hz_irq(void *unused)
{
	struct timeval tmrtc, tmsys;
	time_t diff;

	power_ic_rtc_get_time(&tmrtc);
	do_gettimeofday(&tmsys);
	if (tmsys.tv_usec > 500000) {
		tmsys.tv_sec++;
	}

	if (tmsys.tv_sec < 3) {
		return IRQ_HANDLED;
	}

	if (tmsys.tv_sec > tmrtc.tv_sec) {
		diff = tmsys.tv_sec - tmrtc.tv_sec;
	} else {
		diff = tmrtc.tv_sec - tmsys.tv_sec;
	}
	if (diff > 1) {
		do_gettimeofday(&tmsys);
		power_ic_rtc_set_time(&tmsys);
	}

	return IRQ_HANDLED;
}

static irqreturn_t pcap_alarm_irq(void *unused)
{

	rtc_update_irq(rtc, 1, RTC_AF | RTC_IRQF);

	return IRQ_HANDLED;
}

void rtc_time_to_tm(unsigned long time, struct rtc_time *tm);
int rtc_tm_to_time(struct rtc_time *tm, unsigned long *time);

static int pcap_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	struct timeval power_ic_time;

	power_ic_rtc_get_time_alarm(&power_ic_time);
	rtc_time_to_tm(power_ic_time.tv_sec, tm);

	return 0;
}

static int pcap_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	unsigned long secs;
	int err;
	struct timeval power_ic_time;

	err = rtc_tm_to_time(tm, &secs);
	power_ic_time.tv_sec = secs;
	power_ic_time.tv_usec = 0;
	power_ic_rtc_set_time_alarm(&power_ic_time);

	return 0;
}

static int pcap_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct timeval tmv;
	power_ic_rtc_get_time(&tmv);
	rtc_time_to_tm(tmv.tv_sec, tm);
	return 0;
}

static int pcap_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct timeval tmv;
	rtc_tm_to_time(tm, &tmv.tv_sec);
	power_ic_rtc_set_time(&tmv);
	return 0;
}

static int pcap_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	return 0;
}

static int pcap_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct platform_device *plat_dev = to_platform_device(dev);

	seq_printf(seq, "pcap\t\t: yes\n");
	seq_printf(seq, "id\t\t: %d\n", plat_dev->id);

	return 0;
}

static int pcap_rtc_ioctl(struct device *dev, unsigned int cmd,
			  unsigned long arg)
{
	/* We do support interrupts, they're generated
	 * using the sysfs interface.
	 */
	switch (cmd) {
	case RTC_PIE_ON:
	case RTC_PIE_OFF:
	case RTC_UIE_ON:
	case RTC_UIE_OFF:
	case RTC_AIE_ON:
	case RTC_AIE_OFF:
		return 0;

	default:
		return -ENOIOCTLCMD;
	}

}

static const struct rtc_class_ops pcap_rtc_ops = {
	.proc = pcap_rtc_proc,
	.read_time = pcap_rtc_read_time,
	.set_time = pcap_rtc_set_time,
	.read_alarm = pcap_rtc_read_alarm,
	.set_alarm = pcap_rtc_set_alarm,
	.set_mmss = pcap_rtc_set_mmss,
	.ioctl = pcap_rtc_ioctl,
};

static int pcap_rtc_probe(struct platform_device *plat_dev)
{
	int err;
	rtc = rtc_device_register("pcap", &plat_dev->dev,
				  &pcap_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		goto error;
	}

	platform_set_drvdata(plat_dev, rtc);

	/* request 1Hz event */
	ezx_pcap_register_event(PCAP_IRQ_1HZ, pcap_hz_irq, rtc);
	ezx_pcap_register_event(PCAP_IRQ_TODA, pcap_alarm_irq, rtc);

	return 0;

      error:
	rtc_device_unregister(rtc);
	return err;
}

static int __devexit pcap_rtc_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);
	ezx_pcap_unregister_event(PCAP_IRQ_1HZ);
	rtc_device_unregister(rtc);
	return 0;
}

static struct platform_driver pcap_rtc_driver = {
	.probe = pcap_rtc_probe,
	.remove = __devexit_p(pcap_rtc_remove),
	.driver = {
		   .name = "rtc-pcap",
		   .owner = THIS_MODULE,
		   },
};

static int __init rtc_pcap_init(void)
{
	int err;
	err = platform_driver_register(&pcap_rtc_driver);
	return err;
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

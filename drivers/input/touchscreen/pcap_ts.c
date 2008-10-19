/*
 * pcap_ts.c - Touchscreen driver for Motorola PCAP2 based touchscreen as found
 * 	       in the EZX phone platform.
 *
 *  Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 *  Copyright (C) 2007-2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 *  Based on information found in the original Motorola 2.4.x ezx-ts.c driver.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/ezx-pcap.h>

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>

struct pcap_ts {
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	u16 x, y;
	u16 pressure;
	u8 read_state;
};

struct pcap_ts *pcap_ts;

#define X_AXIS_MIN	0
#define X_AXIS_MAX	1023

#define Y_AXIS_MAX	X_AXIS_MAX
#define Y_AXIS_MIN	X_AXIS_MIN

#define PRESSURE_MAX	X_AXIS_MAX
#define PRESSURE_MIN	X_AXIS_MIN

/* if we try to read faster, pressure reading becomes unreliable */
#define SAMPLE_INTERVAL		(HZ/50)

static void pcap_ts_read_xy(void)
{
	u32 res[2];

	ezx_pcap_get_adc_channel_result(PCAP_ADC_CH_TS_X1, PCAP_ADC_CH_TS_Y1,
									res);
	ezx_pcap_disable_adc();

	switch (pcap_ts->read_state) {
	case PCAP_ADC_TS_M_PRESSURE:
		/* save pressure, start xy read */
		pcap_ts->pressure = res[0];
		pcap_ts->read_state = PCAP_ADC_TS_M_XY;
		schedule_work(&pcap_ts->work);
		break;
	case PCAP_ADC_TS_M_XY:
		pcap_ts->y = res[0];
		pcap_ts->x = res[1];
		if (pcap_ts->x <= X_AXIS_MIN || pcap_ts->x >= X_AXIS_MAX ||
		    pcap_ts->y <= Y_AXIS_MIN || pcap_ts->y >= Y_AXIS_MAX ||
		    pcap_ts->pressure <= PRESSURE_MIN ||
		    pcap_ts->pressure >= PRESSURE_MAX) {
			/* pen has been released */
			input_report_key(pcap_ts->input, BTN_TOUCH, 0);
			input_report_abs(pcap_ts->input, ABS_PRESSURE, 0);

			/* no need for timer, we'll get interrupted with
			 * next touch down event */
			del_timer(&pcap_ts->timer);

			/* ask PCAP2 to interrupt us if touch event happens
			 * again */
			pcap_ts->read_state = PCAP_ADC_TS_M_STANDBY;
			ezx_pcap_unmask_event(PCAP_IRQ_TS);
			schedule_work(&pcap_ts->work);
		} else {
			/* pen is touching the screen*/
			input_report_abs(pcap_ts->input, ABS_X, pcap_ts->x);
			input_report_abs(pcap_ts->input, ABS_Y, pcap_ts->y);
			input_report_key(pcap_ts->input, BTN_TOUCH, 1);
			input_report_abs(pcap_ts->input, ABS_PRESSURE,
						pcap_ts->pressure);

			/* switch back to pressure read mode */
			pcap_ts->read_state = PCAP_ADC_TS_M_PRESSURE;
			mod_timer(&pcap_ts->timer,
					jiffies + SAMPLE_INTERVAL);
		}
		input_sync(pcap_ts->input);
		break;
	default:
		break;
	}
}

static void pcap_ts_work(struct work_struct *unused)
{
	u32 tmp;

	switch (pcap_ts->read_state) {
	case PCAP_ADC_TS_M_STANDBY:
		/* set TS to standby */
		ezx_pcap_read(PCAP_REG_ADC, &tmp);
		tmp &= ~PCAP_ADC_TS_M_MASK;
		tmp |= (PCAP_ADC_TS_M_STANDBY << PCAP_ADC_TS_M_SHIFT);
		ezx_pcap_write(PCAP_REG_ADC, tmp);
		break;
	case PCAP_ADC_TS_M_PRESSURE:
	case PCAP_ADC_TS_M_XY:
		/* start adc conversion */
		ezx_pcap_start_adc(PCAP_ADC_BANK_1, PCAP_ADC_T_NOW,
			(pcap_ts->read_state << PCAP_ADC_TS_M_SHIFT),
						pcap_ts_read_xy, NULL);
		break;
	}
}

static void pcap_ts_event_touch(u32 events)
{
	/* pen touch down, mask touch event and start reading pressure */
	ezx_pcap_mask_event(PCAP_IRQ_TS);
	pcap_ts->read_state = PCAP_ADC_TS_M_PRESSURE;
	schedule_work(&pcap_ts->work);
}

static void pcap_ts_timer_fn(unsigned long data)
{
	schedule_work(&pcap_ts->work);
}

static int __devinit ezxts_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	int err = -ENOMEM;

	pcap_ts = kzalloc(sizeof(*pcap_ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!pcap_ts || !input_dev)
		goto fail;

	INIT_WORK(&pcap_ts->work, pcap_ts_work);

	pcap_ts->read_state = PCAP_ADC_TS_M_STANDBY;

	init_timer(&pcap_ts->timer);
	pcap_ts->timer.data = (unsigned long) pcap_ts;
	pcap_ts->timer.function = &pcap_ts_timer_fn;

	pcap_ts->input = input_dev;

	input_dev->name = "pcap-touchscreen";
	input_dev->phys = "ezxts/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, PRESSURE_MIN,
			     PRESSURE_MAX, 0, 0);

	ezx_pcap_register_event(PCAP_IRQ_TS, pcap_ts_event_touch,
							"Touch Screen");

	err = input_register_device(pcap_ts->input);
	if (err)
		goto fail_touch;

	schedule_work(&pcap_ts->work);

	return 0;

fail_touch:
	ezx_pcap_unregister_event(PCAP_IRQ_TS);
fail:
	input_free_device(input_dev);
	kfree(pcap_ts);

	return err;
}

static int __devexit ezxts_remove(struct platform_device *pdev)
{
	ezx_pcap_unregister_event(PCAP_IRQ_TS);

	del_timer_sync(&pcap_ts->timer);

	input_unregister_device(pcap_ts->input);
	kfree(pcap_ts);

	return 0;
}

#ifdef CONFIG_PM
static int ezxts_suspend(struct platform_device *dev, pm_message_t state)
{
	u32 tmp;
	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp |= PCAP_ADC_TS_REF_LOWPWR;
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	return 0;
}

static int ezxts_resume(struct platform_device *dev)
{
	u32 tmp;
	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~PCAP_ADC_TS_REF_LOWPWR;
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	return 0;
}
#else

#define ezxts_suspend NULL
#define ezxts_resume  NULL

#endif

static struct platform_driver ezxts_driver = {
	.probe		= ezxts_probe,
	.remove		= __devexit_p(ezxts_remove),
	.suspend	= ezxts_suspend,
	.resume		= ezxts_resume,
	.driver		= {
		.name	= "pcap-ts",
		.owner	= THIS_MODULE,
	},
};

static int __init ezxts_init(void)
{
	return platform_driver_register(&ezxts_driver);
}

static void __exit ezxts_exit(void)
{
	platform_driver_unregister(&ezxts_driver);
}

module_init(ezxts_init);
module_exit(ezxts_exit);

MODULE_DESCRIPTION("Motorola PCAP2 touchscreen driver");
MODULE_AUTHOR("Daniel Ribeiro / Harald Welte");
MODULE_LICENSE("GPL");

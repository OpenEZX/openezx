/*
 * linux/drivers/leds/leds-pcap.c
 *
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/leds-pcap.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mfd/ezx-pcap.h>

static void pcap_led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness val)
{
	struct pcap_led *led = container_of(led_cdev, struct pcap_led, ldev);

	led->brightness = val;

	schedule_work(&led->work);
}

static void pcap_led_work(struct work_struct *work)
{
	u32 tmp;
	u8 t, c, e;
	struct pcap_led *led = container_of(work, struct pcap_led, work);

	ezx_pcap_read(led->pcap, PCAP_REG_PERIPH, &tmp);
	switch (led->type) {
	case PCAP_LED0:
		t = PCAP_LED0_T_SHIFT;
		c = PCAP_LED0_C_SHIFT;
		e = PCAP_LED0_EN;
		if (led->brightness)
			led->brightness = 1;
		break;
	case PCAP_LED1:
		t = PCAP_LED1_T_SHIFT;
		c = PCAP_LED1_C_SHIFT;
		e = PCAP_LED1_EN;
		if (led->brightness)
			led->brightness = 1;
		break;
	case PCAP_BL0:
		if (led->brightness > PCAP_BL_MASK)
			led->brightness = PCAP_BL_MASK;
		tmp &= ~(PCAP_BL_MASK << PCAP_BL0_SHIFT);
		tmp |= led->brightness << PCAP_BL0_SHIFT;
		ezx_pcap_write(led->pcap, PCAP_REG_PERIPH, tmp);
		return;
	case PCAP_BL1:
		if (led->brightness > PCAP_BL_MASK)
			led->brightness = PCAP_BL_MASK;
		tmp &= ~(PCAP_BL_MASK << PCAP_BL1_SHIFT);
		tmp |= led->brightness << PCAP_BL1_SHIFT;
		ezx_pcap_write(led->pcap, PCAP_REG_PERIPH, tmp);
		return;
	default:
		return;
	}
	/* turn off */
	tmp &= ~(e | (PCAP_LED_T_MASK << t) | (PCAP_LED_C_MASK << c));

	if (led->brightness) /* turn on */
		tmp |= (e | (led->curr << c) | (led->timing << t));

	if (led->gpio & PCAP_LED_GPIO_EN)
		gpio_set_value((led->gpio & PCAP_LED_GPIO_VAL_MASK),
			((led->gpio & PCAP_LED_GPIO_INVERT) ?
			!led->brightness : led->brightness));

	ezx_pcap_write(led->pcap, PCAP_REG_PERIPH, tmp);
}

static int __devinit pcap_led_probe(struct platform_device *pdev)
{
	int i, err;
	struct pcap_leds_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < pdata->num_leds; i++) {
		struct pcap_led *led = &pdata->leds[i];
		led->ldev.name = led->name;
		led->ldev.brightness_set = pcap_led_set_brightness;
		led->pcap = dev_get_drvdata(pdev->dev.parent);
		if (led->gpio & PCAP_LED_GPIO_EN) {
			int gpio = (led->gpio & PCAP_LED_GPIO_VAL_MASK);
			err = gpio_request(gpio, "PCAP LED");
			if (err) {
				dev_err(&pdev->dev,
					"couldn't request gpio %d\n", gpio);
				goto fail;
			}
			gpio_direction_output(gpio,
				(led->gpio & PCAP_LED_GPIO_INVERT) ? 1 : 0);
		}
		err = led_classdev_register(&pdev->dev, &led->ldev);
		if (err) {
			dev_err(&pdev->dev, "couldn't register LED %s\n",
					led->name);
			goto fail;
		}
		INIT_WORK(&led->work, pcap_led_work);
	}
	return 0;

fail:
	while (i >= 0) {
		led_classdev_unregister(&pdata->leds[--i].ldev);
		cancel_work_sync(&pdata->leds[i].work);
	}
	return err;
}

static int __devexit pcap_led_remove(struct platform_device *pdev)
{
	int i;
	struct pcap_leds_platform_data *pdata = pdev->dev.platform_data;

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&pdata->leds[i].ldev);
		cancel_work_sync(&pdata->leds[i].work);
	}
	return 0;
}

static struct platform_driver pcap_led_driver = {
	.probe  = pcap_led_probe,
	.remove = pcap_led_remove,
	.driver = {
		.name = "pcap-leds",
	},
};

static int __init pcap_led_init(void)
{
	return platform_driver_register(&pcap_led_driver);
}

static void __exit pcap_led_exit(void)
{
	return platform_driver_unregister(&pcap_led_driver);
}

module_init(pcap_led_init);
module_exit(pcap_led_exit);

MODULE_AUTHOR("Daniel Ribeiro <drwyrm@gmail.com>");
MODULE_DESCRIPTION("PCAP LED driver");
MODULE_LICENSE("GPL");

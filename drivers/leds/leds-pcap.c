/*
 * leds-pcap.c - leds driver for PCAP mfd chip used on EZX platform
 *
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
 * Copyright (C) 2009 Antonio Ospite <ospite@studenti.unina.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/leds-pcap.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mfd/ezx-pcap.h>

/* 
 * PCAP supports different types of leds:
 *  - two backlights with 32 possible brightness values
 *  - two simple leds
 * 
 * On some phones on which PCAP2 is used there are some leds with the output
 * connected to a switch controlled by a GPIO.
 * 
 * In Motorola e680, for instance, this allows to control three leds by
 * sharing one PCAP2 simple led between two actual leds.
 */


struct pcap_led_data {
	struct pcap_led *pled;
	struct led_classdev ldev;
	struct work_struct work;
	struct pcap_chip *pcap;
};

static void pcap_led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness val)
{
	struct pcap_led_data *led;

	led = container_of(led_cdev, struct pcap_led_data, ldev);
	led->pled->brightness = val;

	schedule_work(&led->work);
}

static inline void pcap_led_set_led(struct pcap_led_data *led)
{
	u32 tmp;
	u8 t, c, e; /* timing, current, enable shifts */
	struct pcap_led *pled = led->pled;

	switch (pled->type) {
	case PCAP_LED0:
		t = PCAP_LED0_T_SHIFT;
		c = PCAP_LED0_C_SHIFT;
		e = PCAP_LED0_EN;
		break;
	case PCAP_LED1:
		t = PCAP_LED1_T_SHIFT;
		c = PCAP_LED1_C_SHIFT;
		e = PCAP_LED1_EN;
		break;
	default:
		dev_warn(led->ldev.dev, "unknown led type %d\n", pled->type);
		return;
	}

	ezx_pcap_read(led->pcap, PCAP_REG_PERIPH, &tmp);

	/* turn off */
	tmp &= ~(e | (PCAP_LED_T_MASK << t) | (PCAP_LED_C_MASK << c));

	if (pled->brightness) /* turn on */
		tmp |= (e | (pled->curr << c) | (pled->timing << t));

	if (gpio_is_valid(pled->gpio))
		gpio_set_value(pled->gpio,
				!!pled->brightness ^ pled->gpio_invert);

	ezx_pcap_write(led->pcap, PCAP_REG_PERIPH, tmp);
}

static inline void pcap_led_set_bl(struct pcap_led_data *led)
{
	u32 tmp;
	u32 shift;
	struct pcap_led *pled = led->pled;

	switch (pled->type) {
	case PCAP_BL0:
		shift = PCAP_BL0_SHIFT;
		break;
	case PCAP_BL1:
		shift = PCAP_BL1_SHIFT;
		break;
	default:
		dev_warn(led->ldev.dev, "unknown led type %d\n", pled->type);
		return;
	}

	ezx_pcap_read(led->pcap, PCAP_REG_PERIPH, &tmp);
	tmp &= ~(PCAP_BL_MASK << shift);
	tmp |= pled->brightness << shift;
	ezx_pcap_write(led->pcap, PCAP_REG_PERIPH, tmp);
}

static void pcap_led_led_work(struct work_struct *work)
{
	struct pcap_led_data *led;
	
	led = container_of(work, struct pcap_led_data, work);
	pcap_led_set_led(led);
}

static void pcap_led_bl_work(struct work_struct *work)
{
	struct pcap_led_data *led;
	
	led = container_of(work, struct pcap_led_data, work);
	pcap_led_set_bl(led);
}

static int __devinit pcap_led_probe(struct platform_device *pdev)
{
	int i, err;
	struct pcap_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pcap_led_data *leds;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	leds = kzalloc(sizeof(*leds) * pdata->num_leds, GFP_KERNEL);
	if (leds == NULL) {
		return -ENOMEM;
	}

	for (i = 0; i < pdata->num_leds; i++) {
		struct pcap_led_data *led = &leds[i];
		struct pcap_led *pled = &pdata->leds[i];

		led->pled = pled;

		switch (pled->type) {
		case PCAP_LED0:
		case PCAP_LED1:
			led->ldev.max_brightness = 1;
			INIT_WORK(&led->work, pcap_led_led_work);
			break;
		case PCAP_BL0:
		case PCAP_BL1:
			led->ldev.max_brightness = PCAP_BL_MASK;
			INIT_WORK(&led->work, pcap_led_bl_work);
			break;
		default:
			dev_warn(led->ldev.dev, "unknown led type %d\n",
					pled->type);
			continue;
		}

		led->ldev.name = pled->name;
		led->ldev.brightness_set = pcap_led_set_brightness;
		led->ldev.flags |= LED_CORE_SUSPENDRESUME;
		led->pcap = dev_get_drvdata(pdev->dev.parent);

		err = led_classdev_register(&pdev->dev, &led->ldev);
		if (err) {
			dev_err(&pdev->dev, "couldn't register LED %s\n",
					pled->name);
			goto fail;
		}

		if (gpio_is_valid(pled->gpio)) {
			int gpio = pled->gpio;
			err = gpio_request(gpio, "PCAP LED");
			if (err) {
				dev_err(&pdev->dev,
					"couldn't request gpio %d\n", gpio);
				led_classdev_unregister(&led->ldev);
				cancel_work_sync(&led->work);
				goto fail;
			}
			gpio_direction_output(gpio, pled->gpio_invert);
		}
	}


	platform_set_drvdata(pdev, leds);
	return 0;

fail:
	if (i > 0)
		for (i = i - 1; i >= 0; i--) {
			if (gpio_is_valid(pdata->leds[i].gpio))
				gpio_free(pdata->leds[i].gpio);
			led_classdev_unregister(&leds[i].ldev);
			cancel_work_sync(&leds[i].work);
		}

	kfree(leds);

	return err;
}

static int __devexit pcap_led_remove(struct platform_device *pdev)
{
	int i;
	struct pcap_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pcap_led_data *leds = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		if (gpio_is_valid(pdata->leds[i].gpio))
			gpio_free(pdata->leds[i].gpio);
		led_classdev_unregister(&leds[i].ldev);
		cancel_work_sync(&leds[i].work);
	}

	kfree(leds);

	return 0;
}

static struct platform_driver pcap_led_driver = {
	.probe  = pcap_led_probe,
	.remove = __devexit_p(pcap_led_remove),
	.driver = {
		.name   = "pcap-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init pcap_led_init(void)
{
	return platform_driver_register(&pcap_led_driver);
}

static void __exit pcap_led_exit(void)
{
	platform_driver_unregister(&pcap_led_driver);
}

module_init(pcap_led_init);
module_exit(pcap_led_exit);

MODULE_AUTHOR("Daniel Ribeiro <drwyrm@gmail.com>");
MODULE_DESCRIPTION("PCAP LED driver");
MODULE_LICENSE("GPL");

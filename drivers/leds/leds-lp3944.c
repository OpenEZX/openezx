/*
 * leds-lp3944.c - driver for National Semiconductor LP3944 Funlight Chip
 *
 * Copyright (C) 2008 Antonio Ospite <ao2@openezx.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * I2C driver for National Semiconductor LP3944 Funlight Chip
 * http://www.national.com/pf/LP/LP3944.html
 *
 * This helper chip can drive up to 8 leds, with two programmable DIM modes;
 * it could even be used as a gpio expander but this driver assumes it is used
 * as a led controller.
 *
 * The DIM modes are used to set _blink_ patterns for leds, the pattern is
 * specified supplying two parameters:
 *   - period: from 0s to 1.6s
 *   - duty cycle: percentage of the period the led is on, from 0 to 100
 *
 * LP3944 can be found on Motorola A910 smartphone, where it drives the rgb
 * leds, the camera flash light and the displays backlights.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/leds-lp3944.h>

#if 0
/* Read Only Registers */
#define LP3944_REG_INPUT1	0x00	/* LEDs 0-7 InputRegister (Read Only) */
#define LP3944_REG_REGISTER1	0x01	/* None (Read Only) */
#endif

#define LP3944_REG_PSC0		0x02	/* Frequency Prescaler 0 (R/W) */
#define LP3944_REG_PWM0		0x03	/* PWM Register 0 (R/W) */
#define LP3944_REG_PSC1		0x04	/* Frequency Prescaler 1 (R/W) */
#define LP3944_REG_PWM1		0x05	/* PWM Register 1 (R/W) */
#define LP3944_REG_LS0		0x06	/* LEDs 0-3 Selector (R/W) */
#define LP3944_REG_LS1		0x07	/* LEDs 4-7 Selector (R/W) */

/* These registers are not used to control leds in LP3944
#define LP3944_REG_REGISTER8	0x08
#define LP3944_REG_REGISTER9	0x09
*/

#define LP3944_LED_STATUS_MASK	0x03

#define ldev_to_led(c)       container_of(c, struct lp3944_led, ldev)

/* Saved data */
struct lp3944_data {
	struct i2c_client *client;
	struct mutex lock;

	/* Only regs from 2 to 7 are used */
	u8 lp3944_regs[6];
};

static int lp3944_reg_read(struct i2c_client *client, unsigned reg,
			   unsigned *value);
static int lp3944_reg_write(struct i2c_client *client, unsigned reg,
			    unsigned value);

/**
 * Set the period in DIM status
 *
 * @client: the i2c client
 * @dim: either LP3944_DIM0 or LP3944_DIM1
 * @period: period of a blink, that is a on/off cycle, in 1/10 sec
 */
static int lp3944_dim_set_period(struct i2c_client *client, unsigned dim,
			  unsigned period)
{
	unsigned psc_reg;
	unsigned psc_value;
	int err;

	if (dim == LP3944_DIM0)
		psc_reg = LP3944_REG_PSC0;
	else if (dim == LP3944_DIM1)
		psc_reg = LP3944_REG_PSC1;
	else
		return -EINVAL;

	/* Convert period to Prescaler value */
	if (period > LP3944_PERIOD_MAX)
		return -EINVAL;

	psc_value = (period * 255) / LP3944_PERIOD_MAX;

	err = lp3944_reg_write(client, psc_reg, psc_value);

	return err;
}

/**
 * Set the duty cycle in DIM status
 *
 * @client: the i2c client
 * @dim: LP3944_DIM0 | LP3944_DIM1
 * @duty_cycle: percentage of a period in which a led is ON
 */
static int lp3944_dim_set_dutycycle(struct i2c_client *client, unsigned dim,
			     unsigned duty_cycle)
{
	unsigned pwm_reg;
	unsigned pwm_value;
	int err;

	if (dim == LP3944_DIM0)
		pwm_reg = LP3944_REG_PWM0;
	else if (dim == LP3944_DIM1)
		pwm_reg = LP3944_REG_PWM1;
	else
		return -EINVAL;

	/* Convert duty cycle to PWM value */
	if (duty_cycle > LP3944_DUTY_CYCLE_MAX)
		return -EINVAL;

	pwm_value = (duty_cycle * 255) / LP3944_DUTY_CYCLE_MAX;

	err = lp3944_reg_write(client, pwm_reg, pwm_value);

	return err;
}

/**
 * Set the led status
 *
 * @led: a lp3944_led structure as defined in leds-lp3944.h
 * @status: one of LP3944_LED_STATUS_OFF
 *                 LP3944_LED_STATUS_ON
 *                 LP3944_LED_STATUS_DIM0
 *                 LP3944_LED_STATUS_DIM1
 */
static int lp3944_led_set(struct lp3944_led *led, unsigned status)
{
	struct lp3944_data *data = i2c_get_clientdata(led->client);
	unsigned id = led->id;
	unsigned reg;
	unsigned val = 0;
	int err;

	switch (id) {
	case LP3944_LED0:
	case LP3944_LED1:
	case LP3944_LED2:
	case LP3944_LED3:
		reg = LP3944_REG_LS0;
		break;
	case LP3944_LED4:
	case LP3944_LED5:
	case LP3944_LED6:
	case LP3944_LED7:
		id -= LP3944_LED4;
		reg = LP3944_REG_LS1;
		break;
	default:
		return -EINVAL;
	}

	if (status > LP3944_LED_STATUS_DIM1)
		return -EINVAL;

	mutex_lock(&data->lock);
	lp3944_reg_read(led->client, reg, &val);

	val &= ~(LP3944_LED_STATUS_MASK << (id << 1));
	val |= (status << (id << 1));

	pr_debug("%s: led %d, status %d, val: 0x%02x\n",
		 __func__, id, status, val);

	/* set led status */
	err = lp3944_reg_write(led->client, reg, val);
	mutex_unlock(&data->lock);

	return err;
}

static int lp3944_reg_read(struct i2c_client *client, unsigned reg,
			   unsigned *value)
{
	int tmp;

	tmp = i2c_smbus_read_byte_data(client, reg);
	if (tmp < 0)
		return -EINVAL;

	*value = tmp;

	return 0;
}

static int lp3944_reg_write(struct i2c_client *client, unsigned reg,
			    unsigned value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static void lp3944_led_set_brightness(struct led_classdev *led_cdev,
				  enum led_brightness value)
{
	struct lp3944_led *led = ldev_to_led(led_cdev);

	/*
	 * value arg interpretation:
	 *   0 = led OFF
	 *   1 = led ON (failsafe default)
	 *   2 = led in DIM0 mode
	 *   3 = led in DIM1 mode
	 */
	if (value > 3)
		value = 1;

	if (led->type == LP3944_LED_TYPE_LED_INVERTED && value < 2)
		value = 1 - value;

	pr_debug("%s: %d\n", led_cdev->name, value);
	lp3944_led_set(led, value);
}

static int lp3944_configure(struct i2c_client *client,
			    struct lp3944_data *data,
			    struct lp3944_platform_data *pdata)
{
	int i, err = 0;

	for (i = 0; i < pdata->dims_size; i++) {
		lp3944_dim_set_period(client, i, pdata->dims[i].period);
		lp3944_dim_set_dutycycle(client, i, pdata->dims[i].dutycycle);
	}

	for (i = 0; i < pdata->leds_size; i++) {
		struct lp3944_led *pled = &pdata->leds[i];
		pled->client = client;
		pled->id = i;

		switch (pled->type) {

		case LP3944_LED_TYPE_NONE:
			break;

		case LP3944_LED_TYPE_LED:
		case LP3944_LED_TYPE_LED_INVERTED:
			pled->ldev.name = pled->name;
			pled->ldev.brightness_set = lp3944_led_set_brightness;
			err = led_classdev_register(&client->dev, &pled->ldev);
			if (err < 0) {
				dev_err(&client->dev,
					"couldn't register LED %s\n",
					pled->name);
				goto exit;
			}

			/* Set the default led status */
			if (pled->status == LP3944_LED_STATUS_OFF)
				break;
			err = lp3944_led_set(pled, pled->status);
			if (err < 0) {
				dev_err(&client->dev,
					"couldn't set STATUS %d\n",
					pled->status);
				goto exit;
			}
			break;
		}
	}
	return 0;

exit:
	if (i > 0)
		for (i = i - 1; i >= 0; i--)
			switch (pdata->leds[i].type) {
			case LP3944_LED_TYPE_NONE:
				break;
			case LP3944_LED_TYPE_LED:
			case LP3944_LED_TYPE_LED_INVERTED:
				led_classdev_unregister(&pdata->leds[i].ldev);
				break;
			}

	return err;
}

static int __devinit lp3944_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct lp3944_platform_data *lp3944_pdata = client->dev.platform_data;
	struct lp3944_data *data;

	if (lp3944_pdata == NULL) {
		printk(KERN_ERR "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	/* Let's see whether this adapter can support what we need. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "%s: insufficient functionality!\n", __func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct lp3944_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	dev_info(&client->dev, "lp3944 enabled\n");

	lp3944_configure(client, data, lp3944_pdata);
	return 0;
}

static int __devexit lp3944_remove(struct i2c_client *client)
{
	struct lp3944_platform_data *pdata = client->dev.platform_data;
	struct lp3944_data *data = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < pdata->leds_size; i++)
		switch (pdata->leds[i].type) {
		case LP3944_LED_TYPE_NONE:
			break;
		case LP3944_LED_TYPE_LED:
		case LP3944_LED_TYPE_LED_INVERTED:
			led_classdev_unregister(&pdata->leds[i].ldev);
			break;
		}

	kfree(data);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int lp3944_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lp3944_data *data = i2c_get_clientdata(client);
	int reg;

	dev_dbg(&client->dev, "lp3944_suspend\n");

	/* 6 registerst to save, from 0x2 to 0x7 */
	for (reg = LP3944_REG_PSC0; reg < LP3944_REG_LS1; reg++)
		data->lp3944_regs[reg - 2] =
		    i2c_smbus_read_byte_data(client, reg);

	return 0;
}

static int lp3944_resume(struct i2c_client *client)
{
	struct lp3944_data *data = i2c_get_clientdata(client);
	int reg;

	dev_dbg(&client->dev, "lp3944_resume\n");

	for (reg = LP3944_REG_PSC0; reg < LP3944_REG_LS1; reg++)
		i2c_smbus_write_byte_data(client, reg,
					  data->lp3944_regs[reg - 2]);

	return 0;
}
#else

#define lp3944_suspend NULL
#define lp3944_resume NULL

#endif /* CONFIG_PM */

/* lp3944 i2c driver struct */
static const struct i2c_device_id lp3944_id[] = {
	{"lp3944", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lp3944_id);

static struct i2c_driver lp3944_driver = {
	.driver   = {
		   .name = "lp3944",
	},
	.probe    = lp3944_probe,
	.remove   = __devexit_p(lp3944_remove),
	.suspend  = lp3944_suspend,
	.resume   = lp3944_resume,
	.id_table = lp3944_id,
};

static int __init lp3944_module_init(void)
{
	return i2c_add_driver(&lp3944_driver);
}

static void __exit lp3944_module_exit(void)
{
	i2c_del_driver(&lp3944_driver);
}

module_init(lp3944_module_init);
module_exit(lp3944_module_exit);

MODULE_AUTHOR("Antonio Ospite <ao2@openezx.org>");
MODULE_DESCRIPTION("LP3944 Fun Light Chip");
MODULE_LICENSE("GPL");

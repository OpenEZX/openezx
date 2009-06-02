#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>

static struct work_struct bat_work;

struct pcap_bat_struct {
	int status;
	struct power_supply psy;

	struct mutex work_lock; /* protects data */

	int max;
	int min;
	int now;
	int temp;
};

static struct pcap_bat_struct pcap_bat;
struct timer_list bat_timer;
struct pcap_chip *pcap;
static struct regulator *ac_draw;

static int pcap_bat_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	int ret = 0;
	struct pcap_bat_struct *bat = container_of(psy, struct pcap_bat_struct, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
                val->intval = bat->max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bat->max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->min;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat->temp;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void pcap_bat_external_power_changed(struct power_supply *psy)
{
	schedule_work(&bat_work);
}

static void pcap_bat_update(struct pcap_bat_struct *bat)
{
	int old;
	int ret;
	struct power_supply *psy = &bat->psy;
	char ch[2];
	short adc[2];

	mutex_lock(&bat->work_lock);

	ch[0] = PCAP_ADC_CH_BATT;
	ch[1] =	PCAP_ADC_CH_TEMPERATURE;

	ret = pcap_adc_sync(pcap,PCAP_ADC_BANK_0,
			PCAP_ADC_T_NOW, ch, adc);

	bat->now = adc[0];
	bat->temp = adc[1];

	old = bat->status;

	/*
        if (regulator_is_enabled(ac_draw)) {
		if (bat->now < bat->max)
			bat->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			bat->status = POWER_SUPPLY_STATUS_FULL;
	} else {
		bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}*/

	bat->status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (old != bat->status)
		power_supply_changed(psy);

	mutex_unlock(&bat->work_lock);

	mod_timer(&bat_timer, jiffies + HZ);
}

static void pcap_bat_work(struct work_struct *work)
{
	pcap_bat_update(&pcap_bat);
}

static void bat_timer_fn(unsigned long data)
{
	schedule_work(&bat_work);
}


static enum power_supply_property pcap_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
};

static struct pcap_bat_struct pcap_bat = {
	.status = POWER_SUPPLY_STATUS_DISCHARGING,
	.psy = {
		.name		= "main-battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= pcap_bat_props,
		.num_properties	= ARRAY_SIZE(pcap_bat_props),
		.get_property	= pcap_bat_get_property,
		.external_power_changed = pcap_bat_external_power_changed,
		.use_for_apm	= 1,
	},
	.max = 720,
	.min = 390,
};


static int pcap_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	flush_scheduled_work();
	return 0;
}

static int pcap_bat_resume(struct platform_device *dev)
{
	schedule_work(&bat_work);
	return 0;
}

static int __devinit pcap_bat_probe(struct platform_device *pdev)
{
	int ret;
	static struct device *dev;

	dev = &pdev->dev;
	pcap = platform_get_drvdata(pdev);


        ac_draw = regulator_get(dev, "ac_draw");

        if (!ac_draw) {
		printk("couldn't get ac_draw regulator\n");
                return -1;
        }


	mutex_init(&pcap_bat.work_lock);

	ret = power_supply_register(&pdev->dev, &pcap_bat.psy);
	if (ret)
		goto err_psy_reg;

	INIT_WORK(&bat_work, pcap_bat_work);

	init_timer(&bat_timer);
	bat_timer.function = bat_timer_fn;

	schedule_work(&bat_work);
        return 0;

err_psy_reg:
	flush_scheduled_work();

	return ret;
}

static int __devexit pcap_bat_remove(struct platform_device *dev)
{

	power_supply_unregister(&pcap_bat.psy);

	flush_scheduled_work();


	return 0;
}

static struct platform_driver pcap_bat_driver = {
	.driver.name	= "pcap-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= pcap_bat_probe,
	.remove		= __devexit_p(pcap_bat_remove),
	.suspend	= pcap_bat_suspend,
	.resume		= pcap_bat_resume,
};

static int __init pcap_bat_init(void)
{
	return platform_driver_register(&pcap_bat_driver);
}

static void __exit pcap_bat_exit(void)
{
	platform_driver_unregister(&pcap_bat_driver);
}

module_init(pcap_bat_init);
module_exit(pcap_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ilya Petrov <ilya.muromec@gmail.com>");
MODULE_DESCRIPTION("EZX battery driver");

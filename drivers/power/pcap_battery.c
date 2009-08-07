#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/regulator/consumer.h>

#define PCAP_ADC_TO_mV(x)	(((x) * 3) + 2000)
#define PCAP_ADC_TO_mA(x)	((x) < 178 ? 0 : ((x) - 178) * 3165 / 1000)
#define PCAP_ADC_TO_TEMP(x)	(((1024 - (x)) * 1084 / 1000) - 180)

struct pcap_bat_struct {
	int status;
	struct power_supply psy;

	int max;
	int min;
	int now;
	int temp;
};

static struct pcap_bat_struct pcap_bat;
struct pcap_chip *pcap;
static struct regulator *ac_draw;
static void pcap_bat_update(struct pcap_bat_struct *bat);

static int pcap_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	struct pcap_bat_struct *bat = container_of(psy,
						struct pcap_bat_struct, psy);

	pcap_bat_update(&pcap_bat);

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
	pcap_bat_update(&pcap_bat);
}

static void pcap_bat_update(struct pcap_bat_struct *bat)
{
	int old;
	int ret;
	struct power_supply *psy = &bat->psy;
	char ch[2];
	short adc[2];

	ch[0] = PCAP_ADC_CH_BATT;
	ch[1] =	PCAP_ADC_CH_TEMPERATURE;

	ret = pcap_adc_sync(pcap, PCAP_ADC_BANK_0, 0, ch, adc);

	bat->now = PCAP_ADC_TO_mV(adc[0]);
	bat->temp = PCAP_ADC_TO_TEMP(adc[1]);

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
	.max = 4100,
	.min = 2000,
};

static int __devinit pcap_bat_probe(struct platform_device *pdev)
{
	static struct device *dev;

	dev = &pdev->dev;
	pcap = dev_get_drvdata(pdev->dev.parent);


	ac_draw = regulator_get(dev, "ac_draw");

	if (!ac_draw) {
		printk("couldn't get ac_draw regulator\n");
		return -1;
	}

	return power_supply_register(&pdev->dev, &pcap_bat.psy);
}

static int __devexit pcap_bat_remove(struct platform_device *dev)
{
	power_supply_unregister(&pcap_bat.psy);

	return 0;
}

static struct platform_driver pcap_bat_driver = {
	.driver.name	= "pcap-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= pcap_bat_probe,
	.remove		= __devexit_p(pcap_bat_remove),
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

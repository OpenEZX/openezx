#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/mfd/ezx-eoc.h>
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
	struct eoc_chip *eoc;
	struct pcap_chip *pcap;
	struct regulator *reg;
	int vbus_irq;
	int charger_irq;
	int id_irq;
	struct mutex lock;
};

static struct pcap_bat_struct pcap_bat;
static void pcap_bat_update(struct pcap_bat_struct *bat);
static irqreturn_t eoc_batpon_detect(int irq, void *_bat);

static int pcap_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	struct pcap_bat_struct *bat = container_of(psy,
						struct pcap_bat_struct, psy);

	pcap_bat_update(bat);

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
	short adc[7];

	if(! bat->pcap )
		return;


	mutex_lock(&bat->lock);

	ch[0] = PCAP_ADC_CH_BATT;
	ch[1] =	PCAP_ADC_CH_TEMPERATURE;

	ret = pcap_adc_sync(bat->pcap, PCAP_ADC_BANK_0, 0, ch, adc);

	bat->now = PCAP_ADC_TO_mV(adc[0]);
	bat->temp = PCAP_ADC_TO_TEMP(adc[1]);

	mutex_unlock(&bat->lock);

	old = bat->status;

	if(IS_ERR(bat->reg)) {
		bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

        if (regulator_is_enabled(bat->reg)) {
		if (bat->now < bat->max)
			bat->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			bat->status = POWER_SUPPLY_STATUS_FULL;
	} else {
		bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (old != bat->status)
		power_supply_changed(psy);
}

static void eoc_charge_start(struct pcap_bat_struct *bat)
{
	int cr, id;

	if (IS_ERR(bat->reg))
		return;

	if(!regulator_is_enabled(bat->reg)) {
		printk("enable charge regulator\n");
		regulator_enable(bat->reg);
	} else {
		printk("regulator already enabled\n");
	}

	/* FIXME: bitwise mask and shift */
	id = (bat->eoc->sense & EOC_SENSE_ID_FLOAT) |
		(bat->eoc->sense & EOC_SENSE_ID_GROUND);

	switch (id) {
		case 0:
		 printk("looks like charger\n");
		 cr = 900000;
		 break;
		default:
		 printk("looks like usb.\n");
		 cr = 300000;
	}

	regulator_set_current_limit(bat->reg, cr, cr);

	eoc_batpon_detect(0, bat);
}

static void eoc_charge_stop(struct pcap_bat_struct *bat)
{

	if(IS_ERR(bat->reg))
		return;

	if(!regulator_is_enabled(bat->reg)) {
		printk("regulator already disabled\n");
		return;
	} else {
		printk("disable charge regulator\n");
	}

	regulator_set_current_limit(bat->reg, 0, 0);
	regulator_disable(bat->reg);

	eoc_batpon_detect(0, bat);
}

static irqreturn_t eoc_charge_vbus(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;
	int s = bat->eoc->sense;

	printk("vbus detected ");

	if(s & EOC_SENSE_VBUS_3V4)
		printk("3.4V ");
	else
		printk("not 3.4V ");

	if(s & EOC_SENSE_VBUS_4V4) {
		printk("4.4V\n");
		eoc_charge_start(bat);
	} else if (s & EOC_SENSE_VBUS_2V) {
		printk("2V\n");
	} else if (s & EOC_SENSE_VBUS_0V8) {
		printk("0.8V\n");
	} else {
		printk("0V\n");
		eoc_charge_stop(bat);
	}

	pcap_bat_update(bat);
	printk("battery voltage: %d\n",bat->now);

	return IRQ_HANDLED;
}

static irqreturn_t eoc_charge_id(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;

	int id = (bat->eoc->sense & EOC_SENSE_ID_FLOAT) |
		(bat->eoc->sense & EOC_SENSE_ID_GROUND);

	switch (id) {
		case 0:
		 printk("looks like charger\n");
		 eoc_charge_start(bat);
		 break;
	}
 
	return IRQ_HANDLED;
}

static irqreturn_t eoc_reverse_detect(int irq, void *_bat)
{
	printk("reverse charge detected!\n");

	return IRQ_HANDLED;
}

static irqreturn_t eoc_batpon_detect(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;

	if(bat->eoc->sense & EOC_SENSE_BATTERY_PON) {
          printk("not discharged.\n");
	  bat->eoc->power0 &= ~EOC_POWER0_FET_SOFT;
	}else {
          printk("discharged!\n");
	  bat->eoc->power0 |= EOC_POWER0_FET_SOFT;
	  bat->eoc->power0 &= ~EOC_POWER0_FET_STATE;
	}

	schedule_work(&bat->eoc->power0_work);


	return IRQ_HANDLED;
}

static irqreturn_t eoc_se1_detect(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;

	if(bat->eoc->sense & EOC_SENSE_SE1)
		printk("SE1\n");
	else
		printk("no SE1\n");

	return IRQ_HANDLED;
}

static irqreturn_t eoc_cccv_detect(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;

	if(bat->eoc->sense & EOC_SENSE_CC_CV)
		printk("current\n");
	else
		printk("voltage\n");

	return IRQ_HANDLED;
}

static irqreturn_t eoc_current_detect(int irq, void *_bat)
{
	struct pcap_bat_struct *bat = _bat;

	pcap_bat_update(bat);

	if(bat->eoc->sense & EOC_SENSE_CURRENT) {
		printk("current appeared\n");
	} else {
		printk("current disappeared\n");

		if(bat->now > pcap_bat.max) {
			printk("charge complete\n");
			eoc_charge_stop(bat);
		}
	}

	return IRQ_HANDLED;
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
	.max = 4200,
	.min = 2000,
};

static int __devinit pcap_bat_probe(struct platform_device *pdev)
{
	static struct device *dev;

	dev = &pdev->dev;
	pcap_bat.pcap = dev_get_drvdata(pdev->dev.parent);
	mutex_init(&pcap_bat.lock);

	return power_supply_register(&pdev->dev, &pcap_bat.psy);
}

static int __devexit pcap_bat_remove(struct platform_device *dev)
{
	power_supply_unregister(&pcap_bat.psy);

	return 0;
}

static int __devinit eoc_charge_probe(struct platform_device *pdev)
{
	static struct device *dev;
	int vbus_irq, id_irq, charger_irq, se1_irq,
	    rvrs_irq, cccv_irq, curr_irq, battpon_irq;

	dev = &pdev->dev;
	pcap_bat.eoc = platform_get_drvdata(pdev);
	pcap_bat.reg = regulator_get(dev, "ac_draw");

	vbus_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_VBUS);
	id_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_ID);
	charger_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_VBUS_3V4);
	rvrs_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_RVRS_CHRG);

	se1_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_SE1);
	cccv_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_CC_CV);
	curr_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_CHRG_CURR);
	battpon_irq = eoc_to_irq(pcap_bat.eoc, EOC_IRQ_BATTPON);


	/* cable can be connected before boot
	 * to check cable state and start charge process
	 * check SENSE value before enabling IRQ */
	eoc_charge_vbus(0, &pcap_bat);
	eoc_batpon_detect(0, &pcap_bat);

	/* This should detect charger (3.4V) */
	request_irq(charger_irq, eoc_charge_vbus, 0,
			"charger", &pcap_bat);

	/* This shoud indicate VBUS change */
	request_irq(vbus_irq, eoc_charge_vbus, 0,
			"vbus", &pcap_bat);

	/* This indicates id pin state change */
	request_irq(id_irq, eoc_charge_id, 0,
			"charger id", &pcap_bat);

	/* This indicates innormal charge */
	request_irq(rvrs_irq, eoc_reverse_detect, 0,
			"reverse", &pcap_bat);

	/* This indicates single endeded 1 state change */
	request_irq(se1_irq, eoc_se1_detect, 0,
			"cccv", &pcap_bat);

	/* This indicates single endeded 1 state change */
	request_irq(cccv_irq, eoc_cccv_detect, 0,
			"cccv", &pcap_bat);

	/* This indicates battery charge current change */
	request_irq(curr_irq, eoc_current_detect, 0,
			"current",  &pcap_bat);

	request_irq(battpon_irq, eoc_batpon_detect, 0,
			"battpon", &pcap_bat);

	return 0;
}

static int __devexit eoc_charge_remove(struct platform_device *dev)
{

	pcap_bat.reg = NULL; // is this correct?
	return 0;
}

static struct platform_driver pcap_bat_driver = {
	.driver.name	= "pcap-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= pcap_bat_probe,
	.remove		= __devexit_p(pcap_bat_remove),
};

static struct platform_driver eoc_charge_driver = {
	.driver.name	= "eoc_charger",
	.driver.owner	= THIS_MODULE,
	.probe		= eoc_charge_probe,
	.remove		= __devexit_p(eoc_charge_remove),
};

static int __init pcap_bat_init(void)
{
	pcap_bat.reg = NULL;
	pcap_bat.eoc = NULL;
	pcap_bat.pcap = NULL;

	platform_driver_register(&eoc_charge_driver);
	return platform_driver_register(&pcap_bat_driver);
}

static void __exit pcap_bat_exit(void)
{
	platform_driver_unregister(&eoc_charge_driver);
	platform_driver_unregister(&pcap_bat_driver);
}

module_init(pcap_bat_init);
module_exit(pcap_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ilya Petrov <ilya.muromec@gmail.com>");
MODULE_DESCRIPTION("EZX battery driver");

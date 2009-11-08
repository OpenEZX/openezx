
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ezx-eoc.h>
#include <linux/workqueue.h>


static int eoc_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	int mask;
	int charge_current, setup;
	struct eoc_chip *eoc = rdev_get_drvdata(rdev);

	if (max_uA < 100000)
		charge_current = 1;
	else if (max_uA > 1300000)
		charge_current = 14;
	else
		charge_current = max_uA / 100000;

	setup = (charge_current << EMU_ICHRG_SHIFT) & EMU_ICHRG_MASK;
	if (charge_current != 0)
		mask = EMU_ICHRG_MASK | EMU_ICHRG_TR_MASK;
	else
		mask = EMU_ICHRG_MASK;

        int pre= eoc->power0;

	eoc->power0 &= ~mask;
	eoc->power0 |= setup & mask;

        printk("eoc set limit: %x, %x, %x -> %x, %d\n",
            setup, mask, pre, eoc->power0, max_uA);

	schedule_work(&eoc->power0_work);

	return 0;
}

static int eoc_get_current_limit(struct regulator_dev *rdev)
{
	struct eoc_chip *eoc = rdev_get_drvdata(rdev);
	int value = eoc->power0;

	value &= EMU_ICHRG_MASK;
	value >>= EMU_ICHRG_SHIFT;

	return value * 1000;
}

static int eoc_charger_enable(struct regulator_dev *rdev)
{
	struct eoc_chip *eoc = rdev_get_drvdata(rdev);

	eoc->power0 &= ~EMU_VCHRG_MASK;
	eoc->power0 |= EMU_VCHRG_MASK & 7;

        printk("eoc charge enable: %x, %x\n",
            EMU_VCHRG_MASK, eoc->power0);

	schedule_work(&eoc->power0_work);

	return 0;
}

static int eoc_charger_disable(struct regulator_dev *rdev)
{
	struct eoc_chip *eoc = rdev_get_drvdata(rdev);

        printk("eoc disable!\n");

	eoc->power0 &= ~EMU_VCHRG_MASK;
	schedule_work(&eoc->power0_work);

	return 0;
}

static int eoc_charger_is_enabled(struct regulator_dev *rdev)
{
	struct eoc_chip *eoc = rdev_get_drvdata(rdev);
	int value = eoc->power0;

	value &= EMU_VCHRG_MASK;

	return value >= 3;
}

static int eoc_list_voltage(struct regulator_dev *rdev, unsigned selector) {
        return 0;
}

static int eoc_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV){
        return 0;
}

static int eoc_get_voltage(struct regulator_dev *rdev) {
        struct eoc_chip *eoc = rdev_get_drvdata(rdev);
        
        return eoc->power0 & EMU_VCHRG_MASK;
}


static struct regulator_ops eoc_regulators_ops_voltage = {
	.set_voltage       = eoc_set_voltage,
	.get_voltage       = eoc_get_voltage,
        .list_voltage      = eoc_list_voltage,
	.is_enabled        = eoc_charger_is_enabled,
};

static struct regulator_ops eoc_regulators_ops_curr = {
	.set_current_limit = eoc_set_current_limit,
	.get_current_limit = eoc_get_current_limit,
	.enable            = eoc_charger_enable,
	.disable           = eoc_charger_disable,
	.is_enabled        = eoc_charger_is_enabled,
};

static struct regulator_desc eoc_regulator_descs[] = {
  {
	.name  = "eoc_charger",
	.ops   = &eoc_regulators_ops_curr,
	.type  = REGULATOR_CURRENT,
  },
  {     .name = "eoc_charger_voltage",
        .ops  = &eoc_regulators_ops_voltage,
        .type = REGULATOR_VOLTAGE,
  },
};

static int eoc_reg_probe(struct platform_device *pdev)
{
	struct regulator_init_data	*initdata;
	struct regulator_dev		*rdev;

	void *eoc = dev_get_drvdata(pdev->dev.parent);

        printk("regulator id: %d\n", pdev->id);

        initdata = pdev->dev.platform_data;
	if (!initdata)
		return -EINVAL;

	rdev = regulator_register(&eoc_regulator_descs[pdev->id], 
			&pdev->dev, initdata, eoc
	);

	if (IS_ERR(rdev)) {
		printk("can't register eoc regularor");
		return PTR_ERR(rdev);
	}
	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int __devexit eoc_reg_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

MODULE_ALIAS("platform:eoc_reg");

static struct platform_driver eoc_reg_driver = {
	.probe		= eoc_reg_probe,
	.remove		= __devexit_p(eoc_reg_remove),
	.driver.name	= "eoc_reg",
	.driver.owner	= THIS_MODULE,
};

static int __init eoc_reg_init(void)
{
	return platform_driver_register(&eoc_reg_driver);
}
module_init(eoc_reg_init);

static void __exit eoc_reg_exit(void)
{
	platform_driver_unregister(&eoc_reg_driver);
}
module_exit(eoc_reg_exit)

MODULE_DESCRIPTION("EOC regulator driver");
MODULE_LICENSE("GPL");

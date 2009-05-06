/*
 * PCAP2 Regulator Driver
 *
 * Copyright (c) 2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ezx-pcap.h>

struct pcap_regulator {
	u8 reg;
	u8 en;
	u8 index;
	u8 stby;
	u8 lowpwr;
	u8 n_voltages;
	const u16 *voltage_table;
};

static const u16 V1_table[] = {
	2775, 1275, 1600, 1725, 1825, 1925, 2075, 2275,
};

static const u16 V2_table[] = {
	2500, 2775,
};

static const u16 V3_table[] = {
	1075, 1275, 1550, 1725, 1876, 1950, 2075, 2275,
};

static const u16 V48_table[] = {
	1275, 1550, 1725, 1875, 1950, 2075, 2275, 2775,
};

static const u16 V5_table[] = {
	1875, 2275, 2475, 2775,
};

static const u16 V6_table[] = {
	2475, 2775,
};

static const u16 V7_table[] = {
	1875, 2775,
};

static const u16 V9_table[] = {
	1575, 1875, 2475, 2775,
};

static const u16 V10_table[] = {
	5000,
};

static const u16 VSIM_table[] = {
	1875, 3000,
};

static const u16 VSIM2_table[] = {
	1875,
};

static const u16 VVIB_table[] = {
	1300, 1800, 2000, 3000,
};

static const u16 VAUX12_table[] = {
	1875, 2475, 2775, 3000,
};

static const u16 VAUX3_table[] = {
	1200, 1200, 1200, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800,
	3000, 3200, 3400, 3600,
};

static const u16 VAUX4_table[] = {
	1800, 1800, 3000, 5000,
};

static const u16 SW12_table[] = {
	900, 950, 1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450,
	1500, 1600, 1875, 2250,
};

static const u16 SW3_table[] = {
	4000, 4500, 5000, 5500,
};

#define NA 0xff

static struct pcap_regulator vreg_table[] = {
	/*	    REGISTER	      EN  LEV  STBY LOWP NV  V.TABLE       */
	[V1]	= { PCAP_REG_VREG1,   1,  2,   18,  0,   8,  V1_table,     },
	[V2]	= { PCAP_REG_VREG1,   5,  6,   19,  22,  2,  V2_table,     },
	[V3]	= { PCAP_REG_VREG1,   7,  8,   20,  23,  8,  V3_table,     },
	[V4]	= { PCAP_REG_VREG1,   11, 12,  21,  24,  8,  V48_table,    },
	/* V5 STBY and LOWP are on PCAP_REG_VREG2 */
	[V5]	= { PCAP_REG_VREG1,   15, 16,  12,  19,  4,  V5_table,     },

	[V6]	= { PCAP_REG_VREG2,   1,  2,   14,  20,  2,  V6_table,     },
	[V7]	= { PCAP_REG_VREG2,   3,  4,   15,  21,  2,  V7_table,     },
	[V8]	= { PCAP_REG_VREG2,   5,  6,   16,  22,  8,  V48_table,    },
	[V9]	= { PCAP_REG_VREG2,   9,  10,  17,  23,  4,  V9_table,     },
	[V10]	= { PCAP_REG_VREG2,   10, NA,  18,  24,  1,  V10_table,    },

	[VAUX1]	= { PCAP_REG_AUXVREG, 1,  2,   22,  23,  4,  VAUX12_table, },
	[VAUX2]	= { PCAP_REG_AUXVREG, 4,  5,   NA,  NA,  4,  VAUX12_table, },
	[VAUX3]	= { PCAP_REG_AUXVREG, 7,  8,   NA,  NA,  16, VAUX3_table,  },
	[VAUX4]	= { PCAP_REG_AUXVREG, 12, 13,  NA,  NA,  4,  VAUX4_table,  },
	[VSIM]	= { PCAP_REG_AUXVREG, 17, 18,  NA,  NA,  2,  VSIM_table,   },
	[VSIM2]	= { PCAP_REG_AUXVREG, 16, NA,  NA,  NA,  1,  VSIM2_table,  },
	[VVIB]	= { PCAP_REG_AUXVREG, 19, 20,  NA,  NA,  4,  VVIB_table,   },

	[SW1]	= { PCAP_REG_SWCTRL,  1,  2,   NA,  NA,  16, SW12_table,   },
	[SW2]	= { PCAP_REG_SWCTRL,  6,  7,   NA,  NA,  16, SW12_table,   },
	[SW3]	= { PCAP_REG_SWCTRL,  11, 12,  NA,  NA,  4,  SW3_table,    },

	[SW1S]	= { PCAP_REG_LOWPWR,  NA, 12,  NA,  NA,  16, SW12_table,   },
	[SW2S]	= { PCAP_REG_LOWPWR,  NA, 20,  NA,  NA,  0,  SW12_table,   },
};

static int pcap_regulator_set_voltage(struct regulator_dev *rdev,
						int min_uv, int max_uv)
{
	u32 tmp;
	u8 bits;
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	for (bits = 0; bits < vreg->n_voltages; bits++) {
		int uv = vreg->voltage_table[bits] * 1000;
		if (min_uv <= uv && uv <= max_uv) {
			ezx_pcap_read(vreg->reg, &tmp);
			tmp |= bits << vreg->index;
			ezx_pcap_write(vreg->reg, tmp);
			return 0;
		}
	}

	return -EINVAL;
}

static int pcap_regulator_get_voltage(struct regulator_dev *rdev)
{
	u32 tmp;
	int mv;
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	if (vreg->n_voltages == 1)
		return vreg->voltage_table[0] * 1000;

	ezx_pcap_read(vreg->reg, &tmp);
	tmp = ((tmp >> vreg->index) & (vreg->n_voltages - 1));
	mv = vreg->voltage_table[tmp];

	return mv * 1000;
}

static int pcap_regulator_enable(struct regulator_dev *rdev)
{
	u32 tmp;
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	if (vreg->en == NA)
		return -EINVAL;

	ezx_pcap_read(vreg->reg, &tmp);
	tmp |= (1 << vreg->en);
	ezx_pcap_write(vreg->reg, tmp);

	return 0;
}

static int pcap_regulator_disable(struct regulator_dev *rdev)
{
	u32 tmp;
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	if (vreg->en == NA)
		return -EINVAL;

	ezx_pcap_read(vreg->reg, &tmp);
	tmp &= ~(1 << vreg->en);
	ezx_pcap_write(vreg->reg, tmp);

	return 0;
}

static int pcap_regulator_is_enabled(struct regulator_dev *rdev)
{
	u32 tmp;
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	if (vreg->en == NA)
		return -EINVAL;

	ezx_pcap_read(vreg->reg, &tmp);
	return ((tmp >> vreg->en) & 1);
}

static int pcap_regulator_list_voltage(struct regulator_dev *rdev,
							unsigned int index)
{
	struct pcap_regulator *vreg = &vreg_table[rdev_get_id(rdev)];

	return vreg->voltage_table[index] * 1000;
}

static struct regulator_ops pcap_regulator_ops = {
	.list_voltage	= pcap_regulator_list_voltage,
	.set_voltage	= pcap_regulator_set_voltage,
	.get_voltage	= pcap_regulator_get_voltage,
	.enable		= pcap_regulator_enable,
	.disable	= pcap_regulator_disable,
	.is_enabled	= pcap_regulator_is_enabled,
};

#define VREG(vreg)						\
[vreg]	= {							\
		.name		= "vreg",			\
		.id		= vreg,				\
		.ops		= &pcap_regulator_ops,		\
		.type		= REGULATOR_VOLTAGE,		\
		.owner		= THIS_MODULE,			\
	}

static struct regulator_desc pcap_regulators[] = {
	VREG(V1), VREG(V2), VREG(V3), VREG(V4), VREG(V5), VREG(V6), VREG(V7),
	VREG(V8), VREG(V9), VREG(V10), VREG(VAUX1), VREG(VAUX2), VREG(VAUX3),
	VREG(VAUX4), VREG(VSIM), VREG(VVIB), VREG(SW1), VREG(SW2),
};

static int __devinit pcap_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	rdev = regulator_register(&pcap_regulators[pdev->id], &pdev->dev,
				pdev->dev.platform_data, NULL);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	return 0;
}

static int __devexit pcap_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

static struct platform_driver pcap_regulator_driver = {
	.driver = {
		.name = "pcap_regulator",
	},
	.probe = pcap_regulator_probe,
	.remove = __devexit_p(pcap_regulator_remove),
};

static int __init pcap_regulator_init(void)
{
	return platform_driver_register(&pcap_regulator_driver);
}

static void __exit pcap_regulator_exit(void)
{
	platform_driver_unregister(&pcap_regulator_driver);
}

module_init(pcap_regulator_init);
module_exit(pcap_regulator_exit);

MODULE_AUTHOR("Daniel Ribeiro <drwyrm@gmail.com>");
MODULE_DESCRIPTION("PCAP Regulator Driver");
MODULE_LICENSE("GPL");

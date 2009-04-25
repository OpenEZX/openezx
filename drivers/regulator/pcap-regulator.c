/*
 * PCAP Regulator Driver
 *
 * Copyright (c) 2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#error Please dont use this driver yet :)

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <linux/mfd/ezx-pcap.h>

#define PCAP_REGULATOR(_id)			\
	{					\
		.name	= "_id",		\
		.id	= _id,			\
		.ops	= &pcap_regulator_ops,	\
		.type	= REGULATOR_VOLTAGE,	\
		.owner	= THIS_MODULE,		\
	}

static u8 vreg_table[][6] = {
/*	    REGISTER		EN	INDEX	MASK	STBY	LOWPWR*/
[V1]	= { PCAP_REG_VREG1,	1,	2,	0x7,	18,	0,    },
[V2]	= { PCAP_REG_VREG1,	5,	6,	0x1,	19,	22,   },
[V3]	= { PCAP_REG_VREG1,	7,	8,	0x7,	20,	23,   },
[V4]	= { PCAP_REG_VREG1,	11,     12,     0x7,    21,     24,   },
[V5]	= { PCAP_REG_VREG1,	15,     16,     0x3,    0xff,   0xff, },
[V6]	= { PCAP_REG_VREG2,	1,      0xff,   0x0,    0xff,   0xff, },
/* FIXME: I have no idea of V7-V10 bits -WM */
[V7]	= { PCAP_REG_VREG2,	0xff,   0xff,   0x0,    0xff,   0xff, },
[V8]	= { PCAP_REG_VREG2,	0xff,   0xff,   0x0,    0xff,   0xff, },
[V9]	= { PCAP_REG_VREG2,	0xff,   0xff,   0x0,    0xff,   0xff, },
[V10]	= { PCAP_REG_VREG2,	0xff,   0xff,   0x0,    0xff,   0xff, },
[VAUX1]	= { PCAP_REG_AUXVREG,	1,      2,      0x3,    22,     23,   },
[VAUX2]	= { PCAP_REG_AUXVREG,	4,      5,      0x3,    0,      1,    },
[VAUX3]	= { PCAP_REG_AUXVREG,	7,      8,      0xf,    2,      3,    },
[VAUX4]	= { PCAP_REG_AUXVREG,	12,     13,     0x3,    4,      5,    },
[VSIM]	= { PCAP_REG_AUXVREG,	17,     18,     0x1,    0xff,   6,    },
[VSIM2]	= { PCAP_REG_AUXVREG,	16,     0xff,   0x0,    0xff,   7,    },
[VVIB]	= { PCAP_REG_AUXVREG,	19,     20,     0x3,    0xff,   0xff, },
[VC]	= { PCAP_REG_AUXVREG,	0xff,   0xff,   0x0,    24,     0xff, },
[SW1]	= { PCAP_REG_SWCTRL,	1,	2,	0xf,	0xff,	0xff, },
[SW2]	= { PCAP_REG_SWCTRL,	6,	7,	0xf,	0xff,	0xff, },
[SW3]	= { PCAP_REG_SWCTRL,	11,	12,	0x3,	0xff,	0xff, },
[SW1S]	= { PCAP_REG_LOWPWR,	0xff,	12,	0xf,	0xff,	0xff, },
[SW2S]	= { PCAP_REG_LOWPWR,	0xff,	20,	0xf,	0xff,	0xff, },
};

static int pcap_sw1_to_mv(u8 sw)
{
	switch (sw) {
	case 0 ... 0xa:
		return 900 + (sw * 50);
	case 0xb ... 0xc:
		return 1500 + ((sw - 0xb) * 100);
	case 0xd:
		return 1875;
	case 0xe:
		return 2250;
	case 0xf:
		return 4400;
}

static u8 pcap_mv_to_sw1(int mv)
{
	if (mv < 900)
		return 0;
	if (mv < 1500)
		return (mv - 900) / 50;
	if (mv < 1875)
		return ((mv - 1500) / 100) + 0xb;
	if (mv < 2250)
		return 0xd;
	if (mv < 4400)
		return 0xe;
	if (mv >= 4400)
		return 0xf;
}

static int pcap_vsim_to_mv(u8 vsim)
{
	if (vsim == 0)
		return 1800;
	return 3000;
}

static u8 pcap_mv_to_vsim(int mv)
{
	if (mv < 3000)
		return 0;
	return 1;
}

static int pcap_regulator_set_voltage(struct regulator_dev *rdev,
						int min_uv, int max_uv)
{
	u32 tmp;
	u8 bits;
	int vreg = rdev_get_id(rdev);

	if (vreg > PCAP_LAST_VREG || vreg_table[vreg][V_INDEX] == 0xff)
		return -EINVAL;

	switch (vreg) {
	case SW1:
	case SW1S:
		bits = pcap_mv_to_sw1(min_uv / 1000);
		break;
	case VSIM:
		bits = pcap_mv_to_vsim(min_uv / 1000);
		break;
	default: /* FIXME */
		return -EINVAL;
	}

	ezx_pcap_read(vreg_table[vreg][V_REG], &tmp);
	tmp |= (bits & vreg_table[vreg][V_MASK]) << vreg_table[vreg][V_INDEX];
	ezx_pcap_write(vreg_table[vreg][V_REG], tmp);

	if (vreg >= SW1 && vreg <= SW2S)
		udelay(150);

	return 0;
}

static int pcap_regulator_get_voltage(struct regulator_dev *rdev)
{
	u32 tmp;
	int mv;
	int vreg = rdev_get_id(rdev);

	if (vreg > PCAP_LAST_VREG || vreg_table[vreg][V_INDEX] == 0xff)
		return -EINVAL;

	ezx_pcap_read(vreg_table[vreg][V_REG], &tmp);
	tmp = ((tmp >> vreg_table[vreg][V_INDEX]) & vreg_table[vreg][V_MASK]);

	switch (vreg) {
	case SW1:
	case SW1S:
		mv = pcap_sw1_to_mv(tmp);
		break;
	case VSIM:
		mv = pcap_vsim_to_mv(tmp);
		break;
	default: /* FIXME */
		return -EINVAL;
	}

	return mv * 1000;
}

static int pcap_regulator_enable(struct regulator_dev *rdev)
{
	u32 tmp;
	int vreg = rdev_get_id(rdev);

	if (vreg > PCAP_LAST_VREG || vreg_table[vreg][V_EN] == 0xff)
		return -EINVAL;

	ezx_pcap_read(vreg_table[vreg][V_REG], &tmp);
	tmp |= (1 << vreg_table[vreg][V_EN]);
	ezx_pcap_write(vreg_table[vreg][V_REG], tmp);

	return 0;
}

static int pcap_regulator_disable(struct regulator_dev *rdev)
{
	u32 tmp;
	int vreg = rdev_get_id(rdev);

	if (vreg > PCAP_LAST_VREG || vreg_table[vreg][V_EN] == 0xff)
		return -EINVAL;

	ezx_pcap_read(vreg_table[vreg][V_REG], &tmp);
	tmp &= ~(1 << vreg_table[vreg][V_EN]);
	ezx_pcap_write(vreg_table[vreg][V_REG], tmp);

	return 0;
}

static int pcap_regulator_is_enabled(struct regulator_dev *rdev)
{
	u32 tmp;
	u8 reg, shift, mask;
	int vreg = rdev_get_id(rdev);

	if (vreg > PCAP_LAST_VREG)
		return -EINVAL;

	if (vreg == SW1S || vreg == SW2S) /* always enabled */
		return 1;

	ezx_pcap_read(vreg_table[vreg][V_REG], &tmp);
	return ((tmp >> vreg_table[vreg][V_EN]) & 1);
}

static struct regulator_ops pcap_regulator_ops = {
	.set_voltage	= pcap_regulator_set_voltage,
	.get_voltage	= pcap_regulator_get_voltage,
	.enable		= pcap_regulator_enable,
	.disable	= pcap_regulator_disable,
	.is_enabled	= pcap_regulator_is_enabled,
};

static struct regulator_desc pcap_regulators[] = {
	[V1]	= PCAP_REGULATOR(V1),
	[V2]	= PCAP_REGULATOR(V2),
	[V3]	= PCAP_REGULATOR(V3),
	[V4]	= PCAP_REGULATOR(V4),
	[V5]	= PCAP_REGULATOR(V5),
	[V6]	= PCAP_REGULATOR(V6),
	[V7]	= PCAP_REGULATOR(V7),
	[V8]	= PCAP_REGULATOR(V8),
	[V9]	= PCAP_REGULATOR(V9),
	[V10]	= PCAP_REGULATOR(V10),
	[VAUX1]	= PCAP_REGULATOR(VAUX1),
	[VAUX2] = PCAP_REGULATOR(VAUX2),
	[VAUX3]	= PCAP_REGULATOR(VAUX3),
	[VAUX4]	= PCAP_REGULATOR(VAUX4),
	[VSIM]	= PCAP_REGULATOR(VSIM),
	[VSIM2]	= PCAP_REGULATOR(VSIM2),
	[VVIB]	= PCAP_REGULATOR(VVIB),
	[VC]	= PCAP_REGULATOR(VC),
	[SW1]	= PCAP_REGULATOR(SW1),
	[SW2]	= PCAP_REGULATOR(SW2),
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

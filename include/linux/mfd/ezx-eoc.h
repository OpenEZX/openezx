#include <linux/workqueue.h>

#ifndef EZX_EOC_H
#define EZX_EOC_H

#define EOC_REG_ADDR_SIZE		1
#define EOC_REG_DATA_SIZE		3

#define EOC_REG_ISR			32
#define EOC_REG_MSR			33
#define EOC_REG_SENSE			34
#define EOC_REG_POWER_CONTROL_0		35
#define EOC_REG_POWER_CONTROL_1		36
#define EOC_REG_CONN_CONTROL		37

#define EOC_IRQ_VBUS_3V4		0
#define EOC_IRQ_VBUS			1
#define EOC_IRQ_VBUS_OV			2
#define EOC_IRQ_RVRS_CHRG		3
#define EOC_IRQ_ID			4
#define EOC_IRQ_SE1			6
#define EOC_IRQ_CC_CV			7
#define EOC_IRQ_CHRG_CURR		8
#define EOC_IRQ_RVRS_CURR		9
#define EOC_IRQ_CK			10
#define EOC_IRQ_BATTPON			11

#define EOC_SENSE_VBUS_3V4		1
#define EOC_SENSE_VBUS_4V4		(1<<1)
#define EOC_SENSE_VBUS_2V		(1<<2)
#define EOC_SENSE_VBUS_0V8		(1<<3)
#define EOC_SENSE_ID_FLOAT		(1<<4)
#define EOC_SENSE_ID_GROUND		(1<<5)
#define EOC_SENSE_SE1			(1<<6)
#define EOC_SENSE_CC_CV			(1<<7)	
#define EOC_SENSE_CURRENT		(1<<8)	
#define EOC_SENSE_BATTERY_PON		(1<<13)

/* 21044  2, 6(70K_PD), 12(XCVR), 17(MODE(3)) */
#define EOC_CONN_USB_SUSPEND		(1 << 1)
#define EOC_CONN_DPLUS_150K_PU		(1 << 5)
#define EOC_CONN_VBUS_70K_PD		(1 << 6)
#define EOC_CONN_XCVR			(1 << 12)
#define EOC_CONN_MODE(x)		((x & 7) << 14)
#define EOC_CONN_ID_PD			(1 << 20)
#define EOC_CONN_ID_PU			(1 << 22)

/* c 2, 3 */
#define EOC_POWER1_INPUT_SOURCE(x)	((x & 3) << 0)
#define EOC_POWER1_OUTPUT_VOLTAGE	(1 << 2)
#define EOC_POWER1_VUSB			(1 << 3)

/* c00 10, 11 */
#define EOC_POWER0_VBUS_5K_PD		(1 << 19)
#define EOC_POWER0_REVERSE_MODE		(1 << 13)
#define EOC_POWER0_FET_SOFT		(1 << 10)
#define EOC_POWER0_FET_STATE		(1 << 11)

/* Bits in EMU one-chip's power control register 0. */
#define EMU_VCHRG_MASK              0x00000007
#define EMU_ICHRG_MASK              0x00000078
#define EMU_ICHRG_SHIFT             3
#define EMU_ICHRG_TR_MASK           0x00000380
#define EMU_ICHRG_TR_SHIFT          7
#define EMU_FET_OVRD_MASK           0x00000400
#define EMU_FET_CTRL_MASK           0x00000800

#define EOC_NIRQS 11

struct eoc_platform_data {
	unsigned                                irq_base, irq_end;
};

struct eoc_chip {
	unsigned int irq_base;
	struct work_struct isr_work;
	struct work_struct msr_work;
	struct work_struct power0_work;
	int sense;
	int power0;
	int msr;
	struct i2c_client *client;
};

int eoc_to_irq(struct eoc_chip *, int);
int irq_to_eoc(struct eoc_chip *, int);

#endif

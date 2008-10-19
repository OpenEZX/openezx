/*
 * Copyright 2007 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * For further information, please see http://wiki.openezx.org/PCAP2
 */

#ifndef EZX_PCAP_H
#define EZX_PCAP_H

struct pcap_platform_data {
	unsigned int cs;
	unsigned int irq;
	unsigned int config;
	void (*init)(void);	/* board specific init */
};

#define PCAP_CS_INVERTED	1
#define PCAP_SECOND_PORT	2

#define PCAP_REGISTER_WRITE_OP_BIT	0x80000000
#define PCAP_REGISTER_READ_OP_BIT	0x00000000

#define PCAP_REGISTER_VALUE_MASK	0x01ffffff
#define PCAP_REGISTER_ADDRESS_MASK	0x7c000000
#define PCAP_REGISTER_ADDRESS_SHIFT	26
#define PCAP_REGISTER_NUMBER		32
#define PCAP_CLEAR_INTERRUPT_REGISTER	0x01ffffff
#define PCAP_MASK_ALL_INTERRUPT		0x01ffffff

#define pbit(reg, bit)	((reg << PCAP_REGISTER_ADDRESS_SHIFT) | bit)

/* registers acessible by both pcap ports */
#define PCAP_REG_ISR		0x0	/* Interrupt Status */
#define PCAP_REG_MSR		0x1	/* Interrupt Mask */
#define PCAP_REG_PSTAT		0x2	/* Processor Status */
#define PCAP_REG_VREG2		0x6	/* Regulator Bank 2 Control */
#define PCAP_REG_AUXVREG	0x7	/* Auxiliary Regulator Control */
#define PCAP_REG_BATT		0x8	/* Battery Control */
#define PCAP_REG_ADC		0x9	/* AD Control */
#define PCAP_REG_ADR		0xa	/* AD Result */
#define PCAP_REG_CODEC		0xb	/* Audio Codec Control */
#define PCAP_REG_RX_AMPS	0xc	/* RX Audio Amplifiers Control */
#define PCAP_REG_ST_DAC		0xd	/* Stereo DAC Control */
#define PCAP_REG_BUSCTRL	0x14	/* Connectivity Control */
#define PCAP_REG_PERIPH		0x15	/* Peripheral Control */
#define PCAP_REG_LOWPWR		0x18	/* Regulator Low Power Control */
#define PCAP_REG_TX_AMPS	0x1a	/* TX Audio Amplifiers Control */
#define PCAP_REG_GP		0x1b	/* General Purpose */
#define PCAP_REG_TEST1		0x1c
#define PCAP_REG_TEST2		0x1d
#define PCAP_REG_VENDOR_TEST1	0x1e
#define PCAP_REG_VENDOR_TEST2	0x1f


/* registers acessible by pcap port 1 only (a1200, e2 & e6) */
#define PCAP_REG_INT_SEL	0x3	/* Interrupt Select */
#define PCAP_REG_SWCTRL		0x4	/* Switching Regulator Control */
#define PCAP_REG_VREG1		0x5	/* Regulator Bank 1 Control */
#define PCAP_REG_RTC_TOD	0xe	/* RTC Time of Day */
#define PCAP_REG_RTC_TODA	0xf	/* RTC Time of Day Alarm */
#define PCAP_REG_RTC_DAY	0x10	/* RTC Day */
#define PCAP_REG_RTC_DAYA	0x11	/* RTC Day Alarm */
#define PCAP_REG_MTRTMR		0x12	/* AD Monitor Timer */
#define PCAP_REG_PWR		0x13	/* Power Control */
#define PCAP_REG_AUXVREG_MASK	0x16	/* Auxiliary Regulator Mask */
#define PCAP_REG_VENDOR_REV	0x17
#define PCAP_REG_PERIPH_MASK	0x19	/* Peripheral Mask */

/* interrupts - registers 0x0, 0x1, 0x2, 0x3 */
#define PCAP_IRQ_ADCDONE	(1 << 0)	/* AD Conversion Done Port 1 */
#define PCAP_IRQ_TS		(1 << 1)	/* Touch Screen */
#define PCAP_IRQ_1HZ		(1 << 2)	/* 1HZ Timer */
#define PCAP_IRQ_WH		(1 << 3)	/* "...high"??? */
#define PCAP_IRQ_WL		(1 << 4)	/* "...low"??? */
#define PCAP_IRQ_TODA		(1 << 5)	/* RTC Time Of Day?
						     (see "RTC_TODA") */
#define PCAP_IRQ_USB4V		(1 << 6)	/* USB OTG */
#define PCAP_IRQ_ONOFF		(1 << 7)	/* in blob: "ONOFFSNS" */
#define PCAP_IRQ_ONOFF2		(1 << 8)	/* in blob: "ONOFFSNS2" */
#define PCAP_IRQ_USB1V		(1 << 9)	/* USB below 1volt???
						     in blob: "USBDET_1V" */
#define PCAP_IRQ_MOBPORT	(1 << 10)	/* GSM-related?? ("mobport",
				see 958_MotDoc.pdf); in blob: "MOBSENSB" */
#define PCAP_IRQ_MB2		(1 << 11)	/* Mic; in blob: "MB2SNS" */
#define PCAP_IRQ_A1		(1 << 12)	/* Audio jack;
						     in blob: "A1SNS" */
#define PCAP_IRQ_ST		(1 << 13)	/* called "MSTB" in blob */
#define PCAP_IRQ_PC		(1 << 14)
#define PCAP_IRQ_WARM		(1 << 15)
#define PCAP_IRQ_EOL		(1 << 16)	/* battery End Of Life???
					(see below); in blob: "EOL_STAT" */
#define PCAP_IRQ_CLK		(1 << 17)	/* called "CLK_STAT" in blob */
#define PCAP_IRQ_SYSRST		(1 << 18)
#define PCAP_IRQ_DUMMY		(1 << 19)
#define PCAP_IRQ_ADCDONE2	(1 << 20)	/* AD Conversion Done Port 2 */
#define PCAP_IRQ_SOFTRESET	(1 << 21)
#define PCAP_IRQ_MNEXB		(1 << 22)

/* voltage regulators */
#define V1		0
#define V2		1
#define V3		2
#define V4		3
#define V5		4
#define V6		5
#define V7		6
#define V8		7
#define V9		8
#define V10		9
#define VAUX1		10
#define VAUX2		11
#define VAUX3		12
#define VAUX4		13
#define VSIM		14
#define VSIM2		15
#define VVIB		16
#define VC		17

#define V_EN		0
#define V_VAL		1
#define V_MASK		2
#define V_STBY		3
#define V_LOWPWR	4

#define PCAP_BATT_DAC_MASK		0x000000ff
#define PCAP_BATT_DAC_SHIFT		0
#define PCAP_BATT_B_FDBK		(1 << 8)
#define PCAP_BATT_EXT_ISENSE		(1 << 9)
#define PCAP_BATT_V_COIN_MASK		0x00003c00
#define PCAP_BATT_V_COIN_SHIFT		10
#define PCAP_BATT_I_COIN		(1 << 14)
#define PCAP_BATT_COIN_CH_EN		(1 << 15)
#define PCAP_BATT_EOL_SEL_MASK		0x000e0000
#define PCAP_BATT_EOL_SEL_SHIFT		17
#define PCAP_BATT_EOL_CMP_EN		(1 << 20)
#define PCAP_BATT_BATT_DET_EN		(1 << 21)
#define PCAP_BATT_THERMBIAS_CTRL	(1 << 22)

#define PCAP_ADC_ADEN			(1 << 0)
#define PCAP_ADC_RAND			(1 << 1)
#define PCAP_ADC_AD_SEL1		(1 << 2)
#define PCAP_ADC_AD_SEL2		(1 << 3)
#define PCAP_ADC_ADA1_MASK		0x00000070
#define PCAP_ADC_ADA1_SHIFT		4
#define PCAP_ADC_ADA2_MASK		0x00000380
#define PCAP_ADC_ADA2_SHIFT		7
#define PCAP_ADC_ATO_MASK		0x00003c00
#define PCAP_ADC_ATO_SHIFT		10
#define PCAP_ADC_ATOX			(1 << 14)
#define PCAP_ADC_MTR1			(1 << 15)
#define PCAP_ADC_MTR2			(1 << 16)
#define PCAP_ADC_TS_M_MASK		0x000e0000
#define PCAP_ADC_TS_M_SHIFT		17
#define PCAP_ADC_TS_REF_LOWPWR		(1 << 20)
#define PCAP_ADC_TS_REFENB		(1 << 21)
#define PCAP_ADC_BATT_I_POLARITY	(1 << 22)
#define PCAP_ADC_BATT_I_ADC		(1 << 23)

#define PCAP_ADC_BANK_0			0
#define PCAP_ADC_BANK_1			1
/* ADC bank 0 */
#define PCAP_ADC_CH_COIN		0
#define PCAP_ADC_CH_BATT		1
#define PCAP_ADC_CH_BPLUS		2
#define PCAP_ADC_CH_MOBPORTB		3
#define PCAP_ADC_CH_TEMPERATURE		4
#define PCAP_ADC_CH_CHARGER_ID		5
#define PCAP_ADC_CH_AD6			6
/* ADC bank 1 */
#define PCAP_ADC_CH_AD7			0
#define PCAP_ADC_CH_AD8			1
#define PCAP_ADC_CH_AD9			2
#define PCAP_ADC_CH_TS_X1		3
#define PCAP_ADC_CH_TS_X2		4
#define PCAP_ADC_CH_TS_Y1		5
#define PCAP_ADC_CH_TS_Y2		6

#define PCAP_ADC_T_NOW			0
#define PCAP_ADC_T_IN_BURST		1
#define PCAP_ADC_T_OUT_BURST		2

#define PCAP_ADC_ATO_IN_BURST		6
#define PCAP_ADC_ATO_OUT_BURST		0

#define PCAP_ADC_TS_M_XY		1
#define PCAP_ADC_TS_M_PRESSURE		2
#define PCAP_ADC_TS_M_PLATE_X		3
#define PCAP_ADC_TS_M_PLATE_Y		4
#define PCAP_ADC_TS_M_STANDBY		5
#define PCAP_ADC_TS_M_NONTS		6

#define PCAP_ADR_ADD1_MASK		0x000003ff
#define PCAP_ADR_ADD1_SHIFT		0
#define PCAP_ADR_ADD2_MASK		0x000ffc00
#define PCAP_ADR_ADD2_SHIFT		10
#define PCAP_ADR_ADINC1			(1 << 20)
#define PCAP_ADR_ADINC2			(1 << 21)
#define PCAP_ADR_ASC			(1 << 22)
#define PCAP_ADR_ONESHOT		(1 << 23)

#define PCAP_BUSCTRL_FSENB		(1 << 0)
#define PCAP_BUSCTRL_USB_SUSPEND	(1 << 1)
#define PCAP_BUSCTRL_USB_PU		(1 << 2)
#define PCAP_BUSCTRL_USB_PD		(1 << 3)
#define PCAP_BUSCTRL_VUSB_EN		(1 << 4)
#define PCAP_BUSCTRL_USB_PS		(1 << 5)
#define PCAP_BUSCTRL_VUSB_MSTR_EN	(1 << 6)
#define PCAP_BUSCTRL_VBUS_PD_ENB	(1 << 7)
#define PCAP_BUSCTRL_CURRLIM		(1 << 8)
#define PCAP_BUSCTRL_RS232ENB		(1 << 9)
#define PCAP_BUSCTRL_RS232_DIR		(1 << 10)
#define PCAP_BUSCTRL_SE0_CONN		(1 << 11)
#define PCAP_BUSCTRL_USB_PDM		(1 << 12)
#define PCAP_BUSCTRL_BUS_PRI_ADJ	(1 << 24)

#define PCAP_BIT_PERIPH_BL_CTRL0	0x54000001
#define PCAP_BIT_PERIPH_BL_CTRL1	0x54000002
#define PCAP_BIT_PERIPH_BL_CTRL2	0x54000004
#define PCAP_BIT_PERIPH_BL_CTRL3	0x54000008
#define PCAP_BIT_PERIPH_BL_CTRL4	0x54000010
#define PCAP_BIT_PERIPH_LEDR_EN		0x54000020
#define PCAP_BIT_PERIPH_LEDG_EN		0x54000040
#define PCAP_BIT_PERIPH_LEDR_CTRL0	0x54000080
#define PCAP_BIT_PERIPH_LEDR_CTRL1	0x54000100
#define PCAP_BIT_PERIPH_LEDR_CTRL2	0x54000200
#define PCAP_BIT_PERIPH_LEDR_CTRL3	0x54000400
#define PCAP_BIT_PERIPH_LEDG_CTRL0	0x54000800
#define PCAP_BIT_PERIPH_LEDG_CTRL1	0x54001000
#define PCAP_BIT_PERIPH_LEDG_CTRL2	0x54002000
#define PCAP_BIT_PERIPH_LEDG_CTRL3	0x54004000
#define PCAP_BIT_PERIPH_LEDR_I0		0x54008000
#define PCAP_BIT_PERIPH_LEDR_I1		0x54010000
#define PCAP_BIT_PERIPH_LEDG_I0		0x54020000
#define PCAP_BIT_PERIPH_LEDG_I1		0x54040000
#define PCAP_BIT_PERIPH_SKIP		0x54080000
#define PCAP_BIT_PERIPH_BL2_CTRL0	0x54100000
#define PCAP_BIT_PERIPH_BL2_CTRL1	0x54200000
#define PCAP_BIT_PERIPH_BL2_CTRL2	0x54400000
#define PCAP_BIT_PERIPH_BL2_CTRL3	0x54800000
#define PCAP_BIT_PERIPH_BL2_CTRL4	0x55000000

/* LOWPWR */
#define SW1		8
#define SW2		16

#define SW_MODE		0
#define SW_VOLTAGE	4

#define SW_VOLTAGE_900	0x0
#define SW_VOLTAGE_950	0x1
#define SW_VOLTAGE_1000	0x2
#define SW_VOLTAGE_1050	0x3
#define SW_VOLTAGE_1100	0x4
#define SW_VOLTAGE_1150	0x5
#define SW_VOLTAGE_1200	0x6
#define SW_VOLTAGE_1250	0x7
#define SW_VOLTAGE_1300	0x8
#define SW_VOLTAGE_1350	0x9
#define SW_VOLTAGE_1400	0xa
#define SW_VOLTAGE_1500	0xb
#define SW_VOLTAGE_1600	0xc
#define SW_VOLTAGE_1875	0xd
#define SW_VOLTAGE_2250	0xe
#define SW_VOLTAGE_4400	0xf

int ezx_pcap_write(u8, u32);
int ezx_pcap_read(u8, u32 *);
int ezx_pcap_set_sw(u8, u8, u8);
int ezx_pcap_set_vreg(u8, u8, u8);
void ezx_pcap_start_adc(u8, u8, u32, void *, void *);
void ezx_pcap_get_adc_channel_result(u8, u8, u32[]);
void ezx_pcap_get_adc_bank_result(u32[]);
void ezx_pcap_disable_adc(void);
void ezx_pcap_do_general_adc(u8, u8, u32 *);
void ezx_pcap_do_batt_adc(int, u32[]);
int ezx_pcap_register_event(u32, void *, char *);
int ezx_pcap_unregister_event(u32);
void ezx_pcap_mask_event(u32);
void ezx_pcap_unmask_event(u32);

struct pcap_event {
	struct list_head node;
	char *label;
	u32 events;
	void (*callback)(u32, void *);
	void *data;
};

#endif

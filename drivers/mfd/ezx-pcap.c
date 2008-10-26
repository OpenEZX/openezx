/* 
 * Driver for Motorola PCAP2 as present in EZX phones
 *
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 * Copyright (C) 2007-2008 Daniel Ribeiro <drwyrm@gmail.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/mfd/ezx-pcap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>

#include <mach/ssp.h>
#include <mach/pxa-regs.h>
#include <mach/regs-ssp.h>
#include <mach/mfp-pxa27x.h>
#include <mach/irqs.h>

struct pcap_chip {
	struct spi_device *spi;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	void (*adc_done)(void *);
	void *adc_data;
};
static struct pcap_chip pcap;

static LIST_HEAD(event_list);
static DEFINE_MUTEX(event_lock);
static DEFINE_MUTEX(adc_lock);

/* IO */
static int ezx_pcap_putget(u32 *data)
{
	struct spi_transfer t;
	struct spi_message m;

	memset(&t, 0, sizeof t);
	spi_message_init(&m);
	t.len = 4;
	t.tx_buf = (u8 *)data;
	t.rx_buf = (u8 *)data;
	t.bits_per_word = 32;
	spi_message_add_tail(&t, &m);
	return spi_sync(pcap.spi, &m);
}

int ezx_pcap_write(u8 reg_num, u32 value)
{
	value &= PCAP_REGISTER_VALUE_MASK;
	value |= PCAP_REGISTER_WRITE_OP_BIT
		| (reg_num << PCAP_REGISTER_ADDRESS_SHIFT);
	return ezx_pcap_putget(&value);
}
EXPORT_SYMBOL_GPL(ezx_pcap_write);

int ezx_pcap_read(u8 reg_num, u32 *value)
{
	*value = PCAP_REGISTER_READ_OP_BIT
		| (reg_num << PCAP_REGISTER_ADDRESS_SHIFT);

	return ezx_pcap_putget(value);
}
EXPORT_SYMBOL_GPL(ezx_pcap_read);

/* Voltage regulators */
int ezx_pcap_set_sw(u8 sw, u8 what, u8 val)
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_LOWPWR, &tmp);
	tmp &= ~(0xf << (sw + what));
	tmp |= ((val & 0xf) << (sw + what));
	return ezx_pcap_write(PCAP_REG_LOWPWR, tmp);
}
EXPORT_SYMBOL_GPL(ezx_pcap_set_sw);

static u8 vreg_table[][10] = {
	/*		EN	INDEX	MASK	STBY	LOWPWR	*/
	[V1]	= {	1,	2,	0x7,	18,	0,	},
	[V2]	= {	5,	6,	0x1,	19,	22,	},
	[V3]	= {	7,	8,	0x7,	20,	23,	},
	[V4]	= {	11,	12,	0x7,	21,	24,	},
	[V5]	= {	15,	16,	0x3,	0xff,	0xff,	},
	[V6]	= {	1,	0xff,	0x0,	0xff,	0xff,	},
	/* FIXME: I have no idea of V7-V10 bits -WM */
	[V7]	= {	0xff,	0xff,	0x0,	0xff,	0xff,	},
	[V8]	= {	0xff,	0xff,	0x0,	0xff,	0xff,	},
	[V9]	= {	0xff,	0xff,	0x0,	0xff,	0xff,	},
	[V10]	= {	0xff,	0xff,	0x0,	0xff,	0xff,	},
	[VAUX1]	= {	1,	2,	0x3,	22,	23,	},
	[VAUX2]	= {	4,	5,	0x3,	0,	1,	},
	[VAUX3]	= {	7,	8,	0xf,	2,	3,	},
	[VAUX4]	= {	12,	13,	0x3,	4,	5,	},
	[VSIM]	= {	17,	18,	0x1,	0xff,	6,	},
	[VSIM2]	= {	16,	0xff,	0x0,	0xff,	7,	},
	[VVIB]	= {	19,	20,	0x3,	0xff,	0xff,	},
	[VC]	= {	0xff,	0xff,	0x0,	24,	0xff,	},
};

int ezx_pcap_set_vreg(u8 vreg, u8 what, u8 val)
{
	struct pcap_platform_data *pdata = pcap.spi->dev.platform_data;
	u8 reg, shift, mask;
	u32 tmp;

	switch (vreg) {
	case V1 ... V5:
		/* vreg1 is not accessible on port 2 */
		if (pdata->config & PCAP_SECOND_PORT)
			return -EINVAL;
		reg = PCAP_REG_VREG1;
		break;
	case V6 ... V10:
		reg = PCAP_REG_VREG2;
		break;
	case VAUX1 ... VC:
		if ((what == V_LOWPWR || what == V_STBY) && vreg != VAUX1)
			reg = PCAP_REG_LOWPWR;
		else
			reg = PCAP_REG_AUXVREG;
		break;
	default:
		return -EINVAL;
	}

	switch (what) {
	case V_VAL:
		shift = vreg_table[vreg][V_VAL];
		mask = vreg_table[vreg][V_MASK];
		break;
	case V_EN:
	case V_STBY:
	case V_LOWPWR:
		shift = vreg_table[vreg][what];
		mask = 0x1;
		break;
	default:
		return -EINVAL;
	}

	/* invalid setting */
	if (shift == 0xff || val > mask)
		return -EINVAL;

	ezx_pcap_read(reg, &tmp);
	tmp &= ~(mask << shift);
	tmp |= ((val & mask) << shift);
	ezx_pcap_write(reg, tmp);

	return 0;
}
EXPORT_SYMBOL_GPL(ezx_pcap_set_vreg);

/* ADC */
void ezx_pcap_disable_adc(void)
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADEN|PCAP_ADC_BATT_I_ADC|PCAP_ADC_BATT_I_POLARITY);
	tmp |= (PCAP_ADC_TS_M_STANDBY << PCAP_ADC_TS_M_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	mutex_unlock(&adc_lock);
}

static void ezx_pcap_adc_event(struct work_struct *unused)
{
	void (*adc_done)(void *);
	void *adc_data;

	/* caller may call start_adc, so we save adc_done/data before */
	adc_done = pcap.adc_done;
	adc_data = pcap.adc_data;
	pcap.adc_done = pcap.adc_data = NULL;

	/* let caller get the results */
	adc_done(adc_data);
}

void ezx_pcap_start_adc(u8 bank, u8 time, u32 flags,
						void *adc_done, void *adc_data)
{
	u32 adc;
	u32 adr;

	mutex_lock(&adc_lock);

	adc = flags | PCAP_ADC_ADEN;

	if (bank == PCAP_ADC_BANK_1)
		adc |= PCAP_ADC_AD_SEL1;

	ezx_pcap_write(PCAP_REG_ADC, adc);

	pcap.adc_done = adc_done;
	pcap.adc_data = adc_data;

	if (time == PCAP_ADC_T_NOW) {
		ezx_pcap_read(PCAP_REG_ADR, &adr);
		adr = PCAP_ADR_ASC;
		ezx_pcap_write(PCAP_REG_ADR, adr);
		return;
	}

	if (time == PCAP_ADC_T_IN_BURST)
		adc |= (PCAP_ADC_ATO_IN_BURST << PCAP_ADC_ATO_SHIFT);

	ezx_pcap_write(PCAP_REG_ADC, adc);

	ezx_pcap_read(PCAP_REG_ADR, &adr);
	adr &= ~PCAP_ADR_ONESHOT;
	ezx_pcap_write(PCAP_REG_ADR, adr);
	adr |= PCAP_ADR_ONESHOT;
	ezx_pcap_write(PCAP_REG_ADR, adr);
}
EXPORT_SYMBOL_GPL(ezx_pcap_start_adc);

void ezx_pcap_get_adc_channel_result(u8 ch1, u8 ch2, u32 res[])
{
	u32 tmp;

	ezx_pcap_read(PCAP_REG_ADC, &tmp);
	tmp &= ~(PCAP_ADC_ADA1_MASK | PCAP_ADC_ADA2_MASK);
	tmp |= (ch1 << PCAP_ADC_ADA1_SHIFT) | (ch2 << PCAP_ADC_ADA2_SHIFT);
	ezx_pcap_write(PCAP_REG_ADC, tmp);
	ezx_pcap_read(PCAP_REG_ADR, &tmp);
	res[0] = (tmp & PCAP_ADR_ADD1_MASK) >> PCAP_ADR_ADD1_SHIFT;
	res[1] = (tmp & PCAP_ADR_ADD2_MASK) >> PCAP_ADR_ADD2_SHIFT;
}
EXPORT_SYMBOL_GPL(ezx_pcap_get_adc_channel_result);

void ezx_pcap_get_adc_bank_result(u32 res[])
{
	int x;
	u32 tmp[2];

	for (x = 0;x < 7; x += 2) {
		ezx_pcap_get_adc_channel_result(x, (x+1) % 6, tmp);
		res[x] = tmp[0];
		if ((x + 1) < 7)
			res[x+1] = tmp[1];
		else
			res[x+1] = 0;
	}
}
EXPORT_SYMBOL_GPL(ezx_pcap_get_adc_bank_result);

static void adc_complete(void *data)
{
	complete(data);
}

void ezx_pcap_do_general_adc(u8 bank, u8 ch, u32 *res)
{
	u32 tmp[2];
	DECLARE_COMPLETION_ONSTACK(done);

	ezx_pcap_start_adc(bank, PCAP_ADC_T_NOW, 0, adc_complete, &done);
	wait_for_completion(&done);
	ezx_pcap_get_adc_channel_result(ch, 0, tmp);
	ezx_pcap_disable_adc();

	*res = tmp[0];
}
EXPORT_SYMBOL_GPL(ezx_pcap_do_general_adc);

void ezx_pcap_do_batt_adc(int pol, u32 res[])
{
	u32 tmp[7];
	DECLARE_COMPLETION_ONSTACK(done);

	ezx_pcap_start_adc(PCAP_ADC_BANK_0, PCAP_ADC_T_NOW,
				PCAP_ADC_RAND | PCAP_ADC_BATT_I_ADC |
				(PCAP_ADC_CH_BATT << PCAP_ADC_ADA1_SHIFT) |
				(pol ? PCAP_ADC_BATT_I_POLARITY : 0),
				adc_complete, &done);
	wait_for_completion(&done);
	ezx_pcap_get_adc_bank_result(tmp);
	ezx_pcap_disable_adc();

	/* average conversions and translate current value */
	res[0] = (tmp[0] + tmp[2] + tmp[4]) / 3;
	res[1] = (tmp[1] + tmp[3] + tmp[5]) / 3;
	res[1] = (res[1] - 178) * 3165 / 1000;
}
EXPORT_SYMBOL_GPL(ezx_pcap_do_batt_adc);

/* event handling */
static irqreturn_t pcap_irq_handler(int irq, void *dev_id)
{
	queue_work(pcap.workqueue, &pcap.work);
	return IRQ_HANDLED;
}

static void pcap_work(struct work_struct *_pcap)
{
	u32 msr;
	u32 isr;
	u32 service;
	struct pcap_event *cb;

	mutex_lock(&event_lock);
	ezx_pcap_read(PCAP_REG_MSR, &msr);
	ezx_pcap_read(PCAP_REG_ISR, &isr);
	isr &= ~msr;
	
	list_for_each_entry(cb, &event_list, node) {
		service = isr & cb->events;
		if (service) {
			ezx_pcap_write(PCAP_REG_ISR, service);
			cb->callback(service, cb->data);
		}
	}
	mutex_unlock(&event_lock);
}

void ezx_pcap_mask_event(u32 events)
{
	u32 msr;

	ezx_pcap_read(PCAP_REG_MSR, &msr);
	msr |= events;
	ezx_pcap_write(PCAP_REG_MSR, msr);
}
EXPORT_SYMBOL_GPL(ezx_pcap_mask_event);

void ezx_pcap_unmask_event(u32 events)
{
	u32 msr;

	ezx_pcap_read(PCAP_REG_MSR, &msr);
	msr &= ~events;
	ezx_pcap_write(PCAP_REG_MSR, msr);
}
EXPORT_SYMBOL_GPL(ezx_pcap_unmask_event);

int ezx_pcap_register_event(u32 events, void *callback, char *label)
{
	struct pcap_event *cb;

	cb = kzalloc(sizeof(struct pcap_event), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	cb->label = label;
	cb->events = events;
	cb->callback = callback;

	mutex_lock(&event_lock);
	list_add_tail(&cb->node, &event_list);
	mutex_unlock(&event_lock);

	ezx_pcap_unmask_event(events);
	return 0;
}
EXPORT_SYMBOL_GPL(ezx_pcap_register_event);

int ezx_pcap_unregister_event(u32 events)
{
	int ret = -EINVAL;
	struct pcap_event *cb;

	ezx_pcap_mask_event(events);

	mutex_lock(&event_lock);
	list_for_each_entry(cb, &event_list, node) {
		if (cb->events & events) {
			list_del(&cb->node);
			kfree(cb);
			ret = 0;
		}
	}
	mutex_unlock(&event_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ezx_pcap_unregister_event);

/* sysfs interface */
static ssize_t pcap_show_regs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int reg, val;
	char *p = buf;

	for (reg = 0; reg < 32; reg++) {
		ezx_pcap_read(reg, &val);
		p += sprintf(p, "%02d %08x\n", reg, val);
	}
	return p - buf;
}

static ssize_t pcap_store_regs(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg, val;
	char *p = (char *)buf;

	while(p < (buf + size)) {
		if ((sscanf(p, "%ud %x\n", &reg, &val) != 2) ||
			reg < 0 || reg >= 32)
			return -EINVAL;
		p = strchr(p, '\n') + 1;
	}

	p = (char *)buf;
	while(p < (buf + size)) {
		sscanf(p, "%ud %x\n", &reg, &val);
		ezx_pcap_write(reg, val);
		p = strchr(p, '\n') + 1;
	}

	return size;
}

static ssize_t pcap_show_adc_coin(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_COIN, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_battery(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_BATT, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_bplus(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_BPLUS, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_mobportb(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_MOBPORTB, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_temperature(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_TEMPERATURE, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_chargerid(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res;

	ezx_pcap_do_general_adc(PCAP_ADC_BANK_0, PCAP_ADC_CH_CHARGER_ID, &res);
	return sprintf(buf, "%d\n", res);
}
static ssize_t pcap_show_adc_battcurr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 res[2];

	/* FIXME: polarity may change depending on phone */
	ezx_pcap_do_batt_adc(1, res);
	return sprintf(buf, "voltage: %d\ncurrent: %d\n", res[0], res[1]);
}

static DEVICE_ATTR(regs, 0600, pcap_show_regs, pcap_store_regs);
static DEVICE_ATTR(adc_coin, 0400, pcap_show_adc_coin, NULL);
static DEVICE_ATTR(adc_battery, 0400, pcap_show_adc_battery, NULL);
static DEVICE_ATTR(adc_bplus, 0400, pcap_show_adc_bplus, NULL);
static DEVICE_ATTR(adc_mobportb, 0400, pcap_show_adc_mobportb, NULL);
static DEVICE_ATTR(adc_temperature, 0400, pcap_show_adc_temperature, NULL);
static DEVICE_ATTR(adc_chargerid, 0400, pcap_show_adc_chargerid, NULL);
static DEVICE_ATTR(adc_battcurr, 0400, pcap_show_adc_battcurr, NULL);

static int ezx_pcap_setup_sysfs(int create)
{
	int ret = 0;

	if (!create)
		goto remove_all;

	ret = device_create_file(&pcap.spi->dev, &dev_attr_regs);
	if (ret)
		goto ret;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_coin);
	if (ret)
		goto fail1;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_battery);
	if (ret)
		goto fail2;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_bplus);
	if (ret)
		goto fail3;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_mobportb);
	if (ret)
		goto fail4;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_temperature);
	if (ret)
		goto fail5;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_chargerid);
	if (ret)
		goto fail6;
	ret = device_create_file(&pcap.spi->dev, &dev_attr_adc_battcurr);
	if (ret)
		goto fail7;
	
	goto ret;

remove_all:
fail7:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_chargerid);
fail6:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_temperature);
fail5:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_mobportb);
fail4:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_bplus);
fail3:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_battery);
fail2:	device_remove_file(&pcap.spi->dev, &dev_attr_adc_coin);
fail1:	device_remove_file(&pcap.spi->dev, &dev_attr_regs);
ret:	return ret;
}

static int ezx_pcap_remove(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;
	
	ezx_pcap_setup_sysfs(0);
	destroy_workqueue(pcap.workqueue);
	ezx_pcap_unregister_event(PCAP_MASK_ALL_INTERRUPT);
	free_irq(pdata->irq, NULL);

	return 0;
}

static int __devinit ezx_pcap_probe(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;
	int ret = -ENODEV;

	if (!pdata)
		goto ret;

	pcap.spi = spi;
	ret = gpio_request(pdata->cs, "PCAP CS");
	if (ret) {
		dev_err(&spi->dev, "cant request CS gpio\n");
		goto ret;
	}
	gpio_direction_output(pdata->cs,
				(pdata->config & PCAP_CS_INVERTED) ? 1 : 0);

	INIT_WORK(&pcap.work, pcap_work);
	pcap.workqueue = create_singlethread_workqueue("pcapd");
	if (!pcap.workqueue) {
		dev_err(&spi->dev, "cant create pcap thread\n");
		goto ret;
	}

	/* set default register values */
	ezx_pcap_write(PCAP_REG_ADC, 0);
	ezx_pcap_write(PCAP_REG_ADR, 0);
	ezx_pcap_write(PCAP_REG_AUXVREG, 0);

	/* redirect all interrupts to AP */
	if (!(pdata->config & PCAP_SECOND_PORT))
		ezx_pcap_write(PCAP_REG_INT_SEL, 0);

	/* set board-specific settings */
	if (pdata->init)
		pdata->init();

	/* mask/ack all PCAP interrupts */
	ezx_pcap_write(PCAP_REG_MSR, PCAP_MASK_ALL_INTERRUPT);
	ezx_pcap_write(PCAP_REG_ISR, PCAP_CLEAR_INTERRUPT_REGISTER);
	
	ret = ezx_pcap_setup_sysfs(1);
	if (ret) {
		dev_err(&spi->dev, "cant create sysfs files\n");
		goto wq_destroy;
	}

	/* register irq for pcap */
	ret = request_irq(pdata->irq, pcap_irq_handler, IRQF_DISABLED,
		"PCAP", NULL);
	if (ret) {
		dev_err(&spi->dev, "cant request IRQ\n");
		goto wq_destroy;
	}
	set_irq_type(pdata->irq, IRQ_TYPE_EDGE_RISING);
	set_irq_wake(pdata->irq, 1);

	ezx_pcap_register_event((pdata->config & PCAP_SECOND_PORT) ?
			PCAP_IRQ_ADCDONE2 : PCAP_IRQ_ADCDONE,
			ezx_pcap_adc_event, "ADC");
	return 0;

wq_destroy:
	destroy_workqueue(pcap.workqueue);
ret:
	return ret;
}

static struct spi_driver ezxpcap_driver = {
	.probe		= ezx_pcap_probe,
	.remove		= ezx_pcap_remove,
	.driver		= {
		.name   = "ezx-pcap",
		.owner	= THIS_MODULE,
	},
};

static int __init ezx_pcap_init(void)
{
	return spi_register_driver(&ezxpcap_driver);
}

static void __exit ezx_pcap_exit(void)
{
	spi_unregister_driver(&ezxpcap_driver);
}

module_init(ezx_pcap_init);
module_exit(ezx_pcap_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Ribeiro / Harald Welte");
MODULE_DESCRIPTION("Motorola PCAP2 ASIC Driver");

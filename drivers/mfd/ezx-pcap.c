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

struct pcap_chip {
	struct spi_device *spi;
	struct work_struct work;
	struct workqueue_struct *workqueue;
};
static struct pcap_chip pcap;

static LIST_HEAD(event_list);
static DEFINE_MUTEX(event_lock);

/* IO */
#define PCAP_BUFSIZE	4
static int ezx_pcap_putget(u32 *data)
{
	struct spi_transfer t;
	struct spi_message m;
	int status;
	u32 *buf = kmalloc(PCAP_BUFSIZE, GFP_KERNEL);

	if (!buf)
		return -ENOMEM;

	memset(&t, 0, sizeof t);
	spi_message_init(&m);
	t.len = PCAP_BUFSIZE;
	spi_message_add_tail(&t, &m);

	*buf = *data;
	t.tx_buf = (u8 *) buf;
	t.rx_buf = (u8 *) buf;
	status = spi_sync(pcap.spi, &m);

	if (status == 0)
		*data = *buf;
	kfree(buf);

	return status;
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

static u8 vreg_table[][5] = {
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
//	[VC]	= {	0xff,	0xff,	0x0,	24,	0xff,	},
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
	case VAUX1 ... VVIB:
		if ((what == 4 || what == 3) && vreg != VAUX1)
			reg = PCAP_REG_LOWPWR;
		else
			reg = PCAP_REG_AUXVREG;
		break;
	default:
		return -EINVAL;
	}

	switch (what) {
	case 1:
		shift = vreg_table[vreg][1];
		mask = vreg_table[vreg][2];
		break;
	case 0:
	case 3:
	case 4:
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

int ezx_pcap_register_event(u32 events, void *callback, void *data, char *label)
{
	struct pcap_event *cb;

	cb = kzalloc(sizeof(struct pcap_event), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	cb->label = label;
	cb->events = events;
	cb->callback = callback;
	cb->data = data;

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
	struct pcap_event *store;

	ezx_pcap_mask_event(events);

	mutex_lock(&event_lock);
	list_for_each_entry_safe(cb, store, &event_list, node) {
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

static int __devexit ezx_pcap_remove(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;

	ezx_pcap_setup_sysfs(0);
	destroy_workqueue(pcap.workqueue);
	ezx_pcap_unregister_event(PCAP_MASK_ALL_INTERRUPT);
	free_irq(pdata->irq, NULL);
	pcap.spi = NULL;

	return 0;
}

static int __devinit ezx_pcap_probe(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;
	int ret = -ENODEV;

	if (!pdata)
		goto ret;

	if (pcap.spi) {
		ret = -EBUSY;
		goto ret;
	}

	spi->bits_per_word = 32;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret)
		goto ret;

	pcap.spi = spi;

	INIT_WORK(&pcap.work, pcap_work);
	pcap.workqueue = create_singlethread_workqueue("pcapd");
	if (!pcap.workqueue) {
		dev_err(&spi->dev, "cant create pcap thread\n");
		goto null_spi;
	}

	/* redirect interrupts to AP */
	if (!(pdata->config & PCAP_SECOND_PORT))
		ezx_pcap_write(PCAP_REG_INT_SEL, PCAP_IRQ_ADCDONE2);

	/* set board-specific settings */
	if (pdata->init)
		pdata->init();

	ret = ezx_pcap_setup_sysfs(1);
	if (ret) {
		dev_err(&spi->dev, "cant create sysfs files\n");
		goto wq_destroy;
	}

	/* mask/ack all PCAP interrupts */
	ezx_pcap_write(PCAP_REG_MSR, PCAP_MASK_ALL_INTERRUPT);
	ezx_pcap_write(PCAP_REG_ISR, PCAP_CLEAR_INTERRUPT_REGISTER);

	/* register irq for pcap */
	ret = request_irq(pdata->irq, pcap_irq_handler, IRQF_TRIGGER_RISING,
		"PCAP", NULL);
	if (ret) {
		dev_err(&spi->dev, "cant request IRQ\n");
		goto wq_destroy;
	}
	set_irq_wake(pdata->irq, 1);

	return 0;

wq_destroy:
	destroy_workqueue(pcap.workqueue);
null_spi:
	pcap.spi = NULL;
ret:
	return ret;
}

static struct spi_driver ezxpcap_driver = {
	.probe  = ezx_pcap_probe,
	.remove = __devexit_p(ezx_pcap_remove),
	.driver = {
		.name   = "ezx-pcap",
		.owner  = THIS_MODULE,
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

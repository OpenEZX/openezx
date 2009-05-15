/*
 * Driver for Motorola PCAP2 as present in EZX phones
 *
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
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
	u32 msr;
	struct work_struct work;
	struct work_struct msr_work;
	struct workqueue_struct *workqueue;
};
static struct pcap_chip pcap;

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

/* IRQ */
static inline unsigned int irq2pcap(int irq)
{
	return 1 << (irq - PCAP_IRQ(0));
}

static void pcap_mask_irq(unsigned int irq)
{
	pcap.msr |= irq2pcap(irq);
	queue_work(pcap.workqueue, &pcap.msr_work);
}

static void pcap_unmask_irq(unsigned int irq)
{
	pcap.msr &= ~irq2pcap(irq);
	queue_work(pcap.workqueue, &pcap.msr_work);
}

static struct irq_chip pcap_irq_chip = {
	.name	= "pcap",
	.mask	= pcap_mask_irq,
	.unmask	= pcap_unmask_irq,
};

static void pcap_msr_work(struct work_struct *msr_work)
{
	ezx_pcap_write(PCAP_REG_MSR, pcap.msr);
}

static void pcap_work(struct work_struct *_pcap)
{
	u32 msr, isr, service;
	int irq;

	ezx_pcap_read(PCAP_REG_MSR, &msr);
	ezx_pcap_read(PCAP_REG_ISR, &isr);
	ezx_pcap_write(PCAP_REG_ISR, isr);

	local_irq_disable();
	service = isr & ~msr;

	for (irq = PCAP_IRQ(0); service; service >>= 1, irq++) {
		if (service & 1) {
			struct irq_desc *desc = irq_to_desc(irq);

			if (WARN(!desc, KERN_WARNING
					"Invalid PCAP IRQ %d\n", irq))
				break;

			if (desc->status & IRQ_DISABLED)
				note_interrupt(irq, desc, IRQ_NONE);
			else
				desc->handle_irq(irq, desc);
		}
	}
	local_irq_enable();
}

static void pcap_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	desc->chip->ack(irq);
	queue_work(pcap.workqueue, &pcap.work);
	return;
}

static irqreturn_t pcap_test_irq(int irq, void *data)
{
	u32 tmp;
	printk("%s: %d\n", __func__, irq);
	ezx_pcap_read(0, &tmp);
	printk("%s: %08x\n", __func__, tmp);

	return IRQ_HANDLED;
}






/* subdevs */
static int pcap_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int pcap_add_subdev(struct spi_device *spi, struct pcap_subdev *subdev)
{
	struct platform_device *pdev;

	pdev = platform_device_alloc(subdev->name, subdev->id);
	pdev->dev.parent = &spi->dev;
	pdev->dev.platform_data = subdev->platform_data;

	return platform_device_add(pdev);
}

static int __devexit ezx_pcap_remove(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;

	destroy_workqueue(pcap.workqueue);
	free_irq(pdata->irq, NULL);

	/* remove all registered subdevs */
	device_for_each_child(&spi->dev, NULL, pcap_remove_subdev);

	pcap.spi = NULL;

	return 0;
}

static int __devinit ezx_pcap_probe(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;
	int i;
	u32 t;
	int ret = -ENODEV;

	/* platform data is required */
	if (!pdata)
		goto ret;

	/* we support only one pcap device */
	if (pcap.spi) {
		ret = -EBUSY;
		goto ret;
	}

	/* setup spi */
	spi->bits_per_word = 32;
	spi->mode = SPI_MODE_0 | (pdata->config & PCAP_CS_AH ? SPI_CS_HIGH : 0);
	ret = spi_setup(spi);
	if (ret)
		goto ret;

	pcap.spi = spi;

	/* setup irq */
	INIT_WORK(&pcap.work, pcap_work);
	INIT_WORK(&pcap.msr_work, pcap_msr_work);
	pcap.workqueue = create_singlethread_workqueue("pcapd");
	if (!pcap.workqueue) {
		dev_err(&spi->dev, "cant create pcap thread\n");
		goto null_spi;
	}

	/* redirect interrupts to AP */
	if (!(pdata->config & PCAP_SECOND_PORT))
		ezx_pcap_write(PCAP_REG_INT_SEL, 0);

	/* setup irq chip */
	for (i = PCAP_IRQ(0); i <= PCAP_LAST_IRQ; i++) {
		set_irq_chip_and_handler(i, &pcap_irq_chip, handle_simple_irq);
#ifdef CONFIG_ARM
		set_irq_flags(i, IRQF_VALID);
#else
		set_irq_noprobe(i);
#endif
	}

	/* mask/ack all PCAP interrupts */
	ezx_pcap_write(PCAP_REG_MSR, PCAP_MASK_ALL_INTERRUPT);
	ezx_pcap_write(PCAP_REG_ISR, PCAP_CLEAR_INTERRUPT_REGISTER);
	pcap.msr = PCAP_MASK_ALL_INTERRUPT;

	set_irq_type(pdata->irq, IRQ_TYPE_EDGE_RISING);
	set_irq_chained_handler(pdata->irq, pcap_irq_handler);
	set_irq_wake(pdata->irq, 1);

	/* setup subdevs */
	for (i = 0; i < pdata->num_subdevs; i++) {
		ret = pcap_add_subdev(spi, pdata->subdevs[i]);
		if (ret)
			goto remove_subdevs;
	}

	/* board specific quirks */
	if (pdata->init)
		pdata->init();

	/* test irq */
	ret = request_irq(PCAP_IRQ_1HZ, pcap_test_irq, IRQF_DISABLED,
						"1HZ", NULL);
	if (ret)
		printk("error requesting test irq\n");
	ret = request_irq(PCAP_IRQ_TS, pcap_test_irq, IRQF_DISABLED,
						"TS", NULL);
	if (ret)
		printk("error requesting test irq\n");

	for (i = 0; i <= 31; i++) {
		ezx_pcap_read(i, &t);
		printk("%s: %d %08x\n", __func__, i, t);
	}
		


	return 0;

remove_subdevs:
	device_for_each_child(&spi->dev, NULL, pcap_remove_subdev);
	for (i = PCAP_IRQ(0); i <= PCAP_LAST_IRQ; i++)
		set_irq_chip_and_handler(i, NULL, NULL);
	destroy_workqueue(pcap.workqueue);
	pcap.workqueue = NULL;
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

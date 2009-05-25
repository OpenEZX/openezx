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
#include <linux/mfd/ezx-pcap.h>
#include <linux/spi/spi.h>

struct pcap_chip {
	struct spi_device *spi;
	unsigned int irq_base;
	u32 msr;
	struct work_struct work;
	struct work_struct msr_work;
	struct workqueue_struct *workqueue;
	u32 *buf;
};
static struct pcap_chip pcap;

/* IO */
#define PCAP_BUFSIZE	4
static int ezx_pcap_putget(u32 *data)
{
	struct spi_transfer t;
	struct spi_message m;
	int status;

	memset(&t, 0, sizeof t);
	spi_message_init(&m);
	t.len = PCAP_BUFSIZE;
	spi_message_add_tail(&t, &m);

	*pcap.buf = *data;
	t.tx_buf = (u8 *) pcap.buf;
	t.rx_buf = (u8 *) pcap.buf;
	status = spi_sync(pcap.spi, &m);

	if (status == 0)
		*data = *pcap.buf;

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
	return 1 << (irq - pcap.irq_base);
}

int pcap_irq(int irq)
{
	return pcap.irq_base + irq;
}
EXPORT_SYMBOL_GPL(pcap_irq);

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
	struct pcap_platform_data *pdata = pcap.spi->dev.platform_data;
	u32 msr, isr, int_sel, service;
	int irq;

	ezx_pcap_read(PCAP_REG_MSR, &msr);
	ezx_pcap_read(PCAP_REG_ISR, &isr);

	/* We cant service/ack irqs that are assigned to port 2 */
	if (!(pdata->config & PCAP_SECOND_PORT)) {
		ezx_pcap_read(PCAP_REG_INT_SEL, &int_sel);
		isr &= ~int_sel;
	}
	ezx_pcap_write(PCAP_REG_ISR, isr);

	local_irq_disable();
	service = isr & ~msr;

	for (irq = pcap.irq_base; service; service >>= 1, irq++) {
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
	int i;

	for (i = pcap.irq_base; i < (pcap.irq_base + PCAP_NIRQS); i++)
		set_irq_chip_and_handler(i, NULL, NULL);

	destroy_workqueue(pcap.workqueue);

	/* remove all registered subdevs */
	device_for_each_child(&spi->dev, NULL, pcap_remove_subdev);

	pcap.spi = NULL;
	kfree(pcap.buf);

	return 0;
}

static int __devinit ezx_pcap_probe(struct spi_device *spi)
{
	struct pcap_platform_data *pdata = spi->dev.platform_data;
	int i;
	int ret = -ENODEV;

	/* platform data is required */
	if (!pdata)
		goto ret;

	/* we support only one pcap device */
	if (pcap.spi) {
		ret = -EBUSY;
		goto ret;
	}

	pcap.buf = kmalloc(PCAP_BUFSIZE, GFP_KERNEL);
	if (!pcap.buf) {
		ret = -ENOMEM;
		goto ret;
	}

	/* setup spi */
	spi->bits_per_word = 32;
	spi->mode = SPI_MODE_0 | (pdata->config & PCAP_CS_AH ? SPI_CS_HIGH : 0);
	ret = spi_setup(spi);
	if (ret)
		goto free_buf;

	pcap.spi = spi;

	/* setup irq */
	pcap.irq_base = pdata->irq_base;
	INIT_WORK(&pcap.work, pcap_work);
	INIT_WORK(&pcap.msr_work, pcap_msr_work);
	pcap.workqueue = create_singlethread_workqueue("pcapd");
	if (!pcap.workqueue) {
		dev_err(&spi->dev, "cant create pcap thread\n");
		goto null_spi;
	}

	/* redirect interrupts to AP, except adcdone2 */
	if (!(pdata->config & PCAP_SECOND_PORT))
		ezx_pcap_write(PCAP_REG_INT_SEL, PCAP_IRQ_ADCDONE2);

	/* setup irq chip */
	for (i = pcap.irq_base; i < (pcap.irq_base + PCAP_NIRQS); i++) {
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
		ret = pcap_add_subdev(spi, &pdata->subdevs[i]);
		if (ret)
			goto remove_subdevs;
	}

	/* board specific quirks */
	if (pdata->init)
		pdata->init();

	return 0;

remove_subdevs:
	device_for_each_child(&spi->dev, NULL, pcap_remove_subdev);
	for (i = pcap.irq_base; i < (pcap.irq_base + PCAP_NIRQS); i++)
		set_irq_chip_and_handler(i, NULL, NULL);
	destroy_workqueue(pcap.workqueue);
	pcap.workqueue = NULL;
null_spi:
	pcap.spi = NULL;
free_buf:
	kfree(pcap.buf);
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

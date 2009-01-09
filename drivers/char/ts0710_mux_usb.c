/*
 * linux/drivers/usb/ipcusb.c
 *
 * Implementation of a ipc driver based Intel's Bulverde USB Host
 * Controller.
 *
 * Copyright (C) 2003-2005 Motorola
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  2003-Nov-03 - (Motorola) created
 *  2004-Feb-20 - (Motorola) Add Power Manager codes
 *  2004-Apr-14 - (Motorola) Update Suspend/Resume codes
 *  2004-May-10 - (Motorola) Add unlink_urbs codes and do some updates of send
 *			     out urb sequence
 *  2006-Jun-22 - (Harald Welte) port to Linux 2.6.x
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/pxa-regs.h>
#include <mach/ezx-bp.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>
#include <linux/usb.h>

#include "ts0710_mux_usb.h"

/*Macro defined for this driver*/
#define DRIVER_VERSION "1.0alpha1"
#define DRIVER_AUTHOR "Motorola / Harald Welte <laforge@openezx.org>"
#define DRIVER_DESC "USB IPC Driver (TS07.10 lowlevel)"
#define MOTO_IPC_VID		0x22b8
#define MOTO_IPC_PID		0x3006
#define IBUF_SIZE 		32		/*urb size*/
#define IPC_USB_XMIT_SIZE	1024
#define IPC_URB_SIZE		32
#define IPC_USB_WRITE_INIT 	0
#define IPC_USB_WRITE_XMIT	1
#define IPC_USB_PROBE_READY	3
#define IPC_USB_PROBE_NOT_READY	4
#define DBG_MAX_BUF_SIZE	1024
#define ICL_EVENT_INTERVAL	(HZ)
//#define BVD_DEBUG

#define IS_EP_BULK(ep)  ((ep).bmAttributes == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)
/*End defined macro*/

/*global values defined*/
static struct usb_driver 		usb_ipc_driver;
static struct timer_list 		ipcusb_timer;
static struct timer_list 		suspend_timer;
static struct timer_list 		wakeup_timer;
static struct tty_struct		ipcusb_tty;		/* the coresponding tty struct, we just use flip buffer here. */
static struct tty_driver		ipcusb_tty_driver;	/* the coresponding tty driver, we just use write and chars in buff here*/
struct tty_driver *usb_for_mux_driver = NULL;
struct tty_struct *usb_for_mux_tty = NULL;
void (*usb_mux_dispatcher)(struct tty_struct *tty) = NULL;
void (*usb_mux_sender)(void) = NULL;
void (*ipcusb_ap_to_bp)(unsigned char*, int) = NULL;
void (*ipcusb_bp_to_ap)(unsigned char*, int) = NULL;
EXPORT_SYMBOL(usb_for_mux_driver);
EXPORT_SYMBOL(usb_for_mux_tty);
EXPORT_SYMBOL(usb_mux_dispatcher);
EXPORT_SYMBOL(usb_mux_sender);
EXPORT_SYMBOL(ipcusb_ap_to_bp);
EXPORT_SYMBOL(ipcusb_bp_to_ap);
static int sumbit_times = 0;
static int callback_times = 0;
//static unsigned long last_jiff = 0;
void __iomem *__iobase;
#define UHCRHPS3 (__iobase+0x005c)
/*end global values defined*/

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#ifdef BVD_DEBUG
#define bvd_dbg(format, arg...) printk(__FILE__ ": " format "\n" , ## arg)
#else
#define bvd_dbg(format, arg...) do {} while (0)
#endif

/* USB device context */
typedef struct {
	struct list_head list;
	int size;
	char *body;
} buf_list_t;

struct ipc_usb_data {
	u_int8_t 		write_finished_flag;
	u_int8_t		write_flag,
				ipc_flag,
				suspend_flag;
	struct usb_device 	*ipc_dev;
	struct urb 		readurb_mux,
				writeurb_mux,
				writeurb_dsplog;
	char 			*obuf, *ibuf;
	int			writesize;	/* max packet size for the
						   output bulk endpoint *
						   transfer buffers */

	struct circ_buf		xmit;		/* write cric bufffer */
  	struct list_head 	in_buf_list;
	spinlock_t		in_buf_lock;
	char 			bulk_in_ep_mux,
				bulk_out_ep_mux,
				bulk_in_ep_dsplog;
	unsigned int 		ifnum;

	struct tasklet_struct	bh,
				bh_bp;

	spinlock_t		lock;
};

struct ipc_usb_data *bvd_ipc;

#ifdef BVD_DEBUG
static void bvd_dbg_hex(__u8 *buf, int len)
{
	static unsigned char tbuf[DBG_MAX_BUF_SIZE];
	int i, c;

	if (len <= 0)
		return;

	c = 0;
	for (i=0; (i < len) && (c < (DBG_MAX_BUF_SIZE - 3)); i++) {
		sprintf(&tbuf[c], "%02x ",buf[i]);
		c += 3;
	}
	tbuf[c] = 0;

	printk("%s: %s\n", __FUNCTION__, tbuf);
}
#else
#define bvd_dbg_hex(buf, len)
#endif

static int unlink_urbs(struct urb *urb)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&bvd_ipc->lock, flags);

	retval = usb_unlink_urb(urb);
	if (retval != -EINPROGRESS && retval != 0)
		printk("unlink urb err, %d", retval);

	spin_unlock_irqrestore(&bvd_ipc->lock, flags);
	return retval;
}

static void append_to_inbuf_list(struct urb *urb)
{
	buf_list_t *inbuf;
	int count = urb->actual_length;

	// we are called from interrupt context.
	inbuf = kmalloc(sizeof(buf_list_t), GFP_ATOMIC);
	if (!inbuf) {
		printk("append_to_inbuf_list: (%d) out of memory!\n",
			sizeof(buf_list_t));
		return;
	}

	inbuf->size = count;
	inbuf->body = kmalloc(sizeof(char)*count, GFP_ATOMIC);
	if (!inbuf->body) {
		kfree(inbuf);
		printk("append_to_inbuf_list: (%d) out of memory!\n",
			sizeof(char)*count);
		return;
	}
	memcpy(inbuf->body, (unsigned char*)urb->transfer_buffer, count);
	spin_lock(&bvd_ipc->in_buf_lock);
	list_add_tail(&inbuf->list, &bvd_ipc->in_buf_list);
	spin_unlock(&bvd_ipc->in_buf_lock);
}

int get_from_inbuf_list(const unsigned char *buf, int dst_count)
{
	int ret = 0;
	spin_lock(&bvd_ipc->in_buf_lock);
	if (!(list_empty(&bvd_ipc->in_buf_list))) {
		int src_count;
		buf_list_t *inbuf;
		struct list_head *ptr;

		ptr = bvd_ipc->in_buf_list.next;
		inbuf = list_entry(ptr, buf_list_t, list);
		src_count = inbuf->size;
		if (dst_count >= src_count) {
			memcpy((unsigned char *)buf, inbuf->body, src_count);
			ret = src_count;
			list_del(ptr);
			kfree(inbuf->body);
			kfree(inbuf);
		} else {
			bvd_dbg("get_from_inbuf_list: not enough space in destination buffer");
		}
	}
	spin_unlock(&bvd_ipc->in_buf_lock);

	return ret;
}
EXPORT_SYMBOL(get_from_inbuf_list);

static void ipcusb_timeout(unsigned long data)
{
	struct tty_struct *tty = &ipcusb_tty;
	struct urb *urb = (struct urb *)data;

	bvd_dbg("ipcusb_timeout***");

	spin_lock(&bvd_ipc->in_buf_lock);
	while (!(list_empty(&bvd_ipc->in_buf_list))) {
		int count;
		buf_list_t *inbuf;
		struct list_head *ptr = NULL;

		ptr = bvd_ipc->in_buf_list.next;
		inbuf = list_entry(ptr, buf_list_t, list);
		count = inbuf->size;
		if (tty_insert_flip_string(tty, inbuf->body, count) >= count) {
			list_del(ptr);
			kfree(inbuf->body);
			inbuf->body = NULL;
			kfree(inbuf);
		} else {
			bvd_dbg("ipcusb_timeout: bvd_ipc->in_buf_list empty!");
			break;
		}
	}
	spin_unlock(&bvd_ipc->in_buf_lock);

	if (usb_mux_dispatcher)
		usb_mux_dispatcher(tty);	/**call Liu changhui's func.**/

	spin_lock(&bvd_ipc->in_buf_lock);
	if (list_empty(&bvd_ipc->in_buf_list)) {
		urb->actual_length = 0;
		urb->dev = bvd_ipc->ipc_dev;
		if (usb_submit_urb(urb, GFP_ATOMIC))
			bvd_dbg("ipcusb_timeout: failed resubmitting read urb");
		bvd_dbg("ipcusb_timeout: resubmited read urb");
	} else {
		ipcusb_timer.data = (unsigned long)urb;
		mod_timer(&ipcusb_timer, jiffies+(10*HZ/1000));
	}
	spin_unlock(&bvd_ipc->in_buf_lock);
}

static void usb_ipc_read_bulk(struct urb *urb)
{
	int count = urb->actual_length;
	struct tty_struct *tty = &ipcusb_tty;

 	bvd_dbg("usb_ipc_read_bulk: begining!");
	if (urb->status)
		printk("nonzero read bulk status received: %d\n", urb->status);

 	bvd_dbg("usb_ipc_read_bulk: urb->actual_length=%d", urb->actual_length);
 	bvd_dbg("usb_ipc_read_bulk: urb->transfer_buffer:");

	bvd_dbg_hex((unsigned char*)urb->transfer_buffer, urb->actual_length);

	if (count > 0 && ((*ipcusb_bp_to_ap) != NULL))
		(*ipcusb_bp_to_ap)(urb->transfer_buffer, urb->actual_length);

 	if (count > 0) {
 		bvd_dbg("usb_ipc_read_bulk: inserting buffer into in_buf_list");
		bvd_ipc->suspend_flag = 1;

		append_to_inbuf_list(urb);

		if (usb_mux_dispatcher)
			usb_mux_dispatcher(tty); /* call Liu changhui's func. */

		urb->actual_length = 0;
		urb->dev = bvd_ipc->ipc_dev;
		if (usb_submit_urb(urb, GFP_ATOMIC))
			bvd_dbg("failed resubmitting read urb");
		bvd_dbg("usb_ipc_read_bulk: resubmited read urb");
	}

	bvd_dbg("usb_ipc_read_bulk: completed!!!");
}

static void usb_ipc_write_bulk(struct urb *urb)
{
	callback_times++;
	bvd_ipc->write_finished_flag = 1;

	bvd_dbg("usb_ipc_write_bulk: begining!");
	//printk("%s: write_finished_flag=%d\n", __FUNCTION__, bvd_ipc->write_finished_flag);

	if (urb->status)
		printk("nonzero write bulk status received: %d\n", urb->status);

	if (usb_mux_sender)
		usb_mux_sender();		/**call Liu changhui's func**/

	//printk("usb_ipc_write_bulk: mark ipcusb_softint!\n");
	tasklet_schedule(&bvd_ipc->bh);

	bvd_dbg("usb_ipc_write_bulk: finished!");
}

static void wakeup_timeout(unsigned long data)
{
//	GPSR(GPIO_MCU_INT_SW) = GPIO_bit(GPIO_MCU_INT_SW);
	bvd_dbg("wakup_timeout: send GPIO_MCU_INT_SW signal!");
}

static void suspend_timeout(unsigned long data)
{
	if (bvd_ipc->suspend_flag == 1) {
		bvd_ipc->suspend_flag = 0;
		mod_timer(&suspend_timer, jiffies+(5000*HZ/1000));
		bvd_dbg("suspend_timeout: add the suspend timer again");
	} else {
		unlink_urbs(&bvd_ipc->readurb_mux);
		__raw_writel(0x4, UHCRHPS3);
		mdelay(40);
		bvd_dbg("suspend_timeout: send SUSPEND signal! UHCRHPS3=0x%x",
			__raw_readl(UHCRHPS3));
	}
}

static void ipcusb_xmit_data(void)
{
	int c, count = IPC_URB_SIZE;
	int result = 0;
	int buf_flag = 0;
	int buf_num = 0;

	//printk("%s: sumbit_times=%d, callback_times=%d\n", __FUNCTION__, sumbit_times, callback_times);
	if (bvd_ipc->write_finished_flag == 0)
		return;

	while (1) {
		c = CIRC_CNT_TO_END(bvd_ipc->xmit.head, bvd_ipc->xmit.tail,
				    IPC_USB_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		memcpy(bvd_ipc->obuf+buf_num,
		       bvd_ipc->xmit.buf + bvd_ipc->xmit.tail, c);
		buf_flag = 1;
		bvd_ipc->xmit.tail = ((bvd_ipc->xmit.tail + c)
						& (IPC_USB_XMIT_SIZE-1));
		count -= c;
		buf_num += c;
	}

	if (buf_num == 0) {
		bvd_dbg("ipcusb_xmit_data: buf_num=%d, add suspend_timer",
			buf_num);
		bvd_ipc->suspend_flag = 0;
		mod_timer(&suspend_timer, jiffies+(5000*HZ/1000));
	}

	bvd_dbg("ipcusb_xmit_data: buf_num=%d", buf_num);
	bvd_dbg("ipcusb_xmit_data: bvd_ipc->obuf: ");

	bvd_dbg_hex((bvd_ipc->obuf)-buf_num, buf_num);

	if (buf_flag) {
		bvd_ipc->writeurb_mux.transfer_buffer_length = buf_num;
		bvd_dbg("ipcusb_xmit_data: copy data to write urb finished! ");

		if ((__raw_readl(UHCRHPS3) & 0x4) == 0x4) {
			int ret;

			ezx_wake_bp();

			/* Resume BP */
			__raw_writel(0x8, UHCRHPS3);
			mdelay(40);
			bvd_dbg("ipcusb_xmit_data: Send RESUME signal! UHCRHPS3=0x%x",
				 __raw_readl(UHCRHPS3));
			/*send IN token*/
			bvd_ipc->readurb_mux.actual_length = 0;
			bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
			if ((ret = usb_submit_urb(&bvd_ipc->readurb_mux, GFP_ATOMIC)))
				printk("ipcusb_xmit_data: usb_submit_urb(read mux bulk)"
					"failed! status=%d\n", ret);
			bvd_dbg("ipcusb_xmit_data: Send a IN token successfully!");
		}

		sumbit_times++;
		bvd_ipc->write_finished_flag = 0;
		//printk("%s: clear write_finished_flag:%d\n", __FUNCTION__, bvd_ipc->write_finished_flag);
		bvd_ipc->writeurb_mux.dev = bvd_ipc->ipc_dev;
		if ((result = usb_submit_urb(&bvd_ipc->writeurb_mux, GFP_ATOMIC)))
			printk("ipcusb_xmit_data: funky result! result=%d\n", result);

		bvd_dbg("ipcusb_xmit_data: usb_submit_urb finished! result:%d", result);

	}
}

static void usbipc_bh_func(unsigned long param)
{
	ipcusb_xmit_data();
}

//extern void get_halted_bit(void);

static void usbipc_bh_bp_func(unsigned long param)
{
	if ((__raw_readl(UHCRHPS3) & 0x4) == 0x4) {
		__raw_writel(0x8, UHCRHPS3);
		mdelay(40);
		bvd_dbg("ipcusb_softint_send_readurb: Send RESUME signal! "
			"UHCRHPS3=0x%x", __raw_readl(UHCRHPS3));
	}
	if (bvd_ipc->ipc_flag == IPC_USB_PROBE_READY) {
		//get_halted_bit();

		/*send a IN token*/
		bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
		if (usb_submit_urb(&bvd_ipc->readurb_mux, GFP_ATOMIC)) {
			bvd_dbg("ipcusb_softint_send_readurb: "
				"usb_submit_urb(read mux bulk) failed!");
		}
		bvd_dbg("ipcusb_softint_send_readurb: Send a IN token successfully!");
		bvd_ipc->suspend_flag = 0;
		bvd_dbg("ipcusb_softint_send_readurb: add suspend_timer");
		mod_timer(&suspend_timer, jiffies+(5000*HZ/1000));
	}
}

static int usb_ipc_write(struct tty_struct *tty,
			 const unsigned char *buf, int count)
{
	int c, ret = 0;

	bvd_dbg("usb_ipc_write: count=%d, buf: ", count);
	bvd_dbg_hex(buf, count);

	if (count <= 0)
		return 0;

	if (*ipcusb_ap_to_bp != NULL)
		(*ipcusb_ap_to_bp)((unsigned char *)buf, count);

	bvd_ipc->suspend_flag = 1;

	if ((bvd_ipc->ipc_flag == IPC_USB_PROBE_READY) &&
	    (bvd_ipc->xmit.head == bvd_ipc->xmit.tail)) {
		bvd_dbg("usb_ipc_write: set write_flag");
		bvd_ipc->write_flag = IPC_USB_WRITE_XMIT;
	}

	while (1) {
		c = CIRC_SPACE_TO_END(bvd_ipc->xmit.head,
				      bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		memcpy(bvd_ipc->xmit.buf + bvd_ipc->xmit.head, buf, c);
		bvd_ipc->xmit.head = ((bvd_ipc->xmit.head + c)
						& (IPC_USB_XMIT_SIZE-1));
		buf += c;
		count -= c;
		ret += c;
	}
	bvd_dbg("usb_ipc_write: ret=%d, bvd_ipc->xmit.buf: ", ret);

	bvd_dbg_hex(bvd_ipc->xmit.buf, ret);

	if (bvd_ipc->write_flag == IPC_USB_WRITE_XMIT) {
		bvd_ipc->write_flag = IPC_USB_WRITE_INIT;
		bvd_dbg("usb_ipc_write: mark ipcusb_softint");
		tasklet_schedule(&bvd_ipc->bh);
	}

	bvd_dbg("usb_ipc_write: ret=%d\n", ret);
	return ret;
}

static int usb_ipc_chars_in_buffer(struct tty_struct *tty)
{
	return CIRC_CNT(bvd_ipc->xmit.head, bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
}

static int is_probed=0;
void usb_send_readurb(void)
{
	//printk("usb_send_readurb: begining!UHCRHPS3=0x%x, usbh_finished_resume=%d\n", UHCRHPS3, usbh_finished_resume);
	if(!is_probed)
	    return;
	tasklet_schedule(&bvd_ipc->bh_bp);
}

static int usb_ipc_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_config_descriptor *ipccfg;
	struct usb_interface_descriptor *interface;
	struct usb_endpoint_descriptor *endpoint;
	int ep_cnt, readsize, writesize;
	char have_bulk_in_mux, have_bulk_out_mux;

	bvd_dbg("usb_ipc_probe: vendor id 0x%x, device id 0x%x",
		usbdev->descriptor.idVendor, usbdev->descriptor.idProduct);

	if ((usbdev->descriptor.idVendor != MOTO_IPC_VID) ||
	    (usbdev->descriptor.idProduct != MOTO_IPC_PID))
 		return -ENODEV;

	/* a2590c : dsplog interface is not supported by this driver */
	if (intf->minor == 2)	/* dsplog interface number is 2 */
		return -1;

	bvd_dbg("usb_ipc_probe: USB dev address:%p", usbdev);
	bvd_dbg("usb_ipc_probe: ifnum:%u", intf->minor);

	ipccfg = &usbdev->actconfig->desc;
	bvd_dbg("usb_ipc_prob: config%d", ipccfg->bConfigurationValue);
	bvd_dbg("usb_ipc_prob: bNumInterfaces = %d", ipccfg->bNumInterfaces);

	/* After this point we can be a little noisy about what we are trying
	 * to configure, hehe.  */
	if (usbdev->descriptor.bNumConfigurations != 1) {
		printk("usb_ipc_probe: Only one device configuration "
		     "is supported.");
		return -1;
	}

	if (usbdev->config[0].desc.bNumInterfaces != 3) {
		printk("usb_ipc_probe: Only three device interfaces are "
		     "supported.");
		return -1;
	}

	interface = &intf->cur_altsetting->desc;
	/* Start checking for two bulk endpoints or ... FIXME: This is a future
	 * enhancement...*/
	bvd_dbg("usb_ipc_probe: Number of Endpoints:%d",
		(int) interface->bNumEndpoints);
	if (interface->bNumEndpoints != 2) {
		printk("usb_ipc_probe: Only two endpoints supported.");
		return -1;
	}

	ep_cnt = have_bulk_in_mux = have_bulk_out_mux = 0;
	readsize = writesize = 0;

	while (ep_cnt < interface->bNumEndpoints) {
		endpoint = &intf->cur_altsetting->endpoint[ep_cnt].desc;
		bvd_dbg("usb_ipc_probe: endpoint[%i] is: %x", ep_cnt,
			endpoint->bEndpointAddress);

		if (!have_bulk_in_mux && IS_EP_BULK_IN(*endpoint)) {
			bvd_dbg("usb_ipc_probe: bEndpointAddress(IN) is: %x ",
				endpoint->bEndpointAddress);
			have_bulk_in_mux =  endpoint->bEndpointAddress;
			readsize = endpoint->wMaxPacketSize;
			bvd_dbg("usb_ipc_probe: readsize=%d", readsize);
			ep_cnt++;
			continue;
		}

		if (!have_bulk_out_mux && IS_EP_BULK_OUT(*endpoint)) {
			bvd_dbg("usb_ipc_probe: bEndpointAddress(OUT) is: %x ",
				endpoint->bEndpointAddress);
			have_bulk_out_mux = endpoint->bEndpointAddress;
			writesize = endpoint->wMaxPacketSize;
			bvd_dbg("usb_ipc_probe: writesize=%d", writesize);
			ep_cnt++;
			continue;
		}

		printk("usb_ipc_probe: Undetected endpoint ^_^ ");
		/* Shouldn't ever get here unless we have something weird */
		return -1;
	}

	/* Perform a quick check to make sure that everything worked as it
	 * should have.  */

	switch (interface->bNumEndpoints) {
	case 2:
		if (!have_bulk_in_mux || !have_bulk_out_mux) {
			printk("usb_ipc_probe: Two bulk endpoints required.");
			return -1;
		}
		break;
	default:
		printk("usb_ipc_probe: Endpoint determination failed ^_^ ");
		return -1;
	}

	/* Ok, now initialize all the relevant values */
	if (!(bvd_ipc->obuf = (char *)kmalloc(writesize, GFP_KERNEL))) {
		err("usb_ipc_probe: Not enough memory for the output buffer.");
		kfree(bvd_ipc);
		return -1;
	}
	bvd_dbg("usb_ipc_probe: obuf address:%p", bvd_ipc->obuf);

	if (!(bvd_ipc->ibuf = (char *)kmalloc(readsize, GFP_KERNEL))) {
		err("usb_ipc_probe: Not enough memory for the input buffer.");
		kfree(bvd_ipc->obuf);
		kfree(bvd_ipc);
		return -1;
	}
	bvd_dbg("usb_ipc_probe: ibuf address:%p", bvd_ipc->ibuf);

	bvd_ipc->ipc_flag = IPC_USB_PROBE_READY;
	bvd_ipc->write_finished_flag = 1;
	bvd_ipc->suspend_flag = 1;
	bvd_ipc->bulk_in_ep_mux= have_bulk_in_mux;
	bvd_ipc->bulk_out_ep_mux= have_bulk_out_mux;
	bvd_ipc->ipc_dev = usbdev;
	bvd_ipc->writesize = writesize;
	INIT_LIST_HEAD(&bvd_ipc->in_buf_list);
	bvd_ipc->in_buf_lock = SPIN_LOCK_UNLOCKED;

	bvd_ipc->bh.func = usbipc_bh_func;
	bvd_ipc->bh.data = (unsigned long) bvd_ipc;

	bvd_ipc->bh_bp.func = usbipc_bh_bp_func;
	bvd_ipc->bh_bp.data = (unsigned long) bvd_ipc;

	bvd_dbg("after assignements");
	/*Build a write urb*/
	usb_init_urb(&bvd_ipc->writeurb_mux);
	usb_fill_bulk_urb(&bvd_ipc->writeurb_mux, usbdev,
			  usb_sndbulkpipe(bvd_ipc->ipc_dev,
			  		  bvd_ipc->bulk_out_ep_mux),
			  bvd_ipc->obuf, writesize, usb_ipc_write_bulk,
			  bvd_ipc);
	//bvd_ipc->writeurb_mux.transfer_flags |= USB_ASYNC_UNLINK;
	bvd_dbg("after write urb");

	/*Build a read urb and send a IN token first time*/
	usb_init_urb(&bvd_ipc->readurb_mux);
	usb_fill_bulk_urb(&bvd_ipc->readurb_mux, usbdev,
			  usb_rcvbulkpipe(usbdev, bvd_ipc->bulk_in_ep_mux),
			  bvd_ipc->ibuf, readsize, usb_ipc_read_bulk, bvd_ipc);
	//bvd_ipc->readurb_mux.transfer_flags |= USB_ASYNC_UNLINK;
	bvd_dbg("after read urb");

	//usb_driver_claim_interface(&usb_ipc_driver, intf, bvd_ipc);
	bvd_dbg("after claim interface");
	//usb_driver_claim_interface(&usb_ipc_driver, &ipccfg->interface[1], bvd_ipc);

        // a2590c: dsplog is not supported by this driver
	//	usb_driver_claim_interface(&usb_ipc_driver,
	//				   &ipccfg->interface[2], bvd_ipc);
	/*send a IN token first time*/
	bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
	bvd_dbg("after assignement");

	if (usb_submit_urb(&bvd_ipc->readurb_mux, GFP_ATOMIC))
		printk("usb_ipc_prob: usb_submit_urb(read mux bulk) failed!\n");

	bvd_dbg("usb_ipc_prob: Send a IN token successfully!");

	if (bvd_ipc->xmit.head != bvd_ipc->xmit.tail) {
		printk("usb_ipc_probe: mark ipcusb_softint!\n");
		tasklet_schedule(&bvd_ipc->bh);
	}

	printk("usb_ipc_probe: completed probe!\n");
	usb_set_intfdata(intf, &bvd_ipc);
	is_probed=1;
	
	return 0;
}

static void usb_ipc_disconnect(struct usb_interface *intf)
{
	//struct usb_device *usbdev = interface_to_usbdev(intf);
	struct ipc_usb_data *bvd_ipc_disconnect = usb_get_intfdata(intf);


	printk("usb_ipc_disconnect. bvd_ipc_disconnect address: %p\n", bvd_ipc_disconnect);

	//FIXME: Memory leak?
	if ((__raw_readl(UHCRHPS3) & 0x4) == 0)
		usb_unlink_urb(&bvd_ipc_disconnect->readurb_mux);

	usb_unlink_urb(&bvd_ipc_disconnect->writeurb_mux);

	bvd_ipc_disconnect->ipc_flag = IPC_USB_PROBE_NOT_READY;
	kfree(bvd_ipc_disconnect->ibuf);
	kfree(bvd_ipc_disconnect->obuf);

	//usb_driver_release_interface(&usb_ipc_driver,
	//		bvd_ipc_disconnect->ipc_dev->actconfig->interface[0]);
        //usb_driver_release_interface(&usb_ipc_driver,
	//		bvd_ipc_disconnect->ipc_dev->actconfig->interface[1]);

	//a2590c: dsplog interface is not supported by this driver
	//usb_driver_release_interface(&usb_ipc_driver, &bvd_ipc_disconnect->ipc_dev->actconfig->interface[2]);

	bvd_ipc_disconnect->ipc_dev = NULL;

	usb_set_intfdata(intf, NULL);

	printk("usb_ipc_disconnect completed!\n");
}

static struct usb_device_id usb_ipc_id_table[] = {
	{ USB_DEVICE(MOTO_IPC_VID, MOTO_IPC_PID) },
	{ }						/* Terminating entry */
};

static struct usb_driver usb_ipc_driver = {
	.name		= "usb ipc",
	.probe		= usb_ipc_probe,
	.disconnect	= usb_ipc_disconnect,
	.id_table	= usb_ipc_id_table,
};

struct tty_operations ipc_tty_ops={
    .write 		= usb_ipc_write,
    .chars_in_buffer 	= usb_ipc_chars_in_buffer,
};

static int __init usb_ipc_init(void)
{
	int result;

	bvd_dbg("init usb_ipc");

	__iobase = ioremap(0x4C000000,0x1000);

	/*init the related mux interface*/
	if (!(bvd_ipc = kzalloc(sizeof(struct ipc_usb_data), GFP_KERNEL))) {
		err("usb_ipc_init: Out of memory.");
		return -ENOMEM;
	}
	bvd_dbg("usb_ipc_init: Address of bvd_ipc:%p", bvd_ipc);

	if (!(bvd_ipc->xmit.buf = kmalloc(IPC_USB_XMIT_SIZE, GFP_KERNEL))) {
		err("usb_ipc_init: Not enough memory for the input buffer.");
		kfree(bvd_ipc);
		return -ENOMEM;
	}
	bvd_dbg("usb_ipc_init: bvd_ipc->xmit.buf address:%p",
		bvd_ipc->xmit.buf);
	bvd_ipc->ipc_dev = NULL;
	bvd_ipc->xmit.head = bvd_ipc->xmit.tail = 0;
	bvd_ipc->write_flag = IPC_USB_WRITE_INIT;
	spin_lock_init(&bvd_ipc->lock);
	spin_lock_init(&bvd_ipc->in_buf_lock);

	
	tty_set_operations(&ipcusb_tty_driver,&ipc_tty_ops);

	usb_for_mux_driver = &ipcusb_tty_driver;
	usb_for_mux_tty = &ipcusb_tty;

	/* register driver at the USB subsystem */
	// this was called before bvd_ipc was allocated
	result = usb_register(&usb_ipc_driver);
	if (result < 0) {
		err ("usb ipc driver could not be registered");
		return result;
	}

	/* init timers for ipcusb read process and usb suspend */
	init_timer(&ipcusb_timer);
	ipcusb_timer.function = ipcusb_timeout;

	init_timer(&suspend_timer);
	suspend_timer.function = suspend_timeout;

	init_timer(&wakeup_timer);
	wakeup_timer.function = wakeup_timeout;

	printk("USB Host(Bulverde) IPC driver registered.");
	printk(DRIVER_VERSION ":" DRIVER_DESC);

	return 0;
}

static void __exit usb_ipc_exit(void)
{
	bvd_dbg("cleanup bvd_ipc");

	kfree(bvd_ipc->xmit.buf);
	kfree(bvd_ipc);
	iounmap(__iobase);
	usb_deregister(&usb_ipc_driver);

	printk("USB Host(Bulverde) IPC driver deregistered.");
}

module_init(usb_ipc_init);
module_exit(usb_ipc_exit);
EXPORT_SYMBOL(usb_send_readurb);

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
#include <mach/pxa27x.h>
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

/*global values defined*/
static struct timer_list 		ipcusb_timer;
static struct tty_struct		ipcusb_tty;		/* the coresponding tty struct, we just use flip buffer here. */
static struct tty_driver		*ipcusb_tty_driver;	/* the coresponding tty driver, we just use write and chars in buff here*/
struct tty_driver *usb_for_mux_driver = NULL;
struct tty_struct *usb_for_mux_tty = NULL;
void (*usb_mux_sender)(void) = NULL;
EXPORT_SYMBOL(usb_for_mux_driver);
EXPORT_SYMBOL(usb_for_mux_tty);
EXPORT_SYMBOL(usb_mux_sender);

static struct usb_ipc_tty *ipc;

//#define BVD_DEBUG
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
				ipc_flag;
	struct usb_device 	*ipc_dev;
	struct urb 		readurb_mux,
				writeurb_mux,
				writeurb_dsplog;
	char 			*obuf, *ibuf;
	struct circ_buf		xmit;		/* write cric bufffer */
  	struct list_head 	in_buf_list;
	spinlock_t		in_buf_lock;
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
	struct tty_struct *tty = ipc->tty;

	if (urb->status)
		printk("read bulk status received: %d, len %d\n", urb->status, count);

	if (!count)
		return;

	bvd_dbg_hex((unsigned char*)urb->transfer_buffer, urb->actual_length);

	bvd_dbg("usb_ipc_read_bulk: inserting buffer into in_buf_list");

	tty->ldisc.ops->receive_buf(
		tty, (unsigned char*)urb->transfer_buffer,
		NULL, urb->actual_length);

	urb->actual_length = 0;
	urb->dev = bvd_ipc->ipc_dev;
	if (usb_submit_urb(urb, GFP_ATOMIC))
		bvd_dbg("failed resubmitting read urb");
}

static void usb_ipc_write_bulk(struct urb *urb)
{
	bvd_ipc->write_finished_flag = 1;

	if (urb->status)
		printk("nonzero write bulk status received: %d\n", urb->status);

	if (usb_mux_sender)
		usb_mux_sender();

	tasklet_schedule(&bvd_ipc->bh);
}

static void ipcusb_xmit_data(void)
{
	int c, count = IPC_URB_SIZE;
	int buf_flag = 0;
	int buf_num = 0;

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
	if (!buf_flag)
		return;

	bvd_dbg("ipcusb_xmit_data: buf_num=%d", buf_num);
	bvd_dbg("ipcusb_xmit_data: bvd_ipc->obuf: ");
	bvd_dbg_hex((bvd_ipc->obuf)-buf_num, buf_num);

	bvd_ipc->writeurb_mux.transfer_buffer_length = buf_num;
	ezx_wake_bp();

	bvd_ipc->write_finished_flag = 0;

	bvd_ipc->writeurb_mux.dev = bvd_ipc->ipc_dev;
	usb_submit_urb(&bvd_ipc->writeurb_mux, GFP_ATOMIC);
}

static void usbipc_bh_bp_func(unsigned long param)
{
	if (bvd_ipc->ipc_flag =! IPC_USB_PROBE_READY)
		return;

	/*send a IN token*/
	bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
	usb_submit_urb(&bvd_ipc->readurb_mux, GFP_ATOMIC);
}

static int usb_ipc_write(struct tty_struct *tty,
			 const unsigned char *buf, int count)
{
	int c, ret = 0;

	bvd_dbg("usb_ipc_write: count=%d, buf: ", count);
	bvd_dbg_hex(buf, count);

	if (count <= 0)
		return 0;

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

static int usb_ipc_write_room(struct tty_struct *tty) {
	return IPC_USB_XMIT_SIZE;
}

static int usb_ipc_chars_in_buffer(struct tty_struct *tty)
{
	return 0;//CIRC_CNT(bvd_ipc->xmit.head, bvd_ipc->xmit.tail, IPC_USB_XMIT_SIZE);
}

static int usb_ipc_open(struct tty_struct *tty, struct file *file)
{
   
    printk("open: %x, %x\n",tty,ipc);

    int index;

    /* initialize the pointer in case something fails */
    tty->driver_data = NULL;

    /* get the serial object associated with this tty pointer */
    index = tty->index;
    
    if (ipc  == NULL) {

	printk("alloc\n");
        /* first time accessing this device, let's create it */
        ipc = kmalloc(sizeof(*ipc), GFP_KERNEL);
        if (!ipc)
            return -ENOMEM;

        ipc->open_count = 0;
    }

    printk("after alloc\n");


    /* save our structure within the tty structure */
    tty->driver_data = ipc;
    ipc->tty = tty;
    ipc->open_count++;


    if (ipc->open_count == 1) {
	    printk("opened first time\n");
    }

    printk("ipc opened %d times\n",ipc->open_count);

    printk("tty %x %x\n",ipc->tty,tty);
    printk("&tty->ldisc %x\n",&tty->ldisc);
    printk("tty->ldisc.ops %x\n",tty->ldisc.ops);


    return 0;

}

static void usb_ipc_close(struct tty_struct *tty, struct file *file) {

	if (!ipc)
		return;

	if (!ipc->open_count)
		return;

	ipc->open_count--;

	if (ipc->open_count <= 0) {
		printk("closed last time\n");
	}


}

void usb_send_readurb(void)
{
	if (bvd_ipc->ipc_flag != IPC_USB_PROBE_READY)
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
	int ep_cnt, size_in, size_out;
	char ep_in, ep_out;

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
		printk("usb_ipc_probe: Only two endpoints supported got %d\n.",
				interface->bNumEndpoints
		);
		return -1;
	}

	ep_cnt = ep_in = ep_out = 0;
	size_in = size_out = 0;

	while (ep_cnt < interface->bNumEndpoints) {
		endpoint = &intf->cur_altsetting->endpoint[ep_cnt].desc;

                switch(endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
                  case USB_DIR_IN:
                    printk("in\n");
		    ep_in =  endpoint->bEndpointAddress;
		    size_in = endpoint->wMaxPacketSize;
                    break;
                  case USB_DIR_OUT:
                    printk("out\n");
		    ep_out = endpoint->bEndpointAddress;
		    size_out = endpoint->wMaxPacketSize;
                    break;
                  default:
                    printk("wtf is this? %d\n",ep_cnt);

                }


                ep_cnt++;

		printk("usb_ipc_probe: Undetected endpoint\n");
	}

        printk("endpoints: %x/%x, sizes: %d/%d\n",
            ep_in, ep_out, size_in, size_out
        );

	if (! (ep_in && ep_out) ) {
		printk("usb_ipc_probe: Two bulk endpoints required.");
		return -1;
	}
		
	/* Ok, now initialize all the relevant values */
	if (!(bvd_ipc->obuf = (char *)kmalloc(size_out, GFP_KERNEL))) {
		err("usb_ipc_probe: Not enough memory for the output buffer.");
		kfree(bvd_ipc);
		return -1;
	}
	bvd_dbg("usb_ipc_probe: obuf address:%p", bvd_ipc->obuf);

	if (!(bvd_ipc->ibuf = (char *)kmalloc(size_in, GFP_KERNEL))) {
		err("usb_ipc_probe: Not enough memory for the input buffer.");
		kfree(bvd_ipc->obuf);
		kfree(bvd_ipc);
		return -1;
	}
	bvd_dbg("usb_ipc_probe: ibuf address:%p", bvd_ipc->ibuf);

	bvd_ipc->write_finished_flag = 1;
	bvd_ipc->ipc_dev = usbdev;
	INIT_LIST_HEAD(&bvd_ipc->in_buf_list);
	bvd_ipc->in_buf_lock = SPIN_LOCK_UNLOCKED;

	bvd_ipc->bh.func = ipcusb_xmit_data;
	bvd_ipc->bh.data = (unsigned long) bvd_ipc;

	bvd_ipc->bh_bp.func = usbipc_bh_bp_func;
	bvd_ipc->bh_bp.data = (unsigned long) bvd_ipc;

	
	/*Build a write urb*/
	usb_init_urb(&bvd_ipc->writeurb_mux);
	usb_fill_bulk_urb(&bvd_ipc->writeurb_mux, usbdev,
			  usb_sndbulkpipe(usbdev, ep_out),
			  bvd_ipc->obuf, size_out, usb_ipc_write_bulk,
			  bvd_ipc);

	bvd_dbg("after write urb");

	/*Build a read urb and send a IN token first time*/
	usb_init_urb(&bvd_ipc->readurb_mux);
	usb_fill_bulk_urb(&bvd_ipc->readurb_mux, usbdev,
			  usb_rcvbulkpipe(usbdev, ep_in),
			  bvd_ipc->ibuf, size_in, usb_ipc_read_bulk, bvd_ipc);

	bvd_dbg("after read urb");

	bvd_dbg("after claim interface");

	/*send a IN token first time*/
	bvd_ipc->readurb_mux.dev = bvd_ipc->ipc_dev;
	bvd_dbg("after assignement");

	usb_set_intfdata(intf, bvd_ipc);
	bvd_ipc->ipc_flag = IPC_USB_PROBE_READY;

	if (usb_submit_urb(&bvd_ipc->readurb_mux, GFP_ATOMIC))
		printk("usb_ipc_prob: usb_submit_urb(read mux bulk) failed!\n");

	bvd_dbg("usb_ipc_prob: Send a IN token successfully!");

	if (bvd_ipc->xmit.head != bvd_ipc->xmit.tail) {
		printk("usb_ipc_probe: mark ipcusb_softint!\n");
		tasklet_schedule(&bvd_ipc->bh);
	}

	
	return 0;
}

static void usb_ipc_disconnect(struct usb_interface *intf)
{

	usb_unlink_urb(&bvd_ipc->writeurb_mux);	
	usb_unlink_urb(&bvd_ipc->readurb_mux);

	bvd_ipc->ipc_flag = IPC_USB_PROBE_NOT_READY;
	kfree(bvd_ipc->ibuf);
	kfree(bvd_ipc->obuf);

	bvd_ipc->ipc_dev = NULL;

	usb_set_intfdata(intf, NULL);

	printk("usb_ipc_disconnect completed!\n");
}

static struct usb_device_id usb_ipc_id_table[] = {
	{ USB_DEVICE(MOTO_IPC_VID, MOTO_IPC_PID) },
	{ }						/* Terminating entry */
};

static struct usb_driver usb_ipc_driver = {
	.name		= "usb_ipc",
	.probe		= usb_ipc_probe,
	.disconnect	= usb_ipc_disconnect,
	.id_table	= usb_ipc_id_table,
};

struct tty_operations ipc_tty_ops={
    .open		= usb_ipc_open,
    .close		= usb_ipc_close,
    .write 		= usb_ipc_write,
    .write_room		= usb_ipc_write_room,
    .chars_in_buffer 	= usb_ipc_chars_in_buffer,
};

static int __init usb_ipc_init(void)
{
	int result;

	bvd_dbg("init usb_ipc");

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

	ipcusb_tty_driver = alloc_tty_driver(1);
	ipcusb_tty_driver->owner = THIS_MODULE;
	ipcusb_tty_driver->driver_name = "Neptune IPC";
	ipcusb_tty_driver->name = "ttyIPC";
	ipcusb_tty_driver->major = 251;
	ipcusb_tty_driver->minor_start = 0;
	ipcusb_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	ipcusb_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	ipcusb_tty_driver->init_termios = tty_std_termios;
	ipcusb_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	ipcusb_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW;
	
	tty_set_operations(ipcusb_tty_driver, &ipc_tty_ops);
	if (tty_register_driver(ipcusb_tty_driver))
		printk("oops. cant register ipc tty\n");

	printk("registering ipc tty dev %x\n",ipcusb_tty_driver);
	if ( tty_register_device(ipcusb_tty_driver, 0, NULL))
		printk("oops cant register ipc tty dev\n");

	usb_for_mux_driver = ipcusb_tty_driver;
	usb_for_mux_tty = &ipcusb_tty;

	/* register driver at the USB subsystem */
	result = usb_register(&usb_ipc_driver);
	if (result < 0) {
		err ("usb ipc driver could not be registered");
		return result;
	}

	ipc = NULL;

	/* init timers for ipcusb read process and usb suspend */
	init_timer(&ipcusb_timer);
	ipcusb_timer.function = ipcusb_timeout;

	printk("USB Host(Bulverde) IPC driver registered.\n");
	printk(DRIVER_VERSION ":" DRIVER_DESC "\n");

	return 0;
}

static void __exit usb_ipc_exit(void)
{
	bvd_dbg("cleanup bvd_ipc");

	kfree(bvd_ipc->xmit.buf);
	kfree(bvd_ipc);
	usb_deregister(&usb_ipc_driver);

	printk("USB Host(Bulverde) IPC driver deregistered.\n");
}

module_init(usb_ipc_init);
module_exit(usb_ipc_exit);
EXPORT_SYMBOL(usb_send_readurb);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * linux/drivers/usb/gadget/isp1583_udc.c
 *
 * ISP1583 high speed USB device controllers
 *
 * Copyright (C) 2009 guiming zhuo <gmzhuo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>
#include <mach/dma.h>

#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <linux/memory.h>

#include <mach/pxa2xx-regs.h>
#include <linux/gpio.h>

#include "isp1583_udc.h"

#define DRIVER_DESC	"ISP1583 USB Device Controller Gadget"
#define DRIVER_VERSION	"1 Apr 2008"
#define DRIVER_AUTHOR	"guiming zhuo <gmzhuo@gmail.com>"

static const char gadget_name[] = "isp1583_udc";
static const char driver_desc[] = DRIVER_DESC;

static struct isp1583_udc *the_controller;

/*************************** DEBUG FUNCTION ***************************/
#define DEBUG_NORMAL	1
#define DEBUG_VERBOSE	2
#define DMA_ADDR_INVALID 0

#ifdef CONFIG_USB_ISP1583_DEBUG
#define USB_ISP1583_DEBUG_LEVEL 0

static int dprintk(int level, const char *fmt, ...)
{
	static char printk_buf[1024];
	static long prevticks;
	static int invocation;
	va_list args;
	int len;

	if (level > USB_ISP1583_DEBUG_LEVEL)
		return 0;

	len = scnprintf(printk_buf,
			sizeof(printk_buf), "%1lu.%02d USB: ",
			prevticks, invocation++);

	va_start(args, fmt);
	len = vscnprintf(printk_buf + len, sizeof(printk_buf) - len, fmt, args);
	va_end(args);

	return printk(KERN_DEBUG "%s", printk_buf);
}
#else
static int dprintk(int level, const char *fmt, ...)
{
	return 0;
}
#endif

void printk_int_enable(void)
{

	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	l.VALUE = the_controller->regs[ISP1583_INTEN_L_REG];
	m.VALUE = the_controller->regs[ISP1583_INTEN_M_REG];

	printk(KERN_DEBUG "ep0tx %d\n", l.BITS.IEP0TX);
	printk(KERN_DEBUG "ep1tx %d\n", l.BITS.IEP1TX);
	printk(KERN_DEBUG "ep2tx %d\n", l.BITS.IEP2TX);
	printk(KERN_DEBUG "ep3tx %d\n", m.BITS.IEP3TX);
	printk(KERN_DEBUG "ep4tx %d\n", m.BITS.IEP4TX);
	printk(KERN_DEBUG "ep5tx %d\n", m.BITS.IEP5TX);
	printk(KERN_DEBUG "ep6tx %d\n", m.BITS.IEP6TX);
	printk(KERN_DEBUG "ep7tx %d\n", m.BITS.IEP7TX);

	printk(KERN_DEBUG "ep0rx %d\n", l.BITS.IEP0RX);
	printk(KERN_DEBUG "ep1rx %d\n", l.BITS.IEP1RX);
	printk(KERN_DEBUG "ep2rx %d\n", l.BITS.IEP2RX);
	printk(KERN_DEBUG "ep3rx %d\n", m.BITS.IEP3RX);
	printk(KERN_DEBUG "ep4rx %d\n", m.BITS.IEP4RX);
	printk(KERN_DEBUG "ep5rx %d\n", m.BITS.IEP5RX);
	printk(KERN_DEBUG "ep6rx %d\n", m.BITS.IEP6RX);
	printk(KERN_DEBUG "ep7rx %d\n", m.BITS.IEP7RX);

}

/*
 * this function is used to enable/disable IN endpoint interrrupt
 */
static void isp_ep_in_int(int epn, int flag,
			  union INT_ENABLE_LSB *l, union INT_ENABLE_MSB *m)
{

	switch (epn) {
	case 0:
		l->BITS.IEP0TX = flag;
		break;
	case 1:
		l->BITS.IEP1TX = flag;
		break;
	case 2:
		l->BITS.IEP2TX = flag;
		break;
	case 3:
		m->BITS.IEP3TX = flag;
		break;
	case 4:
		m->BITS.IEP4TX = flag;
		break;
	case 5:
		m->BITS.IEP5TX = flag;
		break;
	case 6:
		m->BITS.IEP6TX = flag;
		break;
	case 7:
		m->BITS.IEP7TX = flag;
	default:
		break;
	}
}

/*
 * this function is used to enable/disable OUT endpoint interrrupt
 */
static void isp_ep_out_int(int epn, int flag,
			   union INT_ENABLE_LSB *l, union INT_ENABLE_MSB *m)
{

	switch (epn) {
	case 0:
		l->BITS.IEP0RX = flag;
		break;
	case 1:
		l->BITS.IEP1RX = flag;
		break;
	case 2:
		l->BITS.IEP2RX = flag;
		break;
	case 3:
		m->BITS.IEP3RX = flag;
		break;
	case 4:
		m->BITS.IEP4RX = flag;
		break;
	case 5:
		m->BITS.IEP5RX = flag;
		break;
	case 6:
		m->BITS.IEP6RX = flag;
		break;
	case 7:
		m->BITS.IEP7RX = flag;
	default:
		break;
	}
}

static inline void usb_select_endpoint(int idx)
{

	the_controller->regs[ISP1583_EPINDEX_REG] = idx;
	the_controller->regs[ISP1583_EPINDEX_REG] = idx;
}

static inline void usb_select_dma_endpoint(int idx)
{
	the_controller->regs[ISP1583_DMAEP_REG] = idx;
	the_controller->regs[ISP1583_DMAEP_REG] = idx;
}

static inline void usb_select_setup_endpoint(void)
{
	the_controller->regs[ISP1583_EPINDEX_REG] = DFLOW_EPINDEX_EP0SETUP;
	the_controller->regs[ISP1583_EPINDEX_REG] = DFLOW_EPINDEX_EP0SETUP;
}

static void usb_setup_endpoint(int idx, int max_pkt_size, int type)
{

	if (epidx_n(idx) != 0) {
		usb_select_endpoint(idx);
		the_controller->regs[ISP1583_MAXPKT_REG] = max_pkt_size & 0x7FF;
		the_controller->regs[ISP1583_EPTYPE_REG] =
		    (DFLOW_EPTYPE_DBLBUF | (type & 0x3));

		/* clear buffer ... */
		the_controller->regs[ISP1583_EPCFG_REG] |= DFLOW_CTRLFUN_CLBUF;
		/* ... twice because of double buffering */
		usb_select_endpoint(idx);
		the_controller->regs[ISP1583_EPCFG_REG] |= DFLOW_CTRLFUN_CLBUF;
	}
}

static void usb_enable_endpoint(int idx)
{
	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	if (epidx_n(idx) != 0) {
		usb_select_endpoint(idx);

		/* Enable interrupt */
		l.VALUE = the_controller->regs[ISP1583_INTEN_L_REG];
		m.VALUE = the_controller->regs[ISP1583_INTEN_M_REG];

		if (epidx_dir(idx) == DIR_RX)
			isp_ep_out_int(epidx_n(idx), 1, &l, &m);
		else
			isp_ep_in_int(epidx_n(idx), 1, &l, &m);

		the_controller->regs[ISP1583_INTEN_L_REG] = l.VALUE;
		the_controller->regs[ISP1583_INTEN_M_REG] = m.VALUE;

		/* Enable endpoint */
		the_controller->regs[ISP1583_EPTYPE_REG] |= DFLOW_EPTYPE_ENABLE;
	}
}

static void usb_disable_endpoint(int idx)
{
	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	if (epidx_n(idx) != 0) {
		usb_select_endpoint(idx);

		/* Disable interrupt */
		l.VALUE = the_controller->regs[ISP1583_INTEN_L_REG];
		m.VALUE = the_controller->regs[ISP1583_INTEN_M_REG];

		if (epidx_dir(idx) == DIR_RX)
			isp_ep_out_int(epidx_n(idx), 0, &l, &m);
		else
			isp_ep_in_int(epidx_n(idx), 0, &l, &m);

		the_controller->regs[ISP1583_INTEN_L_REG] = l.VALUE;
		the_controller->regs[ISP1583_INTEN_M_REG] = m.VALUE;

		/* disable endpoint */
		the_controller->regs[ISP1583_EPTYPE_REG] &=
		    ~DFLOW_EPTYPE_ENABLE;
	}
}

static void setup_endpoints(void)
{
	int i;

	usb_setup_endpoint(ep_index(0, DIR_RX), 64, 0);
	usb_setup_endpoint(ep_index(0, DIR_TX), 64, 0);

	usb_enable_endpoint(ep_index(0, DIR_RX));
	usb_enable_endpoint(ep_index(0, DIR_TX));

	for (i = 1; i < 8; i++) {
		usb_disable_endpoint(ep_index(i, DIR_RX));
		usb_disable_endpoint(ep_index(i, DIR_TX));
	}
}

/*------------------------- I/O ----------------------------------*/

/*
 *	isp1583_udc_done
 */
static void isp1583_udc_done(struct isp1583_ep *ep,
			     struct isp1583_request *req, int status)
{
	unsigned halted = ep->halted;

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (req->req.dma) {
		if (req->mapped) {
			dma_unmap_single(ep->gadget->dev.parent,
					 req->req.dma, req->req.length,
					 (ep->bEndpointAddress & USB_DIR_IN)
					 ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			req->req.dma = DMA_ADDR_INVALID;
			req->mapped = 0;
		} else
			dma_sync_single_for_cpu(ep->gadget->dev.parent,
						req->req.dma, req->req.length,
						(ep->
						 bEndpointAddress & USB_DIR_IN)
						? DMA_TO_DEVICE :
						DMA_FROM_DEVICE);

	}

	ep->halted = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->halted = halted;
}

static void isp1583_udc_nuke(struct isp1583_udc *udc,
			     struct isp1583_ep *ep, int status)
{
	/* Sanity check */
	if (&ep->queue == NULL)
		return;

	while (!list_empty(&ep->queue)) {
		struct isp1583_request *req;
		req = list_entry(ep->queue.next, struct isp1583_request, queue);
		isp1583_udc_done(ep, req, status);
	}
}

static inline void isp1583_udc_clear_ep_state(struct isp1583_udc *dev)
{
	unsigned i;

	for (i = 1; i < ISP1583_ENDPOINTS; i++)
		isp1583_udc_nuke(dev, &dev->ep[i], -ECONNABORTED);
}

static void isp1583_stall_endpoint(int idx, struct isp1583_ep *ep)
{
	usb_select_endpoint(idx);
	the_controller->regs[ISP1583_EPCFG_REG] |= DFLOW_CTRLFUN_STALL;
	ep->halted = 1;
}

static void isp1583_unstall_endpoint(int idx, struct isp1583_ep *ep)
{
	usb_select_endpoint(idx);
	the_controller->regs[ISP1583_EPCFG_REG] &= ~DFLOW_CTRLFUN_STALL;
	the_controller->regs[ISP1583_EPTYPE_REG] &= ~DFLOW_EPTYPE_ENABLE;
	the_controller->regs[ISP1583_EPTYPE_REG] |= DFLOW_EPTYPE_ENABLE;
	the_controller->regs[ISP1583_EPCFG_REG] |= DFLOW_CTRLFUN_CLBUF;
	ep->halted = 0;
}

void usb_drv_stall(int endpoint, bool stall, bool in, struct isp1583_ep *ep)
{
	if (stall)
		isp1583_stall_endpoint(ep_index(endpoint, (int)in), ep);
	else
		isp1583_unstall_endpoint(ep_index(endpoint, (int)in), ep);
}

static void isp1583_set_address(struct isp1583_udc *dev, unsigned char addr)
{
	the_controller->regs[ISP1583_ADDR_REG] = (0x80 | (addr & 0x7f));
	dev->ep0state = EP0_IN_STATUS_PHASE;

	usb_select_endpoint(USB_ENDPOINT_CONTROL_OUT);
	the_controller->regs[ISP1583_EPCFG_REG] = USB_EPCONTROL_STATUS_ACK;

}

/*
 * USBWriteEndpoint writes data to the specified endpoint.
 */
int isp1583_send_control(const unsigned char *pData, unsigned long len)
{
	unsigned long ulLength, ulData, tmp;
	int ulIdx = 0;
	struct usb_device_descriptor *pdesc;

	pdesc = (struct usb_device_descriptor *)pData;
	ulLength = (len > 64) ? 64 : len;

	the_controller->isp_dma_flag = 1;
	usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);

	if (ulLength != 64)
		the_controller->regs[ISP1583_BUFFLEN_REG] = ulLength;

	for (ulIdx = 0; ulIdx < ulLength;) {
		ulData = *pData++;
		ulIdx++;
		if (ulIdx < ulLength) {
			tmp = *pData++;
			tmp <<= 8;
			ulData |= tmp;
			ulIdx++;
		}

		the_controller->regs[ISP1583_EPDATA_REG] = ulData;
	}
	the_controller->isp_dma_flag = 0;
	return ulIdx;

}

/*
 * USBGetStatus implements the USB Get_Status device request.
 */
static void isp1583_get_status(struct isp1583_udc *dev,
			       struct usb_ctrlrequest *crq)
{
	unsigned char ucStatus[2];
	unsigned long ulEndpoint;

	u8 ep_num = crq->wIndex & 0x7F;
	u8 is_in = crq->wIndex & USB_DIR_IN;
	dev->ep0state = EP0_IN_DATA_PHASE;

	switch (crq->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		isp1583_send_control((const unsigned char *)&dev->devstatus, 2);
		break;

	case USB_RECIP_INTERFACE:
		ucStatus[0] = 0;
		ucStatus[1] = 0;

		isp1583_send_control(ucStatus, 2);
		break;
	case USB_RECIP_ENDPOINT:
		ulEndpoint = ep_num;
		ulEndpoint <<= 1;
		ulEndpoint |= is_in;

		usb_select_endpoint(ulEndpoint);
		ulEndpoint = the_controller->regs[ISP1583_EPCFG_REG];

		if (ulEndpoint & USB_EPCONTROL_STALL)
			ucStatus[0] = USB_ENDPOINT_STATUS_STALLED;
		else
			ucStatus[0] = 0;

		ucStatus[1] = 0;

		isp1583_send_control(ucStatus, 2);
		break;
	default:
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_OUT,
				       &the_controller->ep[0]);
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_IN,
				       &the_controller->ep[0]);

		break;
	}
}

/*
 * USBClearFeature implements the USB Clear_Feature device request.
 */
/*static void USBClearFeature(void)*/
static void isp1583_clear_feature(struct usb_ctrlrequest *crq,
				  struct isp1583_udc *dev)
{
	unsigned long ulEndpoint;
	u8 ep_num = crq->wIndex & 0x7F;
	u8 is_in = crq->wIndex & USB_DIR_IN;

	dev->ep0state = EP0_IN_STATUS_PHASE;

	/*
	 * The only feature we support is stall on an endpoint.
	 */
	if (((crq->bRequestType & USB_RECIP_MASK) ==
	     USB_RECIP_ENDPOINT) && (crq->wValue == USB_ENDPOINT_HALT)) {
		ulEndpoint = ep_num;
		ulEndpoint <<= 1;
		ulEndpoint |= is_in;
		/*
		   isp1583_unstall_endpoint(ulEndpoint, 0);
		 */
		the_controller->regs[ISP1583_EPCFG_REG] =
		    USB_EPCONTROL_STATUS_ACK;
		printk(KERN_DEBUG "clear feature\n");

	} else {
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_OUT,
				       &the_controller->ep[0]);
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_IN,
				       &the_controller->ep[0]);
	}

}

static void isp1583_set_feature(struct usb_ctrlrequest *crq,
				struct isp1583_udc *dev)
{
	unsigned long ulEndpoint;
	u8 ep_num = crq->wIndex & 0x7F;
	u8 is_in = crq->wIndex & USB_DIR_IN;

	if (((crq->bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) &&
	    (crq->wValue == USB_ENDPOINT_HALT)) {
		ulEndpoint = ep_num;
		ulEndpoint <<= 1;
		ulEndpoint |= is_in;
		the_controller->regs[ISP1583_EPCFG_REG] =
		    USB_EPCONTROL_STATUS_ACK;
		printk(KERN_DEBUG "set feature\n");
		/*
		   isp1583_stall_endpoint(ulEndpoint, 1);
		 */
	} else {
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_OUT,
				       &the_controller->ep[0]);
		isp1583_stall_endpoint(USB_ENDPOINT_CONTROL_IN,
				       &the_controller->ep[0]);
	}
}

static void isp1583_set_interface(struct usb_ctrlrequest *crq,
				  struct isp1583_udc *dev)
{
	dev->ep0state = EP0_IN_STATUS_PHASE;
	the_controller->regs[ISP1583_EPTYPE_REG] &= ~USB_EPTYPE_ENABLE;
	the_controller->regs[ISP1583_EPTYPE_REG] &= ~USB_EPTYPE_ENABLE;

	the_controller->regs[ISP1583_EPCFG_REG] |= USB_EPCONTROL_STATUS_ACK;

}

static int usb_get_packet(unsigned char *buf, int max_len, int *hasMore)
{
	int len, i;

	len = the_controller->regs[ISP1583_BUFFLEN_REG];
	if (len == 0)
		return 0;

	if (max_len < 0 || max_len > len)
		max_len = len;

	i = 0;
	while (i < len) {

		unsigned short d = the_controller->regs[ISP1583_EPDATA_REG];
		if (i < max_len)
			buf[i] = d & 0xff;
		i++;
		if (i < max_len)
			buf[i] = (d >> 8) & 0xff;
		i++;
	}

	return max_len;
}

int usb_write_packet(const unsigned char *pData, unsigned long len)
{
	unsigned long ulData, tmp;
	int ulIdx = 0;

	/*
	 * Write the data into the transmit buffer.
	 */
	for (ulIdx = 0; ulIdx < len;) {
		ulData = *pData++;
		ulIdx++;
		if (ulIdx < len) {
			tmp = *pData++;
			tmp <<= 8;
			ulData |= tmp;
			ulIdx++;
		}

		the_controller->regs[ISP1583_EPDATA_REG] = ulData;
	}

	return ulIdx;

}

/*
 * configure the DMA channel to receive data
 */
static void isp1583_epx_rx_set_dma(unsigned long phys_addr)
{
	unsigned int bEndpointAddress =
	    the_controller->dma_ep->bEndpointAddress;
	the_controller->isp_dma_flag = 1;
	/* configure PXA DMA registers */
	DALGN |= (1 << the_controller->dma_channel);
	DRCMR(0) = 0x80 | the_controller->dma_channel;
	DCSR(the_controller->dma_channel) = DCSR_NODESC;
	DSADR(the_controller->dma_channel) =
	    (the_controller->phy_base + ISP_DMA_DATA_PORT_OFFSET);
	DTADR(the_controller->dma_channel) = phys_addr;
	DCMD(the_controller->dma_channel) =
	    DCMD_INCTRGADDR | DCMD_FLOWSRC | DCMD_WIDTH2 | DCMD_BURST32 |
	    the_controller->isp_dma_len;

	usb_select_endpoint(ep_index(bEndpointAddress & 0x7f, 1) + 5);
	usb_select_dma_endpoint(ep_index(bEndpointAddress & 0x7f, 0));

	the_controller->regs[ISP1583_DMACNT_L_REG] =
	    (unsigned short)(the_controller->isp_dma_len & 0xFFFF);
	the_controller->regs[ISP1583_DMACNT_H_REG] =
	    (unsigned short)(the_controller->isp_dma_len >> 16 & 0xFFFF);

	the_controller->regs[ISP1583_DMACMD_REG] = ISP_GDMA_Write_Command;
	DCSR(the_controller->dma_channel) |= DCSR_RUN | DCSR_NODESC;
}

/*
 * configure the DMA transmit
 */
static void isp1583_epx_tx_set_dma(unsigned long phys_addr)
{
	unsigned int bEndpointAddress =
	    the_controller->dma_ep->bEndpointAddress;

	the_controller->isp_dma_flag = 1;

	/* configure PXA DMA registers */
	DALGN |= (1 << the_controller->dma_channel);
	DRCMR(0) = 0x80 | the_controller->dma_channel;
	DDADR(the_controller->dma_channel) = 0;
	DCSR(the_controller->dma_channel) = DCSR_NODESC;
	DSADR(the_controller->dma_channel) = phys_addr;
	DTADR(the_controller->dma_channel) =
	    (the_controller->phy_base + ISP_DMA_DATA_PORT_OFFSET);
	DCMD(the_controller->dma_channel) =
	    DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_WIDTH2 | DCMD_BURST32 |
	    the_controller->isp_dma_len;
	DCSR(the_controller->dma_channel) |= DCSR_RUN | DCSR_NODESC;

	usb_select_endpoint(ep_index((bEndpointAddress) & 0x7f, 0) + 5);
	usb_select_dma_endpoint(ep_index(bEndpointAddress & 0x7f, 1));

	the_controller->regs[ISP1583_DMACNT_L_REG] =
	    (unsigned short)(the_controller->isp_dma_len & 0xFFFF);
	the_controller->regs[ISP1583_DMACNT_H_REG] =
	    (unsigned short)(the_controller->isp_dma_len >> 16 & 0xFFFF);
	the_controller->regs[ISP1583_DMACMD_REG] = ISP_GDMA_Read_Command;
}

/*
 * ISP1583 DMA interrupt handler
 */
static void isp1583_dev_dma_irq(void)
{
	unsigned long flags;
	struct isp1583_ep *ep = the_controller->dma_ep;
	unsigned int bEndpointAddress = ep->bEndpointAddress;
	struct isp1583_request *req;
	unsigned long dma_addr;
	union CONTROL_REG ctrl;

	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	local_irq_save(flags);
	if (list_empty(&ep->queue)) {
		req = NULL;
		local_irq_restore(flags);
		return;
	}
	req = list_entry(ep->queue.next, struct isp1583_request, queue);

	the_controller->isp_dma_flag = 0;

	/* TX interrupt */
	if (USB_DIR_IN & bEndpointAddress) {
		if (the_controller->isp_dma_index_len >= req->req.length) {
			/* all data have been transmitted by DMA */
			if (req->req.length % ep->fifo_size) {
				usb_select_dma_endpoint(ep_index
							(bEndpointAddress &
							 0x7f, 0) + 5);
				usb_select_endpoint(ep_index
						    (bEndpointAddress & 0x7f,
						     1));

				ctrl.VALUE = 0;
				ctrl.BITS.VENDP = 1;
				the_controller->regs[ISP1583_EPCFG_REG] =
				    ctrl.VALUE;
			}
			req->req.actual = the_controller->isp_dma_index_len;

			isp1583_udc_done(ep, req, 0);
			if (!the_controller->isp_dma_flag) {
				/* set dma endpoint into a random value which is
				 *different with endpoint index
				 */
				usb_select_dma_endpoint(ep_index
							(bEndpointAddress &
							 0x7f, 1) + 5);

				l.VALUE =
				    the_controller->regs[ISP1583_INTEN_L_REG];
				m.VALUE =
				    the_controller->regs[ISP1583_INTEN_M_REG];

				isp_ep_in_int(bEndpointAddress & 0x7f,
					      1, &l, &m);

				the_controller->regs[ISP1583_INTEN_L_REG] =
				    l.VALUE;
				the_controller->regs[ISP1583_INTEN_M_REG] =
				    m.VALUE;
			}
			local_irq_restore(flags);
			return;
		}
		/* re-set the DMA to transfer the other data */
		the_controller->isp_dma_len = req->req.length -
		    the_controller->isp_dma_index_len;
		if (the_controller->isp_dma_len > MAX_DMA_SIZE)
			the_controller->isp_dma_len = MAX_DMA_SIZE;

		dma_addr = req->req.dma + the_controller->isp_dma_index_len;
		the_controller->isp_dma_index_len +=
		    the_controller->isp_dma_len;

		req->req.actual = the_controller->isp_dma_index_len;
		/* enable TX dma transfer */
		isp1583_epx_tx_set_dma(dma_addr);

	} else {		/* RX interrupt */
		/* if all wished data have been received by DMA */
		if (the_controller->isp_dma_index_len >= req->req.length) {
			unsigned long remain;
			req->req.actual = the_controller->isp_dma_index_len;
			usb_select_dma_endpoint(ep_index(bEndpointAddress &
							 0x7f, 0));
			remain = the_controller->regs[ISP1583_DMACNT_H_REG];
			remain <<= 16;
			remain |= the_controller->regs[ISP1583_DMACNT_L_REG];
			req->req.actual = req->req.length - remain;
			isp1583_udc_done(ep, req, 0);
			if (!the_controller->isp_dma_flag) {
				/* set dma endpoint into a random value which is
				 *different with endpoint index
				 */
				usb_select_dma_endpoint(ep_index
							(bEndpointAddress &
							 0x7f, 0) + 5);

				/* enable endpoint interrupt */
				l.VALUE =
				    the_controller->regs[ISP1583_INTEN_L_REG];
				m.VALUE =
				    the_controller->regs[ISP1583_INTEN_M_REG];

				isp_ep_out_int(bEndpointAddress & 0x7f,
					       1, &l, &m);

				the_controller->regs[ISP1583_INTEN_L_REG] =
				    l.VALUE;
				the_controller->regs[ISP1583_INTEN_M_REG] =
				    m.VALUE;
			}
			local_irq_restore(flags);
			return;
		}

		/* re-set the DMA channel to receive other data */
		the_controller->isp_dma_len = req->req.length -
		    the_controller->isp_dma_index_len;
		if (the_controller->isp_dma_len > MAX_DMA_SIZE)
			the_controller->isp_dma_len = MAX_DMA_SIZE;
		dma_addr = req->req.dma + the_controller->isp_dma_index_len;
		the_controller->isp_dma_index_len +=
		    the_controller->isp_dma_len;

		/* enable RX dma transfer */
		isp1583_epx_rx_set_dma(dma_addr);

	}
	local_irq_restore(flags);
}

static int isp1583_packet_xmit(struct isp1583_ep *ep)
{
	struct isp1583_request *req;
	unsigned long len;
	unsigned long ret;
	unsigned long flags;

	local_irq_save(flags);
	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct isp1583_request, queue);

	if (!req || req->req.length == 0) {
		local_irq_restore(flags);
		return 0;
	}

	len = req->req.length - req->req.actual;
	usb_select_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_TX));

	if (the_controller->regs[ISP1583_BUFFST_REG] == 3) {
		local_irq_restore(flags);
		return 0;
	}

	if (len < ep->fifo_size)
		the_controller->regs[ISP1583_BUFFLEN_REG] = len;
	else
		len = ep->fifo_size;

	ret = usb_write_packet(req->req.buf + req->req.actual, len);
	req->req.actual += ret;

	local_irq_restore(flags);
	return ret;
}

static int isp1583_tx_done(struct isp1583_ep *ep)
{
	struct isp1583_request *req;
	unsigned long flags;

	local_irq_save(flags);
	if (the_controller->isp_dma_flag || list_empty(&ep->queue)) {
		req = NULL;
		local_irq_restore(flags);
		return 0;
	}

	req = list_entry(ep->queue.next, struct isp1583_request, queue);

	if (req->req.length == req->req.actual)
		isp1583_udc_done(ep, req, 0);

	local_irq_restore(flags);
	return isp1583_packet_xmit(ep);
}

static int isp1583_packet_recv(struct isp1583_ep *ep)
{
	unsigned long flags;
	struct isp1583_request *req;
	int len;
	int ret;
	int hasMore = 0;

	local_irq_save(flags);
	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct isp1583_request, queue);

	if (!req) {
		printk(KERN_DEBUG "have packet but driver not want it %s\n",
		       ep->ep.name);
		local_irq_restore(flags);
		return 0;
	}

	len = req->req.length - req->req.actual;

	usb_select_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_RX));
	ret = usb_get_packet(req->req.buf + req->req.actual, len, &hasMore);
	req->req.actual += ret;

	if (ret < ep->fifo_size || req->req.length == req->req.actual)
		isp1583_udc_done(ep, req, 0);

	local_irq_restore(flags);

	return ret;
}

static void isp1583_udc_handle_ep0_setup(struct isp1583_udc *dev,
					 struct isp1583_ep *ep,
					 struct usb_ctrlrequest *crq)
{
	int len, ret = 0;
	int hasMore = 0;

	usb_select_setup_endpoint();
	len = usb_get_packet((unsigned char *)crq, sizeof(*crq), &hasMore);

	if (len != sizeof(*crq)) {

		usb_drv_stall(0, true, false, ep);
		usb_drv_stall(0, true, true, ep);
		return;
	}

	/* cope with automagic for some standard requests. */
	dev->req_std = (crq->bRequestType & USB_TYPE_MASK)
	    == USB_TYPE_STANDARD;
	dev->req_config = 0;
	dev->req_pending = 1;

	switch (crq->bRequest) {
	case USB_REQ_SET_ADDRESS:
		isp1583_set_address(dev, crq->wValue);
		return;
		break;
	case USB_REQ_GET_STATUS:

		isp1583_get_status(dev, crq);
		break;

	case USB_REQ_SET_INTERFACE:

		isp1583_set_interface(crq, dev);
		break;

	case USB_REQ_CLEAR_FEATURE:

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		isp1583_clear_feature(crq, dev);
		return;

	case USB_REQ_SET_FEATURE:

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		isp1583_set_feature(crq, dev);

		return;

	default:
		break;
	}

	if (crq->bRequestType & USB_DIR_IN) {
		dev->ep0state = EP0_IN_DATA_PHASE;
		usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);
		the_controller->regs[ISP1583_EPCFG_REG] = USB_EPCONTROL_DSEN;
	} else {
		dev->ep0state = EP0_OUT_DATA_PHASE;
		usb_select_endpoint(USB_ENDPOINT_CONTROL_OUT);
		the_controller->regs[ISP1583_EPCFG_REG] = USB_EPCONTROL_DSEN;
	}

	if (dev && dev->driver && dev->driver->setup)
		ret = dev->driver->setup(&dev->gadget, crq);

	if (ret < 0) {
		printk(KERN_DEBUG "setup fail %d\n", ret);
		printk(KERN_DEBUG "bRequestType %d bRequest %d wValue %d"
		       " bbLeng %d\n",
		       crq->bRequestType, crq->bRequest,
		       crq->wValue, crq->wLength);
		dev->ep0state = EP0_IDLE;
	} else if (dev->req_pending)
		dev->req_pending = 0;

	return;

}

/*
 * udc_enable_interrupts - enable interrupts
 */
void udc_all_interrupts(void)
{
	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	/* enable all default interrupt */
	l.VALUE = 0;
	m.VALUE = 0;
	l.BITS.IESUSP = 1;	/* suspend */
	l.BITS.IERESM = 1;	/* resume */
	l.BITS.IERST = 1;	/* reset */
	l.BITS.IEP0SETUP = 1;	/* setup */
	l.BITS.IEHS_STA = 1;	/* setup */
	l.BITS.IEP0RX = 1;	/* out endpoint 0 */
	l.BITS.IEP0TX = 1;	/* in endpoint 0 */

	l.BITS.IEDMA = 1;	/* DMA interrupt */

	the_controller->regs[0x16] = m.VALUE;
	the_controller->regs[0x14] = l.VALUE;

	/* enable ISP1583 */
	the_controller->regs[0x0] |= 0x80;

	setup_endpoints();
}

/*
 *	isp1583_udc_irq - interrupt handler
 */
static irqreturn_t isp1583_udc_irq(int dummy, void *_dev)
{
	union INTERRUPT_STATUS_LSB int_status_lsb;
	union INTERRUPT_STATUS_MSB int_status_msb;
	u16 mode;
	unsigned long flags;
	local_irq_save(flags);

	/* disable interrupt */
	mode = the_controller->regs[ISP1583_MODE_REG];
	mode &= ISP_MODE_MASK_GLINTENA;
	the_controller->regs[ISP1583_MODE_REG] = mode;

	int_status_lsb.VALUE = the_controller->regs[ISP1583_INTS_L_REG];
	int_status_msb.VALUE = the_controller->regs[ISP1583_INTS_M_REG];
	int_status_lsb.VALUE &= the_controller->regs[ISP1583_INTEN_L_REG];
	int_status_msb.VALUE &= the_controller->regs[ISP1583_INTEN_M_REG];

	/* clear interrupt status */
	the_controller->regs[ISP1583_INTS_L_REG] = int_status_lsb.VALUE;
	the_controller->regs[ISP1583_INTS_M_REG] = int_status_msb.VALUE;

	if (int_status_lsb.BITS.RESET) {

		the_controller->gadget.speed = USB_SPEED_UNKNOWN;
		isp_init();
		udc_all_interrupts();
		the_controller->address = 0;
		the_controller->ep0state = EP0_IDLE;
		the_controller->gadget.speed = USB_SPEED_FULL;

	}

	if (int_status_lsb.BITS.SOF)
		printk(KERN_DEBUG "SOF\n");

	if (int_status_lsb.BITS.PSOF)
		printk(KERN_DEBUG "PSOF\n");

	if (int_status_lsb.BITS.SOF)
		printk(KERN_DEBUG "SOF\n");

	if (int_status_lsb.BITS.HS_STAT) {

		the_controller->hiSpeed = 1;
		udc_all_interrupts();
		the_controller->gadget.speed = USB_SPEED_HIGH;

	}

	if (int_status_lsb.BITS.DMA) {	/* DMA interrupt */
		unsigned short dma_int;
		dma_int = the_controller->regs[ISP1583_DMAINTS_REG];
		the_controller->regs[ISP1583_DMAINTS_REG] = dma_int;

		if (dma_int)
			isp1583_dev_dma_irq();
	}

	if ((int_status_lsb.BITS.RESUME) && (!int_status_lsb.BITS.SUSP))

		/* resume interrupt */
		the_controller->regs[ISP1583_UNLOCK_REG] = ISP_UNLOCK_REGS;

	if (int_status_lsb.BITS.SUSP && (!int_status_lsb.BITS.RESUME))
		;

	if (int_status_lsb.BITS.EP0SETUP) {

		struct usb_ctrlrequest crq;
		isp1583_udc_handle_ep0_setup(the_controller,
					     &the_controller->ep[0], &crq);
	}

	if (int_status_lsb.BITS.EP0TX) {
		/*printk(KERN_DEBUG "EP0 TX DONE\n"); */
		if (the_controller->ep0state == EP0_IN_STATUS_PHASE) {
			the_controller->ep0state = EP0_IDLE;
			usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);
			the_controller->regs[ISP1583_EPCFG_REG] |=
			    USB_EPCONTROL_STATUS_ACK;
		} else if (the_controller->ep0state == EP0_IN_DATA_PHASE) {
			struct isp1583_ep *ep = &the_controller->ep[0];
			struct isp1583_request *req;
			unsigned int leng;

			if (list_empty(&ep->queue))
				req = NULL;
			else
				req = list_entry(ep->queue.next,
						 struct isp1583_request, queue);

			if (req && req->isIn == 1) {
				leng =
				    isp1583_send_control(req->req.buf +
							 req->req.actual,
							 req->req.length -
							 req->req.actual);
				req->req.actual += leng;
				if (req->req.actual == req->req.length)
					isp1583_udc_done(ep, req, 0);
			} else {
				the_controller->ep0state = EP0_IDLE;
				usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);
				the_controller->regs[ISP1583_EPCFG_REG] |=
				    USB_EPCONTROL_STATUS_ACK;
			}
		} else
			the_controller->ep0state = EP0_IDLE;
	}

	if (int_status_lsb.BITS.EP0RX) {
		struct isp1583_ep *ep = &the_controller->ep[0];
		struct isp1583_request *req;
		int len;
		int ret;
		int hasMore = 0;

		if (list_empty(&ep->queue))
			req = NULL;
		else
			req =
			    list_entry(ep->queue.next, struct isp1583_request,
				       queue);

		if (req && req->isIn == 0) {

			len = req->req.length - req->req.actual;

			usb_select_endpoint(USB_ENDPOINT_CONTROL_OUT);
			ret =
			    usb_get_packet(req->req.buf + req->req.actual, len,
					   &hasMore);
			req->req.actual += ret;

			if (ret < ep->fifo_size
			    || req->req.length == req->req.actual) {
				isp1583_udc_done(ep, req, 0);
				usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);
				the_controller->regs[ISP1583_EPCFG_REG] |=
				    USB_EPCONTROL_STATUS_ACK;
			}
		}
	}

	if (int_status_lsb.BITS.EP1RX)
		isp1583_packet_recv(&the_controller->ep[1]);

	if (int_status_lsb.BITS.EP1TX)
		isp1583_tx_done(&the_controller->ep[2]);

	if (int_status_lsb.BITS.EP2RX)
		isp1583_packet_recv(&the_controller->ep[3]);

	if (int_status_lsb.BITS.EP2TX)
		isp1583_tx_done(&the_controller->ep[4]);

	if (int_status_msb.BITS.EP3RX)
		isp1583_packet_recv(&the_controller->ep[5]);

	if (int_status_msb.BITS.EP3TX)
		isp1583_tx_done(&the_controller->ep[6]);

	if (int_status_msb.BITS.EP4RX)
		isp1583_packet_recv(&the_controller->ep[7]);

	if (int_status_msb.BITS.EP4TX)
		isp1583_tx_done(&the_controller->ep[8]);

	if (int_status_msb.BITS.EP5RX)
		isp1583_packet_recv(&the_controller->ep[9]);

	if (int_status_msb.BITS.EP5TX)
		isp1583_tx_done(&the_controller->ep[10]);

	if (int_status_msb.BITS.EP6RX)
		isp1583_packet_recv(&the_controller->ep[11]);

	if (int_status_msb.BITS.EP6TX)
		isp1583_tx_done(&the_controller->ep[12]);

	if (int_status_msb.BITS.EP7RX)
		isp1583_packet_recv(&the_controller->ep[13]);

	if (int_status_msb.BITS.EP7TX)
		isp1583_tx_done(&the_controller->ep[14]);

	/* enable interrupt */
	mode = the_controller->regs[ISP1583_MODE_REG];
	mode |= ISP_MODE_GLINTENA;
	the_controller->regs[ISP1583_MODE_REG] = mode;

	local_irq_restore(flags);

	return IRQ_HANDLED;
}

/*------------------------- isp1583_ep_ops ----------------------------------*/

static inline struct isp1583_ep *to_isp1583_ep(struct usb_ep *ep)
{
	return container_of(ep, struct isp1583_ep, ep);
}

static inline struct isp1583_udc *to_isp1583_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct isp1583_udc, gadget);
}

static inline struct isp1583_request *to_isp1583_req(struct usb_request *req)
{
	return container_of(req, struct isp1583_request, req);
}

/*
 *	isp1583_udc_ep_enable
 */
static int isp1583_udc_ep_enable(struct usb_ep *_ep,
				 const struct usb_endpoint_descriptor *desc)
{
	struct isp1583_udc *dev;
	struct isp1583_ep *ep;
	u32 max;
	unsigned long flags;

	ep = to_isp1583_ep(_ep);

	if (!_ep || !desc || ep->desc
	    || _ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize) & 0x1fff;

	local_irq_save(flags);
	_ep->maxpacket = max & 0x7ff;
	ep->desc = desc;
	ep->halted = 0;
	ep->bEndpointAddress = desc->bEndpointAddress;
	ep->bmAttributes = desc->bmAttributes;
	ep->fifo_size = max;

	usb_setup_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_RX),
			   ep->fifo_size, desc->bmAttributes & 0x3);
	usb_setup_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_TX),
			   ep->fifo_size, desc->bmAttributes & 0x3);

	printk(KERN_DEBUG "eanbel %s addr %d size %d\n", ep->ep.name,
	       ep->bEndpointAddress, ep->fifo_size);
	usb_enable_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_RX));
	usb_enable_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_TX));

	local_irq_restore(flags);
	return 0;
}

/*
 * isp1583_udc_ep_disable
 */
static int isp1583_udc_ep_disable(struct usb_ep *_ep)
{
	struct isp1583_ep *ep = to_isp1583_ep(_ep);
	unsigned long flags;

	if (!_ep || !ep->desc)
		return -EINVAL;

	local_irq_save(flags);

	ep->desc = NULL;
	ep->halted = 1;
	usb_disable_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_RX));
	usb_disable_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_TX));

	local_irq_restore(flags);
	return 0;
}

/*
 * isp1583_udc_alloc_request
 */
static struct usb_request *isp1583_udc_alloc_request(struct usb_ep *_ep,
						     gfp_t mem_flags)
{
	struct isp1583_request *req;

	if (!_ep)
		return NULL;

	req = kzalloc(sizeof(struct isp1583_request), mem_flags);
	if (!req)
		return NULL;

	req->req.dma = 0;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/*
 * isp1583_udc_free_request
 */
static void
isp1583_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct isp1583_ep *ep = to_isp1583_ep(_ep);
	struct isp1583_request *req = to_isp1583_req(_req);

	if (!ep || !_req || (!ep->desc && _ep->name != ep0name))
		return;

	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static int isp1583_dma_packet_xmit(struct isp1583_ep *ep)
{
	struct isp1583_request *req;
	unsigned long flags;
	union INT_ENABLE_LSB l;
	union INT_ENABLE_MSB m;

	local_irq_save(flags);
	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct isp1583_request, queue);

	if (!req || req->req.length == 0) {
		local_irq_restore(flags);
		return 0;
	}
	if (req->req.dma == 0) {
		req->req.dma = dma_map_single(ep->gadget->dev.parent,
					      req->req.buf,
					      req->req.length, DMA_TO_DEVICE);
		req->mapped = 1;
	} else {
		dma_sync_single_for_device(ep->gadget->dev.parent,
					   req->req.dma, req->req.length,
					   DMA_TO_DEVICE);
		req->mapped = 0;
	}

	usb_select_endpoint(ep_index(ep->bEndpointAddress & 0x7f, DIR_RX) + 5);
	if (the_controller->isp_dma_flag == 0) {
		/* disable endpoint interrupt */
		l.VALUE = the_controller->regs[ISP1583_INTEN_L_REG];
		m.VALUE = the_controller->regs[ISP1583_INTEN_M_REG];

		isp_ep_in_int(ep->bEndpointAddress & 0x7f, 0, &l, &m);

		the_controller->regs[ISP1583_INTEN_L_REG] = l.VALUE;
		the_controller->regs[ISP1583_INTEN_M_REG] = m.VALUE;

		the_controller->dma_ep = ep;
		if (req->req.length > MAX_DMA_SIZE)
			the_controller->isp_dma_len = MAX_DMA_SIZE;
		else
			the_controller->isp_dma_len = req->req.length;
		the_controller->isp_dma_index_len = the_controller->isp_dma_len;
		req->req.actual = the_controller->isp_dma_index_len;
		isp1583_epx_tx_set_dma(req->req.dma);
	}
	local_irq_restore(flags);
	return 0;
}

/*
 *	isp1583_udc_queue
 */
static int isp1583_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
			     gfp_t gfp_flags)
{
	struct isp1583_request *req = to_isp1583_req(_req);
	struct isp1583_ep *ep = to_isp1583_ep(_ep);
	struct isp1583_udc *dev;
	unsigned long flags;

	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name)))
		return -EINVAL;

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	local_irq_save(flags);

	if (unlikely(!_req || !_req->complete || !_req->buf)) {
		local_irq_restore(flags);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	if ((ep->bEndpointAddress & 0x7f) &&
	    ((ep->bEndpointAddress & USB_DIR_IN) != 0)) {
		list_add_tail(&req->queue, &ep->queue);
		if (isp1583_dma_packet_xmit(ep) != 0)
			isp1583_packet_xmit(ep);
	} else if ((ep->bEndpointAddress & 0x7f) &&
		   ((ep->bEndpointAddress & USB_DIR_IN) == 0))
		list_add_tail(&req->queue, &ep->queue);
	else {
		int len;
		if ((_req == NULL) || (_req->length == 0)
		    || (_req->buf == NULL)) {
			usb_select_endpoint(USB_ENDPOINT_CONTROL_IN);
			the_controller->regs[ISP1583_EPCFG_REG] =
			    USB_EPCONTROL_STATUS_ACK;
			local_irq_restore(flags);
			return 0;
		}

		if (list_empty(&ep->queue)) {
			if (dev->ep0state == EP0_OUT_DATA_PHASE) {
				req->isIn = 0;
				list_add_tail(&req->queue, &dev->ep[0].queue);
			} else {
				req->isIn = 1;
				len = isp1583_send_control(_req->buf,
							   _req->length);
				_req->actual = len;
				if (len < _req->length)
					list_add_tail(&req->queue, &ep->queue);
				else
					isp1583_udc_done(ep, req, 0);
			}
		}

	}

	local_irq_restore(flags);
	return 0;
}

/*
 *	isp1583_udc_dequeue
 */
static int isp1583_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct isp1583_ep *ep = to_isp1583_ep(_ep);
	int retval = -EINVAL;
	unsigned long flags;
	struct isp1583_request *req = NULL;

	if (!the_controller->driver)
		return -ESHUTDOWN;

	if (!_ep || !_req)
		return retval;

	the_controller = to_isp1583_udc(ep->gadget);

	local_irq_save(flags);

	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init(&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}

	if (retval == 0)
		isp1583_udc_done(ep, req, -ECONNRESET);

	local_irq_restore(flags);
	return retval;
}

static const struct usb_ep_ops isp1583_ep_ops = {
	.enable = isp1583_udc_ep_enable,
	.disable = isp1583_udc_ep_disable,

	.alloc_request = isp1583_udc_alloc_request,
	.free_request = isp1583_udc_free_request,

	.queue = isp1583_udc_queue,
	.dequeue = isp1583_udc_dequeue,

};

/*------------------------- usb_gadget_ops ----------------------------------*/

static int isp1583_udc_get_frame(struct usb_gadget *_gadget)
{
	int d = ((the_controller->regs[ISP1583_FRAMENUM_REG]) & ISP_FNO_MASK);
	return d;

}

static int isp1583_udc_wakeup(struct usb_gadget *_gadget)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	return 0;
}

static void isp1583_udc_disable(struct isp1583_udc *dev);
static void isp1583_udc_enable(struct isp1583_udc *dev);

static int isp1583_udc_set_pullup(struct isp1583_udc *dev, int is_on)
{
	if (is_on)
		isp1583_udc_enable(dev);
	else
		isp1583_udc_disable(dev);

	return 0;
}

static int isp1583_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct isp1583_udc *udc = to_isp1583_udc(gadget);
	if (udc->vbus == (is_active != 0))
		return 0;
	udc->vbus = (is_active != 0);
	isp1583_udc_set_pullup(udc, is_active);
	return 0;
}

static int isp1583_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct isp1583_udc *udc = to_isp1583_udc(gadget);

	isp1583_udc_set_pullup(udc, is_on ? 0 : 1);
	return 0;
}

static int isp1583_vbus_draw(struct usb_gadget *_gadget, unsigned ma)
{
	return 0;
}

static const struct usb_gadget_ops isp1583_ops = {
	.get_frame = isp1583_udc_get_frame,
	.wakeup = isp1583_udc_wakeup,
	.pullup = isp1583_udc_pullup,
	.vbus_session = isp1583_udc_vbus_session,
	.vbus_draw = isp1583_vbus_draw,
};

/*------------------------- gadget driver handling---------------------------*/
/*
 * isp1583_udc_disable
 */
static void isp1583_udc_disable(struct isp1583_udc *dev)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	if (dev->mach && dev->mach->udc_command)
		dev->mach->udc_command(ISP158X_UDC_CMD_DISCONNECT);

	/*
	 * Disable the interrupts for the bulk endpoints.
	 */
	the_controller->regs[ISP1583_INTEN_L_REG] = 0;
	the_controller->regs[ISP1583_INTEN_M_REG] = 0;

	/*
	 * Disable the SoftConnect pull-up.
	 */
	the_controller->regs[ISP1583_INTEN_L_REG] = 0;
	the_controller->regs[ISP1583_MODE_REG] = 0;

	dev->devstatus &= ~2;

	/* Set speed to unknown */
	dev->ep0state = EP0_IDLE;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->disabled = 1;
}

/*
 * isp1583_udc_reinit
 */
static void isp1583_udc_reinit(struct isp1583_udc *dev)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	for (i = 0; i < ISP1583_ENDPOINTS; i++) {
		struct isp1583_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->dev = dev;
		ep->desc = NULL;
		ep->halted = 0;
		INIT_LIST_HEAD(&ep->queue);
	}
}

/*
 * isp1583_udc_enable
 */
static void isp1583_udc_enable(struct isp1583_udc *dev)
{

	dprintk(DEBUG_NORMAL, "isp1583_udc_enable called\n");
	/* disconnect ISP soft connect before switch */
	union USB_MODE mod;

	if (dev->mach && dev->mach->udc_command)
		dev->mach->udc_command(ISP158X_UDC_CMD_CONNECT);
	mod.VALUE = the_controller->regs[ISP1583_MODE_REG];
	mod.BITS.SOFTCT = 0;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;

	/* delay 30ms to make sure the signal is stability */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(3 * HZ / 100);

	/* reset the ISP1583 */
	mod.VALUE = the_controller->regs[ISP1583_MODE_REG];
	mod.BITS.SFRESET = 1;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;
	udelay(500);
	mod.BITS.SFRESET = 0;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;

	/* initialize the ISP1583 */
	isp_init();

	the_controller->disabled = 0;
}

/*
 *	usb_gadget_register_driver
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval;

	dprintk(DEBUG_NORMAL, "usb_gadget_register_driver() '%s'\n",
		driver->driver.name);


	/* Sanity checks */
	if (!the_controller)
		return -ENODEV;

	if (the_controller->driver)
		return -EBUSY;

	if (!driver->bind || !driver->setup || driver->speed < USB_SPEED_FULL) {
		printk(KERN_ERR "Invalid driver: bind %p setup %p speed %d\n",
		       driver->bind, driver->setup, driver->speed);
		return -EINVAL;
	}
#if defined(MODULE)
	if (!driver->unbind) {
		printk(KERN_ERR "Invalid driver: no unbind method\n");
		return -EINVAL;
	}
#endif

	/* Hook the driver */
	the_controller->driver = driver;
	the_controller->gadget.dev.driver = &driver->driver;

	/* Bind the driver */
	retval = device_add(&the_controller->gadget.dev);
	if (retval != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n", retval);
		goto register_error;
	}

	dprintk(DEBUG_NORMAL, "binding gadget driver '%s'\n",
		driver->driver.name);
	retval = driver->bind(&the_controller->gadget);
	if (retval != 0) {
		device_del(&the_controller->gadget.dev);
		goto register_error;
	}

	if (the_controller->transceiver)
		return otg_set_peripheral(the_controller->transceiver,
			&the_controller->gadget);
	return 0;

register_error:
	the_controller->driver = NULL;
	the_controller->gadget.dev.driver = NULL;
	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void stop_activity(struct isp1583_udc *dev,
			  struct usb_gadget_driver *driver)
{

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	isp1583_udc_clear_ep_state(dev);

	/* report disconnect; the driver is already quiesced */
	if (driver)
		driver->disconnect(&dev->gadget);

	/* re-init driver-visible data structures */
	isp1583_udc_reinit(dev);
}

/*
 *	usb_gadget_unregister_driver
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{

	if (!the_controller)
		return -ENODEV;

	if (!driver || driver != the_controller->driver || !driver->unbind)
		return -EINVAL;

	dprintk(DEBUG_NORMAL, "usb_gadget_register_driver() '%s'\n",
		driver->driver.name);

	stop_activity(the_controller, driver);
	/* Disable the_controller */
	isp1583_udc_disable(the_controller);

	driver->unbind(&the_controller->gadget);
	the_controller->driver = NULL;

	device_del(&the_controller->gadget.dev);
	the_controller->driver = NULL;
	if (the_controller->transceiver)
		return otg_set_peripheral(the_controller->transceiver, NULL);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*---------------------------------------------------------------------------*/
static struct isp1583_udc memory = {
	.gadget = {
		   .ops = &isp1583_ops,
		   .ep0 = &memory.ep[0].ep,
		   .name = gadget_name,
		   .is_dualspeed = 1,
		   .dev = {
			   .init_name = "gadget",
			   },
		   },

	/* control endpoint */
	.ep[0] = {
		  .num = 0,
		  .ep = {
			 .name = ep0name,
			 .ops = &isp1583_ep_ops,
			 .maxpacket = EP0_FIFO_SIZE,
			 },
		  .dev = &memory,
		  },

	.ep[1] = {
		  .num = 1,
		  .ep = {
			 .name = "ep1out",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 64,
			 },
		  .bEndpointAddress = 0x01,
		  .dev = &memory,
		  .halted = 1,
		  },
	.ep[2] = {
		  .num = 2,
		  .ep = {
			 .name = "ep1in",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 64,
			 },
		  .bEndpointAddress = 0x81,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[3] = {
		  .num = 3,
		  .ep = {
			 .name = "ep2out",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 128,
			 },
		  .bEndpointAddress = 0x02,
		  .dev = &memory,
		  .halted = 1,
		  },
	.ep[4] = {
		  .num = 4,
		  .ep = {
			 .name = "ep2in",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 128,
			 },
		  .bEndpointAddress = 0x82,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[5] = {
		  .num = 5,
		  .ep = {
			 .name = "ep3out",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 256,
			 },
		  .bEndpointAddress = 0x03,
		  .dev = &memory,
		  .halted = 1,
		  },
	.ep[6] = {
		  .num = 6,
		  .ep = {
			 .name = "ep3in",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 256,
			 },
		  .bEndpointAddress = 0x83,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[7] = {
		  .num = 7,
		  .ep = {
			 .name = "ep4out",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 512,
			 },
		  .bEndpointAddress = 0x04,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[8] = {
		  .num = 8,
		  .ep = {
			 .name = "ep4in",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 512,
			 },
		  .bEndpointAddress = 0x84,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[9] = {
		  .num = 9,
		  .ep = {
			 .name = "ep5out",
			 .ops = &isp1583_ep_ops,
			 .maxpacket = 512,
			 },
		  .bEndpointAddress = 0x05,
		  .dev = &memory,
		  .halted = 1,
		  },

	.ep[10] = {
		   .num = 10,
		   .ep = {
			  .name = "ep5in",
			  .ops = &isp1583_ep_ops,
			  .maxpacket = 512,
			  },
		   .bEndpointAddress = 0x85,
		   .dev = &memory,
		   .halted = 1,
		   },

	.ep[11] = {
		   .num = 11,
		   .ep = {
			  .name = "ep6out",
			  .ops = &isp1583_ep_ops,
			  .maxpacket = 1024,
			  },
		   .bEndpointAddress = 0x06,
		   .dev = &memory,
		   .halted = 1,
		   },

	.ep[12] = {
		   .num = 12,
		   .ep = {
			  .name = "ep6in",
			  .ops = &isp1583_ep_ops,
			  .maxpacket = 1024,
			  },
		   .bEndpointAddress = 0x86,
		   .dev = &memory,
		   .halted = 1,
		   },

	.ep[13] = {
		   .num = 7,
		   .ep = {
			  .name = "ep7out",
			  .ops = &isp1583_ep_ops,
			  .maxpacket = 1024,
			  },
		   .bEndpointAddress = 0x07,
		   .dev = &memory,
		   .halted = 1,
		   },

	.ep[14] = {
		   .num = 8,
		   .ep = {
			  .name = "ep7in",
			  .ops = &isp1583_ep_ops,
			  .maxpacket = 1024,
			  },
		   .bEndpointAddress = 0x87,
		   .dev = &memory,
		   .halted = 1,
		   },

	.disabled = 1,
	.hiSpeed = 0,
	.dma_channel = -1,

};

/*
 * ISP1583 Initialization
 */
void isp_init()
{

	union USB_MODE m;
	union INT_CONFIG i;

	union DMA_CONFIG cfg;
	union DMA_HARDWARE dma_hw;
	union DMA_INT_ENABLE dma_int;

	the_controller->regs[ISP1583_UNLOCK_REG] = ISP_UNLOCK_REGS;

	/*D14_MODE */
	m.VALUE = 0;
	m.BITS.DMACLKON = 1;
	m.BITS.CLKAON = 1;
	m.BITS.GLINTENA = 1;
	m.BITS.SOFTCT = 1;
	the_controller->regs[ISP1583_MODE_REG] = m.VALUE;

	/*D14_INTCONF */
	i.VALUE = 0;
	i.BITS.DDBGMODOUT = 1;
	i.BITS.DDBGMODIN = 1;
	i.BITS.CDBGMOD = 1;
	the_controller->regs[ISP1583_INTCONF_REG] = i.VALUE;

	/*OTG=0 */
	the_controller->regs[ISP1583_OTG_REG] = 0x0000;

	/*set invalid DMA endpoint */
	the_controller->regs[ISP1583_DMAEP_REG] = 0x00e0;

	/*reset dma */
	the_controller->regs[ISP1583_DMACMD_REG] = 0x0011;

	cfg.VALUE = the_controller->regs[ISP1583_DMACFG_REG];
	cfg.BITS.MODE = 0;
	cfg.BITS.DIS_XFER_CNT = 0;
	cfg.BITS.WIDTH = 1;
	cfg.BITS.IGNORE_IORDY = 0;
	cfg.BITS.PIO_MODE = 0;
	cfg.BITS.DMA_MODE = 0;
	the_controller->regs[ISP1583_DMACFG_REG] = cfg.VALUE;

	dma_hw.VALUE = the_controller->regs[ISP1583_DMAHW_REG];
	dma_hw.BITS.ENDIAN = 0;
	dma_hw.BITS.EOT_POL = 0;
	dma_hw.BITS.ACK_POL = 0;
	dma_hw.BITS.DREQ_POL = 1;
	dma_hw.BITS.WRITE_POL = 0;
	dma_hw.BITS.READ_POL = 0;
	the_controller->regs[ISP1583_DMAHW_REG] = dma_hw.VALUE;

	/* enable DMA interrupt */
	dma_int.VALUE = 0;
	dma_int.BITS.IE_DMA_XFER_OK = 1;
	the_controller->regs[ISP1583_DMAINTEN_REG] = dma_int.VALUE;

	/* set burst counter */
	the_controller->regs[ISP1583_DMABSTCNT_REG] = ISP_DMA_BURST_LEN;

}

/*
 * udc_init - initialize the isp the_controller
 */
int udc_init(void)
{

	/* disconnect ISP soft connect before switch */
	union USB_MODE mod;

	mod.VALUE = the_controller->regs[ISP1583_MODE_REG];
	mod.BITS.SOFTCT = 0;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;

	gpio_set_value(94, 1);
	/* delay 30ms to make sure the signal is stability */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(3 * HZ / 100);

	/* reset the ISP1583 */
	mod.VALUE = the_controller->regs[ISP1583_MODE_REG];
	mod.BITS.SFRESET = 1;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;
	udelay(500);
	mod.BITS.SFRESET = 0;
	the_controller->regs[ISP1583_MODE_REG] = mod.VALUE;

	/* initialize the ISP1583 */
	isp_init();

	return 0;
}

void cpu_dma_handler(int chanel, void *data)
{

}

/*
 *	probe - binds to the platform device
 */
static int isp1583_udc_probe(struct platform_device *pdev)
{
	struct resource *mregs;
	struct device *dev = &pdev->dev;
	int retval;

	the_controller = &memory;

	dev_dbg(dev, "%s()\n", __func__);

	mregs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mregs)
		return -ENXIO;

	the_controller->irq = platform_get_irq(pdev, 0);
	if (the_controller->irq < 0)
		return the_controller->irq;
	the_controller->regs =
	    (unsigned short *)ioremap_nocache((unsigned long)mregs->start,
					      0x400);
	if (!the_controller->regs) {
		dev_err(&pdev->dev,
			"Unable to map the_controller I/O memory\n");
		return -ENOMEM;
	}
	the_controller->regs = (unsigned short *)the_controller->regs;

	the_controller->phy_base = mregs->start;

	spin_lock_init(&the_controller->lock);
	the_controller->mach = pdev->dev.platform_data;
	the_controller->transceiver = otg_get_transceiver();


	device_initialize(&the_controller->gadget.dev);
	the_controller->gadget.dev.parent = &pdev->dev;
	the_controller->gadget.dev.dma_mask = pdev->dev.dma_mask;

	the_controller = the_controller;
	platform_set_drvdata(pdev, the_controller);

	isp1583_udc_disable(the_controller);
	isp1583_udc_reinit(the_controller);

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(the_controller->irq, isp1583_udc_irq,
			     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
			     IRQF_SHARED, gadget_name, the_controller);

	if (retval != 0) {
		dev_err(dev, "cannot get irq %i, err %d\n", the_controller->irq,
			retval);
		retval = -EBUSY;
		goto err_map;
	}

	the_controller->vbus = 0;

	the_controller->dma_channel = pxa_request_dma("isp1583",
						      DMA_PRIO_LOW,
						      cpu_dma_handler,
						      the_controller);

	dev_dbg(dev, "probe ok\n");

	return 0;

err_map:
	free_irq(the_controller->irq, the_controller);
	return retval;
}

/*
 *	isp1583_udc_remove
 */
static int isp1583_udc_remove(struct platform_device *pdev)
{
	struct isp1583_udc *udc = platform_get_drvdata(pdev);
	free_irq(platform_get_irq(pdev, 1), udc);
	dev_dbg(&pdev->dev, "%s()\n", __func__);
	if (udc->driver)
		return -EBUSY;

	debugfs_remove(udc->regs_info);
	free_irq(udc->irq, udc);

	iounmap(udc->regs);

	platform_set_drvdata(pdev, NULL);
	pxa_free_dma(udc->dma_channel);

	dev_dbg(&pdev->dev, "%s: remove ok\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int isp1583_udc_suspend(struct platform_device *pdev,
			       pm_message_t message)
{

	return 0;
}

static int isp1583_udc_resume(struct platform_device *pdev)
{

	return 0;
}
#else
#define isp1583_udc_suspend	NULL
#define isp1583_udc_resume	NULL
#endif

static struct platform_driver udc_driver_isp1583 = {
	.driver = {
		   .name = "isp1583-udc",
		   .owner = THIS_MODULE,
		   },
	.probe = isp1583_udc_probe,
	.remove = isp1583_udc_remove,
	.suspend = isp1583_udc_suspend,
	.resume = isp1583_udc_resume,
};

static int __init isp1583_udc_init(void)
{
	int retval;
	retval = platform_driver_register(&udc_driver_isp1583);
	return retval;
}

static void __exit isp1583_udc_exit(void)
{
	platform_driver_unregister(&udc_driver_isp1583);
}

module_init(isp1583_udc_init);
module_exit(isp1583_udc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:isp1583-usbgadget");

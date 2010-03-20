/*
 * linux/drivers/usb/ipcusb.h
 *
 * Implementation of a ipc driver based Intel's Bulverde USB Host
 * Controller.
 *
 * Copyright (C) 2003-2005 Motorola
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
 *  2003-Nov-18 - (Motorola) created
 *
 */
extern struct tty_driver *usb_for_mux_driver;
extern struct tty_struct *usb_for_mux_tty;
extern void (*usb_mux_dispatcher)(struct tty_struct *tty);
extern void (*usb_mux_sender)(void);

struct usb_ipc_tty {
    struct tty_struct   *tty;       /* pointer to the tty for this device */
    int         open_count; /* number of times this port has been opened */
    struct timer_list   *timer;

};

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


#define IS_EP_BULK(ep)  ((ep).bmAttributes == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && ((ep).bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)


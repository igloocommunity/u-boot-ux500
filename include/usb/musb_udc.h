/*
 * Copyright (c) 2009 Wind River Systems, Inc.
 * Tom Rix <Tom.Rix@windriver.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __MUSB_UDC_H__
#define __MUSB_UDC_H__

#include <usbdevice.h>
/* USB Function Module Registers */

/*
 * UDC_BASE is defined for the specific silicon under soc
 * specific cpu.h or related header.
 */

#define UDC_OFFSET(offset)	(UDC_BASE+(offset))

#define UDC_INTSRCR		UDC_OFFSET(0x0A) /* USB Interrupt src reg */
#define UDC_INTCLRR		UDC_OFFSET(0x0A) /* USB Interrupt src clr reg */

#define UDC_FADDR		UDC_OFFSET(0x00)
#define UDC_POWER		UDC_OFFSET(0x01)
#define UDC_INTRTX		UDC_OFFSET(0x02)
#define UDC_INTRRX		UDC_OFFSET(0x04)
#define UDC_INTRTXE		UDC_OFFSET(0x06) /* Enable reg for INTRTX */
#define UDC_INTRRXE		UDC_OFFSET(0x08) /* Enable reg for INTRRX */
#define UDC_INTRUSB		UDC_OFFSET(0x0A)
#define UDC_INTRUSBE		UDC_OFFSET(0x0B)
#define UDC_INDEX		UDC_OFFSET(0x0E)
#define UDC_TESTMODE		UDC_OFFSET(0x0F)
#define UDC_TXMAXP		UDC_OFFSET(0x10)
#define UDC_CSR0		UDC_OFFSET(0x12)
#define UDC_TXCSR		UDC_OFFSET(0x12)
#define UDC_RXMAXP		UDC_OFFSET(0x14)
#define UDC_RXCSR		UDC_OFFSET(0x16)
#define UDC_COUNT0		UDC_OFFSET(0x18)
#define UDC_RXCOUNT		UDC_OFFSET(0x18)
#define UDC_FIFO0		UDC_OFFSET(0x20)
#define UDC_FIFO1		UDC_OFFSET(0x24)
#define UDC_FIFO2		UDC_OFFSET(0x28)
#define UDC_FIFO3		UDC_OFFSET(0x2C)
#define UDC_FIFO4		UDC_OFFSET(0x30)
#define UDC_FIFO5		UDC_OFFSET(0x34)
#define UDC_FIFO6		UDC_OFFSET(0x38)
#define UDC_FIFO7		UDC_OFFSET(0x3C)
#define UDC_FIFO8		UDC_OFFSET(0x40)
#define UDC_FIFO9		UDC_OFFSET(0x44)
#define UDC_FIFO10		UDC_OFFSET(0x48)
#define UDC_FIFO11		UDC_OFFSET(0x4C)
#define UDC_FIFO12		UDC_OFFSET(0x50)
#define UDC_FIFO13		UDC_OFFSET(0x54)
#define UDC_FIFO14		UDC_OFFSET(0x58)
#define UDC_FIFO15		UDC_OFFSET(0x5C)
#define UDC_DEVCTL		UDC_OFFSET(0x60)
#define UDC_TXFIFOSZ		UDC_OFFSET(0x62)
#define UDC_RXFIFOSZ		UDC_OFFSET(0x63)
#define UDC_TXFIFOADDR		UDC_OFFSET(0x64)
#define UDC_RXFIFOADDR		UDC_OFFSET(0x66)

#define UDC_SYSCONFIG		UDC_OFFSET(0x404)
#define UDC_INTERFSEL		UDC_OFFSET(0x40C)
#define UDC_FORCESTDBY		UDC_OFFSET(0x414)

/* MUSB Endpoint parameters */
#define EP0_MAX_PACKET_SIZE	64
#define UDC_OUT_ENDPOINT	2	/* Device RX endpoint */
#define UDC_OUT_PACKET_SIZE	512
#define UDC_IN_ENDPOINT		3	/* Device TX endpoint */
#define UDC_IN_PACKET_SIZE	512
#define UDC_INT_ENDPOINT	1	/* Device Interrupt/Status endpoint */
#define UDC_INT_PACKET_SIZE	16
#define UDC_BULK_PACKET_SIZE	512

#define UDC_MAX_FIFO_SIZE	16384

#define DEV_CONFIG_VALUE	1	/* Only one i.e. CDC */

/* UDC level routines */
void udc_irq(void);
void udc_set_nak(int ep_num);
void udc_unset_nak(int ep_num);
int udc_endpoint_write(struct usb_endpoint_instance *endpoint);
void udc_setup_ep(struct usb_device_instance *device, unsigned int id,
		  struct usb_endpoint_instance *endpoint);
void udc_connect(void);
void udc_disconnect(void);
void udc_enable(struct usb_device_instance *device);
void udc_disable(void);
void udc_startup_events(struct usb_device_instance *device);
int udc_init(void);

#endif /* __MUSB_UDC_H__ */

/*
 * Copyright (C) ST-Ericsson SA 2009
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __MUSB_UDC_H__
#define __MUSB_UDC_H__

#include <linux/byteorder/little_endian.h>
#include <linux/byteorder/generic.h>

/* Endpoint parameters */
#define EP_MAX_PACKET_SIZE	512

#define EP0_MAX_PACKET_SIZE     64
#define UDC_OUT_PACKET_SIZE     EP_MAX_PACKET_SIZE
#define UDC_IN_PACKET_SIZE      EP_MAX_PACKET_SIZE
#define UDC_INT_PACKET_SIZE     EP_MAX_PACKET_SIZE//64
#define UDC_BULK_PACKET_SIZE    EP_MAX_PACKET_SIZE


/* Get offset for a given FIFO */
#define 	FIFO_OFFSET(_bEnd) (0x20 + (_bEnd * 4))

/* Endpoint 0 states */
#define 	END0_STAGE_SETUP	0x0
#define 	END0_STAGE_TX		0x1
#define 	END0_STAGE_RX		0x2
#define 	END0_STAGE_STATUSIN 	0x3
#define 	END0_STAGE_STATUSOUT 	0x4
#define 	END0_STAGE_STALL_BIT 	0x10

/* offsets to registers in flat model */

#define 	HDRC_TXMAXP	0x00
#define 	HDRC_TXCSR	0x02
#define 	HDRC_CSR0	HDRC_TXCSR	/* re-used for EP0 */
#define 	HDRC_RXMAXP	0x04
#define 	HDRC_RXCSR	0x06
#define 	HDRC_RXCOUNT	0x08
#define 	HDRC_COUNT0	HDRC_RXCOUNT	/* re-used for EP0 */
#define 	HDRC_TXTYPE	0x0A
#define 	HDRC_TYPE0	HDRC_TXTYPE	/* re-used for EP0 */
#define 	HDRC_TXINTERVAL	0x0B
#define 	HDRC_NAKLIMIT0	HDRC_TXINTERVAL	/* re-used for EP0 */
#define 	HDRC_RXTYPE	0x0C
#define 	HDRC_RXINTERVAL	0x0D
#define 	HDRC_INDEX	0x0E
#define 	HDRC_FIFOSIZE	0x0F
#define 	HDRC_CONFIGDATA	HDRC_FIFOSIZE	/* re-used for EP0 */

/* INTRUSB */
#define 	INTR_SUSPEND    0x01
#define 	INTR_RESUME     0x02
#define 	INTR_RESET      0x04
#define 	INTR_BABBLE     0x04
#define 	INTR_SOF        0x08 
#define 	INTR_CONNECT    0x10
#define 	INTR_DISCONNECT 0x20
#define 	INTR_SESSREQ    0x40
#define 	INTR_VBUSERROR  0x80   /* FOR SESSION END */
#define 	INTR_EP0        0x01  /* FOR EP0 INTERRUPT */

/* CONFIGDATA */

#define 	CONFIGDATA_MPRXE      0x80	/* auto bulk pkt combining */
#define 	CONFIGDATA_MPTXE      0x40	/* auto bulk pkt splitting */
#define 	CONFIGDATA_BIGENDIAN  0x20
#define 	CONFIGDATA_HBRXE      0x10	/* HB-ISO for RX */
#define 	CONFIGDATA_HBTXE      0x08	/* HB-ISO for TX */
#define 	CONFIGDATA_DYNFIFO    0x04	/* dynamic FIFO sizing */
#define 	CONFIGDATA_SOFTCONE   0x02	/* SoftConnect */
#define 	CONFIGDATA_UTMIDW     0x01   /* data width 0 => 8bits, 1 => 16bits */

/* DEVCTL */
#define 	DEVCTL_BDEVICE    0x80   
#define 	DEVCTL_FSDEV      0x40
#define 	DEVCTL_LSDEV      0x20
#define 	DEVCTL_VBUS       0x18
#define 	DEVCTL_HM         0x04
#define 	DEVCTL_HR         0x02
#define 	DEVCTL_SESSION    0x01

/* TXCSR in Peripheral mode */

#define 	TXCSR_AUTOSET       0x8000
#define 	TXCSR_ISO           0x4000
#define 	TXCSR_MODE          0x2000
#define 	TXCSR_DMAENAB       0x1000
#define 	TXCSR_FRCDATATOG    0x0800
#define 	TXCSR_DMAMODE       0x0400
#define 	TXCSR_CLRDATATOG    0x0040
#define 	TXCSR_FLUSHFIFO     0x0008
#define 	TXCSR_FIFONOTEMPTY  0x0002
#define 	TXCSR_TXPKTRDY      0x0001

#define 	TXCSR_P_INCOMPTX    0x0080
#define 	TXCSR_P_SENTSTALL   0x0020
#define 	TXCSR_P_SENDSTALL   0x0010
#define 	TXCSR_P_UNDERRUN    0x0004

/* RXCSR in Peripheral */

#define 	RXCSR_AUTOCLEAR     0x8000
#define 	RXCSR_DMAENAB       0x2000
#define 	RXCSR_DISNYET       0x1000
#define 	RXCSR_DMAMODE       0x0800
#define 	RXCSR_INCOMPRX      0x0100
#define 	RXCSR_CLRDATATOG    0x0080
#define 	RXCSR_FLUSHFIFO     0x0010
#define 	RXCSR_DATAERROR     0x0008
#define 	RXCSR_FIFOFULL      0x0002
#define 	RXCSR_RXPKTRDY      0x0001


#define 	RXCSR_P_ISO         0x4000
#define 	RXCSR_P_SENTSTALL   0x0040
#define 	RXCSR_P_SENDSTALL   0x0020
#define 	RXCSR_P_OVERRUN     0x0004

#define READ8(base_ptr, offset) *((volatile u8*)((unsigned long)base_ptr + offset))
#define READ16(base_ptr, offset) *((volatile u16*)((unsigned long)base_ptr + offset))
#define READ32(base_ptr, offset) *((volatile uint32_t*)((unsigned long)base_ptr + offset))


#define WRITE8(base_ptr, offset, data)    *(volatile u8*)((unsigned long)base_ptr + offset) = data;   
#define WRITE16(base_ptr, offset, data)    *(volatile u16*)((unsigned long)base_ptr + offset) = data; 
#define WRITE32(base_ptr, offset, data)   *(volatile uint32_t*)((unsigned long)base_ptr + offset) = data; 

#define SELECTEND(base_ptr, end)     		 WRITE8(base_ptr, HDRC_INDEX, end)
#define READCSR8(base_ptr, offset, end)     	 READ8(base_ptr, (offset + 0x10))
#define READCSR16(base_ptr, offset, end)         READ16(base_ptr, (offset + 0x10))
#define WRITECSR8(base_ptr, offset, end, data) 	 WRITE8(base_ptr, (offset + 0x10), data)
#define WRITECSR16(base_ptr, offset, end, data)  WRITE16(base_ptr, (offset + 0x10), data)

#define 	MODE_ULPI 		0x01
#define 	POWER_HSENAB     	0x20
#define 	POWER_SOFTCONN   	0x40

/*EP0 related macros */
/* CSR0 in Peripheral mode */
#define 	CSR0_P_SVDSETUPEND  	0x0080
#define 	CSR0_P_SVDRXPKTRDY  	0x0040
#define 	CSR0_P_SENDSTALL    	0x0020
#define 	CSR0_P_SETUPEND     	0x0010
#define 	CSR0_P_DATAEND      	0x0008
#define 	CSR0_P_SENTSTALL    	0x0004
#define		CSR0_FLUSHFIFO      	0x0100
#define 	CSR0_TXPKTRDY       	0x0002
#define 	CSR0_RXPKTRDY       	0x0001


#define 	EP_ACTIVE		0
#define 	EP_HALTED		1
#define 	EP_DISABLED		2

/* TESTMODE */

#define 	TEST_FORCE_HOST   	0x80
#define 	TEST_FIFO_ACCESS  	0x40
#define 	TEST_FORCE_FS     	0x20
#define 	TEST_FORCE_HS     	0x10
#define 	TEST_PACKET       	0x08
#define 	TEST_K            	0x04
#define 	TEST_J            	0x02
#define 	TEST_SE0_NAK      	0x01

struct endpoint_control {
    __u8     OTG_TX0FAD;
    __u8     FILL4;
    __u8     OTG_TX0HAD;
    __u8     OTG_TX0HP;
    __u8     OTG_RX0FAD;
    __u8     FILL5;
    __u8     OTG_RX0HAD;
    __u8     OTG_RX0HP;
};
struct mg_dev_register
{
    __u8     OTG_FADDR;
    __u8     OTG_PWR;
    __u16    OTG_INTTX;
    __u16    OTG_INTRX;
    __u16    OTG_INTTXEN;
    __u16    OTG_INTRXEN;
    __u8     OTG_INTUSB;
    __u8     OTG_INTUSBEN;
    __u16    OTG_FMNO;
    __u8     OTG_INDX;
    __u8     OTG_TM;
    __u16    OTG_TXMAXP;
    __u16    OTG_CSR0_TXCSR;
    __u16    OTG_RXMAXP;
    __u16    OTG_RXCSR;
    __u16    OTG_CNT0_RXCNT;
    __u8     OTG_TYPE0_TXTYPE;
    __u8     OTG_NAKLMT0_TXINTV;
    __u8     OTG_RXTYPE;
    __u8     OTG_RXINTV;
    __u8     FILL0;
    __u8     OTG_CFD_FSIZE;

    /* FIFO registers */
    __u32    OTG_FIFO[16];
    __u8     OTG_DEVCTL;
    __u8     FILL1;
    __u8     OTG_TXFSZ;
    __u8     OTG_RXFSZ;
    __u16    OTG_TXFA;
    __u16    OTG_RXFA;
    __u32    OTG_VCNTL;
    __u16    OTG_HWVER;
    __u16    FILL2;
    __u8     OTG_UVBCTRL;
    __u8     OTG_UCKIT;
    __u8     OTG_UINTMASK;
    __u8     OTG_UINTSRC;
    __u8     OTG_UREGDATA;
    __u8     OTG_UREGADDR;
    __u8     OTG_UREGCTRL;
    __u8     OTG_URAWDATA;
    __u8     OTG_EPINFO;
    __u8     OTG_RAMINFO;
    __u8     OTG_LINKINFO;
    __u8     OTG_VPLEN;
    __u8     OTG_HSEOF1;
    __u8     OTG_FSEOF1;
    __u8     OTG_LSEOF1;
    __u8     OTG_SOFTRST;

    /* Target Endpoint control registers */
    struct endpoint_control endp_control[16];  
    /* Control Status register for Endpoint 0 */
    __u16    OTG_TX0MAXP;
    __u16    OTG_CSR0;
    __u16    OTG_RX0MAXP;
    __u16    OTG_RX0CSR;
    __u16    OTG_CNT0;
    __u8     OTG_TYPE0;
    __u8     OTG_NAKLMT0;
    __u8     OTG_RX0TYPE;
    __u8     OTG_RX0INTV;
    __u8     FILL36;
    __u8     OTG_CFD;

    /* Control Status register for Endpoint 1 */
    __u16    OTG_TX1MAXP;
    __u16    OTG_TX1CSR;
    __u16    OTG_RX1MAXP;
    __u16    OTG_RX1CSR;
    __u16    OTG_RX1CNT;
    __u8     OTG_TX1TYPE;
    __u8     OTG_TX1INTV;
    __u8     OTG_RX1TYPE;
    __u8     OTG_RX1INTV;
    __u8     FILL37;
    __u8     OTG_FSIZE1;

    /* Control Status register for Endpoint 2 */
    __u16    OTG_TX2MAXP;
    __u16    OTG_TX2CSR;
    __u16    OTG_RX2MAXP;
    __u16    OTG_RX2CSR;
    __u16    OTG_RX2CNT;
    __u8     OTG_TX2TYPE;
    __u8     OTG_TX2INTV;
    __u8     OTG_RX2TYPE;
    __u8     OTG_RX2INTV;
    __u8     FILL38;
    __u8     OTG_FSIZE2;

    /* Control Status register for Endpoint 3 */
    __u16    OTG_TX3MAXP;
    __u16    OTG_TX3CSR;
    __u16    OTG_RX3MAXP;
    __u16    OTG_RX3CSR;
    __u16    OTG_RX3CNT;
    __u8     OTG_TX3TYPE;
    __u8     OTG_TX3INTV;
    __u8     OTG_RX3TYPE;
    __u8     OTG_RX3INTV;
    __u8     FILL39;
    __u8     OTG_FSIZE3;

    /* Control Status register for Endpoint 4 */
    __u16    OTG_TX4MAXP;
    __u16    OTG_TX4CSR;
    __u16    OTG_RX4MAXP;
    __u16    OTG_RX4CSR;
    __u16    OTG_RX4CNT;
    __u8     OTG_TX4TYPE;
    __u8     OTG_TX4INTV;
    __u8     OTG_RX4TYPE;
    __u8     OTG_RX4INTV;
    __u8     FILL40;
    __u8     OTG_FSIZE4;

    /* Control Status register for Endpoint 5 */
    __u16    OTG_TX5MAXP;
    __u16    OTG_TX5CSR;
    __u16    OTG_RX5MAXP;
    __u16    OTG_RX5CSR;
    __u16    OTG_RX5CNT;
    __u8     OTG_TX5TYPE;
    __u8     OTG_TX5INTV;
    __u8     OTG_RX5TYPE;
    __u8     OTG_RX5INTV;
    __u8     FILL41;
    __u8     OTG_FSIZE5;

    /* Control Status register for Endpoint 6 */
    __u16    OTG_TX6MAXP;
    __u16    OTG_TX6CSR;
    __u16    OTG_RX6MAXP;
    __u16    OTG_RX6CSR;
    __u16    OTG_RX6CNT;
    __u8     OTG_TX6TYPE;
    __u8     OTG_TX6INTV;
    __u8     OTG_RX6TYPE;
    __u8     OTG_RX6INTV;
    __u8     FILL42;
    __u8     OTG_FSIZE6;

    /* Control Status register for Endpoint 7 */
    __u16    OTG_TX7MAXP;
    __u16    OTG_TX7CSR;
    __u16    OTG_RX7MAXP;
    __u16    OTG_RX7CSR;
    __u16    OTG_RX7CNT;
    __u8     OTG_TX7TYPE;
    __u8     OTG_TX7INTV;
    __u8     OTG_RX7TYPE;
    __u8     OTG_RX7INTV;
    __u8     FILL43;
    __u8     OTG_FSIZE7;

    /* Control Status register for Endpoint 8 */
    __u16    OTG_TX8MAXP;
    __u16    OTG_TX8CSR;
    __u16    OTG_RX8MAXP;
    __u16    OTG_RX8CSR;
    __u16    OTG_RX8CNT;
    __u8     OTG_TX8TYPE;
    __u8     OTG_TX8INTV;
    __u8     OTG_RX8TYPE;
    __u8     OTG_RX8INTV;
    __u8     FILL44;
    __u8     OTG_FSIZE8;

    /* Control Status register for Endpoint 9 */
    __u16    OTG_TX9MAXP;
    __u16    OTG_TX9CSR;
    __u16    OTG_RX9MAXP;
    __u16    OTG_RX9CSR;
    __u16    OTG_RX9CNT;
    __u8     OTG_TX9TYPE;
    __u8     OTG_TX9INTV;
    __u8     OTG_RX9TYPE;
    __u8     OTG_RX9INTV;
    __u8     FILL45;
    __u8     OTG_FSIZE9;

    /* Control Status register for Endpoint 10 */
    __u16    OTG_TX10MAXP;
    __u16    OTG_TX10CSR;
    __u16    OTG_RX10MAXP;
    __u16    OTG_RX10CSR;
    __u16    OTG_RX10CNT;
    __u8     OTG_TX10TYPE;
    __u8     OTG_TX10INTV;
    __u8     OTG_RX10TYPE;
    __u8     OTG_RX10INTV;
    __u8     FILL46;
    __u8     OTG_FSIZE10;

    /* Control Status register for Endpoint 11 */
    __u16    OTG_TX11MAXP;
    __u16    OTG_TX11CSR;
    __u16    OTG_RX11MAXP;
    __u16    OTG_RX11CSR;
    __u16    OTG_RX11CNT;
    __u8     OTG_TX11TYPE;
    __u8     OTG_TX11INTV;
    __u8     OTG_RX11TYPE;
    __u8     OTG_RX11INTV;
    __u8     FILL47;
    __u8     OTG_FSIZE11;

    /* Control Status register for Endpoint 12 */
    __u16    OTG_TX12MAXP;
    __u16    OTG_TX12CSR;
    __u16    OTG_RX12MAXP;
    __u16    OTG_RX12CSR;
    __u16    OTG_RX12CNT;
    __u8     OTG_TX12TYPE;
    __u8     OTG_TX12INTV;
    __u8     OTG_RX12TYPE;
    __u8     OTG_RX12INTV;
    __u8     FILL48;
    __u8     OTG_FSIZE12;

    /* Control Status register for Endpoint 13 */
    __u16    OTG_TX13MAXP;
    __u16    OTG_TX13CSR;
    __u16    OTG_RX13MAXP;
    __u16    OTG_RX13CSR;
    __u16    OTG_RX13CNT;
    __u8     OTG_TX13TYPE;
    __u8     OTG_TX13INTV;
    __u8     OTG_RX13TYPE;
    __u8     OTG_RX13INTV;
    __u8     FILL49;
    __u8     OTG_FSIZE13;

    /* Control Status register for Endpoint 14 */
    __u16    OTG_TX14MAXP;
    __u16    OTG_TX14CSR;
    __u16    OTG_RX14MAXP;
    __u16    OTG_RX14CSR;
    __u16    OTG_RX14CNT;
    __u8     OTG_TX14TYPE;
    __u8     OTG_TX14INTV;
    __u8     OTG_RX14TYPE;
    __u8     OTG_RX14INTV;
    __u8     FILL50;
    __u8     OTG_FSIZE14;

    /* Control Status register for Endpoint 15 */
    __u16    OTG_TX15MAXP;
    __u16    OTG_TX15CSR;
    __u16    OTG_RX15MAXP;
    __u16    OTG_RX15CSR;
    __u16    OTG_RX15CNT;
    __u8     OTG_TX15TYPE;
    __u8     OTG_TX15INTV;
    __u8     OTG_RX15TYPE;
    __u8     OTG_RX15INTV;
    __u8     FILL51;
    __u8     OTG_FSIZE15;

    __u32    OTG_DMASEL;
    __u8     OTG_TOPCTRL;
};

struct udc_end0_buffer {
	u8 	data[EP0_MAX_PACKET_SIZE];
	u16	count;
};
/* Higher level functions for abstracting away from specific device */
int  udc_init (void);

void udc_enable(struct usb_device_instance *device);
void udc_disable(void);

void udc_connect(void);
void udc_disconnect(void);

void udc_startup_events(struct usb_device_instance *device);
void udc_setup_ep(struct usb_device_instance *device, unsigned int ep, struct usb_endpoint_instance *endpoint);

void udc_irq (void);
/* Flow control */
void udc_set_nak(int epid);
void udc_unset_nak (int epid);
int udc_endpoint_write(struct usb_endpoint_instance *endpoint);

#endif //__MUSB_UDC_H__

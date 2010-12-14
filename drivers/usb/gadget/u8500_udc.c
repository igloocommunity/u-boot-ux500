/*
 * u8500_udc.c - 
 *
 * (C) Copyright 2009 ST-Ericsson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307	 USA
 *
 */

#include <common.h>
#include "u8500_udc.h"

static volatile struct mg_dev_register *pRegs = 0;

int udc_musb_platform_init(void)
{
	u16 top;
	u8 power;
	u8 soft_reset;
	u16	temp;
	pRegs = (volatile struct mg_dev_register *) CONFIG_USB_BASE;

	top =  pRegs->OTG_TOPCTRL;
	pRegs->OTG_TOPCTRL = (top | MODE_ULPI);

	soft_reset =  pRegs->OTG_SOFTRST;
	pRegs->OTG_SOFTRST = (soft_reset | 0x2);

	power = pRegs->OTG_PWR;
	/* Enabling high speed and soft connection */
	power = power | POWER_HSENAB;
	pRegs->OTG_PWR = (power | POWER_SOFTCONN);

	pRegs->OTG_INTUSBEN = 0x0;
	pRegs->OTG_INTTXEN  = 0x0;
	pRegs->OTG_INTRXEN  = 0x0;

	/* off */
	pRegs->OTG_DEVCTL  = 0x0;

	/*  flush pending interrupts */
	temp = pRegs->OTG_INTUSB;
	temp = pRegs->OTG_INTTX;
	temp = pRegs->OTG_INTRX;
	return 0;
}

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

#ifndef _STM_EMMC_H
#define _STM_EMMC_H


#include <common.h>

extern int 	emmc_init (u8); 
int  	emmc_erase(u32, u32);
int  	emmc_read (u32,u32,u32);
int  	emmc_write(u32,u32,u32);
int 	emmc_write_pib(void);


#endif /* !defined(_STM_EMMC_H) */

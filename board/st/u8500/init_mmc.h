/*
 * (C) Copyright 2009
 * STEricsson, <www.stericsson.com>
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

#ifndef __INIT_MMC_H
#define __INIT_MMC_H
#define PUBLIC        /* Extern by default */

#include <common.h>

typedef void    (*t_callback_fct) (u32);
typedef struct
{
    t_callback_fct  fct;
    u32        param;
}   t_callback; 

static int init_mmc(void);
int  init_mmc_fat(void); 
t_mmc_error mmc_fat_read_file    (char *, u32, u32);
int mmc_hw_init (void);
unsigned long mmc_block_read(int dev,unsigned long blknr,lbaint_t blkcnt,void *dest);


#endif /* !defined(__INIT_MMC_H) */

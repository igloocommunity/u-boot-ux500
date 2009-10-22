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

/* Support for ST NOR flash M30L0R7KB02AQ  and M30L0R7KT02AQ */

#include <common.h>
#include <linux/byteorder/swab.h>

#undef DEBUG_FLASH 
flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS]; /* info for FLASH chips  */

/* Board support for 1 or 2 flash devices */
#undef FLASH_PORT_WIDTH32
#define FLASH_PORT_WIDTH16

#ifdef FLASH_PORT_WIDTH16
#define FLASH_PORT_WIDTH		ushort
#define FLASH_PORT_WIDTHV		vu_short
#define SWAP(x)			__swab16(x)
#else
#define FLASH_PORT_WIDTH		ulong
#define FLASH_PORT_WIDTHV		vu_long
#define SWAP(x)			__swab32(x)
#endif

#define FPW	FLASH_PORT_WIDTH
#define FPWV	FLASH_PORT_WIDTHV

/*-----------------------------------------------------------------------
 * Functions
 */
unsigned long flash_init(void);
/*static ulong flash_get_size(FPW * addr, flash_info_t * info);
static int write_data(flash_info_t * info, ulong dest, FPW data);
static void flash_get_offsets(ulong base, flash_info_t * info);*/
void inline spin_wheel(void);
void flash_print_info(flash_info_t * info);
void flash_unprotect_sectors(FPWV * addr);
int flash_erase(flash_info_t * info, int s_first, int s_last);
int write_buff(flash_info_t * info, uchar * src, ulong addr, ulong cnt);

/*-----------------------------------------------------------------------
 */

unsigned long flash_init(void)
{
	ulong size = 0;
	return size;
}

/*-----------------------------------------------------------------------
 */
void flash_print_info(flash_info_t * info)
{
}

/* unprotects a sector for write and erase
 * on some intel parts, this unprotects the entire chip, but it
 * wont hurt to call this additional times per sector...
 */
void flash_unprotect_sectors(FPWV * addr)
{
	return;
}

/*-----------------------------------------------------------------------
 */

int flash_erase(flash_info_t * info, int s_first, int s_last)
{
	return 0;
}

/*-----------------------------------------------------------------------
 * Copy memory to flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 * 4 - Flash not identified
 */

int write_buff(flash_info_t * info, uchar * src, ulong addr, ulong cnt)
{
	return 0;
}

void inline spin_wheel(void)
{
}

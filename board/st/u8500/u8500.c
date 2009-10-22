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

#include <config.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>

#include "common.h"

void init_regs(void);
#if 0
void *memcopy(void *dest, const void *src, size_t count)
{
    u16 *tmp = (u16 *) dest, *s = (u16 *) src;

    count = count / 2;
    while (count--)
        *tmp++ = *s++;

    return dest;
}
#endif
#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
    printf("Boot reached stage %d\n", progress);
}
#endif

static inline void delay(unsigned long loops)
{
    __asm__ volatile ("1:\n"
              "subs %0, %1, #1\n" "bne 1b":"=r" (loops):"0"(loops));
}

/*
* Miscellaneous platform dependent initialisations
*/

int board_init(void)
{    
    DECLARE_GLOBAL_DATA_PTR;
    gd->bd->bi_arch_number = 0x1A4;
    gd->bd->bi_boot_params = 0x00000100;
    //enable the timers in PRCMU reg
    *((volatile unsigned int *)(CFG_PRCMU_BASE + 0x1C8)) = 0x20000;
    icache_enable();
    gpio_init();

    init_regs();
    return 0;
}

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
    setenv("verify", "n");
    return (0);
}
#endif

/******************************
Routine:
Description:
******************************/
int dram_init(void)
{
    DECLARE_GLOBAL_DATA_PTR;
    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_SIZE_1;
    gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
    gd->bd->bi_dram[1].size = PHYS_SDRAM_SIZE_2;

    return 0;
}

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))
unsigned int addr_vall_arr[] = {
0x8011F000, 0x0000FFFF, // Clocks for HSI  TODO Enable reqd only
0x8011F008, 0x00001C44, // Clocks for HSI  TODO Enable reqd only
0x8000F000, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x8000F008, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x8000E120, 0x003C0000, // GPIO for I2C/SD
0x8000E124, 0x00000000, // GPIO for I2C/SD
0x8012F000, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0x8012F008, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0xA03DF000, 0x0000000D, // Clock for MTU Timers
0x8011E00C, 0x00000000, // GPIO ALT FUNC for EMMC
0x8011E004, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E020, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E024, 0x00000000, // GPIO ALT FUNC for EMMC
0x8012E00C, 0x00000000, // GPIO ALT FUNC for SD
0x8012E004, 0x0FFC0000, // GPIO ALT FUNC for SD
0x8012E020, 0x60000000, // GPIO ALT FUNC for SD
0x8012E024, 0x60000000, // GPIO ALT FUNC for SD
0x801571E4, 0x0000000C, // PRCMU settings for B2R2
0x80157024, 0x00000130, // PRCMU settings for EMMC/SD
0xA03FF000, 0x00000003, // USB
0xA03FF008, 0x00000001, // USB
0xA03FE00C, 0x00000000, // USB
0xA03FE020, 0x00000FFF, // USB
0xA03FE024, 0x00000000  // USB
};
#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
	return (0);
}
#endif
void init_regs(void)
{    
    int i;
    for(i = 0; i < ARRAY_SIZE(addr_vall_arr)/2; i++)
    {
        
        *((volatile unsigned int *)(addr_vall_arr[2 * i]))
                                = addr_vall_arr[(2 * i) + 1];
    }    
}

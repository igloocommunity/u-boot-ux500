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
/* --- includes ----------------------------------------------------------- */
#include "common.h"	/* XXX: Arrgghh! "common.h" includes <common.h> */
#include <command.h>
#include "mmc.h"
#include "emmc.h"
#include "gpio.h"
#include <asm/boottime.h>

#define PIB_EMMC_ADDR 0x00
/* ========================================================================
Name:        init_emmc
Description: init embedded multimedia card interface

======================================================================== */
int emmc_init(u8 card_num)
{
    t_mmc_error mmc_error;
    t_mmc_error response;
    gpio_error gpioerror;
    t_logical_address mmcbase;
    int error;

    mmcbase = u8500_is_earlydrop() ? CFG_EMMC_BASE_ED : CFG_EMMC_BASE_V1;

/* Initialize the base address of eMMC */
    mmc_error = mmc_init(card_num, mmcbase);

    if (MMC_OK != mmc_error)
    {
        printf("emmc_init() %d \n", mmc_error);
        goto end;
    }

    /*
     * FIXME The following code is not required on v1.  Check if it is really
     * needed on ED or can be dropped.
     */
    if (u8500_is_earlydrop()) {
#ifndef CONFIG_U8500_V1
	/* Initialize the gpio alternate function for eMMC */
	struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_6_BASE);
	p_gpio_register -> gpio_dats |= 0x0000FFE0;
	p_gpio_register -> gpio_pdis &= ~0x0000FFE0;

	//enable the alternate function of EMMC
	gpioerror = gpio_altfuncenable(GPIO_ALT_EMMC, "EMMC");
	if(gpioerror != GPIO_OK)
	{
	    printf("emmc_init() gpio_altfuncenable %d failed\n", gpioerror);
	    goto end;
	}

#else
	//enable the alternate function of PoP EMMC
	// gpioerror = gpio_altfuncenable(GPIO_ALT_POP_EMMC, "EMMC");
	gpioerror = gpio_altfuncenable(GPIO_ALT_EMMC, "EMMC");
	if (gpioerror != GPIO_OK) {
	    printf("emmc_init() gpio_altfuncenable %d failed \n",
		   gpioerror);
	    goto end;
	}
#endif
    }

    //Power-on the controller
    response = mmc_poweron(card_num);
    if (response != MMC_OK)
    {
	 printf("Error in eMMC power on, response is %d\n",response);
         goto end;
    }
    // Initialise the cards,get CID and CSD on the bus
    response = mmc_initializeCards(card_num);

    if (response != MMC_OK)
    {
        printf(" Error in eMMC initialization\n");
        goto end;
    }

    error = emmc_write_pib();
    if(error)
	    printf("PIB info writing into eMMC failed\n");
    printf("eMMC done\n");

    return 0;

end:
    mmc_poweroff(card_num);
    return 1;
}

struct partition {
	unsigned char boot_ind;		/* 0x80 - active */
	unsigned char head;		/* starting head */
	unsigned char sector;		/* starting sector */
	unsigned char cyl;		/* starting cylinder */
	unsigned char sys_ind;		/* What partition type */
	unsigned char end_head;		/* end head */
	unsigned char end_sector;	/* end sector */
	unsigned char end_cyl;		/* end cylinder */
	u32 start_sect;      /* starting sector counting from 0 */
	u32 nr_sects;		     /* nr of sectors in partition */
} __attribute__((packed));

#define PART(type, start, num)			\
	{					\
		.boot_ind = 0x00,		\
		.head = 0x03,			\
		.sector = 0xD0,			\
		.cyl = 0xff,			\
		.sys_ind = type,		\
		.end_head = 0x03,		\
		.end_sector = 0xd0,		\
		.end_cyl = 0xff,		\
		.start_sect = start,		\
		.nr_sects = num,		\
	}

static struct partition partitions_ed[] = {
	[0] = PART(0x83, 0x000A0000,  0x00004000),	/* Kernel */
	[1] = PART(0x83, 0x000A4000,  0x00080000),	/* Root file system */
	[2] = PART(0x83, 0x00124000,  0x0022c000),
	[3] = PART(0x0c, 0x00350000,  0x00b9a000),
};

static struct partition partitions_v1[] = {
	[0] = PART(0x83, 0x000A0000,  0x00004000),	/* Kernel */
	[1] = PART(0x83, 0x000A4000,  0x00080000),	/* Root file system */
	[2] = PART(0x83, 0x00124000,  0x00000800),	/* Modem parameters */
	[3] = {0},
};

#undef PART

int emmc_write_pib(void)
{
    int i;
    t_mmc_error mmc_error;
    u32 block_offset = PIB_EMMC_ADDR;
    u8 card_num = 4;
    u8 mbr[512];

    memset(mbr, 0, 0x1be);

    if (u8500_is_earlydrop())
        memcpy(mbr + 0x1be, partitions_ed, sizeof(partitions_ed));
    else
        memcpy(mbr + 0x1be, partitions_v1, sizeof(partitions_v1));

    /* magic */
    mbr[0x1fe] = 0x55;
    mbr[0x1ff] = 0xAA;

/*  HACK required for HREF board as erase block size = 512KB */
/*
    mmc_error = mmc_erase(card_num, 0x0, 0x1FF);
    if (mmc_error != MMC_OK) {
        printf(" eMMC erase failed in PIB \n");
        return 1;
    }
*/

    mmc_error =
        mmc_writeblocks(card_num, block_offset, (u32 *) mbr,
                512, 1);
    if (mmc_error != MMC_OK) {
        printf(" eMMC PIB write failed \n");
        return 1;
    }
    return 0;
}

int emmc_erase(u32 start, u32 end)
{
    t_mmc_error mmc_error;
    u8 card_num = 4;
    printf("emmc erase start \n");
    mmc_error = mmc_erase(card_num, start, end);
    if (mmc_error != MMC_OK) {
        printf(" eMMC erase failed \n");
        return 1;
    }
    printf("emmc erase done \n");
    return 0;
}

int emmc_read(u32 block_offset, u32 read_buffer, u32 filesize)
{
	t_mmc_error mmc_error;
	u32 remaining;
	u8 card_num = 4;
	u8 *mem_address = (u8 *) read_buffer;
	u32 n=filesize,blocks;

	remaining = filesize;

	printf(" eMMC read start filesize=0x%x \n", filesize);

	blocks = (n%512==0)?(n/512):(n/512)+1;

	while(blocks>=8)
	{
		mmc_error = mmc_readblocks(card_num, block_offset, (u32 *) mem_address,	512, 8);
		if (mmc_error != MMC_OK)
		{
			printf(" eMMC read blocks failed \n");
			return 1;
		}

		block_offset += 4096;
		mem_address += 4096;
		blocks -=8;
		remaining -= 4096;
        }
        if(blocks)
	{
		mmc_error = mmc_readblocks(card_num, block_offset, (u32 *) mem_address,	512, blocks);
		if (mmc_error != MMC_OK)
		{
			printf(" eMMC read blocks failed \n");
			return 1;
		}
        }

	printf(" eMMC read done \n");
	return 0;
}

int emmc_write(u32 block_offset, u32 write_buffer, u32 filesize)
{
	t_mmc_error mmc_error;
	u32 remaining;
	u8 card_num = 4;
	u8 *mem_address = (u8 *) write_buffer;
	u32 n=filesize,blocks;

	remaining = filesize;

	printf(" eMMC write start filesize=0x%x \n", filesize);

	blocks = (n%512==0)?(n/512):(n/512)+1;
	while(blocks>=8)
	{
		mmc_error = mmc_writeblocks(card_num, block_offset, (u32 *) mem_address,512, 8);
		if (mmc_error != MMC_OK)
		{
			printf(" eMMC write blocks failed \n");
			return 1;
		}

		block_offset += 4096;
		mem_address += 4096;
		blocks -=8;
		remaining -= 4096;
        }
        if(blocks)
	{

		mmc_error = mmc_writeblocks(card_num, block_offset, (u32 *) mem_address,512, blocks);
		if (mmc_error != MMC_OK)
		{
			printf(" eMMC write blocks failed \n");
			return 1;
		}

        }

	printf(" eMMC write done \n");
	return 0;
}

/*
 * command line commands
 */
#ifdef CONFIG_CMD_EMMC
int do_emmc_erase(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	u32       start_address;
	u32       end_address;
	int       load_result = 1;
	u32       error_name  = 0;

	start_address  = simple_strtoul(argv[1],0,16);
	end_address  = simple_strtoul(argv[2],0,16);

	printf("emmc_erase :: start address = %x end_address=0x%x\n",start_address,end_address);

        load_result      = emmc_erase(start_address,end_address);
        if (load_result != 0)
        {
            error_name   = (unsigned long) (-load_result);
            printf("emmc_erase error : failed  \n");
        }
        return(0);
}

U_BOOT_CMD(
	emmc_erase,	3,	0,	do_emmc_erase,
	"- erase the eMMC flash \n",
	"start_address- start address of the eMMC block\n"
	"end_address- end address of the eMMC block\n"
);

int do_emmc_read(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	u32       ram_address;
	u32       block_offset;
	u32       filesize;
	int       load_result = 1;
	u32       error_name  = 0;

	ram_address	= simple_strtoul (argv[1],0,16);
        block_offset	= simple_strtoul (argv[2],0,16);
        filesize	= simple_strtoul (argv[3],0,16);

	boottime_tag("load_image");
	printf("emmc_read :: ram address = 0x%x block address=0x%x \n",ram_address,block_offset);

        load_result      = emmc_read(block_offset,ram_address,filesize);
        if (load_result != 0)
        {
		boottime_remove_last();
            error_name   = (unsigned long) (-load_result);
            printf("emmc_read error : in reading data from eMMC block \n");
        }
        return(0);
}

U_BOOT_CMD(
	emmc_read,	4,	0,	do_emmc_read,
	"- read file from emmc flash \n",
	"ram_address - read from eMMC and copy into ram address\n"
	"block_offset - address to read the file from the eMMC block \n"
	"filesize - size of the file \n"
);

int do_emmc_write(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	u32       ram_address;
	u32       block_offset;
	u32       filesize;
	int       load_result = 1;
	u32       error_name  = 0;

	ram_address	= simple_strtoul (argv[1],0,16);
        block_offset	= simple_strtoul (argv[2],0,16);
        filesize	= simple_strtoul (argv[3],0,16);

	printf("emmc_write :: ram address = %x block address=0x%x \n",ram_address,block_offset);

        load_result      = emmc_write(block_offset,ram_address,filesize);
        if (load_result != 0)
        {
            error_name   = (unsigned long) (-load_result);
            printf("emmc_read error : in writing data into eMMC block \n");
        }
        return(0);
}

U_BOOT_CMD(
	emmc_write,	4,	0,	do_emmc_write,
	"- write file from emmc flash \n",
	"ram_address - write to eMMC by copying from ram address\n"
	"block_offset - address to write the file into the eMMC block \n"
	"filesize - size of the file \n"
);

#endif	/* CONFIG_CMD_EMMC */
/* ------------------------------- End of file ---------------------------- */

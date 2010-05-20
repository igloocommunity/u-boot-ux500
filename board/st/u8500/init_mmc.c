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
#include "common.h"
#include "mmc.h"
#include "init_mmc.h"
#include "gpio.h"
#include "mmc_utils.h"

#ifdef CONFIG_CMD_FAT
#include <part.h>
#include <fat.h>
#endif

#define MMC_CARD_NUM	1
#define EMMC_CARD_NUM	4
enum {
	DEV_EMMC = 0,
	DEV_MMC
};

static block_dev_desc_t mmc_dev;
static block_dev_desc_t emmc_dev;
static u32	CSD[4];

block_dev_desc_t * mmc_get_dev(int dev)
{
	if (dev == DEV_EMMC)
		return (block_dev_desc_t *)(&emmc_dev);
	else if (dev == DEV_MMC)
		return (block_dev_desc_t *)(&mmc_dev);

	printf("mmc_get_dev: unknown dev %d\n", dev);
	return 0;
}

static unsigned long emmc_block_read(int dev, unsigned long blknr,
					lbaint_t blkcnt, void *dest)
{
	unsigned long rc;

	rc = mmc_readblocks(EMMC_CARD_NUM, (u32) (512 * blknr), (u32 *)dest,
			512, blkcnt);
	if (rc != 0) {
		printf("mmc_block_read: readblocks failed %ld\n", rc);
		rc = 0;
	} else {
		rc = blkcnt;
	}
	return rc;
}

unsigned long mmc_block_read(int dev, unsigned long blknr,
					lbaint_t blkcnt, void *dest)
{
	unsigned long i;
	unsigned long src= blknr;
	unsigned long rc = 0;

	for(i = 0; i < blkcnt; i++)
	{
		/* card#, read offset, buffer, blksize, transfer mode */
		mmc_readblock(MMC_CARD_NUM, (u32) (512 * src),
				(u32 *)dest, 512, MMCPOLLING);
		rc++;
		src++;
		dest += 512;
	}
	return rc;
}

int mmc_hw_init(void)
{
    t_mmc_error           response;    

    /* save the GPIO0 AFSELA register*/
    gpio_altfuncenable(GPIO_ALT_SD_CARD0, "MMC");
    /* Power-on the controller*/
    response = mmc_enable ();

    if (response != MMC_OK)
    {
	response = mmc_enable ();
        if (response != MMC_OK)
        {
            printf ("Error in card power on\n");
            goto end;
        }
    }
    /* Initialise the cards on the bus, if any*/
    response = mmc_initCard ();
    if (response != MMC_OK)
    {
	printf ("Error in card initialization\n"); 
	goto end;
    }
    response = mmc_readcsd (CSD);
    if (response != MMC_OK)
    {
        printf ("Error while fetching card info\n");
        goto end;
    } 
    
    return 0;
  end:
    return 1;
}

/*
 * mmc_legacy_init - called from commandline mmc init <dev>
 *
 * Initialise hardware and setup block device structure for fat and ext2
 * commands.
 */
int mmc_legacy_init(int dev)
{

	if (dev == DEV_EMMC) {
		debug("EMMC init\n");
		/* XXX: emmc_init() does write the MBR (called pib)! */
		emmc_init(EMMC_CARD_NUM);
		emmc_dev.if_type = IF_TYPE_MMC;
		emmc_dev.part_type = PART_TYPE_DOS;
		emmc_dev.dev = dev;
		emmc_dev.lun = 0;
		emmc_dev.type = 0;
		emmc_dev.blksz = 512;
		emmc_dev.lba = 0x80000; /* XXX: use real size, here 256 MB */
		sprintf((char*)emmc_dev.vendor, "Unknown vendor");
		sprintf((char*)emmc_dev.product, "Unknown product");
		sprintf((char*)emmc_dev.revision, "N/A");
		emmc_dev.removable = 0;
		emmc_dev.block_read = emmc_block_read;
		return 0;
	} else if (dev == DEV_MMC) {
		debug("MMC init\n");
		return init_mmc();
	}

	printf("mmc_legacy_init: unsupported device# %d\n", dev);
	return -1;
}

/* ========================================================================
   Name:        init_mmc
   Description: init multimedia card interface

   ======================================================================== */
static int init_mmc(void)
{
    t_mmc_error                   mmc_error;
    struct gpio_register * gpio_base_address;
    
    /* Initialize the base address of MMC-SD */
    mmc_error = mmc_init (0,CFG_MMC_BASE);
    
    if (MMC_OK != mmc_error)
    {
        printf("mmc_init():: %d \n", mmc_error);
        return 1 ;
    }    

	if (u8500_is_earlydrop()) {
		gpio_base_address = (void *) IO_ADDRESS(CFG_GPIO_0_BASE);
		gpio_base_address -> gpio_dats |= 0xFFC0000;
		gpio_base_address -> gpio_pdis &= ~0xFFC0000;
	}

	if (mmc_hw_init() != 0) {
		printf("mmc_init: hw init failed\n");
		return -1;
	}
	mmc_dev.if_type = IF_TYPE_MMC;
	mmc_dev.part_type = PART_TYPE_DOS;
	mmc_dev.dev = 0;
	mmc_dev.lun = 0;
	mmc_dev.type = 0;
	mmc_dev.blksz = 512;
	mmc_dev.lba = 0x20000; /* XXX: use real size, here 64 MB */
	sprintf((char*)mmc_dev.vendor, "Unknown vendor");
	sprintf((char*)mmc_dev.product, "Unknown product");
	sprintf((char*)mmc_dev.revision, "N/A");
	mmc_dev.removable = 0;
	mmc_dev.block_read = mmc_block_read;
#ifdef CONFIG_CMD_FAT
	if (fat_register_device(&mmc_dev, 1) != 0) {
		printf("mmc_init: could not register as FAT device\n");
	}
#endif /* CONFIG_CMD_FAT */

	return 0;
}


/* ------------------------------- End of file ---------------------------- */

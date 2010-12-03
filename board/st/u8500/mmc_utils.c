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
#include <linux/ctype.h>

#include <asm/arch/common.h>
#include <asm/arch/cpu.h>
#include <command.h>
#include <mmc.h>
#include <fat.h>

#define PIB_EMMC_ADDR 0x00

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
	[2] = PART(0x83, 0x00000400,  0x00000800),	/* Modem parameters */
	[3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

#undef PART

int write_partition_block(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int err;
	u32 offset = PIB_EMMC_ADDR;
	u8 mbr[512];
	u8 emmc_existing_partition[512];
	struct mmc *boot_dev = NULL;

	(void) cmdtp;  /* Parameter not used! */
	(void) flag; /* Parameter not used! */
	(void) argc; /* Parameter not used! */
	(void) argv; /* Parameter not used! */

	memset(mbr, 0, 0x1be);
	if (u8500_is_earlydrop())
		memcpy(mbr + 0x1be, partitions_ed, sizeof(partitions_ed));
	else
		memcpy(mbr + 0x1be, partitions_v1, sizeof(partitions_v1));

	/* magic */
	mbr[0x1fe] = 0x55;
	mbr[0x1ff] = 0xAA;

	printf("Writing partition block (if needed)...\n");

	boot_dev = find_mmc_device(CONFIG_EMMC_DEV_NUM);
	if (!boot_dev) {
		printf(" Error: eMMC device not found\n");
		return 1;
	}

	err =  boot_dev->block_dev.block_read(CONFIG_EMMC_DEV_NUM,
					offset,
					1,
					(void *) emmc_existing_partition);
	if (err != 1)	{
		printf(" Error: eMMC read failed\n");
		return 1;
	}

	if (memcmp((void *)emmc_existing_partition, (void *)mbr, 512)) {
		err = boot_dev->block_dev.block_write(CONFIG_EMMC_DEV_NUM,
						offset,
						1,
						(void *) mbr);
		if (err != 1) {
			printf(" Error: eMMC write failed\n");
			return 1;
		}
	}
	printf(" eMMC partition block exists now\n");
	return 0;
}

U_BOOT_CMD(
	write_partition_block, 1, 0, write_partition_block,
	"- write partition block on emmc device\n",
	NULL
);

/*
 * command line commands
 */
#ifdef CONFIG_CMD_FAT
static int mmc_read_cmd_file(cmd_tbl_t *cmdtp, int flag, int argc,
			     char *argv[])
{
	long sz;
	char mmc_cmdbuffer[1024];
	struct mmc *mmc_dev;
	(void) cmdtp;  /* Parameter not used! */
	(void) flag; /* Parameter not used! */
	(void) argc; /* Parameter not used! */
	(void) argv; /* Parameter not used! */

	mmc_dev = find_mmc_device(CONFIG_MMC_DEV_NUM);
	if (mmc_dev == NULL) {
		printf("mmc_read_cmd_file: find_mmc_device failed\n");
		return 1;
	}

	if (fat_register_device(&mmc_dev->block_dev, 1) != 0) {
		printf("mmc_read_cmd_file: fat_register_device failed\n");
		return 1;
	}

	sz = file_fat_read("/command.txt", &mmc_cmdbuffer,
			   sizeof(mmc_cmdbuffer) - 1);
	if (sz == -1) {
		printf("No command.txt found in the MMC/SD card\n");
		return 1;
	}

	mmc_cmdbuffer[sz] = '\0';
	setenv("bootcmd", mmc_cmdbuffer);
	return 0;
}

U_BOOT_CMD(
	mmc_read_cmd_file, 1, 0, mmc_read_cmd_file,
	"setup bootcmd env from command.txt on SD",
	NULL
);
#endif

/* ------------------------------- End of file ---------------------------- */

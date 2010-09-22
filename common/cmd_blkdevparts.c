/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Peter Nessrup <peter.nessrup@stericsson.com> for
 * ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */

/*
 * Support for setting blkdevparts environment variable, to be passed to
 * the Kernel on the kernel command line
 */
#include <common.h>
#include <part.h>
#include <malloc.h>

#if !defined(CONFIG_TOC_PARTITION)
#error TOC partition support must be selected
#endif

#define MMC_BLKDEV_ID			"mmcblk"
#define MAX_BLKDEVPARTS_MAXLEN		256
#define MAX_PARTITION_MAXLEN		20

int do_blkdevparts(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	disk_partition_t part_info;
	block_dev_desc_t *dev_desc;
	char *ep;
	int part = 1;	/* Start with partition 1 */
	char *blkdevparts;
	char blkdev_buf[MAX_BLKDEVPARTS_MAXLEN];
	char part_buf[MAX_PARTITION_MAXLEN];
	int dev;
	int m, n;
	const char *env_p;

	debug("do_blkdevparts\n");
	memset(blkdev_buf, 0, sizeof(blkdev_buf));

	if (argc < 3) {
		cmd_usage(cmdtp);
		return 1;
	}

	dev = (int) simple_strtoul(argv[2], &ep, 16);
	dev_desc = get_dev(argv[1], dev);

	if (dev_desc == NULL) {
		printf("Invalid device\n");
		return 1;
	}

	if (dev_desc->part_type != PART_TYPE_TOC) {
		printf("Device is not TOC partitioned!\n");
		return 1;
	}

	/* Only support sd/mmc interface */
	switch (dev_desc->if_type) {
	case IF_TYPE_MMC:
	case IF_TYPE_SD:
		n = sprintf(blkdev_buf, "%s", MMC_BLKDEV_ID);
		break;
	default:
		printf("Interface is not supported: %s\n", argv[1]);
		return 1;
	}

	/* Add the device */
	n += sprintf(&blkdev_buf[n], "%d:", dev);

	while (get_partition_info(dev_desc, part, &part_info) == 0) {
		memset(part_buf, 0, sizeof(part_buf));
		m = 0;
		/*
		 * Build up the bootargs environment
		 * variable with blkdev info
		 */
		debug("Partition found - part: %d!\n", part);
		/* Partition found */
		if (part > 1)
			/* Don't add a comma before the first partition */
			m = sprintf(part_buf, "%s", ",");

		sprintf(&part_buf[m], "%lu@%lu",
			part_info.size, part_info.start);

		/* Append the partition to the blkdev string */
		if ((n + sizeof(part_buf)) <= sizeof(blkdev_buf))
			n += sprintf(&blkdev_buf[n], "%s", part_buf);
		else {
			printf("No space to add partition to blkdevparts\n");
			return 1;
		}
		part++;
	};
	debug("No more partitions\n");

	/* Check if blkdevparts env var already existing */
	env_p = getenv("blkdevparts");
	if (env_p) {
		debug("Appending to existing blkdevparts env var\n");
		/* Allocate a buffer incl the added ;*/
		blkdevparts = malloc(strlen(env_p) + strlen(blkdev_buf) + 1);
		sprintf(blkdevparts, "%s;%s", env_p, blkdev_buf);
		setenv("blkdevparts", blkdevparts);
		/* free up the buffer */
		free(blkdevparts);
	} else
		setenv("blkdevparts", blkdev_buf);

	return 0;
}

U_BOOT_CMD(
	blkdevparts,	3,	0,	do_blkdevparts,
	"assembles partition info for kernel cmd line",
	"<interface> <dev>\n"
	"    - assembles the blkdevparts env var with partition information\n"
	"      for kernel cmd line, read from the <dev> TOC on <interface>"
);

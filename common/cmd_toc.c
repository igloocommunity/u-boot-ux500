/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Peter Nessrup <peter.nessrup@stericsson.com> for
 * ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */

/*
 * Support for printing the TOC and sub TOC structures
 *
 * Support also for reading data pointed to by a TOC entry, into RAM,
 * at the destintion address specified in the TOC entry
 */
#include <common.h>
#include <part.h>

#if !defined(CONFIG_TOC_PARTITION)
#error TOC partition support must be selected
#endif

int do_toc_print (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	block_dev_desc_t *dev_desc;
	char *ep;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	dev_desc = get_dev(argv[1], (int) simple_strtoul(argv[2], &ep, 16));

	if (dev_desc == NULL) {
		printf("Invalid device\n");
		return 1;
	}

	if (dev_desc->part_type == PART_TYPE_TOC)
		print_part(dev_desc);
	else
		printf("Device is not TOC partitioned!\n");

	return 0;
}


U_BOOT_CMD(
	tocprint,	3,	1,	do_toc_print,
	"prints the TOC and sub TOC contents",
	"<interface> <dev>\n"
	"    - print TOC and sub TOC from 'dev' on 'interface'"
);


int do_toc_entry_load(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	u32 offset;
	u32 size;
	u32 addr;
	char loadaddr[16];
	char size_buf[16];
	char offset_buf[16];
	block_dev_desc_t *dev_desc;
	int blks;
	char *ep;

	if (argc < 3) {
		cmd_usage(cmdtp);
		return 1;
	}

	dev_desc = get_dev(argv[1], (int) simple_strtoul(argv[2], &ep, 16));

	if (dev_desc == NULL) {
		printf("Invalid device\n");
		return 1;
	}

	if (dev_desc->part_type != PART_TYPE_TOC) {
		printf("Device is not TOC partitioned!\n");
		return 1;
	}

	if (get_entry_info_toc(dev_desc,
			       argv[3],
			       &offset,
			       &size,
			       &addr) == 0) {

		blks = (size / dev_desc->blksz) +
		       (size % dev_desc->blksz ? 1 : 0);

		if (dev_desc->block_read(dev_desc->dev,
					 offset / dev_desc->blksz,
					 blks,
					 (ulong *) addr) != blks) {
			printf("Unable to read from block device\n");
			return 1;
		}

		sprintf(loadaddr, "0x%x", addr);
		setenv("loadaddr", loadaddr);

		sprintf(size_buf, "0x%x", size);
		setenv("tocentrysize", size_buf);

		sprintf(offset_buf, "0x%x", offset);
		setenv("tocentryoffset", offset_buf);
		return 0;
	} else
			printf("Failed to get TOC entry!\n");

	return 1;
}

U_BOOT_CMD(
	tocload,	4,	0,	do_toc_entry_load,
	"load from the TOC entry to memory",
	"<interface> <dev> <entry ID>\n"
	"    - load the data pointed to by the <entry ID> to the RAM address\n"
	"      specified in the TOC entry loadaddr field, from <dev> on\n"
	"      <interface>"
);

/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Mikael Larsson <mikael.xt.larsson@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#include <common.h>
#include <malloc.h>
#include <part.h>
#include <mmc.h>
#include <asm/arch/ab8500.h>
#include "itp.h"
#include "cspsa_fp.h"


static int itp_load_toc_entry(block_dev_desc_t *block_dev,
			      const char *partname,
			      u32 *loadaddress)
{
	u32 n;
	u32 offset;
	u32 size;

	debug("itp_load_toc_entry: Loading %s\n", partname);

	if (get_entry_info_toc(block_dev, partname, &offset,
			       &size, loadaddress)) {
		printf("itp_load_toc_entry: get_entry_info_toc failed\n");
		return 1;
	}

	size = (size / block_dev->blksz) +
	       ((size % block_dev->blksz) ? 1 : 0);

	n = block_dev->block_read(block_dev->dev,
				  offset / block_dev->blksz,
				  size,
				  loadaddress);

	if (n != size) {
		printf("itp_load_toc_entry: Failed to load %s!\n", partname);
		return 1;
	}

	return 0;
}

/*
 * itp_load_itp - Loads itp depending on config.
 * If itp is loaded ok it will be executed and u-boot execution will stop
 */

int itp_load_itp(block_dev_desc_t *block_dev)
{
	u32 cspsa_key;
	void (*loadaddress)(void) = NULL;

	debug("\nitp_load_itp\n");

	if (cspsa_fp_read(block_dev,
			  ITP_CSPSA_KEY,
			  &cspsa_key)) {
		printf("itp_load_itp: cspsa_fp_read failed\n");
		return 1;
	}

	if (cspsa_key & ITP_LOAD_ITP) {
		if (itp_load_toc_entry(block_dev,
				       ITP_TOC_ITP_NAME,
				       (u32 *)loadaddress)) {
			printf("itp_load_itp: itp_load_partition failed\n");
			return 1;
		}
		loadaddress(); /* U-boot execution will end here*/

		printf("itp_load_itp: itp execution failed\n");
		return 1;
	}

	return 0;
}

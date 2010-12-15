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
#include <asm/arch/itp.h>
#include <asm/arch/cspsa_fp.h>
#include <asm/arch/sec_bridge.h>

#define IPL_ITEM_ID			0x02

static u32 cspsa_key;



static int itp_load_ipl(block_dev_desc_t *block_dev)
{
	u32 offset;
	u32 size;
	u32 loadaddr;
	u32 returnvalue;
	int ab8500_cutid;

	debug("itp_load_ipl\n");

	/* Check if IPL partition is present */
	if (get_entry_info_toc(block_dev, ITP_TOC_IPL_NAME, &offset,
			       &size, &loadaddr)) {
		printf("itp_load_ipl: ipl toc entry not present\n");
		return 1;
	}

	/* Get CutID */
	ab8500_cutid = ab8500_read(AB8500_MISC, AB8500_REV_REG);

	returnvalue = sec_bridge_call_secure_service((u32)ISSWAPI_SECURE_LOAD,
						SEC_ROM_FORCE_CLEAN_MASK,
						IPL_ITEM_ID,
						ab8500_cutid);
	if (returnvalue != SEC_ROM_RET_OK) {
		printf("itp_load_ipl: ISSWAPI_SECURE_LOAD: %d\n",
			returnvalue);
		return 1;
	}

	return 0;
}

static int itp_load_toc_entry(block_dev_desc_t *block_dev,
			      const char *partname,
			      int verify_signature,
			      u32 *loadaddress)
{
	u32 n;
	u32 offset;
	u32 size;
#if defined(CONFIG_SECURE_KERNEL_BOOT)
	u32 real_loadaddr = 0;
	u32 size_in_bytes = 0;
#endif

	debug("itp_load_toc_entry: Loading %s\n", partname);

	if (get_entry_info_toc(block_dev, partname, &offset,
			       &size, loadaddress)) {
		printf("itp_load_toc_entry: %s not present\n", partname);
		return 1;
	}

#if defined(CONFIG_SECURE_KERNEL_BOOT)
	if (verify_signature) {
		size_in_bytes = size;
		real_loadaddr = *loadaddress;
		/*
		 * We might need an offset, since ISSW doesn't support
		 * address 0.
		 */
		if (*loadaddress == 0)
			*loadaddress = *loadaddress + block_dev->blksz;
	}
#else
	if (verify_signature) {
		debug("itp_load_toc_entry: secure boot disabled so verify signature has no effect\n");
	}
#endif

	size = (size / block_dev->blksz) +
	       ((size % block_dev->blksz) ? 1 : 0);

	n = block_dev->block_read(block_dev->dev,
				  offset / block_dev->blksz,
				  size,
				  (void *) *loadaddress);

	if (n != size) {
		printf("itp_load_toc_entry: Failed to load %s!\n", partname);
		return 1;
	}

#if defined(CONFIG_SECURE_KERNEL_BOOT)
	if (verify_signature) {
		debug("itp_load_toc_entry: Verifying image...\n");

		if (sec_bridge_verify_itp_image(loadaddress)) {
			printf("itp_load_toc_entry: Failed to verify image %s!\n", partname);
			return 1;
		}

		if (real_loadaddr != *loadaddress) {
			/*
			 * Loadaddr is moved, need to move it back to ensure
			 * binary is not put out of order...
			 */
			memmove((void *)(real_loadaddr), (void*)*loadaddress, size_in_bytes);
			*loadaddress = real_loadaddr;
		}
	}

#endif

	return 0;
}

int itp_read_config(block_dev_desc_t *block_dev)
{
	if (cspsa_fp_read(block_dev,
			  ITP_CSPSA_KEY,
			  &cspsa_key)) {
		printf("itp_read_config: config not present. "
			   "Using default values\n");
		cspsa_key = (ITP_LOAD_MODEM | ITP_LOAD_KERNEL);
	}
	return 0;
}

int itp_is_itp_in_config(void)
{
	return cspsa_key & ITP_LOAD_ITP;
}

/*
 * itp_load_itp_and_modem - Loads itp and modem depending on config.
 * If itp is loaded ok it will be executed and u-boot execution will stop
 */
int itp_load_itp_and_modem(block_dev_desc_t *block_dev)
{
	int retval = 0;
	u32 loadaddress;

	debug("\nitp_load_itp_and_modem\n");

	if (sec_bridge_init_bridge()) {
		retval = 1;
		goto exit;
	}

	if (cspsa_key & ITP_LOAD_MODEM) {
		if (itp_load_toc_entry(block_dev,
				       ITP_TOC_MODEM_NAME,
				       0, /* verify_signature false */
				       &loadaddress)) {
			retval = 1;
			goto exit;
		}

		if (itp_load_ipl(block_dev)) {
			retval = 1;
			goto exit;
		}
	}

	if (cspsa_key & ITP_LOAD_ITP) {
		if (itp_load_toc_entry(block_dev,
				       ITP_TOC_ITP_NAME,
				       1, /* verify_signature true */
				       &loadaddress)) {
			retval = 1;
			goto exit;
		}
	}

exit:
	/* Always Flush */
	sec_bridge_flush_issw();

	if ((cspsa_key & ITP_LOAD_ITP) && !retval)
		/* U-boot execution will end here */
		((void (*)(void))loadaddress)();

	/* Return on error */
	return retval;
}

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

#define SEC_ROM_FORCE_CLEAN_MASK	0x0020
#define SEC_ROM_RET_OK			0x01
#define ISSWAPI_SECURE_LOAD		0x10000002
#define ISSWAPI_FLUSH_BOOT_CODE		0x11000003
#define IPL_ITEM_ID			0x02

typedef u32 (*boot_rom_bridge_func_t)(const u32 , const u32, const va_list);

static boot_rom_bridge_func_t hw_sec_rom_pub_bridge;

struct sec_rom_cut_desc {
	u32 cutid_addr;
	u32 cutid;
	u32 bridge_func;
};

static const struct sec_rom_cut_desc cuttable[] = {
	{ 0x9001DBF4, 0x008500B0, 0x90017300 },
	{ 0x9001FFF4, 0x008500A1, 0x90018300 },
	{ 0x9001FFF4, 0x005500A0, 0x90018300 },
	{ 0x9001FFF4, 0x008500A0, 0x90018300 },
};

static u32 cspsa_key;

static u32 itp_call_secure_service(const u32 serviceid,
				   const u32 secureconfig,
				   ...)
{
	va_list ap;
	u32 returnvalue;

	va_start(ap, secureconfig);

	returnvalue = hw_sec_rom_pub_bridge(serviceid,
					    secureconfig,
					    ap);

	va_end(ap);
	return returnvalue;
}

static int itp_init_bridge(void)
{
	u8 cutnb = 0;

	hw_sec_rom_pub_bridge = NULL;

	while ((cutnb < ARRAY_SIZE(cuttable)) &&
	       (cuttable[cutnb].cutid != *(u32 *)(cuttable[cutnb].cutid_addr)))
			cutnb++;

	if (cutnb < ARRAY_SIZE(cuttable)) {
		hw_sec_rom_pub_bridge =
			(boot_rom_bridge_func_t)cuttable[cutnb].bridge_func;
		return 0;
	}

	printf("itp_init_bridge: cutid not found\n");
	return 1;
}

static int itp_flush_issw(void)
{
	u32 ret;

	ret = itp_call_secure_service(ISSWAPI_FLUSH_BOOT_CODE,
				      SEC_ROM_FORCE_CLEAN_MASK,
				      0,
				      0);

	if (ret != SEC_ROM_RET_OK) {
		printf("itp_flush_issw: ISSWAPI_FLUSH_BOOT_CODE: %d\n",
			ret);
		return 1;
	}

	return 0;
}

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

	returnvalue = itp_call_secure_service((u32)ISSWAPI_SECURE_LOAD,
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
			      u32 *loadaddress)
{
	u32 n;
	u32 offset;
	u32 size;

	debug("itp_load_toc_entry: Loading %s\n", partname);

	if (get_entry_info_toc(block_dev, partname, &offset,
			       &size, loadaddress)) {
		printf("itp_load_toc_entry: %s not present\n", partname);
		return 1;
	}

	size = (size / block_dev->blksz) +
	       ((size % block_dev->blksz) ? 1 : 0);

	n = block_dev->block_read(block_dev->dev,
				  offset / block_dev->blksz,
				  size,
				  *loadaddress);

	if (n != size) {
		printf("itp_load_toc_entry: Failed to load %s!\n", partname);
		return 1;
	}

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
	void (*loadaddress)(void) = NULL;

	debug("\nitp_load_itp_and_modem\n");

	if (itp_init_bridge()) {
		retval = 1;
		goto exit;
	}

	if (cspsa_key & ITP_LOAD_MODEM) {
		if (itp_load_toc_entry(block_dev,
				       ITP_TOC_MODEM_NAME,
				       (u32 *)loadaddress)) {
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
				       (u32 *)loadaddress)) {
			retval = 1;
			goto exit;
		}
	}

exit:
	/* Always Flush */
	if (hw_sec_rom_pub_bridge != NULL)
		itp_flush_issw();

	if ((cspsa_key & ITP_LOAD_ITP) && !retval)
		loadaddress(); /* U-boot execution will end here*/

	/* Return on error */
	return retval;
}

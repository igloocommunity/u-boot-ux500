/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Mikael Larsson <mikael.xt.larsson@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <common.h>
#include <asm/arch/sec_bridge.h>

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

int sec_bridge_init_bridge(void)
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

	printf("sec_bridge_init_bridge: cutid not found\n");
	return 1;
}

u32 sec_bridge_call_secure_service(const u32 serviceid,
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

int sec_bridge_flush_issw(void)
{
	u32 ret;

	if (hw_sec_rom_pub_bridge != NULL) {

		ret = sec_bridge_call_secure_service(ISSWAPI_FLUSH_BOOT_CODE,
						SEC_ROM_FORCE_CLEAN_MASK,
						0,
						0);

		if (ret != SEC_ROM_RET_OK) {
			printf("sec_bridge_flush_issw: ISSWAPI_FLUSH_BOOT_CODE: %d\n",
				ret);
			return 1;
		}
	}
	return 0;
}

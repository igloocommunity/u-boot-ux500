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
	{ 0x9001DBF4, 0x008500B2, 0x90017300 },
	{ 0x9001DBF4, 0x008500B1, 0x90017300 },
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

	printf("sec_bridge: cutid not found\n");
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
			printf("sec_bridge: ISSWAPI_FLUSH_BOOT_CODE: %d\n",
				ret);
			return 1;
		}
	}
	return 0;
}

/*
 * All this signed header verification code is put here to reuse the static
 * functions defined in this file to call secure world.
 *
 * We are planning to export generic code verifcation to u-boot via another
 * module so the generic code below can be removed at that stage.
 */

#if defined(CONFIG_SECURE_KERNEL_BOOT)

/* Stuff copied from isswapi_types.h */
enum issw_payload_type {
	ISSW_PL_TYPE_TAPP = 0,
	ISSW_PL_TYPE_PRCMU,
	ISSW_PL_TYPE_MEMINIT,
	ISSW_PL_TYPE_X_LOADER,
	ISSW_PL_TYPE_OS_LOADER,
	ISSW_PL_TYPE_APE_NW_CODE,
	ISSW_PL_TYPE_FC_LOADER,
	ISSW_PL_TYPE_MODEM_CODE,
	ISSW_PL_TYPE_FOTA,
	ISSW_PL_TYPE_DNTCERT,
	ISSW_PL_TYPE_AUTHCERT,
	ISSW_PL_TYPE_IPL,
	ISSW_PL_TYPE_FLASH_ARCHIVE,
	ISSW_PL_TYPE_ITP,
	ISSW_PL_TYPE_AUTH_CHALLENGE = -1 /* 0xffffffff */
};


typedef struct issw_signed_header {
	u32    magic;
	u16    size_of_signed_header;
	u16    size_of_signature;
	u32    sign_hash_type; /* see t_hash_type */
	u32    signature_type; /* see t_signature_type */
	u32    hash_type;      /* see t_hash_type */
	u32    payload_type;   /* see enum issw_payload_type */
	u32    flags;	  /* reserved */
	u32    size_of_payload;
	u32    sw_vers_nbr;
	u32    load_address;
	u32    startup_address;
	u32    spare;	  /* reserved */
#if 0
	/* Pseudo code visualize layout of signed header */
	u8     hash[get_hash_length(this.hash_type)];
	u8     signature[size_of_signature];
#endif
} issw_signed_header_t;

#define ISSW_SIGNED_HEADER_MAGIC  0x53484452

#define ISSW_SIGNED_HEADER_HASH(hdr) \
	((u8 *)((issw_signed_header_t *)(hdr) + 1))

#define ISSW_SIGNED_HEADER_HASH_SIZE(hdr) \
	(((issw_signed_header_t *)(hdr))->size_of_signed_header - \
	    ((issw_signed_header_t *)(hdr))->size_of_signature - \
		sizeof(issw_signed_header_t))

#define ISSW_SIGNED_HEADER_SIGNATURE(hdr) \
	(ISSW_SIGNED_HEADER_HASH(hdr) + ISSW_SIGNED_HEADER_HASH_SIZE(hdr))

#define ISSW_SIGNED_HEADER_PAYLOAD(hdr) \
	((u8 *)(hdr) + \
		((issw_signed_header_t *)(hdr))->size_of_signed_header)

static int sec_bridge_verify_signed_header(issw_signed_header_t *hdr,
					   enum issw_payload_type pt)
{
	u32 ret;

	ret = sec_bridge_call_secure_service(ISSWAPI_VERIFY_SIGNED_HEADER,
					     SEC_ROM_FORCE_CLEAN_MASK, hdr, (u32)pt);
	if (ret != SEC_ROM_RET_OK) {
		printf("sec_bridge: "
		       "ISSWAPI_VERIFY_SIGNED_HEADER: %d\n", ret);
		return 1;
	}
	return 0;
}

static int sec_bridge_verify_hash(u8 *hash, u32 hash_size, u8 *payload,
			u32 payload_size, u32 hash_type)
{
	u32 ret;

	ret = sec_bridge_call_secure_service(ISSWAPI_VERIFY_HASH,
				      SEC_ROM_FORCE_CLEAN_MASK,
				      hash, hash_size, payload, payload_size,
				      hash_type);
	if (ret != SEC_ROM_RET_OK) {
		printf("sec_bridge: "
		       "ISSWAPI_VERIFY_HASH: %d\n", ret);
		return 1;
	}
	return 0;
}

static int sec_bridge_verify_image(u32 *img_addr,
				   enum issw_payload_type payload_type)
{
	issw_signed_header_t *hdr = (issw_signed_header_t *) *img_addr;

	debug("sec_bridge_verify_image(img_addr->0x%08x, payload_type:%d)\n", *img_addr, payload_type);

	if (*img_addr  == 0)
		return 1;

	if (sec_bridge_verify_signed_header(hdr, payload_type)) 
		return 1;

	/*
	 * Using a secure service for this since sha256 in u-boot
	 * was incedible slow.
	 */
	if (sec_bridge_verify_hash(ISSW_SIGNED_HEADER_HASH(hdr),
				   ISSW_SIGNED_HEADER_HASH_SIZE(hdr),
				   ISSW_SIGNED_HEADER_PAYLOAD(hdr),
				   hdr->size_of_payload, hdr->hash_type))
		return 1;

	*img_addr = (ulong)ISSW_SIGNED_HEADER_PAYLOAD(hdr);
	debug("sec_bridge: Changed img_addr->0x%08x\n", *img_addr);
	return 0;
}

int sec_bridge_verify_kernel_image(u32 *img_addr) {
	return sec_bridge_verify_image(img_addr, ISSW_PL_TYPE_APE_NW_CODE);
}

int sec_bridge_verify_itp_image(u32 *img_addr) {
	return sec_bridge_verify_image(img_addr, ISSW_PL_TYPE_ITP);
}

#endif /* CONFIG_SECURE_KERNEL_BOOT */

/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Mikael Larsson <mikael.xt.larsson@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef _SEC_BRIDGE_H
#define _SEC_BRIDGE_H

#include <common.h>

#define SEC_ROM_FORCE_CLEAN_MASK	0x0020
#define SEC_ROM_RET_OK			0x01

#define ISSWAPI_SECURE_LOAD		0x10000002
#define ISSWAPI_FLUSH_BOOT_CODE		0x11000003

int sec_bridge_init_bridge(void);
u32 sec_bridge_call_secure_service(const u32 serviceid,
				   const u32 secureconfig, ...);
int sec_bridge_flush_issw(void);
#endif

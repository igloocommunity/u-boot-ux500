/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Joakim Axelsson <joakim.axelsson at stericsson.com>
 *  for ST-Ericsson
 *
 * Origin: Code split from board/st/u8500/u8500.c
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __DB8500_CPU_H__
#define __DB8500_CPU_H__

#include <asm/io.h>
#include <asm/arch/hardware.h>

#define CPUID_DB8500ED		0x410fc090
#define CPUID_DB8500V1		0x411fc091
#define CPUID_DB8500V2		0x412fc091

#define ASICID_DB8500V11	0x008500A1

/*
 * Keep these CPU identity functions inline here because they are short
 * and used by many. Will make for fast optimized compiled code.
 */

static inline unsigned int read_cpuid(void)
{
	unsigned int val;

	/* Main ID register (MIDR) */
	asm("mrc        p15, 0, %0, c0, c0, 0"
	   : "=r" (val)
	   :
	   : "cc");

	return val;
}

static inline int u8500_is_earlydrop(void)
{
	return read_cpuid() == CPUID_DB8500ED;
}

static inline int cpu_is_u8500v1(void)
{
	return read_cpuid() == CPUID_DB8500V1;
}

static inline int cpu_is_u8500v2(void)
{
	return read_cpuid() == CPUID_DB8500V2;
}

static inline unsigned int read_asicid(void)
{
	unsigned int *address;

	if (u8500_is_earlydrop() || cpu_is_u8500v1())
		address = (void *) U8500_ASIC_ID_LOC_ED_V1;
	else
		address = (void *) U8500_ASIC_ID_LOC_V2;

	return readl(address);
}

static inline int cpu_is_u8500v11(void)
{
	return read_asicid() == ASICID_DB8500V11;
}


#endif /* __DB8500_CPU_H__ */


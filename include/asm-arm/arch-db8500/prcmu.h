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

#ifndef __PRCMU_H__
#define __PRCMU_H__

#include <asm/arch/hardware.h>

/* Power, Reset, Clock Management Unit */
/*
 * SVA: Smart Video Accelerator
 * SIA: Smart Imaging Accelerator
 * SGA: Smart Graphic accelerator
 * B2R2: Graphic blitter
 */
#define PRCMU_BASE			U8500_PRCMU_BASE
#define PRCM_ARMCLKFIX_MGT_REG	(PRCMU_BASE + 0x000)
#define PRCM_ACLK_MGT_REG		(PRCMU_BASE + 0x004)
#define PRCM_SVAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x008)
#define PRCM_SIAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x00C)
#define PRCM_SAAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x010)
#define PRCM_SGACLK_MGT_REG		(PRCMU_BASE + 0x014)
#define PRCM_UARTCLK_MGT_REG		(PRCMU_BASE + 0x018)
#define PRCM_MSPCLK_MGT_REG		(PRCMU_BASE + 0x01C)
#define PRCM_I2CCLK_MGT_REG		(PRCMU_BASE + 0x020)
#define PRCM_SDMMCCLK_MGT_REG	(PRCMU_BASE + 0x024)
#define PRCM_SLIMCLK_MGT_REG		(PRCMU_BASE + 0x028)
#define PRCM_PER1CLK_MGT_REG		(PRCMU_BASE + 0x02C)
#define PRCM_PER2CLK_MGT_REG		(PRCMU_BASE + 0x030)
#define PRCM_PER3CLK_MGT_REG		(PRCMU_BASE + 0x034)
#define PRCM_PER5CLK_MGT_REG		(PRCMU_BASE + 0x038)
#define PRCM_PER6CLK_MGT_REG		(PRCMU_BASE + 0x03C)
#define PRCM_PER7CLK_MGT_REG		(PRCMU_BASE + 0x040)
#define PRCM_DMACLK_MGT_REG		(PRCMU_BASE + 0x074)
#define PRCM_B2R2CLK_MGT_REG		(PRCMU_BASE + 0x078)

#define PRCM_PLLSOC0_FREQ_REG		(PRCMU_BASE + 0x080)
#define PRCM_PLLSOC1_FREQ_REG		(PRCMU_BASE + 0x084)
#define PRCM_PLLARM_FREQ_REG		(PRCMU_BASE + 0x088)
#define PRCM_PLLDDR_FREQ_REG		(PRCMU_BASE + 0x08C)
#define PRCM_ARM_CHGCLKREQ_REG	(PRCMU_BASE + 0x114)

#define PRCM_TCR			(PRCMU_BASE + 0x1C8)

/* Init PRCMU relatated things in db8500 SoC platform */
void db8500_prcmu_init(void);

#endif /* __PRCMU_H__ */


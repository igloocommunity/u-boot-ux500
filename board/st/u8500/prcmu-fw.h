/*
 * Copyright (C) 2009 ST-Ericsson SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * Copied from the Linux version:
 * Author: Kumar Sanghvi <kumar.sanghvi@stericsson.com>
 */

#ifndef __MACH_PRCMU_FW_V1_H
#define __MACH_PRCMU_FW_V1_H

#include "prcmu-fw-defs_v1.h"

#define _PRCMU_TCDM_BASE    U8500_PRCMU_TCDM_BASE
#define PRCM_BOOT_STATUS    (_PRCMU_TCDM_BASE + 0xFFF)
#define PRCM_ROMCODE_A2P    (_PRCMU_TCDM_BASE + 0xFFE)
#define PRCM_ROMCODE_P2A    (_PRCMU_TCDM_BASE + 0xFFD)
#define PRCM_XP70_CUR_PWR_STATE (_PRCMU_TCDM_BASE + 0xFFC)      /* 4 BYTES */


#define _PRCM_MBOX_HEADER	    	(_PRCMU_TCDM_BASE + 0xFE8)/*16 bytes*/
#define PRCM_MBOX_HEADER_REQ_MB0	(_PRCM_MBOX_HEADER + 0x0)
#define PRCM_MBOX_HEADER_REQ_MB1	(_PRCM_MBOX_HEADER + 0x1)
#define PRCM_MBOX_HEADER_REQ_MB2	(_PRCM_MBOX_HEADER + 0x2)
#define PRCM_MBOX_HEADER_REQ_MB3	(_PRCM_MBOX_HEADER + 0x3)
#define PRCM_MBOX_HEADER_REQ_MB4	(_PRCM_MBOX_HEADER + 0x4)
#define PRCM_MBOX_HEADER_REQ_MB5	(_PRCM_MBOX_HEADER + 0x5)
#define PRCM_MBOX_HEADER_REQ_MB6	(_PRCM_MBOX_HEADER + 0x6)
#define PRCM_MBOX_HEADER_REQ_MB7	(_PRCM_MBOX_HEADER + 0x7)
#define PRCM_MBOX_HEADER_ACK_MB0	(_PRCM_MBOX_HEADER + 0x8)
#define PRCM_MBOX_HEADER_ACK_MB1	(_PRCM_MBOX_HEADER + 0x9)
#define PRCM_MBOX_HEADER_ACK_MB2	(_PRCM_MBOX_HEADER + 0xA)
#define PRCM_MBOX_HEADER_ACK_MB3	(_PRCM_MBOX_HEADER + 0xB)
#define PRCM_MBOX_HEADER_ACK_MB4	(_PRCM_MBOX_HEADER + 0xC)
#define PRCM_MBOX_HEADER_ACK_MB5	(_PRCM_MBOX_HEADER + 0xD)
#define PRCM_MBOX_HEADER_ACK_MB6	(_PRCM_MBOX_HEADER + 0xE)
#define PRCM_MBOX_HEADER_ACK_MB7	(_PRCM_MBOX_HEADER + 0xF)

/*Req Mailboxes */
#define PRCM_REQ_MB0        (_PRCMU_TCDM_BASE + 0xFDC)    /* 12 bytes  */
#define PRCM_REQ_MB1        (_PRCMU_TCDM_BASE + 0xFD0)    /* 12 bytes  */
#define PRCM_REQ_MB2        (_PRCMU_TCDM_BASE + 0xFC0)    /* 16 bytes  */
#define PRCM_REQ_MB3        (_PRCMU_TCDM_BASE + 0xE4C)    /* 372 bytes  */
#define PRCM_REQ_MB4        (_PRCMU_TCDM_BASE + 0xE48)    /* 4 bytes  */
#define PRCM_REQ_MB5        (_PRCMU_TCDM_BASE + 0xE44)    /* 4 bytes  */
#define PRCM_REQ_MB6        (_PRCMU_TCDM_BASE + 0xE40)    /* 4 bytes  */
#define PRCM_REQ_MB7        (_PRCMU_TCDM_BASE + 0xE3C)    /* 4 bytes  */

/*Ack Mailboxes */
#define PRCM_ACK_MB0        (_PRCMU_TCDM_BASE + 0xE08)    /* 52 bytes  */
#define PRCM_ACK_MB1        (_PRCMU_TCDM_BASE + 0xE04)    /* 4 bytes */
#define PRCM_ACK_MB2        (_PRCMU_TCDM_BASE + 0xE00)    /* 4 bytes */
#define PRCM_ACK_MB3        (_PRCMU_TCDM_BASE + 0xDFC)    /* 4 bytes */
#define PRCM_ACK_MB4        (_PRCMU_TCDM_BASE + 0xDF8)    /* 4 bytes */
#define PRCM_ACK_MB5        (_PRCMU_TCDM_BASE + 0xDF4)    /* 4 bytes */
#define PRCM_ACK_MB6        (_PRCMU_TCDM_BASE + 0xDF0)    /* 4 bytes */
#define PRCM_ACK_MB7	    (_PRCMU_TCDM_BASE + 0xDEC)    /* 4 bytes  */

/* Mailbox 0 REQs */
#define PRCM_REQ_MB0_PWRSTTRH		(PRCM_REQ_MB0 + 0x0)
#define PRCM_REQ_MB0_PWRSTTRH_APPWRST	(PRCM_REQ_MB0 + 0x0)
#define PRCM_REQ_MB0_PWRSTTRH_APPLLST	(PRCM_REQ_MB0 + 0x1)
#define PRCM_REQ_MB0_PWRSTTRH_ULPCLKST	(PRCM_REQ_MB0 + 0x2)
#define PRCM_REQ_MB0_PWRSTTRH_BYTEFILL	(PRCM_REQ_MB0 + 0x3)
#define PRCM_REQ_MB0_WKUP_8500	(PRCM_REQ_MB0 + 0x4)
#define PRCM_REQ_MB0_WKUP_4500	(PRCM_REQ_MB0 + 0x8)


/* Mailbox 0 ACKs */
#define PRCM_ACK_MB0_AP_PWRST_STATUS	(PRCM_ACK_MB0 + 0x0)
#define PRCM_ACK_MB0_PINGPONG_RDP	(PRCM_ACK_MB0 + 0x1)
#define PRCM_ACK_MB0_PINGPONG_WKUP_RST_0 (PRCM_ACK_MB0 + 0x2)
#define PRCM_ACK_MB0_PINGPONG_WKUP_RST_1 (PRCM_ACK_MB0 + 0x3)
#define PRCM_ACK_MB0_WK0_EVENT_8500     (_PRCMU_TCDM_BASE + 0xE0C)
#define PRCM_ACK_MB0_WK0_EVENT_4500     (_PRCMU_TCDM_BASE + 0xE10)
#define PRCM_ACK_MB0_WK1_EVENT_8500     (_PRCMU_TCDM_BASE + 0xE24)
#define PRCM_ACK_MB0_WK1_EVENT_4500     (_PRCMU_TCDM_BASE + 0xE28)
#define PRCM_ACK_MB0_EVENT_4500_NUMBERS	20


/* Mailbox 1 Requests */
#define PRCM_REQ_MB1_ARMOPP     (PRCM_REQ_MB1 + 0x0)
#define PRCM_REQ_MB1_APEOPP     (PRCM_REQ_MB1 + 0x1)
#define PRCM_REQ_MB1_BOOSTOPP   (PRCM_REQ_MB1 + 0x2)

/* Mailbox 1 ACKs */
#define PRCM_ACK_MB1_CURR_ARMOPP        (PRCM_ACK_MB1 + 0x0)
#define PRCM_ACK_MB1_CURR_APEOPP        (PRCM_ACK_MB1 + 0x1)
#define PRCM_ACK_MB1_CURR_BOOSTOPP      (PRCM_ACK_MB1 + 0x2)
#define PRCM_ACK_MB1_CURR_DVFS_STATUS   (PRCM_ACK_MB1 + 0x3)

/* Mailbox 2 REQs */
#define PRCM_REQ_MB2_DPS_SVAMMDSP        (PRCM_REQ_MB2 + 0x0)
#define PRCM_REQ_MB2_DPS_SVAPIPE         (PRCM_REQ_MB2 + 0x1)
#define PRCM_REQ_MB2_DPS_SIAMMDSP        (PRCM_REQ_MB2 + 0x2)
#define PRCM_REQ_MB2_DPS_SIAPIPE         (PRCM_REQ_MB2 + 0x3)
#define PRCM_REQ_MB2_DPS_SGA             (PRCM_REQ_MB2 + 0x4)
#define PRCM_REQ_MB2_DPS_B2R2MCDE        (PRCM_REQ_MB2 + 0x5)
#define PRCM_REQ_MB2_DPS_ESRAM12         (PRCM_REQ_MB2 + 0x6)
#define PRCM_REQ_MB2_DPS_ESRAM34         (PRCM_REQ_MB2 + 0x7)
#define PRCM_REQ_MB2_AUTOPWR_APSLEEP     (PRCM_REQ_MB2 + 0x8)
#define PRCM_REQ_MB2_AUTOPWR_APSLEEP_SIA_PWRON_ENABLE     (PRCM_REQ_MB2 + 0x9)
#define PRCM_REQ_MB2_AUTOPWR_APSLEEP_SVA_PWRON_ENABLE     (PRCM_REQ_MB2 + 0xA)
#define PRCM_REQ_MB2_AUTOPWR_APSLEEP_AUTO_PWRON_ENABLE     (PRCM_REQ_MB2 + 0xB)
#define PRCM_REQ_MB2_AUTOPWR_APIDLE     (PRCM_REQ_MB2 + 0xC)
#define PRCM_REQ_MB2_AUTOPWR_APIDLE_SIA_PWRON_ENABLE     (PRCM_REQ_MB2 + 0xD)
#define PRCM_REQ_MB2_AUTOPWR_APIDLE_SVA_PWRON_ENABLE     (PRCM_REQ_MB2 + 0xE)
#define PRCM_REQ_MB2_AUTOPWR_APIDLE_AUTO_PWRON_ENABLE     (PRCM_REQ_MB2 + 0xF)

/* Mailbox 2 ACKs */
#define PRCM_ACK_MB2_DPS_STATUS          (PRCM_ACK_MB2 + 0x0)

/* Mailbox 5 Requests */
#define PRCM_REQ_MB5_I2COPTYPE_REG	(PRCM_REQ_MB5 + 0x0)
#define PRCM_REQ_MB5_BIT_FIELDS		(PRCM_REQ_MB5 + 0x1)
#define PRCM_REQ_MB5_I2CSLAVE		(PRCM_REQ_MB5 + 0x2)
#define PRCM_REQ_MB5_I2CVAL		(PRCM_REQ_MB5 + 0x3)

/* Mailbox 5 ACKs */
#define PRCM_ACK_MB5_STATUS	(PRCM_ACK_MB5 + 0x1)
#define PRCM_ACK_MB5_SLAVE	(PRCM_ACK_MB5 + 0x2)
#define PRCM_ACK_MB5_VAL	(PRCM_ACK_MB5 + 0x3)

#define LOW_POWER_WAKEUP	1
#define EXE_WAKEUP		0

/* FIXME : Need to Cleanup Code */

#define PRCM_SVAMMDSPSTATUS PRCM_REQ_MB6
#define PRCM_SIAMMDSPSTATUS PRCM_REQ_MB7


#define PRCM_XP70_TRIG_IT10	(1 << 0)
#define PRCM_XP70_TRIG_IT11	(1 << 1)
#define PRCM_XP70_TRIG_IT12	(1 << 2)
#define PRCM_XP70_TRIG_IT14	(1 << 4)
#define PRCM_XP70_TRIG_IT17	(1 << 5)

enum mailbox_t {
	REQ_MB0 = 0,	/* Uses XP70_IT_EVENT_10 */
	REQ_MB1 = 1,	/* Uses XP70_IT_EVENT_11 */
	REQ_MB2 = 2,	/* Uses XP70_IT_EVENT_12 */
	REQ_MB5 = 5,	/* Uses XP70_IT_EVENT_17 */
};

/* Union declaration */



/* ARM to XP70 mailbox definition */
union req_mb0_t {
	struct {
		enum ap_pwrst_trans_t ap_pwrst_trans:8;
		enum intr_wakeup_t fifo_4500wu:8;
		enum ddr_pwrst_t ddr_pwrst:8;
		unsigned int unused:8;
	} req_field;
	unsigned int complete_field;
};

/* mailbox definition for ARM/APE operation */
union req_mb1_t {
	struct {
		enum arm_opp_t arm_opp:8;
		enum ape_opp_t ape_opp:8;
	} req_field;
	unsigned short complete_field;
};

#endif /* __MACH_PRCMU_FW_V1_H */

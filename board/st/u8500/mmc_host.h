/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Ulf Hansson <ulf.hansson@stericsson.com>
 * Author: Martin Lundholm <martin.xa.lundholm@stericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __MMC_NOMADIK_H__
#define __MMC_NOMADIK_H__

/* See SDI (SD card host interface) documentation in U8500 sw specification. */

#define COMMAND_REG_DELAY 	300
#define DATA_REG_DELAY 		10000
#define CLK_CHANGE_DELAY 	2000

#define MAX_ERROR_VALUE -65
#ifndef __ASSEMBLY__
enum mmc_result {
	/* MMC specific error defines */
	MMC_CMD_CRC_FAIL = (MAX_ERROR_VALUE - 1),/* Command response received
						    (but CRC check failed) */
	MMC_DATA_CRC_FAIL = (MAX_ERROR_VALUE - 2),/* Data bock sent/received
						     (CRC check Failed) */
	MMC_CMD_RSP_TIMEOUT = (MAX_ERROR_VALUE - 3),/*Command response timeout
								 */
	MMC_DATA_TIMEOUT = (MAX_ERROR_VALUE - 4),/* Data time out*/
	MMC_TX_UNDERRUN	= (MAX_ERROR_VALUE - 5),/* Transmit FIFO under-run */
	MMC_RX_OVERRUN = (MAX_ERROR_VALUE - 6),	/* Receive FIFO over-run */
	MMC_START_BIT_ERR = (MAX_ERROR_VALUE - 7),/* Start bit not detected on
					   all data signals in widE bus mode */
	MMC_CMD_OUT_OF_RANGE = (MAX_ERROR_VALUE - 8),/* CMD's argument was out
							of range.*/
	MMC_ADDR_MISALIGNED = (MAX_ERROR_VALUE - 9),/* Misaligned address */
	MMC_BLOCK_LEN_ERR = (MAX_ERROR_VALUE - 10),/* Transferred block length
			   is not allowed for the card or the number of
			   transferred bytes does not match the block length*/
	MMC_ERASE_SEQ_ERR = (MAX_ERROR_VALUE - 11),/* An error in the sequence
						of erase command occurs.*/
	MMC_BAD_ERASE_PARAM = (MAX_ERROR_VALUE - 12),/* An Invalid selection
							for erase groups */
	MMC_WRITE_PROT_VIOLATION = (MAX_ERROR_VALUE - 13),/* Attempt to program
						       a write protect block */
	MMC_LOCK_UNLOCK_FAILED = (MAX_ERROR_VALUE - 14),/* Sequence or password
			      error has been detected in unlock command or
			     if there was an attempt to access a locked card */
	MMC_COM_CRC_FAILED = (MAX_ERROR_VALUE - 15),/* CRC check of the
						 previous command failed*/
	MMC_ILLEGAL_CMD	= (MAX_ERROR_VALUE - 16),/* Command is not legal for
						    the card state */
	MMC_CARD_ECC_FAILED = (MAX_ERROR_VALUE - 17),/* Card internal ECC was
				    applied but failed to correct the data */
	MMC_CC_ERROR = (MAX_ERROR_VALUE - 18),/*Internal card controller error
					 */
	MMC_GENERAL_UNKNOWN_ERROR = (MAX_ERROR_VALUE - 19),/* General or
							Unknown error */
	MMC_STREAM_READ_UNDERRUN = (MAX_ERROR_VALUE - 20),/* The card could not
					  sustain data transfer in stream read
					  operation. */
	MMC_STREAM_WRITE_OVERRUN = (MAX_ERROR_VALUE - 21),/* The card could not
				    sustain data programming in stream mode */
	MMC_CID_CSD_OVERWRITE = (MAX_ERROR_VALUE - 22),	/* CID/CSD overwrite
								 error */
	MMC_WP_ERASE_SKIP = (MAX_ERROR_VALUE - 23),/* only partial address
						space was erased */
	MMC_CARD_ECC_DISABLED = (MAX_ERROR_VALUE - 24),/* Command has been
					 executed without using internal ECC */
	MMC_ERASE_RESET = (MAX_ERROR_VALUE - 25),/* Erase sequence was cleared
			     before executing because an out of erase sequence
					 command was received */
	MMC_AKE_SEQ_ERROR = (MAX_ERROR_VALUE - 26),/* Error in sequence of
						authentication. */
	MMC_INVALID_VOLTRANGE = (MAX_ERROR_VALUE - 27),
	MMC_ADDR_OUT_OF_RANGE = (MAX_ERROR_VALUE - 28),
	MMC_SWITCH_ERROR = (MAX_ERROR_VALUE - 29),
	MMC_SDIO_DISABLED = (MAX_ERROR_VALUE - 30),
	MMC_SDIO_FUNCTION_BUSY = (MAX_ERROR_VALUE - 31),
	MMC_SDIO_FUNCTION_FAILED = (MAX_ERROR_VALUE - 32),
	MMC_SDIO_UNKNOWN_FUNCTION = MAX_ERROR_VALUE,
	/* standard error defines */
	MMC_INTERNAL_ERROR = -8,
	MMC_NOT_CONFIGURED = -7,
	MMC_REQUEST_PENDING = -6,
	MMC_REQUEST_NOT_APPLICABLE = -5,
	MMC_INVALID_PARAMETER = -4,
	MMC_UNSUPPORTED_FEATURE = -3,
	MMC_UNSUPPORTED_HW = -2,
	MMC_ERROR = -1,
	MMC_OK = 0,
	MMC_INTERNAL_EVENT = 1,
	MMC_REMAINING_PENDING_EVENTS = 2,
	MMC_REMAINING_FILTER_PENDING_EVENTS = 3,
	MMC_NO_MORE_PENDING_EVENT = 4,
	MMC_NO_MORE_FILTER_PENDING_EVENT = 5,
	MMC_NO_PENDING_EVENT_ERROR = 7,
};
#endif

#define MCLK	(100*1000*1000)

#define INDEX(mask) ( \
    ((mask & 0x00000001) ? 0 : \
    ((mask & 0x00000002) ? 1 : \
    ((mask & 0x00000004) ? 2 : \
    ((mask & 0x00000008) ? 3 : \
    ((mask & 0x00000010) ? 4 : \
    ((mask & 0x00000020) ? 5 : \
    ((mask & 0x00000040) ? 6 : \
    ((mask & 0x00000080) ? 7 : \
    ((mask & 0x00000100) ? 8 : \
    ((mask & 0x00000200) ? 9 : \
    ((mask & 0x00000400) ? 10 : \
    ((mask & 0x00000800) ? 11 : \
    ((mask & 0x00001000) ? 12 : \
    ((mask & 0x00002000) ? 13 : \
    ((mask & 0x00004000) ? 14 : \
    ((mask & 0x00008000) ? 15 : \
    ((mask & 0x00010000) ? 16 : \
    ((mask & 0x00020000) ? 17 : \
    ((mask & 0x00040000) ? 18 : \
    ((mask & 0x00080000) ? 19 : \
    ((mask & 0x00100000) ? 20 : \
    ((mask & 0x00200000) ? 21 : \
    ((mask & 0x00400000) ? 22 : \
    ((mask & 0x00800000) ? 23 : \
    ((mask & 0x01000000) ? 24 : \
    ((mask & 0x02000000) ? 25 : \
    ((mask & 0x04000000) ? 26 : \
    ((mask & 0x08000000) ? 27 : \
    ((mask & 0x10000000) ? 28 : \
    ((mask & 0x20000000) ? 29 : \
    ((mask & 0x40000000) ? 30 : \
    ((mask & 0x80000000) ? 31 : 0) \
    ))))))))))))))))))))))))))))))) \
)

#define MMC_PERIPHERAL_ID0 0x81
#define MMC_PERIPHERAL_ID1 0x11
#define MMC_PERIPHERAL_ID2 0x04
#define MMC_PERIPHERAL_ID3 0x00

/* SDI Power Control register bits */

#define SDI_PWR_PWRCTRL_MASK	(0x00000003)
#define SDI_PWR_PWRCTRL_ON 	(0x00000003)
#define SDI_PWR_PWRCTRL_OFF 	(0x00000000)
#define SDI_PWR_DAT2DIREN	(0x00000004)
#define SDI_PWR_CMDDIREN	(0x00000008)
#define SDI_PWR_DAT0DIREN	(0x00000010)
#define SDI_PWR_DAT31DIREN	(0x00000020)
#define SDI_PWR_OPD		(0x00000040)
#define SDI_PWR_FBCLKEN		(0x00000080)
#define SDI_PWR_DAT74DIREN	(0x00000100)
#define SDI_PWR_RSTEN		(0x00000200)

#define VOLTAGE_WINDOW_MMC	(0x00FF8080)
#define VOLTAGE_WINDOW_SD	(0x80010000)

/* SDI clock control register bits */

#define SDI_CLKCR_CLKDIV_MASK	(0x000000FF)
#define SDI_CLKCR_CLKEN		(0x00000100)
#define SDI_CLKCR_PWRSAV	(0x00000200)
#define SDI_CLKCR_BYPASS	(0x00000400)
#define SDI_CLKCR_WIDBUS_MASK	(0x00001800)
#define SDI_CLKCR_WIDBUS_1	(0x00000000)
#define SDI_CLKCR_WIDBUS_4	(0x00000800)
#define SDI_CLKCR_WIDBUS_8	(0x00001000)
#define SDI_CLKCR_NEGEDGE	(0x00002000)
#define SDI_CLKCR_HWFC_EN	(0x00004000)

#define SDI_CLKCR_CLKDIV_INIT	(0x000000FD)

/* SDI command register bits */

#define SDI_CMD_CMDINDEX_MASK	(0x000000FF)
#define SDI_CMD_WAITRESP	(0x00000040)
#define SDI_CMD_LONGRESP	(0x00000080)
#define SDI_CMD_WAITINT		(0x00000100)
#define SDI_CMD_WAITPEND	(0x00000200)
#define SDI_CMD_CPSMEN		(0x00000400)
#define SDI_CMD_SDIOSUSPEND	(0x00000800)
#define SDI_CMD_ENDCMDCOMPL	(0x00001000)
#define SDI_CMD_NIEN		(0x00002000)
#define SDI_CMD_CE_ATACMD	(0x00004000)
#define SDI_CMD_CBOOTMODEEN	(0x00008000)

/* MMC_DATA_TIMEOUT sets the data timeout in seconds */
#define MMC_DATA_TIMEOUT	(30)

/* SDI Status register bits */

#define SDI_STA_CCRCFAIL	(0x00000001)	/* (1 << 0) */
#define SDI_STA_DCRCFAIL	(0x00000002)	/* (1 << 1) */
#define SDI_STA_CTIMEOUT	(0x00000004)	/* (1 << 2) */
#define SDI_STA_DTIMEOUT	(0x00000008)	/* (1 << 3) */
#define SDI_STA_TXUNDERR	(0x00000010)	/* (1 << 4) */
#define SDI_STA_RXOVERR		(0x00000020)	/* (1 << 5) */
#define SDI_STA_CMDREND		(0x00000040)	/* (1 << 6) */
#define SDI_STA_CMDSENT		(0x00000080)	/* (1 << 7) */
#define SDI_STA_DATAEND		(0x00000100)	/* (1 << 8) */
#define SDI_STA_STBITERR	(0x00000200)	/* (1 << 9) */
#define SDI_STA_DBCKEND		(0x00000400)	/* (1 << 10) */
#define SDI_STA_CMDACT		(0x00000800)	/* (1 << 11) */
#define SDI_STA_TXACT		(0x00001000)	/* (1 << 12) */
#define SDI_STA_RXACT		(0x00002000)	/* (1 << 13) */
#define SDI_STA_TXFIFOBW	(0x00004000)	/* (1 << 14) */
#define SDI_STA_RXFIFOBR	(0x00008000)	/* (1 << 15) */
#define SDI_STA_TXFIFOF		(0x00010000)	/* (1 << 16) */
#define SDI_STA_RXFIFOF		(0x00020000)	/* (1 << 17) */
#define SDI_STA_TXFIFOE		(0x00040000)	/* (1 << 18) */
#define SDI_STA_RXFIFOE		(0x00080000)	/* (1 << 19) */
#define SDI_STA_TXDAVL		(0x00100000)	/* (1 << 20) */
#define SDI_STA_RXDAVL		(0x00200000)	/* (1 << 21) */
#define SDI_STA_SDIOIT		(0x00400000)	/* (1 << 22) */
#define SDI_STA_CEATAEND	(0x00800000)	/* (1 << 23) */
#define SDI_STA_CARDBUSY	(0x01000000)	/* (1 << 24) */
#define SDI_STA_BOOTMODE	(0x02000000)	/* (1 << 25) */
#define SDI_STA_BOOTACKERR	(0x04000000)	/* (1 << 26) */
#define SDI_STA_BOOTACKTIMEOUT	(0x08000000)	/* (1 << 27) */
#define SDI_STA_RSTNEND		(0x10000000)	/* (1 << 28) */

/* SDI Interrupt Clear register bits */

#define SDI_ICR_MASK		(0x1DC007FF)
#define SDI_ICR_CCRCFAILC	(0x00000001)	/* (1 << 0) */
#define SDI_ICR_DCRCFAILC	(0x00000002)	/* (1 << 1) */
#define SDI_ICR_CTIMEOUTC	(0x00000004)	/* (1 << 2) */
#define SDI_ICR_DTIMEOUTC	(0x00000008)	/* (1 << 3) */
#define SDI_ICR_TXUNDERRC	(0x00000010)	/* (1 << 4) */
#define SDI_ICR_RXOVERRC	(0x00000020)	/* (1 << 5) */
#define SDI_ICR_CMDRENDC	(0x00000040)	/* (1 << 6) */
#define SDI_ICR_CMDSENTC	(0x00000080)	/* (1 << 7) */
#define SDI_ICR_DATAENDC	(0x00000100)	/* (1 << 8) */
#define SDI_ICR_STBITERRC	(0x00000200)	/* (1 << 9) */
#define SDI_ICR_DBCKENDC	(0x00000400)	/* (1 << 10) */
#define SDI_ICR_SDIOITC		(0x00400000)	/* (1 << 22) */
#define SDI_ICR_CEATAENDC	(0x00800000)	/* (1 << 23) */
#define SDI_ICR_BUSYENDC	(0x01000000)	/* (1 << 24) */
#define SDI_ICR_BOOTACKERRC	(0x04000000)	/* (1 << 26) */
#define SDI_ICR_BOOTACKTIMEOUTC	(0x08000000)	/* (1 << 27) */
#define SDI_ICR_RSTNENDC	(0x10000000)	/* (1 << 28) */

#define SDI_MASK0_MASK		(0x1FFFFFFF)

/* SDI Data control register bits */

#define SDI_DCTRL_DTEN			(0x00000001)	/* (1 << 0) */
#define SDI_DCTRL_DTDIR_IN		(0x00000002)	/* (1 << 1) */
#define SDI_DCTRL_DTMODE_STREAM		(0x00000004)	/* (1 << 2) */
#define SDI_DCTRL_DMAEN			(0x00000008)	/* (1 << 3) */
#define SDI_DCTRL_DBLOCKSIZE_MASK	(0x000000F0)
#define SDI_DCTRL_RWSTART		(0x00000100)	/* (1 << 8) */
#define SDI_DCTRL_RWSTOP		(0x00000200)	/* (1 << 9) */
#define SDI_DCTRL_RWMOD			(0x00000200)	/* (1 << 10) */
#define SDI_DCTRL_SDIOEN		(0x00000800)	/* (1 << 11) */
#define SDI_DCTRL_DMAREQCTL		(0x00001000)	/* (1 << 12) */
#define SDI_DCTRL_DBOOTMODEEN		(0x00002000)	/* (1 << 13) */
#define SDI_DCTRL_BUSYMODE		(0x00004000)	/* (1 << 14) */
#define SDI_DCTRL_DDR_MODE		(0x00008000)	/* (1 << 15) */
#define SDI_DCTRL_DBLOCKSIZE_V2_MASK	(0x7fff0000)

#define SDI_FIFO_BURST_SIZE		(8)

#ifndef __ASSEMBLY__
struct sdi_registers {
	u32 power;		/* 0x00*/
	u32 clock;		/* 0x04*/
	u32 argument;		/* 0x08*/
	u32 command;		/* 0x0c*/
	u32 respcommand;	/* 0x10*/
	u32 response0;		/* 0x14*/
	u32 response1;		/* 0x18*/
	u32 response2;		/* 0x1c*/
	u32 response3;		/* 0x20*/
	u32 datatimer;		/* 0x24*/
	u32 datalength;		/* 0x28*/
	u32 datactrl;		/* 0x2c*/
	u32 datacount;		/* 0x30*/
	u32 status; 		/* 0x34*/
	u32 status_clear;	/* 0x38*/
	u32 mask0;		/* 0x3c*/
	u32 mask1;		/* 0x40*/
	u32 card_select;	/* 0x44*/
	u32 fifo_count;		/* 0x48*/
	u32 padding1[(0x80-0x4C)>>2];
	u32 fifo; 		/* 0x80*/
	u32 padding2[(0xFE0-0x84)>>2];
	u32 periph_id0;		/* 0xFE0 mmc Peripheral Identi.cation Register*/
	u32 periph_id1;		/* 0xFE4*/
	u32 periph_id2;		/* 0xFE8*/
	u32 periph_id3;		/* 0xFEC*/
	u32 pcell_id0;		/* 0xFF0*/
	u32 pcell_id1;		/* 0xFF4*/
	u32 pcell_id2;		/* 0xFF8*/
	u32 pcell_id3;		/* 0xFFC*/
};
#endif

#endif

/*
 * (C) Copyright 2009
 * STEricsson, <www.stericsson.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __MMC_NOMADIK_P_H__
#define __MMC_NOMADIK_P_H__

#define MAX_ERROR_VALUE -65
typedef enum
{
    /* MMC specific error defines */
    MMC_CMD_CRC_FAIL                    = (MAX_ERROR_VALUE - 1),            /* Command response received (but CRC check failed) */
    MMC_DATA_CRC_FAIL                   = (MAX_ERROR_VALUE - 2),            /* Data bock sent/received (CRC check Failed) */
    MMC_CMD_RSP_TIMEOUT                 = (MAX_ERROR_VALUE - 3),            /* Command response timeout */
    MMC_DATA_TIMEOUT                    = (MAX_ERROR_VALUE - 4),            /* Data time out*/
    MMC_TX_UNDERRUN                     = (MAX_ERROR_VALUE - 5),            /* Transmit FIFO under-run */
    MMC_RX_OVERRUN                      = (MAX_ERROR_VALUE - 6),            /* Receive FIFO over-run */
    MMC_START_BIT_ERR                   = (MAX_ERROR_VALUE - 7),            /* Start bit not detected on all data signals in widE bus mode */
    MMC_CMD_OUT_OF_RANGE                = (MAX_ERROR_VALUE - 8),            /* CMD's argument was out of range.*/
    MMC_ADDR_MISALIGNED                 = (MAX_ERROR_VALUE - 9),            /* Misaligned address */
    MMC_BLOCK_LEN_ERR                   = (MAX_ERROR_VALUE - 10),           /* Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
    MMC_ERASE_SEQ_ERR                   = (MAX_ERROR_VALUE - 11),           /* An error in the sequence of erase command occurs.*/
    MMC_BAD_ERASE_PARAM                 = (MAX_ERROR_VALUE - 12),           /* An Invalid selection for erase groups */
    MMC_WRITE_PROT_VIOLATION            = (MAX_ERROR_VALUE - 13),           /* Attempt to program a write protect block */
    MMC_LOCK_UNLOCK_FAILED              = (MAX_ERROR_VALUE - 14),           /* Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
    MMC_COM_CRC_FAILED                  = (MAX_ERROR_VALUE - 15),           /* CRC check of the previous command failed */
    MMC_ILLEGAL_CMD                     = (MAX_ERROR_VALUE - 16),           /* Command is not legal for the card state */
    MMC_CARD_ECC_FAILED                 = (MAX_ERROR_VALUE - 17),           /* Card internal ECC was applied but failed to correct the data */
    MMC_CC_ERROR                        = (MAX_ERROR_VALUE - 18),           /* Internal card controller error */
    MMC_GENERAL_UNKNOWN_ERROR           = (MAX_ERROR_VALUE - 19),           /* General or Unknown error */
    MMC_STREAM_READ_UNDERRUN            = (MAX_ERROR_VALUE - 20),           /* The card could not sustain data transfer in stream read operation. */
    MMC_STREAM_WRITE_OVERRUN            = (MAX_ERROR_VALUE - 21),           /* The card could not sustain data programming in stream mode */
    MMC_CID_CSD_OVERWRITE               = (MAX_ERROR_VALUE - 22),           /* CID/CSD overwrite error */
    MMC_WP_ERASE_SKIP                   = (MAX_ERROR_VALUE - 23),           /* only partial address space was erased */
    MMC_CARD_ECC_DISABLED               = (MAX_ERROR_VALUE - 24),           /* Command has been executed without using internal ECC */
    MMC_ERASE_RESET                     = (MAX_ERROR_VALUE - 25),           /* Erase sequence was cleared before executing because an out of erase sequence command was received */
    MMC_AKE_SEQ_ERROR                   = (MAX_ERROR_VALUE - 26),           /* Error in sequence of authentication. */
    MMC_INVALID_VOLTRANGE               = (MAX_ERROR_VALUE - 27),
    MMC_ADDR_OUT_OF_RANGE               = (MAX_ERROR_VALUE - 28),
    MMC_SWITCH_ERROR                    = (MAX_ERROR_VALUE - 29),
    MMC_SDIO_DISABLED                   = (MAX_ERROR_VALUE - 30),
    MMC_SDIO_FUNCTION_BUSY              = (MAX_ERROR_VALUE - 31),
    MMC_SDIO_FUNCTION_FAILED            = (MAX_ERROR_VALUE - 32),
    MMC_SDIO_UNKNOWN_FUNCTION           = MAX_ERROR_VALUE,

    /* standard error defines */
    MMC_INTERNAL_ERROR                  = -8,
    MMC_NOT_CONFIGURED                  = -7,
    MMC_REQUEST_PENDING                 = -6,
    MMC_REQUEST_NOT_APPLICABLE          = -5,
    MMC_INVALID_PARAMETER               = -4,
    MMC_UNSUPPORTED_FEATURE             = -3,
    MMC_UNSUPPORTED_HW                  = -2,
    MMC_ERROR                           = -1,
    MMC_OK                              = 0,
    MMC_INTERNAL_EVENT                  = 1,
    MMC_REMAINING_PENDING_EVENTS        = 2,
    MMC_REMAINING_FILTER_PENDING_EVENTS = 3,
    MMC_NO_MORE_PENDING_EVENT           = 4,
    MMC_NO_MORE_FILTER_PENDING_EVENT    = 5,
    MMC_NO_PENDING_EVENT_ERROR          = 7,
} t_mmc_error;

int init_mmc_card(void);

#endif


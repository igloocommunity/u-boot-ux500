/*
 * Copyright (C) ST-Ericsson SA 2009
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
#ifndef _MMC_H_
#define _MMC_H_

#include <common.h>
#include "asm-arm/arch-stw8500/mmc.h"

/*---------------------------------------------------------------------------
 * Enums
 *---------------------------------------------------------------------------*/

extern char *mmc_error_name[];

#define t_mmc_irq_src u32

typedef enum {
    MMC_CMD_CRC_FAIL_INT            = 0x00000001,
    MMC_DATA_CRC_FAIL_INT           = 0x00000002,
    MMC_CMD_TIMEOUT_INT             = 0x00000004,
    MMC_DATA_TIMEOUT_INT            = 0x00000008,
    MMC_TX_UNDERRUN_INT             = 0x00000010,
    MMC_RX_OVERRUN_INT              = 0x00000020,
    MMC_CMD_RESP_OK_INT             = 0x00000040,
    MMC_CMD_SENT_INT                = 0x00000080,
    MMC_DATA_END_INT                = 0x00000100,
    MMC_DATA_BLOCK_OK_INT           = 0x00000400,
    MMC_CMD_ACTIVE_INT              = 0x00000800,
    MMC_TX_ACTIVE_INT               = 0x00001000,
    MMC_RX_ACTIVE_INT               = 0x00002000,
    MMC_TX_FIFO_HALF_EMPTY_INT      = 0x00004000,
    MMC_RX_FIFO_HALF_FULL_INT       = 0x00008000,
    MMC_TX_FIFO_FULL_INT            = 0x00010000,
    MMC_RX_FIFO_FULL_INT            = 0x00020000,
    MMC_TX_FIFO_EMPTY_INT           = 0x00040000,
    MMC_RX_FIFO_EMPTY_INT           = 0x00080000,
    MMC_TX_DATA_AVLBL_INT           = 0x00100000,
    MMC_RX_DATA_AVLBL_INT           = 0x00200000,
    MMC_ALL_STATIC_IT               = 0x000005FF,
    MMC_ALL_IT                      = 0x003FFDFF

} t_mmc_interrupt;

typedef enum {
    MMC_DISABLE  = 0,
    MMC_ENABLE
} t_mmc_state;


typedef enum {
    MMC_POWER_OFF = 0x0,
    MMC_POWER_UP  = 0x2,
    MMC_POWER_ON  = 0x3
} t_mmc_power_state;

typedef enum {
    MMC_PUSH_PULL = 0,
    MMC_OPEN_DRAIN  
} t_mmc_bus_mode;

typedef enum {
    MMC_GO_IDLE_STATE              = 0,
    MMC_SEND_OP_COND               = 1,
    MMC_ALL_SEND_CID               = 2,
    MMC_SET_REL_ADDR               = 3,      
    MMC_SET_DSR                    = 4,      
    MMC_SEL_DESEL_CARD             = 7, 
    MMC_SEND_CSD                   = 9,
    MMC_SEND_CID                   = 10, 
    MMC_READ_DAT_UNTIL_STOP        = 11,
    MMC_STOP_TRANSMISSION          = 12,  
    MMC_SEND_STATUS                = 13, 
    MMC_GO_INACTIVE_STATE          = 15,
    MMC_SET_BLOCKLEN               = 16,
    MMC_READ_SINGLE_BLOCK          = 17,
    MMC_READ_MULT_BLOCK            = 18,
    MMC_WRITE_DAT_UNTIL_STOP       = 20,
    MMC_SET_BLOCK_COUNT            = 23,
    MMC_WRITE_SINGLE_BLOCK         = 24,   
    MMC_WRITE_MULT_BLOCK           = 25,
    MMC_PROG_CID                   = 26, 
    MMC_PROG_CSD                   = 27,
    MMC_SET_WRITE_PROT             = 28,
    MMC_CLR_WRITE_PROT             = 29,  
    MMC_SEND_WRITE_PROT            = 30,
    MMC_ERASE_GRP_START            = 35,  
    MMC_ERASE_GRP_END              = 36,
    MMC_ERASE                      = 38,
    MMC_FAST_IO                    = 39, 
    MMC_GO_IRQ_STATE               = 40, 
    SD_SEND_OP_COND                = 41,
    MMC_LOCK_UNLOCK                = 42, 
    MMC_APP_CMD                    = 55,
    MMC_GEN_CMD                    = 56,
    MMC_NO_CMD                     = 64
} t_mmc_command_index;

typedef enum {
    MMC_SHORT_RESP = 0,
    MMC_LONG_RESP
} t_mmc_response_type;


typedef enum {
    MMC_WRITE= 0,
    MMC_READ
} t_mmc_transfer_direction;

typedef enum {
    MMC_BLOCK = 0,
    MMC_STREAM
} t_mmc_transfer_type;

typedef enum {
    POLLING_MODE = 0,
    INTERRUPT_MODE,
    DMA_MODE
} t_mmc_device_mode;

/*---------------------------------------------------------------------------
 * Structures
 *---------------------------------------------------------------------------*/

typedef struct {
    t_mmc_bus_mode mode;
    t_mmc_state rodctrl;
} t_mmc_bus_configuration;

typedef struct {
    t_mmc_state pwrsave;
    t_mmc_state bypass;
    t_mmc_state widebus;
} t_mmc_clock_control;

typedef struct {
    t_bool IsRespExpected;
    t_bool IsLongResp;
    t_bool IsInterruptMode;
    t_bool IsPending;
    t_mmc_state cmdpath;
} t_mmc_command_control;


#define t_mmc_event   u32

#define t_mmc_filter_mode u32
#define NO_FILTER_MODE		0

typedef enum 
{
    MMC_NEW = 0,
    MMC_OLD
} t_mmc_irq_status_usage;

typedef struct 
{
    t_mmc_irq_status_usage usage;
    u32 IRQStatus;
    u32 EventStatus;
} t_mmc_irq_status;

typedef enum {

    MMC_MULTIMEDIA_CARD,
    MMC_SECURE_DIGITAL_CARD,
    MMC_SECURE_DIGITAL_IO_CARD,
    MMC_HIGH_SPEED_MULTIMEDIA_CARD,
    MMC_SECURE_DIGITAL_IO_COMBO_CARD

} t_mmc_card_type;

typedef struct {
    u32 CID[4];
    u32 CSD[4];
    u16 RCA;
    t_mmc_card_type card_type;
    u8 padding;
    u8 sdio_cccr[4]; /* I/O ready, CCCR/SDIO revision, SD Specification revision, and Card Capability registers */
    int	blockaddressed;
} t_mmc_card_info;

typedef struct {
    t_mmc_error error;
    u16 transferred_bytes;
} t_mmc_last_transfer_info;	    	

typedef enum {
    NO_TRANSFER    = 0,
    TRANSFER_IN_PROGRESS
} t_mmc_transfer_state;

typedef enum {
    WRITE_PROT_WHOLE_CARD_TEMP = 0,
    WRITE_PROT_WHOLE_CARD_PERM,
    WRITE_PROT_SINGLE_GROUP
} t_mmc_write_protect_type;

/*---------------------------------------------------------------------------
 *  Functions Prototype                                                   
 *---------------------------------------------------------------------------*/


t_mmc_error		mmc_init (u8,t_logical_address) ;

t_mmc_error 		mmc_setpowerstate(t_mmc_power_state);
t_mmc_power_state 	mmc_getpowerstate(void);
t_mmc_error 		mmc_setoperatingvoltage(u8);
u8 			mmc_getoperatingvoltage (void);
t_mmc_error 		mmc_configbus(t_mmc_bus_configuration);

t_mmc_error 		mmc_setclock(t_mmc_state);
t_mmc_error 		mmc_configclockcontrol(t_mmc_clock_control);
t_mmc_error 		mmc_setclockfrequency(u8);
t_mmc_error 		mmc_sendcommand(t_mmc_command_index, u32,t_mmc_command_control) ;
t_mmc_command_index 	mmc_getcommandresponse(void) ;
t_mmc_error 		mmc_getresponse(t_mmc_response_type, u32*) ;
t_mmc_error 		mmc_setdatapath(t_mmc_state);
t_mmc_error 		mmc_setdatatimeout(u32);
t_mmc_error 		mmc_setdatalength(u16);
t_mmc_error 		mmc_setdatablocklength(u8);
t_mmc_error 		mmc_settransferdirection(t_mmc_transfer_direction) ;
t_mmc_error 		mmc_settransfertype(t_mmc_transfer_type);
t_mmc_error 		mmc_handledma(t_mmc_state);
u16 			mmc_getdatacounter(void);

t_mmc_error		mmc_selectsdcard(u8);
t_mmc_error 		mmc_poweron(u8) ;
t_mmc_error 		mmc_poweroff(u8);
t_mmc_error 		mmc_initializeCards(u8);
t_mmc_error  	    	mmc_getcardinfo(u8, t_mmc_card_info *) ;
t_mmc_error 		mmc_setdevicemode(t_mmc_device_mode);
t_mmc_error 		mmc_readbytes(u8, u32, u32*, u16) ;
t_mmc_error 		mmc_readblocks(u8, u32, u32*, u16, u16) ;
t_mmc_error 		mmc_writebytes(u8, u32, u32* , u16) ;
t_mmc_error 		mmc_writeblocks(u8, u32, u32*, u16, u16) ;
t_mmc_transfer_state    mmc_gettransferstate(void);
t_mmc_error  		mmc_erase(u8, u32, u32) ;

u8 convert_from_bytes_to_power_of_two (u16 no_of_bytes);

/*New Interrupt strategy(M1 functions)*/
void			mmc_acknowledgementevent(t_mmc_event *);

#endif /* _MMC_H_ */

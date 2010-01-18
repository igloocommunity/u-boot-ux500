/*
 * Copyright (C) ST-Ericsson 2009
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
#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>

#include <configs/u8500.h>
#include "common.h"
#include "mmc.h"
#include "mmc_p.h"

#define MAX_NUM_CARDS 5
t_mmc_register *t_mmc0;     /* Removed so it can be used in mmc_utils.c, by Chris S. */
t_mmc_register *t_mmc4;
t_mmc_register *t_mmc[MAX_NUM_CARDS];
u32 t_mmc_rel_addr = 0x00010000;
u32 emmc_rel_addr = 0x00100000;
u8 no_of_cards = 0;
u8 mmc_card = 0, sd_card = 0;
u32 clockfreq;
t_mmc_card_info card_array[30];
t_mmc_device_mode device_mode = POLLING_MODE;
u32 *source_buffer;
u32 *dest_buffer;
u16 total_no_of_bytes = 0;
u8 selected_card = 0;
t_mmc_error transfer_error = MMC_CMD_CRC_FAIL;
u32 pwddata[8];
t_mmc_event mmc_event;	    // For event management
u32 write_freq = 0;

/* Private Functions*/
t_mmc_error mmc_cmderror(u8);
t_mmc_error mmc_cmdresp145error(u8, u8);
t_mmc_error mmc_cmdresp2error(u8);
t_mmc_error mmc_cmdresp3error(u8);
t_mmc_error mmc_cmdresp6error(u8, u16 *, u8);
t_mmc_error mmc_cmdresp7error(u8);
t_mmc_error mmc_findblocklen(u16 nobytes, u8 * power);
t_mmc_error mmc_sendstatus(u8, u32 *);
t_mmc_error mmc_iscardprogramming(u8, u8 *);

/****************************************************************************/
/*	 NAME : mmc_init()						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine initializes the MMC registers, checks	    */
/*		Peripheral and PCell Id and clears all interrupts.	    */
/* PARAMETERS :								    */
/*	   IN : t_logical_address MMCBaseAddress:MMC registers base address */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error : MMC error code				    */
/****************************************************************************/

t_mmc_error mmc_init(u8 card_num, t_logical_address MMCBaseAddress)
{
	t_mmc_error error;
	if (card_num == 0)
		t_mmc0 = (t_mmc_register *) MMCBaseAddress;
	else if (card_num == 4)
		t_mmc[card_num] = (t_mmc_register *) MMCBaseAddress;
	error = MMC_OK;

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_setpowerstate()					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets the power status of the controller.	    */
/*									    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_Power_state Power state to set			    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error : MMC error code				    */
/****************************************************************************/
t_mmc_error mmc_setpowerstate(t_mmc_power_state MMCPowerState)
{
	t_mmc_error error;

	MMC_SET_CTRL(t_mmc0->mmc_Power, MMCPowerState);
	error = MMC_OK;

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_getpowerstate()					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine returns the power status of the controller.    */
/*									    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_Power_state Power state to set			    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error : MMC error code				    */
/****************************************************************************/

t_mmc_power_state mmc_getpowerstate()
{
	t_mmc_power_state state;
	state =   MMC_READ_BITS(t_mmc0->mmc_Power, MMC_Power_MASK_CTRL, sbMMC_Power_CTRL);
	return (state);
}

/*****************************************************************************/
/*	 NAME : mmc_setoperatingvoltage					     */
/*-------------------------------------------------------------------------- */
/* DESCRIPTION: This routine sets the operating voltage range of .	     */
/*		the controller						     */
/* PARAMETERS :								     */
/*	   IN : u8 The encoded value of the output voltage range to set.*/
/*	  OUT :								     */
/*									     */
/*     RETURN : t_mmc_error : MMC error code				     */
/*****************************************************************************/

t_mmc_error mmc_setoperatingvoltage(u8 value)
{
	t_mmc_error error;
	if (value < 16)
	{
		MMC_SET_VOLT(t_mmc0->mmc_Power, value);
		error = MMC_OK;
	}
	else
		error = MMC_INVALID_PARAMETER;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_getoperatingvoltage					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine returns the encoded value of the current	    */
/*		operating voltage range					    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : u8 The encoded value of the output voltage range to set*/
/****************************************************************************/

u8 mmc_getoperatingvoltage()
{
	u8 voltage;
	voltage =
	MMC_READ_BITS(t_mmc0->mmc_Power, MMC_Power_MASK_VOLT,sbMMC_Power_VOLT);
	return (voltage);
}

/****************************************************************************/
/*	 NAME : mmc_configbus						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine configures the bus in open drain or .	    */
/*		push-pull mode						    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_bus_configuration     Bus configuration to set	    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_configbus(t_mmc_bus_configuration busconfig)
{
	t_mmc_error error;
	MMC_SET_OPEND(t_mmc0->mmc_Power, busconfig.mode);
	MMC_SET_ROD(t_mmc0->mmc_Power, busconfig.rodctrl);
	error = MMC_OK;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_setclock						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets the clock state as enabled or disabled.   */
/*									    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_state	clock state to set.			    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_setclock(t_mmc_state busstate)
{
	t_mmc_error error;
	MMC_SET_CENABLE(t_mmc0->mmc_Clock, busstate);
	error = MMC_OK;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_configclockcontrol					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets configurations for clock and bus mode.    */
/*									    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_Clock_control	clock control state to set.	    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_configclockcontrol(t_mmc_clock_control clockcontrol)
{
	t_mmc_error error;

	MMC_SET_PWRSAVE(t_mmc0->mmc_Clock, clockcontrol.pwrsave);
	MMC_SET_BYPASS(t_mmc0->mmc_Clock, clockcontrol.bypass);
	MMC_SET_WIDEBUS(t_mmc0->mmc_Clock, clockcontrol.widebus);
	error = MMC_OK;

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_setclockfrequency					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets clock frequency for MMCI controller.	    */
/*									    */
/* PARAMETERS :								    */
/*	   IN : u8     Clock divider value for desired frequency.      */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/*     COMMENT:  Clock frequency is calculated by this formula:		    */
/*	MCICLK = MCLK/(2 * [Clock_div +1])				    */

/****************************************************************************/

t_mmc_error mmc_setclockfrequency(u8 clockdiv)
{
	t_mmc_error error;

	MMC_SET_CLKDIV(t_mmc0->mmc_Clock, clockdiv);
	error = MMC_OK;

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_sendcommand						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sends command and enable Command path	    */
/*		state machine.						    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_Command_index: Command to send.			    */
/*		u32: argument to send.				       */
/*		t_mmc_Command_control: Command control parameters to set.   */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_sendcommand(t_mmc_command_index commandindex, u32 argument,
		t_mmc_command_control commcontrol)
{
    t_mmc_error error;
    u32 reg;

    if (commandindex != MMC_NO_CMD)
    {
	reg = commandindex;
	t_mmc0->mmc_Argument = argument;
	reg |= (commcontrol.IsRespExpected << 6);
	reg |= (commcontrol.IsLongResp << 7);
	reg |= (commcontrol.IsInterruptMode << 8);
	reg |= (commcontrol.IsPending << 9);
	reg |= (commcontrol.cmdpath << 10);
	t_mmc0->mmc_Command = reg;
    }

    error = MMC_OK;
    return error;
}

/****************************************************************************/
/*	 NAME : mmc_getresponse					     */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine returns command index of last command for	    */
/*		which response received.				    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_Command_index command index of last command	    */

/****************************************************************************/

t_mmc_command_index mmc_getcommandresponse()
{
    t_mmc_command_index respcommand;
    respcommand = t_mmc0->mmc_RespCommand;
    return respcommand;
}

/****************************************************************************/
/*	 NAME : mmc_getresponse						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine returns response received from the card for    */
/*		the last command.					    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_response_type Expected response type		    */
/*		u32 *	u32 pointer to store response.		  */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_getresponse(t_mmc_response_type resptype, u32 * response)
{
    t_mmc_error error;
    if (resptype == MMC_SHORT_RESP)
	*response = (t_mmc0->mmc_Response0);
    else
    {
	*response = t_mmc0->mmc_Response0;
	*(response + 1) = t_mmc0->mmc_Response1;
	*(response + 2) = t_mmc0->mmc_Response2;
	*(response + 3) = t_mmc0->mmc_Response3;
    }

    error = MMC_OK;
    return error;

}

/****************************************************************************/
/*	 NAME : mmc_setdatapath						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine enables/disables the data path for data transfer.*/
/*									    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_state Specifies the state of the			    */
/*		 data path, whether to enabled or disabled.		    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_setdatapath(t_mmc_state datapath)
{
    t_mmc_error error;
    MMC_SET_DATAPATH(t_mmc0->mmc_DataCtrl, datapath);
    error = MMC_OK;
    return error;
}

/****************************************************************************/
/*	 NAME : mmc_setdatatimeout					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets the data timeout period in card bus	    */
/*		clock periods						    */
/* PARAMETERS :								    */
/*	   IN : u32 Specifies the timeout value of the data path.      */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_setdatatimeout(u32 datatimeout)
{
    t_mmc_error error;
    t_mmc0->mmc_DataTimer = datatimeout;
    error = MMC_OK;
    return error;

}

/******************************************************************************/
/*	 NAME : mmc_setdatalength					      */
/*----------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets the data length (in bytes) for the data     */
/*		transfer.						      */
/* PARAMETERS :								      */
/*	   IN : u16 Specifies the number of data bytes to be transferred.*/
/*	  OUT :								      */
/*									      */
/*     RETURN : t_mmc_error						      */

/******************************************************************************/

t_mmc_error mmc_setdatalength(u16 datalength)
{
    t_mmc_error error;

    t_mmc0->mmc_DataLength = datalength;
    error = MMC_OK;
    return error;
}

/****************************************************************************/
/*	 NAME : mmc_setdatablocklength					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets the data block size (in an encoded fashion)*/
/*		for block data transfer.				    */
/* PARAMETERS :								    */
/*	   IN : u8 Specifies the data block size for block transfer.   */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_setdatablocklength(u8 blocksize)
{
    t_mmc_error error;
    if (blocksize <= MAXBSIZEPOWER)
    {
	MMC_SET_BLOCKSIZE(t_mmc0->mmc_DataCtrl, blocksize);
	error = MMC_OK;
    }
    else
	error = MMC_INVALID_PARAMETER;
    return error;

}

/****************************************************************************/
/*	 NAME : mmc_settransferdirection				    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: routine sets direction for data transfer, whether	    */
/*		 the transfer is a read or write.			    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_transfer_direction  the direction for data transfer.  */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_settransferdirection(t_mmc_transfer_direction transdir)
{
    t_mmc_error error = MMC_OK;
    MMC_SET_DATADIR(t_mmc0->mmc_DataCtrl, transdir);
    return error;
}

/******************************************************************************/
/*	 NAME : mmc_settransfertype					      */
/*----------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets whether data transfer is		      */
/*		 in stream mode or block mode.				      */
/* PARAMETERS :								      */
/*  IN : t_mmc_transfer_type  Specifies the transfer type for data transfer.  */
/* OUT :								      */
/*									      */
/*     RETURN : t_mmc_error						      */
/******************************************************************************/

t_mmc_error mmc_settransfertype(t_mmc_transfer_type transtype)
{
    t_mmc_error error = MMC_OK;
    MMC_SET_MODE(t_mmc0->mmc_DataCtrl, transtype);
    return error;
}

/****************************************************************************/
/*	 NAME : mmc_handledma						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine enables or disables data transfer through DMA. */
/* PARAMETERS :								    */
/*	   IN : t_mmc_state  Specifies whether to enable/disable DMA for    */
/*			     data transfer.				    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_handledma(t_mmc_state dmastate)
{
    t_mmc_error error = MMC_OK;
    MMC_SET_DMA(t_mmc0->mmc_DataCtrl, dmastate);
    return error;
}

/****************************************************************************/
/*	 NAME : mmc_getdatacounter					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine returns number of data elements (in bytes)     */
/*		yet to be transferred.					    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : u16						       */
/****************************************************************************/

u16 mmc_getdatacounter()
{
    u16 no_of_elements;
    no_of_elements = t_mmc0->mmc_DataCnt;
    return no_of_elements;
}

/****************************************************************************/
/*	 NAME : mmc_selectsdcard					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine selects specific SD card.			    */
/* PARAMETERS :								    */
/*	   IN : u8 SD card to select.				       */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_selectsdcard(u8 cardno)
{
    t_mmc_error error;

    if (cardno < 16)
    {
	t_mmc0->mmc_SelectSD = cardno;
	error = MMC_OK;
    }
    else
	error = MMC_REQUEST_NOT_APPLICABLE;

    return error;

}

/****************************************************************************/
/*	 NAME : mmc_Poweron						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine enquires cards about their operating voltage   */
/*		and sets optimal value to supply output voltage. Sends out  */
/*		of range cards to inactive states. Also configures	    */
/*		clock controls.						    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_poweron(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 response, one_msec, count = 0, delay;
	t_bool validvoltage, flag = FALSE;
	u32 address_mode = Byte_Mode;

	validvoltage = FALSE;
	selected_card = 0;
	no_of_cards = 5;

	t_mmc[card_num]->mmc_Power = 0x43;  //PowerOn | OpenDrain;

	t_mmc[card_num]->mmc_Clock = 0x41FF;	//ClkDivInit| ClkEnable | Hwfc_en;//setting clk freq just less than 400KHz
	clockfreq = MCLK / (2 * (ClkDivInit + 1));
	t_mmc[card_num]->mmc_Mask0 &= ~AllInterrupts;
	one_msec = 52000 / ((t_mmc[card_num]->mmc_Power & 0xFF) + 2);

	t_mmc[card_num]->mmc_Argument = 0;
	t_mmc[card_num]->mmc_Command = (GO_IDLE_STATE & ~RespExpected) | CmdPathEnable;
	error = mmc_cmderror(card_num);
	if (error != MMC_OK)
		return error;

	/* send CMD8 to verify SD card interface operating condition */
	t_mmc[card_num]->mmc_Argument = Check_Pattern;
	t_mmc[card_num]->mmc_Command = SD_SEND_IF_COND | RespExpected | CmdPathEnable;

	error = mmc_cmdresp7error(card_num);
	/* IF ERROR IS COMMAND TIMEOUT IT IS MMC CARD */
	if (error == MMC_OK)
	{
		address_mode = Sector_Mode;
	}
	else
	{
		t_mmc[card_num]->mmc_Argument = 0;
		t_mmc[card_num]->mmc_Command = (GO_IDLE_STATE & ~RespExpected) | CmdPathEnable;
		error = mmc_cmderror(card_num);
		if (error != MMC_OK)
			return error;
	}
	t_mmc[card_num]->mmc_Argument = 0;
	t_mmc[card_num]->mmc_Command = APP_CMD | RespExpected | CmdPathEnable;

	for (delay = 0; delay < (one_msec * 27); delay++) ;
	error = mmc_cmdresp145error(APP_CMD, card_num);

	/* IF ERROR IS COMMAND TIMEOUT IT IS MMC CARD */
	if (MMC_OK == error)
	{
		/*SD CARD */
		/*Send CMD41 SD_APP_APP_OP_COND WITH ARGUMENT 0x00FFC000 */
		printf(" initcard:: Set the SD voltage \n");
		while ((!validvoltage) && (count < 0xFFFF))
		{
			if (flag)
			{
				/*SEND CMD55 APP_CMD with RCA as 0*/
				t_mmc[card_num]->mmc_Argument = 0;
				t_mmc[card_num]->mmc_Command = APP_CMD | RespExpected | CmdPathEnable;

				error = mmc_cmdresp145error(APP_CMD, card_num);
				if (error != MMC_OK)
				{
					return (error);
				}
			}

			t_mmc[card_num]->mmc_Argument = VoltageWindowSD | address_mode; /* voltage window */
			t_mmc[card_num]->mmc_Command =	SD_APP_OP_COND | RespExpected | CmdPathEnable;

			error = mmc_cmdresp3error(card_num);
			if (MMC_OK != error)
			{
				return (error);
			}

			response = t_mmc[card_num]->mmc_Response0;
			validvoltage = (t_bool) (((response >> 31) == 1) ? 1 : 0);
			flag = TRUE;
			count++;
		}

		if (count >= 0xFFFF)
		{
			error = MMC_INVALID_VOLTRANGE;
			return (error);
		}

		if (Sector_Mode == address_mode)
		{
			printf(" SD high capacity card detected \n");
		}
		else
		{
			printf(" SD card detected \n");
		}
		sd_card = 1;
	}
	else if (MMC_CMD_RSP_TIMEOUT == error)
	{
		/*Send CMD0 GO_IDLE_STATE*/
		printf(" initcard:: Set the MMC voltage \n");
		t_mmc[card_num]->mmc_Argument = 0;
		t_mmc[card_num]->mmc_Command = (GO_IDLE_STATE & ~RespExpected) | CmdPathEnable;

		error = mmc_cmderror(card_num);
		if (MMC_OK != error)
		{
			return (error);
		}
		address_mode = Byte_Mode;
		validvoltage = FALSE;
		response = 0;

		/* MMC_CARD */
		while ((!validvoltage) && (count < 0xFFFF))
		{
			t_mmc[card_num]->mmc_Argument = 0xc0ff8000; //VoltageWindowMMC | address_mode;
			t_mmc[card_num]->mmc_Command =	SEND_OP_COND | RespExpected | CmdPathEnable;

			error = mmc_cmdresp3error(card_num);
			if (MMC_OK != error)
			{
				return (error);
			}

			for (delay = 0; delay < 1; delay++) ;

			response = t_mmc[card_num]->mmc_Response0;
			validvoltage = (t_bool) (((response >> 31) == 1) ? 1 : 0);
			count++;
		}

		if (count >= 0xFFFF)
		{
			error = MMC_INVALID_VOLTRANGE;
			return (error);
		}

		if (response & Sector_Mode)
		{
			printf(" MMC high capacity card detected \n");
		}
		else
		{
			printf(" EMMC card detected \n");
		}
		mmc_card = 1;
		error = MMC_OK;
	}
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_Poweroff						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine turns the supply output voltage off.	    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/
t_mmc_error mmc_poweroff(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	t_mmc[card_num]->mmc_Power &= ~PowerOn;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_initializeCards						*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine initializes all cards. All cards come into     */
/*		standby state.						    */
/* PARAMETERS :								    */
/*	   IN :								    */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_initializeCards(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u16 rca = emmc_rel_addr;
	t_bool card_initialized = FALSE;

	if ((t_mmc[card_num]->mmc_Power & PowerOn) == 0x00)
	{
		printf("emmc power failed \n");
		error = MMC_REQUEST_NOT_APPLICABLE;
		return error;
	}
	while (!card_initialized)
	{
		t_mmc[card_num]->mmc_Argument = 0x00000000;
		t_mmc[card_num]->mmc_Command = ALL_SEND_CID | RespExpected | LongResponse | CmdPathEnable;

		error = mmc_cmdresp2error(card_num);
		if (MMC_OK != error)
		{
			return (error);
		}

		card_array[card_num].CID[0] = t_mmc[card_num]->mmc_Response3;
		card_array[card_num].CID[1] = t_mmc[card_num]->mmc_Response2;
		card_array[card_num].CID[2] = t_mmc[card_num]->mmc_Response1;
		card_array[card_num].CID[3] = t_mmc[card_num]->mmc_Response0;
		if (mmc_card == 1)
		{
			printf("init the MMC Card\n");
			t_mmc[card_num]->mmc_Argument = rca;//rca << 16;
			t_mmc[card_num]->mmc_Command = SET_REL_ADDR | RespExpected | CmdPathEnable;

			error = mmc_cmdresp145error(SET_REL_ADDR, card_num);
			if (error != MMC_OK)
			{
				printf ("emmc card init response for CMD3 error \n");
				return error;
			}
		}
		else if (sd_card == 1)
		{
			printf("init the SD Card\n");
			t_mmc[card_num]->mmc_Argument = 0;
			t_mmc[card_num]->mmc_Command = SET_REL_ADDR | RespExpected | CmdPathEnable;

			error = mmc_cmdresp6error(SET_REL_ADDR, &rca, card_num);
			if (error != MMC_OK)
			{
				printf ("emmc card init response for CMD6 error \n");
				return error;
			}
		}
		card_array[card_num].RCA = rca;
		t_mmc[card_num]->mmc_Argument = rca;//(u32) rca << 16;
		t_mmc[card_num]->mmc_Command = SEND_CSD | RespExpected | LongResponse | CmdPathEnable;

		error = mmc_cmdresp2error(card_num);
		if (error != MMC_OK)
		{
			printf("emmc card init response for CMD9 error \n");
			return error;
		}
		card_array[card_num].CSD[0] = t_mmc[card_num]->mmc_Response3;
		card_array[card_num].CSD[1] = t_mmc[card_num]->mmc_Response2;
		card_array[card_num].CSD[2] = t_mmc[card_num]->mmc_Response1;
		card_array[card_num].CSD[3] = t_mmc[card_num]->mmc_Response0;

		//rca++;
		error = MMC_OK; //All cards get intialized
		card_initialized = TRUE;

		if (card_num != selected_card)
		{
			t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;
			t_mmc[card_num]->mmc_Command = SEL_DESEL_CARD
				    | RespExpected | CmdPathEnable;
			error = mmc_cmdresp145error(SEL_DESEL_CARD, card_num);

			if (error != MMC_OK)
			{
				printf("SEL_DESEL_CARD ::error=0x%x \n", error);
				return error;
			}
			else
				selected_card = card_num;
		}

		t_mmc[card_num]->mmc_Argument = (u32) (0x03B70201);
		t_mmc[card_num]->mmc_Command =
			APP_SD_SET_BUSWIDTH | RespExpected | CmdPathEnable;

		error = mmc_cmdresp2error(card_num);
		if (error != MMC_OK)
		{
			printf("emmc card init response for CMD6 error \n");
			return error;
		}
	}
	t_mmc[card_num]->mmc_Power = 0x3;
	t_mmc[card_num]->mmc_Clock = 0x7500;

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_setdevicemode					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine sets device mode whether to operate in Polling,*/
/*		Interrupt, dma mode.					    */
/* PARAMETERS :								    */
/*	   IN : t_mmc_device_mode	 mode to for further transmission.  */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_setdevicemode(t_mmc_device_mode mode)
{
	t_mmc_error error;

	switch (mode)
	{
		case POLLING_MODE:
			device_mode = POLLING_MODE;
		break;
		case INTERRUPT_MODE:
			device_mode = INTERRUPT_MODE;
		break;
		case DMA_MODE:
			device_mode = DMA_MODE;
		break;
		default:
			error = MMC_INVALID_PARAMETER;
		return error;
	}

	error = MMC_OK;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_readbytes						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine allows to read bytes from specified address    */
/*		in a card						    */
/* PARAMETERS :								    */
/*	   IN : u8 cardno: card to access			            */
/*		u32 addr : address from where to start reading	            */
/*		u16 no_of_bytes: no. of bytes to read		            */
/*	  OUT : u32* readbuff: buffer to store data		            */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_readbytes(u8 card_num, u32 addr, u32 * readbuff,u16 no_of_bytes)
{
	t_mmc_error error = MMC_OK;
	u32 i;
	u32 timeout = 0;
	u32 *tempbuff = readbuff;

	total_no_of_bytes = 0;

	t_mmc0->mmc_DataCtrl = AllZero;

	if ((card_num > no_of_cards) || (card_num == 0))
	{
		error = MMC_INVALID_PARAMETER;
		return error;
	}

	/* send command for selecting the card */
	if (card_num != selected_card)
	{
		t_mmc0->mmc_Argument = card_array[card_num].RCA;//card_array[card_num - 1].RCA << 16;
		t_mmc0->mmc_Command = SEL_DESEL_CARD | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_REL_ADDR, card_num);
		if (error != MMC_OK)
			return error;
		else
			selected_card = card_num;
	}

	/* now depending on parameter no_of_bytes, send command READ_DAT_UNTIL_STOP */

	if (no_of_bytes == 0)	// this means open-ended stream read,until STOP_TRANSMISSION follows
	{
		if (device_mode != INTERRUPT_MODE)
			return MMC_REQUEST_NOT_APPLICABLE;

		total_no_of_bytes = 65532;

		t_mmc0->mmc_DataLength = 65532;

		t_mmc0->mmc_DataTimer = 0xefffffff;

		t_mmc0->mmc_DataCtrl = ReadDir | StreamMode | DataPathEnable;

		dest_buffer = readbuff;

		t_mmc0->mmc_Clock =   (t_mmc0->mmc_Clock & 0xFFFFFF00) | 0x0000000B;

		t_mmc0->mmc_Argument = addr >> 9;
		t_mmc0->mmc_Command =  READ_DAT_UNTIL_STOP | RespExpected | CmdPathEnable;

		error = mmc_cmdresp145error(READ_DAT_UNTIL_STOP, card_num);
		if (error != MMC_OK)
			return error;

		t_mmc0->mmc_Mask0 = DataCrcFail | DataTimeOut | RxFifoHalfFull | RxOverrun;
	}
	else if (no_of_bytes > 0)
	{
		total_no_of_bytes = no_of_bytes;

		t_mmc0->mmc_DataLength = no_of_bytes;

		t_mmc0->mmc_DataTimer = 0xefffffff;

		t_mmc0->mmc_DataCtrl = ReadDir | StreamMode | DataPathEnable;

		dest_buffer = readbuff;

		t_mmc0->mmc_Clock = (t_mmc0->mmc_Clock & 0xFFFFFF00) | 0x0000000B;

		t_mmc0->mmc_Argument = addr >> 9;
		t_mmc0->mmc_Command =	READ_DAT_UNTIL_STOP | RespExpected | CmdPathEnable;

		error = mmc_cmdresp145error(READ_DAT_UNTIL_STOP, card_num);

		if (error != MMC_OK)
			return error;

		if (device_mode == POLLING_MODE)
		{

			timeout = 0xffffffff;

			while ((timeout > 0) && !(t_mmc0->mmc_Status & (DataCrcFail | DataTimeOut |  DataEnd)))
			{
				timeout--;
				if (t_mmc0->mmc_Status & RxFifoHalfFull)
				{
					for (i = 0; i < 8; i++)
						*(tempbuff + i) =  t_mmc0->mmc_Fifo;
					tempbuff += 8;
				}

			}

			if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
			{
				t_mmc0->mmc_Clear |= DataTimeOut;
				error = MMC_DATA_TIMEOUT;
				transfer_error = MMC_DATA_TIMEOUT;
				return error;
			}
			else if (t_mmc0->mmc_Status & DataCrcFail)
			{
				t_mmc0->mmc_Clear |= DataCrcFail;
				error = MMC_DATA_CRC_FAIL;
				transfer_error = MMC_DATA_CRC_FAIL;
				return error;
			}

			t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

			while (t_mmc0->mmc_Status & RxDataAvlbl)
			{
				*tempbuff = t_mmc0->mmc_Fifo;
				tempbuff++;
			}

			t_mmc0->mmc_Argument = 0x00000000;
			t_mmc0->mmc_Command = STOP_TRANSMISSION | RespExpected | CmdPathEnable;

			error =    mmc_cmdresp145error(STOP_TRANSMISSION, card_num);

			transfer_error = error;
			if (error != MMC_OK)
				return error;

		}

		else if (device_mode == INTERRUPT_MODE)
			t_mmc0->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | RxFifoHalfFull | RxOverrun;

		else if (device_mode == DMA_MODE)
		{
			t_mmc0->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | RxOverrun;
			t_mmc0->mmc_DataCtrl |= DMAEnab;
		}
	}
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_writebytes						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine allows to write bytes starting from a specified*/
/*		address in a card					    */
/* PARAMETERS :								    */
/*	   IN : u8 cardno: card to access				    */
/*		u32 addr : address where to start writing		    */
/*		u16 no_of_bytes: no. of bytes to write		            */
/*	  OUT : u32* writebuff: source buffer			            */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_writebytes(u8 card_num, u32 addr, u32 * writebuff, u16 no_of_bytes)
{
	t_mmc_error error;
	u32 timeout = 0;
	u32 *tempbuff = writebuff;
	u32 i, j, bytes_transferred = 0;
	u8 cardstate;

	total_no_of_bytes = 0;

	t_mmc0->mmc_DataCtrl = AllZero;

	if ((card_num > no_of_cards) || (card_num == 0))
	{
		error = MMC_INVALID_PARAMETER;
		return error;
	}
	/* send command for selecting the card */

	if (card_num != selected_card)
	{
		t_mmc0->mmc_Argument = card_array[card_num].RCA;//card_array[card_num - 1].RCA << 16;
		t_mmc0->mmc_Command =	SEL_DESEL_CARD | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SEL_DESEL_CARD, card_num);

		if (error != MMC_OK)
			return error;
		else
			selected_card = card_num;
	}

	/* now depending on parameter no_of_bytes, send command WRITE_DAT_UNTIL_STOP */

	if (no_of_bytes == 0)	// this means open-ended stream read,until STOP_TRANSMISSION follows
	{
		if (device_mode != INTERRUPT_MODE)
			return MMC_REQUEST_NOT_APPLICABLE;

		t_mmc0->mmc_DataTimer = 0xefffffff;

		total_no_of_bytes = 65532;

		t_mmc0->mmc_DataLength = 65532;

		source_buffer = writebuff;

		t_mmc0->mmc_Clock = (t_mmc0->mmc_Clock & 0xFFFFFF00) | 0x00000031;

		t_mmc0->mmc_Argument = addr >> 9;
		t_mmc0->mmc_Command = WRITE_DAT_UNTIL_STOP | RespExpected | CmdPathEnable;

		error = mmc_cmdresp145error(WRITE_DAT_UNTIL_STOP, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc0->mmc_DataCtrl = (StreamMode & ~ReadDir) | DataPathEnable;

		t_mmc0->mmc_Mask0 =  DataCrcFail | DataTimeOut | TxFifoHalfEmpty | TxUnderrun;

		/*Now the card will send data thru DMA to the destination buffer.CRC/TimeOut error will
		be handled in the interrupt handler*/
	}
	else if (no_of_bytes > 0)
	{
		total_no_of_bytes = no_of_bytes;

		t_mmc0->mmc_DataLength = no_of_bytes;

		t_mmc0->mmc_DataTimer = 0xefffffff;

		source_buffer = writebuff;

		t_mmc0->mmc_Argument = addr >> 9;
		t_mmc0->mmc_Command =  WRITE_DAT_UNTIL_STOP | RespExpected | CmdPathEnable;

		error = mmc_cmdresp145error(WRITE_DAT_UNTIL_STOP, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc0->mmc_DataCtrl = (StreamMode & ~ReadDir) | DataPathEnable;

		if (device_mode == POLLING_MODE)
		{
			timeout = 0xefffffff;

			while ((timeout > 0) && !(t_mmc0->mmc_Status & (DataCrcFail | DataTimeOut | DataEnd)))
			{
				timeout--;
				if (t_mmc0->mmc_Status & TxFifoHalfEmpty)
				{
					if ((total_no_of_bytes -  bytes_transferred) < 32)
					{
						j = ((total_no_of_bytes -   bytes_transferred) % 4 == 0) ?
							((total_no_of_bytes -  bytes_transferred) / 4)
							: ((total_no_of_bytes -  bytes_transferred) / 4 +1);

						for (i = 0; i < j; i++, tempbuff++,bytes_transferred += 4)
							t_mmc0->mmc_Fifo =   *tempbuff;

					}
					else
					{
						for (i = 0; i < 8; i++)
							t_mmc0->mmc_Fifo =  *(tempbuff + i);
						tempbuff += 8;
						bytes_transferred += 32;
					}
				}
			}

			if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
			{
				t_mmc0->mmc_Clear |= DataTimeOut;
				transfer_error = error = MMC_DATA_TIMEOUT;
				return error;
			}
			else if (t_mmc0->mmc_Status & DataCrcFail)
			{
				t_mmc0->mmc_Clear |= DataCrcFail;
				transfer_error = error = MMC_DATA_CRC_FAIL;
				return error;
			}

			t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

			mmc_iscardprogramming(card_num, &cardstate);
			while ((cardstate == 7) || (cardstate == 6))
				mmc_iscardprogramming(card_num, &cardstate);

			t_mmc0->mmc_Argument = 0x00000000;
			t_mmc0->mmc_Command =  STOP_TRANSMISSION | RespExpected | CmdPathEnable;

			transfer_error = error =   mmc_cmdresp145error(STOP_TRANSMISSION, card_num);

			if (error != MMC_OK)
				return error;

		}

		else if (device_mode == INTERRUPT_MODE)

			t_mmc0->mmc_Mask0 =   DataCrcFail | DataTimeOut | DataEnd |  TxFifoHalfEmpty | TxUnderrun;

		else if (device_mode == DMA_MODE)
		{
			t_mmc0->mmc_Mask0 =   DataCrcFail | DataTimeOut | DataEnd | TxUnderrun;
			t_mmc0->mmc_DataCtrl |= DMAEnab;
		}

	}
	error = MMC_OK;
	return error;
}

/****************************************************************************/
/*	 NAME : mmc_readblocks						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine allows to read blocks from a specified	    */
/*		address in a card					    */
/* PARAMETERS :								    */
/*	   IN : u8 cardno: card to access			            */
/*		u32 addr : address from where to start reading	            */
/*		u16 blocksize : size of block in bytes		            */
/*		u16 no_of_blocks: no. of blocks to read		            */
/*	  OUT : u32* readbuff: source buffer			            */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_readblocks(u8 card_num, u32 addr, u32 * readbuff, u16 blocksize, u16 no_of_blocks)
{
	t_mmc_error error = MMC_OK;
	u32 i;
	u32 timeout = 0;
	u8 power;
	u32 *tempbuff = readbuff;

	total_no_of_bytes = 0;

	t_mmc[card_num]->mmc_DataCtrl = AllZero;

	/* send command for selecting the card */
	if ((card_num > no_of_cards) || (card_num == 0))
	{
		error = MMC_INVALID_PARAMETER;
		return error;
	}
	if (card_num != selected_card)
	{
		t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;//0x1 << 16;  //card_array[cardno - 1].RCA << 16;
		t_mmc[card_num]->mmc_Command =	SEL_DESEL_CARD | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SEL_DESEL_CARD, card_num);

		if (error != MMC_OK)
		{
			printf("SEL_DESEL_CARD ::error=0x%x \n", error);
			return error;
		}
		else
			selected_card = card_num;
	}

	if (t_mmc[card_num]->mmc_Response0 & 0x02000000)
		return MMC_LOCK_UNLOCK_FAILED;

	/* now depending on parameter no_of_blocks, send command READ_DAT_UNTIL_STOP */

	//set the block size,both on controller and card

	if ((blocksize > 0) && (blocksize <= 2048) && ((blocksize & (blocksize - 1)) == 0))
	{

		power = convert_from_bytes_to_power_of_two(blocksize);
		t_mmc[card_num]->mmc_DataCtrl = power << 4;

		t_mmc[card_num]->mmc_Argument = (u32) blocksize;
		t_mmc[card_num]->mmc_Command =	SET_BLOCKLEN | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCKLEN, card_num);

		if (error != MMC_OK)
		{
			printf("SET_BLOCKLEN ::error=0x%x \n", error);
			return error;
		}
	}
	else
	{
		printf("SET_BLOCKLEN ::error set proper block len\n");
		error = MMC_INVALID_PARAMETER;
		return error;
	}

	if (no_of_blocks == 0)	// this means open-ended block read,until STOP_TRANSMISSION follows
	{

		if (device_mode != INTERRUPT_MODE)
			return MMC_REQUEST_NOT_APPLICABLE;

		t_mmc[card_num]->mmc_DataTimer = 0xefffffff;

		t_mmc[card_num]->mmc_DataLength =  (65535 / blocksize) * blocksize;

		total_no_of_bytes = (65535 / blocksize) * blocksize;

		t_mmc[card_num]->mmc_Argument = (65535 / blocksize);
		t_mmc[card_num]->mmc_Command = SET_BLOCK_COUNT | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCK_COUNT, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_DataCtrl |= (ReadDir & ~StreamMode) | DataPathEnable;

		dest_buffer = readbuff;

		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command =	READ_MULT_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(READ_MULT_BLOCK, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_Mask0 =  DataCrcFail | DataTimeOut | RxFifoHalfFull | RxOverrun;

		/*Now the card will send data thru DMA to the destination buffer.CRC/TimeOut error will
		be handled in the interrupt handler*/

	}
	else if (no_of_blocks == 1)
	{
		total_no_of_bytes = blocksize;

		t_mmc[card_num]->mmc_DataLength = blocksize;

		t_mmc[card_num]->mmc_DataTimer = 0x0fffffff;

		t_mmc[card_num]->mmc_DataCtrl |=  (ReadDir & ~StreamMode) | DataPathEnable;

		dest_buffer = readbuff;

		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command =	READ_SINGLE_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(READ_SINGLE_BLOCK, card_num);

		if (error != MMC_OK)
		{
			printf("READ_SINGLE_BLOCK ::error=0x%x \n", error);
			return error;
		}
		if (device_mode == POLLING_MODE)
		{

			timeout = 0xefffffff;

			while ((timeout > 0) && !(t_mmc[card_num]->mmc_Status & (RxOverrun | DataCrcFail |
					DataTimeOut | DataBlockEnd)))
			{
				timeout--;
				if (t_mmc[card_num]->mmc_Status & RxFifoHalfFull)
				{
					for (i = 0; i < 8; i++)
					{
						*(tempbuff + i) = t_mmc[card_num]->mmc_Fifo;
					}
					tempbuff += 8;
				}

			}

			if ((timeout == 0)|| (t_mmc[card_num]->mmc_Status & DataTimeOut))
			{
				t_mmc[card_num]->mmc_Clear |= DataTimeOut;
				transfer_error = error = MMC_DATA_TIMEOUT;
				printf("mmc_readblocks::1 MMC_DATA_TIMEOUT \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & DataCrcFail)
			{
				t_mmc[card_num]->mmc_Clear |= DataCrcFail;
				transfer_error = error = MMC_DATA_CRC_FAIL;
				printf("mmc_readblocks::1 MMC_DATA_CRC_FAIL \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & RxOverrun)
			{
				t_mmc[card_num]->mmc_Clear |= RxOverrun;
				transfer_error = error = MMC_RX_OVERRUN;
				printf("mmc_readblocks::1 MMC_RX_OVERRUN \n");
				return error;
			}

			while (t_mmc[card_num]->mmc_Status & RxDataAvlbl)
			{
				*tempbuff = t_mmc[card_num]->mmc_Fifo;
				tempbuff++;
			}
			transfer_error = error;
			t_mmc[card_num]->mmc_Clear = ClrStaticFlags;	//clear all the static status flags
		}

		else if (device_mode == INTERRUPT_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | RxFifoHalfFull  | RxOverrun;
		}
		else if (device_mode == DMA_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | RxOverrun;
			t_mmc[card_num]->mmc_DataCtrl |= DMAEnab;
		}

	}
	else if (no_of_blocks > 1)
	{
		// set the block count,both for the controller and the card
		total_no_of_bytes = no_of_blocks * blocksize;

		t_mmc[card_num]->mmc_DataLength = no_of_blocks * blocksize;

		t_mmc[card_num]->mmc_Argument = (u32) no_of_blocks;
		t_mmc[card_num]->mmc_Command =	SET_BLOCK_COUNT | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCK_COUNT, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_DataTimer = 0xefffffff;

		t_mmc[card_num]->mmc_DataCtrl |= (ReadDir & ~StreamMode) | DataPathEnable;

		dest_buffer = readbuff;

		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command = READ_MULT_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(READ_MULT_BLOCK, card_num);
		if (error != MMC_OK)
			return error;

		if (device_mode == POLLING_MODE)
		{
			timeout = 0xefffffff;
			while ((timeout > 0)&& !(t_mmc[card_num]->mmc_Status & (RxOverrun | DataCrcFail |DataTimeOut | DataEnd)))
			{
				timeout--;
				if (t_mmc[card_num]->mmc_Status & RxFifoHalfFull)
				{
					for (i = 0; i < 8; i++)
					{
						*(tempbuff + i) = t_mmc[card_num]->mmc_Fifo;
					}
					tempbuff += 8;
				}
			}

			if ((timeout == 0) || (t_mmc[card_num]->mmc_Status & DataTimeOut))
			{
				t_mmc[card_num]->mmc_Clear |= DataTimeOut;
				transfer_error = error = MMC_DATA_TIMEOUT;
				return error;

			}
			else if (t_mmc[card_num]->mmc_Status & DataCrcFail)
			{
				t_mmc[card_num]->mmc_Clear |= DataCrcFail;
				transfer_error = error = MMC_DATA_CRC_FAIL;
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & RxOverrun)
			{
				t_mmc[card_num]->mmc_Clear |= RxOverrun;
				transfer_error = error = MMC_RX_OVERRUN;
				return error;
			}

			while (t_mmc[card_num]->mmc_Status & RxDataAvlbl)
			{
				*tempbuff = t_mmc[card_num]->mmc_Fifo;
				tempbuff++;
			}
			transfer_error = error;
			t_mmc[card_num]->mmc_Clear = ClrStaticFlags;	//clear all the static status flags
		}

		else if (device_mode == INTERRUPT_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 = DataCrcFail | DataTimeOut | DataEnd | RxFifoHalfFull | RxOverrun;
		}
		else if (device_mode == DMA_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | RxOverrun;
			t_mmc[card_num]->mmc_DataCtrl |= DMAEnab;
		}
	}

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_writeblocks						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine allows to write blocks starting from a	    */
/*		specified address in a card				    */
/* PARAMETERS :								    */
/*	   IN : u8 cardno: card to access			            */
/*		u32 addr : address from where to start writing	            */
/*		u16 blocksize : size of block in bytes		            */
/*		u16 no_of_blocks: no. of blocks to write	            */
/*	  OUT : u32* writebuff: source buffer			            */
/*									    */
/*     RETURN : t_mmc_error						    */
/****************************************************************************/

t_mmc_error mmc_writeblocks(u8 card_num, u32 addr, u32 * writebuff,
		u16 blocksize, u16 no_of_blocks)
{
	t_mmc_error error = MMC_OK;
	u32 count, rest_words;
	u8 power, cardstate;
	u32 timeout = 0;
	u32 *tempbuff = writebuff;
	u32 bytes_transferred = 0;
	u32 card_status;

	if (NULL == writebuff)
	{
		error = MMC_INVALID_PARAMETER;
		return (error);
	}

	total_no_of_bytes = 0;

	t_mmc[card_num]->mmc_DataCtrl = AllZero;

	/* send command for selecting the card */
	if ((card_num > no_of_cards) || (card_num == 0))
	{
		error = MMC_INVALID_PARAMETER;
		return error;
	}
	if (card_num != selected_card)
	{
		t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;  //card_array[cardno - 1].RCA << 16;
		t_mmc[card_num]->mmc_Command = SEL_DESEL_CARD | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SEL_DESEL_CARD, card_num);

		if (error != MMC_OK)
			return error;
		else
			selected_card = card_num;
	}

	if (t_mmc[card_num]->mmc_Response0 & R1_CARD_IS_LOCKED)
		return MMC_LOCK_UNLOCK_FAILED;

	/* now depending on parameter no_of_blocks, send command READ_DAT_UNTIL_STOP */

	//set the block size,both on controller and card

	if ((blocksize > 0) && (blocksize <= 2048)
		&& (((blocksize & (blocksize - 1)) == 0)))
	{
		power = convert_from_bytes_to_power_of_two(blocksize);
		t_mmc[card_num]->mmc_DataCtrl = power << 4;

		t_mmc[card_num]->mmc_Argument = (u32) blocksize;
		t_mmc[card_num]->mmc_Command = SET_BLOCKLEN | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCKLEN, card_num);

		if (error != MMC_OK)
			return error;

	}
	else
	{
		printf(" mmc_writeblocks::bad block size \n");
		error = MMC_INVALID_PARAMETER;
		return error;
	}

	if (no_of_blocks == 0)	// this means open-ended block read,until STOP_TRANSMISSION follows
	{
		if (device_mode != INTERRUPT_MODE)
			return MMC_REQUEST_NOT_APPLICABLE;

		t_mmc[card_num]->mmc_DataTimer = 0xefffffff;

		t_mmc[card_num]->mmc_DataLength =  (65535 / blocksize) * blocksize;

		total_no_of_bytes = (65535 / blocksize) * blocksize;

		t_mmc[card_num]->mmc_Argument = (65535 / blocksize);
		t_mmc[card_num]->mmc_Command = SET_BLOCK_COUNT | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCK_COUNT, card_num);

		if (error != MMC_OK)
			return error;

		source_buffer = writebuff;

		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command = WRITE_MULT_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(WRITE_MULT_BLOCK, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_DataCtrl |= DataPathEnable & ~(ReadDir | StreamMode);

		t_mmc[card_num]->mmc_Mask0 = DataCrcFail | DataTimeOut | TxFifoHalfEmpty | TxUnderrun;

		/*Now the card will send data thru DMA to the destination buffer.CRC/TimeOut error will
		be handled in the interrupt handler*/

	}
	else if (no_of_blocks == 1)
	{
		total_no_of_bytes = blocksize;

		t_mmc[card_num]->mmc_DataLength = blocksize;

		t_mmc[card_num]->mmc_DataTimer = 0xefffffff;

		source_buffer = writebuff;

		/*Wait till card is ready for data Added*/
		t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;  //card_array[cardno - 1].RCA << 16;
		t_mmc[card_num]->mmc_Command =	SEND_STATUS | RespExpected | CmdPathEnable;

		error = mmc_cmdresp145error(SEND_STATUS, card_num);

		if (error != MMC_OK)
			return error;

		card_status = t_mmc[card_num]->mmc_Response0;
		timeout = 0xefffffff;

		while ((0 == (card_status & 0x00000100)) && (timeout > 0))
		{
			timeout--;
			t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;//0x1 << 16;
			t_mmc[card_num]->mmc_Command =	SEND_STATUS | RespExpected | CmdPathEnable;
			error = mmc_cmdresp145error(SEND_STATUS, card_num);

			if (error != MMC_OK)
				return error;
			card_status = t_mmc[card_num]->mmc_Response0;
		}
		if (timeout == 0)
		{
			return (MMC_DATA_TIMEOUT);
		}

		/*Till here*/
		/*SEND CMD24 WRITE_SINGLE_BLOCK */
		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command = WRITE_SINGLE_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(WRITE_SINGLE_BLOCK, card_num);
		if (error != MMC_OK)
			return error;
		t_mmc[card_num]->mmc_DataCtrl |= DataPathEnable & ~(ReadDir | StreamMode);

		if (device_mode == POLLING_MODE)
		{

			while (!(t_mmc[card_num]->mmc_Status & (DataBlockEnd | TxUnderrun | DataCrcFail | DataTimeOut)))
			{

				if (t_mmc[card_num]-> mmc_Status & TxFifoHalfEmpty)
				{
					if ((total_no_of_bytes - bytes_transferred) < 32)
					{
						rest_words = ((total_no_of_bytes - bytes_transferred) % 4 == 0) ?
							((total_no_of_bytes - bytes_transferred) / 4)
							: ((total_no_of_bytes - bytes_transferred) / 4 +1);

						for (count = 0; count < rest_words; count++, tempbuff++, bytes_transferred += 4)
								t_mmc[card_num]-> mmc_Fifo = *tempbuff;

					}
					else
					{
						for (count = 0; count < 8; count++)
						{
							t_mmc[card_num]->mmc_Fifo = *(tempbuff + count);
						}
						tempbuff += 8;
						bytes_transferred += 32;
					}
				}
			}

			if ((timeout == 0) || (t_mmc[card_num]->mmc_Status & DataTimeOut))
			{
				t_mmc[card_num]->mmc_Clear |= DataTimeOut;
				transfer_error = error = MMC_DATA_TIMEOUT;
				printf(" MMC_DATA_TIMEOUT error \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & DataCrcFail)
			{
				t_mmc[card_num]->mmc_Clear |= DataCrcFail;
				transfer_error = error = MMC_DATA_CRC_FAIL;
				printf(" MMC_DATA_CRC_FAIL error \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & TxUnderrun)
			{
				t_mmc[card_num]->mmc_Clear |= TxUnderrun;
				transfer_error = error = MMC_TX_UNDERRUN;
				printf(" MMC_TX_UNDERRUN underrun error \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & StartBitError)
			{
				t_mmc[card_num]->mmc_Clear |= StartBitError;
				transfer_error = error = MMC_START_BIT_ERR;
				printf(" MMC_START_BIT_ERR start bit error \n");
				return error;
			}
			//clear all the static status flags
			t_mmc[card_num]->mmc_Clear = ClrStaticFlags;
			transfer_error = error;

			error = mmc_iscardprogramming(card_num, &cardstate);
			while (error == MMC_OK && ((cardstate == 7) || (cardstate == 6)))
			error = mmc_iscardprogramming(card_num, &cardstate);
		}

		else if (device_mode == INTERRUPT_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 =  DataCrcFail | DataTimeOut | DataEnd | TxFifoHalfEmpty | TxUnderrun;
		}
		else if (device_mode == DMA_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 = DataCrcFail | DataTimeOut | DataEnd | TxUnderrun;
			t_mmc[card_num]->mmc_DataCtrl |= DMAEnab;
		}

	}
	else if (no_of_blocks > 1)
	{
		// set the block count,both for the controller and the card
		if (no_of_blocks * blocksize > 0x01FFFFFF)
		{
			error = MMC_INVALID_PARAMETER;
			return (error);
		}

		total_no_of_bytes = no_of_blocks * blocksize;

		t_mmc[card_num]->mmc_DataLength = (u32) (no_of_blocks * blocksize);

		t_mmc[card_num]->mmc_Argument = (u32) no_of_blocks;
		t_mmc[card_num]->mmc_Command = SET_BLOCK_COUNT | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SET_BLOCK_COUNT, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_DataTimer = 0xefffffff;

		source_buffer = writebuff;

		/*SEND CMD25 WRITE_MULT_BLOCK with argument data address*/
		t_mmc[card_num]->mmc_Argument = addr >> 9;
		t_mmc[card_num]->mmc_Command = WRITE_MULT_BLOCK | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(WRITE_MULT_BLOCK, card_num);

		if (error != MMC_OK)
			return error;

		t_mmc[card_num]->mmc_DataCtrl |=  DataPathEnable & ~(ReadDir | StreamMode);

		if (device_mode == POLLING_MODE)
		{
			timeout = 0xefffffff;

			while ((timeout > 0) && !(t_mmc[card_num]->mmc_Status & (TxUnderrun | DataCrcFail |
					DataTimeOut | DataEnd | StartBitError)))
			{
				timeout--;
				if (t_mmc[card_num]-> mmc_Status & TxFifoHalfEmpty)
				{
					if ((total_no_of_bytes - bytes_transferred) < 32)
					{
						rest_words = ((total_no_of_bytes - bytes_transferred) % 4 == 0) ?
								((total_no_of_bytes - bytes_transferred) /4)
								: ((total_no_of_bytes - bytes_transferred) / 4 +1);

						for (count = 0; count < rest_words; count++, tempbuff++,bytes_transferred += 4)
							t_mmc[card_num]-> mmc_Fifo = *tempbuff;

					}
					else
					{
						for (count = 0; count < 8; count++)
						{
							t_mmc[card_num]-> mmc_Fifo = *(tempbuff + count);
						}
						tempbuff += 8;
						bytes_transferred += 32;
					}
				}

			}

			if ((timeout == 0) || (t_mmc[card_num]->mmc_Status & DataTimeOut))
			{
				t_mmc[card_num]->mmc_Clear |= DataTimeOut;
				transfer_error = error = MMC_DATA_TIMEOUT;
				printf(" MMC_DATA_TIMEOUT start bit error \n");
				return error;

			}
			else if (t_mmc[card_num]->mmc_Status & DataCrcFail)
			{
				t_mmc[card_num]->mmc_Clear |= DataCrcFail;
				transfer_error = error = MMC_DATA_CRC_FAIL;
				printf(" MMC_DATA_CRC_FAIL start bit error \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & TxUnderrun)
			{
				t_mmc[card_num]->mmc_Clear |= TxUnderrun;
				transfer_error = error = MMC_TX_UNDERRUN;
				printf(" MMC_TX_UNDERRUN start bit error \n");
				return error;
			}
			else if (t_mmc[card_num]->mmc_Status & StartBitError)
			{
				t_mmc[card_num]->mmc_Clear |= StartBitError;
				transfer_error = error = MMC_START_BIT_ERR;
				printf(" MMC_START_BIT_ERR start bit error \n");
				return error;
			}

			t_mmc[card_num]->mmc_Clear = ClrStaticFlags;	//clear all the static status flags
			transfer_error = error;
			error = mmc_iscardprogramming(card_num, &cardstate);
			while ((MMC_OK == error) && ((cardstate == 7) || (cardstate == 6)))
				error =  mmc_iscardprogramming(card_num, &cardstate);
		}
		else if (device_mode == INTERRUPT_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 = DataCrcFail | DataTimeOut | DataEnd |TxFifoHalfEmpty | TxUnderrun;
		}
		else if (device_mode == DMA_MODE)
		{
			t_mmc[card_num]->mmc_Mask0 = DataCrcFail | DataTimeOut | DataEnd | TxUnderrun;
			t_mmc[card_num]->mmc_DataCtrl |= DMAEnab;
		}

	}

	return error;
}

/****************************************************************************/
/*	 NAME : mmc_gettransferstate()					    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine						    */
/* PARAMETERS :								    */
/*									    */
/*     RETURN : t_mmc_transfer_state					    */
/****************************************************************************/

t_mmc_transfer_state mmc_gettransferstate()
{
    if (t_mmc0->mmc_Status & (TxActive | RxActive))
	return TRANSFER_IN_PROGRESS;
    else
	return NO_TRANSFER;
}

/****************************************************************************/
/*	 NAME : mmc_erase						    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION: This routine allows to erase memory area specified  for the */
/*		given card.						    */
/* PARAMETERS :								    */
/*	   IN : u8     Cardno. to access			       */
/*		u32    start address for erase.			       */
/*		u32    Last address for erase.			       */
/*	  OUT :								    */
/*									    */
/*     RETURN : t_mmc_error						    */

/****************************************************************************/

t_mmc_error mmc_erase(u8 card_num, u32 StartAddr, u32 EndAddr)
{
	t_mmc_error error;
	u32 response;
	u8 cardstate;
	u32 delay, max_delay;

	if ((card_num > no_of_cards) || (card_num == 0))
	{
	       printf("mmc_erase:: card no=%d no_of_cards=%d failed \n",card_num, no_of_cards);
		error = MMC_INVALID_PARAMETER;
		return error;
	}
	max_delay = 52000 / ((t_mmc[card_num]->mmc_Clock & 0xFF) + 2);

	/* send command for selecting the card */

	if (card_num != selected_card)
	{
		t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;  //card_array[cardno - 1].RCA << 16;
		t_mmc[card_num]->mmc_Command = SEL_DESEL_CARD | RespExpected | CmdPathEnable;
		error = mmc_cmdresp145error(SEL_DESEL_CARD, card_num);
		if (error != MMC_OK)
			return error;
		else
			selected_card = card_num;
	}

	if (t_mmc[card_num]->mmc_Response0 & R1_CARD_IS_LOCKED)
	{
		error = MMC_LOCK_UNLOCK_FAILED;
		return (error);
	}

	t_mmc[card_num]->mmc_Argument = StartAddr >> 9;
	t_mmc[card_num]->mmc_Command = ERASE_GRP_START | RespExpected | CmdPathEnable;
	error = mmc_cmdresp145error(ERASE_GRP_START, card_num);
	if (error != MMC_OK)
	{
		printf("mmc_erase:: erase start CMD35 failed \n");
		return error;
	}
	response = t_mmc[card_num]->mmc_Response0;

	if (response & (R1_OUT_OF_RANGE | R1_ERASE_PARAM))
	{
		printf("mmc_erase:: erase start CMD35 R1_OUT_OF_RANGE \n");
		error = MMC_BAD_ERASE_PARAM;
		return error;
	}

	if (response & R1_ERASE_SEQ_ERROR)
	{
		printf("mmc_erase:: erase start CMD35 R1_ERASE_SEQ_ERROR \n");
		error = MMC_ERASE_SEQ_ERR;
		return error;
	}

	if (response & (R1_CARD_IS_LOCKED | R1_LOCK_UNLOCK_FAILED))
	{
		printf("mmc_erase:: erase start CMD35 R1_CARD_IS_LOCKED \n");
		error = MMC_LOCK_UNLOCK_FAILED;
		return error;
	}

	t_mmc[card_num]->mmc_Argument = EndAddr >> 9;
	t_mmc[card_num]->mmc_Command = ERASE_GRP_END | RespExpected | CmdPathEnable;
	error = mmc_cmdresp145error(ERASE_GRP_END, card_num);
	if (error != MMC_OK)
	{
		printf("mmc_erase:: erase end CMD36 failed \n");
		return error;
	}
	response = t_mmc[card_num]->mmc_Response0;

	if (response & (R1_OUT_OF_RANGE | R1_ERASE_PARAM))
	{
		printf("mmc_erase:: erase end R1_OUT_OF_RANGE \n");
		error = MMC_BAD_ERASE_PARAM;
		return error;
	}

	if (response & R1_ERASE_SEQ_ERROR)
	{
		printf("mmc_erase:: erase end R1_ERASE_SEQ_ERROR \n");
		error = MMC_ERASE_SEQ_ERR;
		return error;
	}

	if (response & (R1_CARD_IS_LOCKED | R1_LOCK_UNLOCK_FAILED))
	{
		printf("mmc_erase:: erase end R1_CARD_IS_LOCKED\n");
		error = MMC_LOCK_UNLOCK_FAILED;
		return error;
	}

	t_mmc[card_num]->mmc_Argument = 0;
	t_mmc[card_num]->mmc_Command = ERASE | RespExpected | CmdPathEnable;
	error = mmc_cmdresp145error(ERASE, card_num);
	if (error != MMC_OK)
	{
		printf("mmc_erase:: erase CMD38 failed \n");
		return error;
	}
	response = t_mmc[card_num]->mmc_Response0;
	if (response & R1_ERASE_SEQ_ERROR)
	{
		printf("mmc_erase:: erase R1_ERASE_SEQ_ERROR \n");
		error = MMC_ERASE_SEQ_ERR;
		return error;
	}

	if (response & (R1_CARD_IS_LOCKED | R1_LOCK_UNLOCK_FAILED))
	{
		printf("mmc_erase:: erase R1_CARD_IS_LOCKED  \n");
		error = MMC_LOCK_UNLOCK_FAILED;
		return error;
	}

	if (response & R1_WP_ERASE_SKIP)
	{
		printf("mmc_erase:: erase R1_WP_ERASE_SKIP \n");
		error = MMC_WRITE_PROT_VIOLATION;
		return error;
	}
	for (delay = 0; delay < (max_delay * 3); delay++) ;

	mmc_iscardprogramming(card_num, &cardstate);
	while ((MMC_OK == error) && ((cardstate == 7) || (cardstate == 6)))
		mmc_iscardprogramming(card_num, &cardstate);

	error = MMC_OK;
	return error;
}

t_mmc_error mmc_cmderror(u8 card_num)
{
    t_mmc_error error = MMC_OK;
    u32 timeout;

    timeout = 100000;
    while ((timeout > 0) && !(t_mmc[card_num]->mmc_Status & CmdSent))
	timeout--;

    if (timeout == 0) {
	error = MMC_CMD_RSP_TIMEOUT;
	return error;
    }

    t_mmc[card_num]->mmc_Clear = ClrStaticFlags;    //clear all the static flags

    return error;
}

t_mmc_error mmc_cmdresp145error(u8 cmd, u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 100000;
	status = t_mmc[card_num]->mmc_Status;
	while ((timeout > 0) && !(status & (CmdCrcFail | CmdRespEnd | CmdTimeOut)))
	{
		timeout--;
		status = t_mmc[card_num]->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc[card_num]->mmc_Clear |= CmdTimeOut;
		return error;
	}
	else if (status & CmdCrcFail)
	{
		error = MMC_CMD_CRC_FAIL;
		t_mmc[card_num]->mmc_Clear |= CmdCrcFail;
		return error;
	}

	t_mmc[card_num]->mmc_Clear = ClrStaticFlags;	/*Clear Static flags */
	if (t_mmc[card_num]->mmc_RespCommand != cmd)
	{
		/* check response received for CMD7*/
		error = MMC_ILLEGAL_CMD;
		return error;
	}
	return error;
}

t_mmc_error mmc_cmdresp2error(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 100000;
	status = t_mmc[card_num]->mmc_Status;
	while ((timeout > 0) && !(status & (CmdCrcFail | CmdTimeOut | CmdRespEnd)))
	{
		timeout--;
		status = t_mmc[card_num]->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc[card_num]->mmc_Clear |= CmdTimeOut;
		return error;
	}
	else if (status & CmdCrcFail)
	{
		error = MMC_CMD_CRC_FAIL;
		t_mmc[card_num]->mmc_Clear |= CmdCrcFail;
		return error;
	}

	t_mmc[card_num]->mmc_Clear = ClrStaticFlags;	/*Clear Static flags */

	return error;
}

t_mmc_error mmc_cmdresp3error(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 0x100000;
	status = t_mmc[card_num]->mmc_Status;
	while ((timeout > 0) && !(status & (CmdRespEnd | CmdTimeOut | CmdCrcFail)))
	{
		timeout--;
		status = t_mmc[card_num]->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc[card_num]->mmc_Clear |= CmdTimeOut;
		return error;
	}

	t_mmc[card_num]->mmc_Clear = ClrStaticFlags;
	return error;
}

t_mmc_error mmc_cmdresp6error(u8 cmd, u16 * p_rca, u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 status;
	u32 response_r1;

	status = t_mmc[card_num]->mmc_Status;
	while (!(status & (CmdRespEnd | CmdTimeOut | CmdCrcFail)))
	{
		status = t_mmc[card_num]->mmc_Status;
	}

	if (status & CmdTimeOut)
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc[card_num]->mmc_Clear |= CmdTimeOut;
		return (error);
	}
	else if (status & CmdCrcFail)
	{
		error = MMC_CMD_CRC_FAIL;
		t_mmc[card_num]->mmc_Clear |= CmdCrcFail;
		return (error);
	}

	/* CHECK RESPONSE RECEIVED IS OF DESIRED COMMAND */
	if (t_mmc[card_num]->mmc_RespCommand != cmd)
	{
		error = MMC_ILLEGAL_CMD;
		return (error);
	}

	/*Clear Static flags*/
	t_mmc[card_num]->mmc_Clear = ClrStaticFlags;

	/* WE HAVE RECEIVED RESPONSE, RETRIEVE IT.  */
	response_r1 = t_mmc[card_num]->mmc_Response0;

	if (AllZero ==	 (response_r1 & (R6_GEN_UNKNOWN_ERROR | R6_ILLEGAL_CMD | R6_COM_CRC_FAILED)))
	{
		*p_rca = (u16) (response_r1 >> 16);
		return (error);
	}

	if (response_r1 & R6_GEN_UNKNOWN_ERROR)
	{
		return (MMC_GENERAL_UNKNOWN_ERROR);
	}

	if (response_r1 & R6_ILLEGAL_CMD)
	{
		return (MMC_ILLEGAL_CMD);
	}

	if (response_r1 & R6_COM_CRC_FAILED)
	{
		return (MMC_COM_CRC_FAILED);
	}

	return (error);
}

t_mmc_error mmc_cmdresp7error(u8 card_num)
{
	t_mmc_error error = MMC_OK;
	u32 status;
	u32 timeout = 10000;

	status = t_mmc[card_num]->mmc_Status;
	while (!(status & (CmdCrcFail | CmdRespEnd | CmdTimeOut)) && (timeout > 0))
	{
		timeout--;
		status = t_mmc[card_num]->mmc_Status;
	}

	if ((timeout == 0) || (status & CmdTimeOut))
	{
		/* Card is not V2.0 complient or card does not support the set voltage range */
		error = MMC_CMD_RSP_TIMEOUT;
		return (error);
	}

	if (status & CmdRespEnd)
	{
		/* Card is V2.0 complient */
		error = MMC_OK;
		return (error);
	}

	return (error);
}

t_mmc_error mmc_iscardprogramming(u8 card_num, u8 * status)
{
	t_mmc_error error;
	u32 response;

	t_mmc[card_num]->mmc_Argument = card_array[card_num].RCA;//0x1 << 16; //RCA
	t_mmc[card_num]->mmc_Command = SEND_STATUS | RespExpected | CmdPathEnable;

	error = mmc_cmdresp145error(SEND_STATUS, card_num);
	if (error != MMC_OK)
		return error;

	response = t_mmc[card_num]->mmc_Response0;
	*status = (response >> 9) & 0x0000000f;

	return error;

}

t_mmc_error mmc_sendstatus(u8 cardno, u32 * status)
{
	t_mmc_error error = MMC_OK;

	t_mmc0->mmc_Argument = card_array[cardno].RCA;//card_array[cardno - 1].RCA << 16;    //RCA
	t_mmc0->mmc_Command = SEND_STATUS | RespExpected | CmdPathEnable;

	error = mmc_cmdresp145error(SEND_STATUS, cardno);
	if (error != MMC_OK)
		return error;

	*status = t_mmc0->mmc_Response0;

	return error;
}

t_mmc_error mmc_cmderror_2(void)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 0xefffffff;
	status = t_mmc0->mmc_Status;
	while ((timeout > 0) && !(status & (CmdSent| CmdCrcFail | CmdTimeOut | CmdRespEnd)))
	{
		timeout--;
		status = t_mmc0->mmc_Status;
	}

	if (timeout == 0)
	{
		printf(" cmd1 timeout error\n");
		error = MMC_CMD_RSP_TIMEOUT;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static flags

	return error;
}

t_mmc_error mmc_cmdresp2error_2(void)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 0xefffffff;	//(u32)((float)(64/(float)clockfreq )/ ((float)(1/(float)MCLK) + 8*(float)(1/(float)PROCESSOR_CLK)));
	status = t_mmc0->mmc_Status;

	while ((timeout > 0) && !(status & (CmdCrcFail | CmdTimeOut | CmdRespEnd)))
	{
		timeout--;
		status = t_mmc0->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc0->mmc_Clear |= CmdTimeOut;
		return error;
	}
	if (status & CmdCrcFail)
	{
		error = MMC_CMD_CRC_FAIL;
		t_mmc0->mmc_Clear |= CmdCrcFail;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; /*Clear Static flags */

	return error;
}

t_mmc_error mmc_cmdresp3error_2(void)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 0xefffffff;
	status = t_mmc0->mmc_Status;

	while ((timeout > 0) && !(status & (CmdRespEnd | CmdTimeOut | CmdCrcFail)))
	{
		timeout--;
		status = t_mmc0->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc0->mmc_Clear |= CmdTimeOut;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags;

	return error;
}

t_mmc_error mmc_cmdresp145error_2(u8 cmd)
{
	t_mmc_error error = MMC_OK;
	u32 status, timeout;

	timeout = 0xefffffff;
	status = t_mmc0->mmc_Status;

	while ((timeout > 0)  && !(status & (CmdCrcFail | CmdRespEnd | CmdTimeOut)))
	{
		timeout--;
		status = t_mmc0->mmc_Status;
	}
	if ((timeout == 0) || (status & CmdTimeOut))
	{
		error = MMC_CMD_RSP_TIMEOUT;
		t_mmc0->mmc_Clear |= CmdTimeOut;
		return error;
	}
	else if (status & CmdCrcFail)
	{
		error = MMC_CMD_CRC_FAIL;
		t_mmc0->mmc_Clear |= CmdCrcFail;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; /*Clear Static flags */
	if (t_mmc0->mmc_RespCommand != cmd)
	{
		/* check response received for CMD7*/
		error = MMC_ILLEGAL_CMD;
		return error;
	}

	return error;
}

t_mmc_error mmc_cmdreadresp(u32 * tempbuff)
{
	t_mmc_error error = MMC_OK;
	u32 i;
	u32 timeout = 0;

	timeout = 0xefffffff;

	while ((timeout > 0) && !(t_mmc0-> mmc_Status & (RxOverrun | DataCrcFail | DataTimeOut |DataBlockEnd)))
	{
		timeout--;
		if (t_mmc0->mmc_Status & RxFifoHalfFull)
		{
			for (i = 0; i < 8; i++)
			{
				*(tempbuff + i) = t_mmc0->mmc_Fifo;
			}
			tempbuff += 8;
		}
	}
	if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
	{
		t_mmc0->mmc_Clear |= DataTimeOut;
		return error;
	}
	else if (t_mmc0->mmc_Status & DataCrcFail)
	{
		t_mmc0->mmc_Clear |= DataCrcFail;
		error = MMC_DATA_CRC_FAIL;
		return error;
	}
	else if (t_mmc0->mmc_Status & RxOverrun)
	{
		t_mmc0->mmc_Clear |= RxOverrun;
		error = MMC_RX_OVERRUN;
		return error;
	}

	while (t_mmc0->mmc_Status & RxDataAvlbl)
	{
		*tempbuff = t_mmc0->mmc_Fifo;
		tempbuff++;
	}
	t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

	return error;
}

t_mmc_error mmc_waitcmdreadresp(void)
{
	t_mmc_error error = MMC_OK;
	u32 timeout = 0;

	timeout = 0xefffffff;

	while (!(t_mmc0->mmc_Status & (RxOverrun | DataCrcFail | DataTimeOut | DataBlockEnd)))
	{

	}

	if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
	{
		t_mmc0->mmc_Clear |= DataTimeOut;
		return error;
	}
	else if (t_mmc0->mmc_Status & DataCrcFail)
	{
		t_mmc0->mmc_Clear |= DataCrcFail;
		error = MMC_DATA_CRC_FAIL;
		return error;
	}
	else if (t_mmc0->mmc_Status & RxOverrun)
	{
		t_mmc0->mmc_Clear |= RxOverrun;
		error = MMC_RX_OVERRUN;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

	return error;
}

t_mmc_error mmc_iscardprogramming_2(u8 * status)
{
	t_mmc_error error;
	u32 response;
	t_mmc_command_control commcontrol;

	commcontrol.IsRespExpected = TRUE;
	commcontrol.IsLongResp = FALSE;
	commcontrol.IsInterruptMode = FALSE;
	commcontrol.IsPending = FALSE;
	commcontrol.cmdpath = MMC_ENABLE;

	error = mmc_sendcommand(MMC_SEND_STATUS, t_mmc_rel_addr, commcontrol);
	error = mmc_cmdresp145error_2(MMC_SEND_STATUS);

	if (error != MMC_OK)
		return error;

	response = t_mmc0->mmc_Response0;
	*status = (response >> 9) & 0x0000000f;

	return error;
}

t_mmc_error mmc_cmdwriteresp(u8 cardno, u32 * tempbuff, u16 total_num_of_bytes)
{
	u32 timeout = 0;
	t_mmc_error error = MMC_OK;
	u32 bytes_transferred = 0;
	u32 i, j;
	u8 cardstate;

	timeout = 0xefffffff;

	while ((timeout > 0) && !(t_mmc0-> mmc_Status & (DataBlockEnd | TxUnderrun | DataCrcFail | DataTimeOut)))
	{
		timeout--;
		if (t_mmc0->mmc_Status & TxFifoHalfEmpty)
		{
			if ((total_num_of_bytes - bytes_transferred) < 32)
			{
				j = ((total_num_of_bytes - bytes_transferred) % 4 ==0) ?
					((total_num_of_bytes -	bytes_transferred) / 4) :
					((total_num_of_bytes - bytes_transferred) / 4 + 1);

				for (i = 0; i < j; i++, tempbuff++, bytes_transferred += 4)
					t_mmc0->mmc_Fifo = *tempbuff;
			}
			else
			{
				for (i = 0; i < 8; i++)
					t_mmc0->mmc_Fifo = *(tempbuff + i);
				tempbuff += 8;
				bytes_transferred += 32;
			}
		}
	}

	if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
	{
		t_mmc0->mmc_Clear |= DataTimeOut;
		error = MMC_DATA_TIMEOUT;
		printf(" mmc_cmdwriteresp timeout error \n");
		return error;
	}
	else if (t_mmc0->mmc_Status & DataCrcFail)
	{
		t_mmc0->mmc_Clear |= DataCrcFail;
		error = MMC_DATA_CRC_FAIL;
		printf(" mmc_cmdwriteresp CRC error \n");
		return error;
	}
	else if (t_mmc0->mmc_Status & TxUnderrun)
	{
		t_mmc0->mmc_Clear |= TxUnderrun;
		error = MMC_TX_UNDERRUN;
		printf(" mmc_cmdwriteresp underrun error \n");
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

	error = mmc_iscardprogramming_2(&cardstate);
	while (error == MMC_OK && cardstate == 7)
		error = mmc_iscardprogramming_2(&cardstate);

	return error;
}

t_mmc_error mmc_waitcmdwriteresp(u8 cardno, u32 * tempbuff, u16 total_num_of_bytes)
{
	u32 timeout = 0;
	t_mmc_error error = MMC_OK;
	u8 cardstate;

	timeout = 0xefffffff;

	while ((timeout > 0) && !(t_mmc0-> mmc_Status & (DataBlockEnd | TxUnderrun | DataCrcFail | DataTimeOut)))
	{

	}

	if ((timeout == 0) || (t_mmc0->mmc_Status & DataTimeOut))
	{
		t_mmc0->mmc_Clear |= DataTimeOut;
		error = MMC_DATA_TIMEOUT;
		return error;

	}
	else if (t_mmc0->mmc_Status & DataCrcFail)
	{
		t_mmc0->mmc_Clear |= DataCrcFail;
		error = MMC_DATA_CRC_FAIL;
		return error;
	}
	else if (t_mmc0->mmc_Status & TxUnderrun)
	{
		t_mmc0->mmc_Clear |= TxUnderrun;
		error = MMC_TX_UNDERRUN;
		return error;
	}

	t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags

	error = mmc_iscardprogramming_2(&cardstate);
	while (error == MMC_OK && cardstate == 7)
		error = mmc_iscardprogramming_2(&cardstate);

	return error;
}

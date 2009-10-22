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

#include "i2c.h"

/*---------------------------------------------------------------------------
 * defines
 *---------------------------------------------------------------------------*/
/* 8500 has 4 I2C controllers */
t_i2c_registers *gp_i2c_registers[4];

#define I2C_ENDAD_COUNTER       500000
#define I2C_INT_ENDED_COUNTER   5
#define I2C_BTF_COUNTER         5
#define I2C_BTF_COUNTER_POLLING 10
#define I2C_FIFO_FLUSH_COUNTER  500

/*-----------------------------------------------------------------------------
Global variables
-----------------------------------------------------------------------------*/

/* 8500 has 4 I2C Controllers */
 volatile t_i2c_system_context   g_i2c_system_context[4];    

#undef I2C_DEBUG
#ifdef I2C_DEBUG
#define info(fmt,args...)	printf("I2C: "fmt, ##args)
#else
#define info(fmt,args...)	(void) 0;
#endif


/*-----------------------------------------------------------------------------
	Configuration functions
-----------------------------------------------------------------------------*/
/****************************************************************************/
/* NAME			:	I2C_Init												*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Initialize the given I2C controller by specifying the   */
/*                  base logical address.                                   */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_logical_address : The controller's logical address      */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		: t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are invalid    */
/*                I2C_UNSUPPORTED_HW    if peripheral ids are not matched   */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_Init(t_i2c_device_id id, t_logical_address address)
{
    t_i2c_registers *p_i2c_registers;

    /* 
	Check if the controller id is valid.
	*/
	

    if (((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)  ) || (NULL == address))
    {
        return(I2C_INVALID_PARAMETER);
    }
    
        

    p_i2c_registers = (t_i2c_registers *) address;


    if
    (
        (I2C_P_ID_0 != p_i2c_registers->periph_id_0)
    ||  (I2C_P_ID_1 != p_i2c_registers->periph_id_1)
    ||  (I2C_P_ID_2 != p_i2c_registers->periph_id_2)
    ||  (I2C_P_ID_3 != p_i2c_registers->periph_id_3)
    ||  (I2C_CELL_ID_0 != p_i2c_registers->cell_id_0)
    ||  (I2C_CELL_ID_1 != p_i2c_registers->cell_id_1)
    ||  (I2C_CELL_ID_2 != p_i2c_registers->cell_id_2)
    ||  (I2C_CELL_ID_3 != p_i2c_registers->cell_id_3)
    )
    {
        return(I2C_UNSUPPORTED_HW);
    }


    /*
	Initialize the right structure and save the base address.
	*/
    g_i2c_system_context[id].base_address = address;
    g_i2c_system_context[id].freq_scl = 0;
    g_i2c_system_context[id].freq_input = 0;
    g_i2c_system_context[id].mode = I2C_FREQ_MODE_STANDARD;
    g_i2c_system_context[id].own_address = 0;
    g_i2c_system_context[id].enabled = FALSE;
    g_i2c_system_context[id].slave_address = 0;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].data = 0;
    g_i2c_system_context[id].databuffer = NULL;
    g_i2c_system_context[id].count_data = 0;
    g_i2c_system_context[id].register_index = 0;
    g_i2c_system_context[id].operation = (t_i2c_operation) I2C_NO_OPERATION;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = FALSE;

    /*   g_i2c_system_context[id].i2c_device_context... to be initialized*/
    g_i2c_system_context[id].digital_filter_control = I2C_DIGITAL_FILTERS_OFF;
    g_i2c_system_context[id].dma_sync_logic_control = I2C_DISABLE;
    g_i2c_system_context[id].start_byte_procedure = I2C_DISABLE;
    g_i2c_system_context[id].slave_data_setup_time = 0; /* TBD */
    g_i2c_system_context[id].high_speed_master_code = 0;
    g_i2c_system_context[id].bus_control_mode = I2C_BUS_SLAVE_MODE;
    g_i2c_system_context[id].i2c_loopback_mode = I2C_DISABLE;
    g_i2c_system_context[id].general_call_mode_handling = I2C_NO_GENERAL_CALL_HANDLING;

    g_i2c_system_context[id].index_transfer_mode = I2C_TRANSFER_MODE_POLLING;
    g_i2c_system_context[id].data_transfer_mode = I2C_TRANSFER_MODE_POLLING;
    g_i2c_system_context[id].i2c_transmit_interrupt_threshold = 1;
    g_i2c_system_context[id].i2c_receive_interrupt_threshold = 1;
    g_i2c_system_context[id].transmit_burst_length = 0;
    g_i2c_system_context[id].receive_burst_length = 0;
    g_i2c_system_context[id].index_format = I2C_NO_INDEX;
    g_i2c_system_context[id].current_bus_config = I2C_CURRENT_BUS_SLAVE_TRANSMITTER;
    g_i2c_system_context[id].std                = FALSE;

    
    /*Disable the interrupts */
    I2C_WRITE_REG(p_i2c_registers->imscr,I2C_CLEAR);
    /* Disable the controller */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    return(I2C_OK);
}


/****************************************************************************/
/* NAME			:	I2C_SetDeviceConfiguration                  			*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Configure the given I2C controller, by clearing         */
/*                   registers, setting the input clock. The controller and */
/*                   interrupts are disabled after this routine             */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_i2c_device_config : pointer to the structer containg    */
/*                                      the configuration                   */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not invalid*/
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_SetDeviceConfiguration(t_i2c_device_id id, t_i2c_device_config *p_device_config)
{
    t_i2c_error     error_status = I2C_OK;
    t_i2c_registers *p_i2c_registers;


    /* Check if parameters are valid.*/
    
    if (NULL == p_device_config || ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)))
    {
        return(I2C_INVALID_PARAMETER);
    }
  

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Disable the controller.*/
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    /* Save the value.*/
    g_i2c_system_context[id].enabled = FALSE;

    /* Now save the input parameters.*/
    g_i2c_system_context[id].digital_filter_control = p_device_config->i2c_digital_filter_control;
    g_i2c_system_context[id].slave_data_setup_time = p_device_config->slave_data_setup_time;
    g_i2c_system_context[id].dma_sync_logic_control = p_device_config->i2c_dma_sync_logic_control;
    g_i2c_system_context[id].start_byte_procedure = p_device_config->i2c_start_byte_procedure;
    g_i2c_system_context[id].high_speed_master_code = p_device_config->i2c_high_speed_master_code;
    g_i2c_system_context[id].freq_input = p_device_config->input_frequency;
    g_i2c_system_context[id].own_address = p_device_config->controller_i2c_address;

    /* Clear registers.*/
    I2C_WRITE_REG(p_i2c_registers->cr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->scr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->hsmcr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->tftr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->rftr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->dmar, I2C_CLEAR);

    /* Set own address.*/
    error_status = i2cp_SetOwnAddress(id, (u16) g_i2c_system_context[id].own_address);
    if (I2C_OK != error_status)
    {
        return(error_status);
    }

    /* set the  digital filter   */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->cr,
        I2C_CR_FON,
        I2C_CR_SHIFT_FON,
        (u32) g_i2c_system_context[id].digital_filter_control
    );

    /* Set the DMA sync logic */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->cr,
        I2C_CR_DMA_SLE,
        I2C_CR_SHIFT_DMA_SLE,
        (u32) g_i2c_system_context[id].dma_sync_logic_control
    );

    /* Set the Slave Data Set up Time */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->scr,
        I2C_SCR_DATA_SETUP_TIME,
        I2C_SCR_SHIFT_DATA_SETUP_TIME,
        g_i2c_system_context[id].slave_data_setup_time
    );

    /* Disable generation of interrupts.*/
    I2C_DisableIRQSrc((t_i2c_irq_src_id) (((u32) id << (u32) I2CID_SHIFT) | (u32) I2C_IRQ_SRC_ALL)); /* 
	                                            Single Interrupt source matches
	                                            the sequence of device ids they
	                                            are used interchangeably.
	                                            */

    /* Enable the I2C Controller */
    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_SetTransferConfiguration                  			*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Configure the given I2C controller for transfer mode,   */
/*                  baud rate, general call handling and bus access mode.   */
/*                  Additionally fifo levels, loopback control and DMA      */
/*                  burst length are also configured.           */
/*                  This routine enable the I2C controller                  */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_i2c_transfer_config :pointer to the structure containing */
/*                                      the configuration                   */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_UNSUPPORTED_FEATURE if required index and data        */
/*                  transfer modes are not supported.                       */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/

/****************************************************************************/
 t_i2c_error I2C_SetTransferConfiguration(t_i2c_device_id id, t_i2c_transfer_config *p_transfer_config)
{
    t_i2c_error     error_status = I2C_OK;
    t_i2c_registers *p_i2c_registers;

    /* Check if parameters are valid.*/
  
    if (NULL == p_transfer_config || ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)))
    {
        return(I2C_INVALID_PARAMETER);
    }


    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;


    /*
    Error handling for unsopported features according to the platform
    */
    if
    (
        (
            I2C_TRANSFER_MODE_POLLING != p_transfer_config->index_transfer_mode
        &&  I2C_TRANSFER_MODE_POLLING == p_transfer_config->data_transfer_mode
        )
    ||  (    (I2C_TRANSFER_MODE_INTERRUPT == p_transfer_config->index_transfer_mode) 
          && (I2C_TRANSFER_MODE_DMA == p_transfer_config->data_transfer_mode)
        )
        
    ||  (I2C_TRANSFER_MODE_DMA == p_transfer_config->index_transfer_mode)
    )
    {
        return(I2C_UNSUPPORTED_FEATURE);
    }

    /* 
        Clear all the existing state of the controller by clearing PE bit
    */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    /* Now save the input parameters.*/
    g_i2c_system_context[id].i2c_loopback_mode = p_transfer_config->i2c_loopback_mode;
    g_i2c_system_context[id].general_call_mode_handling = p_transfer_config->i2c_slave_general_call_mode;
    g_i2c_system_context[id].index_transfer_mode = p_transfer_config->index_transfer_mode;

    /*Index transfer mode is still relevant even if I2C_NO_INDEX is 
      used since then this mode is used for the address transmission */
      
    g_i2c_system_context[id].data_transfer_mode = p_transfer_config->data_transfer_mode;
    g_i2c_system_context[id].i2c_transmit_interrupt_threshold = p_transfer_config->i2c_transmit_interrupt_threshold;
    g_i2c_system_context[id].i2c_receive_interrupt_threshold = p_transfer_config->i2c_receive_interrupt_threshold;
    g_i2c_system_context[id].transmit_burst_length = p_transfer_config->transmit_burst_length;
    g_i2c_system_context[id].receive_burst_length = p_transfer_config->receive_burst_length;
    g_i2c_system_context[id].freq_scl = p_transfer_config->i2c_transfer_frequency;
    g_i2c_system_context[id].bus_control_mode = p_transfer_config->bus_control_mode;

    g_i2c_system_context[id].multi_operation = FALSE;
    g_i2c_system_context[id].register_index = 0;    /* The index of the slave's registers*/
    g_i2c_system_context[id].index_format = I2C_NO_INDEX;


    /* Set the SCL bus clock frequency. -> transfer frequency*/
    error_status = i2cp_SetBusClock(id, g_i2c_system_context[id].freq_scl, g_i2c_system_context[id].freq_input);
    if (I2C_OK != error_status)
    {
        return(error_status);
    }

    /*Set the loop back mode */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->cr,
        I2C_CR_LM,
        I2C_CR_SHIFT_LM,
        (u32) g_i2c_system_context[id].i2c_loopback_mode
    );

    /*  Enable the general call handing in the controller*/
    /*  Only possible general call handing in this controller is 
	    in the software mode.
	*/
    if (I2C_HARDWARE_GENERAL_CALL_HANDLING == g_i2c_system_context[id].general_call_mode_handling)
    {
        I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_SGCM);
    }
    else
    {
        I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_SGCM);
    }

    /* Disable the Tx DMA */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_DMA_TX_EN);

    /* Disable the Rx DMA */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_DMA_RX_EN);

    /* configure the Tx DMA burst size */
    if (g_i2c_system_context[id].transmit_burst_length >= 1)
    {
        /* set the DMA Tx request mode to Burst */
        I2C_SET_BIT(p_i2c_registers->dmar, I2C_DMA_BURST_TX);

        /* Set the Destination Burst Size */
        I2C_WRITE_FIELD
        (
            p_i2c_registers->dmar,
            I2C_DMA_DBSIZE_TX,
            I2C_DMA_SHIFT_DBSIZE_TX,
            g_i2c_system_context[id].transmit_burst_length
        );
    }
    else
    {
        /* Set the DMA Tx Request mode to Single */
        I2C_CLR_BIT(p_i2c_registers->dmar, I2C_DMA_BURST_TX);
    }

    /* configure the Rx DMA burst size */
    if (g_i2c_system_context[id].receive_burst_length >= 1)
    {
        /* set the DMA Rx request mode to Burst */
        I2C_SET_BIT(p_i2c_registers->dmar, I2C_DMA_BURST_RX);

        /* Set the source burst size */
        I2C_WRITE_FIELD
        (
            p_i2c_registers->dmar,
            I2C_DMA_SBSIZE_RX,
            I2C_DMA_SHIFT_SBSIZE_RX,
            g_i2c_system_context[id].receive_burst_length
        );
    }
    else
    {
        /* Set the DMA Rx Request mode to Single */
        I2C_CLR_BIT(p_i2c_registers->dmar, I2C_DMA_BURST_RX);
    }

    /* Set the Bus control mode */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->cr,
        I2C_CR_OM,
        I2C_CR_SHIFT_OM,
        (u32) g_i2c_system_context[id].bus_control_mode
    );

    /* Set the Transmit Fifo threshold value */
    p_i2c_registers->tftr = g_i2c_system_context[id].i2c_transmit_interrupt_threshold;

    /* Set the Receive Fifo Threshold value */
    p_i2c_registers->rftr = g_i2c_system_context[id].i2c_receive_interrupt_threshold;

    /*Disable the interrupts if index transfer mode is polling */
    if (I2C_TRANSFER_MODE_POLLING == g_i2c_system_context[id].index_transfer_mode)
    {
        I2C_DisableIRQSrc((t_i2c_irq_src_id) (((u32) id << (u32) I2CID_SHIFT) | (u32) I2C_IRQ_SRC_ALL));
    }

    /* Now restore CR register with the values it has before it was disabled.*/
    /*Enable the I2C Controller and set the enable status in the global structure */
    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);
    g_i2c_system_context[id].enabled = TRUE;

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_SetTransferMode                  			        */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Set the transfer modes for the index and data transfer  */
/*                   on the given I2C controller                            */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_i2c_transfer_mode : Index transfer mode                 */
/*                t_i2c_transfer_mode : data  transger mode                 */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_UNSUPPORTED_FEATURE if required index and data        */
/*                  transfer modes are not supported.                       */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_SetTransferMode
(
    t_i2c_device_id      id,
    t_i2c_transfer_mode  index_transfer_mode,
    t_i2c_transfer_mode  data_transfer_mode
)
{
    info(
        "Id is %d, index  and data transfer modes are %lx and %lx",
        id,
        (u32) index_transfer_mode,
        (u32) data_transfer_mode
    );

    if
    (
        (I2C_TRANSFER_MODE_POLLING != index_transfer_mode && I2C_TRANSFER_MODE_POLLING == data_transfer_mode)
    ||  (    (I2C_TRANSFER_MODE_INTERRUPT == index_transfer_mode) 
          && (I2C_TRANSFER_MODE_DMA == data_transfer_mode)
        )

    ||  (I2C_TRANSFER_MODE_DMA == index_transfer_mode)
    )
    {
        return(I2C_UNSUPPORTED_FEATURE);
    }


    /* Check if parameters are valid.*/

    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }


    g_i2c_system_context[id].index_transfer_mode = index_transfer_mode;

    /*Index transfer mode is still relevant even if I2C_NO_INDEX is 
      used since then this mode is used for the addres transmission */
      
    g_i2c_system_context[id].data_transfer_mode = data_transfer_mode;

    /*Disable the interrupts if index tranfer mode is polling  */
    if (I2C_TRANSFER_MODE_POLLING == index_transfer_mode)
    {

        I2C_DisableIRQSrc((t_i2c_irq_src_id) (((u32) id << (u32) I2CID_SHIFT) | (u32) I2C_IRQ_SRC_ALL));

    }

    return(I2C_OK);
}

/****************************************************************************/
/* NAME			:	I2C_SetBusControlMode                  			        */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Set the bus control mode for the data transfer on the   */
/*                given I2C controller.                                     */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_i2c_bus_control_mode : The mode in which I2C bus        */
/*                                         is accessed                      */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/

/****************************************************************************/
 t_i2c_error I2C_SetBusControlMode(t_i2c_device_id id, t_i2c_bus_control_mode bus_control_mode)
{


    t_i2c_registers *p_i2c_registers;
    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;


    /* Check if parameters are valid.*/

    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }


    g_i2c_system_context[id].bus_control_mode = bus_control_mode;

    /* Disable the I2C controller before configuring */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    /* Set the Bus control mode */
    I2C_WRITE_FIELD
    (
        p_i2c_registers->cr,
        I2C_CR_OM,
        I2C_CR_SHIFT_OM,
        (u32) g_i2c_system_context[id].bus_control_mode
    );

    /* Enable the I2C controller */
    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);

    return(I2C_OK);
}

/*-----------------------------------------------------------------------------
	    Configuration functions
-----------------------------------------------------------------------------*/
/****************************************************************************/
/* NAME			:	I2C_FlushFifo                  			                */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Flush the transmit or receive FIFO                      */ 
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                t_i2c_fifo :        FIFO to be flused                     */
/*     InOut    :  None                                                     */
/* 			:  None                                                	    */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*               = I2C_INVALID_PARAMETER  - if input id is wrong            */
/*               = I2C_HW_FAILED          - if FIFO flush bit is not reset  */
/*                            itself after setting. This could be happen if */
/*                            i2c clock frequency is not set                */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/

/****************************************************************************/
 t_i2c_error I2C_FlushFifo(t_i2c_device_id id, t_i2c_fifo fifo)
{

    u32        loop_counter;
    t_i2c_registers *p_i2c_registers;

    /* Check parameters valid */
    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }


    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    switch (fifo)
    {
        case I2C_TRANSMIT_FIFO:
            I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FTX);   /* Flush the Tx Fifo */

            /*Wait till for the Tx Flush bit to reset */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->cr, I2C_CR_FTX, I2C_CR_SHIFT_FTX)
            &&  loop_counter < I2C_FIFO_FLUSH_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_FIFO_FLUSH_COUNTER)
            {
                return(I2C_HW_FAILED);
            }
            break;

        case I2C_RECEIVE_FIFO:
            I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FRX);   /* Flush the Rx Fifo */

            /* Wait till Rx flush bit to reset */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->cr, I2C_CR_FRX, I2C_CR_SHIFT_FRX)
            &&  loop_counter < I2C_FIFO_FLUSH_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_FIFO_FLUSH_COUNTER)
            {
                return(I2C_HW_FAILED);
            }
            break;
    }

    return(I2C_OK);

}





/****************************************************************************/
/* NAME			:	I2C_Enable                  		                    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:Enable the given I2C controller.                           */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_Enable(t_i2c_device_id id)
{
    t_i2c_registers *p_i2c_registers;

    /* Check if parameters are valid.*/
    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }
  

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);

    g_i2c_system_context[id].enabled = TRUE;
    return(I2C_OK);
}

/****************************************************************************/
/* NAME			:	I2C_Disable                  		                    */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:Disble the given I2C controller.                           */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to disabled       		*/
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_Disable(t_i2c_device_id id)
{
    t_i2c_registers *p_i2c_registers;

    /* Check if parameters are valid.*/
    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;
    
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    g_i2c_system_context[id].enabled = FALSE;
    return(I2C_OK);

}



/****************************************************************************/
/* NAME			:	I2C_WriteSingleData                  		            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:This routine is used to write a single data byte to        */
/*                a receiver. Writing can be done to a slave device by      */
/*               using the indexed modes.                                   */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                u16   : The address of the slave to be accessed      */
/*                u16   : The index of the register on the receiver    */
/*                             to which data is written                     */
/*                t_unit16   :The format of the index on receiver side      */
/*                u8 : The data byte to be written to the slave device */
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_SLAVE_ADDRESS_NOT_VALID  If requested slave address   */
/*                                      is not valid                        */
/*                I2C_CONTROLLER_BUSY    if I2C controller is busy          */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_WriteSingleData
(
    t_i2c_device_id      id,
    u16             slave_address,
    t_i2c_index_format   index_format,
    u16             index_value,
    u8              data
)
{

/*
Steps:
    - Check Mode
        - Polling
        - Interrupt
        - DMA
*/

    volatile u32   mcr = 0;
    t_i2c_error         error_status = I2C_OK;
    t_i2c_registers     *p_i2c_registers;
    info(
        "Id is %d, Address is %x, Index format is %d and value is %d, Data is %d",
        id,
        slave_address,
        index_format,
        index_value,
        data
    );

    /* Check if parameters are valid.*/
    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }


    if (!i2cp_AddressIsValid(slave_address))
    {
        return(I2C_SLAVE_ADDRESS_NOT_VALID);
    }

    /* Index transfers are only valid in case the Bus Control Mode is not slave*/
    if ((I2C_BUS_MASTER_MODE != g_i2c_system_context[id].bus_control_mode) && (I2C_NO_INDEX != index_format))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Check if not busy.*/
    if ((g_i2c_system_context[id].operation != I2C_NO_OPERATION))
    {
        return(I2C_CONTROLLER_BUSY);
    }

    /* Save parameters.*/
    g_i2c_system_context[id].slave_address = slave_address;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].register_index = index_value;
    g_i2c_system_context[id].index_format = index_format;
    g_i2c_system_context[id].data = data;
    g_i2c_system_context[id].databuffer = NULL;
    g_i2c_system_context[id].count_data = 1;
    g_i2c_system_context[id].operation = I2C_WRITE;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = FALSE;

    /* Disable all the interrupts to remove previously garbage interrupts */
    switch(id)
    {
    	case I2C0 :
    	     I2C_DisableIRQSrc(I2C0_IRQ_SRC_ALL);
    	     break;
    	case I2C1 :
    	     I2C_DisableIRQSrc(I2C1_IRQ_SRC_ALL);
    	     break;    	
    	case I2C2:
    	     I2C_DisableIRQSrc(I2C2_IRQ_SRC_ALL);    	
    	     break;
    	case I2C3:
    	     I2C_DisableIRQSrc(I2C3_IRQ_SRC_ALL);    	     
    	     break;
    	default:
    	     break;
    }
/*    I2C_DisableIRQSrc((I2C0 == id) ? I2C0_IRQ_SRC_ALL : I2C1_IRQ_SRC_ALL);*/

    /* Check if I2C controller is Master */
    if (I2C_BUS_MASTER_MODE == g_i2c_system_context[id].bus_control_mode)
    {
        /* Master control configuration  */

        /* Set the Master write operation */
        I2C_CLR_BIT(mcr, I2C_MCR_OP);

        /*  start byte procedure configuration */
        I2C_WRITE_FIELD(mcr, I2C_MCR_SB, I2C_MCR_SHIFT_SB, (u32) g_i2c_system_context[id].start_byte_procedure);

        /* Check the General call handling */
        if (g_i2c_system_context[id].general_call_mode_handling != I2C_NO_GENERAL_CALL_HANDLING)
        {
            /* The Transaction is intiated by a general call command */
            I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 0);
        }
        else
        {
            /* Check if Slave address is 10 bit */
            if (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
            {
                /* Set the Address mode to 10 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 2);
            }
            else
            {
                /* Set the Address mode to 7 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);
            }
        }

        /* Store the HS master code */
        if (I2C_FREQ_MODE_HIGH_SPEED == g_i2c_system_context[id].mode)
        {
            p_i2c_registers->hsmcr = g_i2c_system_context[id].high_speed_master_code;
        }

        /* Store  the Slave addres in the Master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A10, I2C_MCR_SHIFT_A10, slave_address);

        /* Configure the STOP condition*/
        /* Current transaction is terminated by STOP condition */
        I2C_SET_BIT(mcr, I2C_MCR_STOP);

        /* Configuring the Frame length */
        switch (index_format)
        {
            case I2C_NO_INDEX:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);
                break;

            case I2C_BYTE_INDEX:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 2);
                break;

            case I2C_HALF_WORD_LITTLE_ENDIAN:
            case I2C_HALF_WORD_BIG_ENDIAN:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 3);
                break;

            default:
                break;
        }

        /*Write the MCR register  */
        p_i2c_registers->mcr = mcr;

    }

    switch (g_i2c_system_context[id].index_transfer_mode)
    {
        case I2C_TRANSFER_MODE_POLLING:
            /*  	
          Index Transfer
          */
            if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
            {
                error_status = i2cp_SlaveIndexReceive(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }
            else
            {
                error_status = i2cp_MasterIndexTransmit(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }

            /*
          Data Transfer
          */
            switch (g_i2c_system_context[id].data_transfer_mode)
            {
                case I2C_TRANSFER_MODE_POLLING:
                    error_status = i2cp_TransmitDataPolling(id, (u8 *) &g_i2c_system_context[id].data);
                    if (I2C_OK != error_status)
                    {
                        return(error_status);
                    }

                    /* Stop Signal to be sent/received for transfer completion*/
                    break;

                case I2C_TRANSFER_MODE_INTERRUPT:
                case I2C_TRANSFER_MODE_DMA:
                default:
                    break;
            }
            break;

        case I2C_TRANSFER_MODE_INTERRUPT:
        case I2C_TRANSFER_MODE_DMA:
        default:
            return(I2C_INVALID_PARAMETER);
    }

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_WriteMultipleData                  		            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:This routine is used to write a multiple data byte to      */
/*                a receiver. Writing can be done to a slave device by      */
/*               using the indexed modes.                                   */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                u16   : The address of the slave to be accessed      */
/*                u16   : The index of the register on the receiver    */
/*                             to which data is written                     */
/*                t_unit16   :The format of the index on receiver side      */
/*                u8*   : The data buffer to be written to the         */
/*                           slave device                                   */
/*                t_unit32   : no of bytes to be transfered                 */
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_SLAVE_ADDRESS_NOT_VALID  If requested slave address   */
/*                                      is not valid                        */
/*                I2C_CONTROLLER_BUSY    if I2C controller is busy          */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_WriteMultipleData
(
    t_i2c_device_id      id,
    u16             slave_address,
    t_i2c_index_format   index_format,
    u16             index_value,
    u8              *p_data,
    u32             count
)
{

/*
Steps:
    - Check Mode
        - Polling
        - Interrupt
        - DMA
*/
    volatile u32   mcr = 0;
    t_i2c_error         error_status = I2C_OK;
    t_i2c_registers     *p_i2c_registers;
    info(
        "Id is %d, Address is %x, Index format is %d and value is %d, Data count is %d and @ is %p",
        id,
        slave_address,
        index_format,
        index_value,
        count,
        (void *) p_data
    );

    /* Check if parameters are valid.*/
    if (((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)) || (NULL == p_data))
    {
        return(I2C_INVALID_PARAMETER);
    }


    if (!i2cp_AddressIsValid(slave_address))
    {
        return(I2C_SLAVE_ADDRESS_NOT_VALID);
    }

    /* Index transfers are only valid in case the Bus Control Mode is not slave*/
    if ((I2C_BUS_MASTER_MODE != g_i2c_system_context[id].bus_control_mode) && (I2C_NO_INDEX != index_format))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Check if not busy.*/
    if ((g_i2c_system_context[id].operation != I2C_NO_OPERATION))
    {
        return(I2C_CONTROLLER_BUSY);
    }

    /* Save parameters.*/
    g_i2c_system_context[id].slave_address = slave_address;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].register_index = index_value;
    g_i2c_system_context[id].index_format = index_format;
    g_i2c_system_context[id].data = 0;
    g_i2c_system_context[id].databuffer = p_data;
    g_i2c_system_context[id].count_data = count;
    g_i2c_system_context[id].operation = I2C_WRITE;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = TRUE;

    /* Disable all the interrupts to remove previously garbage interrupts */
    switch(id)
    {
    	case I2C0 :
    	     I2C_DisableIRQSrc(I2C0_IRQ_SRC_ALL);
    	     break;
    	case I2C1 :
    	     I2C_DisableIRQSrc(I2C1_IRQ_SRC_ALL);
    	     break;    	
    	case I2C2:
    	     I2C_DisableIRQSrc(I2C2_IRQ_SRC_ALL);    	
    	     break;
    	case I2C3:
    	     I2C_DisableIRQSrc(I2C3_IRQ_SRC_ALL);    	     
    	     break;
    	default:
    	     break;
    }
    /*I2C_DisableIRQSrc((I2C0 == id) ? I2C0_IRQ_SRC_ALL : I2C1_IRQ_SRC_ALL);*/

    /* Check if I2C controller is Master */
    if (I2C_BUS_MASTER_MODE == g_i2c_system_context[id].bus_control_mode)
    {
        /* Master control configuration  */

        /* Set the Master write operation */
        I2C_CLR_BIT(mcr, I2C_MCR_OP);

        /*  start byte procedure configuration */
        I2C_WRITE_FIELD(mcr, I2C_MCR_SB, I2C_MCR_SHIFT_SB, (u32) g_i2c_system_context[id].start_byte_procedure);

        /* Check the General call handling */
        if (g_i2c_system_context[id].general_call_mode_handling != I2C_NO_GENERAL_CALL_HANDLING)
        {
            /* The Transaction is intiated by a general call command */
            I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 0);
        }
        else
        {
            /* Check if Slave address is 10 bit */
            if (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
            {
                /* Set the Address mode to 10 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 2);
            }
            else
            {
                /* Set the Address mode to 7 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);
            }
        }

        /* Store the HS master code */
        if (I2C_FREQ_MODE_HIGH_SPEED == g_i2c_system_context[id].mode)
        {
            p_i2c_registers->hsmcr = g_i2c_system_context[id].high_speed_master_code;
        }

        /* Store  the Slave addres in the Master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A10, I2C_MCR_SHIFT_A10, slave_address);

        /* Configure the STOP condition*/
        /* Current transaction is terminated by STOP condition */
        I2C_SET_BIT(mcr, I2C_MCR_STOP);

        /* Configuring the Frame length */
        switch (index_format)
        {
            case I2C_NO_INDEX:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, count);
                break;

            case I2C_BYTE_INDEX:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, (count + 1));
                break;

            case I2C_HALF_WORD_LITTLE_ENDIAN:
            case I2C_HALF_WORD_BIG_ENDIAN:
                I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, (count + 2));
                break;

            default:
                break;
        }

        p_i2c_registers->mcr = mcr;
    }

    switch (g_i2c_system_context[id].index_transfer_mode)
    {
        case I2C_TRANSFER_MODE_POLLING:
            /*  	
         Index Transfer
         */
            if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
            {
                error_status = i2cp_SlaveIndexReceive(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }
            else
            {
                error_status = i2cp_MasterIndexTransmit(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }

            /*
         Data Transfer
         */
            switch (g_i2c_system_context[id].data_transfer_mode)
            {
                case I2C_TRANSFER_MODE_POLLING:
                    error_status = i2cp_TransmitDataPolling(id, g_i2c_system_context[id].databuffer);
                    if (I2C_OK != error_status)
                    {
                        return(error_status);
                    }
                    break;

                case I2C_TRANSFER_MODE_INTERRUPT:
                case I2C_TRANSFER_MODE_DMA:
                default:
                    break;
            }
            break;

        case I2C_TRANSFER_MODE_INTERRUPT:
        case I2C_TRANSFER_MODE_DMA:
        default:
            return(I2C_INVALID_PARAMETER);
    }

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_ReadSingleData                  		            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:This routine is used to read a single  data byte from      */
/*                a transmitter. Read can be done from  a slave device by   */
/*               using the indexed modes.                                   */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                u16   : The address of the slave to be accessed      */
/*                u16   : The index of the register on the transmitter */
/*                              from which data is read                     */
/*                t_unit16   :The format of the index on tranmitter side    */
/*                u8    : The data  to be read from the tranmitter     */
/*                t_unit32   : no of bytes to be transfered                 */
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_SLAVE_ADDRESS_NOT_VALID  If requested slave address   */
/*                                      is not valid                        */
/*                I2C_CONTROLLER_BUSY    if I2C controller is busy          */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_ReadSingleData
(
    t_i2c_device_id      id,
    u16             slave_address,
    t_i2c_index_format   index_format,
    u16             index_value,
    u8              *p_data
)
{

/*
Steps:
    - Check Mode
        - Polling
        - Interrupt
        - DMA
*/
    volatile u32   mcr = 0;
    t_i2c_error         error_status = I2C_OK;
    t_i2c_registers     *p_i2c_registers;
    info(
        "Id is %d, Address is %x, Index format is %d and value is %d, Data count is %d and @ is %p",
        id,
        slave_address,
        index_format,
        index_value,
        (void *) p_data
    );

    /* Check if parameters are valid.*/
    if (((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)) || (NULL == p_data))
    {
        return(I2C_INVALID_PARAMETER);
    }

    if (!i2cp_AddressIsValid(slave_address))
    {
        return(I2C_SLAVE_ADDRESS_NOT_VALID);
    }

    /* Index transfers are only valid in case the Bus Control Mode is not slave*/
    if ((I2C_BUS_MASTER_MODE != g_i2c_system_context[id].bus_control_mode) && (I2C_NO_INDEX != index_format))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Check if not busy.*/
    if ((g_i2c_system_context[id].operation != I2C_NO_OPERATION))
    {
        return(I2C_CONTROLLER_BUSY);
    }

    /* Save parameters.*/
    g_i2c_system_context[id].slave_address = slave_address;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].register_index = index_value;
    g_i2c_system_context[id].index_format = index_format;
    g_i2c_system_context[id].data = 0;
    g_i2c_system_context[id].databuffer = p_data;
    g_i2c_system_context[id].count_data = 1;
    g_i2c_system_context[id].operation = I2C_READ;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = FALSE;

    /* Disable all the interrupts to remove previously garbage interrupts */
    switch(id)
    {
    	case I2C0 :
    	     I2C_DisableIRQSrc(I2C0_IRQ_SRC_ALL);
    	     break;
    	case I2C1 :
    	     I2C_DisableIRQSrc(I2C1_IRQ_SRC_ALL);
    	     break;    	
    	case I2C2:
    	     I2C_DisableIRQSrc(I2C2_IRQ_SRC_ALL);    	
    	     break;
    	case I2C3:
    	     I2C_DisableIRQSrc(I2C3_IRQ_SRC_ALL);    	     
    	     break;
    	default:
    	     break;
    }
    
    /*I2C_DisableIRQSrc((id == I2C0) ? I2C0_IRQ_SRC_ALL : I2C1_IRQ_SRC_ALL);*/

    /* Check if I2C controller is Master */
    if (I2C_BUS_MASTER_MODE == g_i2c_system_context[id].bus_control_mode)
    {

        /*  start byte procedure configuration */
        I2C_WRITE_FIELD(mcr, I2C_MCR_SB, I2C_MCR_SHIFT_SB, (u32) g_i2c_system_context[id].start_byte_procedure);

        /* Check the General call handling */
        if (g_i2c_system_context[id].general_call_mode_handling != I2C_NO_GENERAL_CALL_HANDLING)
        {
            /* The Transaction is intiated by a general call command */
            I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 0);
        }
        else
        {
            /* Check if Slave address is 10 bit */
            if (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
            {
                /* Set the Address mode to 10 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 2);
            }
            else
            {
                /* Set the Address mode to 7 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);
            }
        }

        /* Store the HS master code */
        if (I2C_FREQ_MODE_HIGH_SPEED == g_i2c_system_context[id].mode)
        {
            p_i2c_registers->hsmcr = g_i2c_system_context[id].high_speed_master_code;
        }

        /* Store  the Slave addres in the Master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A10, I2C_MCR_SHIFT_A10, slave_address);

        if
        (
            (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
        &&  (I2C_NO_INDEX == index_format)
        )
        {
            /* Set the Master write operation */
            I2C_CLR_BIT(mcr, I2C_MCR_OP);

            I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 0);

            /* Current transaction is not terminated by STOP condition,
                 a repeated start operation will be fallowed */
            I2C_CLR_BIT(mcr, I2C_MCR_STOP);

            /*Write MCR register   */
            p_i2c_registers->mcr = mcr;

            /* Enable the I2C controller */
            I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);
        }
        else
        {
            /* Master control configuration  */
            if (I2C_NO_INDEX != index_format)
            {
                /* Set the Master write operation */
                I2C_CLR_BIT(mcr, I2C_MCR_OP);
            }
            else
            {
                /* Set the Master read operation */
                I2C_SET_BIT(mcr, I2C_MCR_OP);
            }

            /* Configuring the Frame length */
            switch (index_format)
            {
                case I2C_NO_INDEX:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

                    /* Current transaction is terminated by STOP condition */
                    I2C_SET_BIT(mcr, I2C_MCR_STOP);
                    break;

                case I2C_BYTE_INDEX:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

                    /* Current transaction is not terminated by STOP condition,
	                 a repeated start operation will be fallowed */
                    I2C_CLR_BIT(mcr, I2C_MCR_STOP);
                    break;

                case I2C_HALF_WORD_LITTLE_ENDIAN:
                case I2C_HALF_WORD_BIG_ENDIAN:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 2);

                    /* Current transaction is not terminated by STOP condition,
	                a repeated start operation will be fallowed */
                    I2C_CLR_BIT(mcr, I2C_MCR_STOP);
                    break;

                default:
                    break;
            }

            /*Write MCR register   */
            p_i2c_registers->mcr = mcr;

        }
    }

    switch (g_i2c_system_context[id].index_transfer_mode)
    {
        case I2C_TRANSFER_MODE_POLLING:
            /*  	
         Index Transfer
         */
            if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
            {
                error_status = i2cp_SlaveIndexReceive(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }
            else
            {
                error_status = i2cp_MasterIndexTransmit(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }

            /*
         Data Transfer
         */
            switch (g_i2c_system_context[id].data_transfer_mode)
            {
                case I2C_TRANSFER_MODE_POLLING:
                    error_status = i2cp_ReceiveDataPolling(id, g_i2c_system_context[id].databuffer);
                    if (I2C_OK != error_status)
                    {
                        return(error_status);
                    }

                    /* Stop Signal to be sent/received for transfer completion*/
                    break;

                case I2C_TRANSFER_MODE_INTERRUPT:
                case I2C_TRANSFER_MODE_DMA:
                default:
                    break;
            }
            break;

        case I2C_TRANSFER_MODE_INTERRUPT:
        case I2C_TRANSFER_MODE_DMA:
        default:
            return(I2C_INVALID_PARAMETER);
    }

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_ReadMultipleData                  		            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:This routine is used to read a multiple  data byte from    */
/*                a transmitter. Read can be done from  a slave device by   */
/*               using the indexed modes.                                   */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be initialized		*/
/*                u16   : The address of the slave to be accessed      */
/*                u16   : The index of the register on the transmitter */
/*                              from which data is read                     */
/*                t_unit16   :The format of the index on tranmitter side    */
/*                u8*   : The data buffer to be written to the         */
/*                           slave device                                   */
/*                t_unit32   : no of bytes to be transfered                 */
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*                I2C_SLAVE_ADDRESS_NOT_VALID  If requested slave address   */
/*                                      is not valid                        */
/*                I2C_CONTROLLER_BUSY    if I2C controller is busy          */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_ReadMultipleData
(
    t_i2c_device_id      id,
    u16             slave_address,
    t_i2c_index_format   index_format,
    u16             index_value,
    u8              *p_data,
    u32             count
)
{

/*
Steps:
    - Check Mode
        - Polling
        - Interrupt
        - DMA
*/
    volatile u32   mcr = 0;
    t_i2c_error         error_status = I2C_OK;
    t_i2c_registers     *p_i2c_registers;
    info(
        "Id is %d, Address is %x, Index format is %d and value is %d, Data count is %d and @ is %p",
        id,
        slave_address,
        index_format,
        index_value,
        count,
        (void *) p_data
    );

    /* Check if parameters are valid.*/
    if (((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id) ) || (NULL == p_data))
    {
        return(I2C_INVALID_PARAMETER);
    }
    

    if (!i2cp_AddressIsValid(slave_address))
    {
        return(I2C_SLAVE_ADDRESS_NOT_VALID);
    }

    /* Index transfers are only valid in case the Bus Control Mode is not slave*/
    if ((I2C_BUS_MASTER_MODE != g_i2c_system_context[id].bus_control_mode) && (I2C_NO_INDEX != index_format))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Check if not busy.*/
    if ((g_i2c_system_context[id].operation != I2C_NO_OPERATION))
    {
        return(I2C_CONTROLLER_BUSY);
    }

    /* Save parameters.*/
    g_i2c_system_context[id].slave_address = slave_address;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].register_index = index_value;
    g_i2c_system_context[id].index_format = index_format;
    g_i2c_system_context[id].data = 0;
    g_i2c_system_context[id].databuffer = p_data;
    g_i2c_system_context[id].count_data = count;
    g_i2c_system_context[id].operation = I2C_READ;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = TRUE;

    /* Disable all the interrupts to remove previously garbage interrupts */
    switch(id)
    {
    	case I2C0 :
    	     I2C_DisableIRQSrc(I2C0_IRQ_SRC_ALL);
    	     break;
    	case I2C1 :
    	     I2C_DisableIRQSrc(I2C1_IRQ_SRC_ALL);
    	     break;    	
    	case I2C2:
    	     I2C_DisableIRQSrc(I2C2_IRQ_SRC_ALL);    	
    	     break;
    	case I2C3:
    	     I2C_DisableIRQSrc(I2C3_IRQ_SRC_ALL);    	     
    	     break;
    	default:
    	     break;
    }
/*    I2C_DisableIRQSrc((I2C0 == id) ? I2C0_IRQ_SRC_ALL : I2C1_IRQ_SRC_ALL);*/

    /* Check if I2C controller is Master */
    if (I2C_BUS_MASTER_MODE == g_i2c_system_context[id].bus_control_mode)
    {

        /*  start byte procedure configuration */
        I2C_WRITE_FIELD(mcr, I2C_MCR_SB, I2C_MCR_SHIFT_SB, (u32) g_i2c_system_context[id].start_byte_procedure);

        /* Check the General call handling */
        if (g_i2c_system_context[id].general_call_mode_handling != I2C_NO_GENERAL_CALL_HANDLING)
        {
            /* The Transaction is intiated by a general call command */
            I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 0);
        }
        else
        {
            /* Check if Slave address is 10 bit */
            if (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
            {
                /* Set the Address mode to 10 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 2);
            }
            else
            {
                /* Set the Address mode to 7 bit */
                I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);
            }
        }

        /* Store the HS master code */
        if (I2C_FREQ_MODE_HIGH_SPEED == g_i2c_system_context[id].mode)
        {
            p_i2c_registers->hsmcr = g_i2c_system_context[id].high_speed_master_code;
        }

        /* Store  the Slave addres in the Master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A10, I2C_MCR_SHIFT_A10, slave_address);

        if
        (
            (g_i2c_system_context[id].slave_address < 1024 && g_i2c_system_context[id].slave_address > 127)
        &&  (I2C_NO_INDEX == index_format)
        )
        {
            /* Set the Master write operation */
            I2C_CLR_BIT(mcr, I2C_MCR_OP);

            I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 0);

            /* Current transaction is not terminated by STOP condition,
                 a repeated start operation will be fallowed */
            I2C_CLR_BIT(mcr, I2C_MCR_STOP);

            /*Write MCR register   */
            p_i2c_registers->mcr = mcr;

            /* Enable the I2C controller */
            I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);
        }
        else
        {
            /* Master control configuration  */
            if (I2C_NO_INDEX != index_format)
            {
                /* Set the Master write operation */
                I2C_CLR_BIT(mcr, I2C_MCR_OP);
            }
            else
            {
                /* Set the Master read operation */
                I2C_SET_BIT(mcr, I2C_MCR_OP);
            }

            /* Configuring the Frame length */
            switch (index_format)
            {
                case I2C_NO_INDEX:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, count);

                    /* Current transaction is terminated by STOP condition */
                    I2C_SET_BIT(mcr, I2C_MCR_STOP);
                    break;

                case I2C_BYTE_INDEX:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

                    /* Current transaction is not terminated by STOP condition,
	                 a repeated start operation will be fallowed */
                    I2C_CLR_BIT(mcr, I2C_MCR_STOP);
                    break;

                case I2C_HALF_WORD_LITTLE_ENDIAN:
                case I2C_HALF_WORD_BIG_ENDIAN:
                    I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 2);

                    /* Current transaction is not terminated by STOP condition,
	                a repeated start operation will be fallowed */
                    I2C_CLR_BIT(mcr, I2C_MCR_STOP);
                    break;

                default:
                    break;
            }

            /*Write MCR register   */
            p_i2c_registers->mcr = mcr;

        }
    }

    switch (g_i2c_system_context[id].index_transfer_mode)
    {
        case I2C_TRANSFER_MODE_POLLING:
            /*  	
          Index Transfer
          */
            if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
            {
                error_status = i2cp_SlaveIndexReceive(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }
            else
            {
                error_status = i2cp_MasterIndexTransmit(id);
                if (I2C_OK != error_status)
                {
                    return(error_status);
                }
            }

            /*
          Data Transfer
          */
            switch (g_i2c_system_context[id].data_transfer_mode)
            {
                case I2C_TRANSFER_MODE_POLLING:
                    error_status = i2cp_ReceiveDataPolling(id, g_i2c_system_context[id].databuffer);
                    if (I2C_OK != error_status)
                    {
                        return(error_status);
                    }
                    break;

                case I2C_TRANSFER_MODE_INTERRUPT:
                case I2C_TRANSFER_MODE_DMA:
                default:
                    break;
            }
            break;

        case I2C_TRANSFER_MODE_INTERRUPT:
        case I2C_TRANSFER_MODE_DMA:
        default:
            return(I2C_INVALID_PARAMETER);
    }

    return(error_status);

}

/****************************************************************************/
/* NAME			:	I2C_Cancel                  			                */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	This routine is used to cancel the current transfer     */
/*                  operation, if any.                                      */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller to be canceled  		*/
/*     InOut    :  None                                                     */
/* 			: t_i2c_active_event: It will contain the result of         */
/*                                    the operation                         */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*                I2C_OK                 if it is ok                        */
/*                I2C_INVALID_PARAMETER  if input parameters are not valid  */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_i2c_error I2C_Cancel(t_i2c_device_id id, t_i2c_active_event *event)   /*Only IT mode*/
{
    /* Check if parameters are valid.*/
    if ((NULL == event) || ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id)))
    {
        return(I2C_INVALID_PARAMETER);
    }

    if (I2C_NO_EVENT == g_i2c_system_context[id].active_event)
    {
        event->type = I2C_NO_EVENT;
        event->transfer_data = 0;
        event->id = id;
    }
    else
    {
        event->type = I2C_CANCEL_EVENT;
        event->transfer_data = g_i2c_system_context[id].transfer_data;
        event->id = id;
        g_i2c_system_context[id].active_event = I2C_CANCEL_EVENT;
    }

    i2cp_Abort(id);

    /*Set the I2C operation to No operation  */
    g_i2c_system_context[id].operation = (t_i2c_operation) I2C_NO_OPERATION;

    return(I2C_OK);
}



/****************************************************************************/
/* NAME			:	I2C_IsEventActive                  		                */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:This routine is used to determine if the given event       */
/*                is still active.                                          */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_active_event: the Event to be checked      		    */
/*     InOut    : None                                                      */
/* 			: None                                                      */
/*                                                                          */
/* RETURN		:	t_bool  :   TRUE or FALSE               	            */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
 t_bool I2C_IsEventActive(t_i2c_active_event *event)
{
    if (event->type == g_i2c_system_context[event->id].active_event)
    {
        return(TRUE);
    }
    else
    {
        return(FALSE);
    }
}


/****************************************************************************/
/* NAME			:	I2C_Reset                           		            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	:Reset the I2C Registers for given I2C controller           */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The ID of the controller for reset    */
/*     InOut    :  None                                                     */
/* 			:  None														*/
/*                                                                          */
/* RETURN		:	t_i2c_error  									    	*/
/*                  return  I2C_INVALID_PARAMETER  if id is not correct     */
/*                  else    I2C_OK                                          */
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/
/****************************************************************************/
t_i2c_error I2C_Reset(t_i2c_device_id id)
{
    t_i2c_registers *p_i2c_registers;

    /* Check if parameters are valid.*/
    if ((I2C0 != id) && (I2C1 != id) && (I2C2 != id) && (I2C3 != id))
    {
        return(I2C_INVALID_PARAMETER);
    }

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;


    /* Clear registers.*/
    I2C_WRITE_REG(p_i2c_registers->cr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->scr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->hsmcr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->mcr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->tftr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->rftr, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->dmar, I2C_CLEAR);
    I2C_WRITE_REG(p_i2c_registers->icr, 0x31F0008);
    I2C_WRITE_REG(p_i2c_registers->imscr,I2C_CLEAR);

    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FTX);       /* Flush the Tx Fifo */
    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FRX);       /* Flush the Rx Fifo */

    /*
	Initialize the right structure to default state 
	*/
    g_i2c_system_context[id].freq_scl = 0;
    g_i2c_system_context[id].freq_input = 0;
    g_i2c_system_context[id].mode = I2C_FREQ_MODE_STANDARD;
    g_i2c_system_context[id].own_address = 0;
    g_i2c_system_context[id].enabled = FALSE;
    g_i2c_system_context[id].slave_address = 0;
    g_i2c_system_context[id].status = I2C_STATUS_SLAVE_MODE;
    g_i2c_system_context[id].data = 0;
    g_i2c_system_context[id].databuffer = NULL;
    g_i2c_system_context[id].count_data = 0;
    g_i2c_system_context[id].register_index = 0;
    g_i2c_system_context[id].operation = (t_i2c_operation) I2C_NO_OPERATION;
    g_i2c_system_context[id].active_event = I2C_NO_EVENT;
    g_i2c_system_context[id].transfer_data = 0;
    g_i2c_system_context[id].multi_operation = FALSE;

    /*   g_i2c_system_context[id].i2c_device_context... to be initialized*/
    g_i2c_system_context[id].digital_filter_control = I2C_DIGITAL_FILTERS_OFF;
    g_i2c_system_context[id].dma_sync_logic_control = I2C_DISABLE;
    g_i2c_system_context[id].start_byte_procedure = I2C_DISABLE;
    g_i2c_system_context[id].slave_data_setup_time = 0; /* TBD */
    g_i2c_system_context[id].high_speed_master_code = 0;
    g_i2c_system_context[id].bus_control_mode = I2C_BUS_SLAVE_MODE;
    g_i2c_system_context[id].i2c_loopback_mode = I2C_DISABLE;
    g_i2c_system_context[id].general_call_mode_handling = I2C_NO_GENERAL_CALL_HANDLING;

    g_i2c_system_context[id].index_transfer_mode = I2C_TRANSFER_MODE_POLLING;
    g_i2c_system_context[id].data_transfer_mode = I2C_TRANSFER_MODE_POLLING;
    g_i2c_system_context[id].i2c_transmit_interrupt_threshold = 1;
    g_i2c_system_context[id].i2c_receive_interrupt_threshold = 1;
    g_i2c_system_context[id].transmit_burst_length = 0;
    g_i2c_system_context[id].receive_burst_length = 0;
    g_i2c_system_context[id].index_format = I2C_NO_INDEX;
    g_i2c_system_context[id].current_bus_config = I2C_CURRENT_BUS_SLAVE_TRANSMITTER;
    g_i2c_system_context[id].std =FALSE;

    return(I2C_OK);
}

/*-----------------------------------------------------------------------------
		Private functions
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
Name		: i2cp_SetOwnAddress
Description	: Set the I2C bus address of the controller.
In			: t_i2c_registers* p_i2c_registers	: pointer to the controller's 
											  registers.
			  u32 address				: the slave address of the 
			  								  controller.
InOut		: None
Out			: None
Return value: Always ok / I2C_SLAVE_ADDRESS_NOT_VALID
Type		: Private
Comments	: 


	In all cases, the bits are not cleared when the interface is disabled 
	(PE = 0b).
-----------------------------------------------------------------------------*/
 t_i2c_error i2cp_SetOwnAddress(t_i2c_device_id id, u16 address)
{
    t_i2c_registers *p_i2c_registers;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* CM: check if the given address is valid ???*/
    if (!i2cp_AddressIsValid(address))
    {
        return(I2C_SLAVE_ADDRESS_NOT_VALID);
    }

    if (address < 1024 && address > 127)
    {
        /* Set Slave address mode to 10 bit addressing mode */
        I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_SAM);
        I2C_WRITE_FIELD(p_i2c_registers->scr, I2C_SCR_ADDR, I2C_SCR_SHIFT_ADDR, address);
    }
    else
    {
        /* Set the slave address mode to 7 bit addressing mode */
        I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_SAM);
        I2C_WRITE_FIELD(p_i2c_registers->scr, I2C_SCR_ADDR, I2C_SCR_SHIFT_ADDR, address);
    }

    return(I2C_OK);
}

/*-----------------------------------------------------------------------------
Name		: i2cp_SetBusClock
Description	: Set the I2C bus clock for the given controller.
In			: t_i2c_registers* p_i2c_registers	: pointer to the controller's 
											  registers.
			  u32 freq_scl					: the I2C bus frequency freq_scl (Hz).
			  u32 freq_input					: the input clock frequency (Hz).
InOut		: None
Out			: None
Return value: I2C_OK						: no error.
			  I2C_INVALID_PARAMETER			: wrong id parameter.
			  I2C_freq_scl_NOT_SUPPORTED		: freq_scl is not supported.
Type		: Private
Comments	: The freq_input parameter is only necessary to calculate the I2C bus 
			  frequency and is not used for other purposes.
			  It is not necessary to save the freq_scl as it has been already 
			  saved by I2C_Config().
	
-----------------------------------------------------------------------------*/
t_i2c_error i2cp_SetBusClock(t_i2c_device_id id, u32 freq_scl, u32 freq_input)
{


    /* To be defined */
    u32        value;
    t_i2c_registers *p_i2c_registers;
    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    /* Standard mode */
    if (freq_scl <= (u32) I2C_MAX_STANDARD_SCL)
    {
        value = (u32) (freq_input / (freq_scl * 2));

        /*Set the Standard mode in the control register  */
        I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_SM, I2C_CR_SHIFT_SM, 0x0);

        /* set the Baud rate counter 2 value  */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT2, I2C_BRCR_SHIFT_BRCNT2, value);

        /* Make ensure that BRCNT value set to be zero */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT1, I2C_BRCR_SHIFT_BRCNT1, 0);

        /*Update the Frequency mode in the global strcture */
        g_i2c_system_context[id].mode = I2C_FREQ_MODE_STANDARD;
    }
    else    /* Fast Mode */
    if (freq_scl <= (u32) I2C_MAX_FAST_SCL)
    {
        value = (u32) (freq_input / ((freq_scl * 3) / 2));

        /*Set the Fast mode in the control register  */
        I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_SM, I2C_CR_SHIFT_SM, 0x1);

        /* set the Baud rate counter 2 value  */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT2, I2C_BRCR_SHIFT_BRCNT2, value);

        /* Make ensure that BRCNT value set to be zero */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT1, I2C_BRCR_SHIFT_BRCNT1, 0);

        /*Update the Frequency mode in the global strcture */
        g_i2c_system_context[id].mode = I2C_FREQ_MODE_FAST;
    }
    else    /* High Speed Mode  */
    if (freq_scl <= (u32) I2C_MAX_HIGH_SPEED_SCL)
    {
        value = (u32) (freq_input / ((freq_scl * 3) / 2));

        /*Set the High speed mode in the control register  */
        I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_SM, I2C_CR_SHIFT_SM, 0x2);

        /* set the Baud rate counter 1 value  */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT1, I2C_BRCR_SHIFT_BRCNT1, value);

        /*Update the Frequency mode in the global strcture */
        g_i2c_system_context[id].mode = I2C_FREQ_MODE_HIGH_SPEED;
    }
    else
    {
        return(I2C_LINE_FREQ_NOT_SUPPORTED);
    }

    return(I2C_OK);

}

/*-----------------------------------------------------------------------------
Name		: i2cp_AddressIsValid
Description	: Check if the given address is valid.
In			: u16 address	: the slave address to be checked.
InOut		: None
Out			: None
Return value: TRUE				: address is valid.
			  FALSE				: address is not valid.
Type		: Private
Comments	: Note that the least-significant bit of the address parameter 
			  is not relevant for the addressing of the slave device, for 
			  example 0xE2 and 0xE3 will address the same slave device.


  Reserved addresses:
	SLAVE ADDRESS	R/W BIT		RANGE		DESCRIPTION
	0000 000		0			0			General call address
	0000 000		1			1			START byte(1)
	0000 001		X			2-3			CBUS address(2)
	0000 010		X			4-5			Reserved for different bus format(3)
	0000 011		X			6-7			Reserved for future purposes
	0000 1XX		X			8-15		Hs-mode master code
	
	1111 1XX		X			248-255		Reserved for future purposes
	1111 0XX		X			240-247		10-bit slave addressing

  Note that with 7-bit address:
	0000xxxx and 1111xxxx are reserved.  
-----------------------------------------------------------------------------*/
t_bool i2cp_AddressIsValid(u16 address)
{
    /* Check if more than 10 bits are needed.*/
    if (address > 1023)
    {
        return(FALSE);
    }

    /* 7-bit address. LSB is not considered.*/
    /*Address 0x4 is enabled to support the ST Pepper pot camera  */
    if (address < 128 && !(address == 0 || address == 0x4))
    {
        if ((address < 8) || (address > 119))
        {
            return(FALSE);
        }
    }

    /* CM: add here the 10-bit check.*/
    return(TRUE);
}


/*-----------------------------------------------------------------------------
Name		: i2cp_Abort
Description	: Abort the current transfer operation of the given controller.
In			: t_i2c_device_id id	: the controller to be aborted.
InOut		: None
Out			: None
Return		: I2C_OK		: always no error.
Type		: Private
Comments	: This is called when an unexpected event happens (internal error). 
-----------------------------------------------------------------------------*/
void i2cp_Abort(t_i2c_device_id id)
{

    t_i2c_registers *p_i2c_registers;
    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;
    /*Disable the interrupts  */
    I2C_WRITE_REG(p_i2c_registers->imscr,I2C_CLEAR);

    /*Disable the Controller  */
    I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

    /*Enable the controller */
    I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);

    /*Set the I2C operation to No operation  */
    g_i2c_system_context[id].operation = (t_i2c_operation) I2C_NO_OPERATION;
}



/*-----------------------------------------------------------------------------
Name		: i2cp_SlaveIndexReceive
Description	: 
In			: t_i2c_id              : I2C Controller id 

InOut		: None
Out			: t_i2c_error          error status
Return value: I2C_OK						: no error
			  I2C_INVALID_PARAMETER			: wrong id parameter.

Type		: Private
Comments	: This  function  perform the operations, when 
              I2C controller addressed as a slave 
-----------------------------------------------------------------------------*/
t_i2c_error i2cp_SlaveIndexReceive(t_i2c_device_id id)
{

    u32        loop_counter = 0;
    t_i2c_registers *p_i2c_registers;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    if (I2C_WRITE == g_i2c_system_context[id].operation)
    {
        /* SLAVE TRANSMITTER  */
        /* Waiting for the Read from slave request */
        loop_counter = 0;
        while
        (
            (!I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_RFSR, I2C_INT_SHIFT_RFSR))
        &&  loop_counter < I2C_ENDAD_COUNTER
        )
        {
            loop_counter++;
        };
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            i2cp_Abort(id);
            return(I2C_ADDRESS_MATCH_FAILED);
        }

        /* Acknowledge the Read from slave request */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_RFSR);

        /* Read from slave request recieved */
        /* Flush the Tx Fifo */
        I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FTX);

        /*Wait till for the Tx Flush bit to reset */
        loop_counter = 0;
        while
        (
            I2C_READ_FIELD(p_i2c_registers->cr, I2C_CR_FTX, I2C_CR_SHIFT_FTX)
        &&  loop_counter < I2C_FIFO_FLUSH_COUNTER
        )
        {
            loop_counter++;
        };
        if (loop_counter >= I2C_FIFO_FLUSH_COUNTER)
        {
            return(I2C_HW_FAILED);
        }

        /* update the status */
        g_i2c_system_context[id].status = I2C_STATUS_SLAVE_TRANSMITTER_MODE;
        g_i2c_system_context[id].active_event = I2C_READ_FROM_SLAVE_REQUEST_EVENT;
        g_i2c_system_context[id].current_bus_config = I2C_CURRENT_BUS_SLAVE_TRANSMITTER;

    }
    else
    {
        /* SLAVE RECEIVER  */
        /* Waiting for the Write to slave request */
        loop_counter = 0;
        while
        (
            (!I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_WTSR, I2C_INT_SHIFT_WTSR))
        &&  loop_counter < I2C_ENDAD_COUNTER
        )
        {
            loop_counter++;
        };
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            i2cp_Abort(id);
            return(I2C_ADDRESS_MATCH_FAILED);
        }

        /* Acknowledge the Write to slave request */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_WTSR);

        /* update the status */
        g_i2c_system_context[id].status = I2C_STATUS_SLAVE_TRANSMITTER_MODE;
        g_i2c_system_context[id].active_event = I2C_WRITE_TO_SLAVE_REQUEST_EVENT;
        g_i2c_system_context[id].current_bus_config = I2C_CURRENT_BUS_SLAVE_RECEIVER;

    }

    /* Update the status of the I2C controller  */
    if (I2C_READ == g_i2c_system_context[id].operation)
    {
        g_i2c_system_context[id].status = I2C_STATUS_SLAVE_RECEIVER_MODE;
    }
    else
    {
        g_i2c_system_context[id].status = I2C_STATUS_SLAVE_TRANSMITTER_MODE;
    }

    return(I2C_OK);
}

/*-----------------------------------------------------------------------------
Name		: i2cp_TransmitDataPolling
Description	: Transmit the data in the polling mode 
In			: t_i2c_id              : I2C Controller id 

InOut		: None
Out			: t_i2c_error          error status
Return value: I2C_OK						: no error
			  I2C_INVALID_PARAMETER			: wrong id parameter.

Type		: Private
Comments	: 
-----------------------------------------------------------------------------*/
t_i2c_error i2cp_TransmitDataPolling(t_i2c_device_id id, volatile u8 *p_data)
{

    u32        loop_counter = 0;
    t_i2c_registers *p_i2c_registers;
    t_i2c_error     error_status;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
    {
        /* Slave tranmitter */
        while (g_i2c_system_context[id].count_data != 0)
        {
            /* Check for Tx Fifo not full */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_TXFF, I2C_INT_SHIFT_TXFF)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    return(I2C_TRANSMIT_FIFO_FULL);
                }
            }

            p_i2c_registers->tfr = *p_data;

            g_i2c_system_context[id].transfer_data++;
            g_i2c_system_context[id].count_data--;
            p_data++;
            g_i2c_system_context[id].active_event = I2C_DATA_TX_EVENT;
        }

        /* End of Data transfer */
        /* Check for the Slave tranaction done */
        loop_counter = 0;
        while
        (
            !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_STD, I2C_INT_SHIFT_STD)
        &&  loop_counter < I2C_ENDAD_COUNTER
        )
        {
            loop_counter++;
        };
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            error_status = i2cp_GetAbortCause(id);
            if (error_status != I2C_OK)
            {
                return(error_status);
            }
            else
            {
                i2cp_Abort(id);
            }

            return(I2C_INTERNAL_ERROR);
        }

        /* Slave Transaction has been done */
        /* Acknowledge the Slave Transaction done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_STD);

        g_i2c_system_context[id].active_event = I2C_TRANSFER_OK_EVENT;
        return(I2C_OK);
    }
    else
    {
        /* Master Transmitter */
        while (g_i2c_system_context[id].count_data != 0)
        {
            /* Check for Tx Fifo not full */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_TXFF, I2C_INT_SHIFT_TXFF)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_TRANSMIT_FIFO_FULL);
            }

            p_i2c_registers->tfr = *p_data;

            g_i2c_system_context[id].transfer_data++;
            g_i2c_system_context[id].count_data--;
            p_data++;
            g_i2c_system_context[id].active_event = I2C_DATA_TX_EVENT;
        }

        /* End of Data transfer */
        
        loop_counter = 0;
        /* Check whether the Stop bit has been programmed or not */
        if(I2C_READ_FIELD(p_i2c_registers->mcr, I2C_MCR_STOP, I2C_MCR_SHIFT_STOP))
        {
            /* Check for the Master transaction Done */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTD, I2C_INT_SHIFT_MTD)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        else
        {
        	/* Check for the Master transaction Done Without Stop */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTDWS, I2C_INT_SHIFT_MTDWS)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        
        
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            error_status = i2cp_GetAbortCause(id);
            if (error_status != I2C_OK)
            {
                i2cp_Abort(id);
                return(error_status);
            }
            else
            {
                i2cp_Abort(id);
            }

            return(I2C_INTERNAL_ERROR);
        }
        
        
        /* Master Transaction has been done */
        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);
        
        /* Master Transaction Without Stop has been done */
        /* Acknowledge the Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);
        
        
        g_i2c_system_context[id].active_event = I2C_TRANSFER_OK_EVENT;

        g_i2c_system_context[id].operation = I2C_NO_OPERATION;
        return(I2C_OK);
    }

}

/*-----------------------------------------------------------------------------
Name		: i2cp_ReceiveDataPolling
Description	: Receiving the data in polling mode
In			: t_i2c_id              : I2C Controller id 

InOut		: None
Out			: t_i2c_error          error status
Return value: I2C_OK						: no error
			  I2C_WRONG_PARAMETER			: wrong id parameter.

Type		: Private
Comments	: 
-----------------------------------------------------------------------------*/
t_i2c_error i2cp_ReceiveDataPolling(t_i2c_device_id id, u8 *p_data)
{

    u32        loop_counter = 0;
    t_i2c_error     error_status;
    t_i2c_registers *p_i2c_registers;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    if (I2C_BUS_SLAVE_MODE == g_i2c_system_context[id].bus_control_mode)
    {
        /* Slave Receiver */
        while (g_i2c_system_context[id].count_data != 0)
        {
            /* Wait for the Rx Fifo  empty */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_RXFE, I2C_INT_SHIFT_RXFE)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_RECEIVE_FIFO_EMPTY);
            }

            /* Read the data byte from Rx Fifo */
            *p_data = (u8) p_i2c_registers->rfr;

            g_i2c_system_context[id].transfer_data++;
            g_i2c_system_context[id].count_data--;
            p_data++;
            g_i2c_system_context[id].active_event = I2C_DATA_RX_EVENT;
        }   /* Data Reception has been completed */

        /* Check for the slave transaction done */
        loop_counter = 0;
        while
        (
            !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_STD, I2C_INT_SHIFT_STD)
        &&  loop_counter < I2C_ENDAD_COUNTER
        )
        {
            loop_counter++;
        };
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            error_status = i2cp_GetAbortCause(id);
            if (error_status != I2C_OK)
            {
                return(error_status);
            }
            else
            {
                i2cp_Abort(id);
            }

            return(I2C_INTERNAL_ERROR);
        }

        /* Slave Transaction has been done */
        /* Acknowledge the Slave Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_STD);
        g_i2c_system_context[id].active_event = I2C_TRANSFER_OK_EVENT;
        g_i2c_system_context[id].operation = I2C_NO_OPERATION;

        return(I2C_OK);
    }
    else
    {
        /* Master Receiver */
        while (g_i2c_system_context[id].count_data != 0)
        {
            /* Wait for the Rx Fifo  empty */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_RXFE, I2C_INT_SHIFT_RXFE)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_RECEIVE_FIFO_EMPTY);
            }

            /* Read the data byte from Rx Fifo */
            *p_data = (u8) p_i2c_registers->rfr;

            g_i2c_system_context[id].transfer_data++;
            g_i2c_system_context[id].count_data--;
            p_data++;
            g_i2c_system_context[id].active_event = I2C_DATA_RX_EVENT;
        }   /* Data reception  has been completed */

        loop_counter = 0;
        /* Check whether the Stop bit has been programmed or not */
        if(I2C_READ_FIELD(p_i2c_registers->mcr, I2C_MCR_STOP, I2C_MCR_SHIFT_STOP))
        {
            /* Check for the Master transaction Done */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTD, I2C_INT_SHIFT_MTD)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        else
        {
        	/* Check for the Master transaction Done Without Stop */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTDWS, I2C_INT_SHIFT_MTDWS)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            error_status = i2cp_GetAbortCause(id);
            if (error_status != I2C_OK)
            {
                return(error_status);
            }
            else
            {
                i2cp_Abort(id);
            }

            return(I2C_INTERNAL_ERROR);
        }

        /* Master Transaction has been done */
        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);
        
        /* Master Transaction Without Stop has been done */
        /* Acknowledge the Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

        g_i2c_system_context[id].active_event = I2C_TRANSFER_OK_EVENT;
        g_i2c_system_context[id].operation = I2C_NO_OPERATION;

    }

    return(I2C_OK);
}

/*-----------------------------------------------------------------------------
Name		: i2cp_MasterIndexTransmit
Description	: Transmits the index to slave 
In			: t_i2c_id              : I2C Controller id 

InOut		: None
Out			: t_i2c_error          error status
Return value: I2C_OK						: no error
			  I2C_WRONG_PARAMETER			: wrong id parameter.

Type		: Private
Comments	: 
-----------------------------------------------------------------------------*/
t_i2c_error i2cp_MasterIndexTransmit(t_i2c_device_id id)
{

    volatile u32   mcr = 0;
    u32            loop_counter = 0;
    t_i2c_error         error_status = I2C_OK;
    t_i2c_registers     *p_i2c_registers;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    switch (g_i2c_system_context[id].index_format)
    {
        case I2C_NO_INDEX:
            if (g_i2c_system_context[id].slave_address <= 127)
            {  
               return(I2C_OK);
            }
            
            break;

        case I2C_BYTE_INDEX:
            /* Checking for the Tx fifo not full */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_TXFF, I2C_INT_SHIFT_TXFF)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_TRANSMIT_FIFO_FULL);
            }

            p_i2c_registers->tfr = (0xFF & g_i2c_system_context[id].register_index);

            g_i2c_system_context[id].active_event = I2C_INDEX_TX_EVENT;
            g_i2c_system_context[id].index_format = I2C_NO_INDEX;

            break;

        case I2C_HALF_WORD_LITTLE_ENDIAN:
            /* Checking for the Tx Fifo not full  */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_TXFF, I2C_INT_SHIFT_TXFF)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_TRANSMIT_FIFO_FULL);
            }

            p_i2c_registers->tfr = (0xFF & (u32) g_i2c_system_context[id].register_index);

            p_i2c_registers->tfr = (g_i2c_system_context[id].register_index >> 8);

            g_i2c_system_context[id].index_format = I2C_NO_INDEX;
            g_i2c_system_context[id].active_event = I2C_INDEX_TX_EVENT;
            break;

        case I2C_HALF_WORD_BIG_ENDIAN:
            /* Cheking for the Tx Fifo full */
            loop_counter = 0;
            while
            (
                I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_TXFF, I2C_INT_SHIFT_TXFF)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
            if (loop_counter >= I2C_ENDAD_COUNTER)
            {
                error_status = i2cp_GetAbortCause(id);
                if (error_status != I2C_OK)
                {
                    return(error_status);
                }
                else
                {
                    i2cp_Abort(id);
                }

                return(I2C_TRANSMIT_FIFO_FULL);
            }

            p_i2c_registers->tfr = (g_i2c_system_context[id].register_index >> 8);

            p_i2c_registers->tfr = (0xFF & g_i2c_system_context[id].register_index);

            g_i2c_system_context[id].index_format = I2C_NO_INDEX;
            g_i2c_system_context[id].active_event = I2C_INDEX_TX_EVENT;
            break;

        default:
            break;
    }

    if (g_i2c_system_context[id].operation == I2C_READ)
    {
        loop_counter = 0;
        /* Check whether the Stop bit has been programmed or not */
        if(I2C_READ_FIELD(p_i2c_registers->mcr, I2C_MCR_STOP, I2C_MCR_SHIFT_STOP))
        {
            /* Check for the Master transaction Done */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTD, I2C_INT_SHIFT_MTD)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        else
        {
        	/* Check for the Master transaction Done Without Stop */
            while
            (
                !I2C_READ_FIELD(p_i2c_registers->risr, I2C_INT_MTDWS, I2C_INT_SHIFT_MTDWS)
            &&  loop_counter < I2C_ENDAD_COUNTER
            )
            {
                loop_counter++;
            };
        }
        
        if (loop_counter >= I2C_ENDAD_COUNTER)
        {
            error_status = i2cp_GetAbortCause(id);
            if (error_status != I2C_OK)
            {
                return(error_status);
            }
            else
            {
                i2cp_Abort(id);
            }

            return(I2C_INTERNAL_ERROR);
        }

        /* Master Transaction has been done */
        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);
        
        /* Master Transaction Without Stop has been done */
        /* Acknowledge the Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

        /* Master control configuration for read operation  */
        I2C_SET_BIT(mcr, I2C_MCR_OP);

        /* Configure the STOP condition*/
        I2C_SET_BIT(mcr, I2C_MCR_STOP);

        /* Configuring the Frame length */
        I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, g_i2c_system_context[id].count_data);
        
        I2C_WRITE_FIELD(p_i2c_registers->mcr,I2C_MCR_LENGTH_STOP_OP,I2C_MCR_SHIFT_LENGTH_STOP_OP,mcr);        

    }

    /* added to remove the warning unused variable */
    error_status = error_status;

    /* Update the status of the I2C controller  */
    if (I2C_READ == g_i2c_system_context[id].operation)
    {
        g_i2c_system_context[id].status = I2C_STATUS_MASTER_RECEIVER_MODE;
    }
    else
    {
        g_i2c_system_context[id].status = I2C_STATUS_MASTER_TRANSMITTER_MODE;
    }

    return(I2C_OK);
}

 
/* Private function valid for HS controller */


/****************************************************************************/
/* NAME			:	i2cp_GetAbortCause                  		  	        */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Get the abort Cause                                     */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		 	: t_i2c_device_id 	: The controller that aborted   		*/
/*     InOut    :  None                                                     */
/* 			: None                                                     */
/*                                                                          */
/* RETURN		:	t_i2c_error										    	*/
/*--------------------------------------------------------------------------*/
/* Type              :                                                */
/* REENTRANCY 	     :	Non Re-entrant                                      */
/* REENTRANCY ISSUES :														*/

/****************************************************************************/
t_i2c_error i2cp_GetAbortCause(t_i2c_device_id id)
{
    u8         abort_cause;
    t_i2c_error     error_status;

    t_i2c_registers *p_i2c_registers;

    p_i2c_registers = (t_i2c_registers *) g_i2c_system_context[id].base_address;

    if (I2C_READ_FIELD(p_i2c_registers->sr, I2C_SR_STATUS, I2C_SR_SHIFT_STATUS) == 3)
    {
        abort_cause = (u8) I2C_READ_FIELD(p_i2c_registers->sr, I2C_SR_CAUSE, I2C_SR_SHIFT_CAUSE);

        switch (abort_cause)
        {
            case 0:
                error_status = I2C_ACK_FAIL_ON_ADDRESS;
                break;

            case 1:
                error_status = I2C_ACK_FAIL_ON_DATA;
                break;

            case 2:
                error_status = I2C_ACK_IN_HS_MODE;
                break;

            case 3:
                error_status = I2C_ARBITRATION_LOST;
                break;

            case 4:
                error_status = I2C_BUS_ERROR_DETECTED_START;
                break;

            case 5:
                error_status = I2C_BUS_ERROR_DETECTED_STOP;
                break;

            case 6:
                error_status = I2C_OVERFLOW;
                break;

            default:
                error_status = I2C_INTERNAL_ERROR;
                break;
        }

        return(error_status);
    }

    return(I2C_OK);
}


/****************************************************************************/
/* NAME			:	I2C_SetBaseAddress										*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	This routine initializes I2C register base address.     */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		IN  	:	id          : I2C controller id                         */
/*                    i2c_base_address	:	I2C registers base address		*/
/* 		OUT 	:	None                                                	*/
/*                                                                          */
/* RETURN		:	None											    	*/
/*--------------------------------------------------------------------------*/
/* REENTRANCY 	    :	Non Re-entrant                                      */
/* REENTRANCY ISSUES:														*/
/*			1)		Global variable gp_registers (register base address)	*/
/*					is being modified										*/

/****************************************************************************/
void I2C_SetBaseAddress(t_i2c_device_id id, t_logical_address address)
{
    /* Initializing the I2C controller base address */
    gp_i2c_registers[id] = (t_i2c_registers *) address;
}

/****************************************************************************/
/* NAME			:	I2C_DisableIRQSrc										*/
/*--------------------------------------------------------------------------*/
/* DESCRIPTION	: 	Disable the given I2C controller to generate interrupts. */
/*                                                                          */
/* PARAMETERS	:                                                           */
/* 		IN  	: t_i2c_irq_src_id id	: the IRQ source to be disabled.	*/
/* 		OUT 	:	None                                                	*/
/*                                                                          */
/* RETURN		:	None											    	*/
/*--------------------------------------------------------------------------*/
/* REENTRANCY 	    :	Non Re-entrant                                      */
/* REENTRANCY ISSUES:														*/

/****************************************************************************/
void I2C_DisableIRQSrc(t_i2c_irq_src_id irq_id)
{

    gp_i2c_registers[GETDEVICE((u32)irq_id)]->imscr &= ~((u32)I2C_IRQ_SRC_ALL & (u32)irq_id);

}

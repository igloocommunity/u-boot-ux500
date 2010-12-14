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


#ifndef _U8500_I2C_H_
#define _U8500_I2C_H_

#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>

#include <asm/arch/common.h>
#include <configs/u8500.h>

#define MASK_BIT0     (1UL<<0)
#define MASK_BIT1     (1UL<<1) 
#define MASK_BIT2     (1UL<<2) 
#define MASK_BIT3     (1UL<<3) 
#define MASK_BIT4     (1UL<<4) 
#define MASK_BIT5     (1UL<<5) 
#define MASK_BIT6     (1UL<<6) 
#define MASK_BIT7     (1UL<<7) 
#define MASK_BIT8     (1UL<<8) 
#define MASK_BIT9     (1UL<<9) 
#define MASK_BIT10    (1UL<<10) 
#define MASK_BIT11    (1UL<<11) 
#define MASK_BIT12    (1UL<<12) 
#define MASK_BIT13    (1UL<<13) 
#define MASK_BIT14    (1UL<<14) 
#define MASK_BIT15    (1UL<<15) 
#define MASK_BIT16    (1UL<<16) 
#define MASK_BIT17    (1UL<<17) 
#define MASK_BIT18    (1UL<<18) 
#define MASK_BIT19    (1UL<<19) 
#define MASK_BIT20    (1UL<<20) 
#define MASK_BIT21    (1UL<<21)
#define MASK_BIT22    (1UL<<22) 
#define MASK_BIT23    (1UL<<23) 
#define MASK_BIT24    (1UL<<24) 
#define MASK_BIT25    (1UL<<25) 
#define MASK_BIT26    (1UL<<26) 
#define MASK_BIT27    (1UL<<27) 
#define MASK_BIT28    (1UL<<28) 
#define MASK_BIT29    (1UL<<29) 
#define MASK_BIT30    (1UL<<30)
#define MASK_BIT31    (1UL<<31) 
/*-----------------------------------------------------------------------------
        Typedefs
-----------------------------------------------------------------------------*/
typedef enum{
        I2C0,
        I2C1,
        I2C2,
        I2C3
}t_i2c_device_id;



/*-----------------------------------------------------------------------------

	Generic Macros

-----------------------------------------------------------------------------*/

#define I2C_SET_BIT(reg_name,mask)          ((reg_name) |=  (mask))
#define I2C_CLR_BIT(reg_name,mask)          ((reg_name) &= ~(mask))
#define I2C_WRITE_BIT(reg_name,val,mask)    ((reg_name) =   (((reg_name) & ~(mask)) | ((val) & (mask))))
#define I2C_TEST_BIT(reg_name,val)          ((reg_name) &   (mask))
#define I2C_WRITE_REG(reg_name,val)         ((reg_name) = (val))
#define I2C_READ_REG(reg_name)              (reg_name)
#define I2C_CLEAR                           0x00000000


#define I2C_WRITE_FIELD(reg_name,mask,shift,value) \
                  (reg_name = ((reg_name & ~mask) | (value << shift)))
                  
                  
#define I2C_READ_FIELD(reg_name,mask,shift)    ((reg_name & mask) >> shift )

typedef volatile struct  
{
   	

    u32 cr;                                 /* Control Register                      0x00 */
    u32 scr;                                /* Slave Address Register                0x04 */
    u32 hsmcr;                              /* HS Master code Register               0x08 */
    u32 mcr;                                /* Master Control Register               0x0C */
    u32 tfr;                                /* Transmit Fifo Register                0x10 */
    u32 sr;                                 /* Status Register                       0x14 */
    u32 rfr;                                /* Receiver Fifo Register                0x18 */
    u32 tftr;                               /* Transmit Fifo Threshold Register      0x1C */
    u32 rftr;                               /* Receiver Fifo Threshold Register      0x20 */
    u32 dmar;                               /* DMA register                          0x24 */
    u32 brcr;                               /* Baud Rate Counter Register            0x28 */
    u32 imscr;                              /* Interrupt Mask Set and Clear Register 0x2C */
    u32 risr;                               /* Raw interrupt status register         0x30 */
    u32 misr;                               /* Masked interrupt status register      0x34 */
    u32 icr;                                /* Interrupt Set and Clear Register      0x38 */
    u32 reserved_1[(0xFE0 - 0x3c) >> 2];    /* Reserved	                          0x03C to 0xFE0*/
    u32 periph_id_0;                        /*peripheral ID 0                        0xFE0  */
	u32 periph_id_1;                        /*peripheral ID 1                        0xFE4  */
	u32 periph_id_2;                        /*peripheral ID 2                        0xFE8  */ 
	u32 periph_id_3;                        /*peripheral ID 3                        0xFEC  */
	u32 cell_id_0;                          /*I2C cell   ID 0                        0xFF0  */
	u32 cell_id_1;                          /*I2C cell   ID 1                        0xFF4  */
 	u32 cell_id_2;                          /*I2C cell   ID 2                        0xFF8  */
  	u32 cell_id_3;                          /*I2C cell   ID 3                        0xFFC  */
    

}t_i2c_registers;



/* Control Register */ 
   /* Mask values for control register mask */
#define I2C_CR_PE          MASK_BIT0       /* Peripheral enable*/
#define I2C_CR_OM          0x6             /* Operation mode  */  
#define I2C_CR_SAM         MASK_BIT3       /* Slave Addressing mode */
#define I2C_CR_SM          0x30            /* Speed mode  */ 
#define I2C_CR_SGCM        MASK_BIT6       /* Slave General call mode */
#define I2C_CR_FTX         MASK_BIT7       /* Flush Transmit     */
#define I2C_CR_FRX         MASK_BIT8       /* Flush Receive */
#define I2C_CR_DMA_TX_EN   MASK_BIT9       /* DMA TX Enable */
#define I2C_CR_DMA_RX_EN   MASK_BIT10      /* DMA Rx Enable */
#define I2C_CR_DMA_SLE     MASK_BIT11      /* DMA Synchronization Logic enable */
#define I2C_CR_LM          MASK_BIT12      /* Loop back mode */
#define I2C_CR_FON         0x6000          /* Filtering On */

   /*  shift valus for control register bit fields */
#define I2C_CR_SHIFT_PE          0         /* Peripheral enable*/
#define I2C_CR_SHIFT_OM          1         /* Operation mode  */  
#define I2C_CR_SHIFT_SAM         3         /* Slave Addressing mode */
#define I2C_CR_SHIFT_SM          4         /* Speed mode  */ 
#define I2C_CR_SHIFT_SGCM        6         /* Slave General call mode */
#define I2C_CR_SHIFT_FTX         7         /* Flush Transmit     */
#define I2C_CR_SHIFT_FRX         8         /* Flush Receive */
#define I2C_CR_SHIFT_DMA_TX_EN   9         /* DMA TX Enable */
#define I2C_CR_SHIFT_DMA_RX_EN   10        /* DMA Rx Enable */
#define I2C_CR_SHIFT_DMA_SLE     11        /* DMA Synchronization Logic enable */
#define I2C_CR_SHIFT_LM          12        /* Loop back mode */
#define I2C_CR_SHIFT_FON         13        /* Filtering On */


/* Slave control register*/
  /* Mask values slave control register */
#define I2C_SCR_ADDR                   0x3FF
#define I2C_SCR_DATA_SETUP_TIME        0xFFFF0000

   /* Shift values for Slave control register */
#define I2C_SCR_SHIFT_ADDR               0
#define I2C_SCR_SHIFT_DATA_SETUP_TIME    16 
  
  

/* Master Control Register */
   /* Mask values for Master control register */
#define I2C_MCR_OP      MASK_BIT0          /* Operation */
#define I2C_MCR_A7      0xFE               /* LSB bits of the Address(7-bit ) */
#define I2C_MCR_EA10    0x700             /* Extended Address */
#define I2C_MCR_SB      MASK_BIT11         /* Start byte procedure */
#define I2C_MCR_AM      0x3000             /* Address type */
#define I2C_MCR_STOP    MASK_BIT14         /* stop condition */
#define I2C_MCR_LENGTH  0x3FF8000           /* Frame length */
#define I2C_MCR_A10     0x7FE              /* Define to set the 10 bit address */

#define I2C_MCR_LENGTH_STOP_OP    0x3FFC001  /*mask for length field,stop and operation  */
  /* Shift values for Master control values */

#define I2C_MCR_SHIFT_OP      0            /* Operation */
#define I2C_MCR_SHIFT_A7      1            /* LSB bits of the Address(7-bit ) */
#define I2C_MCR_SHIFT_EA10    8            /* Extended Address */
#define I2C_MCR_SHIFT_SB      11           /* Start byte procedure */
#define I2C_MCR_SHIFT_AM      12           /* Address type */
#define I2C_MCR_SHIFT_STOP    14           /* stop condition */
#define I2C_MCR_SHIFT_LENGTH  15           /* Frame length */
#define I2C_MCR_SHIFT_A10     1            /* define to set the 10 bit addres */
  
#define I2C_MCR_SHIFT_LENGTH_STOP_OP   0

/*  Status Register */
  /* Mask values for Status register */
#define I2C_SR_OP       0x3                /* Operation */
#define I2C_SR_STATUS   0xC                /* Controller Status */
#define I2C_SR_CAUSE    0x70               /* Abort Cause */
#define I2C_SR_TYPE     0x180               /* Receive Type */
#define I2C_SR_LENGTH   0xFF700            /* Transfer length */

/* Shift values for Status register */
#define I2C_SR_SHIFT_OP       0            /* Operation */
#define I2C_SR_SHIFT_STATUS   2            /* Controller Status */
#define I2C_SR_SHIFT_CAUSE    4            /* Abort Cause */
#define I2C_SR_SHIFT_TYPE     7            /* Receive Type */
#define I2C_SR_SHIFT_LENGTH   9            /* Transfer length */
 

/* DMA Register */
  /* Mask values for DMA register */ 
#define I2C_DMA_SBSIZE_RX  0x7            /* Source Burst Size Rx */
#define I2C_DMA_BURST_RX   MASK_BIT3      /* Burst Rx */
#define I2C_DMA_DBSIZE_TX  0x700          /* Destination Burst Size Tx */
#define I2C_DMA_BURST_TX   MASK_BIT11     /* Burst Tx */

  /* Shift values for DMA register */
#define I2C_DMA_SHIFT_SBSIZE_RX  0        /* Source Burst Size Rx */
#define I2C_DMA_SHIFT_BURST_RX   3        /* Burst Rx */
#define I2C_DMA_SHIFT_DBSIZE_TX  8        /* Destination Burst Size Tx */
#define I2C_DMA_SHIFT_BURST_TX   11       /* Burst Tx */

/* Baud rate counter registers */
  /* Mask values for Baud rate counter register */
#define I2C_BRCR_BRCNT2  0xFFFF      /* Baud rate counter value for HS mode  */ 
#define I2C_BRCR_BRCNT1  0xFFFF0000  /* Baud rate counter value for Standard and Fast mode*/

/* Shift values for the Baud rate counter register */
#define I2C_BRCR_SHIFT_BRCNT2    0
#define I2C_BRCR_SHIFT_BRCNT1    16    



/* Interrupt Register  */
   /* Mask values for Interrupt registers */
#define I2C_INT_TXFE      MASK_BIT0       /* Tx fifo empty */
#define I2C_INT_TXFNE     MASK_BIT1       /* Tx Fifo nearly empty */
#define I2C_INT_TXFF      MASK_BIT2       /* Tx Fifo Full */
#define I2C_INT_TXFOVR    MASK_BIT3       /* Tx Fifo over run */
#define I2C_INT_RXFE      MASK_BIT4       /* Rx Fifo Empty */
#define I2C_INT_RXFNF     MASK_BIT5       /* Rx Fifo nearly empty */
#define I2C_INT_RXFF      MASK_BIT6       /* Rx Fifo Full  */
#define I2C_INT_RFSR      MASK_BIT16      /* Read From slave request */
#define I2C_INT_RFSE      MASK_BIT17      /* Read from slave empty */
#define I2C_INT_WTSR      MASK_BIT18      /* Write to Slave request */
#define I2C_INT_MTD       MASK_BIT19      /* Master Transcation Done*/
#define I2C_INT_STD       MASK_BIT20      /* Slave Transaction Done */
#define I2C_INT_MAL       MASK_BIT24      /* Master Arbitation Lost */
#define I2C_INT_BERR      MASK_BIT25      /* Bus Error */
#define I2C_INT_MTDWS     MASK_BIT28      /* Master Transcation Done Without Stop*/
   /* Shift values for Interrupt registers */
#define I2C_INT_SHIFT_TXFE      0               /* Tx fifo empty */
#define I2C_INT_SHIFT_TXFNE     1               /* Tx Fifo nearly empty */
#define I2C_INT_SHIFT_TXFF      2               /* Tx Fifo Full */
#define I2C_INT_SHIFT_TXFOVR    3               /* Tx Fifo over run */
#define I2C_INT_SHIFT_RXFE      4               /* Rx Fifo Empty */
#define I2C_INT_SHIFT_RXFNF     5               /* Rx Fifo nearly empty */
#define I2C_INT_SHIFT_RXFF      6               /* Rx Fifo Full  */
#define I2C_INT_SHIFT_RFSR      16              /* Read From slave request */
#define I2C_INT_SHIFT_RFSE      17              /* Read from slave empty */
#define I2C_INT_SHIFT_WTSR      18              /* Write to Slave request */
#define I2C_INT_SHIFT_MTD       19              /* Master Transcation Done */
#define I2C_INT_SHIFT_STD       20              /* Slave Transaction Done */
#define I2C_INT_SHIFT_MAL       24              /* Master Arbitation Lost */
#define I2C_INT_SHIFT_BERR      25              /* Bus Error */
#define I2C_INT_SHIFT_MTDWS     28              /* Master Transcation Done Without Stop*/



/*-----------------------------------------------------------------------------
	Typedefs
-----------------------------------------------------------------------------*/
typedef enum {
    I2C_FREQ_MODE_STANDARD,	        	/* Standard mode.   */	
	I2C_FREQ_MODE_FAST,	                /* Fast mode.       */
    I2C_FREQ_MODE_HIGH_SPEED            /* High Speed mode. */
} t_i2c_freq_mode;

typedef enum {
	I2C_BUS_SLAVE_MODE = 0,             /* Slave Mode               */
	I2C_BUS_MASTER_MODE,                /* Master Mode              */
	I2C_BUS_MASTER_SLAVE_MODE           /* Dual Configuration Mode  */
} t_i2c_bus_control_mode;

typedef enum {
	I2C_CURRENT_BUS_SLAVE_TRANSMITTER,
	I2C_CURRENT_BUS_SLAVE_RECEIVER,
	I2C_CURRENT_BUS_MASTER_TRANSMITTER,
	I2C_CURRENT_BUS_MASTER_RECEIVER
}t_i2c_current_bus_configuration;

typedef enum {
    I2C_TRANSFER_MODE_POLLING,
    I2C_TRANSFER_MODE_INTERRUPT,
    I2C_TRANSFER_MODE_DMA
}t_i2c_transfer_mode;

typedef enum {
    I2C_DISABLE,
    I2C_ENABLE
}t_i2c_control;

typedef enum {
	I2C_COMMAND_SEND_START,
	I2C_COMMAND_SEND_STOP,
	I2C_COMMAND_SEND_ACKNOWLEDGE,
	I2C_COMMAND_CLEAR_ACKNOWLEDGE,
	I2C_COMMAND_SET_TRANSMIT_DMA,
	I2C_COMMAND_CLEAR_TRANSMIT_DMA,
	I2C_COMMAND_SET_RECEIVE_DMA,
	I2C_COMMAND_CLEAR_RECEIVE_DMA
}t_i2c_command;


typedef enum {
	I2C_STATUS_SLAVE_MODE,					     	/* Controller is in slave mode.*/
	I2C_STATUS_MASTER_MODE,					        /* Controller is in master mode.*/
	I2C_STATUS_MASTER_TRANSMITTER_MODE,				/* Controller is in master transmitter mode.*/
	I2C_STATUS_MASTER_RECEIVER_MODE,				/* Controller is in master receiver mode.*/
	I2C_STATUS_SLAVE_TRANSMITTER_MODE,				/* Controller is in slave transmitter mode.*/
	I2C_STATUS_SLAVE_RECEIVER_MODE					/* Controller is in slave receiver mode.*/
} t_i2c_device_status;

    
typedef enum {
    I2C_TRANSMIT_FIFO,
    I2C_RECEIVE_FIFO
}t_i2c_fifo;


typedef enum {
    I2C_DIGITAL_FILTERS_OFF,
    I2C_DIGITAL_FILTERS_1_CLK_SPIKES,
    I2C_DIGITAL_FILTERS_2_CLK_SPIKES,
    I2C_DIGITAL_FILTERS_4_CLK_SPIKES
}t_i2c_digital_filter;


typedef struct {
    t_i2c_digital_filter   i2c_digital_filter_control;
    t_i2c_control          i2c_dma_sync_logic_control;
    t_i2c_control          i2c_start_byte_procedure;       /*ONLY VALID FOR MASTER MODE TRANSACTIONS*/
    u8                i2c_high_speed_master_code;     /*ONLY VALID FOR MASTER MODE TRANSACTIONS*/
    u16               slave_data_setup_time;         /* Only valid for HS controller */
    u16               controller_i2c_address;
    u32               input_frequency;
} t_i2c_device_config;

typedef enum {
    I2C_NO_GENERAL_CALL_HANDLING,
    I2C_SOFTWARE_GENERAL_CALL_HANDLING,
    I2C_HARDWARE_GENERAL_CALL_HANDLING
} t_i2c_general_call_handling;
    
typedef struct {
    t_i2c_control               i2c_loopback_mode;
    t_i2c_general_call_handling i2c_slave_general_call_mode;
    t_i2c_transfer_mode         index_transfer_mode;
    t_i2c_transfer_mode         data_transfer_mode;
    t_i2c_bus_control_mode      bus_control_mode;
    u8                     i2c_transmit_interrupt_threshold;
    u8                     i2c_receive_interrupt_threshold;
    u8                     transmit_burst_length;
    u8                     receive_burst_length;
    u32                    i2c_transfer_frequency;
} t_i2c_transfer_config;



typedef struct {
    t_logical_address	            base_address;       /* The controller's logical base address.       */
    t_i2c_device_id 	                    id;			/* The controller's id.                         */
    t_i2c_freq_mode	                freq_mode;          /* Standard ,Fast mode or Hs Mode.              */
    t_bool	                        is_enabled;		    /* True means controller is enabled.            */
    t_i2c_bus_control_mode	        bus_control_mode;
    t_i2c_current_bus_configuration	current_bus_config;
    t_i2c_general_call_handling 	general_call_handling;
    u32	                    freq_scl;           /* The I2C bus SCL clock frequency (Hz).        */
    u32	                    freq_input;		    /* The controller's input clock frequency (Hz). */
    u32	                    own_address;        /* The controller's slave address.              */
} t_i2c_info;		                    /* Used to provide information to the user.     */	


typedef enum {
    /*Common to all platforms*/
    I2C_STATE_GENERAL_CALL_DETECTED     = MASK_BIT0,
    I2C_STATE_ARBITRATION_LOST          = MASK_BIT2,
    I2C_STATE_BUSY                      = MASK_BIT12,

    I2C_STATE_TRANSFER_COMPLETE         = MASK_BIT16,
    I2C_STATE_ABORT_NACK_ON_ADDRESS     = MASK_BIT17,
    I2C_STATE_ABORT_NACK_ON_DATA        = MASK_BIT18,
    I2C_STATE_ABORT_ACK_ON_MASTER_CODE  = MASK_BIT19,
    I2C_STATE_BUS_ERROR_DETECTED_START  = MASK_BIT20,
    I2C_STATE_BUS_ERROR_DETECTED_STOP   = MASK_BIT21,
    I2C_STATE_OVERFLOW                  = MASK_BIT22,
    I2C_STATE_HARDWARE_GENERAL_CALL     = MASK_BIT23
} t_i2c_device_states;


typedef enum {
	I2C_NO_PENDG_EVENT_ERROR			= 7, /*U8500_NO_PENDG_EVENT_ERROR,*/
	I2C_NO_MORE_FILTER_PENDG_EVENT	= 5, /*U8500_NO_MORE_FILTER_PENDG_EVENT,*/
	I2C_NO_MORE_PENDG_EVENT		= 5, /*U8500_NO_MORE_PENDG_EVENT,*/
	I2C_REMAG_FILTER_PENDG_EVENTS	= 3,/*U8500_REMAG_FILTER_PENDG_EVENTS,*/
	I2C_REMAG_PENDG_EVENTS		= 2,/*U8500_REMAG_PENDG_EVENTS,*/
	I2C_INTERNAL_EVENT			= 1,/*U8500_INTERNAL_EVENT,*/
	I2C_OK 					= 0, /*U8500_OK,*/	                    /* No error.                                */
    I2C_INTERNAL_ERROR                  = -8, /*U8500_INTERNAL_ERROR,*/
    I2C_NOT_CONFIGURED                  = -7, /*U8500_NOT_CONFIGURED,*/
    I2C_REQUEST_PENDG                 = -6, /*U8500_REQUEST_PENDG,*/
    I2C_REQUEST_NOT_APPLICABLE          = -5, /*U8500_REQUEST_NOT_APPLICABLE,*/
    I2C_INVALID_PARAMETER               = -4, /*U8500_VALID_PARAMETER,*/
    I2C_UNSUPPORTED_FEATURE             = -3, /*U8500_UNSUPPORTED_FEATURE,*/
    I2C_UNSUPPORTED_HW                  = -2, /*U8500_UNSUPPORTED_HW,*/
	I2C_HW_FAILED			= -66, /*(U8500_MAX_ERROR_VALUE -1),*/		/* Generic hardware error.					*/
	I2C_SW_FAILED			= -67, /*(U8500_MAX_ERROR_VALUE -2),*/		/* Generic software error.					*/
	I2C_CONTROLLER_BUSY		= -68, /*(U8500_MAX_ERROR_VALUE -3),*/		/* Transfer on going.						*/
	I2C_LINE_FREQ_NOT_SUPPORTED	= -69, /*(U8500_MAX_ERROR_VALUE -4),*/		/* SCL bus frequency not supported.			*/
	I2C_PUT_FREQ_NOT_SUPPORTED	= -70, /*(U8500_MAX_ERROR_VALUE -5),*/		/* Input frequency not supported.			*/
	I2C_DDC_MODE_NOT_SUPPORTED	= -71, /*(U8500_MAX_ERROR_VALUE -6),*/		/* DDC mode not supported.					*/
	I2C_SLAVE_ADDRESS_NOT_VALID	= -72, /*(U8500_MAX_ERROR_VALUE -7),*/		/* Slave address is reserved or not valid.  */

    I2C_BYTE_TRANSFER_FAILED 	        = -165, /*(U8500_MAX_ERROR_VALUE -100),*/
    I2C_ADDRESS_MATCH_FAILED            = -166, /*(U8500_MAX_ERROR_VALUE -101),*/
    I2C_START_BYTE_FAILED               = -167, /*(U8500_MAX_ERROR_VALUE -102),*/
    I2C_ACKNOWLEDGE_FAILURE             = -168, /*(U8500_MAX_ERROR_VALUE -103),*/
    I2C_STOP_FAILED 	                = -169, /*(U8500_MAX_ERROR_VALUE -104),*/
    I2C_ARBITRATION_LOST                = -170, /*(U8500_MAX_ERROR_VALUE -105),*/
    I2C_BUS_ERROR 	                = -171, /*(U8500_MAX_ERROR_VALUE -106),*/
    I2C_10_BIT_ADDRESS_FAILED           = -172, /*(U8500_MAX_ERROR_VALUE -107),*/
    I2C_SCL_FALL_FAILED                 = -173, /*(U8500_MAX_ERROR_VALUE -108),*/
    I2C_END_ADDRESS_FAILED              = -174, /*(U8500_MAX_ERROR_VALUE -109),*/

    I2C_TRANSMIT_FIFO_FULL              = -175, /*(U8500_MAX_ERROR_VALUE -200),*/
    I2C_RECEIVE_FIFO_EMPTY              = -176, /*(U8500_MAX_ERROR_VALUE -201),*/
    I2C_ACK_FAIL_ON_ADDRESS             = -177, /*(U8500_MAX_ERROR_VALUE -202),*/
    I2C_ACK_FAIL_ON_DATA                = -178, /*(U8500_MAX_ERROR_VALUE -203),*/
    I2C_ACK_IN_HS_MODE                  = -179, /*(U8500_MAX_ERROR_VALUE -204),*/
    I2C_BUS_ERROR_DETECTED_START        = -180, /*(U8500_MAX_ERROR_VALUE -205),*/
    I2C_BUS_ERROR_DETECTED_STOP         = -181, /*(U8500_MAX_ERROR_VALUE -206),*/
    I2C_OVERFLOW                        = -182, /*(U8500_MAX_ERROR_VALUE -207)*/
} t_i2c_error;

typedef enum
{
    /*Common to all platforms*/
	I2C_NO_EVENT				                     = MASK_BIT0,	/* No activity.                                 */
	I2C_TRANSFER_OK_EVENT		                     = MASK_BIT1,	/* Transfer operation ended correctly.			*/
	I2C_CANCEL_EVENT			                     = MASK_BIT2,	/* Transfer operation cancelled by the user.	*/
	I2C_INTERNAL_ERROR_EVENT	                     = MASK_BIT3,	/* Internal error happened.						*/
	I2C_ARBITRATION_LOST_ERROR_EVENT                 = MASK_BIT4,	/* Arbitration Lost happened.					*/

	I2C_AF_ERROR_EVENT			                     = MASK_BIT5,	/* Acknowledge Failure happened.				*/
	I2C_BUS_ERROR_EVENT	                             = MASK_BIT6,	/* Bus Error happened.							*/
	I2C_START_EVENT			                         = MASK_BIT7,	/* START generated.								*/
	I2C_INDEX_TX_EVENT			                     = MASK_BIT8,	/* Register index byte transmitted.				*/
	I2C_DATA_TX_EVENT			                     = MASK_BIT9,	/* Data byte transmitted.						*/
	I2C_DATA_RX_EVENT			                     = MASK_BIT10,	/* Data byte received.							*/
	I2C_WAITG_DATA_RX_EVENT	                     = MASK_BIT11,	/* Waiting for a data byte.						*/

    
    I2C_TRANSMIT_FIFO_EMPTY_EVENT                    = MASK_BIT12,
    I2C_TRANSMIT_FIFO_NEARLY_EMPTY_EVENT             = MASK_BIT13,
    I2C_TRANSMIT_FIFO_FULL_EVENT                     = MASK_BIT14,
    I2C_TRANSMIT_FIFO_OVERRUN_EVENT                  = MASK_BIT15,
    I2C_RECEIVE_FIFO_EMPTY_EVENT                     = MASK_BIT16,
    I2C_RECEIVE_FIFO_NEARLY_FULL_EVENT               = MASK_BIT17,
    I2C_RECEIVE_FIFO_FULL_EVENT                      = MASK_BIT18,
    I2C_READ_FROM_SLAVE_REQUEST_EVENT                = MASK_BIT19,
    I2C_READ_FROM_SLAVE_EMPTY_EVENT                  = MASK_BIT20,
    I2C_WRITE_TO_SLAVE_REQUEST_EVENT                 = MASK_BIT21,
    I2C_MASTER_TRANSACTION_DONE_EVENT                = MASK_BIT22,
    I2C_SLAVE_TRANSACTION_DONE_EVENT                 = MASK_BIT23,
    I2C_ABORT_NACK_ON_ADDRESS_EVENT                  = MASK_BIT24,
    I2C_ABORT_NACK_ON_DATA_EVENT                     = MASK_BIT25,
    I2C_ABORT_ACK_ON_MASTER_CODE_EVENT               = MASK_BIT26,
    I2C_BUS_ERROR_DETECTED_START_EVENT               = MASK_BIT27,
    I2C_BUS_ERROR_DETECTED_STOP_EVENT                = MASK_BIT28,
    I2C_OVERFLOW_EVENT                               = MASK_BIT29,
    I2C_MASTER_TRANSACTION_DONE_WITHOUT_STOP_EVENT   = MASK_BIT30
} t_i2c_event;			               /* Inform the I2C HCL user about the last occurred event.*/


typedef enum
{
    I2C_NO_INDEX,                               /* Current transfer is non-indexed      */
    I2C_BYTE_INDEX,                             /* Current transfer uses 8-bit index    */
    I2C_HALF_WORD_LITTLE_ENDIAN,                /* Current transfer uses 16-bit index 
                                                   in little endian mode                */
    I2C_HALF_WORD_BIG_ENDIAN                    /* Current transfer uses 16-bit index 
                                                   in big endian mode                   */
}t_i2c_index_format;

typedef struct {
    t_i2c_device_id id;
	t_i2c_event		type;		    /* The active event.                            */
	u32		transfer_data;	/* Number of data bytes actually transferred.   */
} t_i2c_active_event;

typedef t_i2c_device_id t_i2c_irq_status;

/*-----------------------------------------------------------------------------
	Configuration functions
-----------------------------------------------------------------------------*/
 t_i2c_error I2C_Init            (t_i2c_device_id id, 
                                        t_logical_address address);

 t_i2c_error I2C_SetDeviceConfiguration   (t_i2c_device_id id,
                                                t_i2c_device_config    *p_device_config);

 t_i2c_error I2C_SetTransferConfiguration (t_i2c_device_id id,
                                                t_i2c_transfer_config  *p_transfer_config);

 t_i2c_error I2C_SetTransferMode ( t_i2c_device_id id, 
                                         t_i2c_transfer_mode index_transfer_mode,
                                         t_i2c_transfer_mode data_transfer_mode);

 t_i2c_error I2C_SetBusControlMode (t_i2c_device_id id, 
                                          t_i2c_bus_control_mode bus_control_mode);

 t_i2c_error I2C_SendCommand          (   t_i2c_device_id,
                                                    t_i2c_command);

                                        
/*-----------------------------------------------------------------------------
	    Configuration functions
-----------------------------------------------------------------------------*/
 t_i2c_error I2C_FlushFifo(t_i2c_device_id , t_i2c_fifo );

/*-----------------------------------------------------------------------------
	    Status functions
-----------------------------------------------------------------------------*/
 t_i2c_error I2C_GetInfo                  (   t_i2c_device_id id, 
                                                    t_i2c_info *p_info);
/*-----------------------------------------------------------------------------
        Operative functions
-----------------------------------------------------------------------------*/
	 t_i2c_error I2C_Enable   (t_i2c_device_id id);

	 t_i2c_error I2C_Disable  (t_i2c_device_id id);

     t_i2c_error I2C_WriteSingleData    (t_i2c_device_id             id,
                                                u16             slave_address, 
                                                t_i2c_index_format   index_format,                                                
                                                u16             index_value,
                                                u8              data);

     t_i2c_error I2C_WriteMultipleData  (t_i2c_device_id             id, 
                                                u16             slave_address, 
                                                t_i2c_index_format   index_format,                                                
                                                u16             index_value,
                                                u8              *p_data, 
                                                u32             count);

     t_i2c_error I2C_ReadSingleData    (t_i2c_device_id             id, 
                                                u16             slave_address, 
                                                t_i2c_index_format   index_format,
                                                u16             index_value,
                                                u8              *p_data);

     t_i2c_error I2C_ReadMultipleData (t_i2c_device_id             id, 
                                                u16             slave_address, 
                                                t_i2c_index_format   index_format,
                                                u16             index_value,
                                                u8              *p_data, 
                                                u32             count);


     t_i2c_error I2C_Cancel              (     t_i2c_device_id id, 
                                                t_i2c_active_event *event);/*Only IT mode*/


     t_bool 			I2C_IsEventActive       (    t_i2c_active_event* event);

     t_bool 			I2C_AcknowledgeEvent    (  t_i2c_active_event* event);
                                                

 t_i2c_error I2C_GetInputClock(           t_i2c_device_id id, 
                                                u32 *p_fIn);
                                            
 t_i2c_error I2C_GetBusClock(             t_i2c_device_id id, 
                                                u32 *p_fSCL);
                                        
 t_i2c_error I2C_GetEnabled(              t_i2c_device_id id, 
                                                t_bool *p_status);

 t_i2c_error I2C_GetDeviceConfiguration(  t_i2c_device_id id,
                                                t_i2c_device_config    *p_device_config);

 t_i2c_error I2C_GetTransferConfiguration(t_i2c_device_id id,
                                                t_i2c_transfer_config  *p_transfer_config); 

 t_i2c_error I2C_GetTransferMode(         t_i2c_device_id id, 
                                                t_i2c_transfer_mode *p_index_transfer_mode,
                                                t_i2c_transfer_mode *p_data_transfer_mode);

 t_i2c_error I2C_GetBusControlMode(       t_i2c_device_id id, 
                                                t_i2c_current_bus_configuration 
                                                                   *p_i2c_current_transfer_mode,
                                                t_i2c_bus_control_mode *p_bus_control_mode);


/*-----------------------------------------------------------------------------
		Power Management functions
-----------------------------------------------------------------------------*/
 t_i2c_error   I2C_Reset(t_i2c_device_id id);

 void          I2C_SaveDeviceContext   (t_i2c_device_id id);

 void          I2C_RestoreDeviceContext(t_i2c_device_id id);


/* Start of _I2CP_H_*/

/*Peripheral ID s  */

#define   I2C_P_ID_0         0x24
#define   I2C_P_ID_1         0x00
#define   I2C_P_ID_2         0x38
#define   I2C_P_ID_3         0x00
#define   I2C_CELL_ID_0      0x0D 
#define   I2C_CELL_ID_1      0xF0
#define   I2C_CELL_ID_2      0x05
#define   I2C_CELL_ID_3      0xB1

/*-----------------------------------------------------------------------------

	Constants

-----------------------------------------------------------------------------*/
typedef enum {
	I2C_NO_OPERATION	= 0xFF,
	I2C_WRITE			= 0x00,
	I2C_READ			= 0x01
} t_i2c_operation;

/*-----------------------------------------------------------------------------
	Typedefs
-----------------------------------------------------------------------------*/
typedef enum { 
	I2C_MAX_STANDARD_SCL    =   100000,	/* Max clock frequency (Hz) for Standard Mode.*/
	I2C_MAX_FAST_SCL	    =   400000,	/* Max clock frequency (Hz) for Fast Mode.*/
	I2C_MAX_HIGH_SPEED_SCL  =  3400000  /* Max clock frequency (Hz) for HS Mode.*/
} I2C_MaxClocks;

typedef enum {
	I2C_DDC1,						/* DDC1 mode.*/
	I2C_DDC2B,						/* DD2 B mode.*/
	I2C_DDC2AB						/* DDC2 AB mode (I2C).*/
} t_i2c_ddc_mode;

typedef enum {
    I2C_IT_TXFE     = MASK_BIT0,     /* Tx fifo empty */
    I2C_IT_TXFNE    = MASK_BIT1,     /* Tx Fifo nearly empty */
    I2C_IT_TXFF     = MASK_BIT2,     /* Tx Fifo Full */
    I2C_IT_TXOVR    = MASK_BIT3,     /* Tx Fifo over run */
    I2C_IT_RXFE     = MASK_BIT4,     /* Rx Fifo Empty */
    I2C_IT_RXFNF    = MASK_BIT5,     /* Rx Fifo nearly empty */
    I2C_IT_RXFF     = MASK_BIT6,     /* Rx Fifo Full  */
    I2C_IT_RFSR     = MASK_BIT16,    /* Read From slave request */
    I2C_IT_RFSE     = MASK_BIT17,    /* Read from slave empty */
    I2C_IT_WTSR     = MASK_BIT18,    /* Write Slave request */
    I2C_IT_MTD      = MASK_BIT19,    /* Master Transcation Done */
    I2C_IT_STD      = MASK_BIT20,    /* Slave Transaction Done */
    I2C_IT_MAL      = MASK_BIT24,    /* Master Arbitation Lost */
    I2C_IT_BERR     = MASK_BIT25,    /* Bus Error */
    I2C_IT_MTDWS      = MASK_BIT28   /* Master Transcation Done Without Stop */
}t_i2c_interrupt;	/* IRQ source numbers.*/


typedef enum { 
	I2C_NO_REG_INDEX_OP,			/* Do not send any register index.*/
	I2C_8_BIT_REG_INDEX_OP,			/* Send a 8-bit register index.*/
	I2C_16_BIT_REG_INDEX_OP			/* Send a 16-bit register index.*/
} t_i2c_reg_op;

typedef u32 t_i2c_device_context[5];

typedef struct {
    /*Device configuration*/
	t_logical_address	    base_address;        	/* The controller's base address.*/
	u32			    freq_scl;	        	/* The I2C bus SCL clock frequency (Hz).*/
	u16			    own_address;	        /* The controller's slave address.*/
    t_i2c_device_context    i2c_device_context;
    t_i2c_digital_filter    digital_filter_control;
    t_i2c_control           dma_sync_logic_control;
    t_i2c_control           start_byte_procedure;
    u8                 high_speed_master_code;
    u16                slave_data_setup_time;
   
    /*Transfer configuration*/
	u32			    freq_input;		        /* The controller's input clock frequency (Hz).*/
	t_i2c_freq_mode		    mode;			        /* Standard or Fast mode.*/
	t_i2c_operation		    operation;		        /* Write or read.*/
	t_i2c_bus_control_mode  bus_control_mode;
    t_i2c_control           i2c_loopback_mode;
    t_i2c_general_call_handling general_call_mode_handling;
    t_i2c_transfer_mode     index_transfer_mode;
    t_i2c_transfer_mode     data_transfer_mode;
    u8                 i2c_transmit_interrupt_threshold;
    u8                 i2c_receive_interrupt_threshold;
    u8                 transmit_burst_length;
    u8                 receive_burst_length;
    u16                burst_length;
	u16			    slave_address;	        /* The slave to talk to.*/
	u16			    register_index;	        /* The index of the slave's registers*/
	t_i2c_index_format      index_format;
	t_bool                  multi_operation;

    /*Device Status*/
	t_bool				    enabled;		        /* True means controller is enabled.*/
	u32			    count_data;		        /* The number of bytes to be transferred.*/
	u32			    transfer_data;	        /* Number of transferred data bytes.*/
	u8*			    databuffer;		        /* Pointer to the data buffer. Used in Multi operation.*/
	t_i2c_current_bus_configuration current_bus_config;
	t_i2c_device_status		    status;			        /* The controller's status.*/
	u8				    data;			        /* Used in Single operation.*/
	t_i2c_event			    active_event;	        /* The current active event.*/
	t_bool                  std;                    /*This variable is used to store the STD interrupt   */
	                                                /*status for 10 bit slave transmitter case */
    
} t_i2c_system_context;


/*-----------------------------------------------------------------------------

	Private service functions
                 
-----------------------------------------------------------------------------*/
 t_i2c_error i2cp_SetOwnAddress(t_i2c_device_id id, u16 address);
 t_i2c_error i2cp_SetBusClock(t_i2c_device_id id, u32 fSCL, u32 fIn);
 t_bool i2cp_AddressIsValid(u16 address);
 void i2cp_Abort(t_i2c_device_id id);

 t_i2c_error  i2cp_SlaveIndexReceive(t_i2c_device_id id );
 t_i2c_error  i2cp_TransmitDataPolling (t_i2c_device_id id, volatile u8* p_data);
 t_i2c_error  i2cp_ReceiveDataPolling(t_i2c_device_id  id, u8* p_data );
 t_i2c_error  i2cp_MasterIndexTransmit(t_i2c_device_id id );


 t_i2c_error  i2cp_GetAbortCause(t_i2c_device_id id );

/* End of _I2CP_H_*/


typedef enum
{
    
    
    I2C0_IRQ_SRC_TRANSMIT_FIFO_EMPTY        		  =  MASK_BIT0,
    I2C0_IRQ_SRC_TRANSMIT_FIFO_NEARLY_EMPTY           =  MASK_BIT1,
    I2C0_IRQ_SRC_TRANSMIT_FIFO_FULL                   =  MASK_BIT2,
    I2C0_IRQ_SRC_TRANSMIT_FIFO_OVERRUN                =  MASK_BIT3,
    I2C0_IRQ_SRC_RECEIVE_FIFO_EMPTY                   =  MASK_BIT4,
    I2C0_IRQ_SRC_RECEIVE_FIFO_NEARLY_FULL             =  MASK_BIT5,
    I2C0_IRQ_SRC_RECEIVE_FIFO_FULL                    =  MASK_BIT6,
    I2C0_IRQ_SRC_READ_FROM_SLAVE_REQUEST              =  MASK_BIT16,
    I2C0_IRQ_SRC_READ_FROM_SLAVE_EMPTY                =  MASK_BIT17,
    I2C0_IRQ_SRC_WRITE_TO_SLAVE_REQUEST               =  MASK_BIT18,
    I2C0_IRQ_SRC_MASTER_TRANSACTION_DONE              =  MASK_BIT19,
    I2C0_IRQ_SRC_SLAVE_TRANSACTION_DONE               =  MASK_BIT20,
    I2C0_IRQ_SRC_MASTER_ARBITRATION_LOST              =  MASK_BIT24,
    I2C0_IRQ_SRC_BUS_ERROR                            =  MASK_BIT25,
    I2C0_IRQ_SRC_MASTER_TRANSACTION_DONE_WITHOUT_STOP =  MASK_BIT28,
    I2C0_IRQ_SRC_ALL                                  =  0x131F007F,
    
    I2C1_IRQ_SRC_TRANSMIT_FIFO_EMPTY                  = MASK_BIT29 | MASK_BIT0,
    I2C1_IRQ_SRC_TRANSMIT_FIFO_NEARLY_EMPTY           = MASK_BIT29 | MASK_BIT1,
    I2C1_IRQ_SRC_TRANSMIT_FIFO_FULL                   = MASK_BIT29 | MASK_BIT2,
    I2C1_IRQ_SRC_TRANSMIT_FIFO_OVERRUN                = MASK_BIT29 | MASK_BIT3,
    I2C1_IRQ_SRC_RECEIVE_FIFO_EMPTY                   = MASK_BIT29 | MASK_BIT4,
    I2C1_IRQ_SRC_RECEIVE_FIFO_NEARLY_FULL   		  = MASK_BIT29 | MASK_BIT5,
    I2C1_IRQ_SRC_RECEIVE_FIFO_FULL                    = MASK_BIT29 | MASK_BIT6,
    I2C1_IRQ_SRC_READ_FROM_SLAVE_REQUEST              = MASK_BIT29 | MASK_BIT16,
    I2C1_IRQ_SRC_READ_FROM_SLAVE_EMPTY                = MASK_BIT29 | MASK_BIT17,
    I2C1_IRQ_SRC_WRITE_TO_SLAVE_REQUEST               = MASK_BIT29 | MASK_BIT18,
    I2C1_IRQ_SRC_MASTER_TRANSACTION_DONE              = MASK_BIT29 | MASK_BIT19,
    I2C1_IRQ_SRC_SLAVE_TRANSACTION_DONE               = MASK_BIT29 | MASK_BIT20,
    I2C1_IRQ_SRC_MASTER_ARBITRATION_LOST              = MASK_BIT29 | MASK_BIT24,
    I2C1_IRQ_SRC_BUS_ERROR                            = MASK_BIT29 | MASK_BIT25,
    I2C1_IRQ_SRC_MASTER_TRANSACTION_DONE_WITHOUT_STOP = MASK_BIT29 | MASK_BIT28,
    I2C1_IRQ_SRC_ALL                                  = MASK_BIT29 | 0x131F007F,

   
    I2C2_IRQ_SRC_TRANSMIT_FIFO_EMPTY                  = MASK_BIT30 | MASK_BIT0,
    I2C2_IRQ_SRC_TRANSMIT_FIFO_NEARLY_EMPTY           = MASK_BIT30 | MASK_BIT1,
    I2C2_IRQ_SRC_TRANSMIT_FIFO_FULL                   = MASK_BIT30 | MASK_BIT2,
    I2C2_IRQ_SRC_TRANSMIT_FIFO_OVERRUN                = MASK_BIT30 | MASK_BIT3,
    I2C2_IRQ_SRC_RECEIVE_FIFO_EMPTY                   = MASK_BIT30 | MASK_BIT4,
    I2C2_IRQ_SRC_RECEIVE_FIFO_NEARLY_FULL             = MASK_BIT30 | MASK_BIT5,
    I2C2_IRQ_SRC_RECEIVE_FIFO_FULL                    = MASK_BIT30 | MASK_BIT6,
    I2C2_IRQ_SRC_READ_FROM_SLAVE_REQUEST              = MASK_BIT30 | MASK_BIT16,
    I2C2_IRQ_SRC_READ_FROM_SLAVE_EMPTY                = MASK_BIT30 | MASK_BIT17,
    I2C2_IRQ_SRC_WRITE_TO_SLAVE_REQUEST               = MASK_BIT30 | MASK_BIT18,
    I2C2_IRQ_SRC_MASTER_TRANSACTION_DONE              = MASK_BIT30 | MASK_BIT19,
    I2C2_IRQ_SRC_SLAVE_TRANSACTION_DONE               = MASK_BIT30 | MASK_BIT20,
    I2C2_IRQ_SRC_MASTER_ARBITRATION_LOST              = MASK_BIT30 | MASK_BIT24,
    I2C2_IRQ_SRC_BUS_ERROR                            = MASK_BIT30 | MASK_BIT25,
    I2C2_IRQ_SRC_MASTER_TRANSACTION_DONE_WITHOUT_STOP = MASK_BIT30 | MASK_BIT28,
    I2C2_IRQ_SRC_ALL                                  = MASK_BIT30 | 0x131F007F,


    
    I2C3_IRQ_SRC_TRANSMIT_FIFO_EMPTY                  = MASK_BIT30 | MASK_BIT29 | MASK_BIT0,
    I2C3_IRQ_SRC_TRANSMIT_FIFO_NEARLY_EMPTY           = MASK_BIT30 | MASK_BIT29 | MASK_BIT1,
    I2C3_IRQ_SRC_TRANSMIT_FIFO_FULL                   = MASK_BIT30 | MASK_BIT29 | MASK_BIT2,
    I2C3_IRQ_SRC_TRANSMIT_FIFO_OVERRUN                = MASK_BIT30 | MASK_BIT29 | MASK_BIT3,
    I2C3_IRQ_SRC_RECEIVE_FIFO_EMPTY                   = MASK_BIT30 | MASK_BIT29 | MASK_BIT4,
    I2C3_IRQ_SRC_RECEIVE_FIFO_NEARLY_FULL             = MASK_BIT30 | MASK_BIT29 | MASK_BIT5,
    I2C3_IRQ_SRC_RECEIVE_FIFO_FULL                    = MASK_BIT30 | MASK_BIT29 | MASK_BIT6,
    I2C3_IRQ_SRC_READ_FROM_SLAVE_REQUEST              = MASK_BIT30 | MASK_BIT29 | MASK_BIT16,
    I2C3_IRQ_SRC_READ_FROM_SLAVE_EMPTY                = MASK_BIT30 | MASK_BIT29 | MASK_BIT17,
    I2C3_IRQ_SRC_WRITE_TO_SLAVE_REQUEST               = MASK_BIT30 | MASK_BIT29 | MASK_BIT18,
    I2C3_IRQ_SRC_MASTER_TRANSACTION_DONE              = MASK_BIT30 | MASK_BIT29 | MASK_BIT19,
    I2C3_IRQ_SRC_SLAVE_TRANSACTION_DONE               = MASK_BIT30 | MASK_BIT29 | MASK_BIT20,
    I2C3_IRQ_SRC_MASTER_ARBITRATION_LOST              = MASK_BIT30 | MASK_BIT29 | MASK_BIT24,
    I2C3_IRQ_SRC_BUS_ERROR                            = MASK_BIT30 | MASK_BIT29 | MASK_BIT25,
    I2C3_IRQ_SRC_MASTER_TRANSACTION_DONE_WITHOUT_STOP = MASK_BIT30 | MASK_BIT29 | MASK_BIT28,
    I2C3_IRQ_SRC_ALL                                  = MASK_BIT30 | MASK_BIT29 |0x131F007F

    
} t_i2c_irq_src_id;



/* Macros for handling the device id */
#define I2CID_SHIFT							29
#define GETDEVICE(irqsrc)					((t_i2c_device_id)((irqsrc >>I2CID_SHIFT ) & 0x3))

/* a macro for masking all interrupts */
#define I2C_IRQ_SRC_ALL     I2C0_IRQ_SRC_ALL

typedef u32 t_i2c_irq_src; /*Combination of various interrupt sources
                                     described by t_i2c_irq_src_id*/

/*-----------------------------------------------------------------------------
	Events and interrupts management functions
-----------------------------------------------------------------------------*/
void             I2C_SetBaseAddress  (t_i2c_device_id id, t_logical_address address );
void             I2C_DisableIRQSrc   (t_i2c_irq_src_id id);
#endif	/* _U8500_I2C_H_               */

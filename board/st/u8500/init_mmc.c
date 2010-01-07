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
/* --- includes ----------------------------------------------------------- */
#include "common.h"
#include "mmc.h"
#include "init_mmc.h"
#include "gpio.h"
#include "mmc_utils.h"
#include "i2c.h"

#define HREF_BOARD_ID   ('1')
#define MOP500_BOARD_ID ('0')
char  Bootargs_buf[512];

#ifdef CONFIG_CMD_FAT
#include <part.h>
#include <fat.h>
#endif

#define LOOP(x) {int i;for(i=0;i<1000;i++);}

#define MMC_CARD_NUM	1
#define EMMC_CARD_NUM	4
enum {
	DEV_EMMC = 0,
	DEV_MMC
};

static block_dev_desc_t mmc_dev;
static block_dev_desc_t emmc_dev;
static u32	CSD[4];

block_dev_desc_t * mmc_get_dev(int dev)
{
	if (dev == DEV_EMMC)
		return (block_dev_desc_t *)(&emmc_dev);
	else if (dev == DEV_MMC)
		return (block_dev_desc_t *)(&mmc_dev);

	printf("mmc_get_dev: unknown dev %d\n", dev);
	return 0;
}

static unsigned long emmc_block_read(int dev, unsigned long blknr,
					lbaint_t blkcnt, void *dest)
{
	unsigned long rc;

	rc = mmc_readblocks(EMMC_CARD_NUM, (u32) (512 * blknr), (u32 *)dest,
			512, blkcnt);
	if (rc != 0) {
		printf("mmc_block_read: readblocks failed %ld\n", rc);
		rc = 0;
	} else {
		rc = blkcnt;
	}
	return rc;
}

unsigned long mmc_block_read(int dev, unsigned long blknr,
					lbaint_t blkcnt, void *dest)
{
	unsigned long i;
	unsigned long src= blknr;
	unsigned long rc = 0;

	for(i = 0; i < blkcnt; i++)
	{
		/* card#, read offset, buffer, blksize, transfer mode */
		mmc_readblock(MMC_CARD_NUM, (u32) (512 * src),
				(u32 *)dest, 512, MMCPOLLING);
		rc++;
		src++;
		dest += 512;
	}
	return rc;
}

int mmc_hw_init(void)
{
    t_mmc_error           response;    

    /* save the GPIO0 AFSELA register*/
    gpio_altfuncenable(GPIO_ALT_SD_CARD0, "MMC");
    /* Power-on the controller*/
    response = mmc_enable ();

    if (response != MMC_OK)
    {
	response = mmc_enable ();
        if (response != MMC_OK)
        {
            printf ("Error in card power on\n");
            goto end;
        }
    }
    /* Initialise the cards on the bus, if any*/
    response = mmc_initCard ();
    if (response != MMC_OK)
    {
	printf ("Error in card initialization\n"); 
	goto end;
    }
    response = mmc_readcsd (CSD);
    if (response != MMC_OK)
    {
        printf ("Error while fetching card info\n");
        goto end;
    } 
    
    return 0;
  end:
    return 1;
}

#define I2C_SCL_FREQ            100000          /* I2C bus clock frequency.*/
#define I2C_INPUT_FREQ          48000000        /* Input clock frequency.*/
#define I2C0_SLAVE_ADDRESS	(0x84 >>1)		/*GPIO expander slave address*/
#define REG_CHIP_ID_INDEX	0x80
#define HS_MASTER_CODE 0x01  		/* High speed Master code */
#define TX_FIFO_THRESHOLD 0x4 /* The threshold below or equal to which the transmit FIFO generates interrupt */
#define RX_FIFO_THRESHOLD 0x4 /* The threshold above or equal to which the receive FIFO generates interrupt */
#define BURST_LENGTH 0 /* The burst length used in the DMA operation */
#define SLAVE_SETUP_TIME 14 /* Slave data setup time */


static void config_extended_gpio(void)
{
  t_i2c_error    error_status;
  t_i2c_device_config  i2c_device_config;
  t_i2c_transfer_config i2c_transfer_config;
  t_i2c_error error_i2c;
  u8 read_data = 0;
  u8 dataArr[2]={0x06,0x06};
  char board_id = HREF_BOARD_ID;

  I2C_SetBaseAddress(I2C0, CFG_I2C0_BASE);
  error_i2c = I2C_Init(I2C0, CFG_I2C0_BASE);
 
  i2c_device_config.controller_i2c_address = 0;
  i2c_device_config.input_frequency = I2C_INPUT_FREQ;
  i2c_device_config.i2c_digital_filter_control = I2C_DIGITAL_FILTERS_OFF;
  i2c_device_config.i2c_dma_sync_logic_control = I2C_DISABLE;
  i2c_device_config.i2c_start_byte_procedure = I2C_DISABLE;

  i2c_transfer_config.i2c_transfer_frequency = I2C_SCL_FREQ;
  i2c_transfer_config.bus_control_mode = I2C_BUS_MASTER_MODE;
 
  i2c_transfer_config.i2c_transmit_interrupt_threshold = TX_FIFO_THRESHOLD;
  i2c_transfer_config.i2c_receive_interrupt_threshold = RX_FIFO_THRESHOLD;
  i2c_transfer_config.transmit_burst_length = BURST_LENGTH;
  i2c_transfer_config.receive_burst_length = BURST_LENGTH;
  i2c_transfer_config.i2c_loopback_mode = I2C_DISABLE;
  i2c_transfer_config.index_transfer_mode = I2C_TRANSFER_MODE_POLLING;
  i2c_transfer_config.data_transfer_mode = I2C_TRANSFER_MODE_POLLING;
  i2c_transfer_config.bus_control_mode = I2C_BUS_MASTER_MODE;
  i2c_transfer_config.i2c_slave_general_call_mode = I2C_NO_GENERAL_CALL_HANDLING;
 
  i2c_device_config.slave_data_setup_time = SLAVE_SETUP_TIME;
 
  error_status = I2C_SetDeviceConfiguration(I2C0, &i2c_device_config);
  if (I2C_OK != error_status)
  {
  printf("\n Error in I2C_SetDeviceConfiguration; err = %d", error_status);
   return;
  }
 
   error_status = I2C_SetTransferConfiguration(I2C0, &i2c_transfer_config);
  if (I2C_OK != error_status)
  {
	printf("\n Error in I2C_SetTransferConfiguration; err = %d", error_status);
	return;
  }
	LOOP(10); 
	error_status = I2C_ReadSingleData (I2C0, I2C0_SLAVE_ADDRESS, I2C_BYTE_INDEX, 0x80, &read_data);
	LOOP(1);
	printf("\nGPIO expander Chip ID %x\n", read_data);
	
	if(error_status){
		printf("\n Error in I2C_ReadSingleData = %d", error_status);
		return;
	}
	if (read_data == 0x01) /*If chip is = 0x1, the platform is MOP500, so config STMPE*/
	{
		printf("\nMOP500 platform\n");
		//config_stmpe();
	        error_status = I2C_WriteSingleData(I2C0, I2C0_SLAVE_ADDRESS, I2C_BYTE_INDEX, 0x89, 0x0C);
		LOOP(5);
		error_status = I2C_WriteSingleData(I2C0, I2C0_SLAVE_ADDRESS, I2C_BYTE_INDEX, 0x83, 0x0C);
		LOOP(5);
		board_id = MOP500_BOARD_ID;
	    }
	else if(read_data==0x03) /* If chip is = 0x3,the platform is HREF, so config Toshiba controller*/
		{
		printf("\nHREF platform\n");	
		//following code is for HREF 
		//set the direction of the GPIO KPY9 and KPY10
		error_status = I2C_WriteSingleData(I2C0, I2C0_SLAVE_ADDRESS, I2C_BYTE_INDEX, 0xC8, 0x06);
		LOOP(5);
		dataArr[0]= 0x06;
		dataArr[1]= 0x06;
		
 		error_status = I2C_WriteMultipleData(I2C0, I2C0_SLAVE_ADDRESS, I2C_BYTE_INDEX, 0xC4, dataArr,2);
		LOOP(5);
		board_id = HREF_BOARD_ID;
 		if(error_status)
 			printf("Error in I2C_WriteMultipleData error = %d",error_status);
		
 	}
	else
		printf("\nunknown platform: chip ID = %x\n", read_data);

	/* Now modify bootargs to save the board_id, required for automatic platform detection */
	char * bootargs = getenv("bootargs");
	if(sizeof(Bootargs_buf) < strlen(bootargs)) {
		printf("ERROR: Insufficient temp buffer, bootargs not modified");
		return;
	}
	strcpy(Bootargs_buf, bootargs);
	bootargs = strstr (Bootargs_buf, "board_id=");
	if(bootargs){
	/*board_id parameter already present , modify correct value*/
		bootargs[9] = board_id;
	}
	else {
	 /*board_id parameter not present , append board_id with proper value*/
		strcat(Bootargs_buf, " board_id=1 ");
		/*point to the last character of string*/
		bootargs = Bootargs_buf + strlen(Bootargs_buf) -2;
		*bootargs = board_id;
	}
	/*Now save the new bootargs*/
	setenv("bootargs", Bootargs_buf);
	saveenv();
	//printf("Bootargs after platform detection:\n%s\n", getenv("bootargs"));
	return;
}


/*
 * mmc_legacy_init - called from commandline mmc init <dev>
 *
 * Initialise hardware and setup block device structure for fat and ext2
 * commands.
 */
int mmc_legacy_init(int dev)
{

	if (dev == DEV_EMMC) {
		printf("EMMC init\n");
		/* XXX: emmc_init() does write the MBR (called pib)! */
		emmc_init(EMMC_CARD_NUM);
		emmc_dev.if_type = IF_TYPE_MMC;
		emmc_dev.part_type = PART_TYPE_DOS;
		emmc_dev.dev = dev;
		emmc_dev.lun = 0;
		emmc_dev.type = 0;
		emmc_dev.blksz = 512;
		emmc_dev.lba = 0x80000; /* XXX: use real size, here 256 MB */
		sprintf((char*)emmc_dev.vendor, "Unknown vendor");
		sprintf((char*)emmc_dev.product, "Unknown product");
		sprintf((char*)emmc_dev.revision, "N/A");
		emmc_dev.removable = 0;
		emmc_dev.block_read = emmc_block_read;
		return 0;
	} else if (dev == DEV_MMC) {
		printf("MMC init\n");
		/* config extended GPIO pins for Level shifter and
		 * SDMMC_ENABLE */
		config_extended_gpio();
		init_mmc();
		return 0;
	}

	printf("mmc_legacy_init: unsupported device# %d\n", dev);
	return -1;
}

/* ========================================================================
   Name:        init_mmc
   Description: init multimedia card interface

   ======================================================================== */
static int init_mmc(void)
{
    t_mmc_error                   mmc_error;
    struct gpio_register * gpio_base_address;
    
    /* Initialize the base address of MMC-SD */
    mmc_error = mmc_init (0,CFG_MMC_BASE);
    
    if (MMC_OK != mmc_error)
    {
        printf("mmc_init():: %d \n", mmc_error);
        return 1 ;
    }    

    gpio_base_address = (void *) IO_ADDRESS(CFG_GPIO_0_BASE);
    gpio_base_address -> gpio_dats |= 0xFFC0000;
    gpio_base_address -> gpio_pdis &= ~0xFFC0000;

	if (mmc_hw_init() != 0) {
		printf("mmc_init: hw init failed\n");
	}
	mmc_dev.if_type = IF_TYPE_MMC;
	mmc_dev.part_type = PART_TYPE_DOS;
	mmc_dev.dev = 0;
	mmc_dev.lun = 0;
	mmc_dev.type = 0;
	mmc_dev.blksz = 512;
	mmc_dev.lba = 0x20000; /* XXX: use real size, here 64 MB */
	sprintf((char*)mmc_dev.vendor, "Unknown vendor");
	sprintf((char*)mmc_dev.product, "Unknown product");
	sprintf((char*)mmc_dev.revision, "N/A");
	mmc_dev.removable = 0;
	mmc_dev.block_read = mmc_block_read;
#ifdef CONFIG_CMD_FAT
	if (fat_register_device(&mmc_dev, 1) != 0) {
		printf("mmc_init: could not register as FAT device\n");
	}
#endif /* CONFIG_CMD_FAT */

	return 0;
}


/* ------------------------------- End of file ---------------------------- */

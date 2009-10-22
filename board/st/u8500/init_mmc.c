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

#ifdef CONFIG_CMD_FAT
#include <part.h>
#include <fat.h>
#endif 

#define LOOP(x) {int i;for(i=0;i<1000;i++);}
#ifdef CONFIG_CMD_FAT
static block_dev_desc_t mmc_dev;
u32  	CSD[4];
char    MMC_CmdBuffer[1024] ;

extern long do_fat_read (const char *filename, void *buffer, unsigned long maxsize,int dols);

block_dev_desc_t * mmc_get_dev(int dev)
{
	return (block_dev_desc_t *)(&mmc_dev);
}

unsigned long mmc_block_read(int dev, unsigned long blknr,lbaint_t blkcnt,
				void *dest)
{
	unsigned long src= blknr;

#if 0
	unsigned long rc = 0;
	unsigned long i;

	for(i = 0; i < blkcnt; i++)
	{		
		mmc_readblock (1, (u32) (512 * src), (u32 *)dest, 512, MMCPOLLING);
		rc ++;
		src++;
		dest += 512;		
	}
#endif
	printf("mmc_block_read: dev# %d, blknr %d, blkcnt %d\n",
			dev, blknr, blkcnt);
	/* card#, read offset, buffer, blocksize, transfer mode */
	mmc_readblock (1, (u32) (512 * src), (u32 *)dest, 512, MMCPOLLING);
	return 1;
}

t_mmc_error   mmc_fat_read_file (char *filename, u32 address, u32 FileSize)
{
	u8                      *mem_address = (u8 *) address;
	t_mmc_error                   mmc_error = MMC_OK;

	if((do_fat_read(filename,mem_address,FileSize,0)) < 0)
	{
		printf("error in reading %s file\n",filename);
		mmc_error = MMC_INVALID_PARAMETER;
	}
	return mmc_error;
}

int mmc_hw_init (void)
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

/*
 * init_mmc_fat - initialise MMC HW and register as FAT device
 */
int init_mmc_fat(void)
{    
    unsigned int size;
    unsigned int sizemult;
    unsigned long num_sects;
    unsigned long sect_size;	/* in bytes */
    unsigned long mmc_size;

    if(mmc_hw_init())
    {    	
    	goto end;
    }
    /*read block len*/
    sect_size = 1<<((CSD[2] << 12 )>>28);
    /*c_size */
    size = ((CSD[2] & 0x3ff)<<2) + (CSD[1]>>30) ;
    /*size of multiplier */
    sizemult = (CSD[1] << 14 )>>29;

    num_sects = (size+1)*(1<<(sizemult+2));
    /* memory capcity of the Inserted Card */
    mmc_size = (num_sects * sect_size)/(1024 * 1024);
    printf("Memory Card size = %d MiB \n", mmc_size);
    printf("sector size is %d, num_sects %d\n", sect_size, num_sects);

    mmc_dev.if_type = IF_TYPE_MMC;
    mmc_dev.part_type = PART_TYPE_DOS;
    mmc_dev.dev = 0;
    mmc_dev.lun = 0;
    mmc_dev.type = 0;
    mmc_dev.blksz = 512;
    mmc_dev.lba = num_sects; /* XXX: assumes 512 blksize */
    sprintf((char*)mmc_dev.vendor, "Unknown vendor");
    sprintf((char*)mmc_dev.product, "Unknown product");
    sprintf((char*)mmc_dev.revision, "N/A");
    mmc_dev.removable = 0;
    mmc_dev.block_read = mmc_block_read;

    /* do fat registration with the SD/MMC device*/
    if (fat_register_device(&mmc_dev, 1) != 0) {    	
		printf("could not register as FAT device\n");
		goto end;
    }
#if 0
    printf(" Size   \t    FileName \n");
    do_fat_read("/", NULL, 0, 1);
    /* do fat read for the commands to be executed*/
    if (do_fat_read("command.txt", &MMC_CmdBuffer[0], sizeof(MMC_CmdBuffer), 0)
		    == -1) {
    	printf(" No command.txt found in the Card\n");
    	return (0);
    }
    setenv("bootcmd", MMC_CmdBuffer);
#endif
    return(0);
end:
   gpio_altfuncdisable(GPIO_ALT_SD_CARD0, "MMC");
   mmc_disable();
   return 1;
}
#endif /* CONFIG_CMD_FAT */

#define I2C_SCL_FREQ            100000          /* I2C bus clock frequency.*/
#define I2C_INPUT_FREQ          48000000        /* Input clock frequency.*/
#define I2C0_SLAVE_ADDRESS	(0x84 >>1)		/*GPIO expander slave address*/
#define REG_CHIP_ID_INDEX	0x80
#define HS_MASTER_CODE 0x01  		/* High speed Master code */
#define TX_FIFO_THRESHOLD 0x4 /* The threshold below or equal to which the transmit FIFO generates interrupt */
#define RX_FIFO_THRESHOLD 0x4 /* The threshold above or equal to which the receive FIFO generates interrupt */
#define BURST_LENGTH 0 /* The burst length used in the DMA operation */
#define SLAVE_SETUP_TIME 14 /* Slave data setup time */


void config_extended_gpio(void)
{
  t_i2c_error    error_status;
  t_i2c_device_config  i2c_device_config;
  t_i2c_transfer_config i2c_transfer_config;
  t_i2c_error error_i2c;
  u8 read_data = 0;
  u8 dataArr[2]={0x06,0x06};
  
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
 		if(error_status)
 			printf("Error in I2C_WriteMultipleData error = %d",error_status);
		
 	}
	else
		printf("\nunknown platform: chip ID = %x\n", read_data);	
	return;
 	
	
}


/* ========================================================================
   Achraf
   Name:        init_mmc_card
   Description: init VIC, GPIO and MMC

   ======================================================================== */
int      mmc_legacy_init(int dev)
{
	/*config extended GPIO pins for Level shifter and SDMMC_ENABLE */
	config_extended_gpio();
    
    	init_mmc();    
	
#ifndef CONFIG_CMD_FAT   
    	if (display_file_list("*") == MMC_CMD_RSP_TIMEOUT)
    	{
        	printf("MMC card not available on board\n");
    	}
#endif
    return 0;
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
    gpio_base_address -> gpio_dats |= 0x1FFC0000;
    gpio_base_address -> gpio_pdis &= ~0x1FFC0000; 
      
#if 0
#ifdef CONFIG_CMD_FAT
    if(init_mmc_fat())
    {
    	printf(" MMC Card not found \n");
    }
#endif
#endif
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
	if (fat_register_device(&mmc_dev, 1) != 0) {    	
		printf("mmc_init: could not register as FAT device\n");
	}

	return 0;
}


/* ------------------------------- End of file ---------------------------- */

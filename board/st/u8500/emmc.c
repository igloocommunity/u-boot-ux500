/*
* (C) Copyright 2009
* ST-Ericsson, <www.stericsson.com>
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
#include "common.h"	/* XXX: Arrgghh! "common.h" includes <common.h> */
#include <command.h>
#include "mmc.h"
#include "emmc.h"
#include "gpio.h"
#include <asm/boottime.h>

#define PIB_EMMC_ADDR 0x00
/* ========================================================================
Name:        init_emmc
Description: init embedded multimedia card interface

======================================================================== */
int emmc_init(u8 card_num)
{
    t_mmc_error mmc_error;
    t_mmc_error response;
    gpio_error gpioerror;    
    int error;

#ifndef CONFIG_U8500_V1
/* Initialize the base address of eMMC */
    mmc_error = mmc_init(card_num, CFG_EMMC_BASE);

    if (MMC_OK != mmc_error) 
    {
        printf("emmc_init() %d \n", mmc_error);
        goto end;
    }
    
    /* Initialize the gpio alternate function for eMMC */
    struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_6_BASE);    
    p_gpio_register -> gpio_dats |= 0x0000FFE0;
    p_gpio_register -> gpio_pdis &= ~0x0000FFE0;
    
    //enable the alternate function of EMMC
    gpioerror = gpio_altfuncenable(GPIO_ALT_EMMC, "EMMC");
    if(gpioerror != GPIO_OK)
    {
        printf("emmc_init() gpio_altfuncenable %d failed\n", gpioerror);
        goto end;
    }

#else
/* Initialize the base address of PoP eMMC */
    mmc_error = mmc_init(card_num, CFG_POP_EMMC_BASE);

    if (MMC_OK != mmc_error) 
    {
        printf("emmc_init() %d \n", mmc_error);
        goto end;
    }
    //enable the alternate function of PoP EMMC
    gpioerror = gpio_altfuncenable(GPIO_ALT_POP_EMMC, "EMMC");
    if (gpioerror != GPIO_OK) {
        printf("emmc_init() gpio_altfuncenable %d failed \n",
               gpioerror);
        goto end;
    }
#endif
    //Power-on the controller
    response = mmc_poweron(card_num);
    if (response != MMC_OK) 
    {
    	 printf("Error in eMMC power on, response is %d\n",response);
         goto end;     
    }
    // Initialise the cards,get CID and CSD on the bus
    response = mmc_initializeCards(card_num);

    if (response != MMC_OK) 
    {
        printf(" Error in eMMC initialization\n");
        goto end;
    }
    
    error = emmc_write_pib();
    if(error)
	    printf("PIB info writing into eMMC failed\n");
    printf("eMMC done\n");
    
    return 0;

end:
#ifndef CONFIG_U8500_V1
    gpio_altfuncdisable(GPIO_ALT_EMMC, "EMMC");
#else
    gpio_altfuncdisable(GPIO_ALT_POP_EMMC, "EMMC");
#endif
    mmc_poweroff(card_num);
    return 1;
}

int emmc_write_pib(void)
{
    int i;
    t_mmc_error mmc_error;
    u32 block_offset = PIB_EMMC_ADDR;
    u8 emmc_last_sector[512];
    u8 card_num = 4;

    for (i = 0; i < 0x1BF; i++) {
        emmc_last_sector[i] = 0;
    }
    emmc_last_sector[0x1BF] = 0x03;
    emmc_last_sector[0x1C0] = 0xD0;
    emmc_last_sector[0x1C1] = 0xFF;
    emmc_last_sector[0x1C2] = 0x83;
    emmc_last_sector[0x1C3] = 0x03;
    emmc_last_sector[0x1C4] = 0xD0;
    emmc_last_sector[0x1C5] = 0xFF;
    emmc_last_sector[0x1C6] = 0x00;
    emmc_last_sector[0x1C7] = 0x00;
    emmc_last_sector[0x1C8] = 0x0A;
    emmc_last_sector[0x1C9] = 0x00;
    emmc_last_sector[0x1CA] = 0x00;
    emmc_last_sector[0x1CB] = 0x40;
    emmc_last_sector[0x1CC] = 0x00;
    emmc_last_sector[0x1CD] = 0x00;
    emmc_last_sector[0x1CE] = 0x00;
    emmc_last_sector[0x1CF] = 0x03;
    emmc_last_sector[0x1D0] = 0xD0;
    emmc_last_sector[0x1D1] = 0xFF;
    emmc_last_sector[0x1D2] = 0x83;
    emmc_last_sector[0x1D3] = 0x03;
    emmc_last_sector[0x1D4] = 0xD0;
    emmc_last_sector[0x1D5] = 0xFF;
    emmc_last_sector[0x1D6] = 0x00;
    emmc_last_sector[0x1D7] = 0x40;
    emmc_last_sector[0x1D8] = 0x0A;
    emmc_last_sector[0x1D9] = 0x00;
    emmc_last_sector[0x1DA] = 0x00;
    emmc_last_sector[0x1DB] = 0x00;
    emmc_last_sector[0x1DC] = 0x08;
    emmc_last_sector[0x1DD] = 0x00;
    emmc_last_sector[0x1DE] = 0x00;
    emmc_last_sector[0x1DF] = 0x03;
    emmc_last_sector[0x1E0] = 0xD0;
    emmc_last_sector[0x1E1] = 0xFF;
    emmc_last_sector[0x1E2] = 0x83;
    emmc_last_sector[0x1E3] = 0x03;
    emmc_last_sector[0x1E4] = 0xD0;
    emmc_last_sector[0x1E5] = 0xFF;
    emmc_last_sector[0x1E6] = 0x00;
    emmc_last_sector[0x1E7] = 0x40;
    emmc_last_sector[0x1E8] = 0x12;
    emmc_last_sector[0x1E9] = 0x00;
    emmc_last_sector[0x1EA] = 0x00;
    emmc_last_sector[0x1EB] = 0xC0;
    emmc_last_sector[0x1EC] = 0x22;
    emmc_last_sector[0x1ED] = 0x00;
    emmc_last_sector[0x1EE] = 0x00;
    emmc_last_sector[0x1EF] = 0x03;
    emmc_last_sector[0x1F0] = 0xD0;
    emmc_last_sector[0x1F1] = 0xFF;
    emmc_last_sector[0x1F2] = 0x0C;
    emmc_last_sector[0x1F3] = 0x03;
    emmc_last_sector[0x1F4] = 0xD0;
    emmc_last_sector[0x1F5] = 0xFF;
    emmc_last_sector[0x1F6] = 0x00;
    emmc_last_sector[0x1F7] = 0x00;
    emmc_last_sector[0x1F8] = 0x35;
    emmc_last_sector[0x1F9] = 0x00;
    emmc_last_sector[0x1FA] = 0x00;
    emmc_last_sector[0x1FB] = 0xA0;
    emmc_last_sector[0x1FC] = 0xB9;
    emmc_last_sector[0x1FD] = 0x00;
    emmc_last_sector[0x1FE] = 0x55;
    emmc_last_sector[0x1FF] = 0xAA;
    
/*  HACK required for HREF board as erase block size = 512KB */  
/*
    mmc_error = mmc_erase(card_num, 0x0, 0x1FF);
    if (mmc_error != MMC_OK) {
        printf(" eMMC erase failed in PIB \n");
        return 1;
    }
*/
    mmc_error =
        mmc_writeblocks(card_num, block_offset, (u32 *) emmc_last_sector,
                512, 1);
    if (mmc_error != MMC_OK) {
        printf(" eMMC PIB write failed \n");
        return 1;
    }
    return 0;
}

int emmc_erase(u32 start, u32 end)
{
    t_mmc_error mmc_error;
    u8 card_num = 4;
    printf("emmc erase start \n");
    mmc_error = mmc_erase(card_num, start, end);
    if (mmc_error != MMC_OK) {
        printf(" eMMC erase failed \n");
        return 1;
    }
    printf("emmc erase done \n");
    return 0;
}

int emmc_read(u32 block_offset, u32 read_buffer, u32 filesize)
{
    	t_mmc_error mmc_error;
    	u32 remaining;
    	u8 card_num = 4;
    	u8 *mem_address = (u8 *) read_buffer;    
    	u32 n=filesize,blocks;
 
    	remaining = filesize;

    	printf(" eMMC read start filesize=0x%x \n", filesize);    	

	boottime_tag_load_kernel();

    	blocks = (n%512==0)?(n/512):(n/512)+1;   	
    	
    	while(blocks>=8)
    	{    		
    		mmc_error = mmc_readblocks(card_num, block_offset, (u32 *) mem_address,	512, 8);
        	if (mmc_error != MMC_OK) 
        	{
        		printf(" eMMC read blocks failed \n");
            		return 1;
        	}

        	block_offset += 4096;
        	mem_address += 4096;
        	blocks -=8;
        	remaining -= 4096;        	
        }
        if(blocks)
    	{    		
    		mmc_error = mmc_readblocks(card_num, block_offset, (u32 *) mem_address,	512, blocks);
        	if (mmc_error != MMC_OK) 
        	{
        		printf(" eMMC read blocks failed \n");
        		return 1;
        	}        	    	
        }
    
     	printf(" eMMC read done \n");
    	return 0;
}

int emmc_write(u32 block_offset, u32 write_buffer, u32 filesize)
{
    	t_mmc_error mmc_error;
    	u32 remaining;
    	u8 card_num = 4;
    	u8 *mem_address = (u8 *) write_buffer;
    	u32 n=filesize,blocks;

    	remaining = filesize;

    	printf(" eMMC write start filesize=0x%x \n", filesize);    	
    
    	blocks = (n%512==0)?(n/512):(n/512)+1;    	        
    	while(blocks>=8)
    	{    		
    		mmc_error = mmc_writeblocks(card_num, block_offset, (u32 *) mem_address,512, 8);
        	if (mmc_error != MMC_OK) 
        	{
        		printf(" eMMC write blocks failed \n");
            		return 1;
        	}

        	block_offset += 4096;
        	mem_address += 4096;
        	blocks -=8;
        	remaining -= 4096;        	
        }
        if(blocks)
    	{
    		
    		mmc_error = mmc_writeblocks(card_num, block_offset, (u32 *) mem_address,512, blocks);
        	if (mmc_error != MMC_OK) 
        	{
        		printf(" eMMC write blocks failed \n");
        		return 1;
        	}       	
        	
        }	
    
    	printf(" eMMC write done \n");
    	return 0;
}

/*
 * command line commands
 */
#ifdef CONFIG_CMD_EMMC
int do_emmc_erase (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	u32       start_address;
    	u32       end_address;    	
    	int       load_result = 1;
    	u32       error_name  = 0;

    	start_address  = simple_strtoul (argv[1],0,16);
    	end_address  = simple_strtoul (argv[2],0,16);        
	
	printf("emmc_erase :: start address = %x end_address=0x%x\n",start_address,end_address);
            
        load_result      = emmc_erase(start_address,end_address);
        if (load_result != 0)
        {
            error_name   = (unsigned long) (-load_result);
            printf("emmc_erase error : failed  \n");         
        }
        return(0);    
}

U_BOOT_CMD(
	emmc_erase,	3,	0,	do_emmc_erase,
	"- erase the eMMC flash \n",
	"start_address- start address of the eMMC block\n"
	"end_address- end address of the eMMC block\n"	
);

int do_emmc_read (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	u32       ram_address;
    	u32       block_offset;
    	u32       filesize;    	
    	int       load_result = 1;
    	u32       error_name  = 0;

    	ram_address  	= simple_strtoul (argv[1],0,16);
        block_offset 	= simple_strtoul (argv[2],0,16);
        filesize  	= simple_strtoul (argv[3],0,16);
	
	printf("emmc_read :: ram address = 0x%x block address=0x%x \n",ram_address,block_offset);
            
        load_result      = emmc_read(block_offset,ram_address,filesize);
        if (load_result != 0)
        {
            error_name   = (unsigned long) (-load_result);
            printf("emmc_read error : in reading data from eMMC block \n");         
        }
        return(0);    
}

U_BOOT_CMD(
	emmc_read,	4,	0,	do_emmc_read,
	"- read file from emmc flash \n",
	"ram_address - read from eMMC and copy into ram address\n"
	"block_offset - address to read the file from the eMMC block \n"
	"filesize - size of the file \n"
);

int do_emmc_write (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	u32       ram_address;
    	u32       block_offset;
    	u32       filesize;    	
    	int       load_result = 1;
    	u32       error_name  = 0;

    	ram_address  	= simple_strtoul (argv[1],0,16);
        block_offset 	= simple_strtoul (argv[2],0,16);
        filesize  	= simple_strtoul (argv[3],0,16);
	
	printf("emmc_write :: ram address = %x block address=0x%x \n",ram_address,block_offset);
            
        load_result      = emmc_write(block_offset,ram_address,filesize);
        if (load_result != 0)
        {
            error_name   = (unsigned long) (-load_result);
            printf("emmc_read error : in writing data into eMMC block \n");         
        }
        return(0);    
}

U_BOOT_CMD(
	emmc_write,	4,	0,	do_emmc_write,
	"- write file from emmc flash \n",
	"ram_address - write to eMMC by copying from ram address\n"
	"block_offset - address to write the file into the eMMC block \n"
	"filesize - size of the file \n"
);

#endif	/* CONFIG_CMD_EMMC */
/* ------------------------------- End of file ---------------------------- */

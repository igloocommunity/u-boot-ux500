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
#include <linux/ctype.h>

#include <common.h>
#include <command.h>
#include "gpio.h"
#include "mmc.h"                // MMC api
#include "mmc_utils.h"          // to access MMC
#include "mmc_p.h"              // t_mmc_register definition

u32       MMCBuffer[1024] ;
static int              IS_SD_CARD;

/* --- exported from other modules ---------------------------------------- */
static  u8               selected_card = 0;
extern  t_mmc_error           mmc_cmderror_2            (void);
extern  t_mmc_error           mmc_cmdresp2error_2       (void);
extern  t_mmc_error           mmc_cmdresp3error_2       (void);
extern  t_mmc_error           mmc_cmdresp145error_2     (u8 cmd);
extern  t_mmc_error           mmc_cmdreadresp           (u32 * tempbuff);
extern  t_mmc_error           mmc_waitcmdreadresp       (void);
extern  t_mmc_error           mmc_cmdwriteresp          (u8 cardno, u32 * tempbuff, u16 total_num_of_bytes);
extern  t_mmc_error           mmc_waitcmdwriteresp      (u8 cardno, u32 * tempbuff, u16 total_num_of_bytes);

extern  t_mmc_register  *t_mmc0;
extern  u32         t_mmc_rel_addr;

t_mmc_boot_record             mmc_boot_record;

void mmc_copyByteH (u8 * sourceAddr, u8 * destAddress, int size)
{
    int                           i;
    for (i = 0; i < size; i++)
    {
        *(destAddress + i) = *(sourceAddr + i);
    }
}

void mmc_shift_memH (int num_of_words, u32 * start_address, u32 * dest_address, u32 shift)
{
    u32                     *address;
    int                           word = 0;
    u32                      high;
    int                           i, word_offset, bit_offset;

    word_offset = (int) (shift / 32);
    bit_offset  = (int) (shift % 32);
    for (i = 0; i < num_of_words; i++)
    {
        *(dest_address + i) = 0;
    }
    for (i = word_offset; i < num_of_words; i++)
    {
        *(dest_address + i - word_offset) = *(start_address + i);
    }
    address = dest_address;
    for (word = 0; word < num_of_words - 1; word++)
    {
        *address = (*address) >> bit_offset;
        address++;
        high = (*address << (32 - bit_offset));
        *(address - 1) |= high;
    }
    *address = (*address) >> bit_offset;
}

u8 convert_from_bytes_to_power_of_two (u16 no_of_bytes)
{
    u8                       count = 0;
    while (no_of_bytes != 1)
    {
        no_of_bytes >>= 1;
        count++;
    }
    return count;
}

t_mmc_error mmc_enable ()
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_command_control         commcontrol;
    t_bool                        validvoltage = FALSE;
    u32                      arg, response;
    int i;

    IS_SD_CARD = 0;

    selected_card       = 0;

    t_mmc0->mmc_Power = 0x1BF;
    t_mmc0->mmc_Clock = 0x41FF;

    commcontrol.IsRespExpected  = FALSE;
    commcontrol.IsLongResp      = FALSE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error   = mmc_sendcommand (MMC_GO_IDLE_STATE, 0x00000000, commcontrol);

    error   = mmc_cmderror_2 ();

    //// Added by Chris Sebastian for SD Card support:
    //// Send ACMD41.  If we do not get a response, it is not an SD card.
    ////               If we do get a response, loop until card is not busy.

    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = FALSE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    for(i = 200; i; i--) {
        error = mmc_sendcommand (MMC_APP_CMD, 0x00000000, commcontrol);      // Let the card know that the next command is an app-cmd.
        error = mmc_cmdresp145error_2(MMC_APP_CMD);
        error = mmc_sendcommand (SD_SEND_OP_COND, 0x00060000, commcontrol);  // I got the 0x00060000 from Linux kernel source.
        error = mmc_cmdresp3error_2 ();
        t_mmc0->mmc_Clear = ClrStaticFlags; //clear all the static status flags.  CmdResp3Error_2 leaves some bits set.
        if (t_mmc0->mmc_Response0 & 0x80000000) {
            printf("SD card detected \n");
            IS_SD_CARD = 1;
            return MMC_OK; // SD CARD DETECTED.  Do not continue with MMC init logic.  Quick escape!
        }
    }

    commcontrol.IsRespExpected  = FALSE;
    commcontrol.IsLongResp      = FALSE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error   = mmc_sendcommand (MMC_GO_IDLE_STATE, 0x00000000, commcontrol);
    error   = mmc_cmderror_2 ();

    arg     = 0x00060000;  // High Voltage MMC, byte mode.  Was 0x00FFC000, but that writes to reserved bits...
    while (!validvoltage)
    {
        // send CMD1 (SEND_OP_COND)
        commcontrol.IsRespExpected = TRUE;
        error   = mmc_sendcommand (MMC_SEND_OP_COND, arg, commcontrol);

        error   = mmc_cmdresp3error_2 ();

        if (error != MMC_OK)
        {
            goto end;
        }
        mmc_getresponse (MMC_SHORT_RESP, &response);

        arg = response;
        validvoltage = (t_bool) (((response >> 31) == 1) ? 1 : 0);

    }
end:
    return error;
}

t_mmc_error  mmc_disable ()
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_power_state             powerstate = MMC_POWER_OFF;

    error   = mmc_setpowerstate (powerstate);
    return error;
}

t_mmc_error                   mmc_initCard ()
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_command_control         commcontrol;
    t_mmc_bus_configuration       busconfig;
    t_mmc_clock_control           clockcontrol;

    // send CMD2 (ALL_SEND_CID)
    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = TRUE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error = mmc_sendcommand (MMC_ALL_SEND_CID, 0x00000000, commcontrol);
    error = mmc_cmdresp2error_2 ();
    // error is MMC_CMD_CRC_FAIL on COB board ?!
    // if( error != MMC_OK)
    // goto end;

    // send CMD3 (SET_RELATIVE_ADDR)

    commcontrol.IsLongResp      = FALSE;
    error               = mmc_sendcommand       (MMC_SET_REL_ADDR, t_mmc_rel_addr, commcontrol);
    error               = mmc_cmdresp145error_2 (MMC_SET_REL_ADDR);
    if(IS_SD_CARD) {
        t_mmc_rel_addr = t_mmc0->mmc_Response0 & 0xFFFF0000;
    }

    if (error != MMC_OK)
    {
        goto end;
    }
    busconfig.mode      = MMC_PUSH_PULL;
    busconfig.rodctrl   = MMC_DISABLE;
    error               = mmc_configbus (busconfig);
    clockcontrol.pwrsave= MMC_DISABLE;
    clockcontrol.bypass = MMC_DISABLE;
    clockcontrol.widebus= MMC_DISABLE;
    error = mmc_setclockfrequency(0x00);	/* 26 MHz */
    if (error != MMC_OK)
    {
        goto end;
    }
    error = mmc_configclockcontrol (clockcontrol);
    if (error != MMC_OK)
    {
        goto end;
    }
end:
    return error;
}

t_mmc_error mmc_readblock (u8 cardno, u32 addr, u32 * readbuff, u16 blocksize, t_mmc_transfer_mode mmc_transfer_mode)
{
    t_mmc_error                   error = MMC_OK;
    u8                       power;
    t_mmc_command_control         commcontrol;
    u32                      response;

    commcontrol.IsRespExpected  = TRUE              ;
    commcontrol.IsLongResp      = FALSE             ;
    commcontrol.IsInterruptMode = FALSE             ;
    commcontrol.IsPending       = FALSE             ;
    commcontrol.cmdpath         = MMC_ENABLE        ;
    error                       = mmc_setdatapath   (MMC_DISABLE);
    error                       = mmc_handledma     (MMC_DISABLE);
    if (error != MMC_OK)
    {
        goto end;
    }
    // send command for selecting the card
    if (cardno != selected_card)
    {
        error   = mmc_sendcommand       (MMC_SEL_DESEL_CARD, t_mmc_rel_addr, commcontrol);
        error   = mmc_cmdresp145error_2 (MMC_SEL_DESEL_CARD);
        if (error != MMC_OK)
        {
            goto end;
        }
        else
        {
            selected_card = cardno;
        }
    }
    error       = mmc_getresponse (MMC_SHORT_RESP, &response);
    if (response & 0x02000000)
    {
        return MMC_LOCK_UNLOCK_FAILED;
    }
    // set the block size,both on controller and card
    if ((blocksize > 0) && (blocksize <= 2048) && ((blocksize & (blocksize - 1)) == 0))
    {
        power = convert_from_bytes_to_power_of_two (blocksize);
        error = mmc_setdatablocklength  (power);
        error = mmc_sendcommand         (MMC_SET_BLOCKLEN, (u32) blocksize, commcontrol);
        error = mmc_cmdresp145error_2   (MMC_SET_BLOCKLEN);
        if (error != MMC_OK)
        {
            return error;
        }
    }
    else
    {
        error = MMC_INVALID_PARAMETER;
        goto end;
    }
    error = mmc_setdatalength           (blocksize  );
    error = mmc_setdatatimeout          (0xefffffff );
    error = mmc_settransferdirection    (MMC_READ   );
    error = mmc_settransfertype         (MMC_BLOCK  );
    error = mmc_setdatapath             (MMC_ENABLE);
    //t_mmc0->mmc_DataCtrl	|=             (ReadDir & ~StreamMode) | DataPathEnable;
    error = mmc_sendcommand             (MMC_READ_SINGLE_BLOCK, addr, commcontrol);
    error = mmc_cmdresp145error_2       (MMC_READ_SINGLE_BLOCK);
    if (error != MMC_OK)
    {
        goto end;
    }
    if (mmc_transfer_mode == MMCDMA)
    {
        error = mmc_waitcmdreadresp ();
    }
    else
    {
        error = mmc_cmdreadresp (readbuff);
    }
    if (error != MMC_OK)
    {
        goto end;
    }
end:
    return error;
}

t_mmc_error mmc_writeblock (u8 cardno, u32 addr, u32 * writebuff, u16 blocksize, t_mmc_transfer_mode mmc_transfer_mode)
{
    t_mmc_error                   error = MMC_OK;
    u8                       power;
    t_mmc_command_control         commcontrol;
    u32                      response;

    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = FALSE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error = mmc_setdatapath         (MMC_DISABLE);
    error = mmc_handledma           (MMC_DISABLE);
    error = mmc_settransferdirection(MMC_WRITE);
    error = mmc_settransfertype     (MMC_BLOCK);

    if (cardno != selected_card)
    {
        error = mmc_sendcommand (MMC_SEL_DESEL_CARD, t_mmc_rel_addr, commcontrol);
        error = mmc_cmdresp145error_2 (MMC_SEL_DESEL_CARD);
        if (error != MMC_OK)
        {
            goto end;
        }
        else
        {
            selected_card = cardno;
        }
    }

    mmc_getresponse (MMC_SHORT_RESP, &response);
    if (response & 0x02000000)
    {
        return MMC_LOCK_UNLOCK_FAILED;
    }
    // set the block size,both on controller and card
    if ((blocksize > 0) && (blocksize <= 2048) && (((blocksize & (blocksize - 1)) == 0)))
    {
        power = convert_from_bytes_to_power_of_two (blocksize);
        error = mmc_setdatablocklength (power);
        error = mmc_sendcommand (MMC_SET_BLOCKLEN, (u32) blocksize, commcontrol);
        error = mmc_cmdresp145error_2 (MMC_SET_BLOCKLEN);
        if (error != MMC_OK)
        {
            goto end;
        }
    }
    else
    {
        error = MMC_INVALID_PARAMETER;
        goto end;
    }

    error = mmc_setdatalength           (blocksize);
    error = mmc_setdatatimeout          (0xefffffff);
    error = mmc_settransferdirection    (MMC_WRITE);
    error = mmc_settransfertype         (MMC_BLOCK);
    error = mmc_setdatapath         (MMC_ENABLE);
    error = mmc_sendcommand         (MMC_WRITE_SINGLE_BLOCK, addr, commcontrol);
    error = mmc_cmdresp145error_2   (MMC_WRITE_SINGLE_BLOCK);
    if (error != MMC_OK)
    {
        goto end;
    }
    if (mmc_transfer_mode == MMCDMA)
    {
        error = mmc_waitcmdwriteresp (cardno, writebuff, blocksize);
    }
    else
    {
        error = mmc_cmdwriteresp (cardno, writebuff, blocksize);
    }
    if (error != MMC_OK)
    {
        goto end;
    }
end:
    return error;
}

t_mmc_error mmc_readcsd (u32 * CSD)
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_command_control         commcontrol;
    u32                      buf[4];

    // send CMD9 (SEND CSD)
    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = TRUE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error = mmc_sendcommand     (MMC_SEND_CSD,  t_mmc_rel_addr, commcontrol);
    error = mmc_cmdresp2error_2 ();
    if (error != MMC_OK)
    {
        goto end;
    }
    error = mmc_getresponse (MMC_LONG_RESP, buf);
    CSD[0] = buf[3];
    CSD[1] = buf[2];
    CSD[2] = buf[1];
    CSD[3] = buf[0];
end:
    return error;
}

t_mmc_error mmc_select_n_switch(void)
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_command_control         commcontrol;
    /* t_mmc_clock_control           clockcontrol; */
    u32                           response;
    u8                            cardno = 1;

    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = FALSE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    // send command for selecting the card
    if (cardno != selected_card)
    {
        error = mmc_sendcommand(MMC_SEL_DESEL_CARD, t_mmc_rel_addr, commcontrol);
        error = mmc_cmdresp145error_2(MMC_SEL_DESEL_CARD);
        if (error != MMC_OK)
        {
            goto end;
        }
        else
        {
            selected_card = cardno;
        }
    }
    error = mmc_getresponse(MMC_SHORT_RESP, &response);
    if (response & 0x02000000)
    {
        error =  MMC_LOCK_UNLOCK_FAILED;
        goto end;
    }
    /* XXX: what is this, why is it commented out, why is it here at all? */
    /*
    error = mmc_sendcommand (MMC_APP_CMD, 0x00000000, commcontrol);
    error = mmc_cmdresp145error_2(MMC_APP_CMD);
    error = mmc_sendcommand (6, 0x2, commcontrol);
    error = mmc_cmdresp145error_2(6);
    if (error != MMC_OK)
    {
        goto end;
    }
    clockcontrol.pwrsave= MMC_DISABLE;
    clockcontrol.bypass = MMC_DISABLE;
    clockcontrol.widebus= MMC_ENABLE;
    error = mmc_configclockcontrol (clockcontrol);
    if (error != MMC_OK)
    {
        goto end;
    }
    */
end:
    return error;
}

t_mmc_error mmc_readcid (u32 * CID)
{
    t_mmc_error                   error = MMC_OK;
    t_mmc_command_control         commcontrol;
    u32                      buf[4];

    // send CMD9 (SEND CSD)
    commcontrol.IsRespExpected  = TRUE;
    commcontrol.IsLongResp      = TRUE;
    commcontrol.IsInterruptMode = FALSE;
    commcontrol.IsPending       = FALSE;
    commcontrol.cmdpath         = MMC_ENABLE;

    error = mmc_sendcommand     (MMC_SEND_CID, t_mmc_rel_addr, commcontrol);
    error = mmc_cmdresp2error_2 ();
    if (error != MMC_OK)
    {
        goto end;
    }
    error = mmc_getresponse (MMC_LONG_RESP, buf);
    CID[0] = buf[3];
    CID[1] = buf[2];
    CID[2] = buf[1];
    CID[3] = buf[0];
end:
    return error;
}

t_mmc_error mmc_read_file (char *filename, u32 address, u32 * FileSize)
{
    u8                      *mem_address = (u8 *) address;
    u8                       sector[512];
    u16                      BytesPerSector;
    u32                      BRStartSector;
    u32                      FATStartSector;
    u32                      DataStartSector;
    u32                      FileStartSector;
    u16                      SectorsPerFAT;
    u8                       SectorsPerCluster;
    u8                       k;
    u16                      MaxRDEntries;
    u16                      ReservedSectors;
    u32                      RDStartSector, j;
    char                          nomefile[12] = "           ";
    int                           filelength;
    u16                      FileStartCluster = 0;
    t_mmc_error                   response, response2;
    int                           i, goout, found, result = PASS;
    u32                      CSD[4];
    char                          mmcfilename[] = "           ";
    char                         *dotpos;

    /////////// Attention please:  /////////////////////
    //                                                //
    // if other Alternate function apart from MMCI    //
    // are selected, the MMCI I/F has some problems   //
    //                                                //
    ////////////////////////////////////////////////////

    gpio_altfuncenable(GPIO_ALT_SD_CARD0, "MMC");
    // Power-on the controller
    response = mmc_enable ();

    if (response != MMC_OK)
    {

        response = mmc_enable (); // This actually IS necessary because it takes time for newly-inserted cards initialize.  ~Chris S.
        if (response != MMC_OK)
        {
            printf ("Error in card power on\n");
            result = FAIL;
            goto end;
        }
    }
    // Initialise the cards on the bus, if any
    response = mmc_initCard ();

    if (response != MMC_OK)
    {
        printf ("Error in card initialization\n");
        result = FAIL;
        goto end;
    }
    // Get card info: CID, CSD, RCA registers
    response = mmc_readcsd (CSD);

    if (response != MMC_OK)
    {
        printf ("Error while fetching card info\n");
        result = FAIL;
        goto end;
    }

    // Select card and switch to 4-Bit, TODO HS if possible
    response = mmc_select_n_switch ();

    if (response != MMC_OK)
    {
        printf ("Error while select or switching to 4-bit/HS\n");
        goto end;
    }

    // Read the MBR
    response = mmc_readblock (1, 0, (u32 *) sector, 512, MMCPOLLING);

    if (response != MMC_OK)
    {
        printf ("Error while reading boot record\n");
        result = FAIL;
        goto end;
    }
    if (sector[446] == 0x00 || sector[446] == 0x80)
    {                           // found a MBR at the beginning of card
        BRStartSector = sector[454];
    }
    else
    {                           // BR at the beginning of card
        BRStartSector = 0x00;
    }
    // Read the BR
    response = mmc_readblock (1, (u32) (512 * BRStartSector), (u32 *) & mmc_boot_record, 512, MMCPOLLING);

    k = 0;
    while (response == MMC_DATA_CRC_FAIL && (k < 2))
    {
        response = mmc_readblock (1, (u32) (512 * BRStartSector), (u32 *) & mmc_boot_record, 512, MMCPOLLING);
        k++;
    }
    if (response != MMC_OK)
    {
        printf ("Error while reading boot record\n");
        result = FAIL;
        goto end;
    }
    mmc_copyByteH        (mmc_boot_record.BytesPerSector, (u8 *) & BytesPerSector, 2);
    mmc_copyByteH        (mmc_boot_record.ReservedSectors, (u8 *) & ReservedSectors, 2);
    FATStartSector      = BRStartSector + ReservedSectors;   // the field at offset 15 contains the number of reserved sectors at
    // the beginning of the media including the boot sector
    mmc_copyByteH        (mmc_boot_record.SectorsPerFat, (u8 *) & SectorsPerFAT, 2);
    SectorsPerCluster   = mmc_boot_record.SectorsPerCluster[0];
    mmc_copyByteH        (mmc_boot_record.NumOfRootEntries, (u8 *) & MaxRDEntries, 2);
    FATStartSector      += SectorsPerFAT;
    RDStartSector       = FATStartSector + SectorsPerFAT;
    DataStartSector     = RDStartSector + (u32) ((MaxRDEntries * 32) / 512);



    // convert filename into mmc compatible file name (upper case , <= 8+3 char, no dot, no extension)
    filelength = strlen (filename);
    dotpos = strchr (filename, '.');
    if (dotpos == NULL)
    {
        // no dot
        if (filelength <= 8)
        {
            for (j = 0; j < filelength; j++)
            {
                mmcfilename[j] = (char) toupper (filename[j]);
            }
        }
        else
        {
            for (j = 0; j < 6; j++)
            {
                mmcfilename[j] = (char) toupper (filename[j]);
            }
            mmcfilename[7] = '~';
            mmcfilename[8] = '1';
            filelength = 8;
        }
    }
    else
    {
        // dot
        if ((dotpos - filename) <= 8)
        {
            for (j = 0; j < (dotpos - filename); j++)
            {
                mmcfilename[j] = (char) toupper (filename[j]);
            }
            for (; j < 8; j++)
            {
                mmcfilename[j] = ' ';
            }
            // copy 3 char after .
            mmcfilename[j++] = (char) toupper (dotpos[1]);
            mmcfilename[j++] = (char) toupper (dotpos[2]);
            mmcfilename[j++] = (char) toupper (dotpos[3]);

        }
        else
        {
            for (j = 0; j < 6; j++)
            {
                mmcfilename[j] = (char) toupper (filename[j]);
            }
            mmcfilename[6] = '~';
            mmcfilename[7] = '1';
            // copy 3 char after .
            mmcfilename[8] = (char) toupper (dotpos[1]);
            mmcfilename[9] = (char) toupper (dotpos[2]);
            mmcfilename[10] = (char) toupper (dotpos[3]);
        }
        filelength = 11;
    }
    mmcfilename[11] = '\0';


    // search Root Directory for entry filename
    goout = 0;
    found = 0;
    for (j = 0; j < (DataStartSector - RDStartSector); j++)
    {
        if (goout == 1)
            break;
        response = mmc_readblock (1, (u32) ((RDStartSector + j) * 512), (u32 *) sector, 512, MMCPOLLING);
        if (response != MMC_OK)
        {
            printf ("Error while reading root directory\n");
            result = FAIL;
            goto end;
        }

        for (i = 0; i < 512; i += 32)
        {
            strncpy (nomefile, (char *) &sector[i], filelength);
            if (strcmp (nomefile, mmcfilename) == 0)
            {
                mmc_copyByteH (&sector[i + 26], (u8 *) & FileStartCluster, 2);
                mmc_copyByteH (&sector[i + 28], (u8 *) FileSize, 4);
                FileStartCluster -= 2;
                goout = 1;
                found = 1;
                break;
            }
            if (nomefile[0] == 0)
            {                   // end of Root Directory
                goout = 1;
                break;
            }
        }
    }

    // size of file                : *FileSize ( 32 bits quantum)
    // number of bytes per sector  : BytesPerSector
    // Beginning of file           : FileStartSector

    printf ("Bytes per sector    : %d \n", BytesPerSector    );
    printf ("Sectors per cluster : %d \n", SectorsPerCluster );
    if (found)
    {
        u32    remaining =  *FileSize     ;
        u32    count;
        u32    burstsize = BytesPerSector ;

        FileStartSector     = ( DataStartSector + (u32)(FileStartCluster * SectorsPerCluster))  ;
        printf                 ("Reading file %s from MMC ..., start sector = %d, address 0x%x \n",filename,FileStartSector,address);
        count               = SectorsPerCluster - (FileStartSector % SectorsPerCluster) ;
        FileStartSector     = FileStartSector * BytesPerSector; // the first burst might be different to align of clusters

        do
        {
            if ( remaining >= burstsize )
            {   // read straight burst

                k = 0;
                do
                {
                    response = mmc_readblock (1, (u32) FileStartSector ,  (u32 *) mem_address, burstsize , MMCPOLLING);
                    k++;

                }   while ((response==MMC_DATA_CRC_FAIL)&&(k<4));
                if ( k != 1) printf ("!");
                FileStartSector += burstsize;
                mem_address     += burstsize;
                remaining       -= burstsize;
            }
            else
            {   // read burst +1 and perform memcpy

                burstsize = ((remaining + (BytesPerSector-1))/BytesPerSector) * BytesPerSector ;
                k = 0;
                do
                {
                    response = mmc_readblock (1, (u32) FileStartSector ,  (u32 *) MMCBuffer , burstsize , MMCPOLLING);
                    k++;
                }   while ((response==MMC_DATA_CRC_FAIL)&&(k<4));
                if ( k != 1) printf ("!");
                memcpy ((char *) mem_address, (char *)MMCBuffer,remaining);
                remaining = 0;
            }
            if (response != MMC_OK)
            {
                printf ("Error while reading file %s\n", filename);
                result = FAIL;
                goto end;
            }
        }   while ( remaining );

    }
    else
    {
        printf ("Unable to find %s on Multi Media Card\n", filename);
        response = MMC_INVALID_PARAMETER;
        result = FAIL;
        goto end;
    }

end:

    return response;
}

/*
 * command line commands
 */

static char       mmc_cmdbuffer[1024] ;

int copy_file_mmc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned long	address;
	u32		filesize;
	int		load_result;
	char		filename[30];

	strcpy(filename , argv[1]);
        address  = simple_strtoul (argv[2], 0, 16);

	printf("copy_file_mmc : filename = %s\n", filename);
	printf("copy_file_mmc : address = %lx\n", address);

        load_result      = mmc_read_file(filename, address, &filesize);
        if (load_result != 0)
        {
            printf("copy_file_mmc error : in loading file \n");
        }
        return(load_result);
}

int mmc_read_cmd_file (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	u32	filesize;
	int	load_result;

        load_result      = mmc_read_file("command.txt",
			(unsigned long)&mmc_cmdbuffer, &filesize);
        if (load_result != 0) {
            printf("mmc_read_cmd_file error : in loading file \n");
        } else {
	    setenv ("bootcmd", mmc_cmdbuffer);
	}
        return(load_result);
}


U_BOOT_CMD(
	copy_file_mmc,	3,	0,	copy_file_mmc,
	"- copy file from mmc card\n",
	"filename address\n"
	"    - load binary file from the MMC/SD card to a memory address\n"
);


U_BOOT_CMD(
	mmc_read_cmd_file,	1,	0,	mmc_read_cmd_file,
	"- copy command file from mmc card\n",
	NULL
);

/* ------------------------------- End of file ---------------------------- */

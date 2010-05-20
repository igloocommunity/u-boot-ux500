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
#ifndef __MMC_UTILS_H
#define __MMC_UTILS_H

#include <common.h>

typedef struct
{
    u8 jump                [3];
    u8 OEMname             [8];
    u8 BytesPerSector      [2];
    u8 SectorsPerCluster   [1];
    u8 ReservedSectors     [2];
    u8 NumOfFAT            [1];
    u8 NumOfRootEntries    [2];
    u8 NumOfSectors        [2];
    u8 MediaDesc           [1];
    u8 SectorsPerFat       [2];
    u8 SectorsPerTrack     [2];
    u8 NumOfHeads          [2];
    u8 NumOfHiddenSectors  [4];
    u8 NumOfTotSectors     [4];
    u8 DriveNumber         [1];
    u8 reserved            [1];
    u8 ExtBootSig          [1];
    u8 VolumeID            [4];
    u8 VolumeLabel         [11];
    u8 FSType              [8];
    u8 LoadProg            [448];
    u8 Signature           [2];
}   t_mmc_boot_record;

typedef struct
{
    u8 filename            [8];
    u8 extension           [3];
    u8 attrib                  ;
    u8 reserved            [10];
    u8 hour                [2];
    u8 date                [2];
    u8 startCluster        [2];
    u8 dimension           [4];
}   t_mmc_root_entry;

typedef struct s_root_elem
{
    char                        name[8];
    char                        ext[3];
    char                        nulll[0xf];
    short                       offset;
    unsigned                    sizee;
}   s_root_elem;

typedef struct s_root
{
    s_root_elem                 elem[16];
}   s_root;

typedef struct s_file_elem
{
    char filename               [8] ;
}   s_file_elem;

typedef struct s_file_list
{
    s_file_elem elem            [30];
}   s_file_list                     ;

typedef enum
{
    MMCPOLLING,
    MMCDMA
}t_mmc_transfer_mode;


t_mmc_error mmc_readblock    (u8 cardno, u32 addr, u32* readbuff, u16 blocksize, t_mmc_transfer_mode transfer_mode);
t_mmc_error mmc_writeblock   (u8 cardno, u32 addr, u32* writebuff, u16 blocksize, t_mmc_transfer_mode transfer_mode);
t_mmc_error mmc_enable       (void);
t_mmc_error mmc_disable      (void);
t_mmc_error mmc_initCard     (void);
t_mmc_error mmc_readcsd      (u32 *response);
t_mmc_error mmc_readcid      (u32 *response) ;
t_mmc_error display_file_list(char *extension);

#endif /* !defined(__MMC_UTILS_H) */



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
#ifndef _MMC_P_H_
#define _MMC_P_H_

#define BIT31 0x80000000
#define PROCESSOR_CLK 200000000
#define MAXBSIZEPOWER 11

#define MCLK 26000000

#define MMC_PERIPHERAL_ID0	0x81
#define MMC_PERIPHERAL_ID1	0x11
#define MMC_PERIPHERAL_ID2	0x04
#define MMC_PERIPHERAL_ID3	0x00


#define MMC_PCELL_ID0	0x0D
#define MMC_PCELL_ID1	0xF0
#define MMC_PCELL_ID2	0x05
#define MMC_PCELL_ID3	0xB1


typedef volatile struct {
    u32 mmc_Power; 		 	// 0x00	
    u32 mmc_Clock;				// 0x04	
    u32 mmc_Argument;		    // 0x08	
    u32 mmc_Command;	    	// 0x0c	
    u32 mmc_RespCommand;		// 0x10	
    u32 mmc_Response0;			// 0x14	
    u32 mmc_Response1;			// 0x18	
    u32 mmc_Response2;			// 0x1c	
    u32 mmc_Response3;			// 0x20	
    u32 mmc_DataTimer;			// 0x24	
    u32 mmc_DataLength;		// 0x28	
    u32 mmc_DataCtrl;			// 0x2c	
    u32 mmc_DataCnt;			// 0x30	
    u32 mmc_Status;			// 0x34	
    u32 mmc_Clear;				// 0x38	
    u32 mmc_Mask0;				// 0x3c		
    u32 mmc_Mask1;				// 0x40	
    u32 mmc_SelectSD;			// 0x44	
    u32 mmc_FifoCnt;			// 0x48
    u32 mmc_unused1[(0x80-0x4C)>>2]; 
    u32 mmc_Fifo;				// 0x80	
    u32 mmc_unused2[(0xFE0-0x84)>>2]; 
    u32 mmc_PeriphId0;			// 0xFE0	mmc Peripheral Identi.cation Register
    u32 mmc_PeriphId1;			// 0xFE4
    u32 mmc_PeriphId2;			// 0xFE8
    u32 mmc_PeriphId3;			// 0xFEC

    u32 mmc_PCellId0;			// 0xFF0	mmc PCell Identi.cation Register
    u32 mmc_PCellId1;			// 0xFF4
    u32 mmc_PCellId2;			// 0xFF8
    u32 mmc_PCellId3;			// 0xFFc
}t_mmc_register;


// Elementary layer 
#define BIT_MASK(__bws) ((u32)(((wb ## __bws)==32)?0xFFFFFFFF:\
            ((1U << (wb ## __bws)) - 1)) << (sb ## __bws))

#define MMC_WRITE_BITS(reg,val,mask,sb)  ((reg) =   (((reg) & ~(mask)) | (((val)<<(sb)) & (mask))))
#define MMC_READ_BITS(reg,mask,sb)                      ((reg & mask)>>sb)

#define wbMMC_Power_CTRL		2
#define sbMMC_Power_CTRL		0
#define MMC_Power_MASK_CTRL		BIT_MASK(MMC_Power_CTRL)

#define wbMMC_Power_VOLT		4
#define sbMMC_Power_VOLT		2
#define MMC_Power_MASK_VOLT		BIT_MASK(MMC_Power_VOLT)

#define wbMMC_Power_OPEND		1
#define sbMMC_Power_OPEND		6
#define MMC_Power_MASK_OPEND	BIT_MASK(MMC_Power_OPEND)

#define wbMMC_Power_ROD			1
#define sbMMC_Power_ROD			7
#define MMC_Power_MASK_ROD	BIT_MASK(MMC_Power_ROD)

#define wbMMC_Clock_CLKDIV		8
#define sbMMC_Clock_CLKDIV		0
#define MMC_Clock_MASK_CLKDIV	BIT_MASK(MMC_Clock_CLKDIV)

#define wbMMC_Clock_ENABLE		1
#define sbMMC_Clock_ENABLE		8
#define MMC_Clock_MASK_ENABLE	BIT_MASK(MMC_Clock_ENABLE)

#define wbMMC_Clock_PWRSAVE		1
#define sbMMC_Clock_PWRSAVE		9
#define MMC_Clock_MASK_PWRSAVE	BIT_MASK(MMC_Clock_PWRSAVE)

#define wbMMC_Clock_BYPASS		1
#define sbMMC_Clock_BYPASS		10
#define MMC_Clock_MASK_BYPASS	BIT_MASK(MMC_Clock_BYPASS)

#define wbMMC_Clock_WIDEBUS		1
#define sbMMC_Clock_WIDEBUS		11
#define MMC_Clock_MASK_WIDEBUS	BIT_MASK(MMC_Clock_WIDEBUS)
#define MMC_SET_WIDEBUS(reg,a)  MMC_WRITE_BITS(reg,a,MMC_Clock_MASK_WIDEBUS,sbMMC_Clock_WIDEBUS)

#define wbMMC_DataPath_ENABLE	1
#define sbMMC_DataPath_ENABLE	0
#define MMC_DataPath_MASK_ENABLE	BIT_MASK(MMC_DataPath_ENABLE)

#define wbMMC_DataPath_BLOCKSIZE	4
#define sbMMC_DataPath_BLOCKSIZE	4
#define MMC_DataPath_MASK_BLOCKSIZE	BIT_MASK(MMC_DataPath_BLOCKSIZE)

#define wbMMC_DataPath_DIRECTION	1
#define sbMMC_DataPath_DIRECTION	1
#define MMC_DataPath_MASK_DIRECTION	BIT_MASK(MMC_DataPath_DIRECTION)

#define wbMMC_DataPath_MODE			1
#define sbMMC_DataPath_MODE			2
#define MMC_DataPath_MASK_MODE	BIT_MASK(MMC_DataPath_MODE)

#define wbMMC_DataPath_DMA			1
#define sbMMC_DataPath_DMA			3
#define MMC_DataPath_MASK_DMA	BIT_MASK(MMC_DataPath_DMA)

#define MMC_SET_CTRL(reg,a)   	MMC_WRITE_BITS(reg,a,MMC_Power_MASK_CTRL,sbMMC_Power_CTRL)
#define MMC_SET_VOLT(reg,a)   	MMC_WRITE_BITS(reg,a,MMC_Power_MASK_VOLT,sbMMC_Power_VOLT)
#define MMC_SET_OPEND(reg,a)   	MMC_WRITE_BITS(reg,a,MMC_Power_MASK_OPEND,sbMMC_Power_OPEND)
#define MMC_SET_ROD(reg,a)    	MMC_WRITE_BITS(reg,a,MMC_Power_MASK_ROD,sbMMC_Power_ROD)
#define MMC_SET_CENABLE(reg,a)  MMC_WRITE_BITS(reg,a,MMC_Clock_MASK_ENABLE,sbMMC_Clock_ENABLE)
#define MMC_SET_PWRSAVE(reg,a)  MMC_WRITE_BITS(reg,a,MMC_Clock_MASK_PWRSAVE,sbMMC_Clock_PWRSAVE)
#define MMC_SET_BYPASS(reg,a)   MMC_WRITE_BITS(reg,a,MMC_Clock_MASK_BYPASS,sbMMC_Clock_BYPASS)

#define MMC_SET_CLKDIV(reg,a)   MMC_WRITE_BITS(reg,a,MMC_Clock_MASK_CLKDIV,sbMMC_Clock_CLKDIV)
#define MMC_SET_DATAPATH(reg,a) MMC_WRITE_BITS(reg,a,MMC_DataPath_MASK_ENABLE,sbMMC_DataPath_ENABLE)
#define MMC_SET_BLOCKSIZE(reg,a) MMC_WRITE_BITS(reg,a,MMC_DataPath_MASK_BLOCKSIZE,sbMMC_DataPath_BLOCKSIZE)
#define MMC_SET_DATADIR(reg,a)  MMC_WRITE_BITS(reg,a,MMC_DataPath_MASK_DIRECTION,sbMMC_DataPath_DIRECTION)
#define MMC_SET_MODE(reg,a)     MMC_WRITE_BITS(reg,a,MMC_DataPath_MASK_MODE,sbMMC_DataPath_MODE)
#define MMC_SET_DMA(reg,a)     MMC_WRITE_BITS(reg,a,MMC_DataPath_MASK_DMA,sbMMC_DataPath_DMA)



// Functional layer 
#define R1_OUT_OF_RANGE   		0x80000000
#define R1_ADDRESS_ERROR 		0x40000000
#define R1_BLOCK_LEN_ERROR  	0x20000000
#define R1_ERASE_SEQ_ERROR 		0x10000000
#define R1_ERASE_PARAM  		0x08000000
#define R1_WP_VIOLATION 		0x04000000 
#define R1_CARD_IS_LOCKED 		0x02000000
#define R1_LOCK_UNLOCK_FAILED 	0x01000000
#define R1_COM_CRC_ERROR 		0x00800000
#define R1_ILLEGAL_COMMAND 		0x00400000
#define R1_CARD_ECC_FAILED 		0x00200000
#define R1_CC_ERROR  			0x00100000
#define R1_UNKNOWN_ERROR  		0x00080000
#define R1_STREAM_READ_UNDERRUN  	0x00040000
#define R1_STREAM_WRITE_OVERRUN		0x00020000
#define R1_CID_CSD_OVERWRITE 		0x00010000
#define R1_WP_ERASE_SKIP 		0x00008000
#define R1_CARD_ECC_DISABLED 		0x00004000
#define R1_ERASE_RESET  		0x00002000
#define R1_AKE_SEQ_ERROR            	0x00000008
#define R1_CURRENT_STATE(x)     ((x & 0x00001E00) >> 9) /* sx, b (4 bits) */
#define R1_READY_FOR_DATA 		0x000000100
#define R1_APP_CMD  			0x000000020

/* DEFINES FOR R6 RESPONSE */
#define R6_GEN_UNKNOWN_ERROR  		0x00002000
#define R6_ILLEGAL_CMD            	0x00004000
#define R6_COM_CRC_FAILED         	0x00008000

#define AllOne          (0xffffffff)
#define AllZero         (0x00000000)


//Power Control register

#define PowerOn          (0x00000003)
#define OpenDrain        (0x00000040)
#define CmdDatEn         (0x0000003C)
#define VoltageWindowMMC (0x80FFC000)
#define VoltageWindowSD  (0x80010000)
#define StuffBits0To32   (0xFFFFFFFF)
#define StuffBits0To15   (0xFFFF)
#define Sector_Mode      (0x40000000)
#define Byte_Mode        (0x00000000)
#define Check_Pattern    (0x000001AA)

//Clock Control register

#define ClkDivInit       (0x00000076) 
#define ClkDivTrans      (0x00000001)
#define ClkEnable        (0x00000100)
#define PwrSave          (0x00000200)  
#define Enab_Bypass      (0x00000400)
#define Widebus_4      	 (0x00000800)
#define Widebus_8      	 (0x00001000)
#define Hwfc_en      	 (0x00004000)

//Command register

//all commands

#define GO_IDLE_STATE               (0)
#define SEND_OP_COND                (1)
#define ALL_SEND_CID                (2)
#define SET_REL_ADDR                (3)      
#define SET_DSR                     (4)      
#define SEL_DESEL_CARD              (7)
#define SEND_EXT_CSD                (8) 
#define SEND_CSD                    (9)
#define SEND_CID                    (10)
#define READ_DAT_UNTIL_STOP         (11)
#define STOP_TRANSMISSION           (12)  
#define SEND_STATUS                 (13) 
#define GO_INACTIVE_STATE           (15)
#define SET_BLOCKLEN                (16)
#define READ_SINGLE_BLOCK           (17)
#define READ_MULT_BLOCK             (18)
#define WRITE_DAT_UNTIL_STOP        (20)
#define SET_BLOCK_COUNT             (23)
#define WRITE_SINGLE_BLOCK          (24)   
#define WRITE_MULT_BLOCK            (25)
#define PROG_CID                    (26) 
#define PROG_CSD                    (27)
#define SET_WRITE_PROT              (28)
#define CLR_WRITE_PROT              (29)  
#define SEND_WRITE_PROT             (30)
#define ERASE_GRP_START             (35)  
#define ERASE_GRP_END               (36)
#define ERASE                       (38)
#define FAST_IO                     (39) 
#define GO_IRQ_STATE                (40) 
#define LOCK_UNLOCK                 (42) 
#define APP_CMD                     (55)
#define GEN_CMD                     (56)

/*Following commands are SD Card Specific commands. MMC_APP_CMD should be sent before sending these commands.*/
#define APP_SD_SET_BUSWIDTH          (6)
#define SD_APP_STAUS                 (13)
#define SD_APP_SEND_NUM_WRITE_BLOCKS (22)
#define SD_APP_OP_COND               (41)
#define SD_APP_SET_CLR_CARD_DETECT   (42)
#define SD_APP_SEND_SCR              (51)
#define SD_SEND_IF_COND              (8)

//Command control register

#define RespExpected        (0x00000040)
#define LongResponse        (0x00000080)        
#define EnabIntrReq         (0x00000100) 
#define EnabCmdPend         (0x00000200)
#define CmdPathEnable       (0x00000400) 


//Data Control register

#define DataPathEnable      (0x00000001)
#define ReadDir             (0x00000002) 
#define StreamMode          (0x00000004) 
#define DMAEnab             (0x00000008)
#define BlockSize           (0x000000F0) 

//Status register

#define CmdCrcFail          (0x00000001)
#define DataCrcFail         (0x00000002)
#define CmdTimeOut          (0x00000004)
#define DataTimeOut         (0x00000008)
#define TxUnderrun          (0x00000010)
#define RxOverrun           (0x00000020)
#define CmdRespEnd          (0x00000040)
#define CmdSent          	(0x00000080)
#define DataEnd          	(0x00000100)
#define StartBitError	    (0x00000200)
#define DataBlockEnd        (0x00000400)
#define CmdActive           (0x00000800)
#define TxActive            (0x00001000)
#define RxActive          	(0x00002000)
#define TxFifoHalfEmpty     (0x00004000)
#define RxFifoHalfFull      (0x00008000)
#define TxFifoFull          (0x00010000)
#define RxFifoFull          (0x00020000)
#define TxFifoEmpty         (0x00040000)
#define RxFifoEmpty         (0x00080000)
#define TxDataAvlbl         (0x00100000)
#define RxDataAvlbl         (0x00200000)
#define AllInterrupts		(0x003FFDFF)
#define ErrorBits           (0xFDFFE008)

#define ClrStaticFlags		(0x00C007FF)

/* COMMAND CLASS SUPPORTED */
#define CCCC_LOCK_UNLOCK    (0x80)
#define CCCC_WRITE_PROT     (0x40)
#define CCCC_ERASE          (0x20)

#endif /*MMCP*/

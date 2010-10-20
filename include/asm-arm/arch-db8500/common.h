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
#ifndef _DB8500_COMMON_H_
#define _DB8500_COMMON_H_
#include <common.h>

#define PASS (1)
#define FAIL (0)
    
#define IO(addr)  (*((u32*) (addr)))
#define HIO(addr) (*((u16*) (addr)))
#define BIO(addr) (*((u8*)  (addr)))
    
/* 
 * macro to get at IO space 
 */
#define IO_ADDRESS(x) (x)

#define REG_WRITE_BITS(reg,val,mask,sb)  (writel(((readl(reg) & ~(mask)) | (((val)<<(sb)) & (mask))), reg))

#define nmdk_error(format, arg...) printf(": " format "\n" , ## arg)

#if !defined(FALSE) &&  !defined(TRUE)   
typedef enum {FALSE, TRUE} t_bool;
#else /* FALSE & TRUE already defined */
typedef enum {BOOL_FALSE, BOOL_TRUE} t_bool;
#endif /* !defined(FALSE) &&  !defined(TRUE) */

/*-----------------------------------------------------------------------------
 * Bit mask definition
 *---------------------------------------------------------------------------*/
#define MASK_NULL8    0x00
#define MASK_NULL16   0x0000
#define MASK_NULL32   0x00000000
#define MASK_ALL8     0xFF 
#define MASK_ALL16    0xFFFF 
#define MASK_ALL32    0xFFFFFFFF

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

#define NOMADIK_INTERNAL_ERROR                  (-8)
#define NOMADIK_NOT_CONFIGURED                  (-7)
#define NOMADIK_REQUEST_PENDING                 (-6)
#define NOMADIK_REQUEST_NOT_APPLICABLE          (-5)
#define NOMADIK_INVALID_PARAMETER               (-4)
#define NOMADIK_UNSUPPORTED_FEATURE             (-3)
#define NOMADIK_UNSUPPORTED_HW                  (-2)
#define NOMADIK_ERROR                           (-1)
#define NOMADIK_OK                              ( 0)
#define NOMADIK_INTERNAL_EVENT                  ( 1)
#define NOMADIK_REMAINING_PENDING_EVENTS        ( 2)
#define NOMADIK_REMAINING_FILTER_PENDING_EVENTS ( 3)
#define NOMADIK_NO_MORE_PENDING_EVENT           ( 4)
#define NOMADIK_NO_MORE_FILTER_PENDING_EVENT    ( 5)
#define NOMADIK_NO_PENDING_EVENT_ERROR          ( 7)


#define NOMADIK_MAX_ERROR_VALUE                 (-65) /* HW specific error codes
                                                  * should start from this offset
                                                  */
/*-----------------------------------------------------------------------------
 * Bit setting or clearing
 *---------------------------------------------------------------------------*/
#define NOMADIK_SET_BITS(reg,mask)			((reg) |=  (mask))
#define NOMADIK_CLEAR_BITS(reg,mask)		((reg) &= ~(mask))
#define NOMADIK_READ_BITS(reg,mask)			((reg) &   (mask))
#define NOMADIK_WRITE_BITS(reg,val,mask)	((reg) =   (((reg) & ~(mask)) | ((val) & (mask))))
#define NOMADIK_READ_REG(reg)				(reg)
#define NOMADIK_WRITE_REG(reg,val)			((reg) = (val))

/*
 * Definition of the different kind of addresses manipulated into a system with MMU
 * (handle physical AND logical addresses)
 */

typedef u32 t_physical_address; 
typedef u32 t_logical_address;

/*function prototypes*/
int board_early_access(block_dev_desc_t *block_dev);
#endif  /* _DB8500_COMMON_H_ */

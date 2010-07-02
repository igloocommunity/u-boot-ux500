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

#ifndef __CONFIG_H
#define __CONFIG_H

/*-----------------------------------------------------------------------
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_U8500		1
#define CONFIG_U8500_ED		1
#define CONFIG_L2_OFF		1

#define CONFIG_SYS_MEMTEST_START	0x00000000
#define CONFIG_SYS_MEMTEST_END	0x1FFFFFFF
#define CONFIG_SYS_HZ		1000		/* must be 1000 */

#ifndef CONFIG_U8500_V1
#define CONFIG_SYS_TIMERBASE	0xA03DA000      /* MTU0 timer */
#else
#define CONFIG_SYS_TIMERBASE	0xA03C6000      /* MTU0 timer */
#endif

#define BOARD_LATE_INIT		1

/*-----------------------------------------------------------------------
 * Size of malloc() pool
 */
#ifdef CONFIG_BOOT_SRAM
#define CONFIG_ENV_SIZE		32*1024
#define CONFIG_SYS_MALLOC_LEN	(CONFIG_ENV_SIZE + 64*1024)
#else
#define CONFIG_ENV_SIZE		128*1024
#define CONFIG_SYS_MALLOC_LEN	(CONFIG_ENV_SIZE + 256*1024)
#endif
#define CONFIG_SYS_GBL_DATA_SIZE	128	/* for initial data */

/*-----------------------------------------------------------------------
 * PL011 Configuration
 */

#define CONFIG_PL011_SERIAL
/*
 * U8500 UART registers base for 3 serial devices
 */
#define CFG_UART0_BASE		0x80120000
#define CFG_UART1_BASE		0x80121000
#define CFG_UART2_BASE		0x80007000
#define CFG_SERIAL0		CFG_UART0_BASE
#define CFG_SERIAL1		CFG_UART1_BASE
#define CFG_SERIAL2		CFG_UART2_BASE
#define CONFIG_PL011_CLOCK	38400000
#define CONFIG_PL01x_PORTS	{ (void *)CFG_SERIAL0, (void *)CFG_SERIAL1, \
				  (void *)CFG_SERIAL2 }
#define CONFIG_CONS_INDEX	2
#define CONFIG_BAUDRATE		115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Devices and file systems
 */
#define CONFIG_MMC		1
#define CONFIG_GENERIC_MMC	1
#define CONFIG_DOS_PARTITION	1

/*
 * Commands
 */
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_BOOTD
#define CONFIG_CMD_BDI
#define CONFIG_CMD_IMI
#define CONFIG_CMD_MISC
#define CONFIG_CMD_RUN
#define CONFIG_CMD_ECHO
#define CONFIG_CMD_CONSOLE
#define CONFIG_CMD_LOADS
#define CONFIG_CMD_LOADB
#define CONFIG_CMD_MMC
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EMMC
#define CONFIG_CMD_SOURCE
#define CONFIG_CMD_I2C

#ifdef CONFIG_USB_TTY
#define CONFIG_BOOTDELAY	-1	/* disable autoboot */
#endif /* CONFIG_USB_TTY */

#ifndef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	1
#endif
#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */

#undef CONFIG_BOOTARGS
#define CONFIG_BOOTCOMMAND	"run emmcboot"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"verify=n\0"							\
	"loadaddr=0x00100000\0"						\
	"console=ttyAMA2,115200n8\0"					\
	"memargs256=mem=96M@0 mem_modem=32M@96M mem=30M@128M "		\
		"pmem=22M@158M pmem_hwb=44M@180M mem_mali=32@224M\0"	\
	"memargs512=mem=96M@0 mem_modem=32M@96M mem=44M@128M "		\
		"pmem=22M@172M mem=30M@194M mem_mali=32M@224M "		\
		"pmem_hwb=54M@256M mem=202M@310M\0"			\
	"commonargs=setenv bootargs cachepolicy=writealloc noinitrd "	\
		"init=init "						\
		"board_id=${board_id} "					\
		"logo.${logo} "						\
		"startup_graphics=${startup_graphics}\0"		\
	"emmcargs=setenv bootargs ${bootargs} "				\
		"root=/dev/mmcblk0p2 "					\
		"rootdelay=1\0"						\
	"addcons=setenv bootargs ${bootargs} "				\
		"console=${console}\0"					\
	"emmcboot=echo Booting from eMMC ...; "				\
		"run commonargs emmcargs addcons memargs;"		\
		"write_partition_block;"				\
		"mmc read 0 ${loadaddr} 0xA0000 0x4000;"		\
		"bootm ${loadaddr}\0"					\
	"cmdfile=mmc init 1;mmc_read_cmd_file;run bootcmd\0"		\
	"flash=mmc init 1;fatload mmc 1 ${loadaddr} flash.scr;"		\
		"source ${loadaddr}\0"					\
	"loaduimage=mmc init 1;fatload mmc 1 ${loadaddr} uImage\0"	\
	"usbtty=cdc_acm\0"						\
	"stdout=serial,usbtty\0"					\
	"stdin=serial,usbtty\0"						\
	"stderr=serial,usbtty\0"

/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */

#define CONFIG_SYS_LONGHELP			/* undef to save memory     */
#define CONFIG_SYS_PROMPT	"U8500 $ "	/* Monitor Command Prompt   */
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O Buffer Size  */

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE \
					+ sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	32	/* max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE /* Boot Arg Buffer Size */

#undef	CONFIG_SYS_CLKS_IN_HZ		/* everything, incl board info, in Hz */
#define CONFIG_SYS_LOAD_ADDR		0x00100000 /* default load address */
#define CONFIG_SYS_LOADS_BAUD_CHANGE	1

#define CONFIG_SYS_HUSH_PARSER		1
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_CMDLINE_EDITING

#define CONFIG_SETUP_MEMORY_TAGS	2
#define CONFIG_INITRD_TAG		1
#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs  */

/*
 * I2C
 */
#define	CONFIG_HARD_I2C			/* I2C with hardware support */
#undef	CONFIG_SOFT_I2C			/* I2C bit-banged */
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		0	/* slave addr of controller */
#define CONFIG_SYS_I2C0_BASE		0x80004000
#define CONFIG_SYS_I2C1_BASE		0x80122000
#define CONFIG_SYS_I2C2_BASE		0x80128000
#define CONFIG_SYS_I2C3_BASE		0x80110000
#define CONFIG_SYS_I2C_BUS_MAX		4

#define CONFIG_SYS_I2C_GPIOE_ADDR	0x42	/* GPIO expander chip addr */
#define CONFIG_TC35892_GPIO
/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */

#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ		(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ		(4*1024)	/* FIQ stack */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM_1			0x00000000	/* DDR-SDRAM Bank #1 */
#define PHYS_SDRAM_SIZE_1		0x10000000	/* 256 MB */

/*-----------------------------------------------------------------------
 * MMC related configs
 */
#define MMC_BLOCK_SIZE			512
#define CFG_MMC_BASE			0x80126000	/* MMC base for 8500  */
#define CONFIG_MMC_DEV_NUM		1

/*-----------------------------------------------------------------------
 * EMMC related configs
 */
#define CFG_EMMC_BASE_V1		0x80005000	/*POP EMMC base of size 256MB for 8500 cut1.0 */
#define CFG_EMMC_BASE_ED		0x80114000	/* EMMC base of size 2GB for 8500  */

#define CONFIG_CMD_ENV
#define CONFIG_CMD_SAVEENV	/* CMD_ENV is obsolete but used in env_emmc.c */
#define CONFIG_ENV_IS_IN_EMMC		1
#define CONFIG_ENV_OFFSET_START		0x13F80000
#define CONFIG_ENV_OFFSET_END		0x13FE0000
#define CONFIG_EMMC_DEV_NUM		0

/*-----------------------------------------------------------------------
 * USB related configs
 */
#define CONFIG_USB_BASE 		0xA03E0000
#define UDC_BASE	 		0xA03E0000

#define CONFIG_USB_DEVICE		1
#define CONFIG_MUSB			1 /* Enable USB driver */
#ifdef CONFIG_USB_TTY
/* Allow console in serial and USB at the same time */
#define CONFIG_CONSOLE_MUX		1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1
#define CONFIG_SYS_CONSOLE_ENV_OVERWRITE
#endif
/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

#define CONFIG_SYS_MAX_FLASH_SECT 	512
#define CONFIG_SYS_MAX_FLASH_BANKS	1	/* max number of memory banks */

/*-----------------------------------------------------------------------
 * Video Logo Related configs
 */
#define CONFIG_VIDEO_LOGO	/* Enable startup logo */
/*
#define CONFIG_SYS_VIDEO_USE_GIMP_HEADER
#define CONFIG_SYS_VIDEO_FB_ADRS 	0x14000000
*/
#define CONFIG_SYS_MCDE_REFRESH_TIME	50

#define CONFIG_SYS_DISPLAY_NATIVE_X_RES	864
#define CONFIG_SYS_DISPLAY_NATIVE_Y_RES	480
/* 2.5V */
#define CONFIG_SYS_DISPLAY_VOLTAGE	2500000

/*------------------------------------------------------------------------------
 * base register values for U8500
 */
#define CFG_PRCMU_BASE		0x80157000	/* Power, reset and clock Management Unit */
#define CFG_SDRAMC_BASE		0x903CF000	/* SDRAMC cnf registers */
#define CFG_FSMC_BASE		0x80000000	/* FSMC Controller */

/*
 * U8500 GPIO register base for 9 banks
 */
#define CFG_GPIO_0_BASE			0x8012E000
#define CFG_GPIO_1_BASE			0x8012E080
#define CFG_GPIO_2_BASE			0x8000E000
#define CFG_GPIO_3_BASE			0x8000E080
#define CFG_GPIO_4_BASE			0x8000E100
#define CFG_GPIO_5_BASE			0x8000E180
#define CFG_GPIO_6_BASE			0x8011E000
#define CFG_GPIO_7_BASE			0x8011E080
#define CFG_GPIO_8_BASE			0xA03FE000

/*
 * U8500 RTC register base
 */
#define CFG_RTC_BASE			0x80154000	/* Real time clock */

/*
 * U8500 Display register base
 */
#define CFG_MCDE_BASE 			0xA0350000
#define CFG_DSI_BASE  			0xA0351000

#endif	/* __CONFIG_H */

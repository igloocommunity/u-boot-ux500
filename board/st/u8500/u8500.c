/*
 * Copyright (C) ST-Ericsson SA 2009
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <config.h>
#include <common.h>
#include <i2c.h>
#include <asm/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/ab8500.h>
#include <asm/arch/prcmu.h>
#include <tc35892.h>
#include <asm/arch/gpio.h>
#include "itp.h"

#include <asm/arch/common.h>
#ifdef CONFIG_VIDEO_LOGO
#include "mcde_display.h"
#endif
#define NOMADIK_PER4_BASE	(0x80150000)
#define NOMADIK_BACKUPRAM0_BASE (NOMADIK_PER4_BASE + 0x00000)
#define NOMADIK_BACKUPRAM1_BASE (NOMADIK_PER4_BASE + 0x01000)

/*
 * Memory controller register
 */
#define DMC_BASE_ADDR			0x80156000
#define DMC_CTL_97			(DMC_BASE_ADDR + 0x184)

int board_id;	/* set in board_late_init() */
int errno;

#ifdef CONFIG_VIDEO_LOGO
static int mcde_error;
#endif

/*
 * Flag to indicate from where to where we have to copy the initialised data.
 * In case we were loaded, its value is -1 and .data must be saved for an
 * eventual restart. It is 1 if .data was restored, i.e. we were restarted,
 * e.g. by kexec.
 */
static volatile int data_init_flag = -1; /* -1 to get it into .data section */

static void config_gpio(void);

/* Get hold of gd pointer */
DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
    printf("Boot reached stage %d\n", progress);
}
#endif

/*
 * Miscellaneous platform dependent initialisations
 */

int board_init(void)
{
	extern char _idata_start[];
	extern char _data_start[];
	extern char _data_end[];
	unsigned long data_len;

	data_len = _data_end - _data_start;
	if (++data_init_flag == 0) {
		/*
		 * first init after reset/loading
		 * save .data section for restart
		 */
		memcpy(_idata_start, _data_start, data_len);
	} else {
		/*
		 * restart, e.g. by kexec
		 * copy back .data section
		 */
		memcpy(_data_start, _idata_start, data_len);
		/* memcpy set data_init_flag back to zero */
		++data_init_flag;
	}

	/*
	 * Setup board (bd) and board-info (bi).
	 * bi_arch_number: Unique id for this board. It will passed in r1 to
	 *    Linux startup code and is the machine_id.
	 * bi_boot_params: Where this board expects params.
	 */
	gd->bd->bi_arch_number = MACH_TYPE_U8500;
	gd->bd->bi_boot_params = 0x00000100;

	gpio_init();
	config_gpio();

	return 0;
}

int dram_init(void)
{
	uint32_t unused_cols_rows;
	unsigned int nrows;
	unsigned int ncols;

	gd->bd->bi_dram[0].start = 0;
	if (u8500_is_earlydrop()) {
		gd->bd->bi_dram[0].size = 0x10000000;	/* 256 MiB */
		return 0;
	}

	/*
	 * Assumption: 2 CS active, both CS have same layout.
	 *             15 rows max, 11 cols max (controller spec).
	 *             memory chip has 8 banks, I/O width 32 bit.
	 * The correct way would be to read MR#8: I/O width and density,
	 * but this requires locking against the PRCMU firmware.
	 * Simplified approach:
	 * Read number of unused rows and columns from mem controller.
	 * size = nCS x 2^(rows+cols) x nbanks x buswidth_bytes
	 */
	unused_cols_rows = readl(DMC_CTL_97);
	nrows = 15 - (unused_cols_rows & 0x07);
	ncols = 11 - ((unused_cols_rows & 0x0700) >> 8);
	gd->bd->bi_dram[0].size = 2 * (1 << (nrows + ncols)) * 8 * 4;

	return 0;
}

#ifdef CONFIG_VIDEO_LOGO
static int dss_init(void)
{
	puts("\nMCDE:  ");

	boottime_tag("splash");

	if (!cpu_is_u8500v11() && !cpu_is_u8500v2()) {
		printf("Only HREF+ or V2 is supported\n");
		goto mcde_error;
	}
	if (mcde_startup()) {
		printf("startup failed\n");
		goto mcde_error;
	}
	if (mcde_display_image()) {
		printf("display_image failed\n");
		goto mcde_error;
	}

	printf("ready\n");
	return 0;

mcde_error:
	return -EINVAL;

}
#endif

/*
 * board_early_access - for functionality that needs to run before
 * board_late_init but after board_init and emmc init.
 */
int board_early_access(block_dev_desc_t *block_dev)
{

	/*
	 * Don't load itp, modem and splash if restarted (eg crashdump).
	 */
	if (!(data_init_flag > 0)) {
#ifdef CONFIG_ITP_LOAD
		itp_read_config(block_dev);
#endif

#ifdef CONFIG_VIDEO_LOGO
		/* only load splash if not itp */
#ifdef CONFIG_ITP_LOAD
		if (!itp_is_itp_in_config())
			mcde_error = dss_init();
#else
		mcde_error = dss_init();
#endif
#endif

#ifdef CONFIG_ITP_LOAD
		if (itp_load_itp_and_modem(block_dev))
			return 1;
#endif
	}
	return 0;
}

#ifdef BOARD_LATE_INIT
#ifdef CONFIG_MMC

#define LDO_VAUX3_ENABLE_MASK		0x3
#define LDO_VAUX3_ENABLE_VAL		0x1
#define LDO_VAUX3_SEL_MASK		0xf
#define LDO_VAUX3_SEL_2V9		0xd
#define LDO_VAUX3_V2_SEL_MASK		0x7
#define LDO_VAUX3_V2_SEL_2V91		0x7

static int hrefplus_mmc_power_init(void)
{
	int ret;
	int voltage_regval;
	int enable_regval;
	int ab8500_revision;

	if (!cpu_is_u8500v11() && !cpu_is_u8500v2())
		return 0;

	/* Get AB8500 revision */
	ret = ab8500_read(AB8500_MISC, AB8500_REV_REG);
	if (ret < 0)
		goto out;

	ab8500_revision = ret;

	/*
	 * On v1.1 HREF boards (HREF+) and v2 boards, Vaux3 needs to be
	 * enabled for the SD card to work.  This is done by enabling
	 * the regulators in the AB8500 via PRCMU I2C transactions.
	 *
	 * This code is derived from the handling of AB8500_LDO_VAUX3 in
	 * ab8500_ldo_enable() and ab8500_ldo_disable() in Linux.
	 *
	 * Turn off and delay is required to have it work across soft reboots.
	 */

	/* Turn off (read-modify-write) */
	ret = ab8500_read(AB8500_REGU_CTRL2, AB8500_REGU_VRF1VAUX3_REGU_REG);
	if (ret < 0)
		goto out;

	enable_regval = ret;

	ret = ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VRF1VAUX3_REGU_REG,
		enable_regval & ~LDO_VAUX3_ENABLE_MASK);
	if (ret < 0)
		goto out;

	/* Delay */
	udelay(10 * 1000);

	/* Set the voltage to 2.91 V or 2.9 V without overriding VRF1 value */
	ret = ab8500_read(AB8500_REGU_CTRL2, AB8500_REGU_VRF1VAUX3_SEL_REG);
	if (ret < 0)
		goto out;

	voltage_regval = ret;

	if (ab8500_revision < 0x20) {
		voltage_regval &= ~LDO_VAUX3_SEL_MASK;
		voltage_regval |= LDO_VAUX3_SEL_2V9;
	} else {
		voltage_regval &= ~LDO_VAUX3_V2_SEL_MASK;
		voltage_regval |= LDO_VAUX3_V2_SEL_2V91;
	}

	ret = ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VRF1VAUX3_SEL_REG,
		voltage_regval);
	if (ret < 0)
		goto out;

	/* Turn on the supply */
	enable_regval &= ~LDO_VAUX3_ENABLE_MASK;
	enable_regval |= LDO_VAUX3_ENABLE_VAL;

	ret = ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VRF1VAUX3_REGU_REG,
		enable_regval);

out:
	return ret;
}
#endif
/*
 * called after all initialisation were done, but before the generic
 * mmc_initialize().
 */
int board_late_init(void)
{
	uchar byte;
#ifdef CONFIG_MMC
	uchar byte_array[] = {0x06, 0x06};
#endif
	char strbuf[80];

	/*
	 * Determine and set board_id environment variable
	 * 0: mop500, 1: href500
	 * Above boards have different GPIO expander chips which we can
	 * distinguish by the chip id.
	 *
	 * The board_id environment variable is needed for the Linux bootargs.
	 */
	(void) i2c_set_bus_num(0);
	(void) i2c_read(CONFIG_SYS_I2C_GPIOE_ADDR, 0x80, 1, &byte, 1);
	if (byte == 0x01) {
		board_id = 0;
		setenv("board_id", "0");
	} else {
		board_id = 1;
		setenv("board_id", "1");
	}
#ifdef CONFIG_MMC
	hrefplus_mmc_power_init();

	/*
	 * config extended GPIO pins for level shifter and
	 * SDMMC_ENABLE
	 */
	if (board_id == 0) {
		/* MOP500 */
		byte = 0x0c;
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0x89, 1, &byte, 1);
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0x83, 1, &byte, 1);
	} else {
		/* HREF */
		/* set the direction of GPIO KPY9 and KPY10 */
		byte = 0x06;
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0xC8, 1, &byte, 1);
		/* must be a multibyte access */
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0xC4, 1,
						&byte_array[0], 2);
	}
#endif /* CONFIG_MMC */
#ifdef CONFIG_VIDEO_LOGO
	if (mcde_error) {
		setenv("startup_graphics", "0");
		setenv("logo", "0");
	} else {
		setenv("startup_graphics", "1");
		setenv("logo", "nologo");
	}
#endif
	/*
	 * Create a memargs variable which points uses either the memargs256 or
	 * memargs512 environment variable, depending on the memory size.
	 * memargs is used to build the bootargs, memargs256 and memargs512 are
	 * stored in the environment.
	 */
	if (gd->bd->bi_dram[0].size == 0x10000000) {
		setenv("memargs", "setenv bootargs ${bootargs} ${memargs256}");
		setenv("mem", "256M");
	} else {
		setenv("memargs", "setenv bootargs ${bootargs} ${memargs512}");
		setenv("mem", "512M");
	}

	/*
	 * Create crashkernel env dynamically since it depends on U-Boot start
	 * address. U-Boot itself is used for dumping.
	 * The 32K offset is hardcoded in the kexec-tools.
	 * Parsed by Linux setup.c:reserve_crashkernel() using
	 * lib/cmdline.c:memparse().
	 * crashkernel=ramsize-range:size[,...][@offset]
	 */
	sprintf(strbuf, "crashkernel=1M@0x%lx", _armboot_start - 0x8000);
	setenv("crashkernel", strbuf);

	/*
	 * Check for a crashdump, if data_init_flag > 0, i.e. we were
	 * restarted e.g. by kexec. Do not check for crashdump if we were just
	 * loaded from the x-loader.
	 */
	if (data_init_flag > 0)
		setenv("preboot", "checkcrash");

	return (0);
}
#endif /* BOARD_LATE_INIT */

static void config_gpio(void)
{
		{
			/* UART2: 29, 30 */
			struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_0_BASE);
			p_gpio_register -> gpio_dats |= 0x60000000;
			p_gpio_register -> gpio_pdis &= ~0x60000000;
		}
		gpio_altfuncenable(GPIO_ALT_UART_2, "UART2");

		{
			/* 197 - 207 */
			struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_6_BASE);
			p_gpio_register -> gpio_dats |= 0x0000ffe0;
			p_gpio_register -> gpio_pdis &= ~0x0000ffe0;
		}
		gpio_altfuncenable(GPIO_ALT_EMMC, "EMMC");

		{
			/* 18 - 28 */
			struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_0_BASE);
			// p_gpio_register -> gpio_dats |= 0x0ffc0000;
			p_gpio_register -> gpio_pdis &= ~0x0ffc0000;
		}
		gpio_altfuncenable(GPIO_ALT_SD_CARD0, "SDCARD");

		if (!u8500_is_earlydrop()) {
			{
				/* 128 - 138 */
				struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_4_BASE);
				p_gpio_register -> gpio_dats |= 0x000007ff;
				p_gpio_register -> gpio_pdis &= ~0x000007ff;
			}

			gpio_altfuncenable(GPIO_ALT_POP_EMMC, "EMMC");
		}
}


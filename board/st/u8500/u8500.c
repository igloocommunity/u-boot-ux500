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
#include <asm/arch/hardware.h>
#include <asm/arch/ab8500.h>
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

/* Power, Reset, Clock Management Unit */
/*
 * SVA: Smart Video Accelerator
 * SIA: Smart Imaging Accelerator
 * SGA: Smart Graphic accelerator
 * B2R2: Graphic blitter
 */
#define PRCMU_BASE	CFG_PRCMU_BASE	/* 0x80157000 for U8500 */
#define PRCM_ARMCLKFIX_MGT_REG		(PRCMU_BASE + 0x000)
#define PRCM_ACLK_MGT_REG		(PRCMU_BASE + 0x004)
#define PRCM_SVAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x008)
#define PRCM_SIAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x00C)
#define PRCM_SAAMMDSPCLK_MGT_REG	(PRCMU_BASE + 0x010)
#define PRCM_SGACLK_MGT_REG		(PRCMU_BASE + 0x014)
#define PRCM_UARTCLK_MGT_REG		(PRCMU_BASE + 0x018)
#define PRCM_MSPCLK_MGT_REG		(PRCMU_BASE + 0x01C)
#define PRCM_I2CCLK_MGT_REG		(PRCMU_BASE + 0x020)
#define PRCM_SDMMCCLK_MGT_REG		(PRCMU_BASE + 0x024)
#define PRCM_SLIMCLK_MGT_REG		(PRCMU_BASE + 0x028)
#define PRCM_PER1CLK_MGT_REG		(PRCMU_BASE + 0x02C)
#define PRCM_PER2CLK_MGT_REG		(PRCMU_BASE + 0x030)
#define PRCM_PER3CLK_MGT_REG		(PRCMU_BASE + 0x034)
#define PRCM_PER5CLK_MGT_REG		(PRCMU_BASE + 0x038)
#define PRCM_PER6CLK_MGT_REG		(PRCMU_BASE + 0x03C)
#define PRCM_PER7CLK_MGT_REG		(PRCMU_BASE + 0x040)
#define PRCM_DMACLK_MGT_REG		(PRCMU_BASE + 0x074)
#define PRCM_B2R2CLK_MGT_REG		(PRCMU_BASE + 0x078)

#define PRCM_PLLSOC0_FREQ_REG		(PRCMU_BASE + 0x080)
#define PRCM_PLLSOC1_FREQ_REG		(PRCMU_BASE + 0x084)
#define PRCM_PLLARM_FREQ_REG		(PRCMU_BASE + 0x088)
#define PRCM_PLLDDR_FREQ_REG		(PRCMU_BASE + 0x08C)
#define PRCM_ARM_CHGCLKREQ_REG		(PRCMU_BASE + 0x114)

#define PRCM_TCR			(PRCMU_BASE + 0x1C8)

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

/* PLLs for clock management registers */
enum {
	GATED = 0,
	PLLSOC0,	/* pllsw = 001, ffs() = 1 */
	PLLSOC1,	/* pllsw = 010, ffs() = 2 */
	PLLDDR,		/* pllsw = 100, ffs() = 3 */
	PLLARM,
};

static struct pll_freq_regs {
	int idx;	/* index fror pll_name and pll_khz arrays */
	uint32_t addr;
} pll_freq_regs[] = {
	{PLLSOC0, PRCM_PLLSOC0_FREQ_REG},
	{PLLSOC1, PRCM_PLLSOC1_FREQ_REG},
	{PLLDDR, PRCM_PLLDDR_FREQ_REG},
	{PLLARM, PRCM_PLLARM_FREQ_REG},
	{0, 0},
};

static const char *pll_name[5] = {"GATED", "SOC0", "SOC1", "DDR", "ARM"};
static uint32_t pll_khz[5];	/* use ffs(pllsw(reg)) as index for 0..3 */

static struct clk_mgt_regs {
	uint32_t addr;
	uint32_t val;
	const char *descr;
} clk_mgt_regs[] = {
	/* register content taken from bootrom settings */
	{PRCM_ARMCLKFIX_MGT_REG, 0x0120, "ARMCLKFIX"}, /* ena, SOC0/0, ??? */
	{PRCM_ACLK_MGT_REG, 0x0125, "ACLK"},	/* ena, SOC0/5, 160 MHz */
	{PRCM_SVAMMDSPCLK_MGT_REG, 0x1122, "SVA"}, /* ena, SOC0/2, 400 MHz */
	{PRCM_SIAMMDSPCLK_MGT_REG, 0x0022, "SIA"}, /* dis, SOC0/2, 400 MHz */
	{PRCM_SAAMMDSPCLK_MGT_REG, 0x0822, "SAA"}, /* dis, SOC0/4, 200 MHz */
	{PRCM_SGACLK_MGT_REG, 0x0024, "SGA"},	/* dis, SOC0/4, 200 MHz */
	{PRCM_UARTCLK_MGT_REG, 0x0300, "UART"},	/* ena, GATED, CLK38 */
	{PRCM_MSPCLK_MGT_REG, 0x0200, "MSP"},	/* dis, GATED, CLK38 */
	{PRCM_I2CCLK_MGT_REG, 0x0130, "I2C"},	/* ena, SOC0/16, 50 MHz */
	{PRCM_SDMMCCLK_MGT_REG, 0x0130, "SDMMC"}, /* ena, SOC0/16, 50 MHz */
	{PRCM_PER1CLK_MGT_REG, 0x126, "PER1"},	/* ena, SOC0/6, 133 MHz */
	{PRCM_PER2CLK_MGT_REG, 0x126, "PER2"},	/* ena, SOC0/6, 133 MHz */
	{PRCM_PER3CLK_MGT_REG, 0x126, "PER3"},	/* ena, SOC0/6, 133 MHz */
	{PRCM_PER5CLK_MGT_REG, 0x126, "PER5"},	/* ena, SOC0/6, 133 MHz */
	{PRCM_PER6CLK_MGT_REG, 0x126, "PER6"},	/* ena, SOC0/6, 133 MHz */
	{PRCM_PER7CLK_MGT_REG, 0x128, "PER7"},	/* ena, SOC0/8, 100 MHz */
	{PRCM_DMACLK_MGT_REG, 0x125, "DMA"},	/* ena, SOC0/5, 160 MHz */
	{PRCM_B2R2CLK_MGT_REG, 0x025, "B2R2"},	/* dis, SOC0/5, 160 MHz */
	{0, 0, NULL},
};

/* U5500 (Maja) alike clock settings */
static struct clk_mgt_regs maja_clk_regs[] = {
	{PRCM_SVAMMDSPCLK_MGT_REG, 0x1124, "SVA"},	/* SOC0/4, 200 MHz */
	{PRCM_SIAMMDSPCLK_MGT_REG, 0x0024, "SIA"},	/* SOC0/6, 133 MHz */
	{PRCM_SGACLK_MGT_REG, 0x0025, "SGA"},		/* SOC0/5, 160 MHz */
	{0, 0, NULL},
};

static void init_regs(void);

DECLARE_GLOBAL_DATA_PTR;
#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
    printf("Boot reached stage %d\n", progress);
}
#endif

#define CPUID_DB8500ED  0x410fc090
#define CPUID_DB8500V1  0x411fc091
#define CPUID_DB8500V2  0x412fc091

#define ASICID_DB8500V11         0x008500A1

static unsigned int read_cpuid(void)
{
	unsigned int val;

	/* Main ID register (MIDR) */
	asm("mrc        p15, 0, %0, c0, c0, 0"
	   : "=r" (val)
	   :
	   : "cc");

	return val;
}

static unsigned int read_asicid(void)
{
	unsigned int *address;

	if (u8500_is_earlydrop() || cpu_is_u8500v1())
		address = (void *) U8500_ASIC_ID_LOC_ED_V1;
	else
		address = (void *) U8500_ASIC_ID_LOC_V2;

	return readl(address);
}

int u8500_is_earlydrop(void)
{
	return read_cpuid() == CPUID_DB8500ED;
}

int cpu_is_u8500v1(void)
{
	return read_cpuid() == CPUID_DB8500V1;
}

int cpu_is_u8500v11(void)
{
	return read_asicid() == ASICID_DB8500V11;
}

int cpu_is_u8500v2(void)
{
	return read_cpuid() == CPUID_DB8500V2;
}

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

    gd->bd->bi_arch_number = MACH_TYPE_U8500;
    gd->bd->bi_boot_params = 0x00000100;

    if (u8500_is_earlydrop()) {
	/* MTU timer clock always enabled (not clocked) */
	writel(0x20000, PRCM_TCR);
    }
    icache_enable();
    gpio_init();

    init_regs();
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

unsigned int addr_vall_arr[] = {
0x8011F000, 0x0000FFFF, // Clocks for HSI  TODO Enable reqd only
0x8011F008, 0x00001CFF, // Clocks for HSI  TODO Enable reqd only
0x8000F000, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x8000F008, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x80157020, 0x00000150, // I2C 48MHz clock
0x8012F000, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0x8012F008, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0xA03DF000, 0x0000000D, // Clock for MTU Timers
0x8011E00C, 0x00000000, // GPIO ALT FUNC for EMMC
0x8011E004, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E020, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E024, 0x00000000, // GPIO ALT FUNC for EMMC
0x8012E000, 0x20000000, // GPIO ALT FUNC for UART
0x8012E00C, 0x00000000, // GPIO ALT FUNC for SD
0x8012E004, 0x0FFC0000, // GPIO ALT FUNC for SD
0x8012E020, 0x60000000, // GPIO ALT FUNC for SD
0x8012E024, 0x60000000, // GPIO ALT FUNC for SD
0x801571E4, 0x0000000C, // PRCMU settings for B2R2, PRCM_APE_RESETN_SET_REG
0x80157024, 0x00000130, // PRCMU settings for EMMC/SD
0xA03FF000, 0x00000003, // USB
0xA03FF008, 0x00000001, // USB
0xA03FE00C, 0x00000000, // USB
0xA03FE020, 0x00000FFF, // USB
0xA03FE024, 0x00000000	// USB
};

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

static void init_regs(void)
{
	/* FIXME Remove magic register array settings for ED also */
	if (u8500_is_earlydrop()) {
		int i;

		for(i = 0; i < ARRAY_SIZE(addr_vall_arr)/2; i++)
		{

			*((volatile unsigned int *)(addr_vall_arr[2 * i]))
				= addr_vall_arr[(2 * i) + 1];
		}
	} else {
		struct prcmu *prcmu = (struct prcmu *) U8500_PRCMU_BASE;

		/* Enable timers */
		writel(1 << 17, &prcmu->tcr);

		u8500_prcmu_enable(&prcmu->per1clk_mgt);
		u8500_prcmu_enable(&prcmu->per2clk_mgt);
		u8500_prcmu_enable(&prcmu->per3clk_mgt);
		u8500_prcmu_enable(&prcmu->per5clk_mgt);
		u8500_prcmu_enable(&prcmu->per6clk_mgt);
		u8500_prcmu_enable(&prcmu->per7clk_mgt);

		u8500_prcmu_enable(&prcmu->uartclk_mgt);
		u8500_prcmu_enable(&prcmu->i2cclk_mgt);

		u8500_prcmu_enable(&prcmu->sdmmcclk_mgt);

		u8500_clock_enable(1, 9, -1);	/* GPIO0 */

		if (u8500_is_earlydrop())
			u8500_clock_enable(2, 12, -1);	/* GPIO1 */
		else
			u8500_clock_enable(2, 11, -1);	/* GPIO1 */

		u8500_clock_enable(3, 8, -1);	/* GPIO2 */
		u8500_clock_enable(5, 1, -1);	/* GPIO3 */

		u8500_clock_enable(3, 6, 6);	/* UART2 */

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

		u8500_clock_enable(1, 5, 5);	/* SDI0 */
		u8500_clock_enable(2, 4, 2);	/* SDI4 */

		if (u8500_is_earlydrop())
			u8500_clock_enable(7, 2, -1);	/* MTU0 */
		else if (cpu_is_u8500v1())
			u8500_clock_enable(6, 7, -1);	/* MTU0 */
		else  if (cpu_is_u8500v2())
			u8500_clock_enable(6, 6, -1);	/* MTU0 */

		if (!u8500_is_earlydrop()) {
			u8500_clock_enable(3, 4, 4);	/* SDI2 */

			{
				/* 128 - 138 */
				struct gpio_register *p_gpio_register = (void *) IO_ADDRESS(CFG_GPIO_4_BASE);
				p_gpio_register -> gpio_dats |= 0x000007ff;
				p_gpio_register -> gpio_pdis &= ~0x000007ff;
			}

			gpio_altfuncenable(GPIO_ALT_POP_EMMC, "EMMC");
		}

		/*
		 * Enabling clocks for all devices which are AMBA devices in the
		 * kernel.  Otherwise they will not get probe()'d because the
		 * peripheral ID register will not be powered.
		 */

		/* XXX: some of these differ between ED/V1 */

		u8500_clock_enable(1, 1, 1);	/* UART1 */
		u8500_clock_enable(1, 0, 0);	/* UART0 */

		u8500_clock_enable(3, 2, 2);	/* SSP1 */
		u8500_clock_enable(3, 1, 1);	/* SSP0 */

		u8500_clock_enable(2, 8, -1);	/* SPI0 */
		u8500_clock_enable(2, 5, 3);	/* MSP2 */
	}
}


/*
 * get_pll_freq_khz - return PLL frequency in kHz
 */
static uint32_t get_pll_freq_khz(uint32_t inclk_khz, uint32_t freq_reg)
{
	uint32_t idf, ldf, odf, seldiv, phi;

	/*
	 * PLLOUTCLK = PHI = (INCLK*LDF)/(2*ODF*IDF) if SELDIV2=0
	 * PLLOUTCLK = PHI = (INCLK*LDF)/(4*ODF*IDF) if SELDIV2=1
	 * where:
	 * IDF=R(2:0) (when R=000, IDF=1d)
	 * LDF = 2*D(7:0) (D must be greater than or equal to 6)
	 * ODF = N(5:0) (when N=000000, 0DF=1d)
	 */

	idf = (freq_reg & 0x70000) >> 16;
	ldf = (freq_reg & 0xff) * 2;
	odf = (freq_reg & 0x3f00) >> 8;
	seldiv = (freq_reg & 0x01000000) >> 24;
	phi = (inclk_khz * ldf) / (2 * odf * idf);
	if (seldiv)
		phi = phi/2;

	return phi;
}

int do_clkinfo(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	uint32_t inclk_khz;
	uint32_t reg, phi;
	uint32_t clk_khz;
	unsigned int clk_sel;
	struct clk_mgt_regs *clks = clk_mgt_regs;
	struct pll_freq_regs *plls = pll_freq_regs;

	/*
	 * Go through list of PLLs.
	 * Initialise pll out frequency array (pll_khz) and print frequency.
	 */
	inclk_khz = 38400;	/* 38.4 MHz */
	while (plls->addr) {
		reg = readl(plls->addr);
		phi = get_pll_freq_khz(inclk_khz, reg);
		pll_khz[plls->idx] = phi;
		printf("%s PLL out frequency: %d.%d Mhz\n",
				pll_name[plls->idx], phi/1000, phi % 1000);
		plls++;
	}

	/* check ARM clock source */
	reg = readl(PRCM_ARM_CHGCLKREQ_REG);
	printf("A9 running on ");
	if (reg & 1)
		printf("external clock");
	else
		printf("ARM PLL");
	printf("\n");

	/* go through list of clk_mgt_reg */
	printf("\n%19s %9s %7s %9s  enabled\n",
			"name(addr)", "value", "PLL", "CLK[MHz]");
	while (clks->addr) {
		reg = readl(clks->addr);
		/* convert bit position into array index */
		clk_sel = ffs((reg >> 5) & 0x7);	/* PLLSW[2:0] */
		printf("%9s(%08x): %08x", clks->descr, clks->addr, reg);
		printf(", %6s", pll_name[clk_sel]);
		if (reg & 0x200)
			clk_khz = 38400;	/* CLK38 is set */
		else if ((reg & 0x1f) == 0)
			/* ARMCLKFIX_MGT is 0x120, e.g. div = 0 ! */
			clk_khz = 0;
		else
			clk_khz = pll_khz[clk_sel] / (reg & 0x1f);
		printf(", %4d.%03d", clk_khz / 1000, clk_khz % 1000);
		printf(", %s\n", (reg & 0x100) ? "ena" : "dis");
		clks++;
	}

	return 0;
}

U_BOOT_CMD(
	clkinfo,	1,	1,	do_clkinfo,
	"print clock info",
	""
);

/*
 * do_clkmaja - change certain register to imitate Maja performance
 */
int do_clkmaja(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	uint32_t val;
	struct clk_mgt_regs *regs = maja_clk_regs;

	while (regs->addr) {
		val = readl(regs->addr);
		printf("%s(%08x): %08x -> %08x\n", regs->descr, regs->addr,
				val, regs->val);
		writel(regs->val, regs->addr);
		regs++;
	}

	return 0;
}

U_BOOT_CMD(
	clkmaja,	1,	1,	do_clkmaja,
	"set some clocks maja alike",
	""
);


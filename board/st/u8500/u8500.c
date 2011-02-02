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
#include <asm/arch/itp.h>
#include <tc35892.h>

#include <asm/arch/common.h>

#include "db8500_pincfg.h"
#include "db8500_pins.h"


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

/*
 * GPIO pin config common for MOP500/HREF boards
 */
pin_cfg_t gpio_cfg_common[] = {
	/* I2C */
	GPIO147_I2C0_SCL,
	GPIO148_I2C0_SDA,
	GPIO16_I2C1_SCL,
	GPIO17_I2C1_SDA,
	GPIO10_I2C2_SDA,
	GPIO11_I2C2_SCL,
	GPIO229_I2C3_SDA,
	GPIO230_I2C3_SCL,

	/* SSP0, to AB8500 */
	GPIO143_SSP0_CLK,
	GPIO144_SSP0_FRM,
	GPIO145_SSP0_RXD | PIN_PULL_DOWN,
	GPIO146_SSP0_TXD,

	/* MMC0 (MicroSD card) */
	GPIO18_MC0_CMDDIR	| PIN_OUTPUT_HIGH,
	GPIO19_MC0_DAT0DIR	| PIN_OUTPUT_HIGH,
	GPIO20_MC0_DAT2DIR	| PIN_OUTPUT_HIGH,
	GPIO21_MC0_DAT31DIR	| PIN_OUTPUT_HIGH,
	GPIO22_MC0_FBCLK	| PIN_INPUT_NOPULL,
	GPIO23_MC0_CLK		| PIN_OUTPUT_LOW,
	GPIO24_MC0_CMD		| PIN_INPUT_PULLUP,
	GPIO25_MC0_DAT0		| PIN_INPUT_PULLUP,
	GPIO26_MC0_DAT1		| PIN_INPUT_PULLUP,
	GPIO27_MC0_DAT2		| PIN_INPUT_PULLUP,
	GPIO28_MC0_DAT3		| PIN_INPUT_PULLUP,

	/* MMC2 (POP eMMC) */
	GPIO128_MC2_CLK		| PIN_OUTPUT_LOW,
	GPIO129_MC2_CMD		| PIN_INPUT_PULLUP,
	GPIO130_MC2_FBCLK	| PIN_INPUT_NOPULL,
	GPIO131_MC2_DAT0	| PIN_INPUT_PULLUP,
	GPIO132_MC2_DAT1	| PIN_INPUT_PULLUP,
	GPIO133_MC2_DAT2	| PIN_INPUT_PULLUP,
	GPIO134_MC2_DAT3	| PIN_INPUT_PULLUP,
	GPIO135_MC2_DAT4	| PIN_INPUT_PULLUP,
	GPIO136_MC2_DAT5	| PIN_INPUT_PULLUP,
	GPIO137_MC2_DAT6	| PIN_INPUT_PULLUP,
	GPIO138_MC2_DAT7	| PIN_INPUT_PULLUP,

	/* MMC4 (On-board eMMC) */
	GPIO197_MC4_DAT3	| PIN_INPUT_PULLUP,
	GPIO198_MC4_DAT2	| PIN_INPUT_PULLUP,
	GPIO199_MC4_DAT1	| PIN_INPUT_PULLUP,
	GPIO200_MC4_DAT0	| PIN_INPUT_PULLUP,
	GPIO201_MC4_CMD		| PIN_INPUT_PULLUP,
	GPIO202_MC4_FBCLK	| PIN_INPUT_NOPULL,
	GPIO203_MC4_CLK		| PIN_OUTPUT_LOW,
	GPIO204_MC4_DAT7	| PIN_INPUT_PULLUP,
	GPIO205_MC4_DAT6	| PIN_INPUT_PULLUP,
	GPIO206_MC4_DAT5	| PIN_INPUT_PULLUP,
	GPIO207_MC4_DAT4	| PIN_INPUT_PULLUP,

	/* UART2, console */
	GPIO29_U2_RXD	| PIN_INPUT_PULLUP,
	GPIO30_U2_TXD	| PIN_OUTPUT_HIGH,
	GPIO31_U2_CTSn	| PIN_INPUT_PULLUP,
	GPIO32_U2_RTSn	| PIN_OUTPUT_HIGH,

	/*
	 * USB, pin 256-267 USB, Is probably already setup correctly from
	 * BootROM/boot stages, but we don't trust that and set it up anyway
	 */
	GPIO256_USB_NXT,
	GPIO257_USB_STP,
	GPIO258_USB_XCLK,
	GPIO259_USB_DIR,
	GPIO260_USB_DAT7,
	GPIO261_USB_DAT6,
	GPIO262_USB_DAT5,
	GPIO263_USB_DAT4,
	GPIO264_USB_DAT3,
	GPIO265_USB_DAT2,
	GPIO266_USB_DAT1,
	GPIO267_USB_DAT0,
};

/*
 * GPIO pin config for HREF+ V60 board.
 * Contains additional settings to common mop500 settings above.
 */
pin_cfg_t gpio_cfg_hrefv60[] = {
       /* SDMMC */
	GPIO169_GPIO	| PIN_OUTPUT_LOW,	/* SDMMC_Enable */
	GPIO5_GPIO	| PIN_OUTPUT_LOW,	/* SDMMC_1V8_3V_SEL */
	GPIO95_GPIO	| PIN_INPUT_PULLUP,	/* SDMMC_CD */

	/* Display Interface */
	GPIO65_GPIO	| PIN_OUTPUT_LOW,	/* DISP1 RST */
	GPIO66_GPIO	| PIN_OUTPUT_LOW,	/* DISP2 RST */
};

pin_cfg_t gpio_cfg_snowball[] = {
	/* MMC0 (MicroSD card) */
	GPIO217_GPIO    | PIN_OUTPUT_HIGH,      /* MMC_EN */
	GPIO218_GPIO    | PIN_INPUT_NOPULL,     /* MMC_CD */
	GPIO228_GPIO    | PIN_OUTPUT_HIGH,      /* SD_SEL */

	/* eMMC */
	GPIO167_GPIO    | PIN_OUTPUT_HIGH,      /* RSTn_MLC */
};

#define BOARD_ID_MOP500		0
#define BOARD_ID_HREF		1
#define BOARD_ID_HREFV60	2
#define BOARD_ID_SNOWBALL      3
int board_id;	/* set in probe_href() */

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

	/* Configure GPIO pins needed by U-boot */
	db8500_gpio_config_pins(gpio_cfg_common, ARRAY_SIZE(gpio_cfg_common));

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

#if CONFIG_SYS_DISPLAY_DSI
static int mcde_display_reset_gpioe(void)
{
	int ret;

	/* Only main display should be initialized */
	ret = tc35892_gpio_dir(CONFIG_SYS_I2C_GPIOE_ADDR,
				TC35892_PIN_KPY7, 1);
	if (ret) {
		printf("%s:Could not set direction for gpio\n", __func__);
		return -EINVAL;
	}
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				TC35892_PIN_KPY7, 0);
	if (ret) {
		printf("%s:Could reset gpio\n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				TC35892_PIN_KPY7, 1);
	if (ret) {
		printf("%s:Could set gpio\n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);

	return ret;
}

/*
 * Reset the primary display - called from mcde driver.
 */
int board_mcde_display_reset(void)
{
	int ret = 0;

	if (board_id >= BOARD_ID_HREFV60) {
		db8500_gpio_set_output(GPIO65_GPIO, 0); /* DISP1 reset */
		mdelay(main_display_data.reset_delay);
		db8500_gpio_set_output(GPIO65_GPIO, 1);
		mdelay(main_display_data.reset_delay);
	} else
		ret = mcde_display_reset_gpioe();

	return ret;
}
#endif /* CONFIG_SYS_DISPLAY_DSI */

static int dss_init(void)
{
	puts("\nMCDE:  ");

	boottime_tag("splash");

	if (!cpu_is_u8500v11() && !cpu_is_u8500v2()) {
		printf("Only HREF+ or V2 is supported\n");
		goto mcde_error;
	}
	if (mcde_splash_image()) {
		printf("startup failed\n");
		goto mcde_error;
	}

	printf("ready\n");
	return 0;

mcde_error:
	return -EINVAL;

}
#endif

/*
 * probe_href - set board_id according to HREF version
 *
 * side-effect: configures additional GPIO pins if necessary.
 */
static void probe_href(void)
{
	uchar byte;

	/*
	 * Determine and set board_id
	 * 0: mop500, 1: href500, 2: href500 2.0 V60 or later
	 * Above boards have different GPIO expander chips which we can
	 * distinguish by the chip id.
	 * HREF+ 2.0 V60 and later have no GPIOE.
	 *
	 */

	if (gd->bd->bi_arch_number == MACH_TYPE_U8500) {
		(void) i2c_set_bus_num(0);
		if (!i2c_read(CONFIG_SYS_I2C_GPIOE_ADDR, 0x80, 1, &byte, 1)) {
			if (byte == 0x01)
				board_id = BOARD_ID_MOP500;
			else
				board_id = BOARD_ID_HREF;
		} else if(u8500_is_snowball()) {
			gd->bd->bi_arch_number = MACH_TYPE_SNOWBALL;

			db8500_gpio_config_pins(gpio_cfg_snowball,
					ARRAY_SIZE(gpio_cfg_snowball));

			board_id = BOARD_ID_SNOWBALL;
		} else{
			/* No GPIOE => HREF+ 2.0 V60 or later */
			gd->bd->bi_arch_number = MACH_TYPE_HREFV60;

			db8500_gpio_config_pins(gpio_cfg_hrefv60,

					ARRAY_SIZE(gpio_cfg_hrefv60));
			board_id = BOARD_ID_HREFV60;
		}
	}
}

#define BATT_OK_SEL1_TH_F_MASK		0xF0
#define BATT_OK_SEL1_TH_F_2V71		0x70

/*
 * board_early_access - for functionality that needs to run before
 * board_late_init but after board_init and emmc init.
 */
int board_early_access(block_dev_desc_t *block_dev)
{
	int ret;
	int battok_regval;

	/* set board_id according to HREF version */
	probe_href();

	/*
	 * In AB8500 rev2.0, the cut-off voltage threshold is set too low
	 * and the AB will power-off when we start with a drained battery
	 * and a charger connected when the backlight is turned on.
	 * Here we will lower the cut-off voltage threshold before
	 * power consumption goes up
	 */
	ret = ab8500_read(AB8500_SYS_CTRL2_BLOCK, AB8500_BATT_OK_REG);
	if (ret < 0)
		return -EINVAL;

	battok_regval = ret;

	/* Mask and set BattOkSel1ThF */
	ret = ab8500_write(AB8500_SYS_CTRL2_BLOCK, AB8500_BATT_OK_REG,
		(battok_regval & ~BATT_OK_SEL1_TH_F_MASK) |
		BATT_OK_SEL1_TH_F_2V71);
	if (ret < 0)
		return -EINVAL;

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
 * Called after all initialisation was done.
 */
int board_late_init(void)
{
	uchar byte;
#ifdef CONFIG_MMC
	uchar byte_array[] = {0x06, 0x06};
#endif
	char strbuf[80];

	/*
	 * The board_id environment variable is needed for the Linux bootargs.
	 */
	if (board_id == 0)
		setenv("board_id", "0");
	else
		setenv("board_id", "1");

	/* For snowball (aka "minikit") we need to raise AB8500's GPIO26 */
	ret = ab8500_read(AB8500_MISC, AB8500_GPIO_DIR4_REG);
	if (ret < 0) {
		printf("error at %s:%i\n", __func__, __LINE__);
		goto out;
	}
	printf("dir4 = %02x\n", ret);
	ret |= 0x2;
	ret = ab8500_write(AB8500_MISC, AB8500_GPIO_DIR4_REG, ret);
	if (ret < 0) {
		printf("error at %s:%i\n", __func__, __LINE__);
		goto out;
	}

	ret = ab8500_read(AB8500_MISC, AB8500_GPIO_OUT4_REG);
	if (ret < 0) {
		printf("error at %s:%i\n", __func__, __LINE__);
		goto out;
	}
	printf("out4 = %02x\n", ret);
	ret |= 0x2;
	ret = ab8500_write(AB8500_MISC, AB8500_GPIO_OUT4_REG, ret);
	if (ret < 0) {
		printf("error at %s:%i\n", __func__, __LINE__);
		goto out;
	}
out:
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
	} else if (board_id == 1) {
		/* HREF < V60 */
		/* set the direction of GPIO KPY9 and KPY10 */
		byte = 0x06;
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0xC8, 1, &byte, 1);
		/* must be a multibyte access */
		(void) i2c_write(CONFIG_SYS_I2C_GPIOE_ADDR, 0xC4, 1,
						&byte_array[0], 2);
	} else
		/* HREF V60 and later */
		/* enable SDMMC */
		db8500_gpio_set_output(GPIO169_GPIO, 1);
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

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

#include <config.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>

#include "common.h"
#define NOMADIK_PER4_BASE       (0x80150000)
#define NOMADIK_BACKUPRAM0_BASE (NOMADIK_PER4_BASE + 0x00000)
#define NOMADIK_BACKUPRAM1_BASE (NOMADIK_PER4_BASE + 0x01000)

/* Power, Reset, Clock Management Unit */
#define PRCMU_BASE	CFG_PRCMU_BASE
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

enum {
	GATED = 0,
	PLLSOC0,	/* pllsw = 001 */
	PLLSOC1,	/* pllsw = 010 */
	PLLDDR,		/* pllsw = 100 */
};

static const char *pll_name[4] = {"GATED", "SOC0", "SOC1", "DDR"};
static uint32_t pll_khz[4];	/* use ffs(pllsw(reg)) as index */

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
	{0, 0, NULL},
};

/* U5500 (Maja) alike clock settings */
static struct clk_mgt_regs maja_clk_regs[] = {
	{PRCM_SVAMMDSPCLK_MGT_REG, 0x1124, "SVA"},	/* SOC0/4, 200 MHz */
	{PRCM_SIAMMDSPCLK_MGT_REG, 0x0024, "SIA"},	/* SOC0/6, 133 MHz */
	{PRCM_SGACLK_MGT_REG, 0x0025, "SGA"},		/* SOC0/5, 160 MHz */
	{0, 0, NULL},
};

extern void (*handler)();
extern void secondary_wfe();

void wake_up_other_cores(void)
{
	handler = secondary_wfe;
	*((volatile unsigned int *)(NOMADIK_BACKUPRAM0_BASE+0x1FF4))= handler;
	*((volatile unsigned int *)(NOMADIK_BACKUPRAM0_BASE+0x1FF0))= 0xA1FEED01;
	asm("SEV");
	return;
}

static void init_regs(void);

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
    gd->bd->bi_arch_number = 0x1A4;
    gd->bd->bi_boot_params = 0x00000100;
    //enable the timers in PRCMU reg
    *((volatile unsigned int *)(CFG_PRCMU_BASE + 0x1C8)) = 0x20000;
    icache_enable();
    gpio_init();

    init_regs();
    return 0;
}

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
    setenv("verify", "n");
    return (0);
}
#endif

int dram_init(void)
{
    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_SIZE_1;
#ifdef CONFIG_U8500_V1
    gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
    gd->bd->bi_dram[1].size = PHYS_SDRAM_SIZE_2;
#endif
    wake_up_other_cores();
    return 0;
}

unsigned int addr_vall_arr[] = {
0x8011F000, 0x0000FFFF, // Clocks for HSI  TODO Enable reqd only
0x8011F008, 0x00001C44, // Clocks for HSI  TODO Enable reqd only
0x8000F000, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x8000F008, 0x00007FFF, // Clocks for I2C  TODO Enable reqd only
0x8000E120, 0x003C0000, // GPIO for I2C/SD
0x8000E124, 0x00000000, // GPIO for I2C/SD
0x80157020, 0x00000130, // I2C 48MHz clock
0x8012F000, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0x8012F008, 0x00007FFF, // Clocks for SD  TODO Enable reqd only
0xA03DF000, 0x0000000D, // Clock for MTU Timers
0x8011E00C, 0x00000000, // GPIO ALT FUNC for EMMC
0x8011E004, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E020, 0x0000FFE0, // GPIO ALT FUNC for EMMC
0x8011E024, 0x00000000, // GPIO ALT FUNC for EMMC
0x8012E00C, 0x00000000, // GPIO ALT FUNC for SD
0x8012E004, 0x0FFC0000, // GPIO ALT FUNC for SD
0x8012E020, 0x60000000, // GPIO ALT FUNC for SD
0x8012E024, 0x60000000, // GPIO ALT FUNC for SD
0x801571E4, 0x0000000C, // PRCMU settings for B2R2
0x80157024, 0x00000130, // PRCMU settings for EMMC/SD
0xA03FF000, 0x00000003, // USB
0xA03FF008, 0x00000001, // USB
0xA03FE00C, 0x00000000, // USB
0xA03FE020, 0x00000FFF, // USB
0xA03FE024, 0x00000000  // USB
};

#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
	return (0);
}
#endif

static void init_regs(void)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(addr_vall_arr)/2; i++)
    {

        *((volatile unsigned int *)(addr_vall_arr[2 * i]))
                                = addr_vall_arr[(2 * i) + 1];
    }
}

/*
 * get_pll_freq_khz - return PLL frequency in kHz
 */
static get_pll_freq_khz(uint32_t inclk_khz, uint32_t freq_reg)
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
	uint32_t inclk_khz = 38400;	/* 38.4 MHz */
	uint32_t reg, phi;
	uint32_t val;
	uint32_t clk_khz;
	unsigned int clk_sel;
	struct clk_mgt_regs *regs = clk_mgt_regs;

	/* DDR PLL */
	reg =  *(uint32_t *)0x8015708C;
	phi = get_pll_freq_khz(inclk_khz, reg);
	pll_khz[PLLDDR] = phi;
	printf("\nDDR PLL out frequency: %d.%d Mhz\n",
			phi/1000, phi % 1000);
	/* ARM PLL */
	reg =  *(uint32_t *)0x80157088;
	phi = get_pll_freq_khz(inclk_khz, reg);
	printf("ARM PLL out frequency: %d.%d Mhz\n",
			phi/1000, phi % 1000);
	reg =  *(uint32_t *)0x80157114; /* PRCM_ARM_CHGCLKREQ_REG */
	printf("A9 running on ");
	if (reg & 1)
		printf("external clock");
	else
		printf("ARM PLL");
	printf("\n");

	/* SOC0 Pll */
	reg =  *(uint32_t *)0x80157080;
	phi = get_pll_freq_khz(inclk_khz, reg);
	pll_khz[PLLSOC0] = phi;
	printf("SOC0 PLL out frequency: %d.%d Mhz\n",
			phi/1000, phi % 1000);

	/* SOC1 Pll */
	reg =  *(uint32_t *)0x80157084;
	phi = get_pll_freq_khz(inclk_khz, reg);
	pll_khz[PLLSOC1] = phi;
	printf("SOC1 PLL out frequency: %d.%d Mhz\n",
			phi/1000, phi % 1000);
	printf("\n");

	/* go through list of clk_mgt_reg */
	while (regs->addr) {
		val = readl(regs->addr);
		/* convert bit position into array index */
		clk_sel = ffs((val >> 5) & 0x7);	/* PLLSW[2:0] */
		printf("%s(%08x): %08x", regs->descr, regs->addr, val);
		printf(", PLL %s", pll_name[clk_sel]);
		if (val & 0x200)
			clk_khz = 38400;	/* CLK38 is set */
		else if ((val & 0x1f) == 0)
			/* ARMCLKFIX_MGT is 0x120, e.g. div = 0 ! */
			clk_khz = 0;
		else
			clk_khz = pll_khz[clk_sel] / (val & 0x1f);
		printf(", CLK %d.%d MHz", clk_khz / 1000, clk_khz % 1000);
		printf(", %s", (val & 0x100) ? "ena" : "dis");
		printf("\n");
		regs++;
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
}

U_BOOT_CMD(
	clkmaja,	1,	1,	do_clkmaja,
	"set some clocks maja alike",
	""
);


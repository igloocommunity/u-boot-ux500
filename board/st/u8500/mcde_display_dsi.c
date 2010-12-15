/*
* Copyright (C) ST-Ericsson SA 2010
*
* Author: Jimmy Rubin <jimmy.rubin@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/


#include <common.h>
#include <command.h>
#include <asm/arch/common.h>
#include "mcde_display.h"
#include "dsilink_regs.h"
#include <tc35892.h>
#include "mcde_regs.h"
#include <malloc.h>
#include "mcde.h"
#include <linux/err.h>
#include <asm/arch/ab8500.h>

static int mcde_enable_gpio(void)
{
	int ret;

	debug("%s: enter\n", __func__);

	/* Only main display should be initialized */
	ret = tc35892_gpio_dir(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 1);
	if (ret) {
		printf("%s:Could not set direction for gpio\n", __func__);
		return -EINVAL;
	}
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 0);
	if (ret) {
		printf("%s:Could reset gpio\n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 1);
	if (ret) {
		printf("%s:Could set gpio\n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);
	return 0;
}

#define DCS_CMD_EXIT_SLEEP_MODE       0x11
#define DCS_CMD_SET_DISPLAY_ON        0x29

int mcde_turn_on_display_dsi(void)
{
	int ret = 0;

	debug("%s: enter\n", __func__);
	ret = mcde_dsi_dcs_write(main_display.port,
				DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
	if (!ret) {
		debug("mcde_dsi_dcs_write "
				"DCS_CMD_EXIT_SLEEP_MODE success!\n");
		ret = mcde_dsi_dcs_write(main_display.port,
				DCS_CMD_SET_DISPLAY_ON,  NULL, 0);
		if (!ret)
			debug("mcde_dsi_dcs_write "
				"DCS_CMD_SET_DISPLAY_ON success!\n");
	}
	return ret;
}

/* aux supplies */
#define MASK_LDO_VAUX1		0x3
#define MASK_LDO_VAUX1_SHIFT	0x0
#define VAUXSEL_VOLTAGE_MASK	0xf

#define VANA_ENABLE_IN_HP_MODE	0x05

#define ENABLE_PWM1		0x01
#define PWM_DUTY_LOW_1024_1024	0xFF
#define PWM_DUTY_HI_1024_1024	0x03

/*
 * regulator layout
 * @voltage: supported voltage
 * @regval: register value to be written
 */
struct regulator_voltage {
	int voltage;
	int regval;
};

/* voltage table for VAUXn regulators */
static struct regulator_voltage vauxn_table[] = {
	{ .voltage = 1100000, .regval  = 0x0, },
	{ .voltage = 1200000, .regval  = 0x1, },
	{ .voltage = 1300000, .regval  = 0x2, },
	{ .voltage = 1400000, .regval  = 0x3, },
	{ .voltage = 1500000, .regval  = 0x4, },
	{ .voltage = 1800000, .regval  = 0x5, },
	{ .voltage = 1850000, .regval  = 0x6, },
	{ .voltage = 1900000, .regval  = 0x7, },
	{ .voltage = 2500000, .regval  = 0x8, },
	{ .voltage = 2650000, .regval  = 0x9, },
	{ .voltage = 2700000, .regval  = 0xa, },
	{ .voltage = 2750000, .regval  = 0xb, },
	{ .voltage = 2800000, .regval  = 0xc, },
	{ .voltage = 2900000, .regval  = 0xd, },
	{ .voltage = 3000000, .regval  = 0xe, },
	{ .voltage = 3300000, .regval  = 0xf, },
};


/*
 * This code is derived from the handling of AB8500_LDO_VAUX1 in
 * ab8500_ldo_is_enabled in Linux.
 */
static int mcde_is_vaux1_enabled(void)
{
	int val;
	val = ab8500_read(AB8500_REGU_CTRL2,
			AB8500_REGU_VAUX12_REGU_REG);
	if (val & MASK_LDO_VAUX1)
		return TRUE;
	return FALSE;
}

/*
 * This code is derived from the handling of AB8500_LDO_VAUX1 in
 * ab8500_ldo_get_voltage in Linux.
 */
static int mcde_get_vaux1_voltage(void)
{
	int val;
	val = ab8500_read(AB8500_REGU_CTRL2,
		AB8500_REGU_VAUX1_SEL_REG);
	return vauxn_table[val & VAUXSEL_VOLTAGE_MASK].voltage;
}

static int mcde_display_power_init(void)
{
	int val;
	int i;

	debug("%s: enter\n", __func__);
	/*
	 * On v1.1 HREF boards (HREF+) and V2 boards
	 * Vaux1 needs to be enabled for the
	 * display to work.  This is done by enabling the regulators in the
	 * AB8500 via PRCMU I2C transactions.
	 *
	 * This code is derived from the handling of AB8500_LDO_VAUX1 in
	 * ab8500_ldo_enable(), ab8500_ldo_disable() and
	 * ab8500_get_best_voltage in Linux.
	 *
	 * Turn off and delay is required to have it work across soft reboots.
	 */

	val = ab8500_read(AB8500_REGU_CTRL2,
		AB8500_REGU_VAUX12_REGU_REG);
	if (val < 0) {
		printf("Read vaux1 status failed\n");
		return -EINVAL;
	}

	/* Turn off */
	if (ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VAUX12_REGU_REG,
			   val & ~MASK_LDO_VAUX1) < 0) {
		printf("Turn off Vaux1 failed\n");
		return -EINVAL;
	}

	udelay(10 * 1000);


	/* Find voltage from vauxn table */
	for (i = 0; i < ARRAY_SIZE(vauxn_table) ; i++) {
		if (vauxn_table[i].voltage == CONFIG_SYS_DISPLAY_VOLTAGE) {
			if (ab8500_write(AB8500_REGU_CTRL2,
				AB8500_REGU_VAUX1_SEL_REG,
				vauxn_table[i].regval) < 0) {
				printf("AB8500_REGU_VAUX1_SEL_REG failed\n");
				return -EINVAL;
			}
			break;
		}
	}

	val = val & ~MASK_LDO_VAUX1;
	val = val | (1 << MASK_LDO_VAUX1_SHIFT);

	/* Turn on the supply */
	if (ab8500_write(AB8500_REGU_CTRL2,
			AB8500_REGU_VAUX12_REGU_REG, val) < 0) {
		printf("Turn on Vaux1 failed\n");
		return -EINVAL;
	}

	/*  DCI & CSI (DSI / PLL / Camera) */ /* Vana & Vpll HP mode */
	if (ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VPLLVANA_REGU_REG,
						VANA_ENABLE_IN_HP_MODE) < 0) {
		printf("Turn on Vana failed\n");
		return -EINVAL;
	}

	/* Enable the PWM control for the backlight Main display */
	if (ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL7_REG,
							ENABLE_PWM1) < 0) {
		printf("Enable PWM1 failed\n");
		return -EINVAL;
	}
	if (ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL1_REG,
						PWM_DUTY_LOW_1024_1024) < 0) {
		printf("PWM_DUTY_LOW_1024_1024 failed\n");
		return -EINVAL;
	}
	if (ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL2_REG,
						PWM_DUTY_HI_1024_1024) < 0) {
		printf("PWM_DUTY_HI_1024_1024 failed\n");
		return -EINVAL;
	}

	if (!mcde_is_vaux1_enabled() || mcde_get_vaux1_voltage()
					!= CONFIG_SYS_DISPLAY_VOLTAGE) {
		printf("VAUX is %d and is set to %d V should be %d\n"
			, mcde_is_vaux1_enabled(), mcde_get_vaux1_voltage()
			, CONFIG_SYS_DISPLAY_VOLTAGE);
		return -EINVAL;
	}

	return 0;
}


int mcde_startup_dsi(struct mcde_platform_data *pdata)
{
	u8 num_dsilinks;
	int ret;

	debug("%s: enter\n", __func__);

	if (main_display.port->mode != MCDE_PORTMODE_CMD) {
		printf("%s: only CMD mode supported\n", __func__);
		return -ENODEV;
	}
	num_dsilinks = main_display.port->phy.dsi.num_data_lanes;
	mcde_init();
	ret = mcde_probe(num_dsilinks, pdata);
	if (ret) {
		printf("%s: mcde_probe() -> %d\n", __func__, ret);
		return ret;
	}
	ret = mcde_display_power_init();
	if (ret) {
		printf("%s: mcde_display_power_init() -> %d\n", __func__, ret);
		return ret;
	}

	ret = mcde_enable_gpio();
	if (ret) {
		printf("%s: mcde_enable_gpio() -> %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

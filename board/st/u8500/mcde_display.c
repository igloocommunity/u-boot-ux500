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
#include <asm/arch/gpio.h>
#include "mcde_display.h"
#include "dsilink_regs.h"
#include <tc35892.h>
#include "mcde_regs.h"
#include <malloc.h>
#include "mcde.h"
#include <linux/err.h>
#include <asm/arch/ab8500.h>
#include <asm/arch/common.h>
#include <part.h>
#include <mmc.h>

#define DEBUG 0
#define dbg_printk(format, arg...)			\
	if (DEBUG)					\
		printf("mcde: " format, ##arg)		\

static struct mcde_chnl_state *chnl;

static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.ifc = 0,
	.link = 0,
	.sync_src = MCDE_SYNCSRC_BTA,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
		},
	},
};

struct mcde_display_generic_platform_data main_display_data = {
	.reset_gpio = TC35892_PIN_KPY7,
	.reset_delay = 10,
};

struct mcde_display_device main_display = {
	.port = &port0,
	.chnl_id = MCDE_CHNL_C0,
	.fifo = MCDE_FIFO_A,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.port_pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.native_x_res = CONFIG_SYS_DISPLAY_NATIVE_X_RES,
	.native_y_res = CONFIG_SYS_DISPLAY_NATIVE_Y_RES,
};

static int mcde_enable_gpio(void)
{
	int ret;
	dbg_printk("Enable GPIO pins!\n");

	/* Only main display should be initialized */
	ret = tc35892_gpio_dir(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 1);
	if (ret) {
		printf("%s:Could not set direction for gpio \n", __func__);
		return -EINVAL;
	}
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 0);
	if (ret) {
		printf("%s:Could reset gpio \n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);
	ret = tc35892_gpio_set(CONFIG_SYS_I2C_GPIOE_ADDR,
				main_display_data.reset_gpio, 1);
	if (ret) {
		printf("%s:Could set gpior\n", __func__);
		return -EINVAL;
	}
	mdelay(main_display_data.reset_delay);

	dbg_printk("All needed GPIOS enabled!\n");
	return 0;
}

#define DCS_CMD_EXIT_SLEEP_MODE       0x11
#define DCS_CMD_SET_DISPLAY_ON        0x29

static int mcde_turn_on_display(void)
{
	int ret = 0;
	dbg_printk("Turn on display!\n");
	ret = mcde_dsi_dcs_write(main_display.port,
				DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
	if (!ret) {
		dbg_printk("mcde_dsi_dcs_write "
				"DCS_CMD_EXIT_SLEEP_MODE success!\n");
		ret = mcde_dsi_dcs_write(main_display.port,
				DCS_CMD_SET_DISPLAY_ON,  NULL, 0);
		if (!ret)
			dbg_printk("mcde_dsi_dcs_write "
				"DCS_CMD_SET_DISPLAY_ON success!\n");
	}

	return ret;
}

/* aux supplies */
#define MASK_LDO_VAUX1		(0x3)
#define MASK_LDO_VAUX1_SHIFT	(0x0)
#define VAUXSEL_VOLTAGE_MASK	(0xf)

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


int mcde_startup(void)
{
	u8 num_dsilinks;
	int ret;

	num_dsilinks = main_display.port->phy.dsi.num_data_lanes;
	mcde_init(num_dsilinks);
	ret = mcde_display_power_init();
	if (ret)
		goto display_power_failed;
	mcde_enable_dss();

	ret = mcde_enable_gpio();
	if (ret)
		goto enable_gpio_failed;

	chnl = mcde_chnl_get(main_display.chnl_id, main_display.fifo,
							main_display.port);
	if (IS_ERR(chnl)) {
		ret = PTR_ERR(chnl);
		printf("%s:Failed to acquire MCDE channel\n", __func__);
		goto get_chnl_failed;
	}

	ret = mcde_turn_on_display();
	if (ret)
		goto display_power_mode_failed;
	mcde_chnl_set_update_area(chnl, 0, 0, main_display.native_x_res,
						main_display.native_y_res);
	mcde_chnl_set_pixel_format(chnl, main_display.port_pixel_format);
	mcde_chnl_apply(chnl);

	return ret;

display_power_mode_failed:
get_chnl_failed:
display_power_failed:
enable_gpio_failed:
	mcde_exit();
	return ret;
}

int mcde_display_image(void)
{
	struct mcde_ovly_state *ovly;
	u32 xpos = 0;
	u32 ypos = 0;
	int ret;
	struct mmc *emmc_dev;
	u32 address = CONFIG_SYS_VIDEO_FB_ADRS;

#ifdef CONFIG_SYS_VIDEO_USE_GIMP_HEADER
	u32	i = 0;
	u8 	pixels[3];
	u16	pixel;
	u16	*sp;
#endif

	ovly = mcde_ovly_get(chnl);
	if (IS_ERR(ovly)) {
		ret = PTR_ERR(ovly);
		printf("Failed to get channel\n");
		return -ret;
	}

	emmc_dev = find_mmc_device(CONFIG_EMMC_DEV_NUM);
	if (emmc_dev == NULL) {
		printf("mcde_display_image: emmc not found.\n");
		return 1;
	}

	if (toc_load_toc_entry(&emmc_dev->block_dev, MCDE_TOC_SPLASH_NAME, 0,
			       0, address)) {
		printf("mcde_display_image: no splash image found.\n");
		return 1;
	}

#ifdef CONFIG_SYS_VIDEO_USE_GIMP_HEADER
	/* Add the image data */
	sp = (u16 *)CONFIG_SYS_VIDEO_FB_ADRS;
	for (i = 0; i < ((MCDE_VIDEO_LOGO_WIDTH*MCDE_VIDEO_LOGO_HEIGHT)); i++) {
		HEADER_PIXEL(header_data, pixels);
		pixels[0] >>= 3; /* Keep 5 bits red */
		pixels[1] >>= 2; /* 6 bits green */
		pixels[2] >>= 3; /* and 5 bits blue */
		pixel = (pixels[0] << 11) | (pixels[1] << 5) | pixels[2];
		*sp++ = pixel;
	}
	mcde_ovly_set_source_buf(ovly, CONFIG_SYS_VIDEO_FB_ADRS);
#else
	mcde_ovly_set_source_buf(ovly, (u32)address);
#endif
	mcde_ovly_set_source_info(ovly, (MCDE_VIDEO_LOGO_WIDTH*2),
					main_display.default_pixel_format);
	mcde_ovly_set_source_area(ovly, 0, 0, MCDE_VIDEO_LOGO_WIDTH,
						MCDE_VIDEO_LOGO_HEIGHT);
	if (MCDE_VIDEO_LOGO_WIDTH == main_display.native_x_res)
		xpos = 0;
	else
		xpos = (main_display.native_x_res - MCDE_VIDEO_LOGO_WIDTH) / 2;

	if (MCDE_VIDEO_LOGO_HEIGHT == main_display.native_y_res)
		ypos = 0;
	else
		ypos = (main_display.native_y_res - MCDE_VIDEO_LOGO_HEIGHT) / 2;

	mcde_ovly_set_dest_pos(ovly, xpos, ypos, 0);
	mcde_ovly_apply(ovly);
	mcde_chnl_update(chnl);
	/* Wait for refresh to be finished */
	mdelay(CONFIG_SYS_MCDE_REFRESH_TIME);
	mcde_exit();
	return 0;
}

/*
 * command line commands
 */


int mcde_power_up(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return mcde_startup();
}

int mcde_disply_bitmap(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return mcde_display_image();
}

U_BOOT_CMD(
	mcde_power_up,	1,	1,	mcde_power_up,
	"Power up display",
	""
);

U_BOOT_CMD(
	mcde_display,	1,	1,	mcde_disply_bitmap,
	"Display bitmap",
	""
);

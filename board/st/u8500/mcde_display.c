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
#include <malloc.h>
#include <asm/arch/common.h>
#include <asm/arch/ab8500.h>
#include <linux/err.h>
#include "mcde.h"
#include "mcde_regs.h"
#include "mcde_display.h"
#include "dsilink_regs.h"

static struct mcde_chnl_state *chnl;

#if CONFIG_SYS_DISPLAY_DSI
static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.ifc = 1,
	.link = 0,
	.sync_src = MCDE_SYNCSRC_BTA,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = 9,
		},
	},
};

struct mcde_display_generic_platform_data main_display_data = {
	.reset_delay = CONFIG_SYS_DISPLAY_RST_DELAY,
};

struct mcde_platform_data platform_data = {
	0,
};

struct mcde_video_mode video_mode = {
	.xres = CONFIG_SYS_DISPLAY_NATIVE_X_RES,
	.yres = CONFIG_SYS_DISPLAY_NATIVE_Y_RES,
	.pixclock = 37037,	/* from kernel */
	.interlaced = 0,
	.bckcol = {255, 255, 255}, /* R, G, B */
};

struct mcde_display_device main_display = {
	.port = &port0,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_C0,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.port_pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.native_x_res = CONFIG_SYS_DISPLAY_NATIVE_X_RES,
	.native_y_res = CONFIG_SYS_DISPLAY_NATIVE_Y_RES,
};
#endif

#if CONFIG_SYS_DISPLAY_DPI
/*
 * STE VUIB card
 */
static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DPI,
	.mode = MCDE_PORTMODE_VID,
	.ifc = 0,
	.link = 1,		/* DPI channel B can only be on link 1 */
	.sync_src = MCDE_SYNCSRC_OFF,   /* sync from output formatter  */
	.update_auto_trig = TRUE,
	.phy = {
		.dpi = {
			.tv_mode = FALSE,
			.clock_div = 2,
			.polarity = DPI_ACT_LOW_VSYNC | DPI_ACT_LOW_HSYNC,
		},
	},
};

struct mcde_display_generic_platform_data main_display_data = {
	0,
};

struct mcde_platform_data platform_data = {
	/* DPI */
	/*
	 * [0] = 3: 24 bits DPI: connect LSB Ch B to D[0:7]
	 * [3] = 4: 24 bits DPI: connect MID Ch B to D[24:31]
	 * [4] = 5: 24 bits DPI: connect MSB Ch B to D[32:39]
	 *
	 * [1] = 3: TV out     : connect LSB Ch B to D[8:15]
	 */
#define DONT_CARE 0
	.outmux = { 3, 3, DONT_CARE, 4, 5 },	/* vuib500 */
#undef DONT_CARE
	.syncmux = 0x01,			/* vuib500 */
};

#define DPI_PIXCLK_FREQ	25000000

struct mcde_video_mode video_mode = {
	.hbp = 40,	/* Horizontal Back Porch */
	.hfp = 8,	/* Horizontal Front Porch */
	.hsw = 96,	/* Horizontal Synchronization pulse Width */
	.vbp1 = 25,	/* Vertical Back Porch 1 */
	.vfp1 = 2,	/* Vertical Front Porch 1 */
	.vbp2 = 0,	/* Vertical Back Porch 2 */
	.vfp2 = 0,	/* Vertical Front Porch 2 */
	.vsw = 2,	/* Vertical Synchronization pulse Width */
	.pixclock = (int)(1e+12 * (1.0 / DPI_PIXCLK_FREQ)),
	.interlaced = 0,
	.bckcol = {255, 255, 255}, /* R, G, B */
};

struct mcde_display_device main_display = {
	.port = &port0,
	.chnl_id = MCDE_CHNL_A,
	.fifo = MCDE_FIFO_A,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.port_pixel_format = MCDE_PORTPIXFMT_DPI_24BPP,
	.native_x_res = CONFIG_SYS_DISPLAY_NATIVE_X_RES,
	.native_y_res = CONFIG_SYS_DISPLAY_NATIVE_Y_RES,
};
#endif

int mcde_turn_on_display(void)
{
	debug("%s: Enter\n", __func__);

	if (main_display.port->type == MCDE_PORTTYPE_DSI)
		return mcde_turn_on_display_dsi();
	else if (main_display.port->type == MCDE_PORTTYPE_DPI)
		return mcde_turn_on_display_dpi();
	return -ENODEV;	/* Should never occur */
}

int mcde_splash_image(void)
{
	int ret = -ENODEV;
	struct mcde_rectangle rect;

	debug("%s: enter\n", __func__);

	if (main_display.port->type == MCDE_PORTTYPE_DSI)
		ret = mcde_startup_dsi(&platform_data);
	else if (main_display.port->type == MCDE_PORTTYPE_DPI)
		ret = mcde_startup_dpi(&platform_data);
	if (ret != 0) {
		printf("%s: mcde_startup... -> %d\n", __func__, ret);
		return ret;
	}

	/* dss enable display */
	chnl = mcde_chnl_get(main_display.chnl_id, main_display.fifo,
							main_display.port);
	if (IS_ERR(chnl)) {
		ret = PTR_ERR(chnl);
		printf("%s: Failed to acquire MCDE channel ret=%d\n",
				__func__, ret);
		goto get_chnl_failed;
	}

	ret = mcde_chnl_set_power_mode(chnl, MCDE_DISPLAY_PM_STANDBY);
	if (ret) {
		printf("%s: mcde_chnl_set_power_mode() -> %d\n",
				__func__, ret);
		goto get_chnl_failed;
	}

	/* dss set video mode */
	ret = mcde_chnl_set_video_mode(chnl, &video_mode);
	if (ret < 0) {
		printf("%s:Failed to set video mode on channel ret=%d\n",
			__func__, ret);
		goto set_video_mode_failed;
	}

	mcde_chnl_set_pixel_format(chnl, main_display.port_pixel_format);
	mcde_chnl_set_update_area(chnl, 0, 0, main_display.native_x_res,
						main_display.native_y_res);
	mcde_chnl_apply(chnl);

	/* chnl setup ok, display image */
	ret = mcde_display_image(chnl);
	if (ret != 0) {
		debug("%s: mcde_display_image() -> %d\n",
			__func__, ret);
	}

	mcde_chnl_apply(chnl);
	rect.x = 0;
	rect.y = 0;
	rect.w = main_display.native_x_res;
	rect.h = main_display.native_y_res;
	mcde_chnl_update(chnl, &rect);

	ret = mcde_turn_on_display();
	if (ret) {
		printf("%s: mcde_turn_on_display() -> %d\n", __func__, ret);
		goto get_chnl_failed;
	}

	ret = mcde_chnl_set_power_mode(chnl, MCDE_DISPLAY_PM_ON);
	if (ret) {
		printf("%s: mcde_chnl_set_power_mode() -> %d\n", __func__, ret);
		goto get_chnl_failed;
	}

	return ret;

get_chnl_failed:
set_video_mode_failed:
	mcde_exit();
	return ret;
}


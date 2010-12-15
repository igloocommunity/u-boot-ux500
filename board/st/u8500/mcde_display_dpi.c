/*
* Copyright (C) ST-Ericsson SA 2010
*
* Author: Torbjorn Svensson <torbjorn.x.svensson@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/


#include <common.h>
#include <command.h>
#include <asm/arch/common.h>
#include "mcde_display.h"
#include "mcde_regs.h"
#include <malloc.h>
#include "mcde.h"
#include <linux/err.h>
#include <asm/arch/ab8500.h>
#include "common.h"

static int mcde_enable_gpio(void)
{
	debug("%s: enter\n", __func__);
	/* placeholder */

	return 0;
}

int mcde_turn_on_display_dpi(void)
{
	debug("Turn on display (dpi)!\n");
	/* placeholder */

	return 0;
}

static int mcde_display_power_init(void)
{
	debug("%s: Entering\n", __func__);
	/* placeholder */

	return 0;
}

int dpi_display_platform_enable(void)
{
	int res = 0;
	debug("%s: Entering\n", __func__);

	/* the gpios used to be enabled here */

	return res;
}

void print_vmode(struct mcde_video_mode *vmode)
{
	if (!vmode) {
		printf("%s: vmode = NULL\n", __func__);
		return;
	}
	debug("resolution: %dx%d\n", vmode->xres, vmode->yres);
	debug("  pixclock: %d\n",    vmode->pixclock);
	debug("       hbp: %d\n",    vmode->hbp);
	debug("       hfp: %d\n",    vmode->hfp);
	debug("       hsw: %d\n",    vmode->hsw);
	debug("       vbp: %d\n",    vmode->vbp1);
	debug("       vfp: %d\n",    vmode->vfp1);
	debug("       vsw: %d\n",    vmode->vsw);
	debug("interlaced: %s\n",    vmode->interlaced ? "true" : "false");
	debug("    bckcol: (%d,%d,%d)\n",
		vmode->bckcol[0], vmode->bckcol[1], vmode->bckcol[2]);
}

int mcde_startup_dpi(struct mcde_platform_data *pdata)
{
	int ret;

	debug("%s: Entering\n", __func__);
	if (main_display.port->mode != MCDE_PORTMODE_VID) {
		printf("%s: Only VIDEO mode supported\n", __func__);
		return -ENODEV;
	}

	mcde_init();
	ret = mcde_probe(1, pdata);
	if (ret != 0) {
		printf("%s: mcde_probe() -> %d\n", __func__, ret);
		return ret;
	}
	ret = mcde_display_power_init();
	if (ret != 0) {
		printf("%s: mcde_display_power_init() -> %d\n", __func__, ret);
		return ret;
	}
	ret = mcde_enable_gpio();
	if (ret != 0) {
		printf("%s: mcde_enable_gpio() -> %d\n", __func__, ret);
		return ret;
	}
	ret =  dpi_display_platform_enable();
	if (ret != 0) {
		printf("%s: dpi_display_platform_enable() -> %d\n",
				__func__, ret);
		return ret;
	}
	print_vmode(&video_mode);

	return 0;
}

/*
* Copyright (C) ST-Ericsson SA 2010
*
* Author: Jimmy Rubin <jimmy.rubin@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/
#ifndef __MCDE_DISPLAY_H
#define __MCDE_DISPLAY_H

#include <common.h>
#include "mcde.h"

extern int cpu_is_u8500v11(void);

struct mcde_display_generic_platform_data {
	/* Platform info */
	int reset_gpio;
	int reset_delay; /* ms */
};

struct mcde_display_device {
	struct mcde_port *port;
	enum mcde_chnl chnl_id;
	enum mcde_fifo fifo;
	enum mcde_ovly_pix_fmt default_pixel_format;
	enum mcde_port_pix_fmt port_pixel_format;
	u16 native_x_res;
	u16 native_y_res;
};

int mcde_startup(void);
int mcde_display_image(void);

#endif /* !defined(__MCDE_UTILS_H) */



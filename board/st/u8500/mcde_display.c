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
#include "gpio.h"
#include "mcde_display.h"
#include "dsilink_regs.h"
#include <tc35892.h>
#include "mcde_regs.h"
#include <malloc.h>
#include "mcde.h"
#include <linux/err.h>
#include <asm/arch/ab8500.h>
#include "common.h"

#ifdef CONFIG_SYS_VIDEO_USE_GIMP_HEADER
#include <asm/arch/mcde_video_logo_gimp.h>
#else
#include <asm/arch/mcde_video_logo.h>
#endif

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
	.reset_delay = 1,
};

struct mcde_display_device main_display = {
	.port = &port0,
	.chnl_id = MCDE_CHNL_C0,
	.fifo = MCDE_FIFO_A,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.port_pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.native_x_res = 864,
	.native_y_res = 480,
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

#define LDO_VAUX1_MASK		0x1
#define LDO_VAUX1_ENABLE	0x1
#define VAUX1_VOLTAGE_2_5V	0x08

#define VANA_ENABLE_IN_HP_MODE	0x05

#define ENABLE_PWM1		0x01
#define PWM_DUTY_LOW_1024_1024	0xFF
#define PWM_DUTY_HI_1024_1024	0x03

static int mcde_display_power_init(void)
{
	int ret;
	int val;

	if (!cpu_is_u8500v11())
		return 0;

	/* Vaux12Regu */
	ret = ab8500_read(AB8500_REGU_CTRL2, AB8500_REGU_VAUX12_REGU_REG);
	if (ret < 0)
		goto out;

	val = ret;

	/* Vaux1 & Vaux2 HP mode */
	ret = ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VAUX12_REGU_REG,
						val | LDO_VAUX1_ENABLE);
	if (ret < 0)
		goto out;

	udelay(10 * 1000);

	/* Set the voltage to 2.5V */
	ret = ab8500_write(AB8500_REGU_CTRL2,
			   AB8500_REGU_VAUX1_SEL_REG, VAUX1_VOLTAGE_2_5V);
	if (ret < 0)
		goto out;

	/*  DCI & CSI (DSI / PLL / Camera) */ /* Vana & Vpll HP mode */
	ab8500_write(AB8500_REGU_CTRL2, AB8500_REGU_VPLLVANA_REGU_REG,
						VANA_ENABLE_IN_HP_MODE);

	/* Enable the PWM control for the backlight Main display */
	ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL7_REG, ENABLE_PWM1);
	ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL1_REG,
						PWM_DUTY_LOW_1024_1024);
	ab8500_write(AB8500_MISC, AB8500_PWM_OUT_CTRL2_REG,
						PWM_DUTY_HI_1024_1024);
out:
	return ret;
}


int mcde_startup(void)
{
	u8 num_dsilinks;
	int ret;
	u32 i;

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
	mcde_ovly_set_source_buf(ovly, (u32)&mcde_video_logo[0]);
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

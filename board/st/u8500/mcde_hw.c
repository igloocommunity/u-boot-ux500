/*
* Copyright (C) ST-Ericsson SA 2010
*
 * ST-Ericsson MCDE driver for u-boot
 *
* Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/

#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <asm/arch/common.h>
#include <asm/arch/cpu.h>
#include <tc35892.h>
#include <malloc.h>

#include <linux/err.h>
#include "dsilink_regs.h"
#include "mcde_regs.h"
#include "mcde.h"
#include <asm/arch/hardware.h>
#define SCREEN_PPL_HIGH 1920

#define false	(0)
#define true	(1)

/* For compatability with kernel code */
#define dev_info(dev, format, arg...)			\
		debug("mcde: " format, ##arg)
#define dev_warn(dev, format, arg...)			\
		debug("mcde: " format, ##arg)
#define dev_dbg(dev, format, arg...)			\
		debug("mcde: " format, ##arg)
#define dev_vdbg(dev, format, arg...)			\
		debug("mcde: " format, ##arg)

#define msleep(m) mdelay(m)

extern void print_vmode(struct mcde_video_mode *vmode);	/* for debugging */

static u8 *mcdeio;
static u8 **dsiio;

static u8 hardware_version;

static inline u32 dsi_rreg(int i, u32 reg)
{
	return readl(dsiio[i] + reg);
}
static inline void dsi_wreg(int i, u32 reg, u32 val)
{
	writel(val, dsiio[i] + reg);
}
#define dsi_rfld(__i, __reg, __fld) \
	((dsi_rreg(__i, __reg) & __reg##_##__fld##_MASK) >> \
		__reg##_##__fld##_SHIFT)
#define dsi_wfld(__i, __reg, __fld, __val) \
	dsi_wreg(__i, __reg, (dsi_rreg(__i, __reg) & \
	~__reg##_##__fld##_MASK) | (((__val) << __reg##_##__fld##_SHIFT) & \
		 __reg##_##__fld##_MASK))

static inline u32 mcde_rreg(u32 reg)
{
	return readl(mcdeio + reg);
}
static inline void mcde_wreg(u32 reg, u32 val)
{
	writel(val, mcdeio + reg);
}
#define mcde_rfld(__reg, __fld) \
	((mcde_rreg(__reg) & __reg##_##__fld##_MASK) >> \
		__reg##_##__fld##_SHIFT)
#define mcde_wfld(__reg, __fld, __val) \
	mcde_wreg(__reg, (mcde_rreg(__reg) & \
	~__reg##_##__fld##_MASK) | (((__val) << __reg##_##__fld##_SHIFT) & \
		 __reg##_##__fld##_MASK))

struct ovly_regs {
	u8   ch_id;
	u8   enabled;
	u32  baseaddress0;
	u32  baseaddress1;
	u8   reset_buf_id;
	u8   bits_per_pixel;
	u8   bpp;
	u8   bgr;
	u8   bebo;
	u8   opq;
	u8   col_conv;
	u8   pixoff;
	u16  ppl;
	u16  lpf;
	u16  cropx;
	u16  cropy;
	u16  xpos;
	u16  ypos;
	u8   z;
};

struct mcde_ovly_state {
	u8 inuse;
	u8 idx; /* MCDE overlay index */
	struct mcde_chnl_state *chnl; /* Owner channel */
	u32 transactionid; /* Apply time stamp */
	u32 transactionid_regs; /* Register update time stamp */

	/* Staged settings */
	u32 paddr;
	u16 stride;
	enum mcde_ovly_pix_fmt pix_fmt;

	u16 src_x;
	u16 src_y;
	u16 dst_x;
	u16 dst_y;
	u16 dst_z;
	u16 w;
	u16 h;

	/* Applied settings */
	struct ovly_regs regs;
};
static struct mcde_ovly_state overlays[] = {
	{ .idx = 0 },
	{ .idx = 1 },
	{ .idx = 2 },
	{ .idx = 3 },
	{ .idx = 4 },
	{ .idx = 5 },
};

struct chnl_regs {
	u8  floen;
	u16 x;
	u16 y;
	u16 ppl;
	u16 lpf;
	u8  bpp;
	u8  internal_clk;
	u16 pcd;
	u8  clksel;
	u8  cdwin;
	u8  bcd;
	u8  synchronized_update;
	u8  roten;

	/* DSI */
	u8  dsipacking;
};

struct col_regs {
	u16 y_red;
	u16 y_green;
	u16 y_blue;
	u16 cb_red;
	u16 cb_green;
	u16 cb_blue;
	u16 cr_red;
	u16 cr_green;
	u16 cr_blue;
	u16 off_red;
	u16 off_green;
	u16 off_blue;
};

struct tv_regs {
	u16 dho; /* TV mode: left border width; destination horizontal offset */
		 /* LCD MODE: horizontal back porch */
	u16 alw; /* TV mode: right border width */
		 /* LCD mode: horizontal front porch */
	u16 hsw; /* horizontal synch width */
	u16 dvo; /* TV mode: top border width; destination horizontal offset */
		 /* LCD MODE: vertical back porch */
	u16 bsl; /* TV mode: bottom border width; blanking start line */
		 /* LCD MODE: vertical front porch */
	/* field 1 */
	u16 bel1; /* TV mode: field total vertical blanking lines */
		 /* LCD mode: vertical sync width */
	u16 fsl1; /* field vbp */
	/* field 2 */
	u16 bel2;
	u16 fsl2;
	u8 tv_mode;
	u8 sel_mode_tv;
	u8 interlaced_en;
	u32 lcdtim1;
};

struct mcde_chnl_state {
	u8 inuse;
	enum mcde_chnl id;
	enum mcde_fifo fifo;
	struct mcde_port port;
	struct mcde_ovly_state *ovly0;
	struct mcde_ovly_state *ovly1;
	const struct chnl_config *cfg;
	u32 transactionid;
	u32 transactionid_regs;

	enum mcde_display_power_mode power_mode;
	/* Staged settings */
	u8 synchronized_update;
	enum mcde_port_pix_fmt pix_fmt;
	struct mcde_video_mode vmode;
	u16 update_x;
	u16 update_y;
	u16 update_w;
	u16 update_h;

	/* Applied settings */
	struct chnl_regs regs;
	struct col_regs  col_regs;
	struct tv_regs   tv_regs;

	u8 continous_running;
};

static struct mcde_chnl_state channels[] = {
	{
		.id = MCDE_CHNL_A,
		.ovly0 = &overlays[0],
		.ovly1 = &overlays[1],
	},
	{
		.id = MCDE_CHNL_B,
		.ovly0 = &overlays[2],
		.ovly1 = &overlays[3],
	},
	{
		.id = MCDE_CHNL_C0,
		.ovly0 = &overlays[4],
		.ovly1 = NULL,
	},
	{
		.id = MCDE_CHNL_C1,
		.ovly0 = &overlays[5],
		.ovly1 = NULL,
	},
};

struct chnl_config {
	/* Key */
	enum mcde_chnl_path path;

	/* Value */
	u8 swap_a_c0;
	u8 swap_a_c0_set;
	u8 swap_b_c1;
	u8 swap_b_c1_set;
	u8 fabmux;
	u8 fabmux_set;
	u8 f01mux;
	u8 f01mux_set;
};

int mcde_chnl_set_video_mode(struct mcde_chnl_state *chnl,
				struct mcde_video_mode *vmode)
{
	if (chnl == NULL || vmode == NULL)
		return -EINVAL;

	chnl->vmode = *vmode;

	return 0;
}

static void dpi_video_mode_apply(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	chnl->tv_regs.interlaced_en = chnl->vmode.interlaced;

	chnl->tv_regs.sel_mode_tv = chnl->port.phy.dpi.tv_mode;
	if (chnl->tv_regs.sel_mode_tv) {
		/* TV mode */
		u32 bel = chnl->vmode.vbp1 + chnl->vmode.vfp1
				+ chnl->vmode.vbp2 + chnl->vmode.vfp2;
		/* -4 since hsw is excluding SAV/EAV, 2 bytes each */
		chnl->tv_regs.hsw  = chnl->vmode.hbp + chnl->vmode.hfp - 4;
		chnl->tv_regs.dho  = MCDE_CONFIG_TVOUT_HBORDER;
		chnl->tv_regs.alw  = MCDE_CONFIG_TVOUT_HBORDER;
		/* in TV mode: bel1 = bel2 + 1 */
		chnl->tv_regs.bel2 = bel / 2;
		chnl->tv_regs.bel1 = bel - chnl->tv_regs.bel2;
		chnl->tv_regs.dvo  = MCDE_CONFIG_TVOUT_VBORDER;
		chnl->tv_regs.bsl  = MCDE_CONFIG_TVOUT_VBORDER;
		chnl->tv_regs.fsl1 = chnl->vmode.vbp1;
		chnl->tv_regs.fsl2 = chnl->vmode.vbp2;
		if (chnl->port.phy.dpi.bus_width == 4)
			chnl->tv_regs.tv_mode = MCDE_TVCRA_TVMODE_SDTV_656P_BE;
		else
			chnl->tv_regs.tv_mode = MCDE_TVCRA_TVMODE_SDTV_656P;
	} else {
		/* LCD mode */
		u32 polarity;
		chnl->tv_regs.hsw  = chnl->vmode.hsw;
		chnl->tv_regs.dho  = chnl->vmode.hbp;
		chnl->tv_regs.alw  = chnl->vmode.hfp;
		chnl->tv_regs.bel1 = chnl->vmode.vsw;
		chnl->tv_regs.bel2 = chnl->tv_regs.bel1;
		chnl->tv_regs.dvo  = chnl->vmode.vbp1 + chnl->vmode.vbp2;
		chnl->tv_regs.bsl  = chnl->vmode.vfp1 + chnl->vmode.vfp2;
		chnl->tv_regs.fsl1 = 0;
		chnl->tv_regs.fsl2 = 0;
		polarity = chnl->port.phy.dpi.polarity;
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IPC(
				(polarity & DPI_ACT_ON_FALLING_EDGE) != 0);
		chnl->tv_regs.lcdtim1 = MCDE_LCDTIM1A_IHS(
				(polarity & DPI_ACT_LOW_HSYNC) != 0);
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IVS(
				(polarity & DPI_ACT_LOW_VSYNC) != 0);
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IOE(
				(polarity & DPI_ACT_LOW_DATA_ENABLE) != 0);
	}
	debug("%s: Leaving\n", __func__);
}

static void update_dpi_registers(enum mcde_chnl chnl_id, struct tv_regs *regs)
{
	u8 idx = chnl_id;

	dev_dbg(&mcde_dev->dev, "%s\n", __func__);
	mcde_wreg(MCDE_TVCRA + idx * MCDE_TVCRA_GROUPOFFSET,
			MCDE_TVCRA_SEL_MOD(regs->sel_mode_tv)             |
			MCDE_TVCRA_INTEREN(regs->interlaced_en)           |
			MCDE_TVCRA_IFIELD(1)                              |
			MCDE_TVCRA_TVMODE(regs->tv_mode)                  |
			MCDE_TVCRA_SDTVMODE(MCDE_TVCRA_SDTVMODE_Y0CBY1CR) |
			MCDE_TVCRA_AVRGEN(0));
	mcde_wreg(MCDE_TVBLUA + idx * MCDE_TVBLUA_GROUPOFFSET,
		MCDE_TVBLUA_TVBLU(MCDE_CONFIG_TVOUT_BACKGROUND_LUMINANCE) |
		MCDE_TVBLUA_TVBCB(MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CB)|
		MCDE_TVBLUA_TVBCR(MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CR));

	/* Vertical timing registers */
	mcde_wreg(MCDE_TVDVOA + idx * MCDE_TVDVOA_GROUPOFFSET,
					MCDE_TVDVOA_DVO1(regs->dvo) |
					MCDE_TVDVOA_DVO2(regs->dvo));
	mcde_wreg(MCDE_TVBL1A + idx * MCDE_TVBL1A_GROUPOFFSET,
					MCDE_TVBL1A_BEL1(regs->bel1) |
					MCDE_TVBL1A_BSL1(regs->bsl));
	mcde_wreg(MCDE_TVBL2A + idx * MCDE_TVBL1A_GROUPOFFSET,
					MCDE_TVBL2A_BEL2(regs->bel2) |
					MCDE_TVBL2A_BSL2(regs->bsl));
	mcde_wreg(MCDE_TVISLA + idx * MCDE_TVISLA_GROUPOFFSET,
					MCDE_TVISLA_FSL1(regs->fsl1) |
					MCDE_TVISLA_FSL2(regs->fsl2));

	/* Horizontal timing registers */
	if (!regs->sel_mode_tv ||
			hardware_version == MCDE_CHIP_VERSION_3_0_8) {
		mcde_wreg(MCDE_TVLBALWA + idx * MCDE_TVLBALWA_GROUPOFFSET,
					MCDE_TVLBALWA_LBW(regs->hsw) |
					MCDE_TVLBALWA_ALW(regs->alw));
		mcde_wreg(MCDE_TVTIM1A + idx * MCDE_TVTIM1A_GROUPOFFSET,
					MCDE_TVTIM1A_DHO(regs->dho));
	} else {
		/*
		 * in earlier versions the LBW and DHO fields are swapped
		 * TV mode only
		 */
		mcde_wreg(MCDE_TVLBALWA + idx * MCDE_TVLBALWA_GROUPOFFSET,
					MCDE_TVLBALWA_LBW(regs->dho) |
					MCDE_TVLBALWA_ALW(regs->alw));
		mcde_wreg(MCDE_TVTIM1A + idx * MCDE_TVTIM1A_GROUPOFFSET,
					MCDE_TVTIM1A_DHO(regs->hsw));
	}
	if (!regs->sel_mode_tv)
		mcde_wreg(MCDE_LCDTIM1A + idx * MCDE_LCDTIM1A_GROUPOFFSET,
								regs->lcdtim1);
	debug("%s: Leaving\n", __func__);
}

static void update_col_registers(enum mcde_chnl chnl_id, struct col_regs *regs)
{
	u8 idx = chnl_id;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	mcde_wreg(MCDE_RGBCONV1A + idx * MCDE_RGBCONV1A_GROUPOFFSET,
				MCDE_RGBCONV1A_YR_RED(regs->y_red) |
				MCDE_RGBCONV1A_YR_GREEN(regs->y_green));
	mcde_wreg(MCDE_RGBCONV2A + idx * MCDE_RGBCONV2A_GROUPOFFSET,
				MCDE_RGBCONV2A_YR_BLUE(regs->y_blue) |
				MCDE_RGBCONV2A_CR_RED(regs->cr_red));
	mcde_wreg(MCDE_RGBCONV3A + idx * MCDE_RGBCONV3A_GROUPOFFSET,
				MCDE_RGBCONV3A_CR_GREEN(regs->cr_green) |
				MCDE_RGBCONV3A_CR_BLUE(regs->cr_blue));
	mcde_wreg(MCDE_RGBCONV4A + idx * MCDE_RGBCONV4A_GROUPOFFSET,
				MCDE_RGBCONV4A_CB_RED(regs->cb_red) |
				MCDE_RGBCONV4A_CB_GREEN(regs->cb_green));
	mcde_wreg(MCDE_RGBCONV5A + idx * MCDE_RGBCONV5A_GROUPOFFSET,
				MCDE_RGBCONV5A_CB_BLUE(regs->cb_blue) |
				MCDE_RGBCONV5A_OFF_RED(regs->off_red));
	mcde_wreg(MCDE_RGBCONV6A + idx * MCDE_RGBCONV6A_GROUPOFFSET,
				MCDE_RGBCONV6A_OFF_GREEN(regs->off_green) |
				MCDE_RGBCONV6A_OFF_BLUE(regs->off_blue));
}

/* MCDE internal helpers */
static u8 portfmt2dsipacking(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DSI_16BPP:
		return MCDE_DSIVID0CONF0_PACKING_RGB565;
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		return MCDE_DSIVID0CONF0_PACKING_RGB666;
	case MCDE_PORTPIXFMT_DSI_18BPP:
	case MCDE_PORTPIXFMT_DSI_24BPP:
	default:
		return MCDE_DSIVID0CONF0_PACKING_RGB888;
	case MCDE_PORTPIXFMT_DSI_YCBCR422:
		return MCDE_DSIVID0CONF0_PACKING_HDTV;
	}
}

static u8 portfmt2bpp(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DPI_16BPP_C1:
	case MCDE_PORTPIXFMT_DPI_16BPP_C2:
	case MCDE_PORTPIXFMT_DPI_16BPP_C3:
	case MCDE_PORTPIXFMT_DSI_16BPP:
	case MCDE_PORTPIXFMT_DSI_YCBCR422:
		return 16;
	case MCDE_PORTPIXFMT_DPI_18BPP_C1:
	case MCDE_PORTPIXFMT_DPI_18BPP_C2:
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		return 18;
	case MCDE_PORTPIXFMT_DSI_18BPP:
	case MCDE_PORTPIXFMT_DPI_24BPP:
	case MCDE_PORTPIXFMT_DSI_24BPP:
		return 24;
	default:
		return 1;
	}
}

static u8 bpp2outbpp(u8 bpp)
{
	switch (bpp) {
	case 16:
		return MCDE_CRA1_OUTBPP_16BPP;
	case 18:
		return MCDE_CRA1_OUTBPP_18BPP;
	case 24:
		return MCDE_CRA1_OUTBPP_24BPP;
	default:
		return 0;
	}
}

static u8 portfmt2cdwin(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DPI_16BPP_C1:
		return MCDE_CRA1_CDWIN_16BBP_C1;
	case MCDE_PORTPIXFMT_DPI_16BPP_C2:
		return MCDE_CRA1_CDWIN_16BBP_C2;
	case MCDE_PORTPIXFMT_DPI_18BPP_C1:
		return MCDE_CRA1_CDWIN_18BBP_C1;
	case MCDE_PORTPIXFMT_DPI_18BPP_C2:
		return MCDE_CRA1_CDWIN_18BBP_C2;
	case MCDE_PORTPIXFMT_DPI_24BPP:
		return MCDE_CRA1_CDWIN_24BBP;

	case MCDE_PORTPIXFMT_DPI_16BPP_C3:
		/* 16 BPP C3 not supported by HW */
		dev_warn(&mcde_dev->dev, "DPI 16 BPP C3 not supported\n");
		/* intentional fall through */
	default:
		/* only DPI formats are relevant */
		return 0;
	}
}

static u32 get_output_fifo_size(enum mcde_fifo fifo)
{
	u32 ret = 1; /* Avoid div by zero */

	switch (fifo) {
	case MCDE_FIFO_A:
	case MCDE_FIFO_B:
		ret = MCDE_FIFO_AB_SIZE;
		break;
	case MCDE_FIFO_C0:
	case MCDE_FIFO_C1:
		ret = MCDE_FIFO_C0C1_SIZE;
		break;
	default:
		dev_vdbg(&mcde_dev->dev, "Unsupported fifo");
		break;
	}
	return ret;
}

static u8 get_dsi_formid(const struct mcde_port *port)
{
	if (port->ifc == DSI_VIDEO_MODE && port->link == 0)
		return MCDE_CTRLA_FORMID_DSI0VID;
	else if (port->ifc == DSI_VIDEO_MODE && port->link == 1)
		return MCDE_CTRLA_FORMID_DSI1VID;
	else if (port->ifc == DSI_VIDEO_MODE && port->link == 2)
		return MCDE_CTRLA_FORMID_DSI2VID;
	else if (port->ifc == DSI_CMD_MODE && port->link == 0)
		return MCDE_CTRLA_FORMID_DSI0CMD;
	else if (port->ifc == DSI_CMD_MODE && port->link == 1)
		return MCDE_CTRLA_FORMID_DSI1CMD;
	else if (port->ifc == DSI_CMD_MODE && port->link == 2)
		return MCDE_CTRLA_FORMID_DSI2CMD;
	return 0;
}


void wait_for_overlay(struct mcde_ovly_state *ovly)
{
	/* ovly not used in u-boot */
}

void wait_for_channel(struct mcde_chnl_state *chnl)
{
	/* ovly not used in u-boot */
}

static int update_channel_static_registers(struct mcde_chnl_state *chnl)
{
	const struct chnl_config *cfg = chnl->cfg;
	const struct mcde_port *port = &chnl->port;

	if (hardware_version == MCDE_CHIP_VERSION_3_0_5) {
		/* Fifo & muxing */
		if (cfg->swap_a_c0_set)
			mcde_wfld(MCDE_CONF0, SWAP_A_C0_V1, cfg->swap_a_c0);
		if (cfg->swap_b_c1_set)
			mcde_wfld(MCDE_CONF0, SWAP_B_C1_V1, cfg->swap_b_c1);
		if (cfg->fabmux_set)
			mcde_wfld(MCDE_CR, FABMUX_V1, cfg->fabmux);
		if (cfg->f01mux_set)
			mcde_wfld(MCDE_CR, F01MUX_V1, cfg->f01mux);

		if (port->type == MCDE_PORTTYPE_DPI) {
			if (port->link == 0)
				mcde_wfld(MCDE_CR, DPIA_EN_V1, true);
			else if (port->link == 1)
				mcde_wfld(MCDE_CR, DPIB_EN_V1, true);
		} else if (port->type == MCDE_PORTTYPE_DSI) {
			if (port->ifc == DSI_VIDEO_MODE && port->link == 0)
				mcde_wfld(MCDE_CR, DSIVID0_EN_V1, true);
			else if (port->ifc == DSI_VIDEO_MODE && port->link == 1)
				mcde_wfld(MCDE_CR, DSIVID1_EN_V1, true);
			else if (port->ifc == DSI_VIDEO_MODE && port->link == 2)
				mcde_wfld(MCDE_CR, DSIVID2_EN_V1, true);
			else if (port->ifc == DSI_CMD_MODE && port->link == 0)
				mcde_wfld(MCDE_CR, DSICMD0_EN_V1, true);
			else if (port->ifc == DSI_CMD_MODE && port->link == 1)
				mcde_wfld(MCDE_CR, DSICMD1_EN_V1, true);
			else if (port->ifc == DSI_CMD_MODE && port->link == 2)
				mcde_wfld(MCDE_CR, DSICMD2_EN_V1, true);
		}

		if (chnl->fifo == MCDE_FIFO_C0)
			mcde_wreg(MCDE_CTRLC0, MCDE_CTRLC0_FIFOWTRMRK(
					get_output_fifo_size(MCDE_FIFO_C0)));
		else if (chnl->fifo == MCDE_FIFO_C1)
			mcde_wreg(MCDE_CTRLC1, MCDE_CTRLC1_FIFOWTRMRK(
					get_output_fifo_size(MCDE_FIFO_C1)));
		else if (port->update_auto_trig &&
					(port->sync_src == MCDE_SYNCSRC_TE0))
			mcde_wreg(MCDE_CTRLC0, MCDE_CTRLC0_FIFOWTRMRK(
					get_output_fifo_size(MCDE_FIFO_C0)));
		else if (port->update_auto_trig &&
					(port->sync_src == MCDE_SYNCSRC_TE1))
			mcde_wreg(MCDE_CTRLC1, MCDE_CTRLC1_FIFOWTRMRK(
					get_output_fifo_size(MCDE_FIFO_C1)));
	} else {
		switch (chnl->fifo) {
		case MCDE_FIFO_A:
			mcde_wreg(MCDE_CHNL0MUXING_V2 + chnl->id *
				MCDE_CHNL0MUXING_V2_GROUPOFFSET,
				MCDE_CHNL0MUXING_V2_FIFO_ID_ENUM(FIFO_A));
			if (port->type == MCDE_PORTTYPE_DPI) {
				mcde_wfld(MCDE_CTRLA, FORMTYPE,
						MCDE_CTRLA_FORMTYPE_DPITV);
				mcde_wfld(MCDE_CTRLA, FORMID, port->link);
				mcde_wfld(MCDE_CTRLA, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_A));
			} else if (port->type == MCDE_PORTTYPE_DSI) {
				mcde_wfld(MCDE_CTRLA, FORMTYPE,
						MCDE_CTRLA_FORMTYPE_DSI);
				mcde_wfld(MCDE_CTRLA, FORMID,
							get_dsi_formid(port));
				mcde_wfld(MCDE_CTRLA, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_A));
			}
			break;
		case MCDE_FIFO_B:
			mcde_wreg(MCDE_CHNL0MUXING_V2 + chnl->id *
				MCDE_CHNL0MUXING_V2_GROUPOFFSET,
				MCDE_CHNL0MUXING_V2_FIFO_ID_ENUM(FIFO_B));
			if (port->type == MCDE_PORTTYPE_DPI) {
				mcde_wfld(MCDE_CTRLB, FORMTYPE,
						MCDE_CTRLB_FORMTYPE_DPITV);
				mcde_wfld(MCDE_CTRLB, FORMID, port->link);
				mcde_wfld(MCDE_CTRLB, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_B));
			} else if (port->type == MCDE_PORTTYPE_DSI) {
				mcde_wfld(MCDE_CTRLB, FORMTYPE,
						MCDE_CTRLB_FORMTYPE_DSI);
				mcde_wfld(MCDE_CTRLB, FORMID,
							get_dsi_formid(port));
				mcde_wfld(MCDE_CTRLB, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_B));
			}

			break;
		case MCDE_FIFO_C0:
			mcde_wreg(MCDE_CHNL0MUXING_V2 + chnl->id *
				MCDE_CHNL0MUXING_V2_GROUPOFFSET,
				MCDE_CHNL0MUXING_V2_FIFO_ID_ENUM(FIFO_C0));
			if (port->type == MCDE_PORTTYPE_DPI)
				return -EINVAL;
			mcde_wfld(MCDE_CTRLC0, FORMTYPE,
						MCDE_CTRLC0_FORMTYPE_DSI);
			mcde_wfld(MCDE_CTRLC0, FORMID, get_dsi_formid(port));
			mcde_wfld(MCDE_CTRLC0, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_C0));
			break;
		case MCDE_FIFO_C1:
			mcde_wreg(MCDE_CHNL0MUXING_V2 + chnl->id *
				MCDE_CHNL0MUXING_V2_GROUPOFFSET,
				MCDE_CHNL0MUXING_V2_FIFO_ID_ENUM(FIFO_C1));
			if (port->type == MCDE_PORTTYPE_DPI)
				return -EINVAL;
			mcde_wfld(MCDE_CTRLC1, FORMTYPE,
						MCDE_CTRLC1_FORMTYPE_DSI);
			mcde_wfld(MCDE_CTRLC1, FORMID, get_dsi_formid(port));
			mcde_wfld(MCDE_CTRLC1, FIFOWTRMRK,
					get_output_fifo_size(MCDE_FIFO_C1));
			break;
		default:
			return -EINVAL;
		}
	}

	/* Formatter */
	if (port->type == MCDE_PORTTYPE_DSI) {
		int i = 0;
		u8 idx = 2 * port->link + port->ifc;
		u8 lnk = port->link;

		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, LINK_EN, true);
		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, true);
		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, READ_EN, true);
		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, REG_TE_EN, true);

		if (hardware_version == MCDE_CHIP_VERSION_3_0_5) {
			if (port->phy.dsi.data_lanes_swap) {
				dev_warn(&mcde_dev->dev,
				"DSI %d data lane remap not available!\n",
				lnk);
				return -EINVAL;
			}
		} else
			dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, DLX_REMAP_EN,
					port->phy.dsi.data_lanes_swap);

		dsi_wreg(lnk, DSI_MCTL_DPHY_STATIC,
			DSI_MCTL_DPHY_STATIC_UI_X4(port->phy.dsi.ui));
		dsi_wreg(lnk, DSI_DPHY_LANES_TRIM,
			DSI_DPHY_LANES_TRIM_DPHY_SPECS_90_81B_ENUM(0_90));
		dsi_wreg(lnk, DSI_MCTL_DPHY_TIMEOUT,
			DSI_MCTL_DPHY_TIMEOUT_CLK_DIV(0xf) |
			DSI_MCTL_DPHY_TIMEOUT_HSTX_TO_VAL(0x3fff) |
			DSI_MCTL_DPHY_TIMEOUT_LPRX_TO_VAL(0x3fff));
		dsi_wreg(lnk, DSI_MCTL_MAIN_PHY_CTL,
			DSI_MCTL_MAIN_PHY_CTL_WAIT_BURST_TIME(0xf) |
			DSI_MCTL_MAIN_PHY_CTL_LANE2_EN(true) |
			DSI_MCTL_MAIN_PHY_CTL_CLK_CONTINUOUS(
				port->phy.dsi.clk_cont));
		dsi_wreg(lnk, DSI_MCTL_ULPOUT_TIME,
			DSI_MCTL_ULPOUT_TIME_CKLANE_ULPOUT_TIME(1) |
			DSI_MCTL_ULPOUT_TIME_DATA_ULPOUT_TIME(1));
		dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_MODE, false);
		dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_PRI, port->ifc == 1);
		dsi_wreg(lnk, DSI_MCTL_MAIN_EN,
			DSI_MCTL_MAIN_EN_PLL_START(true) |
			DSI_MCTL_MAIN_EN_CKLANE_EN(true) |
			DSI_MCTL_MAIN_EN_DAT1_EN(true) |
			DSI_MCTL_MAIN_EN_DAT2_EN(port->phy.dsi.num_data_lanes
				== 2) |
			DSI_MCTL_MAIN_EN_IF1_EN(port->ifc == DSI_VIDEO_MODE) |
			DSI_MCTL_MAIN_EN_IF2_EN(port->ifc == DSI_CMD_MODE));
		while (dsi_rfld(lnk, DSI_MCTL_MAIN_STS, CLKLANE_READY) == 0 ||
		       dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT1_READY) == 0 ||
		       dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT2_READY) == 0) {
			mdelay(1);
			if (i++ == 10) {
				dev_warn(&mcde_dev->dev,
					"DSI lane not ready (link=%d)!\n", lnk);
				return -EINVAL;
			}
		}

		mcde_wreg(MCDE_DSIVID0CONF0 +
			idx * MCDE_DSIVID0CONF0_GROUPOFFSET,
			MCDE_DSIVID0CONF0_BLANKING(0) |
			MCDE_DSIVID0CONF0_VID_MODE(
				port->mode == MCDE_PORTMODE_VID) |
			MCDE_DSIVID0CONF0_CMD8(true) |
			MCDE_DSIVID0CONF0_BIT_SWAP(false) |
			MCDE_DSIVID0CONF0_BYTE_SWAP(false) |
			MCDE_DSIVID0CONF0_DCSVID_NOTGEN(true));

		if (port->mode == MCDE_PORTMODE_CMD) {
			if (port->ifc == DSI_VIDEO_MODE)
				dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF1_ID,
					port->phy.dsi.virt_id);
			else if (port->ifc == DSI_CMD_MODE)
				dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF2_ID,
					port->phy.dsi.virt_id);
		}
	}

	mcde_wfld(MCDE_CR, MCDEEN, true);

	dev_vdbg(&mcde_dev->dev, "Static registers setup, chnl=%d\n", chnl->id);

	return 0;
}

static void update_overlay_registers(u8 idx, struct ovly_regs *regs,
			struct mcde_port *port, enum mcde_fifo fifo,
			u16 update_x, u16 update_y, u16 update_w,
			u16 update_h, u16 stride, u8 interlaced)
{
	u32 lmrgn = (regs->cropx + update_x) * regs->bits_per_pixel;
	u32 tmrgn = (regs->cropy + update_y) * stride;
	u32 ppl = regs->ppl - update_x;
	u32 lpf = regs->lpf - update_y;
	u32 ljinc = stride;
	u32 pixelfetchwtrmrklevel;
	u8  nr_of_bufs = 1;
	u32 fifo_size;

	/* TODO: disable if everything clipped */
	if (!regs->enabled) {
		u32 temp;
		temp = mcde_rreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET);
		mcde_wreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET,
			(temp & ~MCDE_OVL0CR_OVLEN_MASK) |
			MCDE_OVL0CR_OVLEN(false));
		return;
	}

	/*
	* TODO: Preferably most of this is done in some apply function instead
	* of every update. Problem is however that at overlay apply
	* there is no port type info available (and the question is
	* whether it is appropriate to add a port type there).
	* Note that lpf has a dependency on update_y.
	*/
	if (port->type == MCDE_PORTTYPE_DPI && port->phy.dpi.tv_mode)
		/* REVIEW: Why not for DSI? enable in regs? */
		regs->col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_NO_SAT;
	else if (port->type == MCDE_PORTTYPE_DSI) {
		if (port->pixel_format == MCDE_PORTPIXFMT_DSI_YCBCR422)
			regs->col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_NO_SAT;
		else
			regs->col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
		if (interlaced) {
			nr_of_bufs = 2;
			lpf = lpf / 2;
			ljinc *= 2;
		}
	}

	fifo_size = get_output_fifo_size(fifo);
#ifdef CONFIG_AV8100_SDTV
	/* TODO: check if these watermark levels work for HDMI as well. */
	pixelfetchwtrmrklevel = MCDE_PIXFETCH_SMALL_WTRMRKLVL;
#else
	if ((fifo == MCDE_FIFO_A || fifo == MCDE_FIFO_B) &&
					regs->ppl >= fifo_size * 2)
		pixelfetchwtrmrklevel = MCDE_PIXFETCH_LARGE_WTRMRKLVL;
	else
		pixelfetchwtrmrklevel = MCDE_PIXFETCH_MEDIUM_WTRMRKLVL;
#endif /* CONFIG_AV8100_SDTV */

	if (regs->reset_buf_id) {
		u32 sel_mod = MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;
		if (port->update_auto_trig && port->type == MCDE_PORTTYPE_DSI) {
			switch (port->sync_src) {
			case MCDE_SYNCSRC_OFF:
				sel_mod = MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;
				break;
			case MCDE_SYNCSRC_TE0:
			case MCDE_SYNCSRC_TE1:
			default:
				sel_mod = MCDE_EXTSRC0CR_SEL_MOD_AUTO_TOGGLE;
			}
		} else if (port->type == MCDE_PORTTYPE_DPI) {
			sel_mod = port->update_auto_trig ?
					MCDE_EXTSRC0CR_SEL_MOD_AUTO_TOGGLE :
					MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;
		}

		regs->reset_buf_id = false;
	mcde_wreg(MCDE_EXTSRC0CONF + idx * MCDE_EXTSRC0CONF_GROUPOFFSET,
		MCDE_EXTSRC0CONF_BUF_ID(0) |
			MCDE_EXTSRC0CONF_BUF_NB(nr_of_bufs) |
		MCDE_EXTSRC0CONF_PRI_OVLID(idx) |
		MCDE_EXTSRC0CONF_BPP(regs->bpp) |
		MCDE_EXTSRC0CONF_BGR(regs->bgr) |
		MCDE_EXTSRC0CONF_BEBO(regs->bebo) |
		MCDE_EXTSRC0CONF_BEPO(false));
	mcde_wreg(MCDE_EXTSRC0CR + idx * MCDE_EXTSRC0CR_GROUPOFFSET,
			MCDE_EXTSRC0CR_SEL_MOD(sel_mod) |
		MCDE_EXTSRC0CR_MULTIOVL_CTRL_ENUM(PRIMARY) |
		MCDE_EXTSRC0CR_FS_DIV_DISABLE(false) |
		MCDE_EXTSRC0CR_FORCE_FS_DIV(false));
	mcde_wreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET,
		MCDE_OVL0CR_OVLEN(true) |
		MCDE_OVL0CR_COLCCTRL(regs->col_conv) |
		MCDE_OVL0CR_CKEYGEN(false) |
		MCDE_OVL0CR_ALPHAPMEN(true) |
		MCDE_OVL0CR_OVLF(false) |
		MCDE_OVL0CR_OVLR(false) |
		MCDE_OVL0CR_OVLB(false) |
		MCDE_OVL0CR_FETCH_ROPC(0) |
		MCDE_OVL0CR_STBPRIO(0) |
		MCDE_OVL0CR_BURSTSIZE_ENUM(HW_8W) |
		MCDE_OVL0CR_MAXOUTSTANDING_ENUM(4_REQ) |
		MCDE_OVL0CR_ROTBURSTSIZE_ENUM(HW_8W));
	mcde_wreg(MCDE_OVL0CONF + idx * MCDE_OVL0CONF_GROUPOFFSET,
		MCDE_OVL0CONF_PPL(ppl) |
		MCDE_OVL0CONF_EXTSRC_ID(idx) |
		MCDE_OVL0CONF_LPF(lpf));
	mcde_wreg(MCDE_OVL0CONF2 + idx * MCDE_OVL0CONF2_GROUPOFFSET,
		MCDE_OVL0CONF2_BP_ENUM(PER_PIXEL_ALPHA) |
			MCDE_OVL0CONF2_ALPHAVALUE(0x80) |
		MCDE_OVL0CONF2_OPQ(regs->opq) |
		MCDE_OVL0CONF2_PIXOFF(lmrgn & 63) |
		MCDE_OVL0CONF2_PIXELFETCHERWATERMARKLEVEL(32));
	mcde_wreg(MCDE_OVL0LJINC + idx * MCDE_OVL0LJINC_GROUPOFFSET,
			ljinc);
	mcde_wreg(MCDE_OVL0CROP + idx * MCDE_OVL0CROP_GROUPOFFSET,
		MCDE_OVL0CROP_TMRGN(tmrgn) |
		MCDE_OVL0CROP_LMRGN(lmrgn >> 6));
	mcde_wreg(MCDE_OVL0COMP + idx * MCDE_OVL0COMP_GROUPOFFSET,
		MCDE_OVL0COMP_XPOS(regs->xpos) |
		MCDE_OVL0COMP_CH_ID(regs->ch_id) |
		MCDE_OVL0COMP_YPOS(regs->ypos) |
		MCDE_OVL0COMP_Z(regs->z));
	}
	dev_vdbg(&mcde_dev->dev, "Overlay registers setup, idx=%d\n", idx);
}

static void update_overlay_address_registers(u8 idx, struct ovly_regs *regs)
{
	mcde_wreg(MCDE_EXTSRC0A0 + idx * MCDE_EXTSRC0A0_GROUPOFFSET,
		regs->baseaddress0);
	mcde_wreg(MCDE_EXTSRC0A1 + idx * MCDE_EXTSRC0A1_GROUPOFFSET,
		regs->baseaddress1);
}

static void enable_channel(struct mcde_chnl_state *chnl)
{
	const struct mcde_port *port = &chnl->port;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (port->type == MCDE_PORTTYPE_DSI)
		dsi_wfld(port->link, DSI_MCTL_MAIN_PHY_CTL, CLK_CONTINUOUS,
				port->phy.dsi.clk_cont);

	switch (chnl->id) {
	case MCDE_CHNL_A:
		mcde_wfld(MCDE_CRA0, FLOEN, true);
		break;
	case MCDE_CHNL_B:
		mcde_wfld(MCDE_CRB0, FLOEN, true);
		break;
	case MCDE_CHNL_C0:
		mcde_wfld(MCDE_CRC, POWEREN, true);
		mcde_wfld(MCDE_CRC, FLOEN, true);
		mcde_wfld(MCDE_CRC, C1EN, true);
		break;
	case MCDE_CHNL_C1:
		mcde_wfld(MCDE_CRC, POWEREN, true);
		mcde_wfld(MCDE_CRC, FLOEN, true);
		mcde_wfld(MCDE_CRC, C2EN, true);
		break;
	}
}

/* TODO get from register */
#define MCDE_CLK_FREQ_MHZ 160

void update_channel_registers(enum mcde_chnl chnl_id, struct chnl_regs *regs,
				struct mcde_port *port, enum mcde_fifo fifo,
				struct mcde_video_mode *video_mode)
{
	u8 idx = chnl_id;
	u32 out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
	u32 src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	print_vmode(video_mode);

	/* Channel */
	if (port->update_auto_trig && port->type == MCDE_PORTTYPE_DSI) {
		switch (port->sync_src) {
		case MCDE_SYNCSRC_TE0:
			out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_VSYNC0;
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_OUTPUT;
			break;
		case MCDE_SYNCSRC_OFF:
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE;
			break;
		case MCDE_SYNCSRC_TE1:
		default:
			out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_VSYNC1;
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_OUTPUT;
		}
	} else if (port->type == MCDE_PORTTYPE_DPI) {
		src_synch = port->update_auto_trig ?
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_OUTPUT :
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE;
	}

	mcde_wreg(MCDE_CHNL0CONF + idx * MCDE_CHNL0CONF_GROUPOFFSET,
		MCDE_CHNL0CONF_PPL(regs->ppl-1) |
		MCDE_CHNL0CONF_LPF(regs->lpf-1));
	mcde_wreg(MCDE_CHNL0STAT + idx * MCDE_CHNL0STAT_GROUPOFFSET,
		MCDE_CHNL0STAT_CHNLBLBCKGND_EN(true) |
		MCDE_CHNL0STAT_CHNLRD(true));
	mcde_wreg(MCDE_CHNL0SYNCHMOD +
		idx * MCDE_CHNL0SYNCHMOD_GROUPOFFSET,
		MCDE_CHNL0SYNCHMOD_SRC_SYNCH(src_synch) |
		MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC(out_synch_src));
	mcde_wreg(MCDE_CHNL0BCKGNDCOL + idx * MCDE_CHNL0BCKGNDCOL_GROUPOFFSET,
		MCDE_CHNL0BCKGNDCOL_R(video_mode->bckcol[0]) |
		MCDE_CHNL0BCKGNDCOL_G(video_mode->bckcol[1]) |
		MCDE_CHNL0BCKGNDCOL_B(video_mode->bckcol[2]));

	switch (chnl_id) {
	case MCDE_CHNL_A:
		mcde_wreg(MCDE_CRA1,
			MCDE_CRA1_PCD(regs->pcd) |
			MCDE_CRA1_CLKSEL(regs->clksel) |
			MCDE_CRA1_CDWIN(regs->cdwin) |
			MCDE_CRA1_OUTBPP(bpp2outbpp(regs->bpp)) |
			MCDE_CRA1_BCD(regs->bcd) |
			MCDE_CRA1_CLKTYPE(regs->internal_clk)
		);
		break;
	case MCDE_CHNL_B:
		mcde_wreg(MCDE_CRB1,
			MCDE_CRB1_PCD(regs->pcd) |
			MCDE_CRB1_CLKSEL(regs->clksel) |
			MCDE_CRB1_CDWIN(regs->cdwin) |
			MCDE_CRB1_OUTBPP(bpp2outbpp(regs->bpp)) |
			MCDE_CRB1_BCD(regs->bcd) |
			MCDE_CRB1_CLKTYPE(regs->internal_clk)
		);
		break;
	default:
		break;
	}

	/* Formatter */
	if (port->type == MCDE_PORTTYPE_DSI) {
		u8 fidx = 2 * port->link + port->ifc;
		u32 temp, packet;
		/*
		 * pkt_div is used to avoid underflow in output fifo for
		 * large packets
		 */
		u32 pkt_div = 1;
		u32 dsi_delay0 = 0;
		u32 screen_ppl, screen_lpf;

		screen_ppl = video_mode->xres;
		screen_lpf = video_mode->yres;

		if (screen_ppl == SCREEN_PPL_HIGH) {
			pkt_div = (screen_ppl - 1) /
					get_output_fifo_size(fifo) + 1;
		} else {
			pkt_div = screen_ppl /
					(get_output_fifo_size(fifo) * 2) + 1;
		}

		if (video_mode->interlaced)
			screen_lpf /= 2;

		/* pkt_delay_progressive = pixelclock * htot /
		 * (1E12 / 160E6) / pkt_div */
		dsi_delay0 = (video_mode->pixclock + 1) *
			(video_mode->xres + video_mode->hbp +
				video_mode->hfp) /
			(1000000 / MCDE_CLK_FREQ_MHZ) / pkt_div;
		temp = mcde_rreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET);
		mcde_wreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET,
			(temp & ~MCDE_DSIVID0CONF0_PACKING_MASK) |
			MCDE_DSIVID0CONF0_PACKING(regs->dsipacking));
		/* 1==CMD8 */
		packet = ((screen_ppl / pkt_div * regs->bpp) >> 3) + 1;
		mcde_wreg(MCDE_DSIVID0FRAME +
			fidx * MCDE_DSIVID0FRAME_GROUPOFFSET,
			MCDE_DSIVID0FRAME_FRAME(packet * pkt_div * screen_lpf));
		mcde_wreg(MCDE_DSIVID0PKT + fidx * MCDE_DSIVID0PKT_GROUPOFFSET,
			MCDE_DSIVID0PKT_PACKET(packet));
		mcde_wreg(MCDE_DSIVID0SYNC +
			fidx * MCDE_DSIVID0SYNC_GROUPOFFSET,
			MCDE_DSIVID0SYNC_SW(0) |
			MCDE_DSIVID0SYNC_DMA(0));
		mcde_wreg(MCDE_DSIVID0CMDW +
			fidx * MCDE_DSIVID0CMDW_GROUPOFFSET,
			MCDE_DSIVID0CMDW_CMDW_START(DCS_CMD_WRITE_START) |
			MCDE_DSIVID0CMDW_CMDW_CONTINUE(DCS_CMD_WRITE_CONTINUE));
		mcde_wreg(MCDE_DSIVID0DELAY0 +
			fidx * MCDE_DSIVID0DELAY0_GROUPOFFSET,
			MCDE_DSIVID0DELAY0_INTPKTDEL(dsi_delay0));
		mcde_wreg(MCDE_DSIVID0DELAY1 +
			fidx * MCDE_DSIVID0DELAY1_GROUPOFFSET,
			MCDE_DSIVID0DELAY1_TEREQDEL(0) |
			MCDE_DSIVID0DELAY1_FRAMESTARTDEL(0));
	}
	debug("Channel registers setup, chnl=%d\n", chnl_id);
}

void mcde_chnl_set_update_area(struct mcde_chnl_state *chnl,
	u16 x, u16 y, u16 w, u16 h)
{
	if (!chnl->inuse) {
		debug("%s: channel in use chnl=%p\n", __func__, chnl);
		return;
	}

	chnl->update_x = x;
	chnl->update_y = y;
	chnl->update_w = w;
	chnl->update_h = h;
}

/* DSI */

int mcde_dsi_dcs_write(struct mcde_port *port, u8 cmd, u8* data, int len)
{
	int i;
	u32 wrdat[4] = { 0, 0, 0, 0 };
	u32 settings;
	u8 link = port->link;
	u8 virt_id = port->phy.dsi.virt_id;

	debug("Entering %s\n", __func__);
	if (len > MCDE_MAX_DCS_WRITE)
		return -EINVAL;

	wrdat[0] = cmd;
	for (i = 1; i <= len; i++)
		wrdat[i>>2] |= ((u32)data[i-1] << ((i & 3) * 8));

	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(WRITE) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(len > 1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(len+1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(true);
	if (len == 0)
		settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
			DCS_SHORT_WRITE_0);
	else if (len == 1)
		settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
			DCS_SHORT_WRITE_1);
	else
		settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
			DCS_LONG_WRITE);

	dsi_wreg(link, DSI_DIRECT_CMD_MAIN_SETTINGS, settings);
	dsi_wreg(link, DSI_DIRECT_CMD_WRDAT0, wrdat[0]);
	if (len >  3)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT1, wrdat[1]);
	if (len >  7)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT2, wrdat[2]);
	if (len > 11)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT3, wrdat[3]);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);
	dsi_wreg(link, DSI_DIRECT_CMD_SEND, true);
	mdelay(10);

	dsi_wreg(link, DSI_CMD_MODE_STS_CLR, ~0);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);

	return 0;
}

/* MCDE channels */
struct mcde_chnl_state *mcde_chnl_get(enum mcde_chnl chnl_id,
	enum mcde_fifo fifo, const struct mcde_port *port)
{
	int i;
	struct mcde_chnl_state *chnl = NULL;

	debug("%s: Entering chnl_id=%d\n", __func__, chnl_id);
	/* Allocate channel */
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		if (chnl_id == channels[i].id)
			chnl = &channels[i];
	}
	if (!chnl) {
		dev_dbg(&mcde_dev->dev, "Invalid channel, chnl=%d\n", chnl_id);
		return ERR_PTR(-EINVAL);
	}
	if (chnl->inuse) {
		dev_dbg(&mcde_dev->dev, "Channel in use, chnl=%d\n", chnl_id);
		return ERR_PTR(-EBUSY);
	}

	chnl->port = *port;
	chnl->fifo = fifo;
	if (update_channel_static_registers(chnl) < 0)
		return ERR_PTR(-EINVAL);

	chnl->synchronized_update = false;
	chnl->pix_fmt = port->pixel_format;
	chnl->update_x = 0;
	chnl->update_y = 0;
	chnl->update_w = 0;
	chnl->update_h = 0;
	mcde_chnl_apply(chnl);
	chnl->inuse = true;

	debug("%s: Leaving chnl=%p\n", __func__, (void *)chnl);
	return chnl;
}

void mcde_chnl_set_pixel_format(struct mcde_chnl_state *chnl,
	enum mcde_port_pix_fmt pix_fmt)
{
	if (!chnl->inuse) {
		debug("%s: channel in use ovly=%p\n", __func__, chnl);
		return;
	}

	chnl->pix_fmt = pix_fmt;
}

int mcde_chnl_enable_synchronized_update(struct mcde_chnl_state *chnl,
	u8 enable)
{
	if (!chnl->inuse)
		return -EINVAL;
	chnl->synchronized_update = enable;
	return 0;
}

int mcde_chnl_set_power_mode(struct mcde_chnl_state *chnl,
	enum mcde_display_power_mode power_mode)
{
	if (!chnl->inuse)
		return -EINVAL;

	chnl->power_mode = power_mode;
	return 0;
}

void mcde_chnl_apply(struct mcde_chnl_state *chnl)
{
	u8 enable;
	debug("Entering %s\n", __func__);
	if (!chnl->inuse) {
		debug("%s: channel not in use chnl=%p\n", __func__, chnl);
		return;
	}

	enable = (chnl->update_w > 0) && (chnl->update_h > 0);
	chnl->regs.floen = enable;
	chnl->regs.ppl = chnl->update_w;
	chnl->regs.lpf = chnl->update_h;
	chnl->regs.bpp = portfmt2bpp(chnl->pix_fmt);
	chnl->regs.synchronized_update = chnl->synchronized_update;
	if (chnl->port.type == MCDE_PORTTYPE_DSI) {
		chnl->regs.clksel = MCDE_CRA1_CLKSEL_166MHZ;
		chnl->regs.dsipacking = portfmt2dsipacking(chnl->pix_fmt);
	} else if (chnl->port.type == MCDE_PORTTYPE_DPI) {
		if (chnl->port.phy.dpi.tv_mode) {
			chnl->regs.internal_clk = false;
			if (chnl->id == MCDE_CHNL_A)
				chnl->regs.clksel = MCDE_CRA1_CLKSEL_EXT_TV1;
			else
				chnl->regs.clksel = MCDE_CRA1_CLKSEL_EXT_TV2;
		} else {
			chnl->regs.internal_clk = true;
			chnl->regs.clksel = MCDE_CRA1_CLKSEL_LCD;
			chnl->regs.cdwin = portfmt2cdwin(chnl->pix_fmt);
			chnl->regs.bcd = (chnl->port.phy.dpi.clock_div < 2);
			if (!chnl->regs.bcd)
				chnl->regs.pcd =
					chnl->port.phy.dpi.clock_div - 2;
		}
		dpi_video_mode_apply(chnl);
	}
	chnl->transactionid++;

	dev_vdbg(&mcde_dev->dev, "Channel applied, chnl=%d\n", chnl->id);
	return;
}

static void chnl_update_registers(struct mcde_chnl_state *chnl)
{
	/* REVIEW: Move content to update_channel_register */
	/* and remove this one */
	if (chnl->port.type == MCDE_PORTTYPE_DPI)
		update_dpi_registers(chnl->id, &chnl->tv_regs);
	if (chnl->id == MCDE_CHNL_A || chnl->id == MCDE_CHNL_B)
		update_col_registers(chnl->id, &chnl->col_regs);
	update_channel_registers(chnl->id, &chnl->regs, &chnl->port,
						chnl->fifo, &chnl->vmode);

	chnl->transactionid_regs = chnl->transactionid;
}

static void chnl_update_continous(struct mcde_chnl_state *chnl)
{
	debug("%s: Entering\n", __func__);
	if (!chnl->continous_running) {
		if (chnl->transactionid_regs < chnl->transactionid)
			chnl_update_registers(chnl);
		if (chnl->port.sync_src == MCDE_SYNCSRC_TE0)
			mcde_wfld(MCDE_CRC, SYCEN0, true);
		else if (chnl->port.sync_src == MCDE_SYNCSRC_TE1)
			mcde_wfld(MCDE_CRC, SYCEN1, true);
		chnl->continous_running = true;
		/*
		* For main and secondary display,
		* FLOWEN has to be set before a SOFTWARE TRIG
		* Otherwise not overlay interrupt is triggerd
		*/
		enable_channel(chnl);
		if (chnl->port.type == MCDE_PORTTYPE_DSI &&
				chnl->port.sync_src == MCDE_SYNCSRC_OFF) {
			/*
			 * not used in u-boot
			mod_timer(&chnl->auto_sync_timer,
					jiffies +
			msecs_to_jiffies(MCDE_AUTO_SYNC_WATCHDOG * 1000));
			*/
		}
	}
}

static void chnl_update_non_continous(struct mcde_chnl_state *chnl)
{
	debug("%s\n", __func__);
	/* Commit settings to registers */
	wait_for_channel(chnl);
	chnl_update_registers(chnl);
	/*
	* For main and secondary display,
	* FLOWEN has to be set before a SOFTWARE TRIG
	* Otherwise not overlay interrupt is triggerd
	* However FLOWEN must not be triggered before SOFTWARE TRIG
	* if rotation is enabled
	*/
	enable_channel(chnl);
	/* TODO: look at port sync source and synched_update */
	if (chnl->regs.synchronized_update &&
				chnl->power_mode == MCDE_DISPLAY_PM_ON) {
		if (chnl->port.type == MCDE_PORTTYPE_DSI &&
			chnl->port.sync_src == MCDE_SYNCSRC_BTA) {
			while (dsi_rfld(chnl->port.link, DSI_CMD_MODE_STS,
				CSM_RUNNING))
				udelay(100);
			/*dsi_te_request(chnl);	not for u-boot */
		}
	} else {
		mcde_wreg(MCDE_CHNL0SYNCHSW +
			chnl->id * MCDE_CHNL0SYNCHSW_GROUPOFFSET,
			MCDE_CHNL0SYNCHSW_SW_TRIG(true));
		dev_vdbg(&mcde_dev->dev, "Channel update (no sync), chnl=%d\n",
			chnl->id);
	}
	if (chnl->power_mode == MCDE_DISPLAY_PM_ON)
		enable_channel(chnl);
}

static void chnl_update_overlay(struct mcde_chnl_state *chnl,
						struct mcde_ovly_state *ovly)
{
	debug("%s: chnl=%p ovly=%p\n", __func__, chnl, ovly);
	if (!ovly)
		return;
	if (!ovly || (ovly->transactionid_regs >= ovly->transactionid &&
			chnl->transactionid_regs >= chnl->transactionid))
		return;

	update_overlay_address_registers(ovly->idx, &ovly->regs);
	if (ovly->regs.reset_buf_id) {
		if (!chnl->continous_running)
			wait_for_overlay(ovly);

		update_overlay_registers(ovly->idx, &ovly->regs, &chnl->port,
			chnl->fifo, chnl->regs.x, chnl->regs.y,
			chnl->regs.ppl, chnl->regs.lpf, ovly->stride,
			chnl->vmode.interlaced);
		ovly->transactionid_regs = ovly->transactionid;
	} else if (chnl->continous_running) {
		ovly->transactionid_regs = ovly->transactionid;
		wait_for_overlay(ovly);
	}
}

int mcde_chnl_update(struct mcde_chnl_state *chnl,
					struct mcde_rectangle *update_area)
{
	debug("%s: chnl=%p rect=(%d, %d, %d, %d)\n", __func__, chnl,
		update_area->x, update_area->y, update_area->w, update_area->h);

	/* TODO: lock & make wait->trig async */
	if (!chnl->inuse || !update_area
			|| (update_area->w == 0 && update_area->h == 0)) {
		return -EINVAL;
	}

	chnl->regs.x   = update_area->x;
	chnl->regs.y   = update_area->y;
	/* TODO Crop against video_mode.xres and video_mode.yres */
	chnl->regs.ppl = update_area->w;
	chnl->regs.lpf = update_area->h;
	if (chnl->port.type == MCDE_PORTTYPE_DPI &&
						chnl->port.phy.dpi.tv_mode) {
		chnl->regs.ppl -= 2 * MCDE_CONFIG_TVOUT_HBORDER;
		/* subtract double borders, ie. per field */
		chnl->regs.lpf -= 4 * MCDE_CONFIG_TVOUT_VBORDER;
	} else if (chnl->port.type == MCDE_PORTTYPE_DSI &&
			chnl->vmode.interlaced)
		chnl->regs.lpf /= 2;

	chnl_update_overlay(chnl, chnl->ovly0);
	chnl_update_overlay(chnl, chnl->ovly1);

	if (chnl->port.update_auto_trig)
		chnl_update_continous(chnl);
	else
		chnl_update_non_continous(chnl);

	dev_vdbg(&mcde_dev->dev, "Channel updated, chnl=%d\n", chnl->id);
	return 0;
}

/* MCDE overlays */
struct mcde_ovly_state *mcde_ovly_get(struct mcde_chnl_state *chnl)
{
	struct mcde_ovly_state *ovly;

	if (!chnl->inuse) {
		debug("%s: channel in use ovly=%p\n", __func__, chnl);
		return ERR_PTR(-EINVAL);
	}

	if (!chnl->ovly0->inuse)
		ovly = chnl->ovly0;
	else if (chnl->ovly1 && !chnl->ovly1->inuse)
		ovly = chnl->ovly1;
	else {
		debug("%s: overlay in use ovly=%p\n", __func__, ovly);
		ovly = ERR_PTR(-EBUSY);
	}


	if (!IS_ERR(ovly)) {
		ovly->inuse = true;
		ovly->paddr = 0;
		ovly->stride = 0;
		ovly->pix_fmt = MCDE_OVLYPIXFMT_RGB565;
		ovly->src_x = 0;
		ovly->src_y = 0;
		ovly->dst_x = 0;
		ovly->dst_y = 0;
		ovly->dst_z = 0;
		ovly->w = 0;
		ovly->h = 0;
		mcde_ovly_apply(ovly);
	}

	return ovly;
}

void mcde_ovly_set_source_buf(struct mcde_ovly_state *ovly, u32 paddr)
{
	if (!ovly->inuse) {
		debug("%s: overlay not in use ovly=%p\n", __func__, ovly);
		return;
	}

	ovly->paddr = paddr;
}

void mcde_ovly_set_source_info(struct mcde_ovly_state *ovly,
	u32 stride, enum mcde_ovly_pix_fmt pix_fmt)
{
	if (!ovly->inuse) {
		debug("%s: overlay not in use ovly=%p\n", __func__, ovly);
		return;
	}

	ovly->stride = stride;
	ovly->pix_fmt = pix_fmt;
}

void mcde_ovly_set_source_area(struct mcde_ovly_state *ovly,
	u16 x, u16 y, u16 w, u16 h)
{
	if (!ovly->inuse) {
		debug("%s: overlay not in use ovly=%p\n", __func__, ovly);
		return;
	}

	ovly->src_x = x;
	ovly->src_y = y;
	ovly->w = w;
	ovly->h = h;
}

void mcde_ovly_set_dest_pos(struct mcde_ovly_state *ovly, u16 x, u16 y, u8 z)
{
	if (!ovly->inuse) {
		debug("%s: overlay not in use ovly=%p\n", __func__, ovly);
		return;
	}

	ovly->dst_x = x;
	ovly->dst_y = y;
	ovly->dst_z = z;
}

void mcde_ovly_set_pixfmt(struct mcde_ovly_state *ovly,
				enum mcde_ovly_pix_fmt fmt)
{
	if (!ovly->inuse) {
		debug("%s: overlay in use ovly=%p\n", __func__, ovly);
		return;
	}

	ovly->pix_fmt = fmt;
}

void mcde_ovly_apply(struct mcde_ovly_state *ovly)
{
	if (!ovly->inuse) {
		debug("%s: overlay not in use ovly=%p\n", __func__, ovly);
		return;
	}
	ovly->regs.ch_id = ovly->chnl->id;
	ovly->regs.enabled = ovly->paddr != 0;
	ovly->regs.baseaddress0 = ovly->paddr;
	ovly->regs.baseaddress1 = ovly->paddr + ovly->stride;
	/* TODO set to true if interlaced */
	ovly->regs.reset_buf_id = !ovly->chnl->continous_running;
	switch (ovly->pix_fmt) {
	case MCDE_OVLYPIXFMT_RGB565:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB565;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = true;
		break;
	case MCDE_OVLYPIXFMT_RGB888:
		ovly->regs.bits_per_pixel = 24;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB888;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = true;
		break;
	default:
		break;
	}
	ovly->regs.ppl = ovly->w;
	ovly->regs.lpf = ovly->h;
	ovly->regs.cropx = ovly->src_x;
	ovly->regs.cropy = ovly->src_y;
	ovly->regs.xpos = ovly->dst_x;
	ovly->regs.ypos = ovly->dst_y;
	ovly->regs.z = ovly->dst_z > 0;

	ovly->transactionid = ++ovly->chnl->transactionid;
	dev_vdbg(&mcde_dev->dev, "Overlay applied, chnl=%d\n", ovly->chnl->id);
}

/* Level shifter and clamp control registers */
#define PRCM_MMIP_LS_CLAMP_SET     (U8500_PRCMU_BASE + 0x420)
#define PRCM_MMIP_LS_CLAMP_CLR     (U8500_PRCMU_BASE + 0x424)

#define PRCM_PLLDSI_FREQ           (U8500_PRCMU_BASE + 0x500)
#define PRCM_PLLDSI_ENABLE         (U8500_PRCMU_BASE + 0x504)
#define PRCM_YYCLKEN0_MGT_SET      (U8500_PRCMU_BASE + 0x510)
#define PRCM_LCDCLK_MGT            (U8500_PRCMU_BASE + 0x044)
#define PRCM_MCDECLK_MGT           (U8500_PRCMU_BASE + 0x064)
#define PRCM_HDMICLK_MGT           (U8500_PRCMU_BASE + 0x058)
#define PRCM_TVCLK_MGT             (U8500_PRCMU_BASE + 0x07c)
#define PRCM_DSI_PLLOUT_SEL        (U8500_PRCMU_BASE + 0x530)
#define PRCM_DSITVCLK_DIV          (U8500_PRCMU_BASE + 0x52C)
#define PRCM_APE_RESETN_SET        (U8500_PRCMU_BASE + 0x1E4)
#define PRCM_APE_RESETN_CLR        (U8500_PRCMU_BASE + 0x1E8)

/* ePOD and memory power signal control registers */
#define PRCM_EPOD_C_SET            (U8500_PRCMU_BASE + 0x410)
#define PRCM_SRAM_LS_SLEEP         (U8500_PRCMU_BASE + 0x304)

/* Debug power control unit registers */
#define PRCM_POWER_STATE_SET       (U8500_PRCMU_BASE + 0x254)

/* Miscellaneous unit registers */
#define PRCM_DSI_SW_RESET          (U8500_PRCMU_BASE + 0x324)

/*
 * Used by MCDE to setup all necessary PRCMU registers
 */

#define PRCMU_CLAMP_DSS			0x00400400
#define PRCMU_CLAMP_DSIPLL		0x00800800
#define PRCMU_RESET_DSS			0x0000000C
#define PRCMU_RESET_DSIPLL		0x00004000
#define PRCMU_ENABLE_DSS_MEM		0x00200000
#define PRCMU_ENABLE_DSS_LOGIC		0x00100000
#define PRCMU_DSS_SLEEP_OUTPUT_MASK	0x400
#define PRCMU_POWER_ON_DSI		0x00008000

/* PRCMU clock/PLL/reset registers */
#define PRCMU_CLK_PLL_DIV_SHIFT		0
#define PRCMU_CLK_PLL_SW_SHIFT		5
#define PRCMU_CLK_EN			(1 << 8)

#define PRCMU_DSI_CLOCK_SETTING		0x00000148
#define PRCMU_LCDCLKEN			(1 << 17)

#define PRCMU_MCDE_CLOCK_ENABLE		0x01000000
/*
 * from linux prcmu-db8500.c:
 * Set DPI clock to 50000000 Hz
 */
#define PRCMU_DPI_CLOCK_SETTING		(PRCMU_CLK_EN | \
					(2 << PRCMU_CLK_PLL_SW_SHIFT) | \
					(8 << PRCMU_CLK_PLL_DIV_SHIFT))

#define PRCMU_DSI_LP_CLOCK_SETTING	0x00000F00
#define PRCMU_PLLDSI_FREQ_SETTING	0x00020123
#define PRCMU_ENABLE_PLLDSI		0x00000001
#define PRCMU_DSI_PLLOUT_SEL_SETTING	0x00000202
#define PRCMU_ENABLE_ESCAPE_CLOCK	0x07030101
#define PRCMU_DSI_RESET_SW		0x00000007

#define PRCMU_MCDE_DELAY		2


void mcde_enable_dss(void)
{
	u32 temp;
	debug("Entering %s\n", __func__);
#if CONFIG_SYS_DISPLAY_DSI
	/* Clamp DSS out, DSIPLL in/out */
	writel(PRCMU_CLAMP_DSS | PRCMU_CLAMP_DSIPLL, PRCM_MMIP_LS_CLAMP_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Enable DSS_M_INITN, DSS_L_RESETN, DSIPLL_RESETN resets */
	writel(PRCMU_RESET_DSS, PRCM_APE_RESETN_CLR);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on DSS mem */
	writel(PRCMU_ENABLE_DSS_MEM, PRCM_EPOD_C_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on DSS logic */
	writel(PRCMU_ENABLE_DSS_LOGIC, PRCM_EPOD_C_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Release DSS_SLEEP */
	temp = readl(PRCM_SRAM_LS_SLEEP);
	writel(temp & ~PRCMU_DSS_SLEEP_OUTPUT_MASK, PRCM_SRAM_LS_SLEEP);
	mdelay(PRCMU_MCDE_DELAY);
	/* Unclamp DSS out, DSIPLL in/out */
	writel(PRCMU_CLAMP_DSS | PRCMU_CLAMP_DSIPLL, PRCM_MMIP_LS_CLAMP_CLR);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on CSI_DSI */
	writel(PRCMU_POWER_ON_DSI, PRCM_POWER_STATE_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Enable MCDE Clock */
	writel(PRCMU_MCDE_CLOCK_ENABLE, PRCM_YYCLKEN0_MGT_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* PLLDIV=5, PLLSW=2, CLKEN=1 */
	writel(PRCMU_DSI_CLOCK_SETTING, PRCM_HDMICLK_MGT);
	mdelay(PRCMU_MCDE_DELAY);
	/* PLLDIV=14, PLLSW=2, CLKEN=1 */
	writel(PRCMU_DSI_LP_CLOCK_SETTING, PRCM_TVCLK_MGT);
	mdelay(PRCMU_MCDE_DELAY);
	/* D=43, N=1, R=4, SELDIV2=0 */
	writel(PRCMU_PLLDSI_FREQ_SETTING, PRCM_PLLDSI_FREQ);
	mdelay(PRCMU_MCDE_DELAY);
	/* Start DSI PLL */
	writel(PRCMU_ENABLE_PLLDSI, PRCM_PLLDSI_ENABLE);
	mdelay(PRCMU_MCDE_DELAY);
	/* Release DSS_M_INITN, DSS_L_RESETN, DSIPLL_RESETN */
	writel(PRCMU_RESET_DSS | PRCMU_RESET_DSIPLL, PRCM_APE_RESETN_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* DSI0=phi/2, DSI1=phi/2 */
	writel(PRCMU_DSI_PLLOUT_SEL_SETTING, PRCM_DSI_PLLOUT_SEL);
	mdelay(PRCMU_MCDE_DELAY);
	/* Enable ESC clk 0/1/2, div2=3, div1=0x17, div0=0x17 */
	writel(PRCMU_ENABLE_ESCAPE_CLOCK, PRCM_DSITVCLK_DIV);
	mdelay(PRCMU_MCDE_DELAY);
	/* Release DSI reset 0/1/2 */
	writel(PRCMU_DSI_RESET_SW, PRCM_DSI_SW_RESET);
	mdelay(PRCMU_MCDE_DELAY);
#endif
#if CONFIG_SYS_DISPLAY_DPI
	/* Clamp DSS out, DSIPLL in/out */
	writel(PRCMU_CLAMP_DSS, PRCM_MMIP_LS_CLAMP_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Enable DSS_M_INITN, DSS_L_RESETN resets */
	writel(PRCMU_RESET_DSS, PRCM_APE_RESETN_CLR);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on DSS mem */
	writel(PRCMU_ENABLE_DSS_MEM, PRCM_EPOD_C_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on DSS logic */
	writel(PRCMU_ENABLE_DSS_LOGIC, PRCM_EPOD_C_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Release DSS_SLEEP */
	temp = readl(PRCM_SRAM_LS_SLEEP);
	writel(temp & ~PRCMU_DSS_SLEEP_OUTPUT_MASK, PRCM_SRAM_LS_SLEEP);
	mdelay(PRCMU_MCDE_DELAY);
	/* Unclamp DSS out, DSIPLL in/out */
	writel(PRCMU_CLAMP_DSS, PRCM_MMIP_LS_CLAMP_CLR);
	mdelay(PRCMU_MCDE_DELAY);
	/* Enable MCDE Clock */
	writel(PRCMU_MCDE_CLOCK_ENABLE, PRCM_YYCLKEN0_MGT_SET);
	mdelay(PRCMU_MCDE_DELAY);
	/* Set up DPI Clock */
	writel(PRCMU_DPI_CLOCK_SETTING, PRCM_LCDCLK_MGT);
	mdelay(PRCMU_MCDE_DELAY);
	/* Release DSS_M_INITN, DSS_L_RESETN */
	writel(PRCMU_RESET_DSS, PRCM_APE_RESETN_SET);
	mdelay(PRCMU_MCDE_DELAY);
#endif
}

void update_mcde_registers(struct mcde_platform_data *pdata)
{
	/* Setup output muxing */
	mcde_wreg(MCDE_CONF0,
		MCDE_CONF0_IFIFOCTRLWTRMRKLVL(7) |
		MCDE_CONF0_OUTMUX0(pdata->outmux[0]) |
		MCDE_CONF0_OUTMUX1(pdata->outmux[1]) |
		MCDE_CONF0_OUTMUX2(pdata->outmux[2]) |
		MCDE_CONF0_OUTMUX3(pdata->outmux[3]) |
		MCDE_CONF0_OUTMUX4(pdata->outmux[4]) |
		pdata->syncmux);

	/* Setup sync pulse length */
	mcde_wreg(MCDE_VSCRC0,
		MCDE_VSCRC0_VSPMIN(1) |
		MCDE_VSCRC0_VSPMAX(0xff));
	mcde_wreg(MCDE_VSCRC1,
		MCDE_VSCRC1_VSPMIN(1) |
		MCDE_VSCRC1_VSPMAX(0xff));
}

int mcde_probe(u8 num_dsilinks, struct mcde_platform_data *pdata)
{
	int ret = 0;
	int i;
	u8 major_version;
	u8 minor_version;
	u8 development_version;

	debug("MCDE subsystem init begin\n");

	if (!pdata) {
		dev_dbg(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}
	if (num_dsilinks > 0) {
		dsiio = malloc(num_dsilinks * sizeof(*dsiio));
		if (!dsiio) {
			ret = -ENOMEM;
			debug("%s: Failed to malloc dsiio\n", __func__);
			goto failed_dsi_alloc;
		}

		mcdeio = (u8 *)CFG_MCDE_BASE;
		debug("MCDE iomap: 0x%.8X\n", (u32)mcdeio);
		for (i = 0; i < num_dsilinks; i++) {
			dsiio[i] = (u8 *)(CFG_DSI_BASE + i*0x1000);
			debug("MCDE DSI%d iomap: 0x%.8X\n", i, (u32)dsiio[i]);
		}
	}

	mcde_enable_dss();

	update_mcde_registers(pdata);

	major_version = MCDE_REG2VAL(MCDE_PID, MAJOR_VERSION,
							mcde_rreg(MCDE_PID));
	minor_version = MCDE_REG2VAL(MCDE_PID, MINOR_VERSION,
							mcde_rreg(MCDE_PID));
	development_version = MCDE_REG2VAL(MCDE_PID, DEVELOPMENT_VERSION,
							mcde_rreg(MCDE_PID));

	dev_info(&mcde_dev->dev, "MCDE HW revision %u.%u.%u.%u\n",
			major_version, minor_version, development_version,
					mcde_rfld(MCDE_PID, METALFIX_VERSION));

	if (major_version == 3 && minor_version == 0 &&
					development_version >= 8) {
		hardware_version = MCDE_CHIP_VERSION_3_0_8;
		dev_info(&mcde_dev->dev, "V2 HW\n");
	} else if (major_version == 3 && minor_version == 0 &&
					development_version >= 5) {
		hardware_version = MCDE_CHIP_VERSION_3_0_5;
		dev_info(&mcde_dev->dev, "V1 HW\n");
	} else {
		debug("Unsupported HW version\n");
		ret = ENODEV;
		goto failed_hardware_version;
	}

	debug("MCDE subsystem init done\n");
	return 0;

failed_hardware_version:
	free(dsiio);
	dsiio = NULL;

failed_dsi_alloc:
	return ret;
}

void mcde_init(void)
{
	int i;
	int j;

	debug("%s\n", __func__);
	/*
	 * clear and init static data
	 * link channel->overlay
	 */
	for (i = 0, j = 0; i < ARRAY_SIZE(channels); i++) {
		memset(&channels[i], 0, sizeof(struct mcde_chnl_state));
		if (j < 6)
			channels[i].ovly0 = &overlays[j++];
		if (j < 6)
			channels[i].ovly1 = &overlays[j++];
	}
	channels[0].id = MCDE_CHNL_A;
	channels[1].id = MCDE_CHNL_B;
	channels[2].id = MCDE_CHNL_C0;
	channels[3].id = MCDE_CHNL_C1;

	for (i = 0; i < ARRAY_SIZE(overlays); i++) {
		memset(&overlays[i], 0, sizeof(struct mcde_ovly_state));
		overlays[i].idx = i;
	}

	/* link overlay->channel */
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		channels[i].ovly0->chnl = &channels[i];
		if (channels[i].ovly1)
			channels[i].ovly1->chnl = &channels[i];
	}

	return;
}

void mcde_exit(void)
{
	if (dsiio) {
		free(dsiio);
		dsiio = NULL;
	}
}


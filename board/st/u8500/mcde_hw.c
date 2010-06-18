/*
* Copyright (C) ST-Ericsson SA 2010
*
* Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/

#include <common.h>
#include <command.h>
#include "gpio.h"
#include <tc35892.h>
#include <malloc.h>

#include <linux/err.h>
#include "dsilink_regs.h"
#include "mcde_regs.h"
#include "mcde.h"
#include <asm/arch/hardware.h>

#define DEBUG 0
#define dbg_printk(format, arg...)			\
	if (DEBUG)					\
		printf("mcde: " format, ##arg)

u8 *mcdeio;
u8 **dsiio;

static inline u32 dsi_rreg(int __i, u32 __reg)
{
	return readl(dsiio[__i] + __reg);
}
static inline void dsi_wreg(int __i, u32 __reg, u32 __val)
{
	writel(__val, dsiio[__i] + __reg);
}
#define dsi_rfld(__i, __reg, __fld) \
	((dsi_rreg(__i, __reg) & __reg##_##__fld##_MASK) >> \
		__reg##_##__fld##_SHIFT)
#define dsi_wfld(__i, __reg, __fld, __val) \
	dsi_wreg(__i, __reg, (dsi_rreg(__i, __reg) & \
	~__reg##_##__fld##_MASK) | (((__val) << __reg##_##__fld##_SHIFT) & \
		 __reg##_##__fld##_MASK))

static inline u32 mcde_rreg(u32 __reg)
{
	return readl(mcdeio + __reg);
}
static inline void mcde_wreg(u32 __reg, u32 __val)
{
	writel(__val, mcdeio + __reg);
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
	t_bool enabled;
	u32  baseaddress0;
	u32  baseaddress1;
	t_bool buf_id;
	u8   bits_per_pixel;
	u8   bpp;
	t_bool bgr;
	t_bool bebo;
	t_bool opq;
	u8   pixoff;
	u16  ppl;
	u16  lpf;
	u32  ljinc;
	u16  cropx;
	u16  cropy;
	u16  xpos;
	u16  ypos;
	u8   z;
};

struct mcde_ovly_state {
	t_bool inuse;
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
	t_bool floen;
	u16 x;
	u16 y;
	u16 ppl;
	u16 lpf;
	u8  bpp;
	/* DSI */
	u8 dsipacking;
};

struct mcde_chnl_state {
	t_bool inuse;
	enum mcde_chnl id;
	struct mcde_port port;
	struct mcde_ovly_state *ovly0;
	struct mcde_ovly_state *ovly1;
	const struct chnl_config *cfg;
	u32 transactionid;
	u32 transactionid_regs;

	/* Staged settings */
	t_bool synchronized_update;
	enum mcde_port_pix_fmt pix_fmt;
	u16 update_x;
	u16 update_y;
	u16 update_w;
	u16 update_h;

	/* Applied settings */
	struct chnl_regs regs;
};

static struct mcde_chnl_state channels[] = {
	{
		.id = MCDE_CHNL_C0,
		.ovly0 = &overlays[4],
		.ovly1 = NULL,
	},
};


/* MCDE internal helpers */
static u8 portfmt2dsipacking(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DSI_24BPP:
	default:
		return MCDE_DSIVID0CONF0_PACKING_RGB888;
	}
}

static u8 portfmt2bpp(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DSI_24BPP:
		return 24;
	default:
		return 0;
	}
}

#define DSI_UNIT_INTERVAL_0	0x9

void update_channel_static_registers(struct mcde_chnl_state *chnl)
{
	const struct mcde_port *port = &chnl->port;
	int i = 0;
	u8 idx = 2 * port->link + port->ifc;
	u8 lnk = port->link;
	/* Fifo & muxing */
	mcde_wfld(MCDE_CONF0, SWAP_A_C0, TRUE);
	mcde_wfld(MCDE_CR, FABMUX, FALSE);

	/* Formatter */
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, LINK_EN, TRUE);

	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, TRUE);
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, READ_EN, TRUE);
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, REG_TE_EN, TRUE);
	dsi_wreg(lnk, DSI_MCTL_DPHY_STATIC,
		DSI_MCTL_DPHY_STATIC_UI_X4(DSI_UNIT_INTERVAL_0));
	dsi_wreg(lnk, DSI_DPHY_LANES_TRIM,
		DSI_DPHY_LANES_TRIM_DPHY_SPECS_90_81B_ENUM(0_90));
	dsi_wreg(lnk, DSI_MCTL_DPHY_TIMEOUT,
		DSI_MCTL_DPHY_TIMEOUT_CLK_DIV(0xf) |
		DSI_MCTL_DPHY_TIMEOUT_HSTX_TO_VAL(0x3fff) |
		DSI_MCTL_DPHY_TIMEOUT_LPRX_TO_VAL(0x3fff));
	dsi_wreg(lnk, DSI_MCTL_MAIN_PHY_CTL,
		DSI_MCTL_MAIN_PHY_CTL_WAIT_BURST_TIME(0xf) |
		DSI_MCTL_MAIN_PHY_CTL_LANE2_EN(TRUE) |
		DSI_MCTL_MAIN_PHY_CTL_CLK_CONTINUOUS(FALSE));
	dsi_wreg(lnk, DSI_MCTL_ULPOUT_TIME,
		DSI_MCTL_ULPOUT_TIME_CKLANE_ULPOUT_TIME(1) |
		DSI_MCTL_ULPOUT_TIME_DATA_ULPOUT_TIME(1));
	dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_MODE, FALSE);
	dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_PRI, port->ifc == 1);
	dsi_wfld(lnk, DSI_CMD_MODE_CTL, TE_TIMEOUT, 0x3ff);
	dsi_wreg(lnk, DSI_MCTL_MAIN_EN,
		DSI_MCTL_MAIN_EN_PLL_START(TRUE) |
		DSI_MCTL_MAIN_EN_CKLANE_EN(TRUE) |
		DSI_MCTL_MAIN_EN_DAT1_EN(TRUE) |
		DSI_MCTL_MAIN_EN_DAT2_EN(port->phy.dsi.num_data_lanes
			== 2) |
		DSI_MCTL_MAIN_EN_IF1_EN(port->ifc == 0) |
		DSI_MCTL_MAIN_EN_IF2_EN(port->ifc == 1));
	while (dsi_rfld(lnk, DSI_MCTL_MAIN_STS, CLKLANE_READY) == 0 ||
	       dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT1_READY) == 0 ||
	       dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT2_READY) == 0) {
		mdelay(1);
		if (i++ == 10)
			printf("DSI lane not ready (link=%d)!\n", lnk);
	}


	if (port->ifc == 0 && port->link == 0)
		mcde_wfld(MCDE_CR, DSIVID0_EN, TRUE);
	else if (port->ifc == 0 && port->link == 1)
		mcde_wfld(MCDE_CR, DSIVID1_EN, TRUE);
	else if (port->ifc == 0 && port->link == 2)
		mcde_wfld(MCDE_CR, DSIVID2_EN, TRUE);
	else if (port->ifc == 1 && port->link == 0)
		mcde_wfld(MCDE_CR, DSICMD0_EN, TRUE);
	else if (port->ifc == 1 && port->link == 1)
		mcde_wfld(MCDE_CR, DSICMD1_EN, TRUE);
	else if (port->ifc == 1 && port->link == 2)
		mcde_wfld(MCDE_CR, DSICMD2_EN, TRUE);
	mcde_wreg(MCDE_DSIVID0CONF0 +
		idx * MCDE_DSIVID0CONF0_GROUPOFFSET,
		MCDE_DSIVID0CONF0_BLANKING(0) |
		MCDE_DSIVID0CONF0_VID_MODE(0) |
		MCDE_DSIVID0CONF0_CMD8(TRUE) |
		MCDE_DSIVID0CONF0_BIT_SWAP(FALSE) |
		MCDE_DSIVID0CONF0_BYTE_SWAP(FALSE) |
		MCDE_DSIVID0CONF0_DCSVID_NOTGEN(TRUE));

	if (port->ifc == 0)
		dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF1_ID,
			port->phy.dsi.virt_id);
	else if (port->ifc == 1)
		dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF2_ID,
			port->phy.dsi.virt_id);

	mcde_wfld(MCDE_CRC, SYCEN0, TRUE);
	mcde_wreg(MCDE_VSCRC0,
		MCDE_VSCRC0_VSPMIN(1) |
		MCDE_VSCRC0_VSPMAX(0xff));
	mcde_wreg(MCDE_CTRLC0, MCDE_CTRLC0_FIFOWTRMRK(0xa0));

	mcde_wfld(MCDE_CR, MCDEEN, TRUE);
	dbg_printk("Static registers setup, chnl=%d\n", chnl->id);
}

static void update_overlay_registers(u8 idx, struct ovly_regs *regs,
	u16 update_x, u16 update_y, u16 update_w, u16 update_h)
{
	u32 lmrgn = (regs->cropx + update_x) * regs->bits_per_pixel;
	u32 tmrgn = (regs->cropy + update_y) * regs->ljinc;
	u32 ppl = regs->ppl - update_x;
	u32 lpf = regs->lpf - update_y;

	if (!regs->enabled) {
		u32 temp;
		temp = mcde_rreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET);
		mcde_wreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET,
			(temp & ~MCDE_OVL0CR_OVLEN_MASK) |
			MCDE_OVL0CR_OVLEN(FALSE));
		return;
	}

	mcde_wreg(MCDE_EXTSRC0A0 + idx * MCDE_EXTSRC0A0_GROUPOFFSET,
		regs->baseaddress0);
	mcde_wreg(MCDE_EXTSRC0A1 + idx * MCDE_EXTSRC0A1_GROUPOFFSET,
		regs->baseaddress1);
	mcde_wreg(MCDE_EXTSRC0CONF + idx * MCDE_EXTSRC0CONF_GROUPOFFSET,
		MCDE_EXTSRC0CONF_BUF_ID(regs->buf_id) |
		MCDE_EXTSRC0CONF_BUF_NB(2) |
		MCDE_EXTSRC0CONF_PRI_OVLID(idx) |
		MCDE_EXTSRC0CONF_BPP(regs->bpp) |
		MCDE_EXTSRC0CONF_BGR(regs->bgr) |
		MCDE_EXTSRC0CONF_BEBO(regs->bebo) |
		MCDE_EXTSRC0CONF_BEPO(FALSE));
	mcde_wreg(MCDE_EXTSRC0CR + idx * MCDE_EXTSRC0CR_GROUPOFFSET,
		MCDE_EXTSRC0CR_SEL_MOD_ENUM(SOFTWARE_SEL) |
		MCDE_EXTSRC0CR_MULTIOVL_CTRL_ENUM(PRIMARY) |
		MCDE_EXTSRC0CR_FS_DIV_DISABLE(FALSE) |
		MCDE_EXTSRC0CR_FORCE_FS_DIV(FALSE));
	mcde_wreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET,
		MCDE_OVL0CR_OVLEN(TRUE) |
		MCDE_OVL0CR_COLCCTRL_ENUM(DISABLED) |
		MCDE_OVL0CR_CKEYGEN(FALSE) |
		MCDE_OVL0CR_ALPHAPMEN(TRUE) |
		MCDE_OVL0CR_OVLF(FALSE) |
		MCDE_OVL0CR_OVLR(FALSE) |
		MCDE_OVL0CR_OVLB(FALSE) |
		MCDE_OVL0CR_FETCH_ROPC(0) |
		MCDE_OVL0CR_STBPRIO(0) |
		MCDE_OVL0CR_BURSTSIZE(11) | /* TODO: _HW_8W */
		MCDE_OVL0CR_MAXOUTSTANDING(2) | /* TODO: get from ovly */
		MCDE_OVL0CR_ROTBURSTSIZE(2)); /* TODO: _4W, calculate? */
	mcde_wreg(MCDE_OVL0CONF + idx * MCDE_OVL0CONF_GROUPOFFSET,
		MCDE_OVL0CONF_PPL(ppl) |
		MCDE_OVL0CONF_EXTSRC_ID(idx) |
		MCDE_OVL0CONF_LPF(lpf));
	mcde_wreg(MCDE_OVL0CONF2 + idx * MCDE_OVL0CONF2_GROUPOFFSET,
		MCDE_OVL0CONF2_BP_ENUM(PER_PIXEL_ALPHA) |
		MCDE_OVL0CONF2_ALPHAVALUE(128) | /* TODO: Allow setting? */
		MCDE_OVL0CONF2_OPQ(regs->opq) |
		MCDE_OVL0CONF2_PIXOFF(lmrgn & 63) |
		MCDE_OVL0CONF2_PIXELFETCHERWATERMARKLEVEL(32));
	mcde_wreg(MCDE_OVL0LJINC + idx * MCDE_OVL0LJINC_GROUPOFFSET,
		regs->ljinc);
	mcde_wreg(MCDE_OVL0CROP + idx * MCDE_OVL0CROP_GROUPOFFSET,
		MCDE_OVL0CROP_TMRGN(tmrgn) |
		MCDE_OVL0CROP_LMRGN(lmrgn >> 6));
	mcde_wreg(MCDE_OVL0COMP + idx * MCDE_OVL0COMP_GROUPOFFSET,
		MCDE_OVL0COMP_XPOS(regs->xpos) |
		MCDE_OVL0COMP_CH_ID(regs->ch_id) |
		MCDE_OVL0COMP_YPOS(regs->ypos) |
		MCDE_OVL0COMP_Z(regs->z));
	dbg_printk("Overlay registers setup, idx=%d\n", idx);
}

void update_channel_registers(enum mcde_chnl chnl_id, struct chnl_regs *regs,
	struct mcde_port *port)
{
	u8 idx = chnl_id;
	if (!regs->floen) {
		mcde_wfld(MCDE_CRC, C1EN, FALSE);
		if (!mcde_rfld(MCDE_CRC, C2EN))
			mcde_wfld(MCDE_CRC, FLOEN, FALSE);
	}

	/* Channel */
	mcde_wreg(MCDE_CHNL0CONF + idx * MCDE_CHNL0CONF_GROUPOFFSET,
		MCDE_CHNL0CONF_PPL(regs->ppl-1) |
		MCDE_CHNL0CONF_LPF(regs->lpf-1));
	mcde_wreg(MCDE_CHNL0STAT + idx * MCDE_CHNL0STAT_GROUPOFFSET,
		MCDE_CHNL0STAT_CHNLBLBCKGND_EN(FALSE) |
		MCDE_CHNL0STAT_CHNLRD(TRUE));
	mcde_wreg(MCDE_CHNL0SYNCHMOD + idx * MCDE_CHNL0SYNCHMOD_GROUPOFFSET,
		MCDE_CHNL0SYNCHMOD_SRC_SYNCH_ENUM(SOFTWARE) | /* TODO: */
		MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_ENUM(FORMATTER));
	mcde_wreg(MCDE_CHNL0BCKGNDCOL + idx * MCDE_CHNL0BCKGNDCOL_GROUPOFFSET,
		MCDE_CHNL0BCKGNDCOL_B(255) | /* TODO: Temp */
		MCDE_CHNL0BCKGNDCOL_G(255) |
		MCDE_CHNL0BCKGNDCOL_R(255));
	mcde_wreg(MCDE_CHNL0PRIO + idx * MCDE_CHNL0PRIO_GROUPOFFSET,
		MCDE_CHNL0PRIO_CHNLPRIO(0));

	mcde_wfld(MCDE_CRC, POWEREN, TRUE);
	mcde_wfld(MCDE_CRC, FLOEN, TRUE);
	mcde_wfld(MCDE_CRC, C1EN, TRUE);

	/* Formatter */
	{
		u8 fidx = 2 * port->link + port->ifc;
		u32 temp, packet;
		temp = mcde_rreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET);
		mcde_wreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET,
			(temp & ~MCDE_DSIVID0CONF0_PACKING_MASK) |
			MCDE_DSIVID0CONF0_PACKING(regs->dsipacking));
		packet = ((regs->ppl * regs->bpp) >> 3) + 1; /* 1==CMD8 */
		mcde_wreg(MCDE_DSIVID0FRAME +
			fidx * MCDE_DSIVID0FRAME_GROUPOFFSET,
			MCDE_DSIVID0FRAME_FRAME(packet * regs->lpf));
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
			MCDE_DSIVID0DELAY0_INTPKTDEL(0));
		mcde_wreg(MCDE_DSIVID0DELAY1 +
			fidx * MCDE_DSIVID0DELAY1_GROUPOFFSET,
			MCDE_DSIVID0DELAY1_TEREQDEL(0) |
			MCDE_DSIVID0DELAY1_FRAMESTARTDEL(0));
	}
	dbg_printk("Channel registers setup, chnl=%d\n", chnl_id);
}

/* MCDE channels */
struct mcde_chnl_state *mcde_chnl_get(enum mcde_chnl chnl_id,
	enum mcde_fifo fifo, const struct mcde_port *port)
{
	int i;
	struct mcde_chnl_state *chnl = NULL;

	/* Allocate channel */
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		if (chnl_id == channels[i].id)
			chnl = &channels[i];
	}
	if (!chnl) {
		printf("Invalid channel, chnl=%d\n", chnl_id);
		return ERR_PTR(-EINVAL);
	}
	if (chnl->inuse) {
		printf("Channel in use, chnl=%d\n", chnl_id);
		return ERR_PTR(-EBUSY);
	}

	chnl->port = *port;
	chnl->synchronized_update = FALSE;
	chnl->update_x = 0;
	chnl->update_y = 0;
	chnl->update_w = 0;
	chnl->update_h = 0;
	mcde_chnl_apply(chnl);

	update_channel_static_registers(chnl);

	chnl->inuse = TRUE;
	return chnl;
}

void mcde_chnl_set_update_area(struct mcde_chnl_state *chnl,
	u16 x, u16 y, u16 w, u16 h)
{
	if (!chnl->inuse)
		return;

	chnl->update_x = x;
	chnl->update_y = y;
	chnl->update_w = w;
	chnl->update_h = h;
}

void mcde_chnl_set_pixel_format(struct mcde_chnl_state *chnl,
	enum mcde_port_pix_fmt pix_fmt)
{
	if (!chnl->inuse)
		return;

	chnl->pix_fmt = pix_fmt;
}

void mcde_chnl_apply(struct mcde_chnl_state *chnl)
{
	t_bool enable;
	if (!chnl->inuse)
		return;

	enable = chnl->update_w > 0 && chnl->update_h > 0;
	chnl->regs.floen = enable;
	chnl->regs.ppl = chnl->update_w;
	chnl->regs.lpf = chnl->update_h;
	chnl->regs.bpp = portfmt2bpp(chnl->pix_fmt);
	chnl->regs.dsipacking = portfmt2dsipacking(chnl->pix_fmt);
	chnl->transactionid++;
	dbg_printk("Channel applied, chnl=%d\n", chnl->id);
}

void mcde_chnl_update(struct mcde_chnl_state *chnl)
{
	struct mcde_ovly_state *ovly;

	if (!chnl->inuse || !chnl->regs.floen)
		return;

	/* Commit settings to registers */
	ovly = chnl->ovly0;
	if (ovly->transactionid_regs < ovly->transactionid ||
		chnl->transactionid_regs < chnl->transactionid) {
		update_overlay_registers(ovly->idx, &ovly->regs,
			chnl->regs.x, chnl->regs.y,
			chnl->regs.ppl, chnl->regs.lpf);
		ovly->transactionid_regs = ovly->transactionid;
	}
	ovly = chnl->ovly1;
	if (ovly && (
		ovly->transactionid_regs < ovly->transactionid ||
		chnl->transactionid_regs < chnl->transactionid)) {
		update_overlay_registers(ovly->idx, &ovly->regs,
			chnl->regs.x, chnl->regs.y,
			chnl->regs.ppl, chnl->regs.lpf);
		ovly->transactionid_regs = ovly->transactionid;
	}
	if (chnl->transactionid_regs < chnl->transactionid) {
		update_channel_registers(chnl->id, &chnl->regs, &chnl->port);
		chnl->transactionid_regs = chnl->transactionid;
	}

	if (chnl->regs.floen)
		mcde_wreg(MCDE_CHNL0SYNCHSW +
			chnl->id * MCDE_CHNL0SYNCHSW_GROUPOFFSET,
			MCDE_CHNL0SYNCHSW_SW_TRIG(TRUE));
	dbg_printk("Channel updated, chnl=%d\n", chnl->id);
}

/* MCDE overlays */
struct mcde_ovly_state *mcde_ovly_get(struct mcde_chnl_state *chnl)
{
	struct mcde_ovly_state *ovly;

	if (!chnl->inuse)
		return ERR_PTR(-EINVAL);

	if (!chnl->ovly0->inuse)
		ovly = chnl->ovly0;
	else if (chnl->ovly1 && !chnl->ovly1->inuse)
		ovly = chnl->ovly1;
	else
		ovly = ERR_PTR(-EBUSY);

	if (!IS_ERR(ovly)) {
		ovly->inuse = TRUE;
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
	if (!ovly->inuse)
		return;

	ovly->paddr = paddr;
}

void mcde_ovly_set_source_info(struct mcde_ovly_state *ovly,
	u32 stride, enum mcde_ovly_pix_fmt pix_fmt)
{
	if (!ovly->inuse)
		return;

	ovly->stride = stride;
	ovly->pix_fmt = pix_fmt;
}

void mcde_ovly_set_source_area(struct mcde_ovly_state *ovly,
	u16 x, u16 y, u16 w, u16 h)
{
	if (!ovly->inuse)
		return;

	ovly->src_x = x;
	ovly->src_y = y;
	ovly->w = w;
	ovly->h = h;
}

void mcde_ovly_set_dest_pos(struct mcde_ovly_state *ovly, u16 x, u16 y, u8 z)
{
	if (!ovly->inuse)
		return;

	ovly->dst_x = x;
	ovly->dst_y = y;
	ovly->dst_z = z;
}

void mcde_ovly_apply(struct mcde_ovly_state *ovly)
{
	if (!ovly->inuse)
		return;

	ovly->regs.ch_id = ovly->chnl->id;
	ovly->regs.enabled = ovly->paddr != 0;
	if (ovly->regs.buf_id)
		ovly->regs.baseaddress0 = ovly->paddr;
	else
		ovly->regs.baseaddress1 = ovly->paddr;
	ovly->regs.buf_id = !ovly->regs.buf_id;
	switch (ovly->pix_fmt) {
	case MCDE_OVLYPIXFMT_RGB565:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB565;
		ovly->regs.bgr = FALSE;
		ovly->regs.bebo = FALSE;
		ovly->regs.opq = TRUE;
		break;
	case MCDE_OVLYPIXFMT_RGB888:
		ovly->regs.bits_per_pixel = 24;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB888;
		ovly->regs.bgr = TRUE;
		ovly->regs.bebo = FALSE;
		ovly->regs.opq = TRUE;
		break;
	default:
		break;
	}
	ovly->regs.ppl = ovly->w;
	ovly->regs.lpf = ovly->h;
	ovly->regs.ljinc = ovly->stride;
	ovly->regs.cropx = ovly->src_x;
	ovly->regs.cropy = ovly->src_y;
	ovly->regs.xpos = ovly->dst_x;
	ovly->regs.ypos = ovly->dst_y;
	ovly->regs.z = ovly->dst_z > 0; /* 0 or 1 */

	ovly->transactionid = ++ovly->chnl->transactionid;
	dbg_printk("Overlay applied, chnl=%d\n", ovly->chnl->id);
}

/* DSI */
int mcde_dsi_dcs_write(struct mcde_port *port, u8 cmd, u8* data, int len)
{
	int i;
	u32 wrdat[4] = { 0, 0, 0, 0 };
	u32 settings;
	u8 link = port->link;
	u8 virt_id = port->phy.dsi.virt_id;

	if (len > MCDE_MAX_DCS_WRITE)
		return -EINVAL;

	wrdat[0] = cmd;
	for (i = 1; i <= len; i++)
		wrdat[i>>2] |= (data[i-1] << (i & 3));

	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(WRITE) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(len > 1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(len+1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(TRUE);
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
	dsi_wreg(link, DSI_DIRECT_CMD_SEND, TRUE);
	mdelay(10);

	dsi_wreg(link, DSI_CMD_MODE_STS_CLR, ~0);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);
	return 0;
}

/*
* Used by MCDE to setup all necessary PRCMU registers
*/

/* Level shifter and clamp control registers */
#define PRCM_MMIP_LS_CLAMP_SET     (U8500_PRCMU_BASE + 0x420)
#define PRCM_MMIP_LS_CLAMP_CLR     (U8500_PRCMU_BASE + 0x424)

/* PRCMU clock/PLL/reset registers */
#define PRCM_PLLDSI_FREQ           (U8500_PRCMU_BASE + 0x500)
#define PRCM_PLLDSI_ENABLE         (U8500_PRCMU_BASE + 0x504)
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

#define PRCMU_CLAMP_DSS_DSIPLL		0x00600C00
#define PRCMU_RESET_DSS			0x0000000C
#define PRCMU_ENABLE_DSS_MEM		0x00200000
#define PRCMU_ENABLE_DSS_LOGIC		0x00100000
#define PRCMU_DSS_SLEEP_OUTPUT_MASK	0x400
#define PRCMU_UNCLAMP_DSS_DSIPLL	0x00600C00
#define PRCMU_POWER_ON_DSI		0x00008000

#define PRCMU_MCDE_CLOCK_SETTING	0x00000125
#define PRCMU_ENABLE_PLLDSI		0x00000001
#define PRCMU_RELEASE_RESET_DSS		0x0000400C
#define PRCMU_DSI_PLLOUT_SEL_SETTING	0x00000202
#define PRCMU_DSI_RESET_SW		0x00000007

#define PRCMU_DSI_CLOCK_SETTING		0x00000148
#define PRCMU_PLLDSI_FREQ_SETTING	0x00020123
#define PRCMU_DSI_LP_CLOCK_SETTING	0x00000F00
#define PRCMU_ENABLE_ESCAPE_CLOCK	0x07030101

#define PRCMU_MCDE_DELAY			2

void mcde_enable_dss(void)
{
	u32 temp;
	/* Clamp DSS out, DSIPLL in/out, (why not DSS input?) */
	writel(PRCMU_CLAMP_DSS_DSIPLL, PRCM_MMIP_LS_CLAMP_SET);
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
	/* Unclamp DSS out, DSIPLL in/out, (why not DSS input?) */
	writel(PRCMU_UNCLAMP_DSS_DSIPLL, PRCM_MMIP_LS_CLAMP_CLR);
	mdelay(PRCMU_MCDE_DELAY);
	/* Power on CSI_DSI */
	writel(PRCMU_POWER_ON_DSI, PRCM_POWER_STATE_SET);
	mdelay(PRCMU_MCDE_DELAY);
	writel(PRCMU_MCDE_CLOCK_SETTING, PRCM_MCDECLK_MGT);
	mdelay(PRCMU_MCDE_DELAY);
	/* HDMI and TVCLK Should be handled somewhere else */
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
	writel(PRCMU_RELEASE_RESET_DSS, PRCM_APE_RESETN_SET);
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
}

int mcde_init(u8 num_data_lanes)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		channels[i].ovly0->chnl = &channels[i];
		if (channels[i].ovly1)
			channels[i].ovly1->chnl = &channels[i];
	}

	dsiio = malloc(num_data_lanes * sizeof(*dsiio));
	if (!dsiio) {
		printf("%s: Failed to malloc dsiio\n", __func__);
		return -EINVAL;
	}

	mcdeio = (u8 *)CFG_MCDE_BASE;
	dbg_printk("MCDE iomap: 0x%.8X\n", (u32)mcdeio);
	for (i = 0; i < num_data_lanes; i++) {
		dsiio[i] = (u8 *)(CFG_DSI_BASE + i*0x1000);
		dbg_printk("MCDE DSI%d iomap: 0x%.8X\n", i, (u32)dsiio[i]);
	}
	return 0;
}

void mcde_exit(void)
{
	if (dsiio)
		free(dsiio);
}

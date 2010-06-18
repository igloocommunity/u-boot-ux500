/*
* Copyright (C) ST-Ericsson SA 2010
*
* Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
* for ST-Ericsson.
*
* License terms: GNU General Public License (GPL), version 2.
*/
#ifndef __MCDE__H__
#define __MCDE__H__

#define mdelay(n) ({unsigned long msec = (n); while (msec--) udelay(1000); })

/* Physical interface types */
enum mcde_port_type {
	MCDE_PORTTYPE_DSI = 0,
};

/* Interface mode */
enum mcde_port_mode {
	MCDE_PORTMODE_CMD = 0,
};

/* MCDE fifos */
enum mcde_fifo {
	MCDE_FIFO_A  = 0,
};

/* MCDE channels (pixel pipelines) */
enum mcde_chnl {
	MCDE_CHNL_C0 = 2,
};

/* Update sync mode */
enum mcde_sync_src {
	MCDE_SYNCSRC_BTA = 3, /* DSI BTA */
};

struct mcde_port {
	enum mcde_port_type type;
	enum mcde_port_mode mode;
	u8 ifc;
	u8 link;
	enum mcde_sync_src sync_src;
	union {
		struct {
			u8 virt_id;
			u8 num_data_lanes;
		} dsi;
	} phy;
};

/* Overlay pixel formats (input) */
enum mcde_ovly_pix_fmt {
	MCDE_OVLYPIXFMT_RGB565   = 1,
	MCDE_OVLYPIXFMT_RGB888   = 4,
};

/* Interface pixel formats (output) */
enum mcde_port_pix_fmt {
	/* MIPI standard formats */
	MCDE_PORTPIXFMT_DSI_24BPP =        0x34,

};

struct mcde_chnl_state;

struct mcde_chnl_state *mcde_chnl_get(enum mcde_chnl chnl_id,
	enum mcde_fifo fifo, const struct mcde_port *port);
void mcde_chnl_set_update_area(struct mcde_chnl_state *chnl,
	u16 x, u16 y, u16 w, u16 h);
void mcde_chnl_set_pixel_format(struct mcde_chnl_state *chnl,
	enum mcde_port_pix_fmt pix_fmt);
void mcde_chnl_apply(struct mcde_chnl_state *chnl);
void mcde_chnl_update(struct mcde_chnl_state *chnl);

void mcde_enable_dss(void);

/* MCDE overlay */
struct mcde_ovly_state;

struct mcde_ovly_state *mcde_ovly_get(struct mcde_chnl_state *chnl);
void mcde_ovly_set_source_buf(struct mcde_ovly_state *ovly,
	u32 paddr);
void mcde_ovly_set_source_info(struct mcde_ovly_state *ovly,
	u32 stride, enum mcde_ovly_pix_fmt pix_fmt);
void mcde_ovly_set_source_area(struct mcde_ovly_state *ovly,
	u16 x, u16 y, u16 w, u16 h);
void mcde_ovly_set_dest_pos(struct mcde_ovly_state *ovly,
	u16 x, u16 y, u8 z);
void mcde_ovly_apply(struct mcde_ovly_state *ovly);

/* MCDE dsi */

#define MCDE_MAX_DCS_WRITE		15
#define DCS_CMD_WRITE_START           0x2C
#define DCS_CMD_WRITE_CONTINUE        0x3C

int mcde_dsi_dcs_write(struct mcde_port *port, u8 cmd, u8* data, int len);

/* MCDE */

int mcde_init(u8 num_data_lanes);
void mcde_exit(void);

#endif /* __MCDE__H__ */


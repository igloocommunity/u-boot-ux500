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
	MCDE_PORTTYPE_DPI = 1,
};

/* Interface mode */
enum mcde_port_mode {
	MCDE_PORTMODE_CMD = 0,
	MCDE_PORTMODE_VID = 1,
};

/* MCDE fifos */
enum mcde_fifo {
	MCDE_FIFO_A  = 0,
	MCDE_FIFO_B  = 1,
	MCDE_FIFO_C0 = 2,
	MCDE_FIFO_C1 = 3,
};

/* MCDE channels (pixel pipelines) */
enum mcde_chnl {
	MCDE_CHNL_A  = 0,
	MCDE_CHNL_B  = 1,
	MCDE_CHNL_C0 = 2,
	MCDE_CHNL_C1 = 3,
};

/* Display power modes */
enum mcde_display_power_mode {
	MCDE_DISPLAY_PM_OFF     = 0, /* Power off */
	MCDE_DISPLAY_PM_STANDBY = 1, /* DCS sleep mode */
	MCDE_DISPLAY_PM_ON      = 2, /* DCS normal mode, display on */
};

/* Update sync mode */
#define MCDE_CHNLPATH(__chnl, __fifo, __type, __ifc, __link) \
	(((__chnl) << 16) | ((__fifo) << 12) | \
	 ((__type) << 8) | ((__ifc) << 4) | ((__link) << 0))
enum mcde_chnl_path {
	/* Channel A */
	MCDE_CHNLPATH_CHNLA_FIFOA_DPI_0 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_A, MCDE_PORTTYPE_DPI, 0, 0),
	MCDE_CHNLPATH_CHNLA_FIFOA_DSI_IFC0_0 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 0, 0),
	MCDE_CHNLPATH_CHNLA_FIFOA_DSI_IFC0_1 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 0, 1),
	MCDE_CHNLPATH_CHNLA_FIFOC0_DSI_IFC0_2 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 0, 2),
	MCDE_CHNLPATH_CHNLA_FIFOC0_DSI_IFC1_0 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 1, 0),
	MCDE_CHNLPATH_CHNLA_FIFOC0_DSI_IFC1_1 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 1, 1),
	MCDE_CHNLPATH_CHNLA_FIFOA_DSI_IFC1_2 = MCDE_CHNLPATH(MCDE_CHNL_A,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 1, 2),
	/* Channel B */
	MCDE_CHNLPATH_CHNLB_FIFOB_DPI_1 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_B, MCDE_PORTTYPE_DPI, 0, 1),
	MCDE_CHNLPATH_CHNLB_FIFOB_DSI_IFC0_0 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 0, 0),
	MCDE_CHNLPATH_CHNLB_FIFOB_DSI_IFC0_1 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 0, 1),
	MCDE_CHNLPATH_CHNLB_FIFOC1_DSI_IFC0_2 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 0, 2),
	MCDE_CHNLPATH_CHNLB_FIFOC1_DSI_IFC1_0 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 1, 0),
	MCDE_CHNLPATH_CHNLB_FIFOC1_DSI_IFC1_1 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 1, 1),
	MCDE_CHNLPATH_CHNLB_FIFOB_DSI_IFC1_2 = MCDE_CHNLPATH(MCDE_CHNL_B,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 1, 2),
	/* Channel C0 */
	MCDE_CHNLPATH_CHNLC0_FIFOA_DSI_IFC0_0 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 0, 0),
	MCDE_CHNLPATH_CHNLC0_FIFOA_DSI_IFC0_1 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 0, 1),
	MCDE_CHNLPATH_CHNLC0_FIFOC0_DSI_IFC0_2 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 0, 2),
	MCDE_CHNLPATH_CHNLC0_FIFOC0_DSI_IFC1_0 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 1, 0),
	MCDE_CHNLPATH_CHNLC0_FIFOC0_DSI_IFC1_1 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_C0, MCDE_PORTTYPE_DSI, 1, 1),
	MCDE_CHNLPATH_CHNLC0_FIFOA_DSI_IFC1_2 = MCDE_CHNLPATH(MCDE_CHNL_C0,
		MCDE_FIFO_A, MCDE_PORTTYPE_DSI, 1, 2),
	/* Channel C1 */
	MCDE_CHNLPATH_CHNLC1_FIFOB_DSI_IFC0_0 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 0, 0),
	MCDE_CHNLPATH_CHNLC1_FIFOB_DSI_IFC0_1 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 0, 1),
	MCDE_CHNLPATH_CHNLC1_FIFOC1_DSI_IFC0_2 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 0, 2),
	MCDE_CHNLPATH_CHNLC1_FIFOC1_DSI_IFC1_0 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 1, 0),
	MCDE_CHNLPATH_CHNLC1_FIFOC1_DSI_IFC1_1 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_C1, MCDE_PORTTYPE_DSI, 1, 1),
	MCDE_CHNLPATH_CHNLC1_FIFOB_DSI_IFC1_2 = MCDE_CHNLPATH(MCDE_CHNL_C1,
		MCDE_FIFO_B, MCDE_PORTTYPE_DSI, 1, 2),
};

/* Update sync mode */
enum mcde_sync_src {
	MCDE_SYNCSRC_OFF = 0, /* No sync */
	MCDE_SYNCSRC_TE0 = 1, /* MCDE ext TE0 */
	MCDE_SYNCSRC_TE1 = 2, /* MCDE ext TE1 */
	MCDE_SYNCSRC_BTA = 3, /* DSI BTA */
};

/* Interface pixel formats (output) */
enum mcde_port_pix_fmt {
	/* MIPI standard formats */

	MCDE_PORTPIXFMT_DPI_16BPP_C1 =     0x21,
	MCDE_PORTPIXFMT_DPI_16BPP_C2 =     0x22,
	MCDE_PORTPIXFMT_DPI_16BPP_C3 =     0x23,
	MCDE_PORTPIXFMT_DPI_18BPP_C1 =     0x24,
	MCDE_PORTPIXFMT_DPI_18BPP_C2 =     0x25,
	MCDE_PORTPIXFMT_DPI_24BPP =        0x26,

	MCDE_PORTPIXFMT_DSI_16BPP =        0x31,
	MCDE_PORTPIXFMT_DSI_18BPP =        0x32,
	MCDE_PORTPIXFMT_DSI_18BPP_PACKED = 0x33,
	MCDE_PORTPIXFMT_DSI_24BPP =        0x34,

	/* Custom formats */
	MCDE_PORTPIXFMT_DSI_YCBCR422 =     0x40,
};

#define MCDE_PORT_DPI_NO_CLOCK_DIV	0

#define DPI_ACT_HIGH_ALL	0 /* all signals are active high	  */
#define DPI_ACT_LOW_HSYNC	1 /* horizontal sync signal is active low */
#define DPI_ACT_LOW_VSYNC	2 /* vertical sync signal is active low	  */
#define DPI_ACT_LOW_DATA_ENABLE	4 /* data enable signal is active low	  */
#define DPI_ACT_ON_FALLING_EDGE	8 /* drive data on the falling edge of the
				   * pixel clock
				   */

struct mcde_port {
	enum mcde_port_type type;
	enum mcde_port_mode mode;
	enum mcde_port_pix_fmt pixel_format;
	u8 ifc;
	u8 link;
	enum mcde_sync_src sync_src;
	u8 update_auto_trig;
	union {
		struct {
			u8 virt_id;
			u8 num_data_lanes;
			u8 ui;
			u8 clk_cont;
			u8 data_lanes_swap;
		} dsi;
		struct {
			u8 bus_width;
			u8 tv_mode;
			u16 clock_div;	/* use 0 or 1 for no clock divider */
			u32 polarity;	/* see DPI_ACT_LOW_* definitions */
		} dpi;
	} phy;
};

/* Overlay pixel formats (input) */
enum mcde_ovly_pix_fmt {
	MCDE_OVLYPIXFMT_RGB565   = 1,
	MCDE_OVLYPIXFMT_RGB888   = 4,
};

#define MCDE_FIFO_AB_SIZE 640
#define MCDE_FIFO_C0C1_SIZE 160

#define MCDE_PIXFETCH_LARGE_WTRMRKLVL 128
#define MCDE_PIXFETCH_MEDIUM_WTRMRKLVL 32
#define MCDE_PIXFETCH_SMALL_WTRMRKLVL 16

/* Tv-out defines */
#define MCDE_CONFIG_TVOUT_HBORDER 2
#define MCDE_CONFIG_TVOUT_VBORDER 2
#define MCDE_CONFIG_TVOUT_BACKGROUND_LUMINANCE		0x83
#define MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CB	0x9C
#define MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CR	0x2C

/* In seconds */
#define MCDE_AUTO_SYNC_WATCHDOG 5

/* Hardware versions */
#define MCDE_CHIP_VERSION_3_0_8 2
#define MCDE_CHIP_VERSION_3_0_5 1
#define MCDE_CHIP_VERSION_3	0

/* DSI modes */
#define DSI_VIDEO_MODE	0
#define DSI_CMD_MODE	1

/* Video mode descriptor */
struct mcde_video_mode {
	u32 xres;
	u32 yres;
	u32 pixclock;	/* pixel clock in ps (pico seconds) */
	u32 hbp;	/* hor back porch = left_margin */
	u32 hfp;	/* hor front porch equals to right_margin */
	u32 hsw;	/* horizontal sync width */
	u32 vbp1;	/* field 1: vert back porch equals to upper_margin */
	u32 vfp1;	/* field 1: vert front porch equals to lower_margin */
	u32 vbp2;	/* field 2: vert back porch equals to upper_margin */
	u32 vfp2;	/* field 2: vert front porch equals to lower_margin */
	u32 vsw;	/* vertical sync width*/
	u8 interlaced;
	u8 bckcol[3];	/* background color */
};

struct mcde_rectangle {
	u16 x;
	u16 y;
	u16 w;
	u16 h;
};

struct mcde_chnl_state;

struct mcde_chnl_state *mcde_chnl_get(enum mcde_chnl chnl_id,
	enum mcde_fifo fifo, const struct mcde_port *port);
void mcde_chnl_set_update_area(struct mcde_chnl_state *chnl,
	u16 x, u16 y, u16 w, u16 h);
void mcde_chnl_set_pixel_format(struct mcde_chnl_state *chnl,
	enum mcde_port_pix_fmt pix_fmt);
void mcde_chnl_apply(struct mcde_chnl_state *chnl);
int mcde_chnl_update(struct mcde_chnl_state *chnl,
					struct mcde_rectangle *update_area);

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

struct mcde_platform_data {
	/* DSI */
	int num_dsilinks;

	/* DPI */
	u8 outmux[5]; /* MCDE_CONF0.OUTMUXx */
	u8 syncmux;   /* MCDE_CONF0.SYNCMUXx */
};


/* MCDE */

extern int dpi_display_platform_enable(void);
extern void mcde_get_hardware_version(void);
extern void update_mcde_registers(struct mcde_platform_data *pdata);
extern int mcde_chnl_set_video_mode(struct mcde_chnl_state *chnl,
				struct mcde_video_mode *vmode);

extern int mcde_chnl_set_power_mode(struct mcde_chnl_state *chnl,
	enum mcde_display_power_mode power_mode);
extern int mcde_chnl_set_video_mode(struct mcde_chnl_state *chnl,
				struct mcde_video_mode *vmode);
int mcde_turn_on_display_dpi(void);
int mcde_turn_on_display_dsi(void);
int mcde_probe(u8 num_data_lanes, struct mcde_platform_data *pdata);
void mcde_init(void);
void mcde_exit(void);

#endif /* __MCDE__H__ */


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
#include <linux/err.h>
#include <part.h>
#include <mmc.h>
#include <bmp_layout.h>
#include <asm/arch/common.h>
#include "mcde.h"
#include "mcde_display.h"

/* bmp compression constants */
#define BI_RGB		0
#define BI_RLE8		1	/* RLE 8-bit/pixel */
#define BI_RLE4		2	/* RLE 4-bit/pixel */
#define BI_BITFIELDS	3

extern struct mcde_display_device main_display;

static uint32_t read_unaligned32(uint32_t *val)
{
	uint32_t ret;
	memcpy(&ret, val, sizeof(int32_t));
	return ret;
}

static void copy_indexed(u16 *bmp_dst, u8 *bmp_start,
				int dst_pitch, int src_pitch,
				u32 width, u32 height, u8 *palette)
{
	/*
	 * Copy image from flash area to overlay area and
	 * convert format from 8-bit indexed to rgb565.
	 */
	int i;
	int j;
	int o;
	int pad;
	u16 val16;
	u32 w;
	u8 r;
	u8 g;
	u8 b;
	u8 *bmp_src;

	pad = (dst_pitch - 2 * width) / 2;

	/* expanding 8 bit indexes to 16 bit rgb */
	for (i = height - 1, o = 0; i >= 0; i--) {
		bmp_src = (u8 *)(bmp_start + i * src_pitch);
		for (j = 0; j < width; j++) {
			w = bmp_src[j];		/* get index */
			w <<= 2;		/* make offset */
			w += (u32)palette;	/* make address */
			r = *((u8 *)w++);
			g = *((u8 *)w++);
			b = *((u8 *)w);
			val16 = r >> 3;
			val16 |= (g >> 2) << 5;
			val16 |= (b >> 3) << 11;
			bmp_dst[o++] = val16;
		}
		o += pad;
	}
}

static void copy_rgb565(u8 *bmp_dst, u8 *bmp_start,
				int dst_pitch, int src_pitch,
				u32 width, u32 height)
{
	int i;
	u8 *bmp_src;

	/* point after source data */
	bmp_src = (u8 *)(bmp_start + (height * src_pitch));
	for (i = 0; i < height; i++) {
		bmp_src -= src_pitch;
		memcpy(bmp_dst, bmp_src, src_pitch);
		bmp_dst += dst_pitch;
	}
}

static void copy_rgb(u16 *bmp_dst, u8 *bmp_start,
				int dst_pitch, int src_pitch,
				u32 width, u32 height, u32 bitspp)
{
	/*
	 * Copy image from flash area to overlay area and
	 * convert format from (a)rgb888(8) to rgb565.
	 */
	int i;
	int j;
	int o;
	int pad;
	u16 val16;
	u8 r;
	u8 g;
	u8 b;
	u8 *bmp_src;
	u32 src_inc = bitspp / 8;

	pad = (dst_pitch - 2 * width) / 2;

	/* convert to 16 bit */
	for (i = height - 1, o = 0; i >= 0; i--) {
		bmp_src = (u8 *)(bmp_start + i * src_pitch);
		for (j = 0; j < (width * src_inc); j += src_inc) {
			r = bmp_src[j];
			g = bmp_src[j + 1];
			b = bmp_src[j + 2];
			val16 = r >> 3;
			val16 |= (g >> 2) << 5;
			val16 |= (b >> 3) << 11;
			bmp_dst[o++] = val16;
		}
		o += pad;
	}
}

int mcde_display_image(struct mcde_chnl_state *chnl)
{
	int err = 0;
	struct mmc *emmc_dev;
	u32 address = (CONFIG_SYS_VIDEO_FB_ADRS + 2 *
		CONFIG_SYS_DISPLAY_NATIVE_X_RES *
		CONFIG_SYS_DISPLAY_NATIVE_Y_RES); /* after frame buffer */
	u8 *bmp_start;
	struct bmp_header *bmp_header;
	u32 bmp_offset;
	u32 dib_bytesz;
	u32 dib_width;
	u32 dib_height;
	u32 dib_compression;
	u16 dib_bitspp;
	int src_pitch;
	int dst_pitch;
	u32 dib_header_size;
	u8 *palette;
	u32 palette_size;
	u8 *bmp_src;
	u8 *bmp_dst;
	u8 *ovly_mem = (u8 *)CONFIG_SYS_VIDEO_FB_ADRS;	/* frame buffer */
	struct mcde_ovly_state *ovly;
	u32 xpos = 0;
	u32 ypos = 0;

	debug("%s: Enter\n", __func__);
	emmc_dev = find_mmc_device(CONFIG_EMMC_DEV_NUM);
	if (emmc_dev == NULL) {
		printf("mcde_display_image: emmc not found.\n");
		return -ENODEV;
	}

	if (toc_load_toc_entry(&emmc_dev->block_dev, MCDE_TOC_SPLASH_NAME, 0,
			       0, address)) {
		printf("mcde_display_image: no splash image found.\n");
		return -ENOENT;
	}

	/* get bmp_image */
	bmp_start = (u8 *)address;
	debug("%s: bmp start = 0x%p\n", __func__, (void *)bmp_start);

	/* check BMP magic */
	bmp_header = (struct bmp_header *)bmp_start;
	if (bmp_header->signature[0] != 'B' ||
		bmp_header->signature[1] != 'M') {
		printf("%s: unsupported filetype, must be BMP\n", __func__);
		return -EILSEQ;
	}

	/* get offset to bitmap-data from the BMP header */
	bmp_offset = read_unaligned32(&bmp_header->data_offset);
	debug("bmp filesz = %d\n",
		read_unaligned32(&bmp_header->file_size));
	debug("bmp offset = %d\n", bmp_offset);

	dib_width = read_unaligned32((uint32_t *)&bmp_header->width);
	dib_height = read_unaligned32((uint32_t *)&bmp_header->height);
	dib_bytesz = read_unaligned32(&bmp_header->image_size);
	dib_bitspp = bmp_header->bit_count;
	dib_header_size = read_unaligned32(&bmp_header->size);
	debug("dib header_sz = %d\n", dib_header_size);
	debug("dib width = %d\n", dib_width);
	debug("dib height = %d\n", dib_height);
	debug("dib nplanes = %d\n", bmp_header->planes);
	debug("dib bitspp = %d\n", dib_bitspp);
	dib_compression = read_unaligned32(&bmp_header->compression);
	debug("dib compress_type = %d\n", dib_compression);
	debug("dib dib_bytesz = %d\n", dib_bytesz);

	/* calculate palette address */
	palette = ((u8 *)&bmp_header->size + dib_header_size);
	palette_size = ((bmp_start + bmp_offset) - palette);
	debug("palette size = %d\n", palette_size);
	/* if same as image start: no palette */
	if (palette_size == 0)
		palette = NULL;
	debug("palette = 0x%08x\n", (u32)palette);

	/* check validity */
	if ((dib_width > main_display.native_x_res) ||
			(dib_height > main_display.native_y_res)) {
		printf("%s: image to large, must be [%d,%d] or smaller\n",
			__func__, main_display.native_x_res,
			main_display.native_y_res);
		err++;
	}
	if (bmp_header->planes != 1) {
		printf("%s: unsupported nplanes, must be 1\n", __func__);
		err++;
	}

	src_pitch = dib_width * dib_bitspp / 8;
	src_pitch = (src_pitch + 3) >> 2;	/* pad to 32-bit boundary */
	src_pitch <<= 2;
	debug("src_pitch=%d\n", src_pitch);

	if (dib_compression != BI_RGB && dib_compression != BI_BITFIELDS)
		err++;

	if (err != 0)
		return -EINVAL;

	/* set new pitch */
	dst_pitch = dib_width * 16 / 8;		/* dest is 16 bpp */
	dst_pitch = (dst_pitch + 7) >> 3;	/* pad to 64-bit boundary */
	dst_pitch <<= 3;
	debug("dst_pitch=%d\n", dst_pitch);

	/* image is stored upside-down in the file */
	bmp_dst = ovly_mem;
	bmp_src = (u8 *)(bmp_start + bmp_offset);
	debug("bmp copy dst=0x%08x, src=0x%08x, len=%d\n",
		(uint32_t)bmp_dst, (uint32_t)bmp_src, dib_bytesz);

	switch (dib_bitspp) {
	case 8:
		copy_indexed((u16 *)bmp_dst, bmp_src, dst_pitch, src_pitch,
					dib_width, dib_height, palette);
		break;
	case 16:
		copy_rgb565(bmp_dst, bmp_src, dst_pitch, src_pitch,
					dib_width, dib_height);
		break;
	case 24:
	case 32:
		copy_rgb((u16 *)bmp_dst, bmp_src, dst_pitch, src_pitch,
					dib_width, dib_height, dib_bitspp);
		break;
	default:
		printf("%s: unsupported bitspp=%d\n", __func__, dib_bitspp);
		return -EINVAL;
	}
	debug("%s: image OK\n", __func__);

	/* dss_enable_overlay */
	ovly = mcde_ovly_get(chnl);
	if (IS_ERR(ovly)) {
		err = PTR_ERR(ovly);
		printf("%s: Failed to get channel\n", __func__);
		return -err;
	}
	debug("ovly=%p ovly_mem=%p\n", (void *)ovly, (void *)ovly_mem);
	mcde_ovly_set_source_buf(ovly, (u32)ovly_mem);
	mcde_ovly_set_source_info(ovly, dst_pitch,
					main_display.default_pixel_format);
	mcde_ovly_set_source_area(ovly, 0, 0, dib_width, dib_height);
	if (dib_width == main_display.native_x_res)
		xpos = 0;
	else
		xpos = (main_display.native_x_res - dib_width) / 2;

	if (dib_height == main_display.native_y_res)
		ypos = 0;
	else
		ypos = (main_display.native_y_res - dib_height) / 2;
	mcde_ovly_set_dest_pos(ovly, xpos, ypos, 0);
	mcde_ovly_apply(ovly);

	return mcde_turn_on_display();
}


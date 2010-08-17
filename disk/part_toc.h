/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Peter Nessrup <peter.nessrup@stericsson.com> for
 * ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef _DISK_PART_TOC_H
#define _DISK_PART_TOC_H

#define SUPPORTED_SECTOR_SIZE	512
#define TOC_MAGIC		"ISSW"
#define TOC_ID_NO_ENTRY	"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"
#define TOC_MAX_ENTRIES		(SUPPORTED_SECTOR_SIZE/sizeof(toc_entry_t))
#define MAX_NUM_TOCS		4
#define TOC_BOOT_IMAGE_SIZE	(0x20000 / SUPPORTED_SECTOR_SIZE)
#define NO_TOC			-1
#define TOC_ID_LENGTH		12

/* Sub TOC identifier in the 'flags' field is 'ST' */
#define FLAGS_SUB_TOC		0x00005354
#define FLAGS_PARTITION		0x00000001

typedef struct __attribute__ ((__packed__)) {
	u32 offset;
	u32 size;
	u32 flags;
	u32 align;
	u32 loadaddr;
	u8  id[TOC_ID_LENGTH];
} toc_entry_t;

typedef struct {
	u8  id[TOC_ID_LENGTH];
	u32 num_items;
	u32 location;
	u32 cached_section;
	toc_entry_t cache[TOC_MAX_ENTRIES];
} subtoc_t;

typedef struct {
	u32 start;
	u32 size;
	u8  name[12];
	u32 cfg_offset;
	u32 type;
	u32 unused;
} toc_part_entry_t;
#endif	/* _DISK_PART_TOC_H */

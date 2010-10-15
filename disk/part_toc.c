/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Peter Nessrup <peter.nessrup@stericsson.com> for
 * ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */

/*
 * Support for TOC partitioning
 *
 * This implementation is only LE aware
 */

#include <common.h>
#include <command.h>
#include "part_toc.h"

static int toc_location = NO_TOC;
static toc_entry_t toc_main_toc[TOC_MAX_ENTRIES];
static subtoc_t toc_subtoc = {.num_items = 0};

static int is_toc(toc_entry_t *toc)
{
	/* Check for TOC MAGIC */
	debug("is_toc\n");
	return memcmp(toc->id, TOC_MAGIC, sizeof(TOC_MAGIC));
}

static void toc_recalc_entry_offsets(toc_entry_t *toc)
{
	int i;

	debug("recalc_toc_entry_offsets\n");
	/*
	 * Recalculate the offsets that are outside of the boot image (0x20000)
	 * since they need to be absolute and not relative
	 */
	if (toc_location == 0)
		return;

	for (i = 0; i < TOC_MAX_ENTRIES; i++, toc++) {
		/*
		 * Only do this for offsets outside the boot area and not
		 * on empty entries
		 */
		if (memcmp(toc->id,
			    TOC_ID_NO_ENTRY,
			    TOC_ID_LENGTH) == 0) {
			debug("Empty TOC entry\n");
			continue;
		}

		if (toc->offset >= TOC_BOOT_IMAGE_SIZE) {
			debug("Recalculating offset on TOCEntry: %s\n",
			      toc->id);
			toc->offset +=
				(toc_location * SUPPORTED_SECTOR_SIZE);
		}
	}
}

static int toc_subtoc_cache_read(block_dev_desc_t *dev_desc,
				 const int entry_num,
				 subtoc_t *subtoc)
{
	debug("toc_subtoc_cache_read()\n");

	if ((entry_num/TOC_MAX_ENTRIES) != subtoc->cached_section) {
		/* The entry is not in the cached section */
		if (dev_desc->block_read(dev_desc->dev,
					 subtoc->location +
					 (entry_num / TOC_MAX_ENTRIES),
					 1,
					 (ulong *) &subtoc->cache) == 1) {
			debug("Sub TOC entries read into cache\n");
			/* New cached section available */
			subtoc->cached_section = entry_num / TOC_MAX_ENTRIES;
		} else {
			printf("Error trying to read into subtoc cache!\n");
			return 1;
		}
	}
	return 0;
}

static toc_entry_t *toc_get_subtoc_entry_num(block_dev_desc_t *dev_desc,
					     const int num,
					     subtoc_t *subtoc)
{
	debug("toc_get_subtoc_entry_num()\n");
	if (num >= subtoc->num_items)
		return NULL;

	if (toc_subtoc_cache_read(dev_desc, num, subtoc) == 0)
		/*
		 * Now we have the correct cache for the requested entry
		 * Return the requested entry
		 */
		return &subtoc->cache[num % TOC_MAX_ENTRIES];

	printf("Sub TOC entry not found\n");
	return NULL;
}

static toc_entry_t *toc_get_subtoc_entry_id(block_dev_desc_t *dev_desc,
					    const char *toc_id,
					    subtoc_t *subtoc)
{
	int i;
	toc_entry_t *toc_entry;
	u8 toc_id_buf[TOC_ID_LENGTH];

	debug("toc_get_subtoc_entry_id()\n");

	if (subtoc->num_items != 0) {
		/*
		 * Use a tmp buffer to compare the incoming string with the
		 * toc entry id, as it is not a null terminated string in the
		 * toc and we always want to compare all bytes. This to not make
		 * a false match of a subset of the toc entry id
		 */
		memset(toc_id_buf, 0, TOC_ID_LENGTH);
		memcpy(toc_id_buf, toc_id, MIN(strlen(toc_id), TOC_ID_LENGTH));

		for (i = 0; i < subtoc->num_items; i++) {
			toc_entry = toc_get_subtoc_entry_num(dev_desc,
							     i,
							     subtoc);
			if (toc_entry == NULL)
				break;

			if (memcmp(toc_id_buf,
				   toc_entry->id,
				   TOC_ID_LENGTH) == 0) {
				debug("%s found.\n", toc_id);
				return toc_entry;
			}
		}
	}

	printf("Sub TOC entry not found\n");
	return NULL;
}

static toc_entry_t *toc_get_entry_subtoc(toc_entry_t *toc)
{
	int i;

	/* Get a sub TOC entry */
	debug("get_toc_entry_subtoc()\n");

	/* Find sub TOC entry */
	for (i = 0; i < TOC_MAX_ENTRIES; i++, toc++) {
		/* Check the flags for a sub TOC */
		if ((toc->flags & 0x0000FFFF) == FLAGS_SUB_TOC)
			return toc;

		if (memcmp(toc->id,
			    TOC_ID_NO_ENTRY,
			    TOC_ID_LENGTH) == 0)
			/*
			 * Don't iterate the rest of the entries
			 * if an empty entry was found
			 */
			break;
	}

	printf("sub TOC not found!\n");
	return NULL;
}

static toc_entry_t *toc_get_entry(block_dev_desc_t *dev_desc,
				  const char *toc_id,
				  toc_entry_t *toc,
				  subtoc_t *subtoc)
{
	int i;
	u8 toc_id_buf[TOC_ID_LENGTH];

	debug("get_toc_entry()\n");
	/*
	 * Use a tmp buffer to compare the incoming string with the
	 * toc entry id, as it is not a null terminated string in the
	 * toc and we always want to compare all bytes. This to not make
	 * a false match of a subset of the toc entry id
	 */
	memset(toc_id_buf, 0, TOC_ID_LENGTH);
	memcpy(toc_id_buf, toc_id, MIN(strlen(toc_id), TOC_ID_LENGTH));

	/* Find TOC entry */
	for (i = 0; i < TOC_MAX_ENTRIES; i++, toc++) {
		if (memcmp(toc_id_buf, toc->id, TOC_ID_LENGTH) == 0) {
			debug("%s found.\n", toc_id);
			return toc;
		}
		/*
		 * Don't iterate the rest of the entries
		 * if an empty entry was found
		 */
		if (memcmp(toc->id,
			    TOC_ID_NO_ENTRY,
			    TOC_ID_LENGTH) == 0)
			break;
	}

	/* Check if the Id can be found in the sub TOC */
	return toc_get_subtoc_entry_id(dev_desc, toc_id, subtoc);
}

static int toc_get_part_entry(block_dev_desc_t *dev_desc,
			      subtoc_t *subtoc,
			      int part,
			      toc_part_entry_t *part_entry)
{
	int i;
	toc_entry_t *toc_entry;

	debug("get_toc_part_entry\n");
	/*
	 * Search for the partition to be retreived,
	 * Only sub TOC can hold partitions since the Flags field
	 * is used as the identifier
	 */
	if (subtoc->num_items == 0)
		return 1;

	if (part < 1) {
		printf("Invalid partition number: %d\n", part);
		return 1;
	}
	for (i = 0; i < subtoc->num_items; i++) {
		toc_entry = toc_get_subtoc_entry_num(dev_desc,
						     i,
						     subtoc);
		if (toc_entry->flags & FLAGS_PARTITION) {
			/*
			 * We found an entry claiming to be a partition,
			 * decrease the counter
			 */
			if (--part == 0)
				/* This is the requested partition */
				break;
		}
	}

	if (part == 0) {
		/*
		 * We found the wanted partition,
		 * Check alignment
		 */
		if ((toc_entry->offset % dev_desc->blksz) != 0) {
			printf("Partition entry not "
			       "aligned to %lud byte block "
			       "boundary!\n", dev_desc->blksz);
			return 1;
		}
		if ((toc_entry->size % dev_desc->blksz) != 0) {
			printf("Partition size not "
			       "aligned to %lud byte block "
			       "boundary!\n", dev_desc->blksz);
			return 1;
		}
		part_entry->start =
			toc_entry->offset / dev_desc->blksz;
		part_entry->size =
			toc_entry->size / dev_desc->blksz;
		memcpy(part_entry->name, toc_entry->id, sizeof(toc_entry->id));
		return 0;
	}
	return 1;
}

static void toc_print_entry(toc_entry_t *toc_entry)
{
	char buf[TOC_ID_LENGTH + 1];

	if (toc_entry) {
		printf("0x%08x ", toc_entry->offset);
		printf("0x%08x ", toc_entry->size);
		printf("0x%08x ", toc_entry->flags);
		printf("0x%08x ", toc_entry->align);
		printf("0x%08x ", toc_entry->loadaddr);
		memcpy(buf, &toc_entry->id, TOC_ID_LENGTH);
		buf[TOC_ID_LENGTH] = 0;
		printf("\"%s\"\n", buf);
	}
}

static void toc_print_subtoc(block_dev_desc_t *dev_desc, subtoc_t *subtoc)
{
	int i = 0;

	if (subtoc->num_items == 0)
		return;

	printf("Printing Sub TOC entries - pointed out by: %s\n",
		subtoc->id);
	printf("Offset     Size       Flags      Align      LoadAddr   ID\n");
	for (i = 0; i < subtoc->num_items; i++)
		toc_print_entry(toc_get_subtoc_entry_num(dev_desc, i, subtoc));
}

static int toc_print(block_dev_desc_t *dev_desc,
		     toc_entry_t *toc,
		     subtoc_t *subtoc)
{
	int i = 0;

	debug("print_toc\n");

	if (toc_location == NO_TOC) {
		printf("TOC doesn't exist!\n");
		return 1;
	}
	/* Print the whole TOC */
	printf("Printing TOC at %x\n", toc_location);
	printf("Offset     Size       Flags      Align      LoadAddr   ID\n");
	for (i = 0; i < TOC_MAX_ENTRIES; i++, toc++) {
		if (memcmp(toc->id,
			   TOC_ID_NO_ENTRY,
			   TOC_ID_LENGTH) != 0)
			/* Don't print empty TOC entries */
			toc_print_entry(toc);
	}
	toc_print_subtoc(dev_desc, subtoc);

	return 0;
}

static void toc_read_subtoc(block_dev_desc_t *dev_desc,
			    toc_entry_t *toc,
			    subtoc_t *subtoc)
{
	toc_entry_t *subtoc_entry;

	debug("toc_read_subtoc\n");
	/* Reset the sub TOC struct first of all */
	memset(subtoc->id, 0, sizeof(subtoc->id));
	subtoc->num_items = 0;
	memset(subtoc->cache, 0xFF, sizeof(subtoc->cache));
	subtoc->cached_section = 0;

	/* Try to get the sub TOC entry in the root TOC */
	subtoc_entry = toc_get_entry_subtoc(toc);

	if (subtoc_entry == NULL) {
		debug("No sub TOC found\n");
		return;
	}

	/* We found a sub TOC in the root TOC, initialize the struct */
	debug("A sub TOC exists\n");

	/* Read the first block from media to the cache */
	if (dev_desc->block_read(dev_desc->dev,
				 (subtoc_entry->offset)/dev_desc->blksz,
				 1,
				 (ulong *) subtoc->cache) == 1)
		debug("Sub TOC entries read into cache\n");
	else {
		printf("Error trying to read the sub TOC!\n");
		return;
	}

	/* Fill in the rest of the struct */
	memcpy(subtoc->id, subtoc_entry->id, sizeof(subtoc->id));
	/* TOC entries in the subtoc */
	subtoc->num_items = subtoc_entry->size/(sizeof(toc_entry_t));
	/* Location of the subtoc in blocks */
	subtoc->location = subtoc_entry->offset/dev_desc->blksz;
}

static int toc_read(block_dev_desc_t *dev_desc, toc_entry_t *toc)
{
	int i = 0;

	debug("readTOC\n");
	do {
		debug("TOC: #%d\n", i+1);
		/* Read up what should be the TOC */
		if (dev_desc->block_read(dev_desc->dev,
					 i*TOC_BOOT_IMAGE_SIZE,
					 1,
					 (ulong *) toc) != 1)
			return 1;

		/* Search for TOC identifier to validate the TOC */
		if (is_toc(toc) == 0) {
			toc_location = i*TOC_BOOT_IMAGE_SIZE;
			debug("TOC found, TOC location: %x.\n",
			      toc_location);
		}
		i++;
	} while ((toc_location == NO_TOC) && (i < MAX_NUM_TOCS));

	if (toc_location != NO_TOC) {
		/* We found a valid TOC */
		toc_recalc_entry_offsets(toc);
		debug("TOC now exists\n");
		/* Try to find a sub TOC as well */
		toc_read_subtoc(dev_desc, toc, &toc_subtoc);
		return 0;
	}

	return 1;
}

static int toc_init(block_dev_desc_t *dev_desc, toc_entry_t *toc)
{
	static int dev_num = -1;

	debug("initTOC\n");
	if (dev_desc->blksz != SUPPORTED_SECTOR_SIZE) {
		printf("Sector size: %lud is not supported!\n",
			dev_desc->blksz);
		return 1;
	}

	if (dev_num != dev_desc->dev)
		/* New device, re-initialize the TOC */
		toc_location = NO_TOC;

	if (toc_location == NO_TOC) {
		/* Read TOC to see if we have a valid TOC */
		if (toc_read(dev_desc, toc) == 0) {
			/*
			 * We've found a TOC for this device,
			 * save the device number to know which
			 * device have initiated the TOC
			 */
			dev_num = dev_desc->dev;
			return 0;
		}
	} else
		/*
		 * We are trying to initialize the TOC again
		 * for the same device
		 */
		return 0;

	/* No valid TOC found */
	return 1;
}

int get_partition_info_toc(block_dev_desc_t *dev_desc,
			   int part,
			   disk_partition_t *info)
{
	toc_part_entry_t part_entry;

	debug("get_partition_info_toc\n");
	if (toc_init(dev_desc, &toc_main_toc[0]) == 0) {
		/* Get the partition entry */
		if (toc_get_part_entry(dev_desc,
				       &toc_subtoc,
				       part,
				       &part_entry) == 0) {
			info->start = part_entry.start;
			info->size = part_entry.size;
			info->blksz = dev_desc->blksz;
			memcpy(info->name,
			       part_entry.name,
			       sizeof(part_entry.name));
			return 0;
		} else
			debug("Partition entry not found!\n");
	} else
		debug("Init or read TOC failed\n");

	return -1;
}

void print_part_toc(block_dev_desc_t *dev_desc)
{
	debug("print_part_toc\n");
	if (toc_print(dev_desc, &toc_main_toc[0], &toc_subtoc) != 0)
		printf("Failed to print TOC\n");
}

int test_part_toc(block_dev_desc_t *dev_desc)
{
	debug("test_part_toc\n");
	if (toc_init(dev_desc, &toc_main_toc[0]) == 0)
		return 0;
	else
		return 1;
}

int get_entry_info_toc(block_dev_desc_t *dev_desc, const char *toc_id,
		       u32 *offset, u32 *size, u32 *loadaddr)
{
	toc_entry_t *toc_entry;

	/* Initialize the TOC for the device we are trying to read from */
	if (toc_init(dev_desc, &toc_main_toc[0]) == 0) {
		toc_entry = toc_get_entry(dev_desc,
					  toc_id,
					  &toc_main_toc[0],
					  &toc_subtoc);

		if (toc_entry != NULL) {
			*offset = toc_entry->offset;
			*size = toc_entry->size;
			*loadaddr = toc_entry->loadaddr;
			return 0;
		}

		debug("TOC Entry: %s not found!\n", toc_id);
	}
	return 1;
}

/*
 * toc_load_toc_entry - Loads data from the specified toc partition
 *
 * @param [in] dev_desc Pointer to block device.
 * @param [in] toc_id Name of toc partition.
 * @param [in] offset Offset into toc partition (in bytes).
 * @param [in] size Size of data to read (in bytes). If 0 entire toc partion
 *		    will be loaded.
 * @param [in] loadaddr Destination address for loaded data. If 0 then address
 *			from toc will be used.
 * @return Returns 0 on success, 1 on fail.
 */
int toc_load_toc_entry(block_dev_desc_t *dev_desc, const char *toc_id,
		       u32 offset, u32 size, u32 loadaddr)
{
	u32 entry_offset;
	u32 entry_size;
	u32 entry_loadaddr;
	u32 n;

	debug("toc_load_toc_entry: Loading %s\n", toc_id);

	if (size % SUPPORTED_SECTOR_SIZE) {
		printf("toc_load_toc_entry: only sizes of multiple of %d is "
		       "supported\n", SUPPORTED_SECTOR_SIZE);
		return 1;
	}

	if (offset % SUPPORTED_SECTOR_SIZE) {
		printf("toc_load_toc_entry: only offsets of multiple of %d is "
		       "supported\n", SUPPORTED_SECTOR_SIZE);
		return 1;
	}

	if (get_entry_info_toc(dev_desc, toc_id, &entry_offset,
			       &entry_size, &entry_loadaddr)) {
		printf("toc_load_toc_entry: get_entry_info_toc failed\n");
		return 1;
	}

	entry_offset += offset;

	if ((size < entry_size) && (size != 0))
		entry_size = size;

	if (loadaddr != 0)
		entry_loadaddr = loadaddr;

	entry_size = (entry_size / dev_desc->blksz) +
	       ((entry_size % dev_desc->blksz) ? 1 : 0);

	debug("toc_load_toc_entry: entry_offset:0x%X entry_size:%d "
	      "entry_loadaddr:0x%X\n", entry_offset, entry_size,
	      entry_loadaddr);

	n = dev_desc->block_read(dev_desc->dev,
				 entry_offset / dev_desc->blksz,
				 entry_size,
				 (void *)entry_loadaddr);

	if (n != entry_size) {
		printf("toc_load_toc_entry: Failed to load %s!\n", toc_id);
		return 1;
	}

	return 0;
}

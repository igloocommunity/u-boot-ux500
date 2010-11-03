/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Ulf Hansson <ulf.hansson@stericsson.com> for
 * ST-Ericsson
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <common.h>
#include <environment.h>
#include <part.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

/* global var allocated in env_common when doing env_relocate */
env_t *env_ptr = 0;

/* global var used from cmd_nvedit when saveenv */
char *env_name_spec = "EMMC";

/* global funtion used from env_common */
uchar env_get_char_spec(int index)
{
	return ( *((uchar *)(gd->env_addr + index)) );
}

/* global function called before mmc_initialize */
int env_init(void)
{
	set_default_env();
	return 0;
}

/* global function called from cmd_nvedit */
#ifdef CONFIG_CMD_SAVEENV
int saveenv(void)
{
	int ret;
	struct mmc *mmc_dev;
	u32 offset;
	u32 size;
	u32 loadaddr;
	u32 blkcnt;

	printf("Writing to EMMC... ");

	mmc_dev = find_mmc_device(CONFIG_EMMC_DEV_NUM);
	if (!mmc_dev) {
		printf("emmc device not found, so env not saved!\n");
		return 1;
	}

	ret = get_entry_info_toc(&mmc_dev->block_dev,
				CONFIG_ENV_TOC_NAME,
				&offset,
				&size,
				&loadaddr);
	if (ret) {
		printf("env address not found, so env not saved!\n");
		return 1;
	}

	if (size != CONFIG_ENV_SIZE) {
		printf("env size mismatch, so env not saved!\n");
		return 1;
	}

	env_crc_update();

	blkcnt = (size + mmc_dev->write_bl_len - 1) / mmc_dev->write_bl_len;
	ret = mmc_dev->block_dev.block_write(mmc_dev->block_dev.dev,
					offset / mmc_dev->write_bl_len,
					blkcnt,
					(void *)env_ptr);
	if (ret != blkcnt) {
		printf("env write failed, so env not saved!\n");
		return 1;
	}

	printf("done\n");
	return 0;
}
#endif

/* global function called from env_common when doing env_relocate */
void env_relocate_spec (void)
{
	int ret;
	struct mmc *mmc_dev;
	u32 offset;
	u32 size;
	u32 loadaddr;
	u32 blkcnt;

	mmc_dev = find_mmc_device(CONFIG_EMMC_DEV_NUM);
	if (!mmc_dev) {
		printf("env emmc device not found, so setting default env!\n");
		goto err;
	}

	ret = get_entry_info_toc(&mmc_dev->block_dev,
				CONFIG_ENV_TOC_NAME,
				&offset,
				&size,
				&loadaddr);
	if (ret) {
		printf("env not found, so setting default env!\n");
		goto err;
	}

	if (size != CONFIG_ENV_SIZE) {
		printf("env size mismatch, so setting default env!\n");
		goto err;
	}

	blkcnt = (size + mmc_dev->read_bl_len - 1) / mmc_dev->read_bl_len;
	ret = mmc_dev->block_dev.block_read(mmc_dev->block_dev.dev,
					offset / mmc_dev->read_bl_len,
					blkcnt,
					(void *)env_ptr);
	if (ret != blkcnt) {
		printf("env read failed, so setting default env!\n");
		goto err;
	}

	if (crc32(0, env_ptr->data, ENV_SIZE) != env_ptr->crc) {
		printf("env crc failed, so setting default env!\n");
		goto err;
	}
	return;

err:
	set_default_env();
}


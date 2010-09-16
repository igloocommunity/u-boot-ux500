/*
 * Copyright (C) ST-Ericsson SA 2009
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/* #define DEBUG */

#include <common.h>

#if defined(CONFIG_ENV_IS_IN_EMMC) /* Environment is in EMMC Flash */

#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <mmc.h>

#if defined(CONFIG_CMD_ENV) && defined(CONFIG_CMD_EMMC)
#define CMD_SAVEENV
#elif defined(CONFIG_ENV_OFFSET_REDUND)
#error Cannot use CONFIG_ENV_OFFSET_REDUND without CONFIG_CMD_ENV & CONFIG_CMD_EMMC
#endif

#if defined(CONFIG_ENV_SIZE_REDUND) && (CONFIG_ENV_SIZE_REDUND != CONFIG_ENV_SIZE)
#error CONFIG_ENV_SIZE_REDUND should be the same as CONFIG_ENV_SIZE
#endif

#ifdef CONFIG_INFERNO
#error CONFIG_INFERNO not supported yet
#endif

/* references to names in env_common.c */
extern uchar default_environment[];
extern int default_environment_size;

char * env_name_spec = "EMMC";


#ifdef ENV_IS_EMBEDDED
extern uchar environment[];
env_t *env_ptr = (env_t *)(&environment[0]);
#else /* ! ENV_IS_EMBEDDED */
env_t *env_ptr = 0;
#endif /* ENV_IS_EMBEDDED */


DECLARE_GLOBAL_DATA_PTR;

uchar env_get_char_spec (int index)
{
	return ( *((uchar *)(gd->env_addr + index)) );
}

static int emmc_read_write(u32 byte_offset, void *read_buffer,
			   u32 size, u32 write)
{
	int i;
	u32 xferred_bytes;
	struct mmc *boot_dev = NULL;
	u32 blkcnt;

	for (i = 0;; i++) {
		boot_dev = find_mmc_device(i);
		if (!boot_dev)
			return -1;
		if (!strcmp(boot_dev->name, env_name_spec))
			break;
	}

	if (write) {
		blkcnt = size/boot_dev->write_bl_len;
		if (size && !blkcnt)
			blkcnt++;
		xferred_bytes =
		  boot_dev->block_dev.block_write(i,
					    byte_offset/boot_dev->write_bl_len,
						  blkcnt,
						  (u_char *)read_buffer);
		xferred_bytes *= boot_dev->write_bl_len;
	} else {
		blkcnt = size/boot_dev->read_bl_len;
		if (size && !blkcnt)
			blkcnt++;
		xferred_bytes =
		  boot_dev->block_dev.block_read(i,
					     byte_offset/boot_dev->read_bl_len,
						 blkcnt,
						 (u_char *)read_buffer);
		xferred_bytes *= boot_dev->read_bl_len;
	}
	debug("emmc read write:requested:0x%x,done:0x%x", size, xferred_bytes);
	return xferred_bytes;
}


/* this is called before nand_init()
 * so we can't read Nand to validate env data.
 * Mark it OK for now. env_relocate() in env_common.c
 * will call our relocate function which will does
 * the real validation.
 *
 * When using a eMMC boot image (like sequoia_nand), the environment
 * can be embedded or attached to the U-Boot image in eMMC flash. This way
 * the SPL loads not only the U-Boot image from eMMC but also the
 * environment.
 */
int env_init(void)
{
#if defined(ENV_IS_EMBEDDED)
	ulong total;
	int crc1_ok = 0, crc2_ok = 0;
	env_t *tmp_env1, *tmp_env2;

	total = CONFIG_ENV_SIZE;

	tmp_env1 = env_ptr;
	tmp_env2 = (env_t *)((ulong)env_ptr + CONFIG_ENV_SIZE);

	crc1_ok = (crc32(0, tmp_env1->data, ENV_SIZE) == tmp_env1->crc);
	crc2_ok = (crc32(0, tmp_env2->data, ENV_SIZE) == tmp_env2->crc);

	if (!crc1_ok && !crc2_ok)
		gd->env_valid = 0;
	else if(crc1_ok && !crc2_ok)
		gd->env_valid = 1;
	else if(!crc1_ok && crc2_ok)
		gd->env_valid = 2;
	else {
	#ifdef CONFIG_REDUNDAND_ENVIRONMENT
		/* both ok - check serial */
		if(tmp_env1->flags == 255 && tmp_env2->flags == 0)
			gd->env_valid = 2;
		else if(tmp_env2->flags == 255 && tmp_env1->flags == 0)
			gd->env_valid = 1;
		else if(tmp_env1->flags > tmp_env2->flags)
			gd->env_valid = 1;
		else if(tmp_env2->flags > tmp_env1->flags)
			gd->env_valid = 2;
		else /* flags are equal - almost impossible */
	#endif
			gd->env_valid = 1;
	}

	if (gd->env_valid == 1)
		env_ptr = tmp_env1;
	else if (gd->env_valid == 2)
		env_ptr = tmp_env2;
#else /* ENV_IS_EMBEDDED */
	gd->env_addr  = (ulong)&default_environment[0];
	gd->env_valid = 1;
#endif /* ENV_IS_EMBEDDED */

	return (0);
}

#ifdef CMD_SAVEENV
int saveenv(void)
{
	int ret = 0;

	puts ("Writing to EMMC... ");
	env_crc_update();
	ret = emmc_read_write(CONFIG_ENV_OFFSET_START, (void *) env_ptr,
			      CONFIG_ENV_SIZE, 1);

	if (ret !=  CONFIG_ENV_SIZE)
		puts("error in saving environment\n");
	else
		puts("done\n");

	return ret;
}
#endif /* CMD_SAVEENV */
#ifdef CONFIG_ENV_OFFSET_REDUND
void env_relocate_spec (void)
{
#if !defined(ENV_IS_EMBEDDED)
	ulong total;
	int crc1_ok = 0, crc2_ok = 0;
	env_t *tmp_env1, *tmp_env2;

	total = CONFIG_ENV_SIZE;

	tmp_env1 = (env_t *) malloc(CONFIG_ENV_SIZE);
	tmp_env2 = (env_t *) malloc(CONFIG_ENV_SIZE);

	emmc_read_write(CONFIG_ENV_OFFSET_START, (void *) tmp_env1, total, 0);
	emmc_read_write(CONFIG_ENV_OFFSET_START, (void *) tmp_env2, total, 0);

	crc1_ok = (crc32(0, tmp_env1->data, ENV_SIZE) == tmp_env1->crc);
	crc2_ok = (crc32(0, tmp_env2->data, ENV_SIZE) == tmp_env2->crc);

	if(!crc1_ok && !crc2_ok)
		return set_default_env();
	else if(crc1_ok && !crc2_ok)
		gd->env_valid = 1;
	else if(!crc1_ok && crc2_ok)
		gd->env_valid = 2;
	else {
		/* both ok - check serial */
		if(tmp_env1->flags == 255 && tmp_env2->flags == 0)
			gd->env_valid = 2;
		else if(tmp_env2->flags == 255 && tmp_env1->flags == 0)
			gd->env_valid = 1;
		else if(tmp_env1->flags > tmp_env2->flags)
			gd->env_valid = 1;
		else if(tmp_env2->flags > tmp_env1->flags)
			gd->env_valid = 2;
		else /* flags are equal - almost impossible */
			gd->env_valid = 1;

	}

	free(env_ptr);
	if(gd->env_valid == 1) {
		env_ptr = tmp_env1;
		free(tmp_env2);
	} else {
		env_ptr = tmp_env2;
		free(tmp_env1);
	}

#endif /* ! ENV_IS_EMBEDDED */
}
#else /* ! CONFIG_ENV_OFFSET_REDUND */
void env_relocate_spec (void)
{
#if !defined(ENV_IS_EMBEDDED)
	int ret;

	ret = emmc_read_write(CONFIG_ENV_OFFSET_START,
			      (void *)env_ptr, CONFIG_ENV_SIZE, 0);

	if (ret != CONFIG_ENV_SIZE) {
		printf("env read  failed so setting default env\n");
		goto misc;
	}

	if (crc32(0, env_ptr->data, ENV_SIZE) != env_ptr->crc) {
		printf("env crc failed so setting default env\n");
		goto misc;
	}
	return;

misc:
	return set_default_env();
#endif /* ! ENV_IS_EMBEDDED */
}
#endif /* CONFIG_ENV_OFFSET_REDUND */

#endif /* CONFIG_ENV_IS_IN_EMMC */

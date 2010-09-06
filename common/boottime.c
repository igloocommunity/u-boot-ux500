/*
 * Copyright (C) ST-Ericsson SA 2009-2010
 * Jonas Aaberg <jonas.aberg@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <common.h>
#include <boottime.h>
#include <asm/byteorder.h>
#include <asm/setup.h>

#ifdef CONFIG_BOOTTIME

static struct tag_boottime boottime = {
	.idle = 0,
	.total = 0,
};

void boottime_tag(char *name)
{
	if (boottime.num == BOOTTIME_MAX) {
		printf("boottime: out of entries!\n");
		return;
	}

	strncpy((char *)boottime.entry[boottime.num].name,
		name,
		BOOTTIME_MAX_NAME_LEN);
	boottime.entry[boottime.num].name[BOOTTIME_MAX_NAME_LEN - 1] = '\0';
	boottime.entry[boottime.num].time = get_timer_us();

	boottime.num++;
}

struct boottime_entry *boottime_get_entry(unsigned int i)
{
	if (i >= boottime.num)
		return NULL;
	else
		return &boottime.entry[i];
}

void boottime_idle_add(unsigned long time)
{
	boottime.idle += time;
}

unsigned long boottime_idle_done(void)
{
	return get_timer_us();
}

unsigned long boottime_idle_get(void)
{
	return boottime.idle;
}

static int do_boottime(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned int i;
	struct boottime_entry *entry;

	for (i = 0; i < BOOTTIME_MAX; i++) {
		entry = boottime_get_entry(i);
		if (entry == NULL)
			break;
		printf("%s: started at %lu ms\n", entry->name,
		       (unsigned long)entry->time / 1000);
	}
	printf("idle: %d%% (%d ms)\n",
	       100 * (int)boottime_idle_get() / (int)get_timer_us(),
	       (int)boottime_idle_get() / 1000);
	return 0;
}

static int do_boottime_tag (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}
	boottime_tag(argv[1]);

	return 0;
}

U_BOOT_CMD(
	boottime,	1,      1,      do_boottime,
	"print boottime info",
	""
	"    - print boottime tags\n"
);

U_BOOT_CMD(
	boottime_tag,	2,      1,      do_boottime_tag,
	"boottime tag 'name'",
	""
	"    - Add a boottime tag at the current time\n"
);

#else
/*
 * Dummy functions when boottime is not enabled.
 */
static int do_boottime_tag (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return 0;
}

void boottime_tag(char *name)
{

}

void boottime_idle_add(unsigned long time)
{

}

U_BOOT_CMD(
	boottime_tag,	2,      1,      do_boottime_tag,
	"boottime tag 'name'",
	""
	"    - NOT ENABLED: Add CONFIG_BOOTIME to your boards configuration"
	" file to enable boottime.\n"
);

#endif




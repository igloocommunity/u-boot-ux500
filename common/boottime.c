/*
 * Copyright (C) 2009-2010 ST-Ericsson AB
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

static struct tag_boottime boottime = {
	.idle = 0,
	.total = 0,
};

int boottime_tag(char *name)
{
	if (boottime.num == BOOTTIME_MAX) {
		printf("boottime: out of entries!\n");
		return -1;
	}

	strncpy((char *)boottime.entry[boottime.num].name,
		name,
		BOOTTIME_MAX_NAME_LEN);
	boottime.entry[boottime.num].name[BOOTTIME_MAX_NAME_LEN - 1] = '\0';
	boottime.entry[boottime.num].time = get_timer_us();

	boottime.num++;
	return 0;
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

void boottime_remove_last(void)
{
	if (boottime.num > 0)
		boottime.num--;
}





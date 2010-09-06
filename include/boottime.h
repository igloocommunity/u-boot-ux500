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

#ifndef BOOTTIME_H
#define BOOTTIME_H

#define BOOTTIME_MAX_NAME_LEN 64

struct boottime_entry {
	u32 time; /* in us */
	u8  name[BOOTTIME_MAX_NAME_LEN];
};

/**
 * boottime_tag()
 * Add a sample point with a name now. Shall be called before function "name"
 * is executed.
 * @name: Sample point name.
 */
void boottime_tag(char *name);

/**
 * boottime_get_entry()
 *
 * Loads a boottime measure point information.
 * @i: boottime measurement point entry.
 *
 * Returns a boottime entry. NULL, if not existing.
 */
struct boottime_entry *boottime_get_entry(unsigned int i);

/**
 * boottime_idle_get()
 *
 * Returns the amount of time in us that has been spent idling.
 */
unsigned long boottime_idle_get(void);

/**
 * boottime_idle_done()
 *
 * Returns the total time since start in us.
 */
unsigned long boottime_idle_done(void);

/**
 * boottime_idle_add()
 *
 * This function shall be added to all delay() functions.
 * The delay time input to delay() shall be provided to this
 * function as well. It is used to calculate average load
 * during boot.
 * @time: time in us.
 */
void boottime_idle_add(unsigned long time);

#endif

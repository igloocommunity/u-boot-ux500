/*
 * Copyright (C) 2010 ST-Ericsson SA
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com>
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

#include <common.h>
#include <i2c.h>
#include <tc35892.h>

#define GPIODATA0	0xC0
#define GPIODATA1	0xC2
#define GPIODATA2	0xC4
#define GPIODIR0	0xC6
#define GPIODIR1	0xC7
#define GPIODIR2	0xC8

int tc35892_gpio_dir(int addr, int pin, int dir)
{
	uchar reg = GPIODIR0 + pin / 8;
	uchar pos = pin % 8;
	uchar data;
	int ret;

	ret = i2c_read(addr, reg, sizeof(reg), &data, sizeof(data));
	if (ret)
		return ret;

	if (dir)
		data |= (1 << pos);
	else
		data &= ~(1 << pos);

	return i2c_write(addr, reg, sizeof(reg), &data, sizeof(data));
}

int tc35892_gpio_get(int addr, int pin, int val)
{
	uchar reg = GPIODATA0 + (pin / 8) * 2;
	uchar pos = pin % 8;
	uchar data;
	int ret;

	ret = i2c_read(addr, reg, sizeof(reg), &data, sizeof(data));
	if (ret < 0)
		return ret;

	return !!(data & (1 << pos));
}

int tc35892_gpio_set(int addr, int pin, int val)
{
	uchar reg = GPIODATA0 + (pin / 8) * 2;
	uchar pos = pin % 8;
	uchar data[] = {val << pos, 1 << pos};

	return i2c_write(addr, reg, sizeof(reg), data, sizeof(data));
}

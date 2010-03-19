/*
 * Copyright (C) 2010 ST-Ericsson SA
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

#ifndef __TC35892_H__
#define __TC35892_H__

#define TC35892_PIN_KPX0	0
#define TC35892_PIN_KPX1	1
#define TC35892_PIN_KPX2	2
#define TC35892_PIN_KPX3	3
#define TC35892_PIN_KPX4	4
#define TC35892_PIN_KPX5	5
#define TC35892_PIN_KPX6	6
#define TC35892_PIN_KPX7	7
#define TC35892_PIN_KPY0	8
#define TC35892_PIN_KPY1	9
#define TC35892_PIN_KPY2	10
#define TC35892_PIN_KPY3	11
#define TC35892_PIN_KPY4	12
#define TC35892_PIN_KPY5	13
#define TC35892_PIN_KPY6	14
#define TC35892_PIN_KPY7	15
#define TC35892_PIN_KPY8	16
#define TC35892_PIN_KPY9	17
#define TC35892_PIN_KPY10	18
#define TC35892_PIN_KPY11	19
#define TC35892_PIN_PWM0	20
#define TC35892_PIN_PWM1	21
#define TC35892_PIN_PWM2	22
#define TC35892_PIN_EXTIO	23

int tc35892_gpio_dir(int addr, int pin, int dir);
int tc35892_gpio_set(int addr, int pin, int val);
int tc35892_gpio_get(int addr, int pin, int val);

#endif

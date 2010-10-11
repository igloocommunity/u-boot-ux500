/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * based on Linux drivers/rtc/rtc-pl031.c:
 *
 * Real Time Clock interface for ARM AMBA PrimeCell 031 RTC
 *
 * Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * Copyright 2006 (c) MontaVista Software, Inc.
 *
 * Author: Mian Yousaf Kaukab <mian.yousaf.kaukab@stericsson.com>
 * Copyright 2010 (c) ST-Ericsson AB
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <rtc.h>

#if defined(CONFIG_CMD_DATE)

#ifndef CONFIG_SYS_RTC_PL031_BASE
#error CONFIG_SYS_RTC_PL031_BASE is not defined!
#endif

/*
 * Register definitions
 */
#define	RTC_DR		0x00	/* Data read register */
#define	RTC_MR		0x04	/* Match register */
#define	RTC_LR		0x08	/* Data load register */
#define	RTC_CR		0x0c	/* Control register */
#define	RTC_IMSC	0x10	/* Interrupt mask and set register */
#define	RTC_RIS		0x14	/* Raw interrupt status register */
#define	RTC_MIS		0x18	/* Masked interrupt status register */
#define	RTC_ICR		0x1c	/* Interrupt clear register */
/* ST variants have additional timer functionality */
#define RTC_TDR		0x20	/* Timer data read register */
#define RTC_TLR		0x24	/* Timer data load register */
#define RTC_TCR		0x28	/* Timer control register */
#define RTC_YDR		0x30	/* Year data read register */
#define RTC_YMR		0x34	/* Year match register */
#define RTC_YLR		0x38	/* Year data load register */

#define RTC_CR_CWEN	(1 << 26)	/* Clockwatch enable bit */

/* Common bit definations for ST v2 for reading/writing time */
#define RTC_SEC_SHIFT 0
#define RTC_SEC_MASK (0x3F << RTC_SEC_SHIFT) /* Second [0-59] */
#define RTC_MIN_SHIFT 6
#define RTC_MIN_MASK (0x3F << RTC_MIN_SHIFT) /* Minute [0-59] */
#define RTC_HOUR_SHIFT 12
#define RTC_HOUR_MASK (0x1F << RTC_HOUR_SHIFT) /* Hour [0-23] */
#define RTC_WDAY_SHIFT 17
#define RTC_WDAY_MASK (0x7 << RTC_WDAY_SHIFT) /* Day of Week [1-7] 1=Sunday */
#define RTC_MDAY_SHIFT 20
#define RTC_MDAY_MASK (0x1F << RTC_MDAY_SHIFT) /* Day of Month [1-31] */
#define RTC_MON_SHIFT 25
#define RTC_MON_MASK (0xF << RTC_MON_SHIFT) /* Month [1-12] 1=January */

static int pl031_initted = 0;

/*
 * Convert Gregorian date to ST v2 RTC format.
 */
static int pl031_stv2_tm_to_time(struct rtc_time *tm, unsigned long *st_time,
	unsigned long *bcd_year)
{
	int year = tm->tm_year;
	int wday = tm->tm_wday;

	/* wday masking is not working in hardware so wday must be valid */
	if (wday < -1 || wday > 6) {
		printf("invalid wday value %d\n", tm->tm_wday);
		return -1;
	} else if (wday == -1) {
		/* wday is not provided, calculate it here */
		GregorianDay(tm);
		wday = tm->tm_wday;
	}

	*bcd_year = (bin2bcd(year % 100) | bin2bcd(year / 100) << 8);

	*st_time = (tm->tm_mon << RTC_MON_SHIFT)
			|	(tm->tm_mday << RTC_MDAY_SHIFT)
			|	((wday + 1) << RTC_WDAY_SHIFT)
			|	(tm->tm_hour << RTC_HOUR_SHIFT)
			|	(tm->tm_min << RTC_MIN_SHIFT)
			|	(tm->tm_sec << RTC_SEC_SHIFT);

	return 0;
}

/*
 * Convert ST v2 RTC format to Gregorian date.
 */
static int pl031_stv2_time_to_tm(unsigned long st_time, unsigned long bcd_year,
	struct rtc_time *tm)
{
	tm->tm_year = bcd2bin(bcd_year) + (bcd2bin(bcd_year >> 8) * 100);
	tm->tm_mon  = ((st_time & RTC_MON_MASK) >> RTC_MON_SHIFT);
	tm->tm_mday = ((st_time & RTC_MDAY_MASK) >> RTC_MDAY_SHIFT);
	tm->tm_wday = ((st_time & RTC_WDAY_MASK) >> RTC_WDAY_SHIFT) - 1;
	tm->tm_hour = ((st_time & RTC_HOUR_MASK) >> RTC_HOUR_SHIFT);
	tm->tm_min  = ((st_time & RTC_MIN_MASK) >> RTC_MIN_SHIFT);
	tm->tm_sec  = ((st_time & RTC_SEC_MASK) >> RTC_SEC_SHIFT);

	return 0;
}

static int pl031_stv2_read_time(struct rtc_time *tm)
{

	pl031_stv2_time_to_tm(readl(CONFIG_SYS_RTC_PL031_BASE + RTC_DR),
			readl(CONFIG_SYS_RTC_PL031_BASE + RTC_YDR), tm);

	return 0;
}

static int pl031_stv2_set_time(struct rtc_time *tm)
{
	unsigned long time;
	unsigned long bcd_year;
	int ret;

	ret = pl031_stv2_tm_to_time(tm, &time, &bcd_year);
	if (ret == 0) {
		writel(bcd_year, CONFIG_SYS_RTC_PL031_BASE + RTC_YLR);
		writel(time, CONFIG_SYS_RTC_PL031_BASE + RTC_LR);
	}
	/*
	 * The new setting is transferred to the ClockWatch counters on the
	 * next CLK1HZ rising edge after RTC_CWDLR register has been written.
	 */
	udelay(1000 * 1000);

	return ret;
}

/* Enable RTC Start in Control register*/
void rtc_init(void)
{
	writel(readl(CONFIG_SYS_RTC_PL031_BASE + RTC_CR) | RTC_CR_CWEN,
			CONFIG_SYS_RTC_PL031_BASE + RTC_CR);

	pl031_initted = 1;
}

/*
 * Reset the RTC. We set the date back to 1970-01-01.
 */
void rtc_reset(void)
{
	if (!pl031_initted)
		rtc_init();
	/* POR value, 2000-01-01 Sun */
	writel(0x02120000, CONFIG_SYS_RTC_PL031_BASE + RTC_LR);
	writel(0x2000, CONFIG_SYS_RTC_PL031_BASE + RTC_YLR);
	/*
	 * The new setting is transferred to the ClockWatch counters on the
	 * next CLK1HZ rising edge after RTC_CWDLR register has been written.
	 */
	udelay(1000 * 1000);
}

/*
 * Set the RTC
 */
int rtc_set(struct rtc_time *tm)
{

	if (!pl031_initted)
		rtc_init();

	if (tm == NULL) {
		puts("Error setting the date/time\n");
		return -1;
	}

	return pl031_stv2_set_time(tm);
}


/*
 * Get the current time from the RTC
 */
int rtc_get(struct rtc_time *tm)
{

	if (!pl031_initted)
		rtc_init();

	if (tm == NULL) {
		puts("Error getting the date/time\n");
		return -1;
	}

	pl031_stv2_read_time(tm);

	debug("Get DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
		tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}
#endif

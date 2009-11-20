/*
 * (C) Copyright 2009 Alessandro Rubini
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
#include <asm/io.h>
#include <asm/arch/mtu.h>
#include <asm/boottime.h>

/*
 * The timer is a decrementer, we'll left it free running at 2.4MHz.
 * We have 2.4 ticks per microsecond and an overflow in almost 30min
 *
 * EABEJON: On HREF (atleast) there is no way the timer runs at 2.4MHz
 * It is more likely that it around ~100 MHz with 1 as perscaler.
 * Changing the perscaler setting to 16 gives a timer decrease rate of
 * ~6.25MHz.
 *
 * Use the 3rd counter on MTU0 and let it run free since we're interested
 * in how long time it takes to boot uboot+linux. Linux ux500 uses MTU0, 
 * timer0 and MTU1, timer0.
 *
 */

#if 0
#define TIMER_CLOCK		(24 * 100 * 1000)
#define COUNT_TO_USEC(x)	((x) * 5 / 12)	/* overflows at 6min */
#define USEC_TO_COUNT(x)	((x) * 12 / 5)	/* overflows at 6min */
#endif

#define TIMER_CLOCK		(625 * 10 * 1000)
#define COUNT_TO_USEC(x)	((x) * 4 / 25)
#define USEC_TO_COUNT(x)	((x) * 25 / 4)

#define TICKS_PER_HZ		(TIMER_CLOCK / CONFIG_SYS_HZ)
#define TICKS_TO_HZ(x)		((x) / TICKS_PER_HZ)

/* Timer on MTU0 (from 0 to 3) */
#define MTU_TIMER 2


/* macro to read the 32 bit timer: since it decrements, we invert read value */
#define READ_TIMER() (~readl(CONFIG_SYS_TIMERBASE + MTU_VAL(MTU_TIMER)))

/* Configure a free-running, auto-wrap counter with division by 16 as prescaler */
int timer_init(void)
{
	writel(MTU_CRn_ENA | MTU_CRn_PRESCALE_16 | MTU_CRn_32BITS,
	       CONFIG_SYS_TIMERBASE + MTU_CR(MTU_TIMER));
	reset_timer();
	boottime_tag("uboot_init");
	return 0;
}

/* Restart counting from 0 */
void reset_timer(void)
{
	writel(0, CONFIG_SYS_TIMERBASE + MTU_LR(MTU_TIMER)); /* Immediate effect */
}

/* Return how many HZ passed since "base" */
ulong get_timer(ulong base)
{
	return  TICKS_TO_HZ(READ_TIMER()) - base;
}


/* Return how many HZ passed since "base" */
ulong get_raw_timer(void)
{
	return  READ_TIMER();
}


/* Delay x useconds */
void udelay(unsigned long usec)
{
	ulong ini, end;

	ini = READ_TIMER();
	end = ini + USEC_TO_COUNT(usec);
	while ((signed)(end - READ_TIMER()) > 0)
		;
	boottime_idle_add(USEC_TO_COUNT(usec));
}

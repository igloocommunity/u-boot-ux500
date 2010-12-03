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
#include <asm/arch/common.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <boottime.h>

/*
 * The MTU device hosts four different counters, with 4 set of
 * registers. These are register names.
 */

#define MTU_IMSC	0x00	/* Interrupt mask set/clear */
#define MTU_RIS		0x04	/* Raw interrupt status */
#define MTU_MIS		0x08	/* Masked interrupt status */
#define MTU_ICR		0x0C	/* Interrupt clear register */

/* per-timer registers take 0..3 as argument */
#define MTU_LR(x)	(0x10 + 0x10 * (x) + 0x00)	/* Load value */
#define MTU_VAL(x)	(0x10 + 0x10 * (x) + 0x04)	/* Current value */
#define MTU_CR(x)	(0x10 + 0x10 * (x) + 0x08)	/* Control reg */
#define MTU_BGLR(x)	(0x10 + 0x10 * (x) + 0x0c)	/* At next overflow */

/* bits for the control register */
#define MTU_CRn_ENA		0x80
#define MTU_CRn_PERIODIC	0x40	/* if 0 = free-running */
#define MTU_CRn_PRESCALE_MASK	0x0c
#define MTU_CRn_PRESCALE_1		0x00
#define MTU_CRn_PRESCALE_16		0x04
#define MTU_CRn_PRESCALE_256		0x08
#define MTU_CRn_32BITS		0x02
#define MTU_CRn_ONESHOT		0x01	/* if 0 = wraps reloading from BGLR*/

/* Other registers are usual amba/primecell registers, currently not used */
#define MTU_ITCR	0xff0
#define MTU_ITOP	0xff4

#define MTU_PERIPH_ID0	0xfe0
#define MTU_PERIPH_ID1	0xfe4
#define MTU_PERIPH_ID2	0xfe8
#define MTU_PERIPH_ID3	0xfeC

#define MTU_PCELL0	0xff0
#define MTU_PCELL1	0xff4
#define MTU_PCELL2	0xff8
#define MTU_PCELL3	0xffC

/*
 * The MTU is clocked at 133 MHz by default. (V1 and later)
 */
#define TIMER_CLOCK		(133 * 1000 * 1000 / 16)
#define COUNT_TO_USEC(x)	((x) * 16 / 133)
#define USEC_TO_COUNT(x)	((x) * 133 / 16)
#define TICKS_PER_HZ		(TIMER_CLOCK / CONFIG_SYS_HZ)
#define TICKS_TO_HZ(x)		((x) / TICKS_PER_HZ)

/*
 * MTU timer to use (from 0 to 3).
 * Linux ux500 timer0 on MTU0 and timer0 on MTU1
 */
#define MTU_TIMER 2

static unsigned int timerbase;

/* macro to read the 32 bit timer: since it decrements, we invert read value */
#define READ_TIMER() (~readl(timerbase + MTU_VAL(MTU_TIMER)))

/* Configure a free-running, auto-wrap counter with /16 prescaler */
int timer_init(void)
{
	timerbase = u8500_is_earlydrop() ? U8500_MTU0_BASE_ED
			: U8500_MTU0_BASE_V1;

	writel(MTU_CRn_ENA | MTU_CRn_PRESCALE_16 | MTU_CRn_32BITS,
	       timerbase + MTU_CR(MTU_TIMER));
	reset_timer();
	return 0;
}

/* Restart counting from 0 */
void reset_timer(void)
{
	writel(0, timerbase + MTU_LR(MTU_TIMER)); /* Immediate effect */
}

/* Return how many HZ passed since "base" */
ulong get_timer(ulong base)
{
	return  TICKS_TO_HZ(READ_TIMER()) - base;
}

u64 get_timer_us(void)
{
	return  COUNT_TO_USEC(READ_TIMER());
}

/* Delay x useconds */
void udelay(unsigned long usec)
{
	ulong ini, end;

	ini = READ_TIMER();
	end = ini + USEC_TO_COUNT(usec);
	while ((signed)(end - READ_TIMER()) > 0)
		;
	boottime_idle_add(usec);
}

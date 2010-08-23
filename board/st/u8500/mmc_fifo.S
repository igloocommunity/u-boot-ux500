/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Martin Lundholm <martin.xa.lundholm@stericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include "mmc_fifo.h"
#include "mmc_host.h"

/*
 * Function: mmc_fifo_read()
 *
 * int mmc_fifo_read(u32 *fifo, u32 *buf, unsigned int count, u32 *status_reg)
 *
 * Info: Reads data from an MMC (ARM PL180) FIFO
 *
 * Parameters:
 *	fifo 	   - pointer to the first PL180 FIFO register
 *	buf 	   - pointer to a read buffer (32-bit aligned)
 *	count 	   - number of bytes to be read (32-bit aligned)
 *	status_reg - pointer to the PL180 status register
 *
 * Returns '0' if success and PL180 status on failure.
 *
 */

	.globl mmc_fifo_read
mmc_fifo_read:
	push	{r4-r10,lr}
mmc_fifo_read_loop_32_1:
	/* If count is <32B read word-wise */
	cmp	r2,#32
	blo	mmc_fifo_read_loop_4_1
mmc_fifo_read_loop_32_2:
	/* Load SDI_STA to r4 */
	ldr	r4,[r3]
	/* Exit if SDI_STA_DCRCFAIL or SDI_STA_DTIMEOUT is set */
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_read_fail
	/* Wait until SDI_STA_RXFIFOBR is set */
	tst	r4,#SDI_STA_RXFIFOBR
	beq	mmc_fifo_read_loop_32_2
	/* Load and store 8 words */
	ldmia	r0,{r4-r10,lr}
	stmia	r1!,{r4-r10,lr}
	subs	r2,r2,#32
	b	mmc_fifo_read_loop_32_1
mmc_fifo_read_loop_4_1:
	/* Read word wise */
	cmp	r2,#4
	blo	mmc_fifo_read_ok
mmc_fifo_read_loop_4_2:
	/* Load SDI_STA to r4 */
	ldr	r4,[r3]
	/* Exit if SDI_STA_DCRCFAIL or SDI_STA_DTIMEOUT is set */
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_read_fail
	/* Wait until SDI_STA_RXDAVL is set */
	tst	r4,#SDI_STA_RXDAVL
	beq	mmc_fifo_read_loop_4_2
	/* Load and store 1 word */
	ldmia	r0,{r4}
	stmia	r1!,{r4}
	subs	r2,r2,#4
	b	mmc_fifo_read_loop_4_1
mmc_fifo_read_ok:
	/* Wait until SDI_STA_DBCKEND and SDI_STA_DATAEND are set */
	ldr	r4,[r3]
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_read_fail
	and	r5,r4,#(SDI_STA_DBCKEND | SDI_STA_DATAEND)
	cmp	r5,#(SDI_STA_DBCKEND | SDI_STA_DATAEND)
	bne	mmc_fifo_read_ok
mmc_fifo_read_fail:
	mov	r0,r4
	pop	{r4-r10,pc}

/*
 * Function: mmc_fifo_write()
 *
 * int mmc_fifo_write(u32 *buf, u32 *fifo, unsigned int count, u32 *status_reg)
 *
 * Info: Writes data to an MMC (ARM PL180) FIFO
 *
 * Parameters:
 *	buf 	   - pointer to a write buffer (32-bit aligned)
 *	fifo 	   - pointer to the first PL180 FIFO register
 *	count 	   - number of bytes to be written (32-bit aligned)
 *	status_reg - pointer to the PL180 status register
 *
 * Returns '0' if success and PL180 status on failure.
 *
 */

	.globl mmc_fifo_write
mmc_fifo_write:
	push	{r4-r10,lr}
mmc_fifo_write_loop_32_1:
	/* If count is <32B read word-wise */
	cmp	r2,#32
	blo	mmc_fifo_write_loop_4_1
mmc_fifo_write_loop_32_2:
	/* Load SDI_STA to r4 */
	ldr	r4,[r3]
	/* Exit if SDI_STA_DCRCFAIL or SDI_STA_DTIMEOUT is set */
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_write_fail
	/* Wait until SDI_STA_TXFIFOBW is set */
	tst	r4,#SDI_STA_TXFIFOBW
	beq	mmc_fifo_write_loop_32_2
	/* Load and store 8 words */
	ldmia	r0!,{r4-r10,lr}
	stmia	r1,{r4-r10,lr}
	subs	r2,r2,#32
	b	mmc_fifo_write_loop_32_1
mmc_fifo_write_loop_4_1:
	/* Read word wise */
	cmp	r2,#4
	blo	mmc_fifo_write_ok
mmc_fifo_write_loop_4_2:
	/* Load SDI_STA to r4 */
	ldr	r4,[r3]
	/* Exit if SDI_STA_DCRCFAIL or SDI_STA_DTIMEOUT is set */
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_write_fail
	/* Wait until SDI_STA_TXFIFOBW is set */
	tst	r4,#SDI_STA_TXFIFOBW
	beq	mmc_fifo_write_loop_4_2
	/* Load and store 1 word */
	ldmia	r0!,{r4}
	stmia	r1,{r4}
	subs	r2,r2,#4
	b	mmc_fifo_write_loop_4_1
mmc_fifo_write_ok:
	/* Wait until SDI_STA_DBCKEND and SDI_STA_DATAEND are set */
	ldr	r4,[r3]
	ands	r5,r4,#(SDI_STA_DCRCFAIL | SDI_STA_DTIMEOUT)
	bne	mmc_fifo_write_fail
	and	r5,r4,#(SDI_STA_DBCKEND | SDI_STA_DATAEND)
	cmp	r5,#(SDI_STA_DBCKEND | SDI_STA_DATAEND)
	bne	mmc_fifo_write_ok
mmc_fifo_write_fail:
	mov	r0,r4
	pop	{r4-r10,pc}

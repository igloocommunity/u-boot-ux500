/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Martin Lundholm <martin.xa.lundholm@stericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <common.h>

#ifndef __ASSEMBLY__

/*
 * Function: mmc_fifo_read()
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
int mmc_fifo_read(u32 *fifo, u32 *buf, unsigned int count, u32 *status_reg);

/*
 * Function: mmc_fifo_write()
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
int mmc_fifo_write(u32 *buf, u32 *fifo, unsigned int count, u32 *status_reg);

#endif /* __ASSEMBLY__ */

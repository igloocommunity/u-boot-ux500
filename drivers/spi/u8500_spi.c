/*
* Copyright (C) ST-Ericsson SA 2010
* Author: Markus Helgesson <Markus.Helgesson@stericsson.com>
* for ST-Ericsson.
* License terms: GNU General Public License (GPL), version 2.
*
* This driver is influenced by spi-stm.c and stm_spi023.c
* in the linux kernel.
*/

#include <common.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/hardware.h>
#include <spi.h>


#ifndef CONFIG_U8500
#error "This driver currently only supports the u8500 platform. Sorry."
#endif


/*#######################################################################
  Print macros
#########################################################################
*/
#define pr(pre, str, ...)\
	printf(pre "[%s {%u}] " str "\n", __func__, __LINE__, ##__VA_ARGS__)
#define pr_err(str, ...)	pr("E", str, ##__VA_ARGS__)
#define pr_warn(str, ...)	pr("W", str, ##__VA_ARGS__)

#define dpr(pre, str, ...)\
	debug(pre "[%s {%u}] " str "\n", __func__, __LINE__, ##__VA_ARGS__)
#define pr_info(str, ...)	dpr("I", str, ##__VA_ARGS__)
#define pr_dbg(str, ...)	dpr("D", str, ##__VA_ARGS__)


/*#######################################################################
  Register locations
#########################################################################
*/
#ifdef U8500_SPI0_BASE
#define SPI_0_BASE	U8500_SPI0_BASE
#endif

#ifdef U8500_SPI1_BASE
#define SPI_1_BASE	U8500_SPI1_BASE
#endif

#ifdef U8500_SPI2_BASE
#define SPI_2_BASE	U8500_SPI2_BASE
#endif

#ifdef U8500_SPI3_BASE
#define SPI_3_BASE	U8500_SPI3_BASE
#endif


/*#######################################################################
  Macros to access SPI Registers with their offsets
#########################################################################
*/
#define SPI_CR0(r)	((u8 *)r + 0x000)
#define SPI_CR1(r)	((u8 *)r + 0x004)
#define SPI_DR(r)	((u8 *)r + 0x008)
#define SPI_SR(r)	((u8 *)r + 0x00C)
#define SPI_CPSR(r)	((u8 *)r + 0x010)

/*#######################################################################
  SPI Control Register 0  - SPI_CR0
#########################################################################
*/
#define SPI_CR0_MASK_DSS		((u32)(0x1FUL << 0))
#define SPI_CR0_MASK_SPO		((u32)(0x1UL << 6))
#define SPI_CR0_MASK_SPH		((u32)(0x1UL << 7))
#define SPI_CR0_MASK_SCR		((u32)(0xFFUL << 8))

/*#######################################################################
  SPI Control Register 0  - SPI_CR1
#########################################################################
*/
#define SPI_CR1_MASK_SSE		((u32)(0x1UL << 1))
#define SPI_CR1_MASK_RENDN		((u32)(0x1UL << 4))
#define SPI_CR1_MASK_TENDN		((u32)(0x1UL << 5))

/*#######################################################################
  SPI Status Register  - SPI_sr
#########################################################################
*/
#define SPI_SR_MASK_TNF			((u32)(0x1UL << 1))
#define SPI_SR_MASK_RNE			((u32)(0x1UL << 2))
#define SPI_SR_MASK_BSY			((u32)(0x1UL << 4))

/*#######################################################################
  SPI Clock Prescale Register  - SPI_cpsr
#########################################################################
*/
#define SPI_CPSR_MASK_CPSDVSR		((u32)(0xFFUL << 0))

/*#######################################################################
  SPI Clock Parameter ranges
#########################################################################
*/
#define MIN_CPSDVR 0x02
#define MAX_CPSDVR 0xFE
#define MIN_SCR 0x00
#define MAX_SCR 0xFF
#define STM_SPI_CLOCK_FREQ 48000000
#define STM_SPICTLR_CLOCK_FREQ 48000000

/*#######################################################################
  Other macros
#########################################################################
*/
#define SPI_REG_WRITE_BITS(reg, val, mask, sb) \
		((reg) =   (((reg) & ~(mask)) | (((val)<<(sb)) & (mask))))

#define ENABLE_SPI_CONTROLLER(sss)\
	(writel((readl(SPI_CR1(sss->base_addr)) | SPI_CR1_MASK_SSE),\
		SPI_CR1(sss->base_addr)))

#define DISABLE_SPI_CONTROLLER(sss)\
	(writel((readl(SPI_CR1(sss->base_addr)) & (~SPI_CR1_MASK_SSE)),\
		SPI_CR1(sss->base_addr)))

#define FLUSH_SPI_RX_FIFO(sss)\
	do {\
		while (readl(SPI_SR(sss->base_addr)) & SPI_SR_MASK_RNE)\
			readl(SPI_DR(sss->base_addr)) ;\
	} while (readl(SPI_SR(sss->base_addr)) & SPI_SR_MASK_BSY)

#define WAIT_FOR_SPI_CONTROLLER_TO_FINISH(sss)\
	while (readl(SPI_SR(sss->base_addr)) & SPI_SR_MASK_BSY)\
		udelay(1)

/*#######################################################################
  Structures
#########################################################################
*/
struct spi_regs {
	u32 cr0;
	u32 cr1;
	u32 cpsr;
};

struct stm_spi_slave {
	struct spi_slave	slave;
	u8			*base_addr;
	u8			n_bytes;
	struct spi_regs		regs;
	char			dev_name[4];
};

/* Clock parameters, to set SPI clock at a desired freq */
struct spi_clock_params {
	u8 cpsdvsr;     /* value from 2 to 254 (even only!) */
	u8 scr;         /* value from 0 to 255 */
};

struct spi_transfer {
	u8	*current;
	u8	*end;
	u8	dummy_transfer;
};

/*#######################################################################
  Local function declarations
#########################################################################
*/

/*
 * to_stm_spi_slave - Converts spi_slave ptr to struct stm_spi_slave ptr
 */
static inline struct stm_spi_slave *to_stm_spi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct stm_spi_slave, slave);
}

/*
 * spi_write - Write FIFO data in Data register
 *
 * This function writes data in Tx FIFO till it is not full
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary write ptr trans_data->current
 * which maintains current write position in transfer buffer
 */
static void
spi_write(struct stm_spi_slave *sss, struct spi_transfer * trans_data);

 /*
 * spi_read - Read FIFO data in Data register
 *
 * This function reads data in Rx FIFO till it is not empty
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary Read ptr trans_data->current
 * which maintains current read position in transfer buffer
 */
static void
spi_read(struct stm_spi_slave *sss, struct spi_transfer * trans_data);

 /*
  * spi023_eff_freq - Calculates the frequency register values
  */
static int spi023_eff_freq(int freq, struct spi_clock_params *clk_freq);


/*#######################################################################
  Global function definitions (declared in spi.h)
#########################################################################
*/

void spi_init()
{
	pr_dbg("");
	/* nothing to do here */
}

struct spi_slave *spi_setup_slave(u32 bus, u32 cs, u32 max_hz, u32 mode)
{
	struct stm_spi_slave	*sss;
	u8			*base_addr;
	u8			data_size;
	struct spi_clock_params	clk_freq;

	pr_dbg("bus %u, cs %u, max_hz %u, mode 0x%x", bus, cs, max_hz, mode);

	if (!spi_cs_is_valid(bus, cs)) {
		pr_err("Invalid bus or cs in SPI configuration!");
		return NULL;
	}

	switch (bus) {
#ifdef SPI_0_BASE
	case 0:
		base_addr = (u8 *) SPI_0_BASE;
		break;
#endif
#ifdef SPI_1_BASE
	case 1:
		base_addr = (u8 *) SPI_1_BASE;
		break;
#endif
#ifdef SPI_2_BASE
	case 2:
		base_addr = (u8 *) SPI_2_BASE;
		break;
#endif
#ifdef SPI_3_BASE
	case 3:
		base_addr = (u8 *) SPI_3_BASE;
		break;
#endif
	default:
		pr_err("Unknown SPI controller for bus %u.", bus);
		return NULL;
	}

	clk_freq.scr = 0;
	clk_freq.cpsdvsr = 0;
	if (spi023_eff_freq(max_hz, &clk_freq))
		return NULL;

	pr_dbg("clk_freq.cpsdvsr = %u, clk_freq.scr = %u",
		clk_freq.cpsdvsr, clk_freq.scr);

	/*
	 * mode parameter bit definition:
	 *
	 * 0x0001	SPI_CPHA - clock phase
	 * 0x0002	SPI_CPOL - clock polarity
	 * 0x0004	NOT USED	(reserved for SPI_CS_HIGH)
	 * 0x0008	SPI_LSB_FIRST - per-word bits-on-wire
	 * 0x0010	NOT USED	(reserved for SPI_3WIRE)
	 * 0x0020	NOT USED	(reserved for SPI_LOOP)
	 * 0x03C0	NOT USED (bits 6--9)
	 * 0x7C00	Data size select (bits 10--14),
	 *		Size can be 4--32 bits and value is calculated as
	 *		(desired data size - 1) so
	 *			0 0011 - 4-bit data size
	 *			0 0100 - 5 bit data size
	 *			...
	 *			1 1111 - 32-bit data size
	 */
	data_size = ((mode & 0x7C00) >> 10) + 1;
	if ((data_size < 4) || (data_size > 32)) {
		pr_err("Data size %u bits is not supported.", data_size);
		return NULL;
	}

	sss = malloc(sizeof(struct stm_spi_slave));
	if (!sss) {
		pr_err("Failed to allocate struct. Out of memory?");
		return NULL;
	}

	sss->slave.bus = bus;
	sss->slave.cs = cs;
	sss->base_addr = base_addr;

	if (data_size > 16)
		sss->n_bytes = 4;
	else if (data_size > 8)
		sss->n_bytes = 2;
	else
		sss->n_bytes = 1;

	sss->regs.cr0 = 0;
	sss->regs.cr1 = 0;
	sss->regs.cpsr = 0;

	/* data size */
	SPI_REG_WRITE_BITS(
		sss->regs.cr0, (data_size-1), SPI_CR0_MASK_DSS, 0);

	/* clock polarity */
	if (mode & SPI_CPOL)
		SPI_REG_WRITE_BITS(sss->regs.cr0, 1, SPI_CR0_MASK_SPO, 6);

	/* clock phase */
	if (mode & SPI_CPHA)
		SPI_REG_WRITE_BITS(sss->regs.cr0, 1, SPI_CR0_MASK_SPH, 7);

	/* serial clock rate */
	SPI_REG_WRITE_BITS(sss->regs.cr0, clk_freq.scr, SPI_CR0_MASK_SCR, 8);

	/* endian format */
	if (mode & SPI_LSB_FIRST) {
		SPI_REG_WRITE_BITS(sss->regs.cr1, 1, SPI_CR1_MASK_RENDN, 4);
		SPI_REG_WRITE_BITS(sss->regs.cr1, 1, SPI_CR1_MASK_TENDN, 5);
	}

	/* clock prescale divisor */
	SPI_REG_WRITE_BITS(
		sss->regs.cpsr, clk_freq.cpsdvsr, SPI_CPSR_MASK_CPSDVSR, 0);

	pr_dbg("sss->slave.bus =  %u", sss->slave.bus);
	pr_dbg("sss->slave.cs =   %u", sss->slave.cs);
	pr_dbg("sss->base_addr =  0x%p", sss->base_addr);
	pr_dbg("sss->regs.cr0 =   0x%x", sss->regs.cr0);
	pr_dbg("sss->regs.cr1 =   0x%x", sss->regs.cr1);
	pr_dbg("sss->regs.cpsr =  0x%x", sss->regs.cpsr);

	return (struct spi_slave *) sss;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct stm_spi_slave *sss = to_stm_spi_slave(slave);
	pr_dbg("slave 0x%p", slave);

	free(sss);
}

int spi_claim_bus(struct spi_slave *slave)
{
	struct stm_spi_slave *sss = to_stm_spi_slave(slave);
	int ret = 0;

	pr_dbg("slave 0x%p", slave);

	/*
	 * 0. Enable the clock to the SPI controller and
	 *    configure controller specific information.
	 *
	 * TODO: The GPIO pins below are hardcoded for db8500.
	 */
	switch (sss->slave.bus) {
#if defined(SPI_0_BASE) && defined(CONFIG_U8500)
	case 0:
		u8500_clock_enable(2, 8, -1);
		break;
#endif
#if defined(SPI_1_BASE) && defined(CONFIG_U8500)
	case 1:
		u8500_clock_enable(2, 2, -1);
		break;
#endif
#if defined(SPI_2_BASE) && defined(CONFIG_U8500)
	case 2:
		u8500_clock_enable(2, 1, -1);

		/*
		 * For SPI2 on u8500 the GPIO's need to be set to
		 * Other Alt C1 by setting bit 23 in PRCM_GPIOCR
		 * (GPIO muxing control register)
		 *
		 * TODO: This should be done nicer through some driver...
		 */
		writel(readl(U8500_PRCMU_BASE + 0x138) | (1<<23),
		       (U8500_PRCMU_BASE + 0x138));
		break;
#endif
#if defined(SPI_3_BASE) && defined(CONFIG_U8500)
	case 3:
		u8500_clock_enable(1, 7, -1);
		break;
#endif
	default:
		pr_err("Unknown SPI controller for bus %u.", sss->slave.bus);
		return -1;
	}

	/* Sets the device name, for instance "SPI2" */
	strcpy(sss->dev_name, "SPIx");
	sss->dev_name[3] = ('0' + sss->slave.bus);


	/*
	 * 1. Clear SSE bit in SPI_CR1 register. This step is not required
	 *    after an hardware or software reset of the device.
	 *    The input/output pins are switched to GPIO input mode.
	 */
	DISABLE_SPI_CONTROLLER(sss);


	/*
	 * 2. Empty the receive FIFO. This step is not required after an
	 *    hardware or software reset of the device, as the FIFO pointer
	 *    will be cleared.
	 */
	FLUSH_SPI_RX_FIFO(sss);


	/*
	 * 3. Program the GPIO on which SPI port signals are attached to.
	 * This should already be done by the Board GPIO setup
	 */

	/*
	 * 4. Program the SPI clock prescale register, SPI_CPSR,
	 *   then the configuration registers SPI_CR0 and SPI_CR1.
	 */
	writel(sss->regs.cpsr, SPI_CPSR(sss->base_addr));
	writel(sss->regs.cr0, SPI_CR0(sss->base_addr));
	writel(sss->regs.cr1, SPI_CR1(sss->base_addr));


	/*
	 * The remaining steps are done during the transfer:
	 *
	 * 5. write at least one word to the TxFIFO.
	 * 6. The transmit FIFO can optionally be filled before
	 *    enabling the SPI.
	 * 7. Set SSE = 1b to enable the SPI operation.
	 */

	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	pr_dbg("slave 0x%p", slave);

	/* nothing to do here */
}


int spi_xfer(struct spi_slave *slave, u32 bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct stm_spi_slave	*sss = to_stm_spi_slave(slave);
	const u32		n_bytes = bitlen / 8;
	int			ret = 0;
	int			rx_max_tries;
	struct spi_transfer		tx;
	struct spi_transfer		rx;

	pr_dbg("slave 0x%p, bitlen %u, dout 0x%p, din 0x%p, flags 0x%lx",
		slave, bitlen, dout, din, flags);

	if ((dout == NULL) && (din == NULL)) {
		flags |= SPI_XFER_END;
		pr_err("Both dout and din are NULL so there is nothing to do.");
		ret = -1;
		goto end;
	}

	/*
	 * Only accept bit lengths which are byte multiples and
	 * can be divided in chunks according to the configured
	 * data size set (DSS) according to
	 *
	 * DSS, bits	n_bytes
	 * 3--8		n*1
	 * 9--16	n*2
	 * 17--32	n*4
	 *
	 * For instance, if DSS is 18 the driver will read 4 bytes from
	 * dout and write it to the SPI controller which will send the
	 * first 18 bits and simply discard the remaining 14 bits.
	 */
	if ((bitlen % 8) || (n_bytes % sss->n_bytes)) {
		flags |= SPI_XFER_END;
		pr_err("bitlen is %u but %u is required for configured data size.",
			bitlen, (sss->n_bytes * 8));
		ret = -2;
		goto end;
	}

	/*
	 * The SPI protocol works with both a write and a read, so if the user
	 * has not asked for a read or write we need to do a dummy one instead.
	 */
	tx.dummy_transfer = ((dout == NULL) ? 1 : 0);
	rx.dummy_transfer = ((din == NULL)  ? 1 : 0);

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	tx.current = (u8 *) dout;
	tx.end = tx.current + n_bytes;
	rx.current = (u8 *) din;
	rx.end = rx.current + n_bytes;

	while (tx.current < tx.end) {
		DISABLE_SPI_CONTROLLER(sss);
		spi_read(sss, &rx);
		spi_write(sss, &tx);
		ENABLE_SPI_CONTROLLER(sss);
		WAIT_FOR_SPI_CONTROLLER_TO_FINISH(sss);
	}

	/* try getting remaining rx bytes, if any */
	rx_max_tries = 10;
	while ((rx.current < rx.end) && (rx_max_tries--)) {
		spi_read(sss, &rx);
		WAIT_FOR_SPI_CONTROLLER_TO_FINISH(sss);
	}

end:
	if (flags & SPI_XFER_END)
		spi_cs_deactivate(slave);

	pr_dbg("returning with %i", ret);

	return ret;
}


int spi_cs_is_valid(u32 bus, u32 cs)
{
	pr_dbg("bus %u, cs %u", bus, cs);

	if (((bus >= 0) && (bus < 4)) && (cs == 0))
		return 1;
	else
		return 0;
}


void spi_cs_activate(struct spi_slave *slave)
{
	struct stm_spi_slave *sss = to_stm_spi_slave(slave);
	pr_dbg("slave 0x%p", slave);

	FLUSH_SPI_RX_FIFO(sss);
	ENABLE_SPI_CONTROLLER(sss);
}


void spi_cs_deactivate(struct spi_slave *slave)
{
	struct stm_spi_slave *sss = to_stm_spi_slave(slave);
	pr_dbg("slave 0x%p", slave);

	DISABLE_SPI_CONTROLLER(sss);
}



/*#######################################################################
  Local function definitions
#########################################################################
*/

/*
 * spi023_eff_freq() is copied from stm_spi023.c
 * with as few changes as possible.
 */
static int spi023_eff_freq(int freq, struct spi_clock_params *clk_freq)
{
	/* Lets calculate the frequency parameters */
	u32 cpsdvsr = 2;
	u32 scr = 0;
	int freq_found = 0;
	u32 max_tclk;
	u32 min_tclk;
	/* cpsdvscr = 2 & scr 0 */
	max_tclk = (STM_SPICTLR_CLOCK_FREQ / (MIN_CPSDVR * (1 + MIN_SCR)));
	/* cpsdvsr = 254 & scr = 255 */
	min_tclk = (STM_SPICTLR_CLOCK_FREQ / (MAX_CPSDVR * (1 + MAX_SCR)));

	if ((freq <= max_tclk) && (freq >= min_tclk)) {
		while (cpsdvsr <= MAX_CPSDVR && !freq_found) {
			while (scr <= MAX_SCR && !freq_found) {
				if ((STM_SPICTLR_CLOCK_FREQ /
						(cpsdvsr * (1 + scr))) > freq) {
					scr += 1;
				} else {
					/*
					 * This bool is made TRUE when
					 * effective frequency >= target freq
					 * is found
					 */
					freq_found = 1;
					if ((STM_SPICTLR_CLOCK_FREQ /
						(cpsdvsr * (1 + scr)))
							!= freq) {
						if (scr == MIN_SCR) {
							cpsdvsr -= 2;
							scr = MAX_SCR;
						} else {
							scr -= 1;
						}
					}
				}
			}

			if (!freq_found) {
				cpsdvsr += 2;
				scr = MIN_SCR;
			}
		}

		if (cpsdvsr != 0) {
			pr_info("SPI Effec Freq is 0x%x",
				(STM_SPICTLR_CLOCK_FREQ / (cpsdvsr * (1 + scr)))
			);

			clk_freq->cpsdvsr = (u8) (cpsdvsr & 0xFF);
			clk_freq->scr = (u8) (scr & 0xFF);

			pr_dbg("SPI cpsdvsr = %d, scr = %d",
				clk_freq->cpsdvsr, clk_freq->scr);
		}
	} else {
		/* User is asking for out of range Freq. */
		pr_err("setup - ctlr data incorrect: Out of Range Frequency");
		return -1;
	}

	return 0;
}



static void
spi_write(struct stm_spi_slave *sss, struct spi_transfer * trans_data)
{
	pr_dbg("sss 0x%p, trans_data 0x%p, trans_data->current 0x%p ",
		sss, trans_data, trans_data->current);

	while ((readl(SPI_SR(sss->base_addr)) & SPI_SR_MASK_TNF)
			&& (trans_data->current < trans_data->end)) {
		if (trans_data->dummy_transfer) {
			writel(0x0, SPI_DR(sss->base_addr));
			pr_dbg("Wrote 0x%08x", 0);
		} else if (sss->n_bytes == 1) {
			writeb(*(u8 *) (trans_data->current),
				SPI_DR(sss->base_addr));
			pr_dbg("Wrote 0x%08x",
				(u32)(*(u8 *) (trans_data->current)));
		} else if (sss->n_bytes == 2) {
			writew(*(u16 *) (trans_data->current),
				SPI_DR(sss->base_addr));
			pr_dbg("Wrote 0x%08x",
				(u32)(*(u16 *) (trans_data->current)));
		} else if (sss->n_bytes == 4) {
			writel(*(u32 *) (trans_data->current),
				SPI_DR(sss->base_addr));
			pr_dbg("Wrote 0x%08x",
				(u32)(*(u32 *) (trans_data->current)));
		} else {
			pr_err("Invalid SPI configuration, n_bytes = %u",
				sss->n_bytes);
			trans_data->current = trans_data->end;
			return;
		}

		trans_data->current += sss->n_bytes;
	}
}


static void
spi_read(struct stm_spi_slave *sss, struct spi_transfer * trans_data)
{
	pr_dbg("sss 0x%p, trans_data 0x%p, trans_data->current 0x%p ",
		sss, trans_data, trans_data->current);

	while ((readl(SPI_SR(sss->base_addr)) & SPI_SR_MASK_RNE)
			&& (trans_data->current < trans_data->end)) {
		if (trans_data->dummy_transfer) {
			(void)readl(SPI_DR(sss->base_addr));
		} else if (sss->n_bytes == 1) {
			*(u8 *) (trans_data->current) =
				readb(SPI_DR(sss->base_addr));
			pr_dbg("Read 0x%02x",
				(u8)(*(u8 *) (trans_data->current)));
		} else if (sss->n_bytes == 2) {
			*(u16 *) (trans_data->current) =
				readw(SPI_DR(sss->base_addr));
			pr_dbg("Read 0x%04x",
				(u16)(*(u16 *) (trans_data->current)));
		} else if (sss->n_bytes == 4) {
			*(u32 *) (trans_data->current) =
				readl(SPI_DR(sss->base_addr));
			pr_dbg("Read 0x%08x",
				(u32)(*(u32 *) (trans_data->current)));
		} else {
			pr_err("Invalid SPI configuration, n_bytes = %u",
				sss->n_bytes);
			trans_data->current = trans_data->end;
			return;
		}

		trans_data->current += sss->n_bytes;
	}
}

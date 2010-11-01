/*
 * Copyright (C) ST-Ericsson AB 2010
 *
 * Basic U-Boot I2C interface for STn8500/DB8500
 * Author: Michael Brandt <Michael.Brandt@stericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

/*
 * Only 7-bit I2C device addresses are supported.
 */

#include <common.h>
#include <i2c.h>

/* later: #include <asm/arch/i2c.h> */
#include "i2c.h"
#include "gpio.h"
#include <asm/io.h>
#include <asm/arch/clock.h>

typedef enum {
	I2C_NACK_ADDR,
	I2C_NACK_DATA,
	I2C_ACK_MCODE,
	I2C_ARB_LOST,
	I2C_BERR_START,
	I2C_BERR_STOP,
	I2C_OVFL
} i2c_error_t;

#define I2C_ENDAD_COUNTER       (CONFIG_SYS_HZ/100) /* I2C bus timeout */
#define I2C_FIFO_FLUSH_COUNTER	500000		/* flush "timeout" */
#define I2C_SCL_FREQ            100000          /* I2C bus clock frequency.*/
#define I2C_INPUT_FREQ          48000000        /* Input clock frequency.*/
#define TX_FIFO_THRESHOLD	0x4
#define RX_FIFO_THRESHOLD	0x4
#define SLAVE_SETUP_TIME 14 /* Slave data setup time, 250ns for 48MHz i2c_clk */

static unsigned int bus_initialized[CONFIG_SYS_I2C_BUS_MAX];
static unsigned int i2c_bus_num = 0;
static unsigned int i2c_bus_speed[] = {
	CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED,
	CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED
};
static t_i2c_registers *i2c_dev[] = {
	(t_i2c_registers *)CONFIG_SYS_I2C0_BASE,
	(t_i2c_registers *)CONFIG_SYS_I2C1_BASE,
	(t_i2c_registers *)CONFIG_SYS_I2C2_BASE,
	(t_i2c_registers *)CONFIG_SYS_I2C3_BASE,
};

static struct {
	gpio_alt_function altfunc;
	char *dev_name;
} i2c_gpio_altfunc[] = {
	{GPIO_ALT_I2C_0, "i2c0"},
	{GPIO_ALT_I2C_1, "i2c1"},
	{GPIO_ALT_I2C_2, "i2c2"},
	{GPIO_ALT_I2C_3, "i2c3"},
};

static struct {
	int periph;
	int pcken;
	int kcken;
} i2c_clock_bits[] = {
	{3, 3, 3}, /* I2C0 */
	{1, 2, 2}, /* I2C1 */
	{1, 6, 6}, /* I2C2 */
	{2, 0, 0}, /* I2C3 */
};

static int __i2c_set_bus_speed(unsigned int speed)
{
	u32 value;
	t_i2c_registers *p_i2c_registers;

	p_i2c_registers = i2c_dev[i2c_bus_num];

	/* Select standard (100 kbps) speed mode */
        I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_SM, I2C_CR_SHIFT_SM, 0x0);

        /*
	 * Set the Baud Rate Counter 2 value
	 * Baud rate (standard) = fi2cclk / ( (BRCNT2 x 2) + Foncycle )
	 * Foncycle = 0 (no digital filtering)
	 */
        value = (u32) (I2C_INPUT_FREQ / (speed * 2));
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT2,
			I2C_BRCR_SHIFT_BRCNT2, value);

        /* ensure that BRCNT value is zero */
        I2C_WRITE_FIELD(p_i2c_registers->brcr, I2C_BRCR_BRCNT1,
			I2C_BRCR_SHIFT_BRCNT1, 0);

	return I2C_INPUT_FREQ/(value * 2);
}

/*
 * i2c_init - initialize the i2c bus
 *
 *	speed: bus speed (in HZ)
 *	slaveaddr: address of device in slave mode
 *
 *	Slave mode is not implemented.
 */
void i2c_init(int speed, int slaveaddr)
{
	t_i2c_registers *p_i2c_registers;

	debug("i2c_init bus %d, speed %d\n", i2c_bus_num, speed);

	(void) gpio_altfuncenable(i2c_gpio_altfunc[i2c_bus_num].altfunc,
			i2c_gpio_altfunc[i2c_bus_num].dev_name);

	u8500_clock_enable(i2c_clock_bits[i2c_bus_num].periph,
			   i2c_clock_bits[i2c_bus_num].pcken,
			   i2c_clock_bits[i2c_bus_num].kcken);

	p_i2c_registers = i2c_dev[i2c_bus_num];

	/* Disable the controller */
	I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_PE);

	/* Clear registers */
	I2C_WRITE_REG(p_i2c_registers->cr, 0);
	I2C_WRITE_REG(p_i2c_registers->scr, 0);
	I2C_WRITE_REG(p_i2c_registers->hsmcr, 0);
	I2C_WRITE_REG(p_i2c_registers->tftr, 0);
	I2C_WRITE_REG(p_i2c_registers->rftr, 0);
	I2C_WRITE_REG(p_i2c_registers->dmar, 0);

	/* No digital filter */
	I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_FON, I2C_CR_SHIFT_FON,
			I2C_DIGITAL_FILTERS_OFF);

	i2c_bus_speed[i2c_bus_num] = __i2c_set_bus_speed(speed);

        /*
	 * Set our own address.
	 * Set slave address mode to 7 bit addressing mode
	 */
        I2C_CLR_BIT(p_i2c_registers->cr, I2C_CR_SAM);
        I2C_WRITE_FIELD(p_i2c_registers->scr, I2C_SCR_ADDR, I2C_SCR_SHIFT_ADDR,
			slaveaddr);
	/* Slave Data Set up Time */
	I2C_WRITE_FIELD(p_i2c_registers->scr, I2C_SCR_DATA_SETUP_TIME,
			I2C_SCR_SHIFT_DATA_SETUP_TIME, SLAVE_SETUP_TIME);

	/* Set the DMA sync logic */
	I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_DMA_SLE,
			I2C_CR_SHIFT_DMA_SLE, I2C_DISABLE);

	/* Disable interrupts */
	I2C_WRITE_REG(p_i2c_registers->imscr, 0);

	/* Configure bus master mode */
	I2C_WRITE_FIELD(p_i2c_registers->cr, I2C_CR_OM, I2C_CR_SHIFT_OM,
							I2C_BUS_MASTER_MODE);
	/* Set FIFO threshold values */
	writel(TX_FIFO_THRESHOLD, &p_i2c_registers->tftr);
	writel(RX_FIFO_THRESHOLD, &p_i2c_registers->rftr);

	/* Enable the I2C Controller */
	I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_PE);

	bus_initialized[i2c_bus_num] = 1;
}


/*
 * loop_till_bit_clear - polls on a bit till it clears
 * ioreg: register where you want to check status
 * mask: bit mask for the bit you wish to check
 * timeout: timeout in ticks/s
 */
static int loop_till_bit_clear(void *io_reg, u32 mask, unsigned long timeout)
{
	unsigned long timebase = get_timer(0);

	do {
		if ((readl(io_reg) & mask) == 0x0UL)
			return 0;
	} while (get_timer(timebase) < timeout);

	debug("loop_till_bit_clear timed out\n");
	return -1;
}

/*
 * loop_till_bit_set - polls on a bit till it is set.
 * ioreg: register where you want to check status
 * mask: bit mask for the bit you wish to check
 * timeout: timeout in ticks/s
 */
static int loop_till_bit_set(void * io_reg, u32 mask, unsigned long timeout)
{
	unsigned long timebase = get_timer(0);

	do {
		if ((readl(io_reg) & mask) != 0x0UL)
			return 0;
	} while (get_timer(timebase) < timeout);

	debug("loop_till_bit_set timed out\n");
	return -1;
}

/*
 * flush_fifo - flush the I2C TX and RX FIFOs
 */
static void flush_fifo(t_i2c_registers *p_i2c_registers)
{
	int counter = I2C_FIFO_FLUSH_COUNTER;

	/* Flush Tx FIFO */
	I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FTX);
	/* Flush Rx FIFO */
	I2C_SET_BIT(p_i2c_registers->cr, I2C_CR_FRX);
	while (counter--) {
		if (!(readl(&p_i2c_registers->cr) & (I2C_CR_FTX | I2C_CR_FRX)))
			break;
	}
	return;
}

#ifdef DEBUG
static inline void print_abort_reason(t_i2c_registers *p_i2c_registers)
{
	i2c_error_t cause;

	printf("abort: risr %08x, sr %08x\n", p_i2c_registers->risr,
			p_i2c_registers->sr);
	cause = (i2c_error_t) I2C_READ_FIELD(p_i2c_registers->sr, I2C_SR_CAUSE,
			I2C_SR_SHIFT_CAUSE);
	switch (cause) {
	case I2C_NACK_ADDR:
		printf("No Ack received after Slave Address xmission\n");
		break;
	case I2C_NACK_DATA:
		printf("Valid for MASTER_WRITE: No Ack received"
				"during data phase\n");
		break;
	case I2C_ACK_MCODE:
		printf("Master recv ack after xmission of master code"
				"in hs mode\n");
		break;
	case I2C_ARB_LOST:
		printf("Master Lost arbitration\n");
		break;
	case I2C_BERR_START:
		printf("Slave restarts\n");
		break;
	case I2C_BERR_STOP:
		printf("Slave reset\n");
		break;
	case I2C_OVFL:
		printf("Overflow\n");
		break;
	default:
		printf("Unknown error type\n");
	}
}
#endif

/*
 * i2c_abort - called when a I2C transaction failed
 */
static void i2c_abort(t_i2c_registers *p_i2c_registers)
{
#ifdef DEBUG
	print_abort_reason(p_i2c_registers);
#endif
	/* flush RX and TX fifos */
	flush_fifo(p_i2c_registers);

        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);

        /* Acknowledge the Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

	i2c_init(i2c_bus_speed[i2c_bus_num], CONFIG_SYS_I2C_SLAVE);
}

/*
 * write addr, alias index, to I2C bus.
 */
static int i2c_write_addr(t_i2c_registers *p_i2c_registers, uint addr, int alen)
{
	while (alen--) {
		/* Wait until the Tx Fifo is not full */
		if (loop_till_bit_clear((void* )&p_i2c_registers->risr,
					I2C_INT_TXFF, I2C_ENDAD_COUNTER))
		{
			i2c_abort(p_i2c_registers);
			return -1;
		}

		/* MSB first */
		writeb((addr >> (alen * 8)) & 0xff, &p_i2c_registers->tfr);
	}

	return 0;
}

/*
 * Internal simplified read function:
 *   i2c_registers:	Pointer to I2C registers for current bus
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   value:   Where to put the data
 *
 *   Returns: 0 on success, not 0 on failure
 */
static int i2c_read_byte(t_i2c_registers *p_i2c_registers, uchar chip,
		uint addr, int alen, uchar *value)
{
	volatile u32   mcr = 0;

	/* Set the address mode to 7 bit */
	I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);

        /* Store the slave address in the master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A7, I2C_MCR_SHIFT_A7, chip);

	if (alen != 0) {
		/* Master write operation */
		I2C_CLR_BIT(mcr, I2C_MCR_OP);

		/* Configure the Frame length to one byte */
		I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

		/* Repeated start, no stop */
		I2C_CLR_BIT(mcr, I2C_MCR_STOP);

		/* Write Master Control Register */
		writel(mcr, &p_i2c_registers->mcr);

		/* send addr/index */
		if (i2c_write_addr(p_i2c_registers, addr, alen) != 0)
			return -1;

		/* Check for the Master Transaction Done Without Stop */
		if (loop_till_bit_set((void *)&p_i2c_registers->risr,
					I2C_INT_MTDWS, I2C_ENDAD_COUNTER)) {
			return -1;
		}

		/* Master Transaction Without Stop has been done */
		/* Acknowledge the Master Transaction Done Without Stop */
		I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);
	}

	/* Master control configuration for read operation  */
	I2C_SET_BIT(mcr, I2C_MCR_OP);

	/* Configure the STOP condition, we read only one byte */
	I2C_SET_BIT(mcr, I2C_MCR_STOP);

	/* Set the frame length to one byte, we support only 1 byte
	 * reads */
	I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

	I2C_WRITE_FIELD(p_i2c_registers->mcr, I2C_MCR_LENGTH_STOP_OP,
			I2C_MCR_SHIFT_LENGTH_STOP_OP, mcr);

	/*
	 * receive_data_polling
	 */

	/* Wait until the Rx FIFO is not empty */
	if (loop_till_bit_clear((void* )&p_i2c_registers->risr, I2C_INT_RXFE,
			I2C_ENDAD_COUNTER))
	{
		return -1;
	}

	/* Read the data byte from Rx FIFO */
	*value = readb(&p_i2c_registers->rfr);

	/* Wait until the work is done */
	if (loop_till_bit_set((void *)&p_i2c_registers->risr, I2C_INT_MTD,
				I2C_ENDAD_COUNTER)) {
		return -1;
	}

        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);

        /* If MTD is set, Master Transaction Done Without Stop is set too */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

	return 0;
}

/*
 * Internal simplified write function:
 *   i2c_registers:	Pointer to I2C registers for current bus
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   data:    Where to read the data
 *   len:     How many bytes to write
 *
 *   Returns: 0 on success, not 0 on failure
 */
static int __i2c_write(t_i2c_registers *p_i2c_registers, u8 chip, uint addr,
		int alen, u8 *data, int len)
{
	int i;
	volatile u32 mcr = 0;

	/* Set the address mode to 7 bit */
	I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);

        /* Store the slave address in the master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A7, I2C_MCR_SHIFT_A7, chip);

	/* Write operation */
	I2C_CLR_BIT(mcr, I2C_MCR_OP);

        /* Current transaction is terminated by STOP condition */
        I2C_SET_BIT(mcr, I2C_MCR_STOP);

	/* Frame length: addr byte + len */
	/* XXX: fix I2C_WRITE_FIELD macro: use () around value parameter */
	I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH,
			(alen + len));

	/* Write MCR register */
	writel(mcr, &p_i2c_registers->mcr);

	if (i2c_write_addr(p_i2c_registers, addr, alen) != 0)
		return -1;

	for (i = 0; i < len; i++) {

		/* Wait until the Tx FIFO is not full */
		if (loop_till_bit_clear((void *)&p_i2c_registers->risr,
					I2C_INT_TXFF, I2C_ENDAD_COUNTER)) {
			return -1;
		}

		/* it is a 32 bit register with upper 24 reserved R/O */
		writeb(data[i], &p_i2c_registers->tfr);
	}

	/* Check for Master Transaction Done */
	if (loop_till_bit_set((void *)&p_i2c_registers->risr, I2C_INT_MTD,
				I2C_ENDAD_COUNTER)) {
		printf("i2c_write_byte error2: risr %08x\n",
				p_i2c_registers->risr);
		return -1;
	}

        /* Acknowledge Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);

        /* Acknowledge Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

	return 0;
}

/*
 * Probe the given I2C chip address. Returns 0 if a chip responded,
 * not 0 on failure.
 */
int i2c_probe(uchar chip)
{
	volatile u32   mcr = 0;
	t_i2c_registers     *p_i2c_registers;

	if (chip == CONFIG_SYS_I2C_SLAVE)
		return 1;

	p_i2c_registers = i2c_dev[i2c_bus_num];

	/* Set the address mode to 7 bit */
	I2C_WRITE_FIELD(mcr, I2C_MCR_AM, I2C_MCR_SHIFT_AM, 1);

        /* Store the slave address in the master control register */
        I2C_WRITE_FIELD(mcr, I2C_MCR_A10, I2C_MCR_SHIFT_A7, chip);

	/* Read operation */
	I2C_SET_BIT(mcr, I2C_MCR_OP);

	/* Set the frame length to one byte */
	I2C_WRITE_FIELD(mcr, I2C_MCR_LENGTH, I2C_MCR_SHIFT_LENGTH, 1);

	/* Current transaction is terminated by STOP condition */
	I2C_SET_BIT(mcr, I2C_MCR_STOP);

	/* Write MCR register */
	writel(mcr, &p_i2c_registers->mcr);

	/* Wait until the Rx Fifo is not empty */
	if (loop_till_bit_clear((void* )&p_i2c_registers->risr, I2C_INT_RXFE,
			I2C_ENDAD_COUNTER))
	{
		i2c_abort(p_i2c_registers);
		return -1;
	}

	flush_fifo(p_i2c_registers);

        /* Master Transaction has been done */
        /* Acknowledge the Master Transaction Done */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTD);

        /* Master Transaction Without Stop has been done */
        /* Acknowledge the Master Transaction Done Without Stop */
        I2C_SET_BIT(p_i2c_registers->icr, I2C_INT_MTDWS);

	return 0;
}

/*
 * Read/Write interface:
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int i;
	int rc;
	t_i2c_registers *p_i2c_registers;

	if (alen > 2) {
		debug("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	p_i2c_registers = i2c_dev[i2c_bus_num];

	for (i = 0; i < len; i++) {
		rc = i2c_read_byte(p_i2c_registers, chip, addr + i, alen,
				&buffer[i]);
		if (rc != 0) {
			debug("I2C read: I/O error: %d\n", rc);
			i2c_abort(p_i2c_registers);
			return rc;
		}
	}

	return 0;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int rc;
	t_i2c_registers *p_i2c_registers;
	p_i2c_registers = i2c_dev[i2c_bus_num];

	rc = __i2c_write(p_i2c_registers, chip, addr, alen, buffer,
			len);
	if (rc != 0) {
		debug("I2C write: I/O error\n");
		i2c_abort(p_i2c_registers);
		return rc;
	}
	return 0;
}

int i2c_set_bus_num(unsigned int bus)
{
	if (bus > ARRAY_SIZE(i2c_dev) - 1) {
		debug("i2c_set_bus_num: only up to bus %d supported\n", ARRAY_SIZE(i2c_dev)-1);
		return -1;
	}

	i2c_bus_num = bus;

	if(!bus_initialized[i2c_bus_num])
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	return 0;
}

int i2c_set_bus_speed(unsigned int speed)
{

	if (speed > I2C_MAX_STANDARD_SCL) {
		debug("i2c_set_bus_speed: only up to %d supported\n",
				I2C_MAX_STANDARD_SCL);
		return -1;
	}

	/* sets as side effect i2c_bus_speed[i2c_bus_num] */
	i2c_init(speed, CONFIG_SYS_I2C_SLAVE);

	return 0;
}

unsigned int i2c_get_bus_num(void)
{
	return i2c_bus_num;
}

unsigned int i2c_get_bus_speed(void)
{
	return i2c_bus_speed[i2c_bus_num];
}

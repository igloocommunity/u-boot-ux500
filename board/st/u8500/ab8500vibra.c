/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Grzegorz Sygieda <grzegorz.sygieda@tieto.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <common.h>
#include <command.h>
#include <asm/arch/ab8500.h>

/* Control vibrator */
int do_vibrate(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int time_ms = 100;  /* 100 ms */
	int intensity = 50;  /* 50 % */
	int reg;

	/*
	 * We use the last specified parameters, unless new ones are
	 * entered.
	 */
	if (!(flag & CMD_FLAG_REPEAT)) {
		/* Parse vibration time if given */
		if (argc >= 2)
			time_ms = simple_strtoul(argv[1], NULL, 10);

		/* Parse vibration intensity if given */
		if (argc >= 3)
			intensity = simple_strtoul(argv[2], NULL, 10);
	}

	/* disable audio registers reset */
	reg = ab8500_read(AB8500_SYS_CTRL2_BLOCK, AB8500_CTRL3_REG);
	ab8500_write(AB8500_SYS_CTRL2_BLOCK, AB8500_CTRL3_REG, reg | 0x02);
	/* audio clock enable */
	reg = ab8500_read(AB8500_SYS_CTRL2_BLOCK, AB8500_SYSULPCLK_CTRL1_REG);
	ab8500_write(AB8500_SYS_CTRL2_BLOCK,
			AB8500_SYSULPCLK_CTRL1_REG, reg | 0x10);
	/* enable audio supply */
	ab8500_write(AB8500_REGU_CTRL1, AB8500_REGU_VAUDIO_SUPPLY_REG, 0x02);

	/*
	 *  Audio related registers - Vibrator is controled using PWM
	 */
	/* power up audio feature */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_POWER_UP, 0x88);
	/* enable vibra class-D */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_ANA_CONF4, 0x03);
	/* general vibra control */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF1, 0xFF);

	/*
	 *  control register ...  Set PWM intensity 0..100%
	 */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF2, 0);
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF3, intensity);
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF4, 0);
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF5, intensity);

	/* Sleep for time specified */
	udelay(1000 * time_ms);

	/* Set PWM RMS power to zero */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF3, 0);
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_PWM_GEN_CONF5, 0);

	/* audio clock disable */
	reg = ab8500_read(AB8500_SYS_CTRL2_BLOCK, AB8500_SYSULPCLK_CTRL1_REG);
	ab8500_write(AB8500_SYS_CTRL2_BLOCK,
			AB8500_SYSULPCLK_CTRL1_REG, reg & ~0x10);
	/* power down audio feature */
	ab8500_write(AB8500_AUDIO, AB8500_AUDIO_POWER_UP, 0);
	/* disable audio supply */
	ab8500_write(AB8500_REGU_CTRL1, AB8500_REGU_VAUDIO_SUPPLY_REG, 0);

	return 0;
}

U_BOOT_CMD(
	vibrate,    3,    1,    do_vibrate,
	"vibrator control utility\n",
	"<time> <intensity> - vibrate for time, intensity specified\n"
	"<time> - vibration time (default 100 ms)\n"
	"<intensity> - vibration intensity (default 50 %)\n"
);

/*
 * Copyright (C) ST-Ericsson SA 2009
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
#ifndef _MOP500_GPIO_h
#define _MOP500_GPIO_h

#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>

#include "common.h"
#include <configs/u8500.h>

#define GPIO_TOTAL_PINS                 268

#define GPIO_PINS_PER_BLOCK 	32
#define GPIO_BLOCKS_COUNT       (GPIO_TOTAL_PINS/GPIO_PINS_PER_BLOCK +1)
#define GPIO_BLOCK(pin)	( ( ( pin + GPIO_PINS_PER_BLOCK ) >> 5) - 1 )


struct gpio_register {
	u32 gpio_dat;	/* GPIO data register *//*0x000 */
	u32 gpio_dats;	/* GPIO data Set register *//*0x004 */
	u32 gpio_datc;	/* GPIO data Clear register *//*0x008 */
	u32 gpio_pdis;	/* GPIO Pull disable register *//*0x00C */
	u32 gpio_dir;	/* GPIO data direction register *//*0x010 */
	u32 gpio_dirs;	/* GPIO data dir Set register *//*0x014 */
	u32 gpio_dirc;	/* GPIO data dir Clear register *//*0x018 */
	u32 gpio_slpm;	/* GPIO Sleep mode register *//*0x01C */
	u32 gpio_afsa;	/* GPIO AltFun A Select reg *//*0x020 */
	u32 gpio_afsb;	/* GPIO AltFun B Select reg *//*0x024 */
	u32 gpio_lowemi;	/* GPIO low EMI Select reg *//*0x028 */
	u32 reserved_1[(0x040 - 0x02C) >> 2];	/*0x028-0x3C Reserved*/
	u32 gpio_rimsc;	/* GPIO rising edge intr set/clear *//*0x040 */
	u32 gpio_fimsc;	/* GPIO falling edge interrupt set/clear register *//*0x044 */
	u32 gpio_mis;	/* GPIO masked interrupt status register *//*0x048 */
	u32 gpio_ic;	/* GPIO Interrupt Clear register *//*0x04C */
	u32 gpio_rwimsc;	/* GPIO Rising-edge Wakeup IMSC register *//*0x050 */
	u32 gpio_fwimsc;	/* GPIO Falling-edge Wakeup IMSC register *//*0x054 */
	u32 gpio_wks;	/* GPIO Wakeup Status register *//*0x058 */
};

/* Error values returned by functions */
typedef enum {
	GPIO_OK = 0,		/* (0) */
	GPIO_UNSUPPORTED_HW = -2, /* NOMADIK_UNSUPPORTED_HW,	(-2) */
	GPIO_UNSUPPORTED_FEATURE = -3, /* NOMADIK_UNSUPPORTED_FEATURE,	(-3) */
	GPIO_INVALID_PARAMETER = -4, /* NOMADIK_INVALID_PARAMETER,	(-4) */
	GPIO_REQUEST_NOT_APPLICABLE = -5, /* NOMADIK_REQUEST_NOT_APPLICABLE,	(-5) */
	GPIO_REQUEST_PENDING = -6, /* NOMADIK_REQUEST_PENDING,	(-6) */
	GPIO_NOT_CONFIGURED = -7, /* NOMADIK_NOT_CONFIGURED,	(-7) */
	GPIO_INTERNAL_ERROR = -8, /* NOMADIK_INTERNAL_ERROR,	(-8) */
	GPIO_INTERNAL_EVENT = 1, /* NOMADIK_INTERNAL_EVENT,*/
	GPIO_REMAINING_EVENT = 2, /* NOMADIK_REMAINING_PENDING_EVENTS,*/
	GPIO_NO_MORE_PENDING_EVENT = 3, /* NOMADIK_NO_MORE_PENDING_EVENT,*/
	GPIO_INVALID_CLIENT = -25,
	GPIO_INVALID_PIN = -26,
	GPIO_PIN_BUSY = -27,
	GPIO_PIN_NOT_ALLOCATED = -28,
	GPIO_WRONG_CLIENT = -29,
	GPIO_UNSUPPORTED_ALTFUNC = -30,

} gpio_error;

/*GPIO DEVICE ID */
typedef enum {
	GPIO_DEVICE_ID_0,
	GPIO_DEVICE_ID_1,
	GPIO_DEVICE_ID_2,
	GPIO_DEVICE_ID_3,
	GPIO_DEVICE_ID_INVALID
} gpio_device_id;

/*
 * Pin description To be used in SOFTWARE mode: refers to a pin. 
 */
typedef enum {
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
	GPIO_PIN_16,
	GPIO_PIN_17,
	GPIO_PIN_18,
	GPIO_PIN_19,
	GPIO_PIN_20,
	GPIO_PIN_21,
	GPIO_PIN_22,
	GPIO_PIN_23,
	GPIO_PIN_24,
	GPIO_PIN_25,
	GPIO_PIN_26,
	GPIO_PIN_27,
	GPIO_PIN_28,
	GPIO_PIN_29,
	GPIO_PIN_30,
	GPIO_PIN_31,
	GPIO_PIN_32,
	GPIO_PIN_33,
	GPIO_PIN_34,
	GPIO_PIN_35,
	GPIO_PIN_36,
	GPIO_PIN_37,
	GPIO_PIN_38,
	GPIO_PIN_39,
	GPIO_PIN_40,
	GPIO_PIN_41,
	GPIO_PIN_42,
	GPIO_PIN_43,
	GPIO_PIN_44,
	GPIO_PIN_45,
	GPIO_PIN_46,
	GPIO_PIN_47,
	GPIO_PIN_48,
	GPIO_PIN_49,
	GPIO_PIN_50,
	GPIO_PIN_51,
	GPIO_PIN_52,
	GPIO_PIN_53,
	GPIO_PIN_54,
	GPIO_PIN_55,
	GPIO_PIN_56,
	GPIO_PIN_57,
	GPIO_PIN_58,
	GPIO_PIN_59,
	GPIO_PIN_60,
	GPIO_PIN_61,
	GPIO_PIN_62,
	GPIO_PIN_63,
	GPIO_PIN_64,
	GPIO_PIN_65,
	GPIO_PIN_66,
	GPIO_PIN_67,
	GPIO_PIN_68,
	GPIO_PIN_69,
	GPIO_PIN_70,
	GPIO_PIN_71,
	GPIO_PIN_72,
	GPIO_PIN_73,
	GPIO_PIN_74,
	GPIO_PIN_75,
	GPIO_PIN_76,
	GPIO_PIN_77,
	GPIO_PIN_78,
	GPIO_PIN_79,
	GPIO_PIN_80,
	GPIO_PIN_81,
	GPIO_PIN_82,
	GPIO_PIN_83,
	GPIO_PIN_84,
	GPIO_PIN_85,
	GPIO_PIN_86,
	GPIO_PIN_87,
	GPIO_PIN_88,
	GPIO_PIN_89,
	GPIO_PIN_90,
	GPIO_PIN_91,
	GPIO_PIN_92,
	GPIO_PIN_93,
	GPIO_PIN_94,
	GPIO_PIN_95,
	GPIO_PIN_96,
	GPIO_PIN_97,
	GPIO_PIN_98,
	GPIO_PIN_99,
	GPIO_PIN_100,
	GPIO_PIN_101,
	GPIO_PIN_102,
	GPIO_PIN_103,
	GPIO_PIN_104,
	GPIO_PIN_105,
	GPIO_PIN_106,
	GPIO_PIN_107,
	GPIO_PIN_108,
	GPIO_PIN_109,
	GPIO_PIN_110,
	GPIO_PIN_111,
	GPIO_PIN_112,
	GPIO_PIN_113,
	GPIO_PIN_114,
	GPIO_PIN_115,
	GPIO_PIN_116,
	GPIO_PIN_117,
	GPIO_PIN_118,
	GPIO_PIN_119,
	GPIO_PIN_120,
	GPIO_PIN_121,
	GPIO_PIN_122,
	GPIO_PIN_123,
	GPIO_PIN_124,
	GPIO_PIN_125,
	GPIO_PIN_126,
	GPIO_PIN_127,
	GPIO_PIN_128,
	GPIO_PIN_129,
	GPIO_PIN_130,
	GPIO_PIN_131,
	GPIO_PIN_132,
	GPIO_PIN_133,
	GPIO_PIN_134,
	GPIO_PIN_135,
	GPIO_PIN_136,
	GPIO_PIN_137,
	GPIO_PIN_138,
	GPIO_PIN_139,
	GPIO_PIN_140,
	GPIO_PIN_141,
	GPIO_PIN_142,
	GPIO_PIN_143,
	GPIO_PIN_144,
	GPIO_PIN_145,
	GPIO_PIN_146,
	GPIO_PIN_147,
	GPIO_PIN_148,
	GPIO_PIN_149,
	GPIO_PIN_150,
	GPIO_PIN_151,
	GPIO_PIN_152,
	GPIO_PIN_153,
	GPIO_PIN_154,
	GPIO_PIN_155,
	GPIO_PIN_156,
	GPIO_PIN_157,
	GPIO_PIN_158,
	GPIO_PIN_159,
	GPIO_PIN_160,
	GPIO_PIN_161,
	GPIO_PIN_162,
	GPIO_PIN_163,
	GPIO_PIN_164,
	GPIO_PIN_165,
	GPIO_PIN_166,
	GPIO_PIN_167,
	GPIO_PIN_168,
	GPIO_PIN_169,
	GPIO_PIN_170,
	GPIO_PIN_171,
	GPIO_PIN_172,
	GPIO_PIN_173,
	GPIO_PIN_174,
	GPIO_PIN_175,
	GPIO_PIN_176,
	GPIO_PIN_177,
	GPIO_PIN_178,
	GPIO_PIN_179,
	GPIO_PIN_180,
	GPIO_PIN_181,
	GPIO_PIN_182,
	GPIO_PIN_183,
	GPIO_PIN_184,
	GPIO_PIN_185,
	GPIO_PIN_186,
	GPIO_PIN_187,
	GPIO_PIN_188,
	GPIO_PIN_189,
	GPIO_PIN_190,
	GPIO_PIN_191,
	GPIO_PIN_192,
	GPIO_PIN_193,
	GPIO_PIN_194,
	GPIO_PIN_195,
	GPIO_PIN_196,
	GPIO_PIN_197,
	GPIO_PIN_198,
	GPIO_PIN_199,
	GPIO_PIN_200,
	GPIO_PIN_201,
	GPIO_PIN_202,
	GPIO_PIN_203,
	GPIO_PIN_204,
	GPIO_PIN_205,
	GPIO_PIN_206,
	GPIO_PIN_207,
	GPIO_PIN_208,
	GPIO_PIN_209,
	GPIO_PIN_210,
	GPIO_PIN_211,
	GPIO_PIN_212,
	GPIO_PIN_213,
	GPIO_PIN_214,
	GPIO_PIN_215,
	GPIO_PIN_216,
	GPIO_PIN_217,
	GPIO_PIN_218,
	GPIO_PIN_219,
	GPIO_PIN_220,
	GPIO_PIN_221,
	GPIO_PIN_222,
	GPIO_PIN_223,
	GPIO_PIN_224,
	GPIO_PIN_225,
	GPIO_PIN_226,
	GPIO_PIN_227,
	GPIO_PIN_228,
	GPIO_PIN_229,
	GPIO_PIN_230,
	GPIO_PIN_231,
	GPIO_PIN_232,
	GPIO_PIN_233,
	GPIO_PIN_234,
	GPIO_PIN_235,
	GPIO_PIN_236,
	GPIO_PIN_237,
	GPIO_PIN_238,
	GPIO_PIN_239,
	GPIO_PIN_240,
	GPIO_PIN_241,
	GPIO_PIN_242,
	GPIO_PIN_243,
	GPIO_PIN_244,
	GPIO_PIN_245,
	GPIO_PIN_246,
	GPIO_PIN_247,
	GPIO_PIN_248,
	GPIO_PIN_249,
	GPIO_PIN_250,
	GPIO_PIN_251,
	GPIO_PIN_252,
	GPIO_PIN_253,
	GPIO_PIN_254,
	GPIO_PIN_255,
	GPIO_PIN_256,
	GPIO_PIN_257,
	GPIO_PIN_258,
	GPIO_PIN_259,
	GPIO_PIN_260,
	GPIO_PIN_261,
	GPIO_PIN_262,
	GPIO_PIN_263,
	GPIO_PIN_264,
	GPIO_PIN_265,
	GPIO_PIN_266,
	GPIO_PIN_267
} gpio_pin;

/*
 * Alternate Function:
 *  refered in altfun_table to pointout particular altfun to be enabled
 *  when using GPIO_ALT_FUNCTION A/B/C enable/disable operation
 */
typedef enum {
	GPIO_ALT_UART_0_MODEM,
	GPIO_ALT_UART_0_NO_MODEM,
	GPIO_ALT_UART_1,
	GPIO_ALT_UART_2,
	GPIO_ALT_I2C_0,
	GPIO_ALT_I2C_1,
	GPIO_ALT_I2C_2,
	GPIO_ALT_I2C_3,
	GPIO_ALT_MSP_0,
	GPIO_ALT_MSP_1,
	GPIO_ALT_MSP_2,
	GPIO_ALT_MSP_3,
	GPIO_ALT_MSP_4,
	GPIO_ALT_MSP_5,
	GPIO_ALT_SSP_0,
	GPIO_ALT_SSP_1,
	GPIO_ALT_MM_CARD0,
	GPIO_ALT_SD_CARD0,
	GPIO_ALT_DMA_0,
	GPIO_ALT_DMA_1,
	GPIO_ALT_HSI0,
	GPIO_ALT_CCIR656_INPUT,
	GPIO_ALT_CCIR656_OUTPUT,
	GPIO_ALT_LCD_PANEL,
	GPIO_ALT_MDIF,
	GPIO_ALT_SDRAM,
	GPIO_ALT_HAMAC_AUDIO_DBG,
	GPIO_ALT_HAMAC_VIDEO_DBG,
	GPIO_ALT_CLOCK_RESET,
	GPIO_ALT_TSP,
	GPIO_ALT_IRDA,
	GPIO_ALT_USB_MINIMUM,
	GPIO_ALT_USB_I2C,
	GPIO_ALT_OWM,
	GPIO_ALT_PWL,
	GPIO_ALT_FSMC,
	GPIO_ALT_COMP_FLASH,
	GPIO_ALT_SRAM_NOR_FLASH,
	GPIO_ALT_FSMC_ADDLINE_0_TO_15,
	GPIO_ALT_SCROLL_KEY,
	GPIO_ALT_MSHC,
	GPIO_ALT_HPI,
	GPIO_ALT_USB_OTG,
	GPIO_ALT_SDIO,
	GPIO_ALT_HSMMC,
	GPIO_ALT_FSMC_ADD_DATA_0_TO_25,
	GPIO_ALT_HSI1,
	GPIO_ALT_NOR,
	GPIO_ALT_NAND,
	GPIO_ALT_KEYPAD,
	GPIO_ALT_VPIP,
	GPIO_ALT_CAM,
	GPIO_ALT_CCP1,
	GPIO_ALT_EMMC,
	GPIO_ALT_POP_EMMC,
	GPIO_ALT_FUNMAX		/* Add new alt func before this */
} gpio_alt_function;

/* Defines pin assignment(Software mode or Alternate mode) */
typedef enum {
	GPIO_MODE_LEAVE_UNCHANGED,	/* Parameter will be ignored by the function. */
	GPIO_MODE_SOFTWARE,	/* Pin connected to GPIO (SW controlled) */
	GPIO_ALTF_A,		/* Pin connected to alternate function 1 (HW periph 1) */
	GPIO_ALTF_B,		/* Pin connected to alternate function 2 (HW periph 2) */
	GPIO_ALTF_C,		/* Pin connected to alternate function 3 (HW periph 3) */
	GPIO_ALTF_FIND,		/* Pin connected to alternate function 3 (HW periph 3) */
	GPIO_ALTF_DISABLE	/* Pin connected to alternate function 3 (HW periph 3) */
} gpio_mode;

/* Defines GPIO pin direction */
typedef enum {
	GPIO_DIR_LEAVE_UNCHANGED,	/* Parameter will be ignored by the function. */
	GPIO_DIR_INPUT,		/* GPIO set as input */
	GPIO_DIR_OUTPUT		/* GPIO set as output */
} gpio_direction;

/* Interrupt trigger mode */
typedef enum {
	GPIO_TRIG_LEAVE_UNCHANGED,	/* Parameter will be ignored by the function */
	GPIO_TRIG_DISABLE,	/* Triggers no IT */
	GPIO_TRIG_RISING_EDGE,	/* Triggers an IT on a rising edge */
	GPIO_TRIG_FALLING_EDGE,	/* Triggers an IT on a falling edge */
	GPIO_TRIG_BOTH_EDGES,	/* Triggers an IT on a rising and a falling edge */
	GPIO_TRIG_HIGH_LEVEL,	/* Triggers an IT on a high level */
	GPIO_TRIG_LOW_LEVEL	/* Triggers an IT on a low level */
} gpio_trig;			/* Interrupt trigger mode, or disable */

/* Configuration parameters for one GPIO pin.*/
typedef struct {
	gpio_mode mode;		/* Defines mode (SOFTWARE or Alternate). */
	gpio_direction direction;	/* Define pin direction (in SOFTWARE mode only). */
	gpio_trig trig;		/* Interrupt trigger (in SOFTWARE mode only) */
	char *dev_name;		/* Name of client driver who owns the gpio pin */
} gpio_config;

/* GPIO pin data*/
typedef enum {
	GPIO_DATA_LOW,		/* GPIO pin status is low. */
	GPIO_DATA_HIGH		/* GPIO pin status is high. */
} gpio_data;

/* GPIO behaviour in sleep mode */
typedef enum {
	GPIO_SLEEP_MODE_LEAVE_UNCHANGED,	/* Parameter will be ignored by the function. */
	GPIO_SLEEP_MODE_INPUT_DEFAULTVOLT,	/* GPIO is an input with pull up/down enabled 
						   when in sleep mode. */
	GPIO_SLEEP_MODE_CONTROLLED_BY_GPIO	/* GPIO pin  is controlled by GPIO IP. So mode,
						   direction and data values for GPIO pin in
						   sleep mode are determined by configuration
						   set to GPIO pin before entering to sleep mode. */
} gpio_sleep_mode;

/* GPIO ability to wake the system up from sleep mode.*/
typedef enum {
	GPIO_WAKE_LEAVE_UNCHANGED,	/* Parameter will be ignored by the function. */
	GPIO_WAKE_DISABLE,	/* GPIO will not wake the system from sleep mode. */
	GPIO_WAKE_LOW_LEVEL,	/* GPIO will wake the system up on a LOW level. */
	GPIO_WAKE_HIGH_LEVEL,	/* GPIO will wake the system up on a HIGH level. */
	GPIO_WAKE_RISING_EDGE,	/* GPIO will wake the system up on a RISING edge. */
	GPIO_WAKE_FALLING_EDGE,	/* GPIO will wake the system up on a FALLING edge. */
	GPIO_WAKE_BOTH_EDGES	/* GPIO will wake the system up on both RISING and FALLING edge. */
} gpio_wake;

/* Configuration parameters for one GPIO pin in sleep mode.*/
typedef struct {
	gpio_sleep_mode sleep_mode;	/* GPIO behaviour in sleep mode. */
	gpio_wake wake;		/* GPIO ability to wake up the system. */
} gpio_sleep_config;

/*------------------------------------------------------------------------
 * Functions declaration
 * refer ./Documentation/arm/STM-Nomadik/gpio_user_guide.txt
 *----------------------------------------------------------------------*/

extern gpio_error gpio_setpinconfig(gpio_pin pin_id, gpio_config * pin_config);
extern gpio_error gpio_resetpinconfig(gpio_pin pin_id, char *dev_name);
extern int gpio_writepin(gpio_pin pin_id, gpio_data value, char *dev_name);
extern int gpio_readpin(gpio_pin pin_id, gpio_data * value);
extern int gpio_altfuncenable(gpio_alt_function altfunc,
				      char *dev_name);
extern int gpio_altfuncdisable(gpio_alt_function altfunc,
				       char *dev_name);

struct gpio_altfun_data {
	u16 altfun;
	u16 start;
	u16 end;
	t_bool cont;
	u8 type;
};

#endif				/* __INC_GPIO_H */

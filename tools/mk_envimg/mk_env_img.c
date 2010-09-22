/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Rickard Evertsson <rickard.evertsson@stericsson.com>
 * for ST-Ericsson.
 *
 * Parts of this code are copied and / or inspired from
 * boot/u-boot/tools/env/fw_env.c
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <configs/u8500.h>
#include "mk_env_img.h"

#define ENV_SIZE	(CONFIG_ENV_SIZE - sizeof(uint32_t))
#define CONFIG_BUF_SIZE	200

struct env_image_single {
	uint32_t	crc;	/* CRC32 over data bytes    */
	char		data[ENV_SIZE];
} single_image = {.data = "\0"};

#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)
static char default_environment[] = {
#ifdef	CONFIG_BOOTARGS
	"bootargs="	CONFIG_BOOTARGS			"\0"
#endif
#ifdef	CONFIG_BOOTCOMMAND
	"bootcmd="	CONFIG_BOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_RAMBOOTCOMMAND
	"ramboot="	CONFIG_RAMBOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_NFSBOOTCOMMAND
	"nfsboot="	CONFIG_NFSBOOTCOMMAND		"\0"
#endif
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	"bootdelay="	MK_STR(CONFIG_BOOTDELAY)	"\0"
#endif
#if defined(CONFIG_BAUDRATE) && (CONFIG_BAUDRATE >= 0)
	"baudrate="	MK_STR(CONFIG_BAUDRATE)		"\0"
#endif
#ifdef	CONFIG_LOADS_ECHO
	"loads_echo="	MK_STR(CONFIG_LOADS_ECHO)	"\0"
#endif
#ifdef	CONFIG_ETHADDR
	"ethaddr="	MK_STR(CONFIG_ETHADDR)		"\0"
#endif
#ifdef	CONFIG_ETH1ADDR
	"eth1addr="	MK_STR(CONFIG_ETH1ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH2ADDR
	"eth2addr="	MK_STR(CONFIG_ETH2ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH3ADDR
	"eth3addr="	MK_STR(CONFIG_ETH3ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH4ADDR
	"eth4addr="	MK_STR(CONFIG_ETH4ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH5ADDR
	"eth5addr="	MK_STR(CONFIG_ETH5ADDR)		"\0"
#endif
#ifdef	CONFIG_IPADDR
	"ipaddr="	MK_STR(CONFIG_IPADDR)		"\0"
#endif
#ifdef	CONFIG_SERVERIP
	"serverip="	MK_STR(CONFIG_SERVERIP)		"\0"
#endif
#ifdef	CONFIG_SYS_AUTOLOAD
	"autoload="	CONFIG_SYS_AUTOLOAD			"\0"
#endif
#ifdef	CONFIG_PREBOOT
	"preboot="	CONFIG_PREBOOT			"\0"
#endif
#ifdef	CONFIG_ROOTPATH
	"rootpath="	MK_STR(CONFIG_ROOTPATH)		"\0"
#endif
#ifdef	CONFIG_GATEWAYIP
	"gatewayip="	MK_STR(CONFIG_GATEWAYIP)	"\0"
#endif
#ifdef	CONFIG_NETMASK
	"netmask="	MK_STR(CONFIG_NETMASK)		"\0"
#endif
#ifdef	CONFIG_HOSTNAME
	"hostname="	MK_STR(CONFIG_HOSTNAME)		"\0"
#endif
#ifdef	CONFIG_BOOTFILE
	"bootfile="	MK_STR(CONFIG_BOOTFILE)		"\0"
#endif
#ifdef	CONFIG_LOADADDR
	"loadaddr="	MK_STR(CONFIG_LOADADDR)		"\0"
#endif
#ifdef	CONFIG_CLOCKS_IN_MHZ
	"clocks_in_mhz=1\0"
#endif
#if defined(CONFIG_PCI_BOOTDELAY) && (CONFIG_PCI_BOOTDELAY > 0)
	"pcidelay="	MK_STR(CONFIG_PCI_BOOTDELAY)	"\0"
#endif
#ifdef	CONFIG_EXTRA_ENV_SETTINGS
	CONFIG_EXTRA_ENV_SETTINGS
#endif
	"\0"
};

struct env_image_single *single = &single_image;

static int replace_name_value_pair(char *name, char *value);

static char *split_name_value_pair(char *s, char *endpos);

static char *envmatch(char * s1, char * s2);

static int replace_default_values(char *file_name);

static int write_final_image(char *output_image);

static int print_env_image(void);

static void cmd_usage(void)
{
	fprintf(stderr, "## Usage: mk_envimg [-d] output_file [input_file]\n");
}

static int dbg_level;

/*
 * main function
 * Creates an environment image of the size 4 kB
 * The variables specified in the input_file
 * overrides the U-boot default values.
 *
 * Syntax:
 *	mk_envimg [-d] output_file [input_file]
 */
int main(int argc, char *argv[])
{
	char *p;
	char *cmdname = *argv;
	int i = 0;

	if (argc < 2) {
		cmd_usage();
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "-d") == 0) {
		if (argc < 3) {
			cmd_usage();
			return EXIT_FAILURE;
		}
		dbg_level = 1;
	}

	if (mk_envimg(argc, argv) != 0)
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}

/*
 * Create a default image and replace default values with values
 * specified in config file
 * Return values:
 *  OK     - Success
 * -NOK    - Replace default values failure
 * -ENOMEM - No memory available
 * Returns 0 on success
 */

int mk_envimg(int argc, char *argv[])
{
	char *config_file_name = NULL;
	char *output_image;

	/* create an image with default U-boot environment settings */
	memcpy(single->data, default_environment, sizeof default_environment);

	output_image = argv[1 + dbg_level];

	/* check if config_file has been specified as input */
	if (argc > 2 + dbg_level) {
		if (dbg_level)
			printf("argc=%d, argv=%s\n", argc + dbg_level,
			       argv[2 + dbg_level]);

		config_file_name = argv[2 + dbg_level];
	}

	if (config_file_name) {
		/* replace with specified variables in config file */
		if (replace_default_values(config_file_name))
			return -NOK;
	}
	/* write image into file */
	if (write_final_image(output_image))
		return -NOK;

	/* print environment variables to stdout */
	if (dbg_level)
		print_env_image();

	return OK;
}

/*
 * Deletes or sets environment variables.
 * Return codes:
 * OK	  - OK
 * NOK    - non-specific failure
 * EINVAL - need at least 1 argument
 * EROFS  - certain variables ("ethaddr", "serial#") cannot be
 *	    modified or deleted
 *
 */
static int replace_name_value_pair(char *name, char *value)
{
	int len;
	char *env, *nxt;
	char *oldval = NULL;

	/*
	 * search if variable with this name already exists
	 */
	for (nxt = env = single->data; *env; env = nxt + 1) {
		for (nxt = env; *nxt; ++nxt) {
			if (nxt >= &single->data[ENV_SIZE]) {
				fprintf(stderr, "## Error: "
					"environment not terminated\n");
				return -NOK;
			}
		}
		oldval = envmatch(name, env);
		if (oldval != NULL)
			break;
	}

	/*
	 * delete any existing definition
	 */
	if (oldval) {
		/*
		 * ethernet Address and serial# can be set only once
		 */
		if ((strcmp(name, "ethaddr") == 0) ||
			(strcmp(name, "serial#") == 0)) {
			fprintf(stderr, "Can't overwrite \"%s\"\n", name);
			return -NOK;
		}

		if (*++nxt == '\0')
			*env = '\0';
		else {
			for (;;) {
				*env = *nxt++;
				if ((*env == '\0') && (*nxt == '\0'))
					break;
				++env;
			}
		}
		*++env = '\0';
	}

	/* if value exists, append new definition at the end */
	if (value) {
		for (env = single->data; *env || *(env + 1); ++env)
			;
		if (env > single->data)
			++env;
		/*
		 * Overflow when:
		 * "name" + "=" + "val" +"\0\0"  > CONFIG_ENV_SIZE -
		 * (env-environment)
		 */
		len = strlen(name) + 2;
		/* add '=' for first arg, ' ' for all others */
		len += strlen(value);

		if (len > (&single->data[ENV_SIZE] - env)) {
			fprintf(stderr, "Error: environment overflow, \"%s\" \
				deleted\n", name);
			return -NOK;
		}
		while ((*env = *name++) != '\0')
			env++;

		*env = '=';
		while ((*++env = *value++) != '\0')
			;

		/* end is marked with double '\0' */
		*++env = '\0';
	}

	/* update CRC */
	single->crc = crc32(0, (uint8_t *) single->data, ENV_SIZE);

	return OK;
}

/*
 * s1 is either a simple 'name', or a 'name=value' pair.
 * s2 is a 'name=value' pair.
 * If the names match, return the value of s2, else NULL.
 */

static char *envmatch(char * s1, char * s2)
{

	while (*s1 == *s2++)
		if (*s1++ == '=')
			return s2;
	if (*s1 == '\0' && *(s2 - 1) == '=')
		return s2;
	return NULL;
}

/*
 * s is either a 'name' or a 'name=value' pair
 * replace '=' with '\0' so that the 'name' can be printed out of s buffer
 * replace end of line with '\0'
 * returns value or NULL for no value
 */
static char *split_name_value_pair(char *s, char *endpos)
{
	int i;

	if (s == endpos)
		return NULL;
	for (i = 0; i < CONFIG_BUF_SIZE; i++) {
		s++;
		/* if true, no value exists */
		if (s == endpos) {
			*s = '\0';
			return NULL;
		}
		if (*s == '=') {
			/* set endmark for 'name' */
			*s = '\0';
			/* if true, no value exists */
			if (++s == endpos)
				return NULL;

			/* set endmark for 'value' (replace LR & EOF) */
			*endpos = '\0';
			return s;
		}
	}
	fprintf(stderr, "## Error: Corrupt config file\n");
	return NULL;
}

/*
 * Open config file and replace environment variables in
 * default image.
 * Return values:
 * OK      - for success
 * -NOK    - Text string is larger than text buffer
 * -ENOENT - File or folder does not exist
 */
static int replace_default_values(char *file_name)
{
	int ret = 0;
	int i = 0;
	FILE *config_file;
	char *s;
	char *value;
	char s_buf[CONFIG_BUF_SIZE];

	config_file = fopen(file_name, "r");
	if (config_file == NULL) {
		fprintf(stderr, "## Error: File does not exist\n");
		return -ENOENT;
	}

	while (ret != EOF) {
		for (s = &s_buf[0]; i < CONFIG_BUF_SIZE; i++) {
			ret = fgetc(config_file);
			*s = (char) ret;

			if ((*s == '\n') || (ret == EOF)) {
				value = split_name_value_pair(&s_buf[0], s);
				replace_name_value_pair(&s_buf[0], value);
				i = 0;
				break;
			}

			s++;
		}
		if (i == CONFIG_BUF_SIZE) {
			fprintf(stderr, "## Error: Buffer too small\n");
			fclose(config_file);
			return NOK;
		}
	}
	fclose(config_file);
	return OK;
}

/*
 * Create and write env image into file
 * Return values:
 *  OK  - Success
 * -NOK - Open and Write failure
 */
static int write_final_image(char *output_image)
{
	int ret = 0;
	FILE *outfile;

	single->crc = crc32(0, (uint8_t *) single->data, ENV_SIZE);

	if (dbg_level) {
		printf("crc=%x\n", single->crc);
		printf("data=%s, image size=%d, sizeof(data)=%d\n",
		       single->data, CONFIG_ENV_SIZE, ENV_SIZE);
	}
	outfile = fopen(output_image, "w+");
	if (outfile == NULL) {
		fprintf(stderr, "## Error: Failed to open file\n");
		return -NOK;
	}

	ret = fwrite((single), CONFIG_ENV_SIZE, 1, outfile);
	fclose(outfile);
	if (!ret)
		return -NOK;

	return OK;
}

static int print_env_image()
{
	char *env, *nxt;

	/* print all env variables */
	for (env = single->data; *env; env = nxt + 1) {
		for (nxt = env; *nxt; ++nxt) {
			if (nxt >= single->data + ENV_SIZE) {
				fprintf(stderr, "## Error: "
					"environment not terminated\n");
				free(single);
				return -1;
			}
		}
		printf("%s\n", env);
	}
	return 0;
}

/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Martin Lundholm <martin.xa.lundholm@stericsson.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 *
 * cspsa_fp.h
 *
 * API for CSPSA Fast Parameters
 *
 */

#ifndef __CSPSA_FP_H__
#define __CSPSA_FP_H__

/* -------------------------------------------------------------------------- */

#include <common.h>
#include <part.h>

/* -------------------------------------------------------------------------- */

#define CSPSA_NBR_OF_FAST_PARAMETERS	4
#define CSPSA_PARTITION_NAME		"CSPSA0"

/* -------------------------------------------------------------------------- */

int cspsa_fp_read(block_dev_desc_t *block_dev, u32 key, u32 *value);

#endif

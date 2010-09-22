#
# Copyright (C) ST-Ericsson SA 2010
# Author: Rickard Evertsson <rickard.evertsson@stericsson.com>
# for ST-Ericsson.
#
# Influenced by boot/u-boot/tools/env/Makefile
#
# License terms: GNU General Public License (GPL) version 2
#

ifeq ($(SRCTREE),)
SRCTREE := ../..
endif

# Set up default output
ifeq ($(OUTPUT),)
OUTPUT := uboot-env.img
endif

include $(SRCTREE)/config.mk

ifeq ($(obj),)
obj := $(CURDIR)/
endif

SRCS	:= $(obj)crc32.c  mk_env_img.c
HEADERS	:= mk_env_img.h

.PHONY: all

all:	$(obj)mk_envimg create_envimg

$(obj)mk_envimg:	$(SRCS) $(HEADERS)
	$(HOSTCC) -g -I$(SRCTREE)/include -DUSE_HOSTCC $(SRCS) -o $(obj)mk_envimg

clean:
	rm -f $(obj)mk_envimg $(obj)crc32.c $(OUTPUT)

$(obj)crc32.c:
	ln -s $(SRCTREE)/lib_generic/crc32.c $(obj)crc32.c

create_envimg:
	$(obj)mk_envimg $(OUTPUT) $(INPUT)

#########################################################################

include ../../rules.mk

sinclude $(obj)depend

#########################################################################

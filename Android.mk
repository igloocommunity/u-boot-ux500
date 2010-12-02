#Android makefile to build u-boot as a part of Android Build

# Give other modules a nice, symbolic name to use as a dependent
# Yes, there are modules that cannot build unless uboot has
# been built. Typical (only?) example: linux kernel (needs mkimage program)
.phony: build-mkenvimg build-uboot clean-uboot

PRIVATE_UBOOT_ARGS := -C $(BOOT_PATH)/u-boot ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) O=$(UBOOT_OUTPUT)
PRIVATE_OUT := $(abspath $(PRODUCT_OUT)/root)

# The next three assignments are for building mk_envimg host tool.
# BUILD_UBOOT_ENV_IMG_OUTPUT and BUILD_UBOOT_ENV_IMG_INPUT should be set
# in BoardConfig.mk and are, if they are set, added to the cmdline.
PRIVATE_UBOOT_MK_ENVIMG_ARGS := -C $(BOOT_PATH)/u-boot/tools/mk_envimg

ifneq ($(BUILD_UBOOT_ENV_IMG_OUTPUT),)
PRIVATE_UBOOT_MK_ENVIMG_ARGS += OUTPUT=$(BUILD_UBOOT_ENV_IMG_OUTPUT)
endif

ifneq ($(BUILD_UBOOT_ENV_IMG_INPUT),)
PRIVATE_UBOOT_MK_ENVIMG_ARGS += INPUT=$(BUILD_UBOOT_ENV_IMG_INPUT)
endif

# Links the uboot build into the Android build
ALL_PREBUILT += build-uboot

# Add mk_envimg to the build
ALL_PREBUILT += build-mkenvimg

# Configures, builds and installs uboot. UBOOT_DEFCONFIG usually
# comes from the BoardConfig.mk file, but can be overridden on the
# command line or by an environment variable.
# If UBOOT_DEFCONFIG is set to 'local', configuration is skipped.
# This is useful if you want to play with your own, custom configuration.

build-uboot:
ifeq ($(UBOOT_DEFCONFIG),local)
	@echo Skipping uboot configuration, UBOOT_DEFCONFIG set to local
else
	$(MAKE) $(PRIVATE_UBOOT_ARGS) $(UBOOT_DEFCONFIG)
endif
	$(MAKE) $(PRIVATE_UBOOT_ARGS)

build-mkenvimg:
	$(MAKE) $(PRIVATE_UBOOT_MK_ENVIMG_ARGS) all

# An Android clean removes the files built for the current HW configuration,
# such as u8500,
# while a clobber removes all built files (rm -rf $(OUT_DIR)).

clean clobber: clean-uboot clean-mk_envimg

clean-uboot:
	$(MAKE) $(PRIVATE_UBOOT_ARGS) clean

clean-mk_envimg:
	$(MAKE) $(PRIVATE_UBOOT_MK_ENVIMG_ARGS) clean

installclean: installclean-uboot

installclean-uboot:
	rm -f u-boot.bin u-boot.map u-boot.srec

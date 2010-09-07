#Android makefile to build u-boot as a part of Android Build

ifeq ($(ENABLE_CR264527),true)
# Give other modules a nice, symbolic name to use as a dependent
# Yes, there are modules that cannot build unless uboot has
# been built. Typical (only?) example: linux kernel (needs mkimage program)
.phony: build-uboot clean-uboot


PRIVATE_UBOOT_ARGS := -C $(BOOT_PATH)/u-boot ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
PRIVATE_OUT := $(abspath $(PRODUCT_OUT)/root)

# Links the uboot build into the Android build
ALL_PREBUILT += build-uboot


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


# An Android clean removes the files built for the current HW configuration,
# such as u8500,
# while a clobber removes all built files (rm -rf $(OUT_DIR)).
# Uboot only has one build tree, so clean and clobber will be
# the same.

clean clobber : clean-uboot

clean-uboot:
	$(MAKE) $(PRIVATE_UBOOT_ARGS) clean

installclean: installclean-uboot

installclean-uboot:
	rm -f u-boot.bin u-boot.map u-boot.srec


endif

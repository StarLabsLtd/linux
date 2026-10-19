# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_GOOGLE_SMI)		+= gsmi.o
obj-$(CONFIG_COREBOOT_TABLE)               += coreboot_table.o
obj-$(CONFIG_COREBOOT_CFR)		+= coreboot-cfr-driver.o
coreboot-cfr-driver-y			:= coreboot-cfr.o coreboot-cfr-atomic.o
obj-$(CONFIG_COREBOOT_CFR_KUNIT_TEST)	+= coreboot-cfr-test.o
obj-$(CONFIG_COREBOOT_FRAMEBUFFER)         += framebuffer-coreboot.o
obj-$(CONFIG_COREBOOT_MEMCONSOLE_COMMON)   += memconsole.o
obj-$(CONFIG_COREBOOT_MEMCONSOLE)          += memconsole-coreboot.o
obj-$(CONFIG_GOOGLE_MEMCONSOLE_X86_LEGACY) += memconsole-x86-legacy.o

# Must come after coreboot_table.o, as this driver depends on that bus type.
obj-$(CONFIG_COREBOOT_CBMEM)		+= cbmem.o

vpd-sysfs-y := vpd.o vpd_decode.o
obj-$(CONFIG_GOOGLE_VPD)		+= vpd-sysfs.o

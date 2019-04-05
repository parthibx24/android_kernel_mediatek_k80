# drivers/barcelona/gps/Makefile
#
# Makefile for the Barcelona GPS driver.
#
# Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
# Author: Dimitry Andric <dimitry.andric@tomtom.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.

###############################################################################
# Necessary Check

ifeq ($(AUTOCONF_H),)
    $(error AUTOCONF_H is not defined)
endif

ccflags-y += -imacros $(AUTOCONF_H)

ifeq ($(CONFIG_MTK_COMBO_CHIP),)
    $(error CONFIG_MTK_COMBO_CHIP not defined)
endif

ifeq ($(TARGET_BUILD_VARIANT),$(filter $(TARGET_BUILD_VARIANT),userdebug user))
    ldflags-y += -s
endif

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y
###############################################################################

# only WMT align this design flow, but gps use this also.
ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE

ifeq ($(CONFIG_ARM64), y)
    ccflags-y += -D CONFIG_MTK_WCN_ARM64
endif

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
    ccflags-y += -D WMT_IDC_SUPPORT=1
else
    ccflags-y += -D WMT_IDC_SUPPORT=0
endif
ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/connectivity/common
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach
###############################################################################

MODULE_NAME := gps_drv
obj-m += $(MODULE_NAME).o
WMT_SRC_FOLDER := $(TOP)/vendor/mediatek/kernel_modules/connectivity/common

ifneq ($(filter "CONSYS_%",$(CONFIG_MTK_COMBO_CHIP)),)
        ccflags-y += -DSOC_CO_CLOCK_FLAG=1
        ccflags-y += -DWMT_CREATE_NODE_DYNAMIC=1
        ccflags-y += -DREMOVE_MK_NODE=0

ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/$(MTK_PLATFORM)/include

else
        ccflags-y += -DSOC_CO_CLOCK_FLAG=0
        ccflags-y += -DWMT_CREATE_NODE_DYNAMIC=0
        ccflags-y += -DREMOVE_MK_NODE=1
endif

ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/linux/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/core/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/platform/include

ifneq ($(CONFIG_MTK_GPS),)
$(MODULE_NAME)-objs += gps.o
endif
$(MODULE_NAME)-objs += stp_chrdev_gps.o
ifneq ($(CONFIG_MTK_GPS_EMI),)
$(MODULE_NAME)-objs += gps_emi.o
endif

# EOF

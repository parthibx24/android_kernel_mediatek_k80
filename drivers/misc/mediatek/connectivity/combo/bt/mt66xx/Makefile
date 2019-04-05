###############################################################################
# Bluetooth character device driver

###############################################################################
# Necessary Check

ifeq ($(AUTOCONF_H),)
    $(error AUTOCONF_H is not defined)
endif

ccflags-y += -imacros $(AUTOCONF_H)

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y
###############################################################################
# To add WMT dependent Macro and header file, will be removed later

ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

###############################################################################
# To include BT driver dependent header file

WMT_SRC_FOLDER := $(TOP)/vendor/mediatek/kernel_modules/connectivity/common
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/linux/include

###############################################################################

MODULE_NAME := bt_drv
obj-m += $(MODULE_NAME).o

ccflags-y += -D CREATE_NODE_DYNAMIC=1

$(MODULE_NAME)-objs += stp_chrdev_bt.o

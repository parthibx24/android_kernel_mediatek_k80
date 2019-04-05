###############################################################################
# Necessary Check

ifeq ($(AUTOCONF_H),)
    $(error AUTOCONF_H is not defined)
endif

ccflags-y += -imacros $(AUTOCONF_H)

ifeq ($(CONFIG_MTK_COMBO_CHIP),)
    $(error CONFIG_MTK_COMBO_CHIP not defined)
endif

#Force to m
export CONFIG_MTK_COMBO_WIFI=m

ifeq ($(TARGET_BUILD_VARIANT),$(filter $(TARGET_BUILD_VARIANT),userdebug user))
    ldflags-y += -s
endif

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y
###############################################################################

#### WLAN driver version check

ifneq ($(filter "MT6630",$(CONFIG_MTK_COMBO_CHIP)),)
    ccflags-y += -D WLAN_GEN3=1
endif

ifneq ($(filter "CONSYS_6797" "CONSYS_6759" "CONSYS_6758",$(CONFIG_MTK_COMBO_CHIP)),)
    ccflags-y += -D WLAN_GEN3=1
else ifneq ($(filter "CONSYS_%",$(CONFIG_MTK_COMBO_CHIP)),)
    ccflags-y += -D WLAN_GEN2=1
endif

ifneq ($(filter "MT6632",$(CONFIG_MTK_COMBO_CHIP)),)
    ccflags-y += -D WLAN_GEN4=1
endif

###############################################################################
WMT_SRC_FOLDER := $(subst wmt_chrdev_wifi.ko,wmt_drv.ko,$(src))

ccflags-y += \
    -I$(KERNEL_DIR)/drivers/misc/mediatek/include/mt-plat \
    -I$(TOP)/vendor/mediatek/kernel_modules/connectivity/common/common_main/include \
    -I$(TOP)/vendor/mediatek/kernel_modules/connectivity/common/common_main/linux/include

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
    ccflags-y += -D WMT_IDC_SUPPORT=1
else
    ccflags-y += -D WMT_IDC_SUPPORT=0
endif

ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

ifeq ($(CFG_SPM_WORKAROUND_FOR_HOTSPOT), 1)
    ccflags-y += -D CFG_SPM_WORKAROUND_FOR_HOTSPOT=1
endif

ccflags-y += -D CREATE_NODE_DYNAMIC=1

ifeq ($(CONFIG_MTK_TC1_FEATURE), y)
	ccflags-y += -D CFG_TC1_FEATURE=1
        ccflags-y += -DCFG_USE_AOSP_TETHERING_NAME
else
	ccflags-y += -D CFG_TC1_FEATURE=0
endif
ifeq ($(CONFIG_MTK_TC10_FEATURE), y)
    ccflags-y += -DCFG_USE_AOSP_TETHERING_NAME
endif

# Wi-Fi character device driver
obj-m += wmt_chrdev_wifi.o


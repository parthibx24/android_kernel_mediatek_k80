#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

obj-$(CONFIG_MTK_FB) += \
			ddp_wdma_ex.o          \
			ddp_dsi.o              \
			ddp_drv.o              \
			ddp_path.o             \
			ddp_dump.o             \
			ddp_manager.o          \
			ddp_irq.o              \
			ddp_info.o             \
			ddp_pwm_mux.o          \
			ddp_color_format.o     \
			ddp_mmp.o              \
			ddp_met.o              \
			ddp_dpi.o              \
			disp_recorder.o

ccflags-y +=	-I$(srctree)/drivers/misc/mediatek/video/include/                     \
		-I$(srctree)/drivers/misc/mediatek/video/common/                      \
		-I$(srctree)/drivers/misc/mediatek/video/common/rdma10/               \
		-I$(srctree)/drivers/misc/mediatek/video/common/wdma10/               \
		-I$(srctree)/drivers/misc/mediatek/cmdq/v2/                           \
		-I$(srctree)/drivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)/           \
		-I$(srctree)/drivers/misc/mediatek/externaldisplay/$(MTK_PLATFORM)    \
		-I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach/ \
		-I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)         \
		-I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v1/         \
		-I$(srctree)/drivers/misc/mediatek/base/power/include/                \
		-I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/               \
		-I$(srctree)/drivers/misc/mediatek/mmp/                               \
		-I$(srctree)/drivers/misc/mediatek/sync/                              \
		-I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/      \
		-I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/     \

ifeq ($(CONFIG_MACH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/mt6735/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/mt6735/
ccflags-y += -DCONFIG_FOR_ARCH_M1
obj-y += $(subst ",,mt6735)/
endif
ifeq ($(CONFIG_MACH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/mt6735m/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/mt6735m/
ccflags-y += -DCONFIG_FOR_ARCH_M2
obj-y += $(subst ",,mt6735m)/
endif
ifeq ($(CONFIG_MACH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/mt6753/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/mt6753/
ccflags-y += -DCONFIG_FOR_ARCH_M3
obj-y += $(subst ",,mt6753)/
endif

ifeq ($(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT),y)
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += \
	-I$(srctree)/../vendor/trustonic/platform/$(MTK_PLATFORM)/trustlets/tplay/Tltplay/public/ \
	-I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/trustzone/tplay/Tltplay/public/

include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif
endif

ifeq ($(CONFIG_MACH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/mt6735m/
endif

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc/
ccflags-y += -I$(srctree)/drivers/staging/android/ion \
             -I$(srctree)/drivers/staging/android/ion/mtk
ccflags-y += -DMET_USER_EVENT_SUPPORT

ifeq ($(CONFIG_MMPROFILE),y)
ccflags-y += -DDEFAULT_MMP_ENABLE
endif

ifeq ($(CONFIG_MTK_INTERNAL_HDMI_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hdmi/
endif

ifeq ($(CONFIG_MTK_INTERNAL_MHL_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mhltx/
endif

ifeq ($(CONFIG_MTK_DITHERING_SUPPORT), y)
ccflags-y += -DDITHERING_SUPPORT
endif

ifeq ($(CONFIG_MTK_GPU_SUPPORT), y)
ccflags-y += -DHWGPU_SUPPORT
endif

ifeq ($(MTK_MIRAVISION_SUPPORT), yes)
    ccflags-y += -DCONFIG_FOR_SOURCE_PQ
endif

ccflags-$(CONFIG_MTK_LCM_DEVICE_TREE_SUPPORT) += -DMTK_LCM_DEVICE_TREE_SUPPORT

ifndef CONFIG_MTK_ENG_BUILD
ccflags-y += -D_MTK_USER_
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI), "MT8193")
ccflags-y +=    -DHDMI_MT8193_SUPPORT
endif

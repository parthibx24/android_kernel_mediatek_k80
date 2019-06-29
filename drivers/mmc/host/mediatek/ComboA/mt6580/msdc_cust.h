/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MSDC_CUST_MT6580_H_
#define _MSDC_CUST_MT6580_H_

#ifdef CONFIG_FPGA_EARLY_PORTING
#define FPGA_PLATFORM
#else
/*#define MTK_MSDC_BRINGUP_DEBUG*/
#endif

#include <dt-bindings/mmc/mt6580-msdc.h>

//#define CONFIG_MTK_MSDC_BRING_UP_BYPASS

#if !defined(FPGA_PLATFORM)
#include <mach/mt_clkmgr.h>
#endif

#ifndef CONFIG_MTK_MSDC_BRING_UP_BYPASS
#define spm_resource_req(a, b)
#define SPM_RESOURCE_USER_MSDC
#define SPM_RESOURCE_RELEASE
#define SPM_RESOURCE_ALL
#endif

/**************************************************************/
/* Section 1: Device Tree                                     */
/**************************************************************/
/* Names used for device tree lookup */
#define DT_COMPATIBLE_NAME      "mediatek,msdc"

#define MSDC0_IOCFG_NAME        "mediatek,IOCFG_B"
#define MSDC1_IOCFG_NAME        "mediatek,IOCFG_R"

/**************************************************************/
/* Section 2: Power                                           */
/**************************************************************/
#define SD_POWER_DEFAULT_ON     (0)

#if !defined(FPGA_PLATFORM)
#include <mt-plat/upmu_common.h>

#define REG_VEMC_VOSEL          MT6350_VEMC_3V3_CON1
#define REG_VEMC_EN             MT6350_VEMC_3V3_CON0

#define REG_VMC_VOSEL           MT6350_VMC_CON1
#define REG_VMC_EN              MT6350_VMC_CON0

#define REG_VMCH_VOSEL          MT6350_VMCH_CON1
#define REG_VMCH_EN             MT6350_VMCH_CON0

#define MASK_VEMC_VOSEL         0x1
#define SHIFT_VEMC_VOSEL        7
#define FIELD_VEMC_VOSEL        (MASK_VEMC_VOSEL << SHIFT_VEMC_VOSEL)

#define MASK_VEMC_EN            0x1
#define SHIFT_VEMC_EN           14
#define FIELD_VEMC_EN           (MASK_VEMC_EN << SHIFT_VEMC_EN)

#define MASK_VMC_VOSEL          0x1
#define SHIFT_VMC_VOSEL         4
#define FIELD_VMC_VOSEL         (MASK_VMC_VOSEL << SHIFT_VMC_VOSEL)

#define MASK_VMC_EN             0x1
#define SHIFT_VMC_EN            12
#define FIELD_VMC_EN            (MASK_VMC_EN << SHIFT_VMC_EN)

#define MASK_VMCH_VOSEL         0x1
#define SHIFT_VMCH_VOSEL        7
#define FIELD_VMCH_VOSEL        (MASK_VMCH_VOSEL << SHIFT_VMCH_VOSEL)

#define MASK_VMCH_EN            0x1
#define SHIFT_VMCH_EN           14
#define FIELD_VMCH_EN           (MASK_VMCH_EN << SHIFT_VMCH_EN)
#endif

/**************************************************************/
/* Section 3: Clock                                           */
/**************************************************************/
/* list the other value by clock owners' clock table doc if needed */
#define MSDC_CLKSRC_187MHZ      2

#define MSDC0_SRC_0             125000000
#define MSDC0_SRC_1             150000000
#define MSDC0_SRC_2             187000000
#define MSDC0_SRC_3             178000000
#define MSDC0_SRC_4             214000000
#define MSDC0_SRC_5             187000000
#define MSDC0_SRC_6             26000000
#define MSDC0_SRC_7             208000000

#define MSDC1_SRC_0             125000000
#define MSDC1_SRC_1             150000000
#define MSDC1_SRC_2             187000000
#define MSDC1_SRC_3             178000000
#define MSDC1_SRC_4             214000000
#define MSDC1_SRC_5             187000000
#define MSDC1_SRC_6             26000000
#define MSDC1_SRC_7             208000000

#define MSDC_SRC_FPGA           12000000

/**************************************************************/
/* Section 4: GPIO and Pad                                    */
/**************************************************************/
/*--------------------------------------------------------------------------*/
/* MSDC0~1 GPIO and IO Pad Configuration Base                               */
/*--------------------------------------------------------------------------*/
#define MSDC_GPIO_BASE          gpio_base               /* 0x10005000 */
#define MSDC0_IO_PAD_BASE       (msdc_io_cfg_bases[0])  /* 0x10015000 */
#define MSDC1_IO_PAD_BASE       (msdc_io_cfg_bases[1])  /* 0x10017000 */

/*--------------------------------------------------------------------------*/
/* MSDC GPIO Related Register                                               */
/*--------------------------------------------------------------------------*/

/*
 * MSDC0 IO Pad Pinmux register definition.
 */
#define MSDC0_GPIO_MODE5_MWR_ADDR   (MSDC_GPIO_BASE + 0xB0)
#define MSDC0_GPIO_MODE6_MWR_ADDR   (MSDC_GPIO_BASE + 0xC0)

#define MSDC0_GPIO_MODE5_ADDR       (MSDC_GPIO_BASE + 0xA4)
#define MSDC0_GPIO_MODE6_ADDR       (MSDC_GPIO_BASE + 0xB4)

/* MSDC0 41:CLK, 42:CMD, 43:DAT0, 44:DAT1, 45:DAT2, 46:DAT3, 47:DAT4 */
#define MSDC0_CLK_PINMUX_ADDR   MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_CLK_PINMUX_BITS   (0xF << 4)
#define MSDC0_CLK_PINMUX_VAL    (0x9 << 4)

#define MSDC0_CMD_PINMUX_ADDR   MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_CMD_PINMUX_BITS   (0xF << 8)
#define MSDC0_CMD_PINMUX_VAL    (0x9 << 8)

#define MSDC0_DAT0_PINMUX_ADDR  MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_DAT0_PINMUX_BITS  (0xF << 12)
#define MSDC0_DAT0_PINMUX_VAL   (0x9 << 12)

#define MSDC0_DAT1_PINMUX_ADDR  MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_DAT1_PINMUX_BITS  (0xF << 16)
#define MSDC0_DAT1_PINMUX_VAL   (0x9 << 16)

#define MSDC0_DAT2_PINMUX_ADDR  MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_DAT2_PINMUX_BITS  (0xF << 20)
#define MSDC0_DAT2_PINMUX_VAL   (0x9 << 20)

#define MSDC0_DAT3_PINMUX_ADDR  MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_DAT3_PINMUX_BITS  (0xF << 24)
#define MSDC0_DAT3_PINMUX_VAL   (0x9 << 24)

#define MSDC0_DAT4_PINMUX_ADDR  MSDC0_GPIO_MODE5_MWR_ADDR
#define MSDC0_DAT4_PINMUX_BITS  (0xF << 28)
#define MSDC0_DAT4_PINMUX_VAL   (0x9 << 28)

/* MSDC0 48:DAT5, 49:DAT6, 50:DAT7, 51:RSTB */
#define MSDC0_DAT5_PINMUX_ADDR  MSDC0_GPIO_MODE6_MWR_ADDR
#define MSDC0_DAT5_PINMUX_BITS  (0xF << 0)
#define MSDC0_DAT5_PINMUX_VAL   (0x9 << 0)

#define MSDC0_DAT6_PINMUX_ADDR  MSDC0_GPIO_MODE6_MWR_ADDR
#define MSDC0_DAT6_PINMUX_BITS  (0xF << 4)
#define MSDC0_DAT6_PINMUX_VAL   (0x9 << 4)

#define MSDC0_DAT7_PINMUX_ADDR  MSDC0_GPIO_MODE6_MWR_ADDR
#define MSDC0_DAT7_PINMUX_BITS  (0xF << 8)
#define MSDC0_DAT7_PINMUX_VAL   (0x9 << 8)

#define MSDC0_RST_PINMUX_ADDR   MSDC0_GPIO_MODE6_MWR_ADDR
#define MSDC0_RST_PINMUX_BITS   (0xF << 12)
#define MSDC0_RST_PINMUX_VAL    (0x9 << 12)

/*
 * MSDC1 IO Pad Pinmux register definition.
 */
#define MSDC1_GPIO_MODE6_MWR_ADDR   (MSDC_GPIO_BASE + 0xC0)
#define MSDC1_GPIO_MODE7_MWR_ADDR   (MSDC_GPIO_BASE + 0xD0)

#define MSDC1_GPIO_MODE6_ADDR       (MSDC_GPIO_BASE + 0xB4)
#define MSDC1_GPIO_MODE7_ADDR       (MSDC_GPIO_BASE + 0xC4)

/* MSDC1 52:CLK, 53:CMD, 54:DAT3, 55:DAT2 */
#define MSDC1_CLK_PINMUX_ADDR   MSDC1_GPIO_MODE6_MWR_ADDR
#define MSDC1_CLK_PINMUX_BITS   (0xF << 16)
#define MSDC1_CLK_PINMUX_VAL    (0x9 << 16)

#define MSDC1_CMD_PINMUX_ADDR   MSDC1_GPIO_MODE6_MWR_ADDR
#define MSDC1_CMD_PINMUX_BITS   (0xF << 20)
#define MSDC1_CMD_PINMUX_VAL    (0x9 << 20)

#define MSDC1_DAT3_PINMUX_ADDR  MSDC1_GPIO_MODE6_MWR_ADDR
#define MSDC1_DAT3_PINMUX_BITS  (0xF << 24)
#define MSDC1_DAT3_PINMUX_VAL   (0x9 << 24)

#define MSDC1_DAT2_PINMUX_ADDR  MSDC1_GPIO_MODE6_MWR_ADDR
#define MSDC1_DAT2_PINMUX_BITS  (0xF << 28)
#define MSDC1_DAT2_PINMUX_VAL   (0x9 << 28)

/* MSDC1 56:DAT1, 57:DAT0 */
#define MSDC1_DAT1_PINMUX_ADDR  MSDC1_GPIO_MODE7_MWR_ADDR
#define MSDC1_DAT1_PINMUX_BITS  (0xF << 0)
#define MSDC1_DAT1_PINMUX_VAL   (0x9 << 0)

#define MSDC1_DAT0_PINMUX_ADDR  MSDC1_GPIO_MODE7_MWR_ADDR
#define MSDC1_DAT0_PINMUX_BITS  (0xF << 4)
#define MSDC1_DAT0_PINMUX_VAL   (0x9 << 4)

/*
 * MSDC0 IO Pad Configuration register definition.
 */
#define MSDC0_SELGP_OFFSET      (0x00A8)
#define MSDC0_SELGP_BASE        (MSDC0_IO_PAD_BASE + 0x00A8)
#define MSDC0_SELGP_SET         (MSDC0_IO_PAD_BASE + 0x00AC)
#define MSDC0_SELGP_CLR         (MSDC0_IO_PAD_BASE + 0x00B0)
#define MSDC0_SELGP_ALL_MASK    (0x7FF << 0)

#define MSDC0_IES_CFG_OFFSET    (0x003C)
#define MSDC0_IES_CFG_BASE      (MSDC0_IO_PAD_BASE + 0x003C)
#define MSDC0_IES_CFG_SET       (MSDC0_IO_PAD_BASE + 0x0040)
#define MSDC0_IES_CFG_CLR       (MSDC0_IO_PAD_BASE + 0x0044)
#define MSDC0_IES_ALL_MASK      (0x7FF << 10)

#define MSDC0_SR_CFG_OFFSET     (0x0048)
#define MSDC0_SR_CFG_BASE       (MSDC0_IO_PAD_BASE + 0x0048)
#define MSDC0_SR_CFG_SET        (MSDC0_IO_PAD_BASE + 0x004C)
#define MSDC0_SR_CFG_CLR        (MSDC0_IO_PAD_BASE + 0x0050)
#define MSDC0_SR_ALL_MASK       (0x7FF << 10)

#define MSDC0_SMT_CFG_OFFSET    (0x0054)
#define MSDC0_SMT_CFG_BASE      (MSDC0_IO_PAD_BASE + 0x0054)
#define MSDC0_SMT_CFG_SET       (MSDC0_IO_PAD_BASE + 0x0058)
#define MSDC0_SMT_CFG_CLR       (MSDC0_IO_PAD_BASE + 0x005C)
#define MSDC0_SMT_ALL_MASK      (0x7FF << 10)

#define MSDC0_TDSEL_CFG_OFFSET  (0x0090)
#define MSDC0_TDSEL_BASE        (MSDC0_IO_PAD_BASE + 0x0090)
#define MSDC0_TDSEL_SET         (MSDC0_IO_PAD_BASE + 0x0094)
#define MSDC0_TDSEL_CLR         (MSDC0_IO_PAD_BASE + 0x0098)
#define MSDC0_TDSEL_ALL_MASK    (0xF  <<  4)

#define MSDC0_RDSEL_CFG_OFFSET  (0x0084)
#define MSDC0_RDSEL_BASE        (MSDC0_IO_PAD_BASE + 0x0084)
#define MSDC0_RDSEL_SET         (MSDC0_IO_PAD_BASE + 0x0088)
#define MSDC0_RDSEL_CLR         (MSDC0_IO_PAD_BASE + 0x008C)
#define MSDC0_RDSEL_ALL_MASK    (0x3F <<  2)

#define MSDC0_PULL_R0_CFG_OFFSET    (0x0024)
#define MSDC0_PULL_R0_CFG_BASE  (MSDC0_IO_PAD_BASE + 0x0024)
#define MSDC0_PULL_R0_CFG_SET   (MSDC0_IO_PAD_BASE + 0x0028)
#define MSDC0_PULL_R0_CFG_CLR   (MSDC0_IO_PAD_BASE + 0x002c)

#define MSDC0_PULL_R1_CFG_OFFSET    (0x0030)
#define MSDC0_PULL_R1_CFG_BASE  (MSDC0_IO_PAD_BASE + 0x0030)
#define MSDC0_PULL_R1_CFG_SET   (MSDC0_IO_PAD_BASE + 0x0034)
#define MSDC0_PULL_R1_CFG_CLR   (MSDC0_IO_PAD_BASE + 0x0038)

#define MSDC0_DAT7_PULL_R       (0x1 << 0)
#define MSDC0_DAT6_PULL_R       (0x1 << 1)
#define MSDC0_DAT5_PULL_R       (0x1 << 2)
#define MSDC0_DAT4_PULL_R       (0x1 << 3)
#define MSDC0_DAT3_PULL_R       (0x1 << 4)
#define MSDC0_DAT2_PULL_R       (0x1 << 5)
#define MSDC0_DAT1_PULL_R       (0x1 << 6)
#define MSDC0_DAT0_PULL_R       (0x1 << 7)
#define MSDC0_CLK_PULL_R        (0x1 << 8)
#define MSDC0_RST_PULL_R        (0x1 << 9)
#define MSDC0_CMD_PULL_R        (0x1 << 10)

#define MSDC0_PULL_R_ALL_MASK   (MSDC0_DAT7_PULL_R | \
				MSDC0_DAT6_PULL_R | \
				MSDC0_DAT5_PULL_R | \
				MSDC0_DAT4_PULL_R | \
				MSDC0_DAT3_PULL_R | \
				MSDC0_DAT2_PULL_R | \
				MSDC0_DAT1_PULL_R | \
				MSDC0_DAT0_PULL_R | \
				MSDC0_CLK_PULL_R  | \
				MSDC0_RST_PULL_R  | \
				MSDC0_CMD_PULL_R)

#define MSDC0_PULL_SEL_CFG_OFFSET    (0x0018)
#define MSDC0_PULL_SEL_CFG_BASE (MSDC0_IO_PAD_BASE + 0x0018)
#define MSDC0_PULL_SEL_CFG_SET  (MSDC0_IO_PAD_BASE + 0x001C)
#define MSDC0_PULL_SEL_CFG_CLR  (MSDC0_IO_PAD_BASE + 0x0020)

#define MSDC0_DAT7_PULL_SEL     (0x1 << 0)
#define MSDC0_DAT6_PULL_SEL     (0x1 << 1)
#define MSDC0_DAT5_PULL_SEL     (0x1 << 2)
#define MSDC0_DAT4_PULL_SEL     (0x1 << 3)
#define MSDC0_DAT3_PULL_SEL     (0x1 << 4)
#define MSDC0_DAT2_PULL_SEL     (0x1 << 5)
#define MSDC0_DAT1_PULL_SEL     (0x1 << 6)
#define MSDC0_DAT0_PULL_SEL     (0x1 << 7)
#define MSDC0_CLK_PULL_SEL      (0x1 << 8)
#define MSDC0_RST_PULL_SEL      (0x1 << 9)
#define MSDC0_CMD_PULL_SEL      (0x1 << 10)
#define MSDC0_PULL_SEL_ALL_MASK (MSDC0_DAT7_PULL_SEL | \
				MSDC0_DAT6_PULL_SEL | \
				MSDC0_DAT5_PULL_SEL | \
				MSDC0_DAT4_PULL_SEL | \
				MSDC0_DAT3_PULL_SEL | \
				MSDC0_DAT2_PULL_SEL | \
				MSDC0_DAT1_PULL_SEL | \
				MSDC0_DAT0_PULL_SEL | \
				MSDC0_CLK_PULL_SEL  | \
				MSDC0_CMD_PULL_SEL)

#define MSDC0_DRVING0_OFFSET (0x006C)
#define MSDC0_DRVING0_BASE  (MSDC0_IO_PAD_BASE + 0x006C)
#define MSDC0_DRVING0_SET   (MSDC0_IO_PAD_BASE + 0x0070)
#define MSDC0_DRVING0_CLR   (MSDC0_IO_PAD_BASE + 0x0074)
#define MSDC0_DAT7_DRVING   (0x7 << 20)     /* RW */
#define MSDC0_DAT6_DRVING   (0x7 << 23)     /* RW */
#define MSDC0_DAT5_DRVING   (0x7 << 26)     /* RW */
#define MSDC0_DAT4_DRVING   (0x7 << 29)     /* RW */
#define MSDC0_DRVING0_ALL_MASK   (MSDC0_DAT7_DRVING  | \
				MSDC0_DAT6_DRVING  | \
				MSDC0_DAT5_DRVING  | \
				MSDC0_DAT4_DRVING)

#define MSDC0_DRVING1_OFFSET (0x0078)
#define MSDC0_DRVING1_BASE  (MSDC0_IO_PAD_BASE + 0x0078)
#define MSDC0_DRVING1_SET   (MSDC0_IO_PAD_BASE + 0x007C)
#define MSDC0_DRVING1_CLR   (MSDC0_IO_PAD_BASE + 0x0080)
#define MSDC0_DAT3_DRVING   (0x7 <<  0)     /* RW */
#define MSDC0_DAT2_DRVING   (0x7 <<  3)     /* RW */
#define MSDC0_DAT1_DRVING   (0x7 <<  6)     /* RW */
#define MSDC0_DAT0_DRVING   (0x7 <<  9)     /* RW */
#define MSDC0_CLK_DRVING    (0x7 << 12)     /* RW */
#define MSDC0_RST_DRVING    (0x7 << 15)     /* RW */
#define MSDC0_CMD_DRVING    (0x7 << 18)     /* RW */
#define MSDC0_DRVING1_ALL_MASK   (MSDC0_CMD_DRVING   | \
				MSDC0_RST_DRVING   | \
				MSDC0_CLK_DRVING   | \
				MSDC0_DAT0_DRVING  | \
				MSDC0_DAT1_DRVING  | \
				MSDC0_DAT2_DRVING  | \
				MSDC0_DAT3_DRVING)

/*
 * MSDC1 IO Pad Configuration register definition.
 */
#define MSDC1_SELGP_OFFSET      (0x009C)
#define MSDC1_SELGP_BASE        (MSDC1_IO_PAD_BASE + 0x009C)
#define MSDC1_SELGP_SET         (MSDC1_IO_PAD_BASE + 0x00A0)
#define MSDC1_SELGP_CLR         (MSDC1_IO_PAD_BASE + 0x00A4)
#define MSDC1_SELGP_ALL_MASK    (0x3F << 0)

#define MSDC1_IES_CFG_OFFSET    (0x003C)
#define MSDC1_IES_CFG_BASE      (MSDC1_IO_PAD_BASE + 0x003C)
#define MSDC1_IES_CFG_SET       (MSDC1_IO_PAD_BASE + 0x0040)
#define MSDC1_IES_CFG_CLR       (MSDC1_IO_PAD_BASE + 0x0044)
#define MSDC1_IES_ALL_MASK      (0x3F << 0)

#define MSDC1_SR_CFG_OFFSET     (0x0048)
#define MSDC1_SR_CFG_BASE       (MSDC1_IO_PAD_BASE + 0x0048)
#define MSDC1_SR_CFG_SET        (MSDC1_IO_PAD_BASE + 0x004C)
#define MSDC1_SR_CFG_CLR        (MSDC1_IO_PAD_BASE + 0x0050)
#define MSDC1_SR_ALL_MASK       (0x3F << 0)

#define MSDC1_SMT_CFG_OFFSET    (0x0054)
#define MSDC1_SMT_CFG_BASE      (MSDC1_IO_PAD_BASE + 0x0054)
#define MSDC1_SMT_CFG_SET       (MSDC1_IO_PAD_BASE + 0x0058)
#define MSDC1_SMT_CFG_CLR       (MSDC1_IO_PAD_BASE + 0x005C)
#define MSDC1_SMT_ALL_MASK      (0x3F << 0)

#define MSDC1_TDSEL_CFG_OFFSET  (0x0090)
#define MSDC1_TDSEL_BASE        (MSDC1_IO_PAD_BASE + 0x0090)
#define MSDC1_TDSEL_SET         (MSDC1_IO_PAD_BASE + 0x0094)
#define MSDC1_TDSEL_CLR         (MSDC1_IO_PAD_BASE + 0x0098)
#define MSDC1_TDSEL_ALL_MASK    (0xF << 0)

#define MSDC1_RDSEL_CFG_OFFSET  (0x0084)
#define MSDC1_RDSEL_BASE        (MSDC1_IO_PAD_BASE + 0x0084)
#define MSDC1_RDSEL_SET         (MSDC1_IO_PAD_BASE + 0x0088)
#define MSDC1_RDSEL_CLR         (MSDC1_IO_PAD_BASE + 0x008C)
#define MSDC1_RDSEL_ALL_MASK    (0x3F << 0)

#define MSDC1_PULL_R0_CFG_OFFSET    (0x0024)
#define MSDC1_PULL_R0_CFG_BASE  (MSDC1_IO_PAD_BASE + 0x0024)
#define MSDC1_PULL_R0_CFG_SET   (MSDC1_IO_PAD_BASE + 0x0028)
#define MSDC1_PULL_R0_CFG_CLR   (MSDC1_IO_PAD_BASE + 0x002c)

#define MSDC1_PULL_R1_CFG_OFFSET    (0x0030)
#define MSDC1_PULL_R1_CFG_BASE  (MSDC1_IO_PAD_BASE + 0x0030)
#define MSDC1_PULL_R1_CFG_SET   (MSDC1_IO_PAD_BASE + 0x0034)
#define MSDC1_PULL_R1_CFG_CLR   (MSDC1_IO_PAD_BASE + 0x0038)

#define MSDC1_CLK_PULL_R        (0x1 << 0)
#define MSDC1_CMD_PULL_R        (0x1 << 1)
#define MSDC1_DAT3_PULL_R       (0x1 << 2)
#define MSDC1_DAT2_PULL_R       (0x1 << 3)
#define MSDC1_DAT1_PULL_R       (0x1 << 4)
#define MSDC1_DAT0_PULL_R       (0x1 << 5)

#define MSDC1_PULL_R_ALL_MASK   (MSDC1_CLK_PULL_R  | \
				MSDC1_CMD_PULL_R  | \
				MSDC1_DAT3_PULL_R | \
				MSDC1_DAT2_PULL_R | \
				MSDC1_DAT1_PULL_R | \
				MSDC1_DAT0_PULL_R)

#define MSDC1_PULL_SEL_CFG_OFFSET    (0x0018)
#define MSDC1_PULL_SEL_CFG_BASE (MSDC1_IO_PAD_BASE + 0x0018)
#define MSDC1_PULL_SEL_CFG_SET  (MSDC1_IO_PAD_BASE + 0x001C)
#define MSDC1_PULL_SEL_CFG_CLR  (MSDC1_IO_PAD_BASE + 0x0020)

#define MSDC1_CLK_PULL_SEL      (0x1 << 0)
#define MSDC1_CMD_PULL_SEL      (0x1 << 1)
#define MSDC1_DAT3_PULL_SEL     (0x1 << 2)
#define MSDC1_DAT2_PULL_SEL     (0x1 << 3)
#define MSDC1_DAT1_PULL_SEL     (0x1 << 4)
#define MSDC1_DAT0_PULL_SEL     (0x1 << 5)
#define MSDC1_PULL_SEL_ALL_MASK (MSDC1_CLK_PULL_SEL  | \
				MSDC1_CMD_PULL_SEL  | \
				MSDC1_DAT3_PULL_SEL | \
				MSDC1_DAT2_PULL_SEL | \
				MSDC1_DAT1_PULL_SEL | \
				MSDC1_DAT0_PULL_SEL)

#define MSDC1_DRVING_OFFSET     (0x006C)
#define MSDC1_DRVING_BASE       (MSDC1_IO_PAD_BASE + 0x006C)
#define MSDC1_DRVING_SET        (MSDC1_IO_PAD_BASE + 0x0070)
#define MSDC1_DRVING_CLR        (MSDC1_IO_PAD_BASE + 0x0074)
#define MSDC1_CLK_DRVING        (0x7 <<  0)
#define MSDC1_CMD_DRVING        (0x7 <<  3)
#define MSDC1_DAT3_DRVING       (0x7 <<  6)
#define MSDC1_DAT2_DRVING       (0x7 <<  9)
#define MSDC1_DAT1_DRVING       (0x7 << 12)
#define MSDC1_DAT0_DRVING       (0x7 << 15)
#define MSDC1_DRVING_ALL_MASK   (MSDC1_CLK_DRVING   | \
				MSDC1_CMD_DRVING   | \
				MSDC1_DAT3_DRVING  | \
				MSDC1_DAT2_DRVING  | \
				MSDC1_DAT1_DRVING  | \
				MSDC1_DAT0_DRVING)

/**************************************************************/
/* Section 5: Adjustable Driver Parameter                     */
/**************************************************************/
#define HOST_MAX_BLKSZ          (2048)

#define MSDC_OCR_AVAIL\
	(MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 \
	| MMC_VDD_31_32 | MMC_VDD_32_33)
/* data timeout counter. 1048576 * 3 sclk. */
#define DEFAULT_DTOC            (3)

#define MAX_DMA_CNT             (4 * 1024 * 1024)
/* a WIFI transaction may be 50K */
#define MAX_DMA_CNT_SDIO        (0xFFFFFFFF - 255)
/* a LTE  transaction may be 128K */

#define MAX_HW_SGMTS            (MAX_BD_NUM)
#define MAX_PHY_SGMTS           (MAX_BD_NUM)
#define MAX_SGMT_SZ             (MAX_DMA_CNT)
#define MAX_SGMT_SZ_SDIO        (MAX_DMA_CNT_SDIO)

#define HOST_MAX_NUM            (2)
#ifdef CONFIG_PWR_LOSS_MTK_TEST
#define MAX_REQ_SZ              (512 * 65536)
#else
#define MAX_REQ_SZ              (512 * 1024)
#endif

#ifdef FPGA_PLATFORM
#define HOST_MAX_MCLK           (200000000)
#else
#define HOST_MAX_MCLK           (200000000)
#endif
#define HOST_MIN_MCLK           (260000)

/* SD card, bad card handling settings */

/* if continuous data timeout reach the limit */
/* driver will force remove card */
#define MSDC_MAX_DATA_TIMEOUT_CONTINUOUS (100)

/* if continuous power cycle fail reach the limit */
/* driver will force remove card */
#define MSDC_MAX_POWER_CYCLE_FAIL_CONTINUOUS (3)

#define SDCARD_ESD_RECOVERY
/* #define MSDC_HQA */

/**************************************************************/
/* Section 6: BBChip-depenent Tunnig Parameter                */
/**************************************************************/
#define EMMC_MAX_FREQ_DIV               4 /* lower frequence to 12.5M */
#define MSDC_CLKTXDLY                   2

#define VOL_CHG_CNT_DEFAULT_VAL         0x1F4 /* =500 */

/* hw diff: 0xB0[0]=1 */
#define MSDC_PB0_DEFAULT_VAL            0x403C0006
#define MSDC_PB1_DEFAULT_VAL            0xFFFE00C9


#define MSDC_PB2_DEFAULT_RESPWAITCNT    0x3
#define MSDC_PB2_DEFAULT_RESPSTENSEL    0x1
#define MSDC_PB2_DEFAULT_CRCSTSENSEL    0x1

#define MSDC_HW_NO_BUSY_CHECK
#endif /* _MSDC_CUST_MT6580_H_ */

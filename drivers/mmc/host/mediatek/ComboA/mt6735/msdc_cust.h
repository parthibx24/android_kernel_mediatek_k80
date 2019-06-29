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

#ifndef _MSDC_CUST_MT6535_H_
#define _MSDC_CUST_MT6535_H_

#ifdef CONFIG_FPGA_EARLY_PORTING
#define FPGA_PLATFORM
#else
/*#define MTK_MSDC_BRINGUP_DEBUG*/
#endif

#include <dt-bindings/mmc/mt6735-msdc.h>

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


/**************************************************************/
/* Section 2: Power                                           */
/**************************************************************/
#define SD_POWER_DEFAULT_ON     (0)

#if !defined(FPGA_PLATFORM)
#include <mt-plat/upmu_common.h>

//#define REG_VEMC_VOSEL          MT6328_VEMC_3V3_CON1
#define REG_VEMC_VOSEL 			MT6328_PMIC_RG_VEMC_3V3_VOSEL_ADDR
#define REG_VEMC_EN             MT6328_PMIC_RG_VEMC_3V3_EN_ADDR

#define REG_VMC_VOSEL           MT6328_VMC_CON1
#define REG_VMC_EN              MT6328_VMC_CON0

#define REG_VMCH_VOSEL          MT6328_VMCH_CON1
#define REG_VMCH_EN             MT6328_VMCH_CON0


#define MASK_VEMC_VOSEL         MT6328_PMIC_RG_VEMC_3V3_VOSEL_MASK
#define SHIFT_VEMC_VOSEL        MT6328_PMIC_RG_VEMC_3V3_VOSEL_SHIFT
#define FIELD_VEMC_VOSEL        (MASK_VEMC_VOSEL << SHIFT_VEMC_VOSEL)

#define MASK_VEMC_EN            MT6328_PMIC_RG_VEMC_3V3_EN_MASK
#define SHIFT_VEMC_EN           MT6328_PMIC_RG_VEMC_3V3_EN_SHIFT
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


#define MSDCPLL_FREQ            800000000
#define MSDC_CLKSRC_DEFAULT2      2

#define MSDC0_CLKSRC_0        	 26000000
#define MSDC0_CLKSRC_1          800000000
#define MSDC0_CLKSRC_2        	400000000
#define MSDC0_CLKSRC_3        	200000000
#define MSDC0_CLKSRC_4       	182000000
#define MSDC0_CLKSRC_5          136000000
#define MSDC0_CLKSRC_6       	156000000
#define MSDC0_CLKSRC_7        	416000000
#define MSDC0_CLKSRC_8       	 48000000
#define MSDC0_CLKSRC_9       	 91000000
#define MSDC0_CLKSRC_10     	624000000

#define MSDC1_CLKSRC_0        	 26000000
#define MSDC1_CLKSRC_1       	208000000
#define MSDC1_CLKSRC_2       	200000000
#define MSDC1_CLKSRC_3       	182000000
#define MSDC1_CLKSRC_4       	136000000
#define MSDC1_CLKSRC_5       	156000000
#define MSDC1_CLKSRC_6        	 48000000
#define MSDC1_CLKSRC_7        	 91000000


#define MSDC_SRC_FPGA           12000000

/**************************************************************/
/* Section 4: GPIO and Pad                                    */
/**************************************************************/
/*--------------------------------------------------------------------------*/
/* MSDC0~1 GPIO and IO Pad Configuration Base                               */
/*--------------------------------------------------------------------------*/
#define MSDC_GPIO_BASE          gpio_base               /* 0x10211000 */
#define MSDC0_IO_PAD_BASE       MSDC_GPIO_BASE  		/* 0x10211000 */
#define MSDC1_IO_PAD_BASE       MSDC_GPIO_BASE  		/* 0x10211000 */

/*--------------------------------------------------------------------------*/
/* MSDC GPIO Related Register                                               */
/*--------------------------------------------------------------------------*/

/*
 * MSDC0 IO Pad Pinmux register definition.
 */

#define MSDC0_GPIO_MODE18_MWR_ADDR   (MSDC_GPIO_BASE + 0x410)
#define MSDC0_GPIO_MODE19_MWR_ADDR   (MSDC_GPIO_BASE + 0x420)


/* MSDC0 */
#define MSDC0_CLK_PINMUX_ADDR   MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_CLK_PINMUX_BITS   (0x7 << 12)

#define MSDC0_CMD_PINMUX_ADDR   MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_CMD_PINMUX_BITS   (0x7 << 6)

#define MSDC0_DAT0_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_DAT0_PINMUX_BITS  (0x7 << 16)

#define MSDC0_DAT1_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_DAT1_PINMUX_BITS  (0x7 << 19)


#define MSDC0_DAT2_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_DAT2_PINMUX_BITS  (0x7 << 22)


#define MSDC0_DAT3_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_DAT3_PINMUX_BITS  (0x7 << 25)


#define MSDC0_DAT4_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC0_DAT4_PINMUX_BITS  (0x7 << 28)

#define MSDC0_DAT5_PINMUX_ADDR  MSDC0_GPIO_MODE19_MWR_ADDR
#define MSDC0_DAT5_PINMUX_BITS  (0x7 << 0)

#define MSDC0_DAT6_PINMUX_ADDR  MSDC0_GPIO_MODE19_MWR_ADDR
#define MSDC0_DAT6_PINMUX_BITS  (0x7 << 3)

#define MSDC0_DAT7_PINMUX_ADDR  MSDC0_GPIO_MODE19_MWR_ADDR
#define MSDC0_DAT7_PINMUX_BITS  (0x7 << 6)

#define MSDC0_RST_PINMUX_ADDR   MSDC0_GPIO_MODE19_MWR_ADDR
#define MSDC0_RST_PINMUX_BITS   (0x7 << 9)


/*
 * MSDC1 IO Pad Pinmux register definition.
 */
#define MSDC0_GPIO_MODE17_MWR_ADDR   (MSDC_GPIO_BASE + 0x400)

/* MSDC1 */
#define MSDC1_CLK_PINMUX_ADDR   MSDC0_GPIO_MODE17_MWR_ADDR
#define MSDC1_CLK_PINMUX_BITS   (0x7 << 22)

#define MSDC1_CMD_PINMUX_ADDR   MSDC0_GPIO_MODE17_MWR_ADDR
#define MSDC1_CMD_PINMUX_BITS   (0x7 << 19)


#define MSDC1_DAT0_PINMUX_ADDR  MSDC0_GPIO_MODE17_MWR_ADDR
#define MSDC1_DAT0_PINMUX_BITS  (0x7 << 25)

#define MSDC1_DAT1_PINMUX_ADDR  MSDC0_GPIO_MODE17_MWR_ADDR
#define MSDC1_DAT1_PINMUX_BITS  (0x7 << 28)


#define MSDC1_DAT2_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC1_DAT2_PINMUX_BITS  (0x7 << 0)

#define MSDC1_DAT3_PINMUX_ADDR  MSDC0_GPIO_MODE18_MWR_ADDR
#define MSDC1_DAT3_PINMUX_BITS  (0x7 << 3)

/*
 * MSDC0 IO Pad Configuration register definition.
 */


#define MSDC0_IES_CFG_OFFSET    (0xD00)
#define MSDC0_IES_CFG_BASE      (MSDC0_IO_PAD_BASE + 0xD00)
#define MSDC0_IES_ALL_MASK      (0x1F << 0)



#define MSDC0_SR_CFG_OFFSET     (0xD70)
#define MSDC0_SR_CFG_BASE       (MSDC0_IO_PAD_BASE + 0xD70)
#define MSDC0_SR_ALL_MASK       (0xFFFFFFF << 0)


#define MSDC0_SMT_CFG_OFFSET    (0xD10)
#define MSDC0_SMT_CFG_BASE      (MSDC0_IO_PAD_BASE + 0xD10)
#define MSDC0_SMT_ALL_MASK		(0x1f << 0)



#define MSDC0_TDSEL_CFG_OFFSET  (0xD20)
#define MSDC0_TDSEL_BASE        (MSDC0_IO_PAD_BASE + 0xD20)
#define MSDC0_TDSEL_ALL_MASK    (0xFFFFF << 0)



#define MSDC0_RDSEL_CFG_OFFSET  (0xD28)
#define MSDC0_RDSEL_BASE        (MSDC0_IO_PAD_BASE + 0xD28)
#define MSDC0_RDSEL_ALL_MASK    (0x3FFFFFFF << 0)

#define MSDC0_GPIO_PUPD0_OFFSET			(0xD80)
#define MSDC0_GPIO_PUPD0_BASE			(MSDC0_IO_PAD_BASE + 0xD80)
#define MSDC0_PUPD_CMD_DSL_CLK_DAT04_MASK 	(0x77777777 << 0)
#define MSDC0_GPIO_PUPD1_OFFSET			(0xD90)
#define MSDC0_GPIO_PUPD1_BASE			(MSDC0_IO_PAD_BASE + 0xD90)
#define MSDC0_PUPD_DAT567_MASK				(0x777 << 0)


#define MSDC0_DRVING_OFFSET 			(0xD70)
#define MSDC0_DRVING_BASE 			 	(MSDC0_IO_PAD_BASE + 0xD70)
#define MSDC0_DRV_CMD_MASK				(0x7  << 0)
#define MSDC0_DRV_DSL_MASK				(0x7  << 4)
#define MSDC0_DRV_CLK_MASK				(0x7  << 8)
#define MSDC0_DRV_DAT_MASK				(0x7  << 12)
#define MSDC0_DRV_RSTB_MASK				(0x7  << 16)
#define MSDC0_DRV_ALL_MASK				(0x77777 << 0)

/*
 * MSDC1 IO Pad Configuration register definition.
 */

#define MSDC1_IES_CFG_OFFSET    (0xC00)
#define MSDC1_IES_CFG_BASE      (MSDC1_IO_PAD_BASE + 0xC00)
#define MSDC1_IES_ALL_MASK      (0x1F << 0)

#define MSDC1_SR_CFG_OFFSET     (0xC70)
#define MSDC1_SR_CFG_BASE       (MSDC1_IO_PAD_BASE + 0xC70)
#define MSDC1_SR_ALL_MASK       (0xFFFFF << 0)

#define MSDC1_SMT_CFG_OFFSET    (0xC10)
#define MSDC1_SMT_CFG_BASE      (MSDC1_IO_PAD_BASE + 0xC10)
#define MSDC1_SMT_ALL_MASK      (0x7 << 0)

#define MSDC1_TDSEL_CFG_OFFSET  (0xC20)
#define MSDC1_TDSEL_BASE        (MSDC1_IO_PAD_BASE + 0xC20)
#define MSDC1_TDSEL_ALL_MASK    (0xFFF << 8)

#define MSDC1_RDSEL_CFG_OFFSET  (0xC28)
#define MSDC1_RDSEL_BASE        (MSDC1_IO_PAD_BASE + 0xC28)
#define MSDC1_RDSEL_ALL_MASK    (0x7FFF << 12)

#define MSDC1_GPIO_PUPD0_BASE			(MSDC1_IO_PAD_BASE + 0xC80)
#define MSDC1_PUPD_CMD_CLK_DAT_MASK		(0x777777 << 0)

#define MSDC1_DRVING_OFFSET     		(0xC70)
#define MSDC1_DRVING_BASE       		(MSDC1_IO_PAD_BASE + 0xC70)
#define MSDC1_DRV_CMD_MASK				(0x7  << 8)
#define MSDC1_DRV_CLK_MASK				(0x7  << 12)
#define MSDC1_DRV_DAT_MASK				(0x7  << 16)
#define MSDC1_DRV_ALL_MASK				(0x777 << 8)

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

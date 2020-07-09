/* Copyright (C) 2015 MediaTek Inc.
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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include "mtk_sd.h"
#include "dbg.h"
struct msdc_host *mtk_msdc_host[] = {NULL, NULL};
EXPORT_SYMBOL(mtk_msdc_host);

int g_dma_debug[HOST_MAX_NUM] = {0, 0};
u32 latest_int_status[HOST_MAX_NUM] = {0, 0};

unsigned int msdc_latest_transfer_mode[HOST_MAX_NUM] = {
	/* 0 for PIO; 1 for DMA; 3 for nothing */
	MODE_NONE,
	MODE_NONE
};

unsigned int msdc_latest_op[HOST_MAX_NUM] = {
	/* 0 for read; 1 for write; 2 for nothing */
	OPER_TYPE_NUM,
	OPER_TYPE_NUM
};

/* for debug zone */
unsigned int sd_debug_zone[HOST_MAX_NUM] = {
	0,
	0
};
/* for enable/disable register dump */
unsigned int sd_register_zone[HOST_MAX_NUM] = {
	1,
	1
};
/* mode select */
u32 dma_size[HOST_MAX_NUM] = {
	512,
	512
};

u32 drv_mode[HOST_MAX_NUM] = {
	MODE_SIZE_DEP, /* using DMA or not depend on the size */
	//MODE_PIO,
	MODE_SIZE_DEP
};

int dma_force[HOST_MAX_NUM]; /* used for sd ioctrol */

u8 msdc_clock_src[HOST_MAX_NUM] = {
	0,
	0
};

/**************************************************************/
/* Section 1: Device Tree Global Variables                    */
/**************************************************************/
const struct of_device_id msdc_of_ids[] = {
	{   .compatible = DT_COMPATIBLE_NAME, },
	{ },
};

#if !defined(FPGA_PLATFORM)
static void __iomem *gpio_base;

static void __iomem *infracfg_ao_base;
static void __iomem *topckgen_base;
#endif

void __iomem *msdc_io_cfg_bases[HOST_MAX_NUM];

/**************************************************************/
/* Section 2: Power                                           */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
/* used for VMC CAL */
extern void upmu_set_rg_vmc_184(unsigned char x);

int msdc_regulator_set_and_enable(struct regulator *reg, int powerVolt)
{
#ifndef CONFIG_MTK_MSDC_BRING_UP_BYPASS
	regulator_set_voltage(reg, powerVolt, powerVolt);
	return regulator_enable(reg);
#else
	return 0;
#endif
}

void msdc_ldo_power(u32 on, struct regulator *reg, int voltage_mv, u32 *status)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	int voltage_uv = voltage_mv * 1000;

	if (reg == NULL)
		return;

	if (on) { /* want to power on */
		if (*status == 0) {  /* can power on */
			/*Comment out to reduce log */
			/* pr_notice("msdc power on<%d>\n", voltage_uv); */
			(void)msdc_regulator_set_and_enable(reg, voltage_uv);
			*status = voltage_uv;
		} else if (*status == voltage_uv) {
			pr_info("msdc power on <%d> again!\n", voltage_uv);
		} else {
			pr_info("msdc change<%d> to <%d>\n",
				*status, voltage_uv);
			regulator_disable(reg);
			(void)msdc_regulator_set_and_enable(reg, voltage_uv);
			*status = voltage_uv;
		}
	} else {  /* want to power off */
		if (*status != 0) {  /* has been powerred on */
			pr_info("msdc power off\n");
			(void)regulator_disable(reg);
			*status = 0;
		} else {
			pr_info("msdc not power on\n");
		}
	}
#endif
}

void msdc_dump_ldo_sts(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS) \
	|| defined(MTK_MSDC_BRINGUP_DEBUG)
	u32 ldo_en = 0, ldo_vol = 0;
	u32 id = host->id;
	switch (id) {
	case 0:
		pmic_read_interface_nolock(REG_VEMC_EN, &ldo_en, MASK_VEMC_EN,
			SHIFT_VEMC_EN);
		pmic_read_interface_nolock(REG_VEMC_VOSEL, &ldo_vol,
			MASK_VEMC_VOSEL, SHIFT_VEMC_VOSEL);
		SPREAD_PRINTF(buff, size, m,
		" VEMC_EN[0x0A24]=0x%x, should:bit1=1\n,VEMC_VOL[0x0A64]=0x%x,should:[bit[5:4]=2b'10]\n",
			ldo_en, ldo_vol);
		break;
	case 1:
		pmic_read_interface_nolock(REG_VMC_EN, &ldo_en, MASK_VMC_EN,
			SHIFT_VMC_EN);
		pmic_read_interface_nolock(REG_VMC_VOSEL, &ldo_vol,
			MASK_VMC_VOSEL, SHIFT_VMC_VOSEL);
		SPREAD_PRINTF(buff, size, m,
		" VMC_EN=0x%x, VMC_VOL=0x%x [1b'1(3.3V), 1b'0(1.8V)]\n",
			ldo_en, ldo_vol);

		pmic_read_interface_nolock(REG_VMCH_EN, &ldo_en, MASK_VMCH_EN,
			SHIFT_VMCH_EN);
		pmic_read_interface_nolock(REG_VMCH_VOSEL, &ldo_vol,
			MASK_VMCH_VOSEL, SHIFT_VMCH_VOSEL);;
		SPREAD_PRINTF(buff, size, m,
		" VMCH_EN=0x%x, VMCH_VOL=0x%x [1b'1(3.3V), 1b'0(3.0V)]\n",
			ldo_en, ldo_vol);
		break;
	default:
		break;
	}
#endif
}

void msdc_sd_power_switch(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	if (host->id == 1) {
		if (on) {
			upmu_set_rg_vmc_184(1); /* workaround */
		}

		msdc_ldo_power(on, host->mmc->supply.vqmmc, VOL_1800,
			&host->power_io);
		msdc_set_tdsel(host, MSDC_TDRDSEL_1V8, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_1V8, 0);
		host->hw->driving_applied = &host->hw->driving_sdr50;
		msdc_set_driving(host, host->hw->driving_applied);
	}
#endif

}

void msdc_power_calibration_init(struct msdc_host *host)
{

}

int msdc_oc_check(struct msdc_host *host, u32 en)
{
	/* MT6350 no OC */
	return 0;
}

void msdc_emmc_power(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)

	if (on) {
		msdc_set_driving(host, &host->hw->driving);
		msdc_set_tdsel(host, MSDC_TDRDSEL_1V8, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_1V8, 0);
	}
	msdc_ldo_power(on, host->mmc->supply.vmmc,
		VOL_3000, &host->power_flash);
#endif
#ifdef MTK_MSDC_BRINGUP_DEBUG
	msdc_dump_ldo_sts(NULL, 0, NULL, host);
#endif
}

void msdc_sd_power(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	u32 card_on = on;

	switch (host->id) {
	case 1:
		msdc_set_driving(host, &host->hw->driving);
		msdc_set_tdsel(host, MSDC_TDRDSEL_3V, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_3V, 0);
		if (host->hw->flags & MSDC_SD_NEED_POWER)
			card_on = 1;

		/* VMCH VOLSEL */
		msdc_ldo_power(card_on, host->mmc->supply.vmmc, VOL_3300,
			&host->power_flash);

		msdc_ldo_power(on, host->mmc->supply.vqmmc, VOL_3300,
			&host->power_io);

		if (on)
			//msdc_set_sr(host, 0, 0, 0, 0, 0);
		   upmu_set_rg_vmc_184(0); /* workarond */
		break;

	default:
		break;
	}
#endif
#ifdef MTK_MSDC_BRINGUP_DEBUG
	msdc_dump_ldo_sts(NULL, 0, NULL, host);
#endif
}

void msdc_sd_power_off(void)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	struct msdc_host *host = mtk_msdc_host[1];

	if (host) {
		pr_notice("Power Off, SD card\n");

		/* power must be on */
		host->power_io = VOL_3300 * 1000;
		host->power_flash = VOL_3300 * 1000;

		host->power_control(host, 0);

		msdc_set_bad_card_and_remove(host);
	}
#endif
}
EXPORT_SYMBOL(msdc_sd_power_off);
#endif /*if !defined(FPGA_PLATFORM)*/

void msdc_pmic_force_vcore_pwm(bool enable)
{

}

void msdc_set_host_power_control(struct msdc_host *host)
{
	if (host->hw->host_function == MSDC_EMMC) {
		host->power_control = msdc_emmc_power;
	} else if (host->hw->host_function == MSDC_SD) {
		host->power_control = msdc_sd_power;
		host->power_switch = msdc_sd_power_switch;

		#if SD_POWER_DEFAULT_ON
		/* If SD card power is default on, turn it off so that
		 * removable card slot won't keep power when no card plugged
		 */
		if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
			/* turn on first to match HW/SW state*/
			msdc_sd_power(host, 1);
			mdelay(10);
			msdc_sd_power(host, 0);
		}
		#endif
	}

	if (host->power_control != NULL) {
		msdc_power_calibration_init(host);
	} else {
		ERR_MSG("Host function defination error for msdc%d", host->id);
		WARN_ON(1);
	}
}

#if defined(MSDC_HQA)
void msdc_HQA_set_voltage(struct msdc_host *host)
{
}
#endif

/**************************************************************/
/* Section 3: Clock                                           */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
u32 hclks_msdc0[] = {
	MSDC0_CLKSRC_0,
	MSDC0_CLKSRC_1,
	MSDC0_CLKSRC_2,
	MSDC0_CLKSRC_3,
	MSDC0_CLKSRC_4,
	MSDC0_CLKSRC_5,
	MSDC0_CLKSRC_6,
	MSDC0_CLKSRC_7,
	MSDC0_CLKSRC_8,
	MSDC0_CLKSRC_9,
	MSDC0_CLKSRC_10
};

/* msdc1/2 clock source reference value is 200M */
u32 hclks_msdc1[] = {
	MSDC1_CLKSRC_0,
	MSDC1_CLKSRC_1,
	MSDC1_CLKSRC_2,
	MSDC1_CLKSRC_3,
	MSDC1_CLKSRC_4,
	MSDC1_CLKSRC_5,
	MSDC1_CLKSRC_6,
	MSDC1_CLKSRC_7
};

u32 *hclks_msdc_all[] = {
	hclks_msdc0,
	hclks_msdc1
};
u32 *hclks_msdc;

int msdc_get_ccf_clk_pointer(struct platform_device *pdev,
	struct msdc_host *host)
{
	char name[20];

	/* No CCF */
	host->clk_ctl = NULL;
	host->hclk_ctl = NULL;
	/* Just set default clksrc */
	msdc_clk_enable(host);
	sprintf(name, "MSDC%d", host->id);

    if((host->id == 0) || (host->id == 1))
		clkmux_sel(MT_MUX_MSDC30_0 - host->id, MSDC_CLKSRC_DEFAULT2, name);
	else
		pr_err("%s host id is out of range, id=%d\n",__func__, host->id);

	msdc_clk_disable(host);

	return 0;
}

void msdc_select_clksrc(struct msdc_host *host, int clksrc)
{
	host->hclk = msdc_get_hclk(host->id, clksrc);
	host->hw->clk_src = clksrc;

	pr_notice("[%s]: msdc%d select clk_src as %d(%dKHz)\n", __func__,
		host->id, clksrc, host->hclk/1000);

	pr_notice("[%s]: msdc%d not support change clksrc\n", __func__,
		host->id);
}

#include <linux/seq_file.h>

static void msdc_dump_clock_sts_core(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{

}

void msdc_dump_clock_sts(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
	msdc_dump_clock_sts_core(buff, size, m, host);
}

void msdc_clk_enable_and_stable(struct msdc_host *host)
{
	void __iomem *base = host->base;
	u32 div, mode;
	u32 val;

	msdc_clk_enable(host);

	val = MSDC_READ32(MSDC_CFG);
	GET_FIELD(val, CFG_CKDIV_SHIFT, CFG_CKDIV_MASK, div);
	GET_FIELD(val, CFG_CKMOD_SHIFT, CFG_CKMOD_MASK, mode);
	msdc_clk_stable(host, mode, div, 0);
}

extern spinlock_t msdc_cg_lock;
extern unsigned int msdc_cg_cnt;
/* do we need sync object or not */
/* MT_CG_PERI_MSDC30_0="13+32" defined in mt_clkmgr2.h */
void msdc_clk_status(int *status)
{
	int g_clk_gate = 0;
	int i = 0;
	unsigned long flags;

	for (i = 0; i < HOST_MAX_NUM; i++) {
		if (!mtk_msdc_host[i])
			continue;

		spin_lock_irqsave(&msdc_cg_lock, flags);
		if (msdc_cg_cnt > 0)

			g_clk_gate |= 1 << ((i) + (MT_CG_PERI_MSDC30_0-32));

		spin_unlock_irqrestore(&msdc_cg_lock, flags);
	}
	*status = g_clk_gate;
}

#endif /*if !defined(FPGA_PLATFORM)*/

/**************************************************************/
/* Section 4: GPIO and Pad                                    */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
void msdc_dump_vcore(char **buff, unsigned long *size, struct seq_file *m)
{
}

/*****************************************************************************/
/* obtain dump api interface */
/*****************************************************************************/
void msdc_dump_dvfs_reg(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
}

/*
 * Pull DAT0~2 high/low one-by-one
 * and power off card when DAT pin status is not the same pull level
 * 1. PULL DAT0 Low, DAT1/2/3 high
 * 2. PULL DAT1 Low, DAT0/2/3 high
 * 3. PULL DAT2 Low, DAT0/1/3 high
 */

int msdc_io_check(struct msdc_host *host)
{
	int result = 1, i;
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0;
	u32 pupd_patterns[3] = {0x222662, 0x226262, 0x262262};
	u32 check_patterns[3] = {0xe0000, 0xd0000, 0xb0000};

	if (host->id != 1)
		return 1;
	
	for (i = 0; i < 3; i++) {
		MSDC_SET_FIELD(MSDC1_GPIO_PUPD0_BASE, MSDC1_PUPD_CMD_CLK_DAT_MASK,
			pupd_patterns[i]);
		polling_tmo = jiffies + POLLING_PINS;
		while ((MSDC_READ32(MSDC_PS) & 0xF0000) != check_patterns[i]) {
			if (time_after(jiffies, polling_tmo)) {
				pr_err("msdc%d DAT%d pin get wrong, ps = 0x%x!\n",
					host->id, i, MSDC_READ32(MSDC_PS));
				goto POWER_OFF;
			}
		}
	}

	MSDC_SET_FIELD(MSDC1_GPIO_PUPD0_BASE, MSDC1_PUPD_CMD_CLK_DAT_MASK, 0x222262);
	return result;

POWER_OFF:
	host->block_bad_card = 1;
	host->power_control(host, 0);
	return 0;
}

#define MSDC_IOC_ITEMS     8
static unsigned int msdc_ioconfg_reg_offsets[HOST_MAX_NUM][MSDC_IOC_ITEMS] = {
	{
		MSDC0_IES_CFG_OFFSET,
		MSDC0_SR_CFG_OFFSET,
		MSDC0_SMT_CFG_OFFSET,
		MSDC0_TDSEL_CFG_OFFSET,
		MSDC0_RDSEL_CFG_OFFSET,
		MSDC0_DRVING_OFFSET,
		MSDC0_GPIO_PUPD0_OFFSET,
		MSDC0_GPIO_PUPD1_OFFSET
	},
	{
		MSDC1_IES_CFG_OFFSET,
		MSDC1_SR_CFG_OFFSET,
		MSDC1_SMT_CFG_OFFSET,
		MSDC1_TDSEL_CFG_OFFSET,
		MSDC1_RDSEL_CFG_OFFSET,
		MSDC1_PUPD_CMD_CLK_DAT_MASK,
		MSDC1_DRVING_OFFSET
	}
};

static unsigned char msdc_ioconfig_names[MSDC_IOC_ITEMS][16] = {
	"IES",
	"SR",
	"SMT",
	"TDSEL",
	"RDSEL",
	"DRVING",
	"PUPD0",
	"PUPD1"
};

void msdc_dump_padctl_by_id(char **buff, unsigned long *size,
	struct seq_file *m, u32 id)
{
	unsigned int i;
	unsigned int val = 0;
	void __iomem *base = MSDC0_IO_PAD_BASE;

	if (!gpio_base) {
		SPREAD_PRINTF(buff, size, m,
			"err: gpio_base=%p\n",
			gpio_base);
		return;
	}

	if (id == 0) {
		SPREAD_PRINTF(buff, size, m, "MSDC0 shall set pin mode to 1\n");
		SPREAD_PRINTF(buff, size, m, "GPIO_MODE18=%x\n",
			MSDC_READ32(MSDC0_GPIO_MODE18_MWR_ADDR));
		SPREAD_PRINTF(buff, size, m, "GPIO_MODE19=%x\n",
			MSDC_READ32(MSDC0_GPIO_MODE19_MWR_ADDR));

		base = MSDC0_IO_PAD_BASE;
	} else if (id == 1) {

		SPREAD_PRINTF(buff, size, m, "MSDC1 shall set pin mode to 1\n");
		SPREAD_PRINTF(buff, size, m, "GPIO_MODE17=%x\n",
			MSDC_READ32(MSDC0_GPIO_MODE17_MWR_ADDR));

		base = MSDC1_IO_PAD_BASE;
	}


	for (i = 0; i < MSDC_IOC_ITEMS; i++) {
		MSDC_GET_FIELD(base + msdc_ioconfg_reg_offsets[id][i],
		0xffffffff, val);
		SPREAD_PRINTF(buff, size, m, "%s=%x\n",
			msdc_ioconfig_names[i], val);
	}
}

void msdc_set_pin_mode(struct msdc_host *host)
{
	if (host->id == 0) {

		/*
		 * set msdc mode. (MC0_RST, MC0_DAT5~7)
		 */
		MSDC_SET_FIELD(MSDC0_GPIO_MODE18_MWR_ADDR,
			0xffffffff,
			0x12491249);

		MSDC_SET_FIELD(MSDC0_GPIO_MODE19_MWR_ADDR,
			MSDC0_RST_PINMUX_BITS |	MSDC0_DAT7_PINMUX_BITS |
			MSDC0_DAT6_PINMUX_BITS | MSDC0_DAT5_PINMUX_BITS,
			0x249);

	} else if (host->id == 1) {

		/*
		 * set gpio to msdc mode. (clk/cmd/dat1/dat0 in GPIO_MODE17)
		 */
/*fix me : set to what?*/		 
#if 0		 
		MSDC_SET_FIELD(MSDC0_GPIO_MODE17_MWR_ADDR,
			MSDC1_CLK_PINMUX_BITS |	MSDC1_CMD_PINMUX_BITS |
			MSDC1_DAT0_PINMUX_BITS | MSDC1_DAT1_PINMUX_BITS,
			0x12480000);
		/*
		 * set gpio to msdc mode. (dat2/dat3 in GPIO_MODE18)
		 */
		MSDC_SET_FIELD(MSDC0_GPIO_MODE18_MWR_ADDR,
			MSDC1_DAT2_PINMUX_BITS | MSDC1_DAT3_PINMUX_BITS, 0x9);
#endif
	}

}

void msdc_set_ies_by_id(u32 id, int set_ies)
{

}

void msdc_set_smt_by_id(u32 id, int set_smt)
{
	if (id == 0) {
		if (set_smt)
			MSDC_SET_FIELD(MSDC0_SMT_CFG_BASE, MSDC0_SMT_ALL_MASK, 0x1F);
		else
			MSDC_SET_FIELD(MSDC0_SMT_CFG_BASE, MSDC0_SMT_ALL_MASK, 0);

	} else if (id == 1) {
		if (set_smt)
			MSDC_SET_FIELD(MSDC1_SMT_CFG_BASE, MSDC0_SMT_ALL_MASK, 0x7);
		else
			MSDC_SET_FIELD(MSDC1_SMT_CFG_BASE, MSDC0_SMT_ALL_MASK, 0);
	}
}

void msdc_set_tdsel_by_id(u32 id, u32 flag, u32 value)
{
	/*fix ME : this func is different form kernel3.18 */

}

void msdc_set_rdsel_by_id(u32 id, u32 flag, u32 value)
{

}

void msdc_get_tdsel_by_id(u32 id, u32 *value)
{
	if (id == 0)
		MSDC_GET_FIELD(MSDC0_TDSEL_BASE, MSDC0_TDSEL_ALL_MASK, *value);
	else if (id == 1)
		MSDC_GET_FIELD(MSDC1_TDSEL_BASE, MSDC1_TDSEL_ALL_MASK, *value);
}

void msdc_get_rdsel_by_id(u32 id, u32 *value)
{
	if (id == 0)
		MSDC_GET_FIELD(MSDC0_RDSEL_BASE, MSDC0_RDSEL_ALL_MASK, *value);
	else if (id == 1)
		MSDC_GET_FIELD(MSDC1_RDSEL_BASE, MSDC1_RDSEL_ALL_MASK, *value);
}

void msdc_set_sr_by_id(u32 id, int clk, int cmd, int dat, int rst, int ds)
{
/* notice here if need set sr*/
#if 0
	switch (id) {
	case 0:
		MSDC_SET_FIELD(MSDC0_SR_CFG_BASE, MSDC0_SR_CMD_MASK, (cmd != 0));
		MSDC_SET_FIELD(MSDC0_GPIO_DRV0_ADDR, MSDC0_SR_DSL_MASK, (ds != 0));
		MSDC_SET_FIELD(MSDC0_GPIO_DRV0_ADDR, MSDC0_SR_CLK_MASK, (clk != 0));
		MSDC_SET_FIELD(MSDC0_GPIO_DRV0_ADDR, MSDC0_SR_DAT_MASK, (dat != 0));
		MSDC_SET_FIELD(MSDC0_GPIO_DRV0_ADDR, MSDC0_SR_RSTB_MASK, (rst != 0));
		break;
	case 1:
		MSDC_SET_FIELD(MSDC1_DRVING_BASE, MSDC1_SR_CMD_MASK, (cmd != 0));
		MSDC_SET_FIELD(MSDC1_GPIO_DRV0_ADDR, MSDC1_SR_CLK_MASK, (clk != 0));
		MSDC_SET_FIELD(MSDC1_GPIO_DRV0_ADDR, MSDC1_SR_DAT_MASK, (dat != 0));
		break;
	default:
		pr_err("error...[%s] host->id out of range!!!\n", __func__);
		break;
	}
#endif
}

void msdc_set_driving_by_id(u32 id, struct msdc_hw_driving *driving)
{
#ifndef CONFIG_MTK_MSDC_BRING_UP_BYPASS
	pr_notice("msdc%d set driving: clk_drv=%d, cmd_drv=%d, dat_drv=%d, rst_drv=%d, ds_drv=%d\n",
		id,
		driving->clk_drv,
		driving->cmd_drv,
		driving->dat_drv,
		driving->rst_drv,
		driving->ds_drv);
#endif

	if (id == 0) {
		MSDC_SET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_DSL_MASK,
			driving->ds_drv);
		MSDC_SET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_RSTB_MASK,
			driving->rst_drv);
		MSDC_SET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_CMD_MASK,
			driving->cmd_drv);
		MSDC_SET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_CLK_MASK,
			driving->clk_drv);
		MSDC_SET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_DAT_MASK,
			driving->dat_drv);
	} else if (id == 1) {
		MSDC_SET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_CMD_MASK,
			driving->cmd_drv);
		MSDC_SET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_CLK_MASK,
			driving->clk_drv);
		MSDC_SET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_DAT_MASK,
			driving->dat_drv);
	}

}

void msdc_get_driving_by_id(u32 id, struct msdc_hw_driving *driving)
{
	if (id == 0) {
		MSDC_GET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_DAT_MASK,
			driving->dat_drv);
		MSDC_GET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_CLK_MASK,
			driving->clk_drv);
		MSDC_GET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_RSTB_MASK,
			driving->rst_drv);
		MSDC_GET_FIELD(MSDC0_DRVING_BASE, MSDC0_DRV_CMD_MASK,
			driving->cmd_drv);

	} else if (id == 1) {
		MSDC_GET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_CLK_MASK,
			driving->clk_drv);
		MSDC_GET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_CMD_MASK,
			driving->cmd_drv);
		MSDC_GET_FIELD(MSDC1_DRVING_BASE, MSDC1_DRV_DAT_MASK,
			driving->dat_drv);
	}
}

void msdc_pin_config_by_id(u32 id, u32 mode)
{
	if (id == 0) {
		/* 1. don't pull CLK high;
		 * 2. Don't toggle RST to prevent from entering boot mode
		 */

		if (mode == MSDC_PIN_PULL_NONE) {
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_BASE,
				MSDC0_PUPD_CMD_DSL_CLK_DAT04_MASK, 0x44444444);
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_BASE,
				MSDC0_PUPD_DAT567_MASK, 0x444);
		} else if (mode == MSDC_PIN_PULL_DOWN) {
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_BASE,
				MSDC0_PUPD_CMD_DSL_CLK_DAT04_MASK, 0x66666666);
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_BASE,
				MSDC0_PUPD_DAT567_MASK, 0x666);

		} else if (mode == MSDC_PIN_PULL_UP) {
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_BASE,
				MSDC0_PUPD_CMD_DSL_CLK_DAT04_MASK, 0x11111661);
			MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_BASE,
				MSDC0_PUPD_DAT567_MASK, 0x111);
		}

	} else if (id == 1) {
		if (mode == MSDC_PIN_PULL_NONE)
			MSDC_SET_FIELD(MSDC1_GPIO_PUPD0_BASE,
				MSDC1_PUPD_CMD_CLK_DAT_MASK, 0x444444);
		else if (mode == MSDC_PIN_PULL_DOWN)
			MSDC_SET_FIELD(MSDC1_GPIO_PUPD0_BASE,
				MSDC1_PUPD_CMD_CLK_DAT_MASK, 0x666666);
		else if (mode == MSDC_PIN_PULL_UP)
			MSDC_SET_FIELD(MSDC1_GPIO_PUPD0_BASE,
				MSDC1_PUPD_CMD_CLK_DAT_MASK, 0x222262);
	}
}
#endif /*if !defined(FPGA_PLATFORM)*/


/**************************************************************/
/* Section 5: Device Tree Init function                       */
/*            This function is placed here so that all	      */
/*            functions and variables used by it has already  */
/*            been declared                                   */
/**************************************************************/
/*
 * parse pinctl settings
 * Driver strength
 */
#if !defined(FPGA_PLATFORM)
static int msdc_get_pinctl_settings(struct msdc_host *host,
	struct device_node *np)
{
	struct device_node *pinctl_node, *pins_node;
	static char const * const pinctl_names[] = {
		"pinctl", "pinctl_hs400", "pinctl_hs200",
		"pinctl_sdr104", "pinctl_sdr50", "pinctl_ddr50"
	};

	/* sequence shall be the same as sequence in msdc_hw_driving */
	static char const * const pins_names[] = {
		"pins_cmd", "pins_dat", "pins_clk", "pins_rst", "pins_ds"
	};
	unsigned char *pin_drv;
	int i, j;

	host->hw->driving_applied = &host->hw->driving;
	for (i = 0; i < ARRAY_SIZE(pinctl_names); i++) {
		pinctl_node = of_parse_phandle(np, pinctl_names[i], 0);

		if (strcmp(pinctl_names[i], "pinctl") == 0)
			pin_drv = (unsigned char *)&host->hw->driving;
          	else if (strcmp(pinctl_names[i], "pinctl_hs400") == 0)
			pin_drv = (unsigned char *)&host->hw->driving_hs400;
		else if (strcmp(pinctl_names[i], "pinctl_hs200") == 0)
			pin_drv = (unsigned char *)&host->hw->driving_hs200;
		else if (strcmp(pinctl_names[i], "pinctl_sdr104") == 0)
			pin_drv = (unsigned char *)&host->hw->driving_sdr104;
		else if (strcmp(pinctl_names[i], "pinctl_sdr50") == 0)
			pin_drv = (unsigned char *)&host->hw->driving_sdr50;
		else if (strcmp(pinctl_names[i], "pinctl_ddr50") == 0)
			pin_drv = (unsigned char *)&host->hw->driving_ddr50;
		else
			continue;

		for (j = 0; j < ARRAY_SIZE(pins_names); j++) {
			pins_node = of_get_child_by_name(pinctl_node,
				pins_names[j]);

			if (pins_node)
				of_property_read_u8(pins_node,
					"drive-strength", pin_drv);
			pin_drv++;
		}
	}

	return 0;
}
#endif

/* Get msdc register settings
 * 1. internal data delay for tuning, FIXME: can be removed when use data tune?
 * 2. sample edge
 */
static int msdc_get_register_settings(struct msdc_host *host,
	struct device_node *np)
{
	struct device_node *register_setting_node = NULL;

	/* parse hw property settings */
	register_setting_node = of_parse_phandle(np, "register_setting", 0);
	if (register_setting_node) {
		of_property_read_u8(register_setting_node, "cmd_edge",
				&host->hw->cmd_edge);
		of_property_read_u8(register_setting_node, "rdata_edge",
				&host->hw->rdata_edge);
		of_property_read_u8(register_setting_node, "wdata_edge",
				&host->hw->wdata_edge);
	} else {
		pr_notice("[msdc%d] register_setting is not found in DT\n",
			host->id);
	}

	return 0;
}

/*
 *	msdc_of_parse() - parse host's device-tree node
 *	@host: host whose node should be parsed.
 *
 */

int msdc_of_parse(struct platform_device *pdev, struct mmc_host *mmc)
{
	struct device_node *np;
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;
	u8 id;

	np = mmc->parent->of_node; /* mmcx node in project dts */

	if (of_property_read_u8(np, "index", &id)) {
		pr_notice("[%s] host index not specified in device tree\n",
			pdev->dev.of_node->name);
		return -1;
	}
	host->id = id;
	pdev->id = id;

	pr_notice("DT probe %s%d!\n", pdev->dev.of_node->name, id);

	ret = mmc_of_parse(mmc);
	if (ret) {
		pr_notice("%s: mmc of parse error!!: %d\n", __func__, ret);
		return ret;
	}

	host->mmc = mmc;
	host->hw = kzalloc(sizeof(struct msdc_hw), GFP_KERNEL);

	/* iomap register */
	host->base = of_iomap(np, 0);
	if (!host->base) {
		pr_notice("[msdc%d] of_iomap failed\n", mmc->index);
		return -ENOMEM;
	}
	/* get irq # */
	host->irq = irq_of_parse_and_map(np, 0);
	pr_notice("[msdc%d] get irq # %d\n", host->id, host->irq);
	WARN_ON(host->irq < 0);

#if !defined(FPGA_PLATFORM)
	/* get clk_src */
	if (of_property_read_u8(np, "clk_src", &host->hw->clk_src)) {
		pr_notice("[msdc%d] error: clk_src isn't found in device tree.\n",
			host->id);
		WARN_ON(1);
	}

#endif
	/* Returns 0 on success, -EINVAL if the property does not exist,
	 * -ENODATA if property does not have a value, and -EOVERFLOW if the
	 * property data isn't large enough.
	 */
	if (of_property_read_u8(np, "host_function", &host->hw->host_function))
		pr_notice("[msdc%d] host_function isn't found in device tree\n",
			host->id);

	/* get cd_gpio and cd_level */
	cd_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	if (of_property_read_u8(np, "cd_level", &host->hw->cd_level))
		pr_notice("[msdc%d] cd_level isn't found in device tree\n",
			host->id);

	msdc_get_register_settings(host, np);


#if !defined(FPGA_PLATFORM)
	msdc_get_pinctl_settings(host, np);
	mmc->supply.vmmc = regulator_get(mmc_dev(mmc), "vmmc");
	mmc->supply.vqmmc = regulator_get(mmc_dev(mmc), "vqmmc");

#else
	msdc_fpga_pwr_init();
#endif

#if !defined(FPGA_PLATFORM)
	if (host->hw->host_function == MSDC_EMMC) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,msdc0_top");
		host->base_top = of_iomap(np, 0);
	} else if (host->hw->host_function == MSDC_SD) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,msdc1_top");
		host->base_top = of_iomap(np, 0);
	}
	if (host->base_top)
		pr_debug("of_iomap for MSDC%d TOP base @ 0x%p\n",
			host->id, host->base_top);
#endif
	/* device rename */
	if ((host->id == 0) && !device_rename(mmc->parent, "bootdevice"))
		pr_notice("[msdc%d] device renamed to bootdevice.\n", host->id);
	else if ((host->id == 1) && !device_rename(mmc->parent, "externdevice"))
		pr_notice("[msdc%d] device renamed to externdevice.\n",
			host->id);
	else if ((host->id == 0) || (host->id == 1))
		pr_notice("[msdc%d] error: device renamed failed.\n", host->id);

	return host->id;
}

int msdc_dt_init(struct platform_device *pdev, struct mmc_host *mmc)
{
	int id;

#ifndef FPGA_PLATFORM
	struct device_node *np;
#endif

	id = msdc_of_parse(pdev, mmc);
	if (id < 0) {
		pr_notice("%s: msdc_of_parse error!!: %d\n", __func__, id);
		return id;
	}

#ifndef FPGA_PLATFORM
	if (gpio_base == NULL) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,gpio");
		gpio_base = of_iomap(np, 0);
		pr_debug("of_iomap for gpio base @ 0x%p\n", gpio_base);
	}

	if (topckgen_base == NULL) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,CKSYS");
		topckgen_base = of_iomap(np, 0);
		pr_debug("of_iomap for topckgen base @ 0x%p\n",
			topckgen_base);
	}

	if (infracfg_ao_base == NULL) {
		np = of_find_compatible_node(NULL, NULL,
			"mediatek,INFRACFG_AO");
		infracfg_ao_base = of_iomap(np, 0);
		pr_debug("of_iomap for infracfg_ao base @ 0x%p\n",
			infracfg_ao_base);
	}
#endif
	return 0;
}

/**************************************************************/
/* Section 7: For msdc register dump                          */
/**************************************************************/
u16 msdc_offsets[] = {
	OFFSET_MSDC_CFG,
	OFFSET_MSDC_IOCON,
	OFFSET_MSDC_PS,
	OFFSET_MSDC_INT,
	OFFSET_MSDC_INTEN,
	OFFSET_MSDC_FIFOCS,
	OFFSET_SDC_CFG,
	OFFSET_SDC_CMD,
	OFFSET_SDC_ARG,
	OFFSET_SDC_STS,
	OFFSET_SDC_RESP0,
	OFFSET_SDC_RESP1,
	OFFSET_SDC_RESP2,
	OFFSET_SDC_RESP3,
	OFFSET_SDC_BLK_NUM,
	OFFSET_SDC_VOL_CHG,
	OFFSET_SDC_CSTS,
	OFFSET_SDC_CSTS_EN,
	OFFSET_SDC_DCRC_STS,
	OFFSET_SDC_ADV_CFG0,
	OFFSET_EMMC_CFG0,
	OFFSET_EMMC_CFG1,
	OFFSET_EMMC_STS,
	OFFSET_EMMC_IOCON,
	OFFSET_SDC_ACMD_RESP,
	OFFSET_MSDC_DMA_SA_HIGH,
	OFFSET_MSDC_DMA_SA,
	OFFSET_MSDC_DMA_CA,
	OFFSET_MSDC_DMA_CTRL,
	OFFSET_MSDC_DMA_CFG,
	OFFSET_MSDC_DMA_LEN,
	OFFSET_MSDC_DBG_SEL,
	OFFSET_MSDC_DBG_OUT,
	OFFSET_MSDC_PATCH_BIT0,
	OFFSET_MSDC_PATCH_BIT1,
	OFFSET_MSDC_PATCH_BIT2,
	OFFSET_MSDC_PAD_TUNE0,
	OFFSET_MSDC_PAD_TUNE1,
	OFFSET_MSDC_HW_DBG,
	OFFSET_MSDC_VERSION,

	OFFSET_EMMC50_PAD_DS_TUNE,
	OFFSET_EMMC50_PAD_CMD_TUNE,
	OFFSET_EMMC50_PAD_DAT01_TUNE,
	OFFSET_EMMC50_PAD_DAT23_TUNE,
	OFFSET_EMMC50_PAD_DAT45_TUNE,
	OFFSET_EMMC50_PAD_DAT67_TUNE,
	OFFSET_EMMC51_CFG0,
	OFFSET_EMMC50_CFG0,
	OFFSET_EMMC50_CFG1,
	OFFSET_EMMC50_CFG2,
	OFFSET_EMMC50_CFG3,
	OFFSET_EMMC50_CFG4,
	OFFSET_SDC_FIFO_CFG,
	OFFSET_MSDC_AES_SEL,


	0xFFFF /*as mark of end */
};

u16 msdc_offsets_top[] = {
	0xFFFF /*as mark of end */
};

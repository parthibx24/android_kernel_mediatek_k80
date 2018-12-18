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
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <ddp_pwm_mux.h>
#include <ddp_reg.h>
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif
#define PWM_MSG(fmt, arg...) pr_debug("[PWM] " fmt "\n", ##arg)

/*****************************************************************************
 *
 * variable for get clock node fromdts
 *
*****************************************************************************/
#ifndef CONFIG_MTK_CLKMGR /* Common Clock Framework */
static void __iomem *disp_pmw_mux_base;

#ifndef MUX_DISPPWM_ADDR /* disp pwm source clock select register address */
#define MUX_DISPPWM_ADDR (disp_pmw_mux_base + 0xB0)
#endif

/* clock hard code access API */
#define DRV_Reg32(addr) INREG32(addr)
#define clk_readl(addr) DRV_Reg32(addr)
#define clk_writel(addr, val) mt_reg_sync_writel(val, addr)
#endif /* CONFIG_MTK_CLKMGR  */

static unsigned int disp_pwm_get_pwmmux(void)
{
	unsigned int regsrc = 0;

#ifdef CONFIG_MTK_CLKMGR /* MTK Clock Manager */
	regsrc = DISP_REG_GET(CLK_MUX_SEL0);
#else /* Common Clock Framework */
	if (MUX_DISPPWM_ADDR != NULL)
		regsrc = clk_readl(MUX_DISPPWM_ADDR);
	else
		PWM_MSG("mux addr illegal");

#endif
	return regsrc;
}

/*****************************************************************************
 *
 * disp pwm source clock select mux api
 *
*****************************************************************************/
int disp_pwm_set_pwmmux(unsigned int clk_req)
{
	unsigned int regsrc;
	int ret = 0;

	regsrc = disp_pwm_get_pwmmux();
#ifdef CONFIG_MTK_CLKMGR /* MTK Clock Manager */
	if (clk_req == 0)
		clkmux_sel(MT_CLKMUX_PWM_MM_MUX_SEL, MT_CG_SYS_26M, "DISP_PWM");
	else if (clk_req == 1)
		clkmux_sel(MT_CLKMUX_PWM_MM_MUX_SEL, MT_CG_UPLL_D12, "DISP_PWM");
	else
		ret = -1;
#endif
	PWM_MSG("PWM_MUX %x->%x", regsrc, disp_pwm_get_pwmmux());

	return ret;
}

int disp_pwm_clksource_enable(int clk_req)
{
	return -1;
}

int disp_pwm_clksource_disable(int clk_req)
{
	return -1;
}



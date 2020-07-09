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

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>

#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_spm_mtcmos.h>

/* #include <mach/battery_common.h> TBD */
#include <linux/time.h>

/*
 * extern function
 */
#ifdef CONFIG_MTK_SMART_BATTERY
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern bool is_usb_rdy(void);
bool is_dcp_type = false;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)

int hw_charging_get_charger_type(void)
{
	return STANDARD_HOST;
}

#else

static void hw_bc11_init(void)
{
	int timeout_count = 20;

	msleep(200);
	/* add make sure USB Ready */
	if (is_usb_rdy() == false) {
		battery_log(BAT_LOG_CRTI, "CDP, block\n");
		while (is_usb_rdy() == false) {
			msleep(100);
			timeout_count--;
			if (!timeout_count) {
				battery_log(BAT_LOG_CRTI, "CDP, TIMEOUT!!!\n");
				break;
			}
		}
		battery_log(BAT_LOG_CRTI, "CDP, free\n");
	} else
		battery_log(BAT_LOG_CRTI, "CDP, PASS\n");

	Charger_Detect_Init();

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);

	msleep(50);
	/* mdelay(50); */
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);

	msleep(80);
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);


	return wChargerAvail;
}


static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);

	msleep(80);
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);

	msleep(80);
	/* mdelay(80); */

	/* msleep(80); */
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	if (!wChargerAvail) {
		/* RG_bc11_VSRC_EN[1.0] = 10 */
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	}
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);


	Charger_Detect_Release();

}

int hw_charging_get_charger_type(void)
{
#if 0
	return STANDARD_HOST;
	/* return STANDARD_CHARGER; //adaptor */
#else
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
	/********* Step initial  ***************/
	hw_bc11_init();

	/********* Step DCD ***************/
	if (1 == hw_bc11_DCD()) {
		/********* Step A1 ***************/
		if (1 == hw_bc11_stepA1()) {
			CHR_Type_num = APPLE_2_1A_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Apple 2.1A CHARGER!\r\n");
		} else {
			CHR_Type_num = NONSTANDARD_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Non STANDARD CHARGER!\r\n");
		}
	} else {
		/********* Step A2 ***************/
		if (1 == hw_bc11_stepA2()) {
			/********* Step B2 ***************/
			if (1 == hw_bc11_stepB2()) {
				is_dcp_type = true;
				CHR_Type_num = STANDARD_CHARGER;
				battery_log(BAT_LOG_CRTI,
						    "step B2 : STANDARD CHARGER!\r\n");
			} else {
				CHR_Type_num = CHARGING_HOST;
				battery_log(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			}
		} else {
			CHR_Type_num = STANDARD_HOST;
			battery_log(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		}

	}

    /********* Finally setting *******************************/
	hw_bc11_done();

	return CHR_Type_num;
#endif
}
#endif				/* End of #if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA) */
#endif				/* End of #ifdef CONFIG_MTK_SMART_BATTERY */

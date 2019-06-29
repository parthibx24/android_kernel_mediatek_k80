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

#include <mt-plat/aee.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <mt-plat/upmu_common.h>
#ifdef CONFIG_MTK_TC1_FM_AT_SUSPEND
#include <mt_soc_afe_control.h>
#endif
#include <kpd.h>
#include <hal_kpd.h>
#include <mt-plat/mtk_boot_common.h>

#ifdef CONFIG_KPD_PWRKEY_USE_EINT
static u8 kpd_pwrkey_state = !KPD_PWRKEY_POLARITY;
#endif

static int kpd_show_hw_keycode = 1;
#ifndef EVB_PLATFORM
static int kpd_enable_lprst = 1;
#endif
static u16 kpd_keymap_state[KPD_NUM_MEMS] = {
	0xffff, 0xffff, 0xffff, 0xffff, 0x00ff
};

static bool kpd_sb_enable;

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
static void sb_kpd_release_keys(struct input_dev *dev)
{
	int code;

	for (code = 0; code <= KEY_MAX; code++) {
		if (test_bit(code, dev->keybit)) {
			kpd_print("report release event for sb plug in! keycode:%d\n", code);
			input_report_key(dev, code, 0);
			input_sync(dev);
		}
	}
}

void sb_kpd_enable(void)
{
	kpd_sb_enable = true;
	kpd_print("sb_kpd_enable performed!\n");
	mt_reg_sync_writew(0x0, KP_EN);
	sb_kpd_release_keys(kpd_input_dev);
}

void sb_kpd_disable(void)
{
	kpd_sb_enable = false;
	kpd_print("sb_kpd_disable performed!\n");
	mt_reg_sync_writew(0x1, KP_EN);
}
#else
void sb_kpd_enable(void)
{
	kpd_print("sb_kpd_enable empty function for HAL!\n");
}

void sb_kpd_disable(void)
{
	kpd_print("sb_kpd_disable empty function for HAL!\n");
}
#endif

#ifndef EVB_PLATFORM
static void enable_kpd(int enable)
{
	if (enable == 1) {
			mt_reg_sync_writew((u16)(enable), KP_EN);
			kpd_print("KEYPAD is enabled\n");
	} else if (enable == 0) {
			mt_reg_sync_writew((u16)(enable), KP_EN);
			kpd_print("KEYPAD is disabled\n");
		}
}
#endif

void kpd_get_keymap_state(u16 state[])
{
	state[0] = readw(KP_MEM1);
	state[1] = readw(KP_MEM2);
	state[2] = readw(KP_MEM3);
	state[3] = readw(KP_MEM4);
	state[4] = readw(KP_MEM5);
	kpd_print(KPD_SAY "register = %x %x %x %x %x\n",
		state[0], state[1], state[2], state[3], state[4]);
}

void long_press_reboot_function_setting(void)
{
#ifndef EVB_PLATFORM
	if (kpd_enable_lprst && get_boot_mode() == NORMAL_BOOT) {
		kpd_info("Normal Boot long press reboot selection\n");
#ifdef CONFIG_KPD_PMIC_LPRST_TD
		kpd_info("Enable normal mode LPRST\n");
	#ifdef CONFIG_ONEKEY_REBOOT_NORMAL_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD, CONFIG_KPD_PMIC_LPRST_TD);
	#endif

#ifdef CONFIG_TWOKEY_REBOOT_NORMAL_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD, CONFIG_KPD_PMIC_LPRST_TD);
	#endif
#else
		kpd_info("disable normal mode LPRST\n");
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);

#endif
	} else {
		kpd_info("Other Boot Mode long press reboot selection\n");
#ifdef CONFIG_KPD_PMIC_LPRST_TD
		kpd_info("Enable other mode LPRST\n");
	#ifdef CONFIG_ONEKEY_REBOOT_OTHER_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD, CONFIG_KPD_PMIC_LPRST_TD);
	#endif

#ifdef CONFIG_TWOKEY_REBOOT_OTHER_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD, CONFIG_KPD_PMIC_LPRST_TD);
	#endif
#else
		kpd_info("disable other mode LPRST\n");
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
#endif
	}
#else
	pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
	pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
#endif
}

/********************************************************************/
void kpd_wakeup_src_setting(int enable)
{
#ifndef EVB_PLATFORM
#if 0 /* FM not ready */
	int is_fm_radio_playing = 0;

	/* If FM is playing, keep keypad as wakeup source */
	if (ConditionEnterSuspend() == true)
		is_fm_radio_playing = 0;
	else
		is_fm_radio_playing = 1;

	if (is_fm_radio_playing == 0) {
		if (enable == 1) {
			kpd_print("enable kpd work!\n");
			enable_kpd(1);
		} else {
			kpd_print("disable kpd work!\n");
			enable_kpd(0);
		}
	}
#else
	if (enable == 1) {
		kpd_print("enable kpd work!\n");
		enable_kpd(1);
	} else {
		kpd_print("disable kpd work!\n");
		enable_kpd(0);
	}
#endif
#endif
}

/********************************************************************/
void kpd_init_keymap(u32 keymap[])
{
	int i = 0;

	if (kpd_dts_data.kpd_use_extend_type)
		kpd_keymap_state[4] = 0xffff;
	for (i = 0; i < KPD_NUM_KEYS; i++) {
		keymap[i] = kpd_dts_data.kpd_hw_init_map[i];
		/*kpd_print(KPD_SAY "keymap[%d] = %d\n", i,keymap[i]);*/
	}
}

void kpd_init_keymap_state(u16 keymap_state[])
{
	int i = 0;

	for (i = 0; i < KPD_NUM_MEMS; i++)
		keymap_state[i] = kpd_keymap_state[i];
	kpd_info("init_keymap_state done: %x %x %x %x %x!\n", keymap_state[0], keymap_state[1], keymap_state[2],
		 keymap_state[3], keymap_state[4]);
	}

/********************************************************************/

void kpd_set_debounce(u16 val)
{
	mt_reg_sync_writew((u16)(val & KPD_DEBOUNCE_MASK), KP_DEBOUNCE);
}

/********************************************************************/
void kpd_pmic_rstkey_hal(unsigned long pressed)
{
	if (kpd_dts_data.kpd_sw_rstkey != 0) {
		if (!kpd_sb_enable) {
			input_report_key(kpd_input_dev, kpd_dts_data.kpd_sw_rstkey, pressed);
			input_sync(kpd_input_dev);
			if (kpd_show_hw_keycode) {
				kpd_print(KPD_SAY "(%s) HW keycode =%d using PMIC\n",
				       pressed ? "pressed" : "released", kpd_dts_data.kpd_sw_rstkey);
			}
		}
	}
}

void kpd_pmic_pwrkey_hal(unsigned long pressed)
{
#ifdef CONFIG_KPD_PWRKEY_USE_PMIC
	if (!kpd_sb_enable) {
		input_report_key(kpd_input_dev, kpd_dts_data.kpd_sw_pwrkey, pressed);
		input_sync(kpd_input_dev);
		if (kpd_show_hw_keycode) {
			kpd_print(KPD_SAY "(%s) HW keycode =%d using PMIC\n",
			       pressed ? "pressed" : "released", kpd_dts_data.kpd_sw_pwrkey);
		}
#ifdef CONFIG_MTK_AEE_POWERKEY_HANG_DETECT
		/*aee_powerkey_notify_press(pressed);*/
#endif
		}
#endif
}

/***********************************************************************/
void kpd_pwrkey_handler_hal(unsigned long data)
{
#ifdef CONFIG_KPD_PWRKEY_USE_EINT
	bool pressed;
	u8 old_state = kpd_pwrkey_state;

	kpd_pwrkey_state = !kpd_pwrkey_state;
	pressed = (kpd_pwrkey_state == !!KPD_PWRKEY_POLARITY);
	if (kpd_show_hw_keycode)
		kpd_print(KPD_SAY "(%s) HW keycode = using EINT\n", pressed ? "pressed" : "released");
	kpd_backlight_handler(pressed, kpd_dts_data.kpd_sw_pwrkey);
	input_report_key(kpd_input_dev, kpd_dts_data.kpd_sw_pwrkey, pressed);
	kpd_print("report Linux keycode = %u\n", kpd_dts_data.kpd_sw_pwrkey);
	input_sync(kpd_input_dev);

	/* for detecting the return to old_state */
	mt_eint_set_polarity(KPD_PWRKEY_EINT, old_state);
	mt_eint_unmask(KPD_PWRKEY_EINT);
#endif
}

/***********************************************************************/
void mt_eint_register(void)
{
#ifdef CONFIG_KPD_PWRKEY_USE_EINT
	mt_eint_set_sens(KPD_PWRKEY_EINT, KPD_PWRKEY_SENSITIVE);
	mt_eint_set_hw_debounce(KPD_PWRKEY_EINT, KPD_PWRKEY_DEBOUNCE);
	mt_eint_registration(KPD_PWRKEY_EINT, true, KPD_PWRKEY_POLARITY, kpd_pwrkey_eint_handler, false);
#endif
}

/************************************************************************/

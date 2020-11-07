/**
 * Version: Initial
 * Copyright (c) 2015,
 * MediaTek Inc <mediatek.com>
 * Copyright (C) 2017,
 * WIKO S.A.S <wikogeek.com>
 *
 * Version: 20.1107
 * Copyright (C) 2020,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "st7701s_fwvga_dsi_vdo_hf"
#define LCM_VERSION "20.1107" /* YEAR+"."+MONTH+DATE */
#define LCM_ID (0x9806)
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/**
 * REGFLAG_DELAY, used to trigger MDELAY,
 * REGFLAG_END_OF_TABLE, used to mark the end of LCM_setting_table.
 * their values dosen't matter until they,
 * match with any LCM_setting_table->cmd.
 */
#define REGFLAG_DELAY (0xFF)
#define REGFLAG_END_OF_TABLE (0xDD)

#define LCM_DBG_TAG "[LCM]"

#ifdef BUILD_LK
#define LCM_LOGD(str, args...) print(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)
#else
#define LCM_LOGD(str, args...) pr_info(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)
#endif

/* Local Functions */
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)

/* LCM Driver Implementations */

static struct LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[5];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0x11,0,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xE9,0x03}},
	{0xC1,2,{0x12,0x02}},
	{0xC2,2,{0x37,0x08}}, // 1dot
	{0xCC,1,{0x10}},
	{0xB0,16,{0x40,0x03,0x0C,0x14,0x19,0x0C,0x0F,0x09,0x09,0x21,0x07,0x13,0x12,0x10,0x1A,0x14}},
	{0xB1,16,{0x40,0x03,0xCC,0x10,0x17,0x0B,0x0F,0x09,0x09,0x27,0x09,0x17,0x12,0x1C,0x1B,0x14}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x46}}, // VRH
	//{0xB1,1,{0x4B}}, // VCOM
	{0xB2,1,{0x87}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x47}},
	{0xB7,1,{0x85}},
	{0xB8,1,{0x21}},
	{0xB9,1,{0x10}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},

    {0xE0,3,{0x00,0x00,0x02}},
    {0xE1,11,{0x0B,0x00,0x0D,0x00,0x0C,0x00,0x0E,0x00,0x00,0x33,0x33}},
    {0xE2,13,{0x33,0x33,0x44,0x44,0x64,0x00,0x66,0x00,0x65,0x00,0x67,0x00,0x00}},
    {0xE3,4,{0x00,0x00,0x33,0x33}},
    {0xE4,2,{0x44,0x44}},
    {0xE5,16,{0x0C,0x78,0x0C,0xA0,0x0E,0x78,0x0C,0xA0,0x10,0x78,0x0C,0xA0,0x12,0x78,0x0C,0xA0}},
    {0xE6,4,{0x00,0x00,0x33,0x33}},
    {0xE7,2,{0x44,0x44}},
    {0xE8,16,{0x0D,0x78,0x0C,0xA0,0x0F,0x78,0x0C,0xA0,0x11,0x78,0x0C,0xA0,0x13,0x78,0x0C,0xA0}},
    {0xEB,7,{0x02,0x02,0x39,0x39,0xEE,0x44,0x00}},
    {0xEC,2,{0x00,0x00}},
    {0xED,16,{0xFF,0xF1,0x04,0x56,0x72,0x3F,0xFF,0xFF,0xFF,0xFF,0xF3,0x27,0x65,0x40,0x1F,0xFF}},
    {0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{REGFLAG_DELAY,10,{}},
	{0x29,0,{0x00}},
	{REGFLAG_DELAY,50,{}},
	{REGFLAG_END_OF_TABLE,0x00,{}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
    // Sleep Mode On
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    int i;
    for(i = 0; i < count; i++)
    {
        switch (table[i].cmd) {
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;
        case REGFLAG_END_OF_TABLE :
            break;
        default:
            dsi_set_cmdq_V2(table[i].cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
    memset(params, 0, sizeof(struct LCM_PARAMS));

    params->type = 2;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->physical_width = 61.63;
    params->physical_height = 109.65;

    // enable tearing-free
    params->dbi.te_mode = 0;
    params->dbi.te_edge_polarity = 0;

    // DSI
    params->dsi.mode = 1;
    params->dsi.PLL_CLOCK = 190;
    params->dsi.LANE_NUM = 2;
    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;
    params->dsi.data_format.format = 2;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS = 2;
    params->dsi.word_count = FRAME_WIDTH * 3;

    params->dsi.vertical_sync_active =10;
    params->dsi.vertical_backporch = 20;
    params->dsi.vertical_frontporch = 18;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 5;
    params->dsi.horizontal_backporch = 60;
    params->dsi.horizontal_frontporch = 60;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.compatibility_for_nvk = 0;

    params->dsi.ssc_disable = 1;
    params->dsi.ssc_range = 2;

    params->dsi.esd_check_enable = 0;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(25);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting,
               sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    // 60ms for ps function zx
    MDELAY(60); 
    SET_RESET_PIN(0);

    // For debuging lcm_compare_id
    if(lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");
}

/* Get LCM Driver Hooks */
struct LCM_DRIVER st7701s_fwvga_dsi_vdo_hf_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
};
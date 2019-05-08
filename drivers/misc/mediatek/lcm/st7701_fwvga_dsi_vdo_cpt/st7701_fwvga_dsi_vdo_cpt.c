/**
 * Version 19.58
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "st7701_fwvga_dsi_vdo_cpt" "_lcm_drv"
#define LCM_VERSION "19.58" /* YEAR+"."+MONTH+DATE */
#define LCM_ID (0x9807)
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

/* Local Debug Variables */
#define LCM_DBG_TAG "[LCM]"
#define LCM_LOGD(str, args...) pr_info(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)

#ifdef BUILD_LK
#undef LCM_LOGD
#define LCM_LOGD(str, args...) printf(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)
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

static LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[2];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
    {0x11,0,{}},
    {REGFLAG_DELAY,120,{}},
    {0x29,0,{}},
    {REGFLAG_DELAY,120,{}},
    {0x28,1,{0x00}},
    {REGFLAG_DELAY,20,{}},,
    {0x10,1,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0x28,0,{}},
    {0x10,0,{}},
    {REGFLAG_DELAY,120,{}},
    {REGFLAG_END_OF_TABLE, 0, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0, {}}
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

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = 2;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dsi.mode = 2;
    params->dsi.PS = 2;
    params->dsi.LANE_NUM = 2;
    params->dsi.PLL_CLOCK = 200;
    params->dsi.packet_size = 256;

    params->dsi.HS_PRPR = 3;
    params->dsi.LPX = 3;
    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;
    params->dsi.data_format.format = 2;
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.horizontal_sync_active = 50;
    params->dsi.horizontal_backporch = 60;
    params->dsi.horizontal_frontporch = 100;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.vertical_sync_active = 8;
    params->dsi.vertical_backporch = 18;
    params->dsi.vertical_frontporch = 16;
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.lcm_ext_te_enable = 1;

    params->dbi.te_mode = 1;
    params->dbi.te_edge_polarity = 0;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9Cu;

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting,
               sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
    unsigned int data_array[16];
    unsigned int id = 0;
    unsigned char buffer[3];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);

    data_array[0] = 0x13700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x00, buffer, 3);

    id = (buffer[0] << 8) | buffer[1];
    LCM_LOGD("Synced id is 0x%2x", id);

    return (LCM_ID == id) ? 1 : 0;
}

static void lcm_suspend(void)
{
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(50);

    push_table(lcm_deep_sleep_mode_in_setting,
        sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    // For debuging lcm_compare_id
    if(lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");

}

/* Get LCM Driver Hooks */
LCM_DRIVER st7701_fwvga_dsi_vdo_cpt_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};
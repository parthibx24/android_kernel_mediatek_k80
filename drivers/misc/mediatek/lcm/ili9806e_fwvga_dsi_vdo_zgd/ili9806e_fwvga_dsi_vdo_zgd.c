/**
 * Version Initial
 * Copyright (c) 2015,
 * MediaTek Inc <mediatek.com>
 * Copyright (C) 2017,
 * WIKO S.A.S <wikogeek.com>
 *
 * Version 19.417
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "ili9806e_fwvga_dsi_vdo_zgd"
#define LCM_VERSION "19.417" /* YEAR+"."+MONTH+DATE */
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
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)  lcm_util.dsi_write_cmd(cmd)

/* LCM Driver Implementations */

static LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[5];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
    {0x08,1,{0X10}},
    {0x21,1,{0X01}},
    {0x30,1,{0X01}},
    {0x31,1,{0X00}},
    {0x50,1,{0X50}},
    {0x51,1,{0X50}},
    {0x60,1,{0X07}},
    {0x61,1,{0X00}},
    {0x62,1,{0X07}},
    {0x63,1,{0X00}},
    {0x40,1,{0X18}},
    {0x41,1,{0X64}},
    {0x42,1,{0X03}},
    {0x43,1,{0X0A}},
    {0x44,1,{0X06}},
    {0x46,1,{0X55}},
    {0x47,1,{0X55}},

    // {0x52,1,{0X00}},
    {0x53,1,{0X52}},
    {0x57,1,{0X50}},
    // {0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
    {0xA0,1,{0X00}},
    {0xA1,1,{0X08}},
    {0xA2,1,{0X12}},  // 0X1E
    {0xA3,1,{0X12}},
    {0xA4,1,{0X0A}},
    {0xA5,1,{0X1A}},
    {0xA6,1,{0X0D}},
    {0xA7,1,{0X09}},
    {0xA8,1,{0X04}},
    {0xA9,1,{0X0A}},
    {0xAA,1,{0X07}},
    {0xAB,1,{0X04}},
    {0xAC,1,{0X0C}},
    {0xAD,1,{0X2D}},  // 0X2F
    {0xAE,1,{0X29}},
    {0xAF,1,{0X00}},
    {0xC0,1,{0X00}},
    {0xC1,1,{0X02}},
    {0xC2,1,{0X09}},  // 0X18
    {0xC3,1,{0X0F}},
    {0xC4,1,{0X06}},
    {0xC5,1,{0X15}},
    {0xC6,1,{0X06}},
    {0xC7,1,{0X08}},
    {0xC8,1,{0X04}},
    {0xC9,1,{0X07}},
    {0xCA,1,{0X06}},
    {0xCB,1,{0X03}},
    {0xCC,1,{0X0C}},
    {0xCD,1,{0X2C}},  // 0X1E
    {0xCE,1,{0X26}},
    {0xCF,1,{0X00}},
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},
    {0x00,1,{0X21}},
    {0x01,1,{0X06}},
    {0x02,1,{0XA0}},
    {0x03,1,{0X02}},
    {0x04,1,{0X01}},
    {0x05,1,{0X01}},
    {0x06,1,{0X80}},
    {0x07,1,{0X03}},
    {0x08,1,{0X06}},
    {0x09,1,{0X80}},
    {0x0A,1,{0X00}},
    {0x0B,1,{0X00}},
    {0x0C,1,{0X20}},
    {0x0D,1,{0X20}},
    {0x0E,1,{0X09}},
    {0x0F,1,{0X00}},
    {0x10,1,{0XFF}},
    {0x11,1,{0XE0}},
    {0x12,1,{0X00}},
    {0x13,1,{0X00}},
    {0x14,1,{0X00}},
    {0x15,1,{0XC0}},
    {0x16,1,{0X08}},
    {0x17,1,{0X00}},
    {0x18,1,{0X00}},
    {0x19,1,{0X00}},
    {0x1A,1,{0X00}},
    {0x1B,1,{0X00}},
    {0x1C,1,{0X00}},
    {0x1D,1,{0X00}},
    {0x20,1,{0X01}},
    {0x21,1,{0X23}},
    {0x22,1,{0X45}},
    {0x23,1,{0X67}},
    {0x24,1,{0X01}},
    {0x25,1,{0X23}},
    {0x26,1,{0X45}},
    {0x27,1,{0X67}},
    {0x30,1,{0X12}},
    {0x31,1,{0X22}},
    {0x32,1,{0X22}},
    {0x33,1,{0X22}},
    {0x34,1,{0X87}},
    {0x35,1,{0X96}},
    {0x36,1,{0XAA}},
    {0x37,1,{0XDB}},
    {0x38,1,{0XCC}},
    {0x39,1,{0XBD}},
    {0x3A,1,{0X78}},
    {0x3B,1,{0X69}},
    {0x3C,1,{0X22}},
    {0x3D,1,{0X22}},
    {0x3E,1,{0X22}},
    {0x3F,1,{0X22}},
    {0x40,1,{0X22}},
    {0x52,1,{0X10}},
    {0x53,1,{0X10}},
    {0x54,1,{0X13}},
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
    {0x02,1,{0X77}},
    {0x06,1,{0X13}},
    {0xE1,1,{0X79}},
    {0x17,1,{0X22}},
    {0xB3,1,{0X10}},
    {0x26,1,{0XB2}},

    // Change to Page 0
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},

    // TE on
    {0x35,1,{0x00}},

    // Sleep-Out
    {0x11,1,{0x00}},

    {REGFLAG_DELAY, 150, {}},

    // Display On
    {0x29,1,{0x00}},
    {REGFLAG_DELAY, 10, {}},

    /** Note
     Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
     Setting ending by predefined flag */
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
    // Sleep Mode On
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = 
{
    {0xD3,  3,  {0xFF, 0x83, 0x79}},
    {REGFLAG_DELAY, 10, {}},

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

    params->physical_width = 61.63;
    params->physical_height = 109.65;

    // enable tearing-free
    // params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;

    params->dbi.te_mode = 0;
    // params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    params->dsi.mode = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_TWO_LANE;
    // The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count = FRAME_WIDTH * 3;

    params->dsi.vertical_sync_active = 5;
    params->dsi.vertical_backporch = 8;
    params->dsi.vertical_frontporch = 8;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 8;
    params->dsi.horizontal_backporch = 50;
    params->dsi.horizontal_frontporch = 46;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.compatibility_for_nvk = 0;

    params->dsi.ssc_disable = 1;
    params->dsi.ssc_range = 6;
    params->dsi.PLL_CLOCK = 176;

    params->dsi.esd_check_enable = 1;
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

static unsigned int lcm_compare_id(void)
{
    unsigned int data_array[16];
    unsigned int id = 0;
    unsigned char data[3];
    unsigned char buffer[3];

    // Do reset here
    SET_RESET_PIN(1);
    MDELAY(2);
    SET_RESET_PIN(0);
    MDELAY(25);
    SET_RESET_PIN(1);
    MDELAY(120);

    data_array[0] = 0x00063902;
    data_array[1] = 0x0698ffff;
    data_array[2] = 0x00000104;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(10);

    data_array[0] = 0x00023700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x00, buffer[0], 1);

    MDELAY(2);
    read_reg_v2(0x01, buffer[1], 1);

    MDELAY(2);
    read_reg_v2(0x02, buffer[2], 1);

    id = (buffer[0] << 8) | buffer[1];
    LCM_LOGD("Synced id is 0x%2x", id);

    return (LCM_ID == id) ? 1 : 0;
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    MDELAY(60); // 60ms for ps function zx
    SET_RESET_PIN(0);

    // For debuging lcm_compare_id
    if(lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");
}

/* Get LCM Driver Hooks */
LCM_DRIVER ili9806e_fwvga_dsi_vdo_zgd_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};
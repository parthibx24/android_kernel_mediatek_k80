/**
 * Version 19.28
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "jd9161_fwvga_dsi_vdo_dj"
#define LCM_VERSION "19.28" /* YEAR+"."+MONTH+DATE */
#define LCM_ID (0x9161)
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

#define AUXADC_LCM_VOLTAGE_CHANNEL 12
#define AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL 1
#define MIN_VOLTAGE (1000)
#define MAX_VOLTAGE (1800)

/**
 * REGFLAG_DELAY, used to trigger MDELAY,
 * REGFLAG_END_OF_TABLE, used to mark the end of LCM_setting_table.
 * their values dosen't matter until they,
 * match with any LCM_setting_table->cmd.
 */
#define REGFLAG_DELAY (0xFE)
#define REGFLAG_END_OF_TABLE (0xDD) /* END OF REGISTERS MARKER */

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* Local Debug Variables */
#define LCM_DBG_TAG "[LCM]"
#define LCM_LOGD(str, args...) pr_info(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)

#ifdef BUILD_LK
#undef LCM_LOGD
#define LCM_LOGD(str, args...) print(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)
#endif

/* Local Functions */
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

/* LCM Driver Implementations */

static LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
    {0xBF,  3, {0x91,0x61,0xF2}},
    {0xB8,  6, {0x00,0xA7,0x00,0x00,0xA7,0x00}},
    {0xBA,  3, {0x34,0x23,0x00}},
    {0xC3,  1, {0x04}},
    {0xC4,  2, {0x30,0x6A}},
    {0xC7,  9, {0x00,0x01,0x32,0x05,0x65,0x2A,0x1B,0xA5,0xA5}},
    {0xC8, 38, {0x7C,0x5C,0x3F,0x36,0x39,0x2F,0x33,0x19,0x33,0x34,0x39,0x5E,0x53,0x65,0x5E,0x66,0x61,0x57,0x48,0x7C,0x5C,0x3F,0x36,0x39,0x2F,0x33,0x19,0x33,0x34,0x39,0x5E,0x53,0x65,0x5E,0x66,0x61,0x57,0x48}},    
    {0xD4, 16, {0x1F,0x1E,0x05,0x07,0x01,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {0xD5, 16, {0x1F,0x1E,0x04,0x06,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {0xD6, 16, {0x1F,0x1F,0x04,0x06,0x00,0x1E,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {0xD7, 16, {0x1F,0x1F,0x05,0x07,0x01,0x1E,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {0xD8, 20, {0x20,0x00,0x00,0x10,0x03,0x20,0x01,0x02,0x00,0x01,0x02,0x5F,0x5F,0x00,0x00,0x32,0x04,0x5F,0x5F,0x08}}, 
    {0xD9, 19, {0x00,0x0A,0x0A,0x88,0x00,0x00,0x06,0x7B,0x00,0x00,0x00,0x3B,0x2F,0x1F,0x00,0x00,0x00,0x03,0x7B}},
    {0xBE,  1, {0x01}},
    {0xCC, 10, {0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x40,0x00,0x00}},
    
    {0xBE, 1, {0x00}},
    {0x35, 1, {0x00}},
    {0x11, 1, {0x00}},
    
    {REGFLAG_DELAY, 120, {}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    {0xBF, 3, {0x09,0xB1,0x7F}},
    {REGFLAG_END_OF_TABLE, 0, {}}
};

/* unused,
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Normal mode on
    // {0x13, 1, {0x00}},
    // {REGFLAG_DELAY,20,{}},
    
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
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

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params) {

	memset(params, 0, sizeof(LCM_PARAMS));

    params->type = 2;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    params->physical_width  = 61.63;
    params->physical_height = 109.65;

    params->dsi.mode = 1;
    params->dsi.PS = 2;
    params->dsi.LANE_NUM = 2;
    params->dsi.packet_size = 256;
    params->dsi.word_count = FRAME_WIDTH * 3;
    params->dsi.PLL_CLOCK = 170;
    
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.compatibility_for_nvk = 0;

    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;
    params->dsi.data_format.format = 2;

    params->dsi.ssc_disable = 1;
    params->dsi.ssc_range = 6;

    params->dsi.horizontal_sync_active = 20;
    params->dsi.horizontal_backporch = 20;
    params->dsi.horizontal_frontporch = 20;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 6;
    params->dsi.vertical_frontporch = 6;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dbi.te_mode = 0;
    //params->dbi.te_edge_polarity = 0;
    
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
    
    push_table(lcm_initialization_setting,
        sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void) {

    unsigned int data_array[16];
    unsigned int id = 0;
    unsigned char buffer[3];

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
    
    /* 0xDA -> 0x91,  0xDB -> 0x61, 0xDC -> ? */
    read_reg_v2(0xDA, buffer, 3);

    id = (buffer[0] << 8) | buffer[1];
    LCM_LOGD("Synced id is 0x%2x", id);

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
{
    int err = 0;
    int data[4] = {0,0,0,0};
    int rawdata = 0;
    int lcm_voltage = 0;

    err = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawdata);
    if(err < 0) {
        LCM_LOGD("[adc_uboot]: get data error\n");
        return 0;
    }
        
    lcm_voltage = data[0] * 1000 + data[1] * 10;
    
    LCM_LOGD("[adc_uboot]: lcm_vol = %d\n", lcm_voltage);
    
    return ((LCM_ID == id) && (lcm_voltage < MAX_VOLTAGE) && (lcm_voltage > MIN_VOLTAGE)) ? 1 : 0;
}
#endif

    return (LCM_ID == id) ? 1 : 0;
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting,
        sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    // For debuging lcm_compare_id
    if(lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");

}

/* Get LCM Driver Hooks */
LCM_DRIVER jd9161_fwvga_dsi_vdo_dj_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};

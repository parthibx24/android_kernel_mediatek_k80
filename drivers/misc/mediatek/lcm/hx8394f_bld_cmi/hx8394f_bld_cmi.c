/**
 * Version 19.222
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "hx8394f_bld_cmi"
#define LCM_VERSION "19.222" /* YEAR+"."+MONTH+DATE */
#define LCM_ID (0x83940F)
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

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

/* LCM Driver Implementations */

static LCM_UTIL_FUNCS lcm_util = { 0 };

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
    params->dsi.data_format.format = 2;
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.PS = 2;
    params->dsi.LANE_NUM = 3;
    params->dsi.packet_size = 256;
    params->dsi.PLL_CLOCK = 264;

    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;
    params->dsi.ssc_range = 0;

    params->dsi.horizontal_sync_active = 50;
    params->dsi.horizontal_backporch = 50;
    params->dsi.horizontal_frontporch = 50;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 15;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dbi.te_mode = 0;
    params->dbi.te_edge_polarity = 0;
}

static void init_hx8394f_lcm_registers(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x43902;
    data_array[1] = 0x9483FFB9;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x73902;
    data_array[1] = 0x680362BA;
    data_array[2] = 0xC0B26B;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0] = 0xB3902;
    data_array[1] = 0x721248B1;
    data_array[2] = 0xB1543309;
    data_array[3] = 0x2F6B31;
    dsi_set_cmdq(data_array, 4, 1);
    MDELAY(1);

    data_array[0] = 0x73902;
    data_array[1] = 0x648000B2;
    data_array[2] = 0x2F0D0E;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0] = 0x163902;
    data_array[1] = 0x737473B4;
    data_array[2] = 0x1747374;
    data_array[3] = 0x75860C;
    data_array[4] = 0x7374733F;
    data_array[5] = 0x1747374;
    data_array[6] = 0x860C;
    dsi_set_cmdq(data_array, 7, 1);
    MDELAY(1);

    data_array[0] = 0x33902;
    data_array[1] = 0x6262B6;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x223902;
    data_array[1] = 0x70000D3;
    data_array[2] = 0x10074007;
    data_array[3] = 0x8100800;
    data_array[4] = 0x15540800;
    data_array[5] = 0x20E050E;
    data_array[6] = 0x6050615;
    data_array[7] = 0xA0A4447;
    data_array[8] = 0x707104B;
    data_array[9] = 0x400E;
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(1);

    data_array[0] = 0x2D3902;
    data_array[1] = 0x1B1A1AD5;
    data_array[2] = 0x201001B;
    data_array[3] = 0x6050403;
    data_array[4] = 0xA090807;
    data_array[5] = 0x1825240B;

    data_array[7] = (0x1818 << 16) | 0x1818;
    data_array[6] = 0x18272618;
    data_array[8] = (0x1818 << 16) | 0x1818;
    data_array[9] = (0x1818 << 16) | 0x1818;
    data_array[10] = 0x20181818;
    data_array[11] = 0x18181821;
    data_array[12] = 0x18;
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);

    data_array[0] = 0x2D3902;
    data_array[1] = 0x1B1A1AD6;
    data_array[2] = 0x90A0B1B;
    data_array[3] = 0x5060708;
    data_array[4] = 0x1020304;
    data_array[5] = 0x18202100;
    data_array[6] = 0x18262718;
    data_array[7] = 0x18181818;
    data_array[8] = 0x18181818;
    data_array[9] = 0x18181818;
    data_array[10] = 0x25181818;
    data_array[11] = 0x18181824;
    data_array[12] = 0x18;
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);

    data_array[0] = 0x3B3902;
    data_array[1] = 0xE0300E0;
    data_array[2] = 0x1F1B1714;
    data_array[3] = 0x6453401F;
    data_array[4] = 0x88836F64;
    data_array[5] = 0x9297968A;
    data_array[6] = 0x5255AD9E;
    data_array[7] = 0x605B5955;
    data_array[8] = 0x3007F64;
    data_array[9] = 0x1B16130D;
    data_array[10] = 0x53401F1F;
    data_array[11] = 0x836F6464;
    data_array[12] = 0x97968A88;
    data_array[13] = 0x55AD9E92;
    data_array[14] = 0x5B595552;
    data_array[15] = 0x7F6460;
    dsi_set_cmdq(data_array, 16, 1);
    MDELAY(1);

    data_array[0] = 0x33902;
    data_array[1] = 0x311FC0;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0xBCC;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0x2D4;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0x2BD;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0xD3902;
    data_array[1] = 0xFFFFFFD8;
    data_array[2] = 0xFFFFFFFF;
    data_array[3] = 0xFFFFFFFF;
    data_array[4] = 0xFF;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0xBD;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0xBD;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0x1BD;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0xB1;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[1] = 0xBD;
    data_array[0] = 0x23902;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x83902;
    data_array[1] = 0x508140BF;
    data_array[2] = 0x1FC1A00;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0] = 0x110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);

    data_array[0] = 0x290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);
};


static void lcm_init(void)
{
    LCM_LOGD("Starting LCM Initialization!");

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);

    init_hx8394f_lcm_registers();
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
    MDELAY(120);

    data_array[0] = 0x43902;
    data_array[1] = 0x9483FFB9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x33700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x04, buffer, 3);

    id = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    LCM_LOGD("Synced id is 0x%2x", id);

    return (LCM_ID == id) ? 1 : 0;
}

static void lcm_suspend(void)
{
    unsigned int data_array[16];

    LCM_LOGD("Using dsi_set_cmdq v1!");

    // Display Off
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);

    // Sleep In
    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);

    /**
    // For debuging lcm_compare_id
    if(lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");
    */
}

/* Get LCM Driver Hooks */
LCM_DRIVER hx8394f_bld_cmi_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};
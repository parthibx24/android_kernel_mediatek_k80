/**
 * Version 19.221
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "hx8394d_bld_cmi"
#define LCM_VERSION "19.221" /* YEAR+"."+MONTH+DATE */
#define LCM_ID (0x83940D)
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

    params->dsi.vertical_sync_active = 2;
    params->dsi.vertical_backporch = 5;
    params->dsi.vertical_frontporch = 9;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dbi.te_mode = 0;
    params->dbi.te_edge_polarity = 0;

}

static void init_hx8394d_lcm_registers(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x43902;
    data_array[1] = 0x9483FFB9;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x33902;
    data_array[1] = 0x8372BA;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x103902;
    data_array[1] = 0x11116CB1;
    data_array[2] = 0xF1110434;
    data_array[3] = 0x239EE280;
    data_array[4] = 0x58D2C080;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0] = 0xC3902;
    data_array[1] = 0x564C0B2;
    data_array[2] = 0x81C3207;
    data_array[3] = 0x4D1C08;
    dsi_set_cmdq(data_array, 4, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0x7BC;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x43902;
    data_array[1] = 0x10E41BF;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0xD3902;
    data_array[1] = 0x3FF00B4;
    data_array[2] = 0x35A035A;
    data_array[3] = 0x16A015A;
    data_array[4] = 0x6A;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0] = 0x1F3902;
    data_array[1] = 0xD3;
    data_array[2] = 0x100740;
    data_array[3] = 0x1032;
    data_array[4] = 0x4153200;
    data_array[5] = 0x15320405;
    data_array[6] = 0x37270528;
    data_array[7] = 0x370B0033;
    data_array[8] = 0x70710;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0] = 0x2D3902;
    data_array[1] = 0x181818D5;
    data_array[2] = 0x18181818;
    data_array[3] = 0x18181818;
    data_array[4] = 0x25181818;
    data_array[5] = 0x18262724;
    data_array[6] = 0x1040518;
    data_array[7] = 0x3060700;
    data_array[8] = 0x47424302;
    data_array[9] = 0x41444546;
    data_array[10] = 0x23202140;
    data_array[11] = 0x18181822;
    data_array[12] = 0x18;
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);

    data_array[0] = 0x23902;
    data_array[1] = 0x9CC;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x33902;
    data_array[1] = 0x1430C0;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x33902;
    data_array[1] = 0x3D3DB6;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x2B3902;
    data_array[1] = 0x211A05E0;
    data_array[2] = 0x303F3735;
    data_array[3] = 0xC0A074A;
    data_array[4] = 0x17151017;
    data_array[5] = 0x11061514;
    data_array[6] = 0x17051714;
    data_array[7] = 0x3F39381B;
    data_array[8] = 0xB074F2D;
    data_array[9] = 0x120F180D;
    data_array[10] = 0xB131214;
    data_array[11] = 0x191716;
    dsi_set_cmdq(data_array, 12, 1);
    MDELAY(1);

    data_array[0] = 0x53902;
    data_array[1] = 0x40C000C7;
    data_array[2] = 0xC0;
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
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    init_hx8394d_lcm_registers();
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
LCM_DRIVER hx8394d_bld_cmi_lcm_drv =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = lcm_compare_id,
};

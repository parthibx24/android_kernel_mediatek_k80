/**
 * Version 19.221
 * Copyright (C) 2019,
 * parthibx24 <e.inxpired@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

#include "lcm_drv.h"

#ifdef BUILD_LK
#error "univerxal driver won't work in lk"
#endif

/* Local Constants */
#define LCM_NAME "univerxal"
#define LCM_VERSION "19.52" /* YEAR+"."+MONTH+DATE */
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

/* Local Debug Variables */
#define LCM_DBG_TAG "[LCM]"
#define LCM_LOGD(str, args...) pr_info(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)

/* LCM Driver Implementations */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    LCM_UTIL_FUNCS lcm_util = { 0 };
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = 2; // disp_lcm_probe

    params->width = FRAME_WIDTH; // disp_lcm_probe
    params->height = FRAME_HEIGHT; // disp_lcm_probe

    params->dsi.mode = 2; // disp_lcm_is_video_mode()

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

}

/* Get LCM Driver Hooks */
LCM_DRIVER univerxal =
{
    .name           = LCM_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
};

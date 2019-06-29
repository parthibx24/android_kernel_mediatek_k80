/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Shi.Ma <shi.ma@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _DT_BINDINGS_MMC_MT6535_H
#define _DT_BINDINGS_MMC_MT6535_H

#define MSDC_EMMC               (0)
#define MSDC_SD                 (1)
#define MSDC_SDIO               (2)

#define MSDC_CD_HIGH            (1)
#define MSDC_CD_LOW             (0)

/* each PLL have different gears for select
 * software can used mux interface from clock management module to select */

#define MSDC0_CLKSRC_26MHZ		(0)
#define MSDC0_CLKSRC_800MHZ		(1)
#define MSDC0_CLKSRC_400MHZ		(2)
#define MSDC0_CLKSRC_200MHZ		(3)
#define MSDC0_CLKSRC_182MHZ		(4)
#define MSDC0_CLKSRC_136MHZ		(5)
#define MSDC0_CLKSRC_156MHZ		(6)
#define MSDC0_CLKSRC_416MHZ		(7)
#define MSDC0_CLKSRC_48MHZ		(8)
#define MSDC0_CLKSRC_91MHZ		(9)
#define MSDC0_CLKSRC_624MHZ		(10)

#define MSDC1_CLKSRC_26MHZ		(0)
#define MSDC1_CLKSRC_208MHZ		(1)
#define MSDC1_CLKSRC_200MHZ		(2)
#define MSDC1_CLKSRC_182MHZ		(3)
#define MSDC1_CLKSRC_136MHZ		(4)
#define MSDC1_CLKSRC_156MHZ		(5)
#define MSDC1_CLKSRC_48MHZ		(6)
#define MSDC1_CLKSRC_91MHZ		(7)


#define MSDC_SMPL_RISING        (0)
#define MSDC_SMPL_FALLING       (1)
#define MSDC_SMPL_SEPARATE      (2)

#endif /* _DT_BINDINGS_MMC_MT6580_H */

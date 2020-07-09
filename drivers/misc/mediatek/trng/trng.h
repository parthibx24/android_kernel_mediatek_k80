/*
 * Copyright (C) 2018 MediaTek Inc.
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

#ifndef __TRNG_H__
#define __TRNG_H__

#define TRNG_CTRL			(trng_base + 0x0000)
#define TRNG_TIME			(trng_base + 0x0004)
#define TRNG_DATA			(trng_base + 0x0008)

#define TRNG_CG_PDN_SET			(infracfg_base + 0x0088)
#define TRNG_CG_PDN_CLR			(infracfg_base + 0x008C)
#define TRNG_CG_PDN_STATUS		(infracfg_base + 0x0094)

#define TRNG_CTRL_RDY			0x80000000
#define TRNG_CTRL_START			0x00000001
#define TRNG_CG_PDN_VALUE		0x200


#ifdef TRNG_DUMP
static char *dirname = "sec";
struct proc_dir_entry *parent;
#endif

#endif /* __TRNG_H__ */

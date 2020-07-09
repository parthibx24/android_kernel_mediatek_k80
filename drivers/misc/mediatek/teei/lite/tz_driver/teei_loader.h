/*
 * Copyright (C) 2016 Microtrust, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEEI_LOADER_H_
#define _TEEI_LOADER_H_

struct TEEC_UUID {
	uint32_t timeLow;
	uint16_t timeMid;
	uint16_t timeHiAndVersion;
	uint8_t clockSeqAndNode[8];
};

extern struct TEEC_UUID uuid_fp;
extern s32 trusty_mtee_std_call32(u32 smcnr, u32 a0, u32 a1, u32 a2);

#endif

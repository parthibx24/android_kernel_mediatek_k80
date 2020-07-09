/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _TEEI_SMCALL_H_
#define _TEEI_SMCALL_H_
#include <linux/trusty/smcall.h>

#define SMC_ENTITY_LAUNCH		61	/* Trusted OS launch call */
#define SMC_GET_TEE_STATUS	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 4)
#define SMC_NT_SCHED_T		SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 5)
#define SMC_SWITCH_CORE		SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 6)


#define SMC_FC_SEND_LOG_ADDR	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 12)
#define SMC_FC_START_TIME	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 13)

/* Launch TA */
#define SMC_SC_LOAD_TA		SMC_STDCALL_NR(SMC_ENTITY_LAUNCH, 1)
#define SMC_SC_SEND_MODEL_INFO	SMC_STDCALL_NR(SMC_ENTITY_LAUNCH, 2)

#endif /* end _TEEI_SMCALL_H_ */

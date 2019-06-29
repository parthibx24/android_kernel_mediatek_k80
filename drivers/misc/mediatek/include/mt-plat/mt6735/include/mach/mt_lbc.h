/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifndef __MT_LBC_H__
#define __MT_LBC_H__

#define LHD_KIR_PERF 0
#define LHD_KIR_PERF_KERN 1
#define LHD_KIR_FBC 2
#define LHD_KIR_WIFI 3
#define LHD_MAX_KIR 4


struct lhd_limit_data {
	int min;
	int max;
};

int lhd_update_userlimit_cpu_freq(int kicker, int num_cluster, struct lhd_limit_data *freq_limit);
int lhd_update_userlimit_cpu_core(int kicker, int num_cluster, struct lhd_limit_data *core_limit);

#endif /* __MT_LBC_H__ */

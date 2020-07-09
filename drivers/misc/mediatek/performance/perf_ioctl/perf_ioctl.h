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
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

#include <linux/platform_device.h>

#include <linux/ioctl.h>
#include <mach/mt_lbc.h>

#define DEV_MAJOR 121
#define DEV_NAME "debug"
#define TAG "PERF_IOCTL"

struct _FPSGO_PACKAGE {
	__u32 tid;
	union {
		__u32 start;
		__u32 connectedAPI;
	};
	union {
		__u64 frame_time;
		__u64 bufID;
	};
	__u64 frame_id; /* for HWUI only*/
	__s32 queue_SF;
	__u64 identifier;
};

#define FPSGO_QUEUE         _IOW('g', 1, struct _FPSGO_PACKAGE)
#define FPSGO_DEQUEUE       _IOW('g', 3, struct _FPSGO_PACKAGE)
#define FPSGO_VSYNC         _IOW('g', 5, struct _FPSGO_PACKAGE)
#define FPSGO_TOUCH         _IOW('g', 10, struct _FPSGO_PACKAGE)
#define FPSGO_QUEUE_CONNECT _IOW('g', 15, struct _FPSGO_PACKAGE)
#define FPSGO_BQID          _IOW('g', 16, struct _FPSGO_PACKAGE)

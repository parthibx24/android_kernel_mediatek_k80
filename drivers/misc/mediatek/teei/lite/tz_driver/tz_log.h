/*
 * Copyright (C) 2016 Microtrust, Inc.
 * Copyright (C) 2015 Google, Inc.
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

#ifndef _TRUSTY_LOG_H_
#define _TRUSTY_LOG_H_

#define TZ_LOG_SIZE           (PAGE_SIZE * 64)
#define TZ_LINE_BUFFER_SIZE   256

#define TZ_LOG_RATELIMIT_INTERVAL	(1 * HZ)
#define TZ_LOG_RATELIMIT_BURST		200

#define TZ_EMPTY_PARENTHESES(z)     z
#define TZ_VOLATILE(x)              TZ_EMPTY_PARENTHESES(vola##tile x)
#define TZ_NON_VOLATILE(x)          x

#define SMC_SC_SHARED_LOG_VERSION       SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 0)
#define SMC_SC_SHARED_LOG_ADD           SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 1)
#define SMC_SC_SHARED_LOG_RM            SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 2)

#define TRUSTY_LOG_API_VERSION  1


/*
 * Ring buffer that supports one secure producer thread and one
 * linux side consumer thread.
 */
struct log_rb {
	TZ_VOLATILE(uint32_t alloc);
	TZ_VOLATILE(uint32_t put);
	TZ_NON_VOLATILE(uint32_t sz);
	TZ_VOLATILE(char data[0]);
} __packed;

struct boot_log_rb {
	uint32_t get;
	uint32_t put;
	uint32_t sz;
	char data[0];
} __packed;

struct trusty_state {
	struct mutex smc_lock;
	struct atomic_notifier_head notifier;
	struct completion cpu_idle_completion;
	char *version_str;
	u32 api_version;
	void *log_state;
};

struct tz_log_state {
	struct device *dev;

	/*
	 * This lock is here to ensure only one consumer will read
	 * from the log ring buffer at a time.
	 */
	spinlock_t lock;
	struct log_rb *log;
	struct boot_log_rb *boot_log;
	uint32_t get;

	struct page *log_pages;
	struct page *boot_log_pages;

	struct notifier_block call_notifier;
	struct notifier_block panic_notifier;
	char line_buffer[TZ_LINE_BUFFER_SIZE];
};

int tz_log_probe(struct device *dev, struct trusty_state *t_state);
int tz_log_remove(struct device *dev, struct trusty_state *t_state);

#endif


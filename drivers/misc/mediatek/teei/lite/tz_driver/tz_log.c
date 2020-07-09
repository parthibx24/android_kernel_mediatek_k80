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

#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/log2.h>
#include <asm/page.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <teei_trusty.h>
#include <imsg_log.h>
#include <teei_smcall.h>
#include "tz_log.h"

static int log_read_line(struct tz_log_state *s, int put, int get)
{
	struct log_rb *log = s->log;
	int i;
	char c = '\0';
	size_t max_to_read = min((size_t)(put - get),
				 sizeof(s->line_buffer) - 1);
	size_t mask = log->sz - 1;

	for (i = 0; i < max_to_read && c != '\n';)
		s->line_buffer[i++] = c = log->data[get++ & mask];
	s->line_buffer[i] = '\0';

	return i;
}

static void tz_driver_dump_logs(struct tz_log_state *s)
{
	struct log_rb *log = s->log;
	struct boot_log_rb *boot_log = s->boot_log;
	uint32_t get, put, alloc;
	int read_chars;
	static DEFINE_RATELIMIT_STATE(_rs,
				TZ_LOG_RATELIMIT_INTERVAL,
				TZ_LOG_RATELIMIT_BURST);

	if (unlikely(log->put == 0)) {
		IMSG_INFO("TEE log buffer not ready yet\n");
		return;
	}

	WARN_ON(!is_power_of_2(log->sz));

	/*
	 * For this ring buffer, at any given point, alloc >= put >= get.
	 * The producer side of the buffer is not locked, so the put and alloc
	 * pointers must be read in a defined order (put before alloc) so
	 * that the above condition is maintained. A read barrier is needed
	 * to make sure the hardware and compiler keep the reads ordered.
	 */
	get = s->get;
	while ((put = log->put) != get) {
		/* Make sure that the read of put occurs */
		/* before the read of log data */
		rmb();

		/* Read a line from the log */
		read_chars = log_read_line(s, put, get);

		/* Force the loads from log_read_line to complete. */
		rmb();
		alloc = log->alloc;

		/*
		 * Discard the line that was just read if the data could
		 * have been corrupted by the producer.
		 */
		if (alloc - get > log->sz) {
			IMSG_INFO("log overflow.\n");
			get = alloc - log->sz;
			continue;
		}

		/*
		 * Due to UART speed is slow, printing large number of
		 * messages to UART will cause system reset by watchdog.
		 * It can prevent from printing message to uart by using
		 * KERN_DEBUG log level if with default printk setting.
		 * (default setting is print message to uart
		 * if log level >= KERN_INFO)
		 */

		IMSG_PRINTK("[TZ_LOG] %s", s->line_buffer);

		/*
		 * Dump early log to boot log buffer
		 * until boot log buffer is full
		 */
		if (boot_log->put + read_chars < boot_log->sz) {
			char *log_buf = &boot_log->data[boot_log->put];

			memcpy(log_buf, s->line_buffer, read_chars);
			boot_log->put += read_chars;
		}

		/* Print warning message */
		/* if log output frequency is over rate limit */
		__ratelimit(&_rs);

		get += read_chars;

	}
	s->get = get;
}

static int tz_log_call_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct tz_log_state *s;
	unsigned long flags;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	s = container_of(nb, struct tz_log_state, call_notifier);
	spin_lock_irqsave(&s->lock, flags);
	tz_driver_dump_logs(s);
	spin_unlock_irqrestore(&s->lock, flags);

	return NOTIFY_OK;
}

static int tz_log_panic_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct tz_log_state *s;

	/*
	 * Don't grab the spin lock to hold up the panic notifier, even
	 * though this is racy.
	 */
	s = container_of(nb, struct tz_log_state, panic_notifier);
	IMSG_INFO("tz log panic notifier\n");
	tz_driver_dump_logs(s);
	return NOTIFY_OK;
}

int tz_log_probe(struct device *dev, struct trusty_state *t_state)
{
	struct tz_log_state *s;
	int result;
	phys_addr_t pa;

	IMSG_DEBUG("%s\n", __func__);

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		result = -ENOMEM;
		goto error_alloc_state;
	}

	spin_lock_init(&s->lock);
	s->log_pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | GFP_DMA,
				   get_order(TZ_LOG_SIZE));
	if (!s->log_pages) {
		result = -ENOMEM;
		goto error_alloc_log;
	}
	s->log = page_address(s->log_pages);

	pa = page_to_phys(s->log_pages);
	result = trusty_fast_call32(dev, SMC_FC_SEND_LOG_ADDR,
					(u32)(pa), (u32)((u64)pa >> 32),
					TZ_LOG_SIZE);


	s->boot_log_pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | GFP_DMA,
				   get_order(TZ_LOG_SIZE));
	if (!s->boot_log_pages) {
		result = -ENOMEM;
		goto error_alloc_boot_log;
	}
	s->boot_log = page_address(s->boot_log_pages);

	s->boot_log->put = 0;
	s->boot_log->sz = rounddown_pow_of_two(
				TZ_LOG_SIZE - sizeof(struct boot_log_rb));

	s->call_notifier.notifier_call = tz_log_call_notify;
	result = atomic_notifier_chain_register(&t_state->notifier,
							&s->call_notifier);
	if (result < 0) {
		IMSG_ERROR("failed to register tz driver call notifier\n");
		goto error_call_notifier;
	}

	s->panic_notifier.notifier_call = tz_log_panic_notify;
	result = atomic_notifier_chain_register(&panic_notifier_list,
						&s->panic_notifier);
	if (result < 0) {
		IMSG_ERROR("failed to register panic notifier\n");
		goto error_panic_notifier;
	}

	t_state->log_state = s;

	return 0;

error_panic_notifier:
	atomic_notifier_chain_unregister(&t_state->notifier, &s->call_notifier);
error_call_notifier:
	__free_pages(s->boot_log_pages, get_order(TZ_LOG_SIZE));
error_alloc_boot_log:
	__free_pages(s->log_pages, get_order(TZ_LOG_SIZE));
error_alloc_log:
	kfree(s);
error_alloc_state:
	return result;
}

int tz_log_remove(struct device *dev, struct trusty_state *t_state)
{
	struct tz_log_state *s = t_state->log_state;

	atomic_notifier_chain_unregister(&panic_notifier_list,
						&s->panic_notifier);
	atomic_notifier_chain_unregister(&t_state->notifier,
						&s->call_notifier);

	__free_pages(s->log_pages, get_order(TZ_LOG_SIZE));
	__free_pages(s->boot_log_pages, get_order(TZ_LOG_SIZE));
	kfree(s);
	t_state->log_state = NULL;

	return 0;
}


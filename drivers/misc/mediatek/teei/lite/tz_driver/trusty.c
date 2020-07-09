/*
 * Copyright (C) 2013 Google, Inc.
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

#include <linux/version.h>
#include <asm/compiler.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#ifdef CONFIG_MT_TRUSTY_DEBUGFS
#include <linux/random.h>
#endif
#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/trusty/sm_err.h>
#include <teei_trusty.h>
#include <teei_smcall.h>
#include <teei_shm.h>
#include <imsg_log.h>
#include "tz_log.h"
#include "teei_bootprof.h"

#ifdef CONFIG_ARM64
#define SMC_ARG0		"x0"
#define SMC_ARG1		"x1"
#define SMC_ARG2		"x2"
#define SMC_ARG3		"x3"
#define SMC_ARCH_EXTENSION	""
#define SMC_REGISTERS_TRASHED	"x4", "x5", "x6", "x7", "x8", "x9", \
			"x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17"
#else
#define SMC_ARG0		"r0"
#define SMC_ARG1		"r1"
#define SMC_ARG2		"r2"
#define SMC_ARG3		"r3"
#define SMC_ARCH_EXTENSION	".arch_extension sec\n"
#define SMC_REGISTERS_TRASHED	"ip"
#endif

#define TEEI_SHM_POOL

struct smc_call_entry {
	struct device *dev;
	ulong smcnr;
	ulong param0;
	ulong param1;
	ulong param2;
	ulong retVal;
};

struct ut_smc_call_work {
	struct kthread_work work;
	void *data;
};

static int current_cpu_id;
DEFINE_KTHREAD_WORKER(ut_fastcall_worker);
struct task_struct *teei_switch_task;

#if !defined(CONFIG_MACH_MT6580) && !defined(CONFIG_ARCH_MT6570)

static int teei_cpu_id[] = {0x0000, 0x0001, 0x0002, 0x0003, 0x0100, 0x0101,
			0x0102, 0x0103, 0x0200, 0x0201, 0x0202, 0x0203};

static int teei_cpu_callback(struct notifier_block *nfb,
					unsigned long action, void *hcpu);
static struct notifier_block teei_cpu_notifer = {
	.notifier_call = teei_cpu_callback,
};
#endif

static int get_current_cpuid(void)
{
	return current_cpu_id;
}

static inline ulong smc(ulong r0, ulong r1, ulong r2, ulong r3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(r0, r1, r2, r3, 0, 0, 0, 0, &res);

	return res.a0;
}


static void fast_smc_fn(struct kthread_work *work)
{
	struct ut_smc_call_work *switch_work = NULL;
	struct smc_call_entry *entry = NULL;
	ulong retVal = 0;

	switch_work = container_of(work, struct ut_smc_call_work, work);
	entry = (struct smc_call_entry *)switch_work->data;

	retVal = smc(entry->smcnr, entry->param0, entry->param1, entry->param2);

	entry->retVal = retVal;
}


static void handle_fast_smc(struct smc_call_entry *p_entry)
{
	struct ut_smc_call_work usc_work = {
		KTHREAD_WORK_INIT(usc_work.work, fast_smc_fn),
		.data = p_entry,
	};

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	if (!kthread_queue_work(&ut_fastcall_worker, &usc_work.work))
#else
	if (!queue_kthread_work(&ut_fastcall_worker, &usc_work.work))
#endif
		return;

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	kthread_flush_work(&usc_work.work);
#else
	flush_kthread_work(&usc_work.work);
#endif
}

static ulong fast_smc(ulong smcnr, ulong r0, ulong r1, ulong r2)
{
	struct smc_call_entry entry;

	entry.dev = NULL;
	entry.smcnr = smcnr;
	entry.param0 = r0;
	entry.param1 = r1;
	entry.param2 = r2;
	entry.retVal = 0;

	handle_fast_smc(&entry);

	return entry.retVal;
}

#ifdef CONFIG_TRUSTY_WDT_FIQ_ARMV7_SUPPORT
s32 trusty_fast_call32_nodev(u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	WARN_ON(!SMC_IS_FASTCALL(smcnr));
	WARN_ON(SMC_IS_SMC64(smcnr));

	return fast_smc(smcnr, a0, a1, a2);
}
#endif

s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(!s);
	WARN_ON(!SMC_IS_FASTCALL(smcnr));
	WARN_ON(SMC_IS_SMC64(smcnr));

	return fast_smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL(trusty_fast_call32);

#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(!s);
	WARN_ON(!SMC_IS_FASTCALL(smcnr));
	WARN_ON(!SMC_IS_SMC64(smcnr));

	return fast_smc(smcnr, a0, a1, a2);
}
#endif

static ulong __trusty_std_call_inner(struct device *dev, ulong smcnr,
				   ulong a0, ulong a1, ulong a2)
{
	ulong ret = 0;
	int retry = 5;

	dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx)\n",
		__func__, smcnr, a0, a1, a2);
	while (true) {
		ret = smc(smcnr, a0, a1, a2);
		while ((s32)ret == SM_ERR_FIQ_INTERRUPTED)
			ret = smc(SMC_SC_RESTART_FIQ, 0, 0, 0);
		if ((int)ret != SM_ERR_BUSY || !retry)
			break;

		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy, retry\n",
			__func__, smcnr, a0, a1, a2);
		retry--;
	}

	return ret;
}


static void std_smc_fn(struct kthread_work *work)
{
	struct ut_smc_call_work *switch_work = NULL;
	struct smc_call_entry *entry = NULL;
	ulong retVal = 0;

	switch_work = container_of(work, struct ut_smc_call_work, work);
	entry = (struct smc_call_entry *)switch_work->data;

	retVal = __trusty_std_call_inner(entry->dev, entry->smcnr,
				entry->param0, entry->param1, entry->param2);

	entry->retVal = retVal;
}


static void handle_std_smc(struct smc_call_entry *p_entry)
{
	struct ut_smc_call_work usc_work = {
		KTHREAD_WORK_INIT(usc_work.work, std_smc_fn),
		.data = p_entry,
	};

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	if (!kthread_queue_work(&ut_fastcall_worker, &usc_work.work))
#else
	if (!queue_kthread_work(&ut_fastcall_worker, &usc_work.work))
#endif
		return;

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	kthread_flush_work(&usc_work.work);
#else
	flush_kthread_work(&usc_work.work);
#endif
}

static ulong trusty_std_call_inner(struct device *dev, ulong smcnr,
				   ulong a0, ulong a1, ulong a2)
{
	struct smc_call_entry entry;

	entry.dev = dev;
	entry.smcnr = smcnr;
	entry.param0 = a0;
	entry.param1 = a1;
	entry.param2 = a2;
	entry.retVal = 0;

	handle_std_smc(&entry);

	return entry.retVal;
}


static ulong trusty_std_call_helper(struct device *dev, ulong smcnr,
				    ulong a0, ulong a1, ulong a2)
{
	ulong ret;
	int sleep_time = 1;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	while (true) {
		//local_irq_disable();
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_PREPARE,
					   NULL);
		ret = trusty_std_call_inner(dev, smcnr, a0, a1, a2);
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_RETURNED,
					   NULL);
		//local_irq_enable();

		if ((int)ret != SM_ERR_BUSY)
			break;

		if (sleep_time == 256)
			dev_warn(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy\n",
				 __func__, smcnr, a0, a1, a2);
		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy, wait %d ms\n",
			__func__, smcnr, a0, a1, a2, sleep_time);

		msleep(sleep_time);
		if (sleep_time < 1000)
			sleep_time <<= 1;

		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) retry\n",
			__func__, smcnr, a0, a1, a2);
	}

	if (sleep_time > 256)
		dev_warn(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) busy cleared\n",
			 __func__, smcnr, a0, a1, a2);

	return ret;
}

static void trusty_std_call_cpu_idle(struct trusty_state *s)
{
	int ret;
	unsigned long timeout = HZ * 10;

#ifdef CONFIG_TRUSTY_INTERRUPT_FIQ_ONLY
	timeout = HZ / 5; /* 200 ms */
#endif

	ret = wait_for_completion_timeout(&s->cpu_idle_completion, timeout);
	if (!ret) {
		pr_warn("%s: timed out waiting for cpu idle to clear, retry anyway\n",
			__func__);
	}
}

s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	int ret;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(SMC_IS_FASTCALL(smcnr));
	WARN_ON(SMC_IS_SMC64(smcnr));

	if (smcnr != SMC_SC_NOP) {
		mutex_lock(&s->smc_lock);
		reinit_completion(&s->cpu_idle_completion);
	}

	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) started\n",
		__func__, smcnr, a0, a1, a2);

	ret = trusty_std_call_helper(dev, smcnr, a0, a1, a2);
	while (ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {
		dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) interrupted\n",
			__func__, smcnr, a0, a1, a2);
		if (ret == SM_ERR_CPU_IDLE)
			trusty_std_call_cpu_idle(s);
		ret = trusty_std_call_helper(dev, SMC_SC_RESTART_LAST, 0, 0, 0);
	}
	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) returned 0x%x\n",
		__func__, smcnr, a0, a1, a2, ret);

	WARN_ONCE(ret == SM_ERR_PANIC, "trusty crashed");

	if (smcnr == SMC_SC_NOP)
		complete(&s->cpu_idle_completion);
	else
		mutex_unlock(&s->smc_lock);

	return ret;
}
EXPORT_SYMBOL(trusty_std_call32);

void trusty_send_start_time(struct device *dev)
{
	u64 V = 0;
	u32 H = 0, L = 0;
	struct timeval tv;

	do_gettimeofday(&tv);
	V = tv.tv_sec * 1000000ULL + tv.tv_usec;
	L = (u32)(V);
	H = (u32)(V >> 32);

	IMSG_DEBUG("start time = %llu\n", V);
	IMSG_DEBUG("H = %u L = %u\n", H, L);
	trusty_fast_call32(dev, SMC_FC_START_TIME, L, H, 0);
}



#if !defined(CONFIG_MACH_MT6580) && !defined(CONFIG_ARCH_MT6570)

#define TZ_PREFER_BIND_CORE (4)
static bool is_prefer_core(int cpu)
{
	/* bind to a specific core */
	if (cpu == TZ_PREFER_BIND_CORE)
		return true;

	return false;
}

static int find_prefer_core(int excluded_cpu)
{
	int i = 0;
	int prefer_core = -1;

	/* search for prefer cpu firstly */
	for_each_online_cpu(i) {
		if (i == excluded_cpu)
			continue;

		if (is_prefer_core(i)) {
			prefer_core = i;
			break;
		}
	}

	/* if prefer is found, return directly */
	if (prefer_core != -1)
		return prefer_core;

	/* if not found, then search for other online cpu */
	for_each_online_cpu(i) {
		if (i == excluded_cpu)
			continue;

		prefer_core = i;
		/* break when next active cpu has been selected */
		break;
	}

	return prefer_core;
}

static bool is_prefer_core_binded(void)
{
	unsigned int curr = get_current_cpuid();

	if (is_prefer_core(curr))
		return true;

	return false;
}

static bool is_prefer_core_onlined(void)
{
	int i = 0;

	for_each_online_cpu(i) {
		if (is_prefer_core(i))
			return true;
	}

	return false;
}


static ulong __handle_switch_core(unsigned long cpu)
{
	int switch_to_cpu_id = 0;
	ulong retVal = 0;

	switch_to_cpu_id = find_prefer_core(cpu);
	set_cpus_allowed_ptr(teei_switch_task, cpumask_of(switch_to_cpu_id));

	retVal = smc(SMC_SWITCH_CORE, teei_cpu_id[switch_to_cpu_id],
							teei_cpu_id[cpu], 0);

	current_cpu_id = switch_to_cpu_id;

	return 0;
}


static void switch_core_fn(struct kthread_work *work)
{
	struct ut_smc_call_work *switch_work = NULL;
	unsigned long cpu_id = 0;
	ulong retVal = 0;

	switch_work = container_of(work, struct ut_smc_call_work, work);
	cpu_id = (unsigned long)switch_work->data;

	retVal = __handle_switch_core(cpu_id);
}


static void handle_switch_core(unsigned long cpu)
{
	struct ut_smc_call_work usc_work = {
		KTHREAD_WORK_INIT(usc_work.work, switch_core_fn),
		.data = (void *)cpu,
	};

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	if (!kthread_queue_work(&ut_fastcall_worker, &usc_work.work))
#else
	if (!queue_kthread_work(&ut_fastcall_worker, &usc_work.work))
#endif
		return;

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	kthread_flush_work(&usc_work.work);
#else
	flush_kthread_work(&usc_work.work);
#endif
}

static int teei_cpu_callback(struct notifier_block *self,
				unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	unsigned int sched_cpu = get_current_cpuid();

	switch (action) {
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		if (cpu == sched_cpu) {
			IMSG_DEBUG("cpu down prepare for %d.\n", cpu);
			handle_switch_core((unsigned long)cpu);
		} else if (is_prefer_core(cpu))
			IMSG_DEBUG("cpu down prepare for prefer %d.\n", cpu);
		else if (!is_prefer_core_binded()
					&& is_prefer_core_onlined()) {
			IMSG_DEBUG("cpu down prepare for changing %d %d.\n",
					sched_cpu, cpu);
			handle_switch_core((unsigned long)sched_cpu);
		}
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}
#endif


int trusty_call_notifier_register(struct device *dev, struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_register(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_register);

int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_unregister(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_unregister);

static int trusty_remove_child(struct device *dev, void *data)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

ssize_t trusty_version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return scnprintf(buf, PAGE_SIZE, "%s\n", s->version_str);
}

DEVICE_ATTR(trusty_version, 0400, trusty_version_show, NULL);

#ifdef CONFIG_MT_TRUSTY_DEBUGFS
ssize_t trusty_add(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	s32 a, b, ret;

	get_random_bytes(&a, sizeof(s32));
	a &= 0xFF;
	get_random_bytes(&b, sizeof(s32));
	b &= 0xFF;
	ret = trusty_std_call32(dev, MT_SMC_SC_ADD, a, b, 0);
	return scnprintf(buf, PAGE_SIZE, "%d + %d = %d, %s\n", a, b, ret,
		(a + b) == ret ? "PASS" : "FAIL");
}

DEVICE_ATTR(trusty_add, 0400, trusty_add, NULL);

ssize_t trusty_threads(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Dump Trusty threads info to memlog */
	trusty_fast_call32(dev, MT_SMC_FC_THREADS, 0, 0, 0);
	/* Dump threads info from memlog to kmsg*/
	trusty_std_call32(dev, SMC_SC_NOP, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_threads, 0400, trusty_threads, NULL);

ssize_t trusty_threadstats(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Dump Trusty threads info to memlog */
	trusty_fast_call32(dev, MT_SMC_FC_THREADSTATS, 0, 0, 0);
	/* Dump threads info from memlog to kmsg*/
	trusty_std_call32(dev, SMC_SC_NOP, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_threadstats, 0400, trusty_threadstats, NULL);

ssize_t trusty_threadload(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Dump Trusty threads info to memlog */
	trusty_fast_call32(dev, MT_SMC_FC_THREADLOAD, 0, 0, 0);
	/* Dump threads info from memlog to kmsg*/
	trusty_std_call32(dev, SMC_SC_NOP, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_threadload, 0400, trusty_threadload, NULL);

ssize_t trusty_heap_dump(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Dump Trusty threads info to memlog */
	trusty_fast_call32(dev, MT_SMC_FC_HEAP_DUMP, 0, 0, 0);
	/* Dump threads info from memlog to kmsg*/
	trusty_std_call32(dev, SMC_SC_NOP, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_heap_dump, 0400, trusty_heap_dump, NULL);

ssize_t trusty_apps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Dump Trusty threads info to memlog */
	trusty_fast_call32(dev, MT_SMC_FC_APPS, 0, 0, 0);
	/* Dump threads info from memlog to kmsg*/
	trusty_std_call32(dev, SMC_SC_NOP, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_apps, 0400, trusty_apps, NULL);

ssize_t trusty_vdev_reset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	trusty_std_call32(dev, SMC_SC_VDEV_RESET, 0, 0, 0);
	return 0;
}

DEVICE_ATTR(trusty_vdev_reset, 0400, trusty_vdev_reset, NULL);


static void trusty_create_debugfs(struct trusty_state *s, struct device *pdev)
{
	int ret;

	ret = device_create_file(pdev, &dev_attr_trusty_add);
	if (ret)
		goto err_create_trusty_add;

	ret = device_create_file(pdev, &dev_attr_trusty_threads);
	if (ret)
		goto err_create_trusty_threads;

	ret = device_create_file(pdev, &dev_attr_trusty_threadstats);
	if (ret)
		goto err_create_trusty_threadstats;

	ret = device_create_file(pdev, &dev_attr_trusty_threadload);
	if (ret)
		goto err_create_trusty_threadload;

	ret = device_create_file(pdev, &dev_attr_trusty_heap_dump);
	if (ret)
		goto err_create_trusty_heap_dump;

	ret = device_create_file(pdev, &dev_attr_trusty_apps);
	if (ret)
		goto err_create_trusty_apps;

	ret = device_create_file(pdev, &dev_attr_trusty_vdev_reset);
	if (ret)
		goto err_create_trusty_vdev_reset;

	return;

err_create_trusty_vdev_reset:
	device_remove_file(pdev, &dev_attr_trusty_vdev_reset);
err_create_trusty_apps:
	device_remove_file(pdev, &dev_attr_trusty_apps);
err_create_trusty_heap_dump:
	device_remove_file(pdev, &dev_attr_trusty_heap_dump);
err_create_trusty_threadload:
	device_remove_file(pdev, &dev_attr_trusty_threadload);
err_create_trusty_threadstats:
	device_remove_file(pdev, &dev_attr_trusty_threadstats);
err_create_trusty_threads:
	device_remove_file(pdev, &dev_attr_trusty_threads);
err_create_trusty_add:
	device_remove_file(pdev, &dev_attr_trusty_add);
}

#endif /* CONFIG_MT_TRUSTY_DEBUGFS */

const char *trusty_version_str_get(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->version_str;
}
EXPORT_SYMBOL(trusty_version_str_get);

static void trusty_init_version(struct trusty_state *s, struct device *dev)
{
	int ret;
	int i;
	int version_str_len;

	ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, -1, 0, 0);
	if (ret <= 0)
		goto err_get_size;

	version_str_len = ret;

	s->version_str = kmalloc(version_str_len + 1, GFP_KERNEL);
	for (i = 0; i < version_str_len; i++) {
		ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, i, 0, 0);
		if (ret < 0)
			goto err_get_char;
		s->version_str[i] = ret;
	}
	s->version_str[i] = '\0';

	dev_info(dev, "trusty version: %s\n", s->version_str);

	ret = device_create_file(dev, &dev_attr_trusty_version);
	if (ret)
		goto err_create_file;

	return;

err_create_file:
err_get_char:
	kfree(s->version_str);
	s->version_str = NULL;
err_get_size:
	dev_err(dev, "failed to get version: %d\n", ret);
}

u32 trusty_get_api_version(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->api_version;
}
EXPORT_SYMBOL(trusty_get_api_version);

static int trusty_init_api_version(struct trusty_state *s, struct device *dev)
{
	u32 api_version;

	api_version = trusty_fast_call32(dev, SMC_FC_API_VERSION,
					 TRUSTY_API_VERSION_CURRENT, 0, 0);
	if (api_version == SM_ERR_UNDEFINED_SMC)
		api_version = 0;

	if (api_version > TRUSTY_API_VERSION_CURRENT) {
		dev_err(dev, "unsupported api version %u > %u\n",
			api_version, TRUSTY_API_VERSION_CURRENT);
		return -EINVAL;
	}

	dev_info(dev, "selected api version: %u (requested %u)\n",
		 api_version, TRUSTY_API_VERSION_CURRENT);
	s->api_version = api_version;

	return 0;
}

#define TEE_OK  1

static int trusty_probe(struct platform_device *pdev)
{
	ulong tee_status = 0;
	int i = 0;
	int ret;
	struct trusty_state *s;
	struct device_node *node = pdev->dev.of_node;
	struct cpumask mask = { CPU_BITS_NONE };

	TEEI_BOOT_FOOTPRINT("TEEI Choose the CPU");

	for_each_online_cpu(i) {
		current_cpu_id = i;
		IMSG_DEBUG("init stage : current_cpu_id = %d\n",
							current_cpu_id);
#if defined(CONFIG_MACH_MT6580) || defined(CONFIG_ARCH_MT6570)
		break;
#endif
	}

	TEEI_BOOT_FOOTPRINT("TEEI Create The Switch Thread");

	teei_switch_task = kthread_create(kthread_worker_fn,
				&ut_fastcall_worker, "teei_switch_thread");
	if (IS_ERR(teei_switch_task)) {
		IMSG_ERROR("Failed to create switch thread !\n");
		return -EINVAL;
	}

	TEEI_BOOT_FOOTPRINT("TEEI Wake UP The Switch Thread");

	wake_up_process(teei_switch_task);
	cpumask_set_cpu(get_current_cpuid(), &mask);
	set_cpus_allowed_ptr(teei_switch_task, &mask);

#if defined(CONFIG_MACH_MT6580) || defined(CONFIG_ARCH_MT6570)
	/* Core migration not supported */
#else
	TEEI_BOOT_FOOTPRINT("TEEI Register The CPU Notifier");
	register_cpu_notifier(&teei_cpu_notifer);
#endif

	if (!node) {
		dev_err(&pdev->dev, "of_node required\n");
		return -EINVAL;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto err_allocate_state;
	}
	mutex_init(&s->smc_lock);
	ATOMIC_INIT_NOTIFIER_HEAD(&s->notifier);
	init_completion(&s->cpu_idle_completion);
	platform_set_drvdata(pdev, s);

	TEEI_BOOT_FOOTPRINT("TEEI Start The Buffer LOG");
	ret = tz_log_probe(&pdev->dev, s);
	if (ret != 0)
		goto err_log_init;

	while (1) {
		tee_status = trusty_std_call32(&pdev->dev,
						SMC_GET_TEE_STATUS, 0, 0, 0);
		if (tee_status != TEE_OK)
			trusty_std_call32(&pdev->dev, SMC_NT_SCHED_T, 0, 0, 0);
		else
			break;
	}

	TEEI_BOOT_FOOTPRINT("TEEI Sync The REE Time");
	/* send start time */
	trusty_send_start_time(&pdev->dev);

	TEEI_BOOT_FOOTPRINT("TEEI Init Version");
	trusty_init_version(s, &pdev->dev);

	TEEI_BOOT_FOOTPRINT("TEEI Init API Version");
	ret = trusty_init_api_version(s, &pdev->dev);
	if (ret < 0)
		goto err_api_version;

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add children: %d\n", ret);
		goto err_add_children;
	}

#ifdef CONFIG_MT_TRUSTY_DEBUGFS
	trusty_create_debugfs(s, &pdev->dev);
#endif

	return 0;

err_add_children:
err_api_version:
	tz_log_remove(&pdev->dev, s);
err_log_init:
	if (s->version_str) {
		device_remove_file(&pdev->dev, &dev_attr_trusty_version);
		kfree(s->version_str);
	}
	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->smc_lock);
	kfree(s);
err_allocate_state:
	return ret;
}

static int trusty_remove(struct platform_device *pdev)
{
	struct trusty_state *s = platform_get_drvdata(pdev);

	TEEI_BOOT_FOOTPRINT("TEEI Release The Buffer LOG");
	tz_log_remove(&pdev->dev, s);

	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->smc_lock);
	if (s->version_str) {
		device_remove_file(&pdev->dev, &dev_attr_trusty_version);
		kfree(s->version_str);
	}
	kfree(s);
	return 0;
}

static const struct of_device_id trusty_of_match[] = {
	{ .compatible = "android,trusty-smc-v1", },
	{},
};

static struct platform_driver trusty_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver	= {
		.name = "trusty",
		.owner = THIS_MODULE,
		.of_match_table = trusty_of_match,
	},
};

static int __init trusty_driver_init(void)
{
	int retVal = 0;

	TEEI_BOOT_FOOTPRINT("TEEI Trusty Driver Init");

#ifdef TEEI_SHM_POOL
	TEEI_BOOT_FOOTPRINT("TEEI Init The Shared Memory Pool");
	teei_shm_init();
#endif

	retVal = platform_driver_register(&trusty_driver);

	return retVal;
}

static void __exit trusty_driver_exit(void)
{
#ifdef TEEI_SHM_POOL
	TEEI_BOOT_FOOTPRINT("TEEI Release The Shared Memory Pool");
	teei_shm_release();
#endif

	platform_driver_unregister(&trusty_driver);
}

subsys_initcall(trusty_driver_init);
module_exit(trusty_driver_exit);

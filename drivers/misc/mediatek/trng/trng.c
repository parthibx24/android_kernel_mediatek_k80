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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/of_address.h>

#include "sync_write.h"
#include "trng.h"

#define BIT_STREAM_LEN	500000

static void __iomem *trng_base;
static void __iomem *infracfg_base;

static uint32_t get_true_rnd(uint32_t *val)
{
	uint32_t ret = 0;
	uint32_t value = 0;

	if (val == NULL) {
		pr_err("NULL pointer of val\n");
		return -1;
	}

	/* ungate */
	mt_reg_sync_writel(TRNG_CG_PDN_VALUE, TRNG_CG_PDN_CLR);

	/* read random data once and drop it */
	((uint32_t *)val)[0] = readl(TRNG_DATA);

	/* start */
	value = readl(TRNG_CTRL);
	value |= TRNG_CTRL_START;
	mt_reg_sync_writel(value, TRNG_CTRL);

	/* get random data from trng */
	while (1) {
		value = readl(TRNG_CTRL);
		if (value & TRNG_CTRL_RDY)
			break;
	}

	((uint32_t *)val)[0] = readl(TRNG_DATA);

	/* stop */
	value = readl(TRNG_CTRL);
	value &= ~TRNG_CTRL_START;
	mt_reg_sync_writel(value, TRNG_CTRL);

	/* gate */
	mt_reg_sync_writel(TRNG_CG_PDN_VALUE, TRNG_CG_PDN_SET);

	return ret;
}

#ifdef TRNG_DUMP
static int mtk_trng_dump(struct seq_file *m, void *v)
{
	int i;
	int loop;
	size_t val[4] = {0};

	loop = BIT_STREAM_LEN / 128;

	if ((BIT_STREAM_LEN > 0) && ((BIT_STREAM_LEN % 128) != 0))
		loop += 1;

	pr_info("[TRNG] loop %d times!\n", loop);
	while (loop > 0) {
		pr_info("[TRNG] dump 4 sets of 32 bits rnd!\n");

		for (i = 0 ; i < 4 ; i++) {
			get_true_rnd((void *)(val + i));
			pr_info("val[%d] = 0x%08x\n", i, (uint32_t)val[i]);
			seq_printf(m, "0x%08x\n", (uint32_t)val[i]);
		}

		loop--;
	}
	return 0;
}

static int mtk_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_trng_dump, NULL);
}


static const struct file_operations trng_fops = {
	.owner = THIS_MODULE,
	.open = mtk_open_proc,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};
#endif

int add_crng_entropy(void *data)
{
	int index;
	uint32_t buf[4];
	(void) data;

	msleep(1000);
	while (sys_getrandom((char *)&index, sizeof(index), GRND_NONBLOCK)
	== -EAGAIN) {

		for (index = 0; index < 4; ++index)
			get_true_rnd((void *)(buf + index));

		/* Assume quality: 512/1024 = 0.5 */
		add_hwgenerator_randomness((const char *)buf, sizeof(buf),
			sizeof(buf) << 2);
		msleep(1000);
	}

	return 0;
}

static int trng_ioremap(void)
{
	struct device_node *node = NULL;

	/*IO remap*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,trng");

	if (node) {
		trng_base = of_iomap(node, 0);
		infracfg_base = of_iomap(node, 1);
		pr_info("[TRNG] BASE_ADDRESS=%p\n", trng_base);
		pr_info("[TRNG] INFRACFG_ADDRESS=%p\n", infracfg_base);
	} else {
		pr_warn("[TRNG] can't find trng compatible node\n");
		return -1;
	}

	return 0;
}


/*
 * trng_init: module init function.
 */
static int __init trng_init(void)
{
	struct task_struct *task;

	pr_info("[TRNG] module init\n");

	trng_ioremap();

	task = kthread_create(add_crng_entropy, NULL, "trng_entropy");
	if (!IS_ERR(task))
		wake_up_process(task);
	else
		pr_notice("[TRNG] Fail to create kthread trng_entropy\n");

#ifdef TRNG_DUMP
	parent = proc_mkdir(dirname, NULL);
	proc_create("rnd_dump", 0664, parent, &trng_fops);
#endif

	return 0;
}

/*
 * trng_exit: module exit function.
 */
static void __exit trng_exit(void)
{
	pr_info("[TRNG] module exit\n");
}

arch_initcall(trng_init);
module_exit(trng_exit);
MODULE_LICENSE("GPL");

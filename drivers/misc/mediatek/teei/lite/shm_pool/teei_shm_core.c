/*
 * Copyright (c) 2017, MicroTrust
 * Copyright (c) 2015, Linaro Limited
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
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <teei_shm.h>
#include <imsg_log.h>
#include "teei_bootprof.h"

#define SOTER_SHM_NUM_PRIV_PAGES	1
#define ROUND_UP(N, S) ((((N) + (S) - 1) / (S)) * (S))

struct teei_shm_pool *kern_shm_pool;

static struct teei_shm_pool *teei_create_shm_pool(int shared_mem_size)
{
	struct teei_shm_pool *pool;
	unsigned long vaddr;
	phys_addr_t paddr;
	size_t size;
	struct teei_shm_pool_mem_info shared_info;

	vaddr = (unsigned long)__get_free_pages(GFP_KERNEL | GFP_DMA,
			get_order(ROUND_UP(shared_mem_size, PAGE_SIZE)));
	if (vaddr == 0) {

		IMSG_ERROR("[%s][%d] Can NOT alloc the shared memory POOL!\n",
						__func__, __LINE__);
		return ERR_PTR(-ENOMEM);
	}

	IMSG_DEBUG("The virt address of shared memory pool is %lx.\n", vaddr);


	paddr = virt_to_phys((void *)vaddr);
	size = ROUND_UP(shared_mem_size, PAGE_SIZE);
	IMSG_DEBUG("reserved memory @ 0x%llx, size %zx\n",
					(unsigned long long)paddr, size);

	shared_info.vaddr = vaddr;
	shared_info.paddr = paddr;
	shared_info.size = size;

	pool = teei_shm_pool_alloc_res_mem(&shared_info);
	if (IS_ERR(pool)) {
		IMSG_ERROR("Failed to call the teei_shm_pool_alloc_res_mem.\n");
		free_pages(vaddr, get_order(ROUND_UP(
					shared_mem_size, PAGE_SIZE)));
	}

	return pool;
}

#define TEEI_SHM_POOL_SIZE (0x100000)

int teei_shm_init(void)
{
	TEEI_BOOT_FOOTPRINT("TEEI Shm Init Start");

	kern_shm_pool = teei_create_shm_pool(TEEI_SHM_POOL_SIZE);
	if (IS_ERR(kern_shm_pool))
		return PTR_ERR(kern_shm_pool);

	TEEI_BOOT_FOOTPRINT("TEEI Shm Init End");

	return 0;
}

void teei_shm_release(void)
{
	TEEI_BOOT_FOOTPRINT("TEEI Shm Release Start");

	teei_shm_pool_free(kern_shm_pool);

	TEEI_BOOT_FOOTPRINT("TEEI Shm Release End");
}

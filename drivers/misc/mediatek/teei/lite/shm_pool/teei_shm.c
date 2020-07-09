/*
 * Copyright (c) 2015-2016, Linaro Limited
 * Copyright (c) 2017-2018, MICROTRUST
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
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <teei_shm.h>

extern struct teei_shm_pool *kern_shm_pool;

/**
 * teei_shm_kmalloc() - Allocate shared memory
 * @size:	Requested size of shared memory
 * @flags:	Flags setting properties for the requested shared memory.
 */
struct teei_shm *teei_shm_kmalloc(size_t size, u32 flags)
{
	struct teei_shm_pool_mgr *poolm = NULL;
	struct teei_shm *shm;
	void *ret;
	int rc;

	if (!kern_shm_pool) {
		/* teedev has been detached from driver */
		return ERR_PTR(-EINVAL);
	}

	shm = kzalloc(sizeof(*shm), GFP_KERNEL);
	if (!shm)
		return ERR_PTR(-ENOMEM);

	shm->flags = flags;
	poolm = &kern_shm_pool->shared_buf_mgr;

	rc = poolm->ops->alloc(poolm, shm, size);
	if (rc) {
		ret = ERR_PTR(rc);
		goto err;
	}

	return shm;

err:
	kfree(shm);
	return ret;
}
EXPORT_SYMBOL_GPL(teei_shm_kmalloc);

/**
 * tee_shm_free() - Free shared memory
 * @shm:	Handle to shared memory to free
 */
void teei_shm_kfree(struct teei_shm *shm)
{
	struct teei_shm_pool_mgr *poolm = NULL;

	poolm = &kern_shm_pool->shared_buf_mgr;
	poolm->ops->free(poolm, shm);

	kfree(shm);
}
EXPORT_SYMBOL_GPL(teei_shm_kfree);

/**
 * teei_shm_get_pa() - Get physical address of a shared memory plus an offset
 * @shm:	Shared memory handle
 * @offs:	Offset from start of this shared memory
 * @pa:		Physical address to return
 * @returns 0 if offs is within the bounds of this shared memory, else an
 *	error code.
 */
int teei_shm_get_pa(struct teei_shm *shm, size_t offs, phys_addr_t *pa)
{
	if (offs >= shm->size)
		return -EINVAL;
	if (pa)
		*pa = shm->paddr + offs;

	return 0;
}
EXPORT_SYMBOL_GPL(teei_shm_get_pa);

int teei_shm_va2pa(struct teei_shm *shm, void *va, phys_addr_t *pa)
{
	/* Check that we're in the range of the shm */
	if ((char *)va < (char *)shm->kaddr)
		return -EINVAL;
	if ((char *)va >= ((char *)shm->kaddr + shm->size))
		return -EINVAL;

	return teei_shm_get_pa(shm,
		(unsigned long)va - (unsigned long)shm->kaddr, pa);
}
EXPORT_SYMBOL_GPL(teei_shm_va2pa);

/**
 * teei_shm_get_va() - Get virtual address of a shared memory plus an offset
 * @shm:	Shared memory handle
 * @offs:	Offset from start of this shared memory
 * @returns virtual address of the shared memory + offs if offs is within
 *	the bounds of this shared memory, else an ERR_PTR
 */
void *teei_shm_get_va(struct teei_shm *shm, size_t offs)
{
	if (offs >= shm->size)
		return ERR_PTR(-EINVAL);
	return (char *)shm->kaddr + offs;
}
EXPORT_SYMBOL_GPL(teei_shm_get_va);

/**
 * teei_shm_pa2va() - Get virtual address of a physical address
 * @shm:	Shared memory handle
 * @pa:		Physical address to tranlsate
 * @va:		Returned virtual address
 * @returns 0 on success and < 0 on failure
 */
int teei_shm_pa2va(struct teei_shm *shm, phys_addr_t pa, void **va)
{
	/* Check that we're in the range of the shm */
	if (pa < shm->paddr)
		return -EINVAL;
	if (pa >= (shm->paddr + shm->size))
		return -EINVAL;

	if (va) {
		void *v = teei_shm_get_va(shm, pa - shm->paddr);

		if (IS_ERR(v))
			return PTR_ERR(v);
		*va = v;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(teei_shm_pa2va);



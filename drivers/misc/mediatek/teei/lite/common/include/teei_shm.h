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
#ifndef TEEI_SHM_STRUCT_H
#define TEEI_SHM_STRUCT_H

/**
 * struct tee_shm - shared memory object
 * @paddr:	physical address of the shared memory
 * @kaddr:	virtual address of the shared memory
 * @size:	size of shared memory
 * @flags:	defined by TEE_SHM_* in tee_drv.h
 */
struct teei_shm {
	phys_addr_t paddr;
	void *kaddr;
	size_t size;
	u32 flags;
};

struct teei_shm_pool_mem_info {
	unsigned long vaddr;
	phys_addr_t paddr;
	size_t size;
};

struct teei_shm_pool_mgr;

/**
 * struct teei_shm_pool_mgr_ops - shared memory pool manager operations
 * @alloc:	called when allocating shared memory
 * @free:	called when freeing shared memory
 */
struct teei_shm_pool_mgr_ops {
	int (*alloc)(struct teei_shm_pool_mgr *poolmgr, struct teei_shm *shm,
		     size_t size);
	void (*free)(struct teei_shm_pool_mgr *poolmgr, struct teei_shm *shm);
};

/**
 * struct teei_shm_pool_mgr - shared memory manager
 * @ops:		operations
 * @private_data:	private data for the shared memory manager
 */
struct teei_shm_pool_mgr {
	const struct teei_shm_pool_mgr_ops *ops;
	void *private_data;
};

/**
 * struct teei_shm_pool - shared memory pool
 * @private_mgr:	pool manager for shared memory only between kernel
 *			and secure world
 * @destroy:		called when destroying the pool
 * @private_data:	private data for the pool
 */
struct teei_shm_pool {
	struct teei_shm_pool_mgr shared_buf_mgr;
	void (*destroy)(struct teei_shm_pool *pool);
	void *private_data;
};

int teei_shm_init(void);

void teei_shm_release(void);

struct teei_shm *teei_shm_kmalloc(size_t size, u32 flags);

void teei_shm_kfree(struct teei_shm *shm);


struct teei_shm_pool *
teei_shm_pool_alloc_res_mem(struct teei_shm_pool_mem_info *buff_info);

void teei_shm_pool_free(struct teei_shm_pool *pool);

#endif /*TEEI_SHM_STRUCT_H*/

// SPDX-License-Identifier: MIT
/*
 * Copyright 2019 Advanced Micro Devices, Inc.
 */

#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/dma-map-ops.h>
#include <linux/tee_drv.h>
#include <linux/psp.h>
#include "amdtee_private.h"

#if IS_BUILTIN(CONFIG_AMDTEE) && IS_ENABLED(CONFIG_DMA_CMA)
static void *alloc_from_cma(size_t size)
{

	int nr_pages = size >> PAGE_SHIFT;
	struct page *page;

	page = dma_alloc_from_contiguous(NULL, nr_pages, 0, false);
	if (page)
		return page_to_virt(page);

	return NULL;
}

static bool free_from_cma(struct tee_shm *shm)
{

	int nr_pages;
	struct page *page;

	if (!dev_get_cma_area(NULL))
		return false;

	nr_pages = shm->size >> PAGE_SHIFT;
	page = virt_to_page(shm->kaddr);
	return dma_release_from_contiguous(NULL, page, nr_pages);
}
#else
static void *alloc_from_cma(size_t size)
{
	return NULL;
}

static bool free_from_cma(struct tee_shm *shm)
{
	return false;
}
#endif

static int pool_op_alloc(struct tee_shm_pool *pool, struct tee_shm *shm,
			 size_t size, size_t align)
{
	void *va;
	int rc;

	size = PAGE_ALIGN(size);

	va = alloc_from_cma(size);

	if (!va)
		va = alloc_pages_exact(size, GFP_KERNEL | __GFP_ZERO);

	if (!va)
		return -ENOMEM;

	shm->kaddr = (void *)va;
	shm->paddr = __psp_pa((void *)va);
	shm->size = size;

	/* Map the allocated memory in to TEE */
	rc = amdtee_map_shmem(shm);
	if (rc) {
		free_pages_exact(va, size);
		shm->kaddr = NULL;
		return rc;
	}

	return 0;
}

static void pool_op_free(struct tee_shm_pool *pool, struct tee_shm *shm)
{
	/* Unmap the shared memory from TEE */
	amdtee_unmap_shmem(shm);

	if (!free_from_cma(shm))
		free_pages_exact(shm->kaddr, shm->size);

	shm->kaddr = NULL;
}

static void pool_op_destroy_pool(struct tee_shm_pool *pool)
{
	kfree(pool);
}

static const struct tee_shm_pool_ops pool_ops = {
	.alloc = pool_op_alloc,
	.free = pool_op_free,
	.destroy_pool = pool_op_destroy_pool,
};

struct tee_shm_pool *amdtee_config_shm(void)
{
	struct tee_shm_pool *pool = kzalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->ops = &pool_ops;

	return pool;
}

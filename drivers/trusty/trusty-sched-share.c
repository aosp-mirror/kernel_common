// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Google, Inc.
 *
 * This trusty-driver module contains the SMC API for the trusty-driver to
 * communicate with the trusty-kernel for shared memory
 * registration/unregistration.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/trusty/trusty.h>
#include "trusty-sched-share.h"

/**
 * struct trusty_sched_share_state - Trusty share resources state local to Trusty-Driver
 * @dev: ptr to the trusty-device instance
 * @sg: ptr to the scatter-gather list used for shared-memory buffers
 * @sched_shared_mem_id: trusty-priority shared-memory id
 * @sched_shared_vm: vm ptr to the shared-memory block
 * @mem_size: size of trusty shared-memory block in bytes
 * @buf_size: page-aligned size of trusty shared-memory buffer in bytes
 * @num_pages: number of pages containing the allocated shared-memory buffer
 */
struct trusty_sched_share_state {
	struct device *dev;
	struct scatterlist *sg;
	trusty_shared_mem_id_t sched_shared_mem_id;
	char *sched_shared_vm;
	u32 mem_size;
	u32 buf_size;
	u32 num_pages;
	bool is_registered;
	bool vm_is_shared;
};

static inline struct trusty_percpu_data *trusty_get_trusty_percpu_data(
		struct trusty_sched_shared *tsh, int cpu_num)
{
	return (struct trusty_percpu_data *)((unsigned char *)tsh +
			sizeof(struct trusty_sched_shared) +
			(cpu_num * sizeof(struct trusty_percpu_data)));
}

static void trusty_sched_share_reclaim_memory(
		struct trusty_sched_share_state *sched_share_state)
{
	int result;

	if (!sched_share_state->vm_is_shared) {
		dev_warn(sched_share_state->dev,
				"%s called unexpectedly when vm not shared\n", __func__);
		return;
	}

	result = trusty_reclaim_memory(sched_share_state->dev,
				       sched_share_state->sched_shared_mem_id,
				       sched_share_state->sg,
				       sched_share_state->num_pages);
	if (result != 0) {
		dev_err(sched_share_state->dev,
			"trusty_reclaim_memory() failed: ret=%d mem_id=0x%llx\n",
			result, sched_share_state->sched_shared_mem_id);
		/*
		 * It is not safe to free this memory if trusty_reclaim_memory()
		 * failed. Leak it in that case.
		 */
		dev_err(sched_share_state->dev,
			"WARNING: leaking some allocated resources!!\n");
	} else {
		sched_share_state->vm_is_shared = false;
	}
}

int trusty_alloc_sched_share(struct device *device,
		struct trusty_sched_share_state **state)
{
	struct trusty_sched_share_state *sched_share_state = NULL;
	struct trusty_sched_shared *shared;
	uint sched_share_state_size;
	unsigned int cpu;

	sched_share_state_size = sizeof(*sched_share_state);

	sched_share_state = kzalloc(sched_share_state_size, GFP_KERNEL);
	if (!sched_share_state)
		goto err_sched_state_alloc;
	sched_share_state->dev = device;
	sched_share_state->is_registered = false;
	sched_share_state->vm_is_shared = false;

	sched_share_state->mem_size = sizeof(struct trusty_sched_shared) +
				nr_cpu_ids * sizeof(struct trusty_percpu_data);
	sched_share_state->num_pages =
		round_up(sched_share_state->mem_size, PAGE_SIZE) / PAGE_SIZE;
	sched_share_state->buf_size = sched_share_state->num_pages * PAGE_SIZE;

	dev_dbg(sched_share_state->dev,
		"%s: mem_size=%d,  num_pages=%d,  buf_size=%d", __func__,
		sched_share_state->mem_size, sched_share_state->num_pages,
		sched_share_state->buf_size);

	sched_share_state->sched_shared_vm = vzalloc(sched_share_state->buf_size);
	if (!sched_share_state->sched_shared_vm)
		goto err_resources_alloc;
	dev_dbg(sched_share_state->dev, "%s: sched_shared_vm=%p  size=%d\n",
		__func__, sched_share_state->sched_shared_vm, sched_share_state->buf_size);

	shared = (struct trusty_sched_shared *)sched_share_state->sched_shared_vm;
	shared->cpu_count = nr_cpu_ids;
	shared->hdr_size = sizeof(struct trusty_sched_shared);
	shared->percpu_data_size = sizeof(struct trusty_percpu_data);

	for_each_possible_cpu(cpu) {
		trusty_get_trusty_percpu_data(shared, cpu)->ask_shadow_priority
				= TRUSTY_SHADOW_PRIORITY_NORMAL;
	}

	*state = sched_share_state;
	return 0;

err_resources_alloc:
	kfree(sched_share_state);
err_sched_state_alloc:
	return -ENOMEM;
}

void trusty_register_sched_share(struct device *device,
		struct trusty_sched_share_state *sched_share_state)
{
	int result = 0;
	struct scatterlist *sg;
	unsigned char *mem = sched_share_state->sched_shared_vm;
	trusty_shared_mem_id_t mem_id;
	int i;

	/* allocate and initialize scatterlist */
	sched_share_state->sg = kcalloc(sched_share_state->num_pages,
				  sizeof(*sched_share_state->sg), GFP_KERNEL);
	if (!sched_share_state->sg) {
		result = -ENOMEM;
		dev_err(sched_share_state->dev, "%s: failed to alloc sg\n", __func__);
		goto err_rsrc_alloc_sg;
	}

	sg_init_table(sched_share_state->sg, sched_share_state->num_pages);
	for_each_sg(sched_share_state->sg, sg, sched_share_state->num_pages, i) {
		struct page *pg = vmalloc_to_page(mem + (i * PAGE_SIZE));

		if (!pg) {
			result = -ENOMEM;
			dev_err(sched_share_state->dev, "%s: failed to map page i= %d\n",
					__func__, i);
			goto err_rsrc_sg_lookup;
		}
		sg_set_page(sg, pg, PAGE_SIZE, 0);
	}

	/* share memory with Trusty */
	result = trusty_share_memory(sched_share_state->dev, &mem_id, sched_share_state->sg,
				     sched_share_state->num_pages, PAGE_KERNEL);
	if (result != 0) {
		dev_err(sched_share_state->dev, "trusty_share_memory failed: %d\n",
			result);
		goto err_rsrc_share_mem;
	}
	dev_dbg(sched_share_state->dev, "%s: sched_shared_mem_id=0x%llx", __func__,
		mem_id);
	sched_share_state->sched_shared_mem_id = mem_id;
	sched_share_state->vm_is_shared = true;

	dev_dbg(device, "%s: calling api SMC_SC_SCHED_SHARE_REGISTER...\n",
		__func__);

	/* tell sched share code on Trusty side to share priorities */
	result = trusty_std_call32(
		sched_share_state->dev, SMC_SC_SCHED_SHARE_REGISTER,
		(u32)sched_share_state->sched_shared_mem_id,
		(u32)(sched_share_state->sched_shared_mem_id >> 32),
		sched_share_state->buf_size);
	if (result == SM_ERR_UNDEFINED_SMC) {
		dev_warn(
			sched_share_state->dev,
			"trusty-share not supported on secure side, error=%d\n",
			result);
		goto err_smc_std_call32;
	} else if (result < 0) {
		dev_err(device,
			"trusty std call32 (SMC_SC_SCHED_SHARE_REGISTER) failed: %d\n",
			result);
		goto err_smc_std_call32;
	}
	dev_dbg(device, "%s: sched_share_state=%llx\n", __func__,
		(u64)sched_share_state);

	sched_share_state->is_registered = true;

	return;

err_smc_std_call32:
	trusty_sched_share_reclaim_memory(sched_share_state);
err_rsrc_share_mem:
err_rsrc_sg_lookup:
	kfree(sched_share_state->sg);
	sched_share_state->sg = NULL;
err_rsrc_alloc_sg:
	return;

}

void trusty_unregister_sched_share(struct trusty_sched_share_state *sched_share_state)
{
	int result;

	if (!sched_share_state->is_registered)
		return;

	/* ask Trusty to release the Trusty-side resources */
	result = trusty_std_call32(
		sched_share_state->dev, SMC_SC_SCHED_SHARE_UNREGISTER,
		(u32)sched_share_state->sched_shared_mem_id,
		(u32)(sched_share_state->sched_shared_mem_id >> 32), 0);
	if (result) {
		dev_err(sched_share_state->dev,
			"call SMC_SC_SCHED_SHARE_UNREGISTER failed, error=%d\n",
			result);
	}


	trusty_sched_share_reclaim_memory(sched_share_state);

	kfree(sched_share_state->sg);
}

void trusty_free_sched_share(struct trusty_sched_share_state *sched_share_state)
{
	if (!sched_share_state->vm_is_shared)
		vfree(sched_share_state->sched_shared_vm);

	kfree(sched_share_state);
}

static inline int map_trusty_prio_to_linux_nice(int trusty_prio)
{
	int new_nice;

	switch (trusty_prio) {
	case TRUSTY_SHADOW_PRIORITY_HIGH:
		new_nice = LINUX_NICE_FOR_TRUSTY_PRIORITY_HIGH;
		break;
	case TRUSTY_SHADOW_PRIORITY_LOW:
		new_nice = LINUX_NICE_FOR_TRUSTY_PRIORITY_LOW;
		break;
	case TRUSTY_SHADOW_PRIORITY_NORMAL:
	default:
		new_nice = LINUX_NICE_FOR_TRUSTY_PRIORITY_NORMAL;
		break;
	}

	return new_nice;
}

int trusty_get_requested_nice(unsigned int cpu_num, struct trusty_sched_share_state *tcpu_state)
{
	struct trusty_sched_shared *tsh = (struct trusty_sched_shared *)tcpu_state->sched_shared_vm;

	return map_trusty_prio_to_linux_nice(
			trusty_get_trusty_percpu_data(tsh, cpu_num)->ask_shadow_priority);
}

void trusty_set_actual_nice(unsigned int cpu_num,
		struct trusty_sched_share_state *tcpu_state, int act_nice)
{
	struct trusty_sched_shared *tsh = (struct trusty_sched_shared *)tcpu_state->sched_shared_vm;
	int new_prio;

	if (act_nice >= map_trusty_prio_to_linux_nice(TRUSTY_SHADOW_PRIORITY_LOW))
		new_prio = TRUSTY_SHADOW_PRIORITY_LOW;
	else if (act_nice <= map_trusty_prio_to_linux_nice(TRUSTY_SHADOW_PRIORITY_HIGH))
		new_prio = TRUSTY_SHADOW_PRIORITY_HIGH;
	else
		new_prio = TRUSTY_SHADOW_PRIORITY_NORMAL;

	trusty_get_trusty_percpu_data(tsh, cpu_num)->cur_shadow_priority = new_prio;
}

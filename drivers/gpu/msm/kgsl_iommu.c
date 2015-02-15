/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/msm_kgsl.h>
#include <mach/socinfo.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/iommu_domains.h>
#include <stddef.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_mmu.h"
#include "kgsl_sharedmem.h"
#include "kgsl_iommu.h"
#include "adreno_pm4types.h"
#include "adreno.h"
#include "kgsl_trace.h"
#include "z180.h"
#include "kgsl_cffdump.h"


static struct kgsl_iommu_register_list kgsl_iommuv0_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			
	{ 0x0, 1 },			
	{ 0x10, 1 },			
	{ 0x14, 1 },			
	{ 0x20, 1 },			
	{ 0x800, 1 },			
	{ 0x820, 1 },			
	{ 0x03C, 1 },			
	{ 0x818, 1 },			
	{ 0x2C, 1 },			
	{ 0x30, 1 },			
	{ 0, 0 },			
	{ 0, 0 },			
	{ 0, 0 }			
};

static struct kgsl_iommu_register_list kgsl_iommuv1_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			
	{ 0x0, 1 },			
	{ 0x20, 1 },			
	{ 0x28, 1 },			
	{ 0x58, 1 },			
	{ 0x618, 1 },			
	{ 0x008, 1 },			
	{ 0, 0 },			
	{ 0, 0 },			
	{ 0x68, 1 },			
	{ 0x6C, 1 },			
	{ 0x7F0, 1 },			
	{ 0x7F4, 1 },			
	{ 0x2000, 0 }			
};

static struct iommu_access_ops *iommu_access_ops;

static int kgsl_iommu_default_setstate(struct kgsl_mmu *mmu,
		uint32_t flags);
static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu);

static void _iommu_lock(struct kgsl_iommu const *iommu)
{
	if (iommu_access_ops && iommu_access_ops->iommu_lock_acquire)
		iommu_access_ops->iommu_lock_acquire(
						iommu->sync_lock_initialized);
}

static void _iommu_unlock(struct kgsl_iommu const *iommu)
{
	if (iommu_access_ops && iommu_access_ops->iommu_lock_release)
		iommu_access_ops->iommu_lock_release(
						iommu->sync_lock_initialized);
}

struct remote_iommu_petersons_spinlock kgsl_iommu_sync_lock_vars;


static struct page *kgsl_guard_page;

static int get_iommu_unit(struct device *dev, struct kgsl_mmu **mmu_out,
			struct kgsl_iommu_unit **iommu_unit_out)
{
	int i, j, k;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		struct kgsl_mmu *mmu;
		struct kgsl_iommu *iommu;

		if (kgsl_driver.devp[i] == NULL)
			continue;

		mmu = kgsl_get_mmu(kgsl_driver.devp[i]);
		if (mmu == NULL || mmu->priv == NULL)
			continue;

		iommu = mmu->priv;

		for (j = 0; j < iommu->unit_count; j++) {
			struct kgsl_iommu_unit *iommu_unit =
				&iommu->iommu_units[j];
			for (k = 0; k < iommu_unit->dev_count; k++) {
				if (iommu_unit->dev[k].dev == dev) {
					*mmu_out = mmu;
					*iommu_unit_out = iommu_unit;
					return 0;
				}
			}
		}
	}

	return -EINVAL;
}

static struct kgsl_iommu_device *get_iommu_device(struct kgsl_iommu_unit *unit,
		struct device *dev)
{
	int k;

	for (k = 0; unit && k < unit->dev_count; k++) {
		if (unit->dev[k].dev == dev)
			return &(unit->dev[k]);
	}

	return NULL;
}



struct _mem_entry {
	unsigned int gpuaddr;
	unsigned int size;
	unsigned int flags;
	unsigned int priv;
	pid_t pid;
};


static void _prev_entry(struct kgsl_process_private *priv,
	unsigned int faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;

	for (node = rb_first(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

		if (entry->memdesc.gpuaddr > faultaddr)
			break;


		if (entry->memdesc.gpuaddr > ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pid = priv->pid;
		}

		node = rb_next(&entry->node);
	}
}


static void _next_entry(struct kgsl_process_private *priv,
	unsigned int faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;

	for (node = rb_last(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

		if (entry->memdesc.gpuaddr < faultaddr)
			break;


		if (entry->memdesc.gpuaddr < ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pid = priv->pid;
		}

		node = rb_prev(&entry->node);
	}
}

static void _find_mem_entries(struct kgsl_mmu *mmu, unsigned int faultaddr,
	unsigned int ptbase, struct _mem_entry *preventry,
	struct _mem_entry *nextentry)
{
	struct kgsl_process_private *private;
	int id = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);

	memset(preventry, 0, sizeof(*preventry));
	memset(nextentry, 0, sizeof(*nextentry));

	
	nextentry->gpuaddr = 0xFFFFFFFF;

	mutex_lock(&kgsl_driver.process_mutex);

	list_for_each_entry(private, &kgsl_driver.process_list, list) {

		if (private->pagetable && (private->pagetable->name != id))
			continue;

		spin_lock(&private->mem_lock);
		_prev_entry(private, faultaddr, preventry);
		_next_entry(private, faultaddr, nextentry);
		spin_unlock(&private->mem_lock);
	}

	mutex_unlock(&kgsl_driver.process_mutex);
}

static void _print_entry(struct kgsl_device *device, struct _mem_entry *entry)
{
	char name[32];
	memset(name, 0, sizeof(name));

	kgsl_get_memory_usage(name, sizeof(name) - 1, entry->flags);

	KGSL_LOG_DUMP(device,
		"[%8.8X - %8.8X] %s (pid = %d) (%s)\n",
		entry->gpuaddr,
		entry->gpuaddr + entry->size,
		entry->priv & KGSL_MEMDESC_GUARD_PAGE ? "(+guard)" : "",
		entry->pid, name);
}

static void _check_if_freed(struct kgsl_iommu_device *iommu_dev,
	unsigned long addr, unsigned int pid)
{
	unsigned long gpuaddr = addr;
	unsigned long size = 0;
	unsigned int flags = 0;

	char name[32];
	memset(name, 0, sizeof(name));

	if (kgsl_memfree_find_entry(pid, &gpuaddr, &size, &flags)) {
		kgsl_get_memory_usage(name, sizeof(name) - 1, flags);
		KGSL_LOG_DUMP(iommu_dev->kgsldev, "---- premature free ----\n");
		KGSL_LOG_DUMP(iommu_dev->kgsldev,
			"[%8.8lX-%8.8lX] (%s) was already freed by pid %d\n",
			gpuaddr, gpuaddr + size, name, pid);
	}
}

static int kgsl_iommu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long addr, int flags, void *token)
{
	int ret = 0;
	struct kgsl_mmu *mmu;
	struct kgsl_iommu *iommu;
	struct kgsl_iommu_unit *iommu_unit;
	struct kgsl_iommu_device *iommu_dev;
	unsigned int ptbase, fsr;
	unsigned int pid;
	struct _mem_entry prev, next;
	unsigned int fsynr0, fsynr1;
	int write;
	struct kgsl_device *device;
	struct adreno_device *adreno_dev;
	unsigned int no_page_fault_log = 0;
	unsigned int curr_context_id = 0;
	unsigned int curr_global_ts = 0;
	struct kgsl_context *context;

	ret = get_iommu_unit(dev, &mmu, &iommu_unit);
	if (ret)
		goto done;

	device = mmu->device;
	adreno_dev = ADRENO_DEVICE(device);
	if (atomic_read(&mmu->fault)) {
		if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)
			ret = -EBUSY;
		goto done;
	}

	iommu_dev = get_iommu_device(iommu_unit, dev);
	if (!iommu_dev) {
		KGSL_CORE_ERR("Invalid IOMMU device %p\n", dev);
		ret = -ENOSYS;
		goto done;
	}
	iommu = mmu->priv;

	kgsl_sharedmem_readl(&device->memstore, &curr_context_id,
		KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL, current_context));

	context = kgsl_context_get(device, curr_context_id);

	if (context != NULL) {
		kgsl_sharedmem_readl(&device->memstore, &curr_global_ts,
			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
			eoptimestamp));

		
		set_bit(KGSL_CONTEXT_PAGEFAULT, &context->priv);
		context->pagefault_ts = curr_global_ts;

		kgsl_context_put(context);
		context = NULL;
	}

	atomic_set(&mmu->fault, 1);
	iommu_dev->fault = 1;

	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_IOMMU_PAGE_FAULT);
		
		kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
		adreno_dispatcher_schedule(device);
	}

	ptbase = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
					iommu_dev->ctx_id, TTBR0);

	fsr = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSR);
	fsynr0 = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSYNR0);
	fsynr1 = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSYNR1);

	if (msm_soc_version_supports_iommu_v0())
		write = ((fsynr1 & (KGSL_IOMMU_FSYNR1_AWRITE_MASK <<
			KGSL_IOMMU_FSYNR1_AWRITE_SHIFT)) ? 1 : 0);
	else
		write = ((fsynr0 & (KGSL_IOMMU_V1_FSYNR0_WNR_MASK <<
			KGSL_IOMMU_V1_FSYNR0_WNR_SHIFT)) ? 1 : 0);

	pid = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);

	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_LOG_ONE_PER_PAGE)
		no_page_fault_log = kgsl_mmu_log_fault_addr(mmu, ptbase, addr);

	if (!no_page_fault_log) {
		KGSL_MEM_CRIT(iommu_dev->kgsldev,
			"GPU PAGE FAULT: addr = %lX pid = %d\n", addr, pid);
		KGSL_MEM_CRIT(iommu_dev->kgsldev,
		 "context = %d FSR = %X FSYNR0 = %X FSYNR1 = %X(%s fault)\n",
			iommu_dev->ctx_id, fsr, fsynr0, fsynr1,
			write ? "write" : "read");

		_check_if_freed(iommu_dev, addr, pid);

		KGSL_LOG_DUMP(iommu_dev->kgsldev, "---- nearby memory ----\n");

		_find_mem_entries(mmu, addr, ptbase, &prev, &next);

		if (prev.gpuaddr)
			_print_entry(iommu_dev->kgsldev, &prev);
		else
			KGSL_LOG_DUMP(iommu_dev->kgsldev, "*EMPTY*\n");

		KGSL_LOG_DUMP(iommu_dev->kgsldev, " <- fault @ %8.8lX\n", addr);

		if (next.gpuaddr != 0xFFFFFFFF)
			_print_entry(iommu_dev->kgsldev, &next);
		else
			KGSL_LOG_DUMP(iommu_dev->kgsldev, "*EMPTY*\n");

	}

	trace_kgsl_mmu_pagefault(iommu_dev->kgsldev, addr,
			kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase),
			write ? "write" : "read");

	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)
		ret = -EBUSY;
done:
	return ret;
}

static void kgsl_iommu_disable_clk(struct kgsl_mmu *mmu, int ctx_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	struct msm_iommu_drvdata *iommu_drvdata;
	int i, j;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (ctx_id != iommu_unit->dev[j].ctx_id)
				continue;
			atomic_dec(&iommu_unit->dev[j].clk_enable_count);
			BUG_ON(
			atomic_read(&iommu_unit->dev[j].clk_enable_count) < 0);
			iommu_drvdata = dev_get_drvdata(
					iommu_unit->dev[j].dev->parent);
			if (iommu_drvdata->aclk)
				clk_disable_unprepare(iommu_drvdata->aclk);
			if (iommu_drvdata->clk)
				clk_disable_unprepare(iommu_drvdata->clk);
			clk_disable_unprepare(iommu_drvdata->pclk);
		}
	}
}

static void kgsl_iommu_clk_disable_event(struct kgsl_device *device, void *data,
					unsigned int id, unsigned int ts,
					u32 type)
{
	struct kgsl_iommu_disable_clk_param *param = data;

	if ((0 <= timestamp_cmp(ts, param->ts)) ||
		(KGSL_EVENT_CANCELLED == type))
		kgsl_iommu_disable_clk(param->mmu, param->ctx_id);
	else
		
		BUG_ON(1);

	
	kfree(param);
}

static void
kgsl_iommu_disable_clk_on_ts(struct kgsl_mmu *mmu,
				unsigned int ts, int ctx_id)
{
	struct kgsl_iommu_disable_clk_param *param;

	param = kzalloc(sizeof(*param), GFP_KERNEL);
	if (!param) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n", sizeof(*param));
		return;
	}
	param->mmu = mmu;
	param->ctx_id = ctx_id;
	param->ts = ts;

	if (kgsl_add_event(mmu->device, KGSL_MEMSTORE_GLOBAL,
			ts, kgsl_iommu_clk_disable_event, param, mmu)) {
		KGSL_DRV_ERR(mmu->device,
			"Failed to add IOMMU disable clk event\n");
		kfree(param);
	}
}

static int kgsl_iommu_enable_clk(struct kgsl_mmu *mmu,
				int ctx_id)
{
	int ret = 0;
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
	struct msm_iommu_drvdata *iommu_drvdata;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (ctx_id != iommu_unit->dev[j].ctx_id)
				continue;
			iommu_drvdata =
			dev_get_drvdata(iommu_unit->dev[j].dev->parent);
			ret = clk_prepare_enable(iommu_drvdata->pclk);
			if (ret)
				goto done;
			if (iommu_drvdata->clk) {
				ret = clk_prepare_enable(iommu_drvdata->clk);
				if (ret) {
					clk_disable_unprepare(
						iommu_drvdata->pclk);
					goto done;
				}
			}
			if (iommu_drvdata->aclk) {
				ret = clk_prepare_enable(iommu_drvdata->aclk);
				if (ret) {
					if (iommu_drvdata->clk)
						clk_disable_unprepare(
							iommu_drvdata->clk);
					clk_disable_unprepare(
							iommu_drvdata->pclk);
					goto done;
				}
			}
			atomic_inc(&iommu_unit->dev[j].clk_enable_count);
		}
	}
done:
	if (ret) {
		struct kgsl_iommu_unit *iommu_unit;
		if (iommu->unit_count == i)
			i--;
		do {
			for (j--; j >= 0; j--)
				kgsl_iommu_disable_clk(mmu, ctx_id);
			i--;
			if (i >= 0) {
				iommu_unit = &iommu->iommu_units[i];
				j = iommu_unit->dev_count;
			}
		} while (i >= 0);
	}
	return ret;
}

static int kgsl_iommu_pt_equal(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt,
				phys_addr_t pt_base)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	phys_addr_t domain_ptbase;

	if (iommu_pt == NULL)
		return 0;

	domain_ptbase = iommu_get_pt_base_addr(iommu_pt->domain)
			& KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;

	pt_base &= KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;

	return (domain_ptbase == pt_base);

}

static void kgsl_iommu_destroy_pagetable(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	if (iommu_pt->domain)
		msm_unregister_domain(iommu_pt->domain);

	kfree(iommu_pt);
	iommu_pt = NULL;
}

void *kgsl_iommu_create_pagetable(void)
{
	int domain_num;
	struct kgsl_iommu_pt *iommu_pt;

	struct msm_iova_partition kgsl_partition = {
		.start = 0,
		.size = 0xFFFFFFFF,
	};
	struct msm_iova_layout kgsl_layout = {
		.partitions = &kgsl_partition,
		.npartitions = 1,
		.client_name = "kgsl",
		.domain_flags = 0,
	};

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (!iommu_pt) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n",
				sizeof(struct kgsl_iommu_pt));
		return NULL;
	}
	
	if (msm_soc_version_supports_iommu_v0())
		kgsl_layout.domain_flags = MSM_IOMMU_DOMAIN_PT_CACHEABLE;

	domain_num = msm_register_domain(&kgsl_layout);
	if (domain_num >= 0) {
		iommu_pt->domain = msm_get_iommu_domain(domain_num);

		if (iommu_pt->domain) {
			iommu_set_fault_handler(iommu_pt->domain,
				kgsl_iommu_fault_handler, NULL);

			return iommu_pt;
		}
	}

	KGSL_CORE_ERR("Failed to create iommu domain\n");
	kfree(iommu_pt);
	return NULL;
}

static void kgsl_detach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (mmu->priv_bank_table &&
				(KGSL_IOMMU_CONTEXT_PRIV == j))
				iommu_pt = mmu->priv_bank_table->priv;
			if (iommu_unit->dev[j].attached) {
				iommu_detach_device(iommu_pt->domain,
						iommu_unit->dev[j].dev);
				iommu_unit->dev[j].attached = false;
				KGSL_MEM_INFO(mmu->device, "iommu %p detached "
					"from user dev of MMU: %p\n",
					iommu_pt->domain, mmu);
			}
		}
	}
}

static int kgsl_attach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j, ret = 0;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (mmu->priv_bank_table &&
				(KGSL_IOMMU_CONTEXT_PRIV == j))
				iommu_pt = mmu->priv_bank_table->priv;
			if (!iommu_unit->dev[j].attached) {
				ret = iommu_attach_device(iommu_pt->domain,
							iommu_unit->dev[j].dev);
				if (ret) {
					KGSL_MEM_ERR(mmu->device,
						"Failed to attach device, err %d\n",
						ret);
					goto done;
				}
				iommu_unit->dev[j].attached = true;
				KGSL_MEM_INFO(mmu->device,
				"iommu pt %p attached to dev %p, ctx_id %d\n",
				iommu_pt->domain, iommu_unit->dev[j].dev,
				iommu_unit->dev[j].ctx_id);
			}
		}
	}
done:
	return ret;
}

static int _get_iommu_ctxs(struct kgsl_mmu *mmu,
	struct kgsl_device_iommu_data *data, unsigned int unit_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[unit_id];
	int i, j;
	int found_ctx;
	int ret = 0;

	for (j = 0; j < KGSL_IOMMU_MAX_DEVS_PER_UNIT; j++) {
		found_ctx = 0;
		for (i = 0; i < data->iommu_ctx_count; i++) {
			if (j == data->iommu_ctxs[i].ctx_id) {
				found_ctx = 1;
				break;
			}
		}
		if (!found_ctx)
			break;
		if (!data->iommu_ctxs[i].iommu_ctx_name) {
			KGSL_CORE_ERR("Context name invalid\n");
			ret = -EINVAL;
			goto done;
		}
		atomic_set(
		&(iommu_unit->dev[iommu_unit->dev_count].clk_enable_count),
		0);

		iommu_unit->dev[iommu_unit->dev_count].dev =
			msm_iommu_get_ctx(data->iommu_ctxs[i].iommu_ctx_name);
		if (NULL == iommu_unit->dev[iommu_unit->dev_count].dev)
			ret = -EINVAL;
		if (IS_ERR(iommu_unit->dev[iommu_unit->dev_count].dev)) {
			ret = PTR_ERR(
				iommu_unit->dev[iommu_unit->dev_count].dev);
			iommu_unit->dev[iommu_unit->dev_count].dev = NULL;
		}
		if (ret)
			goto done;
		iommu_unit->dev[iommu_unit->dev_count].ctx_id =
						data->iommu_ctxs[i].ctx_id;
		iommu_unit->dev[iommu_unit->dev_count].kgsldev = mmu->device;

		KGSL_DRV_INFO(mmu->device,
				"Obtained dev handle %p for iommu context %s\n",
				iommu_unit->dev[iommu_unit->dev_count].dev,
				data->iommu_ctxs[i].iommu_ctx_name);

		iommu_unit->dev_count++;
	}
done:
	if (!iommu_unit->dev_count && !ret)
		ret = -EINVAL;
	if (ret) {
		if (!msm_soc_version_supports_iommu_v0() &&
			iommu_unit->dev_count)
			ret = 0;
		else
			KGSL_CORE_ERR(
			"Failed to initialize iommu contexts, err: %d\n", ret);
	}

	return ret;
}

static int kgsl_iommu_start_sync_lock(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	uint32_t lock_gpu_addr = 0;

	if (KGSL_DEVICE_3D0 != mmu->device->id ||
		!msm_soc_version_supports_iommu_v0() ||
		!kgsl_mmu_is_perprocess(mmu) ||
		iommu->sync_lock_vars)
		return 0;

	if (!(mmu->flags & KGSL_MMU_FLAGS_IOMMU_SYNC)) {
		KGSL_DRV_ERR(mmu->device,
		"The GPU microcode does not support IOMMUv1 sync opcodes\n");
		return -ENXIO;
	}
	
	lock_gpu_addr = (iommu->sync_lock_desc.gpuaddr +
			iommu->sync_lock_offset);

	kgsl_iommu_sync_lock_vars.flag[PROC_APPS] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_APPS])));
	kgsl_iommu_sync_lock_vars.flag[PROC_GPU] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_GPU])));
	kgsl_iommu_sync_lock_vars.turn = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock, turn)));

	iommu->sync_lock_vars = &kgsl_iommu_sync_lock_vars;

	return 0;
}

#ifdef CONFIG_MSM_IOMMU_GPU_SYNC

static int kgsl_iommu_init_sync_lock(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int status = 0;
	uint32_t lock_phy_addr = 0;
	uint32_t page_offset = 0;

	if (!msm_soc_version_supports_iommu_v0() ||
		!kgsl_mmu_is_perprocess(mmu))
		return status;

	if (KGSL_DEVICE_2D0 == mmu->device->id ||
		KGSL_DEVICE_2D1 == mmu->device->id) {
		return status;
	}

	
	if (iommu->sync_lock_initialized)
		return status;

	iommu_access_ops = msm_get_iommu_access_ops();

	if (iommu_access_ops && iommu_access_ops->iommu_lock_initialize) {
		lock_phy_addr = (uint32_t)
				iommu_access_ops->iommu_lock_initialize();
		if (!lock_phy_addr) {
			iommu_access_ops = NULL;
			return status;
		}
		lock_phy_addr = lock_phy_addr - (uint32_t)MSM_SHARED_RAM_BASE +
				(uint32_t)msm_shared_ram_phys;
	}

	
	page_offset = (lock_phy_addr & (PAGE_SIZE - 1));
	lock_phy_addr = (lock_phy_addr & ~(PAGE_SIZE - 1));
	iommu->sync_lock_desc.physaddr = (unsigned int)lock_phy_addr;
	iommu->sync_lock_offset = page_offset;

	iommu->sync_lock_desc.size =
				PAGE_ALIGN(sizeof(kgsl_iommu_sync_lock_vars));
	status =  memdesc_sg_phys(&iommu->sync_lock_desc,
				 iommu->sync_lock_desc.physaddr,
				 iommu->sync_lock_desc.size);

	if (status) {
		iommu_access_ops = NULL;
		return status;
	}

	
	iommu->sync_lock_initialized = 1;

	return status;
}
#else
static int kgsl_iommu_init_sync_lock(struct kgsl_mmu *mmu)
{
	return 0;
}
#endif

inline unsigned int kgsl_iommu_sync_lock(struct kgsl_mmu *mmu,
						unsigned int *cmds)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
					iommu->sync_lock_vars;
	unsigned int *start = cmds;

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 1;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_TEST_TWO_MEMS, 3);
	*cmds++ = lock_vars->flag[PROC_APPS];
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

inline unsigned int kgsl_iommu_sync_unlock(struct kgsl_mmu *mmu,
					unsigned int *cmds)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
						iommu->sync_lock_vars;
	unsigned int *start = cmds;

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x0;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

static int kgsl_get_iommu_ctxt(struct kgsl_mmu *mmu)
{
	struct platform_device *pdev =
		container_of(mmu->device->parentdev, struct platform_device,
				dev);
	struct kgsl_device_platform_data *pdata_dev = pdev->dev.platform_data;
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	int i, ret = 0;

	
	if (KGSL_IOMMU_MAX_UNITS < pdata_dev->iommu_count) {
		KGSL_CORE_ERR("Too many IOMMU units defined\n");
		ret = -EINVAL;
		goto  done;
	}

	for (i = 0; i < pdata_dev->iommu_count; i++) {
		ret = _get_iommu_ctxs(mmu, &pdata_dev->iommu_data[i], i);
		if (ret)
			break;
	}
	iommu->unit_count = pdata_dev->iommu_count;
done:
	return ret;
}

static int kgsl_set_register_map(struct kgsl_mmu *mmu)
{
	struct platform_device *pdev =
		container_of(mmu->device->parentdev, struct platform_device,
				dev);
	struct kgsl_device_platform_data *pdata_dev = pdev->dev.platform_data;
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct kgsl_iommu_unit *iommu_unit;
	int i = 0, ret = 0;

	for (; i < pdata_dev->iommu_count; i++) {
		struct kgsl_device_iommu_data data = pdata_dev->iommu_data[i];
		iommu_unit = &iommu->iommu_units[i];
		
		if (!data.physstart || !data.physend) {
			KGSL_CORE_ERR("The register range for IOMMU unit not"
					" specified\n");
			ret = -EINVAL;
			goto err;
		}
		iommu_unit->reg_map.hostptr = ioremap(data.physstart,
					data.physend - data.physstart + 1);
		if (!iommu_unit->reg_map.hostptr) {
			KGSL_CORE_ERR("Failed to map SMMU register address "
				"space from %x to %x\n", data.physstart,
				data.physend - data.physstart + 1);
			ret = -ENOMEM;
			i--;
			goto err;
		}
		iommu_unit->reg_map.size = data.physend - data.physstart + 1;
		iommu_unit->reg_map.physaddr = data.physstart;
		ret = memdesc_sg_phys(&iommu_unit->reg_map, data.physstart,
				iommu_unit->reg_map.size);
		if (ret)
			goto err;

		if (!msm_soc_version_supports_iommu_v0())
			iommu_unit->iommu_halt_enable = 1;
		iommu_unit->ahb_base = data.physstart - mmu->device->reg_phys;
	}
	iommu->unit_count = pdata_dev->iommu_count;
	return ret;
err:
	
	for (; i >= 0; i--) {
		iommu_unit = &iommu->iommu_units[i];
		iounmap(iommu_unit->reg_map.hostptr);
		iommu_unit->reg_map.size = 0;
		iommu_unit->reg_map.physaddr = 0;
	}
	return ret;
}

static phys_addr_t kgsl_iommu_get_pt_base_addr(struct kgsl_mmu *mmu,
						struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	return iommu_get_pt_base_addr(iommu_pt->domain) &
			KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

static phys_addr_t kgsl_iommu_get_default_ttbr0(struct kgsl_mmu *mmu,
				unsigned int unit_id,
				enum kgsl_iommu_context_id ctx_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++)
			if (unit_id == i &&
				ctx_id == iommu_unit->dev[j].ctx_id)
				return iommu_unit->dev[j].default_ttbr0;
	}
	return 0;
}

static int kgsl_iommu_setstate(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pagetable,
				unsigned int context_id)
{
	int ret = 0;

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		if (mmu->hwpagetable != pagetable) {
			unsigned int flags = 0;
			mmu->hwpagetable = pagetable;
			flags |= kgsl_mmu_pt_get_flags(mmu->hwpagetable,
							mmu->device->id) |
							KGSL_MMUFLAGS_TLBFLUSH;
			ret = kgsl_setstate(mmu, context_id,
				KGSL_MMUFLAGS_PTUPDATE | flags);
		}
	}

	return ret;
}

static int kgsl_iommu_setup_regs(struct kgsl_mmu *mmu,
				    struct kgsl_pagetable *pt)
{
	int status;
	int i = 0;
	struct kgsl_iommu *iommu = mmu->priv;

	if (!msm_soc_version_supports_iommu_v0())
		return 0;

	for (i = 0; i < iommu->unit_count; i++) {
		status = kgsl_mmu_map_global(pt,
				&(iommu->iommu_units[i].reg_map));
		if (status)
			goto err;
	}

	
	if (iommu->sync_lock_initialized) {
		status = kgsl_mmu_map_global(pt, &iommu->sync_lock_desc);
		if (status)
			goto err;
	}

	return 0;
err:
	for (i--; i >= 0; i--)
		kgsl_mmu_unmap(pt,
				&(iommu->iommu_units[i].reg_map));

	return status;
}

static void kgsl_iommu_cleanup_regs(struct kgsl_mmu *mmu,
					struct kgsl_pagetable *pt)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i;
	for (i = 0; i < iommu->unit_count; i++)
		kgsl_mmu_unmap(pt, &(iommu->iommu_units[i].reg_map));

	if (iommu->sync_lock_desc.gpuaddr)
		kgsl_mmu_unmap(pt, &iommu->sync_lock_desc);
}


static unsigned int kgsl_iommu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
					int iommu_unit, int ctx_id,
					enum kgsl_iommu_reg_map reg)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (iommu->iommu_reg_list[reg].ctx_reg)
		return iommu->iommu_units[iommu_unit].ahb_base +
			iommu->iommu_reg_list[reg].reg_offset +
			(ctx_id << KGSL_IOMMU_CTX_SHIFT) + iommu->ctx_offset;
	else
		return iommu->iommu_units[iommu_unit].ahb_base +
			iommu->iommu_reg_list[reg].reg_offset;
}

static int kgsl_iommu_init(struct kgsl_mmu *mmu)
{
	int status = 0;
	struct kgsl_iommu *iommu;

	atomic_set(&mmu->fault, 0);
	iommu = kzalloc(sizeof(struct kgsl_iommu), GFP_KERNEL);
	if (!iommu) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n",
				sizeof(struct kgsl_iommu));
		return -ENOMEM;
	}

	mmu->priv = iommu;
	status = kgsl_get_iommu_ctxt(mmu);
	if (status)
		goto done;
	status = kgsl_set_register_map(mmu);
	if (status)
		goto done;

	mmu->pt_per_process = KGSL_MMU_USE_PER_PROCESS_PT &&
				(msm_soc_version_supports_iommu_v0() ||
				 iommu->iommu_units[0].iommu_halt_enable);

	if (mmu->pt_per_process) {
#ifndef CONFIG_MSM_KGSL_CFF_DUMP
		mmu->pt_base = PAGE_OFFSET;
		mmu->pt_size = KGSL_IOMMU_GLOBAL_MEM_BASE
				- kgsl_mmu_get_base_addr(mmu) - SZ_1M;
		mmu->use_cpu_map = true;
#else
		mmu->pt_base = KGSL_PAGETABLE_BASE;
		mmu->pt_size = KGSL_IOMMU_GLOBAL_MEM_BASE +
				KGSL_IOMMU_GLOBAL_MEM_SIZE -
				KGSL_PAGETABLE_BASE;
		mmu->use_cpu_map = false;
#endif
	} else {
		mmu->pt_base = KGSL_PAGETABLE_BASE;
#ifndef CONFIG_MSM_KGSL_CFF_DUMP
		mmu->pt_size = SZ_2G;
#else
		mmu->pt_size = KGSL_IOMMU_GLOBAL_MEM_BASE +
				KGSL_IOMMU_GLOBAL_MEM_SIZE -
				KGSL_PAGETABLE_BASE;
#endif
		mmu->use_cpu_map = false;
	}

	status = kgsl_iommu_init_sync_lock(mmu);
	if (status)
		goto done;

	iommu->iommu_reg_list = kgsl_iommuv0_reg;
	iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V0;

	if (msm_soc_version_supports_iommu_v0()) {
		iommu->iommu_reg_list = kgsl_iommuv0_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V0;
	} else {
		iommu->iommu_reg_list = kgsl_iommuv1_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V1;
	}

	kgsl_sharedmem_writel(mmu->device, &mmu->setstate_memory,
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_nop_packet(1));

	if (cpu_is_msm8960()) {
		iommu_ops.mmu_setup_pt = kgsl_iommu_setup_regs;
		iommu_ops.mmu_cleanup_pt = kgsl_iommu_cleanup_regs;
	}

	if (kgsl_guard_page == NULL) {
		kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
				__GFP_HIGHMEM);
		if (kgsl_guard_page == NULL) {
			status = -ENOMEM;
			goto done;
		}
	}

	dev_info(mmu->device->dev, "|%s| MMU type set for device is IOMMU\n",
			__func__);
done:
	if (status) {
		kfree(iommu);
		mmu->priv = NULL;
	}
	return status;
}

static int kgsl_iommu_setup_defaultpagetable(struct kgsl_mmu *mmu)
{
	int status = 0;

	if (msm_soc_version_supports_iommu_v0()) {
		mmu->priv_bank_table =
			kgsl_mmu_getpagetable(mmu,
					KGSL_MMU_PRIV_BANK_TABLE_NAME);
		if (mmu->priv_bank_table == NULL) {
			status = -ENOMEM;
			goto err;
		}
		status = kgsl_iommu_setup_regs(mmu, mmu->priv_bank_table);
		if (status)
			goto err;
	}
	mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu, KGSL_MMU_GLOBAL_PT);
	
	if (mmu->defaultpagetable == NULL) {
		status = -ENOMEM;
		goto err;
	}
	return status;
err:
	if (mmu->priv_bank_table) {
		kgsl_iommu_cleanup_regs(mmu, mmu->priv_bank_table);
		kgsl_mmu_putpagetable(mmu->priv_bank_table);
		mmu->priv_bank_table = NULL;
	}
	if (mmu->defaultpagetable) {
		kgsl_mmu_putpagetable(mmu->defaultpagetable);
		mmu->defaultpagetable = NULL;
	}
	return status;
}

static void kgsl_iommu_lock_rb_in_tlb(struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb;
	struct kgsl_iommu *iommu = mmu->priv;
	unsigned int num_tlb_entries;
	unsigned int tlblkcr = 0;
	unsigned int v2pxx = 0;
	unsigned int vaddr = 0;
	int i, j, k, l;

	if (!iommu->sync_lock_initialized)
		return;

	rb = &adreno_dev->ringbuffer;
	num_tlb_entries = rb->buffer_desc.size / PAGE_SIZE;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			tlblkcr = 0;
			if (cpu_is_msm8960())
				tlblkcr |= ((num_tlb_entries &
					KGSL_IOMMU_TLBLKCR_FLOOR_MASK) <<
					KGSL_IOMMU_TLBLKCR_FLOOR_SHIFT);
			else
				tlblkcr |= (((num_tlb_entries *
					iommu_unit->dev_count) &
					KGSL_IOMMU_TLBLKCR_FLOOR_MASK) <<
					KGSL_IOMMU_TLBLKCR_FLOOR_SHIFT);
			
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIALLCFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIALLCFG_SHIFT);
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIASIDCFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIASIDCFG_SHIFT);
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIVAACFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIVAACFG_SHIFT);
			
			tlblkcr |= ((1 & KGSL_IOMMU_TLBLKCR_LKE_MASK)
				<< KGSL_IOMMU_TLBLKCR_LKE_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
					iommu_unit->dev[j].ctx_id,
					TLBLKCR, tlblkcr);
		}
		for (j = 0; j < iommu_unit->dev_count; j++) {
			
			if (cpu_is_msm8960() &&  KGSL_IOMMU_CONTEXT_PRIV == j)
				continue;
			
			vaddr = rb->buffer_desc.gpuaddr;
			for (k = 0; k < num_tlb_entries; k++) {
				v2pxx = 0;
				v2pxx |= (((k + j * num_tlb_entries) &
					KGSL_IOMMU_V2PXX_INDEX_MASK)
					<< KGSL_IOMMU_V2PXX_INDEX_SHIFT);
				v2pxx |= vaddr & (KGSL_IOMMU_V2PXX_VA_MASK <<
						KGSL_IOMMU_V2PXX_VA_SHIFT);

				KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
						iommu_unit->dev[j].ctx_id,
						V2PUR, v2pxx);
				mb();
				vaddr += PAGE_SIZE;
				for (l = 0; l < iommu_unit->dev_count; l++) {
					tlblkcr = KGSL_IOMMU_GET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[l].ctx_id,
						TLBLKCR);
					mb();
					tlblkcr &=
					~(KGSL_IOMMU_TLBLKCR_VICTIM_MASK
					<< KGSL_IOMMU_TLBLKCR_VICTIM_SHIFT);
					tlblkcr |= (((k + 1 +
					(j * num_tlb_entries)) &
					KGSL_IOMMU_TLBLKCR_VICTIM_MASK) <<
					KGSL_IOMMU_TLBLKCR_VICTIM_SHIFT);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[l].ctx_id,
						TLBLKCR, tlblkcr);
				}
			}
		}
		for (j = 0; j < iommu_unit->dev_count; j++) {
			tlblkcr = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
						iommu_unit->dev[j].ctx_id,
						TLBLKCR);
			mb();
			
			tlblkcr &= ~(KGSL_IOMMU_TLBLKCR_LKE_MASK
				<< KGSL_IOMMU_TLBLKCR_LKE_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
				iommu_unit->dev[j].ctx_id, TLBLKCR, tlblkcr);
		}
	}
}

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
	int status;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
	int sctlr_val = 0;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);

	if (mmu->flags & KGSL_FLAGS_STARTED)
		return 0;

	if (mmu->defaultpagetable == NULL) {
		status = kgsl_iommu_setup_defaultpagetable(mmu);
		if (status)
			return -ENOMEM;
	}
	status = kgsl_iommu_start_sync_lock(mmu);
	if (status)
		return status;

	if (cpu_is_msm8960() && KGSL_DEVICE_3D0 == mmu->device->id) {
		struct kgsl_mh *mh = &(mmu->device->mh);
		BUG_ON(iommu->iommu_units[0].reg_map.gpuaddr != 0 &&
			mh->mpu_base > iommu->iommu_units[0].reg_map.gpuaddr);
		kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000001);

		kgsl_regwrite(mmu->device, MH_MMU_MPU_END,
			mh->mpu_base + mh->mpu_range);
	}

	mmu->hwpagetable = mmu->defaultpagetable;

	status = kgsl_attach_pagetable_iommu_domain(mmu);
	if (status) {
		mmu->hwpagetable = NULL;
		goto done;
	}
	status = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	if (status) {
		KGSL_CORE_ERR("clk enable failed\n");
		goto done;
	}
	status = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_PRIV);
	if (status) {
		kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
		KGSL_CORE_ERR("clk enable failed\n");
		goto done;
	}
	_iommu_lock(iommu);
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {

			if ((!msm_soc_version_supports_iommu_v0()) &&
				(!(adreno_dev->ft_pf_policy &
				   KGSL_FT_PAGEFAULT_GPUHALT_ENABLE))) {
				sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						SCTLR);
				sctlr_val |= (0x1 <<
						KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
				KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						SCTLR, sctlr_val);
			}
			if (sizeof(phys_addr_t) > sizeof(unsigned long)) {
				iommu_unit->dev[j].default_ttbr0 =
						KGSL_IOMMU_GET_CTX_REG_LL(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						TTBR0);
			} else {
				iommu_unit->dev[j].default_ttbr0 =
						KGSL_IOMMU_GET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						TTBR0);
			}
		}
	}
	kgsl_iommu_lock_rb_in_tlb(mmu);
	_iommu_unlock(iommu);

	
	kgsl_cffdump_setmem(mmu->device, mmu->setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_nop_packet(1), sizeof(unsigned int));

	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_CONTEXT_PRIV);
	mmu->flags |= KGSL_FLAGS_STARTED;

done:
	return status;
}

static void kgsl_iommu_flush_tlb_pt_current(struct kgsl_pagetable *pt)
{
	int lock_taken = 0;
	struct kgsl_device *device = pt->mmu->device;
	struct kgsl_iommu *iommu = pt->mmu->priv;

	if (!kgsl_mutex_lock(&device->mutex, &device->mutex_owner))
		lock_taken = 1;

	if (kgsl_mmu_is_perprocess(pt->mmu) &&
		iommu->iommu_units[0].dev[KGSL_IOMMU_CONTEXT_USER].attached &&
		kgsl_iommu_pt_equal(pt->mmu, pt,
		kgsl_iommu_get_current_ptbase(pt->mmu)))
		kgsl_iommu_default_setstate(pt->mmu, KGSL_MMUFLAGS_TLBFLUSH);

	if (lock_taken)
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}

static int
kgsl_iommu_unmap(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc,
		unsigned int *tlb_flags)
{
	int ret = 0;
	unsigned int range = memdesc->size;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;


	unsigned int gpuaddr = memdesc->gpuaddr &  KGSL_MMU_ALIGN_MASK;

	if (range == 0 || gpuaddr == 0)
		return 0;

	if (kgsl_memdesc_has_guard_page(memdesc))
		range += PAGE_SIZE;

	ret = iommu_unmap_range(iommu_pt->domain, gpuaddr, range);
	if (ret) {
		KGSL_CORE_ERR("iommu_unmap_range(%p, %x, %d) failed "
			"with err: %d\n", iommu_pt->domain, gpuaddr,
			range, ret);
		return ret;
	}

	kgsl_iommu_flush_tlb_pt_current(pt);

	return ret;
}

static int
kgsl_iommu_map(struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc,
			unsigned int protflags,
			unsigned int *tlb_flags)
{
	int ret;
	unsigned int iommu_virt_addr;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	int size = memdesc->size;

	BUG_ON(NULL == iommu_pt);

	iommu_virt_addr = memdesc->gpuaddr;

	ret = iommu_map_range(iommu_pt->domain, iommu_virt_addr, memdesc->sg,
				size, protflags);
	if (ret) {
		KGSL_CORE_ERR("iommu_map_range(%p, %x, %p, %d, %x) err: %d\n",
			iommu_pt->domain, iommu_virt_addr, memdesc->sg, size,
			protflags, ret);
		return ret;
	}
	if (kgsl_memdesc_has_guard_page(memdesc)) {
		ret = iommu_map(iommu_pt->domain, iommu_virt_addr + size,
				page_to_phys(kgsl_guard_page), PAGE_SIZE,
				protflags & ~IOMMU_WRITE);
		if (ret) {
			KGSL_CORE_ERR("iommu_map(%p, %x, guard, %x) err: %d\n",
				iommu_pt->domain, iommu_virt_addr + size,
				protflags & ~IOMMU_WRITE,
				ret);
			
			iommu_unmap_range(iommu_pt->domain, iommu_virt_addr,
					  size);
		}
	}

	if (!msm_soc_version_supports_iommu_v0())
		kgsl_iommu_flush_tlb_pt_current(pt);

	return ret;
}

void kgsl_iommu_pagefault_resume(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	if (atomic_read(&mmu->fault)) {
		for (i = 0; i < iommu->unit_count; i++) {
			struct kgsl_iommu_unit *iommu_unit =
						&iommu->iommu_units[i];
			for (j = 0; j < iommu_unit->dev_count; j++) {
				if (iommu_unit->dev[j].fault) {
					kgsl_iommu_enable_clk(mmu, j);
					_iommu_lock(iommu);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						RESUME, 1);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						FSR, 0);
					kgsl_iommu_disable_clk(mmu, j);
					_iommu_unlock(iommu);
					iommu_unit->dev[j].fault = 0;
				}
			}
		}
		atomic_set(&mmu->fault, 0);
	}
}


static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
{
	if (mmu->flags & KGSL_FLAGS_STARTED) {
		
		kgsl_detach_pagetable_iommu_domain(mmu);
		mmu->hwpagetable = NULL;

		mmu->flags &= ~KGSL_FLAGS_STARTED;

		kgsl_iommu_pagefault_resume(mmu);
	}
	
	kgsl_cancel_events(mmu->device, mmu);
}

static int kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i;

	if (mmu->priv_bank_table != NULL) {
		kgsl_iommu_cleanup_regs(mmu, mmu->priv_bank_table);
		kgsl_mmu_putpagetable(mmu->priv_bank_table);
	}

	if (mmu->defaultpagetable != NULL)
		kgsl_mmu_putpagetable(mmu->defaultpagetable);

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_memdesc *reg_map = &iommu->iommu_units[i].reg_map;

		if (reg_map->hostptr)
			iounmap(reg_map->hostptr);
		kgsl_sg_free(reg_map->sg, reg_map->sglen);
		reg_map->priv &= ~KGSL_MEMDESC_GLOBAL;
	}
	
	kgsl_sg_free(iommu->sync_lock_desc.sg, iommu->sync_lock_desc.sglen);
	memset(&iommu->sync_lock_desc, 0, sizeof(iommu->sync_lock_desc));
	iommu->sync_lock_vars = NULL;

	kfree(iommu);

	if (kgsl_guard_page != NULL) {
		__free_page(kgsl_guard_page);
		kgsl_guard_page = NULL;
	}

	return 0;
}

static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu)
{
	phys_addr_t pt_base;
	struct kgsl_iommu *iommu = mmu->priv;
	if (in_interrupt())
		return 0;
	
	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	pt_base = KGSL_IOMMU_GET_CTX_REG(iommu,
				(&iommu->iommu_units[0]),
				KGSL_IOMMU_CONTEXT_USER, TTBR0);
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	return pt_base & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

static int kgsl_iommu_default_setstate(struct kgsl_mmu *mmu,
					uint32_t flags)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int temp;
	int i;
	int ret = 0;
	phys_addr_t pt_base = kgsl_iommu_get_pt_base_addr(mmu,
						mmu->hwpagetable);
	phys_addr_t pt_val;

	ret = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	if (ret) {
		KGSL_DRV_ERR(mmu->device, "Failed to enable iommu clocks\n");
		return ret;
	}

	
	if (msm_soc_version_supports_iommu_v0()) {
		ret = kgsl_idle(mmu->device);
		if (ret)
			return ret;
	}

	
	_iommu_lock(iommu);

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		if (!msm_soc_version_supports_iommu_v0()) {
			ret = kgsl_idle(mmu->device);
			if (ret)
				goto unlock;
		}
		for (i = 0; i < iommu->unit_count; i++) {
			pt_val = kgsl_iommu_get_default_ttbr0(mmu, i,
						KGSL_IOMMU_CONTEXT_USER);

			pt_base &= KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			pt_val |= pt_base;
			if (sizeof(phys_addr_t) > sizeof(unsigned long)) {
				KGSL_IOMMU_SET_CTX_REG_LL(iommu,
					(&iommu->iommu_units[i]),
					KGSL_IOMMU_CONTEXT_USER, TTBR0, pt_val);
			} else {
				KGSL_IOMMU_SET_CTX_REG(iommu,
					(&iommu->iommu_units[i]),
					KGSL_IOMMU_CONTEXT_USER, TTBR0, pt_val);
			}

			mb();
			temp = KGSL_IOMMU_GET_CTX_REG(iommu,
				(&iommu->iommu_units[i]),
				KGSL_IOMMU_CONTEXT_USER, TTBR0);
		}
	}
	
	if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
		unsigned long wait_for_flush;
		for (i = 0; i < iommu->unit_count; i++) {
			KGSL_IOMMU_SET_CTX_REG(iommu, (&iommu->iommu_units[i]),
				KGSL_IOMMU_CONTEXT_USER, TLBIALL, 1);
			mb();
			if (!msm_soc_version_supports_iommu_v0()) {
				wait_for_flush = jiffies +
						msecs_to_jiffies(2000);
				KGSL_IOMMU_SET_CTX_REG(iommu,
					(&iommu->iommu_units[i]),
					KGSL_IOMMU_CONTEXT_USER, TLBSYNC, 0);
				while (KGSL_IOMMU_GET_CTX_REG(iommu,
					(&iommu->iommu_units[i]),
					KGSL_IOMMU_CONTEXT_USER, TLBSTATUS) &
					(KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE)) {
					if (time_after(jiffies,
						wait_for_flush)) {
						KGSL_DRV_ERR(mmu->device,
						"Wait limit reached for IOMMU tlb flush\n");
						break;
					}
					cpu_relax();
				}
			}
		}
	}
unlock:
	
	_iommu_unlock(iommu);

	
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	return ret;
}

static unsigned int kgsl_iommu_get_reg_gpuaddr(struct kgsl_mmu *mmu,
					int iommu_unit, int ctx_id, int reg)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (KGSL_IOMMU_GLOBAL_BASE == reg)
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr;

	if (iommu->iommu_reg_list[reg].ctx_reg)
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr +
			iommu->iommu_reg_list[reg].reg_offset +
			(ctx_id << KGSL_IOMMU_CTX_SHIFT) + iommu->ctx_offset;
	else
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr +
			iommu->iommu_reg_list[reg].reg_offset;
}
static int kgsl_iommu_hw_halt_supported(struct kgsl_mmu *mmu, int iommu_unit)
{
	struct kgsl_iommu *iommu = mmu->priv;
	return iommu->iommu_units[iommu_unit].iommu_halt_enable;
}

static int kgsl_iommu_get_num_iommu_units(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	return iommu->unit_count;
}

static int kgsl_iommu_set_pf_policy(struct kgsl_mmu *mmu,
				unsigned int pf_policy)
{
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);
	int ret = 0;
	unsigned int sctlr_val;

	if ((adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE) ==
		(pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE))
		return ret;
	if (msm_soc_version_supports_iommu_v0())
		return ret;

	ret = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);

	if (ret) {
		KGSL_DRV_ERR(mmu->device, "Failed to enable iommu clocks\n");
		return ret;
	}
	ret = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_PRIV);

	if (ret) {
		KGSL_DRV_ERR(mmu->device, "Failed to enable iommu clocks\n");
		kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
		return ret;
	}
	
	ret = mmu->device->ftbl->idle(mmu->device);
	if (ret) {
		kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
		return ret;
	}

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu,
					iommu_unit,
					iommu_unit->dev[j].ctx_id,
					SCTLR);
			if (pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)
				sctlr_val &= ~(0x1 <<
					KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
			else
				sctlr_val |= (0x1 <<
					KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu,
					iommu_unit,
					iommu_unit->dev[j].ctx_id,
					SCTLR, sctlr_val);
		}
	}
	kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
	return ret;
}

struct kgsl_mmu_ops iommu_ops = {
	.mmu_init = kgsl_iommu_init,
	.mmu_close = kgsl_iommu_close,
	.mmu_start = kgsl_iommu_start,
	.mmu_stop = kgsl_iommu_stop,
	.mmu_setstate = kgsl_iommu_setstate,
	.mmu_device_setstate = kgsl_iommu_default_setstate,
	.mmu_pagefault = NULL,
	.mmu_pagefault_resume = kgsl_iommu_pagefault_resume,
	.mmu_get_current_ptbase = kgsl_iommu_get_current_ptbase,
	.mmu_enable_clk = kgsl_iommu_enable_clk,
	.mmu_disable_clk = kgsl_iommu_disable_clk,
	.mmu_disable_clk_on_ts = kgsl_iommu_disable_clk_on_ts,
	.mmu_get_default_ttbr0 = kgsl_iommu_get_default_ttbr0,
	.mmu_get_reg_gpuaddr = kgsl_iommu_get_reg_gpuaddr,
	.mmu_get_reg_ahbaddr = kgsl_iommu_get_reg_ahbaddr,
	.mmu_get_num_iommu_units = kgsl_iommu_get_num_iommu_units,
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_get_pt_base_addr = kgsl_iommu_get_pt_base_addr,
	.mmu_hw_halt_supported = kgsl_iommu_hw_halt_supported,
	
	.mmu_setup_pt = NULL,
	.mmu_cleanup_pt = NULL,
	.mmu_sync_lock = kgsl_iommu_sync_lock,
	.mmu_sync_unlock = kgsl_iommu_sync_unlock,
	.mmu_set_pf_policy = kgsl_iommu_set_pf_policy,
};

struct kgsl_mmu_pt_ops iommu_pt_ops = {
	.mmu_map = kgsl_iommu_map,
	.mmu_unmap = kgsl_iommu_unmap,
	.mmu_create_pagetable = kgsl_iommu_create_pagetable,
	.mmu_destroy_pagetable = kgsl_iommu_destroy_pagetable,
};

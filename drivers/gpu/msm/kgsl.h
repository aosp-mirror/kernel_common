/* Copyright (c) 2008-2014, The Linux Foundation. All rights reserved.
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
#ifndef __KGSL_H
#define __KGSL_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>
#include <linux/mm.h>

#include <mach/kgsl.h>

#include "kgsl_htc.h"

#define KGSL_NAME "kgsl"

#define KGSL_MEMSTORE_SIZE	((int)(PAGE_SIZE * 2))
#define KGSL_MEMSTORE_GLOBAL	(0)
#define KGSL_MEMSTORE_MAX	(KGSL_MEMSTORE_SIZE / \
		sizeof(struct kgsl_devmemstore) - 1)

#define KGSL_TIMESTAMP_WINDOW 0x80000000

#define DRM_KGSL_GEM_CACHE_OP_TO_DEV	0x0001
#define DRM_KGSL_GEM_CACHE_OP_FROM_DEV	0x0002

#define KGSL_PAGETABLE_ENTRY_SIZE  4

#ifndef CONFIG_MSM_KGSL_CFF_DUMP
#define KGSL_PAGETABLE_BASE	0x10000000
#else
#define KGSL_PAGETABLE_BASE	0xE0000000
#endif

#define KGSL_PT_EXTRA_ENTRIES      16

#define KGSL_PAGETABLE_ENTRIES(_sz) (((_sz) >> PAGE_SHIFT) + \
				     KGSL_PT_EXTRA_ENTRIES)

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
#define KGSL_PAGETABLE_COUNT (CONFIG_MSM_KGSL_PAGE_TABLE_COUNT)
#else
#define KGSL_PAGETABLE_COUNT 1
#endif

#define KGSL_CONTAINER_OF(ptr, type, member) \
		container_of(ptr, type, member)


#define KGSL_STATS_ADD(_size, _stat, _max) \
	do { _stat += (_size); if (_stat > _max) _max = _stat; } while (0)

#define KGSL_MEMFREE_HIST_SIZE	((int)(PAGE_SIZE * 2))

enum {
	KGSL_MEM_ENTRY_KERNEL = 0,
	KGSL_MEM_ENTRY_PMEM,
	KGSL_MEM_ENTRY_ASHMEM,
	KGSL_MEM_ENTRY_USER,
	KGSL_MEM_ENTRY_ION,
	KGSL_MEM_ENTRY_PAGE_ALLOC,
	KGSL_MEM_ENTRY_PRE_ALLOC,
	KGSL_MEM_ENTRY_MAX,
};

#define KGSL_MAX_NUMIBS 100000

struct kgsl_device;
struct kgsl_context;

struct kgsl_driver {
	struct cdev cdev;
	dev_t major;
	struct class *class;
	
	struct device virtdev;
	
	struct kobject *ptkobj;
	struct kobject *prockobj;
	struct kgsl_device *devp[KGSL_DEVICE_MAX];

	
	struct list_head process_list;
	
	struct list_head pagetable_list;
	
	spinlock_t ptlock;
	
	struct mutex process_mutex;

	
	struct mutex devlock;

	void *ptpool;

	struct {
		unsigned int vmalloc;
		unsigned int vmalloc_max;
		unsigned int page_alloc;
		unsigned int page_alloc_max;
		unsigned int coherent;
		unsigned int coherent_max;
		unsigned int mapped;
		unsigned int mapped_max;
		unsigned int histogram[16];
	} stats;
	unsigned int full_cache_threshold;

	struct kgsl_driver_htc_priv priv;
};

extern struct kgsl_driver kgsl_driver;

struct kgsl_pagetable;
struct kgsl_memdesc;
struct kgsl_cmdbatch;

struct kgsl_memdesc_ops {
	int (*vmflags)(struct kgsl_memdesc *);
	int (*vmfault)(struct kgsl_memdesc *, struct vm_area_struct *,
		       struct vm_fault *);
	void (*free)(struct kgsl_memdesc *memdesc);
	int (*map_kernel)(struct kgsl_memdesc *);
	void (*unmap_kernel)(struct kgsl_memdesc *);
};

#define KGSL_MEMDESC_GUARD_PAGE BIT(0)
#define KGSL_MEMDESC_GLOBAL BIT(1)
#define KGSL_MEMDESC_FROZEN BIT(2)
#define KGSL_MEMDESC_MAPPED BIT(3)

struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void *hostptr; 
	unsigned int hostptr_count; 
	unsigned long useraddr; 
	unsigned int gpuaddr;
	phys_addr_t physaddr;
	unsigned int size;
	unsigned int priv; 
	struct scatterlist *sg;
	unsigned int sglen; 
	unsigned int sglen_alloc;  
	struct kgsl_memdesc_ops *ops;
	unsigned int flags; 
	struct kgsl_process_private *private;

	unsigned long sg_create;
	struct scatterlist *sg_backup;
};

struct kgsl_mem_entry {
	struct kref refcount;
	struct kgsl_memdesc memdesc;
	int memtype;
	void *priv_data;
	struct rb_node node;
	unsigned int id;
	unsigned int context_id;
	struct kgsl_process_private *priv;
	
	int pending_free;
	struct kgsl_device_private *dev_priv;
};

#ifdef CONFIG_MSM_KGSL_MMU_PAGE_FAULT
#define MMU_CONFIG 2
#else
#define MMU_CONFIG 1
#endif

void kgsl_mem_entry_destroy(struct kref *kref);
int kgsl_postmortem_dump(struct kgsl_device *device, int manual);

struct kgsl_mem_entry *kgsl_get_mem_entry(struct kgsl_device *device,
		phys_addr_t ptbase, unsigned int gpuaddr, unsigned int size);

struct kgsl_mem_entry *kgsl_sharedmem_find_region(
	struct kgsl_process_private *private, unsigned int gpuaddr,
	size_t size);

void kgsl_get_memory_usage(char *str, size_t len, unsigned int memflags);

void kgsl_signal_event(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp,
		unsigned int type);

void kgsl_signal_events(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int type);

void kgsl_cancel_events(struct kgsl_device *device,
	void *owner);

extern const struct dev_pm_ops kgsl_pm_ops;

int kgsl_suspend_driver(struct platform_device *pdev, pm_message_t state);
int kgsl_resume_driver(struct platform_device *pdev);

void kgsl_trace_regwrite(struct kgsl_device *device, unsigned int offset,
		unsigned int value);

void kgsl_trace_issueibcmds(struct kgsl_device *device, int id,
		struct kgsl_cmdbatch *cmdbatch,
		unsigned int timestamp, unsigned int flags,
		int result, unsigned int type);

int kgsl_open_device(struct kgsl_device *device);

int kgsl_close_device(struct kgsl_device *device);

#ifdef CONFIG_MSM_KGSL_DRM
extern int kgsl_drm_init(struct platform_device *dev);
extern void kgsl_drm_exit(void);
#else
static inline int kgsl_drm_init(struct platform_device *dev)
{
	return 0;
}

static inline void kgsl_drm_exit(void)
{
}
#endif

static inline int kgsl_gpuaddr_in_memdesc(const struct kgsl_memdesc *memdesc,
				unsigned int gpuaddr, unsigned int size)
{
	
	if (!size)
		size = 1;

	
	if (size > UINT_MAX - gpuaddr)
		return 0;

	if (gpuaddr >= memdesc->gpuaddr &&
	    ((gpuaddr + size) <= (memdesc->gpuaddr + memdesc->size))) {
		return 1;
	}
	return 0;
}

static inline void *kgsl_memdesc_map(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->map_kernel)
		memdesc->ops->map_kernel(memdesc);

	return memdesc->hostptr;
}

static inline void kgsl_memdesc_unmap(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->unmap_kernel)
		memdesc->ops->unmap_kernel(memdesc);
}
static inline uint8_t *kgsl_gpuaddr_to_vaddr(struct kgsl_memdesc *memdesc,
					     unsigned int gpuaddr)
{
	void *hostptr = NULL;

	if ((gpuaddr >= memdesc->gpuaddr) &&
		(gpuaddr < (memdesc->gpuaddr + memdesc->size)))
		hostptr = kgsl_memdesc_map(memdesc);

	return hostptr != NULL ? hostptr + (gpuaddr - memdesc->gpuaddr) : NULL;
}

static inline int timestamp_cmp(unsigned int a, unsigned int b)
{
	
	if (a == b)
		return 0;

	
	if ((a > b) && (a - b < KGSL_TIMESTAMP_WINDOW))
		return 1;

	a += KGSL_TIMESTAMP_WINDOW;
	b += KGSL_TIMESTAMP_WINDOW;
	return ((a > b) && (a - b <= KGSL_TIMESTAMP_WINDOW)) ? 1 : -1;
}

static inline int
kgsl_mem_entry_get(struct kgsl_mem_entry *entry)
{
	return kref_get_unless_zero(&entry->refcount);
}

static inline void
kgsl_mem_entry_put(struct kgsl_mem_entry *entry)
{
	kref_put(&entry->refcount, kgsl_mem_entry_destroy);
}

#endif 

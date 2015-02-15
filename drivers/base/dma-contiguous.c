/*
 * Contiguous Memory Allocator for DMA mapping framework
 * Copyright (c) 2010-2011 by Samsung Electronics.
 * Written by:
 *	Marek Szyprowski <m.szyprowski@samsung.com>
 *	Michal Nazarewicz <mina86@mina86.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License or (at your optional) any later version of the license.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 */

#define pr_fmt(fmt) "cma: " fmt

#ifdef CONFIG_CMA_DEBUG
#ifndef DEBUG
#  define DEBUG
#endif
#endif

#include <asm/page.h>
#include <asm/dma-contiguous.h>

#include <linux/memblock.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/page-isolation.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/mm_types.h>
#include <linux/dma-contiguous.h>
#include <trace/events/kmem.h>

#include <htc_debug/stability/htc_report_meminfo.h>

#ifndef SZ_1M
#define SZ_1M (1 << 20)
#endif

struct cma {
	unsigned long	base_pfn;
	unsigned long	count;
	unsigned long	*bitmap;
	bool in_system;
	struct mutex lock;
};

static DEFINE_MUTEX(cma_mutex);

struct cma *dma_contiguous_def_area;
phys_addr_t dma_contiguous_def_base;

static struct cma_area {
	phys_addr_t base;
	unsigned long size;
	struct cma *cma;
	const char *name;
	bool to_system;
} cma_areas[MAX_CMA_AREAS];
static unsigned cma_area_count;


static struct cma_map {
	phys_addr_t base;
	struct device *dev;
} cma_maps[MAX_CMA_AREAS] __initdata;
static unsigned cma_map_count __initdata;

static struct cma *cma_get_area(phys_addr_t base)
{
	int i;
	for (i = 0; i < cma_area_count; i++)
		if (cma_areas[i].base == base)
			return cma_areas[i].cma;
	return NULL;
}

static struct cma *cma_get_area_by_name(const char *name)
{
	int i;
	if (!name)
		return NULL;

	for (i = 0; i < cma_area_count; i++)
		if (cma_areas[i].name && strcmp(cma_areas[i].name, name) == 0)
			return cma_areas[i].cma;
	return NULL;
}



#ifdef CONFIG_CMA_SIZE_MBYTES
#define CMA_SIZE_MBYTES CONFIG_CMA_SIZE_MBYTES
#else
#define CMA_SIZE_MBYTES 0
#endif

static const phys_addr_t size_bytes = CMA_SIZE_MBYTES * SZ_1M;
static phys_addr_t size_cmdline = -1;

static int __init early_cma(char *p)
{
	pr_debug("%s(%s)\n", __func__, p);
	size_cmdline = memparse(p, &p);
	return 0;
}
early_param("cma", early_cma);

#ifdef CONFIG_CMA_SIZE_PERCENTAGE

static phys_addr_t __init __maybe_unused cma_early_percent_memory(void)
{
	struct memblock_region *reg;
	unsigned long total_pages = 0;

	for_each_memblock(memory, reg)
		total_pages += memblock_region_memory_end_pfn(reg) -
			       memblock_region_memory_base_pfn(reg);

	return (total_pages * CONFIG_CMA_SIZE_PERCENTAGE / 100) << PAGE_SHIFT;
}

#else

static inline __maybe_unused phys_addr_t cma_early_percent_memory(void)
{
	return 0;
}

#endif

static __init int cma_activate_area(unsigned long base_pfn, unsigned long count)
{
	unsigned long pfn = base_pfn;
	unsigned i = count >> pageblock_order;
	struct zone *zone;

	WARN_ON_ONCE(!pfn_valid(pfn));
	zone = page_zone(pfn_to_page(pfn));

	do {
		unsigned j;
		base_pfn = pfn;
		for (j = pageblock_nr_pages; j; --j, pfn++) {
			WARN_ON_ONCE(!pfn_valid(pfn));
			if (page_zone(pfn_to_page(pfn)) != zone)
				return -EINVAL;
		}
		init_cma_reserved_pageblock(pfn_to_page(base_pfn));
	} while (--i);
	return 0;
}

static __init struct cma *cma_create_area(unsigned long base_pfn,
				     unsigned long count, bool system)
{
	int bitmap_size = BITS_TO_LONGS(count) * sizeof(long);
	struct cma *cma;
	int ret = -ENOMEM;

	pr_debug("%s(base %08lx, count %lx)\n", __func__, base_pfn, count);

	cma = kmalloc(sizeof *cma, GFP_KERNEL);
	if (!cma)
		return ERR_PTR(-ENOMEM);

	cma->base_pfn = base_pfn;
	cma->count = count;
	cma->in_system = system;
	cma->bitmap = kzalloc(bitmap_size, GFP_KERNEL);

	if (!cma->bitmap)
		goto no_mem;

	if (cma->in_system) {
		ret = cma_activate_area(base_pfn, count);
		if (ret)
			goto error;
	}
	mutex_init(&cma->lock);

	pr_debug("%s: returned %p\n", __func__, (void *)cma);
	return cma;

error:
	kfree(cma->bitmap);
no_mem:
	kfree(cma);
	return ERR_PTR(ret);
}


#ifdef CONFIG_OF
int __init cma_fdt_scan(unsigned long node, const char *uname,
				int depth, void *data)
{
	phys_addr_t base, size;
	unsigned long len;
	__be32 *prop;
	char *name;
	bool in_system;
	phys_addr_t limit = MEMBLOCK_ALLOC_ANYWHERE;

	if (!of_get_flat_dt_prop(node, "linux,contiguous-region", NULL))
		return 0;

	prop = of_get_flat_dt_prop(node, "reg", &len);
	if (!prop || (len != 2 * sizeof(unsigned long)))
		return 0;

	base = be32_to_cpu(prop[0]);
	size = be32_to_cpu(prop[1]);

	name = of_get_flat_dt_prop(node, "label", NULL);
	in_system =
		of_get_flat_dt_prop(node, "linux,reserve-region", NULL) ? 0 : 1;

	prop = of_get_flat_dt_prop(node, "linux,memory-limit", NULL);
	if (prop)
		limit = be32_to_cpu(prop[0]);

	pr_info("Found %s, memory base %lx, size %ld MiB, limit %pa\n", uname,
		(unsigned long)base, (unsigned long)size / SZ_1M, &limit);
	dma_contiguous_reserve_area(size, &base, limit, name,
					in_system);

	return 0;
}
#endif

void __init dma_contiguous_reserve(phys_addr_t limit)
{
	phys_addr_t sel_size = 0;

	pr_debug("%s(limit %08lx)\n", __func__, (unsigned long)limit);

	if (size_cmdline != -1) {
		sel_size = size_cmdline;
	} else {
#ifdef CONFIG_CMA_SIZE_SEL_MBYTES
		sel_size = size_bytes;
#elif defined(CONFIG_CMA_SIZE_SEL_PERCENTAGE)
		sel_size = cma_early_percent_memory();
#elif defined(CONFIG_CMA_SIZE_SEL_MIN)
		sel_size = min(size_bytes, cma_early_percent_memory());
#elif defined(CONFIG_CMA_SIZE_SEL_MAX)
		sel_size = max(size_bytes, cma_early_percent_memory());
#endif
	}

	if (sel_size) {
		phys_addr_t base = 0;
		pr_debug("%s: reserving %ld MiB for global area\n", __func__,
			 (unsigned long)sel_size / SZ_1M);

		if (dma_contiguous_reserve_area(sel_size, &base, limit, NULL,
		    true) == 0)
			dma_contiguous_def_base = base;
	}
#ifdef CONFIG_OF
	of_scan_flat_dt(cma_fdt_scan, NULL);
#endif
};

int __init dma_contiguous_reserve_area(phys_addr_t size, phys_addr_t *res_base,
				       phys_addr_t limit, const char *name,
				       bool to_system)
{
	phys_addr_t base = *res_base;
	phys_addr_t alignment;
	int ret = 0;

	pr_debug("%s(size %lx, base %08lx, limit %08lx)\n", __func__,
		 (unsigned long)size, (unsigned long)base,
		 (unsigned long)limit);

	
	if (cma_area_count == ARRAY_SIZE(cma_areas)) {
		pr_err("Not enough slots for CMA reserved regions!\n");
		return -ENOSPC;
	}

	if (!size)
		return -EINVAL;

	
	alignment = PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	base = ALIGN(base, alignment);
	size = ALIGN(size, alignment);
	limit &= ~(alignment - 1);

	
	if (base) {
		if (memblock_is_region_reserved(base, size) ||
		    memblock_reserve(base, size) < 0) {
			ret = -EBUSY;
			goto err;
		}
	} else {
		phys_addr_t addr = __memblock_alloc_base(size, alignment, limit);
		if (!addr) {
			ret = -ENOMEM;
			goto err;
		} else {
			base = addr;
		}
	}

	cma_areas[cma_area_count].base = base;
	cma_areas[cma_area_count].size = size;
	cma_areas[cma_area_count].name = name;
	cma_areas[cma_area_count].to_system = to_system;
	cma_area_count++;
	*res_base = base;

	pr_info("CMA: reserved %ld MiB at %08lx\n", (unsigned long)size / SZ_1M,
		(unsigned long)base);

	
	dma_contiguous_early_fixup(base, size);
	return 0;
err:
	pr_err("CMA: failed to reserve %ld MiB\n", (unsigned long)size / SZ_1M);
	return ret;
}

int __init dma_contiguous_add_device(struct device *dev, phys_addr_t base)
{
	if (cma_map_count == ARRAY_SIZE(cma_maps)) {
		pr_err("Not enough slots for CMA reserved regions!\n");
		return -ENOSPC;
	}
	cma_maps[cma_map_count].dev = dev;
	cma_maps[cma_map_count].base = base;
	cma_map_count++;
	return 0;
}

#ifdef CONFIG_OF
static void cma_assign_device_from_dt(struct device *dev)
{
	struct device_node *node;
	struct cma *cma;
	const char *name;
	u32 value;

	node = of_parse_phandle(dev->of_node, "linux,contiguous-region", 0);
	if (!node)
		return;
	if (of_property_read_u32(node, "reg", &value) && !value)
		return;

	if (of_property_read_string(node, "label", &name))
		return;

	cma = cma_get_area_by_name(name);
	if (!cma)
		return;

	dev_set_cma_area(dev, cma);
	pr_info("Assigned CMA region at %lx to %s device\n", (unsigned long)value, dev_name(dev));
}

static int cma_device_init_notifier_call(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct device *dev = data;
	if (event == BUS_NOTIFY_ADD_DEVICE && dev->of_node)
		cma_assign_device_from_dt(dev);
	return NOTIFY_DONE;
}

static struct notifier_block cma_dev_init_nb = {
	.notifier_call = cma_device_init_notifier_call,
};
#endif

static int __init cma_init_reserved_areas(void)
{
	struct cma *cma;
	int i;

	for (i = 0; i < cma_area_count; i++) {
		phys_addr_t base = PFN_DOWN(cma_areas[i].base);
		unsigned int count = cma_areas[i].size >> PAGE_SHIFT;
		bool system = cma_areas[i].to_system;

		cma = cma_create_area(base, count, system);
		if (!IS_ERR(cma))
			cma_areas[i].cma = cma;
	}

	dma_contiguous_def_area = cma_get_area(dma_contiguous_def_base);

	for (i = 0; i < cma_map_count; i++) {
		cma = cma_get_area(cma_maps[i].base);
		dev_set_cma_area(cma_maps[i].dev, cma);
	}

#ifdef CONFIG_OF
	bus_register_notifier(&platform_bus_type, &cma_dev_init_nb);
#endif
	return 0;
}
core_initcall(cma_init_reserved_areas);

phys_addr_t cma_get_base(struct device *dev)
{
	struct cma *cma = dev_get_cma_area(dev);

	return cma->base_pfn << PAGE_SHIFT;
}

bool cma_area_exist(struct device *dev)
{
	struct cma *cma = dev_get_cma_area(dev);

	return (cma != dma_contiguous_def_area);
}

static void clear_cma_bitmap(struct cma *cma, unsigned long pfn, int count)
{
	mutex_lock(&cma->lock);
	bitmap_clear(cma->bitmap, pfn - cma->base_pfn, count);
	mutex_unlock(&cma->lock);
}

struct page *dma_alloc_from_contiguous(struct device *dev, int count,
				       unsigned int align)
{
	unsigned long mask, pfn, pageno, start = 0;
	struct cma *cma = dev_get_cma_area(dev);
	struct page *page = NULL;
	int ret = 0;
	int tries = 0;

	if (!cma || !cma->count)
		return NULL;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	pr_debug("%s(cma %p, count %d, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
		return NULL;

	mask = (1 << align) - 1;


	for (;;) {
		mutex_lock(&cma->lock);
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    start, count, mask);
		if (pageno >= cma->count) {
			mutex_unlock(&cma->lock);
			break;
		}
		bitmap_set(cma->bitmap, pageno, count);
		mutex_unlock(&cma->lock);

		pfn = cma->base_pfn + pageno;
		mutex_lock(&cma_mutex);
		if (cma->in_system)
			ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA);
		mutex_unlock(&cma_mutex);
		if (ret == 0) {
			page = pfn_to_page(pfn);
			add_meminfo_total_pages(NR_DMA_PAGES, count);
			break;
		} else if (ret != -EBUSY) {
			pfn = 0;
			clear_cma_bitmap(cma, pfn, count);
			break;
		}
		clear_cma_bitmap(cma, pfn, count);
		tries++;
		trace_dma_alloc_contiguous_retry(tries);

		pr_debug("%s(): memory range at %p is busy, retrying\n",
			 __func__, pfn_to_page(pfn));
		
		start = pageno + mask + 1;
	}

	pr_debug("%s(): returned %p\n", __func__, page);
	return page;
}

bool dma_release_from_contiguous(struct device *dev, struct page *pages,
				 int count)
{
	struct cma *cma = dev_get_cma_area(dev);
	unsigned long pfn;

	if (!cma || !pages)
		return false;

	pr_debug("%s(page %p)\n", __func__, (void *)pages);

	pfn = page_to_pfn(pages);

	if (pfn < cma->base_pfn || pfn >= cma->base_pfn + cma->count)
		return false;

	VM_BUG_ON(pfn + count > cma->base_pfn + cma->count);

	sub_meminfo_total_pages(NR_DMA_PAGES, count);
	if (cma->in_system)
		free_contig_range(pfn, count);
	clear_cma_bitmap(cma, pfn, count);

	return true;
}

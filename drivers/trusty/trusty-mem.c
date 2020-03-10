/*
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/types.h>
#include <linux/printk.h>
#include <linux/trusty/trusty.h>
#include <linux/trusty/smcall.h>

#define MEM_ATTR_STRONGLY_ORDERED (0x00U)
#define MEM_ATTR_DEVICE (0x04U)
#define MEM_ATTR_NORMAL_NON_CACHEABLE (0x44U)
#define MEM_ATTR_NORMAL_WRITE_THROUGH (0xAAU)
#define MEM_ATTR_NORMAL_WRITE_BACK_READ_ALLOCATE (0xEEU)
#define MEM_ATTR_NORMAL_WRITE_BACK_WRITE_ALLOCATE (0xFFU)

#define ATTR_RDONLY (1U << 7)
#define ATTR_INNER_SHAREABLE (3U << 8)

static int get_mem_attr(struct page *page, pgprot_t pgprot)
{
#if defined(CONFIG_ARM64)
	uint64_t mair;
	uint attr_index = (pgprot_val(pgprot) & PTE_ATTRINDX_MASK) >> 2;

	asm ("mrs %0, mair_el1\n" : "=&r" (mair));
	return (mair >> (attr_index * 8)) & 0xff;

#elif defined(CONFIG_ARM_LPAE)
	uint32_t mair;
	uint attr_index = ((pgprot_val(pgprot) & L_PTE_MT_MASK) >> 2);

	if (attr_index >= 4) {
		attr_index -= 4;
		asm volatile("mrc p15, 0, %0, c10, c2, 1\n" : "=&r" (mair));
	} else {
		asm volatile("mrc p15, 0, %0, c10, c2, 0\n" : "=&r" (mair));
	}
	return (mair >> (attr_index * 8)) & 0xff;

#elif defined(CONFIG_ARM)
	/* check memory type */
	switch (pgprot_val(pgprot) & L_PTE_MT_MASK) {
	case L_PTE_MT_WRITEALLOC:
		return MEM_ATTR_NORMAL_WRITE_BACK_WRITE_ALLOCATE;

	case L_PTE_MT_BUFFERABLE:
		return MEM_ATTR_NORMAL_NON_CACHEABLE;

	case L_PTE_MT_WRITEBACK:
		return MEM_ATTR_NORMAL_WRITE_BACK_READ_ALLOCATE;

	case L_PTE_MT_WRITETHROUGH:
		return MEM_ATTR_NORMAL_WRITE_THROUGH;

	case L_PTE_MT_UNCACHED:
		return MEM_ATTR_STRONGLY_ORDERED;

	case L_PTE_MT_DEV_SHARED:
	case L_PTE_MT_DEV_NONSHARED:
		return MEM_ATTR_DEVICE;

	default:
		return -EINVAL;
	}
#else
	return 0;
#endif
}

int trusty_encode_page_info(struct ns_mem_page_info *inf,
			    struct page *page, pgprot_t pgprot)
{
	int mem_attr;
	uint64_t pte;

	if (!inf || !page)
		return -EINVAL;

	/* get physical address */
	pte = (uint64_t) page_to_phys(page);

	/* get memory attributes */
	mem_attr = get_mem_attr(page, pgprot);
	if (mem_attr < 0)
		return mem_attr;

	/* add other attributes */
#if defined(CONFIG_ARM64) || defined(CONFIG_ARM_LPAE)
	pte |= pgprot_val(pgprot);
#elif defined(CONFIG_ARM)
	if (pgprot_val(pgprot) & L_PTE_RDONLY)
		pte |= ATTR_RDONLY;
	if (pgprot_val(pgprot) & L_PTE_SHARED)
		pte |= ATTR_INNER_SHAREABLE; /* inner sharable */
#endif

	inf->attr = (pte & 0x0000FFFFFFFFFFFFull) | ((uint64_t)mem_attr << 48);
	return 0;
}

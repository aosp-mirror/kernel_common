// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Google, Inc.
 */

#include <linux/types.h>
#include <linux/printk.h>
#include <linux/trusty/arm_ffa.h>
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
	u64 mair;
	unsigned int attr_index = (pgprot_val(pgprot) & PTE_ATTRINDX_MASK) >> 2;

	asm ("mrs %0, mair_el1\n" : "=&r" (mair));
	return (mair >> (attr_index * 8)) & 0xff;

#elif defined(CONFIG_ARM_LPAE)
	u32 mair;
	unsigned int attr_index = ((pgprot_val(pgprot) & L_PTE_MT_MASK) >> 2);

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
	u64 pte;
	u8 ffa_mem_attr;
	u8 ffa_mem_perm = 0;

	if (!inf || !page)
		return -EINVAL;

	/* get physical address */
	pte = (u64)page_to_phys(page);

	/* get memory attributes */
	mem_attr = get_mem_attr(page, pgprot);
	if (mem_attr < 0)
		return mem_attr;

	switch (mem_attr) {
	case MEM_ATTR_STRONGLY_ORDERED:
		ffa_mem_attr = FFA_MEM_ATTR_DEVICE_NGNRNE;
		break;

	case MEM_ATTR_DEVICE:
		ffa_mem_attr = FFA_MEM_ATTR_DEVICE_NGNRE;
		break;

	case MEM_ATTR_NORMAL_NON_CACHEABLE:
		ffa_mem_attr = FFA_MEM_ATTR_NORMAL_MEMORY_UNCACHED;
		break;

	case MEM_ATTR_NORMAL_WRITE_BACK_READ_ALLOCATE:
	case MEM_ATTR_NORMAL_WRITE_BACK_WRITE_ALLOCATE:
		ffa_mem_attr = FFA_MEM_ATTR_NORMAL_MEMORY_CACHED_WB;
		break;

	default:
		return -EINVAL;
	}

	inf->paddr = pte;

	/* add other attributes */
#if defined(CONFIG_ARM64) || defined(CONFIG_ARM_LPAE)
	pte |= pgprot_val(pgprot);
#elif defined(CONFIG_ARM)
	if (pgprot_val(pgprot) & L_PTE_RDONLY)
		pte |= ATTR_RDONLY;
	if (pgprot_val(pgprot) & L_PTE_SHARED)
		pte |= ATTR_INNER_SHAREABLE; /* inner sharable */
#endif

	if (!(pte & ATTR_RDONLY))
		ffa_mem_perm |= FFA_MEM_PERM_RW;
	else
		ffa_mem_perm |= FFA_MEM_PERM_RO;

	if ((pte & ATTR_INNER_SHAREABLE) == ATTR_INNER_SHAREABLE)
		ffa_mem_attr |= FFA_MEM_ATTR_INNER_SHAREABLE;

	inf->ffa_mem_attr = ffa_mem_attr;
	inf->ffa_mem_perm = ffa_mem_perm;
	inf->compat_attr = (pte & 0x0000FFFFFFFFFFFFull) |
			   ((u64)mem_attr << 48);
	return 0;
}

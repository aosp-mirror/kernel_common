/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 Google, Inc.
 */
#ifndef __LINUX_TRUSTY_TRUSTY_H
#define __LINUX_TRUSTY_TRUSTY_H

#include <linux/kernel.h>
#include <linux/trusty/sm_err.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/pagemap.h>


#if IS_ENABLED(CONFIG_TRUSTY)
s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);
#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2);
#endif
#else
static inline s32 trusty_std_call32(struct device *dev, u32 smcnr,
				    u32 a0, u32 a1, u32 a2)
{
	return SM_ERR_UNDEFINED_SMC;
}
static inline s32 trusty_fast_call32(struct device *dev, u32 smcnr,
				     u32 a0, u32 a1, u32 a2)
{
	return SM_ERR_UNDEFINED_SMC;
}
#ifdef CONFIG_64BIT
static inline s64 trusty_fast_call64(struct device *dev,
				     u64 smcnr, u64 a0, u64 a1, u64 a2)
{
	return SM_ERR_UNDEFINED_SMC;
}
#endif
#endif

struct notifier_block;
enum {
	TRUSTY_CALL_PREPARE,
	TRUSTY_CALL_RETURNED,
};
int trusty_call_notifier_register(struct device *dev,
				  struct notifier_block *n);
int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n);
const char *trusty_version_str_get(struct device *dev);
u32 trusty_get_api_version(struct device *dev);
bool trusty_get_panic_status(struct device *dev);

struct ns_mem_page_info {
	u64 paddr;
	u8 ffa_mem_attr;
	u8 ffa_mem_perm;
	u64 compat_attr;
};

int trusty_encode_page_info(struct ns_mem_page_info *inf,
			    struct page *page, pgprot_t pgprot);

struct scatterlist;
typedef u64 trusty_shared_mem_id_t;
int trusty_share_memory(struct device *dev, trusty_shared_mem_id_t *id,
			struct scatterlist *sglist, unsigned int nents,
			pgprot_t pgprot);
int trusty_share_memory_compat(struct device *dev, trusty_shared_mem_id_t *id,
			       struct scatterlist *sglist, unsigned int nents,
			       pgprot_t pgprot);
int trusty_transfer_memory(struct device *dev, u64 *id,
			   struct scatterlist *sglist, unsigned int nents,
			   pgprot_t pgprot, u64 tag, bool lend);
int trusty_reclaim_memory(struct device *dev, trusty_shared_mem_id_t id,
			  struct scatterlist *sglist, unsigned int nents);

struct dma_buf;
#ifdef CONFIG_TRUSTY_DMA_BUF_FFA_TAG
u64 trusty_dma_buf_get_ffa_tag(struct dma_buf *dma_buf);
#else
static inline u64 trusty_dma_buf_get_ffa_tag(struct dma_buf *dma_buf)
{
	return 0;
}
#endif

struct trusty_nop {
	struct list_head node;
	u32 args[3];
};

static inline void trusty_nop_init(struct trusty_nop *nop,
				   u32 arg0, u32 arg1, u32 arg2) {
	INIT_LIST_HEAD(&nop->node);
	nop->args[0] = arg0;
	nop->args[1] = arg1;
	nop->args[2] = arg2;
}

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop);
void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop);

#endif

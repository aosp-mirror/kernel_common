// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/trusty/arm_ffa.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/trusty.h>

#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include "trusty-smc.h"
#include "trusty-trace.h"
#include "trusty-sched-share-api.h"


struct trusty_state;
static struct platform_driver trusty_driver;

static bool use_high_wq;
module_param(use_high_wq, bool, 0660);

struct trusty_work {
	struct task_struct *nop_thread;
	wait_queue_head_t nop_event_wait;
};

struct trusty_state {
	struct mutex smc_lock;
	struct atomic_notifier_head notifier;
	struct completion cpu_idle_completion;
	char *version_str;
	u32 api_version;
	bool trusty_panicked;
	struct device *dev;
	struct trusty_work __percpu *nop_works;
	struct list_head nop_queue;
	spinlock_t nop_lock; /* protects nop_queue */
	struct device_dma_parameters dma_parms;
	struct trusty_sched_share_state *trusty_sched_share_state;
	void *ffa_tx;
	void *ffa_rx;
	u16 ffa_local_id;
	u16 ffa_remote_id;
	struct mutex share_memory_msg_lock; /* protects share_memory_msg */
};

static inline unsigned long smc(unsigned long r0, unsigned long r1,
				unsigned long r2, unsigned long r3)
{
	unsigned long ret;

	trace_trusty_smc(r0, r1, r2, r3);
	ret = trusty_smc8(r0, r1, r2, r3, 0, 0, 0, 0).r0;
	trace_trusty_smc_done(ret);
	return ret;
}

s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!s))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	return smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL(trusty_fast_call32);

#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!s))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	return smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL(trusty_fast_call64);
#endif

static unsigned long trusty_std_call_inner(struct device *dev,
					   unsigned long smcnr,
					   unsigned long a0, unsigned long a1,
					   unsigned long a2)
{
	unsigned long ret;
	int retry = 5;

	dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx)\n",
		__func__, smcnr, a0, a1, a2);
	while (true) {
		ret = smc(smcnr, a0, a1, a2);
		while ((s32)ret == SM_ERR_FIQ_INTERRUPTED)
			ret = smc(SMC_SC_RESTART_FIQ, 0, 0, 0);
		if ((int)ret != SM_ERR_BUSY || !retry)
			break;

		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy, retry\n",
			__func__, smcnr, a0, a1, a2);
		retry--;
	}

	return ret;
}

static unsigned long trusty_std_call_helper(struct device *dev,
					    unsigned long smcnr,
					    unsigned long a0, unsigned long a1,
					    unsigned long a2)
{
	unsigned long ret;
	int sleep_time = 1;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	while (true) {
		local_irq_disable();
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_PREPARE,
					   NULL);
		ret = trusty_std_call_inner(dev, smcnr, a0, a1, a2);
		if (ret == SM_ERR_PANIC) {
			s->trusty_panicked = true;
			if (IS_ENABLED(CONFIG_TRUSTY_CRASH_IS_PANIC))
				panic("trusty crashed");
			else
				WARN_ONCE(1, "trusty crashed");
		}

		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_RETURNED,
					   NULL);
		if (ret == SM_ERR_INTERRUPTED) {
			/*
			 * Make sure this cpu will eventually re-enter trusty
			 * even if the std_call resumes on another cpu.
			 */
			trusty_enqueue_nop(dev, NULL);
		}
		local_irq_enable();

		if ((int)ret != SM_ERR_BUSY)
			break;

		if (sleep_time == 256)
			dev_warn(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy\n",
				 __func__, smcnr, a0, a1, a2);
		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) returned busy, wait %d ms\n",
			__func__, smcnr, a0, a1, a2, sleep_time);

		msleep(sleep_time);
		if (sleep_time < 1000)
			sleep_time <<= 1;

		dev_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) retry\n",
			__func__, smcnr, a0, a1, a2);
	}

	if (sleep_time > 256)
		dev_warn(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) busy cleared\n",
			 __func__, smcnr, a0, a1, a2);

	return ret;
}

static void trusty_std_call_cpu_idle(struct trusty_state *s)
{
	int ret;

	ret = wait_for_completion_timeout(&s->cpu_idle_completion, HZ * 10);
	if (!ret) {
		dev_warn(s->dev,
			 "%s: timed out waiting for cpu idle to clear, retry anyway\n",
			 __func__);
	}
}

s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	int ret;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	if (WARN_ON(SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	if (s->trusty_panicked) {
		/*
		 * Avoid calling the notifiers if trusty has panicked as they
		 * can trigger more calls.
		 */
		return SM_ERR_PANIC;
	}

	trace_trusty_std_call32(smcnr, a0, a1, a2);

	if (smcnr != SMC_SC_NOP) {
		mutex_lock(&s->smc_lock);
		reinit_completion(&s->cpu_idle_completion);
	}

	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) started\n",
		__func__, smcnr, a0, a1, a2);

	ret = trusty_std_call_helper(dev, smcnr, a0, a1, a2);
	while (ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {
		dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) interrupted\n",
			__func__, smcnr, a0, a1, a2);
		if (ret == SM_ERR_CPU_IDLE)
			trusty_std_call_cpu_idle(s);
		ret = trusty_std_call_helper(dev, SMC_SC_RESTART_LAST, 0, 0, 0);
	}
	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) returned 0x%x\n",
		__func__, smcnr, a0, a1, a2, ret);

	if (smcnr == SMC_SC_NOP)
		complete(&s->cpu_idle_completion);
	else
		mutex_unlock(&s->smc_lock);

	trace_trusty_std_call32_done(ret);

	return ret;
}
EXPORT_SYMBOL(trusty_std_call32);

int trusty_share_memory(struct device *dev, u64 *id,
			struct scatterlist *sglist, unsigned int nents,
			pgprot_t pgprot)
{
	return trusty_transfer_memory(dev, id, sglist, nents, pgprot, 0,
				      false);
}
EXPORT_SYMBOL(trusty_share_memory);

int trusty_transfer_memory(struct device *dev, u64 *id,
			   struct scatterlist *sglist, unsigned int nents,
			   pgprot_t pgprot, u64 tag, bool lend)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	int ret;
	struct ns_mem_page_info pg_inf;
	struct scatterlist *sg;
	size_t count;
	size_t i;
	size_t len = 0;
	u64 ffa_handle = 0;
	size_t total_len;
	size_t endpoint_count = 1;
	struct ffa_mtd *mtd = s->ffa_tx;
	size_t comp_mrd_offset = offsetof(struct ffa_mtd, emad[endpoint_count]);
	struct ffa_comp_mrd *comp_mrd = s->ffa_tx + comp_mrd_offset;
	struct ffa_cons_mrd *cons_mrd = comp_mrd->address_range_array;
	size_t cons_mrd_offset = (void *)cons_mrd - s->ffa_tx;
	struct smc_ret8 smc_ret;
	u32 cookie_low;
	u32 cookie_high;

	if (WARN_ON(dev->driver != &trusty_driver.driver))
		return -EINVAL;

	if (WARN_ON(nents < 1))
		return -EINVAL;

	if (nents != 1 && s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		dev_err(s->dev, "%s: old trusty version does not support non-contiguous memory objects\n",
			__func__);
		return -EOPNOTSUPP;
	}

	count = dma_map_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
	if (count != nents) {
		dev_err(s->dev, "failed to dma map sg_table\n");
		return -EINVAL;
	}

	sg = sglist;
	ret = trusty_encode_page_info(&pg_inf, phys_to_page(sg_dma_address(sg)),
				      pgprot);
	if (ret) {
		dev_err(s->dev, "%s: trusty_encode_page_info failed\n",
			__func__);
		goto err_encode_page_info;
	}

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		*id = pg_inf.compat_attr;
		return 0;
	}

	len = 0;
	for_each_sg(sglist, sg, nents, i)
		len += sg_dma_len(sg);

	trace_trusty_share_memory(len, nents, lend);

	mutex_lock(&s->share_memory_msg_lock);

	mtd->sender_id = s->ffa_local_id;
	mtd->memory_region_attributes = pg_inf.ffa_mem_attr;
	mtd->reserved_3 = 0;
	mtd->flags = 0;
	mtd->handle = 0;
	mtd->tag = tag;
	mtd->reserved_24_27 = 0;
	mtd->emad_count = endpoint_count;
	for (i = 0; i < endpoint_count; i++) {
		struct ffa_emad *emad = &mtd->emad[i];
		/* TODO: support stream ids */
		emad->mapd.endpoint_id = s->ffa_remote_id;
		emad->mapd.memory_access_permissions = pg_inf.ffa_mem_perm;
		emad->mapd.flags = 0;
		emad->comp_mrd_offset = comp_mrd_offset;
		emad->reserved_8_15 = 0;
	}
	comp_mrd->total_page_count = len / PAGE_SIZE;
	comp_mrd->address_range_count = nents;
	comp_mrd->reserved_8_15 = 0;

	total_len = cons_mrd_offset + nents * sizeof(*cons_mrd);
	sg = sglist;
	while (count) {
		size_t lcount =
			min_t(size_t, count, (PAGE_SIZE - cons_mrd_offset) /
			      sizeof(*cons_mrd));
		size_t fragment_len = lcount * sizeof(*cons_mrd) +
				      cons_mrd_offset;

		for (i = 0; i < lcount; i++) {
			cons_mrd[i].address = sg_dma_address(sg);
			cons_mrd[i].page_count = sg_dma_len(sg) / PAGE_SIZE;
			cons_mrd[i].reserved_12_15 = 0;
			sg = sg_next(sg);
		}
		count -= lcount;
		if (cons_mrd_offset) {
			u32 smc = lend ? SMC_FC_FFA_MEM_LEND :
					 SMC_FC_FFA_MEM_SHARE;
			/* First fragment */
			smc_ret = trusty_smc8(smc, total_len,
					      fragment_len, 0, 0, 0, 0, 0);
		} else {
			smc_ret = trusty_smc8(SMC_FC_FFA_MEM_FRAG_TX,
					      cookie_low, cookie_high,
					      fragment_len, 0, 0, 0, 0);
		}
		if (smc_ret.r0 == SMC_FC_FFA_MEM_FRAG_RX) {
			cookie_low = smc_ret.r1;
			cookie_high = smc_ret.r2;
			dev_dbg(s->dev, "cookie %x %x", cookie_low,
				cookie_high);
			if (!count) {
				/*
				 * We have sent all our descriptors. Expected
				 * SMC_FC_FFA_SUCCESS, not a request to send
				 * another fragment.
				 */
				dev_err(s->dev, "%s: fragment_len %zd/%zd, unexpected SMC_FC_FFA_MEM_FRAG_RX\n",
					__func__, fragment_len, total_len);
				ret = -EIO;
				break;
			}
		} else if (smc_ret.r0 == SMC_FC_FFA_SUCCESS) {
			ffa_handle = smc_ret.r2 | (u64)smc_ret.r3 << 32;
			dev_dbg(s->dev, "%s: fragment_len %zu/%zu, got handle 0x%llx\n",
				__func__, fragment_len, total_len,
				ffa_handle);
			if (count) {
				/*
				 * We have not sent all our descriptors.
				 * Expected SMC_FC_FFA_MEM_FRAG_RX not
				 * SMC_FC_FFA_SUCCESS.
				 */
				dev_err(s->dev, "%s: fragment_len %zu/%zu, unexpected SMC_FC_FFA_SUCCESS, count %zu != 0\n",
					__func__, fragment_len, total_len,
					count);
				ret = -EIO;
				break;
			}
		} else {
			dev_err(s->dev, "%s: fragment_len %zu/%zu, SMC_FC_FFA_MEM_SHARE failed 0x%lx 0x%lx 0x%lx",
				__func__, fragment_len, total_len,
				smc_ret.r0, smc_ret.r1, smc_ret.r2);
			ret = -EIO;
			break;
		}

		cons_mrd = s->ffa_tx;
		cons_mrd_offset = 0;
	}

	mutex_unlock(&s->share_memory_msg_lock);

	if (!ret) {
		*id = ffa_handle;
		dev_dbg(s->dev, "%s: done\n", __func__);
		goto done;
	}

	dev_err(s->dev, "%s: failed %d", __func__, ret);

err_encode_page_info:
	dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
done:
	trace_trusty_share_memory_done(len, nents, lend, ffa_handle, ret);
	return ret;
}
EXPORT_SYMBOL(trusty_transfer_memory);

/*
 * trusty_share_memory_compat - trusty_share_memory wrapper for old apis
 *
 * Call trusty_share_memory and filter out memory attributes if trusty version
 * is old. Used by clients that used to pass just a physical address to trusty
 * instead of a physical address plus memory attributes value.
 */
int trusty_share_memory_compat(struct device *dev, u64 *id,
			       struct scatterlist *sglist, unsigned int nents,
			       pgprot_t pgprot)
{
	int ret;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	ret = trusty_share_memory(dev, id, sglist, nents, pgprot);
	if (!ret && s->api_version < TRUSTY_API_VERSION_PHYS_MEM_OBJ)
		*id &= 0x0000FFFFFFFFF000ull;

	return ret;
}
EXPORT_SYMBOL(trusty_share_memory_compat);

int trusty_reclaim_memory(struct device *dev, u64 id,
			  struct scatterlist *sglist, unsigned int nents)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	int ret = 0;
	struct smc_ret8 smc_ret;

	if (WARN_ON(dev->driver != &trusty_driver.driver))
		return -EINVAL;

	if (WARN_ON(nents < 1))
		return -EINVAL;

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		if (nents != 1) {
			dev_err(s->dev, "%s: not supported\n", __func__);
			return -EOPNOTSUPP;
		}

		dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);

		dev_dbg(s->dev, "%s: done\n", __func__);
		return 0;
	}

	trace_trusty_reclaim_memory(id);
	mutex_lock(&s->share_memory_msg_lock);

	smc_ret = trusty_smc8(SMC_FC_FFA_MEM_RECLAIM, (u32)id, id >> 32, 0, 0,
			      0, 0, 0);
	if (smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FC_FFA_MEM_RECLAIM failed 0x%lx 0x%lx 0x%lx",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		if (smc_ret.r0 == SMC_FC_FFA_ERROR &&
		    smc_ret.r2 == FFA_ERROR_DENIED)
			ret = -EBUSY;
		else
			ret = -EIO;
	}

	mutex_unlock(&s->share_memory_msg_lock);

	if (ret != 0)
		goto err_ffa_mem_reclaim;

	dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);

	dev_dbg(s->dev, "%s: done\n", __func__);

err_ffa_mem_reclaim:
	trace_trusty_reclaim_memory_done(id, ret);
	return ret;
}
EXPORT_SYMBOL(trusty_reclaim_memory);

int trusty_call_notifier_register(struct device *dev, struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_register(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_register);

int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_unregister(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_unregister);

static int trusty_remove_child(struct device *dev, void *data)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static ssize_t trusty_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return scnprintf(buf, PAGE_SIZE, "%s\n", s->version_str ?: "unknown");
}

static DEVICE_ATTR(trusty_version, 0400, trusty_version_show, NULL);

static struct attribute *trusty_attrs[] = {
	&dev_attr_trusty_version.attr,
	NULL,
};
ATTRIBUTE_GROUPS(trusty);

const char *trusty_version_str_get(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->version_str;
}
EXPORT_SYMBOL(trusty_version_str_get);

static int trusty_init_msg_buf(struct trusty_state *s, struct device *dev)
{
	phys_addr_t tx_paddr;
	phys_addr_t rx_paddr;
	int ret;
	struct smc_ret8 smc_ret;

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ)
		return 0;

	/* Get supported FF-A version and check if it is compatible */
	smc_ret = trusty_smc8(SMC_FC_FFA_VERSION, FFA_CURRENT_VERSION, 0, 0,
			      0, 0, 0, 0);
	if (FFA_VERSION_TO_MAJOR(smc_ret.r0) != FFA_CURRENT_VERSION_MAJOR) {
		dev_err(s->dev,
			"%s: Unsupported FF-A version 0x%lx, expected 0x%x\n",
			__func__, smc_ret.r0, FFA_CURRENT_VERSION);
		ret = -EIO;
		goto err_version;
	}

	/* Check that SMC_FC_FFA_MEM_SHARE is implemented */
	smc_ret = trusty_smc8(SMC_FC_FFA_FEATURES, SMC_FC_FFA_MEM_SHARE, 0, 0,
			      0, 0, 0, 0);
	if (smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev,
			"%s: SMC_FC_FFA_FEATURES(SMC_FC_FFA_MEM_SHARE) failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = -EIO;
		goto err_features;
	}

	/*
	 * Set FF-A endpoint IDs.
	 *
	 * Hardcode 0x8000 for the secure os.
	 * TODO: Use FF-A call or device tree to configure this dynamically
	 */
	smc_ret = trusty_smc8(SMC_FC_FFA_ID_GET, 0, 0, 0, 0, 0, 0, 0);
	if (smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev,
			"%s: SMC_FC_FFA_ID_GET failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = -EIO;
		goto err_id_get;
	}

	s->ffa_local_id = smc_ret.r2;
	s->ffa_remote_id = 0x8000;

	s->ffa_tx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_tx) {
		ret = -ENOMEM;
		goto err_alloc_tx;
	}
	tx_paddr = virt_to_phys(s->ffa_tx);
	if (WARN_ON(tx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_unaligned_tx_buf;
	}

	s->ffa_rx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_rx) {
		ret = -ENOMEM;
		goto err_alloc_rx;
	}
	rx_paddr = virt_to_phys(s->ffa_rx);
	if (WARN_ON(rx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_unaligned_rx_buf;
	}

	smc_ret = trusty_smc8(SMC_FCZ_FFA_RXTX_MAP, tx_paddr, rx_paddr, 1, 0,
			      0, 0, 0);
	if (smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FCZ_FFA_RXTX_MAP failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = -EIO;
		goto err_rxtx_map;
	}

	return 0;

err_rxtx_map:
err_unaligned_rx_buf:
	kfree(s->ffa_rx);
	s->ffa_rx = NULL;
err_alloc_rx:
err_unaligned_tx_buf:
	kfree(s->ffa_tx);
	s->ffa_tx = NULL;
err_alloc_tx:
err_id_get:
err_features:
err_version:
	return ret;
}

static void trusty_free_msg_buf(struct trusty_state *s, struct device *dev)
{
	struct smc_ret8 smc_ret;

	smc_ret = trusty_smc8(SMC_FC_FFA_RXTX_UNMAP, 0, 0, 0, 0, 0, 0, 0);
	if (smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FC_FFA_RXTX_UNMAP failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
	} else {
		kfree(s->ffa_rx);
		kfree(s->ffa_tx);
	}
}

static void trusty_init_version(struct trusty_state *s, struct device *dev)
{
	int ret;
	int i;
	int version_str_len;

	ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, -1, 0, 0);
	if (ret <= 0)
		goto err_get_size;

	version_str_len = ret;

	s->version_str = kmalloc(version_str_len + 1, GFP_KERNEL);
	for (i = 0; i < version_str_len; i++) {
		ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, i, 0, 0);
		if (ret < 0)
			goto err_get_char;
		s->version_str[i] = ret;
	}
	s->version_str[i] = '\0';

	dev_info(dev, "trusty version: %s\n", s->version_str);
	return;

err_get_char:
	kfree(s->version_str);
	s->version_str = NULL;
err_get_size:
	dev_err(dev, "failed to get version: %d\n", ret);
}

u32 trusty_get_api_version(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->api_version;
}
EXPORT_SYMBOL(trusty_get_api_version);

bool trusty_get_panic_status(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	if (WARN_ON(dev->driver != &trusty_driver.driver))
		return false;
	return s->trusty_panicked;
}
EXPORT_SYMBOL(trusty_get_panic_status);

static int trusty_init_api_version(struct trusty_state *s, struct device *dev)
{
	u32 api_version;

	api_version = trusty_fast_call32(dev, SMC_FC_API_VERSION,
					 TRUSTY_API_VERSION_CURRENT, 0, 0);
	if (api_version == SM_ERR_UNDEFINED_SMC)
		api_version = 0;

	if (api_version > TRUSTY_API_VERSION_CURRENT) {
		dev_err(dev, "unsupported api version %u > %u\n",
			api_version, TRUSTY_API_VERSION_CURRENT);
		return -EINVAL;
	}

	dev_info(dev, "selected api version: %u (requested %u)\n",
		 api_version, TRUSTY_API_VERSION_CURRENT);
	s->api_version = api_version;

	return 0;
}

static bool dequeue_nop(struct trusty_state *s, u32 *args)
{
	unsigned long flags;
	struct trusty_nop *nop = NULL;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (!list_empty(&s->nop_queue)) {
		nop = list_first_entry(&s->nop_queue,
				       struct trusty_nop, node);
		list_del_init(&nop->node);
		args[0] = nop->args[0];
		args[1] = nop->args[1];
		args[2] = nop->args[2];
	} else {
		args[0] = 0;
		args[1] = 0;
		args[2] = 0;
	}
	spin_unlock_irqrestore(&s->nop_lock, flags);
	return nop;
}

static void locked_nop_work_func(struct trusty_state *s)
{
	int ret;

	ret = trusty_std_call32(s->dev, SMC_SC_LOCKED_NOP, 0, 0, 0);
	if (ret != 0)
		dev_err(s->dev, "%s: SMC_SC_LOCKED_NOP failed %d",
			__func__, ret);

	dev_dbg(s->dev, "%s: done\n", __func__);
}

static void nop_work_func(struct trusty_state *s)
{
	int ret;
	bool next;
	u32 args[3];
	u32 last_arg0;
	int old_nice = task_nice(current);
	bool nice_changed = false;

	dequeue_nop(s, args);
	do {
		/*
		 * In case use_high_wq flaged when trusty is not idle,
		 * change the work's prio directly.
		 */
		if (!WARN_ON(current->policy != SCHED_NORMAL)) {
			if (use_high_wq && task_nice(current) != MIN_NICE) {
				nice_changed = true;
				set_user_nice(current, MIN_NICE);
			} else if (!use_high_wq &&
				   task_nice(current) == MIN_NICE) {
				nice_changed = true;
				set_user_nice(current, 0);
			}
		}

		dev_dbg(s->dev, "%s: %x %x %x\n",
			__func__, args[0], args[1], args[2]);

		last_arg0 = args[0];
		ret = trusty_std_call32(s->dev, SMC_SC_NOP,
					args[0], args[1], args[2]);

		next = dequeue_nop(s, args);

		if (ret == SM_ERR_NOP_INTERRUPTED) {
			next = true;
		} else if (ret != SM_ERR_NOP_DONE) {
			dev_err(s->dev, "%s: SMC_SC_NOP %x failed %d",
				__func__, last_arg0, ret);
			if (last_arg0) {
				/*
				 * Don't break out of the loop if a non-default
				 * nop-handler returns an error.
				 */
				next = true;
			}
		}
	} while (next);
	/*
	 * Restore nice if even changed.
	 */
	if (nice_changed)
		set_user_nice(current, old_nice);
	dev_dbg(s->dev, "%s: done\n", __func__);
}

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop)
{
	unsigned long flags;
	struct trusty_work *tw;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	trace_trusty_enqueue_nop(nop);
	preempt_disable();
	tw = this_cpu_ptr(s->nop_works);
	if (nop) {
		WARN_ON(s->api_version < TRUSTY_API_VERSION_SMP_NOP);

		spin_lock_irqsave(&s->nop_lock, flags);
		if (list_empty(&nop->node))
			list_add_tail(&nop->node, &s->nop_queue);
		spin_unlock_irqrestore(&s->nop_lock, flags);
	}
	wake_up_interruptible(&tw->nop_event_wait);
	preempt_enable();
}
EXPORT_SYMBOL(trusty_enqueue_nop);

void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop)
{
	unsigned long flags;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!nop))
		return;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (!list_empty(&nop->node))
		list_del_init(&nop->node);
	spin_unlock_irqrestore(&s->nop_lock, flags);
}
EXPORT_SYMBOL(trusty_dequeue_nop);

static int trusty_nop_thread(void *context)
{
	struct trusty_state *s = context;
	struct trusty_work *tw = this_cpu_ptr(s->nop_works);
	void (*work_func)(struct trusty_state *s);
	int ret = 0;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	if (s->api_version < TRUSTY_API_VERSION_SMP)
		work_func = locked_nop_work_func;
	else
		work_func = nop_work_func;

	add_wait_queue(&tw->nop_event_wait, &wait);
	for (;;) {
		if (kthread_should_stop())
			break;

		wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);

		/* process work */
		work_func(s);
	};
	remove_wait_queue(&tw->nop_event_wait, &wait);

	return ret;
}

static int trusty_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int cpu;
	struct trusty_state *s;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		dev_err(&pdev->dev, "of_node required\n");
		return -EINVAL;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto err_allocate_state;
	}

	s->dev = &pdev->dev;
	spin_lock_init(&s->nop_lock);
	INIT_LIST_HEAD(&s->nop_queue);
	mutex_init(&s->smc_lock);
	mutex_init(&s->share_memory_msg_lock);
	ATOMIC_INIT_NOTIFIER_HEAD(&s->notifier);
	init_completion(&s->cpu_idle_completion);

	s->dev->dma_parms = &s->dma_parms;
	dma_set_max_seg_size(s->dev, 0xfffff000); /* dma_parms limit */
	/*
	 * Set dma mask to 48 bits. This is the current limit of
	 * trusty_encode_page_info.
	 */
	dma_coerce_mask_and_coherent(s->dev, DMA_BIT_MASK(48));

	platform_set_drvdata(pdev, s);

	trusty_init_version(s, &pdev->dev);

	ret = trusty_init_api_version(s, &pdev->dev);
	if (ret < 0)
		goto err_api_version;

	ret = trusty_init_msg_buf(s, &pdev->dev);
	if (ret < 0)
		goto err_init_msg_buf;

	s->nop_works = alloc_percpu(struct trusty_work);
	if (!s->nop_works) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Failed to allocate works\n");
		goto err_alloc_works;
	}

	for_each_possible_cpu(cpu) {
		struct trusty_work *tw = per_cpu_ptr(s->nop_works, cpu);

		tw->nop_thread = ERR_PTR(-EINVAL);
		init_waitqueue_head(&tw->nop_event_wait);
	}

	for_each_possible_cpu(cpu) {
		struct trusty_work *tw = per_cpu_ptr(s->nop_works, cpu);

		tw->nop_thread = kthread_create(trusty_nop_thread, s,
				"trusty-nop-%d", cpu);
		if (IS_ERR(tw->nop_thread)) {
			ret = PTR_ERR(tw->nop_thread);
			dev_err(s->dev, "%s: failed to create thread for cpu= %d (%p)\n",
					__func__, cpu, tw->nop_thread);
			goto err_thread_create;
		}

		kthread_bind(tw->nop_thread, cpu);
		wake_up_process(tw->nop_thread);
	}

	s->trusty_sched_share_state = trusty_register_sched_share(&pdev->dev);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add children: %d\n", ret);
		goto err_add_children;
	}

	return 0;

err_add_children:
err_thread_create:
	for_each_possible_cpu(cpu) {
		struct trusty_work *tw = per_cpu_ptr(s->nop_works, cpu);

		if (!IS_ERR(tw->nop_thread))
			kthread_stop(tw->nop_thread);
	}
	free_percpu(s->nop_works);
err_alloc_works:
	trusty_free_msg_buf(s, &pdev->dev);
err_init_msg_buf:
err_api_version:
	s->dev->dma_parms = NULL;
	kfree(s->version_str);
	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->share_memory_msg_lock);
	mutex_destroy(&s->smc_lock);
	kfree(s);
err_allocate_state:
	return ret;
}

static int trusty_remove(struct platform_device *pdev)
{
	unsigned int cpu;
	struct trusty_state *s = platform_get_drvdata(pdev);

	trusty_unregister_sched_share(s->trusty_sched_share_state);

	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);

	for_each_possible_cpu(cpu) {
		struct trusty_work *tw = per_cpu_ptr(s->nop_works, cpu);

		kthread_stop(tw->nop_thread);
	}
	free_percpu(s->nop_works);

	mutex_destroy(&s->share_memory_msg_lock);
	mutex_destroy(&s->smc_lock);
	trusty_free_msg_buf(s, &pdev->dev);
	s->dev->dma_parms = NULL;
	kfree(s->version_str);
	kfree(s);
	return 0;
}

static const struct of_device_id trusty_of_match[] = {
	{ .compatible = "android,trusty-smc-v1", },
	{},
};

MODULE_DEVICE_TABLE(trusty, trusty_of_match);

static struct platform_driver trusty_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver	= {
		.name = "trusty",
		.of_match_table = trusty_of_match,
		.dev_groups = trusty_groups,
	},
};

static int __init trusty_driver_init(void)
{
	return platform_driver_register(&trusty_driver);
}

static void __exit trusty_driver_exit(void)
{
	platform_driver_unregister(&trusty_driver);
}

subsys_initcall(trusty_driver_init);
module_exit(trusty_driver_exit);

#define CREATE_TRACE_POINTS
#include "trusty-trace.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty core driver");

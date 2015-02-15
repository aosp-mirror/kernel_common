/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *  Copyright (C) 2010 Linus Walleij
 *  Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/pm_runtime.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include "core.h"
#include "host.h"

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

extern struct workqueue_struct *stats_workqueue;
static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	kfree(host->wlock_name);
	kfree(host);
}

static int mmc_host_runtime_suspend(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return 0;

	if (mmc_is_sd_host(host))
		return 0;

	ret = mmc_suspend_host(host);
	if (ret < 0)
		pr_err("%s: %s: suspend host failed: %d\n", mmc_hostname(host),
		       __func__, ret);

	return ret;
}

static int mmc_host_runtime_resume(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return 0;

	if (mmc_is_sd_host(host)) {
		host->crc_count = 0;
		return 0;
	}
	ret = mmc_resume_host(host);
	if (ret < 0) {
		pr_err("%s: %s: resume host: failed: ret: %d\n",
		       mmc_hostname(host), __func__, ret);
		if (pm_runtime_suspended(dev))
			BUG_ON(1);
	}

	return ret;
}

static int mmc_host_suspend(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;
	unsigned long flags;

	if (!mmc_use_core_pm(host))
		return 0;
	pr_info("%s: %s\n", mmc_hostname(host), __func__);

	spin_lock_irqsave(&host->clk_lock, flags);
	host->dev_status = DEV_SUSPENDING;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	if (!pm_runtime_suspended(dev) || mmc_is_sd_host(host)) {
		ret = mmc_suspend_host(host);
		if (ret < 0)
			pr_err("%s: %s: failed: ret: %d\n", mmc_hostname(host),
			       __func__, ret);
	}
	if (!ret && host->card && mmc_card_sdio(host->card) &&
	    host->ios.clock) {
		spin_lock_irqsave(&host->clk_lock, flags);
		host->clk_old = host->ios.clock;
		host->ios.clock = 0;
		host->clk_gated = true;
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_set_ios(host);
	}
	spin_lock_irqsave(&host->clk_lock, flags);
	host->dev_status = DEV_SUSPENDED;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return ret;
}

static int mmc_host_resume(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int ret = 0;

	if (!mmc_use_core_pm(host))
		return 0;
	pr_info("%s: %s\n", mmc_hostname(host), __func__);
	if (mmc_bus_manual_resume(host) && host->card) {
		host->bus_resume_flags |= MMC_BUSRESUME_NEEDS_RESUME;
		return 0;
	}

	if (!pm_runtime_suspended(dev) || mmc_is_sd_host(host)) {
		ret = mmc_resume_host(host);
		if (ret < 0)
			pr_err("%s: %s: failed: ret: %d\n", mmc_hostname(host),
			       __func__, ret);
	}
	host->dev_status = DEV_RESUMED;
	return ret;
}

static const struct dev_pm_ops mmc_host_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmc_host_suspend, mmc_host_resume)
	SET_RUNTIME_PM_OPS(mmc_host_runtime_suspend, mmc_host_runtime_resume,
			   pm_generic_runtime_idle)
};

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.dev_release	= mmc_host_classdev_release,
	.pm		= &mmc_host_pm_ops,
};

int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);
}

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

static DEFINE_IDR(mmc_host_idr);
static DEFINE_SPINLOCK(mmc_host_lock);

#ifdef CONFIG_MMC_CLKGATE
static ssize_t clkgate_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	return snprintf(buf, PAGE_SIZE, "%lu\n", host->clkgate_delay);
}

static ssize_t clkgate_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clkgate_delay = value;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return count;
}

static void mmc_host_clk_gate_delayed(struct mmc_host *host)
{
	unsigned long tick_ns;
	unsigned long freq = host->ios.clock;
	unsigned long flags;

	if (!freq) {
		pr_debug("%s: frequency set to 0 in disable function, "
			 "this means the clock is already disabled.\n",
			 mmc_hostname(host));
		return;
	}
	spin_lock_irqsave(&host->clk_lock, flags);

	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		tick_ns = DIV_ROUND_UP(1000000000, freq);
		ndelay(host->clk_delay * tick_ns);
	} else {
		
		spin_unlock_irqrestore(&host->clk_lock, flags);
		return;
	}
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		
		mmc_gate_clock(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: gated MCI clock\n", mmc_hostname(host));
	}
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

static void mmc_host_clk_gate_work(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host,
					      clk_gate_work.work);

	mmc_host_clk_gate_delayed(host);
}

void mmc_host_clk_hold(struct mmc_host *host)
{
	unsigned long flags;

	
	cancel_delayed_work_sync(&host->clk_gate_work);
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_ungate_clock(host);

		
		mmc_reset_clk_scale_stats(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: ungated MCI clock\n", mmc_hostname(host));
	}
	host->clk_requests++;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

bool mmc_host_may_gate_card(struct mmc_card *card)
{
	
	if (!card)
		return true;

	if (mmc_card_sdio(card) && card->cccr.async_intr_sup)
		return true;

	return !(card->quirks & MMC_QUIRK_BROKEN_CLK_GATING);
}

void mmc_host_clk_release(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_requests--;
	if (mmc_host_may_gate_card(host->card) &&
	    !host->clk_requests)
		queue_delayed_work(system_nrt_wq, &host->clk_gate_work,
				msecs_to_jiffies(host->clkgate_delay));
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	unsigned long freq;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated)
		freq = host->clk_old;
	else
		freq = host->ios.clock;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return freq;
}

static inline void mmc_host_clk_init(struct mmc_host *host)
{
	host->clk_requests = 0;
	
	host->clk_delay = 8;
	host->clkgate_delay = 0;
	host->clk_gated = false;
	INIT_DELAYED_WORK(&host->clk_gate_work, mmc_host_clk_gate_work);
	spin_lock_init(&host->clk_lock);
	mutex_init(&host->clk_gate_mutex);
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
	if (cancel_delayed_work_sync(&host->clk_gate_work))
		mmc_host_clk_gate_delayed(host);
	if (host->clk_gated)
		mmc_host_clk_hold(host);
	
	WARN_ON(host->clk_requests > 1);
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
	host->clkgate_delay_attr.show = clkgate_delay_show;
	host->clkgate_delay_attr.store = clkgate_delay_store;
	sysfs_attr_init(&host->clkgate_delay_attr.attr);
	host->clkgate_delay_attr.attr.name = "clkgate_delay";
	host->clkgate_delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(&host->class_dev, &host->clkgate_delay_attr))
		pr_err("%s: Failed to create clkgate_delay sysfs entry\n",
				mmc_hostname(host));
}
#else

static inline void mmc_host_clk_init(struct mmc_host *host)
{
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
}

#endif

struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{
	int err;
	struct mmc_host *host;

	if (!idr_pre_get(&mmc_host_idr, GFP_KERNEL))
		return NULL;

	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (!host)
		return NULL;

	spin_lock(&mmc_host_lock);
	err = idr_get_new(&mmc_host_idr, host, &host->index);
	spin_unlock(&mmc_host_lock);
	if (err)
		goto free;

	dev_set_name(&host->class_dev, "mmc%d", host->index);

	host->parent = dev;
	host->class_dev.parent = dev;
	host->class_dev.class = &mmc_host_class;
	device_initialize(&host->class_dev);

	mmc_host_clk_init(host);

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	host->wlock_name = kasprintf(GFP_KERNEL,
			"%s_detect", mmc_hostname(host));
	wake_lock_init(&host->detect_wake_lock, WAKE_LOCK_SUSPEND,
			host->wlock_name);
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);
	INIT_DELAYED_WORK(&host->enable_detect, mmc_enable_detection);
	INIT_DELAYED_WORK(&host->stats_work, mmc_stats);
	INIT_DELAYED_WORK(&host->remove, mmc_remove_sd_card);
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	host->max_segs = 1;
	host->max_seg_size = PAGE_CACHE_SIZE;

	host->max_req_size = PAGE_CACHE_SIZE;
	host->max_blk_size = 512;
	host->max_blk_count = PAGE_CACHE_SIZE / 512;

	return host;

free:
	kfree(host);
	return NULL;
}

EXPORT_SYMBOL(mmc_alloc_host);

static ssize_t show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", mmc_can_scale_clk(host));
}

static ssize_t store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value, freq;
	int retval = -EINVAL;

	if (!host)
		goto out;

	mmc_claim_host(host);
	if (!host->card || kstrtoul(buf, 0, &value))
		goto err;

	if (value && !mmc_can_scale_clk(host)) {
		host->caps2 |= MMC_CAP2_CLK_SCALE;
		mmc_init_clk_scaling(host);

		if (!mmc_can_scale_clk(host)) {
			host->caps2 &= ~MMC_CAP2_CLK_SCALE;
			goto err;
		}
	} else if (!value && mmc_can_scale_clk(host)) {
		host->caps2 &= ~MMC_CAP2_CLK_SCALE;
		mmc_disable_clk_scaling(host);

		
		if (host->bus_ops && host->bus_ops->change_bus_speed) {
			freq = mmc_get_max_frequency(host);
			if (host->bus_ops->change_bus_speed(host, &freq))
				goto err;
		}
		if (host->ops->notify_load &&
				host->ops->notify_load(host, MMC_LOAD_HIGH))
			goto err;
		host->clk_scaling.state = MMC_LOAD_HIGH;
		host->clk_scaling.initialized = false;
	}
	retval = count;
err:
	mmc_release_host(host);
out:
	return retval;
}

static ssize_t show_up_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", host->clk_scaling.up_threshold);
}

#define MAX_PERCENTAGE	100
static ssize_t store_up_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.up_threshold = value;

	pr_debug("%s: clkscale_up_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_down_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			host->clk_scaling.down_threshold);
}

static ssize_t store_down_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.down_threshold = value;

	pr_debug("%s: clkscale_down_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_polling(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%lu milliseconds\n",
			host->clk_scaling.polling_delay_ms);
}

static ssize_t store_polling(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value))
		return -EINVAL;

	host->clk_scaling.polling_delay_ms = value;

	pr_debug("%s: clkscale_polling_delay_ms set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		show_enable, store_enable);
DEVICE_ATTR(polling_interval, S_IRUGO | S_IWUSR,
		show_polling, store_polling);
DEVICE_ATTR(up_threshold, S_IRUGO | S_IWUSR,
		show_up_threshold, store_up_threshold);
DEVICE_ATTR(down_threshold, S_IRUGO | S_IWUSR,
		show_down_threshold, store_down_threshold);

static struct attribute *clk_scaling_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_up_threshold.attr,
	&dev_attr_down_threshold.attr,
	&dev_attr_polling_interval.attr,
	NULL,
};

static struct attribute_group clk_scaling_attr_grp = {
	.name = "clk_scaling",
	.attrs = clk_scaling_attrs,
};

static ssize_t
show_perf(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t rtime_drv, wtime_drv;
	unsigned long rbytes_drv, wbytes_drv;

	spin_lock(&host->lock);

	rbytes_drv = host->perf.rbytes_drv;
	wbytes_drv = host->perf.wbytes_drv;

	rtime_drv = ktime_to_us(host->perf.rtime_drv);
	wtime_drv = ktime_to_us(host->perf.wtime_drv);

	spin_unlock(&host->lock);

	return snprintf(buf, PAGE_SIZE, "Write performance at driver Level:"
					"%lu bytes in %lld microseconds\n"
					"Read performance at driver Level:"
					"%lu bytes in %lld microseconds\n",
					wbytes_drv, wtime_drv,
					rbytes_drv, rtime_drv);
}

static ssize_t
set_perf(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t value;

	sscanf(buf, "%lld", &value);
	spin_lock(&host->lock);
	if (!value) {
		memset(&host->perf, 0, sizeof(host->perf));
		host->perf_enable = false;
	} else {
		host->perf_enable = true;
	}
	spin_unlock(&host->lock);

	return count;
}

static DEVICE_ATTR(perf, S_IRUGO | S_IWUSR,
		show_perf, set_perf);

static ssize_t
show_debug(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	if (!host)
		return 0;
	return sprintf(buf, "%d", host->debug_mask);
}

static ssize_t
set_debug(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	sscanf(buf, "%d", &value);
	host->debug_mask = value;
	pr_info("%s: set debug 0x%x\n", mmc_hostname(host), value);

	return count;
}
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR,
		show_debug, set_debug);

static struct attribute *dev_attrs[] = {
	&dev_attr_perf.attr,
	&dev_attr_debug.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);

	err = pm_runtime_set_active(&host->class_dev);
	if (err)
		pr_err("%s: %s: failed setting runtime active: err: %d\n",
		       mmc_hostname(host), __func__, err);
	else if (mmc_use_core_runtime_pm(host))
		pm_runtime_enable(&host->class_dev);

	err = device_add(&host->class_dev);
	if (err)
		return err;

	device_enable_async_suspend(&host->class_dev);
	

#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	mmc_host_clk_sysfs_init(host);

	host->clk_scaling.up_threshold = 35;
	host->clk_scaling.down_threshold = 5;
	host->clk_scaling.polling_delay_ms = 100;

	err = sysfs_create_group(&host->class_dev.kobj, &clk_scaling_attr_grp);
	if (err)
		pr_err("%s: failed to create clk scale sysfs group with err %d\n",
				__func__, err);

	err = sysfs_create_group(&host->class_dev.kobj, &dev_attr_grp);
	if (err)
		pr_err("%s: failed to create sysfs group with err %d\n",
							 __func__, err);

	mmc_start_host(host);
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);

	return 0;
}

EXPORT_SYMBOL(mmc_add_host);

void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		unregister_pm_notifier(&host->pm_notify);

	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif
	sysfs_remove_group(&host->parent->kobj, &dev_attr_grp);
	sysfs_remove_group(&host->class_dev.kobj, &clk_scaling_attr_grp);

	device_del(&host->class_dev);

	

	mmc_host_clk_exit(host);
}

EXPORT_SYMBOL(mmc_remove_host);

void mmc_free_host(struct mmc_host *host)
{
	spin_lock(&mmc_host_lock);
	idr_remove(&mmc_host_idr, host->index);
	spin_unlock(&mmc_host_lock);
	wake_lock_destroy(&host->detect_wake_lock);

	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);

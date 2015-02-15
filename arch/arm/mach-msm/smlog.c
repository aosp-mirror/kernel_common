/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/export.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/notifier.h>
#include <linux/of.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smem.h>
#include <mach/ramdump.h>
#include <mach/subsystem_notif.h>

#include "smem_private.h"

static uint32_t num_smlog_areas;
static struct ramdump_segment *smlog_ramdump_segments;
static void *smlog_ramdump_dev;
static DEFINE_MUTEX(spinlock_init_lock);

struct restart_notifier_block {
	unsigned processor;
	char *name;
	struct notifier_block nb;
};

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data);

static struct restart_notifier_block restart_notifiers[] = {
	{SMEM_MODEM, "modem", .nb.notifier_call = restart_notifier_cb},
};

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
#if defined(CONFIG_HTC_DEBUG_SSR)
	if (code == SUBSYS_AFTER_SHUTDOWN) {
		struct restart_notifier_block *notifier;

		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_debug("%s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);

		remote_spin_release_all(notifier->processor);

		if (smlog_ramdump_dev) {
			int ret;

			pr_info("%s: saving ramdump\n", __func__);
			ret = do_ramdump(smlog_ramdump_dev,
					smlog_ramdump_segments, num_smlog_areas);
			if (ret < 0)
				pr_err("%s: unable to dump smlog %d\n",
								__func__, ret);
		}
	}
#endif
	return NOTIFY_DONE;
}

static int msm_smlog_late_init(void)
{
	int i;
	void *handle;
	struct restart_notifier_block *nb;

	smlog_ramdump_dev = create_ramdump_device("smlog", NULL);
	if (IS_ERR_OR_NULL(smlog_ramdump_dev)) {
		pr_err("%s: Unable to create smlog ramdump device.\n",
			__func__);
		smlog_ramdump_dev = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(restart_notifiers); i++) {
		nb = &restart_notifiers[i];
		handle = subsys_notif_register_notifier(nb->name, &nb->nb);
		pr_info("%s: registering notif for '%s', handle=%p\n",
				__func__, nb->name, handle);
	}

	return 0;
}

static int msm_smlog_probe(struct platform_device *pdev)
{
	char *key;
	struct resource *r;
	int ret;
	int smlog_idx = 0;
	struct ramdump_segment *ramdump_segments_tmp = NULL;

	pr_info("%s\n", __func__);
	msm_smlog_late_init();

	key = "htc_smlog_res";
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, key);
	if (!r) {
		pr_err("%s: missing '%s'\n", __func__, key);
		return -ENODEV;
	}

	num_smlog_areas = 1;
	ramdump_segments_tmp = kmalloc_array(num_smlog_areas,
			sizeof(struct ramdump_segment), GFP_KERNEL);
	if (!ramdump_segments_tmp) {
		pr_err("%s: ramdump segment kmalloc failed\n", __func__);
		ret = -ENOMEM;
		goto free_smlog_areas;
	}

	ramdump_segments_tmp[smlog_idx].address = r->start;
	ramdump_segments_tmp[smlog_idx].size = resource_size(r);

	pr_info("smlog address= 0x%lx, size= 0x%lx\n",
		ramdump_segments_tmp[smlog_idx].address, ramdump_segments_tmp[smlog_idx].size);

	smlog_ramdump_segments = ramdump_segments_tmp;

	return 0;

free_smlog_areas:

	num_smlog_areas = 0;
	kfree(ramdump_segments_tmp);
	return ret;
}

static struct of_device_id msm_smlog_match_table[] = {
	{ .compatible = "qcom,smlog" },
	{},
};

static struct platform_driver msm_smlog_driver = {
	.probe = msm_smlog_probe,
	.driver = {
		.name = "msm_smlog",
		.owner = THIS_MODULE,
		.of_match_table = msm_smlog_match_table,
	},
};

int __init msm_smlog_init(void)
{
	static bool registered;
	int rc;

	if (registered)
		return 0;

	registered = true;

	rc = platform_driver_register(&msm_smlog_driver);
	if (rc) {
		pr_err("%s: msm_smlog_driver register failed %d\n",
							__func__, rc);
		return rc;
	}

	return 0;
}

module_init(msm_smlog_init);

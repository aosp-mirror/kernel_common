/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2013 HTC Corporation.
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "htc_cpu_usage_stats.h"

static struct platform_device *pdev_local = NULL;

void send_cpu_usage_stats_kobject_uevent(char *buf_pid)
{
    int ok_to_send = 0;
    char *envp[3] = {SUBSYSTEM_CPU_USAGE_STATS, 0, 0};

    if (strlen(buf_pid))
    {
        envp[1] = buf_pid;
        ok_to_send = 1;
    }

    if ((pdev_local != NULL) && ok_to_send)
    {
        kobject_uevent_env(&pdev_local->dev.kobj, KOBJ_CHANGE, envp);
    }
}
EXPORT_SYMBOL_GPL(send_cpu_usage_stats_kobject_uevent);

static int cpu_usage_stats_probe(struct platform_device *pdev)
{
	pdev_local = pdev;
	return 0;
}

static struct platform_driver cpu_usage_stats_driver = {
	.probe = cpu_usage_stats_probe,
	.driver = {
		.name = "cpu_usage_stats",
		.owner = THIS_MODULE,
	},
};

static int __init cpu_usage_stats_init(void)
{
	return platform_driver_register(&cpu_usage_stats_driver);
}

module_init(cpu_usage_stats_init);
MODULE_DESCRIPTION("cpu_usage_stats");
MODULE_LICENSE("GPL");


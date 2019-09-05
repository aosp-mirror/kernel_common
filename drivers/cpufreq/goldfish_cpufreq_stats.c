/*
 *  drivers/cpufreq/cpufreq_stats.c
 *
 *  Copyright (C) 2003-2004 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *  (C) 2004 Zou Nan hai <nanhai.zou@intel.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>

#define to_attr(a) container_of(a, struct freq_attr, attr)

static unsigned long long fake_time;

static DEFINE_PER_CPU(struct kobject *, cpufreq_kobj);

static ssize_t time_in_state_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len = sprintf(buf, "%lu %llu\n", 3000000000, ++fake_time);
	return len;
}

static struct kobj_attribute time_in_state =
	__ATTR_RO_MODE(time_in_state, 0444);

static struct attribute *cpufreq_stats_attrs[] = {
	&time_in_state.attr,
	NULL
};
static struct attribute_group stats_attr_group = {
	.attrs = cpufreq_stats_attrs,
	.name = "stats"
};

static void cpufreq_stats_free(unsigned int cpu)
{
	struct device *dev = get_cpu_device(cpu);
	struct kobject **kobj = &per_cpu(cpufreq_kobj, cpu);

	sysfs_remove_group(&dev->kobj, &stats_attr_group);
	kobject_put(*kobj);
	*kobj=NULL;
}

static void cpufreq_stats_create(unsigned int cpu)
{
	int ret;
	struct device *dev = get_cpu_device(cpu);
	struct kobject **kobj = &per_cpu(cpufreq_kobj, cpu);

	*kobj = kobject_create_and_add("cpufreq", &dev->kobj);
	ret = sysfs_create_group(*kobj, &stats_attr_group);
}

static int __init goldfish_cpufreq_stats_init(void)
{
	unsigned int cpu;

	for_each_online_cpu(cpu)
		cpufreq_stats_create(cpu);

	return 0;
}

static void __exit goldfish_cpufreq_stats_exit(void)
{
	unsigned int cpu;

	for_each_online_cpu(cpu)
		cpufreq_stats_free(cpu);
}

module_init(goldfish_cpufreq_stats_init);
module_exit(goldfish_cpufreq_stats_exit);

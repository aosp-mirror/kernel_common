// SPDX-License-Identifier: GPL-2.0
/*
 * Arch specific cpu topology information
 *
 * Copyright (C) 2016, ARM Ltd.
 * Written by: Juri Lelli, ARM Ltd.
 */

#include <linux/acpi.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched/topology.h>
#include <linux/cpuset.h>

DEFINE_PER_CPU(unsigned long, freq_scale) = SCHED_CAPACITY_SCALE;
DEFINE_PER_CPU(unsigned long, max_cpu_freq);
DEFINE_PER_CPU(unsigned long, max_freq_scale) = SCHED_CAPACITY_SCALE;

void arch_set_freq_scale(struct cpumask *cpus, unsigned long cur_freq,
			 unsigned long max_freq)
{
	unsigned long scale;
	int i;

	scale = (cur_freq << SCHED_CAPACITY_SHIFT) / max_freq;

	for_each_cpu(i, cpus) {
		per_cpu(freq_scale, i) = scale;
		per_cpu(max_cpu_freq, i) = max_freq;
	}
}

void arch_set_max_freq_scale(struct cpumask *cpus,
			     unsigned long policy_max_freq)
{
	unsigned long scale, max_freq;
	int cpu = cpumask_first(cpus);

	if (cpu > nr_cpu_ids)
		return;

	max_freq = per_cpu(max_cpu_freq, cpu);
	if (!max_freq)
		return;

	scale = (policy_max_freq << SCHED_CAPACITY_SHIFT) / max_freq;

	for_each_cpu(cpu, cpus)
		per_cpu(max_freq_scale, cpu) = scale;
}

DEFINE_PER_CPU(unsigned long, cpu_scale) = SCHED_CAPACITY_SCALE;

void topology_set_cpu_scale(unsigned int cpu, unsigned long capacity)
{
	per_cpu(cpu_scale, cpu) = capacity;
}

static ssize_t cpu_capacity_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);

	return sprintf(buf, "%lu\n", topology_get_cpu_scale(cpu->dev.id));
}

static void update_topology_flags_workfn(struct work_struct *work);
static DECLARE_WORK(update_topology_flags_work, update_topology_flags_workfn);

static DEVICE_ATTR_RO(cpu_capacity);

static int register_cpu_capacity_sysctl(void)
{
	int i;
	struct device *cpu;

	for_each_possible_cpu(i) {
		cpu = get_cpu_device(i);
		if (!cpu) {
			pr_err("%s: too early to get CPU%d device!\n",
			       __func__, i);
			continue;
		}
		device_create_file(cpu, &dev_attr_cpu_capacity);
	}

	return 0;
}
subsys_initcall(register_cpu_capacity_sysctl);

static int update_topology;

int topology_update_cpu_topology(void)
{
	return update_topology;
}

/*
 * Updating the sched_domains can't be done directly from cpufreq callbacks
 * due to locking, so queue the work for later.
 */
static void update_topology_flags_workfn(struct work_struct *work)
{
	update_topology = 1;
	rebuild_sched_domains();
	pr_debug("sched_domain hierarchy rebuilt, flags updated\n");
	update_topology = 0;
}

static u32 capacity_scale;
static u32 *raw_capacity;

static int free_raw_capacity(void)
{
	kfree(raw_capacity);
	raw_capacity = NULL;

	return 0;
}

void topology_normalize_cpu_scale(void)
{
	u64 capacity;
	int cpu;

	if (!raw_capacity)
		return;

	pr_debug("cpu_capacity: capacity_scale=%u\n", capacity_scale);
	for_each_possible_cpu(cpu) {
		pr_debug("cpu_capacity: cpu=%d raw_capacity=%u\n",
			 cpu, raw_capacity[cpu]);
		capacity = (raw_capacity[cpu] << SCHED_CAPACITY_SHIFT)
			/ capacity_scale;
		topology_set_cpu_scale(cpu, capacity);
		pr_debug("cpu_capacity: CPU%d cpu_capacity=%lu\n",
			cpu, topology_get_cpu_scale(cpu));
	}
}

bool __init topology_parse_cpu_capacity(struct device_node *cpu_node, int cpu)
{
	static bool cap_parsing_failed;
	int ret;
	u32 cpu_capacity;

	if (cap_parsing_failed)
		return false;

	ret = of_property_read_u32(cpu_node, "capacity-dmips-mhz",
				   &cpu_capacity);
	if (!ret) {
		if (!raw_capacity) {
			raw_capacity = kcalloc(num_possible_cpus(),
					       sizeof(*raw_capacity),
					       GFP_KERNEL);
			if (!raw_capacity) {
				cap_parsing_failed = true;
				return false;
			}
		}
		capacity_scale = max(cpu_capacity, capacity_scale);
		raw_capacity[cpu] = cpu_capacity;
		pr_debug("cpu_capacity: %pOF cpu_capacity=%u (raw)\n",
			cpu_node, raw_capacity[cpu]);
	} else {
		if (raw_capacity) {
			pr_err("cpu_capacity: missing %pOF raw capacity\n",
				cpu_node);
			pr_err("cpu_capacity: partial information: fallback to 1024 for all CPUs\n");
		}
		cap_parsing_failed = true;
		free_raw_capacity();
	}

	return !ret;
}

#ifdef CONFIG_CPU_FREQ
static cpumask_var_t cpus_to_visit;
static void parsing_done_workfn(struct work_struct *work);
static DECLARE_WORK(parsing_done_work, parsing_done_workfn);

static int
init_cpu_capacity_callback(struct notifier_block *nb,
			   unsigned long val,
			   void *data)
{
	struct cpufreq_policy *policy = data;
	int cpu;

	if (!raw_capacity)
		return 0;

	if (val != CPUFREQ_NOTIFY)
		return 0;

	pr_debug("cpu_capacity: init cpu capacity for CPUs [%*pbl] (to_visit=%*pbl)\n",
		 cpumask_pr_args(policy->related_cpus),
		 cpumask_pr_args(cpus_to_visit));

	cpumask_andnot(cpus_to_visit, cpus_to_visit, policy->related_cpus);

	for_each_cpu(cpu, policy->related_cpus) {
		raw_capacity[cpu] = topology_get_cpu_scale(cpu) *
				    policy->cpuinfo.max_freq / 1000UL;
		capacity_scale = max(raw_capacity[cpu], capacity_scale);
	}

	if (cpumask_empty(cpus_to_visit)) {
		topology_normalize_cpu_scale();
		schedule_work(&update_topology_flags_work);
		free_raw_capacity();
		pr_debug("cpu_capacity: parsing done\n");
		schedule_work(&parsing_done_work);
	}

	return 0;
}

static struct notifier_block init_cpu_capacity_notifier = {
	.notifier_call = init_cpu_capacity_callback,
};

static int __init register_cpufreq_notifier(void)
{
	int ret;

	/*
	 * on ACPI-based systems we need to use the default cpu capacity
	 * until we have the necessary code to parse the cpu capacity, so
	 * skip registering cpufreq notifier.
	 */
	if (!acpi_disabled || !raw_capacity)
		return -EINVAL;

	if (!alloc_cpumask_var(&cpus_to_visit, GFP_KERNEL))
		return -ENOMEM;

	cpumask_copy(cpus_to_visit, cpu_possible_mask);

	ret = cpufreq_register_notifier(&init_cpu_capacity_notifier,
					CPUFREQ_POLICY_NOTIFIER);

	if (ret)
		free_cpumask_var(cpus_to_visit);

	return ret;
}
core_initcall(register_cpufreq_notifier);

static void parsing_done_workfn(struct work_struct *work)
{
	cpufreq_unregister_notifier(&init_cpu_capacity_notifier,
					 CPUFREQ_POLICY_NOTIFIER);
	free_cpumask_var(cpus_to_visit);
}

#else
core_initcall(free_raw_capacity);
#endif

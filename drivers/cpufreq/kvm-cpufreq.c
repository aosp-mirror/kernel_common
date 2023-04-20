// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Google LLC
 */

#include <linux/arch_topology.h>
#include <linux/arm-smccc.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

static void kvm_scale_freq_tick(void)
{
	unsigned long scale, cur_freq, max_freq;
	struct arm_smccc_res hvc_res;

	arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_GET_CUR_CPUFREQ_FUNC_ID,
			0, &hvc_res);

	cur_freq = hvc_res.a0;
	max_freq = cpufreq_get_hw_max_freq(task_cpu(current));
	scale = (cur_freq << SCHED_CAPACITY_SHIFT) / max_freq;

	this_cpu_write(freq_scale, (unsigned long)scale);
}

static struct scale_freq_data kvm_sfd = {
	.source = SCALE_FREQ_SOURCE_ARCH,
	.set_freq_scale = kvm_scale_freq_tick,
};

struct remote_data {
	int ret;
	struct cpufreq_frequency_table *table;
};

static void remote_get_freqtbl_num_entries(void *data)
{
	struct arm_smccc_res hvc_res;
	u32 freq = 1UL;
	int *idx = data;

	while (freq != CPUFREQ_TABLE_END) {
		arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_GET_CPUFREQ_TBL_FUNC_ID,
				*idx, &hvc_res);
		if (hvc_res.a0) {
			*idx = -ENODEV;
			return;
		}
		freq = hvc_res.a1;
		(*idx)++;
	}
}

static int kvm_cpufreq_get_freqtbl_num_entries(int cpu)
{
	int num_entries = 0;

	smp_call_function_single(cpu, remote_get_freqtbl_num_entries, &num_entries, true);
	return num_entries;
}

static void remote_populate_freqtbl(void *data)
{
	struct arm_smccc_res hvc_res;
	struct remote_data *freq_data = data;
	struct cpufreq_frequency_table *pos;
	struct cpufreq_frequency_table *table = freq_data->table;
	int idx;

	cpufreq_for_each_entry_idx(pos, table, idx) {
		arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_GET_CPUFREQ_TBL_FUNC_ID,
				idx, &hvc_res);
		if (hvc_res.a0) {
			freq_data->ret = -ENODEV;
			return;
		}
		pos->frequency = hvc_res.a1;
	}
	freq_data->ret = 0;
}

static int kvm_cpufreq_populate_freqtbl(struct cpufreq_frequency_table	*table, int cpu)
{
	struct remote_data freq_data;

	freq_data.table = table;
	smp_call_function_single(cpu, remote_populate_freqtbl, &freq_data, true);
	return freq_data.ret;
}

static unsigned int kvm_cpufreq_setutil_hyp(struct cpufreq_policy *policy)
{
	struct arm_smccc_res hvc_res;
	u32 util = sched_cpu_util_freq(policy->cpu);
	u32 cap = arch_scale_cpu_capacity(policy->cpu);
	u32 threshold = cap - (cap >> 2);

	if (util > threshold)
		util = (cap + threshold) >> 1;

	arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_UTIL_HINT_FUNC_ID,
			     util, &hvc_res);

	return hvc_res.a0;
}

static unsigned int kvm_cpufreq_fast_switch(struct cpufreq_policy *policy,
		unsigned int target_freq)
{
	kvm_cpufreq_setutil_hyp(policy);
	return target_freq;
}

static int kvm_cpufreq_target_index(struct cpufreq_policy *policy,
		unsigned int index)
{
	return kvm_cpufreq_setutil_hyp(policy);
}

static const struct of_device_id kvm_cpufreq_match[] = {
	{ .compatible = "virtual,kvm-cpufreq"},
	{}
};
MODULE_DEVICE_TABLE(of, kvm_cpufreq_match);

static int kvm_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	struct device *cpu_dev;
	struct cpufreq_frequency_table	*table;
	int num_entries;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__,
		       policy->cpu);
		return -ENODEV;
	}

	num_entries = kvm_cpufreq_get_freqtbl_num_entries(policy->cpu);
	if (num_entries == -ENODEV)
		return -ENODEV;

	table = kcalloc(num_entries, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	table[num_entries-1].frequency = CPUFREQ_TABLE_END;

	if (kvm_cpufreq_populate_freqtbl(table, policy->cpu))
		return -ENODEV;

	policy->freq_table = table;
	policy->dvfs_possible_from_any_cpu = false;
	policy->fast_switch_possible = true;
	policy->transition_delay_us = 1;

	/*
	 * Only takes effect if another FIE source such as AMUs
	 * have not been registered.
	 */
	topology_set_scale_freq_source(&kvm_sfd, policy->cpus);

	return 0;
}

static int kvm_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	kfree(policy->freq_table);
	return 0;
}

static int kvm_cpufreq_online(struct cpufreq_policy *policy)
{
	/* Nothing to restore. */
	return 0;
}

static int kvm_cpufreq_offline(struct cpufreq_policy *policy)
{
	/* Empty offline() to avoid exit() being called and freeing resources. */
	return 0;
}

static struct cpufreq_driver cpufreq_kvm_driver = {
	.name		= "kvm-cpufreq",
	.init		= kvm_cpufreq_cpu_init,
	.exit		= kvm_cpufreq_cpu_exit,
	.online         = kvm_cpufreq_online,
	.offline        = kvm_cpufreq_offline,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= kvm_cpufreq_target_index,
	.fast_switch	= kvm_cpufreq_fast_switch,
	.attr		= cpufreq_generic_attr,
};

static int kvm_cpufreq_driver_probe(struct platform_device *pdev)
{
	int ret;

	ret = cpufreq_register_driver(&cpufreq_kvm_driver);
	if (ret) {
		dev_err(&pdev->dev, "KVM CPUFreq driver failed to register: %d\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "KVM CPUFreq driver initialized\n");
	return 0;
}

static int kvm_cpufreq_driver_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&cpufreq_kvm_driver);
	return 0;
}

static struct platform_driver kvm_cpufreq_driver = {
	.probe = kvm_cpufreq_driver_probe,
	.remove = kvm_cpufreq_driver_remove,
	.driver = {
		.name = "kvm-cpufreq",
		.of_match_table = kvm_cpufreq_match,
	},
};

static int __init kvm_cpufreq_init(void)
{
	return platform_driver_register(&kvm_cpufreq_driver);
}
postcore_initcall(kvm_cpufreq_init);

static void __exit kvm_cpufreq_exit(void)
{
	platform_driver_unregister(&kvm_cpufreq_driver);
}
module_exit(kvm_cpufreq_exit);

MODULE_DESCRIPTION("KVM cpufreq driver");
MODULE_LICENSE("GPL");

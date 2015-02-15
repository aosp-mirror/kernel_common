/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "pm.h"

struct msm_pm_time_stats {
	const char *name;
	int64_t first_bucket_time;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
	bool enabled;
};

struct msm_pm_cpu_time_stats {
	struct msm_pm_time_stats stats[MSM_PM_STAT_COUNT];
};

static struct msm_pm_time_stats suspend_stats;

static DEFINE_SPINLOCK(msm_pm_stats_lock);
static DEFINE_PER_CPU_SHARED_ALIGNED(
	struct msm_pm_cpu_time_stats, msm_pm_stats);
/*
 *  Function to update stats
 */
static void update_stats(struct msm_pm_time_stats *stats, int64_t t)
{
	int64_t bt;
	int i;

	if (!stats)
		return;

	stats->total_time += t;
	stats->count++;

	bt = t;
	do_div(bt, stats->first_bucket_time);

	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
			(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
			CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	if (i >= CONFIG_MSM_IDLE_STATS_BUCKET_COUNT)
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	stats->bucket[i]++;

	if (t < stats->min_time[i] || !stats->max_time[i])
		stats->min_time[i] = t;
	if (t > stats->max_time[i])
		stats->max_time[i] = t;
}

/*
 * Add the given time data to the statistics collection.
 */
void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	struct msm_pm_time_stats *stats;
	unsigned long flags;

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	if (id == MSM_PM_STAT_SUSPEND) {
		stats = &suspend_stats;
	} else {
		stats = __get_cpu_var(msm_pm_stats).stats;
		if (!stats[id].enabled)
			goto add_bail;
		stats = &stats[id];
	}
	update_stats(stats, t);
add_bail:
	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
}

static void stats_show(struct seq_file *m,
		struct msm_pm_time_stats *stats,
		int cpu, int suspend)
{
	int64_t bucket_time;
	int64_t s;
	uint32_t ns;
	int i;
	int bucket_count = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;
	int bucket_shift = CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;

	if (!stats || !m)
		return;

	s = stats->total_time;
	ns = do_div(s, NSEC_PER_SEC);
	if (suspend)
		seq_printf(m,
			"%s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			stats->name,
			stats->count,
			s, ns);
	else
		seq_printf(m,
			"[cpu %u] %s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			cpu, stats->name,
			stats->count,
			s, ns);

	bucket_time = stats->first_bucket_time;
	for (i = 0; i < bucket_count; i++) {
		s = bucket_time;
		ns = do_div(s, NSEC_PER_SEC);
		seq_printf(m,
			"   <%6lld.%09u: %7d (%lld-%lld)\n",
			s, ns, stats->bucket[i],
			stats->min_time[i],
			stats->max_time[i]);
			bucket_time <<= bucket_shift;
	}

	seq_printf(m, "  >=%6lld.%09u: %7d (%lld-%lld)\n",
		s, ns, stats->bucket[i],
		stats->min_time[i],
		stats->max_time[i]);
}
/*
 * Write out the power management statistics.
 */
static int msm_pm_stats_show(struct seq_file *m, void *v)
{
	int cpu;
	int id;
	unsigned long flags;

	spin_lock_irqsave(&msm_pm_stats_lock, flags);

	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats;

		stats = per_cpu(msm_pm_stats, cpu).stats;

		for (id = 0; id < MSM_PM_STAT_COUNT; id++) {
			/* Skip the disabled ones */
			if (!stats[id].enabled)
				continue;

			if (id == MSM_PM_STAT_SUSPEND)
				continue;

			stats_show(m, &stats[id], cpu, false);
		}
	}
	stats_show(m, &suspend_stats, cpu, true);
	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	return 0;
}

#define MSM_PM_STATS_RESET "reset"
/*
 * Reset the power management statistics values.
 */
static ssize_t msm_pm_write_proc(struct file *file, const char __user *buffer,
	size_t count, loff_t *off)
{
	char buf[sizeof(MSM_PM_STATS_RESET)];
	int ret;
	unsigned long flags;
	unsigned int cpu;
	size_t len = strnlen(MSM_PM_STATS_RESET, sizeof(MSM_PM_STATS_RESET));

	if (count < sizeof(MSM_PM_STATS_RESET)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, len)) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (strncmp(buf, MSM_PM_STATS_RESET, len)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	spin_lock_irqsave(&msm_pm_stats_lock, flags);
	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats;
		int i;

		stats = per_cpu(msm_pm_stats, cpu).stats;
		for (i = 0; i < MSM_PM_STAT_COUNT; i++) {
			memset(stats[i].bucket,
				0, sizeof(stats[i].bucket));
			memset(stats[i].min_time,
				0, sizeof(stats[i].min_time));
			memset(stats[i].max_time,
				0, sizeof(stats[i].max_time));
			stats[i].count = 0;
			stats[i].total_time = 0;
		}
	}
	memset(suspend_stats.bucket,
		0, sizeof(suspend_stats.bucket));
	memset(suspend_stats.min_time,
		0, sizeof(suspend_stats.min_time));
	memset(suspend_stats.max_time,
		0, sizeof(suspend_stats.max_time));
	suspend_stats.count = 0;
	suspend_stats.total_time = 0;

	spin_unlock_irqrestore(&msm_pm_stats_lock, flags);
	return count;

write_proc_failed:
	return ret;
}
#undef MSM_PM_STATS_RESET

static int msm_pm_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_pm_stats_show, NULL);
}

static const struct file_operations msm_pm_stats_fops = {
	.open		= msm_pm_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= msm_pm_write_proc,
};

void msm_pm_add_stats(enum msm_pm_time_stats_id *enable_stats, int size)
{
	unsigned int cpu;
	struct proc_dir_entry *d_entry;
	int i = 0;

	for_each_possible_cpu(cpu) {
		struct msm_pm_time_stats *stats =
			per_cpu(msm_pm_stats, cpu).stats;

		stats[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request";
		stats[MSM_PM_STAT_REQUESTED_IDLE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_SPIN].name = "idle-spin";
		stats[MSM_PM_STAT_IDLE_SPIN].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi";
		stats[MSM_PM_STAT_IDLE_WFI].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_RETENTION].name = "retention";
		stats[MSM_PM_STAT_RETENTION].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].name =
			"idle-standalone-power-collapse";
		stats[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].
			first_bucket_time = CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE].name =
			"idle-failed-standalone-power-collapse";
		stats[MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE].
			first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].name =
			"idle-power-collapse";
		stats[MSM_PM_STAT_IDLE_POWER_COLLAPSE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE].name =
			"idle-failed-power-collapse";
		stats[MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE].
			first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		stats[MSM_PM_STAT_NOT_IDLE].name = "not-idle";
		stats[MSM_PM_STAT_NOT_IDLE].first_bucket_time =
			CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;

		for (i = 0; i < size; i++)
			stats[enable_stats[i]].enabled = true;

	}
	suspend_stats.name = "system_suspend";
	suspend_stats.first_bucket_time =
		CONFIG_MSM_SUSPEND_STATS_FIRST_BUCKET;

	d_entry = proc_create_data("msm_pm_stats", S_IRUGO | S_IWUSR | S_IWGRP,
			NULL, &msm_pm_stats_fops, NULL);
}

/* arch/arm/mach-msm/perflock.c
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <mach/perflock.h>
#include "acpuclock.h"

#define PERF_LOCK_INITIALIZED	(1U << 0)
#define PERF_LOCK_ACTIVE	(1U << 1)

enum {
	PERF_LOCK_DEBUG = 1U << 0,
	PERF_EXPIRE_DEBUG = 1U << 1,
	PERF_CPUFREQ_NOTIFY_DEBUG = 1U << 2,
	PERF_CPUFREQ_LOCK_DEBUG = 1U << 3,
	PERF_SCREEN_ON_POLICY_DEBUG = 1U << 4,
};

static LIST_HEAD(active_perf_locks);
static LIST_HEAD(inactive_perf_locks);
static LIST_HEAD(active_cpufreq_ceiling_locks);
static LIST_HEAD(inactive_cpufreq_ceiling_locks);
static DEFINE_SPINLOCK(list_lock);
static int initialized;
static int cpufreq_ceiling_initialized;
static unsigned int *perf_acpu_table;
static unsigned int *cpufreq_ceiling_acpu_table;
static unsigned int table_size;
static struct workqueue_struct *perflock_setrate_workqueue;


#ifdef CONFIG_PERF_LOCK_DEBUG
static int debug_mask = PERF_LOCK_DEBUG | PERF_EXPIRE_DEBUG |
	PERF_CPUFREQ_NOTIFY_DEBUG | PERF_CPUFREQ_LOCK_DEBUG;
#else
static int debug_mask = 0;
#endif

static struct kernel_param_ops param_ops_str = {
	.set = param_set_int,
	.get = param_get_int,
};

module_param_cb(debug_mask, &param_ops_str, &debug_mask, S_IWUSR | S_IRUGO);

extern struct kobject *cpufreq_kobj;

static unsigned int get_perflock_speed(void);
static unsigned int get_cpufreq_ceiling_speed(void);

static unsigned int policy_min;
static unsigned int policy_max;

static int param_set_cpu_min_max(const char *val, struct kernel_param *kp)
{
	int ret;
	int cpu;
	int ret2;
	ret = param_set_int(val, kp);
	for_each_online_cpu(cpu) {
	pr_info("%s: cpufreq update cpu:  %d\n", __func__, cpu);
		ret2 = cpufreq_update_policy(cpu);
	pr_info("cpu: %d , ret: %d\n", cpu, ret2);
	}
	return ret;
}

module_param_call(min_cpu_khz, param_set_cpu_min_max, param_get_int,
	&policy_min, S_IWUSR | S_IRUGO);
module_param_call(max_cpu_khz, param_set_cpu_min_max, param_get_int,
	&policy_max, S_IWUSR | S_IRUGO);

static DEFINE_PER_CPU(int, stored_policy_min);
static DEFINE_PER_CPU(int, stored_policy_max);

void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu)
{
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s, freq=%u, cpu%u\n", __func__, freq, cpu);
	per_cpu(stored_policy_max, cpu) = freq;
}

void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu)
{
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s, freq=%u, cpu%u\n", __func__, freq, cpu);
	per_cpu(stored_policy_min, cpu) = freq;
}

static unsigned int get_perflock_speed(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;
	unsigned int perf_level = 0;

	/* Get the maxmimum perf level. */
	if (list_empty(&active_perf_locks))
		return 0;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		if (lock->level > perf_level)
			perf_level = lock->level;
	}
	spin_unlock_irqrestore(&list_lock, irqflags);

	return perf_acpu_table[perf_level];
}

static unsigned int get_cpufreq_ceiling_speed(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;
	unsigned int perf_level = PERF_LOCK_HIGHEST;

	/* Get the minimum ceiling level. */
	if (list_empty(&active_cpufreq_ceiling_locks))
		return 0;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_cpufreq_ceiling_locks, link) {
		if (lock->level < perf_level)
			perf_level = lock->level;
	}
	spin_unlock_irqrestore(&list_lock, irqflags);

	return cpufreq_ceiling_acpu_table[perf_level];
}

void htc_print_active_perf_locks(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;

	spin_lock_irqsave(&list_lock, irqflags);
	if (!list_empty(&active_perf_locks)) {
		pr_info("perf_lock:");
		list_for_each_entry(lock, &active_perf_locks, link) {
			pr_info(" '%s' ", lock->name);
		}
		pr_info("\n");
	}
	if (!list_empty(&active_cpufreq_ceiling_locks)) {
		printk(KERN_WARNING"ceiling_lock:");
		list_for_each_entry(lock, &active_cpufreq_ceiling_locks, link) {
			printk(KERN_WARNING" '%s' ", lock->name);
		}
		pr_info("\n");
	}
	spin_unlock_irqrestore(&list_lock, irqflags);
}

/**
 * perf_lock_init - acquire a perf lock
 * @lock: perf lock to acquire
 * @type: the type of @lock
 * @level: performance level of @lock
 * @name: the name of @lock
 *
 * Acquire @lock with @name and @level. (It doesn't activate the lock.)
 */
void perf_lock_init(struct perf_lock *lock, unsigned int type,
			unsigned int level, const char *name)
{
	unsigned long irqflags = 0;

	WARN_ON(!lock);
	WARN_ON(!name);
	WARN_ON(level >= PERF_LOCK_INVALID);
	WARN_ON(lock->flags & PERF_LOCK_INITIALIZED);

	if ((!name) || (level >= PERF_LOCK_INVALID) ||
			(lock->flags & PERF_LOCK_INITIALIZED)) {
		pr_err("%s: ERROR \"%s\" flags %x level %d\n",
			__func__, name, lock->flags, level);
		return;
	}
	lock->name = name;
	lock->flags = PERF_LOCK_INITIALIZED;
	lock->level = level;
	lock->type = type;

	INIT_LIST_HEAD(&lock->link);
	spin_lock_irqsave(&list_lock, irqflags);
	if (lock->type == TYPE_PERF_LOCK)
		list_add(&lock->link, &inactive_perf_locks);
	if (lock->type == TYPE_CPUFREQ_CEILING)
		list_add(&lock->link, &inactive_cpufreq_ceiling_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);
}
EXPORT_SYMBOL(perf_lock_init);

/**
 * perf_lock - activate a perf lock
 * @lock: perf lock to activate
 *
 * Activate @lock.(Need to init_perf_lock before activate)
 */

void perf_lock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON((lock->flags & PERF_LOCK_INITIALIZED) == 0);
	WARN_ON(lock->flags & PERF_LOCK_ACTIVE);
	if (lock->type == TYPE_PERF_LOCK) {
		WARN_ON(!initialized);
		if (!initialized) {
			if (debug_mask & PERF_LOCK_DEBUG)
				pr_info("%s exit because perflock is not initialized\n", __func__);
			return;
		}
	} else if (lock->type == TYPE_CPUFREQ_CEILING) {
		WARN_ON(!cpufreq_ceiling_initialized);
		if (!cpufreq_ceiling_initialized) {
			if (debug_mask & PERF_LOCK_DEBUG)
				pr_info("%s exit because cpufreq_ceiling is not initialized\n", __func__);
			return;
		}
	}

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d type %u\n",
			__func__, lock->name, lock->flags, lock->level, lock->type);
	if (lock->flags & PERF_LOCK_ACTIVE) {
		pr_err("%s:type(%u) over-locked\n", __func__, lock->type);
		spin_unlock_irqrestore(&list_lock, irqflags);
		return;
	}
	lock->flags |= PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	if (lock->type == TYPE_PERF_LOCK)
		list_add(&lock->link, &active_perf_locks);
	else if (lock->type == TYPE_CPUFREQ_CEILING)
		list_add(&lock->link, &active_cpufreq_ceiling_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

    if (lock->type == TYPE_PERF_LOCK)
		sysfs_notify(cpufreq_kobj, NULL, "perflock_scaling_min");
	else if (lock->type == TYPE_CPUFREQ_CEILING)
		sysfs_notify(cpufreq_kobj, NULL, "perflock_scaling_max");

	return;
}
EXPORT_SYMBOL(perf_lock);

/**
 * perf_unlock - de-activate a perf lock
 * @lock: perf lock to de-activate
 *
 * de-activate @lock.
 */
void perf_unlock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON(!initialized);
	WARN_ON((lock->flags & PERF_LOCK_ACTIVE) == 0);
	if (lock->type == TYPE_PERF_LOCK) {
		WARN_ON(!initialized);
		if (!initialized) {
			if (debug_mask & PERF_LOCK_DEBUG)
				pr_info("%s exit because perflock is not initialized\n", __func__);
			return;
		}
	}
	if (lock->type == TYPE_CPUFREQ_CEILING) {
		WARN_ON(!cpufreq_ceiling_initialized);
		if (!cpufreq_ceiling_initialized) {
			if (debug_mask & PERF_LOCK_DEBUG)
				pr_info("%s exit because cpufreq_ceiling is not initialized\n", __func__);
			return;
		}
	}

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d\n",
			__func__, lock->name, lock->flags, lock->level);
	if (!(lock->flags & PERF_LOCK_ACTIVE)) {
		pr_err("%s: under-locked\n", __func__);
		spin_unlock_irqrestore(&list_lock, irqflags);
		return;
	}
	lock->flags &= ~PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	if (lock->type == TYPE_PERF_LOCK)
		list_add(&lock->link, &inactive_perf_locks);
	else if (lock->type == TYPE_CPUFREQ_CEILING)
		list_add(&lock->link, &inactive_cpufreq_ceiling_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

	if (lock->type == TYPE_PERF_LOCK)
		sysfs_notify(cpufreq_kobj, NULL, "perflock_scaling_min");
	else if (lock->type == TYPE_CPUFREQ_CEILING)
		sysfs_notify(cpufreq_kobj, NULL, "perflock_scaling_max");
}
EXPORT_SYMBOL(perf_unlock);

/**
 * is_perf_lock_active - query if a perf_lock is active or not
 * @lock: target perf lock
 * RETURN: 0: inactive; 1: active
 *
 * query if @lock is active or not
 */
inline int is_perf_lock_active(struct perf_lock *lock)
{
	return (lock->flags & PERF_LOCK_ACTIVE);
}
EXPORT_SYMBOL(is_perf_lock_active);

/**
 * is_perf_locked - query if there is any perf lock activates
 * RETURN: 0: no perf lock activates 1: at least a perf lock activates
 */
int is_perf_locked(void)
{
	return (!list_empty(&active_perf_locks));
}
EXPORT_SYMBOL(is_perf_locked);

/**
 * perflock_find - find perflock by given name
 * INPUT: name   - string name of the perflock
 * RETURN: lock  - pointer of the perflock
 */
static struct perf_lock *perflock_find(const char *name)
{
	struct perf_lock *lock;
	unsigned long irqflags;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		if(!strcmp(lock->name, name)) {
			spin_unlock_irqrestore(&list_lock, irqflags);
			return lock;
		}
	}
	list_for_each_entry(lock, &inactive_perf_locks, link) {
		if(!strcmp(lock->name, name)) {
			spin_unlock_irqrestore(&list_lock, irqflags);
			return lock;
		}
	}
	list_for_each_entry(lock, &active_cpufreq_ceiling_locks, link) {
		if(!strcmp(lock->name, name)) {
			spin_unlock_irqrestore(&list_lock, irqflags);
			return lock;
		}
	}
	list_for_each_entry(lock, &inactive_cpufreq_ceiling_locks, link) {
		if(!strcmp(lock->name, name)) {
			spin_unlock_irqrestore(&list_lock, irqflags);
			return lock;
		}
	}
	spin_unlock_irqrestore(&list_lock, irqflags);

	return NULL;
}

/**
 * perflock_acquire - acquire a perflock
 * INPUT: name      - string name of this perflock
 * RETURN: lock     - pointer of this perf_lock
 */
struct perf_lock *perflock_acquire(const char *name)
{
	struct perf_lock *lock = NULL;

	lock = perflock_find(name);
	if(lock)
		return lock;

	lock = kzalloc(sizeof(struct perf_lock), GFP_KERNEL);
	if(!lock) {
		pr_err("%s: fail to alloc perflock %s\n", __func__, name);
		return NULL; //ENOMEM
	}
	lock->name = name;
	/* Caller MUST init this lock before using it!! */
	lock->flags = 0; //None-INITIALIZED

	return lock;
}
EXPORT_SYMBOL(perflock_acquire);

/**
 * perflock_release - release a perflock
 * INPUT: name - the string name of this perf_lock
 * RETURN: 0   - successful
 *     -ENOMEM - no memory ??
 */
int perflock_release(const char *name)
{
	struct perf_lock *lock = NULL;
	unsigned long irqflags;

	lock = perflock_find(name);
	if(!lock)
		return -ENODEV;

	/* Unlock (move to inactive list), delete and kfree it */
	if(is_perf_lock_active(lock))
		perf_unlock(lock);

	spin_lock_irqsave(&list_lock, irqflags);
	list_del(&lock->link);
	spin_unlock_irqrestore(&list_lock, irqflags);
	kfree(lock);
	return 0;
}
EXPORT_SYMBOL(perflock_release);

static void perf_acpu_table_fixup(void)
{
	int i;
	for (i = 0; i < table_size; ++i) {
		if (perf_acpu_table[i] > policy_max * 1000)
			perf_acpu_table[i] = policy_max * 1000;
		else if (perf_acpu_table[i] < policy_min * 1000)
			perf_acpu_table[i] = policy_min * 1000;
	}

#ifdef PERFLOCK_FIX_UP
	if (table_size >= 1)
		if (perf_acpu_table[table_size - 1] < policy_max * 1000)
			perf_acpu_table[table_size - 1] = policy_max * 1000;
#endif
}

static void cpufreq_ceiling_acpu_table_fixup(void)
{
	int i;
	for (i = 0; i < table_size; ++i) {
		if (cpufreq_ceiling_acpu_table[i] > policy_max * 1000)
			cpufreq_ceiling_acpu_table[i] = policy_max * 1000;
		else if (cpufreq_ceiling_acpu_table[i] < policy_min * 1000)
			cpufreq_ceiling_acpu_table[i] = policy_min * 1000;
	}
}

static void perflock_floor_init(struct perflock_data *pdata)
{
	struct cpufreq_policy policy;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	BUG_ON(cpufreq_frequency_table_cpuinfo(&policy, table));
	policy_min = policy.cpuinfo.min_freq;
	policy_max = policy.cpuinfo.max_freq;

	if (!pdata)
		goto invalid_config;

	perf_acpu_table = pdata->perf_acpu_table;
	table_size = pdata->table_size;
	if (!perf_acpu_table || !table_size)
		goto invalid_config;
	if (table_size < PERF_LOCK_INVALID)
		goto invalid_config;

	perf_acpu_table_fixup();
	perflock_setrate_workqueue = create_workqueue("perflock_setrate_wq");

	initialized = 1;
	pr_info("perflock floor init done\n");

	return;

invalid_config:
	pr_err("%s: invalid configuration data, %p %d %d\n", __func__,
		perf_acpu_table, table_size, PERF_LOCK_INVALID);
}

static void cpufreq_ceiling_init(struct perflock_data *pdata)
{
	struct cpufreq_policy policy;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	BUG_ON(cpufreq_frequency_table_cpuinfo(&policy, table));
	policy_min = policy.cpuinfo.min_freq;
	policy_max = policy.cpuinfo.max_freq;

	if (!pdata)
		goto invalid_config;

	cpufreq_ceiling_acpu_table = pdata->perf_acpu_table;
	table_size = pdata->table_size;
	if (!cpufreq_ceiling_acpu_table || !table_size)
		goto invalid_config;
	if (table_size < PERF_LOCK_INVALID)
		goto invalid_config;

	cpufreq_ceiling_acpu_table_fixup();

	cpufreq_ceiling_initialized = 1;
	pr_info("perflock ceiling init done\n");
	return;

invalid_config:
	pr_err("%s: invalid configuration data, %p %d %d\n", __func__,
		cpufreq_ceiling_acpu_table, table_size, PERF_LOCK_INVALID);
}

static int perf_lock_probe(struct platform_device *pdev)
{
	struct perflock_pdata *pdata = pdev->dev.platform_data;
	pr_info("perflock probe\n");
	if(!pdata->perf_floor && !pdata->perf_ceiling) {
		printk(KERN_INFO "perf_lock Not Initialized\n");
		return -ENODEV;
	}
	if(pdata->perf_floor) {
		perflock_floor_init(pdata->perf_floor);

	}
	if(pdata->perf_ceiling) {
		cpufreq_ceiling_init(pdata->perf_ceiling);
	}
	return 0;
}

static int perf_lock_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver perf_lock_driver = {
	.probe = perf_lock_probe,
	.remove = perf_lock_remove,
	.driver = {
		.name = "perf_lock",
		.owner = THIS_MODULE,
	},
};

static int init_perf_lock(void)
{
	return platform_driver_register(&perf_lock_driver);
}

late_initcall(init_perf_lock);

ssize_t
perflock_scaling_max_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u", get_cpufreq_ceiling_speed() / 1000);
	return ret;
}
ssize_t
perflock_scaling_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}
ssize_t
perflock_scaling_min_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u", get_perflock_speed() / 1000);
	return ret;
}

ssize_t
perflock_scaling_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}

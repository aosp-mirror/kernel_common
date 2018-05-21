// SPDX-License-Identifier: GPL-2.0
/*
 * Energy Model of CPUs
 *
 * Copyright (c) 2018, Arm ltd.
 * Written by: Quentin Perret, Arm ltd.
 */

#define pr_fmt(fmt) "energy_model: " fmt

#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/energy_model.h>
#include <linux/sched/topology.h>

/* Mapping of each CPU to the frequency domain to which it belongs. */
static DEFINE_PER_CPU(struct em_freq_domain *, em_data);

/*
 * Protects the access to em_data. Readers of em_data can be in RCU-critical
 * sections, and can't afford to sleep.
 */
static DEFINE_RWLOCK(em_data_lock);

/*
 * Mutex serializing the registrations of frequency domains. It allows the
 * callbacks defined by drivers to sleep.
 */
static DEFINE_MUTEX(em_fd_mutex);

static struct kobject *em_kobject;

/* Getters for the attributes of em_freq_domain objects */
struct em_fd_attr {
	struct attribute attr;
	ssize_t (*show)(struct em_freq_domain *fd, char *buf);
	ssize_t (*store)(struct em_freq_domain *fd, const char *buf, size_t s);
};

#define EM_ATTR_LEN 13
#define show_table_attr(_attr) \
static ssize_t show_##_attr(struct em_freq_domain *fd, char *buf) \
{ \
	ssize_t cnt = 0; \
	int i; \
	struct em_cs_table *table; \
	rcu_read_lock(); \
	table = rcu_dereference(fd->cs_table);\
	for (i = 0; i < table->nr_cap_states; i++) { \
		if (cnt >= (ssize_t) (PAGE_SIZE / sizeof(char) \
				      - (EM_ATTR_LEN + 2))) \
			goto out; \
		cnt += scnprintf(&buf[cnt], EM_ATTR_LEN + 1, "%lu ", \
				 table->state[i]._attr); \
	} \
out: \
	rcu_read_unlock(); \
	cnt += sprintf(&buf[cnt], "\n"); \
	return cnt; \
}

show_table_attr(power);
show_table_attr(frequency);
show_table_attr(capacity);

static ssize_t show_cpus(struct em_freq_domain *fd, char *buf)
{
	return sprintf(buf, "%*pbl\n", cpumask_pr_args(&fd->cpus));
}

#define fd_attr(_name) em_fd_##_name##_attr
#define define_fd_attr(_name) static struct em_fd_attr fd_attr(_name) = \
		__ATTR(_name, 0444, show_##_name, NULL)

define_fd_attr(power);
define_fd_attr(frequency);
define_fd_attr(capacity);
define_fd_attr(cpus);

static struct attribute *em_fd_default_attrs[] = {
	&fd_attr(power).attr,
	&fd_attr(frequency).attr,
	&fd_attr(capacity).attr,
	&fd_attr(cpus).attr,
	NULL
};

#define to_fd(k) container_of(k, struct em_freq_domain, kobj)
#define to_fd_attr(a) container_of(a, struct em_fd_attr, attr)

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct em_freq_domain *fd = to_fd(kobj);
	struct em_fd_attr *fd_attr = to_fd_attr(attr);
	ssize_t ret;

	ret = fd_attr->show(fd, buf);

	return ret;
}

static const struct sysfs_ops em_fd_sysfs_ops = {
	.show	= show,
};

static struct kobj_type ktype_em_fd = {
	.sysfs_ops	= &em_fd_sysfs_ops,
	.default_attrs	= em_fd_default_attrs,
};

static struct em_cs_table *alloc_cs_table(int nr_states)
{
	struct em_cs_table *cs_table;

	cs_table = kzalloc(sizeof(*cs_table), GFP_NOWAIT);
	if (!cs_table)
		return NULL;

	cs_table->state = kcalloc(nr_states, sizeof(*cs_table->state),
								GFP_NOWAIT);
	if (!cs_table->state) {
		kfree(cs_table);
		return NULL;
	}

	cs_table->nr_cap_states = nr_states;

	return cs_table;
}

static void free_cs_table(struct em_cs_table *table)
{
	if (table) {
		kfree(table->state);
		kfree(table);
	}
}

static void fd_update_cs_table(struct em_cs_table *cs_table, int cpu)
{
	unsigned long cmax = arch_scale_cpu_capacity(NULL, cpu);
	int max_cap_state = cs_table->nr_cap_states - 1;
	unsigned long fmax = cs_table->state[max_cap_state].frequency;
	int i;

	for (i = 0; i < cs_table->nr_cap_states; i++)
		cs_table->state[i].capacity = cmax *
					cs_table->state[i].frequency / fmax;
}

static struct em_freq_domain *em_create_fd(cpumask_t *span, int nr_states,
						struct em_data_callback *cb)
{
	unsigned long opp_eff, prev_opp_eff = ULONG_MAX;
	int i, ret, cpu = cpumask_first(span);
	struct em_freq_domain *fd;
	unsigned long power, freq;

	if (!cb->active_power)
		return NULL;

	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return NULL;

	fd->cs_table = alloc_cs_table(nr_states);
	if (!fd->cs_table)
		goto free_fd;

	/* Copy the span of the frequency domain */
	cpumask_copy(&fd->cpus, span);

	/* Build the list of capacity states for this freq domain */
	for (i = 0, freq = 0; i < nr_states; i++, freq++) {
		ret = cb->active_power(&power, &freq, cpu);
		if (ret)
			goto free_cs_table;

		fd->cs_table->state[i].power = power;
		fd->cs_table->state[i].frequency = freq;

		/*
		 * The hertz/watts efficiency ratio should decrease as the
		 * frequency grows on sane platforms. If not, warn the user
		 * that some high OPPs are more power efficient than some
		 * of the lower ones.
		 */
		opp_eff = freq / power;
		if (opp_eff >= prev_opp_eff)
			pr_warn("%*pbl: hz/watt efficiency: OPP %d >= OPP%d\n",
					cpumask_pr_args(span), i, i - 1);
		prev_opp_eff = opp_eff;
	}
	fd_update_cs_table(fd->cs_table, cpu);

	ret = kobject_init_and_add(&fd->kobj, &ktype_em_fd, em_kobject, "fd%u",
									cpu);
	if (ret)
		pr_warn("%*pbl: failed kobject init\n", cpumask_pr_args(span));

	return fd;

free_cs_table:
	free_cs_table(fd->cs_table);
free_fd:
	kfree(fd);

	return NULL;
}

static void rcu_free_cs_table(struct rcu_head *rp)
{
	struct em_cs_table *table;

	table = container_of(rp, struct em_cs_table, rcu);
	free_cs_table(table);
}

/**
 * em_rescale_cpu_capacity() - Re-scale capacity values of the Energy Model
 *
 * This re-scales the capacity values for all capacity states of all frequency
 * domains of the Energy Model. This should be used when the capacity values
 * of the CPUs are updated at run-time, after the EM was registered.
 */
void em_rescale_cpu_capacity(void)
{
	struct em_cs_table *old_table, *new_table;
	struct em_freq_domain *fd;
	unsigned long flags;
	int nr_states, cpu;

	read_lock_irqsave(&em_data_lock, flags);
	for_each_cpu(cpu, cpu_possible_mask) {
		fd = per_cpu(em_data, cpu);
		if (!fd || cpu != cpumask_first(&fd->cpus))
			continue;

		/* Copy the existing table. */
		old_table = rcu_dereference(fd->cs_table);
		nr_states = old_table->nr_cap_states;
		new_table = alloc_cs_table(nr_states);
		if (!new_table) {
			read_unlock_irqrestore(&em_data_lock, flags);
			return;
		}
		memcpy(new_table->state, old_table->state,
					nr_states * sizeof(*new_table->state));

		/* Re-scale the capacity values on the copy. */
		fd_update_cs_table(new_table, cpumask_first(&fd->cpus));

		/* Replace the table with the rescaled version. */
		rcu_assign_pointer(fd->cs_table, new_table);
		call_rcu(&old_table->rcu, rcu_free_cs_table);
	}
	read_unlock_irqrestore(&em_data_lock, flags);
	pr_debug("Re-scaled CPU capacities\n");
}
EXPORT_SYMBOL_GPL(em_rescale_cpu_capacity);

/**
 * em_cpu_get() - Return the frequency domain for a CPU
 * @cpu : CPU to find the frequency domain for
 *
 * Return: the frequency domain to which 'cpu' belongs, or NULL if it doesn't
 * exist.
 */
struct em_freq_domain *em_cpu_get(int cpu)
{
	struct em_freq_domain *fd;
	unsigned long flags;

	read_lock_irqsave(&em_data_lock, flags);
	fd = per_cpu(em_data, cpu);
	read_unlock_irqrestore(&em_data_lock, flags);

	return fd;
}
EXPORT_SYMBOL_GPL(em_cpu_get);

/**
 * em_register_freq_domain() - Register the Energy Model of a frequency domain
 * @span	: Mask of CPUs in the frequency domain
 * @nr_states	: Number of capacity states to register
 * @cb		: Callback functions providing the data of the Energy Model
 *
 * Create Energy Model tables for a frequency domain using the callbacks
 * defined in cb.
 *
 * If multiple clients register the same frequency domain, all but the first
 * registration will be ignored.
 *
 * Return 0 on success
 */
int em_register_freq_domain(cpumask_t *span, unsigned int nr_states,
						struct em_data_callback *cb)
{
	struct em_freq_domain *fd;
	unsigned long flags;
	int cpu, ret = 0;

	if (!span || !nr_states || !cb)
		return -EINVAL;

	mutex_lock(&em_fd_mutex);

	if (!em_kobject) {
		em_kobject = kobject_create_and_add("energy_model",
						&cpu_subsys.dev_root->kobj);
		if (!em_kobject) {
			ret = -ENODEV;
			goto unlock;
		}
	}

	/* Make sure we don't register again an existing domain. */
	for_each_cpu(cpu, span) {
		if (per_cpu(em_data, cpu)) {
			ret = -EEXIST;
			goto unlock;
		}
	}

	/* Create the frequency domain and add it to the Energy Model. */
	fd = em_create_fd(span, nr_states, cb);
	if (!fd) {
		ret = -EINVAL;
		goto unlock;
	}

	write_lock_irqsave(&em_data_lock, flags);
	for_each_cpu(cpu, span)
		per_cpu(em_data, cpu) = fd;
	write_unlock_irqrestore(&em_data_lock, flags);

	pr_debug("Created freq domain %*pbl\n", cpumask_pr_args(span));
unlock:
	mutex_unlock(&em_fd_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(em_register_freq_domain);

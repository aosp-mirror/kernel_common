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

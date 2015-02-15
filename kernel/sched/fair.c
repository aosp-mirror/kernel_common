/*
 * Completely Fair Scheduling (CFS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2007 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 *  Interactivity improvements by Mike Galbraith
 *  (C) 2007 Mike Galbraith <efault@gmx.de>
 *
 *  Various enhancements by Dmitry Adamushko.
 *  (C) 2007 Dmitry Adamushko <dmitry.adamushko@gmail.com>
 *
 *  Group scheduling enhancements by Srivatsa Vaddagiri
 *  Copyright IBM Corporation, 2007
 *  Author: Srivatsa Vaddagiri <vatsa@linux.vnet.ibm.com>
 *
 *  Scaled math optimizations by Thomas Gleixner
 *  Copyright (C) 2007, Thomas Gleixner <tglx@linutronix.de>
 *
 *  Adaptive scheduling granularity, math enhancements by Peter Zijlstra
 *  Copyright (C) 2007 Red Hat, Inc., Peter Zijlstra <pzijlstr@redhat.com>
 */

#include <linux/latencytop.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/profile.h>
#include <linux/interrupt.h>

#include <trace/events/sched.h>

#include "sched.h"

/*
 * Targeted preemption latency for CPU-bound tasks:
 * (default: 6ms * (1 + ilog(ncpus)), units: nanoseconds)
 *
 * NOTE: this latency value is not the same as the concept of
 * 'timeslice length' - timeslices in CFS are of variable length
 * and have no persistent notion like in traditional, time-slice
 * based scheduling concepts.
 *
 * (to see the precise effective timeslice length of your workload,
 *  run vmstat and monitor the context-switches (cs) field)
 */
unsigned int sysctl_sched_latency = 6000000ULL;
unsigned int normalized_sysctl_sched_latency = 6000000ULL;

/*
 * The initial- and re-scaling of tunables is configurable
 * (default SCHED_TUNABLESCALING_LOG = *(1+ilog(ncpus))
 *
 * Options are:
 * SCHED_TUNABLESCALING_NONE - unscaled, always *1
 * SCHED_TUNABLESCALING_LOG - scaled logarithmical, *1+ilog(ncpus)
 * SCHED_TUNABLESCALING_LINEAR - scaled linear, *ncpus
 */
enum sched_tunable_scaling sysctl_sched_tunable_scaling
	= SCHED_TUNABLESCALING_LOG;

/*
 * Minimal preemption granularity for CPU-bound tasks:
 * (default: 0.75 msec * (1 + ilog(ncpus)), units: nanoseconds)
 */
unsigned int sysctl_sched_min_granularity = 750000ULL;
unsigned int normalized_sysctl_sched_min_granularity = 750000ULL;

/*
 * is kept at sysctl_sched_latency / sysctl_sched_min_granularity
 */
static unsigned int sched_nr_latency = 8;

/*
 * After fork, child runs first. If set to 0 (default) then
 * parent will (try to) run first.
 */
unsigned int sysctl_sched_child_runs_first __read_mostly;

/*
 * Controls whether, when SD_SHARE_PKG_RESOURCES is on, if all
 * tasks go to idle CPUs when woken. If this is off, note that the
 * per-task flag PF_WAKE_ON_IDLE can still cause a task to go to an
 * idle CPU upon being woken.
 */
unsigned int __read_mostly sysctl_sched_wake_to_idle;

/*
 * SCHED_OTHER wake-up granularity.
 * (default: 1 msec * (1 + ilog(ncpus)), units: nanoseconds)
 *
 * This option delays the preemption effects of decoupled workloads
 * and reduces their over-scheduling. Synchronous workloads will still
 * have immediate wakeup/sleep latencies.
 */
unsigned int sysctl_sched_wakeup_granularity = 1000000UL;
unsigned int normalized_sysctl_sched_wakeup_granularity = 1000000UL;

const_debug unsigned int sysctl_sched_migration_cost = 500000UL;

/*
 * The exponential sliding  window over which load is averaged for shares
 * distribution.
 * (default: 10msec)
 */
unsigned int __read_mostly sysctl_sched_shares_window = 10000000UL;

#ifdef CONFIG_CFS_BANDWIDTH
/*
 * Amount of runtime to allocate from global (tg) to local (per-cfs_rq) pool
 * each time a cfs_rq requests quota.
 *
 * Note: in the case that the slice exceeds the runtime remaining (either due
 * to consumption or the quota being specified to be smaller than the slice)
 * we will always only issue the remaining available time.
 *
 * default: 5 msec, units: microseconds
  */
unsigned int sysctl_sched_cfs_bandwidth_slice = 5000UL;
#endif

/*
 * Increase the granularity value when there are more CPUs,
 * because with more CPUs the 'effective latency' as visible
 * to users decreases. But the relationship is not linear,
 * so pick a second-best guess by going with the log2 of the
 * number of CPUs.
 *
 * This idea comes from the SD scheduler of Con Kolivas:
 */
static int get_update_sysctl_factor(void)
{
	unsigned int cpus = min_t(int, num_online_cpus(), 8);
	unsigned int factor;

	switch (sysctl_sched_tunable_scaling) {
	case SCHED_TUNABLESCALING_NONE:
		factor = 1;
		break;
	case SCHED_TUNABLESCALING_LINEAR:
		factor = cpus;
		break;
	case SCHED_TUNABLESCALING_LOG:
	default:
		factor = 1 + ilog2(cpus);
		break;
	}

	return factor;
}

static void update_sysctl(void)
{
	unsigned int factor = get_update_sysctl_factor();

#define SET_SYSCTL(name) \
	(sysctl_##name = (factor) * normalized_sysctl_##name)
	SET_SYSCTL(sched_min_granularity);
	SET_SYSCTL(sched_latency);
	SET_SYSCTL(sched_wakeup_granularity);
#undef SET_SYSCTL
}

void sched_init_granularity(void)
{
	update_sysctl();
}

#if BITS_PER_LONG == 32
# define WMULT_CONST	(~0UL)
#else
# define WMULT_CONST	(1UL << 32)
#endif

#define WMULT_SHIFT	32

/*
 * Shift right and round:
 */
#define SRR(x, y) (((x) + (1UL << ((y) - 1))) >> (y))

/*
 * delta *= weight / lw
 */
static unsigned long
calc_delta_mine(unsigned long delta_exec, unsigned long weight,
		struct load_weight *lw)
{
	u64 tmp;

	/*
	 * weight can be less than 2^SCHED_LOAD_RESOLUTION for task group sched
	 * entities since MIN_SHARES = 2. Treat weight as 1 if less than
	 * 2^SCHED_LOAD_RESOLUTION.
	 */
	if (likely(weight > (1UL << SCHED_LOAD_RESOLUTION)))
		tmp = (u64)delta_exec * scale_load_down(weight);
	else
		tmp = (u64)delta_exec;

	if (!lw->inv_weight) {
		unsigned long w = scale_load_down(lw->weight);

		if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
			lw->inv_weight = 1;
		else if (unlikely(!w))
			lw->inv_weight = WMULT_CONST;
		else
			lw->inv_weight = WMULT_CONST / w;
	}

	/*
	 * Check whether we'd overflow the 64-bit multiplication:
	 */
	if (unlikely(tmp > WMULT_CONST))
		tmp = SRR(SRR(tmp, WMULT_SHIFT/2) * lw->inv_weight,
			WMULT_SHIFT/2);
	else
		tmp = SRR(tmp * lw->inv_weight, WMULT_SHIFT);

	return (unsigned long)min(tmp, (u64)(unsigned long)LONG_MAX);
}


const struct sched_class fair_sched_class;

/**************************************************************
 * CFS operations on generic schedulable entities:
 */

#ifdef CONFIG_FAIR_GROUP_SCHED

/* cpu runqueue to which this cfs_rq is attached */
static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return cfs_rq->rq;
}

/* An entity is a task if it doesn't "own" a runqueue */
#define entity_is_task(se)	(!se->my_q)

static inline struct task_struct *task_of(struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	WARN_ON_ONCE(!entity_is_task(se));
#endif
	return container_of(se, struct task_struct, se);
}

/* Walk up scheduling entities hierarchy */
#define for_each_sched_entity(se) \
		for (; se; se = se->parent)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return p->se.cfs_rq;
}

/* runqueue on which this entity is (to be) queued */
static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	return se->cfs_rq;
}

/* runqueue "owned" by this group */
static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return grp->my_q;
}

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (!cfs_rq->on_list) {
		/*
		 * Ensure we either appear before our parent (if already
		 * enqueued) or force our parent to appear after us when it is
		 * enqueued.  The fact that we always enqueue bottom-up
		 * reduces this to two cases.
		 */
		if (cfs_rq->tg->parent &&
		    cfs_rq->tg->parent->cfs_rq[cpu_of(rq_of(cfs_rq))]->on_list) {
			list_add_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		} else {
			list_add_tail_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		}

		cfs_rq->on_list = 1;
	}
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (cfs_rq->on_list) {
		list_del_rcu(&cfs_rq->leaf_cfs_rq_list);
		cfs_rq->on_list = 0;
	}
}

/* Iterate thr' all leaf cfs_rq's on a runqueue */
#define for_each_leaf_cfs_rq(rq, cfs_rq) \
	list_for_each_entry_rcu(cfs_rq, &rq->leaf_cfs_rq_list, leaf_cfs_rq_list)

/* Do the two (enqueued) entities belong to the same group ? */
static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	if (se->cfs_rq == pse->cfs_rq)
		return 1;

	return 0;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return se->parent;
}

/* return depth at which a sched entity is present in the hierarchy */
static inline int depth_se(struct sched_entity *se)
{
	int depth = 0;

	for_each_sched_entity(se)
		depth++;

	return depth;
}

static void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
	int se_depth, pse_depth;

	/*
	 * preemption test can be made between sibling entities who are in the
	 * same cfs_rq i.e who have a common parent. Walk up the hierarchy of
	 * both tasks until we find their ancestors who are siblings of common
	 * parent.
	 */

	/* First walk up until both entities are at same depth */
	se_depth = depth_se(*se);
	pse_depth = depth_se(*pse);

	while (se_depth > pse_depth) {
		se_depth--;
		*se = parent_entity(*se);
	}

	while (pse_depth > se_depth) {
		pse_depth--;
		*pse = parent_entity(*pse);
	}

	while (!is_same_group(*se, *pse)) {
		*se = parent_entity(*se);
		*pse = parent_entity(*pse);
	}
}

#else	/* !CONFIG_FAIR_GROUP_SCHED */

static inline struct task_struct *task_of(struct sched_entity *se)
{
	return container_of(se, struct task_struct, se);
}

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return container_of(cfs_rq, struct rq, cfs);
}

#define entity_is_task(se)	1

#define for_each_sched_entity(se) \
		for (; se; se = NULL)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return &task_rq(p)->cfs;
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct rq *rq = task_rq(p);

	return &rq->cfs;
}

/* runqueue "owned" by this group */
static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return NULL;
}

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

#define for_each_leaf_cfs_rq(rq, cfs_rq) \
		for (cfs_rq = &rq->cfs; cfs_rq; cfs_rq = NULL)

static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	return 1;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return NULL;
}

static inline void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
}

#endif	/* CONFIG_FAIR_GROUP_SCHED */

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec);

/**************************************************************
 * Scheduling class tree data structure manipulation methods:
 */

static inline u64 max_vruntime(u64 min_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vruntime);
	if (delta > 0)
		min_vruntime = vruntime;

	return min_vruntime;
}

static inline u64 min_vruntime(u64 min_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vruntime);
	if (delta < 0)
		min_vruntime = vruntime;

	return min_vruntime;
}

static inline int entity_before(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

static void update_min_vruntime(struct cfs_rq *cfs_rq)
{
	u64 vruntime = cfs_rq->min_vruntime;

	if (cfs_rq->curr)
		vruntime = cfs_rq->curr->vruntime;

	if (cfs_rq->rb_leftmost) {
		struct sched_entity *se = rb_entry(cfs_rq->rb_leftmost,
						   struct sched_entity,
						   run_node);

		if (!cfs_rq->curr)
			vruntime = se->vruntime;
		else
			vruntime = min_vruntime(vruntime, se->vruntime);
	}

	cfs_rq->min_vruntime = max_vruntime(cfs_rq->min_vruntime, vruntime);
#ifndef CONFIG_64BIT
	smp_wmb();
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
}

/*
 * Enqueue an entity into the rb-tree:
 */
static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct rb_node **link = &cfs_rq->tasks_timeline.rb_node;
	struct rb_node *parent = NULL;
	struct sched_entity *entry;
	int leftmost = 1;

	/*
	 * Find the right place in the rbtree:
	 */
	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct sched_entity, run_node);
		/*
		 * We dont care about collisions. Nodes with
		 * the same key stay together.
		 */
		if (entity_before(se, entry)) {
			link = &parent->rb_left;
		} else {
			link = &parent->rb_right;
			leftmost = 0;
		}
	}

	/*
	 * Maintain a cache of leftmost tree entries (it is frequently
	 * used):
	 */
	if (leftmost)
		cfs_rq->rb_leftmost = &se->run_node;

	rb_link_node(&se->run_node, parent, link);
	rb_insert_color(&se->run_node, &cfs_rq->tasks_timeline);
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->rb_leftmost == &se->run_node) {
		struct rb_node *next_node;

		next_node = rb_next(&se->run_node);
		cfs_rq->rb_leftmost = next_node;
	}

	rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
}

struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *left = cfs_rq->rb_leftmost;

	if (!left)
		return NULL;

	return rb_entry(left, struct sched_entity, run_node);
}

static struct sched_entity *__pick_next_entity(struct sched_entity *se)
{
	struct rb_node *next = rb_next(&se->run_node);

	if (!next)
		return NULL;

	return rb_entry(next, struct sched_entity, run_node);
}

#ifdef CONFIG_SCHED_DEBUG
struct sched_entity *__pick_last_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *last = rb_last(&cfs_rq->tasks_timeline);

	if (!last)
		return NULL;

	return rb_entry(last, struct sched_entity, run_node);
}

/**************************************************************
 * Scheduling class statistics methods:
 */

int sched_proc_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	int factor = get_update_sysctl_factor();

	if (ret || !write)
		return ret;

	sched_nr_latency = DIV_ROUND_UP(sysctl_sched_latency,
					sysctl_sched_min_granularity);

#define WRT_SYSCTL(name) \
	(normalized_sysctl_##name = sysctl_##name / (factor))
	WRT_SYSCTL(sched_min_granularity);
	WRT_SYSCTL(sched_latency);
	WRT_SYSCTL(sched_wakeup_granularity);
#undef WRT_SYSCTL

	return 0;
}
#endif

/*
 * delta /= w
 */
static inline unsigned long
calc_delta_fair(unsigned long delta, struct sched_entity *se)
{
	if (unlikely(se->load.weight != NICE_0_LOAD))
		delta = calc_delta_mine(delta, NICE_0_LOAD, &se->load);

	return delta;
}

/*
 * The idea is to set a period in which each task runs once.
 *
 * When there are too many tasks (sysctl_sched_nr_latency) we have to stretch
 * this period because otherwise the slices get too small.
 *
 * p = (nr <= nl) ? l : l*nr/nl
 */
static u64 __sched_period(unsigned long nr_running)
{
	u64 period = sysctl_sched_latency;
	unsigned long nr_latency = sched_nr_latency;

	if (unlikely(nr_running > nr_latency)) {
		period = sysctl_sched_min_granularity;
		period *= nr_running;
	}

	return period;
}

/*
 * We calculate the wall-time slice from the period by taking a part
 * proportional to the weight.
 *
 * s = p*P[w/rw]
 */
static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
		slice = calc_delta_mine(slice, se->load.weight, load);
	}
	return slice;
}

/*
 * We calculate the vruntime slice of a to be inserted task
 *
 * vs = s/w
 */
static u64 sched_vslice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return calc_delta_fair(sched_slice(cfs_rq, se), se);
}

static void update_cfs_load(struct cfs_rq *cfs_rq, int global_update);
static void update_cfs_shares(struct cfs_rq *cfs_rq);

/*
 * Update the current task's runtime statistics. Skip current tasks that
 * are not in our scheduling class.
 */
static inline void
__update_curr(struct cfs_rq *cfs_rq, struct sched_entity *curr,
	      unsigned long delta_exec)
{
	unsigned long delta_exec_weighted;

	schedstat_set(curr->statistics.exec_max,
		      max((u64)delta_exec, curr->statistics.exec_max));

	curr->sum_exec_runtime += delta_exec;
	schedstat_add(cfs_rq, exec_clock, delta_exec);
	delta_exec_weighted = calc_delta_fair(delta_exec, curr);

	curr->vruntime += delta_exec_weighted;
	update_min_vruntime(cfs_rq);

#if defined CONFIG_SMP && defined CONFIG_FAIR_GROUP_SCHED
	cfs_rq->load_unacc_exec_time += delta_exec;
#endif
}

static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	u64 now = rq_of(cfs_rq)->clock_task;
	unsigned long delta_exec;

	if (unlikely(!curr))
		return;

	/*
	 * Get the amount of time the current task was running
	 * since the last time we changed load (this cannot
	 * overflow on 32 bits):
	 */
	delta_exec = (unsigned long)(now - curr->exec_start);
	if (!delta_exec)
		return;

	__update_curr(cfs_rq, curr, delta_exec);
	curr->exec_start = now;

	if (entity_is_task(curr)) {
		struct task_struct *curtask = task_of(curr);

		trace_sched_stat_runtime(curtask, delta_exec, curr->vruntime);
		cpuacct_charge(curtask, delta_exec);
		account_group_exec_runtime(curtask, delta_exec);
	}

	account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline void
update_stats_wait_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_start, rq_of(cfs_rq)->clock);
}

/*
 * Task is being enqueued - update stats:
 */
static void update_stats_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/*
	 * Are we enqueueing a waiting task? (for current tasks
	 * a dequeue/enqueue event is a NOP)
	 */
	if (se != cfs_rq->curr)
		update_stats_wait_start(cfs_rq, se);
}

static void
update_stats_wait_end(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_max, max(se->statistics.wait_max,
			rq_of(cfs_rq)->clock - se->statistics.wait_start));
	schedstat_set(se->statistics.wait_count, se->statistics.wait_count + 1);
	schedstat_set(se->statistics.wait_sum, se->statistics.wait_sum +
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
#ifdef CONFIG_SCHEDSTATS
	if (entity_is_task(se)) {
		trace_sched_stat_wait(task_of(se),
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
	}
#endif
	schedstat_set(se->statistics.wait_start, 0);
}

static inline void
update_stats_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/*
	 * Mark the end of the wait period if dequeueing a
	 * waiting task:
	 */
	if (se != cfs_rq->curr)
		update_stats_wait_end(cfs_rq, se);
}

/*
 * We are picking a new current task - update its stats:
 */
static inline void
update_stats_curr_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/*
	 * We are starting a new run period:
	 */
	se->exec_start = rq_of(cfs_rq)->clock_task;
}

/**************************************************
 * Scheduling class queueing methods:
 */

static void
account_entity_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_add(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_add(&rq_of(cfs_rq)->load, se->load.weight);
#ifdef CONFIG_SMP
	if (entity_is_task(se))
		list_add(&se->group_node, &rq_of(cfs_rq)->cfs_tasks);
#endif
	cfs_rq->nr_running++;
}

static void
account_entity_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_sub(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_sub(&rq_of(cfs_rq)->load, se->load.weight);
	if (entity_is_task(se))
		list_del_init(&se->group_node);
	cfs_rq->nr_running--;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
/* we need this in update_cfs_load and load-balance functions below */
static inline int throttled_hierarchy(struct cfs_rq *cfs_rq);
# ifdef CONFIG_SMP
static void update_cfs_rq_load_contribution(struct cfs_rq *cfs_rq,
					    int global_update)
{
	struct task_group *tg = cfs_rq->tg;
	long load_avg;

	load_avg = div64_u64(cfs_rq->load_avg, cfs_rq->load_period+1);
	load_avg -= cfs_rq->load_contribution;

	if (global_update || abs(load_avg) > cfs_rq->load_contribution / 8) {
		atomic_add(load_avg, &tg->load_weight);
		cfs_rq->load_contribution += load_avg;
	}
}

static void update_cfs_load(struct cfs_rq *cfs_rq, int global_update)
{
	u64 period = sysctl_sched_shares_window;
	u64 now, delta;
	unsigned long load = cfs_rq->load.weight;

	if (cfs_rq->tg == &root_task_group || throttled_hierarchy(cfs_rq))
		return;

	now = rq_of(cfs_rq)->clock_task;
	delta = now - cfs_rq->load_stamp;

	/* truncate load history at 4 idle periods */
	if (cfs_rq->load_stamp > cfs_rq->load_last &&
	    now - cfs_rq->load_last > 4 * period) {
		cfs_rq->load_period = 0;
		cfs_rq->load_avg = 0;
		delta = period - 1;
	}

	cfs_rq->load_stamp = now;
	cfs_rq->load_unacc_exec_time = 0;
	cfs_rq->load_period += delta;
	if (load) {
		cfs_rq->load_last = now;
		cfs_rq->load_avg += delta * load;
	}

	/* consider updating load contribution on each fold or truncate */
	if (global_update || cfs_rq->load_period > period
	    || !cfs_rq->load_period)
		update_cfs_rq_load_contribution(cfs_rq, global_update);

	while (cfs_rq->load_period > period) {
		/*
		 * Inline assembly required to prevent the compiler
		 * optimising this loop into a divmod call.
		 * See __iter_div_u64_rem() for another example of this.
		 */
		asm("" : "+rm" (cfs_rq->load_period));
		cfs_rq->load_period /= 2;
		cfs_rq->load_avg /= 2;
	}

	if (!cfs_rq->curr && !cfs_rq->nr_running && !cfs_rq->load_avg)
		list_del_leaf_cfs_rq(cfs_rq);
}

static inline long calc_tg_weight(struct task_group *tg, struct cfs_rq *cfs_rq)
{
	long tg_weight;

	/*
	 * Use this CPU's actual weight instead of the last load_contribution
	 * to gain a more accurate current total weight. See
	 * update_cfs_rq_load_contribution().
	 */
	tg_weight = atomic_read(&tg->load_weight);
	tg_weight -= cfs_rq->load_contribution;
	tg_weight += cfs_rq->load.weight;

	return tg_weight;
}

static long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	long tg_weight, load, shares;

	tg_weight = calc_tg_weight(tg, cfs_rq);
	load = cfs_rq->load.weight;

	shares = (tg->shares * load);
	if (tg_weight)
		shares /= tg_weight;

	if (shares < MIN_SHARES)
		shares = MIN_SHARES;
	if (shares > tg->shares)
		shares = tg->shares;

	return shares;
}

static void update_entity_shares_tick(struct cfs_rq *cfs_rq)
{
	if (cfs_rq->load_unacc_exec_time > sysctl_sched_shares_window) {
		update_cfs_load(cfs_rq, 0);
		update_cfs_shares(cfs_rq);
	}
}
# else /* CONFIG_SMP */
static void update_cfs_load(struct cfs_rq *cfs_rq, int global_update)
{
}

static inline long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	return tg->shares;
}

static inline void update_entity_shares_tick(struct cfs_rq *cfs_rq)
{
}
# endif /* CONFIG_SMP */
static void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			    unsigned long weight)
{
	if (se->on_rq) {
		/* commit outstanding execution time */
		if (cfs_rq->curr == se)
			update_curr(cfs_rq);
		account_entity_dequeue(cfs_rq, se);
	}

	update_load_set(&se->load, weight);

	if (se->on_rq)
		account_entity_enqueue(cfs_rq, se);
}

static void update_cfs_shares(struct cfs_rq *cfs_rq)
{
	struct task_group *tg;
	struct sched_entity *se;
	long shares;

	tg = cfs_rq->tg;
	se = tg->se[cpu_of(rq_of(cfs_rq))];
	if (!se || throttled_hierarchy(cfs_rq))
		return;
#ifndef CONFIG_SMP
	if (likely(se->load.weight == tg->shares))
		return;
#endif
	shares = calc_cfs_shares(cfs_rq, tg);

	reweight_entity(cfs_rq_of(se), se, shares);
}
#else /* CONFIG_FAIR_GROUP_SCHED */
static void update_cfs_load(struct cfs_rq *cfs_rq, int global_update)
{
}

static inline void update_cfs_shares(struct cfs_rq *cfs_rq)
{
}

static inline void update_entity_shares_tick(struct cfs_rq *cfs_rq)
{
}
#endif /* CONFIG_FAIR_GROUP_SCHED */

static void enqueue_sleeper(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHEDSTATS
	struct task_struct *tsk = NULL;

	if (entity_is_task(se))
		tsk = task_of(se);

	if (se->statistics.sleep_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.sleep_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.sleep_max))
			se->statistics.sleep_max = delta;

		se->statistics.sleep_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			account_scheduler_latency(tsk, delta >> 10, 1);
			trace_sched_stat_sleep(tsk, delta);
		}
	}
	if (se->statistics.block_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.block_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.block_max))
			se->statistics.block_max = delta;

		se->statistics.block_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			if (tsk->in_iowait) {
				se->statistics.iowait_sum += delta;
				se->statistics.iowait_count++;
				trace_sched_stat_iowait(tsk, delta);
			}

			trace_sched_stat_blocked(tsk, delta);

			/*
			 * Blocking time is in units of nanosecs, so shift by
			 * 20 to get a milliseconds-range estimation of the
			 * amount of time that the task spent sleeping:
			 */
			if (unlikely(prof_on == SLEEP_PROFILING)) {
				profile_hits(SLEEP_PROFILING,
						(void *)get_wchan(tsk),
						delta >> 20);
			}
			account_scheduler_latency(tsk, delta >> 10, 0);
		}
	}
#endif
}

static void check_spread(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	s64 d = se->vruntime - cfs_rq->min_vruntime;

	if (d < 0)
		d = -d;

	if (d > 3*sysctl_sched_latency)
		schedstat_inc(cfs_rq, nr_spread_over);
#endif
}

static void
place_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int initial)
{
	u64 vruntime = cfs_rq->min_vruntime;

	/*
	 * The 'current' period is already promised to the current tasks,
	 * however the extra weight of the new task will slow them down a
	 * little, place the new task so that it fits in the slot that
	 * stays open at the end.
	 */
	if (initial && sched_feat(START_DEBIT))
		vruntime += sched_vslice(cfs_rq, se);

	/* sleeps up to a single latency don't count. */
	if (!initial) {
		unsigned long thresh = sysctl_sched_latency;

		/*
		 * Halve their sleep time's effect, to allow
		 * for a gentler effect of sleepers:
		 */
		if (sched_feat(GENTLE_FAIR_SLEEPERS))
			thresh >>= 1;

		vruntime -= thresh;
	}

	/* ensure we never gain time by being placed backwards. */
	vruntime = max_vruntime(se->vruntime, vruntime);

	se->vruntime = vruntime;
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq);

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	/*
	 * Update the normalized vruntime before updating min_vruntime
	 * through callig update_curr().
	 */
	if (!(flags & ENQUEUE_WAKEUP) || (flags & ENQUEUE_WAKING))
		se->vruntime += cfs_rq->min_vruntime;

	/*
	 * Update run-time statistics of the 'current'.
	 */
	update_curr(cfs_rq);
	update_cfs_load(cfs_rq, 0);
	account_entity_enqueue(cfs_rq, se);
	update_cfs_shares(cfs_rq);

	if (flags & ENQUEUE_WAKEUP) {
		place_entity(cfs_rq, se, 0);
		enqueue_sleeper(cfs_rq, se);
	}

	update_stats_enqueue(cfs_rq, se);
	check_spread(cfs_rq, se);
	if (se != cfs_rq->curr)
		__enqueue_entity(cfs_rq, se);
	se->on_rq = 1;

	if (cfs_rq->nr_running == 1) {
		list_add_leaf_cfs_rq(cfs_rq);
		check_enqueue_throttle(cfs_rq);
	}
}

static void __clear_buddies_last(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->last == se)
			cfs_rq->last = NULL;
		else
			break;
	}
}

static void __clear_buddies_next(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->next == se)
			cfs_rq->next = NULL;
		else
			break;
	}
}

static void __clear_buddies_skip(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->skip == se)
			cfs_rq->skip = NULL;
		else
			break;
	}
}

static void clear_buddies(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->last == se)
		__clear_buddies_last(se);

	if (cfs_rq->next == se)
		__clear_buddies_next(se);

	if (cfs_rq->skip == se)
		__clear_buddies_skip(se);
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	/*
	 * Update run-time statistics of the 'current'.
	 */
	update_curr(cfs_rq);

	update_stats_dequeue(cfs_rq, se);
	if (flags & DEQUEUE_SLEEP) {
#ifdef CONFIG_SCHEDSTATS
		if (entity_is_task(se)) {
			struct task_struct *tsk = task_of(se);

			if (tsk->state & TASK_INTERRUPTIBLE)
				se->statistics.sleep_start = rq_of(cfs_rq)->clock;
			if (tsk->state & TASK_UNINTERRUPTIBLE)
				se->statistics.block_start = rq_of(cfs_rq)->clock;
		}
#endif
	}

	clear_buddies(cfs_rq, se);

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, se);
	se->on_rq = 0;
	update_cfs_load(cfs_rq, 0);
	account_entity_dequeue(cfs_rq, se);

	/*
	 * Normalize the entity after updating the min_vruntime because the
	 * update can refer to the ->curr item and we need to reflect this
	 * movement in our normalized position.
	 */
	if (!(flags & DEQUEUE_SLEEP))
		se->vruntime -= cfs_rq->min_vruntime;

	/* return excess runtime on last dequeue */
	return_cfs_rq_runtime(cfs_rq);

	update_min_vruntime(cfs_rq);
	update_cfs_shares(cfs_rq);
}

/*
 * Preempt the current task with a newly woken task if needed:
 */
static void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned long ideal_runtime, delta_exec;
	struct sched_entity *se;
	s64 delta;

	ideal_runtime = sched_slice(cfs_rq, curr);
	delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	if (delta_exec > ideal_runtime) {
		resched_task(rq_of(cfs_rq)->curr);
		/*
		 * The current task ran long enough, ensure it doesn't get
		 * re-elected due to buddy favours.
		 */
		clear_buddies(cfs_rq, curr);
		return;
	}

	/*
	 * Ensure that a task that missed wakeup preemption by a
	 * narrow margin doesn't have to wait for a full slice.
	 * This also mitigates buddy induced latencies under load.
	 */
	if (delta_exec < sysctl_sched_min_granularity)
		return;

	se = __pick_first_entity(cfs_rq);
	delta = curr->vruntime - se->vruntime;

	if (delta < 0)
		return;

	if (delta > ideal_runtime)
		resched_task(rq_of(cfs_rq)->curr);
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/* 'current' is not kept within the tree. */
	if (se->on_rq) {
		/*
		 * Any task has to be enqueued before it get to execute on
		 * a CPU. So account for the time it spent waiting on the
		 * runqueue.
		 */
		update_stats_wait_end(cfs_rq, se);
		__dequeue_entity(cfs_rq, se);
	}

	update_stats_curr_start(cfs_rq, se);
	cfs_rq->curr = se;
#ifdef CONFIG_SCHEDSTATS
	/*
	 * Track our maximum slice length, if the CPU's load is at
	 * least twice that of our own weight (i.e. dont track it
	 * when there are only lesser-weight tasks around):
	 */
	if (rq_of(cfs_rq)->load.weight >= 2*se->load.weight) {
		se->statistics.slice_max = max(se->statistics.slice_max,
			se->sum_exec_runtime - se->prev_sum_exec_runtime);
	}
#endif
	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se);

/*
 * Pick the next process, keeping these things in mind, in this order:
 * 1) keep things fair between processes/task groups
 * 2) pick the "next" process, since someone really wants that to run
 * 3) pick the "last" process, for cache locality
 * 4) do not run the "skip" process, if something else is available
 */
static struct sched_entity *pick_next_entity(struct cfs_rq *cfs_rq)
{
	struct sched_entity *se = __pick_first_entity(cfs_rq);
	struct sched_entity *left = se;

	/*
	 * Avoid running the skip buddy, if running something else can
	 * be done without getting too unfair.
	 */
	if (cfs_rq->skip == se) {
		struct sched_entity *second = __pick_next_entity(se);
		if (second && wakeup_preempt_entity(second, left) < 1)
			se = second;
	}

	/*
	 * Prefer last buddy, try to return the CPU to a preempted task.
	 */
	if (cfs_rq->last && wakeup_preempt_entity(cfs_rq->last, left) < 1)
		se = cfs_rq->last;

	/*
	 * Someone really wants this to run. If it's not unfair, run it.
	 */
	if (cfs_rq->next && wakeup_preempt_entity(cfs_rq->next, left) < 1)
		se = cfs_rq->next;

	clear_buddies(cfs_rq, se);

	return se;
}

static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	/*
	 * If still on the runqueue then deactivate_task()
	 * was not called and update_curr() has to be done:
	 */
	if (prev->on_rq)
		update_curr(cfs_rq);

	/* throttle cfs_rqs exceeding runtime */
	check_cfs_rq_runtime(cfs_rq);

	check_spread(cfs_rq, prev);
	if (prev->on_rq) {
		update_stats_wait_start(cfs_rq, prev);
		/* Put 'current' back into the tree. */
		__enqueue_entity(cfs_rq, prev);
	}
	cfs_rq->curr = NULL;
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	/*
	 * Update run-time statistics of the 'current'.
	 */
	update_curr(cfs_rq);

	/*
	 * Update share accounting for long-running entities.
	 */
	update_entity_shares_tick(cfs_rq);

#ifdef CONFIG_SCHED_HRTICK
	/*
	 * queued ticks are scheduled to match the slice, so don't bother
	 * validating it and just reschedule.
	 */
	if (queued) {
		resched_task(rq_of(cfs_rq)->curr);
		return;
	}
	/*
	 * don't let the period tick interfere with the hrtick preemption
	 */
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif

	if (cfs_rq->nr_running > 1)
		check_preempt_tick(cfs_rq, curr);
}


/**************************************************
 * CFS bandwidth control machinery
 */

#ifdef CONFIG_CFS_BANDWIDTH

#ifdef HAVE_JUMP_LABEL
static struct static_key __cfs_bandwidth_used;

static inline bool cfs_bandwidth_used(void)
{
	return static_key_false(&__cfs_bandwidth_used);
}

void account_cfs_bandwidth_used(int enabled, int was_enabled)
{
	/* only need to count groups transitioning between enabled/!enabled */
	if (enabled && !was_enabled)
		static_key_slow_inc(&__cfs_bandwidth_used);
	else if (!enabled && was_enabled)
		static_key_slow_dec(&__cfs_bandwidth_used);
}
#else /* HAVE_JUMP_LABEL */
static bool cfs_bandwidth_used(void)
{
	return true;
}

void account_cfs_bandwidth_used(int enabled, int was_enabled) {}
#endif /* HAVE_JUMP_LABEL */

/*
 * default period for cfs group bandwidth.
 * default: 0.1s, units: nanoseconds
 */
static inline u64 default_cfs_period(void)
{
	return 100000000ULL;
}

static inline u64 sched_cfs_bandwidth_slice(void)
{
	return (u64)sysctl_sched_cfs_bandwidth_slice * NSEC_PER_USEC;
}

/*
 * Replenish runtime according to assigned quota and update expiration time.
 * We use sched_clock_cpu directly instead of rq->clock to avoid adding
 * additional synchronization around rq->lock.
 *
 * requires cfs_b->lock
 */
void __refill_cfs_bandwidth_runtime(struct cfs_bandwidth *cfs_b)
{
	u64 now;

	if (cfs_b->quota == RUNTIME_INF)
		return;

	now = sched_clock_cpu(smp_processor_id());
	cfs_b->runtime = cfs_b->quota;
	cfs_b->runtime_expires = now + ktime_to_ns(cfs_b->period);
}

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return &tg->cfs_bandwidth;
}

/* returns 0 on failure to allocate runtime */
static int assign_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct task_group *tg = cfs_rq->tg;
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(tg);
	u64 amount = 0, min_amount, expires;

	/* note: this is a positive sum as runtime_remaining <= 0 */
	min_amount = sched_cfs_bandwidth_slice() - cfs_rq->runtime_remaining;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota == RUNTIME_INF)
		amount = min_amount;
	else {
		/*
		 * If the bandwidth pool has become inactive, then at least one
		 * period must have elapsed since the last consumption.
		 * Refresh the global state and ensure bandwidth timer becomes
		 * active.
		 */
		if (!cfs_b->timer_active) {
			__refill_cfs_bandwidth_runtime(cfs_b);
			__start_cfs_bandwidth(cfs_b);
		}

		if (cfs_b->runtime > 0) {
			amount = min(cfs_b->runtime, min_amount);
			cfs_b->runtime -= amount;
			cfs_b->idle = 0;
		}
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	cfs_rq->runtime_remaining += amount;
	/*
	 * we may have advanced our local expiration to account for allowed
	 * spread between our sched_clock and the one on which runtime was
	 * issued.
	 */
	if ((s64)(expires - cfs_rq->runtime_expires) > 0)
		cfs_rq->runtime_expires = expires;

	return cfs_rq->runtime_remaining > 0;
}

/*
 * Note: This depends on the synchronization provided by sched_clock and the
 * fact that rq->clock snapshots this value.
 */
static void expire_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct rq *rq = rq_of(cfs_rq);

	/* if the deadline is ahead of our clock, nothing to do */
	if (likely((s64)(rq->clock - cfs_rq->runtime_expires) < 0))
		return;

	if (cfs_rq->runtime_remaining < 0)
		return;

	/*
	 * If the local deadline has passed we have to consider the
	 * possibility that our sched_clock is 'fast' and the global deadline
	 * has not truly expired.
	 *
	 * Fortunately we can check determine whether this the case by checking
	 * whether the global deadline has advanced.
	 */

	if ((s64)(cfs_rq->runtime_expires - cfs_b->runtime_expires) >= 0) {
		/* extend local deadline, drift is bounded above by 2 ticks */
		cfs_rq->runtime_expires += TICK_NSEC;
	} else {
		/* global deadline is ahead, expiration has passed */
		cfs_rq->runtime_remaining = 0;
	}
}

static void __account_cfs_rq_runtime(struct cfs_rq *cfs_rq,
				     unsigned long delta_exec)
{
	/* dock delta_exec before expiring quota (as it could span periods) */
	cfs_rq->runtime_remaining -= delta_exec;
	expire_cfs_rq_runtime(cfs_rq);

	if (likely(cfs_rq->runtime_remaining > 0))
		return;

	/*
	 * if we're unable to extend our runtime we resched so that the active
	 * hierarchy can be throttled
	 */
	if (!assign_cfs_rq_runtime(cfs_rq) && likely(cfs_rq->curr))
		resched_task(rq_of(cfs_rq)->curr);
}

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec)
{
	if (!cfs_bandwidth_used() || !cfs_rq->runtime_enabled)
		return;

	__account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttled;
}

/* check whether cfs_rq, or any parent, is throttled */
static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttle_count;
}

/*
 * Ensure that neither of the group entities corresponding to src_cpu or
 * dest_cpu are members of a throttled hierarchy when performing group
 * load-balance operations.
 */
static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	struct cfs_rq *src_cfs_rq, *dest_cfs_rq;

	src_cfs_rq = tg->cfs_rq[src_cpu];
	dest_cfs_rq = tg->cfs_rq[dest_cpu];

	return throttled_hierarchy(src_cfs_rq) ||
	       throttled_hierarchy(dest_cfs_rq);
}

/* updated child weight may affect parent so we have to do this bottom up */
static int tg_unthrottle_up(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	cfs_rq->throttle_count--;
#ifdef CONFIG_SMP
	if (!cfs_rq->throttle_count) {
		u64 delta = rq->clock_task - cfs_rq->load_stamp;

		/* leaving throttled state, advance shares averaging windows */
		cfs_rq->load_stamp += delta;
		cfs_rq->load_last += delta;

		/* update entity weight now that we are on_rq again */
		update_cfs_shares(cfs_rq);
	}
#endif

	return 0;
}

static int tg_throttle_down(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	/* group is entering throttled state, record last load */
	if (!cfs_rq->throttle_count)
		update_cfs_load(cfs_rq, 0);
	cfs_rq->throttle_count++;

	return 0;
}

static void throttle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	long task_delta, dequeue = 1;

	se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

	/* account load preceding throttle */
	rcu_read_lock();
	walk_tg_tree_from(cfs_rq->tg, tg_throttle_down, tg_nop, (void *)rq);
	rcu_read_unlock();

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		struct cfs_rq *qcfs_rq = cfs_rq_of(se);
		/* throttled entity or throttle-on-deactivate */
		if (!se->on_rq)
			break;

		if (dequeue)
			dequeue_entity(qcfs_rq, se, DEQUEUE_SLEEP);
		qcfs_rq->h_nr_running -= task_delta;

		if (qcfs_rq->load.weight)
			dequeue = 0;
	}

	if (!se)
		rq->nr_running -= task_delta;

	cfs_rq->throttled = 1;
	cfs_rq->throttled_timestamp = rq->clock;
	raw_spin_lock(&cfs_b->lock);
	list_add_tail_rcu(&cfs_rq->throttled_list, &cfs_b->throttled_cfs_rq);
	raw_spin_unlock(&cfs_b->lock);
}

void unthrottle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	int enqueue = 1;
	long task_delta;

	se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

	cfs_rq->throttled = 0;
	raw_spin_lock(&cfs_b->lock);
	cfs_b->throttled_time += rq->clock - cfs_rq->throttled_timestamp;
	list_del_rcu(&cfs_rq->throttled_list);
	raw_spin_unlock(&cfs_b->lock);
	cfs_rq->throttled_timestamp = 0;

	update_rq_clock(rq);
	/* update hierarchical throttle state */
	walk_tg_tree_from(cfs_rq->tg, tg_nop, tg_unthrottle_up, (void *)rq);

	if (!cfs_rq->load.weight)
		return;

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		if (se->on_rq)
			enqueue = 0;

		cfs_rq = cfs_rq_of(se);
		if (enqueue)
			enqueue_entity(cfs_rq, se, ENQUEUE_WAKEUP);
		cfs_rq->h_nr_running += task_delta;

		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	if (!se)
		rq->nr_running += task_delta;

	/* determine whether we need to wake up potentially idle cpu */
	if (rq->curr == rq->idle && rq->cfs.nr_running)
		resched_task(rq->curr);
}

static u64 distribute_cfs_runtime(struct cfs_bandwidth *cfs_b,
		u64 remaining, u64 expires)
{
	struct cfs_rq *cfs_rq;
	u64 runtime = remaining;

	rcu_read_lock();
	list_for_each_entry_rcu(cfs_rq, &cfs_b->throttled_cfs_rq,
				throttled_list) {
		struct rq *rq = rq_of(cfs_rq);

		raw_spin_lock(&rq->lock);
		if (!cfs_rq_throttled(cfs_rq))
			goto next;

		runtime = -cfs_rq->runtime_remaining + 1;
		if (runtime > remaining)
			runtime = remaining;
		remaining -= runtime;

		cfs_rq->runtime_remaining += runtime;
		cfs_rq->runtime_expires = expires;

		/* we check whether we're throttled above */
		if (cfs_rq->runtime_remaining > 0)
			unthrottle_cfs_rq(cfs_rq);

next:
		raw_spin_unlock(&rq->lock);

		if (!remaining)
			break;
	}
	rcu_read_unlock();

	return remaining;
}

/*
 * Responsible for refilling a task_group's bandwidth and unthrottling its
 * cfs_rqs as appropriate. If there has been no activity within the last
 * period the timer is deactivated until scheduling resumes; cfs_b->idle is
 * used to track this state.
 */
static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun)
{
	u64 runtime, runtime_expires;
	int idle = 1, throttled;

	raw_spin_lock(&cfs_b->lock);
	/* no need to continue the timer with no bandwidth constraint */
	if (cfs_b->quota == RUNTIME_INF)
		goto out_unlock;

	throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	/* idle depends on !throttled (for the case of a large deficit) */
	idle = cfs_b->idle && !throttled;
	cfs_b->nr_periods += overrun;

	/* if we're going inactive then everything else can be deferred */
	if (idle)
		goto out_unlock;

	__refill_cfs_bandwidth_runtime(cfs_b);

	if (!throttled) {
		/* mark as potentially idle for the upcoming period */
		cfs_b->idle = 1;
		goto out_unlock;
	}

	/* account preceding periods in which throttling occurred */
	cfs_b->nr_throttled += overrun;

	/*
	 * There are throttled entities so we must first use the new bandwidth
	 * to unthrottle them before making it generally available.  This
	 * ensures that all existing debts will be paid before a new cfs_rq is
	 * allowed to run.
	 */
	runtime = cfs_b->runtime;
	runtime_expires = cfs_b->runtime_expires;
	cfs_b->runtime = 0;

	/*
	 * This check is repeated as we are holding onto the new bandwidth
	 * while we unthrottle.  This can potentially race with an unthrottled
	 * group trying to acquire new bandwidth from the global pool.
	 */
	while (throttled && runtime > 0) {
		raw_spin_unlock(&cfs_b->lock);
		/* we can't nest cfs_b->lock while distributing bandwidth */
		runtime = distribute_cfs_runtime(cfs_b, runtime,
						 runtime_expires);
		raw_spin_lock(&cfs_b->lock);

		throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	}

	/* return (any) remaining runtime */
	cfs_b->runtime = runtime;
	/*
	 * While we are ensured activity in the period following an
	 * unthrottle, this also covers the case in which the new bandwidth is
	 * insufficient to cover the existing bandwidth deficit.  (Forcing the
	 * timer to remain active while there are any throttled entities.)
	 */
	cfs_b->idle = 0;
out_unlock:
	if (idle)
		cfs_b->timer_active = 0;
	raw_spin_unlock(&cfs_b->lock);

	return idle;
}

/* a cfs_rq won't donate quota below this amount */
static const u64 min_cfs_rq_runtime = 1 * NSEC_PER_MSEC;
/* minimum remaining period time to redistribute slack quota */
static const u64 min_bandwidth_expiration = 2 * NSEC_PER_MSEC;
/* how long we wait to gather additional slack before distributing */
static const u64 cfs_bandwidth_slack_period = 5 * NSEC_PER_MSEC;

/* are we near the end of the current quota period? */
static int runtime_refresh_within(struct cfs_bandwidth *cfs_b, u64 min_expire)
{
	struct hrtimer *refresh_timer = &cfs_b->period_timer;
	u64 remaining;

	/* if the call-back is running a quota refresh is already occurring */
	if (hrtimer_callback_running(refresh_timer))
		return 1;

	/* is a quota refresh about to occur? */
	remaining = ktime_to_ns(hrtimer_expires_remaining(refresh_timer));
	if (remaining < min_expire)
		return 1;

	return 0;
}

static void start_cfs_slack_bandwidth(struct cfs_bandwidth *cfs_b)
{
	u64 min_left = cfs_bandwidth_slack_period + min_bandwidth_expiration;

	/* if there's a quota refresh soon don't bother with slack */
	if (runtime_refresh_within(cfs_b, min_left))
		return;

	start_bandwidth_timer(&cfs_b->slack_timer,
				ns_to_ktime(cfs_bandwidth_slack_period));
}

/* we know any runtime found here is valid as update_curr() precedes return */
static void __return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	s64 slack_runtime = cfs_rq->runtime_remaining - min_cfs_rq_runtime;

	if (slack_runtime <= 0)
		return;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota != RUNTIME_INF &&
	    cfs_rq->runtime_expires == cfs_b->runtime_expires) {
		cfs_b->runtime += slack_runtime;

		/* we are under rq->lock, defer unthrottling using a timer */
		if (cfs_b->runtime > sched_cfs_bandwidth_slice() &&
		    !list_empty(&cfs_b->throttled_cfs_rq))
			start_cfs_slack_bandwidth(cfs_b);
	}
	raw_spin_unlock(&cfs_b->lock);

	/* even if it's not valid for return we don't want to try again */
	cfs_rq->runtime_remaining -= slack_runtime;
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (!cfs_rq->runtime_enabled || cfs_rq->nr_running)
		return;

	__return_cfs_rq_runtime(cfs_rq);
}

/*
 * This is done with a timer (instead of inline with bandwidth return) since
 * it's necessary to juggle rq->locks to unthrottle their respective cfs_rqs.
 */
static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b)
{
	u64 runtime = 0, slice = sched_cfs_bandwidth_slice();
	u64 expires;

	/* confirm we're still not at a refresh boundary */
	if (runtime_refresh_within(cfs_b, min_bandwidth_expiration))
		return;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota != RUNTIME_INF && cfs_b->runtime > slice) {
		runtime = cfs_b->runtime;
		cfs_b->runtime = 0;
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	if (!runtime)
		return;

	runtime = distribute_cfs_runtime(cfs_b, runtime, expires);

	raw_spin_lock(&cfs_b->lock);
	if (expires == cfs_b->runtime_expires)
		cfs_b->runtime = runtime;
	raw_spin_unlock(&cfs_b->lock);
}

/*
 * When a group wakes up we want to make sure that its quota is not already
 * expired/exceeded, otherwise it may be allowed to steal additional ticks of
 * runtime as update_curr() throttling can not not trigger until it's on-rq.
 */
static void check_enqueue_throttle(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	/* an active group must be handled by the update_curr()->put() path */
	if (!cfs_rq->runtime_enabled || cfs_rq->curr)
		return;

	/* ensure the group is not already throttled */
	if (cfs_rq_throttled(cfs_rq))
		return;

	/* update runtime allocation */
	account_cfs_rq_runtime(cfs_rq, 0);
	if (cfs_rq->runtime_remaining <= 0)
		throttle_cfs_rq(cfs_rq);
}

/* conditionally throttle active cfs_rq's from put_prev_entity() */
static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (likely(!cfs_rq->runtime_enabled || cfs_rq->runtime_remaining > 0))
		return;

	/*
	 * it's possible for a throttled entity to be forced into a running
	 * state (e.g. set_curr_task), in this case we're finished.
	 */
	if (cfs_rq_throttled(cfs_rq))
		return;

	throttle_cfs_rq(cfs_rq);
}

static inline u64 default_cfs_period(void);
static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun);
static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b);

static enum hrtimer_restart sched_cfs_slack_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, slack_timer);
	do_sched_cfs_slack_timer(cfs_b);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sched_cfs_period_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, period_timer);
	ktime_t now;
	int overrun;
	int idle = 0;

	for (;;) {
		now = hrtimer_cb_get_time(timer);
		overrun = hrtimer_forward(timer, now, cfs_b->period);

		if (!overrun)
			break;

		idle = do_sched_cfs_period_timer(cfs_b, overrun);
	}

	return idle ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	raw_spin_lock_init(&cfs_b->lock);
	cfs_b->runtime = 0;
	cfs_b->quota = RUNTIME_INF;
	cfs_b->period = ns_to_ktime(default_cfs_period());

	INIT_LIST_HEAD(&cfs_b->throttled_cfs_rq);
	hrtimer_init(&cfs_b->period_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->period_timer.function = sched_cfs_period_timer;
	hrtimer_init(&cfs_b->slack_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->slack_timer.function = sched_cfs_slack_timer;
}

static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	cfs_rq->runtime_enabled = 0;
	INIT_LIST_HEAD(&cfs_rq->throttled_list);
}

/* requires cfs_b->lock, may release to reprogram timer */
void __start_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	/*
	 * The timer may be active because we're trying to set a new bandwidth
	 * period or because we're racing with the tear-down path
	 * (timer_active==0 becomes visible before the hrtimer call-back
	 * terminates).  In either case we ensure that it's re-programmed
	 */
	while (unlikely(hrtimer_active(&cfs_b->period_timer))) {
		raw_spin_unlock(&cfs_b->lock);
		/* ensure cfs_b->lock is available while we wait */
		hrtimer_cancel(&cfs_b->period_timer);

		raw_spin_lock(&cfs_b->lock);
		/* if someone else restarted the timer then we're done */
		if (cfs_b->timer_active)
			return;
	}

	cfs_b->timer_active = 1;
	start_bandwidth_timer(&cfs_b->period_timer, cfs_b->period);
}

static void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	hrtimer_cancel(&cfs_b->period_timer);
	hrtimer_cancel(&cfs_b->slack_timer);
}

static void unthrottle_offline_cfs_rqs(struct rq *rq)
{
	struct cfs_rq *cfs_rq;

	for_each_leaf_cfs_rq(rq, cfs_rq) {
		struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);

		if (!cfs_rq->runtime_enabled)
			continue;

		/*
		 * clock_task is not advancing so we just need to make sure
		 * there's some valid quota amount
		 */
		cfs_rq->runtime_remaining = cfs_b->quota;
		if (cfs_rq_throttled(cfs_rq))
			unthrottle_cfs_rq(cfs_rq);
	}
}

#else /* CONFIG_CFS_BANDWIDTH */
static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec) {}
static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
static void check_enqueue_throttle(struct cfs_rq *cfs_rq) {}
static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	return 0;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
#endif

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return NULL;
}
static inline void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}
static inline void unthrottle_offline_cfs_rqs(struct rq *rq) {}

#endif /* CONFIG_CFS_BANDWIDTH */

/**************************************************
 * CFS operations on tasks:
 */

#ifdef CONFIG_SCHED_HRTICK
static void hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	WARN_ON(task_rq(p) != rq);

	if (rq->cfs.h_nr_running > 1) {
		u64 slice = sched_slice(cfs_rq, se);
		u64 ran = se->sum_exec_runtime - se->prev_sum_exec_runtime;
		s64 delta = slice - ran;

		if (delta < 0) {
			if (rq->curr == p)
				resched_task(p);
			return;
		}

		/*
		 * Don't schedule slices shorter than 10000ns, that just
		 * doesn't make sense. Rely on vruntime for fairness.
		 */
		if (rq->curr != p)
			delta = max_t(s64, 10000LL, delta);

		hrtick_start(rq, delta);
	}
}

/*
 * called from enqueue/dequeue and updates the hrtick when the
 * current task is from our class.
 */
static void hrtick_update(struct rq *rq)
{
	struct task_struct *curr = rq->curr;

	if (!hrtick_enabled(rq) || curr->sched_class != &fair_sched_class)
		return;

	hrtick_start_fair(rq, curr);
}
#else /* !CONFIG_SCHED_HRTICK */
static inline void
hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
}

static inline void hrtick_update(struct rq *rq)
{
}
#endif

/*
 * The enqueue_task method is called before nr_running is
 * increased. Here we update the fair scheduling stats and
 * then put the task into the rbtree:
 */
static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;

	for_each_sched_entity(se) {
		if (se->on_rq)
			break;
		cfs_rq = cfs_rq_of(se);
		enqueue_entity(cfs_rq, se, flags);

		/*
		 * end evaluation on encountering a throttled cfs_rq
		 *
		 * note: in the case of encountering a throttled cfs_rq we will
		 * post the final h_nr_running increment below.
		*/
		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running++;

		flags = ENQUEUE_WAKEUP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running++;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_load(cfs_rq, 0);
		update_cfs_shares(cfs_rq);
	}

	if (!se)
		inc_nr_running(rq);
	hrtick_update(rq);
}

static void set_next_buddy(struct sched_entity *se);

/*
 * The dequeue_task method is called before nr_running is
 * decreased. We remove the task from the rbtree and
 * update the fair scheduling stats:
 */
static void dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;
	int task_sleep = flags & DEQUEUE_SLEEP;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		dequeue_entity(cfs_rq, se, flags);

		/*
		 * end evaluation on encountering a throttled cfs_rq
		 *
		 * note: in the case of encountering a throttled cfs_rq we will
		 * post the final h_nr_running decrement below.
		*/
		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running--;

		/* Don't dequeue parent if it has other entities besides us */
		if (cfs_rq->load.weight) {
			/*
			 * Bias pick_next to pick a task from this cfs_rq, as
			 * p is sleeping when it is within its sched_slice.
			 */
			if (task_sleep && parent_entity(se))
				set_next_buddy(parent_entity(se));

			/* avoid re-evaluating load for this entity */
			se = parent_entity(se);
			break;
		}
		flags |= DEQUEUE_SLEEP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running--;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_load(cfs_rq, 0);
		update_cfs_shares(cfs_rq);
	}

	if (!se)
		dec_nr_running(rq);
	hrtick_update(rq);
}

#ifdef CONFIG_SMP
/* Used instead of source_load when we know the type == 0 */
static unsigned long weighted_cpuload(const int cpu)
{
	return cpu_rq(cpu)->load.weight;
}

/*
 * Return a low guess at the load of a migration-source cpu weighted
 * according to the scheduling class and "nice" value.
 *
 * We want to under-estimate the load of migration sources, to
 * balance conservatively.
 */
static unsigned long source_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return min(rq->cpu_load[type-1], total);
}

/*
 * Return a high guess at the load of a migration-target cpu weighted
 * according to the scheduling class and "nice" value.
 */
static unsigned long target_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return max(rq->cpu_load[type-1], total);
}

static unsigned long power_of(int cpu)
{
	return cpu_rq(cpu)->cpu_power;
}

static unsigned long cpu_avg_load_per_task(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long nr_running = ACCESS_ONCE(rq->nr_running);

	if (nr_running)
		return rq->load.weight / nr_running;

	return 0;
}


static void task_waking_fair(struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 min_vruntime;

#ifndef CONFIG_64BIT
	u64 min_vruntime_copy;

	do {
		min_vruntime_copy = cfs_rq->min_vruntime_copy;
		smp_rmb();
		min_vruntime = cfs_rq->min_vruntime;
	} while (min_vruntime != min_vruntime_copy);
#else
	min_vruntime = cfs_rq->min_vruntime;
#endif

	se->vruntime -= min_vruntime;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
/*
 * effective_load() calculates the load change as seen from the root_task_group
 *
 * Adding load to a group doesn't make a group heavier, but can cause movement
 * of group shares between cpus. Assuming the shares were perfectly aligned one
 * can calculate the shift in shares.
 *
 * Calculate the effective load difference if @wl is added (subtracted) to @tg
 * on this @cpu and results in a total addition (subtraction) of @wg to the
 * total group weight.
 *
 * Given a runqueue weight distribution (rw_i) we can compute a shares
 * distribution (s_i) using:
 *
 *   s_i = rw_i / \Sum rw_j						(1)
 *
 * Suppose we have 4 CPUs and our @tg is a direct child of the root group and
 * has 7 equal weight tasks, distributed as below (rw_i), with the resulting
 * shares distribution (s_i):
 *
 *   rw_i = {   2,   4,   1,   0 }
 *   s_i  = { 2/7, 4/7, 1/7,   0 }
 *
 * As per wake_affine() we're interested in the load of two CPUs (the CPU the
 * task used to run on and the CPU the waker is running on), we need to
 * compute the effect of waking a task on either CPU and, in case of a sync
 * wakeup, compute the effect of the current task going to sleep.
 *
 * So for a change of @wl to the local @cpu with an overall group weight change
 * of @wl we can compute the new shares distribution (s'_i) using:
 *
 *   s'_i = (rw_i + @wl) / (@wg + \Sum rw_j)				(2)
 *
 * Suppose we're interested in CPUs 0 and 1, and want to compute the load
 * differences in waking a task to CPU 0. The additional task changes the
 * weight and shares distributions like:
 *
 *   rw'_i = {   3,   4,   1,   0 }
 *   s'_i  = { 3/8, 4/8, 1/8,   0 }
 *
 * We can then compute the difference in effective weight by using:
 *
 *   dw_i = S * (s'_i - s_i)						(3)
 *
 * Where 'S' is the group weight as seen by its parent.
 *
 * Therefore the effective change in loads on CPU 0 would be 5/56 (3/8 - 2/7)
 * times the weight of the group. The effect on CPU 1 would be -4/56 (4/8 -
 * 4/7) times the weight of the group.
 */
static long effective_load(struct task_group *tg, int cpu, long wl, long wg)
{
	struct sched_entity *se = tg->se[cpu];

	if (!tg->parent)	/* the trivial, non-cgroup case */
		return wl;

	for_each_sched_entity(se) {
		long w, W;

		tg = se->my_q->tg;

		/*
		 * W = @wg + \Sum rw_j
		 */
		W = wg + calc_tg_weight(tg, se->my_q);

		/*
		 * w = rw_i + @wl
		 */
		w = se->my_q->load.weight + wl;

		/*
		 * wl = S * s'_i; see (2)
		 */
		if (W > 0 && w < W)
			wl = (w * tg->shares) / W;
		else
			wl = tg->shares;

		/*
		 * Per the above, wl is the new se->load.weight value; since
		 * those are clipped to [MIN_SHARES, ...) do so now. See
		 * calc_cfs_shares().
		 */
		if (wl < MIN_SHARES)
			wl = MIN_SHARES;

		/*
		 * wl = dw_i = S * (s'_i - s_i); see (3)
		 */
		wl -= se->load.weight;

		/*
		 * Recursively apply this logic to all parent groups to compute
		 * the final effective load change on the root group. Since
		 * only the @tg group gets extra weight, all parent groups can
		 * only redistribute existing shares. @wl is the shift in shares
		 * resulting from this level per the above.
		 */
		wg = 0;
	}

	return wl;
}
#else

static inline unsigned long effective_load(struct task_group *tg, int cpu,
		unsigned long wl, unsigned long wg)
{
	return wl;
}

#endif

static int wake_affine(struct sched_domain *sd, struct task_struct *p, int sync)
{
	s64 this_load, load;
	int idx, this_cpu, prev_cpu;
	unsigned long tl_per_task;
	struct task_group *tg;
	unsigned long weight;
	int balanced;

	idx	  = sd->wake_idx;
	this_cpu  = smp_processor_id();
	prev_cpu  = task_cpu(p);
	load	  = source_load(prev_cpu, idx);
	this_load = target_load(this_cpu, idx);

	/*
	 * If sync wakeup then subtract the (maximum possible)
	 * effect of the currently running task from the load
	 * of the current CPU:
	 */
	if (sync) {
		tg = task_group(current);
		weight = current->se.load.weight;

		this_load += effective_load(tg, this_cpu, -weight, -weight);
		load += effective_load(tg, prev_cpu, 0, -weight);
	}

	tg = task_group(p);
	weight = p->se.load.weight;

	/*
	 * In low-load situations, where prev_cpu is idle and this_cpu is idle
	 * due to the sync cause above having dropped this_load to 0, we'll
	 * always have an imbalance, but there's really nothing you can do
	 * about that, so that's good too.
	 *
	 * Otherwise check if either cpus are near enough in load to allow this
	 * task to be woken on this_cpu.
	 */
	if (this_load > 0) {
		s64 this_eff_load, prev_eff_load;

		this_eff_load = 100;
		this_eff_load *= power_of(prev_cpu);
		this_eff_load *= this_load +
			effective_load(tg, this_cpu, weight, weight);

		prev_eff_load = 100 + (sd->imbalance_pct - 100) / 2;
		prev_eff_load *= power_of(this_cpu);
		prev_eff_load *= load + effective_load(tg, prev_cpu, 0, weight);

		balanced = this_eff_load <= prev_eff_load;
	} else
		balanced = true;

	/*
	 * If the currently running task will sleep within
	 * a reasonable amount of time then attract this newly
	 * woken task:
	 */
	if (sync && balanced)
		return 1;

	schedstat_inc(p, se.statistics.nr_wakeups_affine_attempts);
	tl_per_task = cpu_avg_load_per_task(this_cpu);

	if (balanced ||
	    (this_load <= load &&
	     this_load + target_load(prev_cpu, idx) <= tl_per_task)) {
		/*
		 * This domain has SD_WAKE_AFFINE and
		 * p is cache cold in this domain, and
		 * there is no bad imbalance.
		 */
		schedstat_inc(sd, ttwu_move_affine);
		schedstat_inc(p, se.statistics.nr_wakeups_affine);

		return 1;
	}
	return 0;
}

/*
 * find_idlest_group finds and returns the least busy CPU group within the
 * domain.
 */
static struct sched_group *
find_idlest_group(struct sched_domain *sd, struct task_struct *p,
		  int this_cpu, int load_idx)
{
	struct sched_group *idlest = NULL, *group = sd->groups;
	unsigned long min_load = ULONG_MAX, this_load = 0;
	int imbalance = 100 + (sd->imbalance_pct-100)/2;

	do {
		unsigned long load, avg_load;
		int local_group;
		int i;

		/* Skip over this group if it has no CPUs allowed */
		if (!cpumask_intersects(sched_group_cpus(group),
					tsk_cpus_allowed(p)))
			continue;

		local_group = cpumask_test_cpu(this_cpu,
					       sched_group_cpus(group));

		/* Tally up the load of all CPUs in the group */
		avg_load = 0;

		for_each_cpu(i, sched_group_cpus(group)) {
			/* Bias balancing toward cpus of our domain */
			if (local_group)
				load = source_load(i, load_idx);
			else
				load = target_load(i, load_idx);

			avg_load += load;
		}

		/* Adjust by relative CPU power of the group */
		avg_load = (avg_load * SCHED_POWER_SCALE) / group->sgp->power;

		if (local_group) {
			this_load = avg_load;
		} else if (avg_load < min_load) {
			min_load = avg_load;
			idlest = group;
		}
	} while (group = group->next, group != sd->groups);

	if (!idlest || 100*this_load < imbalance*min_load)
		return NULL;
	return idlest;
}

/*
 * find_idlest_cpu - find the idlest cpu among the cpus in group.
 */
static int
find_idlest_cpu(struct sched_group *group, struct task_struct *p, int this_cpu)
{
	unsigned long load, min_load = ULONG_MAX;
	int idlest = -1;
	int i;

	/* Traverse only the allowed CPUs */
	for_each_cpu_and(i, sched_group_cpus(group), tsk_cpus_allowed(p)) {
		load = weighted_cpuload(i);

		if (load < min_load || (load == min_load && i == this_cpu)) {
			min_load = load;
			idlest = i;
		}
	}

	return idlest;
}

/*
 * Try and locate an idle CPU in the sched_domain.
 */
static int select_idle_sibling(struct task_struct *p, int target)
{
	int cpu = smp_processor_id();
	int prev_cpu = task_cpu(p);
	struct sched_domain *sd;
	struct sched_group *sg;
	int i;

	/*
	 * If the task is going to be woken-up on this cpu and if it is
	 * already idle, then it is the right target.
	 */
	if (target == cpu && idle_cpu(cpu))
		return cpu;

	/*
	 * If the task is going to be woken-up on the cpu where it previously
	 * ran and if it is currently idle, then it the right target.
	 */
	if (target == prev_cpu && idle_cpu(prev_cpu))
		return prev_cpu;

	if (!sysctl_sched_wake_to_idle &&
	    !(current->flags & PF_WAKE_UP_IDLE) &&
	    !(p->flags & PF_WAKE_UP_IDLE))
		return target;

	/*
	 * Otherwise, iterate the domains and find an elegible idle cpu.
	 */
	sd = rcu_dereference(per_cpu(sd_llc, target));
	for_each_lower_domain(sd) {
		sg = sd->groups;
		do {
			if (!cpumask_intersects(sched_group_cpus(sg),
						tsk_cpus_allowed(p)))
				goto next;

			for_each_cpu(i, sched_group_cpus(sg)) {
				if (!idle_cpu(i))
					goto next;
			}

			target = cpumask_first_and(sched_group_cpus(sg),
					tsk_cpus_allowed(p));
			goto done;
next:
			sg = sg->next;
		} while (sg != sd->groups);
	}
done:
	return target;
}

/*
 * sched_balance_self: balance the current task (running on cpu) in domains
 * that have the 'flag' flag set. In practice, this is SD_BALANCE_FORK and
 * SD_BALANCE_EXEC.
 *
 * Balance, ie. select the least loaded group.
 *
 * Returns the target CPU number, or the same CPU if no balancing is needed.
 *
 * preempt must be disabled.
 */
static int
select_task_rq_fair(struct task_struct *p, int sd_flag, int wake_flags)
{
	struct sched_domain *tmp, *affine_sd = NULL, *sd = NULL;
	int cpu = smp_processor_id();
	int prev_cpu = task_cpu(p);
	int new_cpu = cpu;
	int want_affine = 0;
	int want_sd = 1;
	int sync = wake_flags & WF_SYNC;

	if (p->rt.nr_cpus_allowed == 1)
		return prev_cpu;

	if (sd_flag & SD_BALANCE_WAKE) {
		if (cpumask_test_cpu(cpu, tsk_cpus_allowed(p)))
			want_affine = 1;
		new_cpu = prev_cpu;
	}

	rcu_read_lock();
	for_each_domain(cpu, tmp) {
		if (!(tmp->flags & SD_LOAD_BALANCE))
			continue;

		/*
		 * If power savings logic is enabled for a domain, see if we
		 * are not overloaded, if so, don't balance wider.
		 */
		if (tmp->flags & (SD_POWERSAVINGS_BALANCE|SD_PREFER_LOCAL)) {
			unsigned long power = 0;
			unsigned long nr_running = 0;
			unsigned long capacity;
			int i;

			for_each_cpu(i, sched_domain_span(tmp)) {
				power += power_of(i);
				nr_running += cpu_rq(i)->cfs.nr_running;
			}

			capacity = DIV_ROUND_CLOSEST(power, SCHED_POWER_SCALE);

			if (tmp->flags & SD_POWERSAVINGS_BALANCE)
				nr_running /= 2;

			if (nr_running < capacity)
				want_sd = 0;
		}

		/*
		 * If both cpu and prev_cpu are part of this domain,
		 * cpu is a valid SD_WAKE_AFFINE target.
		 */
		if (want_affine && (tmp->flags & SD_WAKE_AFFINE) &&
		    cpumask_test_cpu(prev_cpu, sched_domain_span(tmp))) {
			affine_sd = tmp;
			want_affine = 0;
		}

		if (!want_sd && !want_affine)
			break;

		if (!(tmp->flags & sd_flag))
			continue;

		if (want_sd)
			sd = tmp;
	}

	if (affine_sd) {
		if (cpu == prev_cpu || wake_affine(affine_sd, p, sync))
			prev_cpu = cpu;

		new_cpu = select_idle_sibling(p, prev_cpu);
		goto unlock;
	}

	while (sd) {
		int load_idx = sd->forkexec_idx;
		struct sched_group *group;
		int weight;

		if (!(sd->flags & sd_flag)) {
			sd = sd->child;
			continue;
		}

		if (sd_flag & SD_BALANCE_WAKE)
			load_idx = sd->wake_idx;

		group = find_idlest_group(sd, p, cpu, load_idx);
		if (!group) {
			sd = sd->child;
			continue;
		}

		new_cpu = find_idlest_cpu(group, p, cpu);
		if (new_cpu == -1 || new_cpu == cpu) {
			/* Now try balancing at a lower domain level of cpu */
			sd = sd->child;
			continue;
		}

		/* Now try balancing at a lower domain level of new_cpu */
		cpu = new_cpu;
		weight = sd->span_weight;
		sd = NULL;
		for_each_domain(cpu, tmp) {
			if (weight <= tmp->span_weight)
				break;
			if (tmp->flags & sd_flag)
				sd = tmp;
		}
		/* while loop will break here if sd == NULL */
	}
unlock:
	rcu_read_unlock();

	return new_cpu;
}
#endif /* CONFIG_SMP */

static unsigned long
wakeup_gran(struct sched_entity *curr, struct sched_entity *se)
{
	unsigned long gran = sysctl_sched_wakeup_granularity;

	/*
	 * Since its curr running now, convert the gran from real-time
	 * to virtual-time in his units.
	 *
	 * By using 'se' instead of 'curr' we penalize light tasks, so
	 * they get preempted easier. That is, if 'se' < 'curr' then
	 * the resulting gran will be larger, therefore penalizing the
	 * lighter, if otoh 'se' > 'curr' then the resulting gran will
	 * be smaller, again penalizing the lighter task.
	 *
	 * This is especially important for buddies when the leftmost
	 * task is higher priority than the buddy.
	 */
	return calc_delta_fair(gran, se);
}

/*
 * Should 'se' preempt 'curr'.
 *
 *             |s1
 *        |s2
 *   |s3
 *         g
 *      |<--->|c
 *
 *  w(c, s1) = -1
 *  w(c, s2) =  0
 *  w(c, s3) =  1
 *
 */
static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se)
{
	s64 gran, vdiff = curr->vruntime - se->vruntime;

	if (vdiff <= 0)
		return -1;

	gran = wakeup_gran(curr, se);
	if (vdiff > gran)
		return 1;

	return 0;
}

static void set_last_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->last = se;
}

static void set_next_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->next = se;
}

static void set_skip_buddy(struct sched_entity *se)
{
	for_each_sched_entity(se)
		cfs_rq_of(se)->skip = se;
}

/*
 * Preempt the current task with a newly woken task if needed:
 */
static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = rq->curr;
	struct sched_entity *se = &curr->se, *pse = &p->se;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	int scale = cfs_rq->nr_running >= sched_nr_latency;
	int next_buddy_marked = 0;

	if (unlikely(se == pse))
		return;

	/*
	 * This is possible from callers such as move_task(), in which we
	 * unconditionally check_prempt_curr() after an enqueue (which may have
	 * lead to a throttle).  This both saves work and prevents false
	 * next-buddy nomination below.
	 */
	if (unlikely(throttled_hierarchy(cfs_rq_of(pse))))
		return;

	if (sched_feat(NEXT_BUDDY) && scale && !(wake_flags & WF_FORK)) {
		set_next_buddy(pse);
		next_buddy_marked = 1;
	}

	/*
	 * We can come here with TIF_NEED_RESCHED already set from new task
	 * wake up path.
	 *
	 * Note: this also catches the edge-case of curr being in a throttled
	 * group (e.g. via set_curr_task), since update_curr() (in the
	 * enqueue of curr) will have resulted in resched being set.  This
	 * prevents us from potentially nominating it as a false LAST_BUDDY
	 * below.
	 */
	if (test_tsk_need_resched(curr))
		return;

	/* Idle tasks are by definition preempted by non-idle tasks. */
	if (unlikely(curr->policy == SCHED_IDLE) &&
	    likely(p->policy != SCHED_IDLE))
		goto preempt;

	/*
	 * Batch and idle tasks do not preempt non-idle tasks (their preemption
	 * is driven by the tick):
	 */
	if (unlikely(p->policy != SCHED_NORMAL))
		return;

	find_matching_se(&se, &pse);
	update_curr(cfs_rq_of(se));
	BUG_ON(!pse);
	if (wakeup_preempt_entity(se, pse) == 1) {
		/*
		 * Bias pick_next to pick the sched entity that is
		 * triggering this preemption.
		 */
		if (!next_buddy_marked)
			set_next_buddy(pse);
		goto preempt;
	}

	return;

preempt:
	resched_task(curr);
	/*
	 * Only set the backward buddy when the current task is still
	 * on the rq. This can happen when a wakeup gets interleaved
	 * with schedule on the ->pre_schedule() or idle_balance()
	 * point, either of which can * drop the rq lock.
	 *
	 * Also, during early boot the idle thread is in the fair class,
	 * for obvious reasons its a bad idea to schedule back to it.
	 */
	if (unlikely(!se->on_rq || curr == rq->idle))
		return;

	if (sched_feat(LAST_BUDDY) && scale && entity_is_task(se))
		set_last_buddy(se);
}

static struct task_struct *pick_next_task_fair(struct rq *rq)
{
	struct task_struct *p;
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;

	if (!cfs_rq->nr_running)
		return NULL;

	do {
		se = pick_next_entity(cfs_rq);
		set_next_entity(cfs_rq, se);
		cfs_rq = group_cfs_rq(se);
	} while (cfs_rq);

	p = task_of(se);
	if (hrtick_enabled(rq))
		hrtick_start_fair(rq, p);

	return p;
}

/*
 * Account for a descheduled task:
 */
static void put_prev_task_fair(struct rq *rq, struct task_struct *prev)
{
	struct sched_entity *se = &prev->se;
	struct cfs_rq *cfs_rq;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		put_prev_entity(cfs_rq, se);
	}
}

/*
 * sched_yield() is very simple
 *
 * The magic of dealing with the ->skip buddy is in pick_next_entity.
 */
static void yield_task_fair(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	struct sched_entity *se = &curr->se;

	/*
	 * Are we the only task in the tree?
	 */
	if (unlikely(rq->nr_running == 1))
		return;

	clear_buddies(cfs_rq, se);

	if (curr->policy != SCHED_BATCH) {
		update_rq_clock(rq);
		/*
		 * Update run-time statistics of the 'current'.
		 */
		update_curr(cfs_rq);
		/*
		 * Tell update_rq_clock() that we've just updated,
		 * so we don't do microscopic update in schedule()
		 * and double the fastpath cost.
		 */
		 rq->skip_clock_update = 1;
	}

	set_skip_buddy(se);
}

static bool yield_to_task_fair(struct rq *rq, struct task_struct *p, bool preempt)
{
	struct sched_entity *se = &p->se;

	/* throttled hierarchies are not runnable */
	if (!se->on_rq || throttled_hierarchy(cfs_rq_of(se)))
		return false;

	/* Tell the scheduler that we'd really like pse to run next. */
	set_next_buddy(se);

	yield_task_fair(rq);

	return true;
}

#ifdef CONFIG_SMP
/**************************************************
 * Fair scheduling class load-balancing methods:
 */

static unsigned long __read_mostly max_load_balance_interval = HZ/10;

#define LBF_ALL_PINNED	0x01
#define LBF_NEED_BREAK	0x02

struct lb_env {
	struct sched_domain	*sd;

	int			src_cpu;
	struct rq		*src_rq;

	int			dst_cpu;
	struct rq		*dst_rq;

	enum cpu_idle_type	idle;
	long			load_move;
	unsigned int		flags;

	unsigned int		loop;
	unsigned int		loop_break;
	unsigned int		loop_max;
};

static DEFINE_PER_CPU(bool, dbs_boost_needed);

/*
 * move_task - move a task from one runqueue to another runqueue.
 * Both runqueues must be locked.
 */
static void move_task(struct task_struct *p, struct lb_env *env)
{
	deactivate_task(env->src_rq, p, 0);
	set_task_cpu(p, env->dst_cpu);
	activate_task(env->dst_rq, p, 0);
	check_preempt_curr(env->dst_rq, p, 0);
	if (task_notify_on_migrate(p))
		per_cpu(dbs_boost_needed, env->dst_cpu) = true;
}

/*
 * Is this task likely cache-hot:
 */
static int
task_hot(struct task_struct *p, u64 now, struct sched_domain *sd)
{
	s64 delta;

	if (p->sched_class != &fair_sched_class)
		return 0;

	if (unlikely(p->policy == SCHED_IDLE))
		return 0;

	/*
	 * Buddy candidates are cache hot:
	 */
	if (sched_feat(CACHE_HOT_BUDDY) && this_rq()->nr_running &&
			(&p->se == cfs_rq_of(&p->se)->next ||
			 &p->se == cfs_rq_of(&p->se)->last))
		return 1;

	if (sysctl_sched_migration_cost == -1)
		return 1;
	if (sysctl_sched_migration_cost == 0)
		return 0;

	delta = now - p->se.exec_start;

	return delta < (s64)sysctl_sched_migration_cost;
}

/*
 * can_migrate_task - may task p from runqueue rq be migrated to this_cpu?
 */
static
int can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot = 0;
	/*
	 * We do not migrate tasks that are:
	 * 1) running (obviously), or
	 * 2) cannot be migrated to this CPU due to cpus_allowed, or
	 * 3) are cache-hot on their current CPU.
	 */
	if (!cpumask_test_cpu(env->dst_cpu, tsk_cpus_allowed(p))) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_affine);
		return 0;
	}
	env->flags &= ~LBF_ALL_PINNED;

	if (task_running(env->src_rq, p)) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_running);
		return 0;
	}

	/*
	 * Aggressive migration if:
	 * 1) task is cache cold, or
	 * 2) too many balance attempts have failed.
	 */

	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd);
	if (!tsk_cache_hot ||
		env->sd->nr_balance_failed > env->sd->cache_nice_tries) {
#ifdef CONFIG_SCHEDSTATS
		if (tsk_cache_hot) {
			schedstat_inc(env->sd, lb_hot_gained[env->idle]);
			schedstat_inc(p, se.statistics.nr_forced_migrations);
		}
#endif
		return 1;
	}

	if (tsk_cache_hot) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_hot);
		return 0;
	}
	return 1;
}

/*
 * move_one_task tries to move exactly one task from busiest to this_rq, as
 * part of active balancing operations within "domain".
 * Returns 1 if successful and 0 otherwise.
 *
 * Called with both runqueues locked.
 */
static int move_one_task(struct lb_env *env)
{
	struct task_struct *p, *n;

	list_for_each_entry_safe(p, n, &env->src_rq->cfs_tasks, se.group_node) {
		if (throttled_lb_pair(task_group(p), env->src_rq->cpu, env->dst_cpu))
			continue;

		if (!can_migrate_task(p, env))
			continue;

		move_task(p, env);
		/*
		 * Right now, this is only the second place move_task()
		 * is called, so we can safely collect move_task()
		 * stats here rather than inside move_task().
		 */
		schedstat_inc(env->sd, lb_gained[env->idle]);
		return 1;
	}
	return 0;
}

static unsigned long task_h_load(struct task_struct *p);

static const unsigned int sched_nr_migrate_break = 32;

/*
 * move_tasks tries to move up to load_move weighted load from busiest to
 * this_rq, as part of a balancing operation within domain "sd".
 * Returns 1 if successful and 0 otherwise.
 *
 * Called with both runqueues locked.
 */
static int move_tasks(struct lb_env *env)
{
	struct list_head *tasks = &env->src_rq->cfs_tasks;
	struct task_struct *p;
	unsigned long load;
	int pulled = 0;

	if (env->load_move <= 0)
		return 0;

	while (!list_empty(tasks)) {
		p = list_first_entry(tasks, struct task_struct, se.group_node);

		env->loop++;
		/* We've more or less seen every task there is, call it quits */
		if (env->loop > env->loop_max)
			break;

		/* take a breather every nr_migrate tasks */
		if (env->loop > env->loop_break) {
			env->loop_break += sched_nr_migrate_break;
			env->flags |= LBF_NEED_BREAK;
			break;
		}

		if (throttled_lb_pair(task_group(p), env->src_cpu, env->dst_cpu))
			goto next;

		load = task_h_load(p);

		if (sched_feat(LB_MIN) && load < 16 && !env->sd->nr_balance_failed)
			goto next;

		if ((load / 2) > env->load_move)
			goto next;

		if (!can_migrate_task(p, env))
			goto next;

		move_task(p, env);
		pulled++;
		env->load_move -= load;

#ifdef CONFIG_PREEMPT
		/*
		 * NEWIDLE balancing is a source of latency, so preemptible
		 * kernels will stop after the first task is pulled to minimize
		 * the critical section.
		 */
		if (env->idle == CPU_NEWLY_IDLE)
			break;
#endif

		/*
		 * We only want to steal up to the prescribed amount of
		 * weighted load.
		 */
		if (env->load_move <= 0)
			break;

		continue;
next:
		list_move_tail(&p->se.group_node, tasks);
	}

	/*
	 * Right now, this is one of only two places move_task() is called,
	 * so we can safely collect move_task() stats here rather than
	 * inside move_task().
	 */
	schedstat_add(env->sd, lb_gained[env->idle], pulled);

	return pulled;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
/*
 * update tg->load_weight by folding this cpu's load_avg
 */
static int update_shares_cpu(struct task_group *tg, int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned long flags;
	struct rq *rq;

	if (!tg->se[cpu])
		return 0;

	rq = cpu_rq(cpu);
	cfs_rq = tg->cfs_rq[cpu];

	raw_spin_lock_irqsave(&rq->lock, flags);

	update_rq_clock(rq);
	update_cfs_load(cfs_rq, 1);

	/*
	 * We need to update shares after updating tg->load_weight in
	 * order to adjust the weight of groups with long running tasks.
	 */
	update_cfs_shares(cfs_rq);

	raw_spin_unlock_irqrestore(&rq->lock, flags);

	return 0;
}

static void update_shares(int cpu)
{
	struct cfs_rq *cfs_rq;
	struct rq *rq = cpu_rq(cpu);

	rcu_read_lock();
	/*
	 * Iterates the task_group tree in a bottom up fashion, see
	 * list_add_leaf_cfs_rq() for details.
	 */
	for_each_leaf_cfs_rq(rq, cfs_rq) {
		/* throttled entities do not contribute to load */
		if (throttled_hierarchy(cfs_rq))
			continue;

		update_shares_cpu(cfs_rq->tg, cpu);
	}
	rcu_read_unlock();
}

/*
 * Compute the cpu's hierarchical load factor for each task group.
 * This needs to be done in a top-down fashion because the load of a child
 * group is a fraction of its parents load.
 */
static int tg_load_down(struct task_group *tg, void *data)
{
	unsigned long load;
	long cpu = (long)data;

	if (!tg->parent) {
		load = cpu_rq(cpu)->load.weight;
	} else {
		load = tg->parent->cfs_rq[cpu]->h_load;
		load *= tg->se[cpu]->load.weight;
		load /= tg->parent->cfs_rq[cpu]->load.weight + 1;
	}

	tg->cfs_rq[cpu]->h_load = load;

	return 0;
}

static void update_h_load(long cpu)
{
	rcu_read_lock();
	walk_tg_tree(tg_load_down, tg_nop, (void *)cpu);
	rcu_read_unlock();
}

static unsigned long task_h_load(struct task_struct *p)
{
	struct cfs_rq *cfs_rq = task_cfs_rq(p);
	unsigned long load;

	load = p->se.load.weight;
	load = div_u64(load * cfs_rq->h_load, cfs_rq->load.weight + 1);

	return load;
}
#else
static inline void update_shares(int cpu)
{
}

static inline void update_h_load(long cpu)
{
}

static unsigned long task_h_load(struct task_struct *p)
{
	return p->se.load.weight;
}
#endif

/********** Helpers for find_busiest_group ************************/
/*
 * sd_lb_stats - Structure to store the statistics of a sched_domain
 * 		during load balancing.
 */
struct sd_lb_stats {
	struct sched_group *busiest; /* Busiest group in this sd */
	struct sched_group *this;  /* Local group in this sd */
	unsigned long total_load;  /* Total load of all groups in sd */
	unsigned long total_pwr;   /*	Total power of all groups in sd */
	unsigned long avg_load;	   /* Average load across all groups in sd */

	/** Statistics of this group */
	unsigned long this_load;
	unsigned long this_load_per_task;
	unsigned long this_nr_running;
	unsigned long this_has_capacity;
	unsigned int  this_idle_cpus;

	/* Statistics of the busiest group */
	unsigned int  busiest_idle_cpus;
	unsigned long max_load;
	unsigned long busiest_load_per_task;
	unsigned long busiest_nr_running;
	unsigned long busiest_group_capacity;
	unsigned long busiest_has_capacity;
	unsigned int  busiest_group_weight;

	int group_imb; /* Is there imbalance in this sd */
#if defined(CONFIG_SCHED_MC) || defined(CONFIG_SCHED_SMT)
	int power_savings_balance; /* Is powersave balance needed for this sd */
	struct sched_group *group_min; /* Least loaded group in sd */
	struct sched_group *group_leader; /* Group which relieves group_min */
	unsigned long min_load_per_task; /* load_per_task in group_min */
	unsigned long leader_nr_running; /* Nr running of group_leader */
	unsigned long min_nr_running; /* Nr running of group_min */
#endif
};

/*
 * sg_lb_stats - stats of a sched_group required for load_balancing
 */
struct sg_lb_stats {
	unsigned long avg_load; /*Avg load across the CPUs of the group */
	unsigned long group_load; /* Total load over the CPUs of the group */
	unsigned long sum_nr_running; /* Nr tasks running in the group */
	unsigned long sum_weighted_load; /* Weighted load of group's tasks */
	unsigned long group_capacity;
	unsigned long idle_cpus;
	unsigned long group_weight;
	int group_imb; /* Is there an imbalance in the group ? */
	int group_has_capacity; /* Is there extra capacity in the group? */
};

/**
 * get_sd_load_idx - Obtain the load index for a given sched domain.
 * @sd: The sched_domain whose load_idx is to be obtained.
 * @idle: The Idle status of the CPU for whose sd load_icx is obtained.
 */
static inline int get_sd_load_idx(struct sched_domain *sd,
					enum cpu_idle_type idle)
{
	int load_idx;

	switch (idle) {
	case CPU_NOT_IDLE:
		load_idx = sd->busy_idx;
		break;

	case CPU_NEWLY_IDLE:
		load_idx = sd->newidle_idx;
		break;
	default:
		load_idx = sd->idle_idx;
		break;
	}

	return load_idx;
}


#if defined(CONFIG_SCHED_MC) || defined(CONFIG_SCHED_SMT)
/**
 * init_sd_power_savings_stats - Initialize power savings statistics for
 * the given sched_domain, during load balancing.
 *
 * @sd: Sched domain whose power-savings statistics are to be initialized.
 * @sds: Variable containing the statistics for sd.
 * @idle: Idle status of the CPU at which we're performing load-balancing.
 */
static inline void init_sd_power_savings_stats(struct sched_domain *sd,
	struct sd_lb_stats *sds, enum cpu_idle_type idle)
{
	/*
	 * Busy processors will not participate in power savings
	 * balance.
	 */
	if (idle == CPU_NOT_IDLE || !(sd->flags & SD_POWERSAVINGS_BALANCE))
		sds->power_savings_balance = 0;
	else {
		sds->power_savings_balance = 1;
		sds->min_nr_running = ULONG_MAX;
		sds->leader_nr_running = 0;
	}
}

/**
 * update_sd_power_savings_stats - Update the power saving stats for a
 * sched_domain while performing load balancing.
 *
 * @group: sched_group belonging to the sched_domain under consideration.
 * @sds: Variable containing the statistics of the sched_domain
 * @local_group: Does group contain the CPU for which we're performing
 * 		load balancing ?
 * @sgs: Variable containing the statistics of the group.
 */
static inline void update_sd_power_savings_stats(struct sched_group *group,
	struct sd_lb_stats *sds, int local_group, struct sg_lb_stats *sgs)
{

	if (!sds->power_savings_balance)
		return;

	/*
	 * If the local group is idle or completely loaded
	 * no need to do power savings balance at this domain
	 */
	if (local_group && (sds->this_nr_running >= sgs->group_capacity ||
				!sds->this_nr_running))
		sds->power_savings_balance = 0;

	/*
	 * If a group is already running at full capacity or idle,
	 * don't include that group in power savings calculations
	 */
	if (!sds->power_savings_balance ||
		sgs->sum_nr_running >= sgs->group_capacity ||
		!sgs->sum_nr_running)
		return;

	/*
	 * Calculate the group which has the least non-idle load.
	 * This is the group from where we need to pick up the load
	 * for saving power
	 */
	if ((sgs->sum_nr_running < sds->min_nr_running) ||
	    (sgs->sum_nr_running == sds->min_nr_running &&
	     group_first_cpu(group) > group_first_cpu(sds->group_min))) {
		sds->group_min = group;
		sds->min_nr_running = sgs->sum_nr_running;
		sds->min_load_per_task = sgs->sum_weighted_load /
						sgs->sum_nr_running;
	}

	/*
	 * Calculate the group which is almost near its
	 * capacity but still has some space to pick up some load
	 * from other group and save more power
	 */
	if (sgs->sum_nr_running + 1 > sgs->group_capacity)
		return;

	if (sgs->sum_nr_running > sds->leader_nr_running ||
	    (sgs->sum_nr_running == sds->leader_nr_running &&
	     group_first_cpu(group) < group_first_cpu(sds->group_leader))) {
		sds->group_leader = group;
		sds->leader_nr_running = sgs->sum_nr_running;
	}
}

/**
 * check_power_save_busiest_group - see if there is potential for some power-savings balance
 * @sds: Variable containing the statistics of the sched_domain
 *	under consideration.
 * @this_cpu: Cpu at which we're currently performing load-balancing.
 * @imbalance: Variable to store the imbalance.
 *
 * Description:
 * Check if we have potential to perform some power-savings balance.
 * If yes, set the busiest group to be the least loaded group in the
 * sched_domain, so that it's CPUs can be put to idle.
 *
 * Returns 1 if there is potential to perform power-savings balance.
 * Else returns 0.
 */
static inline int check_power_save_busiest_group(struct sd_lb_stats *sds,
					int this_cpu, unsigned long *imbalance)
{
	if (!sds->power_savings_balance)
		return 0;

	if (sds->this != sds->group_leader ||
			sds->group_leader == sds->group_min)
		return 0;

	*imbalance = sds->min_load_per_task;
	sds->busiest = sds->group_min;

	return 1;

}
#else /* CONFIG_SCHED_MC || CONFIG_SCHED_SMT */
static inline void init_sd_power_savings_stats(struct sched_domain *sd,
	struct sd_lb_stats *sds, enum cpu_idle_type idle)
{
	return;
}

static inline void update_sd_power_savings_stats(struct sched_group *group,
	struct sd_lb_stats *sds, int local_group, struct sg_lb_stats *sgs)
{
	return;
}

static inline int check_power_save_busiest_group(struct sd_lb_stats *sds,
					int this_cpu, unsigned long *imbalance)
{
	return 0;
}
#endif /* CONFIG_SCHED_MC || CONFIG_SCHED_SMT */


unsigned long default_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return SCHED_POWER_SCALE;
}

unsigned long __weak arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return default_scale_freq_power(sd, cpu);
}

unsigned long default_scale_smt_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long smt_gain = sd->smt_gain;

	smt_gain /= weight;

	return smt_gain;
}

unsigned long __weak arch_scale_smt_power(struct sched_domain *sd, int cpu)
{
	return default_scale_smt_power(sd, cpu);
}

unsigned long scale_rt_power(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	u64 total, available, age_stamp, avg;

	/*
	 * Since we're reading these variables without serialization make sure
	 * we read them once before doing sanity checks on them.
	 */
	age_stamp = ACCESS_ONCE(rq->age_stamp);
	avg = ACCESS_ONCE(rq->rt_avg);

	total = sched_avg_period() + (rq->clock - age_stamp);

	if (unlikely(total < avg)) {
		/* Ensures that power won't end up being negative */
		available = 0;
	} else {
		available = total - avg;
	}

	if (unlikely((s64)total < SCHED_POWER_SCALE))
		total = SCHED_POWER_SCALE;

	total >>= SCHED_POWER_SHIFT;

	return div_u64(available, total);
}

static void update_cpu_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long power = SCHED_POWER_SCALE;
	struct sched_group *sdg = sd->groups;

	if ((sd->flags & SD_SHARE_CPUPOWER) && weight > 1) {
		if (sched_feat(ARCH_POWER))
			power *= arch_scale_smt_power(sd, cpu);
		else
			power *= default_scale_smt_power(sd, cpu);

		power >>= SCHED_POWER_SHIFT;
	}

	sdg->sgp->power_orig = power;

	if (sched_feat(ARCH_POWER))
		power *= arch_scale_freq_power(sd, cpu);
	else
		power *= default_scale_freq_power(sd, cpu);

	power >>= SCHED_POWER_SHIFT;

	power *= scale_rt_power(cpu);
	power >>= SCHED_POWER_SHIFT;

	if (!power)
		power = 1;

	cpu_rq(cpu)->cpu_power = power;
	sdg->sgp->power = power;
}

void update_group_power(struct sched_domain *sd, int cpu)
{
	struct sched_domain *child = sd->child;
	struct sched_group *group, *sdg = sd->groups;
	unsigned long power;
	unsigned long interval;

	interval = msecs_to_jiffies(sd->balance_interval);
	interval = clamp(interval, 1UL, max_load_balance_interval);
	sdg->sgp->next_update = jiffies + interval;

	if (!child) {
		update_cpu_power(sd, cpu);
		return;
	}

	power = 0;

	group = child->groups;
	do {
		power += group->sgp->power;
		group = group->next;
	} while (group != child->groups);

	sdg->sgp->power = power;
}

/*
 * Try and fix up capacity for tiny siblings, this is needed when
 * things like SD_ASYM_PACKING need f_b_g to select another sibling
 * which on its own isn't powerful enough.
 *
 * See update_sd_pick_busiest() and check_asym_packing().
 */
static inline int
fix_small_capacity(struct sched_domain *sd, struct sched_group *group)
{
	/*
	 * Only siblings can have significantly less than SCHED_POWER_SCALE
	 */
	if (!(sd->flags & SD_SHARE_CPUPOWER))
		return 0;

	/*
	 * If ~90% of the cpu_power is still there, we're good.
	 */
	if (group->sgp->power * 32 > group->sgp->power_orig * 29)
		return 1;

	return 0;
}

/**
 * update_sg_lb_stats - Update sched_group's statistics for load balancing.
 * @sd: The sched_domain whose statistics are to be updated.
 * @group: sched_group whose statistics are to be updated.
 * @this_cpu: Cpu for which load balance is currently performed.
 * @idle: Idle status of this_cpu
 * @load_idx: Load index of sched_domain of this_cpu for load calc.
 * @local_group: Does group contain this_cpu.
 * @cpus: Set of cpus considered for load balancing.
 * @balance: Should we balance.
 * @sgs: variable to hold the statistics for this group.
 */
static inline void update_sg_lb_stats(struct sched_domain *sd,
			struct sched_group *group, int this_cpu,
			enum cpu_idle_type idle, int load_idx,
			int local_group, const struct cpumask *cpus,
			int *balance, struct sg_lb_stats *sgs)
{
	unsigned long load, max_cpu_load, min_cpu_load, max_nr_running;
	int i;
	unsigned int balance_cpu = -1, first_idle_cpu = 0;
	unsigned long avg_load_per_task = 0;

	if (local_group)
		balance_cpu = group_first_cpu(group);

	/* Tally up the load of all CPUs in the group */
	max_cpu_load = 0;
	min_cpu_load = ~0UL;
	max_nr_running = 0;

	for_each_cpu_and(i, sched_group_cpus(group), cpus) {
		struct rq *rq = cpu_rq(i);

		/* Bias balancing toward cpus of our domain */
		if (local_group) {
			if (idle_cpu(i) && !first_idle_cpu) {
				first_idle_cpu = 1;
				balance_cpu = i;
			}

			load = target_load(i, load_idx);
		} else {
			load = source_load(i, load_idx);
			if (load > max_cpu_load) {
				max_cpu_load = load;
				max_nr_running = rq->nr_running;
			}
			if (min_cpu_load > load)
				min_cpu_load = load;
		}

		sgs->group_load += load;
		sgs->sum_nr_running += rq->nr_running;
		sgs->sum_weighted_load += weighted_cpuload(i);
		if (idle_cpu(i))
			sgs->idle_cpus++;
	}

	/*
	 * First idle cpu or the first cpu(busiest) in this sched group
	 * is eligible for doing load balancing at this and above
	 * domains. In the newly idle case, we will allow all the cpu's
	 * to do the newly idle load balance.
	 */
	if (local_group) {
		if (idle != CPU_NEWLY_IDLE) {
			if (balance_cpu != this_cpu) {
				*balance = 0;
				return;
			}
			update_group_power(sd, this_cpu);
		} else if (time_after_eq(jiffies, group->sgp->next_update))
			update_group_power(sd, this_cpu);
	}

	/* Adjust by relative CPU power of the group */
	sgs->avg_load = (sgs->group_load*SCHED_POWER_SCALE) / group->sgp->power;

	/*
	 * Consider the group unbalanced when the imbalance is larger
	 * than the average weight of a task.
	 *
	 * APZ: with cgroup the avg task weight can vary wildly and
	 *      might not be a suitable number - should we keep a
	 *      normalized nr_running number somewhere that negates
	 *      the hierarchy?
	 */
	if (sgs->sum_nr_running)
		avg_load_per_task = sgs->sum_weighted_load / sgs->sum_nr_running;

	if ((max_cpu_load - min_cpu_load) >= avg_load_per_task && max_nr_running > 1)
		sgs->group_imb = 1;

	sgs->group_capacity = DIV_ROUND_CLOSEST(group->sgp->power,
						SCHED_POWER_SCALE);
	if (!sgs->group_capacity)
		sgs->group_capacity = fix_small_capacity(sd, group);
	sgs->group_weight = group->group_weight;

	if (sgs->group_capacity > sgs->sum_nr_running)
		sgs->group_has_capacity = 1;
}

/**
 * update_sd_pick_busiest - return 1 on busiest group
 * @sd: sched_domain whose statistics are to be checked
 * @sds: sched_domain statistics
 * @sg: sched_group candidate to be checked for being the busiest
 * @sgs: sched_group statistics
 * @this_cpu: the current cpu
 *
 * Determine if @sg is a busier group than the previously selected
 * busiest group.
 */
static bool update_sd_pick_busiest(struct sched_domain *sd,
				   struct sd_lb_stats *sds,
				   struct sched_group *sg,
				   struct sg_lb_stats *sgs,
				   int this_cpu)
{
	if (sgs->avg_load <= sds->max_load)
		return false;

	if (sgs->sum_nr_running > sgs->group_capacity)
		return true;

	if (sgs->group_imb)
		return true;

	/*
	 * ASYM_PACKING needs to move all the work to the lowest
	 * numbered CPUs in the group, therefore mark all groups
	 * higher than ourself as busy.
	 */
	if ((sd->flags & SD_ASYM_PACKING) && sgs->sum_nr_running &&
	    this_cpu < group_first_cpu(sg)) {
		if (!sds->busiest)
			return true;

		if (group_first_cpu(sds->busiest) > group_first_cpu(sg))
			return true;
	}

	return false;
}

/**
 * update_sd_lb_stats - Update sched_domain's statistics for load balancing.
 * @sd: sched_domain whose statistics are to be updated.
 * @this_cpu: Cpu for which load balance is currently performed.
 * @idle: Idle status of this_cpu
 * @cpus: Set of cpus considered for load balancing.
 * @balance: Should we balance.
 * @sds: variable to hold the statistics for this sched_domain.
 */
static inline void update_sd_lb_stats(struct sched_domain *sd, int this_cpu,
			enum cpu_idle_type idle, const struct cpumask *cpus,
			int *balance, struct sd_lb_stats *sds)
{
	struct sched_domain *child = sd->child;
	struct sched_group *sg = sd->groups;
	struct sg_lb_stats sgs;
	int load_idx, prefer_sibling = 0;

	if (child && child->flags & SD_PREFER_SIBLING)
		prefer_sibling = 1;

	init_sd_power_savings_stats(sd, sds, idle);
	load_idx = get_sd_load_idx(sd, idle);

	do {
		int local_group;

		local_group = cpumask_test_cpu(this_cpu, sched_group_cpus(sg));
		memset(&sgs, 0, sizeof(sgs));
		update_sg_lb_stats(sd, sg, this_cpu, idle, load_idx,
				local_group, cpus, balance, &sgs);

		if (local_group && !(*balance))
			return;

		sds->total_load += sgs.group_load;
		sds->total_pwr += sg->sgp->power;

		/*
		 * In case the child domain prefers tasks go to siblings
		 * first, lower the sg capacity to one so that we'll try
		 * and move all the excess tasks away. We lower the capacity
		 * of a group only if the local group has the capacity to fit
		 * these excess tasks, i.e. nr_running < group_capacity. The
		 * extra check prevents the case where you always pull from the
		 * heaviest group when it is already under-utilized (possible
		 * with a large weight task outweighs the tasks on the system).
		 */
		if (prefer_sibling && !local_group && sds->this_has_capacity)
			sgs.group_capacity = min(sgs.group_capacity, 1UL);

		if (local_group) {
			sds->this_load = sgs.avg_load;
			sds->this = sg;
			sds->this_nr_running = sgs.sum_nr_running;
			sds->this_load_per_task = sgs.sum_weighted_load;
			sds->this_has_capacity = sgs.group_has_capacity;
			sds->this_idle_cpus = sgs.idle_cpus;
		} else if (update_sd_pick_busiest(sd, sds, sg, &sgs, this_cpu)) {
			sds->max_load = sgs.avg_load;
			sds->busiest = sg;
			sds->busiest_nr_running = sgs.sum_nr_running;
			sds->busiest_idle_cpus = sgs.idle_cpus;
			sds->busiest_group_capacity = sgs.group_capacity;
			sds->busiest_load_per_task = sgs.sum_weighted_load;
			sds->busiest_has_capacity = sgs.group_has_capacity;
			sds->busiest_group_weight = sgs.group_weight;
			sds->group_imb = sgs.group_imb;
		}

		update_sd_power_savings_stats(sg, sds, local_group, &sgs);
		sg = sg->next;
	} while (sg != sd->groups);
}

/**
 * check_asym_packing - Check to see if the group is packed into the
 *			sched doman.
 *
 * This is primarily intended to used at the sibling level.  Some
 * cores like POWER7 prefer to use lower numbered SMT threads.  In the
 * case of POWER7, it can move to lower SMT modes only when higher
 * threads are idle.  When in lower SMT modes, the threads will
 * perform better since they share less core resources.  Hence when we
 * have idle threads, we want them to be the higher ones.
 *
 * This packing function is run on idle threads.  It checks to see if
 * the busiest CPU in this domain (core in the P7 case) has a higher
 * CPU number than the packing function is being run on.  Here we are
 * assuming lower CPU number will be equivalent to lower a SMT thread
 * number.
 *
 * Returns 1 when packing is required and a task should be moved to
 * this CPU.  The amount of the imbalance is returned in *imbalance.
 *
 * @sd: The sched_domain whose packing is to be checked.
 * @sds: Statistics of the sched_domain which is to be packed
 * @this_cpu: The cpu at whose sched_domain we're performing load-balance.
 * @imbalance: returns amount of imbalanced due to packing.
 */
static int check_asym_packing(struct sched_domain *sd,
			      struct sd_lb_stats *sds,
			      int this_cpu, unsigned long *imbalance)
{
	int busiest_cpu;

	if (!(sd->flags & SD_ASYM_PACKING))
		return 0;

	if (!sds->busiest)
		return 0;

	busiest_cpu = group_first_cpu(sds->busiest);
	if (this_cpu > busiest_cpu)
		return 0;

	*imbalance = DIV_ROUND_CLOSEST(sds->max_load * sds->busiest->sgp->power,
				       SCHED_POWER_SCALE);
	return 1;
}

/**
 * fix_small_imbalance - Calculate the minor imbalance that exists
 *			amongst the groups of a sched_domain, during
 *			load balancing.
 * @sds: Statistics of the sched_domain whose imbalance is to be calculated.
 * @this_cpu: The cpu at whose sched_domain we're performing load-balance.
 * @imbalance: Variable to store the imbalance.
 */
static inline void fix_small_imbalance(struct sd_lb_stats *sds,
				int this_cpu, unsigned long *imbalance)
{
	unsigned long tmp, pwr_now = 0, pwr_move = 0;
	unsigned int imbn = 2;
	unsigned long scaled_busy_load_per_task;

	if (sds->this_nr_running) {
		sds->this_load_per_task /= sds->this_nr_running;
		if (sds->busiest_load_per_task >
				sds->this_load_per_task)
			imbn = 1;
	} else
		sds->this_load_per_task =
			cpu_avg_load_per_task(this_cpu);

	scaled_busy_load_per_task = sds->busiest_load_per_task
					 * SCHED_POWER_SCALE;
	scaled_busy_load_per_task /= sds->busiest->sgp->power;

	if (sds->max_load - sds->this_load + scaled_busy_load_per_task >=
			(scaled_busy_load_per_task * imbn)) {
		*imbalance = sds->busiest_load_per_task;
		return;
	}

	/*
	 * OK, we don't have enough imbalance to justify moving tasks,
	 * however we may be able to increase total CPU power used by
	 * moving them.
	 */

	pwr_now += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load);
	pwr_now += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load);
	pwr_now /= SCHED_POWER_SCALE;

	/* Amount of load we'd subtract */
	tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
		sds->busiest->sgp->power;
	if (sds->max_load > tmp)
		pwr_move += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load - tmp);

	/* Amount of load we'd add */
	if (sds->max_load * sds->busiest->sgp->power <
		sds->busiest_load_per_task * SCHED_POWER_SCALE)
		tmp = (sds->max_load * sds->busiest->sgp->power) /
			sds->this->sgp->power;
	else
		tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
			sds->this->sgp->power;
	pwr_move += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load + tmp);
	pwr_move /= SCHED_POWER_SCALE;

	/* Move if we gain throughput */
	if (pwr_move > pwr_now)
		*imbalance = sds->busiest_load_per_task;
}

/**
 * calculate_imbalance - Calculate the amount of imbalance present within the
 *			 groups of a given sched_domain during load balance.
 * @sds: statistics of the sched_domain whose imbalance is to be calculated.
 * @this_cpu: Cpu for which currently load balance is being performed.
 * @imbalance: The variable to store the imbalance.
 */
static inline void calculate_imbalance(struct sd_lb_stats *sds, int this_cpu,
		unsigned long *imbalance)
{
	unsigned long max_pull, load_above_capacity = ~0UL;

	sds->busiest_load_per_task /= sds->busiest_nr_running;
	if (sds->group_imb) {
		sds->busiest_load_per_task =
			min(sds->busiest_load_per_task, sds->avg_load);
	}

	/*
	 * In the presence of smp nice balancing, certain scenarios can have
	 * max load less than avg load(as we skip the groups at or below
	 * its cpu_power, while calculating max_load..)
	 */
	if (sds->max_load < sds->avg_load) {
		*imbalance = 0;
		return fix_small_imbalance(sds, this_cpu, imbalance);
	}

	if (!sds->group_imb) {
		/*
		 * Don't want to pull so many tasks that a group would go idle.
		 */
		load_above_capacity = (sds->busiest_nr_running -
						sds->busiest_group_capacity);

		load_above_capacity *= (SCHED_LOAD_SCALE * SCHED_POWER_SCALE);

		load_above_capacity /= sds->busiest->sgp->power;
	}

	/*
	 * We're trying to get all the cpus to the average_load, so we don't
	 * want to push ourselves above the average load, nor do we wish to
	 * reduce the max loaded cpu below the average load. At the same time,
	 * we also don't want to reduce the group load below the group capacity
	 * (so that we can implement power-savings policies etc). Thus we look
	 * for the minimum possible imbalance.
	 * Be careful of negative numbers as they'll appear as very large values
	 * with unsigned longs.
	 */
	max_pull = min(sds->max_load - sds->avg_load, load_above_capacity);

	/* How much load to actually move to equalise the imbalance */
	*imbalance = min(max_pull * sds->busiest->sgp->power,
		(sds->avg_load - sds->this_load) * sds->this->sgp->power)
			/ SCHED_POWER_SCALE;

	/*
	 * if *imbalance is less than the average load per runnable task
	 * there is no guarantee that any tasks will be moved so we'll have
	 * a think about bumping its value to force at least one task to be
	 * moved
	 */
	if (*imbalance < sds->busiest_load_per_task)
		return fix_small_imbalance(sds, this_cpu, imbalance);

}

/******* find_busiest_group() helpers end here *********************/

/**
 * find_busiest_group - Returns the busiest group within the sched_domain
 * if there is an imbalance. If there isn't an imbalance, and
 * the user has opted for power-savings, it returns a group whose
 * CPUs can be put to idle by rebalancing those tasks elsewhere, if
 * such a group exists.
 *
 * Also calculates the amount of weighted load which should be moved
 * to restore balance.
 *
 * @sd: The sched_domain whose busiest group is to be returned.
 * @this_cpu: The cpu for which load balancing is currently being performed.
 * @imbalance: Variable which stores amount of weighted load which should
 *		be moved to restore balance/put a group to idle.
 * @idle: The idle status of this_cpu.
 * @cpus: The set of CPUs under consideration for load-balancing.
 * @balance: Pointer to a variable indicating if this_cpu
 *	is the appropriate cpu to perform load balancing at this_level.
 *
 * Returns:	- the busiest group if imbalance exists.
 *		- If no imbalance and user has opted for power-savings balance,
 *		   return the least loaded group whose CPUs can be
 *		   put to idle by rebalancing its tasks onto our group.
 */
static struct sched_group *
find_busiest_group(struct sched_domain *sd, int this_cpu,
		   unsigned long *imbalance, enum cpu_idle_type idle,
		   const struct cpumask *cpus, int *balance)
{
	struct sd_lb_stats sds;

	memset(&sds, 0, sizeof(sds));

	/*
	 * Compute the various statistics relavent for load balancing at
	 * this level.
	 */
	update_sd_lb_stats(sd, this_cpu, idle, cpus, balance, &sds);

	/*
	 * this_cpu is not the appropriate cpu to perform load balancing at
	 * this level.
	 */
	if (!(*balance))
		goto ret;

	if ((idle == CPU_IDLE || idle == CPU_NEWLY_IDLE) &&
	    check_asym_packing(sd, &sds, this_cpu, imbalance))
		return sds.busiest;

	/* There is no busy sibling group to pull tasks from */
	if (!sds.busiest || sds.busiest_nr_running == 0)
		goto out_balanced;

	sds.avg_load = (SCHED_POWER_SCALE * sds.total_load) / sds.total_pwr;

	/*
	 * If the busiest group is imbalanced the below checks don't
	 * work because they assumes all things are equal, which typically
	 * isn't true due to cpus_allowed constraints and the like.
	 */
	if (sds.group_imb)
		goto force_balance;

	/* SD_BALANCE_NEWIDLE trumps SMP nice when underutilized */
	if (idle == CPU_NEWLY_IDLE && sds.this_has_capacity &&
			!sds.busiest_has_capacity)
		goto force_balance;

	/*
	 * If the local group is more busy than the selected busiest group
	 * don't try and pull any tasks.
	 */
	if (sds.this_load >= sds.max_load)
		goto out_balanced;

	/*
	 * Don't pull any tasks if this group is already above the domain
	 * average load.
	 */
	if (sds.this_load >= sds.avg_load)
		goto out_balanced;

	if (idle == CPU_IDLE) {
		/*
		 * This cpu is idle. If the busiest group load doesn't
		 * have more tasks than the number of available cpu's and
		 * there is no imbalance between this and busiest group
		 * wrt to idle cpu's, it is balanced.
		 */
		if ((sds.this_idle_cpus <= sds.busiest_idle_cpus + 1) &&
		    sds.busiest_nr_running <= sds.busiest_group_weight)
			goto out_balanced;
	} else {
		/*
		 * In the CPU_NEWLY_IDLE, CPU_NOT_IDLE cases, use
		 * imbalance_pct to be conservative.
		 */
		if (100 * sds.max_load <= sd->imbalance_pct * sds.this_load)
			goto out_balanced;
	}

force_balance:
	/* Looks like there is an imbalance. Compute it */
	calculate_imbalance(&sds, this_cpu, imbalance);
	return sds.busiest;

out_balanced:
	/*
	 * There is no obvious imbalance. But check if we can do some balancing
	 * to save power.
	 */
	if (check_power_save_busiest_group(&sds, this_cpu, imbalance))
		return sds.busiest;
ret:
	*imbalance = 0;
	return NULL;
}

/*
 * find_busiest_queue - find the busiest runqueue among the cpus in group.
 */
static struct rq *
find_busiest_queue(struct sched_domain *sd, struct sched_group *group,
		   enum cpu_idle_type idle, unsigned long imbalance,
		   const struct cpumask *cpus)
{
	struct rq *busiest = NULL, *rq;
	unsigned long max_load = 0;
	int i;

	for_each_cpu(i, sched_group_cpus(group)) {
		unsigned long power = power_of(i);
		unsigned long capacity = DIV_ROUND_CLOSEST(power,
							   SCHED_POWER_SCALE);
		unsigned long wl;

		if (!capacity)
			capacity = fix_small_capacity(sd, group);

		if (!cpumask_test_cpu(i, cpus))
			continue;

		rq = cpu_rq(i);
		wl = weighted_cpuload(i);

		/*
		 * When comparing with imbalance, use weighted_cpuload()
		 * which is not scaled with the cpu power.
		 */
		if (capacity && rq->nr_running == 1 && wl > imbalance)
			continue;

		/*
		 * For the load comparisons with the other cpu's, consider
		 * the weighted_cpuload() scaled with the cpu power, so that
		 * the load can be moved away from the cpu that is potentially
		 * running at a lower capacity.
		 */
		wl = (wl * SCHED_POWER_SCALE) / power;

		if (wl > max_load) {
			max_load = wl;
			busiest = rq;
		}
	}

	return busiest;
}

/*
 * Max backoff if we encounter pinned tasks. Pretty arbitrary value, but
 * so long as it is large enough.
 */
#define MAX_PINNED_INTERVAL	512

/* Working cpumask for load_balance and load_balance_newidle. */
DEFINE_PER_CPU(cpumask_var_t, load_balance_tmpmask);

static int need_active_balance(struct sched_domain *sd, int idle,
			       int busiest_cpu, int this_cpu)
{
	if (idle == CPU_NEWLY_IDLE) {

		/*
		 * ASYM_PACKING needs to force migrate tasks from busy but
		 * higher numbered CPUs in order to pack all tasks in the
		 * lowest numbered CPUs.
		 */
		if ((sd->flags & SD_ASYM_PACKING) && busiest_cpu > this_cpu)
			return 1;

		/*
		 * The only task running in a non-idle cpu can be moved to this
		 * cpu in an attempt to completely freeup the other CPU
		 * package.
		 *
		 * The package power saving logic comes from
		 * find_busiest_group(). If there are no imbalance, then
		 * f_b_g() will return NULL. However when sched_mc={1,2} then
		 * f_b_g() will select a group from which a running task may be
		 * pulled to this cpu in order to make the other package idle.
		 * If there is no opportunity to make a package idle and if
		 * there are no imbalance, then f_b_g() will return NULL and no
		 * action will be taken in load_balance_newidle().
		 *
		 * Under normal task pull operation due to imbalance, there
		 * will be more than one task in the source run queue and
		 * move_tasks() will succeed.  ld_moved will be true and this
		 * active balance code will not be triggered.
		 */
		if (sched_mc_power_savings < POWERSAVINGS_BALANCE_WAKEUP)
			return 0;
	}

	return unlikely(sd->nr_balance_failed > sd->cache_nice_tries+2);
}

static int active_load_balance_cpu_stop(void *data);

/*
 * Check this_cpu to ensure it is balanced within domain. Attempt to move
 * tasks if there is an imbalance.
 */
static int load_balance(int this_cpu, struct rq *this_rq,
			struct sched_domain *sd, enum cpu_idle_type idle,
			int *balance)
{
	int ld_moved, active_balance = 0;
	struct sched_group *group;
	unsigned long imbalance;
	struct rq *busiest = NULL;
	unsigned long flags;
	struct cpumask *cpus = __get_cpu_var(load_balance_tmpmask);

	struct lb_env env = {
		.sd		= sd,
		.dst_cpu	= this_cpu,
		.dst_rq		= this_rq,
		.idle		= idle,
		.loop_break	= sched_nr_migrate_break,
	};

	cpumask_copy(cpus, cpu_active_mask);

	schedstat_inc(sd, lb_count[idle]);

redo:
	group = find_busiest_group(sd, this_cpu, &imbalance, idle,
				   cpus, balance);

	if (*balance == 0)
		goto out_balanced;

	if (!group) {
		schedstat_inc(sd, lb_nobusyg[idle]);
		goto out_balanced;
	}

	busiest = find_busiest_queue(sd, group, idle, imbalance, cpus);
	if (!busiest) {
		schedstat_inc(sd, lb_nobusyq[idle]);
		goto out_balanced;
	}

	BUG_ON(busiest == this_rq);

	schedstat_add(sd, lb_imbalance[idle], imbalance);

	ld_moved = 0;
	if (busiest->nr_running > 1) {
		/*
		 * Attempt to move tasks. If find_busiest_group has found
		 * an imbalance but busiest->nr_running <= 1, the group is
		 * still unbalanced. ld_moved simply stays zero, so it is
		 * correctly treated as an imbalance.
		 */
		env.flags |= LBF_ALL_PINNED;
		env.load_move	= imbalance;
		env.src_cpu	= busiest->cpu;
		env.src_rq	= busiest;
		env.loop_max	= min_t(unsigned long, sysctl_sched_nr_migrate, busiest->nr_running);

more_balance:
		local_irq_save(flags);
		double_rq_lock(this_rq, busiest);
		if (!env.loop)
			update_h_load(env.src_cpu);
		ld_moved += move_tasks(&env);
		double_rq_unlock(this_rq, busiest);
		local_irq_restore(flags);

		if (env.flags & LBF_NEED_BREAK) {
			env.flags &= ~LBF_NEED_BREAK;
			goto more_balance;
		}

		/*
		 * some other cpu did the load balance for us.
		 */
		if (ld_moved && this_cpu != smp_processor_id())
			resched_cpu(this_cpu);

		/* All tasks on this runqueue were pinned by CPU affinity */
		if (unlikely(env.flags & LBF_ALL_PINNED)) {
			cpumask_clear_cpu(cpu_of(busiest), cpus);
			if (!cpumask_empty(cpus))
				goto redo;
			goto out_balanced;
		}
	}

	if (!ld_moved) {
		schedstat_inc(sd, lb_failed[idle]);
		/*
		 * Increment the failure counter only on periodic balance.
		 * We do not want newidle balance, which can be very
		 * frequent, pollute the failure counter causing
		 * excessive cache_hot migrations and active balances.
		 */
		if (idle != CPU_NEWLY_IDLE)
			sd->nr_balance_failed++;

		if (need_active_balance(sd, idle, cpu_of(busiest), this_cpu)) {
			raw_spin_lock_irqsave(&busiest->lock, flags);

			/* don't kick the active_load_balance_cpu_stop,
			 * if the curr task on busiest cpu can't be
			 * moved to this_cpu
			 */
			if (!cpumask_test_cpu(this_cpu,
					tsk_cpus_allowed(busiest->curr))) {
				raw_spin_unlock_irqrestore(&busiest->lock,
							    flags);
				env.flags |= LBF_ALL_PINNED;
				goto out_one_pinned;
			}

			/*
			 * ->active_balance synchronizes accesses to
			 * ->active_balance_work.  Once set, it's cleared
			 * only after active load balance is finished.
			 */
			if (!busiest->active_balance) {
				busiest->active_balance = 1;
				busiest->push_cpu = this_cpu;
				active_balance = 1;
			}
			raw_spin_unlock_irqrestore(&busiest->lock, flags);

			if (active_balance)
				stop_one_cpu_nowait(cpu_of(busiest),
					active_load_balance_cpu_stop, busiest,
					&busiest->active_balance_work);

			/*
			 * We've kicked active balancing, reset the failure
			 * counter.
			 */
			sd->nr_balance_failed = sd->cache_nice_tries+1;
		}
	} else {
		sd->nr_balance_failed = 0;
		if (per_cpu(dbs_boost_needed, this_cpu)) {
			per_cpu(dbs_boost_needed, this_cpu) = false;
			atomic_notifier_call_chain(&migration_notifier_head,
						   this_cpu,
						   (void *)cpu_of(busiest));
		}
	}
	if (likely(!active_balance)) {
		/* We were unbalanced, so reset the balancing interval */
		sd->balance_interval = sd->min_interval;
	} else {
		/*
		 * If we've begun active balancing, start to back off. This
		 * case may not be covered by the all_pinned logic if there
		 * is only 1 task on the busy runqueue (because we don't call
		 * move_tasks).
		 */
		if (sd->balance_interval < sd->max_interval)
			sd->balance_interval *= 2;
	}

	goto out;

out_balanced:
	schedstat_inc(sd, lb_balanced[idle]);

	sd->nr_balance_failed = 0;

out_one_pinned:
	/* tune up the balancing interval */
	if (((env.flags & LBF_ALL_PINNED) &&
			sd->balance_interval < MAX_PINNED_INTERVAL) ||
			(sd->balance_interval < sd->max_interval))
		sd->balance_interval *= 2;

	ld_moved = 0;
out:
	trace_sched_load_balance(this_cpu, idle, *balance,
				 group ? group->cpumask[0] : 0,
				 busiest ? busiest->nr_running : 0, imbalance,
				 env.flags, ld_moved, sd->balance_interval);
	return ld_moved;
}

/*
 * idle_balance is called by schedule() if this_cpu is about to become
 * idle. Attempts to pull tasks from other CPUs.
 */
void idle_balance(int this_cpu, struct rq *this_rq)
{
	struct sched_domain *sd;
	int pulled_task = 0;
	unsigned long next_balance = jiffies + HZ;

	this_rq->idle_stamp = this_rq->clock;

	if (this_rq->avg_idle < sysctl_sched_migration_cost)
		return;

	/*
	 * Drop the rq->lock, but keep IRQ/preempt disabled.
	 */
	raw_spin_unlock(&this_rq->lock);

	update_shares(this_cpu);
	rcu_read_lock();
	for_each_domain(this_cpu, sd) {
		unsigned long interval;
		int balance = 1;

		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		if (sd->flags & SD_BALANCE_NEWIDLE) {
			/* If we've pulled tasks over stop searching: */
			pulled_task = load_balance(this_cpu, this_rq,
						   sd, CPU_NEWLY_IDLE, &balance);
		}

		interval = msecs_to_jiffies(sd->balance_interval);
		if (time_after(next_balance, sd->last_balance + interval))
			next_balance = sd->last_balance + interval;
		if (pulled_task) {
			this_rq->idle_stamp = 0;
			break;
		}
	}
	rcu_read_unlock();

	raw_spin_lock(&this_rq->lock);

	if (!pulled_task || time_after(jiffies, this_rq->next_balance)) {
		/*
		 * We are going idle. next_balance may be set based on
		 * a busy processor. So reset next_balance.
		 */
		this_rq->next_balance = next_balance;
	}
}

/*
 * active_load_balance_cpu_stop is run by cpu stopper. It pushes
 * running tasks off the busiest CPU onto idle CPUs. It requires at
 * least 1 task to be running on each physical CPU where possible, and
 * avoids physical / logical imbalances.
 */
static int active_load_balance_cpu_stop(void *data)
{
	struct rq *busiest_rq = data;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd;

	raw_spin_lock_irq(&busiest_rq->lock);

	/* make sure the requested cpu hasn't gone down in the meantime */
	if (unlikely(busiest_cpu != smp_processor_id() ||
		     !busiest_rq->active_balance))
		goto out_unlock;

	/* Is there any task to move? */
	if (busiest_rq->nr_running <= 1)
		goto out_unlock;

	/*
	 * This condition is "impossible", if it occurs
	 * we need to fix it. Originally reported by
	 * Bjorn Helgaas on a 128-cpu setup.
	 */
	BUG_ON(busiest_rq == target_rq);

	/* move a task from busiest_rq to target_rq */
	double_lock_balance(busiest_rq, target_rq);

	/* Search for an sd spanning us and the target CPU. */
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if ((sd->flags & SD_LOAD_BALANCE) &&
		    cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
				break;
	}

	if (likely(sd)) {
		struct lb_env env = {
			.sd		= sd,
			.dst_cpu	= target_cpu,
			.dst_rq		= target_rq,
			.src_cpu	= busiest_rq->cpu,
			.src_rq		= busiest_rq,
			.idle		= CPU_IDLE,
		};

		schedstat_inc(sd, alb_count);

		if (move_one_task(&env))
			schedstat_inc(sd, alb_pushed);
		else
			schedstat_inc(sd, alb_failed);
	}
	rcu_read_unlock();
	double_unlock_balance(busiest_rq, target_rq);
out_unlock:
	busiest_rq->active_balance = 0;
	raw_spin_unlock_irq(&busiest_rq->lock);
	if (per_cpu(dbs_boost_needed, target_cpu)) {
		per_cpu(dbs_boost_needed, target_cpu) = false;
		atomic_notifier_call_chain(&migration_notifier_head,
					   target_cpu,
					   (void *)cpu_of(busiest_rq));
	}
	return 0;
}

#ifdef CONFIG_NO_HZ
/*
 * idle load balancing details
 * - When one of the busy CPUs notice that there may be an idle rebalancing
 *   needed, they will kick the idle load balancer, which then does idle
 *   load balancing for all the idle CPUs.
 */
static struct {
	cpumask_var_t idle_cpus_mask;
	atomic_t nr_cpus;
	unsigned long next_balance;     /* in jiffy units */
} nohz ____cacheline_aligned;

#if defined(CONFIG_SCHED_MC) || defined(CONFIG_SCHED_SMT)
/**
 * lowest_flag_domain - Return lowest sched_domain containing flag.
 * @cpu:	The cpu whose lowest level of sched domain is to
 *		be returned.
 * @flag:	The flag to check for the lowest sched_domain
 *		for the given cpu.
 *
 * Returns the lowest sched_domain of a cpu which contains the given flag.
 */
static inline struct sched_domain *lowest_flag_domain(int cpu, int flag)
{
	struct sched_domain *sd;

	for_each_domain(cpu, sd)
		if (sd->flags & flag)
			break;

	return sd;
}

/**
 * for_each_flag_domain - Iterates over sched_domains containing the flag.
 * @cpu:	The cpu whose domains we're iterating over.
 * @sd:		variable holding the value of the power_savings_sd
 *		for cpu.
 * @flag:	The flag to filter the sched_domains to be iterated.
 *
 * Iterates over all the scheduler domains for a given cpu that has the 'flag'
 * set, starting from the lowest sched_domain to the highest.
 */
#define for_each_flag_domain(cpu, sd, flag) \
	for (sd = lowest_flag_domain(cpu, flag); \
		(sd && (sd->flags & flag)); sd = sd->parent)

/**
 * find_new_ilb - Finds the optimum idle load balancer for nomination.
 * @cpu:	The cpu which is nominating a new idle_load_balancer.
 *
 * Returns:	Returns the id of the idle load balancer if it exists,
 *		Else, returns >= nr_cpu_ids.
 *
 * This algorithm picks the idle load balancer such that it belongs to a
 * semi-idle powersavings sched_domain. The idea is to try and avoid
 * completely idle packages/cores just for the purpose of idle load balancing
 * when there are other idle cpu's which are better suited for that job.
 */
static int find_new_ilb(int cpu)
{
	int ilb = cpumask_first(nohz.idle_cpus_mask);
	struct sched_group *ilbg;
	struct sched_domain *sd;

	/*
	 * Have idle load balancer selection from semi-idle packages only
	 * when power-aware load balancing is enabled
	 */
	if (!(sched_smt_power_savings || sched_mc_power_savings))
		goto out_done;

	/*
	 * Optimize for the case when we have no idle CPUs or only one
	 * idle CPU. Don't walk the sched_domain hierarchy in such cases
	 */
	if (cpumask_weight(nohz.idle_cpus_mask) < 2)
		goto out_done;

	rcu_read_lock();
	for_each_flag_domain(cpu, sd, SD_POWERSAVINGS_BALANCE) {
		ilbg = sd->groups;

		do {
			if (ilbg->group_weight !=
				atomic_read(&ilbg->sgp->nr_busy_cpus)) {
				ilb = cpumask_first_and(nohz.idle_cpus_mask,
							sched_group_cpus(ilbg));
				goto unlock;
			}

			ilbg = ilbg->next;

		} while (ilbg != sd->groups);
	}
unlock:
	rcu_read_unlock();

out_done:
	if (ilb < nr_cpu_ids && idle_cpu(ilb))
		return ilb;

	return nr_cpu_ids;
}
#else /*  (CONFIG_SCHED_MC || CONFIG_SCHED_SMT) */
static inline int find_new_ilb(int call_cpu)
{
	return nr_cpu_ids;
}
#endif

/*
 * Kick a CPU to do the nohz balancing, if it is time for it. We pick the
 * nohz_load_balancer CPU (if there is one) otherwise fallback to any idle
 * CPU (if there is one).
 */
static void nohz_balancer_kick(int cpu)
{
	int ilb_cpu;

	nohz.next_balance++;

	ilb_cpu = find_new_ilb(cpu);

	if (ilb_cpu >= nr_cpu_ids)
		return;

	if (test_and_set_bit(NOHZ_BALANCE_KICK, nohz_flags(ilb_cpu)))
		return;
	/*
	 * Use smp_send_reschedule() instead of resched_cpu().
	 * This way we generate a sched IPI on the target cpu which
	 * is idle. And the softirq performing nohz idle load balance
	 * will be run before returning from the IPI.
	 */
	smp_send_reschedule(ilb_cpu);
	return;
}

static inline void clear_nohz_tick_stopped(int cpu)
{
	if (unlikely(test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))) {
		cpumask_clear_cpu(cpu, nohz.idle_cpus_mask);
		atomic_dec(&nohz.nr_cpus);
		clear_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
	}
}

static inline void set_cpu_sd_state_busy(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	if (!test_bit(NOHZ_IDLE, nohz_flags(cpu)))
		return;
	clear_bit(NOHZ_IDLE, nohz_flags(cpu));

	rcu_read_lock();
	for_each_domain(cpu, sd)
		atomic_inc(&sd->groups->sgp->nr_busy_cpus);
	rcu_read_unlock();
}

void set_cpu_sd_state_idle(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	if (test_bit(NOHZ_IDLE, nohz_flags(cpu)))
		return;
	set_bit(NOHZ_IDLE, nohz_flags(cpu));

	rcu_read_lock();
	for_each_domain(cpu, sd)
		atomic_dec(&sd->groups->sgp->nr_busy_cpus);
	rcu_read_unlock();
}

/*
 * This routine will record that this cpu is going idle with tick stopped.
 * This info will be used in performing idle load balancing in the future.
 */
void select_nohz_load_balancer(int stop_tick)
{
	int cpu = smp_processor_id();

	/*
	 * If this cpu is going down, then nothing needs to be done.
	 */
	if (!cpu_active(cpu))
		return;

	if (stop_tick) {
		if (test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))
			return;

		cpumask_set_cpu(cpu, nohz.idle_cpus_mask);
		atomic_inc(&nohz.nr_cpus);
		set_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
	}
	return;
}

static int __cpuinit sched_ilb_notifier(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DYING:
		clear_nohz_tick_stopped(smp_processor_id());
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}
#endif

static DEFINE_SPINLOCK(balancing);

/*
 * Scale the max load_balance interval with the number of CPUs in the system.
 * This trades load-balance latency on larger machines for less cross talk.
 */
void update_max_interval(void)
{
	max_load_balance_interval = HZ*num_online_cpus()/10;
}

/*
 * It checks each scheduling domain to see if it is due to be balanced,
 * and initiates a balancing operation if so.
 *
 * Balancing parameters are set up in arch_init_sched_domains.
 */
static void rebalance_domains(int cpu, enum cpu_idle_type idle)
{
	int balance = 1;
	struct rq *rq = cpu_rq(cpu);
	unsigned long interval;
	struct sched_domain *sd;
	/* Earliest time when we have to do rebalance again */
	unsigned long next_balance = jiffies + 60*HZ;
	int update_next_balance = 0;
	int need_serialize;

	update_shares(cpu);

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		interval = sd->balance_interval;
		if (idle != CPU_IDLE)
			interval *= sd->busy_factor;

		/* scale ms to jiffies */
		interval = msecs_to_jiffies(interval);
		interval = clamp(interval, 1UL, max_load_balance_interval);

		need_serialize = sd->flags & SD_SERIALIZE;

		if (need_serialize) {
			if (!spin_trylock(&balancing))
				goto out;
		}

		if (time_after_eq(jiffies, sd->last_balance + interval)) {
			if (load_balance(cpu, rq, sd, idle, &balance)) {
				/*
				 * We've pulled tasks over so either we're no
				 * longer idle.
				 */
				idle = CPU_NOT_IDLE;
			}
			sd->last_balance = jiffies;
		}
		if (need_serialize)
			spin_unlock(&balancing);
out:
		if (time_after(next_balance, sd->last_balance + interval)) {
			next_balance = sd->last_balance + interval;
			update_next_balance = 1;
		}

		/*
		 * Stop the load balance at this level. There is another
		 * CPU in our sched group which is doing load balancing more
		 * actively.
		 */
		if (!balance)
			break;
	}
	rcu_read_unlock();

	/*
	 * next_balance will be updated only when there is a need.
	 * When the cpu is attached to null domain for ex, it will not be
	 * updated.
	 */
	if (likely(update_next_balance))
		rq->next_balance = next_balance;
}

#ifdef CONFIG_NO_HZ
/*
 * In CONFIG_NO_HZ case, the idle balance kickee will do the
 * rebalancing for all the cpus for whom scheduler ticks are stopped.
 */
static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle)
{
	struct rq *this_rq = cpu_rq(this_cpu);
	struct rq *rq;
	int balance_cpu;

	if (idle != CPU_IDLE ||
	    !test_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu)))
		goto end;

	for_each_cpu(balance_cpu, nohz.idle_cpus_mask) {
		if (balance_cpu == this_cpu || !idle_cpu(balance_cpu))
			continue;

		/*
		 * If this cpu gets work to do, stop the load balancing
		 * work being done for other cpus. Next load
		 * balancing owner will pick it up.
		 */
		if (need_resched())
			break;

		raw_spin_lock_irq(&this_rq->lock);
		update_rq_clock(this_rq);
		update_cpu_load(this_rq);
		raw_spin_unlock_irq(&this_rq->lock);

		rebalance_domains(balance_cpu, CPU_IDLE);

		rq = cpu_rq(balance_cpu);
		if (time_after(this_rq->next_balance, rq->next_balance))
			this_rq->next_balance = rq->next_balance;
	}
	nohz.next_balance = this_rq->next_balance;
end:
	clear_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu));
}

/*
 * Current heuristic for kicking the idle load balancer in the presence
 * of an idle cpu is the system.
 *   - This rq has more than one task.
 *   - At any scheduler domain level, this cpu's scheduler group has multiple
 *     busy cpu's exceeding the group's power.
 *   - For SD_ASYM_PACKING, if the lower numbered cpu's in the scheduler
 *     domain span are idle.
 */
static inline int nohz_kick_needed(struct rq *rq, int cpu)
{
	unsigned long now = jiffies;
	struct sched_domain *sd;

	if (unlikely(idle_cpu(cpu)))
		return 0;

       /*
	* We may be recently in ticked or tickless idle mode. At the first
	* busy tick after returning from idle, we will update the busy stats.
	*/
	set_cpu_sd_state_busy();
	clear_nohz_tick_stopped(cpu);

	/*
	 * None are in tickless mode and hence no need for NOHZ idle load
	 * balancing.
	 */
	if (likely(!atomic_read(&nohz.nr_cpus)))
		return 0;

	if (time_before(now, nohz.next_balance))
		return 0;

	if (rq->nr_running >= 2)
		goto need_kick;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		struct sched_group *sg = sd->groups;
		struct sched_group_power *sgp = sg->sgp;
		int nr_busy = atomic_read(&sgp->nr_busy_cpus);

		if (sd->flags & SD_SHARE_PKG_RESOURCES && nr_busy > 1)
			goto need_kick_unlock;

		if (sd->flags & SD_ASYM_PACKING && nr_busy != sg->group_weight
		    && (cpumask_first_and(nohz.idle_cpus_mask,
					  sched_domain_span(sd)) < cpu))
			goto need_kick_unlock;

		if (!(sd->flags & (SD_SHARE_PKG_RESOURCES | SD_ASYM_PACKING)))
			break;
	}
	rcu_read_unlock();
	return 0;

need_kick_unlock:
	rcu_read_unlock();
need_kick:
	return 1;
}
#else
static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle) { }
#endif

/*
 * run_rebalance_domains is triggered when needed from the scheduler tick.
 * Also triggered for nohz idle balancing (with nohz_balancing_kick set).
 */
static void run_rebalance_domains(struct softirq_action *h)
{
	int this_cpu = smp_processor_id();
	struct rq *this_rq = cpu_rq(this_cpu);
	enum cpu_idle_type idle = this_rq->idle_balance ?
						CPU_IDLE : CPU_NOT_IDLE;

	rebalance_domains(this_cpu, idle);

	/*
	 * If this cpu has a pending nohz_balance_kick, then do the
	 * balancing on behalf of the other idle cpus whose ticks are
	 * stopped.
	 */
	nohz_idle_balance(this_cpu, idle);
}

static inline int on_null_domain(int cpu)
{
	return !rcu_dereference_sched(cpu_rq(cpu)->sd);
}

/*
 * Trigger the SCHED_SOFTIRQ if it is time to do periodic load balancing.
 */
void trigger_load_balance(struct rq *rq, int cpu)
{
	/* Don't need to rebalance while attached to NULL domain */
	if (time_after_eq(jiffies, rq->next_balance) &&
	    likely(!on_null_domain(cpu)))
		raise_softirq(SCHED_SOFTIRQ);
#ifdef CONFIG_NO_HZ
	if (nohz_kick_needed(rq, cpu) && likely(!on_null_domain(cpu)))
		nohz_balancer_kick(cpu);
#endif
}

static void rq_online_fair(struct rq *rq)
{
	update_sysctl();
}

static void rq_offline_fair(struct rq *rq)
{
	update_sysctl();

	/* Ensure any throttled groups are reachable by pick_next_task */
	unthrottle_offline_cfs_rqs(rq);
}

#endif /* CONFIG_SMP */

/*
 * scheduler tick hitting a task of our scheduling class:
 */
static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &curr->se;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		entity_tick(cfs_rq, se, queued);
	}
}

/*
 * called on fork with the child task as argument from the parent's context
 *  - child not yet on the tasklist
 *  - preemption disabled
 */
static void task_fork_fair(struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se, *curr;
	int this_cpu = smp_processor_id();
	struct rq *rq = this_rq();
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);

	update_rq_clock(rq);

	cfs_rq = task_cfs_rq(current);
	curr = cfs_rq->curr;

	if (unlikely(task_cpu(p) != this_cpu)) {
		rcu_read_lock();
		__set_task_cpu(p, this_cpu);
		rcu_read_unlock();
	}

	update_curr(cfs_rq);

	if (curr)
		se->vruntime = curr->vruntime;
	place_entity(cfs_rq, se, 1);

	if (sysctl_sched_child_runs_first && curr && entity_before(curr, se)) {
		/*
		 * Upon rescheduling, sched_class::put_prev_task() will place
		 * 'current' within the tree based on its new key value.
		 */
		swap(curr->vruntime, se->vruntime);
		resched_task(rq->curr);
	}

	se->vruntime -= cfs_rq->min_vruntime;

	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

/*
 * Priority of the task has changed. Check to see if we preempt
 * the current task.
 */
static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio)
{
	if (!p->se.on_rq)
		return;

	/*
	 * Reschedule if we are currently running on this runqueue and
	 * our priority decreased, or if we are not currently running on
	 * this runqueue and our priority is higher than the current's
	 */
	if (rq->curr == p) {
		if (p->prio > oldprio)
			resched_task(rq->curr);
	} else
		check_preempt_curr(rq, p, 0);
}

static void switched_from_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	/*
	 * Ensure the task's vruntime is normalized, so that when its
	 * switched back to the fair class the enqueue_entity(.flags=0) will
	 * do the right thing.
	 *
	 * If it was on_rq, then the dequeue_entity(.flags=0) will already
	 * have normalized the vruntime, if it was !on_rq, then only when
	 * the task is sleeping will it still have non-normalized vruntime.
	 */
	if (!se->on_rq && p->state != TASK_RUNNING) {
		/*
		 * Fix up our vruntime so that the current sleep doesn't
		 * cause 'unlimited' sleep bonus.
		 */
		place_entity(cfs_rq, se, 0);
		se->vruntime -= cfs_rq->min_vruntime;
	}
}

/*
 * We switched to the sched_fair class.
 */
static void switched_to_fair(struct rq *rq, struct task_struct *p)
{
	if (!p->se.on_rq)
		return;

	/*
	 * We were most likely switched from sched_rt, so
	 * kick off the schedule if running, otherwise just see
	 * if we can still preempt the current task.
	 */
	if (rq->curr == p)
		resched_task(rq->curr);
	else
		check_preempt_curr(rq, p, 0);
}

/* Account for a task changing its policy or group.
 *
 * This routine is mostly called to set cfs_rq->curr field when a task
 * migrates between groups/classes.
 */
static void set_curr_task_fair(struct rq *rq)
{
	struct sched_entity *se = &rq->curr->se;

	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);

		set_next_entity(cfs_rq, se);
		/* ensure bandwidth has been allocated on our new cfs_rq */
		account_cfs_rq_runtime(cfs_rq, 0);
	}
}

void init_cfs_rq(struct cfs_rq *cfs_rq)
{
	cfs_rq->tasks_timeline = RB_ROOT;
	cfs_rq->min_vruntime = (u64)(-(1LL << 20));
#ifndef CONFIG_64BIT
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void task_move_group_fair(struct task_struct *p, int on_rq)
{
	/*
	 * If the task was not on the rq at the time of this cgroup movement
	 * it must have been asleep, sleeping tasks keep their ->vruntime
	 * absolute on their old rq until wakeup (needed for the fair sleeper
	 * bonus in place_entity()).
	 *
	 * If it was on the rq, we've just 'preempted' it, which does convert
	 * ->vruntime to a relative base.
	 *
	 * Make sure both cases convert their relative position when migrating
	 * to another cgroup's rq. This does somewhat interfere with the
	 * fair sleeper stuff for the first placement, but who cares.
	 */
	/*
	 * When !on_rq, vruntime of the task has usually NOT been normalized.
	 * But there are some cases where it has already been normalized:
	 *
	 * - Moving a forked child which is waiting for being woken up by
	 *   wake_up_new_task().
	 * - Moving a task which has been woken up by try_to_wake_up() and
	 *   waiting for actually being woken up by sched_ttwu_pending().
	 *
	 * To prevent boost or penalty in the new cfs_rq caused by delta
	 * min_vruntime between the two cfs_rqs, we skip vruntime adjustment.
	 */
	if (!on_rq && (!p->se.sum_exec_runtime || p->state == TASK_WAKING))
		on_rq = 1;

	if (!on_rq)
		p->se.vruntime -= cfs_rq_of(&p->se)->min_vruntime;
	set_task_rq(p, task_cpu(p));
	if (!on_rq)
		p->se.vruntime += cfs_rq_of(&p->se)->min_vruntime;
}

void free_fair_sched_group(struct task_group *tg)
{
	int i;

	destroy_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		if (tg->cfs_rq)
			kfree(tg->cfs_rq[i]);
		if (tg->se)
			kfree(tg->se[i]);
	}

	kfree(tg->cfs_rq);
	kfree(tg->se);
}

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se;
	int i;

	tg->cfs_rq = kzalloc(sizeof(cfs_rq) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->cfs_rq)
		goto err;
	tg->se = kzalloc(sizeof(se) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->se)
		goto err;

	tg->shares = NICE_0_LOAD;

	init_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		cfs_rq = kzalloc_node(sizeof(struct cfs_rq),
				      GFP_KERNEL, cpu_to_node(i));
		if (!cfs_rq)
			goto err;

		se = kzalloc_node(sizeof(struct sched_entity),
				  GFP_KERNEL, cpu_to_node(i));
		if (!se)
			goto err_free_rq;

		init_cfs_rq(cfs_rq);
		init_tg_cfs_entry(tg, cfs_rq, se, i, parent->se[i]);
	}

	return 1;

err_free_rq:
	kfree(cfs_rq);
err:
	return 0;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	/*
	* Only empty task groups can be destroyed; so we can speculatively
	* check on_list without danger of it being re-added.
	*/
	if (!tg->cfs_rq[cpu]->on_list)
		return;

	raw_spin_lock_irqsave(&rq->lock, flags);
	list_del_leaf_cfs_rq(tg->cfs_rq[cpu]);
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

void init_tg_cfs_entry(struct task_group *tg, struct cfs_rq *cfs_rq,
			struct sched_entity *se, int cpu,
			struct sched_entity *parent)
{
	struct rq *rq = cpu_rq(cpu);

	cfs_rq->tg = tg;
	cfs_rq->rq = rq;
#ifdef CONFIG_SMP
	/* allow initial update_cfs_load() to truncate */
	cfs_rq->load_stamp = 1;
#endif
	init_cfs_rq_runtime(cfs_rq);

	tg->cfs_rq[cpu] = cfs_rq;
	tg->se[cpu] = se;

	/* se could be NULL for root_task_group */
	if (!se)
		return;

	if (!parent)
		se->cfs_rq = &rq->cfs;
	else
		se->cfs_rq = parent->my_q;

	se->my_q = cfs_rq;
	update_load_set(&se->load, 0);
	se->parent = parent;
}

static DEFINE_MUTEX(shares_mutex);

int sched_group_set_shares(struct task_group *tg, unsigned long shares)
{
	int i;
	unsigned long flags;

	/*
	 * We can't change the weight of the root cgroup.
	 */
	if (!tg->se[0])
		return -EINVAL;

	shares = clamp(shares, scale_load(MIN_SHARES), scale_load(MAX_SHARES));

	mutex_lock(&shares_mutex);
	if (tg->shares == shares)
		goto done;

	tg->shares = shares;
	for_each_possible_cpu(i) {
		struct rq *rq = cpu_rq(i);
		struct sched_entity *se;

		se = tg->se[i];
		/* Propagate contribution to hierarchy */
		raw_spin_lock_irqsave(&rq->lock, flags);
		for_each_sched_entity(se)
			update_cfs_shares(group_cfs_rq(se));
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

done:
	mutex_unlock(&shares_mutex);
	return 0;
}
#else /* CONFIG_FAIR_GROUP_SCHED */

void free_fair_sched_group(struct task_group *tg) { }

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	return 1;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu) { }

#endif /* CONFIG_FAIR_GROUP_SCHED */


static unsigned int get_rr_interval_fair(struct rq *rq, struct task_struct *task)
{
	struct sched_entity *se = &task->se;
	unsigned int rr_interval = 0;

	/*
	 * Time slice is 0 for SCHED_OTHER tasks that are on an otherwise
	 * idle runqueue:
	 */
	if (rq->cfs.load.weight)
		rr_interval = NS_TO_JIFFIES(sched_slice(&rq->cfs, se));

	return rr_interval;
}

/*
 * All the scheduling class methods:
 */
const struct sched_class fair_sched_class = {
	.next			= &idle_sched_class,
	.enqueue_task		= enqueue_task_fair,
	.dequeue_task		= dequeue_task_fair,
	.yield_task		= yield_task_fair,
	.yield_to_task		= yield_to_task_fair,

	.check_preempt_curr	= check_preempt_wakeup,

	.pick_next_task		= pick_next_task_fair,
	.put_prev_task		= put_prev_task_fair,

#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_fair,

	.rq_online		= rq_online_fair,
	.rq_offline		= rq_offline_fair,

	.task_waking		= task_waking_fair,
#endif

	.set_curr_task          = set_curr_task_fair,
	.task_tick		= task_tick_fair,
	.task_fork		= task_fork_fair,

	.prio_changed		= prio_changed_fair,
	.switched_from		= switched_from_fair,
	.switched_to		= switched_to_fair,

	.get_rr_interval	= get_rr_interval_fair,

#ifdef CONFIG_FAIR_GROUP_SCHED
	.task_move_group	= task_move_group_fair,
#endif
};

#ifdef CONFIG_SCHED_DEBUG
void print_cfs_stats(struct seq_file *m, int cpu)
{
	struct cfs_rq *cfs_rq;

	rcu_read_lock();
	for_each_leaf_cfs_rq(cpu_rq(cpu), cfs_rq)
		print_cfs_rq(m, cpu, cfs_rq);
	rcu_read_unlock();
}
#endif

__init void init_sched_fair_class(void)
{
#ifdef CONFIG_SMP
	open_softirq(SCHED_SOFTIRQ, run_rebalance_domains);

#ifdef CONFIG_NO_HZ
	nohz.next_balance = jiffies;
	zalloc_cpumask_var(&nohz.idle_cpus_mask, GFP_NOWAIT);
	cpu_notifier(sched_ilb_notifier, 0);
#endif
#endif /* SMP */

}

/*
 * mm/page-writeback.c
 *
 * Copyright (C) 2002, Linus Torvalds.
 * Copyright (C) 2007 Red Hat, Inc., Peter Zijlstra <pzijlstr@redhat.com>
 *
 * Contains functions related to writing back dirty pages at the
 * address_space level.
 *
 * 10Apr2002	Andrew Morton
 *		Initial version
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/writeback.h>
#include <linux/init.h>
#include <linux/backing-dev.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/blkdev.h>
#include <linux/mpage.h>
#include <linux/rmap.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/smp.h>
#include <linux/sysctl.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/buffer_head.h> 
#include <linux/pagevec.h>
#include <linux/mm_inline.h>
#include <trace/events/writeback.h>

#include "internal.h"

#define MAX_PAUSE		max(HZ/5, 1)

#define DIRTY_POLL_THRESH	(128 >> (PAGE_SHIFT - 10))

#define BANDWIDTH_INTERVAL	max(HZ/5, 1)

#define RATELIMIT_CALC_SHIFT	10

static long ratelimit_pages = 32;


int dirty_background_ratio = 10;

unsigned long dirty_background_bytes;

int vm_highmem_is_dirtyable;

int vm_dirty_ratio = 20;

unsigned long vm_dirty_bytes;

unsigned int dirty_writeback_interval = 5 * 100; 

EXPORT_SYMBOL_GPL(dirty_writeback_interval);

unsigned int dirty_expire_interval = 30 * 100; 

int block_dump;

int laptop_mode;

EXPORT_SYMBOL(laptop_mode);


unsigned long global_dirty_limit;

/*
 * Scale the writeback cache size proportional to the relative writeout speeds.
 *
 * We do this by keeping a floating proportion between BDIs, based on page
 * writeback completions [end_page_writeback()]. Those devices that write out
 * pages fastest will get the larger share, while the slower will get a smaller
 * share.
 *
 * We use page writeout completions because we are interested in getting rid of
 * dirty pages. Having them written out is the primary goal.
 *
 * We introduce a concept of time, a period over which we measure these events,
 * because demand can/will vary over time. The length of this period itself is
 * measured in page writeback completions.
 *
 */
static struct prop_descriptor vm_completions;



static unsigned long highmem_dirtyable_memory(unsigned long total)
{
#ifdef CONFIG_HIGHMEM
	int node;
	unsigned long x = 0;

	for_each_node_state(node, N_HIGH_MEMORY) {
		struct zone *z =
			&NODE_DATA(node)->node_zones[ZONE_HIGHMEM];

		x += zone_page_state(z, NR_FREE_PAGES) +
		     zone_reclaimable_pages(z) - z->dirty_balance_reserve;
	}
	if ((long)x < 0)
		x = 0;

	return min(x, total);
#else
	return 0;
#endif
}

unsigned long global_dirtyable_memory(void)
{
	unsigned long x;

	x = global_page_state(NR_FREE_PAGES) + global_reclaimable_pages();
	x -= min(x, dirty_balance_reserve);

	if (!vm_highmem_is_dirtyable)
		x -= highmem_dirtyable_memory(x);

	return x + 1;	
}

void global_dirty_limits(unsigned long *pbackground, unsigned long *pdirty)
{
	unsigned long background;
	unsigned long dirty;
	unsigned long uninitialized_var(available_memory);
	struct task_struct *tsk;

	if (!vm_dirty_bytes || !dirty_background_bytes)
		available_memory = global_dirtyable_memory();

	if (vm_dirty_bytes)
		dirty = DIV_ROUND_UP(vm_dirty_bytes, PAGE_SIZE);
	else
		dirty = (vm_dirty_ratio * available_memory) / 100;

	if (dirty_background_bytes)
		background = DIV_ROUND_UP(dirty_background_bytes, PAGE_SIZE);
	else
		background = (dirty_background_ratio * available_memory) / 100;

	if (background >= dirty)
		background = dirty / 2;
	tsk = current;
	if (tsk->flags & PF_LESS_THROTTLE || rt_task(tsk)) {
		background += background / 4;
		dirty += dirty / 4;
	}
	*pbackground = background;
	*pdirty = dirty;
	trace_global_dirty_state(background, dirty);
}

static unsigned long zone_dirtyable_memory(struct zone *zone)
{
	unsigned long nr_pages = zone_page_state(zone, NR_FREE_PAGES) +
		zone_reclaimable_pages(zone);

	
	nr_pages -= min(nr_pages, zone->dirty_balance_reserve);
	return nr_pages;
}

static unsigned long zone_dirty_limit(struct zone *zone)
{
	unsigned long zone_memory = zone_dirtyable_memory(zone);
	struct task_struct *tsk = current;
	unsigned long dirty;

	if (vm_dirty_bytes)
		dirty = DIV_ROUND_UP(vm_dirty_bytes, PAGE_SIZE) *
			zone_memory / global_dirtyable_memory();
	else
		dirty = vm_dirty_ratio * zone_memory / 100;

	if (tsk->flags & PF_LESS_THROTTLE || rt_task(tsk))
		dirty += dirty / 4;

	return dirty;
}

bool zone_dirty_ok(struct zone *zone)
{
	unsigned long limit = zone_dirty_limit(zone);

	return zone_page_state(zone, NR_FILE_DIRTY) +
	       zone_page_state(zone, NR_UNSTABLE_NFS) +
	       zone_page_state(zone, NR_WRITEBACK) <= limit;
}

static int calc_period_shift(void)
{
	unsigned long dirty_total;

	if (vm_dirty_bytes)
		dirty_total = vm_dirty_bytes / PAGE_SIZE;
	else
		dirty_total = (vm_dirty_ratio * global_dirtyable_memory()) /
				100;
	return 2 + ilog2(dirty_total - 1);
}

static void update_completion_period(void)
{
	int shift = calc_period_shift();
	prop_change_shift(&vm_completions, shift);

	writeback_set_ratelimit();
}

int dirty_background_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		dirty_background_bytes = 0;
	return ret;
}

int dirty_background_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		dirty_background_ratio = 0;
	return ret;
}

int dirty_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int old_ratio = vm_dirty_ratio;
	int ret;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write && vm_dirty_ratio != old_ratio) {
		update_completion_period();
		vm_dirty_bytes = 0;
	}
	return ret;
}

int dirty_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	unsigned long old_bytes = vm_dirty_bytes;
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write && vm_dirty_bytes != old_bytes) {
		update_completion_period();
		vm_dirty_ratio = 0;
	}
	return ret;
}

static inline void __bdi_writeout_inc(struct backing_dev_info *bdi)
{
	__inc_bdi_stat(bdi, BDI_WRITTEN);
	__prop_inc_percpu_max(&vm_completions, &bdi->completions,
			      bdi->max_prop_frac);
}

void bdi_writeout_inc(struct backing_dev_info *bdi)
{
	unsigned long flags;

	local_irq_save(flags);
	__bdi_writeout_inc(bdi);
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(bdi_writeout_inc);

static void bdi_writeout_fraction(struct backing_dev_info *bdi,
		long *numerator, long *denominator)
{
	prop_fraction_percpu(&vm_completions, &bdi->completions,
				numerator, denominator);
}

static unsigned int bdi_min_ratio;

int bdi_set_min_ratio(struct backing_dev_info *bdi, unsigned int min_ratio)
{
	int ret = 0;

	spin_lock_bh(&bdi_lock);
	if (min_ratio > bdi->max_ratio) {
		ret = -EINVAL;
	} else {
		min_ratio -= bdi->min_ratio;
		if (bdi_min_ratio + min_ratio < 100) {
			bdi_min_ratio += min_ratio;
			bdi->min_ratio += min_ratio;
		} else {
			ret = -EINVAL;
		}
	}
	spin_unlock_bh(&bdi_lock);

	return ret;
}

int bdi_set_max_ratio(struct backing_dev_info *bdi, unsigned max_ratio)
{
	int ret = 0;

	if (max_ratio > 100)
		return -EINVAL;

	spin_lock_bh(&bdi_lock);
	if (bdi->min_ratio > max_ratio) {
		ret = -EINVAL;
	} else {
		bdi->max_ratio = max_ratio;
		bdi->max_prop_frac = (PROP_FRAC_BASE * max_ratio) / 100;
	}
	spin_unlock_bh(&bdi_lock);

	return ret;
}
EXPORT_SYMBOL(bdi_set_max_ratio);

static unsigned long dirty_freerun_ceiling(unsigned long thresh,
					   unsigned long bg_thresh)
{
	return (thresh + bg_thresh) / 2;
}

static unsigned long hard_dirty_limit(unsigned long thresh)
{
	return max(thresh, global_dirty_limit);
}

unsigned long bdi_dirty_limit(struct backing_dev_info *bdi, unsigned long dirty)
{
	u64 bdi_dirty;
	long numerator, denominator;

	bdi_writeout_fraction(bdi, &numerator, &denominator);

	bdi_dirty = (dirty * (100 - bdi_min_ratio)) / 100;
	bdi_dirty *= numerator;
	do_div(bdi_dirty, denominator);

	bdi_dirty += (dirty * bdi->min_ratio) / 100;
	if (bdi_dirty > (dirty * bdi->max_ratio) / 100)
		bdi_dirty = dirty * bdi->max_ratio / 100;

	return bdi_dirty;
}

static unsigned long bdi_position_ratio(struct backing_dev_info *bdi,
					unsigned long thresh,
					unsigned long bg_thresh,
					unsigned long dirty,
					unsigned long bdi_thresh,
					unsigned long bdi_dirty)
{
	unsigned long write_bw = bdi->avg_write_bandwidth;
	unsigned long freerun = dirty_freerun_ceiling(thresh, bg_thresh);
	unsigned long limit = hard_dirty_limit(thresh);
	unsigned long x_intercept;
	unsigned long setpoint;		
	unsigned long bdi_setpoint;
	unsigned long span;
	long long pos_ratio;		
	long x;

	if (unlikely(dirty >= limit))
		return 0;

	setpoint = (freerun + limit) / 2;
	x = div_s64((setpoint - dirty) << RATELIMIT_CALC_SHIFT,
		    limit - setpoint + 1);
	pos_ratio = x;
	pos_ratio = pos_ratio * x >> RATELIMIT_CALC_SHIFT;
	pos_ratio = pos_ratio * x >> RATELIMIT_CALC_SHIFT;
	pos_ratio += 1 << RATELIMIT_CALC_SHIFT;


	if (unlikely(bdi_thresh > thresh))
		bdi_thresh = thresh;
	bdi_thresh = max(bdi_thresh, (limit - dirty) / 8);
	x = div_u64((u64)bdi_thresh << 16, thresh + 1);
	bdi_setpoint = setpoint * (u64)x >> 16;
	span = (thresh - bdi_thresh + 8 * write_bw) * (u64)x >> 16;
	x_intercept = bdi_setpoint + span;

	if (bdi_dirty < x_intercept - span / 4) {
		pos_ratio = div_u64(pos_ratio * (x_intercept - bdi_dirty),
				    x_intercept - bdi_setpoint + 1);
	} else
		pos_ratio /= 4;

	x_intercept = bdi_thresh / 2;
	if (bdi_dirty < x_intercept) {
		if (bdi_dirty > x_intercept / 8)
			pos_ratio = div_u64(pos_ratio * x_intercept, bdi_dirty);
		else
			pos_ratio *= 8;
	}

	return pos_ratio;
}

static void bdi_update_write_bandwidth(struct backing_dev_info *bdi,
				       unsigned long elapsed,
				       unsigned long written)
{
	const unsigned long period = roundup_pow_of_two(3 * HZ);
	unsigned long avg = bdi->avg_write_bandwidth;
	unsigned long old = bdi->write_bandwidth;
	u64 bw;

	/*
	 * bw = written * HZ / elapsed
	 *
	 *                   bw * elapsed + write_bandwidth * (period - elapsed)
	 * write_bandwidth = ---------------------------------------------------
	 *                                          period
	 */
	bw = written - bdi->written_stamp;
	bw *= HZ;
	if (unlikely(elapsed > period)) {
		do_div(bw, elapsed);
		avg = bw;
		goto out;
	}
	bw += (u64)bdi->write_bandwidth * (period - elapsed);
	bw >>= ilog2(period);

	if (avg > old && old >= (unsigned long)bw)
		avg -= (avg - old) >> 3;

	if (avg < old && old <= (unsigned long)bw)
		avg += (old - avg) >> 3;

out:
	bdi->write_bandwidth = bw;
	bdi->avg_write_bandwidth = avg;
}

static void update_dirty_limit(unsigned long thresh, unsigned long dirty)
{
	unsigned long limit = global_dirty_limit;

	if (limit < thresh) {
		limit = thresh;
		goto update;
	}

	thresh = max(thresh, dirty);
	if (limit > thresh) {
		limit -= (limit - thresh) >> 5;
		goto update;
	}
	return;
update:
	global_dirty_limit = limit;
}

static void global_update_bandwidth(unsigned long thresh,
				    unsigned long dirty,
				    unsigned long now)
{
	static DEFINE_SPINLOCK(dirty_lock);
	static unsigned long update_time;

	if (time_before(now, update_time + BANDWIDTH_INTERVAL))
		return;

	spin_lock(&dirty_lock);
	if (time_after_eq(now, update_time + BANDWIDTH_INTERVAL)) {
		update_dirty_limit(thresh, dirty);
		update_time = now;
	}
	spin_unlock(&dirty_lock);
}

static void bdi_update_dirty_ratelimit(struct backing_dev_info *bdi,
				       unsigned long thresh,
				       unsigned long bg_thresh,
				       unsigned long dirty,
				       unsigned long bdi_thresh,
				       unsigned long bdi_dirty,
				       unsigned long dirtied,
				       unsigned long elapsed)
{
	unsigned long freerun = dirty_freerun_ceiling(thresh, bg_thresh);
	unsigned long limit = hard_dirty_limit(thresh);
	unsigned long setpoint = (freerun + limit) / 2;
	unsigned long write_bw = bdi->avg_write_bandwidth;
	unsigned long dirty_ratelimit = bdi->dirty_ratelimit;
	unsigned long dirty_rate;
	unsigned long task_ratelimit;
	unsigned long balanced_dirty_ratelimit;
	unsigned long pos_ratio;
	unsigned long step;
	unsigned long x;

	dirty_rate = (dirtied - bdi->dirtied_stamp) * HZ / elapsed;

	pos_ratio = bdi_position_ratio(bdi, thresh, bg_thresh, dirty,
				       bdi_thresh, bdi_dirty);
	task_ratelimit = (u64)dirty_ratelimit *
					pos_ratio >> RATELIMIT_CALC_SHIFT;
	task_ratelimit++; 

	balanced_dirty_ratelimit = div_u64((u64)task_ratelimit * write_bw,
					   dirty_rate | 1);
	if (unlikely(balanced_dirty_ratelimit > write_bw))
		balanced_dirty_ratelimit = write_bw;


	step = 0;
	if (dirty < setpoint) {
		x = min(bdi->balanced_dirty_ratelimit,
			 min(balanced_dirty_ratelimit, task_ratelimit));
		if (dirty_ratelimit < x)
			step = x - dirty_ratelimit;
	} else {
		x = max(bdi->balanced_dirty_ratelimit,
			 max(balanced_dirty_ratelimit, task_ratelimit));
		if (dirty_ratelimit > x)
			step = dirty_ratelimit - x;
	}

	step >>= dirty_ratelimit / (2 * step + 1);
	step = (step + 7) / 8;

	if (dirty_ratelimit < balanced_dirty_ratelimit)
		dirty_ratelimit += step;
	else
		dirty_ratelimit -= step;

	bdi->dirty_ratelimit = max(dirty_ratelimit, 1UL);
	bdi->balanced_dirty_ratelimit = balanced_dirty_ratelimit;

	trace_bdi_dirty_ratelimit(bdi, dirty_rate, task_ratelimit);
}

void __bdi_update_bandwidth(struct backing_dev_info *bdi,
			    unsigned long thresh,
			    unsigned long bg_thresh,
			    unsigned long dirty,
			    unsigned long bdi_thresh,
			    unsigned long bdi_dirty,
			    unsigned long start_time)
{
	unsigned long now = jiffies;
	unsigned long elapsed = now - bdi->bw_time_stamp;
	unsigned long dirtied;
	unsigned long written;

	if (elapsed < BANDWIDTH_INTERVAL)
		return;

	dirtied = percpu_counter_read(&bdi->bdi_stat[BDI_DIRTIED]);
	written = percpu_counter_read(&bdi->bdi_stat[BDI_WRITTEN]);

	if (elapsed > HZ && time_before(bdi->bw_time_stamp, start_time))
		goto snapshot;

	if (thresh) {
		global_update_bandwidth(thresh, dirty, now);
		bdi_update_dirty_ratelimit(bdi, thresh, bg_thresh, dirty,
					   bdi_thresh, bdi_dirty,
					   dirtied, elapsed);
	}
	bdi_update_write_bandwidth(bdi, elapsed, written);

snapshot:
	bdi->dirtied_stamp = dirtied;
	bdi->written_stamp = written;
	bdi->bw_time_stamp = now;
}

static void bdi_update_bandwidth(struct backing_dev_info *bdi,
				 unsigned long thresh,
				 unsigned long bg_thresh,
				 unsigned long dirty,
				 unsigned long bdi_thresh,
				 unsigned long bdi_dirty,
				 unsigned long start_time)
{
	if (time_is_after_eq_jiffies(bdi->bw_time_stamp + BANDWIDTH_INTERVAL))
		return;
	spin_lock(&bdi->wb.list_lock);
	__bdi_update_bandwidth(bdi, thresh, bg_thresh, dirty,
			       bdi_thresh, bdi_dirty, start_time);
	spin_unlock(&bdi->wb.list_lock);
}

static unsigned long dirty_poll_interval(unsigned long dirty,
					 unsigned long thresh)
{
	if (thresh > dirty)
		return 1UL << (ilog2(thresh - dirty) >> 1);

	return 1;
}

static long bdi_max_pause(struct backing_dev_info *bdi,
			  unsigned long bdi_dirty)
{
	long bw = bdi->avg_write_bandwidth;
	long t;

	t = bdi_dirty / (1 + bw / roundup_pow_of_two(1 + HZ / 8));
	t++;

	return min_t(long, t, MAX_PAUSE);
}

static long bdi_min_pause(struct backing_dev_info *bdi,
			  long max_pause,
			  unsigned long task_ratelimit,
			  unsigned long dirty_ratelimit,
			  int *nr_dirtied_pause)
{
	long hi = ilog2(bdi->avg_write_bandwidth);
	long lo = ilog2(bdi->dirty_ratelimit);
	long t;		
	long pause;	
	int pages;	

	
	t = max(1, HZ / 100);

	if (hi > lo)
		t += (hi - lo) * (10 * HZ) / 1024;

	t = min(t, 1 + max_pause / 2);
	pages = dirty_ratelimit * t / roundup_pow_of_two(HZ);

	if (pages < DIRTY_POLL_THRESH) {
		t = max_pause;
		pages = dirty_ratelimit * t / roundup_pow_of_two(HZ);
		if (pages > DIRTY_POLL_THRESH) {
			pages = DIRTY_POLL_THRESH;
			t = HZ * DIRTY_POLL_THRESH / dirty_ratelimit;
		}
	}

	pause = HZ * pages / (task_ratelimit + 1);
	if (pause > max_pause) {
		t = max_pause;
		pages = task_ratelimit * t / roundup_pow_of_two(HZ);
	}

	*nr_dirtied_pause = pages;
	return pages >= DIRTY_POLL_THRESH ? 1 + t / 2 : t;
}

static void balance_dirty_pages(struct address_space *mapping,
				unsigned long pages_dirtied)
{
	unsigned long nr_reclaimable;	
	unsigned long bdi_reclaimable;
	unsigned long nr_dirty;  
	unsigned long bdi_dirty;
	unsigned long freerun;
	unsigned long background_thresh;
	unsigned long dirty_thresh;
	unsigned long bdi_thresh;
	long period;
	long pause;
	long max_pause;
	long min_pause;
	int nr_dirtied_pause;
	bool dirty_exceeded = false;
	unsigned long task_ratelimit;
	unsigned long dirty_ratelimit;
	unsigned long pos_ratio;
	struct backing_dev_info *bdi = mapping->backing_dev_info;
	unsigned long start_time = jiffies;

	for (;;) {
		unsigned long now = jiffies;

		/*
		 * Unstable writes are a feature of certain networked
		 * filesystems (i.e. NFS) in which data may have been
		 * written to the server's write cache, but has not yet
		 * been flushed to permanent storage.
		 */
		nr_reclaimable = global_page_state(NR_FILE_DIRTY) +
					global_page_state(NR_UNSTABLE_NFS);
		nr_dirty = nr_reclaimable + global_page_state(NR_WRITEBACK);

		global_dirty_limits(&background_thresh, &dirty_thresh);

		freerun = dirty_freerun_ceiling(dirty_thresh,
						background_thresh);
		if (nr_dirty <= freerun) {
			current->dirty_paused_when = now;
			current->nr_dirtied = 0;
			current->nr_dirtied_pause =
				dirty_poll_interval(nr_dirty, dirty_thresh);
			break;
		}

		if (unlikely(!writeback_in_progress(bdi)))
			bdi_start_background_writeback(bdi);

		bdi_thresh = bdi_dirty_limit(bdi, dirty_thresh);

		if (bdi_thresh < 2 * bdi_stat_error(bdi)) {
			bdi_reclaimable = bdi_stat_sum(bdi, BDI_RECLAIMABLE);
			bdi_dirty = bdi_reclaimable +
				    bdi_stat_sum(bdi, BDI_WRITEBACK);
		} else {
			bdi_reclaimable = bdi_stat(bdi, BDI_RECLAIMABLE);
			bdi_dirty = bdi_reclaimable +
				    bdi_stat(bdi, BDI_WRITEBACK);
		}

		dirty_exceeded = (bdi_dirty > bdi_thresh) &&
				  (nr_dirty > dirty_thresh);
		if (dirty_exceeded && !bdi->dirty_exceeded)
			bdi->dirty_exceeded = 1;

		bdi_update_bandwidth(bdi, dirty_thresh, background_thresh,
				     nr_dirty, bdi_thresh, bdi_dirty,
				     start_time);

		dirty_ratelimit = bdi->dirty_ratelimit;
		pos_ratio = bdi_position_ratio(bdi, dirty_thresh,
					       background_thresh, nr_dirty,
					       bdi_thresh, bdi_dirty);
		task_ratelimit = ((u64)dirty_ratelimit * pos_ratio) >>
							RATELIMIT_CALC_SHIFT;
		max_pause = bdi_max_pause(bdi, bdi_dirty);
		min_pause = bdi_min_pause(bdi, max_pause,
					  task_ratelimit, dirty_ratelimit,
					  &nr_dirtied_pause);

		if (unlikely(task_ratelimit == 0)) {
			period = max_pause;
			pause = max_pause;
			goto pause;
		}
		period = HZ * pages_dirtied / task_ratelimit;
		pause = period;
		if (current->dirty_paused_when)
			pause -= now - current->dirty_paused_when;
		if (pause < min_pause) {
			trace_balance_dirty_pages(bdi,
						  dirty_thresh,
						  background_thresh,
						  nr_dirty,
						  bdi_thresh,
						  bdi_dirty,
						  dirty_ratelimit,
						  task_ratelimit,
						  pages_dirtied,
						  period,
						  min(pause, 0L),
						  start_time);
			if (pause < -HZ) {
				current->dirty_paused_when = now;
				current->nr_dirtied = 0;
			} else if (period) {
				current->dirty_paused_when += period;
				current->nr_dirtied = 0;
			} else if (current->nr_dirtied_pause <= pages_dirtied)
				current->nr_dirtied_pause += pages_dirtied;
			break;
		}
		if (unlikely(pause > max_pause)) {
			
			now += min(pause - max_pause, max_pause);
			pause = max_pause;
		}

pause:
		trace_balance_dirty_pages(bdi,
					  dirty_thresh,
					  background_thresh,
					  nr_dirty,
					  bdi_thresh,
					  bdi_dirty,
					  dirty_ratelimit,
					  task_ratelimit,
					  pages_dirtied,
					  period,
					  pause,
					  start_time);
		__set_current_state(TASK_KILLABLE);
		io_schedule_timeout(pause);

		current->dirty_paused_when = now + pause;
		current->nr_dirtied = 0;
		current->nr_dirtied_pause = nr_dirtied_pause;

		if (task_ratelimit)
			break;

		if (bdi_dirty <= bdi_stat_error(bdi))
			break;

		if (fatal_signal_pending(current))
			break;
	}

	if (!dirty_exceeded && bdi->dirty_exceeded)
		bdi->dirty_exceeded = 0;

	if (writeback_in_progress(bdi))
		return;

	if (laptop_mode)
		return;

	if (nr_reclaimable > background_thresh)
		bdi_start_background_writeback(bdi);
}

void set_page_dirty_balance(struct page *page, int page_mkwrite)
{
	if (set_page_dirty(page) || page_mkwrite) {
		struct address_space *mapping = page_mapping(page);

		if (mapping)
			balance_dirty_pages_ratelimited(mapping);
	}
}

static DEFINE_PER_CPU(int, bdp_ratelimits);

DEFINE_PER_CPU(int, dirty_throttle_leaks) = 0;

void balance_dirty_pages_ratelimited_nr(struct address_space *mapping,
					unsigned long nr_pages_dirtied)
{
	struct backing_dev_info *bdi = mapping->backing_dev_info;
	int ratelimit;
	int *p;

	if (!bdi_cap_account_dirty(bdi))
		return;

	ratelimit = current->nr_dirtied_pause;
	if (bdi->dirty_exceeded)
		ratelimit = min(ratelimit, 32 >> (PAGE_SHIFT - 10));

	preempt_disable();
	p =  &__get_cpu_var(bdp_ratelimits);
	if (unlikely(current->nr_dirtied >= ratelimit))
		*p = 0;
	else if (unlikely(*p >= ratelimit_pages)) {
		*p = 0;
		ratelimit = 0;
	}
	p = &__get_cpu_var(dirty_throttle_leaks);
	if (*p > 0 && current->nr_dirtied < ratelimit) {
		nr_pages_dirtied = min(*p, ratelimit - current->nr_dirtied);
		*p -= nr_pages_dirtied;
		current->nr_dirtied += nr_pages_dirtied;
	}
	preempt_enable();

	if (unlikely(current->nr_dirtied >= ratelimit))
		balance_dirty_pages(mapping, current->nr_dirtied);
}
EXPORT_SYMBOL(balance_dirty_pages_ratelimited_nr);

void throttle_vm_writeout(gfp_t gfp_mask)
{
	unsigned long background_thresh;
	unsigned long dirty_thresh;

        for ( ; ; ) {
		global_dirty_limits(&background_thresh, &dirty_thresh);
		dirty_thresh = hard_dirty_limit(dirty_thresh);

                dirty_thresh += dirty_thresh / 10;      

                if (global_page_state(NR_UNSTABLE_NFS) +
			global_page_state(NR_WRITEBACK) <= dirty_thresh)
                        	break;
                congestion_wait(BLK_RW_ASYNC, HZ/10);

		if ((gfp_mask & (__GFP_FS|__GFP_IO)) != (__GFP_FS|__GFP_IO))
			break;
        }
}

int dirty_writeback_centisecs_handler(ctl_table *table, int write,
	void __user *buffer, size_t *length, loff_t *ppos)
{
	proc_dointvec(table, write, buffer, length, ppos);
	bdi_arm_supers_timer();
	return 0;
}

#ifdef CONFIG_BLOCK
void laptop_mode_timer_fn(unsigned long data)
{
	struct request_queue *q = (struct request_queue *)data;
	int nr_pages = global_page_state(NR_FILE_DIRTY) +
		global_page_state(NR_UNSTABLE_NFS);

	if (bdi_has_dirty_io(&q->backing_dev_info))
		bdi_start_writeback(&q->backing_dev_info, nr_pages,
					WB_REASON_LAPTOP_TIMER);
}

void laptop_io_completion(struct backing_dev_info *info)
{
	mod_timer(&info->laptop_mode_wb_timer, jiffies + laptop_mode);
}

/*
 * We're in laptop mode and we've just synced. The sync's writes will have
 * caused another writeback to be scheduled by laptop_io_completion.
 * Nothing needs to be written back anymore, so we unschedule the writeback.
 */
void laptop_sync_completion(void)
{
	struct backing_dev_info *bdi;

	rcu_read_lock();

	list_for_each_entry_rcu(bdi, &bdi_list, bdi_list)
		del_timer(&bdi->laptop_mode_wb_timer);

	rcu_read_unlock();
}
#endif


void writeback_set_ratelimit(void)
{
	unsigned long background_thresh;
	unsigned long dirty_thresh;
	global_dirty_limits(&background_thresh, &dirty_thresh);
	ratelimit_pages = dirty_thresh / (num_online_cpus() * 32);
	if (ratelimit_pages < 16)
		ratelimit_pages = 16;
}

static int __cpuinit
ratelimit_handler(struct notifier_block *self, unsigned long u, void *v)
{
	writeback_set_ratelimit();
	return NOTIFY_DONE;
}

static struct notifier_block __cpuinitdata ratelimit_nb = {
	.notifier_call	= ratelimit_handler,
	.next		= NULL,
};

void __init page_writeback_init(void)
{
	int shift;

	writeback_set_ratelimit();
	register_cpu_notifier(&ratelimit_nb);

	shift = calc_period_shift();
	prop_descriptor_init(&vm_completions, shift);
}

/**
 * tag_pages_for_writeback - tag pages to be written by write_cache_pages
 * @mapping: address space structure to write
 * @start: starting page index
 * @end: ending page index (inclusive)
 *
 * This function scans the page range from @start to @end (inclusive) and tags
 * all pages that have DIRTY tag set with a special TOWRITE tag. The idea is
 * that write_cache_pages (or whoever calls this function) will then use
 * TOWRITE tag to identify pages eligible for writeback.  This mechanism is
 * used to avoid livelocking of writeback by a process steadily creating new
 * dirty pages in the file (thus it is important for this function to be quick
 * so that it can tag pages faster than a dirtying process can create them).
 */
void tag_pages_for_writeback(struct address_space *mapping,
			     pgoff_t start, pgoff_t end)
{
#define WRITEBACK_TAG_BATCH 4096
	unsigned long tagged;

	do {
		spin_lock_irq(&mapping->tree_lock);
		tagged = radix_tree_range_tag_if_tagged(&mapping->page_tree,
				&start, end, WRITEBACK_TAG_BATCH,
				PAGECACHE_TAG_DIRTY, PAGECACHE_TAG_TOWRITE);
		spin_unlock_irq(&mapping->tree_lock);
		WARN_ON_ONCE(tagged > WRITEBACK_TAG_BATCH);
		cond_resched();
		
	} while (tagged >= WRITEBACK_TAG_BATCH && start);
}
EXPORT_SYMBOL(tag_pages_for_writeback);

/**
 * write_cache_pages - walk the list of dirty pages of the given address space and write all of them.
 * @mapping: address space structure to write
 * @wbc: subtract the number of written pages from *@wbc->nr_to_write
 * @writepage: function called for each page
 * @data: data passed to writepage function
 *
 * If a page is already under I/O, write_cache_pages() skips it, even
 * if it's dirty.  This is desirable behaviour for memory-cleaning writeback,
 * but it is INCORRECT for data-integrity system calls such as fsync().  fsync()
 * and msync() need to guarantee that all the data which was dirty at the time
 * the call was made get new I/O started against them.  If wbc->sync_mode is
 * WB_SYNC_ALL then we were called for data integrity and we must wait for
 * existing IO to complete.
 *
 * To avoid livelocks (when other process dirties new pages), we first tag
 * pages which should be written back with TOWRITE tag and only then start
 * writing them. For data-integrity sync we have to be careful so that we do
 * not miss some pages (e.g., because some other process has cleared TOWRITE
 * tag we set). The rule we follow is that TOWRITE tag can be cleared only
 * by the process clearing the DIRTY tag (and submitting the page for IO).
 */
int write_cache_pages(struct address_space *mapping,
		      struct writeback_control *wbc, writepage_t writepage,
		      void *data)
{
	int ret = 0;
	int done = 0;
	struct pagevec pvec;
	int nr_pages;
	pgoff_t uninitialized_var(writeback_index);
	pgoff_t index;
	pgoff_t end;		
	pgoff_t done_index;
	int cycled;
	int range_whole = 0;
	int tag;

	pagevec_init(&pvec, 0);
	if (wbc->range_cyclic) {
		writeback_index = mapping->writeback_index; 
		index = writeback_index;
		if (index == 0)
			cycled = 1;
		else
			cycled = 0;
		end = -1;
	} else {
		index = wbc->range_start >> PAGE_CACHE_SHIFT;
		end = wbc->range_end >> PAGE_CACHE_SHIFT;
		if (wbc->range_start == 0 && wbc->range_end == LLONG_MAX)
			range_whole = 1;
		cycled = 1; 
	}
	if (wbc->sync_mode == WB_SYNC_ALL || wbc->tagged_writepages)
		tag = PAGECACHE_TAG_TOWRITE;
	else
		tag = PAGECACHE_TAG_DIRTY;
retry:
	if (wbc->sync_mode == WB_SYNC_ALL || wbc->tagged_writepages)
		tag_pages_for_writeback(mapping, index, end);
	done_index = index;
	while (!done && (index <= end)) {
		int i;

		nr_pages = pagevec_lookup_tag(&pvec, mapping, &index, tag,
			      min(end - index, (pgoff_t)PAGEVEC_SIZE-1) + 1);
		if (nr_pages == 0)
			break;

		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];

			if (page->index > end) {
				done = 1;
				break;
			}

			done_index = page->index;

			lock_page(page);

			if (unlikely(page->mapping != mapping)) {
continue_unlock:
				unlock_page(page);
				continue;
			}

			if (!PageDirty(page)) {
				
				goto continue_unlock;
			}

			if (PageWriteback(page)) {
				if (wbc->sync_mode != WB_SYNC_NONE)
					wait_on_page_writeback(page);
				else
					goto continue_unlock;
			}

			BUG_ON(PageWriteback(page));
			if (!clear_page_dirty_for_io(page))
				goto continue_unlock;

			trace_wbc_writepage(wbc, mapping->backing_dev_info);
			ret = (*writepage)(page, wbc, data);
			if (unlikely(ret)) {
				if (ret == AOP_WRITEPAGE_ACTIVATE) {
					unlock_page(page);
					ret = 0;
				} else {
					done_index = page->index + 1;
					done = 1;
					break;
				}
			}

			/*
			 * We stop writing back only if we are not doing
			 * integrity sync. In case of integrity sync we have to
			 * keep going until we have written all the pages
			 * we tagged for writeback prior to entering this loop.
			 */
			if (--wbc->nr_to_write <= 0 &&
			    wbc->sync_mode == WB_SYNC_NONE) {
				done = 1;
				break;
			}
		}
		pagevec_release(&pvec);
		cond_resched();
	}
	if (!cycled && !done) {
		cycled = 1;
		index = 0;
		end = writeback_index - 1;
		goto retry;
	}
	if (wbc->range_cyclic || (range_whole && wbc->nr_to_write > 0))
		mapping->writeback_index = done_index;

	return ret;
}
EXPORT_SYMBOL(write_cache_pages);

static int __writepage(struct page *page, struct writeback_control *wbc,
		       void *data)
{
	struct address_space *mapping = data;
	int ret = mapping->a_ops->writepage(page, wbc);
	mapping_set_error(mapping, ret);
	return ret;
}

/**
 * generic_writepages - walk the list of dirty pages of the given address space and writepage() all of them.
 * @mapping: address space structure to write
 * @wbc: subtract the number of written pages from *@wbc->nr_to_write
 *
 * This is a library function, which implements the writepages()
 * address_space_operation.
 */
int generic_writepages(struct address_space *mapping,
		       struct writeback_control *wbc)
{
	struct blk_plug plug;
	int ret;

	
	if (!mapping->a_ops->writepage)
		return 0;

	blk_start_plug(&plug);
	ret = write_cache_pages(mapping, wbc, __writepage, mapping);
	blk_finish_plug(&plug);
	return ret;
}

EXPORT_SYMBOL(generic_writepages);

int do_writepages(struct address_space *mapping, struct writeback_control *wbc)
{
	int ret;

	if (wbc->nr_to_write <= 0)
		return 0;
	if (mapping->a_ops->writepages)
		ret = mapping->a_ops->writepages(mapping, wbc);
	else
		ret = generic_writepages(mapping, wbc);
	return ret;
}

int write_one_page(struct page *page, int wait)
{
	struct address_space *mapping = page->mapping;
	int ret = 0;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_ALL,
		.nr_to_write = 1,
	};

	BUG_ON(!PageLocked(page));

	if (wait)
		wait_on_page_writeback(page);

	if (clear_page_dirty_for_io(page)) {
		page_cache_get(page);
		ret = mapping->a_ops->writepage(page, &wbc);
		if (ret == 0 && wait) {
			wait_on_page_writeback(page);
			if (PageError(page))
				ret = -EIO;
		}
		page_cache_release(page);
	} else {
		unlock_page(page);
	}
	return ret;
}
EXPORT_SYMBOL(write_one_page);

int __set_page_dirty_no_writeback(struct page *page)
{
	if (!PageDirty(page))
		return !TestSetPageDirty(page);
	return 0;
}

void account_page_dirtied(struct page *page, struct address_space *mapping)
{
	if (mapping_cap_account_dirty(mapping)) {
		__inc_zone_page_state(page, NR_FILE_DIRTY);
		__inc_zone_page_state(page, NR_DIRTIED);
		__inc_bdi_stat(mapping->backing_dev_info, BDI_RECLAIMABLE);
		__inc_bdi_stat(mapping->backing_dev_info, BDI_DIRTIED);
		task_io_account_write(PAGE_CACHE_SIZE);
		collect_io_stats(PAGE_CACHE_SIZE, WRITE);
		current->nr_dirtied++;
		this_cpu_inc(bdp_ratelimits);
	}
}
EXPORT_SYMBOL(account_page_dirtied);

void account_page_writeback(struct page *page)
{
	inc_zone_page_state(page, NR_WRITEBACK);
}
EXPORT_SYMBOL(account_page_writeback);

int __set_page_dirty_nobuffers(struct page *page)
{
	if (!TestSetPageDirty(page)) {
		struct address_space *mapping = page_mapping(page);
		struct address_space *mapping2;

		if (!mapping)
			return 1;

		spin_lock_irq(&mapping->tree_lock);
		mapping2 = page_mapping(page);
		if (mapping2) { 
			BUG_ON(mapping2 != mapping);
			WARN_ON_ONCE(!PagePrivate(page) && !PageUptodate(page));
			account_page_dirtied(page, mapping);
			radix_tree_tag_set(&mapping->page_tree,
				page_index(page), PAGECACHE_TAG_DIRTY);
		}
		spin_unlock_irq(&mapping->tree_lock);
		if (mapping->host) {
			
			__mark_inode_dirty(mapping->host, I_DIRTY_PAGES);
		}
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(__set_page_dirty_nobuffers);

/*
 * Call this whenever redirtying a page, to de-account the dirty counters
 * (NR_DIRTIED, BDI_DIRTIED, tsk->nr_dirtied), so that they match the written
 * counters (NR_WRITTEN, BDI_WRITTEN) in long term. The mismatches will lead to
 * systematic errors in balanced_dirty_ratelimit and the dirty pages position
 * control.
 */
void account_page_redirty(struct page *page)
{
	struct address_space *mapping = page->mapping;
	if (mapping && mapping_cap_account_dirty(mapping)) {
		current->nr_dirtied--;
		dec_zone_page_state(page, NR_DIRTIED);
		dec_bdi_stat(mapping->backing_dev_info, BDI_DIRTIED);
	}
}
EXPORT_SYMBOL(account_page_redirty);

int redirty_page_for_writepage(struct writeback_control *wbc, struct page *page)
{
	wbc->pages_skipped++;
	account_page_redirty(page);
	return __set_page_dirty_nobuffers(page);
}
EXPORT_SYMBOL(redirty_page_for_writepage);

int set_page_dirty(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

	if (likely(mapping)) {
		int (*spd)(struct page *) = mapping->a_ops->set_page_dirty;
		/*
		 * readahead/lru_deactivate_page could remain
		 * PG_readahead/PG_reclaim due to race with end_page_writeback
		 * About readahead, if the page is written, the flags would be
		 * reset. So no problem.
		 * About lru_deactivate_page, if the page is redirty, the flag
		 * will be reset. So no problem. but if the page is used by readahead
		 * it will confuse readahead and make it restart the size rampup
		 * process. But it's a trivial problem.
		 */
		ClearPageReclaim(page);
#ifdef CONFIG_BLOCK
		if (!spd)
			spd = __set_page_dirty_buffers;
#endif
		return (*spd)(page);
	}
	if (!PageDirty(page)) {
		if (!TestSetPageDirty(page))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(set_page_dirty);

int set_page_dirty_lock(struct page *page)
{
	int ret;

	lock_page(page);
	ret = set_page_dirty(page);
	unlock_page(page);
	return ret;
}
EXPORT_SYMBOL(set_page_dirty_lock);

int clear_page_dirty_for_io(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

	BUG_ON(!PageLocked(page));

	if (mapping && mapping_cap_account_dirty(mapping)) {
		if (page_mkclean(page))
			set_page_dirty(page);
		if (TestClearPageDirty(page)) {
			dec_zone_page_state(page, NR_FILE_DIRTY);
			dec_bdi_stat(mapping->backing_dev_info,
					BDI_RECLAIMABLE);
			return 1;
		}
		return 0;
	}
	return TestClearPageDirty(page);
}
EXPORT_SYMBOL(clear_page_dirty_for_io);

int test_clear_page_writeback(struct page *page)
{
	struct address_space *mapping = page_mapping(page);
	int ret;

	if (mapping) {
		struct backing_dev_info *bdi = mapping->backing_dev_info;
		unsigned long flags;

		spin_lock_irqsave(&mapping->tree_lock, flags);
		ret = TestClearPageWriteback(page);
		if (ret) {
			radix_tree_tag_clear(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_WRITEBACK);
			if (bdi_cap_account_writeback(bdi)) {
				__dec_bdi_stat(bdi, BDI_WRITEBACK);
				__bdi_writeout_inc(bdi);
			}
		}
		spin_unlock_irqrestore(&mapping->tree_lock, flags);
	} else {
		ret = TestClearPageWriteback(page);
	}
	if (ret) {
		dec_zone_page_state(page, NR_WRITEBACK);
		inc_zone_page_state(page, NR_WRITTEN);
	}
	return ret;
}

int test_set_page_writeback(struct page *page)
{
	struct address_space *mapping = page_mapping(page);
	int ret;

	if (mapping) {
		struct backing_dev_info *bdi = mapping->backing_dev_info;
		unsigned long flags;

		spin_lock_irqsave(&mapping->tree_lock, flags);
		ret = TestSetPageWriteback(page);
		if (!ret) {
			radix_tree_tag_set(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_WRITEBACK);
			if (bdi_cap_account_writeback(bdi))
				__inc_bdi_stat(bdi, BDI_WRITEBACK);
		}
		if (!PageDirty(page))
			radix_tree_tag_clear(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_DIRTY);
		radix_tree_tag_clear(&mapping->page_tree,
				     page_index(page),
				     PAGECACHE_TAG_TOWRITE);
		spin_unlock_irqrestore(&mapping->tree_lock, flags);
	} else {
		ret = TestSetPageWriteback(page);
	}
	if (!ret)
		account_page_writeback(page);
	return ret;

}
EXPORT_SYMBOL(test_set_page_writeback);

int mapping_tagged(struct address_space *mapping, int tag)
{
	return radix_tree_tagged(&mapping->page_tree, tag);
}
EXPORT_SYMBOL(mapping_tagged);

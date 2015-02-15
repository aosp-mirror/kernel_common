/*
 * ROW (Read Over Write) I/O scheduler.
 *
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/blktrace_api.h>
#include <linux/hrtimer.h>

enum row_queue_prio {
	ROWQ_PRIO_HIGH_READ = 0,
	ROWQ_PRIO_HIGH_SWRITE,
	ROWQ_PRIO_REG_READ,
	ROWQ_PRIO_REG_SWRITE,
	ROWQ_PRIO_REG_WRITE,
	ROWQ_PRIO_LOW_READ,
	ROWQ_PRIO_LOW_SWRITE,
	ROWQ_MAX_PRIO,
};

#define ROWQ_HIGH_PRIO_IDX	ROWQ_PRIO_HIGH_READ
#define ROWQ_REG_PRIO_IDX	ROWQ_PRIO_REG_READ
#define ROWQ_LOW_PRIO_IDX	ROWQ_PRIO_LOW_READ

struct row_queue_params {
	bool idling_enabled;
	int quantum;
	bool is_urgent;
	unsigned int expire;
};

static const struct row_queue_params row_queues_def[] = {
	{true, 10, true, 0},	
	{false, 1, false, 0},	
	{true, 100, true, 0},	
	{false, 1, false, 3 * HZ},	
	{false, 1, false, 3 * HZ},	
	{false, 1, false, 0},	
	{false, 1, false, 3 * HZ}	
};

#define ROW_IDLE_TIME_MSEC 5
#define ROW_READ_FREQ_MSEC 5

struct rowq_idling_data {
	ktime_t			last_insert_time;
	bool			begin_idling;
};

struct row_queue {
	struct row_data		*rdata;
	struct list_head	fifo;
	enum row_queue_prio	prio;

	unsigned int		nr_dispatched;

	unsigned int		nr_req;
	int			disp_quantum;

	
	struct rowq_idling_data	idle_data;

	unsigned int		dispatch_cnt;
};

struct idling_data {
	s64				idle_time_ms;
	s64				freq_ms;

	struct hrtimer			hr_timer;
	struct work_struct		idle_work;
	enum row_queue_prio		idling_queue_idx;
};

struct starvation_data {
	int				starvation_limit;
	int				starvation_counter;
};

struct row_data {
	struct request_queue		*dispatch_queue;

	struct row_queue row_queues[ROWQ_MAX_PRIO];

	struct idling_data		rd_idle_data;
	unsigned int			nr_reqs[2];
	bool				urgent_in_flight;
	struct request			*pending_urgent_rq;
	int				last_served_ioprio_class;

#define	ROW_REG_STARVATION_TOLLERANCE	5000
	struct starvation_data		reg_prio_starvation;
#define	ROW_LOW_STARVATION_TOLLERANCE	10000
	struct starvation_data		low_prio_starvation;

	unsigned int			cycle_flags;

	unsigned int			last_update_jiffies;
};

#define RQ_ROWQ(rq) ((struct row_queue *) ((rq)->elv.priv[0]))

#define row_log(q, fmt, args...)   \
	blk_add_trace_msg(q, "%s():" fmt , __func__, ##args)
#define row_log_rowq(rdata, rowq_id, fmt, args...)		\
	blk_add_trace_msg(rdata->dispatch_queue, "rowq%d " fmt, \
		rowq_id, ##args)

static inline void row_mark_rowq_unserved(struct row_data *rd,
					 enum row_queue_prio qnum)
{
	rd->cycle_flags |= (1 << qnum);
}

static inline void row_clear_rowq_unserved(struct row_data *rd,
					  enum row_queue_prio qnum)
{
	rd->cycle_flags &= ~(1 << qnum);
}

static inline int row_rowq_unserved(struct row_data *rd,
				   enum row_queue_prio qnum)
{
	return rd->cycle_flags & (1 << qnum);
}

static inline void __maybe_unused row_dump_queues_stat(struct row_data *rd)
{
	int i;

	row_log(rd->dispatch_queue, " Queues status:");
	for (i = 0; i < ROWQ_MAX_PRIO; i++)
		row_log(rd->dispatch_queue,
			"queue%d: dispatched= %d, nr_req=%d", i,
			rd->row_queues[i].nr_dispatched,
			rd->row_queues[i].nr_req);
}

#define ROW_DUMP_REQ_STAT_MSECS	    4500
static inline void __maybe_unused row_dump_reg_and_low_stat(struct row_data *rd)
{
	int i;
	unsigned int total_dispatch_cnt = 0;
	bool print_statistics = false;

	for (i = ROWQ_PRIO_REG_SWRITE; i < ROWQ_MAX_PRIO; i++) {
		if (!list_empty(&rd->row_queues[i].fifo)) {
			struct request *check_req = list_entry_rq(rd->row_queues[i].fifo.next);
			unsigned int check_jiffies = ((unsigned long) (check_req)->csd.list.next);

			if (time_after(jiffies,
			    check_jiffies + msecs_to_jiffies(ROW_DUMP_REQ_STAT_MSECS))) {
				printk("ROW scheduler: request(pid:%d)"
					" stays in queue[%d][nr_reqs:%d] for %ums\n",
					check_req->pid, i, rd->row_queues[i].nr_req,
					jiffies_to_msecs(jiffies - check_jiffies));
				print_statistics = true;
			}
		}
	}

	if (!print_statistics)
		return;

	printk("ROW scheduler: dispatched request statistics:");

	for (i = 0; i < ROWQ_MAX_PRIO; i++) {
		printk(" Q[%d]: %u;", i, rd->row_queues[i].dispatch_cnt);
		total_dispatch_cnt += rd->row_queues[i].dispatch_cnt;
		rd->row_queues[i].dispatch_cnt = 0;
	}
	printk("\n%u requests dispatched in %umsec\n",
		total_dispatch_cnt, jiffies_to_msecs(jiffies - rd->last_update_jiffies));

	rd->last_update_jiffies = jiffies;
}

static void kick_queue(struct work_struct *work)
{
	struct idling_data *read_data =
		container_of(work, struct idling_data, idle_work);
	struct row_data *rd =
		container_of(read_data, struct row_data, rd_idle_data);

	blk_run_queue(rd->dispatch_queue);
}


static enum hrtimer_restart row_idle_hrtimer_fn(struct hrtimer *hr_timer)
{
	struct idling_data *read_data =
		container_of(hr_timer, struct idling_data, hr_timer);
	struct row_data *rd =
		container_of(read_data, struct row_data, rd_idle_data);

	row_log_rowq(rd, rd->rd_idle_data.idling_queue_idx,
			 "Performing delayed work");
	
	rd->row_queues[rd->rd_idle_data.idling_queue_idx].
			idle_data.begin_idling = false;
	rd->rd_idle_data.idling_queue_idx = ROWQ_MAX_PRIO;

	if (!rd->nr_reqs[READ] && !rd->nr_reqs[WRITE])
		row_log(rd->dispatch_queue, "No requests in scheduler");
	else
		kblockd_schedule_work(rd->dispatch_queue,
			&read_data->idle_work);
	return HRTIMER_NORESTART;
}

static inline bool row_regular_req_pending(struct row_data *rd)
{
	int i;

	for (i = ROWQ_REG_PRIO_IDX; i < ROWQ_LOW_PRIO_IDX; i++)
		if (!list_empty(&rd->row_queues[i].fifo))
			return true;
	return false;
}

static inline bool row_low_req_pending(struct row_data *rd)
{
	int i;

	for (i = ROWQ_LOW_PRIO_IDX; i < ROWQ_MAX_PRIO; i++)
		if (!list_empty(&rd->row_queues[i].fifo))
			return true;
	return false;
}


static void row_add_request(struct request_queue *q,
			    struct request *rq)
{
	struct row_data *rd = (struct row_data *)q->elevator->elevator_data;
	struct row_queue *rqueue = RQ_ROWQ(rq);
	s64 diff_ms;
	bool queue_was_empty = list_empty(&rqueue->fifo);

	list_add_tail(&rq->queuelist, &rqueue->fifo);
	rd->nr_reqs[rq_data_dir(rq)]++;
	rqueue->nr_req++;
	rq_set_fifo_time(rq, jiffies); 

	if (rq->cmd_flags & REQ_URGENT) {
		WARN_ON(1);
		blk_dump_rq_flags(rq, "");
		rq->cmd_flags &= ~REQ_URGENT;
	}

	if (row_queues_def[rqueue->prio].idling_enabled) {
		if (rd->rd_idle_data.idling_queue_idx == rqueue->prio &&
		    hrtimer_active(&rd->rd_idle_data.hr_timer)) {
			if (hrtimer_try_to_cancel(
				&rd->rd_idle_data.hr_timer) >= 0) {
				row_log_rowq(rd, rqueue->prio,
				    "Canceled delayed work on %d",
				    rd->rd_idle_data.idling_queue_idx);
				rd->rd_idle_data.idling_queue_idx =
					ROWQ_MAX_PRIO;
			}
		}
		diff_ms = ktime_to_ms(ktime_sub(ktime_get(),
				rqueue->idle_data.last_insert_time));
		if (unlikely(diff_ms < 0)) {
			pr_err("%s(): time delta error: diff_ms < 0",
				__func__);
			rqueue->idle_data.begin_idling = false;
			return;
		}
		if (diff_ms < rd->rd_idle_data.freq_ms) {
			rqueue->idle_data.begin_idling = true;
			row_log_rowq(rd, rqueue->prio, "Enable idling");
		} else {
			rqueue->idle_data.begin_idling = false;
			row_log_rowq(rd, rqueue->prio, "Disable idling (%ldms)",
				(long)diff_ms);
		}

		rqueue->idle_data.last_insert_time = ktime_get();
	}
	if (row_queues_def[rqueue->prio].is_urgent &&
	    !rd->pending_urgent_rq && !rd->urgent_in_flight) {
		
		if (rqueue->prio < ROWQ_REG_PRIO_IDX &&
		    rd->last_served_ioprio_class != IOPRIO_CLASS_RT &&
		    queue_was_empty) {
			row_log_rowq(rd, rqueue->prio,
				"added (high prio) urgent request");
			rq->cmd_flags |= REQ_URGENT;
			rd->pending_urgent_rq = rq;
		} else  if (row_rowq_unserved(rd, rqueue->prio)) {
			
			row_log_rowq(rd, rqueue->prio,
				"added urgent request (total on queue=%d)",
				rqueue->nr_req);
			rq->cmd_flags |= REQ_URGENT;
			rd->pending_urgent_rq = rq;
		}
	} else
		row_log_rowq(rd, rqueue->prio,
			"added request (total on queue=%d)", rqueue->nr_req);
}

static int row_reinsert_req(struct request_queue *q,
			    struct request *rq)
{
	struct row_data    *rd = q->elevator->elevator_data;
	struct row_queue   *rqueue = RQ_ROWQ(rq);

	if (!rqueue || rqueue->prio >= ROWQ_MAX_PRIO)
		return -EIO;

	list_add(&rq->queuelist, &rqueue->fifo);
	rd->nr_reqs[rq_data_dir(rq)]++;
	rqueue->nr_req++;

	row_log_rowq(rd, rqueue->prio,
		"%s request reinserted (total on queue=%d)",
		(rq_data_dir(rq) == READ ? "READ" : "write"), rqueue->nr_req);

	if (rq->cmd_flags & REQ_URGENT) {
		WARN_ON(1);
		if (!rd->urgent_in_flight) {
			pr_err("%s(): no urgent in flight", __func__);
		} else {
			rd->urgent_in_flight = false;
			pr_err("%s(): reinserting URGENT %s req",
				__func__,
				(rq_data_dir(rq) == READ ? "READ" : "WRITE"));
			if (rd->pending_urgent_rq) {
				pr_err("%s(): urgent rq is pending",
					__func__);
				rd->pending_urgent_rq->cmd_flags &= ~REQ_URGENT;
			}
			rd->pending_urgent_rq = rq;
		}
	}
	return 0;
}

static void row_completed_req(struct request_queue *q, struct request *rq)
{
	struct row_data *rd = q->elevator->elevator_data;

	 if (rq->cmd_flags & REQ_URGENT) {
		if (!rd->urgent_in_flight) {
			WARN_ON(1);
			pr_err("%s(): URGENT req but urgent_in_flight = F",
				__func__);
		}
		rd->urgent_in_flight = false;
		rq->cmd_flags &= ~REQ_URGENT;
	}
	row_log(q, "completed %s %s req.",
		(rq->cmd_flags & REQ_URGENT ? "URGENT" : "regular"),
		(rq_data_dir(rq) == READ ? "READ" : "WRITE"));
}

static bool row_urgent_pending(struct request_queue *q)
{
	struct row_data *rd = q->elevator->elevator_data;

	if (rd->urgent_in_flight) {
		row_log(rd->dispatch_queue, "%d urgent requests in flight",
			rd->urgent_in_flight);
		return false;
	}

	if (rd->pending_urgent_rq) {
		row_log(rd->dispatch_queue, "Urgent request pending");
		return true;
	}

	row_log(rd->dispatch_queue, "no urgent request pending/in flight");
	return false;
}

static void row_remove_request(struct row_data *rd,
			       struct request *rq)
{
	struct row_queue *rqueue = RQ_ROWQ(rq);

	list_del_init(&(rq)->queuelist);
	if (rd->pending_urgent_rq == rq)
		rd->pending_urgent_rq = NULL;
	else
		BUG_ON(rq->cmd_flags & REQ_URGENT);
	rqueue->nr_req--;
	rd->nr_reqs[rq_data_dir(rq)]--;
}

static void row_dispatch_insert(struct row_data *rd, struct request *rq)
{
	struct row_queue *rqueue = RQ_ROWQ(rq);

	row_remove_request(rd, rq);
	elv_dispatch_sort(rd->dispatch_queue, rq);
	if (rq->cmd_flags & REQ_URGENT) {
		WARN_ON(rd->urgent_in_flight);
		rd->urgent_in_flight = true;
	}
	rqueue->nr_dispatched++;
	rqueue->dispatch_cnt++;
	row_clear_rowq_unserved(rd, rqueue->prio);
	row_log_rowq(rd, rqueue->prio,
		" Dispatched request %p nr_disp = %d", rq,
		rqueue->nr_dispatched);
	if (rqueue->prio < ROWQ_REG_PRIO_IDX) {
		rd->last_served_ioprio_class = IOPRIO_CLASS_RT;
		if (row_regular_req_pending(rd))
			rd->reg_prio_starvation.starvation_counter++;
		if (row_low_req_pending(rd))
			rd->low_prio_starvation.starvation_counter++;
	} else if (rqueue->prio < ROWQ_LOW_PRIO_IDX) {
		rd->last_served_ioprio_class = IOPRIO_CLASS_BE;
		rd->reg_prio_starvation.starvation_counter = 0;
		if (row_low_req_pending(rd))
			rd->low_prio_starvation.starvation_counter++;
	} else {
		rd->last_served_ioprio_class = IOPRIO_CLASS_IDLE;
		rd->low_prio_starvation.starvation_counter = 0;
	}
}

static int row_get_ioprio_class_to_serve(struct row_data *rd, int force)
{
	int i;
	int ret = IOPRIO_CLASS_NONE;

	if (!rd->nr_reqs[READ] && !rd->nr_reqs[WRITE]) {
		row_log(rd->dispatch_queue, "No more requests in scheduler");
		goto check_idling;
	}

	
	for (i = 0; i < ROWQ_REG_PRIO_IDX; i++) {
		if (!list_empty(&rd->row_queues[i].fifo)) {
			if (hrtimer_active(&rd->rd_idle_data.hr_timer)) {
				if (hrtimer_try_to_cancel(
					&rd->rd_idle_data.hr_timer) >= 0) {
					row_log(rd->dispatch_queue,
					"Canceling delayed work on %d. RT pending",
					     rd->rd_idle_data.idling_queue_idx);
					rd->rd_idle_data.idling_queue_idx =
						ROWQ_MAX_PRIO;
				}
			}

			if (row_regular_req_pending(rd) &&
			    (rd->reg_prio_starvation.starvation_counter >=
			     rd->reg_prio_starvation.starvation_limit))
				ret = IOPRIO_CLASS_BE;
			else if (row_low_req_pending(rd) &&
			    (rd->low_prio_starvation.starvation_counter >=
			     rd->low_prio_starvation.starvation_limit))
				ret = IOPRIO_CLASS_IDLE;
			else
				ret = IOPRIO_CLASS_RT;

			goto done;
		}
	}

	if (hrtimer_active(&rd->rd_idle_data.hr_timer)) {
		row_log(rd->dispatch_queue, "Delayed work pending. Exiting");
		goto done;
	}
check_idling:
	
	for (i = 0; i < ROWQ_REG_PRIO_IDX && !force; i++) {
		if (rd->row_queues[i].idle_data.begin_idling &&
		    row_queues_def[i].idling_enabled)
			goto initiate_idling;
	}

	
	for (i = ROWQ_REG_PRIO_IDX; i < ROWQ_LOW_PRIO_IDX; i++) {
		if (list_empty(&rd->row_queues[i].fifo)) {
			
			if (rd->row_queues[i].idle_data.begin_idling &&
			    !force && row_queues_def[i].idling_enabled)
				goto initiate_idling;
		} else {
			if (row_low_req_pending(rd) &&
			    (rd->low_prio_starvation.starvation_counter >=
			     rd->low_prio_starvation.starvation_limit))
				ret = IOPRIO_CLASS_IDLE;
			else
				ret = IOPRIO_CLASS_BE;
			goto done;
		}
	}

	if (rd->nr_reqs[READ] || rd->nr_reqs[WRITE])
		ret = IOPRIO_CLASS_IDLE;
	goto done;

initiate_idling:
	hrtimer_start(&rd->rd_idle_data.hr_timer,
		ktime_set(0, rd->rd_idle_data.idle_time_ms * NSEC_PER_MSEC),
		HRTIMER_MODE_REL);

	rd->rd_idle_data.idling_queue_idx = i;
	row_log_rowq(rd, i, "Scheduled delayed work on %d. exiting", i);

done:
	return ret;
}

static void row_restart_cycle(struct row_data *rd,
				int start_idx, int end_idx)
{
	int i;

	row_dump_queues_stat(rd);
	for (i = start_idx; i < end_idx; i++) {
		if (rd->row_queues[i].nr_dispatched <
		    rd->row_queues[i].disp_quantum)
			row_mark_rowq_unserved(rd, i);
		rd->row_queues[i].nr_dispatched = 0;
	}
	row_log(rd->dispatch_queue, "Restarting cycle for class @ %d-%d",
		start_idx, end_idx);
}

static int row_get_next_queue(struct request_queue *q, struct row_data *rd,
				int start_idx, int end_idx)
{
	int i = start_idx;
	bool restart = true;
	int ret = -EIO;
	bool print_debug_log = false;

	do {
		if (list_empty(&rd->row_queues[i].fifo) ||
		    rd->row_queues[i].nr_dispatched >=
		    rd->row_queues[i].disp_quantum) {
			if ((i == ROWQ_PRIO_HIGH_READ || i == ROWQ_PRIO_REG_READ)
			    && rd->row_queues[i].nr_dispatched >=
			    rd->row_queues[i].disp_quantum)
				print_debug_log = true;

			i++;
			if (i == end_idx && restart) {
				
				row_restart_cycle(rd, start_idx, end_idx);
				i = start_idx;
				restart = false;
			}
		} else {
			ret = i;
			break;
		}
	} while (i < end_idx);

	if (print_debug_log)
		row_dump_reg_and_low_stat(rd);

#define EXPIRE_REQUEST_THRESHOLD       20

	if (ret == ROWQ_PRIO_REG_READ) {
		struct request *check_req;
		bool reset_quantum = false;

		for (i = ret + 1; i < end_idx; i++) {
			if (!row_queues_def[i].expire)
				continue;

			if (list_empty(&rd->row_queues[i].fifo)) {
				reset_quantum = true;
				continue;
			}

			check_req = list_entry_rq(rd->row_queues[i].fifo.next);

			if (time_after(jiffies,
			    ((unsigned long) (check_req)->csd.list.next)
			    + row_queues_def[i].expire) &&
			    rd->row_queues[i].nr_req > EXPIRE_REQUEST_THRESHOLD) {
				rd->row_queues[ret].disp_quantum =
					row_queues_def[ret].quantum / 2;
				reset_quantum = false;
				break;
			} else
				reset_quantum = true;
		}

		if (reset_quantum)
			rd->row_queues[ret].disp_quantum =
				row_queues_def[ret].quantum;
	}


	return ret;
}

static int row_dispatch_requests(struct request_queue *q, int force)
{
	struct row_data *rd = (struct row_data *)q->elevator->elevator_data;
	int ret = 0, currq, ioprio_class_to_serve, start_idx, end_idx;

	if (force && hrtimer_active(&rd->rd_idle_data.hr_timer)) {
		if (hrtimer_try_to_cancel(&rd->rd_idle_data.hr_timer) >= 0) {
			row_log(rd->dispatch_queue,
				"Canceled delayed work on %d - forced dispatch",
				rd->rd_idle_data.idling_queue_idx);
			rd->rd_idle_data.idling_queue_idx = ROWQ_MAX_PRIO;
		}
	}

	if (rd->pending_urgent_rq) {
		row_log(rd->dispatch_queue, "dispatching urgent request");
		row_dispatch_insert(rd, rd->pending_urgent_rq);
		ret = 1;
		goto done;
	}

	ioprio_class_to_serve = row_get_ioprio_class_to_serve(rd, force);
	row_log(rd->dispatch_queue, "Dispatching from %d priority class",
		ioprio_class_to_serve);

	switch (ioprio_class_to_serve) {
	case IOPRIO_CLASS_NONE:
		rd->last_served_ioprio_class = IOPRIO_CLASS_NONE;
		goto done;
	case IOPRIO_CLASS_RT:
		start_idx = ROWQ_HIGH_PRIO_IDX;
		end_idx = ROWQ_REG_PRIO_IDX;
		break;
	case IOPRIO_CLASS_BE:
		start_idx = ROWQ_REG_PRIO_IDX;
		end_idx = ROWQ_LOW_PRIO_IDX;
		break;
	case IOPRIO_CLASS_IDLE:
		start_idx = ROWQ_LOW_PRIO_IDX;
		end_idx = ROWQ_MAX_PRIO;
		break;
	default:
		pr_err("%s(): Invalid I/O priority class", __func__);
		goto done;
	}

	currq = row_get_next_queue(q, rd, start_idx, end_idx);

	
	if (currq >= 0) {
		row_dispatch_insert(rd,
			rq_entry_fifo(rd->row_queues[currq].fifo.next));
		ret = 1;
	}
done:
	return ret;
}

static void *row_init_queue(struct request_queue *q)
{

	struct row_data *rdata;
	int i;

	rdata = kmalloc_node(sizeof(*rdata),
			     GFP_KERNEL | __GFP_ZERO, q->node);
	if (!rdata)
		return NULL;

	memset(rdata, 0, sizeof(*rdata));
	for (i = 0; i < ROWQ_MAX_PRIO; i++) {
		INIT_LIST_HEAD(&rdata->row_queues[i].fifo);
		rdata->row_queues[i].disp_quantum = row_queues_def[i].quantum;
		rdata->row_queues[i].rdata = rdata;
		rdata->row_queues[i].prio = i;
		rdata->row_queues[i].idle_data.begin_idling = false;
		rdata->row_queues[i].idle_data.last_insert_time =
			ktime_set(0, 0);
	}

	rdata->reg_prio_starvation.starvation_limit =
			ROW_REG_STARVATION_TOLLERANCE;
	rdata->low_prio_starvation.starvation_limit =
			ROW_LOW_STARVATION_TOLLERANCE;
	rdata->rd_idle_data.idle_time_ms = ROW_IDLE_TIME_MSEC;
	rdata->rd_idle_data.freq_ms = ROW_READ_FREQ_MSEC;
	hrtimer_init(&rdata->rd_idle_data.hr_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rdata->rd_idle_data.hr_timer.function = &row_idle_hrtimer_fn;

	INIT_WORK(&rdata->rd_idle_data.idle_work, kick_queue);
	rdata->last_served_ioprio_class = IOPRIO_CLASS_NONE;
	rdata->rd_idle_data.idling_queue_idx = ROWQ_MAX_PRIO;
	rdata->dispatch_queue = q;
	rdata->last_update_jiffies = jiffies;

	return rdata;
}

static void row_exit_queue(struct elevator_queue *e)
{
	struct row_data *rd = (struct row_data *)e->elevator_data;
	int i;

	for (i = 0; i < ROWQ_MAX_PRIO; i++)
		BUG_ON(!list_empty(&rd->row_queues[i].fifo));
	if (hrtimer_cancel(&rd->rd_idle_data.hr_timer))
		pr_err("%s(): idle timer was active!", __func__);
	rd->rd_idle_data.idling_queue_idx = ROWQ_MAX_PRIO;
	kfree(rd);
}

static void row_merged_requests(struct request_queue *q, struct request *rq,
				 struct request *next)
{
	struct row_queue   *rqueue = RQ_ROWQ(next);

	list_del_init(&next->queuelist);
	rqueue->nr_req--;
	if (rqueue->rdata->pending_urgent_rq == next) {
		pr_err("\n\nROW_WARNING: merging pending urgent!");
		rqueue->rdata->pending_urgent_rq = rq;
		rq->cmd_flags |= REQ_URGENT;
		WARN_ON(!(next->cmd_flags & REQ_URGENT));
		next->cmd_flags &= ~REQ_URGENT;
	}
	rqueue->rdata->nr_reqs[rq_data_dir(rq)]--;
}

static enum row_queue_prio row_get_queue_prio(struct request *rq,
				struct row_data *rd)
{
	const int data_dir = rq_data_dir(rq);
	const bool is_sync = rq_is_sync(rq);
	enum row_queue_prio q_type = ROWQ_MAX_PRIO;
	int ioprio_class = IOPRIO_PRIO_CLASS(rq->elv.icq->ioc->ioprio);

	switch (ioprio_class) {
	case IOPRIO_CLASS_RT:
		if (data_dir == READ)
			q_type = ROWQ_PRIO_HIGH_READ;
		else if (is_sync)
			q_type = ROWQ_PRIO_HIGH_SWRITE;
		else {
			pr_err("%s:%s(): got a simple write from RT_CLASS. How???",
				rq->rq_disk->disk_name, __func__);
			q_type = ROWQ_PRIO_REG_WRITE;
		}
		break;
	case IOPRIO_CLASS_IDLE:
		if (data_dir == READ)
			q_type = ROWQ_PRIO_LOW_READ;
		else if (is_sync)
			q_type = ROWQ_PRIO_LOW_SWRITE;
		else {
			pr_err("%s:%s(): got a simple write from IDLE_CLASS. How???",
				rq->rq_disk->disk_name, __func__);
			q_type = ROWQ_PRIO_REG_WRITE;
		}
		break;
	case IOPRIO_CLASS_NONE:
	case IOPRIO_CLASS_BE:
	default:
		if (data_dir == READ)
			q_type = ROWQ_PRIO_REG_READ;
		else if (is_sync)
			q_type = ROWQ_PRIO_REG_SWRITE;
		else
			q_type = ROWQ_PRIO_REG_WRITE;
		break;
	}

	return q_type;
}

static int
row_set_request(struct request_queue *q, struct request *rq, gfp_t gfp_mask)
{
	struct row_data *rd = (struct row_data *)q->elevator->elevator_data;
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	rq->elv.priv[0] =
		(void *)(&rd->row_queues[row_get_queue_prio(rq, rd)]);
	spin_unlock_irqrestore(q->queue_lock, flags);

	return 0;
}

static ssize_t row_var_show(int var, char *page)
{
	return snprintf(page, 100, "%d\n", var);
}

static ssize_t row_var_store(int *var, const char *page, size_t count)
{
	int err;
	err = kstrtoul(page, 10, (unsigned long *)var);

	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct row_data *rowd = e->elevator_data;			\
	int __data = __VAR;						\
	return row_var_show(__data, (page));			\
}
SHOW_FUNCTION(row_hp_read_quantum_show,
	rowd->row_queues[ROWQ_PRIO_HIGH_READ].disp_quantum);
SHOW_FUNCTION(row_rp_read_quantum_show,
	rowd->row_queues[ROWQ_PRIO_REG_READ].disp_quantum);
SHOW_FUNCTION(row_hp_swrite_quantum_show,
	rowd->row_queues[ROWQ_PRIO_HIGH_SWRITE].disp_quantum);
SHOW_FUNCTION(row_rp_swrite_quantum_show,
	rowd->row_queues[ROWQ_PRIO_REG_SWRITE].disp_quantum);
SHOW_FUNCTION(row_rp_write_quantum_show,
	rowd->row_queues[ROWQ_PRIO_REG_WRITE].disp_quantum);
SHOW_FUNCTION(row_lp_read_quantum_show,
	rowd->row_queues[ROWQ_PRIO_LOW_READ].disp_quantum);
SHOW_FUNCTION(row_lp_swrite_quantum_show,
	rowd->row_queues[ROWQ_PRIO_LOW_SWRITE].disp_quantum);
SHOW_FUNCTION(row_rd_idle_data_show, rowd->rd_idle_data.idle_time_ms);
SHOW_FUNCTION(row_rd_idle_data_freq_show, rowd->rd_idle_data.freq_ms);
SHOW_FUNCTION(row_reg_starv_limit_show,
	rowd->reg_prio_starvation.starvation_limit);
SHOW_FUNCTION(row_low_starv_limit_show,
	rowd->low_prio_starvation.starvation_limit);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX)			\
static ssize_t __FUNC(struct elevator_queue *e,				\
		const char *page, size_t count)				\
{									\
	struct row_data *rowd = e->elevator_data;			\
	int __data;						\
	int ret = row_var_store(&__data, (page), count);		\
	if (__data < (MIN))						\
		__data = (MIN);						\
	else if (__data > (MAX))					\
		__data = (MAX);						\
	*(__PTR) = __data;						\
	return ret;							\
}
STORE_FUNCTION(row_hp_read_quantum_store,
&rowd->row_queues[ROWQ_PRIO_HIGH_READ].disp_quantum, 1, INT_MAX);
STORE_FUNCTION(row_rp_read_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_REG_READ].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_hp_swrite_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_HIGH_SWRITE].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_rp_swrite_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_REG_SWRITE].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_rp_write_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_REG_WRITE].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_lp_read_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_LOW_READ].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_lp_swrite_quantum_store,
			&rowd->row_queues[ROWQ_PRIO_LOW_SWRITE].disp_quantum,
			1, INT_MAX);
STORE_FUNCTION(row_rd_idle_data_store, &rowd->rd_idle_data.idle_time_ms,
			1, INT_MAX);
STORE_FUNCTION(row_rd_idle_data_freq_store, &rowd->rd_idle_data.freq_ms,
			1, INT_MAX);
STORE_FUNCTION(row_reg_starv_limit_store,
			&rowd->reg_prio_starvation.starvation_limit,
			1, INT_MAX);
STORE_FUNCTION(row_low_starv_limit_store,
			&rowd->low_prio_starvation.starvation_limit,
			1, INT_MAX);

#undef STORE_FUNCTION

#define ROW_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, row_##name##_show, \
				      row_##name##_store)

static struct elv_fs_entry row_attrs[] = {
	ROW_ATTR(hp_read_quantum),
	ROW_ATTR(rp_read_quantum),
	ROW_ATTR(hp_swrite_quantum),
	ROW_ATTR(rp_swrite_quantum),
	ROW_ATTR(rp_write_quantum),
	ROW_ATTR(lp_read_quantum),
	ROW_ATTR(lp_swrite_quantum),
	ROW_ATTR(rd_idle_data),
	ROW_ATTR(rd_idle_data_freq),
	ROW_ATTR(reg_starv_limit),
	ROW_ATTR(low_starv_limit),
	__ATTR_NULL
};

static struct elevator_type iosched_row = {
	.ops = {
		.elevator_merge_req_fn		= row_merged_requests,
		.elevator_dispatch_fn		= row_dispatch_requests,
		.elevator_add_req_fn		= row_add_request,
		.elevator_reinsert_req_fn	= row_reinsert_req,
		.elevator_is_urgent_fn		= row_urgent_pending,
		.elevator_completed_req_fn	= row_completed_req,
		.elevator_former_req_fn		= elv_rb_former_request,
		.elevator_latter_req_fn		= elv_rb_latter_request,
		.elevator_set_req_fn		= row_set_request,
		.elevator_init_fn		= row_init_queue,
		.elevator_exit_fn		= row_exit_queue,
	},
	.icq_size = sizeof(struct io_cq),
	.icq_align = __alignof__(struct io_cq),
	.elevator_attrs = row_attrs,
	.elevator_name = "row",
	.elevator_owner = THIS_MODULE,
};

static int __init row_init(void)
{
	elv_register(&iosched_row);
	return 0;
}

static void __exit row_exit(void)
{
	elv_unregister(&iosched_row);
}

module_init(row_init);
module_exit(row_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Read Over Write IO scheduler");

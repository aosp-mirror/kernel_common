/*
 * Copyright (C) 1991, 1992 Linus Torvalds
 * Copyright (C) 1994,      Karl Keyte: Added support for disk statistics
 * Elevator latency, (C) 2000  Andrea Arcangeli <andrea@suse.de> SuSE
 * Queue request tables / lock, selectable elevator, Jens Axboe <axboe@suse.de>
 * kernel-doc documentation started by NeilBrown <neilb@cse.unsw.edu.au>
 *	-  July2000
 * bio rewrite, highmem i/o, etc, Jens Axboe <axboe@suse.de> - may 2001
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backing-dev.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/kernel_stat.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/writeback.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/fault-inject.h>
#include <linux/list_sort.h>
#include <linux/delay.h>
#include <linux/ratelimit.h>
#include <linux/pm_runtime.h>

#define CREATE_TRACE_POINTS
#include <trace/events/block.h>

#include "blk.h"

EXPORT_TRACEPOINT_SYMBOL_GPL(block_bio_remap);
EXPORT_TRACEPOINT_SYMBOL_GPL(block_rq_remap);
EXPORT_TRACEPOINT_SYMBOL_GPL(block_bio_complete);

DEFINE_IDA(blk_queue_ida);

static struct kmem_cache *request_cachep;

struct kmem_cache *blk_requestq_cachep;

static struct workqueue_struct *kblockd_workqueue;

extern atomic_t emmc_reboot;

static void drive_stat_acct(struct request *rq, int new_io)
{
	struct hd_struct *part;
	int rw = rq_data_dir(rq);
	int cpu;

	if (!blk_do_io_stat(rq))
		return;

	cpu = part_stat_lock();

	if (!new_io) {
		part = rq->part;
		part_stat_inc(cpu, part, merges[rw]);
	} else {
		part = disk_map_sector_rcu(rq->rq_disk, blk_rq_pos(rq));
		if (!hd_struct_try_get(part)) {
			part = &rq->rq_disk->part0;
			hd_struct_get(part);
		}
		part_round_stats(cpu, part);
		part_inc_in_flight(part, rw);
		rq->part = part;
	}

	part_stat_unlock();
}

void blk_queue_congestion_threshold(struct request_queue *q)
{
	int nr;

	nr = q->nr_requests - (q->nr_requests / 8) + 1;
	if (nr > q->nr_requests)
		nr = q->nr_requests;
	q->nr_congestion_on = nr;

	nr = q->nr_requests - (q->nr_requests / 8) - (q->nr_requests / 16) - 1;
	if (nr < 1)
		nr = 1;
	q->nr_congestion_off = nr;
}

struct backing_dev_info *blk_get_backing_dev_info(struct block_device *bdev)
{
	struct backing_dev_info *ret = NULL;
	struct request_queue *q = bdev_get_queue(bdev);

	if (q)
		ret = &q->backing_dev_info;
	return ret;
}
EXPORT_SYMBOL(blk_get_backing_dev_info);

void blk_rq_init(struct request_queue *q, struct request *rq)
{
	memset(rq, 0, sizeof(*rq));

	INIT_LIST_HEAD(&rq->queuelist);
	INIT_LIST_HEAD(&rq->timeout_list);
	rq->cpu = -1;
	rq->q = q;
	rq->__sector = (sector_t) -1;
	INIT_HLIST_NODE(&rq->hash);
	RB_CLEAR_NODE(&rq->rb_node);
	rq->cmd = rq->__cmd;
	rq->cmd_len = BLK_MAX_CDB;
	rq->tag = -1;
	rq->ref_count = 1;
	rq->start_time = jiffies;
	set_start_time_ns(rq);
	rq->part = NULL;
}
EXPORT_SYMBOL(blk_rq_init);

static void req_bio_endio(struct request *rq, struct bio *bio,
			  unsigned int nbytes, int error)
{
	if (error)
		clear_bit(BIO_UPTODATE, &bio->bi_flags);
	else if (!test_bit(BIO_UPTODATE, &bio->bi_flags))
		error = -EIO;

	if (unlikely(nbytes > bio->bi_size)) {
		printk(KERN_ERR "%s: want %u bytes done, %u left\n",
		       __func__, nbytes, bio->bi_size);
		nbytes = bio->bi_size;
	}

	if (unlikely(rq->cmd_flags & REQ_QUIET))
		set_bit(BIO_QUIET, &bio->bi_flags);

	bio->bi_size -= nbytes;
	bio->bi_sector += (nbytes >> 9);

	if (bio_integrity(bio))
		bio_integrity_advance(bio, nbytes);

	
	if (bio->bi_size == 0 && !(rq->cmd_flags & REQ_FLUSH_SEQ))
		bio_endio(bio, error);
}

void blk_dump_rq_flags(struct request *rq, char *msg)
{
	int bit;

	printk(KERN_INFO "%s: dev %s: type=%x, flags=%x\n", msg,
		rq->rq_disk ? rq->rq_disk->disk_name : "?", rq->cmd_type,
		rq->cmd_flags);

	printk(KERN_INFO "  sector %llu, nr/cnr %u/%u\n",
	       (unsigned long long)blk_rq_pos(rq),
	       blk_rq_sectors(rq), blk_rq_cur_sectors(rq));
	printk(KERN_INFO "  bio %p, biotail %p, buffer %p, len %u\n",
	       rq->bio, rq->biotail, rq->buffer, blk_rq_bytes(rq));

	if (rq->cmd_type == REQ_TYPE_BLOCK_PC) {
		printk(KERN_INFO "  cdb: ");
		for (bit = 0; bit < BLK_MAX_CDB; bit++)
			printk("%02x ", rq->cmd[bit]);
		printk("\n");
	}
}
EXPORT_SYMBOL(blk_dump_rq_flags);

static void blk_delay_work(struct work_struct *work)
{
	struct request_queue *q;

	q = container_of(work, struct request_queue, delay_work.work);
	spin_lock_irq(q->queue_lock);
	__blk_run_queue(q);
	spin_unlock_irq(q->queue_lock);
}

void blk_delay_queue(struct request_queue *q, unsigned long msecs)
{
	queue_delayed_work(kblockd_workqueue, &q->delay_work,
				msecs_to_jiffies(msecs));
}
EXPORT_SYMBOL(blk_delay_queue);

void blk_start_queue(struct request_queue *q)
{
	WARN_ON(!irqs_disabled());

	queue_flag_clear(QUEUE_FLAG_STOPPED, q);
	__blk_run_queue(q);
}
EXPORT_SYMBOL(blk_start_queue);

void blk_stop_queue(struct request_queue *q)
{
	__cancel_delayed_work(&q->delay_work);
	queue_flag_set(QUEUE_FLAG_STOPPED, q);
}
EXPORT_SYMBOL(blk_stop_queue);

void blk_sync_queue(struct request_queue *q)
{
	del_timer_sync(&q->timeout);
	cancel_delayed_work_sync(&q->delay_work);
}
EXPORT_SYMBOL(blk_sync_queue);

void __blk_run_queue(struct request_queue *q)
{
	if (unlikely(blk_queue_stopped(q)))
		return;

	if (!q->notified_urgent &&
		q->elevator->type->ops.elevator_is_urgent_fn &&
		q->urgent_request_fn &&
		q->elevator->type->ops.elevator_is_urgent_fn(q) &&
		list_empty(&q->flush_data_in_flight)) {
		q->notified_urgent = true;
		q->urgent_request_fn(q);
	} else
		q->request_fn(q);
}
EXPORT_SYMBOL(__blk_run_queue);

void blk_run_queue_async(struct request_queue *q)
{
	if (likely(!blk_queue_stopped(q))) {
		__cancel_delayed_work(&q->delay_work);
		queue_delayed_work(kblockd_workqueue, &q->delay_work, 0);
	}
}
EXPORT_SYMBOL(blk_run_queue_async);

void blk_run_queue(struct request_queue *q)
{
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	__blk_run_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_run_queue);

void blk_put_queue(struct request_queue *q)
{
	kobject_put(&q->kobj);
}
EXPORT_SYMBOL(blk_put_queue);

void blk_drain_queue(struct request_queue *q, bool drain_all)
{
	while (true) {
		bool drain = false;
		int i;

		spin_lock_irq(q->queue_lock);

		elv_drain_elevator(q);
		if (drain_all)
			blk_throtl_drain(q);

		if (!list_empty(&q->queue_head))
			__blk_run_queue(q);

		drain |= q->rq.elvpriv;

		if (drain_all) {
			drain |= !list_empty(&q->queue_head);
			for (i = 0; i < 2; i++) {
				drain |= q->rq.count[i];
				drain |= q->in_flight[i];
				drain |= !list_empty(&q->flush_queue[i]);
			}
		}

		spin_unlock_irq(q->queue_lock);

		if (!drain)
			break;
		msleep(10);
	}
}

void blk_cleanup_queue(struct request_queue *q)
{
	spinlock_t *lock = q->queue_lock;

	
	mutex_lock(&q->sysfs_lock);
	queue_flag_set_unlocked(QUEUE_FLAG_DEAD, q);

	spin_lock_irq(lock);
	queue_flag_set(QUEUE_FLAG_NOMERGES, q);
	queue_flag_set(QUEUE_FLAG_NOXMERGES, q);
	queue_flag_set(QUEUE_FLAG_DEAD, q);

	if (q->queue_lock != &q->__queue_lock)
		q->queue_lock = &q->__queue_lock;

	spin_unlock_irq(lock);
	mutex_unlock(&q->sysfs_lock);

	if (q->elevator)
		blk_drain_queue(q, true);

	
	del_timer_sync(&q->backing_dev_info.laptop_mode_wb_timer);
	blk_sync_queue(q);

	
	blk_put_queue(q);
}
EXPORT_SYMBOL(blk_cleanup_queue);

static int blk_init_free_list(struct request_queue *q)
{
	struct request_list *rl = &q->rq;

	if (unlikely(rl->rq_pool))
		return 0;

	rl->count[BLK_RW_SYNC] = rl->count[BLK_RW_ASYNC] = 0;
	rl->starved[BLK_RW_SYNC] = rl->starved[BLK_RW_ASYNC] = 0;
	rl->elvpriv = 0;
	init_waitqueue_head(&rl->wait[BLK_RW_SYNC]);
	init_waitqueue_head(&rl->wait[BLK_RW_ASYNC]);

	rl->rq_pool = mempool_create_node(BLKDEV_MIN_RQ, mempool_alloc_slab,
				mempool_free_slab, request_cachep, q->node);

	if (!rl->rq_pool)
		return -ENOMEM;

	return 0;
}

struct request_queue *blk_alloc_queue(gfp_t gfp_mask)
{
	return blk_alloc_queue_node(gfp_mask, -1);
}
EXPORT_SYMBOL(blk_alloc_queue);

struct request_queue *blk_alloc_queue_node(gfp_t gfp_mask, int node_id)
{
	struct request_queue *q;
	int err;

	q = kmem_cache_alloc_node(blk_requestq_cachep,
				gfp_mask | __GFP_ZERO, node_id);
	if (!q)
		return NULL;

	q->id = ida_simple_get(&blk_queue_ida, 0, 0, gfp_mask);
	if (q->id < 0)
		goto fail_q;

	q->backing_dev_info.ra_pages =
			(VM_MAX_READAHEAD * 1024) / PAGE_CACHE_SIZE;
	q->backing_dev_info.state = 0;
	q->backing_dev_info.capabilities = BDI_CAP_MAP_COPY;
	q->backing_dev_info.name = "block";
	q->node = node_id;

	err = bdi_init(&q->backing_dev_info);
	if (err)
		goto fail_id;

	if (blk_throtl_init(q))
		goto fail_id;

	setup_timer(&q->backing_dev_info.laptop_mode_wb_timer,
		    laptop_mode_timer_fn, (unsigned long) q);
	setup_timer(&q->timeout, blk_rq_timed_out_timer, (unsigned long) q);
	INIT_LIST_HEAD(&q->timeout_list);
	INIT_LIST_HEAD(&q->icq_list);
	INIT_LIST_HEAD(&q->flush_queue[0]);
	INIT_LIST_HEAD(&q->flush_queue[1]);
	INIT_LIST_HEAD(&q->flush_data_in_flight);
	INIT_DELAYED_WORK(&q->delay_work, blk_delay_work);

	kobject_init(&q->kobj, &blk_queue_ktype);

	mutex_init(&q->sysfs_lock);
	spin_lock_init(&q->__queue_lock);

	q->queue_lock = &q->__queue_lock;

	return q;

fail_id:
	ida_simple_remove(&blk_queue_ida, q->id);
fail_q:
	kmem_cache_free(blk_requestq_cachep, q);
	return NULL;
}
EXPORT_SYMBOL(blk_alloc_queue_node);


struct request_queue *blk_init_queue(request_fn_proc *rfn, spinlock_t *lock)
{
	return blk_init_queue_node(rfn, lock, -1);
}
EXPORT_SYMBOL(blk_init_queue);

struct request_queue *
blk_init_queue_node(request_fn_proc *rfn, spinlock_t *lock, int node_id)
{
	struct request_queue *uninit_q, *q;

	uninit_q = blk_alloc_queue_node(GFP_KERNEL, node_id);
	if (!uninit_q)
		return NULL;

	q = blk_init_allocated_queue(uninit_q, rfn, lock);
	if (!q)
		blk_cleanup_queue(uninit_q);

	return q;
}
EXPORT_SYMBOL(blk_init_queue_node);

struct request_queue *
blk_init_allocated_queue(struct request_queue *q, request_fn_proc *rfn,
			 spinlock_t *lock)
{
	if (!q)
		return NULL;

	if (blk_init_free_list(q))
		return NULL;

	q->request_fn		= rfn;
	q->prep_rq_fn		= NULL;
	q->unprep_rq_fn		= NULL;
	q->queue_flags		= QUEUE_FLAG_DEFAULT;

	
	if (lock)
		q->queue_lock		= lock;

	blk_queue_make_request(q, blk_queue_bio);

	q->sg_reserved_size = INT_MAX;

	if (!elevator_init(q, NULL)) {
		blk_queue_congestion_threshold(q);
		return q;
	}

	return NULL;
}
EXPORT_SYMBOL(blk_init_allocated_queue);

bool blk_get_queue(struct request_queue *q)
{
	if (likely(!blk_queue_dead(q))) {
		__blk_get_queue(q);
		return true;
	}

	return false;
}
EXPORT_SYMBOL(blk_get_queue);

static inline void blk_free_request(struct request_queue *q, struct request *rq)
{
	if (rq->cmd_flags & REQ_ELVPRIV) {
		elv_put_request(q, rq);
		if (rq->elv.icq)
			put_io_context(rq->elv.icq->ioc);
	}

	mempool_free(rq, q->rq.rq_pool);
}

static struct request *
blk_alloc_request(struct request_queue *q, struct io_cq *icq,
		  unsigned int flags, gfp_t gfp_mask)
{
	struct request *rq = mempool_alloc(q->rq.rq_pool, gfp_mask);

	if (!rq)
		return NULL;

	blk_rq_init(q, rq);

	rq->cmd_flags = flags | REQ_ALLOCED;

	if (flags & REQ_ELVPRIV) {
		rq->elv.icq = icq;
		if (unlikely(elv_set_request(q, rq, gfp_mask))) {
			mempool_free(rq, q->rq.rq_pool);
			return NULL;
		}
		
		if (icq)
			get_io_context(icq->ioc);
	}

	return rq;
}

static inline int ioc_batching(struct request_queue *q, struct io_context *ioc)
{
	if (!ioc)
		return 0;

	return ioc->nr_batch_requests == q->nr_batching ||
		(ioc->nr_batch_requests > 0
		&& time_before(jiffies, ioc->last_waited + BLK_BATCH_TIME));
}

static void ioc_set_batching(struct request_queue *q, struct io_context *ioc)
{
	if (!ioc || ioc_batching(q, ioc))
		return;

	ioc->nr_batch_requests = q->nr_batching;
	ioc->last_waited = jiffies;
}

static void __freed_request(struct request_queue *q, int sync)
{
	struct request_list *rl = &q->rq;

	if (rl->count[sync] < queue_congestion_off_threshold(q))
		blk_clear_queue_congested(q, sync);

	if (rl->count[sync] + 1 <= q->nr_requests) {
		if (waitqueue_active(&rl->wait[sync]))
			wake_up(&rl->wait[sync]);

		blk_clear_queue_full(q, sync);
	}
}

static void freed_request(struct request_queue *q, unsigned int flags)
{
	struct request_list *rl = &q->rq;
	int sync = rw_is_sync(flags);

	rl->count[sync]--;
	if (flags & REQ_ELVPRIV)
		rl->elvpriv--;

	__freed_request(q, sync);

	if (unlikely(rl->starved[sync ^ 1]))
		__freed_request(q, sync ^ 1);
}

static bool blk_rq_should_init_elevator(struct bio *bio)
{
	if (!bio)
		return true;

	if (bio->bi_rw & (REQ_FLUSH | REQ_FUA))
		return false;

	return true;
}

static struct request *get_request(struct request_queue *q, int rw_flags,
				   struct bio *bio, gfp_t gfp_mask)
{
	struct request *rq = NULL;
	struct request_list *rl = &q->rq;
	struct elevator_type *et;
	struct io_context *ioc;
	struct io_cq *icq = NULL;
	const bool is_sync = rw_is_sync(rw_flags) != 0;
	bool retried = false;
	int may_queue;
retry:
	et = q->elevator->type;
	ioc = current->io_context;

	if (unlikely(blk_queue_dead(q)))
		return NULL;

	may_queue = elv_may_queue(q, rw_flags);
	if (may_queue == ELV_MQUEUE_NO)
		goto rq_starved;

	if (rl->count[is_sync]+1 >= queue_congestion_on_threshold(q)) {
		if (rl->count[is_sync]+1 >= q->nr_requests) {
			if (!ioc && !retried) {
				spin_unlock_irq(q->queue_lock);
				create_io_context(current, gfp_mask, q->node);
				spin_lock_irq(q->queue_lock);
				retried = true;
				goto retry;
			}

			if (!blk_queue_full(q, is_sync)) {
				ioc_set_batching(q, ioc);
				blk_set_queue_full(q, is_sync);
			} else {
				if (may_queue != ELV_MQUEUE_MUST
						&& !ioc_batching(q, ioc)) {
					goto out;
				}
			}
		}
		blk_set_queue_congested(q, is_sync);
	}

	if (rl->count[is_sync] >= (3 * q->nr_requests / 2))
		goto out;

	rl->count[is_sync]++;
	rl->starved[is_sync] = 0;

	if (blk_rq_should_init_elevator(bio) &&
	    !test_bit(QUEUE_FLAG_ELVSWITCH, &q->queue_flags)) {
		rw_flags |= REQ_ELVPRIV;
		rl->elvpriv++;
		if (et->icq_cache && ioc)
			icq = ioc_lookup_icq(ioc, q);
	}

	if (blk_queue_io_stat(q))
		rw_flags |= REQ_IO_STAT;
	spin_unlock_irq(q->queue_lock);

	
	if ((rw_flags & REQ_ELVPRIV) && unlikely(et->icq_cache && !icq)) {
		icq = ioc_create_icq(q, gfp_mask);
		if (!icq)
			goto fail_icq;
	}

	rq = blk_alloc_request(q, icq, rw_flags, gfp_mask);

fail_icq:
	if (unlikely(!rq)) {
		spin_lock_irq(q->queue_lock);
		freed_request(q, rw_flags);

rq_starved:
		if (unlikely(rl->count[is_sync] == 0))
			rl->starved[is_sync] = 1;

		goto out;
	}

	if (ioc_batching(q, ioc))
		ioc->nr_batch_requests--;

	trace_block_getrq(q, bio, rw_flags & 1);
out:
	return rq;
}

static struct request *get_request_wait(struct request_queue *q, int rw_flags,
					struct bio *bio)
{
	const bool is_sync = rw_is_sync(rw_flags) != 0;
	struct request *rq;

	rq = get_request(q, rw_flags, bio, GFP_NOIO);
	while (!rq) {
		DEFINE_WAIT(wait);
		struct request_list *rl = &q->rq;

		if (unlikely(blk_queue_dead(q)))
			return NULL;

		prepare_to_wait_exclusive(&rl->wait[is_sync], &wait,
				TASK_UNINTERRUPTIBLE);

		trace_block_sleeprq(q, bio, rw_flags & 1);

		spin_unlock_irq(q->queue_lock);
		io_schedule();

		create_io_context(current, GFP_NOIO, q->node);
		ioc_set_batching(q, current->io_context);

		spin_lock_irq(q->queue_lock);
		finish_wait(&rl->wait[is_sync], &wait);

		rq = get_request(q, rw_flags, bio, GFP_NOIO);
	};

	return rq;
}

struct request *blk_get_request(struct request_queue *q, int rw, gfp_t gfp_mask)
{
	struct request *rq;

	spin_lock_irq(q->queue_lock);
	if (gfp_mask & __GFP_WAIT)
		rq = get_request_wait(q, rw, NULL);
	else
		rq = get_request(q, rw, NULL, gfp_mask);
	if (!rq)
		spin_unlock_irq(q->queue_lock);
	

	return rq;
}
EXPORT_SYMBOL(blk_get_request);

struct request *blk_make_request(struct request_queue *q, struct bio *bio,
				 gfp_t gfp_mask)
{
	struct request *rq = blk_get_request(q, bio_data_dir(bio), gfp_mask);

	if (unlikely(!rq))
		return ERR_PTR(-ENOMEM);

	for_each_bio(bio) {
		struct bio *bounce_bio = bio;
		int ret;

		blk_queue_bounce(q, &bounce_bio);
		ret = blk_rq_append_bio(q, rq, bounce_bio);
		if (unlikely(ret)) {
			blk_put_request(rq);
			return ERR_PTR(ret);
		}
	}

	return rq;
}
EXPORT_SYMBOL(blk_make_request);

void blk_requeue_request(struct request_queue *q, struct request *rq)
{
	blk_delete_timer(rq);
	blk_clear_rq_complete(rq);
	trace_block_rq_requeue(q, rq);

	if (blk_rq_tagged(rq))
		blk_queue_end_tag(q, rq);

	BUG_ON(blk_queued_rq(rq));

	if (rq->cmd_flags & REQ_URGENT) {
		pr_debug("%s(): requeueing an URGENT request", __func__);
		WARN_ON(!q->dispatched_urgent);
		q->dispatched_urgent = false;
	}
	elv_requeue_request(q, rq);
}
EXPORT_SYMBOL(blk_requeue_request);

int blk_reinsert_request(struct request_queue *q, struct request *rq)
{
	if (unlikely(!rq) || unlikely(!q))
		return -EIO;

	blk_delete_timer(rq);
	blk_clear_rq_complete(rq);
	trace_block_rq_requeue(q, rq);

	if (blk_rq_tagged(rq))
		blk_queue_end_tag(q, rq);

	BUG_ON(blk_queued_rq(rq));
	if (rq->cmd_flags & REQ_URGENT) {
		pr_debug("%s(): reinserting an URGENT request", __func__);
		WARN_ON(!q->dispatched_urgent);
		q->dispatched_urgent = false;
	}

	return elv_reinsert_request(q, rq);
}
EXPORT_SYMBOL(blk_reinsert_request);

bool blk_reinsert_req_sup(struct request_queue *q)
{
	if (unlikely(!q))
		return false;
	return q->elevator->type->ops.elevator_reinsert_req_fn ? true : false;
}
EXPORT_SYMBOL(blk_reinsert_req_sup);

static void add_acct_request(struct request_queue *q, struct request *rq,
			     int where)
{
	drive_stat_acct(rq, 1);
	__elv_add_request(q, rq, where);
}

static void part_round_stats_single(int cpu, struct hd_struct *part,
				    unsigned long now)
{
	if (now == part->stamp)
		return;

	if (part_in_flight(part)) {
		__part_stat_add(cpu, part, time_in_queue,
				part_in_flight(part) * (now - part->stamp));
		__part_stat_add(cpu, part, io_ticks, (now - part->stamp));
	}
	part->stamp = now;
}

void part_round_stats(int cpu, struct hd_struct *part)
{
	unsigned long now = jiffies;

	if (part->partno)
		part_round_stats_single(cpu, &part_to_disk(part)->part0, now);
	part_round_stats_single(cpu, part, now);
}
EXPORT_SYMBOL_GPL(part_round_stats);

#ifdef CONFIG_PM_RUNTIME
static void blk_pm_put_request(struct request *rq)
{
	if (rq->q->dev && !(rq->cmd_flags & REQ_PM) && !--rq->q->nr_pending)
		pm_runtime_mark_last_busy(rq->q->dev);
}
#else
static inline void blk_pm_put_request(struct request *rq) {}
#endif

void __blk_put_request(struct request_queue *q, struct request *req)
{
	if (unlikely(!q))
		return;
	if (unlikely(--req->ref_count))
		return;

	blk_pm_put_request(req);

	elv_completed_request(q, req);

	
	WARN_ON(req->bio && !bio_flagged(req->bio, BIO_DONTFREE));


	if (req->cmd_flags & REQ_ALLOCED) {
		unsigned int flags = req->cmd_flags;

		BUG_ON(!list_empty(&req->queuelist));
		BUG_ON(!hlist_unhashed(&req->hash));

		blk_free_request(q, req);
		freed_request(q, flags);
	}
}
EXPORT_SYMBOL_GPL(__blk_put_request);

void blk_put_request(struct request *req)
{
	unsigned long flags;
	struct request_queue *q = req->q;

	spin_lock_irqsave(q->queue_lock, flags);
	__blk_put_request(q, req);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_put_request);

void blk_add_request_payload(struct request *rq, struct page *page,
		unsigned int len)
{
	struct bio *bio = rq->bio;

	bio->bi_io_vec->bv_page = page;
	bio->bi_io_vec->bv_offset = 0;
	bio->bi_io_vec->bv_len = len;

	bio->bi_size = len;
	bio->bi_vcnt = 1;
	bio->bi_phys_segments = 1;

	rq->__data_len = rq->resid_len = len;
	rq->nr_phys_segments = 1;
	rq->buffer = bio_data(bio);
}
EXPORT_SYMBOL_GPL(blk_add_request_payload);

static bool bio_attempt_back_merge(struct request_queue *q, struct request *req,
				   struct bio *bio)
{
	const int ff = bio->bi_rw & REQ_FAILFAST_MASK;

	if (!ll_back_merge_fn(q, req, bio))
		return false;

	trace_block_bio_backmerge(q, bio);

	if ((req->cmd_flags & REQ_FAILFAST_MASK) != ff)
		blk_rq_set_mixed_merge(req);

	req->biotail->bi_next = bio;
	req->biotail = bio;
	req->__data_len += bio->bi_size;
	req->ioprio = ioprio_best(req->ioprio, bio_prio(bio));

	drive_stat_acct(req, 0);
	return true;
}

static bool bio_attempt_front_merge(struct request_queue *q,
				    struct request *req, struct bio *bio)
{
	const int ff = bio->bi_rw & REQ_FAILFAST_MASK;

	if (!ll_front_merge_fn(q, req, bio))
		return false;

	trace_block_bio_frontmerge(q, bio);

	if ((req->cmd_flags & REQ_FAILFAST_MASK) != ff)
		blk_rq_set_mixed_merge(req);

	bio->bi_next = req->bio;
	req->bio = bio;

	req->buffer = bio_data(bio);
	req->__sector = bio->bi_sector;
	req->__data_len += bio->bi_size;
	req->ioprio = ioprio_best(req->ioprio, bio_prio(bio));

	drive_stat_acct(req, 0);
	return true;
}

static bool attempt_plug_merge(struct request_queue *q, struct bio *bio,
			       unsigned int *request_count)
{
	struct blk_plug *plug;
	struct request *rq;
	bool ret = false;

	plug = current->plug;
	if (!plug)
		goto out;
	*request_count = 0;

	list_for_each_entry_reverse(rq, &plug->list, queuelist) {
		int el_ret;

		if (rq->q == q)
			(*request_count)++;

		if (rq->q != q || !blk_rq_merge_ok(rq, bio))
			continue;

		el_ret = blk_try_merge(rq, bio);
		if (el_ret == ELEVATOR_BACK_MERGE) {
			ret = bio_attempt_back_merge(q, rq, bio);
			if (ret)
				break;
		} else if (el_ret == ELEVATOR_FRONT_MERGE) {
			ret = bio_attempt_front_merge(q, rq, bio);
			if (ret)
				break;
		}
	}
out:
	return ret;
}

void init_request_from_bio(struct request *req, struct bio *bio)
{
	req->cmd_type = REQ_TYPE_FS;

	req->cmd_flags |= bio->bi_rw & REQ_COMMON_MASK;
	if (bio->bi_rw & REQ_RAHEAD)
		req->cmd_flags |= REQ_FAILFAST_MASK;

	req->enter_time = ktime_get();
	req->pid = task_pid_nr(current);
	req->errors = 0;
	req->__sector = bio->bi_sector;
	req->ioprio = bio_prio(bio);
	blk_rq_bio_prep(req->q, req, bio);
}
EXPORT_SYMBOL(init_request_from_bio);

void blk_queue_bio(struct request_queue *q, struct bio *bio)
{
	const bool sync = !!(bio->bi_rw & REQ_SYNC);
	struct blk_plug *plug;
	int el_ret, rw_flags, where = ELEVATOR_INSERT_SORT;
	struct request *req;
	unsigned int request_count = 0;

	blk_queue_bounce(q, &bio);

	if (bio->bi_rw & (REQ_FLUSH | REQ_FUA)) {
		spin_lock_irq(q->queue_lock);
		where = ELEVATOR_INSERT_FLUSH;
		goto get_rq;
	}

	if (attempt_plug_merge(q, bio, &request_count))
		return;

	spin_lock_irq(q->queue_lock);

	el_ret = elv_merge(q, &req, bio);
	if (el_ret == ELEVATOR_BACK_MERGE) {
		if (bio_attempt_back_merge(q, req, bio)) {
			elv_bio_merged(q, req, bio);
			if (!attempt_back_merge(q, req))
				elv_merged_request(q, req, el_ret);
			goto out_unlock;
		}
	} else if (el_ret == ELEVATOR_FRONT_MERGE) {
		if (bio_attempt_front_merge(q, req, bio)) {
			elv_bio_merged(q, req, bio);
			if (!attempt_front_merge(q, req))
				elv_merged_request(q, req, el_ret);
			goto out_unlock;
		}
	}

get_rq:
	rw_flags = bio_data_dir(bio);
	if (sync)
		rw_flags |= REQ_SYNC;

	req = get_request_wait(q, rw_flags, bio);
	if (unlikely(!req)) {
		bio_endio(bio, -ENODEV);	
		goto out_unlock;
	}

	init_request_from_bio(req, bio);

	if (test_bit(QUEUE_FLAG_SAME_COMP, &q->queue_flags))
		req->cpu = raw_smp_processor_id();

	plug = current->plug;
	if (plug) {
		if (list_empty(&plug->list))
			trace_block_plug(q);
		else {
			if (!plug->should_sort) {
				struct request *__rq;

				__rq = list_entry_rq(plug->list.prev);
				if (__rq->q != q)
					plug->should_sort = 1;
			}
			if (request_count >= BLK_MAX_REQUEST_COUNT) {
				blk_flush_plug_list(plug, false);
				trace_block_plug(q);
			}
		}
		list_add_tail(&req->queuelist, &plug->list);
		drive_stat_acct(req, 1);
	} else {
		spin_lock_irq(q->queue_lock);
		add_acct_request(q, req, where);
		__blk_run_queue(q);
out_unlock:
		spin_unlock_irq(q->queue_lock);
	}
}
EXPORT_SYMBOL_GPL(blk_queue_bio);	

static inline void blk_partition_remap(struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;

	if (bio_sectors(bio) && bdev != bdev->bd_contains) {
		struct hd_struct *p = bdev->bd_part;

		bio->bi_sector += p->start_sect;
		bio->bi_bdev = bdev->bd_contains;

		trace_block_bio_remap(bdev_get_queue(bio->bi_bdev), bio,
				      bdev->bd_dev,
				      bio->bi_sector - p->start_sect);
	}
}

static void handle_bad_sector(struct bio *bio)
{
	char b[BDEVNAME_SIZE];

	printk(KERN_INFO "attempt to access beyond end of device\n");
	printk(KERN_INFO "%s: rw=%ld, want=%Lu, limit=%Lu\n",
			bdevname(bio->bi_bdev, b),
			bio->bi_rw,
			(unsigned long long)bio->bi_sector + bio_sectors(bio),
			(long long)(i_size_read(bio->bi_bdev->bd_inode) >> 9));

	set_bit(BIO_EOF, &bio->bi_flags);
}

#ifdef CONFIG_FAIL_MAKE_REQUEST

static DECLARE_FAULT_ATTR(fail_make_request);

static int __init setup_fail_make_request(char *str)
{
	return setup_fault_attr(&fail_make_request, str);
}
__setup("fail_make_request=", setup_fail_make_request);

static bool should_fail_request(struct hd_struct *part, unsigned int bytes)
{
	return part->make_it_fail && should_fail(&fail_make_request, bytes);
}

static int __init fail_make_request_debugfs(void)
{
	struct dentry *dir = fault_create_debugfs_attr("fail_make_request",
						NULL, &fail_make_request);

	return IS_ERR(dir) ? PTR_ERR(dir) : 0;
}

late_initcall(fail_make_request_debugfs);

#else 

static inline bool should_fail_request(struct hd_struct *part,
					unsigned int bytes)
{
	return false;
}

#endif 

static inline int bio_check_eod(struct bio *bio, unsigned int nr_sectors)
{
	sector_t maxsector;

	if (!nr_sectors)
		return 0;

	
	maxsector = i_size_read(bio->bi_bdev->bd_inode) >> 9;
	if (maxsector) {
		sector_t sector = bio->bi_sector;

		if (maxsector < nr_sectors || maxsector - nr_sectors < sector) {
			handle_bad_sector(bio);
			return 1;
		}
	}

	return 0;
}

static noinline_for_stack bool
generic_make_request_checks(struct bio *bio)
{
	struct request_queue *q;
	int nr_sectors = bio_sectors(bio);
	int err = -EIO;
	char b[BDEVNAME_SIZE];
	struct hd_struct *part;

	might_sleep();

	if (bio_check_eod(bio, nr_sectors))
		goto end_io;

	q = bdev_get_queue(bio->bi_bdev);
	if (unlikely(!q)) {
		printk(KERN_ERR
		       "generic_make_request: Trying to access "
			"nonexistent block-device %s (%Lu)\n",
			bdevname(bio->bi_bdev, b),
			(long long) bio->bi_sector);
		goto end_io;
	}

	if (unlikely(!(bio->bi_rw & (REQ_DISCARD | REQ_SANITIZE)) &&
		     nr_sectors > queue_max_hw_sectors(q))) {
		printk(KERN_ERR "bio too big device %s (%u > %u)\n",
		       bdevname(bio->bi_bdev, b),
		       bio_sectors(bio),
		       queue_max_hw_sectors(q));
		goto end_io;
	}

	part = bio->bi_bdev->bd_part;
	if (should_fail_request(part, bio->bi_size) ||
	    should_fail_request(&part_to_disk(part)->part0,
				bio->bi_size))
		goto end_io;

	blk_partition_remap(bio);

	if (bio_integrity_enabled(bio) && bio_integrity_prep(bio))
		goto end_io;

	if (bio_check_eod(bio, nr_sectors))
		goto end_io;

	if ((bio->bi_rw & (REQ_FLUSH | REQ_FUA)) && !q->flush_flags) {
		bio->bi_rw &= ~(REQ_FLUSH | REQ_FUA);
		if (!nr_sectors) {
			err = 0;
			goto end_io;
		}
	}

	if ((bio->bi_rw & REQ_DISCARD) &&
	    (!blk_queue_discard(q) ||
	     ((bio->bi_rw & REQ_SECURE) &&
	      !blk_queue_secdiscard(q)))) {
		err = -EOPNOTSUPP;
		goto end_io;
	}

	if ((bio->bi_rw & REQ_SANITIZE) &&
	    (!blk_queue_sanitize(q))) {
		pr_info("%s - got a SANITIZE request but the queue "
		       "doesn't support sanitize requests", __func__);
		err = -EOPNOTSUPP;
		goto end_io;
	}

	if (blk_throtl_bio(q, bio))
		return false;	

	trace_block_bio_queue(q, bio);
	return true;

end_io:
	bio_endio(bio, err);
	return false;
}

void generic_make_request(struct bio *bio)
{
	struct bio_list bio_list_on_stack;

	if (!generic_make_request_checks(bio))
		return;

	if (current->bio_list) {
		bio_list_add(current->bio_list, bio);
		return;
	}

	BUG_ON(bio->bi_next);
	bio_list_init(&bio_list_on_stack);
	current->bio_list = &bio_list_on_stack;
	do {
		struct request_queue *q = bdev_get_queue(bio->bi_bdev);

		q->make_request_fn(q, bio);

		bio = bio_list_pop(current->bio_list);
	} while (bio);
	current->bio_list = NULL; 
}
EXPORT_SYMBOL(generic_make_request);

void submit_bio(int rw, struct bio *bio)
{
	int count = bio_sectors(bio);

	bio->bi_rw |= rw;

	if (bio_has_data(bio) &&
	    (!(rw & (REQ_DISCARD | REQ_SANITIZE)))) {
		if (rw & WRITE) {
			count_vm_events(PGPGOUT, count);
		} else {
			task_io_account_read(bio->bi_size);
			collect_io_stats(bio->bi_size, READ);
			count_vm_events(PGPGIN, count);
		}

		if (unlikely(block_dump)) {
			char b[BDEVNAME_SIZE];
			printk(KERN_DEBUG "%s(%d): %s block %Lu on %s (%u sectors)\n",
			current->comm, task_pid_nr(current),
				(rw & WRITE) ? "WRITE" : "READ",
				(unsigned long long)bio->bi_sector,
				bdevname(bio->bi_bdev, b),
				count);
		}
	}

	generic_make_request(bio);
}
EXPORT_SYMBOL(submit_bio);

int blk_rq_check_limits(struct request_queue *q, struct request *rq)
{
	if (rq->cmd_flags & (REQ_DISCARD | REQ_SANITIZE))
		return 0;

	if (blk_rq_sectors(rq) > queue_max_sectors(q) ||
	    blk_rq_bytes(rq) > queue_max_hw_sectors(q) << 9) {
		printk(KERN_ERR "%s: over max size limit.\n", __func__);
		return -EIO;
	}

	blk_recalc_rq_segments(rq);
	if (rq->nr_phys_segments > queue_max_segments(q)) {
		printk(KERN_ERR "%s: over max segments limit.\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(blk_rq_check_limits);

int blk_insert_cloned_request(struct request_queue *q, struct request *rq)
{
	unsigned long flags;
	int where = ELEVATOR_INSERT_BACK;

	if (blk_rq_check_limits(q, rq))
		return -EIO;

	if (rq->rq_disk &&
	    should_fail_request(&rq->rq_disk->part0, blk_rq_bytes(rq)))
		return -EIO;

	spin_lock_irqsave(q->queue_lock, flags);
	if (unlikely(blk_queue_dead(q))) {
		spin_unlock_irqrestore(q->queue_lock, flags);
		return -ENODEV;
	}

	BUG_ON(blk_queued_rq(rq));

	if (rq->cmd_flags & (REQ_FLUSH|REQ_FUA))
		where = ELEVATOR_INSERT_FLUSH;

	add_acct_request(q, rq, where);
	if (where == ELEVATOR_INSERT_FLUSH)
		__blk_run_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(blk_insert_cloned_request);

unsigned int blk_rq_err_bytes(const struct request *rq)
{
	unsigned int ff = rq->cmd_flags & REQ_FAILFAST_MASK;
	unsigned int bytes = 0;
	struct bio *bio;

	if (!(rq->cmd_flags & REQ_MIXED_MERGE))
		return blk_rq_bytes(rq);

	for (bio = rq->bio; bio; bio = bio->bi_next) {
		if ((bio->bi_rw & ff) != ff)
			break;
		bytes += bio->bi_size;
	}

	
	BUG_ON(blk_rq_bytes(rq) && !bytes);
	return bytes;
}
EXPORT_SYMBOL_GPL(blk_rq_err_bytes);

static void blk_account_io_completion(struct request *req, unsigned int bytes)
{
	if (blk_do_io_stat(req)) {
		const int rw = rq_data_dir(req);
		struct hd_struct *part;
		int cpu;

		cpu = part_stat_lock();
		part = req->part;
		part_stat_add(cpu, part, sectors[rw], bytes >> 9);
		part_stat_unlock();
	}
}

static void blk_account_io_done(struct request *req)
{
	if (blk_do_io_stat(req) && !(req->cmd_flags & REQ_FLUSH_SEQ)) {
		unsigned long duration = jiffies - req->start_time;
		const int rw = rq_data_dir(req);
		struct hd_struct *part;
		int cpu;

		cpu = part_stat_lock();
		part = req->part;

		part_stat_inc(cpu, part, ios[rw]);
		part_stat_add(cpu, part, ticks[rw], duration);
		part_round_stats(cpu, part);
		part_dec_in_flight(part, rw);

		hd_struct_put(part);
		part_stat_unlock();
	}
}

#ifdef CONFIG_PM_RUNTIME
static struct request *blk_pm_peek_request(struct request_queue *q,
					   struct request *rq)
{
	if (q->dev && (q->rpm_status == RPM_SUSPENDED ||
	    (q->rpm_status != RPM_ACTIVE && !(rq->cmd_flags & REQ_PM))))
		return NULL;
	else
		return rq;
}
#else
static inline struct request *blk_pm_peek_request(struct request_queue *q,
						  struct request *rq)
{
	return rq;
}
#endif

struct request *blk_peek_request(struct request_queue *q)
{
	struct request *rq;
	int ret;

	while ((rq = __elv_next_request(q)) != NULL) {

		rq = blk_pm_peek_request(q, rq);
		if (!rq)
			break;

		if (!(rq->cmd_flags & REQ_STARTED)) {
			if (rq->cmd_flags & REQ_SORTED)
				elv_activate_rq(q, rq);

			rq->cmd_flags |= REQ_STARTED;
			if (rq->cmd_flags & REQ_URGENT) {
				WARN_ON(q->dispatched_urgent);
				q->dispatched_urgent = true;
			}
			trace_block_rq_issue(q, rq);
		}

		if (!q->boundary_rq || q->boundary_rq == rq) {
			q->end_sector = rq_end_sector(rq);
			q->boundary_rq = NULL;
		}

		if (rq->cmd_flags & REQ_DONTPREP)
			break;

		if (q->dma_drain_size && blk_rq_bytes(rq)) {
			rq->nr_phys_segments++;
		}

		if (!q->prep_rq_fn)
			break;

		ret = q->prep_rq_fn(q, rq);
		if (ret == BLKPREP_OK) {
			break;
		} else if (ret == BLKPREP_DEFER) {
			if (q->dma_drain_size && blk_rq_bytes(rq) &&
			    !(rq->cmd_flags & REQ_DONTPREP)) {
				--rq->nr_phys_segments;
			}

			rq = NULL;
			break;
		} else if (ret == BLKPREP_KILL) {
			rq->cmd_flags |= REQ_QUIET;
			blk_start_request(rq);
			__blk_end_request_all(rq, -EIO);
		} else {
			printk(KERN_ERR "%s: bad return=%d\n", __func__, ret);
			break;
		}
	}

	return rq;
}
EXPORT_SYMBOL(blk_peek_request);

void blk_dequeue_request(struct request *rq)
{
	struct request_queue *q = rq->q;

	BUG_ON(list_empty(&rq->queuelist));
	BUG_ON(ELV_ON_HASH(rq));

	list_del_init(&rq->queuelist);

	if (blk_account_rq(rq)) {
		q->in_flight[rq_is_sync(rq)]++;
		set_io_start_time_ns(rq);
	}
}

void blk_start_request(struct request *req)
{
	blk_dequeue_request(req);

	req->resid_len = blk_rq_bytes(req);
	if (unlikely(blk_bidi_rq(req)))
		req->next_rq->resid_len = blk_rq_bytes(req->next_rq);

	blk_add_timer(req);
}
EXPORT_SYMBOL(blk_start_request);

struct request *blk_fetch_request(struct request_queue *q)
{
	struct request *rq;

	rq = blk_peek_request(q);
	if (rq)
		blk_start_request(rq);
	return rq;
}
EXPORT_SYMBOL(blk_fetch_request);

bool blk_update_request(struct request *req, int error, unsigned int nr_bytes)
{
	int total_bytes, bio_nbytes, next_idx = 0;
	struct bio *bio;

	if (!req->bio)
		return false;

	trace_block_rq_complete(req->q, req);

	if (req->cmd_type == REQ_TYPE_FS)
		req->errors = 0;

	if (error && req->cmd_type == REQ_TYPE_FS &&
	    !(req->cmd_flags & REQ_QUIET)) {
		char *error_type;

		switch (error) {
		case -ENOLINK:
			error_type = "recoverable transport";
			break;
		case -EREMOTEIO:
			error_type = "critical target";
			break;
		case -EBADE:
			error_type = "critical nexus";
			break;
		case -EIO:
		default:
			error_type = "I/O";
			break;
		}
		printk_ratelimited(
			KERN_ERR "end_request: %s error, dev %s, sector %llu\n",
			error_type,
			req->rq_disk ? req->rq_disk->disk_name : "?",
			(unsigned long long)blk_rq_pos(req));
	}

	blk_account_io_completion(req, nr_bytes);

	total_bytes = bio_nbytes = 0;

	if (bio_flagged(req->bio, BIO_DONTFREE))
		return false;

	while ((bio = req->bio) != NULL) {
		int nbytes;

		if (nr_bytes >= bio->bi_size) {
			req->bio = bio->bi_next;
			nbytes = bio->bi_size;
			req_bio_endio(req, bio, nbytes, error);
			next_idx = 0;
			bio_nbytes = 0;
		} else {
			int idx = bio->bi_idx + next_idx;

			if (unlikely(idx >= bio->bi_vcnt)) {
				blk_dump_rq_flags(req, "__end_that");
				printk(KERN_ERR "%s: bio idx %d >= vcnt %d\n",
				       __func__, idx, bio->bi_vcnt);
				break;
			}

			nbytes = bio_iovec_idx(bio, idx)->bv_len;
			BIO_BUG_ON(nbytes > bio->bi_size);

			if (unlikely(nbytes > nr_bytes)) {
				bio_nbytes += nr_bytes;
				total_bytes += nr_bytes;
				break;
			}

			next_idx++;
			bio_nbytes += nbytes;
		}

		total_bytes += nbytes;
		nr_bytes -= nbytes;

		bio = req->bio;
		if (bio) {
			if (unlikely(nr_bytes <= 0))
				break;
		}
	}

	if (!req->bio) {
		req->__data_len = 0;
		return false;
	}

	if (bio_nbytes) {
		req_bio_endio(req, bio, bio_nbytes, error);
		bio->bi_idx += next_idx;
		bio_iovec(bio)->bv_offset += nr_bytes;
		bio_iovec(bio)->bv_len -= nr_bytes;
	}

	req->__data_len -= total_bytes;
	req->buffer = bio_data(req->bio);

	
	if (req->cmd_type == REQ_TYPE_FS || (req->cmd_flags & REQ_DISCARD))
		req->__sector += total_bytes >> 9;

	
	if (req->cmd_flags & REQ_MIXED_MERGE) {
		req->cmd_flags &= ~REQ_FAILFAST_MASK;
		req->cmd_flags |= req->bio->bi_rw & REQ_FAILFAST_MASK;
	}

	if (blk_rq_bytes(req) < blk_rq_cur_bytes(req)) {
		blk_dump_rq_flags(req, "request botched");
		req->__data_len = blk_rq_cur_bytes(req);
	}

	
	blk_recalc_rq_segments(req);

	return true;
}
EXPORT_SYMBOL_GPL(blk_update_request);

static bool blk_update_bidi_request(struct request *rq, int error,
				    unsigned int nr_bytes,
				    unsigned int bidi_bytes)
{
	if (blk_update_request(rq, error, nr_bytes))
		return true;

	
	if (unlikely(blk_bidi_rq(rq)) &&
	    blk_update_request(rq->next_rq, error, bidi_bytes))
		return true;

	if (blk_queue_add_random(rq->q))
		add_disk_randomness(rq->rq_disk);

	return false;
}

void blk_unprep_request(struct request *req)
{
	struct request_queue *q = req->q;

	req->cmd_flags &= ~REQ_DONTPREP;
	if (q->unprep_rq_fn)
		q->unprep_rq_fn(q, req);
}
EXPORT_SYMBOL_GPL(blk_unprep_request);

static void blk_finish_request(struct request *req, int error)
{
	if (blk_rq_tagged(req))
		blk_queue_end_tag(req->q, req);

	BUG_ON(blk_queued_rq(req));

	if (unlikely(laptop_mode) && req->cmd_type == REQ_TYPE_FS)
		laptop_io_completion(&req->q->backing_dev_info);

	blk_delete_timer(req);

	if (req->cmd_flags & REQ_DONTPREP)
		blk_unprep_request(req);


	blk_account_io_done(req);

	if (req->end_io)
		req->end_io(req, error);
	else {
		if (blk_bidi_rq(req))
			__blk_put_request(req->next_rq->q, req->next_rq);

		__blk_put_request(req->q, req);
	}
}

static bool blk_end_bidi_request(struct request *rq, int error,
				 unsigned int nr_bytes, unsigned int bidi_bytes)
{
	struct request_queue *q = rq->q;
	unsigned long flags;

	if (blk_update_bidi_request(rq, error, nr_bytes, bidi_bytes))
		return true;

	spin_lock_irqsave(q->queue_lock, flags);
	blk_finish_request(rq, error);
	spin_unlock_irqrestore(q->queue_lock, flags);

	return false;
}

bool __blk_end_bidi_request(struct request *rq, int error,
				   unsigned int nr_bytes, unsigned int bidi_bytes)
{
	if (blk_update_bidi_request(rq, error, nr_bytes, bidi_bytes))
		return true;

	blk_finish_request(rq, error);

	return false;
}

bool blk_end_request(struct request *rq, int error, unsigned int nr_bytes)
{
	return blk_end_bidi_request(rq, error, nr_bytes, 0);
}
EXPORT_SYMBOL(blk_end_request);

void blk_end_request_all(struct request *rq, int error)
{
	bool pending;
	unsigned int bidi_bytes = 0;

	if (unlikely(blk_bidi_rq(rq)))
		bidi_bytes = blk_rq_bytes(rq->next_rq);

	pending = blk_end_bidi_request(rq, error, blk_rq_bytes(rq), bidi_bytes);
	BUG_ON(pending);
}
EXPORT_SYMBOL(blk_end_request_all);

bool blk_end_request_cur(struct request *rq, int error)
{
	return blk_end_request(rq, error, blk_rq_cur_bytes(rq));
}
EXPORT_SYMBOL(blk_end_request_cur);

bool blk_end_request_err(struct request *rq, int error)
{
	WARN_ON(error >= 0);
	return blk_end_request(rq, error, blk_rq_err_bytes(rq));
}
EXPORT_SYMBOL_GPL(blk_end_request_err);

bool __blk_end_request(struct request *rq, int error, unsigned int nr_bytes)
{
	return __blk_end_bidi_request(rq, error, nr_bytes, 0);
}
EXPORT_SYMBOL(__blk_end_request);

void __blk_end_request_all(struct request *rq, int error)
{
	bool pending;
	unsigned int bidi_bytes = 0;

	if (unlikely(blk_bidi_rq(rq)))
		bidi_bytes = blk_rq_bytes(rq->next_rq);

	pending = __blk_end_bidi_request(rq, error, blk_rq_bytes(rq), bidi_bytes);
	BUG_ON(pending);
}
EXPORT_SYMBOL(__blk_end_request_all);

bool __blk_end_request_cur(struct request *rq, int error)
{
	return __blk_end_request(rq, error, blk_rq_cur_bytes(rq));
}
EXPORT_SYMBOL(__blk_end_request_cur);

bool __blk_end_request_err(struct request *rq, int error)
{
	WARN_ON(error >= 0);
	return __blk_end_request(rq, error, blk_rq_err_bytes(rq));
}
EXPORT_SYMBOL_GPL(__blk_end_request_err);

void blk_rq_bio_prep(struct request_queue *q, struct request *rq,
		     struct bio *bio)
{
	
	rq->cmd_flags |= bio->bi_rw & REQ_WRITE;

	if (bio_has_data(bio)) {
		rq->nr_phys_segments = bio_phys_segments(q, bio);
		rq->buffer = bio_data(bio);
	}
	rq->__data_len = bio->bi_size;
	rq->bio = rq->biotail = bio;

	if (bio->bi_bdev)
		rq->rq_disk = bio->bi_bdev->bd_disk;
}

#if ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE
void rq_flush_dcache_pages(struct request *rq)
{
	struct req_iterator iter;
	struct bio_vec *bvec;

	rq_for_each_segment(bvec, rq, iter)
		flush_dcache_page(bvec->bv_page);
}
EXPORT_SYMBOL_GPL(rq_flush_dcache_pages);
#endif

int blk_lld_busy(struct request_queue *q)
{
	if (q->lld_busy_fn)
		return q->lld_busy_fn(q);

	return 0;
}
EXPORT_SYMBOL_GPL(blk_lld_busy);

void blk_rq_unprep_clone(struct request *rq)
{
	struct bio *bio;

	while ((bio = rq->bio) != NULL) {
		rq->bio = bio->bi_next;

		bio_put(bio);
	}
}
EXPORT_SYMBOL_GPL(blk_rq_unprep_clone);

static void __blk_rq_prep_clone(struct request *dst, struct request *src)
{
	dst->cpu = src->cpu;
	dst->cmd_flags = (src->cmd_flags & REQ_CLONE_MASK) | REQ_NOMERGE;
	dst->cmd_type = src->cmd_type;
	dst->__sector = blk_rq_pos(src);
	dst->__data_len = blk_rq_bytes(src);
	dst->nr_phys_segments = src->nr_phys_segments;
	dst->ioprio = src->ioprio;
	dst->extra_len = src->extra_len;
}

int blk_rq_prep_clone(struct request *rq, struct request *rq_src,
		      struct bio_set *bs, gfp_t gfp_mask,
		      int (*bio_ctr)(struct bio *, struct bio *, void *),
		      void *data)
{
	struct bio *bio, *bio_src;

	if (!bs)
		bs = fs_bio_set;

	blk_rq_init(NULL, rq);

	__rq_for_each_bio(bio_src, rq_src) {
		bio = bio_alloc_bioset(gfp_mask, bio_src->bi_max_vecs, bs);
		if (!bio)
			goto free_and_out;

		__bio_clone(bio, bio_src);

		if (bio_integrity(bio_src) &&
		    bio_integrity_clone(bio, bio_src, gfp_mask, bs))
			goto free_and_out;

		if (bio_ctr && bio_ctr(bio, bio_src, data))
			goto free_and_out;

		if (rq->bio) {
			rq->biotail->bi_next = bio;
			rq->biotail = bio;
		} else
			rq->bio = rq->biotail = bio;
	}

	__blk_rq_prep_clone(rq, rq_src);

	return 0;

free_and_out:
	if (bio)
		bio_free(bio, bs);
	blk_rq_unprep_clone(rq);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(blk_rq_prep_clone);

int kblockd_schedule_work(struct request_queue *q, struct work_struct *work)
{
	return queue_work(kblockd_workqueue, work);
}
EXPORT_SYMBOL(kblockd_schedule_work);

int kblockd_schedule_delayed_work(struct request_queue *q,
			struct delayed_work *dwork, unsigned long delay)
{
	return queue_delayed_work(kblockd_workqueue, dwork, delay);
}
EXPORT_SYMBOL(kblockd_schedule_delayed_work);

#define PLUG_MAGIC	0x91827364

void blk_start_plug(struct blk_plug *plug)
{
	struct task_struct *tsk = current;

	plug->magic = PLUG_MAGIC;
	INIT_LIST_HEAD(&plug->list);
	INIT_LIST_HEAD(&plug->cb_list);
	plug->should_sort = 0;

	if (!tsk->plug) {
		tsk->plug = plug;
	}
}
EXPORT_SYMBOL(blk_start_plug);

static int plug_rq_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct request *rqa = container_of(a, struct request, queuelist);
	struct request *rqb = container_of(b, struct request, queuelist);

	return !(rqa->q <= rqb->q);
}

static void queue_unplugged(struct request_queue *q, unsigned int depth,
			    bool from_schedule)
	__releases(q->queue_lock)
{
	trace_block_unplug(q, depth, !from_schedule);

	if (unlikely(blk_queue_dead(q))) {
		spin_unlock(q->queue_lock);
		return;
	}

	if (from_schedule) {
		spin_unlock(q->queue_lock);
		blk_run_queue_async(q);
	} else {
		__blk_run_queue(q);
		spin_unlock(q->queue_lock);
	}

}

static void flush_plug_callbacks(struct blk_plug *plug)
{
	LIST_HEAD(callbacks);

	if (list_empty(&plug->cb_list))
		return;

	list_splice_init(&plug->cb_list, &callbacks);

	while (!list_empty(&callbacks)) {
		struct blk_plug_cb *cb = list_first_entry(&callbacks,
							  struct blk_plug_cb,
							  list);
		list_del(&cb->list);
		cb->callback(cb);
	}
}

void blk_flush_plug_list(struct blk_plug *plug, bool from_schedule)
{
	struct request_queue *q;
	unsigned long flags;
	struct request *rq;
	LIST_HEAD(list);
	unsigned int depth;

	BUG_ON(plug->magic != PLUG_MAGIC);

	flush_plug_callbacks(plug);
	if (list_empty(&plug->list))
		return;

	list_splice_init(&plug->list, &list);

	if (plug->should_sort) {
		list_sort(NULL, &list, plug_rq_cmp);
		plug->should_sort = 0;
	}

	q = NULL;
	depth = 0;

	local_irq_save(flags);
	while (!list_empty(&list)) {
		rq = list_entry_rq(list.next);
		list_del_init(&rq->queuelist);
		BUG_ON(!rq->q);
		if (rq->q != q) {
			if (q)
				queue_unplugged(q, depth, from_schedule);
			q = rq->q;
			depth = 0;
			spin_lock(q->queue_lock);
		}

		if (unlikely(blk_queue_dead(q))) {
			__blk_end_request_all(rq, -ENODEV);
			continue;
		}

		if (rq->cmd_flags & (REQ_FLUSH | REQ_FUA))
			__elv_add_request(q, rq, ELEVATOR_INSERT_FLUSH);
		else
			__elv_add_request(q, rq, ELEVATOR_INSERT_SORT_MERGE);

		depth++;
	}

	if (q)
		queue_unplugged(q, depth, from_schedule);

	local_irq_restore(flags);
}

void blk_finish_plug(struct blk_plug *plug)
{
	blk_flush_plug_list(plug, false);

	if (plug == current->plug)
		current->plug = NULL;
}
EXPORT_SYMBOL(blk_finish_plug);

#ifdef CONFIG_PM_RUNTIME
void blk_pm_runtime_init(struct request_queue *q, struct device *dev)
{
	q->dev = dev;
	q->rpm_status = RPM_ACTIVE;
	pm_runtime_set_autosuspend_delay(q->dev, -1);
	pm_runtime_use_autosuspend(q->dev);
}
EXPORT_SYMBOL(blk_pm_runtime_init);

int blk_pre_runtime_suspend(struct request_queue *q)
{
	int ret = 0;

	spin_lock_irq(q->queue_lock);
	if (q->nr_pending) {
		ret = -EBUSY;
		pm_runtime_mark_last_busy(q->dev);
	} else {
		q->rpm_status = RPM_SUSPENDING;
	}
	spin_unlock_irq(q->queue_lock);
	return ret;
}
EXPORT_SYMBOL(blk_pre_runtime_suspend);

void blk_post_runtime_suspend(struct request_queue *q, int err)
{
	spin_lock_irq(q->queue_lock);
	if (!err) {
		q->rpm_status = RPM_SUSPENDED;
	} else {
		q->rpm_status = RPM_ACTIVE;
		pm_runtime_mark_last_busy(q->dev);
	}
	spin_unlock_irq(q->queue_lock);
}
EXPORT_SYMBOL(blk_post_runtime_suspend);

void blk_pre_runtime_resume(struct request_queue *q)
{
	spin_lock_irq(q->queue_lock);
	q->rpm_status = RPM_RESUMING;
	spin_unlock_irq(q->queue_lock);
}
EXPORT_SYMBOL(blk_pre_runtime_resume);

void blk_post_runtime_resume(struct request_queue *q, int err)
{
	spin_lock_irq(q->queue_lock);
	if (!err) {
		q->rpm_status = RPM_ACTIVE;
		__blk_run_queue(q);
		pm_runtime_mark_last_busy(q->dev);
		pm_request_autosuspend(q->dev);
	} else {
		q->rpm_status = RPM_SUSPENDED;
	}
	spin_unlock_irq(q->queue_lock);
}
EXPORT_SYMBOL(blk_post_runtime_resume);
#endif

int __init blk_dev_init(void)
{
	BUILD_BUG_ON(__REQ_NR_BITS > 8 *
			sizeof(((struct request *)0)->cmd_flags));

	
	kblockd_workqueue = alloc_workqueue("kblockd",
					    WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kblockd_workqueue)
		panic("Failed to create kblockd\n");

	request_cachep = kmem_cache_create("blkdev_requests",
			sizeof(struct request), 0, SLAB_PANIC, NULL);

	blk_requestq_cachep = kmem_cache_create("blkdev_queue",
			sizeof(struct request_queue), 0, SLAB_PANIC, NULL);

	return 0;
}

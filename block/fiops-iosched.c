/*
 * IOPS based IO scheduler. Based on CFQ.
 *  Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 *  Shaohua Li <shli@kernel.org>
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/jiffies.h>
#include <linux/rbtree.h>
#include <linux/ioprio.h>
#include "blk.h"

#define VIOS_SCALE_SHIFT 10
#define VIOS_SCALE (1 << VIOS_SCALE_SHIFT)

struct fiops_rb_root {
	struct rb_root rb;
	struct rb_node *left;
	unsigned count;

	u64 min_vios;
};
#define FIOPS_RB_ROOT	(struct fiops_rb_root) { .rb = RB_ROOT}

struct fiops_data {
	struct request_queue *queue;

	struct fiops_rb_root service_tree;

	unsigned int busy_queues;
	unsigned int in_flight[2];

	struct work_struct unplug_work;
};

struct fiops_ioc {
	struct io_cq icq;

	unsigned int flags;
	struct fiops_data *fiopsd;
	struct rb_node rb_node;
	u64 vios; /* key in service_tree */
	struct fiops_rb_root *service_tree;

	unsigned int in_flight;

	struct rb_root sort_list;
	struct list_head fifo;

	pid_t pid;
};

#define ioc_service_tree(ioc) (&((ioc)->fiopsd->service_tree))
#define RQ_CIC(rq)		icq_to_cic((rq)->elv.icq)

enum ioc_state_flags {
	FIOPS_IOC_FLAG_on_rr = 0,	/* on round-robin busy list */
};

#define FIOPS_IOC_FNS(name)						\
static inline void fiops_mark_ioc_##name(struct fiops_ioc *ioc)	\
{									\
	ioc->flags |= (1 << FIOPS_IOC_FLAG_##name);			\
}									\
static inline void fiops_clear_ioc_##name(struct fiops_ioc *ioc)	\
{									\
	ioc->flags &= ~(1 << FIOPS_IOC_FLAG_##name);			\
}									\
static inline int fiops_ioc_##name(const struct fiops_ioc *ioc)	\
{									\
	return ((ioc)->flags & (1 << FIOPS_IOC_FLAG_##name)) != 0;	\
}

FIOPS_IOC_FNS(on_rr);
#undef FIOPS_IOC_FNS

static inline struct fiops_ioc *icq_to_cic(struct io_cq *icq)
{
	/* cic->icq is the first member, %NULL will convert to %NULL */
	return container_of(icq, struct fiops_ioc, icq);
}

static inline struct fiops_ioc *fiops_cic_lookup(struct fiops_data *fiopsd,
					       struct io_context *ioc)
{
	if (ioc)
		return icq_to_cic(ioc_lookup_icq(ioc, fiopsd->queue));
	return NULL;
}

/*
 * The below is leftmost cache rbtree addon
 */
static struct fiops_ioc *fiops_rb_first(struct fiops_rb_root *root)
{
	/* Service tree is empty */
	if (!root->count)
		return NULL;

	if (!root->left)
		root->left = rb_first(&root->rb);

	if (root->left)
		return rb_entry(root->left, struct fiops_ioc, rb_node);

	return NULL;
}

static void rb_erase_init(struct rb_node *n, struct rb_root *root)
{
	rb_erase(n, root);
	RB_CLEAR_NODE(n);
}

static void fiops_rb_erase(struct rb_node *n, struct fiops_rb_root *root)
{
	if (root->left == n)
		root->left = NULL;
	rb_erase_init(n, &root->rb);
	--root->count;
}

static inline u64 max_vios(u64 min_vios, u64 vios)
{
	s64 delta = (s64)(vios - min_vios);
	if (delta > 0)
		min_vios = vios;

	return min_vios;
}

static void fiops_update_min_vios(struct fiops_rb_root *service_tree)
{
	struct fiops_ioc *ioc;

	ioc = fiops_rb_first(service_tree);
	if (!ioc)
		return;
	service_tree->min_vios = max_vios(service_tree->min_vios, ioc->vios);
}

/*
 * The fiopsd->service_trees holds all pending fiops_ioc's that have
 * requests waiting to be processed. It is sorted in the order that
 * we will service the queues.
 */
static void fiops_service_tree_add(struct fiops_data *fiopsd,
	struct fiops_ioc *ioc)
{
	struct rb_node **p, *parent;
	struct fiops_ioc *__ioc;
	struct fiops_rb_root *service_tree = ioc_service_tree(ioc);
	u64 vios;
	int left;

	/* New added IOC */
	if (RB_EMPTY_NODE(&ioc->rb_node))
		vios = max_vios(service_tree->min_vios, ioc->vios);
	else {
		vios = ioc->vios;
		/* ioc->service_tree might not equal to service_tree */
		fiops_rb_erase(&ioc->rb_node, ioc->service_tree);
		ioc->service_tree = NULL;
	}

	left = 1;
	parent = NULL;
	ioc->service_tree = service_tree;
	p = &service_tree->rb.rb_node;
	while (*p) {
		struct rb_node **n;

		parent = *p;
		__ioc = rb_entry(parent, struct fiops_ioc, rb_node);

		/*
		 * sort by key, that represents service time.
		 */
		if (vios <  __ioc->vios)
			n = &(*p)->rb_left;
		else {
			n = &(*p)->rb_right;
			left = 0;
		}

		p = n;
	}

	if (left)
		service_tree->left = &ioc->rb_node;

	ioc->vios = vios;
	rb_link_node(&ioc->rb_node, parent, p);
	rb_insert_color(&ioc->rb_node, &service_tree->rb);
	service_tree->count++;

	fiops_update_min_vios(service_tree);
}

/*
 * Update ioc's position in the service tree.
 */
static void fiops_resort_rr_list(struct fiops_data *fiopsd,
	struct fiops_ioc *ioc)
{
	/*
	 * Resorting requires the ioc to be on the RR list already.
	 */
	if (fiops_ioc_on_rr(ioc))
		fiops_service_tree_add(fiopsd, ioc);
}

/*
 * add to busy list of queues for service, trying to be fair in ordering
 * the pending list according to last request service
 */
static void fiops_add_ioc_rr(struct fiops_data *fiopsd, struct fiops_ioc *ioc)
{
	BUG_ON(fiops_ioc_on_rr(ioc));
	fiops_mark_ioc_on_rr(ioc);

	fiopsd->busy_queues++;

	fiops_resort_rr_list(fiopsd, ioc);
}

/*
 * Called when the ioc no longer has requests pending, remove it from
 * the service tree.
 */
static void fiops_del_ioc_rr(struct fiops_data *fiopsd, struct fiops_ioc *ioc)
{
	BUG_ON(!fiops_ioc_on_rr(ioc));
	fiops_clear_ioc_on_rr(ioc);

	if (!RB_EMPTY_NODE(&ioc->rb_node)) {
		fiops_rb_erase(&ioc->rb_node, ioc->service_tree);
		ioc->service_tree = NULL;
	}

	BUG_ON(!fiopsd->busy_queues);
	fiopsd->busy_queues--;
}

/*
 * rb tree support functions
 */
static void fiops_del_rq_rb(struct request *rq)
{
	struct fiops_ioc *ioc = RQ_CIC(rq);

	elv_rb_del(&ioc->sort_list, rq);
}

static void fiops_add_rq_rb(struct request *rq)
{
	struct fiops_ioc *ioc = RQ_CIC(rq);
	struct fiops_data *fiopsd = ioc->fiopsd;

	elv_rb_add(&ioc->sort_list, rq);

	if (!fiops_ioc_on_rr(ioc))
		fiops_add_ioc_rr(fiopsd, ioc);
}

static void fiops_reposition_rq_rb(struct fiops_ioc *ioc, struct request *rq)
{
	elv_rb_del(&ioc->sort_list, rq);
	fiops_add_rq_rb(rq);
}

static void fiops_remove_request(struct request *rq)
{
	list_del_init(&rq->queuelist);
	fiops_del_rq_rb(rq);
}

static u64 fiops_scaled_vios(struct fiops_data *fiopsd,
	struct fiops_ioc *ioc, struct request *rq)
{
	return VIOS_SCALE;
}

/* return vios dispatched */
static u64 fiops_dispatch_request(struct fiops_data *fiopsd,
	struct fiops_ioc *ioc)
{
	struct request *rq;
	struct request_queue *q = fiopsd->queue;

	rq = rq_entry_fifo(ioc->fifo.next);

	fiops_remove_request(rq);
	elv_dispatch_add_tail(q, rq);

	fiopsd->in_flight[rq_is_sync(rq)]++;
	ioc->in_flight++;

	return fiops_scaled_vios(fiopsd, ioc, rq);
}

static int fiops_forced_dispatch(struct fiops_data *fiopsd)
{
	struct fiops_ioc *ioc;
	int dispatched = 0;

	while ((ioc = fiops_rb_first(&fiopsd->service_tree)) != NULL) {
		while (!list_empty(&ioc->fifo)) {
			fiops_dispatch_request(fiopsd, ioc);
			dispatched++;
		}
		if (fiops_ioc_on_rr(ioc))
			fiops_del_ioc_rr(fiopsd, ioc);
	}
	return dispatched;
}

static struct fiops_ioc *fiops_select_ioc(struct fiops_data *fiopsd)
{
	struct fiops_ioc *ioc;

	if (RB_EMPTY_ROOT(&fiopsd->service_tree.rb))
		return NULL;
	ioc = fiops_rb_first(&fiopsd->service_tree);
	return ioc;
}

static void fiops_charge_vios(struct fiops_data *fiopsd,
	struct fiops_ioc *ioc, u64 vios)
{
	struct fiops_rb_root *service_tree = ioc->service_tree;
	ioc->vios += vios;

	if (RB_EMPTY_ROOT(&ioc->sort_list))
		fiops_del_ioc_rr(fiopsd, ioc);
	else
		fiops_resort_rr_list(fiopsd, ioc);

	fiops_update_min_vios(service_tree);
}

static int fiops_dispatch_requests(struct request_queue *q, int force)
{
	struct fiops_data *fiopsd = q->elevator->elevator_data;
	struct fiops_ioc *ioc;
	u64 vios;

	if (unlikely(force))
		return fiops_forced_dispatch(fiopsd);

	ioc = fiops_select_ioc(fiopsd);
	if (!ioc)
		return 0;

	vios = fiops_dispatch_request(fiopsd, ioc);

	fiops_charge_vios(fiopsd, ioc, vios);
	return 1;
}

static void fiops_insert_request(struct request_queue *q, struct request *rq)
{
	struct fiops_ioc *ioc = RQ_CIC(rq);

	list_add_tail(&rq->queuelist, &ioc->fifo);

	fiops_add_rq_rb(rq);
}

/*
 * scheduler run of queue, if there are requests pending and no one in the
 * driver that will restart queueing
 */
static inline void fiops_schedule_dispatch(struct fiops_data *fiopsd)
{
	if (fiopsd->busy_queues)
		kblockd_schedule_work(fiopsd->queue, &fiopsd->unplug_work);
}

static void fiops_completed_request(struct request_queue *q, struct request *rq)
{
	struct fiops_data *fiopsd = q->elevator->elevator_data;
	struct fiops_ioc *ioc = RQ_CIC(rq);

	fiopsd->in_flight[rq_is_sync(rq)]--;
	ioc->in_flight--;

	if (fiopsd->in_flight[0] + fiopsd->in_flight[1] == 0)
		fiops_schedule_dispatch(fiopsd);
}

static struct request *
fiops_find_rq_fmerge(struct fiops_data *fiopsd, struct bio *bio)
{
	struct task_struct *tsk = current;
	struct fiops_ioc *cic;

	cic = fiops_cic_lookup(fiopsd, tsk->io_context);

	if (cic) {
		sector_t sector = bio->bi_sector + bio_sectors(bio);

		return elv_rb_find(&cic->sort_list, sector);
	}

	return NULL;
}

static int fiops_merge(struct request_queue *q, struct request **req,
		     struct bio *bio)
{
	struct fiops_data *fiopsd = q->elevator->elevator_data;
	struct request *__rq;

	__rq = fiops_find_rq_fmerge(fiopsd, bio);
	if (__rq && elv_rq_merge_ok(__rq, bio)) {
		*req = __rq;
		return ELEVATOR_FRONT_MERGE;
	}

	return ELEVATOR_NO_MERGE;
}

static void fiops_merged_request(struct request_queue *q, struct request *req,
			       int type)
{
	if (type == ELEVATOR_FRONT_MERGE) {
		struct fiops_ioc *ioc = RQ_CIC(req);

		fiops_reposition_rq_rb(ioc, req);
	}
}

static void
fiops_merged_requests(struct request_queue *q, struct request *rq,
		    struct request *next)
{
	struct fiops_ioc *ioc = RQ_CIC(rq);
	struct fiops_data *fiopsd = q->elevator->elevator_data;

	fiops_remove_request(next);

	ioc = RQ_CIC(next);
	/*
	 * all requests of this task are merged to other tasks, delete it
	 * from the service tree.
	 */
	if (fiops_ioc_on_rr(ioc) && RB_EMPTY_ROOT(&ioc->sort_list))
		fiops_del_ioc_rr(fiopsd, ioc);
}

static int fiops_allow_merge(struct request_queue *q, struct request *rq,
			   struct bio *bio)
{
	struct fiops_data *fiopsd = q->elevator->elevator_data;
	struct fiops_ioc *cic;

	/*
	 * Lookup the ioc that this bio will be queued with. Allow
	 * merge only if rq is queued there.
	 */
	cic = fiops_cic_lookup(fiopsd, current->io_context);

	return cic == RQ_CIC(rq);
}

static void fiops_exit_queue(struct elevator_queue *e)
{
	struct fiops_data *fiopsd = e->elevator_data;

	cancel_work_sync(&fiopsd->unplug_work);

	kfree(fiopsd);
}

static void fiops_kick_queue(struct work_struct *work)
{
	struct fiops_data *fiopsd =
		container_of(work, struct fiops_data, unplug_work);
	struct request_queue *q = fiopsd->queue;

	spin_lock_irq(q->queue_lock);
	__blk_run_queue(q);
	spin_unlock_irq(q->queue_lock);
}

static void *fiops_init_queue(struct request_queue *q)
{
	struct fiops_data *fiopsd;

	fiopsd = kzalloc_node(sizeof(*fiopsd), GFP_KERNEL, q->node);
	if (!fiopsd)
		return NULL;

	fiopsd->queue = q;

	fiopsd->service_tree = FIOPS_RB_ROOT;

	INIT_WORK(&fiopsd->unplug_work, fiops_kick_queue);

	return fiopsd;
}

static void fiops_init_icq(struct io_cq *icq)
{
	struct fiops_data *fiopsd = icq->q->elevator->elevator_data;
	struct fiops_ioc *ioc = icq_to_cic(icq);

	RB_CLEAR_NODE(&ioc->rb_node);
	INIT_LIST_HEAD(&ioc->fifo);
	ioc->sort_list = RB_ROOT;

	ioc->fiopsd = fiopsd;

	ioc->pid = current->pid;
}

static struct elevator_type iosched_fiops = {
	.ops = {
		.elevator_merge_fn =		fiops_merge,
		.elevator_merged_fn =		fiops_merged_request,
		.elevator_merge_req_fn =	fiops_merged_requests,
		.elevator_allow_merge_fn =	fiops_allow_merge,
		.elevator_dispatch_fn =		fiops_dispatch_requests,
		.elevator_add_req_fn =		fiops_insert_request,
		.elevator_completed_req_fn =	fiops_completed_request,
		.elevator_former_req_fn =	elv_rb_former_request,
		.elevator_latter_req_fn =	elv_rb_latter_request,
		.elevator_init_icq_fn =		fiops_init_icq,
		.elevator_init_fn =		fiops_init_queue,
		.elevator_exit_fn =		fiops_exit_queue,
	},
	.icq_size	=	sizeof(struct fiops_ioc),
	.icq_align	=	__alignof__(struct fiops_ioc),
	.elevator_name =	"fiops",
	.elevator_owner =	THIS_MODULE,
};

static int __init fiops_init(void)
{
	return elv_register(&iosched_fiops);
}

static void __exit fiops_exit(void)
{
	elv_unregister(&iosched_fiops);
}

module_init(fiops_init);
module_exit(fiops_exit);

MODULE_AUTHOR("Jens Axboe, Shaohua Li <shli@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IOPS based IO scheduler");

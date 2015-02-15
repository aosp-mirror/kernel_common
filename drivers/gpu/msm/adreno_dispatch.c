/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/err.h>

#include "kgsl.h"
#include "adreno.h"
#include "adreno_ringbuffer.h"
#include "adreno_trace.h"
#include "kgsl_sharedmem.h"
#include "kgsl_htc.h"

#define CMDQUEUE_NEXT(_i, _s) (((_i) + 1) % (_s))

static unsigned int _context_cmdqueue_size = 50;

static unsigned int _context_queue_wait = 10000;

static unsigned int _context_cmdbatch_burst = 5;

static unsigned int _fault_throttle_time = 3000;
static unsigned int _fault_throttle_burst = 3;

static unsigned int _dispatcher_inflight = 15;

static unsigned int _cmdbatch_timeout = 2000;

static unsigned int _fault_timer_interval = 200;

static unsigned int fault_detect_regs[FT_DETECT_REGS_COUNT];

static unsigned int fault_detect_ts;

static void fault_detect_read(struct kgsl_device *device)
{
	int i;

	fault_detect_ts = kgsl_readtimestamp(device, NULL,
		KGSL_TIMESTAMP_RETIRED);

	for (i = 0; i < FT_DETECT_REGS_COUNT; i++) {
		if (ft_detect_regs[i] == 0)
			continue;
		kgsl_regread(device, ft_detect_regs[i],
			&fault_detect_regs[i]);
	}
}

static inline bool _isidle(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int ts, i;

	if (!kgsl_pwrctrl_isenabled(device))
		goto ret;

	ts = kgsl_readtimestamp(device, NULL, KGSL_TIMESTAMP_RETIRED);

	
	if (adreno_hw_isidle(device) ||
			(ts == adreno_dev->ringbuffer.global_ts))
		goto ret;

	return false;

ret:
	for (i = 0; i < FT_DETECT_REGS_COUNT; i++)
		fault_detect_regs[i] = 0;
	return true;
}

static int fault_detect_read_compare(struct kgsl_device *device)
{
	int i, ret = 0;
	unsigned int ts;

	
	if (_isidle(device) == true)
		ret = 1;

	for (i = 0; i < FT_DETECT_REGS_COUNT; i++) {
		unsigned int val;

		if (ft_detect_regs[i] == 0)
			continue;
		kgsl_regread(device, ft_detect_regs[i], &val);
		if (val != fault_detect_regs[i])
			ret = 1;
		fault_detect_regs[i] = val;
	}

	ts = kgsl_readtimestamp(device, NULL, KGSL_TIMESTAMP_RETIRED);
	if (ts != fault_detect_ts)
		ret = 1;

	fault_detect_ts = ts;

	return ret;
}

static inline struct kgsl_cmdbatch *adreno_dispatcher_get_cmdbatch(
		struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch = NULL;
	int pending;

	spin_lock(&drawctxt->lock);
	if (drawctxt->cmdqueue_head != drawctxt->cmdqueue_tail) {
		cmdbatch = drawctxt->cmdqueue[drawctxt->cmdqueue_head];


		spin_lock(&cmdbatch->lock);
		pending = list_empty(&cmdbatch->synclist) ? 0 : 1;

		if (pending) {
			if (!timer_pending(&cmdbatch->timer))
				mod_timer(&cmdbatch->timer, jiffies + (5 * HZ));
			spin_unlock(&cmdbatch->lock);
		} else {
			spin_unlock(&cmdbatch->lock);
			del_timer_sync(&cmdbatch->timer);
		}

		if (pending) {
			cmdbatch = ERR_PTR(-EAGAIN);
			goto done;
		}

		drawctxt->cmdqueue_head =
			CMDQUEUE_NEXT(drawctxt->cmdqueue_head,
			ADRENO_CONTEXT_CMDQUEUE_SIZE);
		drawctxt->queued--;
	}

done:
	spin_unlock(&drawctxt->lock);

	return cmdbatch;
}

static inline int adreno_dispatcher_requeue_cmdbatch(
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch)
{
	unsigned int prev;
	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base) ||
		drawctxt->state == ADRENO_CONTEXT_STATE_INVALID) {
		spin_unlock(&drawctxt->lock);
		
		kgsl_cmdbatch_destroy(cmdbatch);
		return -EINVAL;
	}

	prev = drawctxt->cmdqueue_head == 0 ?
		(ADRENO_CONTEXT_CMDQUEUE_SIZE - 1) :
		(drawctxt->cmdqueue_head - 1);


	BUG_ON(prev == drawctxt->cmdqueue_tail);

	drawctxt->cmdqueue[prev] = cmdbatch;
	drawctxt->queued++;

	
	drawctxt->cmdqueue_head = prev;
	spin_unlock(&drawctxt->lock);
	return 0;
}

static void  dispatcher_queue_context(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (kgsl_context_detached(&drawctxt->base))
		return;

	spin_lock(&dispatcher->plist_lock);

	if (plist_node_empty(&drawctxt->pending)) {
		
		if (_kgsl_context_get(&drawctxt->base)) {
			trace_dispatch_queue_context(drawctxt);
			plist_add(&drawctxt->pending, &dispatcher->pending);
		}
	}

	spin_unlock(&dispatcher->plist_lock);
}

static int sendcmd(struct adreno_device *adreno_dev,
	struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	dispatcher->inflight++;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	if (dispatcher->inflight == 1 &&
			!test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
		
		ret = kgsl_active_count_get(device);
		if (ret) {
			dispatcher->inflight--;
			kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
			return ret;
		}

		set_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
	}

	ret = adreno_ringbuffer_submitcmd(adreno_dev, cmdbatch);


	if (dispatcher->inflight == 1) {
		if (ret == 0)
			fault_detect_read(device);
		else {
			kgsl_active_count_put(device);
			clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
		}
	}

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	if (ret) {
		dispatcher->inflight--;
		KGSL_DRV_ERR(device,
			"Unable to submit command to the ringbuffer %d\n", ret);
		return ret;
	}

	trace_adreno_cmdbatch_submitted(cmdbatch, dispatcher->inflight);

	dispatcher->cmdqueue[dispatcher->tail] = cmdbatch;
	dispatcher->tail = (dispatcher->tail + 1) %
		ADRENO_DISPATCH_CMDQUEUE_SIZE;

	if (dispatcher->inflight == 1) {
		cmdbatch->expires = jiffies +
			msecs_to_jiffies(_cmdbatch_timeout);
		mod_timer(&dispatcher->timer, cmdbatch->expires);

		
		if (adreno_dev->fast_hang_detect)
			mod_timer(&dispatcher->fault_timer,
				jiffies +
				msecs_to_jiffies(_fault_timer_interval));
	}

	return 0;
}

static int dispatcher_context_sendcmds(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int count = 0;
	int requeued = 0;
	unsigned int timestamp;

	while ((count < _context_cmdbatch_burst) &&
		(dispatcher->inflight < _dispatcher_inflight)) {
		int ret;
		struct kgsl_cmdbatch *cmdbatch;

		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		cmdbatch = adreno_dispatcher_get_cmdbatch(drawctxt);


		if (IS_ERR_OR_NULL(cmdbatch)) {
			if (IS_ERR(cmdbatch) && PTR_ERR(cmdbatch) == -EAGAIN)
				requeued = 1;
			break;
		}


		if (cmdbatch->flags & KGSL_CONTEXT_SYNC) {
			kgsl_cmdbatch_destroy(cmdbatch);
			continue;
		}

		timestamp = cmdbatch->timestamp;

		ret = sendcmd(adreno_dev, cmdbatch);

		if (ret) {
			requeued = adreno_dispatcher_requeue_cmdbatch(drawctxt,
				cmdbatch) ? 0 : 1;
			break;
		}

		drawctxt->submitted_timestamp = timestamp;

		count++;
	}


	if (count)
		wake_up_all(&drawctxt->wq);


	return (count || requeued) ? 1 : 0;
}

static int _adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_context *drawctxt, *next;
	struct plist_head requeue;
	int ret;

	
	if (adreno_gpu_fault(adreno_dev) != 0)
			return 0;

	plist_head_init(&requeue);

	
	while (dispatcher->inflight < _dispatcher_inflight) {

		
		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		spin_lock(&dispatcher->plist_lock);

		if (plist_head_empty(&dispatcher->pending)) {
			spin_unlock(&dispatcher->plist_lock);
			break;
		}

		
		drawctxt = plist_first_entry(&dispatcher->pending,
			struct adreno_context, pending);

		plist_del(&drawctxt->pending, &dispatcher->pending);

		spin_unlock(&dispatcher->plist_lock);

		if (kgsl_context_detached(&drawctxt->base) ||
			drawctxt->state == ADRENO_CONTEXT_STATE_INVALID) {
			kgsl_context_put(&drawctxt->base);
			continue;
		}

		ret = dispatcher_context_sendcmds(adreno_dev, drawctxt);

		if (ret > 0) {
			spin_lock(&dispatcher->plist_lock);


			if (plist_node_empty(&drawctxt->pending))
				plist_add(&drawctxt->pending, &requeue);
			else
				kgsl_context_put(&drawctxt->base);

			spin_unlock(&dispatcher->plist_lock);
		} else {

			kgsl_context_put(&drawctxt->base);
		}
	}

	

	spin_lock(&dispatcher->plist_lock);

	plist_for_each_entry_safe(drawctxt, next, &requeue, pending) {
		plist_del(&drawctxt->pending, &requeue);
		plist_add(&drawctxt->pending, &dispatcher->pending);
	}

	spin_unlock(&dispatcher->plist_lock);

	return 0;
}

int adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	mutex_lock(&dispatcher->mutex);
	ret = _adreno_dispatcher_issuecmds(adreno_dev);
	mutex_unlock(&dispatcher->mutex);

	return ret;
}

static int _check_context_queue(struct adreno_context *drawctxt)
{
	int ret;

	spin_lock(&drawctxt->lock);


	if (drawctxt->state == ADRENO_CONTEXT_STATE_INVALID)
		ret = 1;
	else
		ret = drawctxt->queued < _context_cmdqueue_size ? 1 : 0;

	spin_unlock(&drawctxt->lock);

	return ret;
}

static int get_timestamp(struct adreno_context *drawctxt,
		struct kgsl_cmdbatch *cmdbatch, unsigned int *timestamp)
{
	
	if (cmdbatch->flags & KGSL_CONTEXT_SYNC) {
		*timestamp = 0;
		return 0;
	}

	if (drawctxt->base.flags & KGSL_CONTEXT_USER_GENERATED_TS) {
		if (timestamp_cmp(drawctxt->timestamp, *timestamp) >= 0)
			return -ERANGE;

		drawctxt->timestamp = *timestamp;
	} else
		drawctxt->timestamp++;

	*timestamp = drawctxt->timestamp;
	return 0;
}

int adreno_dispatcher_queue_cmd(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch,
		uint32_t *timestamp)
{
	int ret;

	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EINVAL;
	}


	if (test_and_clear_bit(ADRENO_CONTEXT_FORCE_PREAMBLE, &drawctxt->priv))
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);


	if (test_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->priv)) {
		set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);


		if (cmdbatch->flags & KGSL_CONTEXT_END_OF_FRAME) {
			clear_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->priv);

			set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE, &drawctxt->priv);
		}
	}

	

	while (drawctxt->queued >= _context_cmdqueue_size) {
		trace_adreno_drawctxt_sleep(drawctxt);
		spin_unlock(&drawctxt->lock);

		ret = wait_event_interruptible_timeout(drawctxt->wq,
			_check_context_queue(drawctxt),
			msecs_to_jiffies(_context_queue_wait));

		spin_lock(&drawctxt->lock);
		trace_adreno_drawctxt_wake(drawctxt);

		if (ret <= 0) {
			spin_unlock(&drawctxt->lock);
			return (ret == 0) ? -ETIMEDOUT : (int) ret;
		}
	}

	if (drawctxt->state == ADRENO_CONTEXT_STATE_INVALID) {
		spin_unlock(&drawctxt->lock);
		return -EDEADLK;
	}
	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EINVAL;
	}

	ret = get_timestamp(drawctxt, cmdbatch, timestamp);
	if (ret) {
		spin_unlock(&drawctxt->lock);
		return ret;
	}

	cmdbatch->timestamp = *timestamp;

	
	if (!(cmdbatch->flags & KGSL_CONTEXT_SYNC))
		drawctxt->queued_timestamp = *timestamp;


	if (drawctxt->base.flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE)
		set_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy);
	else
		cmdbatch->fault_policy = adreno_dev->ft_policy;

	
	drawctxt->cmdqueue[drawctxt->cmdqueue_tail] = cmdbatch;
	drawctxt->cmdqueue_tail = (drawctxt->cmdqueue_tail + 1) %
		ADRENO_CONTEXT_CMDQUEUE_SIZE;

	drawctxt->queued++;
	trace_adreno_cmdbatch_queued(cmdbatch, drawctxt->queued);


	spin_unlock(&drawctxt->lock);

	
	dispatcher_queue_context(adreno_dev, drawctxt);


	if (adreno_dev->dispatcher.inflight < _context_cmdbatch_burst)
		adreno_dispatcher_issuecmds(adreno_dev);

	return 0;
}

static int _mark_context(int id, void *ptr, void *data)
{
	unsigned int guilty = *((unsigned int *) data);
	struct kgsl_context *context = ptr;


	if (guilty == 0 || guilty == context->id)
		context->reset_status =
			KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT;
	else if (context->reset_status !=
		KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT)
		context->reset_status =
			KGSL_CTX_STAT_INNOCENT_CONTEXT_RESET_EXT;

	return 0;
}

static void mark_guilty_context(struct kgsl_device *device, unsigned int id)
{
	

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, _mark_context, &id);
	read_unlock(&device->context_lock);
}

static void cmdbatch_skip_ib(struct kgsl_cmdbatch *cmdbatch,
				unsigned int base)
{
	int i;

	for (i = 0; i < cmdbatch->ibcount; i++) {
		if (cmdbatch->ibdesc[i].gpuaddr == base) {
			cmdbatch->ibdesc[i].sizedwords = 0;
			if (base)
				return;
		}
	}
}

static void cmdbatch_skip_cmd(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int i;

	for (i = 1; i < count; i++) {
		if (replay[i]->context->id == cmdbatch->context->id) {
			replay[i]->fault_policy = replay[0]->fault_policy;
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			set_bit(KGSL_FT_SKIPCMD, &replay[i]->fault_recovery);
			break;
		}
	}

	if ((i == count) && drawctxt) {
		set_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->priv);
		drawctxt->fault_policy = replay[0]->fault_policy;
	}

	
	set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);
	cmdbatch->fault_recovery = 0;
}

static void cmdbatch_skip_frame(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int skip = 1;
	int i;

	for (i = 0; i < count; i++) {


		if (replay[i]->context->id != cmdbatch->context->id)
			continue;


		if (skip) {
			set_bit(CMDBATCH_FLAG_SKIP, &replay[i]->priv);

			if (replay[i]->flags & KGSL_CONTEXT_END_OF_FRAME)
				skip = 0;
		} else {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			return;
		}
	}


	if (skip && drawctxt)
		set_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->priv);


	if (!skip && drawctxt)
		set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE, &drawctxt->priv);
}

static void remove_invalidated_cmdbatches(struct kgsl_device *device,
		struct kgsl_cmdbatch **replay, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		struct kgsl_cmdbatch *cmd = replay[i];
		struct adreno_context *drawctxt;

		if (cmd == NULL)
			continue;

		drawctxt = ADRENO_CONTEXT(cmd->context);

		if (kgsl_context_detached(cmd->context) ||
			drawctxt->state == ADRENO_CONTEXT_STATE_INVALID) {
			replay[i] = NULL;

			kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
			kgsl_cancel_events_timestamp(device, cmd->context,
				cmd->timestamp);
			kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

			kgsl_cmdbatch_destroy(cmd);
		}
	}
}

static char _pidname[TASK_COMM_LEN];

static inline const char *_kgsl_context_comm(struct kgsl_context *context)
{
	struct task_struct *task = NULL;

	if (context)
		task = find_task_by_vpid(context->pid);

	if (task)
		get_task_comm(_pidname, task);
	else
		snprintf(_pidname, TASK_COMM_LEN, "unknown");

	return _pidname;
}

#define pr_fault(_d, _c, fmt, args...) \
		dev_err((_d)->dev, "%s[%d]: " fmt, \
		_kgsl_context_comm((_c)->context), \
		(_c)->context->pid, ##args)


static void adreno_fault_header(struct kgsl_device *device,
	struct kgsl_cmdbatch *cmdbatch)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int status, base, rptr, wptr, ib1base, ib2base, ib1sz, ib2sz;

	kgsl_regread(device,
			adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS),
			&status);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_BASE),
		&base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_RPTR),
		&rptr);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_RB_WPTR),
		&wptr);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB1_BASE),
		&ib1base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ),
		&ib1sz);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB2_BASE),
		&ib2base);
	kgsl_regread(device,
		adreno_getreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ),
		&ib2sz);

	trace_adreno_gpu_fault(cmdbatch->context->id, cmdbatch->timestamp,
		status, rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);

	pr_fault(device, cmdbatch,
		"gpu fault ctx %d ts %d status %8.8X rb %4.4x/%4.4x ib1 %8.8x/%4.4x ib2 %8.8x/%4.4x\n",
		cmdbatch->context->id, cmdbatch->timestamp, status,
		rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);
}

void adreno_fault_skipcmd_detached(struct kgsl_device *device,
				 struct adreno_context *drawctxt,
				 struct kgsl_cmdbatch *cmdbatch)
{
	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->priv) &&
			kgsl_context_detached(&drawctxt->base)) {
		pr_fault(device, cmdbatch, "gpu %s ctx %d\n",
			 "detached", cmdbatch->context->id);
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->priv);
	}
}

static int dispatcher_do_fault(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	unsigned int ptr;
	unsigned int reg, base;
	struct kgsl_cmdbatch **replay = NULL;
	struct kgsl_cmdbatch *cmdbatch;
	int ret, i, count = 0;
	int keepfault, fault, first = 0;
	int fault_pid = 0;

	fault = atomic_xchg(&dispatcher->fault, 0);
	keepfault = fault;
	if (fault == 0)
		return 0;
	if (dispatcher->inflight == 0) {
		KGSL_DRV_WARN(device,
		"dispatcher_do_fault with 0 inflight commands\n");
		if (kgsl_pwrctrl_isenabled(device))
			kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
		return 0;
	}

	
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	cmdbatch = dispatcher->cmdqueue[dispatcher->head];

	fault_pid = cmdbatch->context->pid;

	trace_adreno_cmdbatch_fault(cmdbatch, fault);


	if (fault & ADRENO_TIMEOUT_FAULT) {
		adreno_readreg(adreno_dev, ADRENO_REG_CP_ME_CNTL, &reg);
		reg |= (1 << 27) | (1 << 28);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, reg);
	}

	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BASE, &base);


	if (!test_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy)) {
		adreno_fault_header(device, cmdbatch);

		if (device->pm_dump_enable)
			kgsl_postmortem_dump(device, 0);

		kgsl_device_snapshot(device, 1);
	}

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	
	replay = kzalloc(sizeof(*replay) * dispatcher->inflight, GFP_KERNEL);

	if (replay == NULL) {
		unsigned int ptr = dispatcher->head;

		
		mark_guilty_context(device, 0);

		while (ptr != dispatcher->tail) {
			struct kgsl_context *context =
				dispatcher->cmdqueue[ptr]->context;

			adreno_drawctxt_invalidate(device, context);
			kgsl_cmdbatch_destroy(dispatcher->cmdqueue[ptr]);

			ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
		}


		count = 0;
		goto replay;
	}

	
	ptr = dispatcher->head;

	while (ptr != dispatcher->tail) {
		replay[count++] = dispatcher->cmdqueue[ptr];
		ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
	}


	cmdbatch = replay[0];

	if (test_bit(KGSL_FT_THROTTLE, &cmdbatch->fault_policy)) {
		if (time_after(jiffies, (cmdbatch->context->fault_time
				+ msecs_to_jiffies(_fault_throttle_time)))) {
			cmdbatch->context->fault_time = jiffies;
			cmdbatch->context->fault_count = 1;
		} else {
			cmdbatch->context->fault_count++;
			if (cmdbatch->context->fault_count >
					_fault_throttle_burst) {
				set_bit(KGSL_FT_DISABLE,
						&cmdbatch->fault_policy);
				pr_fault(device, cmdbatch,
					 "gpu fault threshold exceeded %d faults in %d msecs\n",
					 _fault_throttle_burst,
					 _fault_throttle_time);
			}
		}
	}


	if (test_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy) ||
		test_bit(KGSL_FT_TEMP_DISABLE, &cmdbatch->fault_policy)) {
		pr_fault(device, cmdbatch, "gpu skipped ctx %d ts %d\n",
			cmdbatch->context->id, cmdbatch->timestamp);

		mark_guilty_context(device, cmdbatch->context->id);
		adreno_drawctxt_invalidate(device, cmdbatch->context);
	}


	set_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy);


	if (fault & ADRENO_HARD_FAULT)
		clear_bit(KGSL_FT_REPLAY, &(cmdbatch->fault_policy));


	if (fault & ADRENO_TIMEOUT_FAULT)
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);


	if (test_bit(KGSL_CONTEXT_PAGEFAULT, &cmdbatch->context->priv)) {
		
		clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy);
		clear_bit(KGSL_CONTEXT_PAGEFAULT, &cmdbatch->context->priv);
	}


	
	if (test_and_clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_REPLAY));
		set_bit(KGSL_FT_REPLAY, &cmdbatch->fault_recovery);
		goto replay;
	}


	if (test_and_clear_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPIB));
		set_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_recovery);

		for (i = 0; i < count; i++) {
			if (replay[i] != NULL &&
				replay[i]->context->id == cmdbatch->context->id)
				cmdbatch_skip_ib(replay[i], base);
		}

		goto replay;
	}

	
	if (test_and_clear_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPCMD));

		
		cmdbatch_skip_cmd(cmdbatch, replay, count);

		goto replay;
	}

	if (test_and_clear_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch,
			BIT(KGSL_FT_SKIPFRAME));
		set_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_recovery);

		cmdbatch_skip_frame(cmdbatch, replay, count);
		goto replay;
	}

	

	pr_fault(device, cmdbatch, "gpu failed ctx %d ts %d\n",
		cmdbatch->context->id, cmdbatch->timestamp);

	
	mark_guilty_context(device, cmdbatch->context->id);

	
	adreno_drawctxt_invalidate(device, cmdbatch->context);


replay:
	
	dispatcher->inflight = 0;
	dispatcher->head = dispatcher->tail = 0;

	
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	ret = adreno_reset(device);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	
	fault = atomic_xchg(&dispatcher->fault, 0);

	
	BUG_ON(ret);

	
	remove_invalidated_cmdbatches(device, replay, count);

	
	for (i = 0; i < count; i++) {

		int ret;

		if (replay[i] == NULL)
			continue;


		if (first == 0) {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			first = 1;
		}


		set_bit(CMDBATCH_FLAG_WFI, &replay[i]->priv);

		ret = sendcmd(adreno_dev, replay[i]);


		if (ret) {
			pr_fault(device, replay[i],
				"gpu reset failed ctx %d ts %d\n",
				replay[i]->context->id, replay[i]->timestamp);

			
			mark_guilty_context(device, replay[i]->context->id);

			adreno_drawctxt_invalidate(device, replay[i]->context);
			remove_invalidated_cmdbatches(device, &replay[i],
				count - i);
		}
	}

	kfree(replay);

	adreno_fault_panic(device, fault_pid, keepfault);

	return 1;
}

static inline int cmdbatch_consumed(struct kgsl_cmdbatch *cmdbatch,
		unsigned int consumed, unsigned int retired)
{
	return ((timestamp_cmp(cmdbatch->timestamp, consumed) >= 0) &&
		(timestamp_cmp(retired, cmdbatch->timestamp) < 0));
}

static void _print_recovery(struct kgsl_device *device,
		struct kgsl_cmdbatch *cmdbatch)
{
	static struct {
		unsigned int mask;
		const char *str;
	} flags[] = { ADRENO_FT_TYPES };

	int i, nr = find_first_bit(&cmdbatch->fault_recovery, BITS_PER_LONG);
	char *result = "unknown";

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		if (flags[i].mask == BIT(nr)) {
			result = (char *) flags[i].str;
			break;
		}
	}

	pr_fault(device, cmdbatch,
		"gpu %s ctx %d ts %d policy %lX\n",
		result, cmdbatch->context->id, cmdbatch->timestamp,
		cmdbatch->fault_recovery);
}

static void adreno_dispatcher_work(struct work_struct *work)
{
	struct adreno_dispatcher *dispatcher =
		container_of(work, struct adreno_dispatcher, work);
	struct adreno_device *adreno_dev =
		container_of(dispatcher, struct adreno_device, dispatcher);
	struct kgsl_device *device = &adreno_dev->dev;
	int count = 0;
	int fault_handled = 0;

	mutex_lock(&dispatcher->mutex);

	while (dispatcher->head != dispatcher->tail) {
		uint32_t consumed, retired = 0;
		struct kgsl_cmdbatch *cmdbatch =
			dispatcher->cmdqueue[dispatcher->head];
		struct adreno_context *drawctxt;
		BUG_ON(cmdbatch == NULL);

		drawctxt = ADRENO_CONTEXT(cmdbatch->context);


		retired = kgsl_readtimestamp(device, cmdbatch->context,
				KGSL_TIMESTAMP_RETIRED);

		if ((timestamp_cmp(cmdbatch->timestamp, retired) <= 0)) {


			if (cmdbatch->fault_recovery != 0) {
				struct adreno_context *drawctxt =
					ADRENO_CONTEXT(cmdbatch->context);

				
				set_bit(ADRENO_CONTEXT_FAULT, &drawctxt->priv);

				_print_recovery(device, cmdbatch);
			}

			trace_adreno_cmdbatch_retired(cmdbatch,
				dispatcher->inflight - 1);

			
			dispatcher->inflight--;

			
			dispatcher->cmdqueue[dispatcher->head] = NULL;

			
			dispatcher->head = CMDQUEUE_NEXT(dispatcher->head,
				ADRENO_DISPATCH_CMDQUEUE_SIZE);

			
			kgsl_cmdbatch_destroy(cmdbatch);

			

			if (dispatcher->inflight > 0) {
				cmdbatch =
					dispatcher->cmdqueue[dispatcher->head];
				cmdbatch->expires = jiffies +
					msecs_to_jiffies(_cmdbatch_timeout);
			}

			count++;
			continue;
		}


		if (dispatcher_do_fault(device))
			goto done;
		fault_handled = 1;

		
		consumed = kgsl_readtimestamp(device, cmdbatch->context,
			KGSL_TIMESTAMP_CONSUMED);


		if (!adreno_dev->long_ib_detect ||
			drawctxt->base.flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE)
			break;


		if (time_is_after_jiffies(cmdbatch->expires))
			break;

		

		pr_fault(device, cmdbatch,
			"gpu timeout ctx %d ts %d\n",
			cmdbatch->context->id, cmdbatch->timestamp);

		adreno_set_gpu_fault(adreno_dev, ADRENO_TIMEOUT_FAULT);

		dispatcher_do_fault(device);
		fault_handled = 1;
		break;
	}

	if (!fault_handled && dispatcher_do_fault(device))
		goto done;

	if (dispatcher->inflight == 0 && count) {
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		queue_work(device->work_queue, &device->ts_expired_ws);
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	}

	
	if (dispatcher->inflight < _dispatcher_inflight)
		_adreno_dispatcher_issuecmds(adreno_dev);

done:
	
	if (dispatcher->inflight) {
		struct kgsl_cmdbatch *cmdbatch
			= dispatcher->cmdqueue[dispatcher->head];

		
		mod_timer(&dispatcher->timer, cmdbatch->expires);

		
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		kgsl_pwrscale_idle(device);
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	} else {
		
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		del_timer_sync(&dispatcher->fault_timer);

		if (test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
			kgsl_active_count_put(device);
			clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
		}

		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	}

	
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	kgsl_pwrscale_idle(device);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	mutex_unlock(&dispatcher->mutex);
}

void adreno_dispatcher_schedule(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	queue_work(device->work_queue, &dispatcher->work);
}

void adreno_dispatcher_queue_context(struct kgsl_device *device,
	struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	dispatcher_queue_context(adreno_dev, drawctxt);
	adreno_dispatcher_schedule(device);
}


void adreno_dispatcher_fault_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (adreno_dev->fast_hang_detect == 0)
		return;

	if (adreno_gpu_fault(adreno_dev)) {
		adreno_dispatcher_schedule(device);
		return;
	}


	if (!fault_detect_read_compare(device)) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_SOFT_FAULT);
		adreno_dispatcher_schedule(device);
	} else {
		mod_timer(&dispatcher->fault_timer,
			jiffies + msecs_to_jiffies(_fault_timer_interval));
	}
}

void adreno_dispatcher_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;
	struct kgsl_device *device = &adreno_dev->dev;

	adreno_dispatcher_schedule(device);
}
void adreno_dispatcher_irq_fault(struct kgsl_device *device)
{
	adreno_set_gpu_fault(ADRENO_DEVICE(device), ADRENO_HARD_FAULT);
	adreno_dispatcher_schedule(device);
}

void adreno_dispatcher_start(struct kgsl_device *device)
{
	complete_all(&device->cmdbatch_gate);

	
	adreno_dispatcher_schedule(device);
}

void adreno_dispatcher_stop(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);
}

void adreno_dispatcher_close(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	mutex_lock(&dispatcher->mutex);
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);

	while (dispatcher->head != dispatcher->tail) {
		kgsl_cmdbatch_destroy(dispatcher->cmdqueue[dispatcher->head]);
		dispatcher->head = (dispatcher->head + 1)
			% ADRENO_DISPATCH_CMDQUEUE_SIZE;
	}

	mutex_unlock(&dispatcher->mutex);

	kobject_put(&dispatcher->kobj);
}

struct dispatcher_attribute {
	struct attribute attr;
	ssize_t (*show)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, char *);
	ssize_t (*store)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, const char *buf,
			size_t count);
	unsigned int max;
	unsigned int *value;
};

#define DISPATCHER_UINT_ATTR(_name, _mode, _max, _value) \
	struct dispatcher_attribute dispatcher_attr_##_name =  { \
		.attr = { .name = __stringify(_name), .mode = _mode }, \
		.show = _show_uint, \
		.store = _store_uint, \
		.max = _max, \
		.value = &(_value), \
	}

#define to_dispatcher_attr(_a) \
	container_of((_a), struct dispatcher_attribute, attr)
#define to_dispatcher(k) container_of(k, struct adreno_dispatcher, kobj)

static ssize_t _store_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long val;
	int ret = kstrtoul(buf, 0, &val);

	if (ret)
		return ret;

	if (!val || (attr->max && (val > attr->max)))
		return -EINVAL;

	*((unsigned int *) attr->value) = val;
	return size;
}

static ssize_t _show_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
		*((unsigned int *) attr->value));
}

static DISPATCHER_UINT_ATTR(inflight, 0644, ADRENO_DISPATCH_CMDQUEUE_SIZE,
	_dispatcher_inflight);
static DISPATCHER_UINT_ATTR(context_cmdqueue_size, 0644,
	ADRENO_CONTEXT_CMDQUEUE_SIZE - 1, _context_cmdqueue_size);
static DISPATCHER_UINT_ATTR(context_burst_count, 0644, 0,
	_context_cmdbatch_burst);
static DISPATCHER_UINT_ATTR(cmdbatch_timeout, 0644, 0, _cmdbatch_timeout);
static DISPATCHER_UINT_ATTR(context_queue_wait, 0644, 0, _context_queue_wait);
static DISPATCHER_UINT_ATTR(fault_detect_interval, 0644, 0,
	_fault_timer_interval);
static DISPATCHER_UINT_ATTR(fault_throttle_time, 0644, 0,
	_fault_throttle_time);
static DISPATCHER_UINT_ATTR(fault_throttle_burst, 0644, 0,
	_fault_throttle_burst);

static struct attribute *dispatcher_attrs[] = {
	&dispatcher_attr_inflight.attr,
	&dispatcher_attr_context_cmdqueue_size.attr,
	&dispatcher_attr_context_burst_count.attr,
	&dispatcher_attr_cmdbatch_timeout.attr,
	&dispatcher_attr_context_queue_wait.attr,
	&dispatcher_attr_fault_detect_interval.attr,
	&dispatcher_attr_fault_throttle_time.attr,
	&dispatcher_attr_fault_throttle_burst.attr,
	NULL,
};

static ssize_t dispatcher_sysfs_show(struct kobject *kobj,
				   struct attribute *attr, char *buf)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->show)
		ret = pattr->show(dispatcher, pattr, buf);

	return ret;
}

static ssize_t dispatcher_sysfs_store(struct kobject *kobj,
				    struct attribute *attr,
				    const char *buf, size_t count)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->store)
		ret = pattr->store(dispatcher, pattr, buf, count);

	return ret;
}

static void dispatcher_sysfs_release(struct kobject *kobj)
{
}

static const struct sysfs_ops dispatcher_sysfs_ops = {
	.show = dispatcher_sysfs_show,
	.store = dispatcher_sysfs_store
};

static struct kobj_type ktype_dispatcher = {
	.sysfs_ops = &dispatcher_sysfs_ops,
	.default_attrs = dispatcher_attrs,
	.release = dispatcher_sysfs_release
};

int adreno_dispatcher_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	memset(dispatcher, 0, sizeof(*dispatcher));

	mutex_init(&dispatcher->mutex);

	setup_timer(&dispatcher->timer, adreno_dispatcher_timer,
		(unsigned long) adreno_dev);

	setup_timer(&dispatcher->fault_timer, adreno_dispatcher_fault_timer,
		(unsigned long) adreno_dev);

	INIT_WORK(&dispatcher->work, adreno_dispatcher_work);

	plist_head_init(&dispatcher->pending);
	spin_lock_init(&dispatcher->plist_lock);

	ret = kobject_init_and_add(&dispatcher->kobj, &ktype_dispatcher,
		&device->dev->kobj, "dispatch");

	return ret;
}

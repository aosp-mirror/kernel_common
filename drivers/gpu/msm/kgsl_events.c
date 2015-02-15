/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/module.h>
#include <kgsl_device.h>

#include "kgsl_trace.h"

static inline struct list_head *_get_list_head(struct kgsl_device *device,
		struct kgsl_context *context)
{
	return (context) ? &context->events : &device->events;
}

static void _add_event_to_list(struct list_head *head, struct kgsl_event *event)
{
	struct list_head *n;

	for (n = head->next; n != head; n = n->next) {
		struct kgsl_event *e =
			list_entry(n, struct kgsl_event, list);

		if (timestamp_cmp(e->timestamp, event->timestamp) > 0) {
			list_add(&event->list, n->prev);
			break;
		}
	}

	if (n == head)
		list_add_tail(&event->list, head);
}

static inline void _do_signal_event(struct kgsl_device *device,
		struct kgsl_event *event, unsigned int timestamp,
		unsigned int type)
{
	int id = event->context ? event->context->id : KGSL_MEMSTORE_GLOBAL;

	trace_kgsl_fire_event(id, timestamp, type, jiffies - event->created,
		event->func);

	if (event->func)
		event->func(device, event->priv, id, timestamp, type);

	list_del(&event->list);
	kgsl_context_put(event->context);
	kfree(event);
}

static void _retire_events(struct kgsl_device *device,
		struct list_head *head, unsigned int timestamp)
{
	struct kgsl_event *event, *tmp;

	list_for_each_entry_safe(event, tmp, head, list) {
		if (timestamp_cmp(timestamp, event->timestamp) < 0)
			break;

		_do_signal_event(device, event, event->timestamp,
			KGSL_EVENT_TIMESTAMP_RETIRED);
	}
}

static struct kgsl_event *_find_event(struct kgsl_device *device,
		struct list_head *head, unsigned int timestamp,
		kgsl_event_func func, void *priv)
{
	struct kgsl_event *event, *tmp;

	list_for_each_entry_safe(event, tmp, head, list) {
		if (timestamp == event->timestamp && func == event->func &&
			event->priv == priv)
			return event;
	}

	return NULL;
}

static void _signal_event(struct kgsl_device *device,
		struct list_head *head, unsigned int timestamp,
		unsigned int cur, unsigned int type)
{
	struct kgsl_event *event, *tmp;

	list_for_each_entry_safe(event, tmp, head, list) {
		if (timestamp_cmp(timestamp, event->timestamp) == 0)
			_do_signal_event(device, event, cur, type);
	}
}

static void _signal_events(struct kgsl_device *device,
		struct list_head *head, uint32_t timestamp,
		unsigned int type)
{
	struct kgsl_event *event, *tmp;

	list_for_each_entry_safe(event, tmp, head, list)
		_do_signal_event(device, event, timestamp, type);

}

void kgsl_signal_event(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp,
		unsigned int type)
{
	struct list_head *head = _get_list_head(device, context);
	uint32_t cur;

	BUG_ON(!mutex_is_locked(&device->mutex));

	cur = kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_RETIRED);
	_signal_event(device, head, timestamp, cur, type);

	if (context && list_empty(&context->events))
		list_del_init(&context->events_list);
}
EXPORT_SYMBOL(kgsl_signal_event);

void kgsl_signal_events(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int type)
{
	struct list_head *head = _get_list_head(device, context);
	uint32_t cur;

	BUG_ON(!mutex_is_locked(&device->mutex));


	cur = kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_RETIRED);

	_signal_events(device, head, cur, type);


	if (context)
		list_del_init(&context->events_list);
}
EXPORT_SYMBOL(kgsl_signal_events);

int kgsl_add_event(struct kgsl_device *device, u32 id, u32 ts,
	kgsl_event_func func, void *priv, void *owner)
{
	struct kgsl_event *event;
	unsigned int queued = 0, cur_ts;
	struct kgsl_context *context = NULL;

	BUG_ON(!mutex_is_locked(&device->mutex));

	if (func == NULL)
		return -EINVAL;

	if (id != KGSL_MEMSTORE_GLOBAL) {
		context = kgsl_context_get(device, id);
		if (context == NULL)
			return -EINVAL;
	}

	if (context == NULL ||
		((context->flags & KGSL_CONTEXT_USER_GENERATED_TS) == 0)) {

		queued = kgsl_readtimestamp(device, context,
						KGSL_TIMESTAMP_QUEUED);

		if (timestamp_cmp(ts, queued) > 0) {
			kgsl_context_put(context);
			return -EINVAL;
		}
	}

	cur_ts = kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_RETIRED);


	if (timestamp_cmp(cur_ts, ts) >= 0) {
		trace_kgsl_fire_event(id, cur_ts, ts, 0, func);

		func(device, priv, id, ts, KGSL_EVENT_TIMESTAMP_RETIRED);
		kgsl_context_put(context);
		return 0;
	}

	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (event == NULL) {
		kgsl_context_put(context);
		return -ENOMEM;
	}

	event->context = context;
	event->timestamp = ts;
	event->priv = priv;
	event->func = func;
	event->owner = owner;
	event->created = jiffies;

	trace_kgsl_register_event(id, ts, func);

	

	if (context) {
		_add_event_to_list(&context->events, event);


		if (list_empty(&context->events_list))
			list_add_tail(&context->events_list,
				&device->events_pending_list);

	} else
		_add_event_to_list(&device->events, event);

	queue_work(device->work_queue, &device->ts_expired_ws);
	return 0;
}
EXPORT_SYMBOL(kgsl_add_event);

void kgsl_cancel_events(struct kgsl_device *device, void *owner)
{
	struct kgsl_event *event, *event_tmp;
	unsigned int cur;

	BUG_ON(!mutex_is_locked(&device->mutex));

	cur = kgsl_readtimestamp(device, NULL, KGSL_TIMESTAMP_RETIRED);

	list_for_each_entry_safe(event, event_tmp, &device->events, list) {
		if (event->owner != owner)
			continue;

		_do_signal_event(device, event, cur, KGSL_EVENT_CANCELLED);
	}
}
EXPORT_SYMBOL(kgsl_cancel_events);


void kgsl_cancel_event(struct kgsl_device *device, struct kgsl_context *context,
		unsigned int timestamp, kgsl_event_func func,
		void *priv)
{
	struct kgsl_event *event;
	struct list_head *head;

	BUG_ON(!mutex_is_locked(&device->mutex));

	head = _get_list_head(device, context);

	event = _find_event(device, head, timestamp, func, priv);

	if (event) {
		unsigned int cur = kgsl_readtimestamp(device, context,
			KGSL_TIMESTAMP_RETIRED);

		_do_signal_event(device, event, cur, KGSL_EVENT_CANCELLED);
	}
}
EXPORT_SYMBOL(kgsl_cancel_event);

static inline int _mark_next_event(struct kgsl_device *device,
		struct list_head *head)
{
	struct kgsl_event *event;

	if (!list_empty(head)) {
		event = list_first_entry(head, struct kgsl_event, list);


		if (device->ftbl->next_event)
			return device->ftbl->next_event(device, event);
	}

	return 0;
}

static int kgsl_process_context_events(struct kgsl_device *device,
		struct kgsl_context *context)
{
	while (1) {
		unsigned int timestamp = kgsl_readtimestamp(device, context,
			KGSL_TIMESTAMP_RETIRED);

		_retire_events(device, &context->events, timestamp);


		if (!_mark_next_event(device, &context->events))
			break;
	}


	return list_empty(&context->events) ? 0 : 1;
}

void kgsl_process_events(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
		ts_expired_ws);
	struct kgsl_context *context, *tmp;
	uint32_t timestamp;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	timestamp = kgsl_readtimestamp(device, NULL, KGSL_TIMESTAMP_RETIRED);
	_retire_events(device, &device->events, timestamp);
	_mark_next_event(device, &device->events);

	
	list_for_each_entry_safe(context, tmp, &device->events_pending_list,
		events_list) {

		if (_kgsl_context_get(context)) {

			if (kgsl_process_context_events(device, context) == 0)
				list_del_init(&context->events_list);
			kgsl_context_put(context);
		}
	}

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}
EXPORT_SYMBOL(kgsl_process_events);

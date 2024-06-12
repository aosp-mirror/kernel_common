/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2024 Google, Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM rust_binder

#if !defined(_RUST_BINDER_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _RUST_BINDER_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(rust_binder_ioctl,
	TP_PROTO(unsigned int cmd, unsigned long arg),
	TP_ARGS(cmd, arg),

	TP_STRUCT__entry(
		__field(unsigned int, cmd)
		__field(unsigned long, arg)
	),
	TP_fast_assign(
		__entry->cmd = cmd;
		__entry->arg = arg;
	),
	TP_printk("cmd=0x%x arg=0x%lx", __entry->cmd, __entry->arg)
);

DECLARE_EVENT_CLASS(rust_binder_function_return_class,
	TP_PROTO(int ret),
	TP_ARGS(ret),
	TP_STRUCT__entry(
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->ret = ret;
	),
	TP_printk("ret=%d", __entry->ret)
);

#define DEFINE_RBINDER_FUNCTION_RETURN_EVENT(name)	\
DEFINE_EVENT(rust_binder_function_return_class, name,	\
	TP_PROTO(int ret), \
	TP_ARGS(ret))

DEFINE_RBINDER_FUNCTION_RETURN_EVENT(rust_binder_ioctl_done);
DEFINE_RBINDER_FUNCTION_RETURN_EVENT(rust_binder_read_done);
DEFINE_RBINDER_FUNCTION_RETURN_EVENT(rust_binder_write_done);

TRACE_EVENT(rust_binder_wait_for_work,
	TP_PROTO(bool proc_work, bool transaction_stack, bool thread_todo),
	TP_ARGS(proc_work, transaction_stack, thread_todo),

	TP_STRUCT__entry(
		__field(bool, proc_work)
		__field(bool, transaction_stack)
		__field(bool, thread_todo)
	),
	TP_fast_assign(
		__entry->proc_work = proc_work;
		__entry->transaction_stack = transaction_stack;
		__entry->thread_todo = thread_todo;
	),
	TP_printk("proc_work=%d transaction_stack=%d thread_todo=%d",
		  __entry->proc_work, __entry->transaction_stack,
		  __entry->thread_todo)
);

TRACE_EVENT(rust_binder_transaction,
	TP_PROTO(bool reply, rust_binder_transaction t),
	TP_ARGS(reply, t),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, target_node)
		__field(int, from_proc)
		__field(int, to_proc)
		__field(int, reply)
		__field(unsigned int, code)
		__field(unsigned int, flags)
	),
	TP_fast_assign(
		rust_binder_thread from_thread = rust_binder_transaction_from_thread(t);
		rust_binder_process from = rust_binder_thread_proc(from_thread);
		rust_binder_process to = rust_binder_transaction_to_proc(t);
		rust_binder_node target_node = rust_binder_transaction_target_node(t);

		__entry->debug_id = rust_binder_transaction_debug_id(t);
		__entry->target_node = target_node ? rust_binder_node_debug_id(target_node) : 0;
		__entry->from_proc = rust_binder_process_task(from)->pid;
		__entry->to_proc = rust_binder_process_task(to)->pid;
		__entry->reply = reply;
		__entry->code = rust_binder_transaction_code(t);
		__entry->flags = rust_binder_transaction_flags(t);
	),
	TP_printk("transaction=%d target_node=%d dest_proc=%d from_proc=%d reply=%d flags=0x%x code=0x%x",
		__entry->debug_id, __entry->target_node, __entry->to_proc,
		__entry->from_proc, __entry->reply, __entry->flags,
		__entry->code)
);

TRACE_EVENT(rust_binder_transaction_thread_selected,
	TP_PROTO(rust_binder_transaction t, rust_binder_thread thread),
	TP_ARGS(t, thread),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, to_thread)
	),
	TP_fast_assign(
		__entry->debug_id = rust_binder_transaction_debug_id(t);
		__entry->to_thread = rust_binder_thread_id(thread);
	),
	TP_printk("transaction=%d thread=%d", __entry->debug_id, __entry->to_thread)
);

TRACE_EVENT(rust_binder_transaction_received,
	TP_PROTO(rust_binder_transaction t),
	TP_ARGS(t),
	TP_STRUCT__entry(
		__field(int, debug_id)
	),
	TP_fast_assign(
		__entry->debug_id = rust_binder_transaction_debug_id(t);
	),
	TP_printk("transaction=%d", __entry->debug_id)
);

#endif /* _RUST_BINDER_TRACE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

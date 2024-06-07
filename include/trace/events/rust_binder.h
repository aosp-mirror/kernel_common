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

TRACE_EVENT(rust_binder_transaction,
	TP_PROTO(int debug_id, bool reply),
	TP_ARGS(debug_id, reply),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, reply)
	),
	TP_fast_assign(
		__entry->debug_id = debug_id;
		__entry->reply = reply;
	),
	TP_printk("transaction=%d reply=%d", __entry->debug_id, __entry->reply)
);

#endif /* _RUST_BINDER_TRACE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

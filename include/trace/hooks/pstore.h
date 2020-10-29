/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM pstore

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_PSTORE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_PSTORE_H

#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>

#if defined(CONFIG_TRACEPOINTS) && defined(CONFIG_ANDROID_VENDOR_HOOKS)
DECLARE_HOOK(android_vh_pstore_console_mkfile,
	TP_PROTO(loff_t *i_size),
	TP_ARGS(i_size));

DECLARE_HOOK(android_vh_pstore_console_read,
	TP_PROTO(void __user *to, size_t count,
		 loff_t *ppos, const void *from, size_t available, ssize_t *ret),
	TP_ARGS(to, count, ppos, from, available, ret));
#else
#define trace_android_vh_pstore_console_mkfile(i_size)
#define trace_android_vh_pstore_console_read(to, count, ppos, from, available, ret)
#endif

#endif /* _TRACE_HOOK_PSTORE_H */
/* This part must be outside protection */
#include <trace/define_trace.h>

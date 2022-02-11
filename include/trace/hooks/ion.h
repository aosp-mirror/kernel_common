/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ion
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_ION_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_ION_H
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>

struct ion_buffer;
DECLARE_HOOK(android_vh_ion_buffer_release,
		TP_PROTO(struct ion_buffer *data),
		TP_ARGS(data));
#endif /* _TRACE_HOOK_ION_H */
/* This part must be outside protection */
#include <trace/define_trace.h>

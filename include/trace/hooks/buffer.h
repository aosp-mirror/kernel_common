/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM buffer

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_BUFFER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_BUFFER_H

#include <linux/types.h>
#include <trace/hooks/vendor_hooks.h>

DECLARE_HOOK(android_vh_bh_lru_install,
	TP_PROTO(struct page *page, bool *flush),
	TP_ARGS(page, flush));

/* macro versions of hooks are no longer required */

#endif /* _TRACE_HOOK_BUFFER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

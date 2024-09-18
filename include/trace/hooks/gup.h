/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM gup
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks
#if !defined(_TRACE_HOOK_GUP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_GUP_H
#include <trace/hooks/vendor_hooks.h>

struct page;

DECLARE_HOOK(android_vh_try_grab_compound_head,
       TP_PROTO(struct page *page, int refs, unsigned int flags, bool *ret),
       TP_ARGS(page, refs, flags, ret));

DECLARE_HOOK(android_vh___get_user_pages_remote,
       TP_PROTO(int *locked, unsigned int *gup_flags, struct page **pages),
       TP_ARGS(locked, gup_flags, pages));

DECLARE_HOOK(android_vh_get_user_pages,
       TP_PROTO(unsigned int *gup_flags, struct page **pages),
       TP_ARGS(gup_flags, pages));

DECLARE_HOOK(android_vh_internal_get_user_pages_fast,
       TP_PROTO(unsigned int *gup_flags, struct page **pages),
       TP_ARGS(gup_flags, pages));

DECLARE_HOOK(android_vh_pin_user_pages,
       TP_PROTO(unsigned int *gup_flags, struct page **pages),
       TP_ARGS(gup_flags, pages));
#endif /* _TRACE_HOOK_GUP_H */
/* This part must be outside protection */
#include <trace/define_trace.h>

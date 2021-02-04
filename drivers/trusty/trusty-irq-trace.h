/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Google, Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM trusty

#if !defined(_TRUSTY_IRQ_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRUSTY_IRQ_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(trusty_irq,
	TP_PROTO(int irq),
	TP_ARGS(irq),
	TP_STRUCT__entry(
		__field(int, irq)
	),
	TP_fast_assign(
		__entry->irq = irq;
	),
	TP_printk("irq=%d", __entry->irq)
);

#endif /* _TRUSTY_IRQ_TRACE_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trusty-irq-trace
#include <trace/define_trace.h>

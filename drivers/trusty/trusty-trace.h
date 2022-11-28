/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Google, Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM trusty

#if !defined(_TRUSTY_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRUSTY_TRACE_H

#include <linux/tracepoint.h>
#include <linux/trusty/smcall.h>

/* One of the requirements of checkpatch.pl script is to have "parentheses
 * to sheild macro arguments from the effects of operator precedence."
 * Otherwise checkpatch.pl script shall throw below error
 * "ERROR: Macros with complex values should be enclosed in parentheses"
 *
 * Hence extra parenthese are used to avoid the above checkpatch error.
 * And the extra paentheses are removed using DELETE_PAREN to avoid
 * compilation errors.
 */
#define _DELETE_PAREN(args...)	args
#define DELETE_PAREN(arg)	_DELETE_PAREN(_DELETE_PAREN arg)

/*
 * SMC fast call and std call numbers from linux/trusty/smcall.h
 */
#define SMC_NAME_LIST (			\
	smc_name(SC_RESTART_LAST)	\
	smc_name(SC_LOCKED_NOP)		\
	smc_name(SC_RESTART_FIQ)	\
	smc_name(SC_NOP)		\
	smc_name(FC_RESERVED)		\
	smc_name(FC_FIQ_EXIT)		\
	smc_name(FC_REQUEST_FIQ)	\
	smc_name(FC_GET_NEXT_IRQ)	\
	smc_name(FC_CPU_SUSPEND)	\
	smc_name(FC_CPU_RESUME)		\
	smc_name(FC_AARCH_SWITCH)	\
	smc_name(FC_GET_VERSION_STR)	\
	smc_name(FC_API_VERSION)	\
	smc_name(SC_VIRTIO_GET_DESCR)	\
	smc_name(SC_VIRTIO_START)	\
	smc_name(SC_VIRTIO_STOP)	\
	smc_name(SC_VDEV_RESET)		\
	smc_name(SC_VDEV_KICK_VQ)	\
	smc_name_end(NC_VDEV_KICK_VQ)	\
	)

#undef smc_name
#undef smc_name_end

#define smc_name_define_enum(x)	(TRACE_DEFINE_ENUM(SMC_##x);)
#define smc_name(x)		DELETE_PAREN(smc_name_define_enum(x))
#define smc_name_end(x)		DELETE_PAREN(smc_name_define_enum(x))

DELETE_PAREN(SMC_NAME_LIST)

#undef smc_name
#undef smc_name_end

#define smc_name(x)	{ SMC_##x, #x },
#define smc_name_end(x)	{ SMC_##x, #x }

#define smc_show_name(x)	\
	__print_symbolic(x, DELETE_PAREN(SMC_NAME_LIST))

DECLARE_EVENT_CLASS(trusty_smc4_class,
	TP_PROTO(unsigned long r0, unsigned long r1, unsigned long r2,
		 unsigned long r3),
	TP_ARGS(r0, r1, r2, r3),
	TP_STRUCT__entry(
		__field(unsigned long, r0)
		__field(unsigned long, r1)
		__field(unsigned long, r2)
		__field(unsigned long, r3)
	),
	TP_fast_assign(
		__entry->r0 = r0;
		__entry->r1 = r1;
		__entry->r2 = r2;
		__entry->r3 = r3;
	),
	TP_printk("smcnr=%s r0=0x%lx r1=0x%lx r2=0x%lx r3=0x%lx", smc_show_name(__entry->r0),
		__entry->r0, __entry->r1, __entry->r2, __entry->r3)
);

#define DEFINE_TRUSTY_SMC4_EVENT(name)	\
DEFINE_EVENT(trusty_smc4_class, name,	\
	TP_PROTO(unsigned long r0, unsigned long r1, unsigned long r2, \
		 unsigned long r3), \
	TP_ARGS(r0, r1, r2, r3))

DEFINE_TRUSTY_SMC4_EVENT(trusty_std_call32);
DEFINE_TRUSTY_SMC4_EVENT(trusty_smc);

DECLARE_EVENT_CLASS(trusty_smc_return_class,
	TP_PROTO(unsigned long ret),
	TP_ARGS(ret),
	TP_STRUCT__entry(
		__field(unsigned long, ret)
	),
	TP_fast_assign(
		__entry->ret = ret;
	),
	TP_printk("ret:ulong=%lu (0x%lx) (ret:s32=%d)", __entry->ret,
		__entry->ret, (s32)__entry->ret)
);

#define DEFINE_TRUSTY_SMC_RETURN_EVENT(name)	\
DEFINE_EVENT(trusty_smc_return_class, name,	\
	TP_PROTO(unsigned long ret), \
	TP_ARGS(ret))

DEFINE_TRUSTY_SMC_RETURN_EVENT(trusty_std_call32_done);
DEFINE_TRUSTY_SMC_RETURN_EVENT(trusty_smc_done);

TRACE_EVENT(trusty_share_memory,
	TP_PROTO(size_t len, unsigned int nents, bool lend),
	TP_ARGS(len, nents, lend),
	TP_STRUCT__entry(
		__field(size_t, len)
		__field(unsigned int, nents)
		__field(bool, lend)
	),
	TP_fast_assign(
		__entry->len = len;
		__entry->nents = nents;
		__entry->lend = lend;
	),
	TP_printk("len=%zu, nents=%u, lend=%u", __entry->len, __entry->nents, __entry->lend)
);

TRACE_EVENT(trusty_share_memory_done,
	TP_PROTO(size_t len, unsigned int nents, bool lend, u64 handle, int ret),
	TP_ARGS(len, nents, lend, handle, ret),
	TP_STRUCT__entry(
		__field(size_t, len)
		__field(unsigned int, nents)
		__field(bool, lend)
		__field(u64, handle)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->len = len;
		__entry->nents = nents;
		__entry->lend = lend;
		__entry->handle = handle;
		__entry->ret = ret;
	),
	TP_printk("len=%zu, nents=%u, lend=%u, ffa_handle=0x%llx, ret=%d", __entry->len,
		__entry->nents, __entry->lend, __entry->handle, __entry->ret)
);

TRACE_EVENT(trusty_enqueue_nop,
	TP_PROTO(struct trusty_nop *nop),
	TP_ARGS(nop),
	TP_STRUCT__entry(
		__field(u32, arg1)
		__field(u32, arg2)
		__field(u32, arg3)
	),
	TP_fast_assign(
		__entry->arg1 = nop ? nop->args[0] : 0U;
		__entry->arg2 = nop ? nop->args[1] : 0U;
		__entry->arg3 = nop ? nop->args[2] : 0U;
	),
	TP_printk("arg1=0x%x, arg2=0x%x, arg3=0x%x", __entry->arg1, __entry->arg2, __entry->arg3)
);

#define CPUNICE_CAUSE_LIST (			\
	cpu_nice(CAUSE_DEFAULT)	\
	cpu_nice(CAUSE_USE_HIGH_WQ)		\
	cpu_nice(CAUSE_TRUSTY_REQ)	\
	cpu_nice_end(CAUSE_NOP_ESCALATE)	\
	)

#undef cpu_nice
#undef cpu_nice_end

#define cpu_nice_define_enum(x)	(TRACE_DEFINE_ENUM(CPUNICE_##x);)
#define cpu_nice(x)		DELETE_PAREN(cpu_nice_define_enum(x))
#define cpu_nice_end(x)		DELETE_PAREN(cpu_nice_define_enum(x))

DELETE_PAREN(CPUNICE_CAUSE_LIST)

#undef cpu_nice
#undef cpu_nice_end

#define cpu_nice(x)	{ CPUNICE_##x, #x },
#define cpu_nice_end(x)	{ CPUNICE_##x, #x }

#define cpunice_show_cause(x)	\
	__print_symbolic(x, DELETE_PAREN(CPUNICE_CAUSE_LIST))

TRACE_EVENT(trusty_change_cpu_nice,
	TP_PROTO(s32 cur_nice, s32 req_nice, u32 cause_id),
	TP_ARGS(cur_nice, req_nice, cause_id),
	TP_STRUCT__entry(
		__field(s32, cur_nice)
		__field(s32, req_nice)
		__field(u32, cause_id)
	),
	TP_fast_assign(
		__entry->cur_nice = cur_nice;
		__entry->req_nice = req_nice;
		__entry->cause_id = cause_id;
	),
	TP_printk("%d->%d (%s)", __entry->cur_nice, __entry->req_nice,
			cpunice_show_cause(__entry->cause_id))
);

TRACE_EVENT(trusty_reclaim_memory,
	TP_PROTO(u64 id),
	TP_ARGS(id),
	TP_STRUCT__entry(
		__field(u64, id)
	),
	TP_fast_assign(
		__entry->id = id;
	),
	TP_printk("id=%llu", __entry->id)
);

TRACE_EVENT(trusty_reclaim_memory_done,
	TP_PROTO(u64 id, int ret),
	TP_ARGS(id, ret),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->ret = ret;
	),
	TP_printk("id=%llu ret=%d (0x%x)", __entry->id, __entry->ret, __entry->ret)
);

#endif /* _TRUSTY_TRACE_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trusty-trace
#include <trace/define_trace.h>

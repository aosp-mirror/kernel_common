/*
 * Per core/cpu state
 *
 * Used to coordinate shared registers between HT threads or
 * among events on a single PMU.
 */

#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/export.h>

#include <asm/hardirq.h>
#include <asm/apic.h>

#include "perf_event.h"

/*
 * Intel PerfMon, used on Core and later.
 */
static u64 intel_perfmon_event_map[PERF_COUNT_HW_MAX] __read_mostly =
{
  [PERF_COUNT_HW_CPU_CYCLES]		= 0x003c,
  [PERF_COUNT_HW_INSTRUCTIONS]		= 0x00c0,
  [PERF_COUNT_HW_CACHE_REFERENCES]	= 0x4f2e,
  [PERF_COUNT_HW_CACHE_MISSES]		= 0x412e,
  [PERF_COUNT_HW_BRANCH_INSTRUCTIONS]	= 0x00c4,
  [PERF_COUNT_HW_BRANCH_MISSES]		= 0x00c5,
  [PERF_COUNT_HW_BUS_CYCLES]		= 0x013c,
  [PERF_COUNT_HW_REF_CPU_CYCLES]	= 0x0300, /* pseudo-encoding */
};

static struct event_constraint intel_core_event_constraints[] __read_mostly =
{
	INTEL_EVENT_CONSTRAINT(0x11, 0x2), /* FP_ASSIST */
	INTEL_EVENT_CONSTRAINT(0x12, 0x2), /* MUL */
	INTEL_EVENT_CONSTRAINT(0x13, 0x2), /* DIV */
	INTEL_EVENT_CONSTRAINT(0x14, 0x1), /* CYCLES_DIV_BUSY */
	INTEL_EVENT_CONSTRAINT(0x19, 0x2), /* DELAYED_BYPASS */
	INTEL_EVENT_CONSTRAINT(0xc1, 0x1), /* FP_COMP_INSTR_RET */
	EVENT_CONSTRAINT_END
};

static struct event_constraint intel_core2_event_constraints[] __read_mostly =
{
	FIXED_EVENT_CONSTRAINT(0x00c0, 0), /* INST_RETIRED.ANY */
	FIXED_EVENT_CONSTRAINT(0x003c, 1), /* CPU_CLK_UNHALTED.CORE */
	FIXED_EVENT_CONSTRAINT(0x0300, 2), /* CPU_CLK_UNHALTED.REF */
	INTEL_EVENT_CONSTRAINT(0x10, 0x1), /* FP_COMP_OPS_EXE */
	INTEL_EVENT_CONSTRAINT(0x11, 0x2), /* FP_ASSIST */
	INTEL_EVENT_CONSTRAINT(0x12, 0x2), /* MUL */
	INTEL_EVENT_CONSTRAINT(0x13, 0x2), /* DIV */
	INTEL_EVENT_CONSTRAINT(0x14, 0x1), /* CYCLES_DIV_BUSY */
	INTEL_EVENT_CONSTRAINT(0x18, 0x1), /* IDLE_DURING_DIV */
	INTEL_EVENT_CONSTRAINT(0x19, 0x2), /* DELAYED_BYPASS */
	INTEL_EVENT_CONSTRAINT(0xa1, 0x1), /* RS_UOPS_DISPATCH_CYCLES */
	INTEL_EVENT_CONSTRAINT(0xc9, 0x1), /* ITLB_MISS_RETIRED (T30-9) */
	INTEL_EVENT_CONSTRAINT(0xcb, 0x1), /* MEM_LOAD_RETIRED */
	EVENT_CONSTRAINT_END
};

static struct event_constraint intel_nehalem_event_constraints[] __read_mostly =
{
	FIXED_EVENT_CONSTRAINT(0x00c0, 0), /* INST_RETIRED.ANY */
	FIXED_EVENT_CONSTRAINT(0x003c, 1), /* CPU_CLK_UNHALTED.CORE */
	FIXED_EVENT_CONSTRAINT(0x0300, 2), /* CPU_CLK_UNHALTED.REF */
	INTEL_EVENT_CONSTRAINT(0x40, 0x3), /* L1D_CACHE_LD */
	INTEL_EVENT_CONSTRAINT(0x41, 0x3), /* L1D_CACHE_ST */
	INTEL_EVENT_CONSTRAINT(0x42, 0x3), /* L1D_CACHE_LOCK */
	INTEL_EVENT_CONSTRAINT(0x43, 0x3), /* L1D_ALL_REF */
	INTEL_EVENT_CONSTRAINT(0x48, 0x3), /* L1D_PEND_MISS */
	INTEL_EVENT_CONSTRAINT(0x4e, 0x3), /* L1D_PREFETCH */
	INTEL_EVENT_CONSTRAINT(0x51, 0x3), /* L1D */
	INTEL_EVENT_CONSTRAINT(0x63, 0x3), /* CACHE_LOCK_CYCLES */
	EVENT_CONSTRAINT_END
};

static struct extra_reg intel_nehalem_extra_regs[] __read_mostly =
{
	INTEL_EVENT_EXTRA_REG(0xb7, MSR_OFFCORE_RSP_0, 0xffff, RSP_0),
	EVENT_EXTRA_END
};

static struct event_constraint intel_westmere_event_constraints[] __read_mostly =
{
	FIXED_EVENT_CONSTRAINT(0x00c0, 0), /* INST_RETIRED.ANY */
	FIXED_EVENT_CONSTRAINT(0x003c, 1), /* CPU_CLK_UNHALTED.CORE */
	FIXED_EVENT_CONSTRAINT(0x0300, 2), /* CPU_CLK_UNHALTED.REF */
	INTEL_EVENT_CONSTRAINT(0x51, 0x3), /* L1D */
	INTEL_EVENT_CONSTRAINT(0x60, 0x1), /* OFFCORE_REQUESTS_OUTSTANDING */
	INTEL_EVENT_CONSTRAINT(0x63, 0x3), /* CACHE_LOCK_CYCLES */
	INTEL_EVENT_CONSTRAINT(0xb3, 0x1), /* SNOOPQ_REQUEST_OUTSTANDING */
	EVENT_CONSTRAINT_END
};

static struct event_constraint intel_snb_event_constraints[] __read_mostly =
{
	FIXED_EVENT_CONSTRAINT(0x00c0, 0), /* INST_RETIRED.ANY */
	FIXED_EVENT_CONSTRAINT(0x003c, 1), /* CPU_CLK_UNHALTED.CORE */
	FIXED_EVENT_CONSTRAINT(0x0300, 2), /* CPU_CLK_UNHALTED.REF */
	INTEL_EVENT_CONSTRAINT(0x48, 0x4), /* L1D_PEND_MISS.PENDING */
	INTEL_UEVENT_CONSTRAINT(0x01c0, 0x2), /* INST_RETIRED.PREC_DIST */
	INTEL_EVENT_CONSTRAINT(0xcd, 0x8), /* MEM_TRANS_RETIRED.LOAD_LATENCY */
	EVENT_CONSTRAINT_END
};

static struct extra_reg intel_westmere_extra_regs[] __read_mostly =
{
	INTEL_EVENT_EXTRA_REG(0xb7, MSR_OFFCORE_RSP_0, 0xffff, RSP_0),
	INTEL_EVENT_EXTRA_REG(0xbb, MSR_OFFCORE_RSP_1, 0xffff, RSP_1),
	EVENT_EXTRA_END
};

static struct event_constraint intel_v1_event_constraints[] __read_mostly =
{
	EVENT_CONSTRAINT_END
};

static struct event_constraint intel_gen_event_constraints[] __read_mostly =
{
	FIXED_EVENT_CONSTRAINT(0x00c0, 0), /* INST_RETIRED.ANY */
	FIXED_EVENT_CONSTRAINT(0x003c, 1), /* CPU_CLK_UNHALTED.CORE */
	FIXED_EVENT_CONSTRAINT(0x0300, 2), /* CPU_CLK_UNHALTED.REF */
	EVENT_CONSTRAINT_END
};

static struct extra_reg intel_snb_extra_regs[] __read_mostly = {
	INTEL_EVENT_EXTRA_REG(0xb7, MSR_OFFCORE_RSP_0, 0x3fffffffffull, RSP_0),
	INTEL_EVENT_EXTRA_REG(0xbb, MSR_OFFCORE_RSP_1, 0x3fffffffffull, RSP_1),
	EVENT_EXTRA_END
};

static u64 intel_pmu_event_map(int hw_event)
{
	return intel_perfmon_event_map[hw_event];
}

static __initconst const u64 snb_hw_cache_event_ids
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(L1D) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0xf1d0, /* MEM_UOP_RETIRED.LOADS        */
		[ C(RESULT_MISS)   ] = 0x0151, /* L1D.REPLACEMENT              */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0xf2d0, /* MEM_UOP_RETIRED.STORES       */
		[ C(RESULT_MISS)   ] = 0x0851, /* L1D.ALL_M_REPLACEMENT        */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x024e, /* HW_PRE_REQ.DL1_MISS          */
	},
 },
 [ C(L1I ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0280, /* ICACHE.MISSES */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		/* OFFCORE_RESPONSE.ANY_DATA.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_DATA.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_WRITE) ] = {
		/* OFFCORE_RESPONSE.ANY_RFO.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_RFO.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_PREFETCH) ] = {
		/* OFFCORE_RESPONSE.PREFETCH.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.PREFETCH.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
 },
 [ C(DTLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x81d0, /* MEM_UOP_RETIRED.ALL_LOADS */
		[ C(RESULT_MISS)   ] = 0x0108, /* DTLB_LOAD_MISSES.CAUSES_A_WALK */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x82d0, /* MEM_UOP_RETIRED.ALL_STORES */
		[ C(RESULT_MISS)   ] = 0x0149, /* DTLB_STORE_MISSES.MISS_CAUSES_A_WALK */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(ITLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x1085, /* ITLB_MISSES.STLB_HIT         */
		[ C(RESULT_MISS)   ] = 0x0185, /* ITLB_MISSES.CAUSES_A_WALK    */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(BPU ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c4, /* BR_INST_RETIRED.ALL_BRANCHES */
		[ C(RESULT_MISS)   ] = 0x00c5, /* BR_MISP_RETIRED.ALL_BRANCHES */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(NODE) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },

};

static __initconst const u64 westmere_hw_cache_event_ids
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(L1D) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x010b, /* MEM_INST_RETIRED.LOADS       */
		[ C(RESULT_MISS)   ] = 0x0151, /* L1D.REPL                     */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x020b, /* MEM_INST_RETURED.STORES      */
		[ C(RESULT_MISS)   ] = 0x0251, /* L1D.M_REPL                   */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x014e, /* L1D_PREFETCH.REQUESTS        */
		[ C(RESULT_MISS)   ] = 0x024e, /* L1D_PREFETCH.MISS            */
	},
 },
 [ C(L1I ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0380, /* L1I.READS                    */
		[ C(RESULT_MISS)   ] = 0x0280, /* L1I.MISSES                   */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		/* OFFCORE_RESPONSE.ANY_DATA.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_DATA.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	/*
	 * Use RFO, not WRITEBACK, because a write miss would typically occur
	 * on RFO.
	 */
	[ C(OP_WRITE) ] = {
		/* OFFCORE_RESPONSE.ANY_RFO.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_RFO.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_PREFETCH) ] = {
		/* OFFCORE_RESPONSE.PREFETCH.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.PREFETCH.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
 },
 [ C(DTLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x010b, /* MEM_INST_RETIRED.LOADS       */
		[ C(RESULT_MISS)   ] = 0x0108, /* DTLB_LOAD_MISSES.ANY         */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x020b, /* MEM_INST_RETURED.STORES      */
		[ C(RESULT_MISS)   ] = 0x010c, /* MEM_STORE_RETIRED.DTLB_MISS  */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(ITLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x01c0, /* INST_RETIRED.ANY_P           */
		[ C(RESULT_MISS)   ] = 0x0185, /* ITLB_MISSES.ANY              */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(BPU ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c4, /* BR_INST_RETIRED.ALL_BRANCHES */
		[ C(RESULT_MISS)   ] = 0x03e8, /* BPU_CLEARS.ANY               */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(NODE) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
 },
};

/*
 * Nehalem/Westmere MSR_OFFCORE_RESPONSE bits;
 * See IA32 SDM Vol 3B 30.6.1.3
 */

#define NHM_DMND_DATA_RD	(1 << 0)
#define NHM_DMND_RFO		(1 << 1)
#define NHM_DMND_IFETCH		(1 << 2)
#define NHM_DMND_WB		(1 << 3)
#define NHM_PF_DATA_RD		(1 << 4)
#define NHM_PF_DATA_RFO		(1 << 5)
#define NHM_PF_IFETCH		(1 << 6)
#define NHM_OFFCORE_OTHER	(1 << 7)
#define NHM_UNCORE_HIT		(1 << 8)
#define NHM_OTHER_CORE_HIT_SNP	(1 << 9)
#define NHM_OTHER_CORE_HITM	(1 << 10)
        			/* reserved */
#define NHM_REMOTE_CACHE_FWD	(1 << 12)
#define NHM_REMOTE_DRAM		(1 << 13)
#define NHM_LOCAL_DRAM		(1 << 14)
#define NHM_NON_DRAM		(1 << 15)

#define NHM_LOCAL		(NHM_LOCAL_DRAM|NHM_REMOTE_CACHE_FWD)
#define NHM_REMOTE		(NHM_REMOTE_DRAM)

#define NHM_DMND_READ		(NHM_DMND_DATA_RD)
#define NHM_DMND_WRITE		(NHM_DMND_RFO|NHM_DMND_WB)
#define NHM_DMND_PREFETCH	(NHM_PF_DATA_RD|NHM_PF_DATA_RFO)

#define NHM_L3_HIT	(NHM_UNCORE_HIT|NHM_OTHER_CORE_HIT_SNP|NHM_OTHER_CORE_HITM)
#define NHM_L3_MISS	(NHM_NON_DRAM|NHM_LOCAL_DRAM|NHM_REMOTE_DRAM|NHM_REMOTE_CACHE_FWD)
#define NHM_L3_ACCESS	(NHM_L3_HIT|NHM_L3_MISS)

static __initconst const u64 nehalem_hw_cache_extra_regs
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_READ|NHM_L3_ACCESS,
		[ C(RESULT_MISS)   ] = NHM_DMND_READ|NHM_L3_MISS,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_WRITE|NHM_L3_ACCESS,
		[ C(RESULT_MISS)   ] = NHM_DMND_WRITE|NHM_L3_MISS,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_PREFETCH|NHM_L3_ACCESS,
		[ C(RESULT_MISS)   ] = NHM_DMND_PREFETCH|NHM_L3_MISS,
	},
 },
 [ C(NODE) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_READ|NHM_LOCAL|NHM_REMOTE,
		[ C(RESULT_MISS)   ] = NHM_DMND_READ|NHM_REMOTE,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_WRITE|NHM_LOCAL|NHM_REMOTE,
		[ C(RESULT_MISS)   ] = NHM_DMND_WRITE|NHM_REMOTE,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = NHM_DMND_PREFETCH|NHM_LOCAL|NHM_REMOTE,
		[ C(RESULT_MISS)   ] = NHM_DMND_PREFETCH|NHM_REMOTE,
	},
 },
};

static __initconst const u64 nehalem_hw_cache_event_ids
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(L1D) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x010b, /* MEM_INST_RETIRED.LOADS       */
		[ C(RESULT_MISS)   ] = 0x0151, /* L1D.REPL                     */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x020b, /* MEM_INST_RETURED.STORES      */
		[ C(RESULT_MISS)   ] = 0x0251, /* L1D.M_REPL                   */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x014e, /* L1D_PREFETCH.REQUESTS        */
		[ C(RESULT_MISS)   ] = 0x024e, /* L1D_PREFETCH.MISS            */
	},
 },
 [ C(L1I ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0380, /* L1I.READS                    */
		[ C(RESULT_MISS)   ] = 0x0280, /* L1I.MISSES                   */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		/* OFFCORE_RESPONSE.ANY_DATA.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_DATA.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	/*
	 * Use RFO, not WRITEBACK, because a write miss would typically occur
	 * on RFO.
	 */
	[ C(OP_WRITE) ] = {
		/* OFFCORE_RESPONSE.ANY_RFO.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.ANY_RFO.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_PREFETCH) ] = {
		/* OFFCORE_RESPONSE.PREFETCH.LOCAL_CACHE */
		[ C(RESULT_ACCESS) ] = 0x01b7,
		/* OFFCORE_RESPONSE.PREFETCH.ANY_LLC_MISS */
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
 },
 [ C(DTLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f40, /* L1D_CACHE_LD.MESI   (alias)  */
		[ C(RESULT_MISS)   ] = 0x0108, /* DTLB_LOAD_MISSES.ANY         */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f41, /* L1D_CACHE_ST.MESI   (alias)  */
		[ C(RESULT_MISS)   ] = 0x010c, /* MEM_STORE_RETIRED.DTLB_MISS  */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0x0,
	},
 },
 [ C(ITLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x01c0, /* INST_RETIRED.ANY_P           */
		[ C(RESULT_MISS)   ] = 0x20c8, /* ITLB_MISS_RETIRED            */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(BPU ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c4, /* BR_INST_RETIRED.ALL_BRANCHES */
		[ C(RESULT_MISS)   ] = 0x03e8, /* BPU_CLEARS.ANY               */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(NODE) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x01b7,
		[ C(RESULT_MISS)   ] = 0x01b7,
	},
 },
};

static __initconst const u64 core2_hw_cache_event_ids
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(L1D) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f40, /* L1D_CACHE_LD.MESI          */
		[ C(RESULT_MISS)   ] = 0x0140, /* L1D_CACHE_LD.I_STATE       */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f41, /* L1D_CACHE_ST.MESI          */
		[ C(RESULT_MISS)   ] = 0x0141, /* L1D_CACHE_ST.I_STATE       */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x104e, /* L1D_PREFETCH.REQUESTS      */
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(L1I ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0080, /* L1I.READS                  */
		[ C(RESULT_MISS)   ] = 0x0081, /* L1I.MISSES                 */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x4f29, /* L2_LD.MESI                 */
		[ C(RESULT_MISS)   ] = 0x4129, /* L2_LD.ISTATE               */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x4f2A, /* L2_ST.MESI                 */
		[ C(RESULT_MISS)   ] = 0x412A, /* L2_ST.ISTATE               */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(DTLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f40, /* L1D_CACHE_LD.MESI  (alias) */
		[ C(RESULT_MISS)   ] = 0x0208, /* DTLB_MISSES.MISS_LD        */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x0f41, /* L1D_CACHE_ST.MESI  (alias) */
		[ C(RESULT_MISS)   ] = 0x0808, /* DTLB_MISSES.MISS_ST        */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(ITLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c0, /* INST_RETIRED.ANY_P         */
		[ C(RESULT_MISS)   ] = 0x1282, /* ITLBMISSES                 */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(BPU ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c4, /* BR_INST_RETIRED.ANY        */
		[ C(RESULT_MISS)   ] = 0x00c5, /* BP_INST_RETIRED.MISPRED    */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
};

static __initconst const u64 atom_hw_cache_event_ids
				[PERF_COUNT_HW_CACHE_MAX]
				[PERF_COUNT_HW_CACHE_OP_MAX]
				[PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
 [ C(L1D) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x2140, /* L1D_CACHE.LD               */
		[ C(RESULT_MISS)   ] = 0,
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x2240, /* L1D_CACHE.ST               */
		[ C(RESULT_MISS)   ] = 0,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0x0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(L1I ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x0380, /* L1I.READS                  */
		[ C(RESULT_MISS)   ] = 0x0280, /* L1I.MISSES                 */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(LL  ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x4f29, /* L2_LD.MESI                 */
		[ C(RESULT_MISS)   ] = 0x4129, /* L2_LD.ISTATE               */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x4f2A, /* L2_ST.MESI                 */
		[ C(RESULT_MISS)   ] = 0x412A, /* L2_ST.ISTATE               */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(DTLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x2140, /* L1D_CACHE_LD.MESI  (alias) */
		[ C(RESULT_MISS)   ] = 0x0508, /* DTLB_MISSES.MISS_LD        */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = 0x2240, /* L1D_CACHE_ST.MESI  (alias) */
		[ C(RESULT_MISS)   ] = 0x0608, /* DTLB_MISSES.MISS_ST        */
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = 0,
		[ C(RESULT_MISS)   ] = 0,
	},
 },
 [ C(ITLB) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c0, /* INST_RETIRED.ANY_P         */
		[ C(RESULT_MISS)   ] = 0x0282, /* ITLB.MISSES                */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
 [ C(BPU ) ] = {
	[ C(OP_READ) ] = {
		[ C(RESULT_ACCESS) ] = 0x00c4, /* BR_INST_RETIRED.ANY        */
		[ C(RESULT_MISS)   ] = 0x00c5, /* BP_INST_RETIRED.MISPRED    */
	},
	[ C(OP_WRITE) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
	[ C(OP_PREFETCH) ] = {
		[ C(RESULT_ACCESS) ] = -1,
		[ C(RESULT_MISS)   ] = -1,
	},
 },
};

static inline bool intel_pmu_needs_lbr_smpl(struct perf_event *event)
{
	/* user explicitly requested branch sampling */
	if (has_branch_stack(event))
		return true;

	/* implicit branch sampling to correct PEBS skid */
	if (x86_pmu.intel_cap.pebs_trap && event->attr.precise_ip > 1)
		return true;

	return false;
}

static void intel_pmu_disable_all(void)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);

	wrmsrl(MSR_CORE_PERF_GLOBAL_CTRL, 0);

	if (test_bit(X86_PMC_IDX_FIXED_BTS, cpuc->active_mask))
		intel_pmu_disable_bts();

	intel_pmu_pebs_disable_all();
	intel_pmu_lbr_disable_all();
}

static void intel_pmu_enable_all(int added)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);

	intel_pmu_pebs_enable_all();
	intel_pmu_lbr_enable_all();
	wrmsrl(MSR_CORE_PERF_GLOBAL_CTRL,
			x86_pmu.intel_ctrl & ~cpuc->intel_ctrl_guest_mask);

	if (test_bit(X86_PMC_IDX_FIXED_BTS, cpuc->active_mask)) {
		struct perf_event *event =
			cpuc->events[X86_PMC_IDX_FIXED_BTS];

		if (WARN_ON_ONCE(!event))
			return;

		intel_pmu_enable_bts(event->hw.config);
	}
}

/*
 * Workaround for:
 *   Intel Errata AAK100 (model 26)
 *   Intel Errata AAP53  (model 30)
 *   Intel Errata BD53   (model 44)
 *
 * The official story:
 *   These chips need to be 'reset' when adding counters by programming the
 *   magic three (non-counting) events 0x4300B5, 0x4300D2, and 0x4300B1 either
 *   in sequence on the same PMC or on different PMCs.
 *
 * In practise it appears some of these events do in fact count, and
 * we need to programm all 4 events.
 */
static void intel_pmu_nhm_workaround(void)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	static const unsigned long nhm_magic[4] = {
		0x4300B5,
		0x4300D2,
		0x4300B1,
		0x4300B1
	};
	struct perf_event *event;
	int i;

	/*
	 * The Errata requires below steps:
	 * 1) Clear MSR_IA32_PEBS_ENABLE and MSR_CORE_PERF_GLOBAL_CTRL;
	 * 2) Configure 4 PERFEVTSELx with the magic events and clear
	 *    the corresponding PMCx;
	 * 3) set bit0~bit3 of MSR_CORE_PERF_GLOBAL_CTRL;
	 * 4) Clear MSR_CORE_PERF_GLOBAL_CTRL;
	 * 5) Clear 4 pairs of ERFEVTSELx and PMCx;
	 */

	/*
	 * The real steps we choose are a little different from above.
	 * A) To reduce MSR operations, we don't run step 1) as they
	 *    are already cleared before this function is called;
	 * B) Call x86_perf_event_update to save PMCx before configuring
	 *    PERFEVTSELx with magic number;
	 * C) With step 5), we do clear only when the PERFEVTSELx is
	 *    not used currently.
	 * D) Call x86_perf_event_set_period to restore PMCx;
	 */

	/* We always operate 4 pairs of PERF Counters */
	for (i = 0; i < 4; i++) {
		event = cpuc->events[i];
		if (event)
			x86_perf_event_update(event);
	}

	for (i = 0; i < 4; i++) {
		wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + i, nhm_magic[i]);
		wrmsrl(MSR_ARCH_PERFMON_PERFCTR0 + i, 0x0);
	}

	wrmsrl(MSR_CORE_PERF_GLOBAL_CTRL, 0xf);
	wrmsrl(MSR_CORE_PERF_GLOBAL_CTRL, 0x0);

	for (i = 0; i < 4; i++) {
		event = cpuc->events[i];

		if (event) {
			x86_perf_event_set_period(event);
			__x86_pmu_enable_event(&event->hw,
					ARCH_PERFMON_EVENTSEL_ENABLE);
		} else
			wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + i, 0x0);
	}
}

static void intel_pmu_nhm_enable_all(int added)
{
	if (added)
		intel_pmu_nhm_workaround();
	intel_pmu_enable_all(added);
}

static inline u64 intel_pmu_get_status(void)
{
	u64 status;

	rdmsrl(MSR_CORE_PERF_GLOBAL_STATUS, status);

	return status;
}

static inline void intel_pmu_ack_status(u64 ack)
{
	wrmsrl(MSR_CORE_PERF_GLOBAL_OVF_CTRL, ack);
}

static void intel_pmu_disable_fixed(struct hw_perf_event *hwc)
{
	int idx = hwc->idx - X86_PMC_IDX_FIXED;
	u64 ctrl_val, mask;

	mask = 0xfULL << (idx * 4);

	rdmsrl(hwc->config_base, ctrl_val);
	ctrl_val &= ~mask;
	wrmsrl(hwc->config_base, ctrl_val);
}

static void intel_pmu_disable_event(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);

	if (unlikely(hwc->idx == X86_PMC_IDX_FIXED_BTS)) {
		intel_pmu_disable_bts();
		intel_pmu_drain_bts_buffer();
		return;
	}

	cpuc->intel_ctrl_guest_mask &= ~(1ull << hwc->idx);
	cpuc->intel_ctrl_host_mask &= ~(1ull << hwc->idx);

	/*
	 * must disable before any actual event
	 * because any event may be combined with LBR
	 */
	if (intel_pmu_needs_lbr_smpl(event))
		intel_pmu_lbr_disable(event);

	if (unlikely(hwc->config_base == MSR_ARCH_PERFMON_FIXED_CTR_CTRL)) {
		intel_pmu_disable_fixed(hwc);
		return;
	}

	x86_pmu_disable_event(event);

	if (unlikely(event->attr.precise_ip))
		intel_pmu_pebs_disable(event);
}

static void intel_pmu_enable_fixed(struct hw_perf_event *hwc)
{
	int idx = hwc->idx - X86_PMC_IDX_FIXED;
	u64 ctrl_val, bits, mask;

	/*
	 * Enable IRQ generation (0x8),
	 * and enable ring-3 counting (0x2) and ring-0 counting (0x1)
	 * if requested:
	 */
	bits = 0x8ULL;
	if (hwc->config & ARCH_PERFMON_EVENTSEL_USR)
		bits |= 0x2;
	if (hwc->config & ARCH_PERFMON_EVENTSEL_OS)
		bits |= 0x1;

	/*
	 * ANY bit is supported in v3 and up
	 */
	if (x86_pmu.version > 2 && hwc->config & ARCH_PERFMON_EVENTSEL_ANY)
		bits |= 0x4;

	bits <<= (idx * 4);
	mask = 0xfULL << (idx * 4);

	rdmsrl(hwc->config_base, ctrl_val);
	ctrl_val &= ~mask;
	ctrl_val |= bits;
	wrmsrl(hwc->config_base, ctrl_val);
}

static void intel_pmu_enable_event(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);

	if (unlikely(hwc->idx == X86_PMC_IDX_FIXED_BTS)) {
		if (!__this_cpu_read(cpu_hw_events.enabled))
			return;

		intel_pmu_enable_bts(hwc->config);
		return;
	}
	/*
	 * must enabled before any actual event
	 * because any event may be combined with LBR
	 */
	if (intel_pmu_needs_lbr_smpl(event))
		intel_pmu_lbr_enable(event);

	if (event->attr.exclude_host)
		cpuc->intel_ctrl_guest_mask |= (1ull << hwc->idx);
	if (event->attr.exclude_guest)
		cpuc->intel_ctrl_host_mask |= (1ull << hwc->idx);

	if (unlikely(hwc->config_base == MSR_ARCH_PERFMON_FIXED_CTR_CTRL)) {
		intel_pmu_enable_fixed(hwc);
		return;
	}

	if (unlikely(event->attr.precise_ip))
		intel_pmu_pebs_enable(event);

	__x86_pmu_enable_event(hwc, ARCH_PERFMON_EVENTSEL_ENABLE);
}

/*
 * Save and restart an expired event. Called by NMI contexts,
 * so it has to be careful about preempting normal event ops:
 */
int intel_pmu_save_and_restart(struct perf_event *event)
{
	x86_perf_event_update(event);
	return x86_perf_event_set_period(event);
}

static void intel_pmu_reset(void)
{
	struct debug_store *ds = __this_cpu_read(cpu_hw_events.ds);
	unsigned long flags;
	int idx;

	if (!x86_pmu.num_counters)
		return;

	local_irq_save(flags);

	printk("clearing PMU state on CPU#%d\n", smp_processor_id());

	for (idx = 0; idx < x86_pmu.num_counters; idx++) {
		checking_wrmsrl(x86_pmu_config_addr(idx), 0ull);
		checking_wrmsrl(x86_pmu_event_addr(idx),  0ull);
	}
	for (idx = 0; idx < x86_pmu.num_counters_fixed; idx++)
		checking_wrmsrl(MSR_ARCH_PERFMON_FIXED_CTR0 + idx, 0ull);

	if (ds)
		ds->bts_index = ds->bts_buffer_base;

	local_irq_restore(flags);
}

/*
 * This handler is triggered by the local APIC, so the APIC IRQ handling
 * rules apply:
 */
static int intel_pmu_handle_irq(struct pt_regs *regs)
{
	struct perf_sample_data data;
	struct cpu_hw_events *cpuc;
	int bit, loops;
	u64 status;
	int handled;

	perf_sample_data_init(&data, 0);

	cpuc = &__get_cpu_var(cpu_hw_events);

	/*
	 * Some chipsets need to unmask the LVTPC in a particular spot
	 * inside the nmi handler.  As a result, the unmasking was pushed
	 * into all the nmi handlers.
	 *
	 * This handler doesn't seem to have any issues with the unmasking
	 * so it was left at the top.
	 */
	apic_write(APIC_LVTPC, APIC_DM_NMI);

	intel_pmu_disable_all();
	handled = intel_pmu_drain_bts_buffer();
	status = intel_pmu_get_status();
	if (!status) {
		intel_pmu_enable_all(0);
		return handled;
	}

	loops = 0;
again:
	intel_pmu_ack_status(status);
	if (++loops > 100) {
		WARN_ONCE(1, "perfevents: irq loop stuck!\n");
		perf_event_print_debug();
		intel_pmu_reset();
		goto done;
	}

	inc_irq_stat(apic_perf_irqs);

	intel_pmu_lbr_read();

	/*
	 * PEBS overflow sets bit 62 in the global status register
	 */
	if (__test_and_clear_bit(62, (unsigned long *)&status)) {
		handled++;
		x86_pmu.drain_pebs(regs);
	}

	for_each_set_bit(bit, (unsigned long *)&status, X86_PMC_IDX_MAX) {
		struct perf_event *event = cpuc->events[bit];

		handled++;

		if (!test_bit(bit, cpuc->active_mask))
			continue;

		if (!intel_pmu_save_and_restart(event))
			continue;

		data.period = event->hw.last_period;

		if (has_branch_stack(event))
			data.br_stack = &cpuc->lbr_stack;

		if (perf_event_overflow(event, &data, regs))
			x86_pmu_stop(event, 0);
	}

	/*
	 * Repeat if there is more work to be done:
	 */
	status = intel_pmu_get_status();
	if (status)
		goto again;

done:
	intel_pmu_enable_all(0);
	return handled;
}

static struct event_constraint *
intel_bts_constraints(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	unsigned int hw_event, bts_event;

	if (event->attr.freq)
		return NULL;

	hw_event = hwc->config & INTEL_ARCH_EVENT_MASK;
	bts_event = x86_pmu.event_map(PERF_COUNT_HW_BRANCH_INSTRUCTIONS);

	if (unlikely(hw_event == bts_event && hwc->sample_period == 1))
		return &bts_constraint;

	return NULL;
}

static bool intel_try_alt_er(struct perf_event *event, int orig_idx)
{
	if (!(x86_pmu.er_flags & ERF_HAS_RSP_1))
		return false;

	if (event->hw.extra_reg.idx == EXTRA_REG_RSP_0) {
		event->hw.config &= ~INTEL_ARCH_EVENT_MASK;
		event->hw.config |= 0x01bb;
		event->hw.extra_reg.idx = EXTRA_REG_RSP_1;
		event->hw.extra_reg.reg = MSR_OFFCORE_RSP_1;
	} else if (event->hw.extra_reg.idx == EXTRA_REG_RSP_1) {
		event->hw.config &= ~INTEL_ARCH_EVENT_MASK;
		event->hw.config |= 0x01b7;
		event->hw.extra_reg.idx = EXTRA_REG_RSP_0;
		event->hw.extra_reg.reg = MSR_OFFCORE_RSP_0;
	}

	if (event->hw.extra_reg.idx == orig_idx)
		return false;

	return true;
}

/*
 * manage allocation of shared extra msr for certain events
 *
 * sharing can be:
 * per-cpu: to be shared between the various events on a single PMU
 * per-core: per-cpu + shared by HT threads
 */
static struct event_constraint *
__intel_shared_reg_get_constraints(struct cpu_hw_events *cpuc,
				   struct perf_event *event,
				   struct hw_perf_event_extra *reg)
{
	struct event_constraint *c = &emptyconstraint;
	struct er_account *era;
	unsigned long flags;
	int orig_idx = reg->idx;

	/* already allocated shared msr */
	if (reg->alloc)
		return NULL; /* call x86_get_event_constraint() */

again:
	era = &cpuc->shared_regs->regs[reg->idx];
	/*
	 * we use spin_lock_irqsave() to avoid lockdep issues when
	 * passing a fake cpuc
	 */
	raw_spin_lock_irqsave(&era->lock, flags);

	if (!atomic_read(&era->ref) || era->config == reg->config) {

		/* lock in msr value */
		era->config = reg->config;
		era->reg = reg->reg;

		/* one more user */
		atomic_inc(&era->ref);

		/* no need to reallocate during incremental event scheduling */
		reg->alloc = 1;

		/*
		 * need to call x86_get_event_constraint()
		 * to check if associated event has constraints
		 */
		c = NULL;
	} else if (intel_try_alt_er(event, orig_idx)) {
		raw_spin_unlock_irqrestore(&era->lock, flags);
		goto again;
	}
	raw_spin_unlock_irqrestore(&era->lock, flags);

	return c;
}

static void
__intel_shared_reg_put_constraints(struct cpu_hw_events *cpuc,
				   struct hw_perf_event_extra *reg)
{
	struct er_account *era;

	/*
	 * only put constraint if extra reg was actually
	 * allocated. Also takes care of event which do
	 * not use an extra shared reg
	 */
	if (!reg->alloc)
		return;

	era = &cpuc->shared_regs->regs[reg->idx];

	/* one fewer user */
	atomic_dec(&era->ref);

	/* allocate again next time */
	reg->alloc = 0;
}

static struct event_constraint *
intel_shared_regs_constraints(struct cpu_hw_events *cpuc,
			      struct perf_event *event)
{
	struct event_constraint *c = NULL, *d;
	struct hw_perf_event_extra *xreg, *breg;

	xreg = &event->hw.extra_reg;
	if (xreg->idx != EXTRA_REG_NONE) {
		c = __intel_shared_reg_get_constraints(cpuc, event, xreg);
		if (c == &emptyconstraint)
			return c;
	}
	breg = &event->hw.branch_reg;
	if (breg->idx != EXTRA_REG_NONE) {
		d = __intel_shared_reg_get_constraints(cpuc, event, breg);
		if (d == &emptyconstraint) {
			__intel_shared_reg_put_constraints(cpuc, xreg);
			c = d;
		}
	}
	return c;
}

struct event_constraint *
x86_get_event_constraints(struct cpu_hw_events *cpuc, struct perf_event *event)
{
	struct event_constraint *c;

	if (x86_pmu.event_constraints) {
		for_each_event_constraint(c, x86_pmu.event_constraints) {
			if ((event->hw.config & c->cmask) == c->code)
				return c;
		}
	}

	return &unconstrained;
}

static struct event_constraint *
intel_get_event_constraints(struct cpu_hw_events *cpuc, struct perf_event *event)
{
	struct event_constraint *c;

	c = intel_bts_constraints(event);
	if (c)
		return c;

	c = intel_pebs_constraints(event);
	if (c)
		return c;

	c = intel_shared_regs_constraints(cpuc, event);
	if (c)
		return c;

	return x86_get_event_constraints(cpuc, event);
}

static void
intel_put_shared_regs_event_constraints(struct cpu_hw_events *cpuc,
					struct perf_event *event)
{
	struct hw_perf_event_extra *reg;

	reg = &event->hw.extra_reg;
	if (reg->idx != EXTRA_REG_NONE)
		__intel_shared_reg_put_constraints(cpuc, reg);

	reg = &event->hw.branch_reg;
	if (reg->idx != EXTRA_REG_NONE)
		__intel_shared_reg_put_constraints(cpuc, reg);
}

static void intel_put_event_constraints(struct cpu_hw_events *cpuc,
					struct perf_event *event)
{
	intel_put_shared_regs_event_constraints(cpuc, event);
}

static int intel_pmu_hw_config(struct perf_event *event)
{
	int ret = x86_pmu_hw_config(event);

	if (ret)
		return ret;

	if (event->attr.precise_ip &&
	    (event->hw.config & X86_RAW_EVENT_MASK) == 0x003c) {
		/*
		 * Use an alternative encoding for CPU_CLK_UNHALTED.THREAD_P
		 * (0x003c) so that we can use it with PEBS.
		 *
		 * The regular CPU_CLK_UNHALTED.THREAD_P event (0x003c) isn't
		 * PEBS capable. However we can use INST_RETIRED.ANY_P
		 * (0x00c0), which is a PEBS capable event, to get the same
		 * count.
		 *
		 * INST_RETIRED.ANY_P counts the number of cycles that retires
		 * CNTMASK instructions. By setting CNTMASK to a value (16)
		 * larger than the maximum number of instructions that can be
		 * retired per cycle (4) and then inverting the condition, we
		 * count all cycles that retire 16 or less instructions, which
		 * is every cycle.
		 *
		 * Thereby we gain a PEBS capable cycle counter.
		 */
		u64 alt_config = X86_CONFIG(.event=0xc0, .inv=1, .cmask=16);


		alt_config |= (event->hw.config & ~X86_RAW_EVENT_MASK);
		event->hw.config = alt_config;
	}

	if (intel_pmu_needs_lbr_smpl(event)) {
		ret = intel_pmu_setup_lbr_filter(event);
		if (ret)
			return ret;
	}

	if (event->attr.type != PERF_TYPE_RAW)
		return 0;

	if (!(event->attr.config & ARCH_PERFMON_EVENTSEL_ANY))
		return 0;

	if (x86_pmu.version < 3)
		return -EINVAL;

	if (perf_paranoid_cpu() && !capable(CAP_SYS_ADMIN))
		return -EACCES;

	event->hw.config |= ARCH_PERFMON_EVENTSEL_ANY;

	return 0;
}

struct perf_guest_switch_msr *perf_guest_get_msrs(int *nr)
{
	if (x86_pmu.guest_get_msrs)
		return x86_pmu.guest_get_msrs(nr);
	*nr = 0;
	return NULL;
}
EXPORT_SYMBOL_GPL(perf_guest_get_msrs);

static struct perf_guest_switch_msr *intel_guest_get_msrs(int *nr)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	struct perf_guest_switch_msr *arr = cpuc->guest_switch_msrs;

	arr[0].msr = MSR_CORE_PERF_GLOBAL_CTRL;
	arr[0].host = x86_pmu.intel_ctrl & ~cpuc->intel_ctrl_guest_mask;
	arr[0].guest = x86_pmu.intel_ctrl & ~cpuc->intel_ctrl_host_mask;

	*nr = 1;
	return arr;
}

static struct perf_guest_switch_msr *core_guest_get_msrs(int *nr)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	struct perf_guest_switch_msr *arr = cpuc->guest_switch_msrs;
	int idx;

	for (idx = 0; idx < x86_pmu.num_counters; idx++)  {
		struct perf_event *event = cpuc->events[idx];

		arr[idx].msr = x86_pmu_config_addr(idx);
		arr[idx].host = arr[idx].guest = 0;

		if (!test_bit(idx, cpuc->active_mask))
			continue;

		arr[idx].host = arr[idx].guest =
			event->hw.config | ARCH_PERFMON_EVENTSEL_ENABLE;

		if (event->attr.exclude_host)
			arr[idx].host &= ~ARCH_PERFMON_EVENTSEL_ENABLE;
		else if (event->attr.exclude_guest)
			arr[idx].guest &= ~ARCH_PERFMON_EVENTSEL_ENABLE;
	}

	*nr = x86_pmu.num_counters;
	return arr;
}

static void core_pmu_enable_event(struct perf_event *event)
{
	if (!event->attr.exclude_host)
		x86_pmu_enable_event(event);
}

static void core_pmu_enable_all(int added)
{
	struct cpu_hw_events *cpuc = &__get_cpu_var(cpu_hw_events);
	int idx;

	for (idx = 0; idx < x86_pmu.num_counters; idx++) {
		struct hw_perf_event *hwc = &cpuc->events[idx]->hw;

		if (!test_bit(idx, cpuc->active_mask) ||
				cpuc->events[idx]->attr.exclude_host)
			continue;

		__x86_pmu_enable_event(hwc, ARCH_PERFMON_EVENTSEL_ENABLE);
	}
}

PMU_FORMAT_ATTR(event,	"config:0-7"	);
PMU_FORMAT_ATTR(umask,	"config:8-15"	);
PMU_FORMAT_ATTR(edge,	"config:18"	);
PMU_FORMAT_ATTR(pc,	"config:19"	);
PMU_FORMAT_ATTR(any,	"config:21"	); /* v3 + */
PMU_FORMAT_ATTR(inv,	"config:23"	);
PMU_FORMAT_ATTR(cmask,	"config:24-31"	);

static struct attribute *intel_arch_formats_attr[] = {
	&format_attr_event.attr,
	&format_attr_umask.attr,
	&format_attr_edge.attr,
	&format_attr_pc.attr,
	&format_attr_inv.attr,
	&format_attr_cmask.attr,
	NULL,
};

static __initconst const struct x86_pmu core_pmu = {
	.name			= "core",
	.handle_irq		= x86_pmu_handle_irq,
	.disable_all		= x86_pmu_disable_all,
	.enable_all		= core_pmu_enable_all,
	.enable			= core_pmu_enable_event,
	.disable		= x86_pmu_disable_event,
	.hw_config		= x86_pmu_hw_config,
	.schedule_events	= x86_schedule_events,
	.eventsel		= MSR_ARCH_PERFMON_EVENTSEL0,
	.perfctr		= MSR_ARCH_PERFMON_PERFCTR0,
	.event_map		= intel_pmu_event_map,
	.max_events		= ARRAY_SIZE(intel_perfmon_event_map),
	.apic			= 1,
	/*
	 * Intel PMCs cannot be accessed sanely above 32 bit width,
	 * so we install an artificial 1<<31 period regardless of
	 * the generic event period:
	 */
	.max_period		= (1ULL << 31) - 1,
	.get_event_constraints	= intel_get_event_constraints,
	.put_event_constraints	= intel_put_event_constraints,
	.event_constraints	= intel_core_event_constraints,
	.guest_get_msrs		= core_guest_get_msrs,
	.format_attrs		= intel_arch_formats_attr,
};

struct intel_shared_regs *allocate_shared_regs(int cpu)
{
	struct intel_shared_regs *regs;
	int i;

	regs = kzalloc_node(sizeof(struct intel_shared_regs),
			    GFP_KERNEL, cpu_to_node(cpu));
	if (regs) {
		/*
		 * initialize the locks to keep lockdep happy
		 */
		for (i = 0; i < EXTRA_REG_MAX; i++)
			raw_spin_lock_init(&regs->regs[i].lock);

		regs->core_id = -1;
	}
	return regs;
}

static int intel_pmu_cpu_prepare(int cpu)
{
	struct cpu_hw_events *cpuc = &per_cpu(cpu_hw_events, cpu);

	if (!(x86_pmu.extra_regs || x86_pmu.lbr_sel_map))
		return NOTIFY_OK;

	cpuc->shared_regs = allocate_shared_regs(cpu);
	if (!cpuc->shared_regs)
		return NOTIFY_BAD;

	return NOTIFY_OK;
}

static void intel_pmu_cpu_starting(int cpu)
{
	struct cpu_hw_events *cpuc = &per_cpu(cpu_hw_events, cpu);
	int core_id = topology_core_id(cpu);
	int i;

	init_debug_store_on_cpu(cpu);
	/*
	 * Deal with CPUs that don't clear their LBRs on power-up.
	 */
	intel_pmu_lbr_reset();

	cpuc->lbr_sel = NULL;

	if (!cpuc->shared_regs)
		return;

	if (!(x86_pmu.er_flags & ERF_NO_HT_SHARING)) {
		for_each_cpu(i, topology_thread_cpumask(cpu)) {
			struct intel_shared_regs *pc;

			pc = per_cpu(cpu_hw_events, i).shared_regs;
			if (pc && pc->core_id == core_id) {
				cpuc->kfree_on_online = cpuc->shared_regs;
				cpuc->shared_regs = pc;
				break;
			}
		}
		cpuc->shared_regs->core_id = core_id;
		cpuc->shared_regs->refcnt++;
	}

	if (x86_pmu.lbr_sel_map)
		cpuc->lbr_sel = &cpuc->shared_regs->regs[EXTRA_REG_LBR];
}

static void intel_pmu_cpu_dying(int cpu)
{
	struct cpu_hw_events *cpuc = &per_cpu(cpu_hw_events, cpu);
	struct intel_shared_regs *pc;

	pc = cpuc->shared_regs;
	if (pc) {
		if (pc->core_id == -1 || --pc->refcnt == 0)
			kfree(pc);
		cpuc->shared_regs = NULL;
	}

	fini_debug_store_on_cpu(cpu);
}

static void intel_pmu_flush_branch_stack(void)
{
	/*
	 * Intel LBR does not tag entries with the
	 * PID of the current task, then we need to
	 * flush it on ctxsw
	 * For now, we simply reset it
	 */
	if (x86_pmu.lbr_nr)
		intel_pmu_lbr_reset();
}

PMU_FORMAT_ATTR(offcore_rsp, "config1:0-63");

static struct attribute *intel_arch3_formats_attr[] = {
	&format_attr_event.attr,
	&format_attr_umask.attr,
	&format_attr_edge.attr,
	&format_attr_pc.attr,
	&format_attr_any.attr,
	&format_attr_inv.attr,
	&format_attr_cmask.attr,

	&format_attr_offcore_rsp.attr, /* XXX do NHM/WSM + SNB breakout */
	NULL,
};

static __initconst const struct x86_pmu intel_pmu = {
	.name			= "Intel",
	.handle_irq		= intel_pmu_handle_irq,
	.disable_all		= intel_pmu_disable_all,
	.enable_all		= intel_pmu_enable_all,
	.enable			= intel_pmu_enable_event,
	.disable		= intel_pmu_disable_event,
	.hw_config		= intel_pmu_hw_config,
	.schedule_events	= x86_schedule_events,
	.eventsel		= MSR_ARCH_PERFMON_EVENTSEL0,
	.perfctr		= MSR_ARCH_PERFMON_PERFCTR0,
	.event_map		= intel_pmu_event_map,
	.max_events		= ARRAY_SIZE(intel_perfmon_event_map),
	.apic			= 1,
	/*
	 * Intel PMCs cannot be accessed sanely above 32 bit width,
	 * so we install an artificial 1<<31 period regardless of
	 * the generic event period:
	 */
	.max_period		= (1ULL << 31) - 1,
	.get_event_constraints	= intel_get_event_constraints,
	.put_event_constraints	= intel_put_event_constraints,

	.format_attrs		= intel_arch3_formats_attr,

	.cpu_prepare		= intel_pmu_cpu_prepare,
	.cpu_starting		= intel_pmu_cpu_starting,
	.cpu_dying		= intel_pmu_cpu_dying,
	.guest_get_msrs		= intel_guest_get_msrs,
	.flush_branch_stack	= intel_pmu_flush_branch_stack,
};

static __init void intel_clovertown_quirk(void)
{
	/*
	 * PEBS is unreliable due to:
	 *
	 *   AJ67  - PEBS may experience CPL leaks
	 *   AJ68  - PEBS PMI may be delayed by one event
	 *   AJ69  - GLOBAL_STATUS[62] will only be set when DEBUGCTL[12]
	 *   AJ106 - FREEZE_LBRS_ON_PMI doesn't work in combination with PEBS
	 *
	 * AJ67 could be worked around by restricting the OS/USR flags.
	 * AJ69 could be worked around by setting PMU_FREEZE_ON_PMI.
	 *
	 * AJ106 could possibly be worked around by not allowing LBR
	 *       usage from PEBS, including the fixup.
	 * AJ68  could possibly be worked around by always programming
	 *	 a pebs_event_reset[0] value and coping with the lost events.
	 *
	 * But taken together it might just make sense to not enable PEBS on
	 * these chips.
	 */
	printk(KERN_WARNING "PEBS disabled due to CPU errata.\n");
	x86_pmu.pebs = 0;
	x86_pmu.pebs_constraints = NULL;
}

static __init void intel_sandybridge_quirk(void)
{
	printk(KERN_WARNING "PEBS disabled due to CPU errata.\n");
	x86_pmu.pebs = 0;
	x86_pmu.pebs_constraints = NULL;
}

static const struct { int id; char *name; } intel_arch_events_map[] __initconst = {
	{ PERF_COUNT_HW_CPU_CYCLES, "cpu cycles" },
	{ PERF_COUNT_HW_INSTRUCTIONS, "instructions" },
	{ PERF_COUNT_HW_BUS_CYCLES, "bus cycles" },
	{ PERF_COUNT_HW_CACHE_REFERENCES, "cache references" },
	{ PERF_COUNT_HW_CACHE_MISSES, "cache misses" },
	{ PERF_COUNT_HW_BRANCH_INSTRUCTIONS, "branch instructions" },
	{ PERF_COUNT_HW_BRANCH_MISSES, "branch misses" },
};

static __init void intel_arch_events_quirk(void)
{
	int bit;

	/* disable event that reported as not presend by cpuid */
	for_each_set_bit(bit, x86_pmu.events_mask, ARRAY_SIZE(intel_arch_events_map)) {
		intel_perfmon_event_map[intel_arch_events_map[bit].id] = 0;
		printk(KERN_WARNING "CPUID marked event: \'%s\' unavailable\n",
				intel_arch_events_map[bit].name);
	}
}

static __init void intel_nehalem_quirk(void)
{
	union cpuid10_ebx ebx;

	ebx.full = x86_pmu.events_maskl;
	if (ebx.split.no_branch_misses_retired) {
		/*
		 * Erratum AAJ80 detected, we work it around by using
		 * the BR_MISP_EXEC.ANY event. This will over-count
		 * branch-misses, but it's still much better than the
		 * architectural event which is often completely bogus:
		 */
		intel_perfmon_event_map[PERF_COUNT_HW_BRANCH_MISSES] = 0x7f89;
		ebx.split.no_branch_misses_retired = 0;
		x86_pmu.events_maskl = ebx.full;
		printk(KERN_INFO "CPU erratum AAJ80 worked around\n");
	}
}

__init int intel_pmu_init(void)
{
	union cpuid10_edx edx;
	union cpuid10_eax eax;
	union cpuid10_ebx ebx;
	unsigned int unused;
	int version;

	if (!cpu_has(&boot_cpu_data, X86_FEATURE_ARCH_PERFMON)) {
		switch (boot_cpu_data.x86) {
		case 0x6:
			return p6_pmu_init();
		case 0xf:
			return p4_pmu_init();
		}
		return -ENODEV;
	}

	/*
	 * Check whether the Architectural PerfMon supports
	 * Branch Misses Retired hw_event or not.
	 */
	cpuid(10, &eax.full, &ebx.full, &unused, &edx.full);
	if (eax.split.mask_length < ARCH_PERFMON_EVENTS_COUNT)
		return -ENODEV;

	version = eax.split.version_id;
	if (version < 2)
		x86_pmu = core_pmu;
	else
		x86_pmu = intel_pmu;

	x86_pmu.version			= version;
	x86_pmu.num_counters		= eax.split.num_counters;
	x86_pmu.cntval_bits		= eax.split.bit_width;
	x86_pmu.cntval_mask		= (1ULL << eax.split.bit_width) - 1;

	x86_pmu.events_maskl		= ebx.full;
	x86_pmu.events_mask_len		= eax.split.mask_length;

	/*
	 * Quirk: v2 perfmon does not report fixed-purpose events, so
	 * assume at least 3 events:
	 */
	if (version > 1)
		x86_pmu.num_counters_fixed = max((int)edx.split.num_counters_fixed, 3);

	/*
	 * v2 and above have a perf capabilities MSR
	 */
	if (version > 1) {
		u64 capabilities;

		rdmsrl(MSR_IA32_PERF_CAPABILITIES, capabilities);
		x86_pmu.intel_cap.capabilities = capabilities;
	}

	intel_ds_init();

	x86_add_quirk(intel_arch_events_quirk); /* Install first, so it runs last */

	/*
	 * Install the hw-cache-events table:
	 */
	switch (boot_cpu_data.x86_model) {
	case 14: /* 65 nm core solo/duo, "Yonah" */
		pr_cont("Core events, ");
		break;

	case 15: /* original 65 nm celeron/pentium/core2/xeon, "Merom"/"Conroe" */
		x86_add_quirk(intel_clovertown_quirk);
	case 22: /* single-core 65 nm celeron/core2solo "Merom-L"/"Conroe-L" */
	case 23: /* current 45 nm celeron/core2/xeon "Penryn"/"Wolfdale" */
	case 29: /* six-core 45 nm xeon "Dunnington" */
		memcpy(hw_cache_event_ids, core2_hw_cache_event_ids,
		       sizeof(hw_cache_event_ids));

		intel_pmu_lbr_init_core();

		x86_pmu.event_constraints = intel_core2_event_constraints;
		x86_pmu.pebs_constraints = intel_core2_pebs_event_constraints;
		pr_cont("Core2 events, ");
		break;

	case 26: /* 45 nm nehalem, "Bloomfield" */
	case 30: /* 45 nm nehalem, "Lynnfield" */
	case 46: /* 45 nm nehalem-ex, "Beckton" */
		memcpy(hw_cache_event_ids, nehalem_hw_cache_event_ids,
		       sizeof(hw_cache_event_ids));
		memcpy(hw_cache_extra_regs, nehalem_hw_cache_extra_regs,
		       sizeof(hw_cache_extra_regs));

		intel_pmu_lbr_init_nhm();

		x86_pmu.event_constraints = intel_nehalem_event_constraints;
		x86_pmu.pebs_constraints = intel_nehalem_pebs_event_constraints;
		x86_pmu.enable_all = intel_pmu_nhm_enable_all;
		x86_pmu.extra_regs = intel_nehalem_extra_regs;

		/* UOPS_ISSUED.STALLED_CYCLES */
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] =
			X86_CONFIG(.event=0x0e, .umask=0x01, .inv=1, .cmask=1);
		/* UOPS_EXECUTED.CORE_ACTIVE_CYCLES,c=1,i=1 */
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] =
			X86_CONFIG(.event=0xb1, .umask=0x3f, .inv=1, .cmask=1);

		x86_add_quirk(intel_nehalem_quirk);

		pr_cont("Nehalem events, ");
		break;

	case 28: /* Atom */
		memcpy(hw_cache_event_ids, atom_hw_cache_event_ids,
		       sizeof(hw_cache_event_ids));

		intel_pmu_lbr_init_atom();

		x86_pmu.event_constraints = intel_gen_event_constraints;
		x86_pmu.pebs_constraints = intel_atom_pebs_event_constraints;
		pr_cont("Atom events, ");
		break;

	case 37: /* 32 nm nehalem, "Clarkdale" */
	case 44: /* 32 nm nehalem, "Gulftown" */
	case 47: /* 32 nm Xeon E7 */
		memcpy(hw_cache_event_ids, westmere_hw_cache_event_ids,
		       sizeof(hw_cache_event_ids));
		memcpy(hw_cache_extra_regs, nehalem_hw_cache_extra_regs,
		       sizeof(hw_cache_extra_regs));

		intel_pmu_lbr_init_nhm();

		x86_pmu.event_constraints = intel_westmere_event_constraints;
		x86_pmu.enable_all = intel_pmu_nhm_enable_all;
		x86_pmu.pebs_constraints = intel_westmere_pebs_event_constraints;
		x86_pmu.extra_regs = intel_westmere_extra_regs;
		x86_pmu.er_flags |= ERF_HAS_RSP_1;

		/* UOPS_ISSUED.STALLED_CYCLES */
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] =
			X86_CONFIG(.event=0x0e, .umask=0x01, .inv=1, .cmask=1);
		/* UOPS_EXECUTED.CORE_ACTIVE_CYCLES,c=1,i=1 */
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] =
			X86_CONFIG(.event=0xb1, .umask=0x3f, .inv=1, .cmask=1);

		pr_cont("Westmere events, ");
		break;

	case 42: /* SandyBridge */
		x86_add_quirk(intel_sandybridge_quirk);
	case 45: /* SandyBridge, "Romely-EP" */
		memcpy(hw_cache_event_ids, snb_hw_cache_event_ids,
		       sizeof(hw_cache_event_ids));

		intel_pmu_lbr_init_snb();

		x86_pmu.event_constraints = intel_snb_event_constraints;
		x86_pmu.pebs_constraints = intel_snb_pebs_event_constraints;
		x86_pmu.extra_regs = intel_snb_extra_regs;
		/* all extra regs are per-cpu when HT is on */
		x86_pmu.er_flags |= ERF_HAS_RSP_1;
		x86_pmu.er_flags |= ERF_NO_HT_SHARING;

		/* UOPS_ISSUED.ANY,c=1,i=1 to count stall cycles */
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND] =
			X86_CONFIG(.event=0x0e, .umask=0x01, .inv=1, .cmask=1);
		/* UOPS_DISPATCHED.THREAD,c=1,i=1 to count stall cycles*/
		intel_perfmon_event_map[PERF_COUNT_HW_STALLED_CYCLES_BACKEND] =
			X86_CONFIG(.event=0xb1, .umask=0x01, .inv=1, .cmask=1);

		pr_cont("SandyBridge events, ");
		break;

	default:
		switch (x86_pmu.version) {
		case 1:
			x86_pmu.event_constraints = intel_v1_event_constraints;
			pr_cont("generic architected perfmon v1, ");
			break;
		default:
			/*
			 * default constraints for v2 and up
			 */
			x86_pmu.event_constraints = intel_gen_event_constraints;
			pr_cont("generic architected perfmon, ");
			break;
		}
	}

	return 0;
}

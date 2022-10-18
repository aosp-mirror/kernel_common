/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2022 Google, Inc.
 *
 * This header file defines the SMC API and the shared data info between
 * Linux and Trusty.
 *
 * Important: Copy of this header file is used in Trusty.
 * Trusty header file:
 *   trusty/trusty/kernel/lib/trusty/include/lib/trusty/trusty_share.h
 * Please keep the copies in sync.
 */
#ifndef _TRUSTY_SCHED_SHARE_H_
#define _TRUSTY_SCHED_SHARE_H_

#include <linux/trusty/smcall.h>

/*
 * trusty-shadow-priority valid values
 */
#define TRUSTY_SHADOW_PRIORITY_LOW 1
#define TRUSTY_SHADOW_PRIORITY_NORMAL 2
#define TRUSTY_SHADOW_PRIORITY_HIGH 3

/**
 * struct trusty_percpu_data - per-cpu trusty shared data
 * @cur_shadow_priority: set by Trusty-Driver/Linux
 * @ask_shadow_priority: set by Trusty Kernel
 */
struct trusty_percpu_data {
	u32 cur_shadow_priority;
	u32 ask_shadow_priority;
};

/**
 * struct trusty_sched_shared - information in the shared memory.
 * @hdr_size: size of the trusty_shared data-structure.
 *            An instance of this data-structure is embedded at
 *            the very beginning of the shared-memory block.
 * @cpu_count: max number of available CPUs in the system.
 * @percpu_data_size: size of the per-cpu data structure.
 *                    The shared-memory block contains an array
 *                    of per-cpu instances of a data-structure that
 *                    can be indexed by cpu_id.
 *
 * NOTE: At the end of this data-structure, additional space is
 * allocated to accommodate a variable length array as follows:
 * 'struct trusty_percpu_data percpu_data_table[]',
 * with 'cpu_count' as its number of elements.
 */
struct trusty_sched_shared {
	u32 hdr_size;
	u32 cpu_count;
	u32 percpu_data_size;
	/* Additional space is allocated here as noted above */
};

#endif /* _TRUSTY_SCHED_SHARE_H_ */

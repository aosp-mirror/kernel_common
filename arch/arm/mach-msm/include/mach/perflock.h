/* arch/arm/mach-msm/perflock.h
 *
 * MSM performance lock driver header
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_PERF_LOCK_H
#define __ARCH_ARM_MACH_PERF_LOCK_H

#include <linux/list.h>
#include <linux/cpufreq.h>

/*
 * Performance level determine differnt EBI1 rate
 */

enum {
	TYPE_PERF_LOCK = 0,	/* default performance lock*/
	TYPE_CPUFREQ_CEILING,	/* cpufreq ceiling lock */
};

enum {
	PERF_LOCK_LOWEST,	/* Lowest performance */
	PERF_LOCK_LOW,	/* Low performance */
	PERF_LOCK_MEDIUM,	/* Medium performance */
	PERF_LOCK_HIGH,	/* High performance */
	PERF_LOCK_HIGHEST,	/* Highest performance */
	PERF_LOCK_INVALID,
};

struct perf_lock {
	struct list_head link;
	unsigned int flags;
	unsigned int level;
	const char *name;
	unsigned int type;
};

struct perflock_data {
	unsigned int *perf_acpu_table;
	unsigned int table_size;
};

struct perflock_pdata {
	struct perflock_data *perf_floor;
	struct perflock_data *perf_ceiling;
};


#ifndef CONFIG_PERFLOCK
static inline void perf_lock_init(struct perf_lock *lock, unsigned int type,
	unsigned int level, const char *name) { return; }
static inline void perf_lock(struct perf_lock *lock) { return; }
static inline void perf_unlock(struct perf_lock *lock) { return; }
static inline int is_perf_lock_active(struct perf_lock *lock) { return 0; }
static inline int is_perf_locked(void) { return 0; }
static inline void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu) { return; }
static inline void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu) { return; }
static inline void htc_print_active_perf_locks(void) { return; }
static inline int perflock_override(const struct cpufreq_policy *policy) { return 0; }
static inline struct perf_lock *perflock_acquire(const char *name) { return NULL; }
static inline int perflock_release(const char *name) { return 0; }
#else
extern void perf_lock_init(struct perf_lock *lock, unsigned int type,
	unsigned int level, const char *name);
extern void perf_lock(struct perf_lock *lock);
extern void perf_unlock(struct perf_lock *lock);
extern int is_perf_lock_active(struct perf_lock *lock);
extern int is_perf_locked(void);
extern void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu);
extern void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu);
extern int perflock_override(const struct cpufreq_policy *policy, const unsigned int new_freq);
extern void htc_print_active_perf_locks(void);
extern struct perf_lock *perflock_acquire(const char *name);
extern int perflock_release(const char *name);
#endif


#endif


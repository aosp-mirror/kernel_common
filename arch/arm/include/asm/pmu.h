/*
 *  linux/arch/arm/include/asm/pmu.h
 *
 *  Copyright (C) 2009 picoChip Designs Ltd, Jamie Iles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ARM_PMU_H__
#define __ARM_PMU_H__

#include <linux/interrupt.h>
#include <linux/perf_event.h>

/*
 * Types of PMUs that can be accessed directly and require mutual
 * exclusion between profiling tools.
 */
enum arm_pmu_type {
	ARM_PMU_DEVICE_CPU	= 0,
	ARM_PMU_DEVICE_L2CC	= 1,
	ARM_NUM_PMU_DEVICES,
};

/*
 * struct arm_pmu_platdata - ARM PMU platform data
 *
 * @handle_irq: an optional handler which will be called from the
 *	interrupt and passed the address of the low level handler,
 *	and can be used to implement any platform specific handling
 *	before or after calling it.
 * @request_pmu_irq: an optional handler in case the platform wants
 *	to use a percpu IRQ API call. e.g. request_percpu_irq
 * @free_pmu_irq: an optional handler in case the platform wants
 *	to use a percpu IRQ API call. e.g. free_percpu_irq
 * @enable_irq: an optional handler which will be called after
 *	request_irq and be used to handle some platform specific
 *	irq enablement
 * @disable_irq: an optional handler which will be called before
 *	free_irq and be used to handle some platform specific
 *	irq disablement
 */
struct arm_pmu_platdata {
	irqreturn_t (*handle_irq)(int irq, void *dev,
				  irq_handler_t pmu_handler);
	int	(*request_pmu_irq)(int irq, irq_handler_t *irq_h);
	void	(*free_pmu_irq)(int irq);
	void (*enable_irq)(int irq);
	void (*disable_irq)(int irq);
};

extern int multicore_request_irq(int irq, irq_handler_t *handle_irq);
extern void multicore_free_irq(int irq);
extern struct arm_pmu_platdata multicore_data;

#ifdef CONFIG_CPU_HAS_PMU

/**
 * reserve_pmu() - reserve the hardware performance counters
 *
 * Reserve the hardware performance counters in the system for exclusive use.
 * Returns 0 on success or -EBUSY if the lock is already held.
 */
extern int
reserve_pmu(enum arm_pmu_type type);

/**
 * release_pmu() - Relinquish control of the performance counters
 *
 * Release the performance counters and allow someone else to use them.
 */
extern void
release_pmu(enum arm_pmu_type type);

#else /* CONFIG_CPU_HAS_PMU */

#include <linux/err.h>

static inline int
reserve_pmu(enum arm_pmu_type type)
{
	return -ENODEV;
}

static inline void
release_pmu(enum arm_pmu_type type)	{ }

#endif /* CONFIG_CPU_HAS_PMU */

#ifdef CONFIG_HW_PERF_EVENTS

/* The events for a given PMU register set. */
struct pmu_hw_events {
	/*
	 * The events that are active on the PMU for the given index.
	 */
	struct perf_event	**events;

	/*
	 * A 1 bit for an index indicates that the counter is being used for
	 * an event. A 0 means that the counter can be used.
	 */
	unsigned long           *used_mask;

	/*
	 * Hardware lock to serialize accesses to PMU registers. Needed for the
	 * read/modify/write sequences.
	 */
	raw_spinlock_t		pmu_lock;
};

struct arm_pmu {
	struct pmu	pmu;
	enum arm_perf_pmu_ids id;
	enum arm_pmu_type type;
	cpumask_t	active_irqs;
	const char	*name;
	int		num_events;
	atomic_t	active_events;
	struct mutex	reserve_mutex;
	u64		max_period;
	struct platform_device	*plat_device;
	irqreturn_t	(*handle_irq)(int irq_num, void *dev);
	int		(*request_pmu_irq)(int irq, irq_handler_t *irq_h);
	void		(*free_pmu_irq)(int irq);
	void		(*enable)(struct hw_perf_event *evt, int idx, int cpu);
	void		(*disable)(struct hw_perf_event *evt, int idx);
	int		(*get_event_idx)(struct pmu_hw_events *hw_events,
					 struct hw_perf_event *hwc);
	int		(*set_event_filter)(struct hw_perf_event *evt,
					    struct perf_event_attr *attr);
	u32		(*read_counter)(int idx);
	void		(*write_counter)(int idx, u32 val);
	void		(*start)(void);
	void		(*stop)(void);
	void		(*reset)(void *);
	int		(*map_event)(struct perf_event *event);
	struct pmu_hw_events	*(*get_hw_events)(void);
	int	(*test_set_event_constraints)(struct perf_event *event);
	int	(*clear_event_constraints)(struct perf_event *event);
	void		(*save_pm_registers)(void *hcpu);
	void		(*restore_pm_registers)(void *hcpu);
};

#define to_arm_pmu(p) (container_of(p, struct arm_pmu, pmu))

int armpmu_register(struct arm_pmu *armpmu, char *name, int type);

u64 armpmu_event_update(struct perf_event *event,
			struct hw_perf_event *hwc,
			int idx);

int armpmu_event_set_period(struct perf_event *event,
			    struct hw_perf_event *hwc,
			    int idx);

#endif /* CONFIG_HW_PERF_EVENTS */

#endif /* __ARM_PMU_H__ */

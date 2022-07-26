/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Common arm64 stack unwinder code.
 *
 * Copyright (C) 2012 ARM Ltd.
 */
#ifndef __ASM_STACKTRACE_COMMON_H
#define __ASM_STACKTRACE_COMMON_H

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/types.h>

enum stack_type {
	STACK_TYPE_UNKNOWN,
	STACK_TYPE_TASK,
	STACK_TYPE_IRQ,
	STACK_TYPE_OVERFLOW,
	STACK_TYPE_SDEI_NORMAL,
	STACK_TYPE_SDEI_CRITICAL,
	__NR_STACK_TYPES
};

struct stack_info {
	unsigned long low;
	unsigned long high;
	enum stack_type type;
};

/*
 * A snapshot of a frame record or fp/lr register values, along with some
 * accounting information necessary for robust unwinding.
 *
 * @fp:          The fp value in the frame record (or the real fp)
 * @pc:          The lr value in the frame record (or the real lr)
 *
 * @stacks_done: Stacks which have been entirely unwound, for which it is no
 *               longer valid to unwind to.
 *
 * @prev_fp:     The fp that pointed to this frame record, or a synthetic value
 *               of 0. This is used to ensure that within a stack, each
 *               subsequent frame record is at an increasing address.
 * @prev_type:   The type of stack this frame record was on, or a synthetic
 *               value of STACK_TYPE_UNKNOWN. This is used to detect a
 *               transition from one stack to another.
 *
 * @task:        The task being unwound.
 */
struct unwind_state {
	unsigned long fp;
	unsigned long pc;
	DECLARE_BITMAP(stacks_done, __NR_STACK_TYPES);
	unsigned long prev_fp;
	enum stack_type prev_type;
	struct task_struct *task;
};

static inline bool on_overflow_stack(unsigned long sp, unsigned long size,
				     struct stack_info *info);

static inline bool on_stack(unsigned long sp, unsigned long size,
			    unsigned long low, unsigned long high,
			    enum stack_type type, struct stack_info *info)
{
	if (!low)
		return false;

	if (sp < low || sp + size < sp || sp + size > high)
		return false;

	if (info) {
		info->low = low;
		info->high = high;
		info->type = type;
	}
	return true;
}

static inline bool on_accessible_stack_common(const struct task_struct *tsk,
					      unsigned long sp,
					      unsigned long size,
					      struct stack_info *info)
{
	if (info)
		info->type = STACK_TYPE_UNKNOWN;

	/*
	 * Both the kernel and nvhe hypervisor make use of
	 * an overflow_stack
	 */
	return on_overflow_stack(sp, size, info);
}

static inline void unwind_init_common(struct unwind_state *state,
				      struct task_struct *task)
{
	state->task = task;

	/*
	 * Prime the first unwind.
	 *
	 * In unwind_next() we'll check that the FP points to a valid stack,
	 * which can't be STACK_TYPE_UNKNOWN, and the first unwind will be
	 * treated as a transition to whichever stack that happens to be. The
	 * prev_fp value won't be used, but we set it to 0 such that it is
	 * definitely not an accessible stack address.
	 */
	bitmap_zero(state->stacks_done, __NR_STACK_TYPES);
	state->prev_fp = 0;
	state->prev_type = STACK_TYPE_UNKNOWN;
}

#endif	/* __ASM_STACKTRACE_COMMON_H */

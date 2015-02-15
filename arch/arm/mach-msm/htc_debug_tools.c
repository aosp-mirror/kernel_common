/* Copyright (c) 2013, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel_stat.h>
#include <mach/htc_debug_tools.h>

/* n.b.:
 * 1. sched_clock is not irq safe
 * 2. 32 bit: overflows every 4,294,967,296 msecs
 */
unsigned long htc_debug_get_sched_clock_ms(void)
{
	unsigned long long timestamp;
	timestamp = sched_clock();
	do_div(timestamp, NSEC_PER_MSEC);
	return ((unsigned long) timestamp);
}

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)

#define PET_CHECK_THRESHOLD	12
#define NSEC_PER_THRESHOLD	PET_CHECK_THRESHOLD * NSEC_PER_SEC
static int pet_check_counter = PET_CHECK_THRESHOLD;
static unsigned long long last_pet_check;
static unsigned long long htc_debug_watchdog_last_pet;

void htc_debug_watchdog_check_pet(unsigned long long timestamp)
{
	if (!htc_debug_watchdog_enabled() || !htc_debug_watchdog_last_pet)
		return;

	if (timestamp - htc_debug_watchdog_last_pet > (unsigned long long)NSEC_PER_THRESHOLD) {
		if (timestamp - last_pet_check > (unsigned long long)NSEC_PER_SEC) {
			last_pet_check = timestamp;
			/* FIXME:
			 * the value of pet_check_counter is not necessary 'seconds'
			 * also, we could avoid using the variable pet_check_counter
			 */
			pr_info("\n%s: MSM watchdog was blocked for more than %d seconds!\n",
				__func__, pet_check_counter++);
			pr_info("%s: Prepare to dump stack...\n",
				__func__);
			dump_stack();
#if defined(CONFIG_HTC_DEBUG_WORKQUEUE)
			pr_info("%s: Prepare to dump pending works on global workqueue...\n",
				__func__);
			htc_debug_workqueue_show_pending_work_on_gcwq();
#endif /* CONFIG_HTC_DEBUG_WORKQUEUE */
			pr_info("\n ### Show Blocked State ###\n");
			show_state_filter(TASK_UNINTERRUPTIBLE);
		}
	}
}

void htc_debug_watchdog_update_last_pet(unsigned long long last_pet)
{
	htc_debug_watchdog_last_pet = last_pet;
	/* resets the counter */
	pet_check_counter = PET_CHECK_THRESHOLD;
}

/* TODO: support this funciton with CONFIG_SPARSE_IRQ */
#if !defined(CONFIG_SPARSE_IRQ)
static unsigned int last_irqs[NR_IRQS];
void htc_debug_watchdog_dump_irqs(unsigned int dump)
{
	int n;
	if (dump) {
		pr_debug("\nWatchdog dump irqs:\n");
		pr_debug("irqnr       total  since-last   status  name\n");
	}
	for (n = 1; n < NR_IRQS; n++) {
		struct irqaction *act = irq_desc[n].action;
		if (!act && !kstat_irqs(n))
			continue;
		if (dump) {
			pr_debug("%5d: %10u %11u %8x  %s\n", n,
				kstat_irqs(n),
				kstat_irqs(n) - last_irqs[n],
				irq_desc[n].status_use_accessors,
				(act && act->name) ? act->name : "???");
		}
		last_irqs[n] = kstat_irqs(n);
	}
}
#endif /* !defined(CONFIG_SPARSE_IRQ) */

#endif /* CONFIG_HTC_DEBUG_WATCHDOG */

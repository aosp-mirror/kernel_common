/*
 *  linux/arch/arm/kernel/irq.c
 *
 *  Copyright (C) 1992 Linus Torvalds
 *  Modifications for ARM processor Copyright (C) 1995-2000 Russell King.
 *
 *  Support for Dynamic Tick Timer Copyright (C) 2004-2005 Nokia Corporation.
 *  Dynamic Tick Timer written by Tony Lindgren <tony@atomide.com> and
 *  Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This file contains the code used by various IRQ handling routines:
 *  asking for different IRQ's should be done through these routines
 *  instead of just grabbing them. Thus setups with different IRQ numbers
 *  shouldn't result in any weird surprises, and installing new handlers
 *  should be easier.
 *
 *  IRQ's are in fact implemented a bit like signal handlers for the kernel.
 *  Naturally it's not a 1:1 relation, but there are similarities.
 */
#include <linux/kernel_stat.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/random.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/kallsyms.h>
#include <linux/proc_fs.h>
#include <linux/export.h>
#include <linux/slab.h>

#include <asm/exception.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <asm/perftypes.h>

#ifndef irq_finish
#define irq_finish(irq) do { } while (0)
#endif

unsigned long irq_err_count;

int arch_show_interrupts(struct seq_file *p, int prec)
{
#ifdef CONFIG_FIQ
	show_fiq_list(p, prec);
#endif
#ifdef CONFIG_SMP
	show_ipi_list(p, prec);
#endif
	seq_printf(p, "%*s: %10lu\n", prec, "Err", irq_err_count);
	return 0;
}

#ifdef CONFIG_HTC_POWER_DEBUG
unsigned int *previous_irqs;
static int pre_nr_irqs = 0;
static void htc_show_interrupt(int i)
{
        struct irqaction *action;
        unsigned long flags;
        struct irq_desc *desc;

        if (i < nr_irqs) {
                desc = irq_to_desc(i);

		if(desc == NULL) {
			printk("%s:%d irq:%d lookup failed\n", __FUNCTION__, __LINE__, i);
			return;
		}

                raw_spin_lock_irqsave(&desc->lock, flags);
                action = desc->action;
                if (!action)
                        goto unlock;
                if (!(kstat_irqs_cpu(i, 0)) || previous_irqs[i] == (kstat_irqs_cpu(i, 0)))
                        goto unlock;
                printk("%3d:", i);
                printk("%6u\t", kstat_irqs_cpu(i, 0)- previous_irqs[i]);
                printk("%s", action->name);
                for (action = action->next; action; action = action->next)
                        printk(", %s", action->name);
                printk("\n");
                previous_irqs[i] = kstat_irqs_cpu(i, 0);
unlock:
                raw_spin_unlock_irqrestore(&desc->lock, flags);
        } else if (i == nr_irqs) {
                if (previous_irqs[nr_irqs] == irq_err_count)
                        return;
                printk("Err: %lud\n", irq_err_count-previous_irqs[nr_irqs]);
                previous_irqs[nr_irqs] = irq_err_count;
        }
}

void htc_show_interrupts(void)
{
        int i = 0;
		if(pre_nr_irqs != nr_irqs) {
			pre_nr_irqs = nr_irqs;
			previous_irqs = (unsigned int *)kcalloc(nr_irqs, sizeof(int),GFP_KERNEL);
		}
		for (i = 0; i <= nr_irqs; i++)
			htc_show_interrupt(i);
}
#endif

void handle_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	perf_mon_interrupt_in();
	irq_enter();

	if (unlikely(irq >= nr_irqs)) {
		if (printk_ratelimit())
			printk(KERN_WARNING "Bad IRQ%u\n", irq);
		ack_bad_irq(irq);
	} else {
		generic_handle_irq(irq);
	}

	
	irq_finish(irq);

	irq_exit();
	set_irq_regs(old_regs);
	perf_mon_interrupt_out();
}

asmlinkage void __exception_irq_entry
asm_do_IRQ(unsigned int irq, struct pt_regs *regs)
{
	handle_IRQ(irq, regs);
}

void set_irq_flags(unsigned int irq, unsigned int iflags)
{
	unsigned long clr = 0, set = IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_NOAUTOEN;

	if (irq >= nr_irqs) {
		printk(KERN_ERR "Trying to set irq flags for IRQ%d\n", irq);
		return;
	}

	if (iflags & IRQF_VALID)
		clr |= IRQ_NOREQUEST;
	if (iflags & IRQF_PROBE)
		clr |= IRQ_NOPROBE;
	if (!(iflags & IRQF_NOAUTOEN))
		clr |= IRQ_NOAUTOEN;
	
	irq_modify_status(irq, clr, set & ~clr);
}

void __init init_IRQ(void)
{
	machine_desc->init_irq();
}

#ifdef CONFIG_SPARSE_IRQ
int __init arch_probe_nr_irqs(void)
{
	nr_irqs = machine_desc->nr_irqs ? machine_desc->nr_irqs : NR_IRQS;
	return nr_irqs;
}
#endif

#ifdef CONFIG_HOTPLUG_CPU

static bool migrate_one_irq(struct irq_desc *desc)
{
	struct irq_data *d = irq_desc_get_irq_data(desc);
	const struct cpumask *affinity = d->affinity;
	struct irq_chip *c;
	bool ret = false;

	if (irqd_is_per_cpu(d) || !cpumask_test_cpu(smp_processor_id(), affinity))
		return false;

	if (cpumask_any_and(affinity, cpu_online_mask) >= nr_cpu_ids) {
		affinity = cpu_online_mask;
		ret = true;
	}

	c = irq_data_get_irq_chip(d);
	if (!c->irq_set_affinity)
		pr_debug("IRQ%u: unable to set affinity\n", d->irq);
	else if (c->irq_set_affinity(d, affinity, true) == IRQ_SET_MASK_OK && ret)
		cpumask_copy(d->affinity, affinity);

	return ret;
}

void migrate_irqs(void)
{
	unsigned int i;
	struct irq_desc *desc;
	unsigned long flags;

	local_irq_save(flags);

	for_each_irq_desc(i, desc) {
		bool affinity_broken;

		raw_spin_lock(&desc->lock);
		affinity_broken = migrate_one_irq(desc);
		raw_spin_unlock(&desc->lock);

		if (affinity_broken && printk_ratelimit())
			pr_warning("IRQ%u no longer affine to CPU%u\n", i,
				smp_processor_id());
	}

	local_irq_restore(flags);
}
#endif 

/*
 *  linux/arch/arm/common/gic.c
 *
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Interrupt architecture for the GIC:
 *
 * o There is one Interrupt Distributor, which receives interrupts
 *   from system devices and sends them to the Interrupt Controllers.
 *
 * o There is one CPU Interface per CPU, which sends interrupts sent
 *   by the Distributor, and interrupts generated locally, to the
 *   associated CPU. The base address of the CPU interface is usually
 *   aliased so that the same address points to different chips depending
 *   on the CPU it is accessed from.
 *
 * Note that IRQs 0-31 are special - they are local to each CPU.
 * As such, the enable set/clear, pending set/clear and active bit
 * registers are banked per-cpu for these sources.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/cpu_pm.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>

#include <asm/irq.h>
#include <asm/exception.h>
#include <asm/smp_plat.h>
#include <asm/mach/irq.h>
#include <asm/hardware/gic.h>
#include <asm/system.h>

#include <mach/socinfo.h>
#ifdef CONFIG_HTC_POWER_DEBUG
#include <mach/irqs.h>
#endif

#if defined(CONFIG_HTC_DEBUG_WATCHDOG) || defined(CONFIG_HTC_DEBUG_RTB)
#include <mach/htc_debug_tools.h>
#endif
#include <mach/msm_rtb.h>

union gic_base {
	void __iomem *common_base;
	void __percpu __iomem **percpu_base;
};

struct gic_chip_data {
	unsigned int irq_offset;
	union gic_base dist_base;
	union gic_base cpu_base;
	bool need_access_lock;
#ifdef CONFIG_CPU_PM
	u32 saved_spi_enable[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(1020, 16)];
	u32 saved_spi_target[DIV_ROUND_UP(1020, 4)];
	u32 saved_dist_pri[DIV_ROUND_UP(1020, 4)];
	u32 __percpu *saved_ppi_enable;
	u32 __percpu *saved_ppi_conf;
#endif
	u32 saved_dist_isr[DIV_ROUND_UP(1020, 32)];
	struct irq_domain *domain;
	unsigned int gic_irqs;
#ifdef CONFIG_GIC_NON_BANKED
	void __iomem *(*get_base)(union gic_base *);
#endif
	unsigned int max_irq;
#ifdef CONFIG_PM
	unsigned int wakeup_irqs[32];
	unsigned int enabled_irqs[32];
#endif
};

static DEFINE_RAW_SPINLOCK(irq_controller_lock);

#ifdef CONFIG_CPU_PM
static unsigned int saved_dist_ctrl, saved_cpu_ctrl;
#endif

struct irq_chip gic_arch_extn = {
	.irq_eoi	= NULL,
	.irq_mask	= NULL,
	.irq_unmask	= NULL,
	.irq_retrigger	= NULL,
	.irq_set_type	= NULL,
	.irq_set_wake	= NULL,
	.irq_disable	= NULL,
};

#ifndef MAX_GIC_NR
#define MAX_GIC_NR	1
#endif

static struct gic_chip_data gic_data[MAX_GIC_NR] __read_mostly;

#ifdef CONFIG_GIC_NON_BANKED
static void __iomem *gic_get_percpu_base(union gic_base *base)
{
	return *__this_cpu_ptr(base->percpu_base);
}

static void __iomem *gic_get_common_base(union gic_base *base)
{
	return base->common_base;
}

static inline void __iomem *gic_data_dist_base(struct gic_chip_data *data)
{
	return data->get_base(&data->dist_base);
}

static inline void __iomem *gic_data_cpu_base(struct gic_chip_data *data)
{
	return data->get_base(&data->cpu_base);
}

static inline void gic_set_base_accessor(struct gic_chip_data *data,
					 void __iomem *(*f)(union gic_base *))
{
	data->get_base = f;
}
#else
#define gic_data_dist_base(d)	((d)->dist_base.common_base)
#define gic_data_cpu_base(d)	((d)->cpu_base.common_base)
#define gic_set_base_accessor(d,f)
#endif

static inline void __iomem *gic_dist_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_dist_base(gic_data);
}

static inline void __iomem *gic_cpu_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_cpu_base(gic_data);
}

static inline unsigned int gic_irq(struct irq_data *d)
{
	return d->hwirq;
}

#if defined(CONFIG_CPU_V7) && defined(CONFIG_GIC_SECURE)
static const inline bool is_cpu_secure(void)
{
	unsigned int dscr;

	asm volatile ("mrc p14, 0, %0, c0, c1, 0" : "=r" (dscr));

	
	if (BIT(18) & dscr)
		return false;
	else
		return true;
}
#else
static const inline bool is_cpu_secure(void)
{
	return false;
}
#endif

static void gic_mask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_CLEAR + (gic_irq(d) / 32) * 4);
	if (gic_arch_extn.irq_mask)
		gic_arch_extn.irq_mask(d);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_unmask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	if (gic_arch_extn.irq_unmask)
		gic_arch_extn.irq_unmask(d);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_SET + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_disable_irq(struct irq_data *d)
{
	if (gic_arch_extn.irq_disable)
		gic_arch_extn.irq_disable(d);
}

#ifdef CONFIG_PM
static int gic_suspend_one(struct gic_chip_data *gic)
{
	unsigned int i;
	void __iomem *base = gic_data_dist_base(gic);

	for (i = 0; i * 32 < gic->max_irq; i++) {
		gic->enabled_irqs[i]
			= readl_relaxed(base + GIC_DIST_ENABLE_SET + i * 4);
		
		writel_relaxed(0xffffffff, base + GIC_DIST_ENABLE_CLEAR + i * 4);
		
		writel_relaxed(gic->wakeup_irqs[i],
			base + GIC_DIST_ENABLE_SET + i * 4);
	}
	mb();
	return 0;
}

static int gic_suspend(void)
{
	int i;
	for (i = 0; i < MAX_GIC_NR; i++)
		gic_suspend_one(&gic_data[i]);
	return 0;
}

extern int msm_show_resume_irq_mask;

static void gic_show_resume_irq(struct gic_chip_data *gic)
{
	unsigned int i;
	u32 enabled;
	unsigned long pending[32];
	void __iomem *base = gic_data_dist_base(gic);

	if (!msm_show_resume_irq_mask)
		return;

	raw_spin_lock(&irq_controller_lock);
	for (i = 0; i * 32 < gic->max_irq; i++) {
		enabled = readl_relaxed(base + GIC_DIST_ENABLE_CLEAR + i * 4);
		pending[i] = readl_relaxed(base + GIC_DIST_PENDING_SET + i * 4);
		pending[i] &= enabled;
	}
	raw_spin_unlock(&irq_controller_lock);

	for (i = find_first_bit(pending, gic->max_irq);
	     i < gic->max_irq;
	     i = find_next_bit(pending, gic->max_irq, i+1)) {
		pr_warning("%s: %d triggered", __func__,
					i + gic->irq_offset);
#ifdef CONFIG_HTC_POWER_DEBUG
                if (EE0_KRAIT_HLOS_SPMI_PERIPH_IRQ != i + gic->irq_offset)
                        if (TLMM_MSM_SUMMARY_IRQ != i + gic->irq_offset)
                                pr_info("[WAKEUP] Resume caused by gic-%d\n", i + gic->irq_offset);
#endif
	}
}

static void gic_resume_one(struct gic_chip_data *gic)
{
	unsigned int i;
	void __iomem *base = gic_data_dist_base(gic);

	gic_show_resume_irq(gic);
	for (i = 0; i * 32 < gic->max_irq; i++) {
		
		writel_relaxed(0xffffffff, base + GIC_DIST_ENABLE_CLEAR + i * 4);
		
		writel_relaxed(gic->enabled_irqs[i],
			base + GIC_DIST_ENABLE_SET + i * 4);
	}
	mb();
}

static void gic_resume(void)
{
	int i;
	for (i = 0; i < MAX_GIC_NR; i++)
		gic_resume_one(&gic_data[i]);
}

static struct syscore_ops gic_syscore_ops = {
	.suspend = gic_suspend,
	.resume = gic_resume,
};

static int __init gic_init_sys(void)
{
	register_syscore_ops(&gic_syscore_ops);
	return 0;
}
arch_initcall(gic_init_sys);

#endif

static void gic_eoi_irq(struct irq_data *d)
{
	struct gic_chip_data *gic = irq_data_get_irq_chip_data(d);

	if (gic_arch_extn.irq_eoi) {
		raw_spin_lock(&irq_controller_lock);
		gic_arch_extn.irq_eoi(d);
		raw_spin_unlock(&irq_controller_lock);
	}

	if (gic->need_access_lock)
		raw_spin_lock(&irq_controller_lock);
	writel_relaxed_no_log(gic_irq(d), gic_cpu_base(d) + GIC_CPU_EOI);
	if (gic->need_access_lock)
		raw_spin_unlock(&irq_controller_lock);
}

static int gic_set_type(struct irq_data *d, unsigned int type)
{
	void __iomem *base = gic_dist_base(d);
	unsigned int gicirq = gic_irq(d);
	u32 enablemask = 1 << (gicirq % 32);
	u32 enableoff = (gicirq / 32) * 4;
	u32 confmask = 0x2 << ((gicirq % 16) * 2);
	u32 confoff = (gicirq / 16) * 4;
	bool enabled = false;
	u32 val;

	
	if (gicirq < 16)
		return -EINVAL;

	if (type != IRQ_TYPE_LEVEL_HIGH && type != IRQ_TYPE_EDGE_RISING)
		return -EINVAL;

	raw_spin_lock(&irq_controller_lock);

	if (gic_arch_extn.irq_set_type)
		gic_arch_extn.irq_set_type(d, type);

	val = readl_relaxed(base + GIC_DIST_CONFIG + confoff);
	if (type == IRQ_TYPE_LEVEL_HIGH)
		val &= ~confmask;
	else if (type == IRQ_TYPE_EDGE_RISING)
		val |= confmask;

	if (readl_relaxed(base + GIC_DIST_ENABLE_SET + enableoff) & enablemask) {
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_CLEAR + enableoff);
		enabled = true;
	}

	writel_relaxed(val, base + GIC_DIST_CONFIG + confoff);

	if (enabled)
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_SET + enableoff);

	raw_spin_unlock(&irq_controller_lock);

	return 0;
}

static int gic_retrigger(struct irq_data *d)
{
	if (gic_arch_extn.irq_retrigger)
		return gic_arch_extn.irq_retrigger(d);

	
	return 0;
}

#ifdef CONFIG_SMP
static int gic_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
			    bool force)
{
	void __iomem *reg = gic_dist_base(d) + GIC_DIST_TARGET + (gic_irq(d) & ~3);
	unsigned int shift = (gic_irq(d) % 4) * 8;
	unsigned int cpu = cpumask_any_and(mask_val, cpu_online_mask);
	u32 val, mask, bit;

	if (cpu >= 8 || cpu >= nr_cpu_ids)
		return -EINVAL;

	mask = 0xff << shift;
	bit = 1 << (cpu_logical_map(cpu) + shift);

	raw_spin_lock(&irq_controller_lock);
	val = readl_relaxed_no_log(reg) & ~mask;
	writel_relaxed_no_log(val | bit, reg);
	raw_spin_unlock(&irq_controller_lock);

	return IRQ_SET_MASK_OK;
}
#endif

#ifdef CONFIG_PM
static int gic_set_wake(struct irq_data *d, unsigned int on)
{
	int ret = -ENXIO;
	unsigned int reg_offset, bit_offset;
	unsigned int gicirq = gic_irq(d);
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);

	
	WARN_ON(gicirq < 32);

	reg_offset = gicirq / 32;
	bit_offset = gicirq % 32;

	if (on)
		gic_data->wakeup_irqs[reg_offset] |=  1 << bit_offset;
	else
		gic_data->wakeup_irqs[reg_offset] &=  ~(1 << bit_offset);

	if (gic_arch_extn.irq_set_wake)
		ret = gic_arch_extn.irq_set_wake(d, on);

	return ret;
}

#else
#define gic_set_wake	NULL
#endif

asmlinkage void __exception_irq_entry gic_handle_irq(struct pt_regs *regs)
{
	u32 irqstat, irqnr;
	struct gic_chip_data *gic = &gic_data[0];
	void __iomem *cpu_base = gic_data_cpu_base(gic);

	do {
		if (gic->need_access_lock)
			raw_spin_lock(&irq_controller_lock);
		irqstat = readl_relaxed_no_log(cpu_base + GIC_CPU_INTACK);
		if (gic->need_access_lock)
			raw_spin_unlock(&irq_controller_lock);
		irqnr = irqstat & ~0x1c00;

		if (likely(irqnr > 15 && irqnr < 1021)) {
#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
			
			if (irqnr == 19 && smp_processor_id() == 0) {
				unsigned long long timestamp = sched_clock();
#if defined(CONFIG_HTC_DEBUG_RTB)
				unsigned long long timestamp_ms = timestamp;
				do_div(timestamp_ms, NSEC_PER_MSEC);
				uncached_logk_pc(LOGK_IRQ,
					(void *)((unsigned long)timestamp_ms),
					(void *)irqnr);
#endif
				htc_debug_watchdog_check_pet(timestamp);
			}
#if defined(CONFIG_HTC_DEBUG_RTB)
			else {
				uncached_logk_pc(LOGK_IRQ,
					(void *)htc_debug_get_sched_clock_ms(),
					(void *)irqnr);
			}
#endif
#endif 
			irqnr = irq_find_mapping(gic->domain, irqnr);
			handle_IRQ(irqnr, regs);
			uncached_logk(LOGK_IRQ, (void *)(uintptr_t)irqnr);
			continue;
		}
		if (irqnr < 16) {
#if defined(CONFIG_HTC_DEBUG_RTB)
			uncached_logk_pc(LOGK_IRQ,
				(void *)htc_debug_get_sched_clock_ms(),
				(void *)irqnr);
#endif
			if (gic->need_access_lock)
				raw_spin_lock(&irq_controller_lock);
			writel_relaxed_no_log(irqstat, cpu_base + GIC_CPU_EOI);
			if (gic->need_access_lock)
				raw_spin_unlock(&irq_controller_lock);
#ifdef CONFIG_SMP
			handle_IPI(irqnr, regs);
			uncached_logk(LOGK_IRQ, (void *)(uintptr_t)irqnr);
#endif
			continue;
		}
		break;
	} while (1);
}

static void gic_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct gic_chip_data *chip_data = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned int cascade_irq, gic_irq;
	unsigned long status;

	chained_irq_enter(chip, desc);

	raw_spin_lock(&irq_controller_lock);
	status = readl_relaxed(gic_data_cpu_base(chip_data) + GIC_CPU_INTACK);
	raw_spin_unlock(&irq_controller_lock);

	gic_irq = (status & 0x3ff);
	if (gic_irq == 1023)
		goto out;

	cascade_irq = irq_find_mapping(chip_data->domain, gic_irq);
	if (unlikely(gic_irq < 32 || gic_irq > 1020))
		do_bad_IRQ(cascade_irq, desc);
	else
		generic_handle_irq(cascade_irq);

 out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip gic_chip = {
	.name			= "GIC",
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_retrigger		= gic_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
	.irq_disable		= gic_disable_irq,
	.irq_set_wake		= gic_set_wake,
};

void __init gic_cascade_irq(unsigned int gic_nr, unsigned int irq)
{
	if (gic_nr >= MAX_GIC_NR)
		BUG();
	if (irq_set_handler_data(irq, &gic_data[gic_nr]) != 0)
		BUG();
	irq_set_chained_handler(irq, gic_handle_cascade_irq);
}

static void __init gic_dist_init(struct gic_chip_data *gic)
{
	unsigned int i;
	u32 cpumask;
	unsigned int gic_irqs = gic->gic_irqs;
	void __iomem *base = gic_data_dist_base(gic);
	u32 cpu = cpu_logical_map(smp_processor_id());

	cpumask = 1 << cpu;
	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	writel_relaxed(0, base + GIC_DIST_CTRL);

	for (i = 32; i < gic_irqs; i += 16)
		writel_relaxed(0, base + GIC_DIST_CONFIG + i * 4 / 16);

	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);

	if (is_cpu_secure())
		for (i = 32; i < gic_irqs; i += 32)
			writel_relaxed(0xFFFFFFFF,
					base + GIC_DIST_ISR + i * 4 / 32);

	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(0xa0a0a0a0, base + GIC_DIST_PRI + i * 4 / 4);

	for (i = 32; i < gic_irqs; i += 32)
		writel_relaxed(0xffffffff, base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);

	gic->max_irq = gic_irqs;

	if (is_cpu_secure())
		writel_relaxed(3, base + GIC_DIST_CTRL);
	else
		writel_relaxed(1, base + GIC_DIST_CTRL);

	mb();
}

static void __cpuinit gic_cpu_init(struct gic_chip_data *gic)
{
	void __iomem *dist_base = gic_data_dist_base(gic);
	void __iomem *base = gic_data_cpu_base(gic);
	int i;

	if (gic->need_access_lock)
		raw_spin_lock(&irq_controller_lock);
	writel_relaxed(0xffff0000, dist_base + GIC_DIST_ENABLE_CLEAR);
	writel_relaxed(0x0000ffff, dist_base + GIC_DIST_ENABLE_SET);

	
	if (is_cpu_secure())
		writel_relaxed(0xFFFFFFFF, dist_base + GIC_DIST_ISR);

	for (i = 0; i < 32; i += 4)
		writel_relaxed(0xa0a0a0a0, dist_base + GIC_DIST_PRI + i * 4 / 4);

	writel_relaxed(0xf0, base + GIC_CPU_PRIMASK);

	if (is_cpu_secure())
		writel_relaxed(0xF, base + GIC_CPU_CTRL);
	else
		writel_relaxed(1, base + GIC_CPU_CTRL);
	if (gic->need_access_lock)
		raw_spin_unlock(&irq_controller_lock);
    mb();
}

#ifdef CONFIG_CPU_PM
static void gic_dist_save(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	void __iomem *dist_base;
	int i;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	saved_dist_ctrl = readl_relaxed(dist_base + GIC_DIST_CTRL);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		gic_data[gic_nr].saved_spi_conf[i] =
			readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		gic_data[gic_nr].saved_spi_target[i] =
			readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		gic_data[gic_nr].saved_dist_pri[i] =
			readl_relaxed(dist_base + GIC_DIST_PRI + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic_data[gic_nr].saved_spi_enable[i] =
			readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);
	if (is_cpu_secure()) {
		for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
			gic_data[gic_nr].saved_dist_isr[i] =
				readl_relaxed(dist_base + GIC_DIST_ISR + i * 4);
	}
}

static void gic_dist_restore(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	unsigned int i;
	void __iomem *dist_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	writel_relaxed(0, dist_base + GIC_DIST_CTRL);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_conf[i],
			dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(gic_data[gic_nr].saved_dist_pri[i],
			dist_base + GIC_DIST_PRI + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_target[i],
			dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_enable[i],
			dist_base + GIC_DIST_ENABLE_SET + i * 4);

	if (is_cpu_secure()) {
		for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
			writel_relaxed(gic_data[gic_nr].saved_dist_isr[i],
					dist_base + GIC_DIST_ISR + i * 4);
	}

	writel_relaxed(saved_dist_ctrl, dist_base + GIC_DIST_CTRL);
}

static void gic_cpu_save(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	saved_cpu_ctrl = readl_relaxed_no_log(cpu_base + GIC_CPU_CTRL);

	for (i = 0; i < DIV_ROUND_UP(32, 4); i++)
		gic_data[gic_nr].saved_dist_pri[i] = readl_relaxed_no_log(
							dist_base +
							GIC_DIST_PRI + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		ptr[i] = readl_relaxed_no_log(dist_base +
				GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		ptr[i] = readl_relaxed_no_log(dist_base +
				GIC_DIST_CONFIG + i * 4);

}

static void gic_cpu_restore(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		writel_relaxed_no_log(ptr[i], dist_base +
			GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		writel_relaxed_no_log(ptr[i], dist_base +
			GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(32, 4); i++)
		writel_relaxed_no_log(gic_data[gic_nr].saved_dist_pri[i],
			dist_base + GIC_DIST_PRI + i * 4);

	writel_relaxed_no_log(0xf0, cpu_base + GIC_CPU_PRIMASK);
	writel_relaxed_no_log(saved_cpu_ctrl, cpu_base + GIC_CPU_CTRL);
}

static int gic_notifier(struct notifier_block *self, unsigned long cmd,	void *v)
{
	int i;

	for (i = 0; i < MAX_GIC_NR; i++) {
#ifdef CONFIG_GIC_NON_BANKED
		
		if (!gic_data[i].get_base)
			continue;
#endif
		switch (cmd) {
		case CPU_PM_ENTER:
			gic_cpu_save(i);
			break;
		case CPU_PM_ENTER_FAILED:
		case CPU_PM_EXIT:
			gic_cpu_restore(i);
			break;
		case CPU_CLUSTER_PM_ENTER:
			gic_dist_save(i);
			break;
		case CPU_CLUSTER_PM_ENTER_FAILED:
		case CPU_CLUSTER_PM_EXIT:
			gic_dist_restore(i);
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block gic_notifier_block = {
	.notifier_call = gic_notifier,
};

static void __init gic_pm_init(struct gic_chip_data *gic)
{
	gic->saved_ppi_enable = __alloc_percpu(DIV_ROUND_UP(32, 32) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_enable);

	gic->saved_ppi_conf = __alloc_percpu(DIV_ROUND_UP(32, 16) * 4,
		sizeof(u32));
	BUG_ON(!gic->saved_ppi_conf);

	if (gic == &gic_data[0])
		cpu_pm_register_notifier(&gic_notifier_block);
}
#else
static void __init gic_pm_init(struct gic_chip_data *gic)
{
}

static void gic_cpu_restore(unsigned int gic_nr)
{
}

static void gic_cpu_save(unsigned int gic_nr)
{
}

static void gic_dist_restore(unsigned int gic_nr)
{
}

static void gic_dist_save(unsigned int gic_nr)
{
}
#endif

static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	if (hw < 32) {
		irq_set_percpu_devid(irq);
		irq_set_chip_and_handler(irq, &gic_chip,
					 handle_percpu_devid_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
	} else {
		irq_set_chip_and_handler(irq, &gic_chip,
					 handle_fasteoi_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}
	irq_set_chip_data(irq, d->host_data);
	return 0;
}

static int gic_irq_domain_xlate(struct irq_domain *d,
				struct device_node *controller,
				const u32 *intspec, unsigned int intsize,
				unsigned long *out_hwirq, unsigned int *out_type)
{
	if (d->of_node != controller)
		return -EINVAL;
	if (intsize < 3)
		return -EINVAL;

	
	*out_hwirq = intspec[1] + 16;

	
	if (!intspec[0])
		*out_hwirq += 16;

	*out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

const struct irq_domain_ops gic_irq_domain_ops = {
	.map = gic_irq_domain_map,
	.xlate = gic_irq_domain_xlate,
};

void __init gic_init_bases(unsigned int gic_nr, int irq_start,
			   void __iomem *dist_base, void __iomem *cpu_base,
			   u32 percpu_offset, struct device_node *node)
{
	irq_hw_number_t hwirq_base;
	struct gic_chip_data *gic;
	int gic_irqs, irq_base;

	BUG_ON(gic_nr >= MAX_GIC_NR);

	gic = &gic_data[gic_nr];
	if (cpu_is_msm8625() &&
			(SOCINFO_VERSION_MAJOR(socinfo_get_version()) <= 1))
		gic->need_access_lock = true;

#ifdef CONFIG_GIC_NON_BANKED
	if (percpu_offset) { 
		unsigned int cpu;

		gic->dist_base.percpu_base = alloc_percpu(void __iomem *);
		gic->cpu_base.percpu_base = alloc_percpu(void __iomem *);
		if (WARN_ON(!gic->dist_base.percpu_base ||
			     !gic->cpu_base.percpu_base)) {
			free_percpu(gic->dist_base.percpu_base);
			free_percpu(gic->cpu_base.percpu_base);
			return;
		}

		for_each_possible_cpu(cpu) {
			unsigned long offset = percpu_offset * cpu_logical_map(cpu);
			*per_cpu_ptr(gic->dist_base.percpu_base, cpu) = dist_base + offset;
			*per_cpu_ptr(gic->cpu_base.percpu_base, cpu) = cpu_base + offset;
		}

		gic_set_base_accessor(gic, gic_get_percpu_base);
	} else
#endif
	{			
		WARN(percpu_offset,
		     "GIC_NON_BANKED not enabled, ignoring %08x offset!",
		     percpu_offset);
		gic->dist_base.common_base = dist_base;
		gic->cpu_base.common_base = cpu_base;
		gic_set_base_accessor(gic, gic_get_common_base);
	}

	if (gic_nr == 0 && (irq_start & 31) > 0) {
		hwirq_base = 16;
		if (irq_start != -1)
			irq_start = (irq_start & ~31) + 16;
	} else {
		hwirq_base = 32;
	}

	gic_irqs = readl_relaxed(gic_data_dist_base(gic) + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	gic->gic_irqs = gic_irqs;

	gic_irqs -= hwirq_base; 
	irq_base = irq_alloc_descs(irq_start, 16, gic_irqs, numa_node_id());
	if (IS_ERR_VALUE(irq_base)) {
		WARN(1, "Cannot allocate irq_descs @ IRQ%d, assuming pre-allocated\n",
		     irq_start);
		irq_base = irq_start;
	}
	gic->domain = irq_domain_add_legacy(node, gic_irqs, irq_base,
				    hwirq_base, &gic_irq_domain_ops, gic);
	if (WARN_ON(!gic->domain))
		return;

	gic_chip.flags |= gic_arch_extn.flags;
	gic_dist_init(gic);
	gic_cpu_init(gic);
	gic_pm_init(gic);
}

void __cpuinit gic_secondary_init(unsigned int gic_nr)
{
	BUG_ON(gic_nr >= MAX_GIC_NR);

	gic_cpu_init(&gic_data[gic_nr]);
}

#ifdef CONFIG_SMP
void gic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	int cpu;
	unsigned long sgir;
	unsigned long map = 0;
	unsigned long flags = 0;
	struct gic_chip_data *gic = &gic_data[0];

	
	for_each_cpu(cpu, mask)
		map |= 1 << cpu_logical_map(cpu);

	sgir = (map << 16) | irq;
	if (is_cpu_secure())
		sgir |= (1 << 15);

	dsb();

	if (gic->need_access_lock)
		raw_spin_lock_irqsave(&irq_controller_lock, flags);
	
	writel_relaxed_no_log(sgir, gic_data_dist_base(gic) + GIC_DIST_SOFTINT);
	if (gic->need_access_lock)
		raw_spin_unlock_irqrestore(&irq_controller_lock, flags);
	mb();
}
#endif

void gic_set_irq_secure(unsigned int irq)
{
	unsigned int gicd_isr_reg, gicd_pri_reg;
	unsigned int mask = 0xFFFFFF00;
	struct gic_chip_data *gic_data = &gic_data[0];
	struct irq_data *d = irq_get_irq_data(irq);

	if (is_cpu_secure()) {
		raw_spin_lock(&irq_controller_lock);
		gicd_isr_reg = readl_relaxed(gic_dist_base(d) +
				GIC_DIST_ISR + gic_irq(d) / 32 * 4);
		gicd_isr_reg &= ~BIT(gic_irq(d) % 32);
		writel_relaxed(gicd_isr_reg, gic_dist_base(d) +
				GIC_DIST_ISR + gic_irq(d) / 32 * 4);
		
		gicd_pri_reg = readl_relaxed(gic_dist_base(d) +
					GIC_DIST_PRI + (gic_irq(d) * 4 / 4));
		gicd_pri_reg &= mask;
		gicd_pri_reg |= 0x80; 
		writel_relaxed(gicd_pri_reg, gic_dist_base(d) + GIC_DIST_PRI +
				gic_irq(d) * 4 / 4);
		mb();
		raw_spin_unlock(&irq_controller_lock);
	} else {
		WARN(1, "Trying to run secure operation from Non-secure mode");
	}
}

#ifdef CONFIG_OF
static int gic_cnt __initdata = 0;

int __init gic_of_init(struct device_node *node, struct device_node *parent)
{
	void __iomem *cpu_base;
	void __iomem *dist_base;
	u32 percpu_offset;
	int irq;

	if (WARN_ON(!node))
		return -ENODEV;

	dist_base = of_iomap(node, 0);
	WARN(!dist_base, "unable to map gic dist registers\n");

	cpu_base = of_iomap(node, 1);
	WARN(!cpu_base, "unable to map gic cpu registers\n");

	if (of_property_read_u32(node, "cpu-offset", &percpu_offset))
		percpu_offset = 0;

	gic_init_bases(gic_cnt, -1, dist_base, cpu_base, percpu_offset, node);

	if (parent) {
		irq = irq_of_parse_and_map(node, 0);
		gic_cascade_irq(gic_cnt, irq);
	}
	gic_cnt++;
	return 0;
}
#endif
bool gic_is_irq_pending(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	struct gic_chip_data *gic_data = &gic_data[0];
	u32 mask, val;

	WARN_ON(!irqs_disabled());
	raw_spin_lock(&irq_controller_lock);
	mask = 1 << (gic_irq(d) % 32);
	val = readl(gic_dist_base(d) +
			GIC_DIST_ENABLE_SET + (gic_irq(d) / 32) * 4);
	
	WARN_ON(val & mask);
	val = readl(gic_dist_base(d) +
			GIC_DIST_PENDING_SET + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
	return (bool) (val & mask);
}

void gic_clear_irq_pending(unsigned int irq)
{
	struct gic_chip_data *gic_data = &gic_data[0];
	struct irq_data *d = irq_get_irq_data(irq);

	u32 mask, val;
	WARN_ON(!irqs_disabled());
	raw_spin_lock(&irq_controller_lock);
	mask = 1 << (gic_irq(d) % 32);
	val = readl(gic_dist_base(d) +
			GIC_DIST_ENABLE_SET + (gic_irq(d) / 32) * 4);
	
	WARN_ON(val & mask);
	writel(mask, gic_dist_base(d) +
			GIC_DIST_PENDING_CLEAR + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
}

#ifdef CONFIG_ARCH_MSM8625
unsigned int msm_gic_spi_ppi_pending(void)
{
	unsigned int i, bit = 0;
	unsigned int pending_enb = 0, pending = 0;
	unsigned long value = 0;
	struct gic_chip_data *gic = &gic_data[0];
	void __iomem *base = gic_data_dist_base(gic);
	unsigned long flags;

	raw_spin_lock_irqsave(&irq_controller_lock, flags);
	for (i = 0; (i * 32) < gic->max_irq; i++) {
		pending = readl_relaxed(base +
				GIC_DIST_PENDING_SET + i * 4);
		pending_enb = readl_relaxed(base +
				GIC_DIST_ENABLE_SET + i * 4);
		value = pending & pending_enb;

		if (value) {
			for (bit = 0; bit < 32; bit++) {
				bit = find_next_bit(&value, 32, bit);
				if ((bit + 32 * i) != MSM8625_INT_A9_M2A_5) {
					raw_spin_unlock_irqrestore(
						&irq_controller_lock, flags);
					return 1;
				}
			}
		}
	}
	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);

	return 0;
}
#endif

void msm_gic_save(void)
{
	unsigned int i;
	struct gic_chip_data *gic = &gic_data[0];
	void __iomem *base = gic_data_dist_base(gic);

	gic_cpu_save(0);
	gic_dist_save(0);

	
	for (i = 0; (i * 32) < gic->max_irq; i++) {
		raw_spin_lock(&irq_controller_lock);
		writel_relaxed(0xffffffff, base
				+ GIC_DIST_ENABLE_CLEAR + i * 4);
		raw_spin_unlock(&irq_controller_lock);
	}
}

void msm_gic_restore(void)
{
	gic_dist_restore(0);
	gic_cpu_restore(0);
}

void gic_configure_and_raise(unsigned int irq, unsigned int cpu)
{
	struct gic_chip_data *gic = &gic_data[0];
	struct irq_data *d = irq_get_irq_data(irq);
	void __iomem *base = gic_data_dist_base(gic);
	unsigned int value = 0, byte_offset, offset, bit;
	unsigned long flags;

	offset = ((gic_irq(d) / 32) * 4);
	bit = BIT(gic_irq(d) % 32);

	raw_spin_lock_irqsave(&irq_controller_lock, flags);

	value = __raw_readl(base + GIC_DIST_ACTIVE_BIT + offset);
	__raw_writel(value | bit, base + GIC_DIST_ACTIVE_BIT + offset);
	mb();

	value = __raw_readl(base + GIC_DIST_TARGET + (gic_irq(d) / 4) * 4);
	byte_offset = (gic_irq(d) % 4) * 8;
	value |= 1 << (cpu + byte_offset);
	__raw_writel(value, base + GIC_DIST_TARGET + (gic_irq(d) / 4) * 4);
	mb();

	value =  __raw_readl(base + GIC_DIST_ENABLE_SET + offset);
	__raw_writel(value | bit, base + GIC_DIST_ENABLE_SET + offset);
	mb();

	raw_spin_unlock_irqrestore(&irq_controller_lock, flags);
}

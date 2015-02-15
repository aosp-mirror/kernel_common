/*
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *  Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/regulator/krait-regulator.h>

#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/mach-types.h>
#include <asm/smp_plat.h>

#include <mach/socinfo.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <mach/htc_footprint.h>
#endif

#include "pm.h"
#include "platsmp.h"
#include "scm-boot.h"
#include "spm.h"

#define VDD_SC1_ARRAY_CLAMP_GFS_CTL 0x15A0
#define SCSS_CPU1CORE_RESET 0xD80
#define SCSS_DBG_STATUS_CORE_PWRDUP 0xE64

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen".
 */
volatile int pen_release = -1;

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	WARN_ON(msm_platform_secondary_init(cpu));

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int __cpuinit release_secondary_sim(unsigned long base, unsigned int cpu)
{
	void *base_ptr = ioremap_nocache(base + (cpu * 0x10000), SZ_4K);
	if (!base_ptr)
		return -ENODEV;

	writel_relaxed(0x800, base_ptr+0x04);
	writel_relaxed(0x3FFF, base_ptr+0x14);

	mb();
	iounmap(base_ptr);
	return 0;
}

static int __cpuinit scorpion_release_secondary(void)
{
	void *base_ptr = ioremap_nocache(0x00902000, SZ_4K*2);
	if (!base_ptr)
		return -EINVAL;

	writel_relaxed(0, base_ptr + VDD_SC1_ARRAY_CLAMP_GFS_CTL);
	dmb();
	writel_relaxed(0, base_ptr + SCSS_CPU1CORE_RESET);
	writel_relaxed(3, base_ptr + SCSS_DBG_STATUS_CORE_PWRDUP);
	mb();
	iounmap(base_ptr);

	return 0;
}

static int __cpuinit msm8960_release_secondary(unsigned long base,
						unsigned int cpu)
{
	void *base_ptr = ioremap_nocache(base + (cpu * 0x10000), SZ_4K);
	if (!base_ptr)
		return -ENODEV;

	msm_spm_turn_on_cpu_rail(cpu);

	writel_relaxed(0x109, base_ptr+0x04);
	writel_relaxed(0x101, base_ptr+0x04);
	mb();
	ndelay(300);

	writel_relaxed(0x121, base_ptr+0x04);
	mb();
	udelay(2);

	writel_relaxed(0x120, base_ptr+0x04);
	mb();
	udelay(2);

	writel_relaxed(0x100, base_ptr+0x04);
	mb();
	udelay(100);

	writel_relaxed(0x180, base_ptr+0x04);
	mb();
	iounmap(base_ptr);
	return 0;
}

static int __cpuinit msm8974_release_secondary(unsigned long base,
						unsigned int cpu)
{
	void *base_ptr = ioremap_nocache(base + (cpu * 0x10000), SZ_4K);

	if (!base_ptr)
		return -ENODEV;

	secondary_cpu_hs_init(base_ptr, cpu);

	writel_relaxed(0x021, base_ptr+0x04);
	mb();
	udelay(2);

	writel_relaxed(0x020, base_ptr+0x04);
	mb();
	udelay(2);

	writel_relaxed(0x000, base_ptr+0x04);
	mb();

	writel_relaxed(0x080, base_ptr+0x04);
	mb();
	iounmap(base_ptr);
	return 0;
}

static int __cpuinit arm_release_secondary(unsigned long base, unsigned int cpu)
{
	void *base_ptr = ioremap_nocache(base + (cpu * 0x10000), SZ_4K);
	if (!base_ptr)
		return -ENODEV;

	writel_relaxed(0x00000033, base_ptr+0x04);
	mb();

	writel_relaxed(0x10000001, base_ptr+0x14);
	mb();
	udelay(2);

	writel_relaxed(0x00000031, base_ptr+0x04);
	mb();

	writel_relaxed(0x00000039, base_ptr+0x04);
	mb();
	udelay(2);

	writel_relaxed(0x00020038, base_ptr+0x04);
	mb();
	udelay(2);


	writel_relaxed(0x00020008, base_ptr+0x04);
	mb();

	writel_relaxed(0x00020088, base_ptr+0x04);
	mb();

	iounmap(base_ptr);
	return 0;
}

static int __cpuinit release_from_pen(unsigned int cpu)
{
	unsigned long timeout;

	/* Set preset_lpj to avoid subsequent lpj recalculations */
	preset_lpj = loops_per_jiffy;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */
	gic_raise_softirq(cpumask_of(cpu), 1);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

DEFINE_PER_CPU(int, cold_boot_done);

int __cpuinit scorpion_boot_secondary(unsigned int cpu,
				      struct task_struct *idle)
{
	pr_debug("Starting secondary CPU %d\n", cpu);

	if (per_cpu(cold_boot_done, cpu) == false) {
		scorpion_release_secondary();
		per_cpu(cold_boot_done, cpu) = true;
	}
	return release_from_pen(cpu);
}

int __cpuinit msm8960_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	pr_debug("Starting secondary CPU %d\n", cpu);

	if (per_cpu(cold_boot_done, cpu) == false) {
		msm8960_release_secondary(0x02088000, cpu);
		per_cpu(cold_boot_done, cpu) = true;
	}
	return release_from_pen(cpu);
}

int __cpuinit msm8974_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	pr_debug("Starting secondary CPU %d\n", cpu);

	if (per_cpu(cold_boot_done, cpu) == false) {
		if (of_board_is_sim())
			release_secondary_sim(0xf9088000, cpu);
		else if (!of_board_is_rumi())
			msm8974_release_secondary(0xf9088000, cpu);

		per_cpu(cold_boot_done, cpu) = true;
	}
	return release_from_pen(cpu);
}

int __cpuinit arm_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	pr_debug("Starting secondary CPU %d\n", cpu);

	if (per_cpu(cold_boot_done, cpu) == false) {
		if (of_board_is_sim())
			release_secondary_sim(0xf9088000, cpu);
		else if (!of_board_is_rumi())
			arm_release_secondary(0xf9088000, cpu);

		per_cpu(cold_boot_done, cpu) = true;
	}
	return release_from_pen(cpu);
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init msm_smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

static void __init arm_smp_init_cpus(void)
{
	unsigned int i, ncores;

	ncores = (__raw_readl(MSM_APCS_GCC_BASE + 0x30)) & 0xF;

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

static int cold_boot_flags[] __initdata = {
	0,
	SCM_FLAG_COLDBOOT_CPU1,
	SCM_FLAG_COLDBOOT_CPU2,
	SCM_FLAG_COLDBOOT_CPU3,
};

static void __init msm_platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int cpu, map;
	unsigned int flags = 0;

	for_each_present_cpu(cpu) {
		map = cpu_logical_map(cpu);
		if (map > ARRAY_SIZE(cold_boot_flags)) {
			set_cpu_present(cpu, false);
			__WARN();
			continue;
		}
		flags |= cold_boot_flags[map];
	}

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	init_cpu_debug_counter_for_cold_boot();
#endif

	if (scm_set_boot_addr(virt_to_phys(msm_secondary_startup), flags))
		pr_warn("Failed to set CPU boot address\n");
}

struct smp_operations arm_smp_ops __initdata = {
	.smp_init_cpus = arm_smp_init_cpus,
	.smp_prepare_cpus = msm_platform_smp_prepare_cpus,
	.smp_secondary_init = platform_secondary_init,
	.smp_boot_secondary = arm_boot_secondary,
	.cpu_kill = platform_cpu_kill,
	.cpu_die = platform_cpu_die,
	.cpu_disable = platform_cpu_disable
};

struct smp_operations msm8974_smp_ops __initdata = {
	.smp_init_cpus = msm_smp_init_cpus,
	.smp_prepare_cpus = msm_platform_smp_prepare_cpus,
	.smp_secondary_init = platform_secondary_init,
	.smp_boot_secondary = msm8974_boot_secondary,
	.cpu_kill = platform_cpu_kill,
	.cpu_die = platform_cpu_die,
	.cpu_disable = platform_cpu_disable
};

struct smp_operations msm8960_smp_ops __initdata = {
	.smp_init_cpus = msm_smp_init_cpus,
	.smp_prepare_cpus = msm_platform_smp_prepare_cpus,
	.smp_secondary_init = platform_secondary_init,
	.smp_boot_secondary = msm8960_boot_secondary,
	.cpu_kill = platform_cpu_kill,
	.cpu_die = platform_cpu_die,
	.cpu_disable = platform_cpu_disable
};

struct smp_operations scorpion_smp_ops __initdata = {
	.smp_init_cpus = msm_smp_init_cpus,
	.smp_prepare_cpus = msm_platform_smp_prepare_cpus,
	.smp_secondary_init = platform_secondary_init,
	.smp_boot_secondary = scorpion_boot_secondary,
	.cpu_kill = platform_cpu_kill,
	.cpu_die = platform_cpu_die,
	.cpu_disable = platform_cpu_disable
};

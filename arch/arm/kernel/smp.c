/*
 *  linux/arch/arm/kernel/smp.c
 *
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cache.h>
#include <linux/profile.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <linux/completion.h>

#include <linux/atomic.h>
#include <asm/smp.h>
#include <asm/cacheflush.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/exception.h>
#include <asm/idmap.h>
#include <asm/topology.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/ptrace.h>
#include <asm/localtimer.h>
#include <asm/smp_plat.h>
#include <asm/mach/arch.h>

struct secondary_data secondary_data;

enum ipi_msg_type {
	IPI_CPU_START = 1,
	IPI_TIMER = 2,
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_CPU_BACKTRACE,
};

static DECLARE_COMPLETION(cpu_running);

static struct smp_operations smp_ops;

void __init smp_set_ops(struct smp_operations *ops)
{
	if (ops)
		smp_ops = *ops;
};

int __cpuinit __cpu_up(unsigned int cpu)
{
	struct cpuinfo_arm *ci = &per_cpu(cpu_data, cpu);
	struct task_struct *idle = ci->idle;
	int ret;

	if (!idle) {
		idle = fork_idle(cpu);
		if (IS_ERR(idle)) {
			printk(KERN_ERR "CPU%u: fork() failed\n", cpu);
			return PTR_ERR(idle);
		}
		ci->idle = idle;
	} else {
		init_idle(idle, cpu);
	}

	secondary_data.stack = task_stack_page(idle) + THREAD_START_SP;
	secondary_data.pgdir = virt_to_phys(idmap_pgd);
	secondary_data.swapper_pg_dir = virt_to_phys(swapper_pg_dir);
	__cpuc_flush_dcache_area(&secondary_data, sizeof(secondary_data));
	outer_clean_range(__pa(&secondary_data), __pa(&secondary_data + 1));

	ret = boot_secondary(cpu, idle);
	if (ret == 0) {
		wait_for_completion_timeout(&cpu_running,
						 msecs_to_jiffies(1000));

		if (!cpu_online(cpu)) {
			pr_crit("CPU%u: failed to come online\n", cpu);
			ret = -EIO;
		}
	} else {
		pr_err("CPU%u: failed to boot: %d\n", cpu, ret);
	}

	secondary_data.stack = NULL;
	secondary_data.pgdir = 0;

	return ret;
}

void __attribute__((weak)) __init smp_init_cpus(void)
{
	if (smp_ops.smp_init_cpus)
		smp_ops.smp_init_cpus();
}

void __attribute__((weak)) __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	if (smp_ops.smp_prepare_cpus)
		smp_ops.smp_prepare_cpus(max_cpus);
}

void __attribute__((weak)) __cpuinit platform_secondary_init(unsigned int cpu)
{
	if (smp_ops.smp_secondary_init)
		smp_ops.smp_secondary_init(cpu);
}

int __attribute__((weak)) __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (smp_ops.smp_boot_secondary)
		return smp_ops.smp_boot_secondary(cpu, idle);
	return -ENOSYS;
}

#ifdef CONFIG_HOTPLUG_CPU
static void percpu_timer_stop(void);

int __attribute__((weak)) platform_cpu_kill(unsigned int cpu)
{
	if (smp_ops.cpu_kill)
		return smp_ops.cpu_kill(cpu);
	return 1;
}

void __attribute__((weak)) platform_cpu_die(unsigned int cpu)
{
	if (smp_ops.cpu_die)
		smp_ops.cpu_die(cpu);
}

int __attribute__((weak)) platform_cpu_disable(unsigned int cpu)
{
	if (smp_ops.cpu_disable)
		return smp_ops.cpu_disable(cpu);

	return cpu == 0 ? -EPERM : 0;
}

int __cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();
	struct task_struct *p;
	int ret;

	ret = platform_cpu_disable(cpu);
	if (ret)
		return ret;

	set_cpu_online(cpu, false);

	migrate_irqs();

	percpu_timer_stop();

	flush_cache_all();
	local_flush_tlb_all();

	read_lock(&tasklist_lock);
	for_each_process(p) {
		if (p->mm)
			cpumask_clear_cpu(cpu, mm_cpumask(p->mm));
	}
	read_unlock(&tasklist_lock);

	return 0;
}

static DECLARE_COMPLETION(cpu_died);

void __cpu_die(unsigned int cpu)
{
	if (!wait_for_completion_timeout(&cpu_died, msecs_to_jiffies(5000))) {
		pr_err("CPU%u: cpu didn't die\n", cpu);
		return;
	}
	pr_debug("CPU%u: shutdown\n", cpu);

	if (!platform_cpu_kill(cpu))
		printk("CPU%u: unable to kill\n", cpu);
}

void __ref cpu_die(void)
{
	unsigned int cpu = smp_processor_id();

	idle_task_exit();

	local_irq_disable();
	mb();

	
	complete(&cpu_died);

	platform_cpu_die(cpu);

	__asm__("mov	sp, %0\n"
	"	mov	fp, #0\n"
	"	b	secondary_start_kernel"
		:
		: "r" (task_stack_page(current) + THREAD_SIZE - 8));
}
#endif 

static void __cpuinit smp_store_cpu_info(unsigned int cpuid)
{
	struct cpuinfo_arm *cpu_info = &per_cpu(cpu_data, cpuid);

	cpu_info->loops_per_jiffy = loops_per_jiffy;

	store_cpu_topology(cpuid);
}

asmlinkage void __cpuinit secondary_start_kernel(void)
{
	struct mm_struct *mm = &init_mm;
	unsigned int cpu = smp_processor_id();

	atomic_inc(&mm->mm_count);
	current->active_mm = mm;
	cpumask_set_cpu(cpu, mm_cpumask(mm));
	cpu_switch_mm(mm->pgd, mm);
	enter_lazy_tlb(mm, current);
	local_flush_tlb_all();

	pr_debug("CPU%u: Booted secondary processor\n", cpu);

	cpu_init();
	preempt_disable();
	trace_hardirqs_off();

	platform_secondary_init(cpu);

	notify_cpu_starting(cpu);

	calibrate_delay();

	smp_store_cpu_info(cpu);

	set_cpu_online(cpu, true);
	complete(&cpu_running);

	percpu_timer_setup();

	local_irq_enable();
	local_fiq_enable();

	cpu_idle();
}

void __init smp_cpus_done(unsigned int max_cpus)
{
	int cpu;
	unsigned long bogosum = 0;

	for_each_online_cpu(cpu)
		bogosum += per_cpu(cpu_data, cpu).loops_per_jiffy;

	printk(KERN_INFO "SMP: Total of %d processors activated "
	       "(%lu.%02lu BogoMIPS).\n",
	       num_online_cpus(),
	       bogosum / (500000/HZ),
	       (bogosum / (5000/HZ)) % 100);
}

void __init smp_prepare_boot_cpu(void)
{
	unsigned int cpu = smp_processor_id();

	per_cpu(cpu_data, cpu).idle = current;
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = num_possible_cpus();

	init_cpu_topology();

	smp_store_cpu_info(smp_processor_id());

	if (max_cpus > ncores)
		max_cpus = ncores;
	if (ncores > 1 && max_cpus) {
		percpu_timer_setup();

		init_cpu_present(cpu_possible_mask);

		platform_smp_prepare_cpus(max_cpus);
	}
}

static void (*smp_cross_call)(const struct cpumask *, unsigned int);

void __init set_smp_cross_call(void (*fn)(const struct cpumask *, unsigned int))
{
	smp_cross_call = fn;
}

void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	smp_cross_call(mask, IPI_CALL_FUNC);
}

void arch_send_call_function_single_ipi(int cpu)
{
	smp_cross_call(cpumask_of(cpu), IPI_CALL_FUNC_SINGLE);
}

static const char *ipi_types[NR_IPI] = {
#define S(x,s)	[x - IPI_CPU_START] = s
	S(IPI_CPU_START, "CPU start interrupts"),
	S(IPI_TIMER, "Timer broadcast interrupts"),
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
	S(IPI_CPU_BACKTRACE, "CPU backtrace"),
};

void show_ipi_list(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI; i++) {
		seq_printf(p, "%*s%u: ", prec - 1, "IPI", i);

		for_each_present_cpu(cpu)
			seq_printf(p, "%10u ",
				   __get_irq_stat(cpu, ipi_irqs[i]));

		seq_printf(p, " %s\n", ipi_types[i]);
	}
}

u64 smp_irq_stat_cpu(unsigned int cpu)
{
	u64 sum = 0;
	int i;

	for (i = 0; i < NR_IPI; i++)
		sum += __get_irq_stat(cpu, ipi_irqs[i]);

	return sum;
}

static DEFINE_PER_CPU(struct clock_event_device, percpu_clockevent);

static void ipi_timer(void)
{
	struct clock_event_device *evt = &__get_cpu_var(percpu_clockevent);
	evt->event_handler(evt);
}

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
static void smp_timer_broadcast(const struct cpumask *mask)
{
	smp_cross_call(mask, IPI_TIMER);
}
#else
#define smp_timer_broadcast	NULL
#endif

static void broadcast_timer_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt)
{
}

static void __cpuinit broadcast_timer_setup(struct clock_event_device *evt)
{
	evt->name	= "dummy_timer";
	evt->features	= CLOCK_EVT_FEAT_ONESHOT |
			  CLOCK_EVT_FEAT_PERIODIC |
			  CLOCK_EVT_FEAT_DUMMY;
	evt->rating	= 400;
	evt->mult	= 1;
	evt->set_mode	= broadcast_timer_set_mode;

	clockevents_register_device(evt);
}

static struct local_timer_ops *lt_ops;

#ifdef CONFIG_LOCAL_TIMERS
int local_timer_register(struct local_timer_ops *ops)
{
	if (lt_ops)
		return -EBUSY;

	lt_ops = ops;
	return 0;
}
#endif

void __cpuinit percpu_timer_setup(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *evt = &per_cpu(percpu_clockevent, cpu);

	evt->cpumask = cpumask_of(cpu);
	evt->broadcast = smp_timer_broadcast;

	if (!lt_ops || lt_ops->setup(evt))
		broadcast_timer_setup(evt);
}

#ifdef CONFIG_HOTPLUG_CPU
static void percpu_timer_stop(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *evt = &per_cpu(percpu_clockevent, cpu);

	if (lt_ops)
		lt_ops->stop(evt);
}
#endif

static DEFINE_RAW_SPINLOCK(stop_lock);

DEFINE_PER_CPU(struct pt_regs, regs_before_stop);
static void ipi_cpu_stop(unsigned int cpu, struct pt_regs *regs)
{
	if (system_state == SYSTEM_BOOTING ||
	    system_state == SYSTEM_RUNNING) {
		per_cpu(regs_before_stop, cpu) = *regs;
		raw_spin_lock(&stop_lock);
		printk(KERN_CRIT "CPU%u: stopping\n", cpu);
		dump_stack();
		raw_spin_unlock(&stop_lock);
	}

	set_cpu_active(cpu, false);

	local_fiq_disable();
	local_irq_disable();

	flush_cache_all();

	while (1)
		cpu_relax();
}

static cpumask_t backtrace_mask;
static DEFINE_RAW_SPINLOCK(backtrace_lock);

static unsigned long backtrace_flag;

void smp_send_all_cpu_backtrace(void)
{
	unsigned int this_cpu = smp_processor_id();
	int i;

	if (test_and_set_bit(0, &backtrace_flag))
		return;

	cpumask_copy(&backtrace_mask, cpu_online_mask);
	cpu_clear(this_cpu, backtrace_mask);

	pr_info("Backtrace for cpu %d (current):\n", this_cpu);
	dump_stack();

	pr_info("\nsending IPI to all other CPUs:\n");
	if (!cpus_empty(backtrace_mask))
		smp_cross_call(&backtrace_mask, IPI_CPU_BACKTRACE);

	
	for (i = 0; i < 10 * 1000; i++) {
		if (cpumask_empty(&backtrace_mask))
			break;
		mdelay(1);
	}

	clear_bit(0, &backtrace_flag);
	smp_mb__after_clear_bit();
}

static void ipi_cpu_backtrace(unsigned int cpu, struct pt_regs *regs)
{
	if (cpu_isset(cpu, backtrace_mask)) {
		raw_spin_lock(&backtrace_lock);
		pr_warning("IPI backtrace for cpu %d\n", cpu);
		show_regs(regs);
		raw_spin_unlock(&backtrace_lock);
		cpu_clear(cpu, backtrace_mask);
	}
}

asmlinkage void __exception_irq_entry do_IPI(int ipinr, struct pt_regs *regs)
{
	handle_IPI(ipinr, regs);
}

void handle_IPI(int ipinr, struct pt_regs *regs)
{
	unsigned int cpu = smp_processor_id();
	struct pt_regs *old_regs = set_irq_regs(regs);

	if (ipinr >= IPI_CPU_START && ipinr < IPI_CPU_START + NR_IPI)
		__inc_irq_stat(cpu, ipi_irqs[ipinr - IPI_CPU_START]);

	switch (ipinr) {
	case IPI_CPU_START:
		
		break;
	case IPI_TIMER:
		irq_enter();
		ipi_timer();
		irq_exit();
		break;

	case IPI_RESCHEDULE:
		scheduler_ipi();
		break;

	case IPI_CALL_FUNC:
		irq_enter();
		generic_smp_call_function_interrupt();
		irq_exit();
		break;

	case IPI_CALL_FUNC_SINGLE:
		irq_enter();
		generic_smp_call_function_single_interrupt();
		irq_exit();
		break;

	case IPI_CPU_STOP:
		irq_enter();
		ipi_cpu_stop(cpu, regs);
		irq_exit();
		break;

	case IPI_CPU_BACKTRACE:
		ipi_cpu_backtrace(cpu, regs);
		break;

	default:
		printk(KERN_CRIT "CPU%u: Unknown IPI message 0x%x\n",
		       cpu, ipinr);
		break;
	}
	set_irq_regs(old_regs);
}

void smp_send_reschedule(int cpu)
{
	BUG_ON(cpu_is_offline(cpu));
	smp_cross_call(cpumask_of(cpu), IPI_RESCHEDULE);
}

#ifdef CONFIG_HOTPLUG_CPU
static void smp_kill_cpus(cpumask_t *mask)
{
	unsigned int cpu;
	for_each_cpu(cpu, mask)
		platform_cpu_kill(cpu);
}
#else
static void smp_kill_cpus(cpumask_t *mask) { }
#endif

void smp_send_stop(void)
{
	unsigned long timeout;
	struct cpumask mask;

	cpumask_copy(&mask, cpu_online_mask);
	cpumask_clear_cpu(smp_processor_id(), &mask);
	if (!cpumask_empty(&mask))
		smp_cross_call(&mask, IPI_CPU_STOP);

	
	timeout = USEC_PER_SEC;
	while (num_active_cpus() > 1 && timeout--)
		udelay(1);

	if (num_active_cpus() > 1)
		pr_warning("SMP: failed to stop secondary CPUs\n");

	smp_kill_cpus(&mask);
}

int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}

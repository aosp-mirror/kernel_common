/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_KVM_CLOCK_H
#define _ASM_X86_KVM_CLOCK_H

#include <linux/percpu.h>
#include <asm/pvclock.h>

extern struct clocksource kvm_clock;

#ifdef CONFIG_KVM_VIRT_SUSPEND_TIMING_GUEST
u64 kvm_get_suspend_time(void);
#else
static inline u64 kvm_get_suspend_time(void)
{
	return 0;
}
#endif

DECLARE_PER_CPU(struct pvclock_vsyscall_time_info *, hv_clock_per_cpu);

static inline struct pvclock_vcpu_time_info *this_cpu_pvti(void)
{
	return &this_cpu_read(hv_clock_per_cpu)->pvti;
}

static inline struct pvclock_vsyscall_time_info *this_cpu_hvclock(void)
{
	return this_cpu_read(hv_clock_per_cpu);
}

#endif /* _ASM_X86_KVM_CLOCK_H */

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_KVM_CLOCK_H
#define _ASM_X86_KVM_CLOCK_H

extern struct clocksource kvm_clock;

#ifdef CONFIG_KVM_VIRT_SUSPEND_TIMING_GUEST
u64 kvm_get_suspend_time(void);
#else
static inline u64 kvm_get_suspend_time(void)
{
	return 0;
}
#endif

#endif /* _ASM_X86_KVM_CLOCK_H */

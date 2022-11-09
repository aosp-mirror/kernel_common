/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Google LLC
 */

#ifndef _UCLAMP_KVM_H_
#define _UCLAMP_KVM_H_

#ifdef CONFIG_HAVE_KVM_UCLAMP_SYNC
bool kvm_arch_uclamp_supported(void);
void kvm_arch_set_vcpu_uclamp(u32 min, u32 max);
#else
static inline bool kvm_arch_uclamp_supported(void)
{
	return false;
}
#define kvm_arch_set_vcpu_uclamp(min, max) do {} while (0)
#endif

#endif /* _UCLAMP_KVM_H_ */

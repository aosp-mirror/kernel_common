// SPDX-License-Identifier: GPL-2.0-only
/*
 * KVM Uclamp service driver
 * Copyright (C) 2022, Google LLC
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/lockdep.h>
#include <linux/uclamp_kvm.h>

#include <asm/arch_timer.h>
#include <asm/hypervisor.h>

enum {
	KVM_UCLAMP_INIT = 0,
	KVM_UCLAMP_SUPPORTED,
	KVM_UCLAMP_UNSUPPORTED,
};

struct uclamp_value {
	u32 min;
	u32 max;
};

static int kvm_arch_uclamp_support __ro_after_init;
/* Store per-vcpu current uclamp values */
static DEFINE_PER_CPU(struct uclamp_value, current_uclamp);

static int __init kvm_arch_uclamp_init(void)
{
	int ret;

	/* Check whether the host supports this service at first. */
	ret = kvm_arm_hyp_service_available(ARM_SMCCC_KVM_FUNC_UCLAMP);
	if (ret <= 0)
		kvm_arch_uclamp_support = KVM_UCLAMP_UNSUPPORTED;
	else
		kvm_arch_uclamp_support = KVM_UCLAMP_SUPPORTED;

	return 0;
}
late_initcall(kvm_arch_uclamp_init);

bool kvm_arch_uclamp_supported(void)
{
	return kvm_arch_uclamp_support == KVM_UCLAMP_SUPPORTED;
}

/* Caller must ensure the preemption is disabled. */
void kvm_arch_set_vcpu_uclamp(u32 min, u32 max)
{
	struct arm_smccc_res hvc_res;
	struct uclamp_value *uc;

	lockdep_assert_preemption_disabled();

	uc = this_cpu_ptr(&current_uclamp);
	/* If the uclamp values are not changed, skip calling the host. */
	if (!kvm_arch_uclamp_supported() || (uc->min == min && uc->max == max))
		return;

	arm_smccc_1_1_invoke(ARM_SMCCC_VENDOR_HYP_KVM_UCLAMP_FUNC_ID,
			     min, max, &hvc_res);
	uc->min = min;
	uc->max = max;
}

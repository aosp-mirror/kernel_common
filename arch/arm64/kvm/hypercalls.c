// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 Arm Ltd.

#include <linux/arm-smccc.h>
#include <linux/kvm_host.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>

#include <asm/kvm_emulate.h>

#include <kvm/arm_hypercalls.h>
#include <kvm/arm_psci.h>


static void kvm_sched_set_uclamp(struct kvm_vcpu *vcpu, u64 *val)
{
	struct sched_attr attr = {
		.sched_flags = SCHED_FLAG_UTIL_CLAMP,
	};
	int ret;

	attr.sched_util_min = smccc_get_arg1(vcpu);
	attr.sched_util_max = smccc_get_arg2(vcpu);

	ret = sched_setattr(current, &attr);

	val[0] = (u64)ret;
	val[1] = 0;
	val[2] = 0;
	val[3] = 0;
}

int kvm_hvc_call_handler(struct kvm_vcpu *vcpu)
{
	u32 func_id = smccc_get_function(vcpu);
	u64 val[4] = {SMCCC_RET_NOT_SUPPORTED};
	u32 feature;
	gpa_t gpa;

	switch (func_id) {
	case ARM_SMCCC_VERSION_FUNC_ID:
		val[0] = ARM_SMCCC_VERSION_1_1;
		break;
	case ARM_SMCCC_ARCH_FEATURES_FUNC_ID:
		feature = smccc_get_arg1(vcpu);
		switch (feature) {
		case ARM_SMCCC_ARCH_WORKAROUND_1:
			switch (arm64_get_spectre_v2_state()) {
			case SPECTRE_VULNERABLE:
				break;
			case SPECTRE_MITIGATED:
				val[0] = SMCCC_RET_SUCCESS;
				break;
			case SPECTRE_UNAFFECTED:
				val[0] = SMCCC_ARCH_WORKAROUND_RET_UNAFFECTED;
				break;
			}
			break;
		case ARM_SMCCC_ARCH_WORKAROUND_2:
			switch (arm64_get_spectre_v4_state()) {
			case SPECTRE_VULNERABLE:
				break;
			case SPECTRE_MITIGATED:
				/*
				 * SSBS everywhere: Indicate no firmware
				 * support, as the SSBS support will be
				 * indicated to the guest and the default is
				 * safe.
				 *
				 * Otherwise, expose a permanent mitigation
				 * to the guest, and hide SSBS so that the
				 * guest stays protected.
				 */
				if (cpus_have_final_cap(ARM64_SSBS))
					break;
				fallthrough;
			case SPECTRE_UNAFFECTED:
				val[0] = SMCCC_RET_NOT_REQUIRED;
				break;
			}
			break;
		case ARM_SMCCC_ARCH_WORKAROUND_3:
			switch (arm64_get_spectre_bhb_state()) {
			case SPECTRE_VULNERABLE:
				break;
			case SPECTRE_MITIGATED:
				val[0] = SMCCC_RET_SUCCESS;
				break;
			case SPECTRE_UNAFFECTED:
				val[0] = SMCCC_ARCH_WORKAROUND_RET_UNAFFECTED;
				break;
			}
			break;
		case ARM_SMCCC_HV_PV_TIME_FEATURES:
			val[0] = SMCCC_RET_SUCCESS;
			break;
		}
		break;
	case ARM_SMCCC_HV_PV_TIME_FEATURES:
		val[0] = kvm_hypercall_pv_features(vcpu);
		break;
	case ARM_SMCCC_HV_PV_TIME_ST:
		gpa = kvm_init_stolen_time(vcpu);
		if (gpa != GPA_INVALID)
			val[0] = gpa;
		break;
	case ARM_SMCCC_VENDOR_HYP_CALL_UID_FUNC_ID:
		val[0] = ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_0;
		val[1] = ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_1;
		val[2] = ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_2;
		val[3] = ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_3;
		break;
	case ARM_SMCCC_VENDOR_HYP_KVM_FEATURES_FUNC_ID:
		val[0] = BIT(ARM_SMCCC_KVM_FUNC_FEATURES);
		val[ARM_SMCCC_KVM_FUNC_UCLAMP / 32] |= BIT(ARM_SMCCC_KVM_FUNC_UCLAMP % 32);
		break;
	case ARM_SMCCC_VENDOR_HYP_KVM_UCLAMP_FUNC_ID:
		kvm_sched_set_uclamp(vcpu, val);
		break;
	case ARM_SMCCC_TRNG_VERSION:
	case ARM_SMCCC_TRNG_FEATURES:
	case ARM_SMCCC_TRNG_GET_UUID:
	case ARM_SMCCC_TRNG_RND32:
	case ARM_SMCCC_TRNG_RND64:
		return kvm_trng_call(vcpu);
	default:
		return kvm_psci_call(vcpu);
	}

	smccc_set_retval(vcpu, val[0], val[1], val[2], val[3]);
	return 1;
}

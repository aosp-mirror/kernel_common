.. SPDX-License-Identifier: GPL-2.0

get_cur_cpufreq support for arm/arm64
=============================

Get_cur_cpufreq support is used to get current frequency(in KHz) of the
current CPU that the vCPU thread is running on.

* ARM_SMCCC_VENDOR_HYP_KVM_GET_CUR_CPUFREQ_FUNC_ID: 0x86000041

This hypercall uses the SMC32/HVC32 calling convention:

ARM_SMCCC_VENDOR_HYP_KVM_GET_CUR_CPUFREQ_FUNC_ID
    ==============    ========    =====================================
    Function ID:      (uint32)    0x86000041
    Return Values:    (int32)     NOT_SUPPORTED(-1) on error, or
                      (uint32)    Frequency in KHz of current CPU that the
                                  vCPU thread is running on.
    Endianness:                   Must be the same endianness
                                  as the host.
    ==============    ========    =====================================

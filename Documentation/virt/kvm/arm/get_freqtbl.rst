.. SPDX-License-Identifier: GPL-2.0

get_freqtbl support for arm/arm64
=============================

Allows guest to query the frequency(in KHz) table of the current CPU that
the vCPU thread is running on.

* ARM_SMCCC_VENDOR_HYP_KVM_GET_CPUFREQ_TBL_FUNC_ID: 0x86000043

This hypercall uses the SMC32/HVC32 calling convention:

ARM_SMCCC_VENDOR_HYP_KVM_GET_CPUFREQ_TBL_FUNC_ID
    ==============    ========    =====================================
    Function ID:      (uint32)    0x86000043
    Arguments:        (uint32)    index of the current CPU's frequency table
    Return Values:    (int32)     NOT_SUPPORTED(-1) on error, or
                      (uint32)    Frequency table entry of requested index
                                  in KHz
                                  of current CPU(r1)
    Endianness:                   Must be the same endianness
                                  as the host.
    ==============    ========    =====================================

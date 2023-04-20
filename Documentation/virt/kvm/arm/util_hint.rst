.. SPDX-License-Identifier: GPL-2.0

Util_hint support for arm64
============================

Util_hint is used for sharing the utilization value from the guest
to the host.

* ARM_SMCCC_HYP_KVM_UTIL_HINT_FUNC_ID: 0x86000042

This hypercall using the SMC32/HVC32 calling convention:

ARM_SMCCC_HYP_KVM_UTIL_HINT_FUNC_ID
    ==============    =========   ============================
    Function ID:      (uint32)    0x86000042
    Arguments:        (uint32)    util value(0-1024) where 1024 represents
                                  the highest performance point normalized
                                  across all CPUs
    Return values:    (int32)     NOT_SUPPORTED(-1) on error.
    Endianness:                   Must be the same endianness
                                  as the host.
    ==============    ========    ============================

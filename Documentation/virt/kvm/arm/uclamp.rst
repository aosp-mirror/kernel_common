.. SPDX-License-Identifier: GPL-2.0

Uclamp_sync support for arm64
============================

Uclamp_sync is used for utilization uclamp value sync between host and
guests. This transfers the uclamp_min and uclamp_max values of the
tasks running in the guest to the host vCPU using a KVM-specific
hypercall.

* ARM_SMCCC_HYP_KVM_UCLAMP_FUNC_ID: 0x86000040

This hypercall using the SMC32/HVC32 calling convention:

ARM_SMCCC_HYP_KVM_UCLAMP_FUNC_ID
    ==============    =========   ============================
    Function ID:      (uint32)    0x86000040
    Arguments:        (uint32)    uclamp min value(0-1024)
                      (uint32)    uclamp max value(0-1024)
    Return values:    (int32)     NOT_SUPPORTED(-1) on error.
    Endianness:                   Must be the same endianness
                                  to the host.
    ==============    ========    ============================

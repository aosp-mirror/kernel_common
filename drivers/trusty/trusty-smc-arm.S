/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/linkage.h>

.arch_extension sec

ENTRY(trusty_smc8)
    /* Save stack location where r3-r7 smc arguments are stored */
    mov     r12, sp

    /* Save original r4-r7 values as caller expects these to be preserved */
    push    {r4-r7}

    /* Save return value pointer and return address */
    push    {r0, lr}

    /* arm abi shifts arguments when returning a struct, shift them back */
    mov     r0, r1
    mov     r1, r2
    mov     r2, r3

    /* Load stack based arguments */
    ldmia   r12, {r3-r7}

    smc     #0

    /* Restore return address and get return value pointer */
    pop     {r12, lr}

    /* Copy 8-register smc return value to struct smc_ret8 return value */
    stmia   r12, {r0-r7}

    /* Restore original r4-r7 values */
    pop     {r4-r7}

    /* Return */
    bx      lr

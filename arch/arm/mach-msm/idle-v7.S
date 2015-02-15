/*
 * Idle processing for ARMv7-based Qualcomm SoCs.
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, 2011-2013 The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/linkage.h>
#include <linux/threads.h>
#include <asm/assembler.h>

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <mach/htc_footprint.h>
#endif

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
/* r0 already has the cpu number */
.macro set_cpu_foot_print_virt, x
	stmfd	sp!, {r5-r7}
	mrc	p15, 0, r5, c0, c0, 5	/* MPIDR */
	ands	r5, r5, #15		/* What CPU am I */
	ldr	r6, =mnemosyne_base			/* Get the base of footprint */
	ldr	r6, [r6]

	cmp	r6, #0
	beq	skip_footprint\@				/* if base is NULL, just skip */

	ldr	r7, =mnemosyne_kernel_footprint_cpu	/* Get the offset of an element */
	add	r6, r6, r7				/* Get the address of an element */

	mov	r7, #\x
	strb	r7, [r6, r5, LSL #2] /* write footprint x, x must be 1 byte*/
	dsb						/* ensure data are written. */
skip_footprint\@:
	ldmfd	sp!, {r5-r7}
.endm
#endif

	.arm
ENTRY(msm_pm_boot_entry)
	mrc     p15, 0, r0, c0, c0, 5    /* MPIDR                          */
	and     r0, r0, #15              /* what CPU am I                  */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	ldr	r1, =mnemosyne_phys				/* Get the base of footprint */
	ldr	r2, =msm_pm_boot_entry
	adr	r3, msm_pm_boot_entry
	add	r1, r1, r3					/* translate virt to phys addr    */
	sub	r1, r1, r2
	ldr	r4, [r1]					/* Keep mnemosyne_phys in R4 */

	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_kernel_footprint_cpu		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	mov	r2, #2              /* write footprint 2 */
	strb	r2, [r3, r0, LSL #2]
	dsb						/* ensure data are written. */
1:
#endif

	ldr	r1, =msm_pc_debug_counters_phys /*phys addr for IMEM reg */
	ldr	r2, =msm_pm_boot_entry
	adr	r3, msm_pm_boot_entry
	add	r1, r1, r3               /* translate virt to phys addr    */
	sub	r1, r1, r2
	ldr	r1,[r1]

	cmp	r1, #0
	beq	skip_pc_debug3
	add	r1, r1, r0, LSL #4	/* debug location for this CPU */
	add	r1, #4			/* warmboot entry counter*/
	ldr	r2, [r1]
	add	r2, #1
	str	r2, [r1]

skip_pc_debug3:
	ldr     r1, =msm_pm_boot_vector
	ldr     r2, =msm_pm_boot_entry
	adr     r3, msm_pm_boot_entry
	add     r1, r1, r3               /* translate virt to phys addr    */
	sub     r1, r1, r2

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_kernel_footprint_cpu		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	mov	r2, #3              /* write footprint 3 */
	strb	r2, [r3, r0, LSL #2]
	dsb						/* ensure data are written. */
1:
#endif

	add     r1, r1, r0, LSL #2       /* locate boot vector for our cpu */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_cpu_reset_vector_address		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	str	r1, [r3, r0, LSL#2]

	ldr	r2, =mnemosyne_cpu_reset_vector_address_value	/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	ldr	r2, [r1]
	str	r2, [r3, r0, LSL#2]
	dsb						/* ensure data are written. */
1:
#endif

	ldr     pc, [r1]                 /* jump                           */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
ENTRY(get_pm_boot_vector_symbol_address)
	ldr	r2, =msm_pm_boot_vector
	str	r2, [r0]
	bx	lr
#endif

3:	.long	.
	.data

	.globl msm_pm_pc_pgd
msm_pm_pc_pgd:
	.long	0x0

	.globl msm_saved_state
msm_saved_state:
	.long	0x0

	.globl msm_saved_state_phys
msm_saved_state_phys:
	.long	0x0

	.globl msm_pm_boot_vector
msm_pm_boot_vector:
	.space  4 * NR_CPUS

	.globl msm_pc_debug_counters_phys
msm_pc_debug_counters_phys:
	.long 0x0

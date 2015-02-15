/*
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *  Copyright (c) 2010, 2012 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <linux/init.h>
#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <mach/htc_footprint.h>
#endif

__CPUINIT

/*
 * MSM specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 *
 * This is executing in physical space with cache's off.
 */
ENTRY(msm_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5 	@ MPIDR
	and	r0, r0, #15		@ What CPU am I

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	ldr	r7, =mnemosyne_phys
	ldr	r1, =msm_secondary_startup
	adr	r2, msm_secondary_startup
	add	r7, r7, r2
	sub	r7, r7, r1						/* virt to phys */
	ldr	r7, [r7]					/* Store mnemosyne_phys in R7 */

	cmp	r7, #0
	beq	skip_footprint					/* If phys is NULL, skip it */

	ldr	r2, =mnemosyne_kernel_exit_counter_from_cpu	/* Get the offset of an element */
	add	r1, r7, r2					/* Get the address of an element */
	ldr     r2, [r1, r0, LSL #2]
	add 	r2, r2, #1
	str	r2, [r1, r0, LSL #2]
	dsb

	ldr	r2, =mnemosyne_kernel_footprint_cpu	/* Get the offset of an element */
	add	r1, r7, r2				/* Get the address of an element */
	ldr	r2, =0x05
	strb	r2, [r1, r0, LSL #2] /* write footprint 5, only the lowest byte */
	dsb

skip_footprint:
#endif

	adr	r4, 1f			@ address of
	ldmia	r4, {r5, r6}		@ load curr addr and pen_rel addr
	sub	r4, r4, r5		@ determine virtual/phys offsets
	add	r6, r6, r4		@ apply
pen:
	ldr	r7, [r6]		@ pen_rel has cpu to remove from reset
	cmp	r7, r0			@ are we lucky?
	bne	pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup
ENDPROC(msm_secondary_startup)

1:	.long	.
	.long	pen_release

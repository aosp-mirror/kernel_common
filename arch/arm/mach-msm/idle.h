/* Copyright (c) 2007-2009,2012 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ARCH_ARM_MACH_MSM_IDLE_H_
#define _ARCH_ARM_MACH_MSM_IDLE_H_

#ifdef CONFIG_CPU_V7
void msm_pm_boot_entry(void);
#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
void get_pm_boot_vector_symbol_address(unsigned *addr);
#endif
extern unsigned long msm_pm_pc_pgd;
extern unsigned long msm_pm_boot_vector[NR_CPUS];
#else
static inline void msm_pm_boot_entry(void)
{
	/* empty */
}
#endif
#endif

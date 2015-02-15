/* Copyright (c) 2010, 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_SCM_BOOT_H
#define __MACH_SCM_BOOT_H

#define SCM_BOOT_ADDR				0x1
#define SCM_FLAG_COLDBOOT_CPU1		0x01
#define SCM_FLAG_COLDBOOT_CPU2		0x08
#define SCM_FLAG_COLDBOOT_CPU3		0x20
#define SCM_FLAG_WARMBOOT_CPU1		0x02
#define SCM_FLAG_WARMBOOT_CPU0		0x04
#define SCM_FLAG_WARMBOOT_CPU2		0x10
#define SCM_FLAG_WARMBOOT_CPU3		0x40

#ifdef CONFIG_MSM_SCM
int scm_set_boot_addr(phys_addr_t addr, unsigned int flags);
#else
static inline int scm_set_boot_addr(phys_addr_t addr, unsigned int flags)
{
	return 0;
}
#endif

#endif

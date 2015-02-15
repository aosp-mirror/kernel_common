/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#ifndef _ARCH_ARM_MACH_MSM_PM_BOOT_H
#define _ARCH_ARM_MACH_MSM_PM_BOOT_H

/* 8x25 specific macros */
#define MPA5_CFG_CTL_REG	0x30
/* end */

enum {
	MSM_PM_BOOT_CONFIG_TZ		    ,
	MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
	MSM_PM_BOOT_CONFIG_REMAP_BOOT_ADDR  ,
};

struct msm_pm_boot_platform_data {
	int mode;
	phys_addr_t  p_addr;
	void __iomem *v_addr;
};

#ifdef CONFIG_PM
int __init msm_pm_boot_init(struct msm_pm_boot_platform_data *pdata);
#else
static inline int __init msm_pm_boot_init(
		struct msm_pm_boot_platform_data *pdata)
{
	return 0;
}
#endif

void msm_pm_boot_config_before_pc(unsigned int cpu, unsigned long entry);
void msm_pm_boot_config_after_pc(unsigned int cpu);

#endif

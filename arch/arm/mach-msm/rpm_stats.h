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

#ifndef __ARCH_ARM_MACH_MSM_RPM_STATS_H
#define __ARCH_ARM_MACH_MSM_RPM_STATS_H

#include <linux/types.h>

struct msm_rpmstats_platform_data {
	phys_addr_t phys_addr_base;
	u32 phys_size;
	u32 version;
};

struct msm_rpm_master_stats_platform_data {
	phys_addr_t phys_addr_base;
	u32 phys_size;
	char **masters;
	/*
	 * RPM maintains PC stats for each master in MSG RAM,
	 * it allocates 256 bytes for this use.
	 * No of masters differs for different targets.
	 * Based on the number of masters, linux rpm stat
	 * driver reads (32 * num_masters) bytes to display
	 * master stats.
	 */
	 s32 num_masters;
	 u32 master_offset;
	 u32 version;
};

#ifdef CONFIG_HTC_POWER_DEBUG
void msm_rpm_dump_stat(void);
int htc_get_xo_vddmin_info(uint32_t *xo_count, uint64_t *xo_time, uint32_t *vddmin_count, uint64_t *vddmin_time);
#endif

#endif

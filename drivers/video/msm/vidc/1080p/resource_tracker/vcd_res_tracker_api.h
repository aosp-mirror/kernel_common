/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
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
#ifndef _VIDEO_720P_RESOURCE_TRACKER_API_H_
#define _VIDEO_720P_RESOURCE_TRACKER_API_H_

#include "vcd_core.h"
#include "vcd_ddl.h"
#include "vcd_ddl_utils.h"

void res_trk_init(struct device *device, u32 irq);
u32 res_trk_power_up(void);
u32 res_trk_power_down(void);
u32 res_trk_enable_clocks(void);
u32 res_trk_disable_clocks(void);
u32 res_trk_get_max_perf_level(u32 *pn_max_perf_lvl);
u32 res_trk_set_perf_level(u32 req_perf_lvl, u32 *pn_set_perf_lvl,
	struct vcd_dev_ctxt *dev_ctxt);
u32 res_trk_get_curr_perf_level(u32 *pn_perf_lvl);
u32 res_trk_download_firmware(void);
u32 res_trk_get_core_type(void);
u32 res_trk_get_firmware_addr(struct ddl_buf_addr *firm_addr);
int res_trk_get_mem_type(void);
unsigned int res_trk_get_ion_flags(void);
u32 res_trk_get_enable_ion(void);
u32 res_trk_is_cp_enabled(void);
u32 res_trk_get_disable_fullhd(void);
struct ion_client *res_trk_get_ion_client(void);
u32 res_trk_get_disable_dmx(void);
u32 res_trk_get_min_dpb_count(void);
void res_trk_set_mem_type(enum ddl_mem_area mem_type);
int res_trk_enable_iommu_clocks(void);
int res_trk_disable_iommu_clocks(void);
int res_trk_check_for_sec_session(void);
int res_trk_open_secure_session(void);
int res_trk_close_secure_session(void);
void res_trk_secure_set(void);
void res_trk_secure_unset(void);
u32 get_res_trk_perf_level(enum vcd_perf_level);
int res_trk_enable_footswitch(void);
int res_trk_disable_footswitch(void);
void res_trk_release_fw_addr(void);
u32 res_trk_estimate_perf_level(u32 pn_perf_lvl);
u32 res_trk_get_clk_rate(unsigned long *phclk_rate);
#endif

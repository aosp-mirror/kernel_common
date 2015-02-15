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
 */

#ifndef _ARCH_ARM_MACH_MSM_MDM2_H
#define _ARCH_ARM_MACH_MSM_MDM2_H
#include "sysmon.h"

struct mdm_vddmin_resource {
	int rpm_id;
	int ap2mdm_vddmin_gpio;
	unsigned int modes;
	unsigned int drive_strength;
	int mdm2ap_vddmin_gpio;
};

struct mdm_platform_data {
	char *mdm_version;
	int ramdump_delay_ms;
	int ps_hold_delay_ms;
	int soft_reset_inverted;
	int early_power_on;
	int sfr_query;
	int no_powerdown_after_ramdumps;
	struct mdm_vddmin_resource *vddmin_resource;
	struct platform_device *peripheral_platform_device;
	const unsigned int ramdump_timeout_ms;
	int image_upgrade_supported;
	struct gpiomux_setting *mdm2ap_status_gpio_run_cfg;
	int send_shdn;
	int cascading_ssr;
	int sysmon_subsys_id_valid;
	enum subsys_id sysmon_subsys_id;
};

#endif


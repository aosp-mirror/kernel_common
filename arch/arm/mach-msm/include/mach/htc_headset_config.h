/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_config.h
 *
 * HTC headset configurations.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#ifndef HTC_HEADSET_CONFIG_H
#define HTC_HEADSET_CONFIG_H

#define HTC_HEADSET_VERSION	6 /* 8960 ADC uA to mA */
#define HTC_HEADSET_BRANCH	"KERNEL_3_0_PROJECT_8960"

#define HTC_HEADSET_KERNEL_3_0
#define HTC_HEADSET_CONFIG_QPNP_ADC

#if 0 /* All Headset Configurations */
#define HTC_HEADSET_KERNEL_3_0
#define HTC_HEADSET_CONFIG_MSM_RPC
#define HTC_HEADSET_CONFIG_QUICK_BOOT
#define HTC_HEADSET_CONFIG_PMIC_8XXX_ADC
#endif

#endif

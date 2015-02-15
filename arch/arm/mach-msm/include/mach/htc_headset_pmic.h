/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_pmic.h
 *
 * HTC PMIC headset driver.
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

#ifndef HTC_HEADSET_PMIC_H
#define HTC_HEADSET_PMIC_H

#define DRIVER_HS_PMIC_RPC_KEY			(1 << 0)
#define DRIVER_HS_PMIC_DYNAMIC_THRESHOLD	(1 << 1)
#define DRIVER_HS_PMIC_ADC			(1 << 2)
#define DRIVER_HS_PMIC_EDGE_IRQ			(1 << 3)

#define HS_PMIC_HTC_CURRENT_THRESHOLD		500

#define HS_PMIC_RPC_CLIENT_PROG			0x30000061
#define HS_PMIC_RPC_CLIENT_VERS			0x00010001
#define HS_PMIC_RPC_CLIENT_VERS_1_1		0x00010001
#define HS_PMIC_RPC_CLIENT_VERS_2_1		0x00020001
#define HS_PMIC_RPC_CLIENT_VERS_3_1		0x00030001

#define HS_PMIC_RPC_CLIENT_PROC_NULL		0
#define HS_PMIC_RPC_CLIENT_PROC_THRESHOLD	65

enum {
	HS_PMIC_RPC_ERR_SUCCESS,
};

enum {
	HS_PMIC_CONTROLLER_0,
	HS_PMIC_CONTROLLER_1,
	HS_PMIC_CONTROLLER_2,
};

enum {
	HS_PMIC_SC_SWITCH_TYPE,
	HS_PMIC_OC_SWITCH_TYPE,
};

struct hs_pmic_rpc_request {
	struct rpc_request_hdr hdr;
	uint32_t hs_controller;
	uint32_t hs_switch;
	uint32_t current_uA;
};

struct hs_pmic_rpc_reply {
	struct rpc_reply_hdr hdr;
	uint32_t status;
	uint32_t data;
};

struct hs_pmic_current_threshold {
	uint32_t adc_max;
	uint32_t adc_min;
	uint32_t current_uA;
};

struct htc_headset_pmic_platform_data {
	unsigned int driver_flag;
	unsigned int hpin_gpio;
	unsigned int hpin_irq;
	unsigned int key_gpio;
	unsigned int key_irq;
	unsigned int key_enable_gpio;
	unsigned int hs_controller;
	unsigned int hs_switch;

	/* ADC read method*/
#ifndef HTC_HEADSET_CONFIG_QPNP_ADC
	unsigned int adc_mpp;		/* 8064 platform */
	unsigned int adc_amux;		/* 8064 platform */
#else
	unsigned int adc_channel;	/* 8974 platform */
#endif
	/* ADC tables */
	uint32_t adc_mic;
	uint32_t adc_mic_bias[2];
	uint32_t adc_remote[6];
	uint32_t adc_metrico[2];
};

struct htc_35mm_pmic_info {
	struct htc_headset_pmic_platform_data pdata;
	unsigned int hpin_irq_type;
	unsigned int hpin_debounce;
	unsigned int key_irq_type;
	struct wake_lock hs_wake_lock;
	struct class* htc_accessory_class;
	struct hrtimer timer;
	struct device* pmic_dev;
#ifdef HTC_HEADSET_CONFIG_QPNP_ADC
	struct device *dev;
#endif
};

#endif

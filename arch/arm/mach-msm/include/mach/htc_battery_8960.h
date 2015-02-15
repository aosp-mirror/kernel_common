/*
 * Copyright (C) 2007 HTC Incorporated
 * Author: Jay Tu (jay_tu@htc.com)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _HTC_BATTERY_H_
#define _HTC_BATTERY_H_

#include "htc_battery_common.h"
#include "htc_gauge.h"
#include "htc_charger.h"

#define ADC_REPLY_ARRAY_SIZE		5

#define HTC_BATT_IOCTL_MAGIC		0xba

#define DEBUG_LOG_LENGTH		1024

#define HTC_BATT_IOCTL_READ_SOURCE \
	_IOR(HTC_BATT_IOCTL_MAGIC, 1, unsigned int)
#define HTC_BATT_IOCTL_SET_BATT_ALARM \
	_IOW(HTC_BATT_IOCTL_MAGIC, 2, unsigned int)
#define HTC_BATT_IOCTL_GET_ADC_VREF \
	_IOR(HTC_BATT_IOCTL_MAGIC, 3, unsigned int[ADC_REPLY_ARRAY_SIZE])
#define HTC_BATT_IOCTL_GET_ADC_ALL \
	_IOR(HTC_BATT_IOCTL_MAGIC, 4, struct battery_adc_reply)
#define HTC_BATT_IOCTL_CHARGER_CONTROL \
	_IOW(HTC_BATT_IOCTL_MAGIC, 5, unsigned int)
#define HTC_BATT_IOCTL_UPDATE_BATT_INFO \
	_IOW(HTC_BATT_IOCTL_MAGIC, 6, struct battery_info_reply)
#define HTC_BATT_IOCTL_BATT_DEBUG_LOG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 7, char[DEBUG_LOG_LENGTH])
#define HTC_BATT_IOCTL_SET_VOLTAGE_ALARM \
	_IOW(HTC_BATT_IOCTL_MAGIC, 8, struct battery_vol_alarm)
#define HTC_BATT_IOCTL_SET_ALARM_TIMER_FLAG \
	_IOW(HTC_BATT_IOCTL_MAGIC, 9, unsigned int)

#define REGULAR_BATTERRY_TIMER		"regular_timer"
#define CABLE_DETECTION			"cable"
#define CHARGER_IC_INTERRUPT		"charger_int"

#define XOADC_MPP			0
#define PM_MPP_AIN_AMUX			1

#define MBAT_IN_LOW_TRIGGER		0
#define MBAT_IN_HIGH_TRIGGER		1

extern int radio_set_cable_status(int charger_type);

struct battery_adc_reply {
	u32 adc_voltage[ADC_REPLY_ARRAY_SIZE];
	u32 adc_current[ADC_REPLY_ARRAY_SIZE];
	u32 adc_temperature[ADC_REPLY_ARRAY_SIZE];
	u32 adc_battid[ADC_REPLY_ARRAY_SIZE];
};

struct mpp_config_data {
	u32 vol[2];
	u32 curr[2];
	u32 temp[2];
	u32 battid[2];
};

struct battery_vol_alarm {
	int lower_threshold;
	int upper_threshold;
	int enable;
};

extern unsigned int system_rev;

enum {
	GUAGE_NONE,
	GUAGE_MODEM,
	GUAGE_DS2784,
	GUAGE_DS2746,
	GUAGE_BQ27510,
};

enum {
	LINEAR_CHARGER,
	SWITCH_CHARGER_TPS65200,
};

enum {
	BATT_TIMER_WAKE_LOCK = 0,
	BATT_IOCTL_WAKE_LOCK,
};

enum {
	HTC_BATT_DEBUG_UEVT = 1U << 1,
	HTC_BATT_DEBUG_USER_QUERY = 1U << 2,
	HTC_BATT_DEBUG_USB_NOTIFY = 1U << 3,
	HTC_BATT_DEBUG_FULL_LOG = 1U << 4,
};

struct htc_battery_platform_data {
	int gpio_mbat_in;
	int gpio_mbat_in_trigger_level;
	int gpio_mchg_en_n;
	int gpio_iset;
	int gpio_adp_9v;
	int guage_driver;
	int charger;
	struct mpp_config_data mpp_data;
	int chg_limit_active_mask;
#ifdef CONFIG_DUTY_CYCLE_LIMIT
	int chg_limit_timer_sub_mask;
#endif
	int critical_low_voltage_mv;
	int *critical_alarm_vol_ptr;
	int critical_alarm_vol_cols;
	int force_shutdown_batt_vol;
	int overload_vol_thr_mv;
	int overload_curr_thr_ma;
	bool usb_temp_monitor_enable;
	int usb_temp_overheat_increase_threshold;
	int normal_usb_temp_threshold;
	int usb_temp_overheat_threshold;
	int smooth_chg_full_delay_min;
	int decreased_batt_level_check;
	struct htc_gauge igauge;
	struct htc_charger icharger;
	int (*notify_pnpmgr_charging_enabled)(int charging_enabled);
};

enum {
	BATT_ALARM_DISABLE_MODE,
	BATT_ALARM_NORMAL_MODE,
	BATT_ALARM_CRITICAL_MODE,
};

#endif

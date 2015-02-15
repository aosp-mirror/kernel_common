/*
 * Copyright (C) 2010 HTC Incorporated
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
#ifndef HTC_BATTERY_CORE_H
#define HTC_BATTERY_CORE_H
#include <mach/board.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <mach/htc_battery_common.h>

#define BATT_LOG(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_INFO "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_ERR(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] err:" x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_EMBEDDED(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
	OVER_VCHG,
	BATT_STATE,
	BATT_CABLEIN,
	USB_TEMP,
	USB_OVERHEAT,
};

enum htc_batt_rt_attr {
	HTC_BATT_RT_VOLTAGE = 0,
	HTC_BATT_RT_CURRENT,
	HTC_BATT_RT_TEMPERATURE,
	HTC_BATT_RT_VOLTAGE_UV,
#if defined(CONFIG_MACH_DUMMY)
	HTC_USB_RT_TEMPERATURE,
#endif
	HTC_BATT_RT_ID,
};

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 batt_discharg_current;
	u32 level;
	u32 level_raw;
	u32 charging_source;
	u32 charging_enabled;
	u32 full_bat;
	u32 full_level;
	u32 full_level_dis_batt_chg;
	u32 over_vchg;
	s32 temp_fault;
	u32 batt_state;
	u32 overload;
	u32 cable_ready;
	s32 usb_temp;
	u32 usb_overheat;
};

struct htc_battery_core {
	int (*func_get_batt_rt_attr)(enum htc_batt_rt_attr attr, int* val);
	int (*func_show_batt_attr)(struct device_attribute *attr, char *buf);
	int (*func_show_cc_attr)(struct device_attribute *attr, char *buf);
	int (*func_show_htc_extension_attr)(struct device_attribute *attr, char *buf);
	int (*func_get_battery_info)(struct battery_info_reply *buffer);
	int (*func_charger_control)(enum charger_control_flag);
	int (*func_context_event_handler)(enum batt_context_event);
	void (*func_set_full_level)(int full_level);
	int (*func_notify_pnpmgr_charging_enabled)(int charging_enabled);
	int (*func_set_max_input_current)(int target_ma);
	void (*func_set_full_level_dis_batt_chg)(int full_level_dis_batt_chg);
	int (*func_get_chg_status)(enum power_supply_property);
	int (*func_set_chg_property)(enum power_supply_property, int val);
	void (*func_trigger_store_battery_data)(int trigger_flag);
	void (*func_qb_mode_shutdown_status)(int trigger_flag);
	int (*func_ftm_charger_control)(enum ftm_charger_control_flag);
};
#ifdef CONFIG_HTC_BATT_CORE
void htc_battery_update_batt_uevent(void);
extern int htc_battery_core_update_changed(void);
extern int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery);
const struct battery_info_reply* htc_battery_core_get_batt_info_rep(void);
#else
static void htc_battery_update_batt_uevent(void) { return 0; }
static int htc_battery_core_update_changed(void) { return 0; }
static int htc_battery_core_register(struct device *dev, struct htc_battery_core *htc_battery) { return 0; }
static struct battery_info_reply* htc_battery_core_get_batt_info_rep(void)
{ return NULL; }
#endif
#endif

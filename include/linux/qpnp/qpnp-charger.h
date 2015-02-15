/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#ifndef __QPNP_CHARGER_H
#define __QPNP_CHARGER_H

#include <linux/errno.h>
#include <linux/power_supply.h>
#ifdef CONFIG_HTC_BATT_8960
#include <mach/htc_charger.h>
#endif

#define FALSE       0
#define TRUE        1

#define CHARGER_STORE_MAGIC_NUM		0xDDAACC00
#define BATTERY_HBOOT_MAGIC_NUM		0xDDAACC11
#define CHARGER_STORE_MAGIC_OFFSET		1056	
#define CHARGER_STORE_PRE_DELTA_VDDMAX_OFFSET	1080	

struct ext_usb_chg_pm8941 {
	const char	*name;
	void		*ctx;
	int		(*start_charging) (void *ctx);
	int		(*stop_charging) (void *ctx);
	bool		(*is_trickle) (void *ctx);
	struct htc_charger	*ichg;
};

#ifdef CONFIG_QPNP_CHARGER
#ifdef CONFIG_HTC_BATT_8960
int pm8941_is_pwr_src_plugged_in(void);
int pm8941_get_batt_temperature(int *result);
int pm8941_get_batt_voltage(int *result);
int pm8941_is_batt_temp_fault_disable_chg(int *result);
int pm8941_is_batt_temperature_fault(int *result);
int pm8941_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable);
int pm8941_charger_enable(bool enable);
int pm8941_pwrsrc_enable(bool enable);
int pm8941_get_battery_status(void);
int pm8941_get_batt_present(void);
int pm8941_get_charging_source(int *result);
int pm8941_get_charging_enabled(int *result);
int pm8941_get_charge_type(void);
int pm8941_get_chg_usb_iusbmax(void);
int pm8941_get_chg_curr_settled(void);
int pm8941_get_chg_vinmin(void);
int pm8941_get_input_voltage_regulation(void);
int pm8941_set_chg_curr_settled(int val);
int pm8941_set_chg_vin_min(int val);
int pm8941_set_chg_iusbmax(int val);
int pm8941_is_charger_ovp(int* result);
int pm8941_set_hsml_target_ma(int target_ma);
int pm8941_dump_all(void);
int pm8941_is_batt_full(int *result);
int pm8941_is_batt_full_eoc_stop(int *result);
int pm8941_charger_get_attr_text(char *buf, int size);
int pm8941_gauge_get_attr_text(char *buf, int size);
int pm8941_fake_chg_gone_irq_handler(void);
int pm8941_fake_usbin_valid_irq_handler(void);
int pm8941_fake_coarse_det_usb_irq_handler(void);
#ifdef CONFIG_DUTY_CYCLE_LIMIT
int pm8941_limit_charge_enable(int chg_limit_reason,
			 int chg_limit_timer_sub_mask,
			 int limit_charge_timer_ma);
#else
int pm8941_limit_charge_enable(bool enable);
#endif
int pm8941_limit_input_current(bool enable, int reason);
int pm8941_is_chg_safety_timer_timeout(int *result);
int pm8941_get_usb_temperature(int *result);
int pm8941_store_battery_charger_data_emmc(void);
int pm8941_usb_overheat_otg_mode_check(void);
int pm8941_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src);
#endif
#else 
#ifdef CONFIG_HTC_BATT_8960
static inline int pm8941_is_pwr_src_plugged_in(void)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_temperature(int *result)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_voltage(int *result)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_temp_fault_disable_chg(int *result)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_temperature_fault(int *result)
{
	return -ENXIO;
}
static inline int pm8941_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable)
{
	return -ENXIO;
}
static inline int pm8941_charger_enable(bool enable)
{
	return -ENXIO;
}
static inline int pm8941_is_charger_ovp(int* result)
{
	return -ENXIO;
}
static inline int pm8941_pwrsrc_enable(bool enable)
{
	return -ENXIO;
}
static inline int pm8941_get_battery_status(void)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_present(void)
{
	return -ENXIO;
}
static inline int pm8941_get_charging_source(int *result)
{
	return -ENXIO;
}
static inline int pm8941_get_charging_enabled(int *result)
{
	return -ENXIO;
}
static inline int pm8941_get_charge_type(void)
{
	return -ENXIO;
}
static inline int pm8941_get_chg_usb_iusbmax(void)
{
	return -ENXIO;
}
static inline int pm8941_get_chg_curr_settled(void)
{
	return -ENXIO;
}
static inline int pm8941_get_chg_vinmin(void)
{
	return -ENXIO;
}
static inline int pm8941_get_input_voltage_regulation(void)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_curr_settled(int val)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_vin_min(int val)
{
	return -ENXIO;
}
static inline int pm8941_set_chg_iusbmax(int val)
{
	return -ENXIO
}
static inline int pm8941_set_hsml_target_ma(int target_ma)
{
	return -ENXIO;
}
static inline int pm8941_dump_all(void)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_full(int *result)
{
	return -ENXIO;
}
static inline int pm8941_is_batt_full_eoc_stop(int *result)
{
	return -ENXIO;
}
static inline int pm8941_charger_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int pm8941_gauge_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int pm8941_fake_chg_gone_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_fake_usbin_valid_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_fake_coarse_det_usb_irq_handler(void)
{
	return -ENXIO;
}
static inline int pm8941_is_chg_safety_timer_timeout(int *result)
{
	return -ENXIO;
}
#ifdef CONFIG_DUTY_CYCLE_LIMIT
static inline int pm8941_limit_charge_enable(int chg_limit_reason,
			 int chg_limit_timer_sub_mask,
			 int limit_charge_timer_ma)
#else
static inline int pm8941_limit_charge_enable(bool enable)
{
	return -ENXIO;
}
#endif
static int pm8941_limit_input_current(bool enable, int reason);
{
	return -ENXIO;
}
static inline int pm8941_get_usb_temperature(int *result)
{
	return -ENXIO;
}
static inline int pm8941_store_battery_charger_data_emmc(void)
{
	return -ENXIO;
}
static inline int pm8941_usb_overheat_otg_mode_check(void)
{
	return -ENXIO;
}
static inline int pm8941_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	return -ENXIO;
}
#endif 
#endif 
#endif 


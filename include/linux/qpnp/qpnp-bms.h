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

#ifndef __QPNP_BMS_H
#define __QPNP_BMS_H

#define FALSE       0
#define TRUE        1

#define OCV_UPDATE_STOP_BIT_CABLE_IN			(1)
#define OCV_UPDATE_STOP_BIT_BATT_LEVEL			(1<<1)
#define OCV_UPDATE_STOP_BIT_ATTR_FILE			(1<<2)
#define OCV_UPDATE_STOP_BIT_BOOT_UP			(1<<3)
#define OCV_UPDATE_STOP_BIT_CABLE_OUT			(1<<4)

#ifdef CONFIG_QPNP_BMS
#ifdef CONFIG_HTC_BATT_8960
int pm8941_bms_get_batt_current(int *result);
int pm8941_bms_get_batt_soc(int *result);
int pm8941_bms_get_batt_cc(int *result);
int pm8941_get_batt_id(int *result);
int pm8941_bms_get_batt_soc(int *result);
int pm8941_bms_get_batt_current(int *result);
int pm8941_bms_dump_all(void);
int pm8941_bms_get_fcc(void);
int pm8941_bms_get_attr_text(char *buf, int size);
int pm8941_bms_store_battery_gauge_data_emmc(void);
int pm8941_bms_store_battery_ui_soc(int soc_ui);
int pm8941_bms_get_battery_ui_soc(void);
int pm8941_batt_lower_alarm_threshold_set(int threshold_mV);
int pm8941_check_soc_for_sw_ocv(void);
int pm8941_bms_batt_full_fake_ocv(void);
int pm8941_bms_enter_qb_mode(void);
int pm8941_bms_exit_qb_mode(void);
int pm8941_qb_mode_pwr_consumption_check(unsigned long time_stamp);
int emmc_misc_write(int val, int offset);
int pm8941_get_batt_id_mv(int *result);
#endif 
#else 
#ifdef CONFIG_HTC_BATT_8960
static int inline pm8941_bms_get_batt_current(int *result)
{
	return -ENXIO;
}
static int inline pm8941_bms_get_batt_soc(int *result)
{
	return -ENXIO;
}
static int inline pm8941_bms_get_batt_cc(int *result)
{
	return -ENXIO;
}
static int inline pm8941_get_batt_id(int *result)
{
	return -ENXIO;
}
static int pm8941_batt_lower_alarm_threshold_set(int threshold_mV)
{
	return -ENXIO;
}
static inline int pm8941_bms_get_batt_soc(int *result)
{
	return -ENXIO;
}
static inline int pm8941_bms_get_batt_current(int *result)
{
	return -ENXIO;
}
static inline int pm8941_bms_dump_all(void)
{
	return -ENXIO;
}
static inline int pm8941_bms_get_fcc(void)
{
	return -ENXIO;
}
static inline int pm8941_bms_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int pm8941_bms_store_battery_gauge_data_emmc(void)
{
	return -ENXIO;
}
static inline int pm8941_bms_store_battery_ui_soc(int soc_ui)
{
	return -ENXIO;
}
static inline int pm8941_bms_get_battery_ui_soc(void)
{
	return -ENXIO;
}
static inline int pm8941_check_soc_for_sw_ocv(void)
{
    return -ENXIO;
}
static inline int pm8941_bms_batt_full_fake_ocv(void)
{
	return -ENXIO;
}
static inline int pm8941_bms_enter_qb_mode(void)
{
	return -ENXIO;
}
static inline int pm8941_bms_exit_qb_mode(void)
{
	return -ENXIO;
}
static inline int pm8941_qb_mode_pwr_consumption_check(unsigned long time_stamp)
{
	return -ENXIO;
}
static inline int emmc_misc_write(int val, int offset)
{
	return -ENXIO;
}
static inline int pm8941_get_batt_id_mv(int *result)
{
	return -ENXIO;
}
#endif 
#endif 

#endif 

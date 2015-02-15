/*
 *
 * Copyright (C) 2011 HTC Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <mach/htc_battery_core.h>
#include <linux/android_alarm.h>
#include <mach/devices_cmdline.h>
#include <mach/devices_dtb.h>
#include <linux/qpnp/qpnp-charger.h>

#define USB_MA_0       (0)
#define USB_MA_500     (500)
#define USB_MA_1500    (1500)
#define USB_MA_1600    (1600)

#define DWC3_DCP	2

static ssize_t htc_battery_show_property(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t htc_battery_rt_attr_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static int htc_power_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val);

static int htc_power_usb_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val);

static int htc_battery_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp);

static int htc_power_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp);

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val);

static ssize_t htc_battery_charger_ctrl_timer(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

extern int htc_battery_is_support_qc20(void);
extern int htc_battery_check_cable_type_from_usb(void);
extern int board_ftm_mode(void);

#if 1
#define HTC_BATTERY_ATTR(_name)                                             \
{                                                                           \
	.attr = { .name = #_name, .mode = S_IRUGO},  \
	.show = htc_battery_show_property,                                  \
	.store = NULL,                                                      \
}
#else
#define HTC_BATTERY_ATTR(_name)                                             \
{                                                                           \
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },  \
	.show = htc_battery_show_property,                                  \
	.store = NULL,                                                      \
}
#endif

struct htc_battery_core_info {
	int present;
	int htc_charge_full;
	unsigned long update_time;
	struct mutex info_lock;
	struct battery_info_reply rep;
	struct htc_battery_core func;
};

static struct htc_battery_core_info battery_core_info;
static int battery_register = 1;
static int battery_over_loading;

static struct alarm batt_charger_ctrl_alarm;
static struct work_struct batt_charger_ctrl_work;
struct workqueue_struct *batt_charger_ctrl_wq;
static unsigned int charger_ctrl_stat;
static unsigned int ftm_charger_ctrl_stat;

static int test_power_monitor;
static int test_ftm_mode;

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_OVERLOAD,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_USB_OVERHEAT,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TYPE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
		.set_property = htc_battery_set_property,
		.property_is_writeable = htc_battery_property_is_writeable,
	},

	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
		.set_property = htc_power_usb_set_property,
		.property_is_writeable = htc_power_property_is_writeable,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

static BLOCKING_NOTIFIER_HEAD(wireless_charger_notifier_list);
int register_notifier_wireless_charger(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&wireless_charger_notifier_list, nb);
}

int unregister_notifier_wireless_charger(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&wireless_charger_notifier_list, nb);
}

static int zcharge_enabled;
int htc_battery_get_zcharge_mode(void)
{
#if 0	
	return zcharge_enabled;
#else
	return 1;
#endif
}
static int __init enable_zcharge_setup(char *str)
{
	int rc;
	unsigned long cal;

	rc = strict_strtoul(str, 10, &cal);

	if (rc)
		return rc;

	zcharge_enabled = cal;
	return 1;
}
__setup("enable_zcharge=", enable_zcharge_setup);

static int htc_battery_get_charging_status(void)
{
	enum charger_type_t charger;
	int ret;

	mutex_lock(&battery_core_info.info_lock);
	charger = battery_core_info.rep.charging_source;
	mutex_unlock(&battery_core_info.info_lock);

	if (battery_core_info.rep.batt_id == 255)
		charger = CHARGER_UNKNOWN;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
	case CHARGER_9V_AC:
	case CHARGER_WIRELESS:
	case CHARGER_MHL_AC:
	case CHARGER_DETECTING:
	case CHARGER_UNKNOWN_USB:
	case CHARGER_NOTIFY:
		if (battery_core_info.htc_charge_full)
			ret = POWER_SUPPLY_STATUS_FULL;
		else {
			if (battery_core_info.rep.charging_enabled != 0)
				ret = POWER_SUPPLY_STATUS_CHARGING;
			else
				ret = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return ret;
}

static ssize_t htc_battery_show_batt_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return battery_core_info.func.func_show_batt_attr(attr, buf);
}

static ssize_t htc_battery_show_cc_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return battery_core_info.func.func_show_cc_attr(attr, buf);
}

static ssize_t htc_battery_show_htc_extension_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	if (battery_core_info.func.func_show_htc_extension_attr)
		return battery_core_info.func.func_show_htc_extension_attr(attr, buf);
	return 0;
}

static ssize_t htc_battery_set_delta(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long delta = 0;

	delta = simple_strtoul(buf, NULL, 10);

	if (delta > 100)
		return -EINVAL;

	return count;
}

static ssize_t htc_battery_debug_flag(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long debug_flag;
	debug_flag = simple_strtoul(buf, NULL, 10);

	if (debug_flag > 100 || debug_flag == 0)
		return -EINVAL;

	return 0;
}

static ssize_t htc_battery_set_full_level(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	unsigned long percent = 100;

	rc = strict_strtoul(buf, 10, &percent);
	if (rc)
		return rc;

	if (percent > 100 || percent == 0)
		return -EINVAL;

	if (!battery_core_info.func.func_set_full_level) {
		BATT_ERR("No set full level function!");
		return -ENOENT;
	}

	battery_core_info.func.func_set_full_level(percent);

	return count;
}

static ssize_t htc_battery_set_full_level_dis_batt_chg(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	unsigned long percent = 100;

	rc = strict_strtoul(buf, 10, &percent);
	if (rc)
		return rc;

	if (percent > 100 || percent == 0)
		return -EINVAL;

	if (!battery_core_info.func.func_set_full_level_dis_batt_chg) {
		BATT_ERR("No set full level (disable battery charging only) function!");
		return -ENOENT;
	}

	battery_core_info.func.func_set_full_level_dis_batt_chg(percent);

	return count;
}

int htc_battery_charger_disable()
{
	int rc = 0;

	if (!battery_core_info.func.func_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}
	rc = battery_core_info.func.func_charger_control(STOP_CHARGER);
	if (rc < 0)
		BATT_ERR("charger control failed!");

	return rc;
}

int htc_battery_pwrsrc_disable()
{
	int rc = 0;

	if (!battery_core_info.func.func_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}
	rc = battery_core_info.func.func_charger_control(DISABLE_PWRSRC);
	if (rc < 0)
		BATT_ERR("charger control failed!");

	return rc;
}

int htc_battery_set_max_input_current(int target_ma)
{
	int rc = 0;

	if (!battery_core_info.func.func_set_max_input_current) {
		BATT_ERR("No max input current function!");
		return -ENOENT;
	}
	rc = battery_core_info.func.func_set_max_input_current(target_ma);
	if (rc < 0)
		BATT_ERR("max input current control failed!");

	return rc;
}

static ssize_t htc_battery_charger_stat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", charger_ctrl_stat);

	return i;
}

static ssize_t htc_battery_charger_switch(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	BATT_LOG("Set charger_control:%lu", enable);
	if (enable >= END_CHARGER)
		return -EINVAL;

	if (!battery_core_info.func.func_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}

	rc = battery_core_info.func.func_charger_control(enable);
	if (rc < 0) {
		BATT_ERR("charger control failed!");
		return rc;
	}
	charger_ctrl_stat = enable;

	alarm_cancel(&batt_charger_ctrl_alarm);

	return count;
}

static ssize_t htc_battery_ftm_charger_stat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", ftm_charger_ctrl_stat);

	return i;
}

static ssize_t htc_battery_ftm_charger_switch(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	BATT_LOG("Set charger_control:%lu", enable);
	if (enable >= FTM_END_CHARGER)
		return -EINVAL;

	if (!battery_core_info.func.func_ftm_charger_control) {
		BATT_ERR("No charger control function!");
		return -ENOENT;
	}

	rc = battery_core_info.func.func_ftm_charger_control(enable);
	if (rc < 0) {
		BATT_ERR("charger control failed!");
		return rc;
	}
	ftm_charger_ctrl_stat = enable;

	alarm_cancel(&batt_charger_ctrl_alarm);

	return count;
}

static ssize_t htc_battery_set_phone_call(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long phone_call = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &phone_call);
	if (rc)
		return rc;

	BATT_LOG("set context phone_call=%lu", phone_call);

	if (!battery_core_info.func.func_context_event_handler) {
		BATT_ERR("No context_event_notify function!");
		return -ENOENT;
	}

	if (phone_call)
		battery_core_info.func.func_context_event_handler(EVENT_TALK_START);
	else
		battery_core_info.func.func_context_event_handler(EVENT_TALK_STOP);

	return count;
}

static ssize_t htc_battery_set_play_music(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long play_music = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &play_music);
	if (rc)
		return rc;

	BATT_LOG("set context play music=%lu", play_music);

	if (!battery_core_info.func.func_context_event_handler) {
		BATT_ERR("No context_event_notify function!");
		return -ENOENT;
	}

	if (play_music)
		battery_core_info.func.func_context_event_handler(EVENT_MUSIC_START);
	else
		battery_core_info.func.func_context_event_handler(EVENT_MUSIC_STOP);

	return count;
}

static ssize_t htc_battery_set_network_search(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long network_search = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &network_search);
	if (rc)
		return rc;

	BATT_LOG("Set context network_search=%lu", network_search);

	if (!battery_core_info.func.func_context_event_handler) {
		BATT_ERR("No context_event_notify function!");
		return -ENOENT;
	}

	if (network_search) {
		battery_core_info.func.func_context_event_handler(
									EVENT_NETWORK_SEARCH_START);
	} else {
		battery_core_info.func.func_context_event_handler(
									EVENT_NETWORK_SEARCH_STOP);
	}

	return count;
}
static ssize_t htc_battery_set_navigation(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long navigation = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &navigation);
	if (rc)
		return rc;

	BATT_LOG("Set context navigation=%lu", navigation);

	if (!battery_core_info.func.func_context_event_handler) {
		BATT_ERR("No context_event_notify function!");
		return -ENOENT;
	}

	if (navigation) {
		battery_core_info.func.func_context_event_handler(
									EVENT_NAVIGATION_START);
	} else {
		battery_core_info.func.func_context_event_handler(
									EVENT_NAVIGATION_STOP);
	}

	return count;
}
static ssize_t htc_battery_set_context_event(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long event = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &event);
	if (rc)
		return rc;

	BATT_LOG("Set context event = %lu", event);

	if (!battery_core_info.func.func_context_event_handler) {
		BATT_ERR("No context_event_notify function!");
		return -ENOENT;
	}

	battery_core_info.func.func_context_event_handler(event);

	return count;
}

static ssize_t htc_battery_trigger_store_battery_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	unsigned long trigger_flag = 0;

	rc = strict_strtoul(buf, 10, &trigger_flag);
	if (rc)
		return rc;

	BATT_LOG("Set context trigger_flag = %lu", trigger_flag);

	if((trigger_flag != 0) && (trigger_flag != 1))
		return -EINVAL;

	if (!battery_core_info.func.func_trigger_store_battery_data) {
		BATT_ERR("No set trigger store battery data function!");
		return -ENOENT;
	}

	battery_core_info.func.func_trigger_store_battery_data(trigger_flag);

	return count;
}

static ssize_t htc_battery_qb_mode_shutdown_status(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	unsigned long trigger_flag = 0;

	rc = strict_strtoul(buf, 10, &trigger_flag);
	if (rc)
		return rc;

	BATT_LOG("Set context trigger_flag = %lu", trigger_flag);

	if((trigger_flag != 0) && (trigger_flag != 1))
		return -EINVAL;

	if (!battery_core_info.func.func_qb_mode_shutdown_status) {
		BATT_ERR("No set trigger qb mode shutdown status function!");
		return -ENOENT;
	}

	battery_core_info.func.func_qb_mode_shutdown_status(trigger_flag);

	return count;
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
	HTC_BATTERY_ATTR(over_vchg),
	HTC_BATTERY_ATTR(batt_state),
	HTC_BATTERY_ATTR(batt_cable_in),
	HTC_BATTERY_ATTR(usb_temp),
	HTC_BATTERY_ATTR(usb_overheat),

	__ATTR(batt_attr_text, S_IRUGO, htc_battery_show_batt_attr, NULL),
	__ATTR(batt_power_meter, S_IRUGO, htc_battery_show_cc_attr, NULL),
	__ATTR(htc_extension, S_IRUGO, htc_battery_show_htc_extension_attr, NULL),
};

static struct device_attribute htc_set_delta_attrs[] = {
	__ATTR(delta, S_IWUSR | S_IWGRP, NULL, htc_battery_set_delta),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_full_level),
	__ATTR(full_level_dis_batt_chg, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_full_level_dis_batt_chg),
	__ATTR(batt_debug_flag, S_IWUSR | S_IWGRP, NULL,
		htc_battery_debug_flag),
	__ATTR(charger_control, S_IWUSR | S_IWGRP, htc_battery_charger_stat,
		htc_battery_charger_switch),
	__ATTR(charger_timer, S_IWUSR | S_IWGRP, NULL,
		htc_battery_charger_ctrl_timer),
	__ATTR(phone_call, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_phone_call),
	__ATTR(play_music, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_play_music),
	__ATTR(network_search, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_network_search),
	__ATTR(navigation, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_navigation),
	__ATTR(context_event, S_IWUSR | S_IWGRP, NULL,
		htc_battery_set_context_event),
	__ATTR(store_battery_data, S_IWUSR | S_IWGRP, NULL,
		htc_battery_trigger_store_battery_data),
	__ATTR(qb_mode_shutdown, S_IWUSR | S_IWGRP, NULL,
		htc_battery_qb_mode_shutdown_status),
	__ATTR(ftm_charger_control, S_IWUSR | S_IWGRP, htc_battery_ftm_charger_stat,
		htc_battery_ftm_charger_switch),
};

static struct device_attribute htc_battery_rt_attrs[] = {
	__ATTR(batt_vol_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
	__ATTR(batt_current_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
	__ATTR(batt_temp_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
	__ATTR(voltage_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
#if defined(CONFIG_MACH_DUMMY)
	__ATTR(usb_temp_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
#endif
	__ATTR(batt_id_now, S_IRUGO, htc_battery_rt_attr_show, NULL),
};


static int htc_battery_create_attrs(struct device *dev)
{
	int i = 0, j = 0, k = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	for (j = 0; j < ARRAY_SIZE(htc_set_delta_attrs); j++) {
		rc = device_create_file(dev, &htc_set_delta_attrs[j]);
		if (rc)
			goto htc_delta_attrs_failed;
	}

	for (k = 0; k < ARRAY_SIZE(htc_battery_rt_attrs); k++) {
		rc = device_create_file(dev, &htc_battery_rt_attrs[k]);
		if (rc)
			goto htc_rt_attrs_failed;
	}

	goto succeed;

htc_rt_attrs_failed:
	while (k--)
		device_remove_file(dev, &htc_battery_rt_attrs[k]);
htc_delta_attrs_failed:
	while (j--)
		device_remove_file(dev, &htc_set_delta_attrs[j]);
htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
	return rc;
}

static int htc_battery_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	int ret = 0;

	if (!val) {
		pr_err("%s: val is null, return!\n", __func__);
		return -EINVAL;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
#if 0
		if (battery_core_info.func.func_set_chg_property)
			ret = battery_core_info.func.func_set_chg_property(psp,
				val->intval / 1000);
		else {
			pr_info("%s: function doesn't exist! psp=%d\n", __func__, psp);
			return ret;
		}
		break;
#else
		return ret;
#endif
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		if (battery_core_info.func.func_set_chg_property)
			ret = battery_core_info.func.func_set_chg_property(psp, val->intval);
		else {
			pr_info("%s: function doesn't exist! psp=%d\n", __func__, psp);
			return ret;
		}
		break;
	default:
		pr_info("%s: invalid type, psp=%d\n", __func__, psp);
		return -EINVAL;
	}
	pr_info("%s: batt power_supply_changed, psp=%d, intval=%d, ret=%d\n",
		__func__, psp, val->intval, ret);
	power_supply_changed(&htc_power_supplies[BATTERY_SUPPLY]);
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (battery_core_info.rep.temp_fault != -1) {
			if (battery_core_info.rep.temp_fault == 1)
				val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		else if (battery_core_info.rep.batt_temp >= 480 ||
			battery_core_info.rep.batt_temp <= 0)
			val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_core_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&battery_core_info.info_lock);
		val->intval = battery_core_info.rep.level;
		mutex_unlock(&battery_core_info.info_lock);
		break;
	case POWER_SUPPLY_PROP_OVERLOAD:
		val->intval = battery_core_info.rep.overload;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		if (battery_core_info.func.func_get_chg_status) {
			val->intval = battery_core_info.func.func_get_chg_status(psp);
			if (val->intval == (-EINVAL))
				pr_info("%s: function not ready. psp=%d\n", __func__, psp);
		} else
			pr_info("%s: function doesn't exist! psp=%d\n", __func__, psp);
		break;
	case POWER_SUPPLY_PROP_USB_OVERHEAT:
		val->intval = battery_core_info.rep.usb_overheat;
		break;
	default:
		pr_info("%s: invalid type, psp=%d\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}


static int usb_voltage_max = 0;
static int htc_power_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	enum charger_type_t charger;

	mutex_lock(&battery_core_info.info_lock);

	charger = battery_core_info.rep.charging_source;

#if 0
	if (battery_core_info.rep.batt_id == 255)
		charger = CHARGER_BATTERY;
#endif

	mutex_unlock(&battery_core_info.info_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (charger == CHARGER_AC ||
			    charger == CHARGER_9V_AC
			    ||  charger == CHARGER_MHL_AC)
				val->intval = 1;
			else
				val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (charger == CHARGER_USB ||
			    charger == CHARGER_UNKNOWN_USB ||
			    charger == CHARGER_DETECTING)
				val->intval = 1;
			else
				val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_WIRELESS)
			val->intval = (charger == CHARGER_WIRELESS ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB_DCP)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX :
		if (charger == CHARGER_BATTERY)
			val->intval = USB_MA_0;
		else if (charger == CHARGER_AC
				|| charger == CHARGER_9V_AC
				|| charger == CHARGER_WIRELESS)
			val->intval = USB_MA_1500 * 1000;
		else
			val->intval = USB_MA_500 * 1000;

		if (psy->type == POWER_SUPPLY_TYPE_USB_DCP)
			val->intval = USB_MA_1600 * 1000;
		break;
	
	case POWER_SUPPLY_PROP_PRESENT :
		if (charger == CHARGER_BATTERY)
			val->intval  = 0;
		else
			val->intval  = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
				psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = usb_voltage_max;
		else
			return val->intval  = 0;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = psy->type;
		break;
	default:
		pr_info("%s: invalid type, psp=%d\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

static int htc_power_usb_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		usb_voltage_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		psy->type = val->intval;
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(&htc_power_supplies[USB_SUPPLY]);
	return 0;
}

static int htc_battery_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		return 1;
	default:
		break;
	}

	return 0;
}

static int htc_power_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static ssize_t htc_battery_show_property(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	mutex_lock(&battery_core_info.info_lock);

	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		if(battery_core_info.rep.charging_source == CHARGER_MHL_AC) {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", CHARGER_AC);
		}
		else {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.charging_source);
		}
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.charging_enabled);
		break;
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.full_bat);
		break;
	case OVER_VCHG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.over_vchg);
		break;
	case BATT_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.batt_state);
		break;
	case BATT_CABLEIN:
		if(battery_core_info.rep.charging_source == CHARGER_BATTERY)
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		else
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 1);
		break;
	case USB_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.usb_temp);
		break;
	case USB_OVERHEAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				battery_core_info.rep.usb_overheat);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&battery_core_info.info_lock);

	if (i < 0)
		BATT_ERR("%s: battery: attribute is not supported: %d",
			__func__, off);

	return i;
}

static ssize_t htc_battery_rt_attr_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	int val = 0;
	int rc = 0;
	const ptrdiff_t attr_index = attr - htc_battery_rt_attrs;

	if (!battery_core_info.func.func_get_batt_rt_attr) {
		BATT_ERR("%s: func_get_batt_rt_attr does not exist", __func__);
		return -EINVAL;
	}

	rc = battery_core_info.func.func_get_batt_rt_attr(attr_index, &val);
	if (rc) {
		BATT_ERR("%s: get_batt_rt_attrs[%d] failed", __func__, attr_index);
		return -EINVAL;
	}

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
	return i;
}

static ssize_t htc_battery_charger_ctrl_timer(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int rc;
	unsigned long time_out = 0;
	ktime_t interval;
	ktime_t next_alarm;

	rc = strict_strtoul(buf, 10, &time_out);
	if (rc)
		return rc;

	if (time_out > 65536)
		return -EINVAL;

	if (time_out > 0) {
		rc = battery_core_info.func.func_charger_control(STOP_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		interval = ktime_set(time_out, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&batt_charger_ctrl_alarm,
					next_alarm, next_alarm);
		charger_ctrl_stat = STOP_CHARGER;
	} else if (time_out == 0) {
		rc = battery_core_info.func.func_charger_control(
							ENABLE_CHARGER);
		if (rc < 0) {
			BATT_ERR("charger control failed!");
			return rc;
		}
		alarm_cancel(&batt_charger_ctrl_alarm);
		charger_ctrl_stat = ENABLE_CHARGER;
	}

	return count;
}

static void batt_charger_ctrl_func(struct work_struct *work)
{
	int rc;

	rc = battery_core_info.func.func_charger_control(ENABLE_CHARGER);
	if (rc) {
		BATT_ERR("charger control failed!");
		return;
	}

	charger_ctrl_stat = (unsigned int)ENABLE_CHARGER;
}

static void batt_charger_ctrl_alarm_handler(struct alarm *alarm)
{
	BATT_LOG("charger control alarm is timeout.");

	queue_work(batt_charger_ctrl_wq, &batt_charger_ctrl_work);
}

static unsigned int get_htc_debug_flag(void)
{

        unsigned int cfg = 0 ;
    
        if (get_tamper_sf() == 0) {
                if ((get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON)
					|| (get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE)) {
                        cfg = 1 ;
                }
        }
	return cfg;
}

void htc_battery_update_batt_uevent(void)
{
	power_supply_changed(&htc_power_supplies[BATTERY_SUPPLY]);
	BATT_LOG("%s: power_supply_changed: battery", __func__);
}

int htc_battery_core_update_changed(void)
{
	struct battery_info_reply new_batt_info_rep;
	int is_send_batt_uevent = 0;
	int is_send_usb_uevent = 0;
	int is_send_ac_uevent = 0;
	int is_send_wireless_charger_uevent = 0;
	static int batt_temp_over_68c_count = 0;
	unsigned int dbg_cfg = 0 ;

	if (battery_register) {
		BATT_ERR("No battery driver exists.");
		return -1;
	}

	mutex_lock(&battery_core_info.info_lock);
	memcpy(&new_batt_info_rep, &battery_core_info.rep, sizeof(struct battery_info_reply));
	mutex_unlock(&battery_core_info.info_lock);
	if (battery_core_info.func.func_get_battery_info) {
		battery_core_info.func.func_get_battery_info(&new_batt_info_rep);
	} else {
		BATT_ERR("no func_get_battery_info hooked.");
		return -EINVAL;
	}

	mutex_lock(&battery_core_info.info_lock);
	if (battery_core_info.rep.charging_source != new_batt_info_rep.charging_source) {
		if (CHARGER_BATTERY == battery_core_info.rep.charging_source ||
			CHARGER_BATTERY == new_batt_info_rep.charging_source)
			is_send_batt_uevent = 1;
		if (CHARGER_USB == battery_core_info.rep.charging_source ||
			CHARGER_USB == new_batt_info_rep.charging_source)
			is_send_usb_uevent = 1;
		if (CHARGER_AC == battery_core_info.rep.charging_source ||
			CHARGER_AC == new_batt_info_rep.charging_source)
			is_send_ac_uevent = 1;
		if (CHARGER_MHL_AC == battery_core_info.rep.charging_source ||
			CHARGER_MHL_AC == new_batt_info_rep.charging_source)
			is_send_ac_uevent = 1;
		if (CHARGER_WIRELESS == battery_core_info.rep.charging_source ||
			CHARGER_WIRELESS == new_batt_info_rep.charging_source)
			is_send_wireless_charger_uevent = 1;

		
		if (htc_battery_is_support_qc20()) {
			if (CHARGER_AC == new_batt_info_rep.charging_source) {
				if (htc_battery_check_cable_type_from_usb() == DWC3_DCP) {
					power_supply_set_supply_type(&htc_power_supplies[USB_SUPPLY], POWER_SUPPLY_TYPE_USB_DCP);
					is_send_usb_uevent = 1;
					is_send_ac_uevent = 0;
				}
			} else if (CHARGER_AC == battery_core_info.rep.charging_source) {
				if (htc_power_supplies[USB_SUPPLY].type == POWER_SUPPLY_TYPE_USB_DCP) {
					power_supply_set_supply_type(&htc_power_supplies[USB_SUPPLY], POWER_SUPPLY_TYPE_USB);
					is_send_usb_uevent = 1;
					is_send_ac_uevent = 0;
				}
			}
		}
	}
	if ((!is_send_batt_uevent) &&
		((battery_core_info.rep.level != new_batt_info_rep.level) ||
		(battery_core_info.rep.batt_vol != new_batt_info_rep.batt_vol) ||
		(battery_core_info.rep.over_vchg != new_batt_info_rep.over_vchg) ||
		(battery_core_info.rep.batt_temp != new_batt_info_rep.batt_temp))) {
		is_send_batt_uevent = 1;
	}

	if ((battery_core_info.rep.charging_enabled != 0) &&
		(new_batt_info_rep.charging_enabled != 0)) {
		if (battery_core_info.rep.level > new_batt_info_rep.level)
			battery_over_loading++;
		else
			battery_over_loading = 0;
	}

	
	if (battery_core_info.func.func_notify_pnpmgr_charging_enabled) {
		if (battery_core_info.rep.charging_enabled !=
				new_batt_info_rep.charging_enabled)
			battery_core_info.func.func_notify_pnpmgr_charging_enabled(
										new_batt_info_rep.charging_enabled);
	}

	memcpy(&battery_core_info.rep, &new_batt_info_rep, sizeof(struct battery_info_reply));

	if (battery_core_info.rep.batt_temp > 680) {
		batt_temp_over_68c_count++;
		if (batt_temp_over_68c_count < 3) {
			pr_info("[BATT] batt_temp_over_68c_count=%d, (temp=%d)\n",
					batt_temp_over_68c_count, battery_core_info.rep.batt_temp);
			battery_core_info.rep.batt_temp = 680;
		}
	} else {
		
		batt_temp_over_68c_count = 0;
	}

	
	
	if (test_power_monitor || (test_ftm_mode == 1)) {
		BATT_LOG("test_power_monitor(%d) or test_ftm_mode(%d) is set: overwrite fake batt info.",
				test_power_monitor, test_ftm_mode);
		battery_core_info.rep.batt_id = 77;
		battery_core_info.rep.batt_temp = 330;
		battery_core_info.rep.level = 77;
		battery_core_info.rep.temp_fault = 0;
	}

	if (battery_core_info.rep.charging_source <= 0) {
		if (battery_core_info.rep.batt_id == 255) {
			pr_info("[BATT] Ignore invalid id when no charging_source");
			battery_core_info.rep.batt_id = 66;
		}
	}
#if 0
	battery_core_info.rep.batt_vol = new_batt_info_rep.batt_vol;
	battery_core_info.rep.batt_id = new_batt_info_rep.batt_id;
	battery_core_info.rep.batt_temp = new_batt_info_rep.batt_temp;
	battery_core_info.rep.batt_current = new_batt_info_rep.batt_current;
	battery_core_info.rep.batt_discharg_current = new_batt_info_rep.batt_discharg_current;
	battery_core_info.rep.level = new_batt_info_rep.level;
	battery_core_info.rep.charging_source = new_batt_info_rep.charging_source;
	battery_core_info.rep.charging_enabled = new_batt_info_rep.charging_enabled;
	battery_core_info.rep.full_bat = new_batt_info_rep.full_bat;
	battery_core_info.rep.over_vchg = new_batt_info_rep.over_vchg;
	battery_core_info.rep.temp_fault = new_batt_info_rep.temp_fault;
	battery_core_info.rep.batt_state = new_batt_info_rep.batt_state;
#endif

	if (battery_core_info.rep.charging_source == CHARGER_BATTERY)
		battery_core_info.htc_charge_full = 0;
	else {
		if (battery_core_info.htc_charge_full &&
				(battery_core_info.rep.level == 100))
			battery_core_info.htc_charge_full = 1;
		else {
			if (battery_core_info.rep.level == 100)
				battery_core_info.htc_charge_full = 1;
			else
				battery_core_info.htc_charge_full = 0;
		}

		
		if (battery_over_loading >= 2) {
			battery_core_info.htc_charge_full = 0;
			battery_over_loading = 0;
		}
	}

	battery_core_info.update_time = jiffies;
	mutex_unlock(&battery_core_info.info_lock);

	
	dbg_cfg = get_htc_debug_flag();

	BATT_EMBEDDED("ID=%d,level=%d,level_raw=%d,vol=%d,temp=%d,current=%d,"
		"chg_src=%d,chg_en=%d,full_bat=%d,over_vchg=%d,"
		"batt_state=%d,cable_ready=%d,overload=%d,ui_chg_full=%d,"
		"usb_temp=%d,usb_overheat=%d,CFG=0x%x",
			battery_core_info.rep.batt_id,
			battery_core_info.rep.level,
			battery_core_info.rep.level_raw,
			battery_core_info.rep.batt_vol,
			battery_core_info.rep.batt_temp,
			battery_core_info.rep.batt_current,
			battery_core_info.rep.charging_source,
			battery_core_info.rep.charging_enabled,
			battery_core_info.rep.full_bat,
			battery_core_info.rep.over_vchg,
			battery_core_info.rep.batt_state,
			battery_core_info.rep.cable_ready,
			battery_core_info.rep.overload,
			battery_core_info.htc_charge_full,
			battery_core_info.rep.usb_temp,
			battery_core_info.rep.usb_overheat,
			dbg_cfg);


	
	if (is_send_batt_uevent) {
		power_supply_changed(&htc_power_supplies[BATTERY_SUPPLY]);
		BATT_LOG("power_supply_changed: battery");
	}
	if (is_send_usb_uevent) {
		power_supply_changed(&htc_power_supplies[USB_SUPPLY]);
		BATT_LOG("power_supply_changed: usb");
	}
	if (is_send_ac_uevent) {
		power_supply_changed(&htc_power_supplies[AC_SUPPLY]);
		BATT_LOG("power_supply_changed: ac");
	}
	if (is_send_wireless_charger_uevent) {
		power_supply_changed(&htc_power_supplies[WIRELESS_SUPPLY]);
		BATT_LOG("power_supply_changed: wireless");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(htc_battery_core_update_changed);

int htc_battery_core_register(struct device *dev,
				struct htc_battery_core *htc_battery)
{
	int i, rc = 0;

	if (!battery_register) {
		BATT_ERR("Only one battery driver could exist.");
		return -1;
	}
	battery_register = 0;

	test_power_monitor =
		(get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY) ? 1 : 0;

	test_ftm_mode = board_ftm_mode();

	mutex_init(&battery_core_info.info_lock);

	if (htc_battery->func_get_batt_rt_attr)
		battery_core_info.func.func_get_batt_rt_attr =
					htc_battery->func_get_batt_rt_attr;
	if (htc_battery->func_show_batt_attr)
		battery_core_info.func.func_show_batt_attr =
					htc_battery->func_show_batt_attr;
	if (htc_battery->func_show_cc_attr)
		battery_core_info.func.func_show_cc_attr =
					htc_battery->func_show_cc_attr;
	if (htc_battery->func_show_htc_extension_attr)
		battery_core_info.func.func_show_htc_extension_attr =
					htc_battery->func_show_htc_extension_attr;
	if (htc_battery->func_get_battery_info)
		battery_core_info.func.func_get_battery_info =
					htc_battery->func_get_battery_info;
	if (htc_battery->func_charger_control)
		battery_core_info.func.func_charger_control =
					htc_battery->func_charger_control;
	if (htc_battery->func_set_max_input_current)
		battery_core_info.func.func_set_max_input_current =
					htc_battery->func_set_max_input_current;	
	if (htc_battery->func_context_event_handler)
		battery_core_info.func.func_context_event_handler =
					htc_battery->func_context_event_handler;

	if (htc_battery->func_set_full_level)
		battery_core_info.func.func_set_full_level =
					htc_battery->func_set_full_level;
	if (htc_battery->func_set_full_level_dis_batt_chg)
		battery_core_info.func.func_set_full_level_dis_batt_chg =
					htc_battery->func_set_full_level_dis_batt_chg;
	if (htc_battery->func_notify_pnpmgr_charging_enabled)
		battery_core_info.func.func_notify_pnpmgr_charging_enabled =
					htc_battery->func_notify_pnpmgr_charging_enabled;
	if (htc_battery->func_get_chg_status)
		battery_core_info.func.func_get_chg_status =
					htc_battery->func_get_chg_status;
	if (htc_battery->func_set_chg_property)
		battery_core_info.func.func_set_chg_property =
					htc_battery->func_set_chg_property;
	if (htc_battery->func_trigger_store_battery_data)
		battery_core_info.func.func_trigger_store_battery_data =
					htc_battery->func_trigger_store_battery_data;
	if (htc_battery->func_qb_mode_shutdown_status)
		battery_core_info.func.func_qb_mode_shutdown_status =
					htc_battery->func_qb_mode_shutdown_status;
	if (htc_battery->func_ftm_charger_control)
		battery_core_info.func.func_ftm_charger_control =
					htc_battery->func_ftm_charger_control;

	
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("Failed to register power supply"
				" (%d)\n", rc);
	}

	
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	
	charger_ctrl_stat = ENABLE_CHARGER;
	INIT_WORK(&batt_charger_ctrl_work, batt_charger_ctrl_func);
	alarm_init(&batt_charger_ctrl_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			batt_charger_ctrl_alarm_handler);
	batt_charger_ctrl_wq =
			create_singlethread_workqueue("charger_ctrl_timer");

	
	battery_core_info.update_time = jiffies;
	battery_core_info.present = 1;
	battery_core_info.htc_charge_full = 0;
	battery_core_info.rep.charging_source = CHARGER_BATTERY;
	battery_core_info.rep.batt_id = 1;
	battery_core_info.rep.batt_vol = 4000;
	battery_core_info.rep.batt_temp = 285;
	battery_core_info.rep.batt_current = 162;
	battery_core_info.rep.level = 66;
	battery_core_info.rep.level_raw = 0;
	battery_core_info.rep.full_bat = 1580000;
	battery_core_info.rep.full_level = 100;
	battery_core_info.rep.full_level_dis_batt_chg = 100;
	
	battery_core_info.rep.temp_fault = -1;
	
	battery_core_info.rep.batt_state = 0;
	battery_core_info.rep.cable_ready = 0;
	battery_core_info.rep.overload = 0;
	battery_core_info.rep.usb_temp = 285;
	battery_core_info.rep.usb_overheat = 0;

	battery_over_loading = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(htc_battery_core_register);

const struct battery_info_reply* htc_battery_core_get_batt_info_rep(void)
{
	return &battery_core_info.rep;
}
EXPORT_SYMBOL_GPL(htc_battery_core_get_batt_info_rep);

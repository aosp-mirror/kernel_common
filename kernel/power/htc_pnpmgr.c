/* linux/kernel/power/htc_pnpmgr.c
 *
 * Copyright (C) 2012 HTC Corporation.
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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/cpu.h>

#include "power.h"

#define MAX_BUF 100

struct kobject *cpufreq_kobj;
static struct kobject *hotplug_kobj;
static struct kobject *thermal_kobj;
static struct kobject *apps_kobj;
static struct kobject *pnpmgr_kobj;
static struct kobject *adaptive_policy_kobj;
static struct kobject *sysinfo_kobj;
static struct kobject *battery_kobj;

#define define_string_show(_name, str_buf)				\
static ssize_t _name##_show						\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)		\
{									\
	return scnprintf(buf, sizeof(str_buf), "%s", str_buf);	\
}

#define define_string_store(_name, str_buf, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	strncpy(str_buf, buf, sizeof(str_buf) - 1);				\
	str_buf[sizeof(str_buf) - 1] = '\0';				\
	(store_cb)(#_name);					\
	sysfs_notify(kobj, NULL, #_name);			\
	return n;						\
}

#define define_int_show(_name, int_val)				\
static ssize_t _name##_show					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%d", int_val);			\
}

#define define_int_store(_name, int_val, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	int val;						\
	if (sscanf(buf, "%d", &val) > 0) {			\
		int_val = val;					\
		(store_cb)(#_name);				\
		sysfs_notify(kobj, NULL, #_name);		\
		return n;					\
	}							\
	return -EINVAL;						\
}

static char activity_buf[MAX_BUF];
static char non_activity_buf[MAX_BUF];
static char media_mode_buf[MAX_BUF];
static int app_timeout_expired;
static int is_touch_boosted;;

static void null_cb(const char *attr) {
	do { } while (0);
}

define_string_show(activity_trigger, activity_buf);
define_string_store(activity_trigger, activity_buf, null_cb);
power_attr(activity_trigger);

define_string_show(non_activity_trigger, non_activity_buf);
define_string_store(non_activity_trigger, non_activity_buf, null_cb);
power_attr(non_activity_trigger);

define_string_show(media_mode, media_mode_buf);
define_string_store(media_mode, media_mode_buf, null_cb);
power_attr(media_mode);

static int thermal_c0_value = 9999999;
#if (CONFIG_NR_CPUS >= 2)
static int thermal_c1_value = 9999999;
#if (CONFIG_NR_CPUS == 4)
static int thermal_c2_value = 9999999;
static int thermal_c3_value = 9999999;
#endif
#endif
static int thermal_final_value = 9999999;
static int thermal_g0_value = 999999999;
static int thermal_final_cpu_value = 9999999;
static int thermal_final_gpu_value = 999999999;
static int thermal_batt_value;
static int data_throttling_value;

define_int_show(thermal_c0, thermal_c0_value);
define_int_store(thermal_c0, thermal_c0_value, null_cb);
power_attr(thermal_c0);

#if (CONFIG_NR_CPUS >= 2)
define_int_show(thermal_c1, thermal_c1_value);
define_int_store(thermal_c1, thermal_c1_value, null_cb);
power_attr(thermal_c1);
#if (CONFIG_NR_CPUS == 4)
define_int_show(thermal_c2, thermal_c2_value);
define_int_store(thermal_c2, thermal_c2_value, null_cb);
power_attr(thermal_c2);

define_int_show(thermal_c3, thermal_c3_value);
define_int_store(thermal_c3, thermal_c3_value, null_cb);
power_attr(thermal_c3);
#endif
#endif

define_int_show(thermal_final, thermal_final_value);
define_int_store(thermal_final, thermal_final_value, null_cb);
power_attr(thermal_final);

define_int_show(thermal_g0, thermal_g0_value);
define_int_store(thermal_g0, thermal_g0_value, null_cb);
power_attr(thermal_g0);

define_int_show(thermal_batt, thermal_batt_value);
define_int_store(thermal_batt, thermal_batt_value, null_cb);
power_attr(thermal_batt);

define_int_show(thermal_final_cpu, thermal_final_cpu_value);
define_int_store(thermal_final_cpu, thermal_final_cpu_value, null_cb);
power_attr(thermal_final_cpu);

define_int_show(thermal_final_gpu, thermal_final_gpu_value);
define_int_store(thermal_final_gpu, thermal_final_gpu_value, null_cb);
power_attr(thermal_final_gpu);

static unsigned int info_gpu_max_clk = 400000000;
void set_gpu_clk(unsigned int value)
{
        info_gpu_max_clk = value;
}

ssize_t
gpu_max_clk_show(struct kobject *kobj, struct kobj_attribute *attr,
                char *buf)
{
       int ret = 0;
        ret = sprintf(buf, "%u", info_gpu_max_clk);
        return ret;
}
power_ro_attr(gpu_max_clk);

define_int_show(pause_dt, data_throttling_value);
define_int_store(pause_dt, data_throttling_value, null_cb);
power_attr(pause_dt);

#ifdef CONFIG_HOTPLUG_CPU
static char mp_nw_arg[MAX_BUF];
static char mp_tw_arg[MAX_BUF];
static char mp_ns_arg[MAX_BUF];
static char mp_ts_arg[MAX_BUF];
static int mp_decision_ms_value;
static int mp_min_cpus_value;
static int mp_max_cpus_value;
static int mp_spc_enabled_value;
static int mp_sync_enabled_value;
static char mp_util_high_and_arg[MAX_BUF];
static char mp_util_high_or_arg[MAX_BUF];
static char mp_util_low_and_arg[MAX_BUF];
static char mp_util_low_or_arg[MAX_BUF];

define_string_show(mp_nw, mp_nw_arg);
define_string_store(mp_nw, mp_nw_arg, null_cb);
power_attr(mp_nw);

define_string_show(mp_tw, mp_tw_arg);
define_string_store(mp_tw, mp_tw_arg, null_cb);
power_attr(mp_tw);

define_string_show(mp_ns, mp_ns_arg);
define_string_store(mp_ns, mp_ns_arg, null_cb);
power_attr(mp_ns);

define_string_show(mp_ts, mp_ts_arg);
define_string_store(mp_ts, mp_ts_arg, null_cb);
power_attr(mp_ts);

define_int_show(mp_decision_ms, mp_decision_ms_value);
define_int_store(mp_decision_ms, mp_decision_ms_value, null_cb);
power_attr(mp_decision_ms);

define_int_show(mp_min_cpus, mp_min_cpus_value);
define_int_store(mp_min_cpus, mp_min_cpus_value, null_cb);
power_attr(mp_min_cpus);

define_int_show(mp_max_cpus, mp_max_cpus_value);
define_int_store(mp_max_cpus, mp_max_cpus_value, null_cb);
power_attr(mp_max_cpus);

define_int_show(mp_spc_enabled, mp_spc_enabled_value);
define_int_store(mp_spc_enabled, mp_spc_enabled_value, null_cb);
power_attr(mp_spc_enabled);

define_int_show(mp_sync_enabled, mp_sync_enabled_value);
define_int_store(mp_sync_enabled, mp_sync_enabled_value, null_cb);
power_attr(mp_sync_enabled);

define_string_show(mp_util_high_and, mp_util_high_and_arg);
define_string_store(mp_util_high_and, mp_util_high_and_arg, null_cb);
power_attr(mp_util_high_and);

define_string_show(mp_util_high_or, mp_util_high_or_arg);
define_string_store(mp_util_high_or, mp_util_high_or_arg, null_cb);
power_attr(mp_util_high_or);

define_string_show(mp_util_low_and, mp_util_low_and_arg);
define_string_store(mp_util_low_and, mp_util_low_and_arg, null_cb);
power_attr(mp_util_low_and);

define_string_show(mp_util_low_or, mp_util_low_or_arg);
define_string_store(mp_util_low_or, mp_util_low_or_arg, null_cb);
power_attr(mp_util_low_or);
#endif 

#ifdef CONFIG_PERFLOCK
extern ssize_t
perflock_scaling_max_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
extern ssize_t
perflock_scaling_min_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
power_attr(perflock_scaling_max);
power_attr(perflock_scaling_min);
#endif

#ifdef CONFIG_HOTPLUG_CPU
ssize_t
cpu_hotplug_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%u", num_online_cpus());
	return ret;
}
ssize_t
cpu_hotplug_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	sysfs_notify(hotplug_kobj, NULL, "cpu_hotplug");
	return 0;
}
power_attr(cpu_hotplug);
#endif

static int charging_enabled_value;

define_int_show(charging_enabled, charging_enabled_value);
ssize_t
charging_enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}
power_attr(charging_enabled);

int pnpmgr_battery_charging_enabled(int charging_enabled)
{
	pr_debug("%s: result = %d\n", __func__, charging_enabled);
	if (charging_enabled_value != charging_enabled) {
		charging_enabled_value = charging_enabled;
		sysfs_notify(battery_kobj, NULL, "charging_enabled");
	}

	return 0;
}

static struct delayed_work touch_boost_work;
static ssize_t
touch_boost_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d", is_touch_boosted);
}
static ssize_t
touch_boost_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 0) {
			cancel_delayed_work(&touch_boost_work);
			flush_scheduled_work();
			is_touch_boosted = 0;
			sysfs_notify(hotplug_kobj, NULL, "touch_boost");
		}
		else if (val && !is_touch_boosted){
			is_touch_boosted = 1;
			sysfs_notify(hotplug_kobj, NULL, "touch_boost");
			schedule_delayed_work(&touch_boost_work,msecs_to_jiffies(val));
		}
		return n;
	}
	return -EINVAL;
}
power_attr(touch_boost);


static struct attribute *cpufreq_g[] = {
#ifdef CONFIG_PERFLOCK
	&perflock_scaling_max_attr.attr,
	&perflock_scaling_min_attr.attr,
#endif
	NULL,
};

static struct attribute *hotplug_g[] = {
#ifdef CONFIG_HOTPLUG_CPU
	&mp_nw_attr.attr,
	&mp_tw_attr.attr,
	&mp_ns_attr.attr,
	&mp_ts_attr.attr,
	&mp_decision_ms_attr.attr,
	&mp_min_cpus_attr.attr,
	&mp_max_cpus_attr.attr,
	&cpu_hotplug_attr.attr,
	&mp_spc_enabled_attr.attr,
	&mp_sync_enabled_attr.attr,
	&mp_util_high_and_attr.attr,
	&mp_util_high_or_attr.attr,
	&mp_util_low_and_attr.attr,
	&mp_util_low_or_attr.attr,
	&touch_boost_attr.attr,
#endif
	NULL,
};

static struct attribute *thermal_g[] = {
	&thermal_c0_attr.attr,
#if (CONFIG_NR_CPUS >= 2)
	&thermal_c1_attr.attr,
#if (CONFIG_NR_CPUS == 4)
	&thermal_c2_attr.attr,
	&thermal_c3_attr.attr,
#endif
#endif
	&thermal_final_attr.attr,
	&thermal_g0_attr.attr,
	&thermal_final_cpu_attr.attr,
	&thermal_final_gpu_attr.attr,
	&thermal_batt_attr.attr,
	&pause_dt_attr.attr,
	NULL,
};

static struct timer_list app_timer;
static ssize_t
app_timeout_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d", app_timeout_expired);
}
static ssize_t
app_timeout_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 0) {
			del_timer_sync(&app_timer);
			app_timeout_expired = 0;
			sysfs_notify(apps_kobj, NULL, "app_timeout");
		}
		else {
			del_timer_sync(&app_timer);
			app_timer.expires = jiffies + HZ * val;
			app_timer.data = 0;
			add_timer(&app_timer);
		}
		return n;
	}
	return -EINVAL;
}
power_attr(app_timeout);

static struct attribute *apps_g[] = {
	&activity_trigger_attr.attr,
	&non_activity_trigger_attr.attr,
	&media_mode_attr.attr,
	&app_timeout_attr.attr,
	NULL,
};

static struct attribute *sysinfo_g[] = {
       &gpu_max_clk_attr.attr,
       NULL,
};

static struct attribute *battery_g[] = {
	&charging_enabled_attr.attr,
	NULL,
};

static struct attribute_group cpufreq_attr_group = {
	.attrs = cpufreq_g,
};

static struct attribute_group hotplug_attr_group = {
	.attrs = hotplug_g,
};

static struct attribute_group thermal_attr_group = {
	.attrs = thermal_g,
};

static struct attribute_group apps_attr_group = {
	.attrs = apps_g,
};

static struct attribute_group sysinfo_attr_group = {
       .attrs = sysinfo_g,
};

static struct attribute_group battery_attr_group = {
	.attrs = battery_g,
};

#ifdef CONFIG_HOTPLUG_CPU
static int __cpuinit cpu_hotplug_callback(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	switch (action) {
		
		case CPU_ONLINE:
		case CPU_ONLINE_FROZEN:
			sysfs_notify(hotplug_kobj, NULL, "cpu_hotplug");
			break;
		case CPU_DEAD:
		case CPU_DEAD_FROZEN:
			break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_callback,
	.priority = -10, 
};
#endif

static unsigned int slack_time_ms;
static unsigned int step_time_ms;
static unsigned int max_powersave_bias;
static unsigned int powersave_bias_step;
static unsigned int parameter_changed;
static unsigned int adaptive_policy_enabled = 1;

define_int_show(slack_time_ms, slack_time_ms);
define_int_store(slack_time_ms, slack_time_ms, null_cb);
power_attr(slack_time_ms);

define_int_show(step_time_ms, step_time_ms);
define_int_store(step_time_ms, step_time_ms, null_cb);
power_attr(step_time_ms);

define_int_show(max_powersave_bias, max_powersave_bias);
define_int_store(max_powersave_bias, max_powersave_bias, null_cb);
power_attr(max_powersave_bias);

define_int_show(powersave_bias_step, powersave_bias_step);
define_int_store(powersave_bias_step, powersave_bias_step, null_cb);
power_attr(powersave_bias_step);


define_int_show(parameter_changed, parameter_changed);
define_int_store(parameter_changed, parameter_changed, null_cb);
power_attr(parameter_changed);

define_int_show(enabled, adaptive_policy_enabled);
define_int_store(enabled, adaptive_policy_enabled, null_cb);
power_attr(enabled);


static struct attribute *adaptive_attr[] = {
	&slack_time_ms_attr.attr,
	&step_time_ms_attr.attr,
	&max_powersave_bias_attr.attr,
	&powersave_bias_step_attr.attr,
	&parameter_changed_attr.attr,
	&enabled_attr.attr,
	NULL,
};

static struct attribute_group adaptive_attr_group = {
	.attrs = adaptive_attr,
};

static void app_timeout_handler(unsigned long data)
{
	app_timeout_expired = 1;
	sysfs_notify(apps_kobj, NULL, "app_timeout");
}

static void touch_boost_handler(struct work_struct *work)
{
	is_touch_boosted = 0;
	sysfs_notify(hotplug_kobj, NULL, "touch_boost");
}

static int __init pnpmgr_init(void)
{
	int ret;

	init_timer(&app_timer);
	app_timer.function = app_timeout_handler;

	INIT_DELAYED_WORK(&touch_boost_work, touch_boost_handler);

	pnpmgr_kobj = kobject_create_and_add("pnpmgr", power_kobj);

	if (!pnpmgr_kobj) {
		pr_err("%s: Can not allocate enough memory for pnpmgr.\n", __func__);
		return -ENOMEM;
	}

	cpufreq_kobj = kobject_create_and_add("cpufreq", pnpmgr_kobj);
	hotplug_kobj = kobject_create_and_add("hotplug", pnpmgr_kobj);
	thermal_kobj = kobject_create_and_add("thermal", pnpmgr_kobj);
	apps_kobj = kobject_create_and_add("apps", pnpmgr_kobj);
	sysinfo_kobj = kobject_create_and_add("sysinfo", pnpmgr_kobj);
	battery_kobj = kobject_create_and_add("battery", pnpmgr_kobj);
	adaptive_policy_kobj = kobject_create_and_add("adaptive_policy", power_kobj);

	if (!cpufreq_kobj || !hotplug_kobj || !thermal_kobj || !apps_kobj || !sysinfo_kobj || !battery_kobj || !adaptive_policy_kobj) {
		pr_err("%s: Can not allocate enough memory.\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(cpufreq_kobj, &cpufreq_attr_group);
	ret |= sysfs_create_group(hotplug_kobj, &hotplug_attr_group);
	ret |= sysfs_create_group(thermal_kobj, &thermal_attr_group);
	ret |= sysfs_create_group(apps_kobj, &apps_attr_group);
	ret |= sysfs_create_group(sysinfo_kobj, &sysinfo_attr_group);
	ret |= sysfs_create_group(battery_kobj, &battery_attr_group);
	ret |= sysfs_create_group(adaptive_policy_kobj, &adaptive_attr_group);

	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

#ifdef CONFIG_HOTPLUG_CPU
	register_hotcpu_notifier(&cpu_hotplug_notifier);
#endif

	return 0;
}

static void  __exit pnpmgr_exit(void)
{
	sysfs_remove_group(cpufreq_kobj, &cpufreq_attr_group);
	sysfs_remove_group(hotplug_kobj, &hotplug_attr_group);
	sysfs_remove_group(thermal_kobj, &thermal_attr_group);
	sysfs_remove_group(apps_kobj, &apps_attr_group);
	sysfs_remove_group(sysinfo_kobj, &sysinfo_attr_group);
	sysfs_remove_group(battery_kobj, &battery_attr_group);
	sysfs_remove_group(adaptive_policy_kobj, &adaptive_attr_group);
#ifdef CONFIG_HOTPLUG_CPU
	unregister_hotcpu_notifier(&cpu_hotplug_notifier);
#endif
}

module_init(pnpmgr_init);
module_exit(pnpmgr_exit);

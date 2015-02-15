/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/android_alarm.h>
#include <linux/thermal.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_thermal_ioctl.h>
#include <mach/devices_dtb.h>
#include <mach/rpm-smd.h>
#include <mach/scm.h>

#define MAX_CURRENT_UA 1000000
#define MAX_RAILS 5
#define MAX_THRESHOLD 2
#define MONITOR_ALL_TSENS -1
#define BYTES_PER_FUSE_ROW  8
#define MAX_EFUSE_VALUE  16
#define THERM_SECURE_BITE_CMD 8

static struct msm_thermal_data msm_thermal_info;
static struct delayed_work check_temp_work;
static bool core_control_enabled;
static uint32_t cpus_offlined;
static DEFINE_MUTEX(core_control_mutex);
static uint32_t wakeup_ms;
static struct alarm thermal_rtc;
static struct kobject *tt_kobj;
static struct kobject *cc_kobj;
static struct work_struct timer_work;
static struct task_struct *hotplug_task;
static struct task_struct *freq_mitigation_task;
static struct task_struct *thermal_monitor_task;
static struct completion hotplug_notify_complete;
static struct completion freq_mitigation_complete;
static struct completion thermal_monitor_complete;

static int enabled;
static int polling_enabled;
static int rails_cnt;
static int psm_rails_cnt;
static int ocr_rail_cnt;
static int limit_idx;
static int limit_idx_low;
static int limit_idx_high;
static int max_tsens_num;
static struct cpufreq_frequency_table *table;
static uint32_t usefreq;
static int freq_table_get;
static bool vdd_rstr_enabled;
static bool vdd_rstr_nodes_called;
static bool vdd_rstr_probed;
static bool psm_enabled;
static bool psm_nodes_called;
static bool psm_probed;
static bool hotplug_enabled;
static bool freq_mitigation_enabled;
static bool ocr_enabled;
static bool ocr_nodes_called;
static bool ocr_probed;
static bool interrupt_mode_enable;
static bool msm_thermal_probed;
static bool therm_reset_enabled;
static int *tsens_id_map;
static DEFINE_MUTEX(vdd_rstr_mutex);
static DEFINE_MUTEX(psm_mutex);
static DEFINE_MUTEX(ocr_mutex);
static uint32_t min_freq_limit;
static uint32_t default_cpu_temp_limit;
static bool default_temp_limit_enabled;
static bool default_temp_limit_probed;
static bool default_temp_limit_nodes_called;

enum thermal_threshold {
	HOTPLUG_THRESHOLD_HIGH,
	HOTPLUG_THRESHOLD_LOW,
	FREQ_THRESHOLD_HIGH,
	FREQ_THRESHOLD_LOW,
	THRESHOLD_MAX_NR,
};

enum sensor_id_type {
	THERM_ZONE_ID,
	THERM_TSENS_ID,
	THERM_ID_MAX_NR,
};

struct cpu_info {
	uint32_t cpu;
	const char *sensor_type;
	enum sensor_id_type id_type;
	uint32_t sensor_id;
	bool offline;
	bool user_offline;
	bool hotplug_thresh_clear;
	struct sensor_threshold threshold[THRESHOLD_MAX_NR];
	bool max_freq;
	uint32_t user_max_freq;
	uint32_t user_min_freq;
	uint32_t limited_max_freq;
	uint32_t limited_min_freq;
	bool freq_thresh_clear;
};

struct threshold_info;
struct therm_threshold {
	int32_t sensor_id;
	struct sensor_threshold threshold[MAX_THRESHOLD];
	int32_t trip_triggered;
	void (*notify)(struct therm_threshold *);
	struct threshold_info *parent;
};

struct threshold_info {
	uint32_t thresh_ct;
	bool thresh_triggered;
	struct therm_threshold *thresh_list;
};

struct rail {
	const char *name;
	uint32_t freq_req;
	uint32_t min_level;
	uint32_t num_levels;
	int32_t curr_level;
	uint32_t levels[3];
	struct kobj_attribute value_attr;
	struct kobj_attribute level_attr;
	struct regulator *reg;
	struct attribute_group attr_gp;
};

struct psm_rail {
	const char *name;
	uint8_t init;
	uint8_t mode;
	struct kobj_attribute mode_attr;
	struct rpm_regulator *reg;
	struct regulator *phase_reg;
	struct attribute_group attr_gp;
};

enum msm_thresh_list {
	MSM_THERM_RESET,
	MSM_VDD_RESTRICTION,
	MSM_LIST_MAX_NR,
};

static struct psm_rail *psm_rails;
static struct psm_rail *ocr_rails;
static struct rail *rails;
static struct cpu_info cpus[NR_CPUS];
static struct threshold_info *thresh;

struct vdd_rstr_enable {
	struct kobj_attribute ko_attr;
	uint32_t enabled;
};

enum efuse_data {
	EFUSE_ADDRESS = 0,
	EFUSE_SIZE,
	EFUSE_ROW,
	EFUSE_START_BIT,
	EFUSE_BIT_MASK,
	EFUSE_DATA_MAX,
};

enum PMIC_SW_MODE {
	PMIC_AUTO_MODE  = RPM_REGULATOR_MODE_AUTO,
	PMIC_IPEAK_MODE = RPM_REGULATOR_MODE_IPEAK,
	PMIC_PWM_MODE   = RPM_REGULATOR_MODE_HPM,
};

enum ocr_request {
	OPTIMUM_CURRENT_MIN,
	OPTIMUM_CURRENT_MAX,
	OPTIMUM_CURRENT_NR,
};

#define VDD_RES_RO_ATTRIB(_rail, ko_attr, j, _name) \
	ko_attr.attr.name = __stringify(_name); \
	ko_attr.attr.mode = 0444; \
	ko_attr.show = vdd_rstr_reg_##_name##_show; \
	ko_attr.store = NULL; \
	sysfs_attr_init(&ko_attr.attr); \
	_rail.attr_gp.attrs[j] = &ko_attr.attr;

#define VDD_RES_RW_ATTRIB(_rail, ko_attr, j, _name) \
	ko_attr.attr.name = __stringify(_name); \
	ko_attr.attr.mode = 0644; \
	ko_attr.show = vdd_rstr_reg_##_name##_show; \
	ko_attr.store = vdd_rstr_reg_##_name##_store; \
	sysfs_attr_init(&ko_attr.attr); \
	_rail.attr_gp.attrs[j] = &ko_attr.attr;

#define VDD_RSTR_ENABLE_FROM_ATTRIBS(attr) \
	(container_of(attr, struct vdd_rstr_enable, ko_attr));

#define VDD_RSTR_REG_VALUE_FROM_ATTRIBS(attr) \
	(container_of(attr, struct rail, value_attr));

#define VDD_RSTR_REG_LEVEL_FROM_ATTRIBS(attr) \
	(container_of(attr, struct rail, level_attr));

#define OCR_RW_ATTRIB(_rail, ko_attr, j, _name) \
	ko_attr.attr.name = __stringify(_name); \
	ko_attr.attr.mode = 0644; \
	ko_attr.show = ocr_reg_##_name##_show; \
	ko_attr.store = ocr_reg_##_name##_store; \
	sysfs_attr_init(&ko_attr.attr); \
	_rail.attr_gp.attrs[j] = &ko_attr.attr;

#define PSM_RW_ATTRIB(_rail, ko_attr, j, _name) \
	ko_attr.attr.name = __stringify(_name); \
	ko_attr.attr.mode = 0644; \
	ko_attr.show = psm_reg_##_name##_show; \
	ko_attr.store = psm_reg_##_name##_store; \
	sysfs_attr_init(&ko_attr.attr); \
	_rail.attr_gp.attrs[j] = &ko_attr.attr;

#define PSM_REG_MODE_FROM_ATTRIBS(attr) \
	(container_of(attr, struct psm_rail, mode_attr));

static int  msm_thermal_cpufreq_callback(struct notifier_block *nfb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	uint32_t max_freq_req = cpus[policy->cpu].limited_max_freq;
	uint32_t min_freq_req = cpus[policy->cpu].limited_min_freq;

	switch (event) {
	case CPUFREQ_INCOMPATIBLE:
		pr_debug("mitigating CPU%d to freq max: %u min: %u\n",
		policy->cpu, max_freq_req, min_freq_req);

		cpufreq_verify_within_limits(policy, min_freq_req,
			max_freq_req);

		if (max_freq_req < min_freq_req)
			pr_err("Invalid frequency request Max:%u Min:%u\n",
				max_freq_req, min_freq_req);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block msm_thermal_cpufreq_notifier = {
	.notifier_call = msm_thermal_cpufreq_callback,
};

static int check_freq_table(void)
{
	int ret = 0;
	struct cpufreq_frequency_table *table = NULL;

	table = cpufreq_frequency_get_table(0);
	if (!table) {
		pr_debug("error reading cpufreq table\n");
		return -EINVAL;
	}
	freq_table_get = 1;

	return ret;
}

static void update_cpu_freq(int cpu)
{
	int ret = 0;

	if (cpu_online(cpu)) {
		ret = cpufreq_update_policy(cpu);
		if (ret)
			pr_err("Unable to update policy for cpu:%d. err:%d\n",
				cpu, ret);
	}
}

static int update_cpu_min_freq_all(uint32_t min)
{
	uint32_t cpu = 0;
	int ret = 0;

	if (!freq_table_get) {
		ret = check_freq_table();
		if (ret) {
			pr_err("Fail to get freq table. err:%d\n", ret);
			return ret;
		}
	}
	
	min = min(min, table[limit_idx_high].frequency);

	pr_debug("Requesting min freq:%u for all CPU's\n", min);
	if (freq_mitigation_task) {
		min_freq_limit = min;
		complete(&freq_mitigation_complete);
	} else {
		get_online_cpus();
		for_each_possible_cpu(cpu) {
			cpus[cpu].limited_min_freq = min;
			update_cpu_freq(cpu);
		}
		put_online_cpus();
	}

	return ret;
}

static int vdd_restriction_apply_freq(struct rail *r, int level)
{
	int ret = 0;

	if (level == r->curr_level)
		return ret;

	
	if (level == -1) {
		ret = update_cpu_min_freq_all(r->min_level);
		if (ret)
			return ret;
		else
			r->curr_level = -1;
	} else if (level >= 0 && level < (r->num_levels)) {
		ret = update_cpu_min_freq_all(r->levels[level]);
		if (ret)
			return ret;
		else
			r->curr_level = level;
	} else {
		pr_err("level input:%d is not within range\n", level);
		return -EINVAL;
	}

	return ret;
}

static int vdd_restriction_apply_voltage(struct rail *r, int level)
{
	int ret = 0;

	if (r->reg == NULL) {
		pr_err("%s don't have regulator handle. can't apply vdd\n",
				r->name);
		return -EFAULT;
	}
	if (level == r->curr_level)
		return ret;

	
	if (level == -1) {
		ret = regulator_set_voltage(r->reg, r->min_level,
			r->levels[r->num_levels - 1]);
		if (!ret)
			r->curr_level = -1;
		pr_debug("Requested min level for %s. curr level: %d\n",
				r->name, r->curr_level);
	} else if (level >= 0 && level < (r->num_levels)) {
		ret = regulator_set_voltage(r->reg, r->levels[level],
			r->levels[r->num_levels - 1]);
		if (!ret)
			r->curr_level = level;
		pr_debug("Requesting level %d for %s. curr level: %d\n",
			r->levels[level], r->name, r->levels[r->curr_level]);
	} else {
		pr_err("level input:%d is not within range\n", level);
		return -EINVAL;
	}

	return ret;
}

static int psm_set_mode_all(int mode)
{
	int i = 0;
	int fail_cnt = 0;
	int ret = 0;

	pr_debug("Requesting PMIC Mode: %d\n", mode);
	for (i = 0; i < psm_rails_cnt; i++) {
		if (psm_rails[i].mode != mode) {
			ret = rpm_regulator_set_mode(psm_rails[i].reg, mode);
			if (ret) {
				pr_err("Cannot set mode:%d for %s. err:%d",
					mode, psm_rails[i].name, ret);
				fail_cnt++;
			} else
				psm_rails[i].mode = mode;
		}
	}

	return fail_cnt ? (-EFAULT) : ret;
}

static ssize_t default_cpu_temp_limit_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", default_cpu_temp_limit);
}

static int vdd_rstr_en_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vdd_rstr_enable *en = VDD_RSTR_ENABLE_FROM_ATTRIBS(attr);

	return snprintf(buf, PAGE_SIZE, "%d\n", en->enabled);
}

static ssize_t vdd_rstr_en_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int i = 0;
	uint8_t en_cnt = 0;
	uint8_t dis_cnt = 0;
	uint32_t val = 0;
	struct kernel_param kp;
	struct vdd_rstr_enable *en = VDD_RSTR_ENABLE_FROM_ATTRIBS(attr);

	mutex_lock(&vdd_rstr_mutex);
	kp.arg = &val;
	ret = param_set_bool(buf, &kp);
	if (ret) {
		pr_err("Invalid input %s for enabled\n", buf);
		goto done_vdd_rstr_en;
	}

	if ((val == 0) && (en->enabled == 0))
		goto done_vdd_rstr_en;

	for (i = 0; i < rails_cnt; i++) {
		if (rails[i].freq_req == 1 && freq_table_get)
			ret = vdd_restriction_apply_freq(&rails[i],
					(val) ? 0 : -1);
		else
			ret = vdd_restriction_apply_voltage(&rails[i],
			(val) ? 0 : -1);

		if (ret)
			pr_err("Set vdd restriction for %s failed\n",
					rails[i].name);
		else {
			if (val)
				en_cnt++;
			else
				dis_cnt++;
		}
	}
	
	if (val && en_cnt)
		en->enabled = 1;
	else if (!val && (dis_cnt == rails_cnt))
		en->enabled = 0;
	pr_debug("%s vdd restriction. curr: %d\n",
			(val) ? "Enable" : "Disable", en->enabled);

done_vdd_rstr_en:
	mutex_unlock(&vdd_rstr_mutex);
	return count;
}

static struct vdd_rstr_enable vdd_rstr_en = {
	.ko_attr.attr.name = __stringify(enabled),
	.ko_attr.attr.mode = 0644,
	.ko_attr.show = vdd_rstr_en_show,
	.ko_attr.store = vdd_rstr_en_store,
	.enabled = 1,
};

static struct attribute *vdd_rstr_en_attribs[] = {
	&vdd_rstr_en.ko_attr.attr,
	NULL,
};

static struct attribute_group vdd_rstr_en_attribs_gp = {
	.attrs  = vdd_rstr_en_attribs,
};

static int vdd_rstr_reg_value_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int val = 0;
	struct rail *reg = VDD_RSTR_REG_VALUE_FROM_ATTRIBS(attr);
	
	if (reg->curr_level < 0)
		val = reg->curr_level;
	else
		val = reg->levels[reg->curr_level];

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static int vdd_rstr_reg_level_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct rail *reg = VDD_RSTR_REG_LEVEL_FROM_ATTRIBS(attr);
	return snprintf(buf, PAGE_SIZE, "%d\n", reg->curr_level);
}

static ssize_t vdd_rstr_reg_level_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	struct rail *reg = VDD_RSTR_REG_LEVEL_FROM_ATTRIBS(attr);

	mutex_lock(&vdd_rstr_mutex);
	if (vdd_rstr_en.enabled == 0)
		goto done_store_level;

	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s for level\n", buf);
		goto done_store_level;
	}

	if (val < 0 || val > reg->num_levels - 1) {
		pr_err(" Invalid number %d for level\n", val);
		goto done_store_level;
	}

	if (val != reg->curr_level) {
		if (reg->freq_req == 1 && freq_table_get)
			update_cpu_min_freq_all(reg->levels[val]);
		else {
			ret = vdd_restriction_apply_voltage(reg, val);
			if (ret) {
				pr_err( \
				"Set vdd restriction for regulator %s failed. err:%d\n",
				reg->name, ret);
				goto done_store_level;
			}
		}
		reg->curr_level = val;
		pr_debug("Request level %d for %s\n",
				reg->curr_level, reg->name);
	}

done_store_level:
	mutex_unlock(&vdd_rstr_mutex);
	return count;
}

static int request_optimum_current(struct psm_rail *rail, enum ocr_request req)
{
	int ret = 0;

	if ((!rail) || (req >= OPTIMUM_CURRENT_NR) ||
		(req < 0)) {
		pr_err("%s:%s Invalid input\n", KBUILD_MODNAME, __func__);
		ret = -EINVAL;
		goto request_ocr_exit;
	}

	ret = regulator_set_optimum_mode(rail->phase_reg,
		(req == OPTIMUM_CURRENT_MAX) ? MAX_CURRENT_UA : 0);
	if (ret < 0) {
		pr_err("%s: Optimum current request failed\n", KBUILD_MODNAME);
		goto request_ocr_exit;
	}
	ret = 0; 
	pr_debug("%s: Requested optimum current mode: %d\n",
		KBUILD_MODNAME, req);

request_ocr_exit:
	return ret;
}

static int ocr_set_mode_all(enum ocr_request req)
{
	int ret = 0, i;

	for (i = 0; i < ocr_rail_cnt; i++) {
		if (ocr_rails[i].mode == req)
			continue;
		ret = request_optimum_current(&ocr_rails[i], req);
		if (ret)
			goto ocr_set_mode_exit;
		ocr_rails[i].mode = req;
	}

ocr_set_mode_exit:
	return ret;
}

static int ocr_reg_mode_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);
	return snprintf(buf, PAGE_SIZE, "%d\n", reg->mode);
}

static ssize_t ocr_reg_mode_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);

	if (!ocr_enabled)
		return count;

	mutex_lock(&ocr_mutex);
	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("%s: Invalid input %s for mode\n",
			KBUILD_MODNAME, buf);
		goto done_ocr_store;
	}

	if ((val != OPTIMUM_CURRENT_MAX) &&
		(val != OPTIMUM_CURRENT_MIN)) {
		pr_err("%s: Invalid value %d for mode\n",
			KBUILD_MODNAME, val);
		goto done_ocr_store;
	}

	if (val != reg->mode) {
		ret = request_optimum_current(reg, val);
		if (ret)
			goto done_ocr_store;
		reg->mode = val;
	}

done_ocr_store:
	mutex_unlock(&ocr_mutex);
	return count;
}

static int psm_reg_mode_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);
	return snprintf(buf, PAGE_SIZE, "%d\n", reg->mode);
}

static ssize_t psm_reg_mode_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);

	mutex_lock(&psm_mutex);
	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s for mode\n", buf);
		goto done_psm_store;
	}

	if ((val != PMIC_PWM_MODE) && (val != PMIC_AUTO_MODE)) {
		pr_err("Invalid number %d for mode\n", val);
		goto done_psm_store;
	}

	if (val != reg->mode) {
		ret = rpm_regulator_set_mode(reg->reg, val);
		if (ret) {
			pr_err("Fail to set Mode:%d for %s. err:%d\n",
			val, reg->name, ret);
			goto done_psm_store;
		}
		reg->mode = val;
	}

done_psm_store:
	mutex_unlock(&psm_mutex);
	return count;
}

static int check_sensor_id(int sensor_id)
{
	int i = 0;
	bool hw_id_found = false;
	int ret = 0;

	for (i = 0; i < max_tsens_num; i++) {
		if (sensor_id == tsens_id_map[i]) {
			hw_id_found = true;
			break;
		}
	}
	if (!hw_id_found) {
		pr_err("Invalid sensor hw id:%d\n", sensor_id);
		return -EINVAL;
	}

	return ret;
}

static int create_sensor_id_map(void)
{
	int i = 0;
	int ret = 0;

	tsens_id_map = kzalloc(sizeof(int) * max_tsens_num,
			GFP_KERNEL);
	if (!tsens_id_map) {
		pr_err("Cannot allocate memory for tsens_id_map\n");
		return -ENOMEM;
	}

	for (i = 0; i < max_tsens_num; i++) {
		ret = tsens_get_hw_id_mapping(i, &tsens_id_map[i]);
		
		if (ret) {
			if (ret == -ENXIO) {
				tsens_id_map[i] = i;
				ret = 0;
			} else {
				pr_err("Failed to get hw id for id:%d.err:%d\n",
						i, ret);
				goto fail;
			}
		}
	}

	return ret;
fail:
	kfree(tsens_id_map);
	return ret;
}

static int vdd_restriction_apply_all(int en)
{
	int i = 0;
	int en_cnt = 0;
	int dis_cnt = 0;
	int fail_cnt = 0;
	int ret = 0;

	for (i = 0; i < rails_cnt; i++) {
		if (rails[i].freq_req == 1 && freq_table_get)
			ret = vdd_restriction_apply_freq(&rails[i],
					en ? 0 : -1);
		else
			ret = vdd_restriction_apply_voltage(&rails[i],
					en ? 0 : -1);
		if (ret) {
			pr_err("Failed to %s for %s. err:%d",
					(en) ? "enable" : "disable",
					rails[i].name, ret);
			fail_cnt++;
		} else {
			if (en)
				en_cnt++;
			else
				dis_cnt++;
		}
	}

	
	if (en && en_cnt)
		vdd_rstr_en.enabled = 1;
	else if (!en && (dis_cnt == rails_cnt))
		vdd_rstr_en.enabled = 0;

	if (fail_cnt)
		return -EFAULT;
	return ret;
}

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	int i = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_err("error reading cpufreq table\n");
		ret = -EINVAL;
		goto fail;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	limit_idx_low = 0;
	limit_idx_high = limit_idx = i - 1;
	BUG_ON(limit_idx_high <= 0 || limit_idx_high <= limit_idx_low);
fail:
	return ret;
}

static int set_and_activate_threshold(uint32_t sensor_id,
	struct sensor_threshold *threshold)
{
	int ret = 0;

	ret = sensor_set_trip(sensor_id, threshold);
	if (ret != 0) {
		pr_err("sensor:%u Error in setting trip:%d. err:%d\n",
			sensor_id, threshold->trip, ret);
		goto set_done;
	}

	ret = sensor_activate_trip(sensor_id, threshold, true);
	if (ret != 0) {
		pr_err("sensor:%u Error in enabling trip:%d. err:%d\n",
			sensor_id, threshold->trip, ret);
		goto set_done;
	}

set_done:
	return ret;
}

static int therm_get_temp(uint32_t id, enum sensor_id_type type, long *temp)
{
	int ret = 0;
	struct tsens_device tsens_dev;

	if (!temp) {
		pr_err("Invalid value\n");
		ret = -EINVAL;
		goto get_temp_exit;
	}

	switch (type) {
	case THERM_ZONE_ID:
		tsens_dev.sensor_num = tsens_id_map[id];
		break;
	case THERM_TSENS_ID:
		tsens_dev.sensor_num = id;
		break;
	default:
		pr_err("Invalid type\n");
		ret = -EINVAL;
		goto get_temp_exit;
		break;
	}

	ret = tsens_get_temp(&tsens_dev, temp);
	if (ret) {
		pr_err("Unable to read TSENS sensor:%d\n",
			tsens_dev.sensor_num);
		goto get_temp_exit;
	}

get_temp_exit:
	return ret;
}

static int set_threshold(uint32_t zone_id,
	struct sensor_threshold *threshold)
{
	int i = 0, ret = 0;
	long temp;

	if ((!threshold) || (zone_id >= max_tsens_num)) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto set_threshold_exit;
	}

	ret = therm_get_temp(zone_id, THERM_ZONE_ID, &temp);
	if (ret) {
		pr_err("Unable to read temperature for zone:%d. err:%d\n",
			zone_id, ret);
		goto set_threshold_exit;
	}

	while (i < MAX_THRESHOLD) {
		switch (threshold[i].trip) {
		case THERMAL_TRIP_CONFIGURABLE_HI:
			if (threshold[i].temp >= temp) {
				ret = set_and_activate_threshold(zone_id,
					&threshold[i]);
				if (ret)
					goto set_threshold_exit;
			}
			break;
		case THERMAL_TRIP_CONFIGURABLE_LOW:
			if (threshold[i].temp <= temp) {
				ret = set_and_activate_threshold(zone_id,
					&threshold[i]);
				if (ret)
					goto set_threshold_exit;
			}
			break;
		default:
			pr_err("zone:%u Invalid trip:%d\n", zone_id,
					threshold[i].trip);
			break;
		}
		i++;
	}
set_threshold_exit:
	return ret;
}

static void msm_thermal_bite(int tsens_id, long temp)
{
	pr_err("TSENS:%d reached temperature:%ld. System reset\n",
		tsens_id, temp);
	scm_call_atomic1(SCM_SVC_BOOT, THERM_SECURE_BITE_CMD, 0);
}

static int do_therm_reset(void)
{
	int ret = 0, i;
	long temp = 0;

	if (!therm_reset_enabled)
		return ret;

	for (i = 0; i < thresh[MSM_THERM_RESET].thresh_ct; i++) {
		ret = therm_get_temp(
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id,
			THERM_TSENS_ID,
			&temp);
		if (ret) {
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id,
			ret);
			continue;
		}

		if (temp >= msm_thermal_info.therm_reset_temp_degC)
			msm_thermal_bite(
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id, temp);
	}

	return ret;
}

static void therm_reset_notify(struct therm_threshold *thresh_data)
{
	long temp;
	int ret = 0;

	if (!therm_reset_enabled)
		return;

	if (!thresh_data) {
		pr_err("Invalid input\n");
		return;
	}

	switch (thresh_data->trip_triggered) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		ret = therm_get_temp(thresh_data->sensor_id,
				THERM_TSENS_ID, &temp);
		if (ret)
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
				thresh_data->sensor_id, ret);
		msm_thermal_bite(tsens_id_map[thresh_data->sensor_id],
					temp);
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		break;
	default:
		pr_err("Invalid trip type\n");
		break;
	}
	set_threshold(thresh_data->sensor_id, thresh_data->threshold);
}

#ifdef CONFIG_SMP
static void __ref do_core_control(long temp)
{
	int i = 0;
	int ret = 0;

	if (!core_control_enabled)
		return;

	mutex_lock(&core_control_mutex);
	if (msm_thermal_info.core_control_mask &&
		temp >= msm_thermal_info.core_limit_temp_degC) {
		for (i = num_possible_cpus(); i > 0; i--) {
			if (!(msm_thermal_info.core_control_mask & BIT(i)))
				continue;
			if (cpus_offlined & BIT(i) && !cpu_online(i))
				continue;
			pr_info("Set Offline: CPU%d Temp: %ld\n",
					i, temp);
			ret = cpu_down(i);
			if (ret)
				pr_err("Error %d offline core %d\n",
					ret, i);
			cpus_offlined |= BIT(i);
			break;
		}
	} else if (msm_thermal_info.core_control_mask && cpus_offlined &&
		temp <= (msm_thermal_info.core_limit_temp_degC -
			msm_thermal_info.core_temp_hysteresis_degC)) {
		for (i = 0; i < num_possible_cpus(); i++) {
			if (!(cpus_offlined & BIT(i)))
				continue;
			cpus_offlined &= ~BIT(i);
			pr_info("Allow Online CPU%d Temp: %ld\n",
					i, temp);
			if (cpu_online(i))
				continue;
			ret = cpu_up(i);
			if (ret)
				pr_err("Error %d online core %d\n",
						ret, i);
			break;
		}
	}
	mutex_unlock(&core_control_mutex);
}
static int __ref update_offline_cores(int val)
{
	uint32_t cpu = 0;
	int ret = 0;

	if (!core_control_enabled)
		return 0;

	cpus_offlined = msm_thermal_info.core_control_mask & val;

	for_each_possible_cpu(cpu) {
		if (!(cpus_offlined & BIT(cpu)))
			continue;
		if (!cpu_online(cpu))
			continue;
		ret = cpu_down(cpu);
		if (ret)
			pr_err("Unable to offline CPU%d. err:%d\n",
				cpu, ret);
		else
			pr_debug("Offlined CPU%d\n", cpu);
	}
	return ret;
}

static __ref int do_hotplug(void *data)
{
	int ret = 0;
	uint32_t cpu = 0, mask = 0;

	if (!core_control_enabled) {
		pr_debug("Core control disabled\n");
		return -EINVAL;
	}

	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&hotplug_notify_complete) != 0)
			;
		INIT_COMPLETION(hotplug_notify_complete);
		mask = 0;

		mutex_lock(&core_control_mutex);
		for_each_possible_cpu(cpu) {
			if (hotplug_enabled &&
				cpus[cpu].hotplug_thresh_clear) {
				set_threshold(cpus[cpu].sensor_id,
				&cpus[cpu].threshold[HOTPLUG_THRESHOLD_HIGH]);

				cpus[cpu].hotplug_thresh_clear = false;
			}
			if (cpus[cpu].offline || cpus[cpu].user_offline)
				mask |= BIT(cpu);
		}
		if (mask != cpus_offlined)
			update_offline_cores(mask);
		mutex_unlock(&core_control_mutex);
		sysfs_notify(cc_kobj, NULL, "cpus_offlined");
	}

	return ret;
}
#else
static void do_core_control(long temp)
{
	return;
}

static __ref int do_hotplug(void *data)
{
	return 0;
}
#endif

static int do_ocr(void)
{
	long temp = 0;
	int ret = 0;
	int i = 0, j = 0;
	int auto_cnt = 0;

	if (!ocr_enabled)
		return ret;

	mutex_lock(&ocr_mutex);
	for (i = 0; i < max_tsens_num; i++) {
		ret = therm_get_temp(tsens_id_map[i], THERM_TSENS_ID, &temp);
		if (ret) {
			pr_debug("%s: Unable to read TSENS sensor %d\n",
					__func__, tsens_id_map[i]);
			auto_cnt++;
			continue;
		}

		if (temp > msm_thermal_info.ocr_temp_degC) {
			if (ocr_rails[0].init != OPTIMUM_CURRENT_NR)
				for (j = 0; j < ocr_rail_cnt; j++)
					ocr_rails[j].init = OPTIMUM_CURRENT_NR;
			ret = ocr_set_mode_all(OPTIMUM_CURRENT_MAX);
			if (ret)
				pr_err("Error setting max ocr. err:%d\n",
					ret);
			else
				pr_debug("Requested MAX OCR. tsens:%d Temp:%ld",
					tsens_id_map[i], temp);
			goto do_ocr_exit;
		} else if (temp <= (msm_thermal_info.ocr_temp_degC -
			msm_thermal_info.ocr_temp_hyst_degC))
			auto_cnt++;
	}

	if (auto_cnt == max_tsens_num ||
		ocr_rails[0].init != OPTIMUM_CURRENT_NR) {
		if (ocr_rails[0].init != OPTIMUM_CURRENT_NR)
			for (j = 0; j < ocr_rail_cnt; j++)
				ocr_rails[j].init = OPTIMUM_CURRENT_NR;
		ret = ocr_set_mode_all(OPTIMUM_CURRENT_MIN);
		if (ret) {
			pr_err("Error setting min optimum current\n");
			goto do_ocr_exit;
		} else {
			pr_debug("Requested MIN OCR. Temp:%ld", temp);
		}
	}

do_ocr_exit:
	mutex_unlock(&ocr_mutex);
	return ret;
}

static int do_vdd_restriction(void)
{
	long temp = 0;
	int ret = 0;
	int i = 0;
	int dis_cnt = 0;

	if (!vdd_rstr_enabled)
		return ret;

	if (usefreq && !freq_table_get) {
		if (check_freq_table())
			return ret;
	}

	mutex_lock(&vdd_rstr_mutex);
	for (i = 0; i < max_tsens_num; i++) {
		ret = therm_get_temp(tsens_id_map[i], THERM_TSENS_ID, &temp);
		if (ret) {
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
				tsens_id_map[i], ret);
			dis_cnt++;
			continue;
		}
		if (temp <=  msm_thermal_info.vdd_rstr_temp_degC) {
			ret = vdd_restriction_apply_all(1);
			if (ret) {
				pr_err( \
				"Enable vdd rstr for all failed. err:%d\n",
					ret);
				goto exit;
			}
			pr_debug("Enabled Vdd Restriction tsens:%d. Temp:%ld\n",
			thresh[MSM_VDD_RESTRICTION].thresh_list[i].sensor_id,
			temp);
			goto exit;
		} else if (temp > msm_thermal_info.vdd_rstr_temp_hyst_degC)
			dis_cnt++;
	}
	if (dis_cnt == max_tsens_num) {
		ret = vdd_restriction_apply_all(0);
		if (ret) {
			pr_err("Disable vdd rstr for all failed. err:%d\n",
					ret);
			goto exit;
		}
		pr_debug("Disabled Vdd Restriction\n");
	}
exit:
	mutex_unlock(&vdd_rstr_mutex);
	return ret;
}

static int do_psm(void)
{
	long temp = 0;
	int ret = 0;
	int i = 0;
	int auto_cnt = 0;

	mutex_lock(&psm_mutex);
	for (i = 0; i < max_tsens_num; i++) {
		ret = therm_get_temp(tsens_id_map[i], THERM_TSENS_ID, &temp);
		if (ret) {
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
					tsens_id_map[i], ret);
			auto_cnt++;
			continue;
		}

		if (temp >  msm_thermal_info.psm_temp_degC) {
			ret = psm_set_mode_all(PMIC_PWM_MODE);
			if (ret) {
				pr_err("Set pwm mode for all failed. err:%d\n",
						ret);
				goto exit;
			}
			pr_debug("Requested PMIC PWM Mode tsens:%d. Temp:%ld\n",
					tsens_id_map[i], temp);
			break;
		} else if (temp <= msm_thermal_info.psm_temp_hyst_degC)
			auto_cnt++;
	}

	if (auto_cnt == max_tsens_num) {
		ret = psm_set_mode_all(PMIC_AUTO_MODE);
		if (ret) {
			pr_err("Set auto mode for all failed. err:%d\n", ret);
			goto exit;
		}
		pr_debug("Requested PMIC AUTO Mode\n");
	}

exit:
	mutex_unlock(&psm_mutex);
	return ret;
}

static void __ref do_freq_control(long temp)
{
	uint32_t cpu = 0;
	uint32_t max_freq = cpus[cpu].limited_max_freq;

	if (temp >= msm_thermal_info.limit_temp_degC) {
		if (limit_idx == limit_idx_low)
			return;

		limit_idx -= msm_thermal_info.bootup_freq_step;
		if (limit_idx < limit_idx_low)
			limit_idx = limit_idx_low;
		max_freq = table[limit_idx].frequency;
		pr_info("msm_thermal: TSENS sensor %ld C\n", temp);

	} else if (temp < msm_thermal_info.limit_temp_degC -
		 msm_thermal_info.temp_hysteresis_degC) {
		if (limit_idx == limit_idx_high)
			return;

		limit_idx += msm_thermal_info.bootup_freq_step;
		if (limit_idx >= limit_idx_high) {
			limit_idx = limit_idx_high;
			max_freq = UINT_MAX;
		} else
			max_freq = table[limit_idx].frequency;
	}

	if (max_freq == cpus[cpu].limited_max_freq)
		return;

	
	get_online_cpus();
	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.bootup_freq_control_mask & BIT(cpu)))
			continue;
		pr_info("Limiting CPU%d max frequency to %u. Temp:%ld\n",
			cpu, max_freq, temp);
		cpus[cpu].limited_max_freq = max_freq;
		update_cpu_freq(cpu);
	}
	put_online_cpus();
}

static void __ref check_temp(struct work_struct *work)
{
	static int limit_init;
	long temp = 0;
	int ret = 0;

	do_therm_reset();

	ret = therm_get_temp(msm_thermal_info.sensor_id, THERM_TSENS_ID, &temp);
	if (ret) {
		pr_err("Unable to read TSENS sensor:%d. err:%d\n",
				msm_thermal_info.sensor_id, ret);
		goto reschedule;
	}

	if (!limit_init) {
		ret = msm_thermal_get_freq_table();
		if (ret)
			goto reschedule;
		else
			limit_init = 1;
	}

	do_core_control(temp);
	do_vdd_restriction();
	do_psm();
	do_ocr();
	do_freq_control(temp);

reschedule:
	if (polling_enabled)
		schedule_delayed_work(&check_temp_work,
				msecs_to_jiffies(msm_thermal_info.poll_ms));
}

static int __ref msm_thermal_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	uint32_t cpu = (uint32_t)hcpu;

	if (action == CPU_UP_PREPARE || action == CPU_UP_PREPARE_FROZEN) {
		if (core_control_enabled &&
			(msm_thermal_info.core_control_mask & BIT(cpu)) &&
			(cpus_offlined & BIT(cpu))) {
			pr_debug("Preventing CPU%d from coming online.\n",
				cpu);
			return NOTIFY_BAD;
		}
	}

	pr_debug("voting for CPU%d to be online\n", cpu);
	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_thermal_cpu_notifier = {
	.notifier_call = msm_thermal_cpu_callback,
};

static void thermal_rtc_setup(void)
{
	ktime_t wakeup_time;
	ktime_t curr_time;

	curr_time = alarm_get_elapsed_realtime();
	wakeup_time = ktime_add_us(curr_time,
			(wakeup_ms * USEC_PER_MSEC));
	alarm_start_range(&thermal_rtc, wakeup_time,
			wakeup_time);
	pr_debug("%s: Current Time: %ld %ld, Alarm set to: %ld %ld\n",
			KBUILD_MODNAME,
			ktime_to_timeval(curr_time).tv_sec,
			ktime_to_timeval(curr_time).tv_usec,
			ktime_to_timeval(wakeup_time).tv_sec,
			ktime_to_timeval(wakeup_time).tv_usec);

}

static void timer_work_fn(struct work_struct *work)
{
	sysfs_notify(tt_kobj, NULL, "wakeup_ms");
}

static void thermal_rtc_callback(struct alarm *al)
{
	struct timeval ts;
	ts = ktime_to_timeval(alarm_get_elapsed_realtime());
	schedule_work(&timer_work);
	pr_debug("%s: Time on alarm expiry: %ld %ld\n", KBUILD_MODNAME,
			ts.tv_sec, ts.tv_usec);
}

static int hotplug_notify(enum thermal_trip_type type, int temp, void *data)
{
	struct cpu_info *cpu_node = (struct cpu_info *)data;

	pr_info("%s reach temp threshold: %d\n", cpu_node->sensor_type, temp);

	if (!(msm_thermal_info.core_control_mask & BIT(cpu_node->cpu)))
		return 0;
	switch (type) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		if (!(cpu_node->offline))
			cpu_node->offline = 1;
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		if (cpu_node->offline)
			cpu_node->offline = 0;
		break;
	default:
		break;
	}
	if (hotplug_task) {
		cpu_node->hotplug_thresh_clear = true;
		complete(&hotplug_notify_complete);
	} else {
		pr_err("Hotplug task is not initialized\n");
	}
	return 0;
}
static int hotplug_init_cpu_offlined(void)
{
	long temp = 0;
	uint32_t cpu = 0;

	if (!hotplug_enabled)
		return 0;

	mutex_lock(&core_control_mutex);
	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.core_control_mask & BIT(cpus[cpu].cpu)))
			continue;
		if (therm_get_temp(cpus[cpu].sensor_id, cpus[cpu].id_type,
					&temp)) {
			pr_err("Unable to read TSENS sensor:%d.\n",
				cpus[cpu].sensor_id);
			mutex_unlock(&core_control_mutex);
			return -EINVAL;
		}

		if (temp >= msm_thermal_info.hotplug_temp_degC)
			cpus[cpu].offline = 1;
		else if (temp <= (msm_thermal_info.hotplug_temp_degC -
			msm_thermal_info.hotplug_temp_hysteresis_degC))
			cpus[cpu].offline = 0;
	}
	mutex_unlock(&core_control_mutex);

	if (hotplug_task)
		complete(&hotplug_notify_complete);
	else {
		pr_err("Hotplug task is not initialized\n");
		return -EINVAL;
	}
	return 0;
}

static void hotplug_init(void)
{
	uint32_t cpu = 0;
	struct sensor_threshold *hi_thresh = NULL, *low_thresh = NULL;

	if (hotplug_task)
		return;

	if (!hotplug_enabled)
		goto init_kthread;

	for_each_possible_cpu(cpu) {
		cpus[cpu].sensor_id =
			sensor_get_id((char *)cpus[cpu].sensor_type);
		cpus[cpu].id_type = THERM_ZONE_ID;
		if (!(msm_thermal_info.core_control_mask & BIT(cpus[cpu].cpu)))
			continue;

		hi_thresh = &cpus[cpu].threshold[HOTPLUG_THRESHOLD_HIGH];
		low_thresh = &cpus[cpu].threshold[HOTPLUG_THRESHOLD_LOW];
		hi_thresh->temp = msm_thermal_info.hotplug_temp_degC;
		hi_thresh->trip = THERMAL_TRIP_CONFIGURABLE_HI;
		low_thresh->temp = msm_thermal_info.hotplug_temp_degC -
				msm_thermal_info.hotplug_temp_hysteresis_degC;
		low_thresh->trip = THERMAL_TRIP_CONFIGURABLE_LOW;
		hi_thresh->notify = low_thresh->notify = hotplug_notify;
		hi_thresh->data = low_thresh->data = (void *)&cpus[cpu];

		set_threshold(cpus[cpu].sensor_id, hi_thresh);
	}
init_kthread:
	init_completion(&hotplug_notify_complete);
	hotplug_task = kthread_run(do_hotplug, NULL, "msm_thermal:hotplug");
	if (IS_ERR(hotplug_task)) {
		pr_err("Failed to create do_hotplug thread. err:%ld\n",
				PTR_ERR(hotplug_task));
		return;
	}
	if (hotplug_init_cpu_offlined())
		kthread_stop(hotplug_task);
}

static __ref int do_freq_mitigation(void *data)
{
	int ret = 0;
	uint32_t cpu = 0, max_freq_req = 0, min_freq_req = 0;

	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&freq_mitigation_complete) != 0)
			;
		INIT_COMPLETION(freq_mitigation_complete);

		get_online_cpus();
		for_each_possible_cpu(cpu) {
			max_freq_req = (cpus[cpu].max_freq) ?
					msm_thermal_info.freq_limit :
					UINT_MAX;
			max_freq_req = min(max_freq_req,
					cpus[cpu].user_max_freq);

			min_freq_req = max(min_freq_limit,
					cpus[cpu].user_min_freq);

			if ((max_freq_req == cpus[cpu].limited_max_freq)
				&& (min_freq_req ==
				cpus[cpu].limited_min_freq))
				goto reset_threshold;

			cpus[cpu].limited_max_freq = max_freq_req;
			cpus[cpu].limited_min_freq = min_freq_req;
			update_cpu_freq(cpu);
reset_threshold:
			if (freq_mitigation_enabled &&
				cpus[cpu].freq_thresh_clear) {
				set_threshold(cpus[cpu].sensor_id,
				&cpus[cpu].threshold[FREQ_THRESHOLD_HIGH]);

				cpus[cpu].freq_thresh_clear = false;
			}
		}
		put_online_cpus();
	}
	return ret;
}

static int freq_mitigation_notify(enum thermal_trip_type type,
	int temp, void *data)
{
	struct cpu_info *cpu_node = (struct cpu_info *) data;

	pr_debug("%s reached temp threshold: %d\n",
		cpu_node->sensor_type, temp);

	if (!(msm_thermal_info.freq_mitig_control_mask &
		BIT(cpu_node->cpu)))
		return 0;

	switch (type) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		if (!cpu_node->max_freq) {
			pr_info("Mitigating CPU%d frequency to %d\n",
				cpu_node->cpu,
				msm_thermal_info.freq_limit);

			cpu_node->max_freq = true;
		}
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		if (cpu_node->max_freq) {
			pr_info("Removing frequency mitigation for CPU%d\n",
				cpu_node->cpu);

			cpu_node->max_freq = false;
		}
		break;
	default:
		break;
	}

	if (freq_mitigation_task) {
		cpu_node->freq_thresh_clear = true;
		complete(&freq_mitigation_complete);
	} else {
		pr_err("Frequency mitigation task is not initialized\n");
	}

	return 0;
}

static void freq_mitigation_init(void)
{
	uint32_t cpu = 0;
	struct sensor_threshold *hi_thresh = NULL, *low_thresh = NULL;

	if (freq_mitigation_task)
		return;
	if (!freq_mitigation_enabled)
		goto init_freq_thread;

	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.freq_mitig_control_mask & BIT(cpu)))
			continue;
		hi_thresh = &cpus[cpu].threshold[FREQ_THRESHOLD_HIGH];
		low_thresh = &cpus[cpu].threshold[FREQ_THRESHOLD_LOW];

		hi_thresh->temp = msm_thermal_info.freq_mitig_temp_degc;
		hi_thresh->trip = THERMAL_TRIP_CONFIGURABLE_HI;
		low_thresh->temp = msm_thermal_info.freq_mitig_temp_degc -
			msm_thermal_info.freq_mitig_temp_hysteresis_degc;
		low_thresh->trip = THERMAL_TRIP_CONFIGURABLE_LOW;
		hi_thresh->notify = low_thresh->notify =
			freq_mitigation_notify;
		hi_thresh->data = low_thresh->data = (void *)&cpus[cpu];

		set_threshold(cpus[cpu].sensor_id, hi_thresh);
	}
init_freq_thread:
	init_completion(&freq_mitigation_complete);
	freq_mitigation_task = kthread_run(do_freq_mitigation, NULL,
		"msm_thermal:freq_mitig");

	if (IS_ERR(freq_mitigation_task)) {
		pr_err("Failed to create frequency mitigation thread. err:%ld\n",
				PTR_ERR(freq_mitigation_task));
		return;
	}
}

int msm_thermal_set_frequency(uint32_t cpu, uint32_t freq, bool is_max)
{
	int ret = 0;

	if (cpu >= num_possible_cpus()) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto set_freq_exit;
	}

	pr_debug("Userspace requested %s frequency %u for CPU%u\n",
			(is_max) ? "Max" : "Min", freq, cpu);
	if (is_max) {
		if (cpus[cpu].user_max_freq == freq)
			goto set_freq_exit;

		cpus[cpu].user_max_freq = freq;
	} else {
		if (cpus[cpu].user_min_freq == freq)
			goto set_freq_exit;

		cpus[cpu].user_min_freq = freq;
	}

	if (freq_mitigation_task) {
		complete(&freq_mitigation_complete);
	} else {
		pr_err("Frequency mitigation task is not initialized\n");
		ret = -ESRCH;
		goto set_freq_exit;
	}

set_freq_exit:
	return ret;
}

int therm_set_threshold(struct threshold_info *thresh_inp)
{
	int ret = 0, i = 0, err = 0;
	struct therm_threshold *thresh_ptr;

	if (!thresh_inp) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto therm_set_exit;
	}

	thresh_inp->thresh_triggered = false;
	for (i = 0; i < thresh_inp->thresh_ct; i++) {
		thresh_ptr = &thresh_inp->thresh_list[i];
		thresh_ptr->trip_triggered = -1;
		err = set_threshold(thresh_ptr->sensor_id,
			thresh_ptr->threshold);
		if (err) {
			ret = err;
			err = 0;
		}
	}

therm_set_exit:
	return ret;
}

static void vdd_restriction_notify(struct therm_threshold *trig_thresh)
{
	int ret = 0;
	static uint32_t vdd_sens_status;

	if (!vdd_rstr_enabled)
		return;
	if (!trig_thresh) {
		pr_err("Invalid input\n");
		return;
	}
	if (trig_thresh->trip_triggered < 0)
		goto set_and_exit;

	mutex_lock(&vdd_rstr_mutex);
	pr_debug("sensor:%d reached %s thresh for Vdd restriction\n",
		tsens_id_map[trig_thresh->sensor_id],
		(trig_thresh->trip_triggered == THERMAL_TRIP_CONFIGURABLE_HI) ?
		"high" : "low");
	switch (trig_thresh->trip_triggered) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		if (vdd_sens_status & BIT(trig_thresh->sensor_id))
			vdd_sens_status ^= BIT(trig_thresh->sensor_id);
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		vdd_sens_status |= BIT(trig_thresh->sensor_id);
		break;
	default:
		pr_err("Unsupported trip type\n");
		goto unlock_and_exit;
		break;
	}

	ret = vdd_restriction_apply_all((vdd_sens_status) ? 1 : 0);
	if (ret) {
		pr_err("%s vdd rstr votlage for all failed\n",
			(vdd_sens_status) ?
			"Enable" : "Disable");
			goto unlock_and_exit;
	}

unlock_and_exit:
	mutex_unlock(&vdd_rstr_mutex);
set_and_exit:
	set_threshold(trig_thresh->sensor_id, trig_thresh->threshold);
	return;
}

static __ref int do_thermal_monitor(void *data)
{
	int ret = 0, i, j;
	struct therm_threshold *sensor_list;

	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&thermal_monitor_complete) != 0)
			;
		INIT_COMPLETION(thermal_monitor_complete);

		for (i = 0; i < MSM_LIST_MAX_NR; i++) {
			if (!thresh[i].thresh_triggered)
				continue;
			thresh[i].thresh_triggered = false;
			for (j = 0; j < thresh[i].thresh_ct; j++) {
				sensor_list = &thresh[i].thresh_list[j];
				if (sensor_list->trip_triggered < 0)
					continue;
				sensor_list->notify(sensor_list);
				sensor_list->trip_triggered = -1;
			}
		}
	}
	return ret;
}

static void thermal_monitor_init(void)
{
	if (thermal_monitor_task)
		return;

	init_completion(&thermal_monitor_complete);
	thermal_monitor_task = kthread_run(do_thermal_monitor, NULL,
		"msm_thermal:therm_monitor");
	if (IS_ERR(thermal_monitor_task)) {
		pr_err("Failed to create thermal monitor thread. err:%ld\n",
				PTR_ERR(thermal_monitor_task));
		goto init_exit;
	}

	if (therm_reset_enabled)
		therm_set_threshold(&thresh[MSM_THERM_RESET]);

	if (vdd_rstr_enabled)
		therm_set_threshold(&thresh[MSM_VDD_RESTRICTION]);

init_exit:
	return;
}

static int msm_thermal_notify(enum thermal_trip_type type, int temp, void *data)
{
	struct therm_threshold *thresh_data = (struct therm_threshold *)data;

	if (thermal_monitor_task) {
		thresh_data->trip_triggered = type;
		thresh_data->parent->thresh_triggered = true;
		complete(&thermal_monitor_complete);
	} else {
		pr_err("Thermal monitor task is not initialized\n");
	}
	return 0;
}

static int init_threshold(enum msm_thresh_list index,
	int sensor_id, int32_t hi_temp, int32_t low_temp,
	void (*callback)(struct therm_threshold *))
{
	int ret = 0, i;
	struct therm_threshold *thresh_ptr;

	if (!callback || index >= MSM_LIST_MAX_NR || index < 0
		|| sensor_id == -ENODEV) {
		pr_err("Invalid input. sensor:%d. index:%d\n",
				sensor_id, index);
		ret = -EINVAL;
		goto init_thresh_exit;
	}
	if (thresh[index].thresh_list) {
		pr_err("threshold id:%d already initialized\n", index);
		ret = -EEXIST;
		goto init_thresh_exit;
	}

	thresh[index].thresh_ct = (sensor_id == MONITOR_ALL_TSENS) ?
						max_tsens_num : 1;
	thresh[index].thresh_triggered = false;
	thresh[index].thresh_list = kzalloc(sizeof(struct therm_threshold) *
					thresh[index].thresh_ct, GFP_KERNEL);
	if (!thresh[index].thresh_list) {
		pr_err("kzalloc failed for thresh index:%d\n", index);
		ret = -ENOMEM;
		goto init_thresh_exit;
	}

	thresh_ptr = thresh[index].thresh_list;
	if (sensor_id == MONITOR_ALL_TSENS) {
		for (i = 0; i < max_tsens_num; i++) {
			thresh_ptr[i].sensor_id = tsens_id_map[i];
			thresh_ptr[i].notify = callback;
			thresh_ptr[i].trip_triggered = -1;
			thresh_ptr[i].parent = &thresh[index];
			thresh_ptr[i].threshold[0].temp = hi_temp;
			thresh_ptr[i].threshold[0].trip =
				THERMAL_TRIP_CONFIGURABLE_HI;
			thresh_ptr[i].threshold[1].temp = low_temp;
			thresh_ptr[i].threshold[1].trip =
				THERMAL_TRIP_CONFIGURABLE_LOW;
			thresh_ptr[i].threshold[0].notify =
			thresh_ptr[i].threshold[1].notify = msm_thermal_notify;
			thresh_ptr[i].threshold[0].data =
			thresh_ptr[i].threshold[1].data =
				(void *)&thresh_ptr[i];
		}
	} else {
		thresh_ptr->sensor_id = sensor_id;
		thresh_ptr->notify = callback;
		thresh_ptr->trip_triggered = -1;
		thresh_ptr->parent = &thresh[index];
		thresh_ptr->threshold[0].temp = hi_temp;
		thresh_ptr->threshold[0].trip =
			THERMAL_TRIP_CONFIGURABLE_HI;
		thresh_ptr->threshold[1].temp = low_temp;
		thresh_ptr->threshold[1].trip =
			THERMAL_TRIP_CONFIGURABLE_LOW;
		thresh_ptr->threshold[0].notify =
		thresh_ptr->threshold[1].notify = msm_thermal_notify;
		thresh_ptr->threshold[0].data =
		thresh_ptr->threshold[1].data = (void *)thresh_ptr;
	}

init_thresh_exit:
	return ret;
}

static void __ref disable_msm_thermal(void)
{
	uint32_t cpu = 0;

	
	cancel_delayed_work_sync(&check_temp_work);

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		if (cpus[cpu].limited_max_freq == UINT_MAX &&
			cpus[cpu].limited_min_freq == 0)
			continue;
		pr_info("Max frequency reset for CPU%d\n", cpu);
		cpus[cpu].limited_max_freq = UINT_MAX;
		cpus[cpu].limited_min_freq = 0;
		update_cpu_freq(cpu);
	}
	put_online_cpus();
}

static void interrupt_mode_init(void)
{
	if (!msm_thermal_probed) {
		interrupt_mode_enable = true;
		return;
	}
	if (polling_enabled) {
		pr_info("Interrupt mode init\n");
		polling_enabled = 0;
		disable_msm_thermal();
		hotplug_init();
		freq_mitigation_init();
		thermal_monitor_init();
	}
}

static int __ref set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	if (!enabled)
		interrupt_mode_init();
	else
		pr_info("no action for enabled = %d\n",
			enabled);

	pr_info("enabled = %d\n", enabled);

	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};

module_param_cb(enabled, &module_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "enforce thermal limit on cpu");

static ssize_t show_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", core_control_enabled);
}

static ssize_t __ref store_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s. err:%d\n", buf, ret);
		goto done_store_cc;
	}

	if (core_control_enabled == !!val)
		goto done_store_cc;

	core_control_enabled = !!val;
	if (core_control_enabled) {
		pr_info("Core control enabled\n");
		register_cpu_notifier(&msm_thermal_cpu_notifier);
		if (hotplug_task)
			complete(&hotplug_notify_complete);
		else
			pr_err("Hotplug task is not initialized\n");
	} else {
		pr_info("Core control disabled\n");
		unregister_cpu_notifier(&msm_thermal_cpu_notifier);
	}

done_store_cc:
	return count;
}

static ssize_t show_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpus_offlined);
}

static ssize_t __ref store_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	uint32_t val = 0;
	uint32_t cpu;

	mutex_lock(&core_control_mutex);
	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s. err:%d\n", buf, ret);
		goto done_cc;
	}

	if (polling_enabled) {
		pr_err("Ignoring request; polling thread is enabled.\n");
		goto done_cc;
	}

	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.core_control_mask & BIT(cpu)))
			continue;
		cpus[cpu].user_offline = !!(val & BIT(cpu));
		pr_debug("\"%s\"(PID:%i) requests %s CPU%d.\n", current->comm,
			current->pid, (cpus[cpu].user_offline) ? "offline" :
			"online", cpu);
	}

	if (hotplug_task)
		complete(&hotplug_notify_complete);
	else
		pr_err("Hotplug task is not initialized\n");
done_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static __refdata struct kobj_attribute cc_enabled_attr =
__ATTR(enabled, 0644, show_cc_enabled, store_cc_enabled);

static __refdata struct kobj_attribute cpus_offlined_attr =
__ATTR(cpus_offlined, 0644, show_cpus_offlined, store_cpus_offlined);

static __refdata struct attribute *cc_attrs[] = {
	&cc_enabled_attr.attr,
	&cpus_offlined_attr.attr,
	NULL,
};

static __refdata struct attribute_group cc_attr_group = {
	.attrs = cc_attrs,
};

static ssize_t show_wakeup_ms(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", wakeup_ms);
}

static ssize_t store_wakeup_ms(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	ret = kstrtouint(buf, 10, &wakeup_ms);

	if (ret) {
		pr_err("%s: Trying to set invalid wakeup timer\n",
				KBUILD_MODNAME);
		return ret;
	}

	if (wakeup_ms > 0) {
		thermal_rtc_setup();
		pr_debug("%s: Timer started for %ums\n", KBUILD_MODNAME,
				wakeup_ms);
	} else {
		ret = alarm_cancel(&thermal_rtc);
		if (ret)
			pr_debug("%s: Timer canceled\n", KBUILD_MODNAME);
		else
			pr_debug("%s: No active timer present to cancel\n",
					KBUILD_MODNAME);

	}
	return count;
}

static __refdata struct kobj_attribute timer_attr =
__ATTR(wakeup_ms, 0644, show_wakeup_ms, store_wakeup_ms);

static __refdata struct attribute *tt_attrs[] = {
	&timer_attr.attr,
	NULL,
};

static __refdata struct attribute_group tt_attr_group = {
	.attrs = tt_attrs,
};

static __init int msm_thermal_add_cc_nodes(void)
{
	struct kobject *module_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		ret = -ENOENT;
		goto done_cc_nodes;
	}

	cc_kobj = kobject_create_and_add("core_control", module_kobj);
	if (!cc_kobj) {
		pr_err("cannot create core control kobj\n");
		ret = -ENOMEM;
		goto done_cc_nodes;
	}

	ret = sysfs_create_group(cc_kobj, &cc_attr_group);
	if (ret) {
		pr_err("cannot create sysfs group. err:%d\n", ret);
		goto done_cc_nodes;
	}

	return 0;

done_cc_nodes:
	if (cc_kobj)
		kobject_del(cc_kobj);
	return ret;
}

static __init int msm_thermal_add_timer_nodes(void)
{
	struct kobject *module_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module\n",
			KBUILD_MODNAME);
		ret = -ENOENT;
		goto failed;
	}

	tt_kobj = kobject_create_and_add("thermal_timer", module_kobj);
	if (!tt_kobj) {
		pr_err("%s: cannot create timer kobj\n",
				KBUILD_MODNAME);
		ret = -ENOMEM;
		goto failed;
	}

	ret = sysfs_create_group(tt_kobj, &tt_attr_group);
	if (ret) {
		pr_err("%s: cannot create group\n", KBUILD_MODNAME);
		goto failed;
	}

	return 0;

failed:
	if (tt_kobj)
		kobject_del(tt_kobj);
	return ret;
}

int msm_thermal_pre_init(void)
{
	int ret = 0;

	tsens_get_max_sensor_num(&max_tsens_num);
	if (create_sensor_id_map()) {
		pr_err("Creating sensor id map failed\n");
		ret = -EINVAL;
		goto pre_init_exit;
	}

	if (!thresh) {
		thresh = kzalloc(
				sizeof(struct threshold_info) * MSM_LIST_MAX_NR,
				GFP_KERNEL);
		if (!thresh) {
			pr_err("kzalloc failed\n");
			ret = -ENOMEM;
			goto pre_init_exit;
		}
		memset(thresh, 0, sizeof(struct threshold_info) *
			MSM_LIST_MAX_NR);
	}
pre_init_exit:
	return ret;
}

int msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;
	uint32_t cpu;

	for_each_possible_cpu(cpu) {
		cpus[cpu].cpu = cpu;
		cpus[cpu].offline = 0;
		cpus[cpu].user_offline = 0;
		cpus[cpu].hotplug_thresh_clear = false;
		cpus[cpu].max_freq = false;
		cpus[cpu].user_max_freq = UINT_MAX;
		cpus[cpu].user_min_freq = 0;
		cpus[cpu].limited_max_freq = UINT_MAX;
		cpus[cpu].limited_min_freq = 0;
		cpus[cpu].freq_thresh_clear = false;
	}
	BUG_ON(!pdata);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	if (check_sensor_id(msm_thermal_info.sensor_id)) {
		pr_err("Invalid sensor:%d for polling\n",
				msm_thermal_info.sensor_id);
		return -EINVAL;
	}

	enabled = 1;
	polling_enabled = 1;
	ret = cpufreq_register_notifier(&msm_thermal_cpufreq_notifier,
			CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_err("cannot register cpufreq notifier. err:%d\n", ret);

	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	schedule_delayed_work(&check_temp_work, 0);

	if (num_possible_cpus() > 1)
		register_cpu_notifier(&msm_thermal_cpu_notifier);

	return ret;
}

static int ocr_reg_init(struct platform_device *pdev)
{
	int ret = 0;
	int i, j;

	for (i = 0; i < ocr_rail_cnt; i++) {
		for (j = 0; j < rails_cnt; j++) {
			if (!strcmp(ocr_rails[i].name, rails[j].name)) {
				if (rails[j].reg == NULL)
					break;
				ocr_rails[i].phase_reg = rails[j].reg;
				goto reg_init;
			}

		}
		ocr_rails[i].phase_reg = devm_regulator_get(&pdev->dev,
					ocr_rails[i].name);
		if (IS_ERR_OR_NULL(ocr_rails[i].phase_reg)) {
			ret = PTR_ERR(ocr_rails[i].phase_reg);
			if (ret != -EPROBE_DEFER) {
				pr_err("%s, could not get regulator: %s\n",
					__func__, ocr_rails[i].name);
				ocr_rails[i].phase_reg = NULL;
				ocr_rails[i].mode = 0;
				ocr_rails[i].init = 0;
			}
			return ret;
		}
reg_init:
		ocr_rails[i].mode = OPTIMUM_CURRENT_MIN;
	}
	return ret;
}

static int vdd_restriction_reg_init(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	for (i = 0; i < rails_cnt; i++) {
		if (rails[i].freq_req == 1) {
			usefreq |= BIT(i);
			check_freq_table();
			if (freq_table_get)
				ret = vdd_restriction_apply_freq(&rails[i], 0);
			else
				pr_info("Defer vdd rstr freq init.\n");
		} else {
			rails[i].reg = devm_regulator_get(&pdev->dev,
					rails[i].name);
			if (IS_ERR_OR_NULL(rails[i].reg)) {
				ret = PTR_ERR(rails[i].reg);
				if (ret != -EPROBE_DEFER) {
					pr_err( \
					"could not get regulator: %s. err:%d\n",
					rails[i].name, ret);
					rails[i].reg = NULL;
					rails[i].curr_level = -2;
					return ret;
				}
				pr_info("Defer regulator %s probe\n",
					rails[i].name);
				return ret;
			}
			ret = vdd_restriction_apply_voltage(&rails[i], 0);
		}
	}

	return ret;
}

static int psm_reg_init(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	int j = 0;

	for (i = 0; i < psm_rails_cnt; i++) {
		psm_rails[i].reg = rpm_regulator_get(&pdev->dev,
				psm_rails[i].name);
		if (IS_ERR_OR_NULL(psm_rails[i].reg)) {
			ret = PTR_ERR(psm_rails[i].reg);
			if (ret != -EPROBE_DEFER) {
				pr_err("couldn't get rpm regulator %s. err%d\n",
					psm_rails[i].name, ret);
				psm_rails[i].reg = NULL;
				goto psm_reg_exit;
			}
			pr_info("Defer regulator %s probe\n",
					psm_rails[i].name);
			return ret;
		}
		
		psm_rails[i].init = PMIC_PWM_MODE;
		ret = rpm_regulator_set_mode(psm_rails[i].reg,
				psm_rails[i].init);
		if (ret) {
			pr_err("Cannot set PMIC PWM mode. err:%d\n", ret);
			return ret;
		} else
			psm_rails[i].mode = PMIC_PWM_MODE;
	}

	return ret;

psm_reg_exit:
	if (ret) {
		for (j = 0; j < i; j++) {
			if (psm_rails[j].reg != NULL)
				rpm_regulator_put(psm_rails[j].reg);
		}
	}

	return ret;
}

static struct kobj_attribute default_cpu_temp_limit_attr =
		__ATTR_RO(default_cpu_temp_limit);

static int msm_thermal_add_default_temp_limit_nodes(void)
{
	struct kobject *module_kobj = NULL;
	int ret = 0;

	if (!default_temp_limit_probed) {
		default_temp_limit_nodes_called = true;
		return ret;
	}
	if (!default_temp_limit_enabled)
		return ret;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		return -ENOENT;
	}

	sysfs_attr_init(&default_cpu_temp_limit_attr.attr);
	ret = sysfs_create_file(module_kobj, &default_cpu_temp_limit_attr.attr);
	if (ret) {
		pr_err(
		"cannot create default_cpu_temp_limit attribute. err:%d\n",
		ret);
		return ret;
	}
	return ret;
}

static int msm_thermal_add_vdd_rstr_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *vdd_rstr_kobj = NULL;
	struct kobject *vdd_rstr_reg_kobj[MAX_RAILS] = {0};
	int rc = 0;
	int i = 0;

	if (!vdd_rstr_probed) {
		vdd_rstr_nodes_called = true;
		return rc;
	}

	if (vdd_rstr_probed && rails_cnt == 0)
		return rc;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		rc = -ENOENT;
		goto thermal_sysfs_add_exit;
	}

	vdd_rstr_kobj = kobject_create_and_add("vdd_restriction", module_kobj);
	if (!vdd_rstr_kobj) {
		pr_err("cannot create vdd_restriction kobject\n");
		rc = -ENOMEM;
		goto thermal_sysfs_add_exit;
	}

	rc = sysfs_create_group(vdd_rstr_kobj, &vdd_rstr_en_attribs_gp);
	if (rc) {
		pr_err("cannot create kobject attribute group. err:%d\n", rc);
		rc = -ENOMEM;
		goto thermal_sysfs_add_exit;
	}

	for (i = 0; i < rails_cnt; i++) {
		vdd_rstr_reg_kobj[i] = kobject_create_and_add(rails[i].name,
					vdd_rstr_kobj);
		if (!vdd_rstr_reg_kobj[i]) {
			pr_err("cannot create kobject for %s\n",
					rails[i].name);
			rc = -ENOMEM;
			goto thermal_sysfs_add_exit;
		}

		rails[i].attr_gp.attrs = kzalloc(sizeof(struct attribute *) * 3,
					GFP_KERNEL);
		if (!rails[i].attr_gp.attrs) {
			pr_err("kzalloc failed\n");
			rc = -ENOMEM;
			goto thermal_sysfs_add_exit;
		}

		VDD_RES_RW_ATTRIB(rails[i], rails[i].level_attr, 0, level);
		VDD_RES_RO_ATTRIB(rails[i], rails[i].value_attr, 1, value);
		rails[i].attr_gp.attrs[2] = NULL;

		rc = sysfs_create_group(vdd_rstr_reg_kobj[i],
				&rails[i].attr_gp);
		if (rc) {
			pr_err("cannot create attribute group for %s. err:%d\n",
					rails[i].name, rc);
			goto thermal_sysfs_add_exit;
		}
	}

	return rc;

thermal_sysfs_add_exit:
	if (rc) {
		for (i = 0; i < rails_cnt; i++) {
			kobject_del(vdd_rstr_reg_kobj[i]);
			kfree(rails[i].attr_gp.attrs);
		}
		if (vdd_rstr_kobj)
			kobject_del(vdd_rstr_kobj);
	}
	return rc;
}

static int msm_thermal_add_ocr_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *ocr_kobj = NULL;
	struct kobject *ocr_reg_kobj[MAX_RAILS] = {0};
	int rc = 0;
	int i = 0;

	if (!ocr_probed) {
		ocr_nodes_called = true;
		return rc;
	}

	if (ocr_probed && ocr_rail_cnt == 0)
		return rc;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		rc = -ENOENT;
		goto ocr_node_exit;
	}

	ocr_kobj = kobject_create_and_add("opt_curr_req", module_kobj);
	if (!ocr_kobj) {
		pr_err("%s: cannot create ocr kobject\n", KBUILD_MODNAME);
		rc = -ENOMEM;
		goto ocr_node_exit;
	}

	for (i = 0; i < ocr_rail_cnt; i++) {
		ocr_reg_kobj[i] = kobject_create_and_add(ocr_rails[i].name,
					ocr_kobj);
		if (!ocr_reg_kobj[i]) {
			pr_err("%s: cannot create for kobject for %s\n",
					KBUILD_MODNAME, ocr_rails[i].name);
			rc = -ENOMEM;
			goto ocr_node_exit;
		}
		ocr_rails[i].attr_gp.attrs = kzalloc( \
				sizeof(struct attribute *) * 2, GFP_KERNEL);
		if (!ocr_rails[i].attr_gp.attrs) {
			rc = -ENOMEM;
			goto ocr_node_exit;
		}

		OCR_RW_ATTRIB(ocr_rails[i], ocr_rails[i].mode_attr, 0, mode);
		ocr_rails[i].attr_gp.attrs[1] = NULL;

		rc = sysfs_create_group(ocr_reg_kobj[i], &ocr_rails[i].attr_gp);
		if (rc) {
			pr_err("%s: cannot create attribute group for %s\n",
				KBUILD_MODNAME, ocr_rails[i].name);
			goto ocr_node_exit;
		}
	}

ocr_node_exit:
	if (rc) {
		for (i = 0; i < ocr_rail_cnt; i++) {
			if (ocr_reg_kobj[i])
				kobject_del(ocr_reg_kobj[i]);
			if (ocr_rails[i].attr_gp.attrs) {
				kfree(ocr_rails[i].attr_gp.attrs);
				ocr_rails[i].attr_gp.attrs = NULL;
			}
		}
		if (ocr_kobj)
			kobject_del(ocr_kobj);
	}
	return rc;
}

static int msm_thermal_add_psm_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *psm_kobj = NULL;
	struct kobject *psm_reg_kobj[MAX_RAILS] = {0};
	int rc = 0;
	int i = 0;

	if (!psm_probed) {
		psm_nodes_called = true;
		return rc;
	}

	if (psm_probed && psm_rails_cnt == 0)
		return rc;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		rc = -ENOENT;
		goto psm_node_exit;
	}

	psm_kobj = kobject_create_and_add("pmic_sw_mode", module_kobj);
	if (!psm_kobj) {
		pr_err("cannot create psm kobject\n");
		rc = -ENOMEM;
		goto psm_node_exit;
	}

	for (i = 0; i < psm_rails_cnt; i++) {
		psm_reg_kobj[i] = kobject_create_and_add(psm_rails[i].name,
					psm_kobj);
		if (!psm_reg_kobj[i]) {
			pr_err("cannot create kobject for %s\n",
					psm_rails[i].name);
			rc = -ENOMEM;
			goto psm_node_exit;
		}
		psm_rails[i].attr_gp.attrs = kzalloc( \
				sizeof(struct attribute *) * 2, GFP_KERNEL);
		if (!psm_rails[i].attr_gp.attrs) {
			pr_err("kzalloc failed\n");
			rc = -ENOMEM;
			goto psm_node_exit;
		}

		PSM_RW_ATTRIB(psm_rails[i], psm_rails[i].mode_attr, 0, mode);
		psm_rails[i].attr_gp.attrs[1] = NULL;

		rc = sysfs_create_group(psm_reg_kobj[i], &psm_rails[i].attr_gp);
		if (rc) {
			pr_err("cannot create attribute group for %s. err:%d\n",
					psm_rails[i].name, rc);
			goto psm_node_exit;
		}
	}

	return rc;

psm_node_exit:
	if (rc) {
		for (i = 0; i < psm_rails_cnt; i++) {
			kobject_del(psm_reg_kobj[i]);
			kfree(psm_rails[i].attr_gp.attrs);
		}
		if (psm_kobj)
			kobject_del(psm_kobj);
	}
	return rc;
}

static int probe_vdd_rstr(struct device_node *node,
		struct msm_thermal_data *data, struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	int arr_size;
	char *key = NULL;
	struct device_node *child_node = NULL;

	rails = NULL;

	key = "qcom,vdd-restriction-temp";
	ret = of_property_read_u32(node, key, &data->vdd_rstr_temp_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,vdd-restriction-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->vdd_rstr_temp_hyst_degC);
	if (ret)
		goto read_node_fail;

	for_each_child_of_node(node, child_node) {
		rails_cnt++;
	}

	if (rails_cnt == 0)
		goto read_node_fail;
	if (rails_cnt >= MAX_RAILS) {
		pr_err("Too many rails:%d.\n", rails_cnt);
		return -EFAULT;
	}

	rails = kzalloc(sizeof(struct rail) * rails_cnt,
				GFP_KERNEL);
	if (!rails) {
		pr_err("Fail to allocate memory for rails.\n");
		return -ENOMEM;
	}

	i = 0;
	for_each_child_of_node(node, child_node) {
		key = "qcom,vdd-rstr-reg";
		ret = of_property_read_string(child_node, key, &rails[i].name);
		if (ret)
			goto read_node_fail;

		key = "qcom,levels";
		if (!of_get_property(child_node, key, &arr_size))
			goto read_node_fail;
		rails[i].num_levels = arr_size/sizeof(__be32);
		if (rails[i].num_levels >
			sizeof(rails[i].levels)/sizeof(uint32_t)) {
			pr_err("Array size:%d too large for index:%d\n",
				rails[i].num_levels, i);
			return -EFAULT;
		}
		ret = of_property_read_u32_array(child_node, key,
				rails[i].levels, rails[i].num_levels);
		if (ret)
			goto read_node_fail;

		key = "qcom,freq-req";
		rails[i].freq_req = of_property_read_bool(child_node, key);
		if (rails[i].freq_req)
			rails[i].min_level = 0;
		else {
			key = "qcom,min-level";
			ret = of_property_read_u32(child_node, key,
				&rails[i].min_level);
			if (ret)
				goto read_node_fail;
		}

		rails[i].curr_level = -1;
		rails[i].reg = NULL;
		i++;
	}

	if (rails_cnt) {
		ret = vdd_restriction_reg_init(pdev);
		if (ret) {
			pr_err("Err regulator init. err:%d. KTM continues.\n",
					ret);
			goto read_node_fail;
		}
		ret = init_threshold(MSM_VDD_RESTRICTION, MONITOR_ALL_TSENS,
			data->vdd_rstr_temp_hyst_degC, data->vdd_rstr_temp_degC,
			vdd_restriction_notify);
		if (ret) {
			pr_err("Error in initializing thresholds. err:%d\n",
					ret);
			goto read_node_fail;
		}
		vdd_rstr_enabled = true;
	}
read_node_fail:
	vdd_rstr_probed = true;
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		kfree(rails);
		rails_cnt = 0;
	}
	if (ret == -EPROBE_DEFER)
		vdd_rstr_probed = false;
	return ret;
}

static int get_efuse_temp_map(struct device_node *node,
				int *efuse_values,
				int *efuse_temp)
{
	uint32_t i, j, efuse_arr_cnt = 0;
	int ret = 0, efuse_map_cnt = 0;
	uint32_t data[2 * MAX_EFUSE_VALUE];

	char *key = "qcom,efuse-temperature-map";
	if (!of_get_property(node, key, &efuse_map_cnt)
		|| efuse_map_cnt <= 0) {
		pr_debug("Property %s not defined.\n", key);
		return -ENODEV;
	}

	if (efuse_map_cnt % (sizeof(__be32) * 2)) {
		pr_err("Invalid number(%d) of entry for %s\n",
				efuse_map_cnt, key);
		return -EINVAL;
	}

	efuse_arr_cnt = efuse_map_cnt / sizeof(__be32);

	ret = of_property_read_u32_array(node, key, data, efuse_arr_cnt);
	if (ret)
		return -EINVAL;

	efuse_map_cnt /= (sizeof(__be32) * 2);

	j = 0;
	for (i = 0; i < efuse_map_cnt; i++) {
		efuse_values[i] = data[j++];
		efuse_temp[i] = data[j++];
	}

	return efuse_map_cnt;
}

static int probe_thermal_efuse_read(struct device_node *node,
			struct msm_thermal_data *data,
			struct platform_device *pdev)
{
	u64 efuse_bits;
	int ret = 0;
	int i = 0;
	int efuse_map_cnt = 0;
	int efuse_data_cnt = 0;
	char *key = NULL;
	void __iomem *efuse_base = NULL;
	uint32_t efuse_data[EFUSE_DATA_MAX] = {0};
	uint32_t efuse_values[MAX_EFUSE_VALUE] = {0};
	uint32_t efuse_temp[MAX_EFUSE_VALUE] = {0};
	uint32_t default_temp = 0;
	uint8_t thermal_efuse_data = 0;

	if (default_temp_limit_probed)
		goto read_efuse_exit;

	key = "qcom,default-temp";
	if (of_property_read_u32(node, key, &default_temp))
		default_temp = 0;

	default_cpu_temp_limit = default_temp;

	key = "qcom,efuse-data";
	if (!of_get_property(node, key, &efuse_data_cnt) ||
		efuse_data_cnt <= 0) {
		ret = -ENODEV;
		goto read_efuse_fail;
	}
	efuse_data_cnt /= sizeof(__be32);

	if (efuse_data_cnt != EFUSE_DATA_MAX) {
		pr_err("Invalid number of efuse data. data cnt %d\n",
			efuse_data_cnt);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	ret = of_property_read_u32_array(node, key, efuse_data,
						efuse_data_cnt);
	if (ret)
		goto read_efuse_fail;

	if (efuse_data[EFUSE_ADDRESS] == 0 ||
		efuse_data[EFUSE_SIZE] == 0 ||
		efuse_data[EFUSE_BIT_MASK] == 0) {
		pr_err("Invalid efuse data: address:%x len:%d bitmask%x\n",
			efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_SIZE],
			efuse_data[EFUSE_BIT_MASK]);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_map_cnt = get_efuse_temp_map(node, efuse_values,
						efuse_temp);
	if (efuse_map_cnt <= 0 ||
		efuse_map_cnt > (efuse_data[EFUSE_BIT_MASK] + 1)) {
		pr_err("Invalid efuse-temperature-map. cnt%d\n",
			efuse_map_cnt);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_base = ioremap(efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_SIZE]);
	if (!efuse_base) {
		pr_err("Unable to map efuse_addr:%x with size%d\n",
			efuse_data[EFUSE_ADDRESS],
			efuse_data[EFUSE_SIZE]);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_bits = readll_relaxed(efuse_base
		+ efuse_data[EFUSE_ROW] * BYTES_PER_FUSE_ROW);

	thermal_efuse_data = (efuse_bits >> efuse_data[EFUSE_START_BIT]) &
						efuse_data[EFUSE_BIT_MASK];

	
	for (; i < efuse_map_cnt; i++) {
		if (efuse_values[i] == thermal_efuse_data) {
			default_cpu_temp_limit = efuse_temp[i];
			break;
		}
	}
	if (i >= efuse_map_cnt) {
		if (!default_temp) {
			pr_err("No matching efuse value. value:%d\n",
				thermal_efuse_data);
			ret = -EINVAL;
			goto read_efuse_fail;
		}
	}

	pr_debug(
	"Efuse address:0x%x [row:%d] = 0x%llx @%d:mask:0x%x = 0x%x temp:%d\n",
		efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_ROW], efuse_bits,
		efuse_data[EFUSE_START_BIT], efuse_data[EFUSE_BIT_MASK],
		thermal_efuse_data, default_cpu_temp_limit);

	default_temp_limit_enabled = true;

read_efuse_fail:
	if (efuse_base)
		iounmap(efuse_base);
	default_temp_limit_probed = true;
	if (ret) {
		if (!default_temp) {
			dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
				__func__, node->full_name, key);
		} else {
			default_temp_limit_enabled = true;
			pr_debug("Default cpu temp limit is %d\n",
					default_cpu_temp_limit);
			ret = 0;
		}
	}
read_efuse_exit:
	return ret;
}

static int probe_ocr(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	int ret = 0;
	int j = 0;
	char *key = NULL;

	if (ocr_probed) {
		pr_info("%s: Nodes already probed\n",
			__func__);
		goto read_ocr_exit;
	}
	ocr_rails = NULL;

	key = "qti,pmic-opt-curr-temp";
	ret = of_property_read_u32(node, key, &data->ocr_temp_degC);
	if (ret)
		goto read_ocr_fail;

	key = "qti,pmic-opt-curr-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->ocr_temp_hyst_degC);
	if (ret)
		goto read_ocr_fail;

	key = "qti,pmic-opt-curr-regs";
	ocr_rail_cnt = of_property_count_strings(node, key);
	ocr_rails = kzalloc(sizeof(struct psm_rail) * ocr_rail_cnt,
			GFP_KERNEL);
	if (!ocr_rails) {
		pr_err("%s: Fail to allocate memory for ocr rails\n", __func__);
		ocr_rail_cnt = 0;
		return -ENOMEM;
	}

	for (j = 0; j < ocr_rail_cnt; j++) {
		ret = of_property_read_string_index(node, key, j,
				&ocr_rails[j].name);
		if (ret)
			goto read_ocr_fail;
		ocr_rails[j].phase_reg = NULL;
		ocr_rails[j].init = OPTIMUM_CURRENT_MAX;
	}

	if (ocr_rail_cnt) {
		ret = ocr_reg_init(pdev);
		if (ret) {
			pr_info("%s:Failed to get regulators. KTM continues.\n",
					__func__);
			goto read_ocr_fail;
		}
		ocr_enabled = true;
		ocr_nodes_called = false;
		if (ocr_set_mode_all(OPTIMUM_CURRENT_MAX))
			pr_err("Set max optimum current failed\n");
	}

read_ocr_fail:
	ocr_probed = true;
	if (ret) {
		dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
			__func__, node->full_name, key);
		if (ocr_rails)
			kfree(ocr_rails);
		ocr_rails = NULL;
		ocr_rail_cnt = 0;
	}
	if (ret == -EPROBE_DEFER)
		ocr_probed = false;
read_ocr_exit:
	return ret;
}

static int probe_psm(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	int ret = 0;
	int j = 0;
	char *key = NULL;

	psm_rails = NULL;

	key = "qcom,pmic-sw-mode-temp";
	ret = of_property_read_u32(node, key, &data->psm_temp_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,pmic-sw-mode-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->psm_temp_hyst_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,pmic-sw-mode-regs";
	psm_rails_cnt = of_property_count_strings(node, key);
	psm_rails = kzalloc(sizeof(struct psm_rail) * psm_rails_cnt,
			GFP_KERNEL);
	if (!psm_rails) {
		pr_err("Fail to allocate memory for psm rails\n");
		psm_rails_cnt = 0;
		return -ENOMEM;
	}

	for (j = 0; j < psm_rails_cnt; j++) {
		ret = of_property_read_string_index(node, key, j,
				&psm_rails[j].name);
		if (ret)
			goto read_node_fail;
	}

	if (psm_rails_cnt) {
		ret = psm_reg_init(pdev);
		if (ret) {
			pr_err("Err regulator init. err:%d. KTM continues.\n",
					ret);
			goto read_node_fail;
		}
		psm_enabled = true;
	}

read_node_fail:
	psm_probed = true;
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		kfree(psm_rails);
		psm_rails_cnt = 0;
	}
	if (ret == -EPROBE_DEFER)
		psm_probed = false;
	return ret;
}

static int probe_cc(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	uint32_t cpu_cnt = 0;
	int ret = 0;
	uint32_t cpu = 0;

	if (num_possible_cpus() > 1) {
		core_control_enabled = 1;
		hotplug_enabled = 1;
	}

	key = "qcom,core-limit-temp";
	ret = of_property_read_u32(node, key, &data->core_limit_temp_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,core-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->core_temp_hysteresis_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,core-control-mask";
	ret = of_property_read_u32(node, key, &data->core_control_mask);
	if (ret)
		goto read_node_fail;

	key = "qcom,hotplug-temp";
	ret = of_property_read_u32(node, key, &data->hotplug_temp_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,hotplug-temp-hysteresis";
	ret = of_property_read_u32(node, key,
			&data->hotplug_temp_hysteresis_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,cpu-sensors";
	cpu_cnt = of_property_count_strings(node, key);
	if (cpu_cnt < num_possible_cpus()) {
		pr_err("Wrong number of cpu sensors:%d\n", cpu_cnt);
		ret = -EINVAL;
		goto hotplug_node_fail;
	}

	for_each_possible_cpu(cpu) {
		ret = of_property_read_string_index(node, key, cpu,
				&cpus[cpu].sensor_type);
		if (ret)
			goto hotplug_node_fail;
	}

read_node_fail:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			KBUILD_MODNAME, node->full_name, key, ret);
		core_control_enabled = 0;
	}

	return ret;

hotplug_node_fail:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			KBUILD_MODNAME, node->full_name, key, ret);
		hotplug_enabled = 0;
	}

	return ret;
}

static int probe_therm_reset(struct device_node *node,
		struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	int ret = 0;

	key = "qcom,therm-reset-temp";
	ret = of_property_read_u32(node, key, &data->therm_reset_temp_degC);
	if (ret)
		goto PROBE_RESET_EXIT;

	ret = init_threshold(MSM_THERM_RESET, MONITOR_ALL_TSENS,
		data->therm_reset_temp_degC, data->therm_reset_temp_degC - 10,
		therm_reset_notify);
	if (ret) {
		pr_err("Therm reset data structure init failed\n");
		goto PROBE_RESET_EXIT;
	}

	therm_reset_enabled = true;

PROBE_RESET_EXIT:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		therm_reset_enabled = false;
	}
	return ret;
}

static int probe_freq_mitigation(struct device_node *node,
		struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	int ret = 0;

	key = "qcom,freq-mitigation-temp";
	ret = of_property_read_u32(node, key, &data->freq_mitig_temp_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-temp-hysteresis";
	ret = of_property_read_u32(node, key,
		&data->freq_mitig_temp_hysteresis_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-value";
	ret = of_property_read_u32(node, key, &data->freq_limit);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-control-mask";
	ret = of_property_read_u32(node, key, &data->freq_mitig_control_mask);
	if (ret)
		goto PROBE_FREQ_EXIT;

	freq_mitigation_enabled = 1;

PROBE_FREQ_EXIT:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		freq_mitigation_enabled = 0;
	}
	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	memset(&data, 0, sizeof(struct msm_thermal_data));
	ret = msm_thermal_pre_init();
	if (ret) {
		pr_err("thermal pre init failed. err:%d\n", ret);
		goto fail;
	}

	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;

	key = "qcom,poll-ms";
	ret = of_property_read_u32(node, key, &data.poll_ms);
	if (ret)
		goto fail;

	key = "qcom,limit-temp";
	ret = of_property_read_u32(node, key, &data.limit_temp_degC);
	if (ret)
		goto fail;

	key = "qcom,temp-hysteresis";
	ret = of_property_read_u32(node, key, &data.temp_hysteresis_degC);
	if (ret)
		goto fail;

	key = "qcom,freq-step";
	ret = of_property_read_u32(node, key, &data.bootup_freq_step);
	if (ret)
		goto fail;

	key = "qcom,freq-control-mask";
	ret = of_property_read_u32(node, key, &data.bootup_freq_control_mask);

#if defined(CONFIG_MACH_EYE_UL)
	if(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) {
		data.poll_ms = 100;
		data.bootup_freq_step = 4;
		printk("[Thermal-Driver]Polling time changed to:%dms, bootup_freq_step changed to:%d \n", data.poll_ms, data.bootup_freq_step);
	}
#endif

	ret = probe_cc(node, &data, pdev);

	ret = probe_freq_mitigation(node, &data, pdev);
	ret = probe_therm_reset(node, &data, pdev);

	ret = probe_psm(node, &data, pdev);
	if (ret == -EPROBE_DEFER)
		goto fail;
	ret = probe_vdd_rstr(node, &data, pdev);
	if (ret == -EPROBE_DEFER)
		goto fail;
	ret = probe_ocr(node, &data, pdev);
	if (ret == -EPROBE_DEFER)
		goto fail;
	probe_thermal_efuse_read(node, &data, pdev);

	if (psm_nodes_called) {
		msm_thermal_add_psm_nodes();
		psm_nodes_called = false;
	}
	if (vdd_rstr_nodes_called) {
		msm_thermal_add_vdd_rstr_nodes();
		vdd_rstr_nodes_called = false;
	}
	if (ocr_nodes_called) {
		msm_thermal_add_ocr_nodes();
		ocr_nodes_called = false;
	}
	if (default_temp_limit_nodes_called) {
		msm_thermal_add_default_temp_limit_nodes();
		default_temp_limit_nodes_called = false;
	}
	msm_thermal_ioctl_init();
	ret = msm_thermal_init(&data);
	msm_thermal_probed = true;

	if (interrupt_mode_enable) {
		interrupt_mode_init();
		interrupt_mode_enable = false;
	}

	return ret;
fail:
	if (ret)
		pr_err("Failed reading node=%s, key=%s. err:%d\n",
			node->full_name, key, ret);

	return ret;
}

static int msm_thermal_dev_exit(struct platform_device *inp_dev)
{
	msm_thermal_ioctl_cleanup();
	if (thresh) {
		if (vdd_rstr_enabled)
			kfree(thresh[MSM_VDD_RESTRICTION].thresh_list);
		kfree(thresh);
		thresh = NULL;
	}
	return 0;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
	.remove = msm_thermal_dev_exit,
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

int __init msm_thermal_late_init(void)
{
	if (num_possible_cpus() > 1)
		msm_thermal_add_cc_nodes();
	msm_thermal_add_psm_nodes();
	msm_thermal_add_vdd_rstr_nodes();
	msm_thermal_add_ocr_nodes();
	msm_thermal_add_default_temp_limit_nodes();
	alarm_init(&thermal_rtc, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			thermal_rtc_callback);
	INIT_WORK(&timer_work, timer_work_fn);
	msm_thermal_add_timer_nodes();

	interrupt_mode_init();
	return 0;
}
late_initcall(msm_thermal_late_init);


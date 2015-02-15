/*
 *  thermal.c - Generic Thermal Management Sysfs support.
 *
 *  Copyright (C) 2008 Intel Corp
 *  Copyright (C) 2008 Zhang Rui <rui.zhang@intel.com>
 *  Copyright (C) 2008 Sujith Thomas <sujith.thomas@intel.com>
 *  Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/idr.h>
#include <linux/thermal.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>
#include <net/netlink.h>
#include <net/genetlink.h>

MODULE_AUTHOR("Zhang Rui");
MODULE_DESCRIPTION("Generic thermal management sysfs support");
MODULE_LICENSE("GPL");

#ifdef pr_emerg
#undef pr_emerg
#endif
#define pr_emerg(fmt, args...) \
	printk(KERN_EMERG "[THERMAL] " pr_fmt(fmt), ## args)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, args...) \
	printk(KERN_ERR "[THERMAL] " pr_fmt(fmt), ## args)

#ifdef pr_warning
#undef pr_warning
#endif
#define pr_warning(fmt, args...) \
	printk(KERN_WARNING "[THERMAL] " pr_fmt(fmt), ## args)

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, args...) \
	printk(KERN_INFO "[THERMAL] " pr_fmt(fmt), ## args)

#if defined(DEBUG)
#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) \
	printk(KERN_DEBUG "[THERMAL] " pr_fmt(fmt), ## args)
#endif

struct thermal_cooling_device_instance {
	int id;
	char name[THERMAL_NAME_LENGTH];
	struct thermal_zone_device *tz;
	struct thermal_cooling_device *cdev;
	int trip;
	char attr_name[THERMAL_NAME_LENGTH];
	struct device_attribute attr;
	struct list_head node;
};

static char *truly_shutdown[2] = { "CPU_TEMP=SHUTDOWN_TEMP", NULL };
static char *shutdown_waring[2]    = { "CPU_TEMP=CLOSE_TO_SHUTDOWN_TEMP", NULL };
static char *clr_shutdown_warning[2]   = { "CPU_TEMP=CLOSE_TO_SHUTDOWN_TEMP_CLR", NULL };

static DEFINE_IDR(thermal_tz_idr);
static DEFINE_IDR(thermal_cdev_idr);
static DEFINE_MUTEX(thermal_idr_lock);

static LIST_HEAD(thermal_tz_list);
static LIST_HEAD(thermal_cdev_list);
static DEFINE_MUTEX(thermal_list_lock);

static LIST_HEAD(sensor_info_list);
static DEFINE_MUTEX(sensor_list_lock);

static struct sensor_info *get_sensor(uint32_t sensor_id)
{
	struct sensor_info *pos, *var;

	list_for_each_entry_safe(pos, var, &sensor_info_list, sensor_list) {
		if (pos->sensor_id == sensor_id)
			return pos;
	}

	return NULL;
}

int sensor_get_id(char *name)
{
	struct sensor_info *pos, *var;

	if (!name)
		return -ENODEV;

	list_for_each_entry_safe(pos, var, &sensor_info_list, sensor_list) {
		if (!strcmp(pos->tz->type, name))
			return pos->sensor_id;
	}

	return -ENODEV;
}
EXPORT_SYMBOL(sensor_get_id);

static void init_sensor_trip(struct sensor_info *sensor)
{
	int ret = 0, i = 0;
	enum thermal_trip_type type;

	for (i = 0; ((sensor->max_idx == -1) ||
		(sensor->min_idx == -1)) &&
		(sensor->tz->ops->get_trip_type) &&
		(i < sensor->tz->trips); i++) {

		sensor->tz->ops->get_trip_type(sensor->tz, i, &type);
		if (type == THERMAL_TRIP_CONFIGURABLE_HI)
			sensor->max_idx = i;
		if (type == THERMAL_TRIP_CONFIGURABLE_LOW)
			sensor->min_idx = i;
		type = 0;
	}

	ret = sensor->tz->ops->get_trip_temp(sensor->tz,
		sensor->min_idx, &sensor->threshold_min);
	if (ret)
		pr_err("Unable to get MIN trip temp. sensor:%d err:%d\n",
				sensor->sensor_id, ret);

	ret = sensor->tz->ops->get_trip_temp(sensor->tz,
		sensor->max_idx, &sensor->threshold_max);
	if (ret)
		pr_err("Unable to get MAX trip temp. sensor:%d err:%d\n",
				sensor->sensor_id, ret);
}

static int __update_sensor_thresholds(struct sensor_info *sensor)
{
	long max_of_low_thresh = LONG_MIN;
	long min_of_high_thresh = LONG_MAX;
	struct sensor_threshold *pos, *var;
	int ret = 0;

	if (!sensor->tz->ops->set_trip_temp ||
		!sensor->tz->ops->activate_trip_type ||
		!sensor->tz->ops->get_trip_type ||
		!sensor->tz->ops->get_trip_temp) {
		ret = -ENODEV;
		goto update_done;
	}

	if ((sensor->max_idx == -1) || (sensor->min_idx == -1))
		init_sensor_trip(sensor);

	list_for_each_entry_safe(pos, var, &sensor->threshold_list, list) {
		if (!pos->active)
			continue;
		if (pos->trip == THERMAL_TRIP_CONFIGURABLE_LOW) {
			if (pos->temp > max_of_low_thresh)
				max_of_low_thresh = pos->temp;
		}
		if (pos->trip == THERMAL_TRIP_CONFIGURABLE_HI) {
			if (pos->temp < min_of_high_thresh)
				min_of_high_thresh = pos->temp;
		}
	}

	pr_debug("sensor %d: Thresholds: max of low: %ld min of high: %ld\n",
		sensor->sensor_id, max_of_low_thresh,
		min_of_high_thresh);

	if ((min_of_high_thresh != sensor->threshold_max) &&
		(min_of_high_thresh != LONG_MAX)) {
		ret = sensor->tz->ops->set_trip_temp(sensor->tz,
			sensor->max_idx, min_of_high_thresh);
		if (ret) {
			pr_err("sensor %d: Unable to set high threshold %d",
				sensor->sensor_id, ret);
			goto update_done;
		}
		sensor->threshold_max = min_of_high_thresh;
	}
	ret = sensor->tz->ops->activate_trip_type(sensor->tz,
		sensor->max_idx,
		(min_of_high_thresh == LONG_MAX) ?
		THERMAL_TRIP_ACTIVATION_DISABLED :
		THERMAL_TRIP_ACTIVATION_ENABLED);
	if (ret) {
		pr_err("sensor %d: Unable to activate high threshold %d",
			sensor->sensor_id, ret);
		goto update_done;
	}

	if ((max_of_low_thresh != sensor->threshold_min) &&
		(max_of_low_thresh != LONG_MIN)) {
		ret = sensor->tz->ops->set_trip_temp(sensor->tz,
			sensor->min_idx, max_of_low_thresh);
		if (ret) {
			pr_err("sensor %d: Unable to set low threshold %d",
				sensor->sensor_id, ret);
			goto update_done;
		}
		sensor->threshold_min = max_of_low_thresh;
	}
	ret = sensor->tz->ops->activate_trip_type(sensor->tz,
		sensor->min_idx,
		(max_of_low_thresh == LONG_MIN) ?
		THERMAL_TRIP_ACTIVATION_DISABLED :
		THERMAL_TRIP_ACTIVATION_ENABLED);
	if (ret) {
		pr_err("sensor %d: Unable to activate low threshold %d",
			sensor->sensor_id, ret);
		goto update_done;
	}

	pr_debug("sensor %d: low: %ld high: %ld\n",
		sensor->sensor_id,
		sensor->threshold_min, sensor->threshold_max);

update_done:
	return ret;
}

static void sensor_update_work(struct work_struct *work)
{
	struct sensor_info *sensor = container_of(work, struct sensor_info,
						work);
	int ret = 0;
	mutex_lock(&sensor->lock);
	ret = __update_sensor_thresholds(sensor);
	if (ret)
		pr_err("sensor %d: Error %d setting threshold\n",
			sensor->sensor_id, ret);
	mutex_unlock(&sensor->lock);
}

int thermal_sensor_trip(struct thermal_zone_device *tz,
		enum thermal_trip_type trip, long temp)
{
	struct sensor_threshold *pos, *var;
	int ret = -ENODEV;

	if (trip != THERMAL_TRIP_CONFIGURABLE_HI &&
			trip != THERMAL_TRIP_CONFIGURABLE_LOW)
		return 0;

	if (list_empty(&tz->sensor.threshold_list))
		return 0;

	list_for_each_entry_safe(pos, var, &tz->sensor.threshold_list, list) {
		if ((pos->trip != trip) || (!pos->active))
			continue;
		if (((trip == THERMAL_TRIP_CONFIGURABLE_LOW) &&
			(pos->temp <= tz->sensor.threshold_min) &&
			(pos->temp >= temp)) ||
			((trip == THERMAL_TRIP_CONFIGURABLE_HI) &&
				(pos->temp >= tz->sensor.threshold_max) &&
				(pos->temp <= temp))) {
			pos->active = 0;
			pos->notify(trip, temp, pos->data);
		}
	}

	schedule_work(&tz->sensor.work);

	return ret;
}
EXPORT_SYMBOL(thermal_sensor_trip);

int sensor_activate_trip(uint32_t sensor_id,
	struct sensor_threshold *threshold, bool enable)
{
	struct sensor_info *sensor = get_sensor(sensor_id);
	int ret = 0;

	if (!sensor || !threshold) {
		pr_err("Sensor %d: uninitialized data\n",
			sensor_id);
		ret = -ENODEV;
		goto activate_trip_exit;
	}

	mutex_lock(&sensor->lock);
	threshold->active = (enable) ? 1 : 0;
	ret = __update_sensor_thresholds(sensor);
	mutex_unlock(&sensor->lock);

activate_trip_exit:
	return ret;
}
EXPORT_SYMBOL(sensor_activate_trip);

int sensor_set_trip(uint32_t sensor_id, struct sensor_threshold *threshold)
{
	struct sensor_threshold *pos, *var;
	struct sensor_info *sensor = get_sensor(sensor_id);

	if (!sensor)
		return -ENODEV;

	if (!threshold || !threshold->notify)
		return -EFAULT;

	mutex_lock(&sensor->lock);
	list_for_each_entry_safe(pos, var, &sensor->threshold_list, list) {
		if (pos == threshold)
			break;
	}

	if (pos != threshold) {
		INIT_LIST_HEAD(&threshold->list);
		list_add(&threshold->list, &sensor->threshold_list);
	}
	threshold->active = 0; 
	mutex_unlock(&sensor->lock);

	return 0;

}
EXPORT_SYMBOL(sensor_set_trip);

int sensor_cancel_trip(uint32_t sensor_id, struct sensor_threshold *threshold)
{
	struct sensor_threshold *pos, *var;
	struct sensor_info *sensor = get_sensor(sensor_id);
	int ret = 0;

	if (!sensor)
		return -ENODEV;

	mutex_lock(&sensor->lock);
	list_for_each_entry_safe(pos, var, &sensor->threshold_list, list) {
		if (pos == threshold) {
			pos->active = 0;
			list_del(&pos->list);
			break;
		}
	}

	ret = __update_sensor_thresholds(sensor);
	mutex_unlock(&sensor->lock);

	return ret;
}
EXPORT_SYMBOL(sensor_cancel_trip);

static int tz_notify_trip(enum thermal_trip_type type, int temp, void *data)
{
	struct thermal_zone_device *tz = (struct thermal_zone_device *)data;

	pr_debug("sensor %d tripped: type %d temp %d\n",
			tz->sensor.sensor_id, type, temp);

	return 0;
}

static void get_trip_threshold(struct thermal_zone_device *tz, int trip,
	struct sensor_threshold **threshold)
{
	enum thermal_trip_type type;

	tz->ops->get_trip_type(tz, trip, &type);

	if (type == THERMAL_TRIP_CONFIGURABLE_HI)
		*threshold = &tz->tz_threshold[0];
	else if (type == THERMAL_TRIP_CONFIGURABLE_LOW)
		*threshold = &tz->tz_threshold[1];
	else
		*threshold = NULL;
}

int sensor_set_trip_temp(struct thermal_zone_device *tz,
		int trip, long temp)
{
	int ret = 0;
	struct sensor_threshold *threshold = NULL;

	if (!tz->ops->get_trip_type)
		return -EPERM;

	get_trip_threshold(tz, trip, &threshold);
	if (threshold) {
		threshold->temp = temp;
		ret = sensor_set_trip(tz->sensor.sensor_id, threshold);
	} else {
		ret = tz->ops->set_trip_temp(tz, trip, temp);
	}

	return ret;
}

int sensor_init(struct thermal_zone_device *tz)
{
	struct sensor_info *sensor = &tz->sensor;

	sensor->sensor_id = tz->id;
	sensor->tz = tz;
	sensor->threshold_min = LONG_MIN;
	sensor->threshold_max = LONG_MAX;
	sensor->max_idx = -1;
	sensor->min_idx = -1;
	mutex_init(&sensor->lock);
	INIT_LIST_HEAD(&sensor->sensor_list);
	INIT_LIST_HEAD(&sensor->threshold_list);
	INIT_LIST_HEAD(&tz->tz_threshold[0].list);
	INIT_LIST_HEAD(&tz->tz_threshold[1].list);
	tz->tz_threshold[0].notify = tz_notify_trip;
	tz->tz_threshold[0].data = tz;
	tz->tz_threshold[0].trip = THERMAL_TRIP_CONFIGURABLE_HI;
	tz->tz_threshold[1].notify = tz_notify_trip;
	tz->tz_threshold[1].data = tz;
	tz->tz_threshold[1].trip = THERMAL_TRIP_CONFIGURABLE_LOW;
	list_add(&sensor->sensor_list, &sensor_info_list);
	INIT_WORK(&sensor->work, sensor_update_work);

	return 0;
}

static int get_idr(struct idr *idr, struct mutex *lock, int *id)
{
	int err;

again:
	if (unlikely(idr_pre_get(idr, GFP_KERNEL) == 0))
		return -ENOMEM;

	if (lock)
		mutex_lock(lock);
	err = idr_get_new(idr, NULL, id);
	if (lock)
		mutex_unlock(lock);
	if (unlikely(err == -EAGAIN))
		goto again;
	else if (unlikely(err))
		return err;

	*id = *id & MAX_ID_MASK;
	return 0;
}

static void release_idr(struct idr *idr, struct mutex *lock, int id)
{
	if (lock)
		mutex_lock(lock);
	idr_remove(idr, id);
	if (lock)
		mutex_unlock(lock);
}


#define to_thermal_zone(_dev) \
	container_of(_dev, struct thermal_zone_device, device)

static ssize_t
type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);

	return sprintf(buf, "%s\n", tz->type);
}

static ssize_t
temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	long temperature;
	int ret;

	if (!tz->ops->get_temp)
		return -EPERM;

	ret = tz->ops->get_temp(tz, &temperature);

	if (ret)
		return ret;

	return sprintf(buf, "%ld\n", temperature);
}

static ssize_t
temp_notify_store(struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	unsigned int status=0;
	unsigned long  value=0;
	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	if (value == 1) 
		status = kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, clr_shutdown_warning);
	if (value == 2)
		status = kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, shutdown_waring);
	if (value == 3)
		status = kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, truly_shutdown);

	pr_info("cpu temp uevent :temp=%s,sucess=%d\n",buf,status);

	return count;
}

static ssize_t
mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	enum thermal_device_mode mode;
	int result;

	if (!tz->ops->get_mode)
		return -EPERM;

	result = tz->ops->get_mode(tz, &mode);
	if (result)
		return result;

	return sprintf(buf, "%s\n", mode == THERMAL_DEVICE_ENABLED ? "enabled"
		       : "disabled");
}

static ssize_t
mode_store(struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int result;

	if (!tz->ops->set_mode)
		return -EPERM;

	if (!strncmp(buf, "enabled", sizeof("enabled") - 1))
		result = tz->ops->set_mode(tz, THERMAL_DEVICE_ENABLED);
	else if (!strncmp(buf, "disabled", sizeof("disabled") - 1))
		result = tz->ops->set_mode(tz, THERMAL_DEVICE_DISABLED);
	else
		result = -EINVAL;

	if (result)
		return result;

	return count;
}

static ssize_t
trip_point_type_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	enum thermal_trip_type type;
	int trip, result;

	if (!tz->ops->get_trip_type)
		return -EPERM;

	if (!sscanf(attr->attr.name, "trip_point_%d_type", &trip))
		return -EINVAL;

	result = tz->ops->get_trip_type(tz, trip, &type);
	if (result)
		return result;

	switch (type) {
	case THERMAL_TRIP_CRITICAL:
		return sprintf(buf, "critical\n");
	case THERMAL_TRIP_HOT:
		return sprintf(buf, "hot\n");
	case THERMAL_TRIP_CONFIGURABLE_HI:
		return sprintf(buf, "configurable_hi\n");
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		return sprintf(buf, "configurable_low\n");
	case THERMAL_TRIP_CRITICAL_LOW:
		return sprintf(buf, "critical_low\n");
	case THERMAL_TRIP_PASSIVE:
		return sprintf(buf, "passive\n");
	case THERMAL_TRIP_ACTIVE:
		return sprintf(buf, "active\n");
	default:
		return sprintf(buf, "unknown\n");
	}
}

static ssize_t
trip_point_type_activate(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int trip, result = 0;
	bool activate;
	struct sensor_threshold *threshold = NULL;

	if (!tz->ops->get_trip_type ||
		!tz->ops->activate_trip_type) {
		result = -EPERM;
		goto trip_activate_exit;
	}

	if (!sscanf(attr->attr.name, "trip_point_%d_type", &trip)) {
		result = -EINVAL;
		goto trip_activate_exit;
	}

	if (!strcmp(buf, "enabled")) {
		activate = true;
	} else if (!strcmp(buf, "disabled")) {
		activate = false;
	} else {
		result = -EINVAL;
		goto trip_activate_exit;
	}

	get_trip_threshold(tz, trip, &threshold);
	if (threshold)
		result = sensor_activate_trip(tz->sensor.sensor_id,
			threshold, activate);
	else
		result = tz->ops->activate_trip_type(tz, trip,
			activate ? THERMAL_TRIP_ACTIVATION_ENABLED :
			THERMAL_TRIP_ACTIVATION_DISABLED);

trip_activate_exit:
	if (result)
		return result;

	return count;
}

static ssize_t
trip_point_temp_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int trip, ret;
	long temperature;

	if (!tz->ops->get_trip_temp)
		return -EPERM;

	if (!sscanf(attr->attr.name, "trip_point_%d_temp", &trip))
		return -EINVAL;

	ret = tz->ops->get_trip_temp(tz, trip, &temperature);

	if (ret)
		return ret;

	return sprintf(buf, "%ld\n", temperature);
}

static ssize_t
trip_point_temp_set(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int trip, ret;
	long temperature;

	if (!tz->ops->set_trip_temp)
		return -EPERM;

	if (!sscanf(attr->attr.name, "trip_point_%d_temp", &trip))
		return -EINVAL;

	if (!sscanf(buf, "%ld", &temperature))
		return -EINVAL;

	ret = sensor_set_trip_temp(tz, trip, temperature);

	if (ret)
		return ret;

	return count;
}

static ssize_t
passive_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	struct thermal_cooling_device *cdev = NULL;
	int state;

	if (!sscanf(buf, "%d\n", &state))
		return -EINVAL;

	if (state && state < 1000)
		return -EINVAL;

	if (state && !tz->forced_passive) {
		mutex_lock(&thermal_list_lock);
		list_for_each_entry(cdev, &thermal_cdev_list, node) {
			if (!strncmp("Processor", cdev->type,
				     sizeof("Processor")))
				thermal_zone_bind_cooling_device(tz,
								 THERMAL_TRIPS_NONE,
								 cdev);
		}
		mutex_unlock(&thermal_list_lock);
		if (!tz->passive_delay)
			tz->passive_delay = 1000;
	} else if (!state && tz->forced_passive) {
		mutex_lock(&thermal_list_lock);
		list_for_each_entry(cdev, &thermal_cdev_list, node) {
			if (!strncmp("Processor", cdev->type,
				     sizeof("Processor")))
				thermal_zone_unbind_cooling_device(tz,
								   THERMAL_TRIPS_NONE,
								   cdev);
		}
		mutex_unlock(&thermal_list_lock);
		tz->passive_delay = 0;
	}

	tz->tc1 = 1;
	tz->tc2 = 1;

	tz->forced_passive = state;

	thermal_zone_device_update(tz);

	return count;
}

static ssize_t
passive_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);

	return sprintf(buf, "%d\n", tz->forced_passive);
}

static DEVICE_ATTR(type, 0444, type_show, NULL);
static DEVICE_ATTR(temp, 0444, temp_show, NULL);
static DEVICE_ATTR(mode, 0644, mode_show, mode_store);
static DEVICE_ATTR(passive, S_IRUGO | S_IWUSR, passive_show, passive_store);
static DEVICE_ATTR(temp_notify, S_IWUSR, NULL, temp_notify_store);

static struct device_attribute trip_point_attrs[] = {
	__ATTR(trip_point_0_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_0_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_1_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_1_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_2_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_2_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_3_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_3_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_4_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_4_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_5_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_5_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_6_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_6_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_7_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_7_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_8_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_8_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_9_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_9_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_10_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_10_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
	__ATTR(trip_point_11_type, 0644, trip_point_type_show,
					trip_point_type_activate),
	__ATTR(trip_point_11_temp, 0644, trip_point_temp_show,
					trip_point_temp_set),
};

#define to_cooling_device(_dev)	\
	container_of(_dev, struct thermal_cooling_device, device)

static ssize_t
thermal_cooling_device_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);

	return sprintf(buf, "%s\n", cdev->type);
}

static ssize_t
thermal_cooling_device_max_state_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	unsigned long state;
	int ret;

	ret = cdev->ops->get_max_state(cdev, &state);
	if (ret)
		return ret;
	return sprintf(buf, "%ld\n", state);
}

static ssize_t
thermal_cooling_device_cur_state_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	unsigned long state;
	int ret;

	ret = cdev->ops->get_cur_state(cdev, &state);
	if (ret)
		return ret;
	return sprintf(buf, "%ld\n", state);
}

static ssize_t
thermal_cooling_device_cur_state_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	unsigned long state;
	int result;

	if (!sscanf(buf, "%ld\n", &state))
		return -EINVAL;

	if ((long)state < 0)
		return -EINVAL;

	result = cdev->ops->set_cur_state(cdev, state);
	if (result)
		return result;
	return count;
}

static struct device_attribute dev_attr_cdev_type =
__ATTR(type, 0444, thermal_cooling_device_type_show, NULL);
static DEVICE_ATTR(max_state, 0444,
		   thermal_cooling_device_max_state_show, NULL);
static DEVICE_ATTR(cur_state, 0644,
		   thermal_cooling_device_cur_state_show,
		   thermal_cooling_device_cur_state_store);

static ssize_t
thermal_cooling_device_trip_point_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device_instance *instance;

	instance =
	    container_of(attr, struct thermal_cooling_device_instance, attr);

	if (instance->trip == THERMAL_TRIPS_NONE)
		return sprintf(buf, "-1\n");
	else
		return sprintf(buf, "%d\n", instance->trip);
}


#if defined(CONFIG_THERMAL_HWMON)

#include <linux/hwmon.h>

struct thermal_hwmon_device {
	char type[THERMAL_NAME_LENGTH];
	struct device *device;
	int count;
	struct list_head tz_list;
	struct list_head node;
};

struct thermal_hwmon_attr {
	struct device_attribute attr;
	char name[16];
};

struct thermal_hwmon_temp {
	struct list_head hwmon_node;
	struct thermal_zone_device *tz;
	struct thermal_hwmon_attr temp_input;	
	struct thermal_hwmon_attr temp_crit;	
};

static LIST_HEAD(thermal_hwmon_list);

static ssize_t
name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_hwmon_device *hwmon = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", hwmon->type);
}
static DEVICE_ATTR(name, 0444, name_show, NULL);

static ssize_t
temp_input_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long temperature;
	int ret;
	struct thermal_hwmon_attr *hwmon_attr
			= container_of(attr, struct thermal_hwmon_attr, attr);
	struct thermal_hwmon_temp *temp
			= container_of(hwmon_attr, struct thermal_hwmon_temp,
				       temp_input);
	struct thermal_zone_device *tz = temp->tz;

	ret = tz->ops->get_temp(tz, &temperature);

	if (ret)
		return ret;

	return sprintf(buf, "%ld\n", temperature);
}

static ssize_t
temp_crit_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct thermal_hwmon_attr *hwmon_attr
			= container_of(attr, struct thermal_hwmon_attr, attr);
	struct thermal_hwmon_temp *temp
			= container_of(hwmon_attr, struct thermal_hwmon_temp,
				       temp_crit);
	struct thermal_zone_device *tz = temp->tz;
	long temperature;
	int ret;

	ret = tz->ops->get_trip_temp(tz, 0, &temperature);
	if (ret)
		return ret;

	return sprintf(buf, "%ld\n", temperature);
}


static struct thermal_hwmon_device *
thermal_hwmon_lookup_by_type(const struct thermal_zone_device *tz)
{
	struct thermal_hwmon_device *hwmon;

	mutex_lock(&thermal_list_lock);
	list_for_each_entry(hwmon, &thermal_hwmon_list, node)
		if (!strcmp(hwmon->type, tz->type)) {
			mutex_unlock(&thermal_list_lock);
			return hwmon;
		}
	mutex_unlock(&thermal_list_lock);

	return NULL;
}

static struct thermal_hwmon_temp *
thermal_hwmon_lookup_temp(const struct thermal_hwmon_device *hwmon,
			  const struct thermal_zone_device *tz)
{
	struct thermal_hwmon_temp *temp;

	mutex_lock(&thermal_list_lock);
	list_for_each_entry(temp, &hwmon->tz_list, hwmon_node)
		if (temp->tz == tz) {
			mutex_unlock(&thermal_list_lock);
			return temp;
		}
	mutex_unlock(&thermal_list_lock);

	return NULL;
}

static int
thermal_add_hwmon_sysfs(struct thermal_zone_device *tz)
{
	struct thermal_hwmon_device *hwmon;
	struct thermal_hwmon_temp *temp;
	int new_hwmon_device = 1;
	int result;

	hwmon = thermal_hwmon_lookup_by_type(tz);
	if (hwmon) {
		new_hwmon_device = 0;
		goto register_sys_interface;
	}

	hwmon = kzalloc(sizeof(struct thermal_hwmon_device), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	INIT_LIST_HEAD(&hwmon->tz_list);
	strlcpy(hwmon->type, tz->type, THERMAL_NAME_LENGTH);
	hwmon->device = hwmon_device_register(NULL);
	if (IS_ERR(hwmon->device)) {
		result = PTR_ERR(hwmon->device);
		goto free_mem;
	}
	dev_set_drvdata(hwmon->device, hwmon);
	result = device_create_file(hwmon->device, &dev_attr_name);
	if (result)
		goto free_mem;

 register_sys_interface:
	temp = kzalloc(sizeof(struct thermal_hwmon_temp), GFP_KERNEL);
	if (!temp) {
		result = -ENOMEM;
		goto unregister_name;
	}

	temp->tz = tz;
	hwmon->count++;

	snprintf(temp->temp_input.name, THERMAL_NAME_LENGTH,
		 "temp%d_input", hwmon->count);
	temp->temp_input.attr.attr.name = temp->temp_input.name;
	temp->temp_input.attr.attr.mode = 0444;
	temp->temp_input.attr.show = temp_input_show;
	sysfs_attr_init(&temp->temp_input.attr.attr);
	result = device_create_file(hwmon->device, &temp->temp_input.attr);
	if (result)
		goto free_temp_mem;

	if (tz->ops->get_crit_temp) {
		unsigned long temperature;
		if (!tz->ops->get_crit_temp(tz, &temperature)) {
			snprintf(temp->temp_crit.name, THERMAL_NAME_LENGTH,
				"temp%d_crit", hwmon->count);
			temp->temp_crit.attr.attr.name = temp->temp_crit.name;
			temp->temp_crit.attr.attr.mode = 0444;
			temp->temp_crit.attr.show = temp_crit_show;
			sysfs_attr_init(&temp->temp_crit.attr.attr);
			result = device_create_file(hwmon->device,
						    &temp->temp_crit.attr);
			if (result)
				goto unregister_input;
		}
	}

	mutex_lock(&thermal_list_lock);
	if (new_hwmon_device)
		list_add_tail(&hwmon->node, &thermal_hwmon_list);
	list_add_tail(&temp->hwmon_node, &hwmon->tz_list);
	mutex_unlock(&thermal_list_lock);

	return 0;

 unregister_input:
	device_remove_file(hwmon->device, &temp->temp_input.attr);
 free_temp_mem:
	kfree(temp);
 unregister_name:
	if (new_hwmon_device) {
		device_remove_file(hwmon->device, &dev_attr_name);
		hwmon_device_unregister(hwmon->device);
	}
 free_mem:
	if (new_hwmon_device)
		kfree(hwmon);

	return result;
}

static void
thermal_remove_hwmon_sysfs(struct thermal_zone_device *tz)
{
	struct thermal_hwmon_device *hwmon;
	struct thermal_hwmon_temp *temp;

	hwmon = thermal_hwmon_lookup_by_type(tz);
	if (unlikely(!hwmon)) {
		
		dev_dbg(&tz->device, "hwmon device lookup failed!\n");
		return;
	}

	temp = thermal_hwmon_lookup_temp(hwmon, tz);
	if (unlikely(!temp)) {
		
		dev_dbg(&tz->device, "temperature input lookup failed!\n");
		return;
	}

	device_remove_file(hwmon->device, &temp->temp_input.attr);
	if (tz->ops->get_crit_temp)
		device_remove_file(hwmon->device, &temp->temp_crit.attr);

	mutex_lock(&thermal_list_lock);
	list_del(&temp->hwmon_node);
	kfree(temp);
	if (!list_empty(&hwmon->tz_list)) {
		mutex_unlock(&thermal_list_lock);
		return;
	}
	list_del(&hwmon->node);
	mutex_unlock(&thermal_list_lock);

	device_remove_file(hwmon->device, &dev_attr_name);
	hwmon_device_unregister(hwmon->device);
	kfree(hwmon);
}
#else
static int
thermal_add_hwmon_sysfs(struct thermal_zone_device *tz)
{
	return 0;
}

static void
thermal_remove_hwmon_sysfs(struct thermal_zone_device *tz)
{
}
#endif

static void thermal_zone_device_set_polling(struct thermal_zone_device *tz,
					    int delay)
{
	cancel_delayed_work(&(tz->poll_queue));

	if (!delay)
		return;

	if (delay > 1000)
		queue_delayed_work(system_freezable_wq, &(tz->poll_queue),
				      round_jiffies(msecs_to_jiffies(delay)));
	else
		queue_delayed_work(system_freezable_wq, &(tz->poll_queue),
				      msecs_to_jiffies(delay));
}

static void thermal_zone_device_passive(struct thermal_zone_device *tz,
					int temp, int trip_temp, int trip)
{
	int trend = 0;
	struct thermal_cooling_device_instance *instance;
	struct thermal_cooling_device *cdev;
	long state, max_state;

	if (temp >= trip_temp) {
		tz->passive = true;

		trend = (tz->tc1 * (temp - tz->last_temperature)) +
			(tz->tc2 * (temp - trip_temp));

		
		if (trend > 0) {
			list_for_each_entry(instance, &tz->cooling_devices,
					    node) {
				if (instance->trip != trip)
					continue;
				cdev = instance->cdev;
				cdev->ops->get_cur_state(cdev, &state);
				cdev->ops->get_max_state(cdev, &max_state);
				if (state++ < max_state)
					cdev->ops->set_cur_state(cdev, state);
			}
		} else if (trend < 0) { 
			list_for_each_entry(instance, &tz->cooling_devices,
					    node) {
				if (instance->trip != trip)
					continue;
				cdev = instance->cdev;
				cdev->ops->get_cur_state(cdev, &state);
				cdev->ops->get_max_state(cdev, &max_state);
				if (state > 0)
					cdev->ops->set_cur_state(cdev, --state);
			}
		}
		return;
	}

	list_for_each_entry(instance, &tz->cooling_devices, node) {
		if (instance->trip != trip)
			continue;
		cdev = instance->cdev;
		cdev->ops->get_cur_state(cdev, &state);
		cdev->ops->get_max_state(cdev, &max_state);
		if (state > 0)
			cdev->ops->set_cur_state(cdev, --state);
		if (state == 0)
			tz->passive = false;
	}
}

static void thermal_zone_device_check(struct work_struct *work)
{
	struct thermal_zone_device *tz = container_of(work, struct
						      thermal_zone_device,
						      poll_queue.work);
	thermal_zone_device_update(tz);
}

int thermal_zone_bind_cooling_device(struct thermal_zone_device *tz,
				     int trip,
				     struct thermal_cooling_device *cdev)
{
	struct thermal_cooling_device_instance *dev;
	struct thermal_cooling_device_instance *pos;
	struct thermal_zone_device *pos1;
	struct thermal_cooling_device *pos2;
	int result;

	if (trip >= tz->trips || (trip < 0 && trip != THERMAL_TRIPS_NONE))
		return -EINVAL;

	list_for_each_entry(pos1, &thermal_tz_list, node) {
		if (pos1 == tz)
			break;
	}
	list_for_each_entry(pos2, &thermal_cdev_list, node) {
		if (pos2 == cdev)
			break;
	}

	if (tz != pos1 || cdev != pos2)
		return -EINVAL;

	dev =
	    kzalloc(sizeof(struct thermal_cooling_device_instance), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->tz = tz;
	dev->cdev = cdev;
	dev->trip = trip;
	result = get_idr(&tz->idr, &tz->lock, &dev->id);
	if (result)
		goto free_mem;

	sprintf(dev->name, "cdev%d", dev->id);
	result =
	    sysfs_create_link(&tz->device.kobj, &cdev->device.kobj, dev->name);
	if (result)
		goto release_idr;

	snprintf(dev->attr_name, THERMAL_NAME_LENGTH, "cdev%d_trip_point", dev->id);
	sysfs_attr_init(&dev->attr.attr);
	dev->attr.attr.name = dev->attr_name;
	dev->attr.attr.mode = 0444;
	dev->attr.show = thermal_cooling_device_trip_point_show;
	result = device_create_file(&tz->device, &dev->attr);
	if (result)
		goto remove_symbol_link;

	mutex_lock(&tz->lock);
	list_for_each_entry(pos, &tz->cooling_devices, node)
	    if (pos->tz == tz && pos->trip == trip && pos->cdev == cdev) {
		result = -EEXIST;
		break;
	}
	if (!result)
		list_add_tail(&dev->node, &tz->cooling_devices);
	mutex_unlock(&tz->lock);

	if (!result)
		return 0;

	device_remove_file(&tz->device, &dev->attr);
remove_symbol_link:
	sysfs_remove_link(&tz->device.kobj, dev->name);
release_idr:
	release_idr(&tz->idr, &tz->lock, dev->id);
free_mem:
	kfree(dev);
	return result;
}
EXPORT_SYMBOL(thermal_zone_bind_cooling_device);

int thermal_zone_unbind_cooling_device(struct thermal_zone_device *tz,
				       int trip,
				       struct thermal_cooling_device *cdev)
{
	struct thermal_cooling_device_instance *pos, *next;

	mutex_lock(&tz->lock);
	list_for_each_entry_safe(pos, next, &tz->cooling_devices, node) {
		if (pos->tz == tz && pos->trip == trip && pos->cdev == cdev) {
			list_del(&pos->node);
			mutex_unlock(&tz->lock);
			goto unbind;
		}
	}
	mutex_unlock(&tz->lock);

	return -ENODEV;

unbind:
	device_remove_file(&tz->device, &pos->attr);
	sysfs_remove_link(&tz->device.kobj, pos->name);
	release_idr(&tz->idr, &tz->lock, pos->id);
	kfree(pos);
	return 0;
}
EXPORT_SYMBOL(thermal_zone_unbind_cooling_device);

static void thermal_release(struct device *dev)
{
	struct thermal_zone_device *tz;
	struct thermal_cooling_device *cdev;

	if (!strncmp(dev_name(dev), "thermal_zone",
		     sizeof("thermal_zone") - 1)) {
		tz = to_thermal_zone(dev);
		kfree(tz);
	} else {
		cdev = to_cooling_device(dev);
		kfree(cdev);
	}
}

static struct class thermal_class = {
	.name = "thermal",
	.dev_release = thermal_release,
};

struct thermal_cooling_device *
thermal_cooling_device_register(char *type, void *devdata,
				const struct thermal_cooling_device_ops *ops)
{
	struct thermal_cooling_device *cdev;
	struct thermal_zone_device *pos;
	int result;

	if (strlen(type) >= THERMAL_NAME_LENGTH)
		return ERR_PTR(-EINVAL);

	if (!ops || !ops->get_max_state || !ops->get_cur_state ||
	    !ops->set_cur_state)
		return ERR_PTR(-EINVAL);

	cdev = kzalloc(sizeof(struct thermal_cooling_device), GFP_KERNEL);
	if (!cdev)
		return ERR_PTR(-ENOMEM);

	result = get_idr(&thermal_cdev_idr, &thermal_idr_lock, &cdev->id);
	if (result) {
		kfree(cdev);
		return ERR_PTR(result);
	}

	strcpy(cdev->type, type);
	cdev->ops = ops;
	cdev->device.class = &thermal_class;
	cdev->devdata = devdata;
	dev_set_name(&cdev->device, "cooling_device%d", cdev->id);
	result = device_register(&cdev->device);
	if (result) {
		release_idr(&thermal_cdev_idr, &thermal_idr_lock, cdev->id);
		kfree(cdev);
		return ERR_PTR(result);
	}

	
	if (type) {
		result = device_create_file(&cdev->device, &dev_attr_cdev_type);
		if (result)
			goto unregister;
	}

	result = device_create_file(&cdev->device, &dev_attr_max_state);
	if (result)
		goto unregister;

	result = device_create_file(&cdev->device, &dev_attr_cur_state);
	if (result)
		goto unregister;

	mutex_lock(&thermal_list_lock);
	list_add(&cdev->node, &thermal_cdev_list);
	list_for_each_entry(pos, &thermal_tz_list, node) {
		if (!pos->ops->bind)
			continue;
		result = pos->ops->bind(pos, cdev);
		if (result)
			break;

	}
	mutex_unlock(&thermal_list_lock);

	if (!result)
		return cdev;

unregister:
	release_idr(&thermal_cdev_idr, &thermal_idr_lock, cdev->id);
	device_unregister(&cdev->device);
	return ERR_PTR(result);
}
EXPORT_SYMBOL(thermal_cooling_device_register);

void thermal_cooling_device_unregister(struct
				       thermal_cooling_device
				       *cdev)
{
	struct thermal_zone_device *tz;
	struct thermal_cooling_device *pos = NULL;

	if (!cdev)
		return;

	mutex_lock(&thermal_list_lock);
	list_for_each_entry(pos, &thermal_cdev_list, node)
	    if (pos == cdev)
		break;
	if (pos != cdev) {
		
		mutex_unlock(&thermal_list_lock);
		return;
	}
	list_del(&cdev->node);
	list_for_each_entry(tz, &thermal_tz_list, node) {
		if (!tz->ops->unbind)
			continue;
		tz->ops->unbind(tz, cdev);
	}
	mutex_unlock(&thermal_list_lock);
	if (cdev->type[0])
		device_remove_file(&cdev->device, &dev_attr_cdev_type);
	device_remove_file(&cdev->device, &dev_attr_max_state);
	device_remove_file(&cdev->device, &dev_attr_cur_state);

	release_idr(&thermal_cdev_idr, &thermal_idr_lock, cdev->id);
	device_unregister(&cdev->device);
	return;
}
EXPORT_SYMBOL(thermal_cooling_device_unregister);


void thermal_zone_device_update(struct thermal_zone_device *tz)
{
	int count, ret = 0;
	long temp, trip_temp;
	enum thermal_trip_type trip_type;
	struct thermal_cooling_device_instance *instance;
	struct thermal_cooling_device *cdev;
	int denominator;

	mutex_lock(&tz->lock);

	if (!strcmp(tz->type, "pm8941_tz") || !strcmp(tz->type, "pm8841_tz"))
		denominator = 1000;
	else
		denominator = 1;

	if (tz->ops->get_temp(tz, &temp)) {
		
		pr_warn("failed to read out thermal zone %d\n", tz->id);
		goto leave;
	}
	pr_info("%s: Current temperature (%ld C)\n",tz->device.kobj.name, temp/denominator);

	for (count = 0; count < tz->trips; count++) {
		tz->ops->get_trip_type(tz, count, &trip_type);
		tz->ops->get_trip_temp(tz, count, &trip_temp);

		pr_info("trip_temp:%ld, type:%d\n", trip_temp/denominator, trip_type);

		switch (trip_type) {
		case THERMAL_TRIP_CRITICAL:
			if (temp >= trip_temp) {
				if (tz->ops->notify)
					ret = tz->ops->notify(tz, count,
							      trip_type);
				if (!ret) {
					pr_info("%s:%s: current temp(%ld) higher or equal max limit temp(%ld)\n",
						__func__, tz->device.kobj.name, temp/denominator, trip_temp/denominator);
#if defined(CONFIG_THERMAL_MAX_TEMP_PROTECT)
					pr_emerg("Critical temperature reached (%ld C), shutting down\n",
						 temp/denominator);
					orderly_poweroff(true);
#endif
				}
			}
			break;
		case THERMAL_TRIP_HOT:
			if (temp >= trip_temp)
				if (tz->ops->notify)
					tz->ops->notify(tz, count, trip_type);
			break;
		case THERMAL_TRIP_CONFIGURABLE_HI:
			if (temp >= trip_temp)
				if (tz->ops->notify)
					tz->ops->notify(tz, count, trip_type);
			break;
		case THERMAL_TRIP_CONFIGURABLE_LOW:
			if (temp <= trip_temp)
				if (tz->ops->notify)
					tz->ops->notify(tz, count, trip_type);
			break;
		case THERMAL_TRIP_CRITICAL_LOW:
			if (temp <= trip_temp) {
				if (tz->ops->notify)
					ret = tz->ops->notify(tz, count,
								trip_type);
			if (!ret) {
#if defined(CONFIG_THERMAL_MIN_TEMP_PROTECT)
				pr_emerg("Critical temperature reached (%ld C), \
					shutting down.\n", temp/denominator);
				orderly_poweroff(true);
#endif
				}

			}
			break;
		case THERMAL_TRIP_ACTIVE:
			list_for_each_entry(instance, &tz->cooling_devices,
					    node) {
				if (instance->trip != count)
					continue;

				cdev = instance->cdev;

				if (temp >= trip_temp)
					cdev->ops->set_cur_state(cdev, 1);
				else
					cdev->ops->set_cur_state(cdev, 0);
			}
			break;
		case THERMAL_TRIP_PASSIVE:
			if (temp >= trip_temp || tz->passive)
				thermal_zone_device_passive(tz, temp,
							    trip_temp, count);
			break;
		}
	}

	if (tz->forced_passive)
		thermal_zone_device_passive(tz, temp, tz->forced_passive,
					    THERMAL_TRIPS_NONE);

	tz->last_temperature = temp;

leave:
	if (tz->passive)
		thermal_zone_device_set_polling(tz, tz->passive_delay);
	else if (tz->polling_delay)
		thermal_zone_device_set_polling(tz, tz->polling_delay);
	else
		thermal_zone_device_set_polling(tz, 0);
	mutex_unlock(&tz->lock);
}
EXPORT_SYMBOL(thermal_zone_device_update);

struct thermal_zone_device *thermal_zone_device_register(char *type,
	int trips, void *devdata,
	const struct thermal_zone_device_ops *ops,
	int tc1, int tc2, int passive_delay, int polling_delay)
{
	struct thermal_zone_device *tz;
	struct thermal_cooling_device *pos;
	enum thermal_trip_type trip_type;
	int result;
	int count;
	int passive = 0;

	if (strlen(type) >= THERMAL_NAME_LENGTH)
		return ERR_PTR(-EINVAL);

	if (trips > THERMAL_MAX_TRIPS || trips < 0)
		return ERR_PTR(-EINVAL);

	if (!ops || !ops->get_temp)
		return ERR_PTR(-EINVAL);

	tz = kzalloc(sizeof(struct thermal_zone_device), GFP_KERNEL);
	if (!tz)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&tz->cooling_devices);
	idr_init(&tz->idr);
	mutex_init(&tz->lock);
	result = get_idr(&thermal_tz_idr, &thermal_idr_lock, &tz->id);
	if (result) {
		kfree(tz);
		return ERR_PTR(result);
	}

	strcpy(tz->type, type);
	tz->ops = ops;
	tz->device.class = &thermal_class;
	tz->devdata = devdata;
	tz->trips = trips;
	tz->tc1 = tc1;
	tz->tc2 = tc2;
	tz->passive_delay = passive_delay;
	tz->polling_delay = polling_delay;

	dev_set_name(&tz->device, "thermal_zone%d", tz->id);
	result = device_register(&tz->device);
	if (result) {
		release_idr(&thermal_tz_idr, &thermal_idr_lock, tz->id);
		kfree(tz);
		return ERR_PTR(result);
	}

	
	if (type) {
		result = device_create_file(&tz->device, &dev_attr_type);
		if (result)
			goto unregister;
	}

	result = device_create_file(&tz->device, &dev_attr_temp);
	if (result)
		goto unregister;

	if (ops->get_mode) {
		result = device_create_file(&tz->device, &dev_attr_mode);
		if (result)
			goto unregister;
	}

	for (count = 0; count < trips; count++) {
		result = device_create_file(&tz->device,
					    &trip_point_attrs[count * 2]);
		if (result)
			break;
		result = device_create_file(&tz->device,
					    &trip_point_attrs[count * 2 + 1]);
		if (result)
			goto unregister;
		tz->ops->get_trip_type(tz, count, &trip_type);
		if (trip_type == THERMAL_TRIP_PASSIVE)
			passive = 1;
	}

	if (!passive)
		result = device_create_file(&tz->device,
					    &dev_attr_passive);

	if (result)
		goto unregister;

	result = device_create_file(&tz->device, &dev_attr_temp_notify);
	if (result)
		goto unregister;

	result = thermal_add_hwmon_sysfs(tz);
	if (result)
		goto unregister;

	mutex_lock(&thermal_list_lock);
	list_add_tail(&tz->node, &thermal_tz_list);
	if (ops->bind)
		list_for_each_entry(pos, &thermal_cdev_list, node) {
		result = ops->bind(tz, pos);
		if (result)
			break;
		}
	sensor_init(tz);
	mutex_unlock(&thermal_list_lock);

	INIT_DELAYED_WORK(&(tz->poll_queue), thermal_zone_device_check);

	thermal_zone_device_update(tz);

	if (!result)
		return tz;

unregister:
	release_idr(&thermal_tz_idr, &thermal_idr_lock, tz->id);
	device_unregister(&tz->device);
	return ERR_PTR(result);
}
EXPORT_SYMBOL(thermal_zone_device_register);

void thermal_zone_device_unregister(struct thermal_zone_device *tz)
{
	struct thermal_cooling_device *cdev;
	struct thermal_zone_device *pos = NULL;
	int count;

	if (!tz)
		return;

	mutex_lock(&thermal_list_lock);
	list_for_each_entry(pos, &thermal_tz_list, node)
	    if (pos == tz)
		break;
	if (pos != tz) {
		
		mutex_unlock(&thermal_list_lock);
		return;
	}
	list_del(&tz->node);
	if (tz->ops->unbind)
		list_for_each_entry(cdev, &thermal_cdev_list, node)
		    tz->ops->unbind(tz, cdev);
	mutex_unlock(&thermal_list_lock);

	thermal_zone_device_set_polling(tz, 0);

	if (tz->type[0])
		device_remove_file(&tz->device, &dev_attr_type);
	device_remove_file(&tz->device, &dev_attr_temp);
	if (tz->ops->get_mode)
		device_remove_file(&tz->device, &dev_attr_mode);

	for (count = 0; count < tz->trips; count++) {
		device_remove_file(&tz->device,
				   &trip_point_attrs[count * 2]);
		device_remove_file(&tz->device,
				   &trip_point_attrs[count * 2 + 1]);
	}
	thermal_remove_hwmon_sysfs(tz);
	flush_work(&tz->sensor.work);
	mutex_lock(&thermal_list_lock);
	list_del(&tz->sensor.sensor_list);
	mutex_unlock(&thermal_list_lock);
	release_idr(&thermal_tz_idr, &thermal_idr_lock, tz->id);
	idr_destroy(&tz->idr);
	mutex_destroy(&tz->lock);
	device_unregister(&tz->device);
	return;
}
EXPORT_SYMBOL(thermal_zone_device_unregister);

#ifdef CONFIG_NET
static struct genl_family thermal_event_genl_family = {
	.id = GENL_ID_GENERATE,
	.name = THERMAL_GENL_FAMILY_NAME,
	.version = THERMAL_GENL_VERSION,
	.maxattr = THERMAL_GENL_ATTR_MAX,
};

static struct genl_multicast_group thermal_event_mcgrp = {
	.name = THERMAL_GENL_MCAST_GROUP_NAME,
};

int thermal_generate_netlink_event(u32 orig, enum events event)
{
	struct sk_buff *skb;
	struct nlattr *attr;
	struct thermal_genl_event *thermal_event;
	void *msg_header;
	int size;
	int result;
	static unsigned int thermal_event_seqnum;

	
	size = nla_total_size(sizeof(struct thermal_genl_event)) +
	       nla_total_size(0);

	skb = genlmsg_new(size, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	
	msg_header = genlmsg_put(skb, 0, thermal_event_seqnum++,
				 &thermal_event_genl_family, 0,
				 THERMAL_GENL_CMD_EVENT);
	if (!msg_header) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	
	attr = nla_reserve(skb, THERMAL_GENL_ATTR_EVENT,
			   sizeof(struct thermal_genl_event));

	if (!attr) {
		nlmsg_free(skb);
		return -EINVAL;
	}

	thermal_event = nla_data(attr);
	if (!thermal_event) {
		nlmsg_free(skb);
		return -EINVAL;
	}

	memset(thermal_event, 0, sizeof(struct thermal_genl_event));

	thermal_event->orig = orig;
	thermal_event->event = event;

	
	result = genlmsg_end(skb, msg_header);
	if (result < 0) {
		nlmsg_free(skb);
		return result;
	}

	result = genlmsg_multicast(skb, 0, thermal_event_mcgrp.id, GFP_ATOMIC);
	if (result)
		pr_info("failed to send netlink event:%d\n", result);

	return result;
}
EXPORT_SYMBOL(thermal_generate_netlink_event);

static int genetlink_init(void)
{
	int result;

	result = genl_register_family(&thermal_event_genl_family);
	if (result)
		return result;

	result = genl_register_mc_group(&thermal_event_genl_family,
					&thermal_event_mcgrp);
	if (result)
		genl_unregister_family(&thermal_event_genl_family);
	return result;
}

static void genetlink_exit(void)
{
	genl_unregister_family(&thermal_event_genl_family);
}
#else 
static inline int genetlink_init(void) { return 0; }
static inline void genetlink_exit(void) {}
#endif 

static int __init thermal_init(void)
{
	int result = 0;

	result = class_register(&thermal_class);
	if (result) {
		idr_destroy(&thermal_tz_idr);
		idr_destroy(&thermal_cdev_idr);
		mutex_destroy(&thermal_idr_lock);
		mutex_destroy(&thermal_list_lock);
	}
	result = genetlink_init();
	return result;
}

static void __exit thermal_exit(void)
{
	class_unregister(&thermal_class);
	idr_destroy(&thermal_tz_idr);
	idr_destroy(&thermal_cdev_idr);
	mutex_destroy(&thermal_idr_lock);
	mutex_destroy(&thermal_list_lock);
	genetlink_exit();
}

fs_initcall(thermal_init);
module_exit(thermal_exit);

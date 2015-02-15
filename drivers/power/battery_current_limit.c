/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define BCL_DEV_NAME "battery_current_limit"
#define BCL_NAME_LENGTH 20
/*
 * Default BCL poll interval 1000 msec
 */
#define BCL_POLL_INTERVAL 1000
/*
 * Mininum BCL poll interval 10 msec
 */
#define MIN_BCL_POLL_INTERVAL 10
#define BATTERY_VOLTAGE_MIN 3400

static const char bcl_type[] = "bcl";

/*
 * Battery Current Limit Enable or Not
 */
enum bcl_device_mode {
	BCL_DEVICE_DISABLED = 0,
	BCL_DEVICE_ENABLED,
};

/*
 * Battery Current Limit Iavail Threshold Mode set
 */
enum bcl_iavail_threshold_mode {
	BCL_IAVAIL_THRESHOLD_DISABLED = 0,
	BCL_IAVAIL_THRESHOLD_ENABLED,
};

/*
 * Battery Current Limit Iavail Threshold Mode
 */
enum bcl_iavail_threshold_type {
	BCL_IAVAIL_LOW_THRESHOLD_TYPE = 0,
	BCL_IAVAIL_HIGH_THRESHOLD_TYPE,
	BCL_IAVAIL_THRESHOLD_TYPE_MAX,
};

/**
 * BCL control block
 *
 */
struct bcl_context {
	/* BCL device */
	struct device *dev;

	/* BCL related config parameter */
	/* BCL mode enable or not */
	enum bcl_device_mode bcl_mode;
	/* BCL Iavail Threshold Activate or Not */
	enum bcl_iavail_threshold_mode
		bcl_threshold_mode[BCL_IAVAIL_THRESHOLD_TYPE_MAX];
	/* BCL Iavail Threshold value in milli Amp */
	int bcl_threshold_value_ma[BCL_IAVAIL_THRESHOLD_TYPE_MAX];
	/* BCL Type */
	char bcl_type[BCL_NAME_LENGTH];
	/* BCL poll in msec */
	int bcl_poll_interval_msec;

	/* BCL realtime value based on poll */
	/* BCL realtime vbat in mV*/
	int bcl_vbat_mv;
	/* BCL realtime rbat in mOhms*/
	int bcl_rbat_mohm;
	/*BCL realtime iavail in milli Amp*/
	int bcl_iavail;
	/*BCL vbatt min in mV*/
	int bcl_vbat_min;
	/* BCL period poll delay work structure  */
	struct delayed_work     bcl_iavail_work;

};

static struct bcl_context *gbcl;

static int bcl_get_battery_voltage(int *vbatt_mv)
{
	static struct power_supply *psy;
	union power_supply_propval ret = {0,};

	if (psy == NULL) {
		psy = power_supply_get_by_name("battery");
		if (psy  == NULL) {
			pr_err("failed to get ps battery\n");
			return -EINVAL;
		}
	}

	if (psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret))
		return -EINVAL;

	if (ret.intval <= 0)
		return -EINVAL;

	*vbatt_mv = ret.intval / 1000;
	return 0;
}


static int bcl_get_resistance(int *rbatt_mohm)
{
	static struct power_supply *psy;
	union power_supply_propval ret = {0,};

	if (psy == NULL) {
		psy = power_supply_get_by_name("bms");
		if (psy == NULL) {
			pr_err("failed to get ps bms\n");
			return -EINVAL;
		}
	}
	if (psy->get_property(psy, POWER_SUPPLY_PROP_RESISTANCE, &ret))
		return -EINVAL;

	if (ret.intval <= 0)
		return -EINVAL;

	*rbatt_mohm = ret.intval / 1000;

	return 0;
}

/*
 * BCL iavail calculation and trigger notification to user space
 * if iavail cross threshold
 */
static void bcl_calculate_iavail_trigger(void)
{
	int iavail_ma = 0;
	int vbatt_mv;
	int rbatt_mohm;
	bool threshold_cross = false;

	if (!gbcl) {
		pr_err("called before initialization\n");
		return;
	}

	if (bcl_get_battery_voltage(&vbatt_mv))
		return;

	if (bcl_get_resistance(&rbatt_mohm))
		return;

	iavail_ma = (vbatt_mv - gbcl->bcl_vbat_min) * 1000 / rbatt_mohm;

	gbcl->bcl_rbat_mohm = rbatt_mohm;
	gbcl->bcl_vbat_mv = vbatt_mv;
	gbcl->bcl_iavail = iavail_ma;

	pr_debug("iavail %d, vbatt %d rbatt %d\n", iavail_ma, vbatt_mv,
			rbatt_mohm);

	if ((gbcl->bcl_threshold_mode[BCL_IAVAIL_HIGH_THRESHOLD_TYPE] ==
				BCL_IAVAIL_THRESHOLD_ENABLED)
		&& (iavail_ma >=
		gbcl->bcl_threshold_value_ma[BCL_IAVAIL_HIGH_THRESHOLD_TYPE]))
		threshold_cross = true;
	else if ((gbcl->bcl_threshold_mode[BCL_IAVAIL_LOW_THRESHOLD_TYPE]
				== BCL_IAVAIL_THRESHOLD_ENABLED)
		&& (iavail_ma <=
		gbcl->bcl_threshold_value_ma[BCL_IAVAIL_LOW_THRESHOLD_TYPE]))
		threshold_cross = true;

	if (threshold_cross)
		sysfs_notify(&gbcl->dev->kobj, NULL, "type");
}

/*
 * BCL iavail work
 */
static void bcl_iavail_work(struct work_struct *work)
{
	struct bcl_context *bcl = container_of(work,
			struct bcl_context, bcl_iavail_work.work);

	if (gbcl->bcl_mode == BCL_DEVICE_ENABLED) {
		bcl_calculate_iavail_trigger();
		/* restart the delay work for caculating imax */
		schedule_delayed_work(&bcl->bcl_iavail_work,
			msecs_to_jiffies(bcl->bcl_poll_interval_msec));
	}
}

/*
 * Set BCL mode
 */
static void bcl_mode_set(enum bcl_device_mode mode)
{
	if (!gbcl)
		return;

	if (gbcl->bcl_mode == mode)
		return;

	if (gbcl->bcl_mode == BCL_DEVICE_DISABLED
		&& mode == BCL_DEVICE_ENABLED) {
		gbcl->bcl_mode = mode;
		bcl_iavail_work(&(gbcl->bcl_iavail_work.work));
		return;
	} else if (gbcl->bcl_mode == BCL_DEVICE_ENABLED
		&& mode == BCL_DEVICE_DISABLED) {
		gbcl->bcl_mode = mode;
		cancel_delayed_work_sync(&(gbcl->bcl_iavail_work));
		return;
	}

	return;
}

#define show_bcl(name, variable, format) \
static ssize_t \
name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	if (gbcl) \
		return snprintf(buf, PAGE_SIZE, format, gbcl->variable); \
	else \
		return  -EPERM; \
}

show_bcl(type, bcl_type, "%s\n")
show_bcl(vbat, bcl_vbat_mv, "%d\n")
show_bcl(rbat, bcl_rbat_mohm, "%d\n")
show_bcl(iavail, bcl_iavail, "%d\n")
show_bcl(vbat_min, bcl_vbat_min, "%d\n");
show_bcl(poll_interval, bcl_poll_interval_msec, "%d\n")

static ssize_t
mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!gbcl)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n",
		gbcl->bcl_mode == BCL_DEVICE_ENABLED ? "enabled"
			: "disabled");
}

static ssize_t
mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (!gbcl)
		return -EPERM;

	if (!strncmp(buf, "enabled", 7))
		bcl_mode_set(BCL_DEVICE_ENABLED);
	else if (!strncmp(buf, "disabled", 8))
		bcl_mode_set(BCL_DEVICE_DISABLED);
	else
		return -EINVAL;

	return count;
}

static ssize_t
poll_interval_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;

	if (!gbcl)
		return -EPERM;

	if (!sscanf(buf, "%d", &value))
		return -EINVAL;

	if (value < MIN_BCL_POLL_INTERVAL)
		return -EINVAL;

	gbcl->bcl_poll_interval_msec = value;

	return count;
}

static ssize_t vbat_min_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;
	int ret;

	if (!gbcl)
		return -EPERM;

	ret = kstrtoint(buf, 10, &value);

	if (ret || (value < 0)) {
		pr_err("Incorrect vbatt min value\n");
		return -EINVAL;
	}

	gbcl->bcl_vbat_min = value;
	return count;
}

static ssize_t iavail_low_threshold_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!gbcl)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n",
		gbcl->bcl_threshold_mode[BCL_IAVAIL_LOW_THRESHOLD_TYPE]
		== BCL_IAVAIL_THRESHOLD_ENABLED ? "enabled" : "disabled");
}

static ssize_t iavail_low_threshold_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (!gbcl)
		return -EPERM;

	if (!strncmp(buf, "enabled", 7))
		gbcl->bcl_threshold_mode[BCL_IAVAIL_LOW_THRESHOLD_TYPE]
			= BCL_IAVAIL_THRESHOLD_ENABLED;
	else if (!strncmp(buf, "disabled", 7))
		gbcl->bcl_threshold_mode[BCL_IAVAIL_LOW_THRESHOLD_TYPE]
			= BCL_IAVAIL_THRESHOLD_DISABLED;
	else
		return -EINVAL;

	return count;
}
static ssize_t iavail_high_threshold_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!gbcl)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n",
		gbcl->bcl_threshold_mode[BCL_IAVAIL_HIGH_THRESHOLD_TYPE]
		== BCL_IAVAIL_THRESHOLD_ENABLED ? "enabled" : "disabled");
}

static ssize_t iavail_high_threshold_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (!gbcl)
		return -EPERM;

	if (!strncmp(buf, "enabled", 7))
		gbcl->bcl_threshold_mode[BCL_IAVAIL_HIGH_THRESHOLD_TYPE]
			= BCL_IAVAIL_THRESHOLD_ENABLED;
	else if (!strncmp(buf, "disabled", 7))
		gbcl->bcl_threshold_mode[BCL_IAVAIL_HIGH_THRESHOLD_TYPE]
			= BCL_IAVAIL_THRESHOLD_DISABLED;
	else
		return -EINVAL;

	return count;
}

static ssize_t iavail_low_threshold_value_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!gbcl)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		gbcl->bcl_threshold_value_ma[BCL_IAVAIL_LOW_THRESHOLD_TYPE]);
}


static ssize_t iavail_low_threshold_value_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val;
	int ret;

	ret = kstrtoint(buf, 10, &val);

	if (ret || (val < 0)) {
		pr_err("Incorrect available current threshold value\n");
		return -EINVAL;
	}

	gbcl->bcl_threshold_value_ma[BCL_IAVAIL_LOW_THRESHOLD_TYPE] = val;

	return count;
}
static ssize_t iavail_high_threshold_value_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	if (!gbcl)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		gbcl->bcl_threshold_value_ma[BCL_IAVAIL_HIGH_THRESHOLD_TYPE]);
}

static ssize_t iavail_high_threshold_value_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val;
	int ret;

	ret = kstrtoint(buf, 10, &val);

	if (ret || (val < 0)) {
		pr_err("Incorrect available current threshold value\n");
		return -EINVAL;
	}

	gbcl->bcl_threshold_value_ma[BCL_IAVAIL_HIGH_THRESHOLD_TYPE] = val;

	return count;
}

/*
 * BCL device attributes
 */
static struct device_attribute bcl_dev_attr[] = {
	__ATTR(type, 0444, type_show, NULL),
	__ATTR(iavail, 0444, iavail_show, NULL),
	__ATTR(vbat_min, 0644, vbat_min_show, vbat_min_store),
	__ATTR(vbat, 0444, vbat_show, NULL),
	__ATTR(rbat, 0444, rbat_show, NULL),
	__ATTR(mode, 0644, mode_show, mode_store),
	__ATTR(poll_interval, 0644,
		poll_interval_show, poll_interval_store),
	__ATTR(iavail_low_threshold_mode, 0644,
		iavail_low_threshold_mode_show,
		iavail_low_threshold_mode_store),
	__ATTR(iavail_high_threshold_mode, 0644,
		iavail_high_threshold_mode_show,
		iavail_high_threshold_mode_store),
	__ATTR(iavail_low_threshold_value, 0644,
		iavail_low_threshold_value_show,
		iavail_low_threshold_value_store),
	__ATTR(iavail_high_threshold_value, 0644,
		iavail_high_threshold_value_show,
		iavail_high_threshold_value_store),
};

static int create_bcl_sysfs(struct bcl_context *bcl)
{
	int result = 0;
	int num_attr = sizeof(bcl_dev_attr)/sizeof(struct device_attribute);
	int i;

	for (i = 0; i < num_attr; i++) {
		result = device_create_file(bcl->dev, &bcl_dev_attr[i]);
		if (result < 0)
			return result;
	}

	return 0;
}

static void remove_bcl_sysfs(struct bcl_context *bcl)
{
	int num_attr = sizeof(bcl_dev_attr)/sizeof(struct device_attribute);
	int i;

	for (i = 0; i < num_attr; i++)
		device_remove_file(bcl->dev, &bcl_dev_attr[i]);

	return;
}

static int __devinit bcl_probe(struct platform_device *pdev)
{
	struct bcl_context *bcl;
	int ret = 0;

	bcl = kzalloc(sizeof(struct bcl_context), GFP_KERNEL);

	if (!bcl) {
		pr_err("Cannot allocate bcl_context\n");
		return -ENOMEM;
	}

	gbcl = bcl;

	/* For BCL */
	/* Init default BCL params */
	bcl->dev = &pdev->dev;
	bcl->bcl_mode = BCL_DEVICE_DISABLED;
	bcl->bcl_threshold_mode[BCL_IAVAIL_LOW_THRESHOLD_TYPE] =
					BCL_IAVAIL_THRESHOLD_DISABLED;
	bcl->bcl_threshold_mode[BCL_IAVAIL_HIGH_THRESHOLD_TYPE] =
					BCL_IAVAIL_THRESHOLD_DISABLED;
	bcl->bcl_threshold_value_ma[BCL_IAVAIL_LOW_THRESHOLD_TYPE] = 0;
	bcl->bcl_threshold_value_ma[BCL_IAVAIL_HIGH_THRESHOLD_TYPE] = 0;
	bcl->bcl_vbat_min = BATTERY_VOLTAGE_MIN;
	snprintf(bcl->bcl_type, BCL_NAME_LENGTH, "%s", bcl_type);
	bcl->bcl_poll_interval_msec = BCL_POLL_INTERVAL;
	ret = create_bcl_sysfs(bcl);
	if (ret < 0) {
		pr_err("Cannot create bcl sysfs\n");
		kfree(bcl);
		return ret;
	}
	platform_set_drvdata(pdev, bcl);
	INIT_DELAYED_WORK_DEFERRABLE(&bcl->bcl_iavail_work, bcl_iavail_work);

	return 0;
}

static int __devexit bcl_remove(struct platform_device *pdev)
{
	remove_bcl_sysfs(gbcl);
	kfree(gbcl);
	gbcl = NULL;
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id bcl_match_table[] = {
	{.compatible = "qcom,bcl"},
	{},
};

static struct platform_driver bcl_driver = {
	.probe	= bcl_probe,
	.remove	= __devexit_p(bcl_remove),
	.driver	= {
		.name	= BCL_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = bcl_match_table,
	},
};

static int __init bcl_init(void)
{
	return platform_driver_register(&bcl_driver);
}

static void __exit bcl_exit(void)
{
	platform_driver_unregister(&bcl_driver);
}

late_initcall(bcl_init);
module_exit(bcl_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("battery current limit driver");
MODULE_ALIAS("platform:" BCL_DEV_NAME);

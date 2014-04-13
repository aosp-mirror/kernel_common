/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * Copyright 2014 Paul Reioux
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

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <linux/hrtimer.h>
#include <linux/spmi.h>
#include <linux/qpnp/vibrator.h>
#include "../staging/android/timed_output.h"
#include <linux/vibtrig.h>

#define QPNP_VIB_VERSION_MAJOR		1
#define QPNP_VIB_VERSION_MINOR		0

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler enabler;
#endif

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	spinlock_t lock;
};

extern struct qpnp_vib *vib_dev;

static int update_vib_level(int level)
{
	int ret = 0;
	
	if (vib_dev != NULL)
		vib_dev->vtg_level = level;
	else
		ret = -1;

	return ret;
}

static ssize_t qpnp_vib_level_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (vib_dev != NULL)
		ret = sprintf(buf, "%d\n", vib_dev->vtg_level);

	return ret;
}

static ssize_t qpnp_vib_level_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int data;

	if(sscanf(buf, "%d\n", &data) == 1) {
		update_vib_level(data);
	}

	return count;
}

static ssize_t qpnp_vib_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
			QPNP_VIB_VERSION_MAJOR, QPNP_VIB_VERSION_MINOR);
}

static struct kobj_attribute qpnp_vib_level_attribute = 
	__ATTR(qpnp_vib_level, 0666,
		qpnp_vib_level_show,
		qpnp_vib_level_store);

static struct kobj_attribute qpnp_vib_version_attribute = 
	__ATTR(qpnp_vib_version, 0444 ,
		qpnp_vib_version_show,
		NULL);

static struct attribute *qpnp_vib_attrs[] =
	{
		&qpnp_vib_level_attribute.attr,
		&qpnp_vib_version_attribute.attr,
		NULL,
	};

static struct attribute_group qpnp_vib_attr_group =
	{
		.attrs = qpnp_vib_attrs,
	};

static struct kobject *qpnp_vib_kobj;

int __init qpnp_vib_sysfs_init(void)
{
	int sysfs_result;

	pr_info("faux123: qpnp vibrator sysfs\n");

	qpnp_vib_kobj =
		kobject_create_and_add("qpnp_vib", kernel_kobj);
	if (!qpnp_vib_kobj) {
		pr_err("%s kobject create failed!\n", __func__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(qpnp_vib_kobj,
			&qpnp_vib_attr_group);

        if (sysfs_result) {
		pr_info("%s create failed!\n", __func__);
		kobject_put(qpnp_vib_kobj);
	}
	return sysfs_result;
}

module_init(qpnp_vib_sysfs_init);

MODULE_LICENSE("GPL v2"); 
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("generic qualcomm vibrator sysfs driver");


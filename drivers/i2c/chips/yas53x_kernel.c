/*
 * Copyright (c) 2013 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "yas53x_drv.c"

#define YAS53X_NAME			"yas53x"
#define YAS53X_MIN_DELAY		(200) 

#define D(x...) printk(KERN_DEBUG "[COMP][YAS53X] " x)
#define I(x...) printk(KERN_INFO "[COMP][YAS53X] " x)
#define E(x...) printk(KERN_ERR "[COMP][YAS53X] " x)

struct yas53x_data {
	struct device *dev;
	struct class *class;
	struct input_dev *input_data;
	struct delayed_work work;
	struct mutex mutex, enable_mutex;
	int32_t last_data[3];
	int8_t hard_offset[3];
	int32_t delay;
	atomic_t enable;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
};

static struct i2c_client *this_client;

static int yas53x_i2c_open(void) {
	return 0;
}

static int yas53x_i2c_close(void) {
	return 0;
}

static int yas53x_i2c_write(uint8_t addr, const uint8_t *buf, int len) {
	uint8_t tmp[16];
	if (sizeof(tmp) - 1 < len)
		return -1;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	if (i2c_master_send(this_client, tmp, len + 1) < 0)
		return -1;
	return 0;
}

static int yas53x_i2c_read(uint8_t addr, uint8_t *buf, int len) {
	struct i2c_msg msg[2];
	int err;
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&this_client->dev,
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}
	return 0;
}

static void yas53x_msleep(int ms) {
	usleep_range(ms * 1000, (ms + 1) * 1000);
}

static void yas53x_current_time(uint32_t *msec) {
	*msec = jiffies_to_msecs(jiffies);
}

static struct yas_mag_driver this_drv = {
	.callback = {
		.device_open	= yas53x_i2c_open,
		.device_close	= yas53x_i2c_close,
		.device_write	= yas53x_i2c_write,
		.device_read	= yas53x_i2c_read,
		.msleep		= yas53x_msleep,
		.current_time	= yas53x_current_time,
	},
};

static int yas53x_enable(struct yas53x_data *data) {
	if (!atomic_cmpxchg(&data->enable, 0, 1))
		schedule_delayed_work(&data->work, 0);
	return 0;
}

static int yas53x_disable(struct yas53x_data *data) {
	if (atomic_cmpxchg(&data->enable, 1, 0))
		cancel_delayed_work_sync(&data->work);
	return 0;
}

static ssize_t yas53x_delay_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	int32_t delay;
	mutex_lock(&data->mutex);
	delay = data->delay;
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d\n", delay);
}

static ssize_t yas53x_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	long value;
	if (kstrtol(buf, 10, &value) < 0)
		return -EINVAL;
	if (value < 0)
		value = 0;
	if (YAS53X_MIN_DELAY < value)
		value = YAS53X_MIN_DELAY;
	mutex_lock(&data->mutex);
	data->delay = value;
	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t yas53x_enable_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", atomic_read(&data->enable));
}

static ssize_t yas53x_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	long value;
	if (kstrtol(buf, 10, &value) < 0)
		return -EINVAL;
	if (value)
		yas53x_enable(data);
	else
		yas53x_disable(data);
	return count;
}

static ssize_t yas53x_position_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	int position;
	mutex_lock(&data->mutex);
	position = this_drv.get_position();
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d\n", position);
}

static ssize_t yas53x_position_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	long value;
	if (kstrtol(buf, 10, &value) < 0)
		return -EINVAL;
	mutex_lock(&data->mutex);
	this_drv.set_position(value);
	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t yas53x_data_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	int32_t last[3], i;
	mutex_lock(&data->mutex);
	for (i = 0; i < 3; i++)
		last[i] = data->last_data[i];
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d %d %d\n", last[0], last[1], last[2]);
}

static ssize_t yas53x_offset_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int i;
	mutex_lock(&data->mutex);
	for (i = 0; i < 3; i++)
		hard_offset[i] = data->hard_offset[i];
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d %d %d\n", hard_offset[0],
			hard_offset[1], hard_offset[2]);
}

static ssize_t yas53x_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	int tmp[3];
	int8_t hard_offset[3];
	int i;
	sscanf(buf, "%d %d %d", &tmp[0], &tmp[1], &tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t) tmp[i];
	mutex_lock(&data->mutex);
	if (this_drv.set_offset(hard_offset) == 0)
		for (i = 0; i < 3; i++)
			data->hard_offset[i] = hard_offset[i];
	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t yas53x_self_test_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	struct yas_self_test_result r;
	int rt;
	mutex_lock(&data->mutex);
	rt = this_drv.self_test(&r);
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d\n", rt, r.id,
			r.xy1y2[0], r.xy1y2[1], r.xy1y2[2], r.dir, r.sx, r.sy,
			r.xyz[0], r.xyz[1], r.xyz[2]);
}

static ssize_t yas53x_self_test_noise_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	struct yas_vector raw_xyz;
	int rt;
	mutex_lock(&data->mutex);
	rt = this_drv.self_test_noise(&raw_xyz);
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d %d %d %d\n", rt, raw_xyz.v[0], raw_xyz.v[1],
			raw_xyz.v[2]);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP, yas53x_delay_show, yas53x_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, yas53x_enable_show, yas53x_enable_store);
static DEVICE_ATTR(data, S_IRUGO, yas53x_data_show, NULL);
static DEVICE_ATTR(self_test, S_IRUSR, yas53x_self_test_show, NULL);
static DEVICE_ATTR(self_test_noise, S_IRUSR, yas53x_self_test_noise_show, NULL);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR, yas53x_position_show, yas53x_position_store);
static DEVICE_ATTR(offset, S_IRUGO|S_IWUSR, yas53x_offset_show, yas53x_offset_store);
static struct attribute *yas53x_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_data.attr,
	&dev_attr_position.attr,
	&dev_attr_offset.attr,
	&dev_attr_self_test.attr,
	&dev_attr_self_test_noise.attr,
	NULL
};
static struct attribute_group yas53x_attribute_group = {
	.attrs = yas53x_attributes
};

static void yas53x_input_work_func(struct work_struct *work) {
	struct yas53x_data *data
		= container_of((struct delayed_work *)work,
			struct yas53x_data, work);
	struct yas_mag_data mag;
	static int32_t cnt;
	int32_t delay;
	uint32_t time_before, time_after;
	int rt, i, notify_dummy = 0;

	mutex_lock(&data->mutex);
	yas53x_current_time(&time_before);
	rt = this_drv.measure(&mag);
	if (rt & YAS_REPORT_HARD_OFFSET_CHANGED)
		this_drv.get_offset(data->hard_offset);
	if (data->last_data[0] == mag.xyz.v[0]
			&& data->last_data[1] == mag.xyz.v[1]
			&& data->last_data[2] == mag.xyz.v[2])
		notify_dummy = 1;
	for (i = 0; i < 3; i++)
		data->last_data[i] = mag.xyz.v[i];
	yas53x_current_time(&time_after);
	delay = data->delay - (time_after - time_before);
	mutex_unlock(&data->mutex);
	
	input_report_abs(data->input_data, ABS_X, mag.xyz.v[0]);
	input_report_abs(data->input_data, ABS_Y, mag.xyz.v[1]);
	input_report_abs(data->input_data, ABS_Z, mag.xyz.v[2]);
	if (notify_dummy)
		input_report_abs(data->input_data, ABS_RUDDER, cnt++);
	input_sync(data->input_data);
	if (delay <= 0)
		delay = 1;
	schedule_delayed_work(&data->work, msecs_to_jiffies(delay));
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas53x_early_suspend(struct early_suspend *h) {
	struct yas53x_data *data = container_of(h, struct yas53x_data, sus);
	if (atomic_read(&data->enable))
		cancel_delayed_work_sync(&data->work);
}


static void yas53x_late_resume(struct early_suspend *h) {
	struct yas53x_data *data = container_of(h, struct yas53x_data, sus);
	if (atomic_read(&data->enable))
		schedule_delayed_work(&data->work, 0);
}
#endif

static int yas53x_suspend(struct device *dev) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	if (atomic_read(&data->enable))
		cancel_delayed_work_sync(&data->work);
	return 0;
}

static int yas53x_resume(struct device *dev) {
	struct yas53x_data *data = i2c_get_clientdata(this_client);
	if (atomic_read(&data->enable))
		schedule_delayed_work(&data->work, 0);
	return 0;
}

static int yas53x_parse_dt(struct device *dev, struct yas53x_platform_data *pdata) {
	struct property *prop = NULL;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;

	prop = of_find_property(dt, "compass_yas53x,chip_layout", NULL);
	if (prop) {
		of_property_read_u32(dt, "compass_yas53x,chip_layout", &buf);
		pdata->chip_layout = buf;
		D("%s: chip_layout = %d\n", __func__, pdata->chip_layout);
	} else
		D("%s: compass_yas53x,chip_layout not found\n", __func__);
	return 0;
}

static int __devinit yas53x_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct yas53x_platform_data *pdata;
	struct yas53x_data *data = NULL;
	struct input_dev *input_data = NULL;
	int rt, sysfs_created = 0;
	int data_registered = 0, i;

	i2c_set_clientdata(client, NULL);
	data = kzalloc(sizeof(struct yas53x_data), GFP_KERNEL);
	D("compass probe start\n");
	if (data == NULL) {
		rt = -ENOMEM;
		goto err;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->sus.suspend = yas53x_early_suspend;
	data->sus.resume = yas53x_late_resume;
	register_early_suspend(&data->sus);
#endif
	atomic_set(&data->enable, 0);
	data->delay = YAS53X_MIN_DELAY;
	for (i = 0; i < 3; i++)
		data->last_data[i] = 0;
	INIT_DELAYED_WORK(&data->work, yas53x_input_work_func);
	mutex_init(&data->mutex);
	mutex_init(&data->enable_mutex);

	input_data = input_allocate_device();
	if (input_data == NULL) {
		rt = -ENOMEM;
		E("Failed to allocate input_data device\n");
		goto err;
	}
	input_data->name = YAS53X_NAME;
	input_data->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_data->evbit);
	input_set_abs_params(input_data, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_RUDDER, INT_MIN, INT_MAX, 0, 0);
	input_data->dev.parent = &client->dev;
	rt = input_register_device(input_data);
	if (rt) {
		D("Unable to register input_data device: %s\n",
				input_data->name);
		goto err;
	}
	data_registered = 1;

	data->class = class_create(THIS_MODULE, "yas_sensors");
	if (IS_ERR(data->class)) {
		E("couldn't create sysfs class\n");
		goto err;
	}
	data->dev = device_create(data->class, NULL, 0, "%s", YAS53X_NAME);
	if (unlikely(IS_ERR(data->dev))) {
		rt = PTR_ERR(data->dev);
		data->dev = NULL;
		goto err;
	}

	rt = sysfs_create_group(&data->dev->kobj, &yas53x_attribute_group);
	if (rt) {
		E("yas53x_probe: sysfs_create_group failed[%s]\n",
					YAS53X_NAME);
		goto err;
	}
	sysfs_created = 1;

	this_client = client;
	data->input_data = input_data;
	input_set_drvdata(input_data, data);

	rt = yas_mag_driver_init(&this_drv);
	if (rt < 0) {
		E("yas_mag_driver_init failed[%d]\n", rt);
		goto err;
	}
	rt = this_drv.init();
	if (rt < 0) {
		E("this_drv.init() failed[%d]\n", rt);
		goto err;
	}

	if (client->dev.of_node) {
		D("Device Tree parsing.\n");
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(&client->dev, "%s: memory allocation for pdata failed.", __func__);
		} else
			yas53x_parse_dt(&client->dev, pdata);
	} else {
		pdata = client->dev.platform_data;
		pdata->chip_layout = *(int *)client->dev.platform_data;
	}

	if (pdata != NULL)
		this_drv.set_position(pdata->chip_layout);
	this_drv.get_offset(data->hard_offset);
	i2c_set_clientdata(client, data);
	D("compass probe end\n");
	return 0;

err:
	if (data != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&data->sus);
#endif
		if (data->dev != NULL) {
			if (sysfs_created)
				sysfs_remove_group(&data->dev->kobj,
						&yas53x_attribute_group);
			device_unregister(data->dev);
		}
		if (data->class != NULL)
			class_destroy(data->class);
		if (input_data != NULL) {
			if (data_registered)
				input_unregister_device(input_data);
			else
				input_free_device(input_data);
		}
		kfree(data);
	}
	return rt;
}

static int yas53x_remove(struct i2c_client *client) {
	struct yas53x_data *data = i2c_get_clientdata(client);
	if (data != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&data->sus);
#endif
		yas53x_disable(data);
		this_drv.term();
		sysfs_remove_group(&data->dev->kobj, &yas53x_attribute_group);
		device_unregister(data->dev);
		class_destroy(data->class);
		input_unregister_device(data->input_data);
		kfree(data);
	}
	return 0;
}

static const struct dev_pm_ops yas53x_pm_ops = {
#ifdef CONFIG_PM
	.suspend = yas53x_suspend,
	.resume  = yas53x_resume,
#endif
};

static struct i2c_device_id yas53x_id[] = {
	{YAS53X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, yas53x_id);
#ifdef CONFIG_OF
static struct of_device_id yas53x_match_table[] = {
	{.compatible = "htc_compass,yas53x" },
	{},
};
#else
#define yas53x_match_table NULL
#endif

static struct i2c_driver yas53x_i2c_driver = {
	.driver = {
		.name	= YAS53X_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = yas53x_match_table,
#ifdef CONFIG_PM
		.pm		= &yas53x_pm_ops,
#endif
	},
	.probe	=	yas53x_probe,
	.remove =	yas53x_remove,
	.id_table	=	yas53x_id,

};
module_i2c_driver(yas53x_i2c_driver);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_DESCRIPTION("YAS53x Geomagnetic Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_VERSION);

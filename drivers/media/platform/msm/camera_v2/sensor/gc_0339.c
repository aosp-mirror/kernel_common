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
#include "msm_sensor.h"
#include <linux/regulator/consumer.h>

#define GC_0339_SENSOR_NAME "gc_0339"
DEFINE_MSM_MUTEX(gc_0339_mut);

static struct msm_sensor_ctrl_t gc_0339_s_ctrl;

struct msm_sensor_power_setting gc_0339_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 8,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

struct msm_sensor_power_setting gc_0339_power_down_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
	},
};

static struct v4l2_subdev_info gc_0339_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gc_0339_i2c_id[] = {
	{GC_0339_SENSOR_NAME, (kernel_ulong_t)&gc_0339_s_ctrl},
	{ }
};

static int32_t msm_gc_0339_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gc_0339_s_ctrl);
}

static struct i2c_driver gc_0339_i2c_driver = {
	.id_table = gc_0339_i2c_id,
	.probe  = msm_gc_0339_i2c_probe,
	.driver = {
		.name = GC_0339_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc_0339_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id gc_0339_dt_match[] = {
	{.compatible = "htc,gc_0339", .data = &gc_0339_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc_0339_dt_match);

static struct platform_driver gc_0339_platform_driver = {
	.driver = {
		.name = "htc,gc_0339",
		.owner = THIS_MODULE,
		.of_match_table = gc_0339_dt_match,
	},
};

static const char *gc_0339Vendor = "Samsung";
static const char *gc_0339NAME = "gc_0339";
static const char *gc_0339Size = "5.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", gc_0339Vendor, gc_0339NAME, gc_0339Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_gc_0339;

static int gc_0339_sysfs_init(void)
{
	int ret ;
	pr_info("gc_0339:kobject creat and add\n");
	android_gc_0339 = kobject_create_and_add("android_camera2", NULL);
	if (android_gc_0339 == NULL) {
		pr_info("gc_0339_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("gc_0339:sysfs_create_file\n");
	ret = sysfs_create_file(android_gc_0339, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("gc_0339_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_gc_0339);
	}

	return 0 ;
}

static int32_t gc_0339_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(gc_0339_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gc_0339_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	pr_info("gc_0339_init_module");
	rc = platform_driver_probe(&gc_0339_platform_driver,
		gc_0339_platform_probe);
	if (!rc) {
		gc_0339_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&gc_0339_i2c_driver);
}

static void __exit gc_0339_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gc_0339_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc_0339_s_ctrl);
		platform_driver_unregister(&gc_0339_platform_driver);
	} else
		i2c_del_driver(&gc_0339_i2c_driver);
	return;
}

int32_t gc_0339_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
	
    printk("cleandbg %s %d\n", __func__, __LINE__);
    pr_info("%s: +\n", __func__);

    s_ctrl->power_setting_array.power_setting = gc_0339_power_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(gc_0339_power_setting);
    status = msm_sensor_power_up(s_ctrl);
    pr_info("%s: -\n", __func__);
    printk("cleandbg %s %d\n", __func__, __LINE__);
    return status;
}

int32_t gc_0339_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    int i = 0;
    int j = 0;
    int data_size;
    pr_info("%s: +\n", __func__);
    s_ctrl->power_setting_array.power_setting = gc_0339_power_down_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(gc_0339_power_down_setting);

    
    for(i = 0; i < s_ctrl->power_setting_array.size;  i++)
    {
        data_size = sizeof(gc_0339_power_setting[i].data)/sizeof(void *);
        for (j =0; j < data_size; j++ )
        gc_0339_power_down_setting[i].data[j] = gc_0339_power_setting[i].data[j];
    }
    status = msm_sensor_power_down(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}

int32_t gc_0339_msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
				s_ctrl->sensordata->sensor_name);
		return rc;
	}

	pr_info("%s: read id: %x expected id %x:\n", __func__, chipid,
			s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return 0;
}

static struct msm_sensor_fn_t gc_0339_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = gc_0339_sensor_power_up,
	.sensor_power_down = gc_0339_sensor_power_down,
	.sensor_match_id = gc_0339_msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t gc_0339_s_ctrl = {
	.sensor_i2c_client = &gc_0339_sensor_i2c_client,
	.power_setting_array.power_setting = gc_0339_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc_0339_power_setting),
	.msm_sensor_mutex = &gc_0339_mut,
	.sensor_v4l2_subdev_info = gc_0339_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc_0339_subdev_info),
	.func_tbl = &gc_0339_sensor_func_tbl,
};

module_init(gc_0339_init_module);
module_exit(gc_0339_exit_module);
MODULE_DESCRIPTION("gc_0339");
MODULE_LICENSE("GPL v2");

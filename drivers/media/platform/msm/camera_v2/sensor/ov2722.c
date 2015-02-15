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
#define OV2722_SENSOR_NAME "ov2722"
DEFINE_MSM_MUTEX(ov2722_mut);

static struct msm_sensor_ctrl_t ov2722_s_ctrl;

struct msm_sensor_power_setting ov2722_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 2,
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
		.config_val = GPIO_OUT_LOW,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

struct msm_sensor_power_setting ov2722_power_down_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 2,
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
		.delay = 20,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static int ov2722_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int i;
	uint16_t read_data = 0;
	uint16_t id_addr[4] = {0x3d09,0x3d0a,0x3d0b,0x3d0c};
	static uint16_t id_data[4] = {0,0,0,0};
	static int first=true;

	pr_info("%s called\n", __func__);
	if (!first) {
		cdata->cfg.fuse.fuse_id_word1 = id_data[0];
		cdata->cfg.fuse.fuse_id_word2 = id_data[1];
		cdata->cfg.fuse.fuse_id_word3 = id_data[2];
		cdata->cfg.fuse.fuse_id_word4 = id_data[3];

		pr_info("ov2722: catched fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n", 
			cdata->cfg.fuse.fuse_id_word1,
			cdata->cfg.fuse.fuse_id_word2,
			cdata->cfg.fuse.fuse_id_word3,
			cdata->cfg.fuse.fuse_id_word4);
		return 0;
	}
	first = false;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 1, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: i2c_write failed\n", __func__);
		return rc;
	}
	
	for (i = 0x3d00; i <= 0x3d1f; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_write 0x%x failed\n", __func__, i);
			return rc;
		}
	}
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 1, MSM_CAMERA_I2C_BYTE_DATA);
    msleep(10);

	for (i = 0; i < 4; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, id_addr[i], &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: i2c_read 0x%x failed\n", __func__, i);
			return rc;
		}
		id_data[i] = read_data & 0xff;
	}

	cdata->cfg.fuse.fuse_id_word1 = id_data[0];
	cdata->cfg.fuse.fuse_id_word2 = id_data[1];
	cdata->cfg.fuse.fuse_id_word3 = id_data[2];
	cdata->cfg.fuse.fuse_id_word4 = id_data[3];

	pr_info("ov2722: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n", 
		cdata->cfg.fuse.fuse_id_word1,
		cdata->cfg.fuse.fuse_id_word2,
		cdata->cfg.fuse.fuse_id_word3,
		cdata->cfg.fuse.fuse_id_word4);

	return rc;

}

static struct v4l2_subdev_info ov2722_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov2722_i2c_id[] = {
	{OV2722_SENSOR_NAME, (kernel_ulong_t)&ov2722_s_ctrl},
	{ }
};

static int32_t msm_ov2722_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov2722_s_ctrl);
}

static struct i2c_driver ov2722_i2c_driver = {
	.id_table = ov2722_i2c_id,
	.probe  = msm_ov2722_i2c_probe,
	.driver = {
		.name = OV2722_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2722_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov2722_dt_match[] = {
	{.compatible = "htc,ov2722", .data = &ov2722_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov2722_dt_match);

static struct platform_driver ov2722_platform_driver = {
	.driver = {
		.name = "htc,ov2722",
		.owner = THIS_MODULE,
		.of_match_table = ov2722_dt_match,
	},
};

static const char *ov2722Vendor = "Omnivision";
static const char *ov2722NAME = "ov2722";
static const char *ov2722Size = "2.1M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov2722Vendor, ov2722NAME, ov2722Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov2722;

static int ov2722_sysfs_init(void)
{
	int ret ;
	pr_info("ov2722:kobject creat and add\n");
	android_ov2722 = kobject_create_and_add("android_camera2", NULL);
	if (android_ov2722 == NULL) {
		pr_info("ov2722_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov2722:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov2722, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov2722_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov2722);
	}

	return 0 ;
}

static int32_t ov2722_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov2722_dt_match, &pdev->dev);
	
	if (!match) {
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov2722_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov2722_platform_driver,
		ov2722_platform_probe);
	if (!rc) {
		ov2722_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov2722_i2c_driver);
}

static void __exit ov2722_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov2722_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov2722_s_ctrl);
		platform_driver_unregister(&ov2722_platform_driver);
	} else
		i2c_del_driver(&ov2722_i2c_driver);
	return;
}

int32_t ov2722_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    pr_info("%s: +\n", __func__);
    s_ctrl->power_setting_array.power_setting = ov2722_power_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(ov2722_power_setting);
    status = msm_sensor_power_up(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}
int32_t ov2722_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    int i = 0;
    int j = 0;
    int data_size;
    pr_info("%s: +\n", __func__);
    s_ctrl->power_setting_array.power_setting = ov2722_power_down_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(ov2722_power_down_setting);

    
    for(i = 0; i < s_ctrl->power_setting_array.size;  i++)
    {
        data_size = sizeof(ov2722_power_down_setting[i].data)/sizeof(void *);
        for (j =0; j < data_size; j++ )
        ov2722_power_down_setting[i].data[j] = ov2722_power_setting[i].data[j];
    }

    status = msm_sensor_power_down(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}

static struct msm_sensor_fn_t ov2722_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov2722_sensor_power_up,
	.sensor_power_down = ov2722_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_i2c_read_fuseid = ov2722_read_fuseid,
};

static struct msm_sensor_ctrl_t ov2722_s_ctrl = {
	.sensor_i2c_client = &ov2722_sensor_i2c_client,
	.power_setting_array.power_setting = ov2722_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov2722_power_setting),
	.msm_sensor_mutex = &ov2722_mut,
	.sensor_v4l2_subdev_info = ov2722_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2722_subdev_info),
	.func_tbl = &ov2722_sensor_func_tbl,
};

module_init(ov2722_init_module);
module_exit(ov2722_exit_module);
MODULE_DESCRIPTION("ov2722");
MODULE_LICENSE("GPL v2");

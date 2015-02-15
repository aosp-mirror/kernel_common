/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#define OV8825_SENSOR_NAME "ov8825"
DEFINE_MSM_MUTEX(ov8825_mut);

static struct msm_sensor_ctrl_t ov8825_s_ctrl;

static struct msm_sensor_power_setting ov8825_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov8825_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8825_i2c_id[] = {
	{OV8825_SENSOR_NAME, (kernel_ulong_t)&ov8825_s_ctrl},
	{ }
};

static int32_t msm_ov8825_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8825_s_ctrl);
}

static struct i2c_driver ov8825_i2c_driver = {
	.id_table = ov8825_i2c_id,
	.probe  = msm_ov8825_i2c_probe,
	.driver = {
		.name = OV8825_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8825_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8825_dt_match[] = {
	{.compatible = "qcom,ov8825", .data = &ov8825_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8825_dt_match);

static struct platform_driver ov8825_platform_driver = {
	.driver = {
		.name = "qcom,ov8825",
		.owner = THIS_MODULE,
		.of_match_table = ov8825_dt_match,
	},
};

static int32_t ov8825_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8825_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8825_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8825_platform_driver,
		ov8825_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8825_i2c_driver);
}

static void __exit ov8825_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8825_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8825_s_ctrl);
		platform_driver_unregister(&ov8825_platform_driver);
	} else
		i2c_del_driver(&ov8825_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t ov8825_s_ctrl = {
	.sensor_i2c_client = &ov8825_sensor_i2c_client,
	.power_setting_array.power_setting = ov8825_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8825_power_setting),
	.msm_sensor_mutex = &ov8825_mut,
	.sensor_v4l2_subdev_info = ov8825_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8825_subdev_info),
};

module_init(ov8825_init_module);
module_exit(ov8825_exit_module);
MODULE_DESCRIPTION("ov8825");
MODULE_LICENSE("GPL v2");

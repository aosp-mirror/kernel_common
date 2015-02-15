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
#define imx219_SENSOR_NAME "imx219"
DEFINE_MSM_MUTEX(imx219_mut);

static struct msm_sensor_ctrl_t imx219_s_ctrl;

static struct msm_sensor_power_setting imx219_power_setting[] = {
	{
		.seq_type = SENSOR_VREG_NCP6924,
		.seq_val = NCP6924_VAF,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VCM_PWD,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG_NCP6924,
		.seq_val = NCP6924_VIO,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG_NCP6924,
		.seq_val = NCP6924_VDIG,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG_NCP6924,
		.seq_val = NCP6924_VANA,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info imx219_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx219_i2c_id[] = {
	{imx219_SENSOR_NAME, (kernel_ulong_t)&imx219_s_ctrl},
	{ }
};

static int32_t msm_imx219_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx219_s_ctrl);
}

static struct i2c_driver imx219_i2c_driver = {
	.id_table = imx219_i2c_id,
	.probe  = msm_imx219_i2c_probe,
	.driver = {
		.name = imx219_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx219_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx219_dt_match[] = {
	{
		.compatible = "htc,imx219",
		.data = &imx219_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, imx219_dt_match);

static struct platform_driver imx219_platform_driver = {
	.driver = {
		.name = "htc,imx219",
		.owner = THIS_MODULE,
		.of_match_table = imx219_dt_match,
	},
};

static const char *imx219Vendor = "sony";
static const char *imx219NAME = "imx219";
static const char *imx219Size = "8M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	pr_info("%s called\n", __func__);

	sprintf(buf, "%s %s %s\n", imx219Vendor, imx219NAME, imx219Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_imx219;

static int imx219_sysfs_init(void)
{
	int ret ;
	pr_info("%s: imx219:kobject creat and add\n", __func__);

	android_imx219 = kobject_create_and_add("android_camera", NULL);
	if (android_imx219 == NULL) {
		pr_info("imx219_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("imx219:sysfs_create_file\n");
	ret = sysfs_create_file(android_imx219, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("imx219_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_imx219);
	}

	return 0 ;
}

static int imx219_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int i,j;
	uint16_t read_data = 0;
	static int32_t valid_layer = -1;
	static uint8_t otp[14];
	static int first=true;

	pr_info("%s called\n", __func__);

	if (first) {
		first = false;

		for(j=6;j>0;j--){
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: i2c_write b 0x0100 fail\n", __func__);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x4539, 0xff, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: i2c_write b 0x4539 fail\n", __func__);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3453, 0x0a, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: i2c_write b 0x3453 fail\n", __func__);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3200, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: msm_camera_i2c_write 0x3200 failed\n", __func__);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3202, j--, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: msm_camera_i2c_write 0x3202 failed\n", __func__);

			msleep(10);

			for (i = 0; i < 4; i++) {
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, (0x3204 + i), &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0)
					pr_err("%s: msm_camera_i2c_read 0x%x failed\n", __func__, (0x3204 + i));

				otp[10+i] = (uint8_t)(read_data&0x00FF);
				if (read_data)
					valid_layer = j;
				read_data = 0;
			}

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3202, j, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
				pr_err("%s: msm_camera_i2c_write 0x3202 failed\n", __func__);
			for (i = 0; i < 10; i++) {
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, (0x3204 + i), &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0)
					pr_err("%s: msm_camera_i2c_read 0x%x failed\n", __func__, (0x3204 + i));

				otp[i] = (uint8_t)(read_data&0x00FF);
				if (read_data)
					valid_layer = j;
				read_data = 0;
			}
			if (valid_layer!=-1)
				break;
		}
	}
	pr_info("%s: OTP valid layer = %d\n", __func__,  valid_layer);

	
	cdata->cfg.fuse.fuse_id_word1 = 0;
	cdata->cfg.fuse.fuse_id_word2 = otp[0xA];
	cdata->cfg.fuse.fuse_id_word3 = otp[0xB];
	cdata->cfg.fuse.fuse_id_word4 = otp[0xC];

	
	cdata->af_value.VCM_VENDOR_ID_VERSION = otp[8];
	cdata->af_value.AF_INF_MSB = otp[0];
	cdata->af_value.AF_INF_LSB = otp[1];
	cdata->af_value.AF_MACRO_MSB = otp[2];
	cdata->af_value.AF_MACRO_LSB = otp[3];

	pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[4]);
	pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[5]);
	pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[6]);
	pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[7]);
	pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[8]);

	pr_info("IMX219: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
	   cdata->cfg.fuse.fuse_id_word1,
	   cdata->cfg.fuse.fuse_id_word2,
	   cdata->cfg.fuse.fuse_id_word3,
	   cdata->cfg.fuse.fuse_id_word4);

	pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
	pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
	pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
	pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

	cdata->af_value.VCM_VENDOR = otp[4];

    strlcpy(cdata->af_value.ACT_NAME, "ti201_act", sizeof("ti201_act"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);

	return 0;

}

static int32_t imx219_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx219_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx219_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx219_platform_driver,
		imx219_platform_probe);
	if (!rc) {
		imx219_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx219_i2c_driver);
}

static void __exit imx219_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx219_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx219_s_ctrl);
		platform_driver_unregister(&imx219_platform_driver);
	} else
		i2c_del_driver(&imx219_i2c_driver);
	return;
}

static struct msm_sensor_fn_t imx219_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_i2c_read_fuseid = imx219_read_fuseid,
};

static struct msm_sensor_ctrl_t imx219_s_ctrl = {
	.sensor_i2c_client = &imx219_sensor_i2c_client,
	.power_setting_array.power_setting = imx219_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx219_power_setting),
	.msm_sensor_mutex = &imx219_mut,
	.sensor_v4l2_subdev_info = imx219_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx219_subdev_info),
	.func_tbl = &imx219_sensor_func_tbl,
};

module_init(imx219_init_module);
module_exit(imx219_exit_module);
MODULE_DESCRIPTION("imx219");
MODULE_LICENSE("GPL v2");

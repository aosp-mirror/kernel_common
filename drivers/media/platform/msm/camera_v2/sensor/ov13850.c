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
#define ov13850_SENSOR_NAME "ov13850"
DEFINE_MSM_MUTEX(ov13850_mut);

static struct msm_sensor_ctrl_t ov13850_s_ctrl;

static struct msm_sensor_power_setting ov13850_power_setting[] = {
#ifdef CONFIG_REGULATOR_NCP6924
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
		.seq_val = NCP6924_VANA,
		.config_val = 1,
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
#else
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VCM_PWD,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
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
		.delay = 5,
	},
#endif
};

static struct v4l2_subdev_info ov13850_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov13850_i2c_id[] = {
	{ov13850_SENSOR_NAME, (kernel_ulong_t)&ov13850_s_ctrl},
	{ }
};

static int32_t msm_ov13850_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov13850_s_ctrl);
}

static struct i2c_driver ov13850_i2c_driver = {
	.id_table = ov13850_i2c_id,
	.probe  = msm_ov13850_i2c_probe,
	.driver = {
		.name = ov13850_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov13850_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov13850_dt_match[] = {
	{.compatible = "htc,ov13850", .data = &ov13850_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov13850_dt_match);

static struct platform_driver ov13850_platform_driver = {
	.driver = {
		.name = "htc,ov13850",
		.owner = THIS_MODULE,
		.of_match_table = ov13850_dt_match,
	},
};

static const char *ov13850Vendor = "OmniVision";
static const char *ov13850NAME = "ov13850";
static const char *ov13850Size = "13.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov13850Vendor, ov13850NAME, ov13850Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov13850;

static int ov13850_sysfs_init(void)
{
	int ret ;
	pr_info("ov13850:kobject creat and add\n");
	android_ov13850 = kobject_create_and_add("android_camera", NULL);
	if (android_ov13850 == NULL) {
		pr_info("ov13850_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov13850:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov13850, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov13850_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov13850);
	}

	return 0 ;
}

static int32_t ov13850_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov13850_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov13850_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov13850_platform_driver,
		ov13850_platform_probe);
	if (!rc) {
		ov13850_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov13850_i2c_driver);
}

static void __exit ov13850_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov13850_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov13850_s_ctrl);
		platform_driver_unregister(&ov13850_platform_driver);
	} else
		i2c_del_driver(&ov13850_i2c_driver);
	return;
}

static int ov13850_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	#define OV13850_LITEON_OTP_SIZE 0xF

	const short addr[3][OV13850_LITEON_OTP_SIZE] = {
        
        {0x7220,0x7221,0x7222,0x7223,0x7224,0x7225,0x7226,0x7227,0x7228,0x7229,0x722A,0x722B,0x722C,0x722D,0x722E}, 
        {0x722F,0x7230,0x7231,0x7232,0x7233,0x7234,0x7235,0x7236,0x7237,0x7238,0x7239,0x723A,0x723B,0x723C,0x723D}, 
        {0x723E,0x723F,0x7240,0x7241,0x7242,0x7243,0x7244,0x7245,0x7246,0x7247,0x7248,0x7249,0x724A,0x724B,0x724C}, 
	};
	static uint8_t otp[OV13850_LITEON_OTP_SIZE];
	static int first= true;
	uint16_t read_data = 0;

	int32_t i,j;
	int32_t rc = 0;
	const int32_t offset = 0x00;
	static int32_t valid_layer=-1;
	uint16_t addr_start=0x7000;
	uint16_t addr_end=0x73ff;

	pr_info("%s called\n", __func__);
	if (first) {
		first = false;

		if (rc < 0)
			pr_info("%s: i2c_write recommend settings fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5002, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x5002 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0x40, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x3d84 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, addr_start, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write w 0x3d88 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8a, addr_end, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write w 0x3d8a fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x3d81 fail\n", __func__);

		msleep(10);

		
		for (j=2; j>=0; j--) {
			for (i=0; i<OV13850_LITEON_OTP_SIZE; i++) {
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr[j][i]+offset, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0){
					pr_err("%s: i2c_read 0x%x failed\n", __func__, addr[j][i]);
                    return rc;
				}
				otp[i]= read_data;
				if (read_data)
					valid_layer = j;
			}
			if (valid_layer!=-1)
				break;
		}
		pr_info("%s: OTP valid layer = %d\n", __func__,  valid_layer);


		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5002, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_info("%s: i2c_write b 0x5002 fail\n", __func__);
	}

	
	if (cdata != NULL) {
		cdata->cfg.fuse.fuse_id_word1 = 0;
		cdata->cfg.fuse.fuse_id_word2 = otp[5];
		cdata->cfg.fuse.fuse_id_word3 = otp[6];
		cdata->cfg.fuse.fuse_id_word4 = otp[7];

		
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];
		cdata->af_value.AF_INF_MSB = otp[0x9];
		cdata->af_value.AF_INF_LSB = otp[0xA];
		cdata->af_value.AF_MACRO_MSB = otp[0xB];
		cdata->af_value.AF_MACRO_LSB = otp[0xC];

		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

		pr_info("OV13850: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
		  cdata->cfg.fuse.fuse_id_word1,
		  cdata->cfg.fuse.fuse_id_word2,
		  cdata->cfg.fuse.fuse_id_word3,
		  cdata->cfg.fuse.fuse_id_word4);

		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

		cdata->af_value.VCM_VENDOR = otp[0];

		strlcpy(cdata->af_value.ACT_NAME, "ti201_act", sizeof("ti201_act"));
		pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
	}
	else {
		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	}
	return rc;
}

int32_t ov13850_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0) {
		if(first == 0) {
			pr_info("%s read_fuseid\n",__func__);
			rc1 = ov13850_read_fuseid(NULL, s_ctrl);
		}
	}
	first = 1;
	return rc;
}

static struct msm_sensor_fn_t ov13850_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
    
	.sensor_match_id = ov13850_sensor_match_id,
	
	.sensor_i2c_read_fuseid = ov13850_read_fuseid,
};

static struct msm_sensor_ctrl_t ov13850_s_ctrl = {
	.sensor_i2c_client = &ov13850_sensor_i2c_client,
	.power_setting_array.power_setting = ov13850_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov13850_power_setting),
	.msm_sensor_mutex = &ov13850_mut,
	.sensor_v4l2_subdev_info = ov13850_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov13850_subdev_info),
	.func_tbl = &ov13850_sensor_func_tbl,
};

module_init(ov13850_init_module);
module_exit(ov13850_exit_module);
MODULE_DESCRIPTION("ov13850");
MODULE_LICENSE("GPL v2");

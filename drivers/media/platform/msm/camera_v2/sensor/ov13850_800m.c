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
#define ov13850_800m_SENSOR_NAME "ov13850_800m"
#define DUAL_CAL_OTP_SIZE 1024
static uint8_t otp[21];
static uint8_t otp_mem[DUAL_CAL_OTP_SIZE];
static uint8_t *path= "/data/OTPData.dat";
DEFINE_MSM_MUTEX(ov13850_800m_mut);

static struct msm_sensor_ctrl_t ov13850_800m_s_ctrl;

static struct msm_sensor_power_setting ov13850_800m_power_setting[] = {
#ifdef CONFIG_REGULATOR_NCP6924
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,  
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CHECK_CAMID,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov13850_800m_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov13850_800m_i2c_id[] = {
	{ov13850_800m_SENSOR_NAME, (kernel_ulong_t)&ov13850_800m_s_ctrl},
	{ }
};

static int32_t msm_ov13850_800m_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov13850_800m_s_ctrl);
}

static struct i2c_driver ov13850_800m_i2c_driver = {
	.id_table = ov13850_800m_i2c_id,
	.probe  = msm_ov13850_800m_i2c_probe,
	.driver = {
		.name = ov13850_800m_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov13850_800m_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov13850_800m_dt_match[] = {
	{.compatible = "htc,ov13850_800m", .data = &ov13850_800m_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov13850_800m_dt_match);

static struct platform_driver ov13850_800m_platform_driver = {
	.driver = {
		.name = "htc,ov13850_800m",
		.owner = THIS_MODULE,
		.of_match_table = ov13850_800m_dt_match,
	},
};

static const char *ov13850_800mVendor = "OmniVision";
static const char *ov13850_800mNAME = "ov13850_800m";
static const char *ov13850_800mSize = "13.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov13850_800mVendor, ov13850_800mNAME, ov13850_800mSize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static ssize_t otp_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%02X%02X%02X%02X%02X\n", otp[0],otp[1],otp[2],otp[3],otp[4]);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(otp_info, 0444, otp_info_show, NULL);

static struct kobject *android_ov13850_800m;

static int ov13850_800m_sysfs_init(void)
{
	int ret ;
	pr_info("ov13850_800m:kobject creat and add\n");
	android_ov13850_800m = kobject_create_and_add("android_camera", NULL);
	if (android_ov13850_800m == NULL) {
		pr_info("ov13850_800m_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov13850_800m:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov13850_800m, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov13850_800m_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov13850_800m);
	}

	ret = sysfs_create_file(android_ov13850_800m, &dev_attr_otp_info.attr);
	if (ret) {
		pr_info("ov13850_800m_sysfs_init: sysfs_create_file " \
		"failed\n");
	}

	return 0 ;
}

static int32_t ov13850_800m_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov13850_800m_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov13850_800m_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov13850_800m_platform_driver,
		ov13850_800m_platform_probe);
	if (!rc) {
		ov13850_800m_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov13850_800m_i2c_driver);
}

static void __exit ov13850_800m_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov13850_800m_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov13850_800m_s_ctrl);
		platform_driver_unregister(&ov13850_800m_platform_driver);
	} else
		i2c_del_driver(&ov13850_800m_i2c_driver);
	return;
}

int32_t ov13850_800m_read_otp_memory(uint8_t *otpPtr, struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t addr_start = 0x7220; 
	uint16_t addr_end = 0x72af;   
	uint16_t read_data = 0;
	int32_t i;
	int32_t rc = 0;
	int32_t read_cnt =0;
	read_cnt = addr_end - addr_start + 1;

	pr_info("%s called\n", __func__);

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

	for (i=0; i<read_cnt; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr_start+i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0){
			pr_err("%s: i2c_read 0x%x failed\n", __func__, addr_start+i);
               return rc;
		}
		otp_mem[(addr_start - 0x7000) + i] = read_data;
	}


	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5002, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_info("%s: i2c_write b 0x5002 fail\n", __func__);
	pr_info("%s: read OTP memory done\n", __func__);
	return rc;

}


static void ov13850_800m_parse_otp_mem(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t offset = 0x220;
	int32_t i,j;
	int32_t valid_layer=-1;

	
	for (j=2; j>=0; j--) {
		offset = 0x220;
		for (i=0; i<15; i++) {
			otp[i]= otp_mem[offset + j*15 + i];
			if (otp[i])
				valid_layer = j;
		}

		offset = 0x24D;
		for (i=0; i<6; i++)
			otp[15 + i]= otp_mem[offset + j*6 + i];

		if (valid_layer!=-1)
			break;
	}
	pr_info("%s: OTP valid layer = %d\n", __func__,  valid_layer);

	for(i=0; i<5; i++)
	  s_ctrl->sensordata->sensor_info->OTP_INFO[i] = otp[i];

	s_ctrl->sensordata->sensor_info->fuse_id[0] = otp[5];
	s_ctrl->sensordata->sensor_info->fuse_id[1] = otp[6];
	s_ctrl->sensordata->sensor_info->fuse_id[2] = otp[7];
	s_ctrl->sensordata->sensor_info->fuse_id[3] = otp[13];

	pr_info("%s: OTP Module vendor = 0x%x\n",				__func__,  otp[0]);
	pr_info("%s: OTP LENS = 0x%x\n",						__func__,  otp[1]);
	pr_info("%s: OTP Sensor Version = 0x%x\n",				__func__,  otp[2]);
	pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",	__func__,  otp[3]);
	pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

	pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,	otp[5]);
	pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,	otp[6]);
	pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,	otp[7]);
	pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,	otp[13]);

	pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  otp[9]);
	pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  otp[10]);
	pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",	__func__,  otp[11]);
	pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",	__func__,  otp[12]);

	pr_info("%s: OTP BAIS Calibration data = 0x%x\n",			__func__,  otp[15]);
	pr_info("%s: OTP OFFSET Calibration data = 0x%x\n", 		__func__,  otp[16]);
	pr_info("%s: OTP VCM bottom mech. Limit (MSByte) = 0x%x\n", __func__,  otp[17]);
	pr_info("%s: OTP VCM bottom mech. Limit (LSByte) = 0x%x\n", __func__,  otp[18]);
	pr_info("%s: OTP VCM top mech. Limit (MSByte) = 0x%x\n",	__func__,  otp[19]);
	pr_info("%s: OTP VCM top mech. Limit (LSByte) = 0x%x\n",	__func__,  otp[20]);

}


static int ov13850_800m_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    static int read_otp = true;
    struct file* f;

	if (read_otp)
	{
		f = msm_fopen (path, O_CREAT|O_RDWR|O_TRUNC, 0666);
		if (f) {
			msm_fwrite (f, 0, otp_mem, DUAL_CAL_OTP_SIZE);
			msm_fclose (f);
			pr_info ("%s: dump OTP memory successfully\n", __func__);
		} else {
			pr_err ("%s: fail to open file to write OTP memory\n", __func__);
		}
		read_otp = false;
	}


	if(cdata == NULL){
		return -1;
	}

	cdata->cfg.fuse.fuse_id_word1 = otp[5];
	cdata->cfg.fuse.fuse_id_word2 = otp[6];
	cdata->cfg.fuse.fuse_id_word3 = otp[7];
	cdata->cfg.fuse.fuse_id_word4 = otp[13];

	
	cdata->af_value.AF_INF_MSB = otp[9];
	cdata->af_value.AF_INF_LSB = otp[10];
	cdata->af_value.AF_MACRO_MSB = otp[11];
	cdata->af_value.AF_MACRO_LSB = otp[12];
	cdata->af_value.VCM_BIAS = otp[15];
	cdata->af_value.VCM_OFFSET = otp[16];
	cdata->af_value.VCM_BOTTOM_MECH_MSB = otp[17];
	cdata->af_value.VCM_BOTTOM_MECH_LSB = otp[18];
	cdata->af_value.VCM_TOP_MECH_MSB = otp[19];
	cdata->af_value.VCM_TOP_MECH_LSB = otp[20];

	cdata->lens_id = otp[1];
	cdata->af_value.VCM_VENDOR = otp[0];
	cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
	cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];

	
	strlcpy(cdata->af_value.ACT_NAME, "lc898212_act", sizeof("lc898212_act"));
	if (otp[3] == 0x31)
	    strlcpy(cdata->af_value.ACT_NAME, "ti201_act", sizeof("ti201_act"));

	pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
	pr_info("OTP info %02X %02X %02X %02X %02X fuseid %02X %02X %02X %02X",otp[0],otp[1],otp[2],otp[3],otp[4],otp[5],otp[6],otp[7],otp[13]);

	return 0;
}

int32_t ov13850_800m_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0) {
		if(first == 0) {
			rc1 = ov13850_800m_read_otp_memory(otp_mem, s_ctrl);
			ov13850_800m_parse_otp_mem(s_ctrl);
			if (rc1<0) {
				pr_err("%s: imx214_800m_fov87_read_otp_memory failed %d\n", __func__, rc1);
			} else
				first = 1;
		}
	}
	return rc;
}

static struct msm_sensor_fn_t ov13850_800m_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
    
	.sensor_match_id = ov13850_800m_sensor_match_id,
	
	.sensor_i2c_read_fuseid = ov13850_800m_read_fuseid,
};

static struct msm_sensor_ctrl_t ov13850_800m_s_ctrl = {
	.sensor_i2c_client = &ov13850_800m_sensor_i2c_client,
	.power_setting_array.power_setting = ov13850_800m_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov13850_800m_power_setting),
	.msm_sensor_mutex = &ov13850_800m_mut,
	.sensor_v4l2_subdev_info = ov13850_800m_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov13850_800m_subdev_info),
	.func_tbl = &ov13850_800m_sensor_func_tbl,
};

module_init(ov13850_800m_init_module);
module_exit(ov13850_800m_exit_module);
MODULE_DESCRIPTION("ov13850_800m");
MODULE_LICENSE("GPL v2");

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
#define imx214_SENSOR_NAME "imx214"
#define DUAL_CAL_OTP_SIZE 1024
static uint8_t otp[18];
static uint8_t otp_mem[DUAL_CAL_OTP_SIZE];
DEFINE_MSM_MUTEX(imx214_mut);

static struct msm_sensor_ctrl_t imx214_s_ctrl;

static struct msm_sensor_power_setting imx214_power_setting[] = {
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
		.seq_val = CAM_VDIG,
		.config_val = 1,
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
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 1,
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
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info imx214_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id imx214_i2c_id[] = {
	{imx214_SENSOR_NAME, (kernel_ulong_t)&imx214_s_ctrl},
	{ }
};

static int32_t msm_imx214_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx214_s_ctrl);
}

static struct i2c_driver imx214_i2c_driver = {
	.id_table = imx214_i2c_id,
	.probe  = msm_imx214_i2c_probe,
	.driver = {
		.name = imx214_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx214_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx214_dt_match[] = {
	{.compatible = "htc,imx214", .data = &imx214_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx214_dt_match);

static struct platform_driver imx214_platform_driver = {
	.driver = {
		.name = "htc,imx214",
		.owner = THIS_MODULE,
		.of_match_table = imx214_dt_match,
	},
};

static const char *imx214Vendor = "Sony";
static const char *imx214NAME = "imx214";
static const char *imx214Size = "13M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", imx214Vendor, imx214NAME, imx214Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static ssize_t otp_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%02X%02X%02X%02X%02X\n", otp[3],otp[4],otp[5],otp[6],otp[7]);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(otp_info, 0444, otp_info_show, NULL);

static struct kobject *android_imx214;

static int imx214_sysfs_init(void)
{
	int ret ;
	pr_info("imx214:kobject creat and add\n");
	android_imx214 = kobject_create_and_add("android_camera", NULL);
	if (android_imx214 == NULL) {
		pr_info("imx214_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("imx214:sysfs_create_file\n");
	ret = sysfs_create_file(android_imx214, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("imx214_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_imx214);
	}

	ret = sysfs_create_file(android_imx214, &dev_attr_otp_info.attr);
	if (ret) {
		pr_info("imx214_sysfs_init: sysfs_create_file " \
		"failed\n");
	}

	return 0 ;
}

int32_t imx214_read_otp_memory(uint8_t *otpPtr, struct sensorb_cfg_data *cdata, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t read_data = 0;
	int page = 0, i = 0, j = 0;
	short OTP_addr = 0x0A04;

	
	for (page = 0; page < 16; page++)
	{
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A02, page, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x0A02 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write w 0x0A00 fail\n", __func__);

		for (i = 0; i < 64; i++) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					OTP_addr,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0){
				pr_err("%s: i2c_read 0x%x failed\n", __func__, OTP_addr);
				return rc;
			}
			otpPtr[j] = read_data;
			OTP_addr += 0x1;
			j++;
		}
		OTP_addr = 0x0A04;
	}
	pr_info("%s: read OTP memory done\n", __func__);
	return rc;
}

static int imx214_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    int32_t i = 0, j = 0;
    uint16_t read_data = 0;
    static int first= true;
    int valid_layer = -1;
    
    static int read_otp = true;
    uint8_t *path= "/data/OTPData.dat";
    struct file* f;
    

    if (first)
    {
        first = false;
        for(j = 2; j >= 0; j--)
        {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A02, j, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x0A02 fail\n", __func__);

            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x0A00 fail\n", __func__);

            msleep(10);

            for(i = 0; i < 3; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x0A04 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[OTP]%s: i2c_read 0x%x failed\n", __func__, (0x0A04 + i));
                else
                {
                    otp[i] = read_data;
                    if(read_data)
                        valid_layer = j;
                    read_data = 0;
                }
            }

            for(i = 0; i < 15; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, (0x0A14 + i), &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
                    pr_err("[OTP]%s: i2c_read 0x%x failed\n", __func__, (0x0A14 + i));
                else
                {
                    otp[3+i] = read_data;
                    if(read_data)
                        valid_layer = j;
                    read_data = 0;
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[OTP]%s: valid_layer:%d \n", __func__,valid_layer);
                break;
            }
        }

  for(i=0; i<5; i++)
      s_ctrl->sensordata->sensor_info->OTP_INFO[i] = otp[3+i];

  s_ctrl->sensordata->sensor_info->fuse_id[0] = 0;
  s_ctrl->sensordata->sensor_info->fuse_id[1] = otp[0];
  s_ctrl->sensordata->sensor_info->fuse_id[2] = otp[1];
  s_ctrl->sensordata->sensor_info->fuse_id[3] = otp[2];

        
        pr_info("%s: read OTP for dual cam calibration\n", __func__);
        imx214_read_otp_memory(otp_mem, cdata, s_ctrl);
        if (rc<0) {
            pr_err("%s: imx214_read_otp_memory failed %d\n", __func__, rc);
            return rc;
        }
        
    }

    if(cdata != NULL)
    {
    
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
    

    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[0];
    cdata->cfg.fuse.fuse_id_word3 = otp[1];
    cdata->cfg.fuse.fuse_id_word4 = otp[2];

    cdata->af_value.VCM_VENDOR = otp[3];
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp[7];
    cdata->af_value.VCM_BIAS = otp[8];
    cdata->af_value.VCM_OFFSET = otp[9];
    cdata->af_value.VCM_BOTTOM_MECH_MSB = otp[10];
    cdata->af_value.VCM_BOTTOM_MECH_LSB = otp[11];
    cdata->af_value.AF_INF_MSB = otp[12];
    cdata->af_value.AF_INF_LSB = otp[13];
    cdata->af_value.AF_MACRO_MSB = otp[14];
    cdata->af_value.AF_MACRO_LSB = otp[15];
    cdata->af_value.VCM_TOP_MECH_MSB = otp[16];
    cdata->af_value.VCM_TOP_MECH_LSB = otp[17];

    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[3]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[4]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[5]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[6]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[7]);

    pr_info("%s: OTP fuse 0 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word1);
    pr_info("%s: OTP fuse 1 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word2);
    pr_info("%s: OTP fuse 2 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word3);
    pr_info("%s: OTP fuse 3 = 0x%x\n", __func__,  cdata->cfg.fuse.fuse_id_word4);

    pr_info("%s: OTP BAIS Calibration data = 0x%x\n",           __func__,  cdata->af_value.VCM_BIAS);
    pr_info("%s: OTP OFFSET Calibration data = 0x%x\n",         __func__,  cdata->af_value.VCM_OFFSET);
    pr_info("%s: OTP VCM bottom mech. Limit (MSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_MSB);
    pr_info("%s: OTP VCM bottom mech. Limit (LSByte) = 0x%x\n", __func__,  cdata->af_value.VCM_BOTTOM_MECH_LSB);
    pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
    pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
    pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
    pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);
    pr_info("%s: OTP VCM top mech. Limit (MSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_MSB);
    pr_info("%s: OTP VCM top mech. Limit (LSByte) = 0x%x\n",    __func__,  cdata->af_value.VCM_TOP_MECH_LSB);

    strlcpy(cdata->af_value.ACT_NAME, "lc898212_act", sizeof("lc898212_act"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[3]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[4]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[5]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[6]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[7]);
	}
    return rc;

}

static int32_t imx214_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx214_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx214_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx214_platform_driver,
		imx214_platform_probe);
	if (!rc) {
		imx214_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx214_i2c_driver);
}

static void __exit imx214_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx214_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx214_s_ctrl);
		platform_driver_unregister(&imx214_platform_driver);
	} else
		i2c_del_driver(&imx214_i2c_driver);
	return;
}

int32_t imx214_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t rc1 = 0;
	static int first = 0;
	rc = msm_sensor_match_id(s_ctrl);
	if(rc == 0)
	{
	    if(first == 0)
	    {
	        pr_info("%s read_fuseid\n",__func__);
	        rc1 = imx214_read_fuseid(NULL, s_ctrl);
	        first = 1;
	    }
	}
	return rc;
}

static struct msm_sensor_fn_t imx214_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = imx214_sensor_match_id,
	.sensor_i2c_read_fuseid = imx214_read_fuseid,
};

static struct msm_sensor_ctrl_t imx214_s_ctrl = {
	.sensor_i2c_client = &imx214_sensor_i2c_client,
	.power_setting_array.power_setting = imx214_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx214_power_setting),
	.msm_sensor_mutex = &imx214_mut,
	.sensor_v4l2_subdev_info = imx214_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx214_subdev_info),
	.func_tbl = &imx214_sensor_func_tbl
};

module_init(imx214_init_module);
module_exit(imx214_exit_module);
MODULE_DESCRIPTION("imx214");
MODULE_LICENSE("GPL v2");

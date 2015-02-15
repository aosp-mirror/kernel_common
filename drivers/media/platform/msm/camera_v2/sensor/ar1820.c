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
#define ar1820_SENSOR_NAME "ar1820"
DEFINE_MSM_MUTEX(ar1820_mut);

static struct msm_sensor_ctrl_t ar1820_s_ctrl;

static struct msm_sensor_power_setting ar1820_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
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
};


static struct msm_sensor_power_setting ar1820_MCLK_19M_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
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
		.config_val = 19200000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct v4l2_subdev_info ar1820_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ar1820_i2c_id[] = {
	{ar1820_SENSOR_NAME, (kernel_ulong_t)&ar1820_s_ctrl},
	{ }
};

static int32_t msm_ar1820_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ar1820_s_ctrl);
}

static struct i2c_driver ar1820_i2c_driver = {
	.id_table = ar1820_i2c_id,
	.probe  = msm_ar1820_i2c_probe,
	.driver = {
		.name = ar1820_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ar1820_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ar1820_dt_match[] = {
	{.compatible = "htc,ar1820", .data = &ar1820_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ar1820_dt_match);

static struct platform_driver ar1820_platform_driver = {
	.driver = {
		.name = "htc,ar1820",
		.owner = THIS_MODULE,
		.of_match_table = ar1820_dt_match,
	},
};

static const char *ar1820Vendor = "Aptina";
static const char *ar1820NAME = "ar1820";
static const char *ar1820Size = "18M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ar1820Vendor, ar1820NAME, ar1820Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ar1820;

static int ar1820_sysfs_init(void)
{
	int ret ;
	pr_info("ar1820:kobject creat and add\n");
	android_ar1820 = kobject_create_and_add("android_camera", NULL);
	if (android_ar1820 == NULL) {
		pr_info("ar1820_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ar1820:sysfs_create_file\n");
	ret = sysfs_create_file(android_ar1820, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ar1820_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ar1820);
	}

	return 0 ;
}

#define OTP_LAYER_0_SIZE 10
#define OTP_LAYER_1_SIZE 4
static int ar1820_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    int32_t i = 0, j = 0;
    uint16_t read_data = 0;
    static uint8_t otp_layer_0_data[OTP_LAYER_0_SIZE];
    static uint8_t otp_layer_1_data[OTP_LAYER_1_SIZE];
    static int first= true;
    uint16_t layer_0[3] = {0x3400, 0x3200, 0x3000};
    uint16_t layer_1[3] = {0x3500, 0x3300, 0x3100};
    int OTP_ready = 0;
    int valid_layer = -1;

    if (first)
    {
        // Initialize OTPM for reading
        // disable streaming
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x301A, 0x0218, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("[OTP]%s: i2c_write w 0x301A fail\n", __func__);

        // timing parameters for OTP read
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3134, 0xCD95, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("[OTP]%s: i2c_write w 0x3134 fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3054, 0x0400, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            pr_info("[OTP]%s: i2c_write w 0x3054 fail\n", __func__);

        for(j = 0; j < 3; j++)
        {
            // choose record type
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x304C, layer_0[j], MSM_CAMERA_I2C_WORD_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x304C rec 0x%x fail\n", __func__, layer_0[j]);

            // auto read start
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x304A, 0x0210, MSM_CAMERA_I2C_WORD_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x304C fail\n", __func__);

            OTP_ready = 0;
            //POLL 0x304A bits 5 and 6 for read end and success
            for(i = 0; i < 5; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x304A, &read_data, MSM_CAMERA_I2C_WORD_DATA);//0x37c2
                if (rc < 0)
                    pr_err("[OTP]%s: (%d)i2c_read 0x304A failed\n", __func__, i);
                else
                {
                    if((read_data & 0x60)==0x60)
                    {
                         pr_info("[OTP]%s:(%d) read w 0x304C:0x%x, OTP ready\n", __func__,i, read_data);
                         OTP_ready = 1;
                         break;
                    }
                }
                msleep(1);
            }

            if(OTP_ready == 0)
            {
                pr_info("[OTP]%s: 0x%x, OTP not ready\n", __func__, layer_0[j]);
                continue;
            }

            //Read OTPM data
            for(i = 0; i < OTP_LAYER_0_SIZE; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x3800 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);//0x37c2
                if (rc < 0)
                    pr_err("[OTP]%s: (%d)i2c_read 0x%x failed\n", __func__, i, 0x3800 + i);
                else
                {
                    otp_layer_0_data[i] = read_data;
                    if(read_data)
                    {
                        valid_layer = j;
                    }
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[OTP]%s: valid_layer:%d \n", __func__,valid_layer);
                break;
            }
        }


        for(j = 0; j < 3; j++)
        {
            // choose record type
            if(valid_layer != -1)
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x304C, layer_1[valid_layer], MSM_CAMERA_I2C_WORD_DATA);
            else
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x304C, layer_1[j], MSM_CAMERA_I2C_WORD_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x304C rec 0x%x fail\n", __func__, layer_1[j]);

            // auto read start
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x304A, 0x0210, MSM_CAMERA_I2C_WORD_DATA);
            if (rc < 0)
                pr_info("[OTP]%s: i2c_write w 0x304C fail\n", __func__);

            OTP_ready = 0;
            //POLL 0x304A bits 5 and 6 for read end and success
            for(i = 0; i < 5; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x304A, &read_data, MSM_CAMERA_I2C_WORD_DATA);//0x37c2
                if (rc < 0)
                    pr_err("[OTP]%s: (%d)i2c_read 0x304A failed\n", __func__, i);
                else
                {
                    if((read_data & 0x60)==0x60)
                    {
                         pr_info("[OTP]%s:(%d) read w 0x304C:0x%x, OTP ready\n", __func__,i, read_data);
                         OTP_ready = 1;
                         break;
                    }
                }
                    msleep(1);
            }

            if(OTP_ready == 0)
            {
                pr_info("[OTP]%s: 0x%x, OTP not ready\n", __func__, layer_1[j]);
                continue;
            }

            for(i = 0; i < OTP_LAYER_1_SIZE; i++)
            {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x3800 + i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);//0x37c2
                if (rc < 0)
                    pr_err("[OTP]%s: (%d)i2c_read 300%x failed\n", __func__, i, i);
                else
                {
                    otp_layer_1_data[i] = read_data;
                    if(read_data && valid_layer == -1)
                    {
                        valid_layer = j;
                    }
                }
            }

            if(valid_layer != -1)
            {
                pr_info("[OTP]%s: valid_layer:%d \n", __func__,valid_layer);
                first = false;
                break;
            }
        }

        if(otp_layer_0_data[4] == 0x41) //workaround for Optical, check VCM ID to decide MCLK frequency
        {
            pr_info("%s: modify MLCK to 19.2M\n",__func__);
            ar1820_s_ctrl.power_setting_array.power_setting = ar1820_MCLK_19M_power_setting;
        }
    }

    // fuseid
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = 0;
    cdata->cfg.fuse.fuse_id_word3 = 0;
    cdata->cfg.fuse.fuse_id_word4 = 0;

    // vcm
    cdata->af_value.VCM_BIAS = 0;
    cdata->af_value.VCM_OFFSET = 0;
    cdata->af_value.VCM_BOTTOM_MECH_MSB = 0;
    cdata->af_value.VCM_BOTTOM_MECH_LSB = 0;
    cdata->af_value.AF_INF_MSB = otp_layer_0_data[0];
    cdata->af_value.AF_INF_LSB = otp_layer_0_data[1];
    cdata->af_value.AF_MACRO_MSB = otp_layer_0_data[2];
    cdata->af_value.AF_MACRO_LSB = otp_layer_0_data[3];
    cdata->af_value.VCM_TOP_MECH_MSB = 0;
    cdata->af_value.VCM_TOP_MECH_LSB = 0;
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp_layer_0_data[4];
    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp_layer_0_data[6]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp_layer_0_data[8]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp_layer_0_data[7]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp_layer_0_data[5]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp_layer_0_data[4]);

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

    cdata->af_value.VCM_VENDOR = otp_layer_0_data[6];

    strlcpy(cdata->af_value.ACT_NAME, "ti201_act", sizeof("ti201_act"));
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
    return rc;

}
/*HTC_END*/


static int32_t ar1820_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ar1820_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ar1820_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ar1820_platform_driver,
		ar1820_platform_probe);
	if (!rc) {
		ar1820_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ar1820_i2c_driver);
}

static void __exit ar1820_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ar1820_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ar1820_s_ctrl);
		platform_driver_unregister(&ar1820_platform_driver);
	} else
		i2c_del_driver(&ar1820_i2c_driver);
	return;
}

static struct msm_sensor_fn_t ar1820_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_i2c_read_fuseid = ar1820_read_fuseid,
};

static struct msm_sensor_ctrl_t ar1820_s_ctrl = {
	.sensor_i2c_client = &ar1820_sensor_i2c_client,
	.power_setting_array.power_setting = ar1820_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ar1820_power_setting),
	.msm_sensor_mutex = &ar1820_mut,
	.sensor_v4l2_subdev_info = ar1820_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ar1820_subdev_info),
	.func_tbl = &ar1820_sensor_func_tbl
};

module_init(ar1820_init_module);
module_exit(ar1820_exit_module);
MODULE_DESCRIPTION("ar1820");
MODULE_LICENSE("GPL v2");

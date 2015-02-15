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
#include <mach/devices_cmdline.h>
#define OV4688_SENSOR_NAME "ov4688"
#include <linux/completion.h>
#ifdef CONFIG_RAWCHIPII
#include "yushanII.h"
#include "ilp0100_ST_api.h"
#include "ilp0100_customer_sensor_config.h"
#endif

int GPIO_Camera_Sync = 85;

static struct work_struct ov4688_work;
struct msm_rawchip2_cfg_data cfg_data;
void ov4688_do_restart_stream(struct work_struct *ws);
atomic_t restart_is_running;
DEFINE_MSM_MUTEX(ov4688_mut);

static struct msm_sensor_ctrl_t ov4688_s_ctrl;

static struct msm_sensor_power_setting ov4688_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 1,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 1,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
#if 0 
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VCM_PWD,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct v4l2_subdev_info ov4688_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov4688_i2c_id[] = {
	{OV4688_SENSOR_NAME, (kernel_ulong_t)&ov4688_s_ctrl},
	{ }
};

static int32_t msm_ov4688_i2c_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
       return msm_sensor_i2c_probe(client, id, &ov4688_s_ctrl);
}

static struct i2c_driver ov4688_i2c_driver = {
	.id_table = ov4688_i2c_id,
	.probe  = msm_ov4688_i2c_probe,
	.driver = {
		.name = OV4688_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov4688_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov4688_dt_match[] = {
	{.compatible = "htc,ov4688", .data = &ov4688_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov4688_dt_match);

static struct platform_driver ov4688_platform_driver = {
	.driver = {
		.name = "htc,ov4688",
		.owner = THIS_MODULE,
		.of_match_table = ov4688_dt_match,
	},
};


static const char *ov4688Vendor = "ov";
static const char *ov4688NAME = "ov4688";
static const char *ov4688Size = "4M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	pr_info("%s called\n", __func__);

	sprintf(buf, "%s %s %s\n", ov4688Vendor, ov4688NAME, ov4688Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_ov4688;

static int ov4688_sysfs_init(void)
{
	int ret ;
	pr_info("%s: ov4688:kobject creat and add\n", __func__);

	android_ov4688 = kobject_create_and_add("android_camera", NULL);
	if (android_ov4688 == NULL) {
		pr_info("ov4688_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("ov4688:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov4688, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("ov4688_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_ov4688);
	}

	return 0 ;
}

int32_t vd4688_read_otp_memory(uint8_t *otpPtr, struct sensorb_cfg_data *cdata, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	const int32_t offset = 0x7000;
	uint16_t read_data = 0;
	int i=0;
	short OTP_addr=0x0;

	
		for (i=0; i<512; ++i) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					OTP_addr+offset,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0){
				pr_err("%s: i2c_read 0x%x failed\n", __func__, OTP_addr);
				return rc;
			}
			otpPtr[i] = read_data;
			OTP_addr += 0x1;
		}

		pr_info("%s: read OTP memory done\n", __func__);
	

	return rc;
}

static int ov4688_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
    #define OV4688_LITEON_OTP_SIZE 0x12

    const short addr[3][OV4688_LITEON_OTP_SIZE] = {
        
        {0x126,0x127,0x128,0x129,0x12a,0x110,0x111,0x112,0x12b,0x12c,0x11e,0x11f,0x120,0x121,0x122,0x123,0x124,0x125}, 
        {0x144,0x145,0x146,0x147,0x148,0x12e,0x12f,0x130,0x149,0x14a,0x13c,0x13d,0x13e,0x13f,0x140,0x141,0x142,0x143}, 
        {0x162,0x163,0x164,0x165,0x166,0x14c,0x14d,0x14e,0x167,0x168,0x15a,0x15b,0x15c,0x15d,0x15e,0x15f,0x160,0x161}, 
    };
    static uint8_t otp[OV4688_LITEON_OTP_SIZE];
	static int first= true;
	static int read_vcm = true;
	static int read_otp = true;
	uint8_t *path= "/data/OTPData.dat";
	static uint8_t otp_mem[512];
	struct file* f;
	uint16_t read_data = 0;

    int32_t i,j;
    int32_t rc = 0;
    const int32_t offset = 0x7000;
    static int32_t valid_layer=-1;
    uint16_t addr_start=0x7000;
    uint16_t addr_end=0x71ff;
#if defined(CONFIG_ACT_OIS_BINDER)
	extern void HtcActOisBinder_set_OIS_OTP(uint8_t *otp_data, uint8_t otp_size);

	#define LITEON_OIS_OTP_SIZE 34

	const static short ois_addr[3][LITEON_OIS_OTP_SIZE] = {
	    
	    {0x16A,0x16B,0x16C,0x16D,0x16E,0x16F,0x170,0x171,0x172,0x173,0x174,0x175,0x176,0x177,0x178,0x179,0x17A,0x17B,0x17C,0x17D,0x17D,0x17F,0x180,0x181,0x124,0x125,0x12B,0x12C,0x1B2,0x1B3,0x1B4,0x1B5,0x1B6,0x1B7}, 
	    {0x182,0x183,0x184,0x185,0x186,0x187,0x188,0x189,0x18A,0x18B,0x18C,0x18D,0x18E,0x18F,0x190,0x191,0x192,0x193,0x194,0x195,0x196,0x197,0x198,0x199,0x142,0x143,0x149,0x14A,0x1B8,0x1B9,0x1BA,0x1BB,0x1BC,0x1BD}, 
	    {0x19A,0x19B,0x19C,0x19D,0x19E,0x19F,0x1A0,0x1A1,0x1A2,0x1A3,0x1A4,0x1A5,0x1A6,0x1A7,0x1A8,0x1A9,0x1AA,0x1AB,0x1AC,0x1AD,0x1AE,0x1AF,0x1B0,0x1B1,0x160,0x161,0x167,0x168,0x1BE,0x1BF,0x1C0,0x1C1,0x1C2,0x1C3}, 
	};
    int32_t ois_valid_layer=-1;
    static uint8_t ois_otp[LITEON_OIS_OTP_SIZE];
#endif
	struct vcm_driver_ic_info {
		uint8_t driver_ic;
		const char *act_name;
	};

	static struct vcm_driver_ic_info ov4688_vcm_driver_ic_info[] = {
		{0x21, "ti201_act"},
		{0x11, "lc898212_act"},
		{0x01, "rumbas_act"},
	};
	if (first || read_vcm) {
	    first = false;

        

        if (rc < 0)
            pr_info("%s: i2c_write recommend settings fail\n", __func__);

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);

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

        
        for (j=2; j>=0; --j) {
            for (i=0; i<OV4688_LITEON_OTP_SIZE; ++i) {
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

        ov4688_s_ctrl.driver_ic = otp[3];

        
        pr_err("%s: read OTP 0x%x~0x%x\n", __func__, addr_start, addr_end);
        
        
        {
            rc = vd4688_read_otp_memory(otp_mem, cdata, s_ctrl);
            if (rc<0) {
                pr_err("%s: vd4688_read_otp_memory failed %d\n", __func__, rc);
                return rc;
            }
        }
        

    #if defined(CONFIG_ACT_OIS_BINDER)
        if(ov4688_s_ctrl.driver_ic == 0x21) {
        
        for (j=2; j>=0; --j) {
            for (i=0; i<LITEON_OIS_OTP_SIZE; ++i) {
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                        s_ctrl->sensor_i2c_client,
                        ois_addr[j][i]+offset,
                        &read_data,
                        MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0){
                    pr_err("%s: i2c_read 0x%x failed\n", __func__, ois_addr[j][i]);
                    return rc;
                }
                

                ois_otp[i]= read_data;

                if (read_data)
                    ois_valid_layer = j;
            }
            if (ois_valid_layer!=-1)
                break;
        }
        pr_info("%s: OTP OIS valid layer = %d\n", __func__,  ois_valid_layer);

        if (ois_valid_layer!=-1) {
            for(i=0; i<LITEON_OIS_OTP_SIZE;i ++)
                pr_info("read out OTP OIS data = 0x%x\n", ois_otp[i]);

            HtcActOisBinder_set_OIS_OTP(ois_otp, LITEON_OIS_OTP_SIZE);
        }
        }
    #endif
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            pr_info("%s: i2c_write b 0x0100 fail\n", __func__);
    }

    if (board_mfg_mode())
        msm_dump_otp_to_file (OV4688_SENSOR_NAME, addr[valid_layer], otp, sizeof(otp));

    
    if(cdata != NULL)
    {
        if (read_otp)
        {
            f = msm_fopen (path, O_CREAT|O_RDWR|O_TRUNC, 0666);
            if (f) {
                msm_fwrite (f,0,otp_mem,512);
                msm_fclose (f);
                pr_info ("%s: dump OTP memory successfully\n", __func__);
            } else {
                pr_err ("%s: fail to open file to write OTP memory\n", __func__);
            }
            read_otp = false;
        }
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = otp[5];
    cdata->cfg.fuse.fuse_id_word3 = otp[6];
    cdata->cfg.fuse.fuse_id_word4 = otp[7];

    
    cdata->af_value.VCM_BIAS = otp[8];
    cdata->af_value.VCM_OFFSET = otp[9];
    cdata->af_value.VCM_BOTTOM_MECH_MSB = otp[0xa];
    cdata->af_value.VCM_BOTTOM_MECH_LSB = otp[0xb];
    cdata->af_value.AF_INF_MSB = otp[0xc];
    cdata->af_value.AF_INF_LSB = otp[0xd];
    cdata->af_value.AF_MACRO_MSB = otp[0xe];
    cdata->af_value.AF_MACRO_LSB = otp[0xf];
    cdata->af_value.VCM_TOP_MECH_MSB = otp[0x10];
    cdata->af_value.VCM_TOP_MECH_LSB = otp[0x11];
    cdata->af_value.VCM_VENDOR_ID_VERSION = otp[4];
    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);

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

    cdata->af_value.VCM_VENDOR = otp[0];

    for(i=0; i<sizeof(ov4688_vcm_driver_ic_info)/sizeof(struct vcm_driver_ic_info); i++) {
        if(ov4688_s_ctrl.driver_ic == ov4688_vcm_driver_ic_info[i].driver_ic) {
            strlcpy(cdata->af_value.ACT_NAME,
                ov4688_vcm_driver_ic_info[i].act_name,
                sizeof(cdata->af_value.ACT_NAME));
            break;
        }
    }
    pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
    read_vcm = 0;
	}
	else
	{
	    pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  otp[0]);
	    pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  otp[1]);
	    pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  otp[2]);
	    pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  otp[3]);
	    pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  otp[4]);
	    for(i=0; i<sizeof(ov4688_vcm_driver_ic_info)/sizeof(struct vcm_driver_ic_info); i++) {
	    if(ov4688_s_ctrl.driver_ic == ov4688_vcm_driver_ic_info[i].driver_ic) {
	        read_vcm = 0;
	        pr_info("%s: got VCM ID\n",               __func__);
	        break;
	    }
    }
	}
	return rc;
}

static int32_t ov4688_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov4688_dt_match, &pdev->dev);
	
	if (!match) {
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov4688_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov4688_platform_driver,
		ov4688_platform_probe);
	if (!rc) {
		ov4688_sysfs_init();
		INIT_WORK(&ov4688_work, ov4688_do_restart_stream);
		atomic_set(&restart_is_running, 0);
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov4688_i2c_driver);
}

static void __exit ov4688_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov4688_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov4688_s_ctrl);
		platform_driver_unregister(&ov4688_platform_driver);
	} else
		i2c_del_driver(&ov4688_i2c_driver);
	return;
}

static struct msm_camera_i2c_reg_array ov4688_recommend_settings[] = {

    {0x0103, 0x01},
    {0x3638, 0x00},
    {0x0300, 0x00},
    {0x0302, 0x30}, 
    {0x0303, 0x01}, 
    {0x0304, 0x03},
    {0x030b, 0x00},
    {0x030d, 0x1e},
    {0x030e, 0x04},
    {0x030f, 0x01},
    {0x0312, 0x01},
    {0x031e, 0x00},
    {0x3000, 0x20},
    {0x3002, 0x00},
    {0x3018, 0x72},
    {0x3020, 0x93},
    {0x3021, 0x03}, 
    {0x3022, 0x01},
    {0x3031, 0x0a},
    {0x3305, 0xf1},
    {0x3307, 0x04},
    {0x3309, 0x29},
    {0x3500, 0x00},
    {0x3501, 0x60},
    {0x3502, 0x00},
    {0x3503, 0x04}, 
    {0x3504, 0x00},
    {0x3505, 0x00},
    {0x3506, 0x00},
    {0x3507, 0x00},
    {0x3508, 0x00},
    {0x3509, 0x80},
    {0x350a, 0x00},
    {0x350b, 0x00},
    {0x350c, 0x00},
    {0x350d, 0x00},
    {0x350e, 0x00},
    {0x350f, 0x80},
    {0x3510, 0x00},
    {0x3511, 0x00},
    {0x3512, 0x00},
    {0x3513, 0x00},
    {0x3514, 0x00},
    {0x3515, 0x80},
    {0x3516, 0x00},
    {0x3517, 0x00},
    {0x3518, 0x00},
    {0x3519, 0x00},
    {0x351a, 0x00},
    {0x351b, 0x80},
    {0x351c, 0x00},
    {0x351d, 0x00},
    {0x351e, 0x00},
    {0x351f, 0x00},
    {0x3520, 0x00},
    {0x3521, 0x80},
    {0x3522, 0x08},
    {0x3524, 0x08},
    {0x3526, 0x08},
    {0x3528, 0x08},
    {0x352a, 0x08},
    {0x3602, 0x00},

    {0x3603, 0x01},    

    {0x3604, 0x02},
    {0x3605, 0x00},
    {0x3606, 0x00},
    {0x3607, 0x00},
    {0x3609, 0x12},
    {0x360a, 0x40},
    {0x360c, 0x08},
    {0x360f, 0xe5},
    {0x3608, 0x8f},
    {0x3611, 0x00},
    {0x3613, 0xf7},
    {0x3616, 0x58},
    {0x3619, 0x99},
    {0x361b, 0x60},
    {0x361c, 0x7a},
    {0x361e, 0x79},
    {0x361f, 0x02},
    {0x3633, 0x10},
    {0x3634, 0x10},
    {0x3635, 0x10},
    {0x3636, 0x15},
    {0x3646, 0x86},
    {0x364a, 0x0b},
    {0x3700, 0x17},
    {0x3701, 0x22},
    {0x3703, 0x10},
    {0x370a, 0x37},
    {0x3705, 0x00},
    {0x3706, 0x63},
    {0x3709, 0x3c},
    {0x370b, 0x01},
    {0x370c, 0x30},
    {0x3710, 0x24},
    {0x3711, 0x0c},
    {0x3716, 0x00},
    {0x3720, 0x28},
    {0x3729, 0x7b},
    {0x372a, 0x84},
    {0x372b, 0xbd},
    {0x372c, 0xbc},
    {0x372e, 0x52},
    {0x373c, 0x0e},
    {0x373e, 0x33},
    {0x3743, 0x10},
    {0x3744, 0x88},
    {0x374a, 0x43},
    {0x374c, 0x00},
    {0x374e, 0x23},
    {0x3751, 0x7b},
    {0x3752, 0x84},
    {0x3753, 0xbd},
    {0x3754, 0xbc},
    {0x3756, 0x52},
    {0x375c, 0x00},
    {0x3760, 0x00},
    {0x3761, 0x00},
    {0x3762, 0x00},
    {0x3763, 0x00},
    {0x3764, 0x00},
    {0x3767, 0x04},
    {0x3768, 0x04},
    {0x3769, 0x08},
    {0x376a, 0x08},
    {0x376c, 0x00},
    {0x376d, 0x00},
    {0x376e, 0x00},
    {0x3773, 0x00},
    {0x3774, 0x51},
    {0x3776, 0xbd},
    {0x3777, 0xbd},

    {0x3781, 0x18},
    {0x3783, 0x25},
    {0x3841, 0x02},
    {0x3846, 0x08},
    {0x3847, 0x07},
    {0x3d85, 0x36},
    {0x3d8c, 0x71},
    {0x3d8d, 0xcb},
    {0x3f0a, 0x00},
    {0x4000, 0x71},
    {0x4002, 0x04},
    {0x4003, 0x14},

    {0x4004, 0x01},
    {0x4005, 0x00},

    {0x400e, 0x00},
    {0x4011, 0x00},
    {0x401a, 0x00},
    {0x401b, 0x00},
    {0x401c, 0x00},
    {0x401d, 0x00},
    {0x401f, 0x00},
    {0x4020, 0x00},
    {0x4021, 0x10},
    {0x4028, 0x00},
    {0x4029, 0x02},
    {0x402a, 0x06},
    {0x402b, 0x04},
    {0x402c, 0x02},
    {0x402d, 0x02},
    {0x402e, 0x0e},
    {0x402f, 0x04},
    {0x4302, 0xff},
    {0x4303, 0xff},
    {0x4304, 0x00},
    {0x4305, 0x00},
    {0x4306, 0x00},
    {0x4308 , 0x03}, 

    {0x4500, 0x6c},
    {0x4501, 0xc4},
    {0x4503, 0x02},
    {0x4800, 0x04},
    {0x4813, 0x08},
    {0x481f, 0x40},
    {0x4829, 0x78},
    {0x4837, 0x1c},
    {0x4b00, 0x2a},
    {0x4b0d, 0x00},
    {0x4d00, 0x04},
    {0x4d01, 0x42}, 
    {0x4d02, 0xd1},
    {0x4d03, 0x93},
    {0x4d04, 0xf5},
    {0x4d05, 0xc1},
    {0x5000, 0xf3},
    {0x5001, 0x11},
    {0x5004, 0x00},
    {0x500a, 0x00},
    {0x500b, 0x00},
    {0x5032, 0x00},

    
    {0x5500, 0x00},
    {0x5501, 0x10},
    {0x5502, 0x01},
    {0x5503, 0x0f},
    

    {0x5040, 0x00},
    {0x8000, 0x00},
    {0x8001, 0x00},
    {0x8002, 0x00},
    {0x8003, 0x00},
    {0x8004, 0x00},
    {0x8005, 0x00},
    {0x8006, 0x00},
    {0x8007, 0x00},
    {0x8008, 0x00},
    {0x3638, 0x00},
    {0x3105, 0x31},
    {0x301a, 0xf9}, 

    
    {0x484b, 0x05},
    {0x4805, 0x03},
    {0x3508, 0x07},
    {0x3601, 0x01},
    {0x3603, 0x01},
    
};

static  struct msm_camera_i2c_reg_setting init_settings = {
  .reg_setting = ov4688_recommend_settings,
  .size = ARRAY_SIZE(ov4688_recommend_settings),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array start_reg_array[] = {
	{0x0100, 0x01},

	{0x301a, 0xf9},
	{0xffff, 10},
	{0x301a, 0xf1},
	{0x4805, 0x00},
	{0x301a, 0xf0},
};

static struct msm_camera_i2c_reg_array start_reg_array2[] = {
	{0x3105, 0x11},
	{0x301a, 0xf1}, 
	{0x4805, 0x00}, 
	{0x301a, 0xf0},
	{0x3208, 0x00},
	{0x302a, 0x00},
	{0x302a, 0x00},
	{0x302a, 0x00},
	{0x302a, 0x00},
	{0x302a, 0x00},
	{0x3601, 0x00},
	{0x3638, 0x00},
	{0x3208, 0x10},
	{0x3208, 0xa0},
};



static  struct msm_camera_i2c_reg_setting start_settings = {
  .reg_setting = start_reg_array,
  .size = ARRAY_SIZE(start_reg_array),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static  struct msm_camera_i2c_reg_setting start_settings2 = {
  .reg_setting = start_reg_array2,
  .size = ARRAY_SIZE(start_reg_array2),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array stop_reg_array[] = {
	{0x0100, 0x00},
    {0xffff, 100},
	{0x301a, 0xf9},
	{0x4805, 0x03},
};

static struct msm_camera_i2c_reg_setting stop_settings = {
  .reg_setting = stop_reg_array,
  .size = ARRAY_SIZE(stop_reg_array),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

struct completion raw2_stream_on_complete;
int raw2_wait_frame_ms = 600;

void ov4688_stop_restart_stream(void)
{
    pr_info("%s:\n", __func__);
    complete(&raw2_stream_on_complete);
}


void ov4688_do_restart_stream(struct work_struct *ws)
{
    int rc =0;

	if (atomic_read(&restart_is_running)){
		pr_info("%s: last restart is running %d, abort\n", __func__, atomic_read(&restart_is_running));
		return;
	}
	pr_info("%s: wait %d ms\n", __func__, raw2_wait_frame_ms);
	atomic_set(&restart_is_running, 1);

#ifdef CONFIG_RAWCHIPII
	rc = wait_for_completion_interruptible_timeout(
		&raw2_stream_on_complete,
		msecs_to_jiffies(raw2_wait_frame_ms));
	if (rc == 0) {
		pr_err("%s: wait timeout\n", __func__);
	} else {
		atomic_set(&restart_is_running, 0);
		pr_err("%s: abort\n", __func__);
		return;
	}

	mutex_lock(ov4688_s_ctrl.msm_sensor_mutex);

	if (ov4688_s_ctrl.sensor_state != MSM_SENSOR_POWER_UP) {
		atomic_set(&restart_is_running, 0);
		mutex_unlock(ov4688_s_ctrl.msm_sensor_mutex);
		pr_info("%s: quit in sensor_state %d\n", __func__, ov4688_s_ctrl.sensor_state);
		return;
	}

	if(YushanII_Get_reloadInfo() == 0){
		pr_info("%s: stop YushanII first", __func__);
		Ilp0100_stop();
	}
	ov4688_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_write_table(
		ov4688_s_ctrl.sensor_i2c_client, &stop_settings);
	mdelay(30);

	YushanII_Reset();

	YushanII_Init(&ov4688_s_ctrl,&cfg_data);
	mdelay(10);

	ov4688_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_write_table(
		ov4688_s_ctrl.sensor_i2c_client, &start_settings);
	mutex_unlock(ov4688_s_ctrl.msm_sensor_mutex);
#endif
    atomic_set(&restart_is_running, 0);
	pr_info("%s: X\n", __func__);
}

void ov4688_restart_stream(void)
{
	pr_info("%s:\n", __func__);
	schedule_work(&ov4688_work);
}

int32_t ov4688_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;


	switch (cdata->cfgtype) {
	case CFG_SET_START_STREAM:
		mutex_lock(s_ctrl->msm_sensor_mutex);
		pr_info("%s:%d CFG_SET_START_STREAM ++\n", __func__, __LINE__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &start_settings);

		mdelay(10);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &start_settings2);
		pr_info("%s:%d CFG_SET_START_STREAM --\n", __func__, __LINE__);
		mutex_unlock(s_ctrl->msm_sensor_mutex);

		break;

	case CFG_POWER_UP:
		pr_info("%s:%d CFG_POWER_UP\n", __func__, __LINE__);
		#ifdef CONFIG_RAWCHIPII
		YushanII_set_sensor_specific_function(s_ctrl->func_tbl);
		#endif
		rc = msm_sensor_config(s_ctrl, argp);
		break;

	case CFG_POWER_DOWN:
		pr_info("%s:%d CFG_POWER_DOWN\n", __func__, __LINE__);
		rc = msm_sensor_config(s_ctrl, argp);
		break;

#ifdef CONFIG_RAWCHIPII
	case CFG_RAWCHIPII_SETTING:
		if (s_ctrl->sensordata->htc_image != 1) 
			break;

		mutex_lock(s_ctrl->msm_sensor_mutex);
		if (copy_from_user(&cfg_data, (void *)cdata->cfg.setting,
			sizeof(struct msm_rawchip2_cfg_data))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			mutex_unlock(s_ctrl->msm_sensor_mutex);
			break;
		}
		if (atomic_read(&restart_is_running)){
			complete(&raw2_stream_on_complete);
			pr_info("%s: last restart is running %d, stop it\n", __func__, atomic_read(&restart_is_running));
		}
		init_completion(&raw2_stream_on_complete);

		pr_info("%s: CFG_RAWCHIPII_SETTING", __func__);
		YushanII_Init(s_ctrl,&cfg_data);
		mutex_unlock(s_ctrl->msm_sensor_mutex);
		break;

	case CFG_RAWCHIPII_STOP:
		if (s_ctrl->sensordata->htc_image != 1) 
			break;

		mutex_lock(s_ctrl->msm_sensor_mutex);
		if (atomic_read(&restart_is_running)){
			complete(&raw2_stream_on_complete);
			pr_info("%s: last restart is running %d, stop it\n", __func__, atomic_read(&restart_is_running));
		}

		if(YushanII_Get_reloadInfo() == 0){
			pr_info("%s: stop YushanII first", __func__);
			Ilp0100_stop();
		}
		mutex_unlock(s_ctrl->msm_sensor_mutex);

		break;
#endif
	default:
		rc = msm_sensor_config(s_ctrl, argp);
		break;
	}


	return rc;
}
int32_t ov4688_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
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
	        rc1 = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(s_ctrl->sensor_i2c_client, &init_settings);
	        mdelay(5);
	        rc1 = ov4688_read_fuseid(NULL, s_ctrl);
	    }
	}
	first = 1;
	return rc;
}

int32_t ov4688_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    long rc = 0;
    pr_info("%s: +\n", __func__);
    rc = gpio_request(GPIO_Camera_Sync, "OV4688");
    if (rc < 0) {
    pr_err("GPIO(%d) request failed\n", GPIO_Camera_Sync);
    }
    status = msm_sensor_power_up(s_ctrl);
    pr_info("%s: -\n", __func__);
    return status;
}

int32_t ov4688_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    pr_info("%s: +\n", __func__);
    status = msm_sensor_power_down(s_ctrl);
    gpio_free(GPIO_Camera_Sync);
    pr_info("%s: -\n", __func__);
    return status;
}

static struct msm_sensor_fn_t ov4688_sensor_func_tbl = {
	.sensor_config = ov4688_sensor_config,
	.sensor_power_up = ov4688_sensor_power_up,
	.sensor_power_down = ov4688_sensor_power_down,
	.sensor_match_id = ov4688_sensor_match_id,
	.sensor_i2c_read_fuseid = ov4688_read_fuseid,
	.sensor_yushanII_restart_stream = ov4688_restart_stream,
	.sensor_yushanII_stop_restart_stream = ov4688_stop_restart_stream,
};

static struct msm_sensor_ctrl_t ov4688_s_ctrl = {
	.sensor_i2c_client = &ov4688_sensor_i2c_client,
	.power_setting_array.power_setting = ov4688_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov4688_power_setting),
	.msm_sensor_mutex = &ov4688_mut,
	.sensor_v4l2_subdev_info = ov4688_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov4688_subdev_info),
	.func_tbl = &ov4688_sensor_func_tbl, 
};

module_init(ov4688_init_module);
module_exit(ov4688_exit_module);
MODULE_DESCRIPTION("ov4688");
MODULE_LICENSE("GPL v2");

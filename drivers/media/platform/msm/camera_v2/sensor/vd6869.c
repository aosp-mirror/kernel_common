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
#define VD6869_SENSOR_NAME "vd6869"
DEFINE_MSM_MUTEX(vd6869_mut);

static struct msm_sensor_ctrl_t vd6869_s_ctrl;

static struct msm_sensor_power_setting vd6869_power_setting[] = {
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

static struct v4l2_subdev_info vd6869_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id vd6869_i2c_id[] = {
	{VD6869_SENSOR_NAME, (kernel_ulong_t)&vd6869_s_ctrl},
	{ }
};

static int32_t msm_vd6869_i2c_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
       return msm_sensor_i2c_probe(client, id, &vd6869_s_ctrl);
}

static struct i2c_driver vd6869_i2c_driver = {
	.id_table = vd6869_i2c_id,
	.probe  = msm_vd6869_i2c_probe,
	.driver = {
		.name = VD6869_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client vd6869_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id vd6869_dt_match[] = {
	{.compatible = "htc,vd6869", .data = &vd6869_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, vd6869_dt_match);

static struct platform_driver vd6869_platform_driver = {
	.driver = {
		.name = "htc,vd6869",
		.owner = THIS_MODULE,
		.of_match_table = vd6869_dt_match,
	},
};

#define VD6869_VER_UNKNOWN 0xFF

struct vd6869_ver_map {
	uint8_t val;
	char *str;
};

static struct vd6869_ver_map vd6869_ver_tab[] = {  
	{ 0x09, "(0.9)"},	
	{ 0x0A, "(0.9e)"},	
	{ 0x10, "(1.0)"},	
	{ VD6869_VER_UNKNOWN, "(unknown)"}  
};

static uint8_t vd6869_ver = VD6869_VER_UNKNOWN;
static uint8_t vd6869_year_mon = 0;
static uint8_t vd6869_date = 0;

static const char *vd6869Vendor = "st";
static const char *vd6869NAME = "vd6869";
static const char *vd6869Size = "cinesensor";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	uint8_t i = 0;
	uint8_t len = ARRAY_SIZE(vd6869_ver_tab);
	char vd6869NAME_ver[32];
	uint16_t year = 0;
	uint8_t month = 0;
	uint8_t date = 0;
	pr_info("%s called\n", __func__);

	memset(vd6869NAME_ver, 0, sizeof(vd6869NAME_ver));
	for (i = 0; i < len; i++) {
		if (vd6869_ver == vd6869_ver_tab[i].val)
			break;
	}
	if (i < len)  
		snprintf(vd6869NAME_ver, sizeof(vd6869NAME_ver), "%s%s",
			vd6869NAME, vd6869_ver_tab[i].str);
	else  
		snprintf(vd6869NAME_ver, sizeof(vd6869NAME_ver), "%s%s-%02X",
			vd6869NAME, vd6869_ver_tab[len - 1].str, vd6869_ver);
	pr_info("%s: version(%d) : %s\n", __func__, vd6869_ver, vd6869NAME_ver);

	year  = ((vd6869_year_mon & 0xf0)>> 4);
	month = (vd6869_year_mon & 0x0f);
	date  = ((vd6869_date & 0xf8) >> 3);

	if((year == 0)&&(month == 0)&&(date == 0))
		pr_err("%s: Invalid OTP date\n", __func__);
	else
		year += 2000; 

	snprintf(buf, PAGE_SIZE, "%s %s %s %04d-%02d-%02d \n", vd6869Vendor, vd6869NAME_ver, vd6869Size, year, month, date);
	  
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_vd6869;

static int vd6869_sysfs_init(void)
{
	int ret ;
	pr_info("%s: vd6869:kobject creat and add\n", __func__);

	android_vd6869 = kobject_create_and_add("android_camera", NULL);
	if (android_vd6869 == NULL) {
		pr_info("vd6869_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("vd6869:sysfs_create_file\n");
	ret = sysfs_create_file(android_vd6869, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("vd6869_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_vd6869);
	}

	return 0 ;
}

struct vcm_driver_ic_info {
	uint8_t driver_ic;
	const char *act_name;
};

static struct vcm_driver_ic_info vd6869_vcm_driver_ic_info[] = {
	{0x21, "ti201_act"},	
	{0x01, "rumbas_act"},	
};


#define OTP_BUFFER_OFFSET 0x33FA
#define OTP_STATUS_REG 0x3302
#define OTP_WAIT_TIMEOUT 200
#define LITEON_OTP_SIZE 0x12

const static short liteon_otp_addr[3][LITEON_OTP_SIZE] = {
    
    {0x3C8,0x3C9,0x3CA,0x3CB,0x3CC,0x3A0,0x3A1,0x3A2,0x3CD,0x3CE,0x3C0,0x3C1,0x3C2,0x3C3,0x3C4,0x3C5,0x3C6,0x3C7}, 
    {0x3D8,0x3D9,0x3DA,0x3DB,0x3DC,0x380,0x381,0x382,0x3DD,0x3DE,0x3D0,0x3D1,0x3D2,0x3D3,0x3D4,0x3D5,0x3D6,0x3D7}, 
    {0x3B8,0x3B9,0x3BA,0x3BB,0x3BC,0x388,0x389,0x38A,0x3BD,0x3BE,0x3B0,0x3B1,0x3B2,0x3B3,0x3B4,0x3B5,0x3B6,0x3B7}, 
};

#if defined(CONFIG_ACT_OIS_BINDER)
extern void HtcActOisBinder_set_OIS_OTP(uint8_t *otp_data, uint8_t otp_size);

#define LITEON_OIS_OTP_SIZE 34
const static short ois_addr[3][LITEON_OIS_OTP_SIZE] = {
    
    {0x090,0x091,0x092,0x093,0x094,0x095,0x096,0x097,0x098,0x099,0x09A,0x09B,0x09C,0x09D,0x09E,0x09F,0x0A0,0x0A1,0x0A2,0x0A3,0x0A4,0x0A5,0x0A6,0x0A7,0x0A8,0x0A9,0x0AA,0x0AB,0x0AC,0x0AD,0x0AE,0x0AF,0x0F0,0x0F4}, 
    {0x0B0,0x0B1,0x0B2,0x0B3,0x0B4,0x0B5,0x0B6,0x0B7,0x0B8,0x0B9,0x0BA,0x0BB,0x0BC,0x0BD,0x0BE,0x0BF,0x0C0,0x0C1,0x0C2,0x0C3,0x0C4,0x0C5,0x0C6,0x0C7,0x0C8,0x0C9,0x0CA,0x0CB,0x0CC,0x0CD,0x0CE,0x0CF,0x0F8,0x0F9}, 
    {0x0D0,0x0D1,0x0D2,0x0D3,0x0D4,0x0D5,0x0D6,0x0D7,0x0D8,0x0D9,0x0DA,0x0DB,0x0DC,0x0DD,0x0DE,0x0DF,0x0E0,0x0E1,0x0E2,0x0E3,0x0E4,0x0E5,0x0E6,0x0E7,0x0E8,0x0E9,0x0EA,0x0EB,0x0EC,0x0ED,0x0EE,0x0EF,0x0FC,0x0FD}, 
};
#endif

static struct msm_camera_i2c_reg_array otp_reg_settings[] = {
    {0x44c0, 0x01},
    {0x4500, 0x01},
    {0x44e4, 0x00},
    {0x4524, 0x00},
    {0x4584, 0x01},
    {0x44ec, 0x01},
    {0x44ed, 0x80},
    {0x44f0, 0x04},
    {0x44f1, 0xb0},
    {0x452c, 0x01},
    {0x452d, 0x80},
    {0x4530, 0x04},
    {0x4531, 0xb0},
	{0x3305, 0x00},
    {0x3303, 0x01},
	{0x3304, 0x00},
    {0x3301, 0x02},
};

static  struct msm_camera_i2c_reg_setting otp_settings = {
  .reg_setting = otp_reg_settings,
  .size = ARRAY_SIZE(otp_reg_settings),
  .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
  .cmd_type = MSM_CAMERA_I2C_COMMAND_WRITE,
};

static int vd6869_shut_down_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data){
	int rc=0,i;
	for(i = 0; i < OTP_WAIT_TIMEOUT;i++){
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client,
				addr,
				data,
				MSM_CAMERA_I2C_BYTE_DATA);

		if(rc < 0){
			pr_err("%s shut down OTP error 0x%x:0x%x",__func__,addr,data);
		}else{
			pr_info("%s shut down OTP success 0x%x:0x%x",__func__,addr,data);
			return rc;
		}
		mdelay(1);
	}
	pr_err("%s shut down time out 0x%x",__func__,addr);
	return rc;
}

static int vd6869_init_otp(struct msm_sensor_ctrl_t *s_ctrl){
	int i,rc = 0;
	uint16_t read_data = 0;

	
	for(i = 0 ;i < OTP_WAIT_TIMEOUT; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client,
				&otp_settings);

		if(rc < 0)
			pr_err("%s write otp init table error.... retry", __func__);
		else{
			pr_info("%s OTP table init done",__func__);
			break;
		}
		mdelay(1);
	}

	
	for(i = 0 ;i < OTP_WAIT_TIMEOUT; i++){
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				OTP_STATUS_REG,
				&read_data,
				MSM_CAMERA_I2C_BYTE_DATA);

		if(rc < 0){
			pr_err("%s read OTP status error",__func__);
		} else if(read_data == 0x00){
			rc = vd6869_shut_down_otp(s_ctrl,0x4584,0x00);
			if(rc < 0)
				return rc;
			rc = vd6869_shut_down_otp(s_ctrl,0x44c0,0x00);
			if(rc < 0)
				return rc;
			rc = vd6869_shut_down_otp(s_ctrl,0x4500,0x00);
			if(rc < 0)
				return rc;
			break;
		}
		mdelay(1);
	}
	return rc;
}

int32_t vd6869_read_otp_valid_layer(struct msm_sensor_ctrl_t *s_ctrl, int8_t *valid_layer, bool first)
{
	int32_t rc = 0;
	const int32_t offset = OTP_BUFFER_OFFSET;
	uint16_t read_data = 0;
	int j=2,i=0;
	static int8_t layer=-1;
	if (first) {
		for (j=2; j>=0; --j) {
			for (i=0; i<LITEON_OTP_SIZE; ++i) {
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,
						liteon_otp_addr[j][i]+offset, 
						&read_data,
						MSM_CAMERA_I2C_BYTE_DATA);
				if (rc < 0){
					pr_err("%s: i2c_read 0x%x failed\n", __func__, liteon_otp_addr[j][i]);
					return rc;
				}
				

				if (read_data) {
					layer = j;
					break;
				}
			}
			if (layer!=-1)
				break;
		}
	}

	*valid_layer = layer;
	pr_info("%s: OTP valid layer = %d\n", __func__,  *valid_layer);
	return rc;
}

int32_t vd6869_read_otp_memory(struct sensorb_cfg_data *cdata, struct msm_sensor_ctrl_t *s_ctrl, bool first)
{
	int32_t rc = 0;
	const int32_t offset = OTP_BUFFER_OFFSET;
	uint16_t read_data = 0;
	int i=0;
	short OTP_addr=0x0;
	static uint8_t otp[1024];
	uint8_t *path= "/data/OTPData.dat";
	struct file* f;

	if (first) {
		for (i=0; i<1024; ++i) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					OTP_addr+offset,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0){
				pr_err("%s: i2c_read 0x%x failed\n", __func__, OTP_addr);
				return rc;
			}
			otp[i] = read_data;
			OTP_addr += 0x1;
		}

		f = msm_fopen (path, O_CREAT|O_RDWR|O_TRUNC, 0666);
		if (f) {
			msm_fwrite (f,0,otp,1024);
			msm_fclose (f);
		} else {
			pr_err ("%s: fail to open file\n", __func__);
		}

		pr_info("%s: read OTP memory done\n", __func__);
	}

	return rc;
}

static int vd6869_read_module_vendor(struct msm_sensor_ctrl_t *s_ctrl, uint8_t valid_layer, uint8_t* module_vendor, uint8_t* driver_ic, bool first)
{
	int32_t rc = 0;
	uint16_t read_data = 0;
	static uint8_t moduler=0,driver=0;
	const int32_t offset = OTP_BUFFER_OFFSET;
	int16_t retry_count = 0;
	int16_t retry_max = 10;

	if (first) {
		do {
			if (retry_count > 0) {
				rc = vd6869_init_otp(s_ctrl);
				if (rc<0)
					return rc;
			}

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					liteon_otp_addr[valid_layer][0]+offset,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				pr_err("%s: i2c_read 0x%x failed\n", __func__, liteon_otp_addr[valid_layer][0]+offset);
				return rc;
			}
			moduler = (uint8_t)(read_data&0x00FF);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					liteon_otp_addr[valid_layer][3]+offset,
					&read_data,
					MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				pr_err("%s: i2c_read 0x%x failed\n", __func__, liteon_otp_addr[valid_layer][3]+offset);
				return rc;
			}
			driver = (uint8_t)(read_data&0x00FF);

			if (driver == 0 || moduler == 0) {
				pr_err("OTP read error : driver=0x%x, moduler=0x%x  Apply retry mechanism  retry_count=%d\n", driver, moduler, retry_count);
				retry_count++;
				msleep(10);
			}

		} while ((driver == 0 || moduler == 0) && (retry_count <= retry_max));

	}

	*module_vendor = moduler;
	*driver_ic = driver;
	vd6869_s_ctrl.driver_ic = driver;
	pr_info("module_vendor = 0x%x\n", *module_vendor);
	pr_info("driver_ic = 0x%x\n", *driver_ic);
	return rc;
}

int vd6869_read_fuseid_liteon(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl, bool first, uint8_t valid_layer)
{
	int32_t i;
	int32_t rc = 0;
	const int32_t offset = OTP_BUFFER_OFFSET;

    static uint8_t otp[LITEON_OTP_SIZE];
	uint16_t read_data = 0;
#if defined(CONFIG_ACT_OIS_BINDER)
	int32_t j, ois_valid_layer=-1;
	static uint8_t ois_otp[LITEON_OIS_OTP_SIZE];
#endif

	if (first) {
        for (i=0; i<LITEON_OTP_SIZE; ++i) {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                    s_ctrl->sensor_i2c_client,
                    liteon_otp_addr[valid_layer][i]+offset,
                    &read_data,
                    MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0){
                pr_err("%s: i2c_read 0x%x failed\n", __func__, liteon_otp_addr[valid_layer][i]);
                return rc;
            }
            

            otp[i]= read_data;
        }
        pr_info("%s: OTP valid layer = %d\n", __func__,  valid_layer);

    #if defined(CONFIG_ACT_OIS_BINDER)
        
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

        	HtcActOisBinder_set_OIS_OTP(ois_otp, LITEON_OIS_OTP_SIZE);
        }
    #endif
    }
    
    vd6869_ver = otp[2]; 
    cdata->sensor_ver = otp[2];

    if (board_mfg_mode())
        msm_dump_otp_to_file (VD6869_SENSOR_NAME, liteon_otp_addr[valid_layer], otp, sizeof(otp));

    
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

    return 0;
}

int vd6869_read_fuseid_sharp(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl, bool first, uint8_t valid_layer)
{
    #define SHARP_OTP_SIZE 0x12
    #define SHARP_OTP_LAYER 3
    const static short sharp_otp_addr[SHARP_OTP_LAYER][SHARP_OTP_SIZE] = {
        
        {0x3A0,0x3A1,0x3C0,0x3C1,0x3C2,0x3C3,0x3C4,0x3C5,0x3C6,0x3C7,0x3C8,0x3C9,0x3CA,0x3CB,0x3CC,0x3CD,0x3CE,0x3CF},
        {0x380,0x381,0x3D0,0x3D1,0x3D2,0x3D3,0x3D4,0x3D5,0x3D6,0x3D7,0x3D8,0x3D9,0x3DA,0x3DB,0x3DC,0x3DD,0x3DE,0x3DF},
        {0x388,0x389,0x3B0,0x3B1,0x3B2,0x3B3,0x3B4,0x3B5,0x3B6,0x3B7,0x3B8,0x3B9,0x3BA,0x3BB,0x3BC,0x3BD,0x3BE,0x3BF}
    };
    #define SHARP_FUSEID_SIZE 0xc
    const static short sharp_fuseid_addr[SHARP_FUSEID_SIZE]={0x3f4,0x3f5,0x3f6,0x3f7,0x3f8,0x3f9,0x3fa,0x3fb,0x3fc,0x3fd,0x3fe,0x3ff};

    int32_t rc = 0;
    int i;
    uint16_t read_data = 0;
    static uint8_t otp[SHARP_OTP_SIZE+SHARP_FUSEID_SIZE];
    const int32_t offset = OTP_BUFFER_OFFSET;

    if (first) {
        for (i=0; i<SHARP_OTP_SIZE; ++i) {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                    s_ctrl->sensor_i2c_client,
                    sharp_otp_addr[valid_layer][i]+offset,
                    &read_data,
                    MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0){
                pr_err("%s: i2c_read 0x%x failed\n", __func__, sharp_otp_addr[valid_layer][i]);
                return rc;
            }
            

            otp[i]= read_data;
        }
        pr_info("%s: OTP valid layer = %d\n", __func__,  valid_layer);
        
        for (i=0;i<SHARP_FUSEID_SIZE;++i) {
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                sharp_fuseid_addr[i]+offset,
                &read_data,
                MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0){
               pr_err("%s: i2c_read 0x%x failed\n", __func__, sharp_fuseid_addr[i]);
               return rc;
            }
            

            otp[i+SHARP_OTP_SIZE]= read_data;
        }
    }

    if (board_mfg_mode()) {
        if (valid_layer >= SHARP_OTP_LAYER)
            pr_err("%s:%d\n", __func__, __LINE__);
        else
            msm_dump_otp_to_file (VD6869_SENSOR_NAME, sharp_otp_addr[valid_layer], otp, sizeof (otp));
    }

    vd6869_year_mon = otp[0];
    vd6869_date = otp[1];

    cdata->sensor_ver = vd6869_ver = otp[0xc];
    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 =
        (otp[0x12]<<24) |
        (otp[0x13]<<16) |
        (otp[0x14]<<8) |
        (otp[0x15]);
    cdata->cfg.fuse.fuse_id_word3 =
        (otp[0x16]<<24) |
        (otp[0x17]<<16) |
        (otp[0x18]<<8) |
        (otp[0x19]);
    cdata->cfg.fuse.fuse_id_word4 =
        (otp[0x1a]<<24) |
        (otp[0x1b]<<16) |
        (otp[0x1c]<<8) |
        (otp[0x1d]);
    
    cdata->af_value.VCM_START_MSB = otp[2];
    cdata->af_value.VCM_START_LSB = otp[3];
    cdata->af_value.AF_INF_MSB = otp[4];
    cdata->af_value.AF_INF_LSB = otp[5];
    cdata->af_value.AF_MACRO_MSB = otp[6];
    cdata->af_value.AF_MACRO_LSB = otp[7];
    cdata->af_value.ACT_ID = otp[0xe];

    pr_info("vd6869_year_mon=0x%x\n", vd6869_year_mon);
    pr_info("vd6869_date=0x%x\n", vd6869_date);
    pr_info("%s: VenderID=%x, LensID=%x, SensorID=%02x, DriverId=%02x\n", __func__,
        otp[0xa], otp[0xb], otp[0xc], otp[0xd]);
    pr_info("%s: ModuleFuseID= %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", __func__,
        otp[0x12], otp[0x13], otp[0x14], otp[0x15], otp[0x16], otp[0x17], otp[0x18], otp[0x19], otp[0x1a], otp[0x1b], otp[0x1c], otp[0x1d]);

    pr_info("vd6869: fuse->fuse_id_word1:%x\n",cdata->cfg.fuse.fuse_id_word1);
    pr_info("vd6869: fuse->fuse_id_word2:0x%08x\n",cdata->cfg.fuse.fuse_id_word2);
    pr_info("vd6869: fuse->fuse_id_word3:0x%08x\n",cdata->cfg.fuse.fuse_id_word3);
    pr_info("vd6869: fuse->fuse_id_word4:0x%08x\n",cdata->cfg.fuse.fuse_id_word4);

    pr_info("vd6869: VCM START:0x%02x\n", cdata->af_value.VCM_START_MSB << 8 |cdata->af_value.VCM_START_LSB);
    pr_info("vd6869: Infinity position:0x%02x\n", cdata->af_value.AF_INF_MSB << 8 | cdata->af_value.AF_INF_LSB);
    pr_info("vd6869: Macro position:0x%02x\n", cdata->af_value.AF_MACRO_MSB << 8 | cdata->af_value.AF_MACRO_LSB);

    pr_info("VCM_START_MSB =0x%x\n", cdata->af_value.VCM_START_MSB);
    pr_info("VCM_START_LSB =0x%x\n", cdata->af_value.VCM_START_LSB);
    pr_info("AF_INF_MSB =0x%x\n", cdata->af_value.AF_INF_MSB);
    pr_info("AF_INF_LSB =0x%x\n", cdata->af_value.AF_INF_LSB);
    pr_info("AF_MACRO_MSB =0x%x\n", cdata->af_value.AF_MACRO_MSB);
    pr_info("AF_MACRO_LSB =0x%x\n", cdata->af_value.AF_MACRO_LSB);
    pr_info("ACT_ID =0x%x\n", cdata->af_value.ACT_ID);

    return 0;
}

static int vd6869_read_fuseid(struct sensorb_cfg_data *cdata,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	uint8_t module_vendor;
	uint8_t driver_ic;
	static bool first=true;
	int8_t valid_layer = -1;
    uint8_t i; 

	if (first) {
		rc = vd6869_init_otp(s_ctrl);
		if (rc<0)
			return rc;

		rc = vd6869_read_otp_valid_layer (s_ctrl,&valid_layer,first);
		if (rc<0) {
			pr_err("%s: failed %d\n", __func__, rc);
			first = false;
			return rc;
		}
	}

	if (valid_layer < 0)
		valid_layer = 0;

	rc = vd6869_read_module_vendor (s_ctrl, valid_layer, &module_vendor, &driver_ic, first);
	if (rc<0) {
		pr_err("%s: failed %d\n", __func__, rc);
		first = false;
		return rc;
	}
	cdata->af_value.VCM_VENDOR = module_vendor;

	for(i=0; i<sizeof(vd6869_vcm_driver_ic_info)/sizeof(struct vcm_driver_ic_info); i++) {
		if(driver_ic == vd6869_vcm_driver_ic_info[i].driver_ic) {
			strlcpy(cdata->af_value.ACT_NAME,
					vd6869_vcm_driver_ic_info[i].act_name,
					sizeof(cdata->af_value.ACT_NAME));
			break;
		}
	}
	pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);

	
	
	{
		rc = vd6869_read_otp_memory(cdata, s_ctrl, first);
		if (rc<0) {
			pr_err("%s: vd6869_read_otp_memory failed %d\n", __func__, rc);
			return rc;
		}
	}

	switch (module_vendor) {
		case 0x1:
			rc = vd6869_read_fuseid_sharp (cdata, s_ctrl, first, valid_layer);
			break;
		case 0x2:
		    rc = vd6869_read_fuseid_liteon (cdata, s_ctrl, first, valid_layer);
			break;
		default:
			pr_err("%s unknown module vendor = 0x%x\n",__func__, module_vendor);
			break;
	}

	first = false;
	return rc;
}

static int32_t vd6869_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(vd6869_dt_match, &pdev->dev);
	
	if (!match) {
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init vd6869_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&vd6869_platform_driver,
		vd6869_platform_probe);
	if (!rc) {
		vd6869_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&vd6869_i2c_driver);
}

static void __exit vd6869_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (vd6869_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&vd6869_s_ctrl);
		platform_driver_unregister(&vd6869_platform_driver);
	} else
		i2c_del_driver(&vd6869_i2c_driver);
	return;
}

static struct msm_sensor_fn_t vd6869_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_i2c_read_fuseid = vd6869_read_fuseid,
};

static struct msm_sensor_ctrl_t vd6869_s_ctrl = {
	.sensor_i2c_client = &vd6869_sensor_i2c_client,
	.power_setting_array.power_setting = vd6869_power_setting,
	.power_setting_array.size = ARRAY_SIZE(vd6869_power_setting),
	.msm_sensor_mutex = &vd6869_mut,
	.sensor_v4l2_subdev_info = vd6869_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(vd6869_subdev_info),
	.func_tbl = &vd6869_sensor_func_tbl, 
};

module_init(vd6869_init_module);
module_exit(vd6869_exit_module);
MODULE_DESCRIPTION("vd6869");
MODULE_LICENSE("GPL v2");

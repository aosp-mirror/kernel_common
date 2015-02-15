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
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include <linux/regulator/consumer.h>

#define MI380_SENSOR_NAME "mi380"
DEFINE_MSM_MUTEX(mi380_mut);

#define SENSOR_SUCCESS 0
static struct msm_camera_i2c_client mi380_sensor_i2c_client;
static uint32_t mi380_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level);
static uint32_t mi380_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level);
static uint32_t mi380_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level);
static uint32_t mi380_set_wb(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level);
static uint32_t mi380_SetEffect(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level);

#define CHECK_STATE_TIME 100
static int just_power_on = 1;
static int op_mode = 0;
static struct i2c_client mi380_client_t; 
static struct i2c_client *mi380_client = &mi380_client_t;
static struct msm_sensor_ctrl_t mi380_s_ctrl;
enum mi380_width {
	WORD_LEN,
	BYTE_LEN
};

struct mi380_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum mi380_width width;
	unsigned short mdelay_time;
};
static int mi380_i2c_write(unsigned short saddr,
				 unsigned short waddr, unsigned short wdata,
				 enum mi380_width width);
static int32_t mi380_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata);

static int mi380_i2c_write_reg(unsigned short waddr, unsigned short wdata)
{
	int rc = -EIO;



	rc = mi380_sensor_i2c_client.i2c_func_tbl->i2c_write(&mi380_sensor_i2c_client, waddr, wdata, MSM_CAMERA_I2C_WORD_DATA);

	if (rc < 0)
		pr_info("i2c_write failed, addr = 0x%x, val = 0x%x!\n", waddr, wdata);

	return rc;
}

enum {
	CAMERA_EFFECT_NONE = 0,
	CAMERA_EFFECT_MONO,
	CAMERA_EFFECT_RED,
	CAMERA_EFFECT_GREEN,
	CAMERA_EFFECT_BLUE,
	CAMERA_EFFECT_YELLOW,
	CAMERA_EFFECT_NEGATIVE,
	CAMERA_EFFECT_SEPIA,
	CAMERA_EFFECT_AQUA,
	CAMERA_EFFECT_MAX
};

static struct msm_camera_i2c_reg_array m_wb_auto[] = {
	{0x098C, 0x2306},
	{0x0990, 0x03C0},
	{0x098C, 0x2308},
	{0x0990, 0xFD7C},
	{0x098C, 0x230A},
	{0x0990, 0xFFF7},
	{0x098C, 0x230C},
	{0x0990, 0xFF25},
	{0x098C, 0x230E},
	{0x0990, 0x0384},
	{0x098C, 0x2310},
	{0x0990, 0xFFD6},
	{0x098C, 0x2312},
	{0x0990, 0xFED2},
	{0x098C, 0x2314},
	{0x0990, 0xFCB2},
	{0x098C, 0x2316},
	{0x0990, 0x068E},
	{0x098C, 0x2318},
	{0x0990, 0x001B},
	{0x098C, 0x231A},
	{0x0990, 0x0039},
	{0x098C, 0x231C},
	{0x0990, 0xFF65},
	{0x098C, 0x231E},
	{0x0990, 0x0052},
	{0x098C, 0x2320},
	{0x0990, 0x0012},
	{0x098C, 0x2322},
	{0x0990, 0x0007},
	{0x098C, 0x2324},
	{0x0990, 0xFFCF},
	{0x098C, 0x2326},
	{0x0990, 0x0037},
	{0x098C, 0x2328},
	{0x0990, 0x00DB},
	{0x098C, 0x232A},
	{0x0990, 0x01C8},
	{0x098C, 0x232C},
	{0x0990, 0xFC9F},
	{0x098C, 0x232E},
	{0x0990, 0x0010},
	{0x098C, 0x2330},
	{0x0990, 0xFFF3},
	
	{0x098C, 0xA34A},
	{0x0990, 0x0059},
	{0x098C, 0xA34B},
	{0x0990, 0x00E6},
	{0x098C, 0xA34C},
	{0x0990, 0x0059},
	{0x098C, 0xA34D},
	{0x0990, 0x00E6},
	{0x098C, 0xA351},
	{0x0990, 0x0000},
	{0x098C, 0xA352},
	{0x0990, 0x007F},
};

static struct msm_camera_i2c_reg_array m_wb_fluorescent[] = {
	{0x098C, 0xA353},
	{0x0990, 0x0043},
	{0x098C, 0xA34E},
	{0x0990, 0x00A0},
	{0x098C, 0xA34F},
	{0x0990, 0x0086},
	{0x098C, 0xA350},
	{0x0990, 0x008A}
};


static struct msm_camera_i2c_reg_array m_wb_incandescent[] = {
	{0x098C, 0xA353},
	{0x0990, 0x000B},
	{0x098C, 0xA34E},
	{0x0990, 0x0090},
	{0x098C, 0xA34F},
	{0x0990, 0x0085},
	{0x098C, 0xA350},
	{0x0990, 0x00A0}
};

static struct msm_camera_i2c_reg_array m_wb_daylight[] = {
	{0x098C, 0xA353},
	{0x0990, 0x007F},
	{0x098C, 0xA34E},
	{0x0990, 0x00A2},
	{0x098C, 0xA34F},
	{0x0990, 0x0085},
	{0x098C, 0xA350},
	{0x0990, 0x0080}
};

static struct msm_camera_i2c_reg_array m_wb_cloudy[] = {
	{0x098C, 0xA353},
	{0x0990, 0x007F},
	{0x098C, 0xA34E},
	{0x0990, 0x00B2},
	{0x098C, 0xA34F},
	{0x0990, 0x0095},
	{0x098C, 0xA350},
	{0x0990, 0x0060}
};

enum wb_mode{
	CAMERA_AWB_AUTO = 0,
	CAMERA_AWB_INDOOR_HOME = 2,
	CAMERA_AWB_INDOOR_OFFICE = 3,
	CAMERA_AWB_SUNNY = 5,
	CAMERA_AWB_CLOUDY = 6,
};

static struct msm_camera_i2c_reg_array contract_setup_tb_m0[] = {
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0023},
	{0x098C, 0xAB3E},
	{0x0990, 0x0045},
	{0x098C, 0xAB3F},
	{0x0990, 0x0064},
	{0x098C, 0xAB40},
	{0x0990, 0x0080},
	{0x098C, 0xAB41},
	{0x0990, 0x0099},
	{0x098C, 0xAB42},
	{0x0990, 0x00B0},
	{0x098C, 0xAB43},
	{0x0990, 0x00C1},
	{0x098C, 0xAB44},
	{0x0990, 0x00CF},
	{0x098C, 0xAB45},
	{0x0990, 0x00D9},
	{0x098C, 0xAB46},
	{0x0990, 0x00E1},
	{0x098C, 0xAB47},
	{0x0990, 0x00E8},
	{0x098C, 0xAB48},
	{0x0990, 0x00EE},
	{0x098C, 0xAB49},
	{0x0990, 0x00F2},
	{0x098C, 0xAB4A},
	{0x0990, 0x00F6},
	{0x098C, 0xAB4B},
	{0x0990, 0x00F9},
	{0x098C, 0xAB4C},
	{0x0990, 0x00FB},
	{0x098C, 0xAB4D},
	{0x0990, 0x00FD},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},
};

static struct msm_camera_i2c_reg_array contract_setup_tb_m1[] = {
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x001B},
	{0x098C, 0xAB3E},
	{0x0990, 0x002E},
	{0x098C, 0xAB3F},
	{0x0990, 0x004C},
	{0x098C, 0xAB40},
	{0x0990, 0x0078},
	{0x098C, 0xAB41},
	{0x0990, 0x0098},
	{0x098C, 0xAB42},
	{0x0990, 0x00B0},
	{0x098C, 0xAB43},
	{0x0990, 0x00C1},
	{0x098C, 0xAB44},
	{0x0990, 0x00CF},
	{0x098C, 0xAB45},
	{0x0990, 0x00D9},
	{0x098C, 0xAB46},
	{0x0990, 0x00E1},
	{0x098C, 0xAB47},
	{0x0990, 0x00E8},
	{0x098C, 0xAB48},
	{0x0990, 0x00EE},
	{0x098C, 0xAB49},
	{0x0990, 0x00F2},
	{0x098C, 0xAB4A},
	{0x0990, 0x00F6},
	{0x098C, 0xAB4B},
	{0x0990, 0x00F9},
	{0x098C, 0xAB4C},
	{0x0990, 0x00FB},
	{0x098C, 0xAB4D},
	{0x0990, 0x00FD},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},
};

static struct msm_camera_i2c_reg_array contract_setup_tb_m2[] = {
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0014},
	{0x098C, 0xAB3E},
	{0x0990, 0x0027},
	{0x098C, 0xAB3F},
	{0x0990, 0x0041},
	{0x098C, 0xAB40},
	{0x0990, 0x0074},
	{0x098C, 0xAB41},
	{0x0990, 0x0093},
	{0x098C, 0xAB42},
	{0x0990, 0x00AD},
	{0x098C, 0xAB43},
	{0x0990, 0x00C1},
	{0x098C, 0xAB44},
	{0x0990, 0x00CA},
	{0x098C, 0xAB45},
	{0x0990, 0x00D4},
	{0x098C, 0xAB46},
	{0x0990, 0x00DC},
	{0x098C, 0xAB47},
	{0x0990, 0x00E4},
	{0x098C, 0xAB48},
	{0x0990, 0x00E9},
	{0x098C, 0xAB49},
	{0x0990, 0x00EE},
	{0x098C, 0xAB4A},
	{0x0990, 0x00F2},
	{0x098C, 0xAB4B},
	{0x0990, 0x00F5},
	{0x098C, 0xAB4C},
	{0x0990, 0x00F8},
	{0x098C, 0xAB4D},
	{0x0990, 0x00FD},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},

};

static struct msm_camera_i2c_reg_array contract_setup_tb_m3[] = {
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0008},
	{0x098C, 0xAB3E},
	{0x0990, 0x0017},
	{0x098C, 0xAB3F},
	{0x0990, 0x002F},
	{0x098C, 0xAB40},
	{0x0990, 0x0050},
	{0x098C, 0xAB41},
	{0x0990, 0x006D},
	{0x098C, 0xAB42},
	{0x0990, 0x0088},
	{0x098C, 0xAB43},
	{0x0990, 0x009E},
	{0x098C, 0xAB44},
	{0x0990, 0x00AF},
	{0x098C, 0xAB45},
	{0x0990, 0x00BD},
	{0x098C, 0xAB46},
	{0x0990, 0x00C9},
	{0x098C, 0xAB47},
	{0x0990, 0x00D3},
	{0x098C, 0xAB48},
	{0x0990, 0x00DB},
	{0x098C, 0xAB49},
	{0x0990, 0x00E3},
	{0x098C, 0xAB4A},
	{0x0990, 0x00EA},
	{0x098C, 0xAB4B},
	{0x0990, 0x00F0},
	{0x098C, 0xAB4C},
	{0x0990, 0x00F5},
	{0x098C, 0xAB4D},
	{0x0990, 0x00FA},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},

};

static struct msm_camera_i2c_reg_array contract_setup_tb_m4[] = {
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0006},
	{0x098C, 0xAB3E},
	{0x0990, 0x0012},
	{0x098C, 0xAB3F},
	{0x0990, 0x0027},
	{0x098C, 0xAB40},
	{0x0990, 0x0048},
	{0x098C, 0xAB41},
	{0x0990, 0x0069},
	{0x098C, 0xAB42},
	{0x0990, 0x008A},
	{0x098C, 0xAB43},
	{0x0990, 0x00A4},
	{0x098C, 0xAB44},
	{0x0990, 0x00B7},
	{0x098C, 0xAB45},
	{0x0990, 0x00C6},
	{0x098C, 0xAB46},
	{0x0990, 0x00D1},
	{0x098C, 0xAB47},
	{0x0990, 0x00DB},
	{0x098C, 0xAB48},
	{0x0990, 0x00E2},
	{0x098C, 0xAB49},
	{0x0990, 0x00E9},
	{0x098C, 0xAB4A},
	{0x0990, 0x00EE},
	{0x098C, 0xAB4B},
	{0x0990, 0x00F3},
	{0x098C, 0xAB4C},
	{0x0990, 0x00F7},
	{0x098C, 0xAB4D},
	{0x0990, 0x00FB},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},

};


static int contrast_trans(int level)
{
       if(level == 0)return 0;
       if(level == 1)return 0;
       if(level == 2)return 1;
       if(level == 3)return 1;
       if(level == 4)return 2;
       if(level == 5)return 3;
       if(level == 6)return 3;
       if(level == 7)return 3;
       if(level == 8)return 3;
       if(level == 9)return 4;
       if(level == 10)return 4;
       return 3;
}

static uint32_t mi380_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
    int rc;
    struct msm_camera_i2c_reg_setting conf_array;
    

    switch(contrast_trans(level)) 
    {
        case 0:
		conf_array.delay = 5;
		conf_array.reg_setting = contract_setup_tb_m0;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(contract_setup_tb_m0);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
		break;

        case 1:
		conf_array.delay = 5;
		conf_array.reg_setting = contract_setup_tb_m1;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(contract_setup_tb_m1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
             break;

        case 2:
		conf_array.delay = 5;
		conf_array.reg_setting = contract_setup_tb_m2;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(contract_setup_tb_m2);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
             break;

        case 3:
		conf_array.delay = 5;
		conf_array.reg_setting = contract_setup_tb_m3;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(contract_setup_tb_m3);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
             break;

        case 4:
	case 5:
		conf_array.delay = 5;
		conf_array.reg_setting = contract_setup_tb_m4;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(contract_setup_tb_m4);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
             break;
    }

    return SENSOR_SUCCESS;
}
static uint32_t mi380_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
    

    switch(level/8)
    {
        case 0:
            mi380_i2c_write_reg(0x098C, 0xAB22);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x326C, 0x0400);
            break;

        case 1:
            mi380_i2c_write_reg(0x098C, 0xAB22);
            mi380_i2c_write_reg(0x0990, 0x0001);
            mi380_i2c_write_reg(0x326C, 0x0600);
            break;

        case 2:
            mi380_i2c_write_reg(0x098C, 0xAB22);
            mi380_i2c_write_reg(0x0990, 0x0005);
            mi380_i2c_write_reg(0x326C, 0x0B00);
            break;

        case 3:
            mi380_i2c_write_reg(0x098C, 0xAB22);
            mi380_i2c_write_reg(0x0990, 0x0006);
            mi380_i2c_write_reg(0x326C, 0x0B00);
            break;

        case 4:
            mi380_i2c_write_reg(0x098C, 0xAB22);
            mi380_i2c_write_reg(0x0990, 0x0007);
            mi380_i2c_write_reg(0x326C, 0x0FF0);
            break;

    }

    return SENSOR_SUCCESS;
}
static uint32_t mi380_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
    

    switch(level/2) 
    {
        case 0:
            mi380_i2c_write_reg(0x098C, 0xAB20);
            mi380_i2c_write_reg(0x0990, 0x0010);
            mi380_i2c_write_reg(0x098C, 0xAB24);
            mi380_i2c_write_reg(0x0990, 0x0009);
            break;

        case 1:
            mi380_i2c_write_reg(0x098C, 0xAB20);
            mi380_i2c_write_reg(0x0990, 0x0035);
            mi380_i2c_write_reg(0x098C, 0xAB24);
            mi380_i2c_write_reg(0x0990, 0x0025);
            break;

        case 2:
            mi380_i2c_write_reg(0x098C, 0xAB20);
            mi380_i2c_write_reg(0x0990, 0x0048);
            mi380_i2c_write_reg(0x098C, 0xAB24);
            mi380_i2c_write_reg(0x0990, 0x0033);
            break;

        case 3:
            mi380_i2c_write_reg(0x098C, 0xAB20);
            mi380_i2c_write_reg(0x0990, 0x0063);
            mi380_i2c_write_reg(0x098C, 0xAB24);
            mi380_i2c_write_reg(0x0990, 0x0045);
            break;

        case 4:
        case 5:
            mi380_i2c_write_reg(0x098C, 0xAB20);
            mi380_i2c_write_reg(0x0990, 0x0076);
            mi380_i2c_write_reg(0x098C, 0xAB24);
            mi380_i2c_write_reg(0x0990, 0x0053);
            break;

    }

    return SENSOR_SUCCESS;
}


static uint32_t  mi380_set_wb(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
    int rc = 0, k;
    uint16_t check_value = 0;
    struct msm_camera_i2c_reg_setting conf_array;
    

    switch(level)
    {
        case CAMERA_AWB_AUTO:
            mi380_i2c_write_reg(0x098C, 0xA11F);
            mi380_i2c_write_reg(0x0990, 0x0001);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
	    for (k = 0; k < CHECK_STATE_TIME; k++) {  
		    rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				    0xA103, WORD_LEN);
		    rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				    &check_value);
		    if (check_value == 0x0000) 
			    break;
		    msleep(1);
	    }
	    if (k == CHECK_STATE_TIME) 
		    return -EIO;
	    conf_array.delay = 5;
	    conf_array.reg_setting = m_wb_auto;
	    conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	    conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
	    conf_array.size = ARRAY_SIZE(m_wb_auto);
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write_table(s_ctrl->sensor_i2c_client,
				    &conf_array);
        break;

        case CAMERA_AWB_INDOOR_OFFICE:
            mi380_i2c_write_reg(0x098C, 0xA115);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA11F);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
	    for (k = 0; k < CHECK_STATE_TIME; k++) {  
		    rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				    0xA103, WORD_LEN);
		    rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				    &check_value);
		    if (check_value == 0x0000) 
			    break;
		    msleep(1);
	    }
	    if (k == CHECK_STATE_TIME) 
		    return -EIO;
	    conf_array.delay = 5;
	    conf_array.reg_setting = m_wb_fluorescent;
	    conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	    conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
	    conf_array.size = ARRAY_SIZE(m_wb_fluorescent);
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write_table(s_ctrl->sensor_i2c_client,
				    &conf_array);
        break;

        case CAMERA_AWB_INDOOR_HOME:
            mi380_i2c_write_reg(0x098C, 0xA115);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA11F);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
	    for (k = 0; k < CHECK_STATE_TIME; k++) {  
		    rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				    0xA103, WORD_LEN);
		    rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				    &check_value);
		    if (check_value == 0x0000) 
			    break;
		    msleep(1);
	    }
	    if (k == CHECK_STATE_TIME) 
		    return -EIO;
	    conf_array.delay = 5;
	    conf_array.reg_setting = m_wb_incandescent;
	    conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	    conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
	    conf_array.size = ARRAY_SIZE(m_wb_incandescent);
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write_table(s_ctrl->sensor_i2c_client,
				    &conf_array);
        break;

        case CAMERA_AWB_SUNNY:
            mi380_i2c_write_reg(0x098C, 0xA115);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA11F);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
	    for (k = 0; k < CHECK_STATE_TIME; k++) {  
		    rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				    0xA103, WORD_LEN);
		    rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				    &check_value);
		    if (check_value == 0x0000) 
			    break;
		    msleep(1);
	    }
	    if (k == CHECK_STATE_TIME) 
		    return -EIO;
	    conf_array.delay = 5;
	    conf_array.reg_setting = m_wb_daylight;
	    conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	    conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
	    conf_array.size = ARRAY_SIZE(m_wb_daylight);
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write_table(s_ctrl->sensor_i2c_client,
				    &conf_array);
        break;

        case CAMERA_AWB_CLOUDY:
            mi380_i2c_write_reg(0x098C, 0xA115);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA11F);
            mi380_i2c_write_reg(0x0990, 0x0000);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
	    for (k = 0; k < CHECK_STATE_TIME; k++) {  
		    rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				    0xA103, WORD_LEN);
		    rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				    &check_value);
		    if (check_value == 0x0000) 
			    break;
		    msleep(1);
	    }
	    if (k == CHECK_STATE_TIME) 
		    return -EIO;
	    conf_array.delay = 5;
	    conf_array.reg_setting = m_wb_cloudy;
	    conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	    conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
	    conf_array.size = ARRAY_SIZE(m_wb_cloudy);
	    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		    i2c_write_table(s_ctrl->sensor_i2c_client,
				    &conf_array);
        break;
    }

    return SENSOR_SUCCESS;
}

static uint32_t mi380_SetEffect(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
    int k, rc;
    uint16_t check_value = 0;
    
    switch(level)
    {
        case CAMERA_EFFECT_NONE:
            mi380_i2c_write_reg(0x098C, 0x2759);
            mi380_i2c_write_reg(0x0990, 0x6440);
            mi380_i2c_write_reg(0x098C, 0x275B);
            mi380_i2c_write_reg(0x0990, 0x6440);
            mi380_i2c_write_reg(0x098C, 0x2763);
            mi380_i2c_write_reg(0x0990, 0xB023);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
            break;

        case CAMERA_EFFECT_MONO:
            mi380_i2c_write_reg(0x098C, 0x2759);
            mi380_i2c_write_reg(0x0990, 0x6441);
            mi380_i2c_write_reg(0x098C, 0x275B);
            mi380_i2c_write_reg(0x0990, 0x6441);
            mi380_i2c_write_reg(0x098C, 0x2763);
            mi380_i2c_write_reg(0x0990, 0xB023);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
            break;

        case CAMERA_EFFECT_SEPIA:
            mi380_i2c_write_reg(0x098C, 0x2759);
            mi380_i2c_write_reg(0x0990, 0x6442);
            mi380_i2c_write_reg(0x098C, 0x275B);
            mi380_i2c_write_reg(0x0990, 0x6442);
            mi380_i2c_write_reg(0x098C, 0x2763);
            mi380_i2c_write_reg(0x0990, 0xB023);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
            break;

        case CAMERA_EFFECT_NEGATIVE:
            mi380_i2c_write_reg(0x098C, 0x2759);
            mi380_i2c_write_reg(0x0990, 0x6443);
            mi380_i2c_write_reg(0x098C, 0x275B);
            mi380_i2c_write_reg(0x0990, 0x6443);
            mi380_i2c_write_reg(0x098C, 0x2763);
            mi380_i2c_write_reg(0x0990, 0xB023);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
            break;

        case CAMERA_EFFECT_AQUA:
            mi380_i2c_write_reg(0x098C, 0x2759);
            mi380_i2c_write_reg(0x0990, 0x6442);
            mi380_i2c_write_reg(0x098C, 0x275B);
            mi380_i2c_write_reg(0x0990, 0x6442);
            mi380_i2c_write_reg(0x098C, 0x2763);
            mi380_i2c_write_reg(0x0990, 0x30D0);
            mi380_i2c_write_reg(0x098C, 0xA103);
            mi380_i2c_write_reg(0x0990, 0x0005);
            break;
    }


	for (k = 0; k < CHECK_STATE_TIME; k++) {  
		rc = mi380_i2c_write(mi380_client->addr, 0x098C,
			0xA103, WORD_LEN);
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) 
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME) 
		return -EIO;

    return SENSOR_SUCCESS;
}

static int mi380_set_brightness(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
	int rc = 0;

	switch (level) {
	case -4: 
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x001F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00CA, WORD_LEN);
			if (rc < 0)
				return -EIO;

			break;

	case -3: 
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0025, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case -2:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0030, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case -1:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0038, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case 0: 
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x004A, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case 1:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0051, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case 2:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0059, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C7, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case 3:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x005F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C7, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case 4:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0068, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C6, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	default:
		pr_info("%s: Not support brightness value = %d\n",
			__func__, level);
		 return -EINVAL;
	}
	return 0;
}

enum antibanding_mode {
CAMERA_ANTI_BANDING_50HZ,
CAMERA_ANTI_BANDING_60HZ,
CAMERA_ANTI_BANDING_AUTO,
};

static int mi380_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, uint32_t antibanding_value)
{
	int rc = 0;
	unsigned short check_value = 0;
	int iRetryCnt = 20;

	switch (antibanding_value) {
	case CAMERA_ANTI_BANDING_50HZ:
	while ((check_value != 0xE0) && (iRetryCnt-- > 0)) {
		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA404, WORD_LEN);
		rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00C0, WORD_LEN);
			if (rc < 0)
				return -EIO;

		msleep(5);

		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA404, WORD_LEN);
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0990, &check_value);
	}

	if (check_value != 0xE0)
		pr_info("%s: check_value: 0x%X, retry failed!\n", __func__, check_value);
		break;
	case CAMERA_ANTI_BANDING_60HZ:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA404, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0080, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_ANTI_BANDING_AUTO: 
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA404, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0080, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	default:
		pr_info("%s: Not support antibanding value = %d\n",
		   __func__, antibanding_value);
		return -EINVAL;
	}
	return 0;

}

enum iso_mode {
CAMERA_ISO_MODE_AUTO,
CAMERA_ISO_MODE_100,
CAMERA_ISO_MODE_200,
CAMERA_ISO_MODE_400,
CAMERA_ISO_MODE_800,
};

static int mi380_set_iso(struct msm_sensor_ctrl_t *s_ctrl, uint32_t iso_value)
{
	int rc = 0, k = 0;
	unsigned short check_value;

	switch (iso_value) {
	case CAMERA_ISO_MODE_AUTO:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20E, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0080, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) 
			return -EIO;

		break;
	case CAMERA_ISO_MODE_100:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20E, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0026, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) 
			return -EIO;

		break;
	case CAMERA_ISO_MODE_200:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20E, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0046, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) 
			return -EIO;

		break;
	case CAMERA_ISO_MODE_400:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20E, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0078, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) 
			return -EIO;

		break;
	case CAMERA_ISO_MODE_800:
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20E, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x00A0, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) 
			return -EIO;

		break;
	default:
		pr_info("%s: Not support ISO value = %d\n",
			__func__, iso_value);
		 return -EINVAL;
	}
	return 0;
}

static int mi380_detect_sensor_status(void)
{
	int rc = 0, k = 0;
	unsigned short check_value;

	for (k = 0; k < CHECK_STATE_TIME; k++) {	
		rc = mi380_i2c_write(mi380_client->addr, 0x098C,
			0xA103, WORD_LEN);
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) 
			break;

		msleep(1);
	}

	if (k == CHECK_STATE_TIME) 
		pr_info("mi380_detect_sensor_status,time out");

	return 0;
}


static int mi380_set_fps(struct msm_sensor_ctrl_t *s_ctrl, uint32_t level)
{
	pr_info("mi380_set_fps, fps_div=%d", level);

	if (level == 10) {
		mi380_i2c_write(mi380_client->addr, 0x098C, 0x271F, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x067E, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0006, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();

		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x000C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();
	} else if (level == 15) {
		mi380_i2c_write(mi380_client->addr, 0x098C, 0x271F, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0454, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0006, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();

		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0004, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();
	} else if (level == 1015) {
		mi380_i2c_write(mi380_client->addr, 0x098C, 0x271F, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0454, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0006, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();

		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x000C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();
	} else if (level == 0) {
		mi380_i2c_write(mi380_client->addr, 0x098C, 0x271F, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x022A, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0006, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();

		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA20C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x000C, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA215, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0008, WORD_LEN);

		mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
		mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
		mdelay(1);

		mi380_detect_sensor_status();
	}

	return 0;
}


struct mi380_reg {
	struct mi380_i2c_reg_conf *power_up_tbl;
	uint16_t power_up_tbl_size;
	struct mi380_i2c_reg_conf *register_init_1;
	uint16_t register_init_size_1;
	struct mi380_i2c_reg_conf *register_init_2;
	uint16_t register_init_size_2;
	struct mi380_i2c_reg_conf *contract_tb0;
	uint16_t contract_tb0_size;
	struct mi380_i2c_reg_conf *contract_tb1;
	uint16_t contract_tb1_size;
	struct mi380_i2c_reg_conf *contract_tb2;
	uint16_t contract_tb2_size;
	struct mi380_i2c_reg_conf *contract_tb3;
	uint16_t contract_tb3_size;
	struct mi380_i2c_reg_conf *contract_tb4;
	uint16_t contract_tb4_size;
	struct mi380_i2c_reg_conf *wb_auto;
	uint16_t wb_auto_size;
	struct mi380_i2c_reg_conf *wb_fluorescent;
	uint16_t wb_fluorescent_size;
	struct mi380_i2c_reg_conf *wb_incandescent;
	uint16_t wb_incandescent_size;
	struct mi380_i2c_reg_conf *wb_daylight;
	uint16_t wb_daylight_size;
	struct mi380_i2c_reg_conf *wb_cloudy;
	uint16_t wb_cloudy_size;
};

#include "mi380_reg.c"
static int suspend_fail_retry_count_2;
#define SUSPEND_FAIL_RETRY_MAX_2 0
int g_csi_if = 1;

struct msm_sensor_power_setting mi380_power_setting[] = {
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

struct msm_sensor_power_setting mi380_power_down_setting[] = {
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
static struct msm_camera_i2c_reg_conf mi380_start_settings[] = {
	{0x3400,0x7a30},
};

static struct msm_camera_i2c_reg_conf mi380_stop_settings[] = {
	{0x3400,0x7a32},
};

#if 0
static struct msm_camera_i2c_reg_conf mi380_init_tb1[] = {
	{0x098C, 0xA11D}, 	
	{0x0990, 0x0001},
	{0x098C, 0xA249},
	{0x0990, 0x0002},
	{0x098C, 0xA24F}, 	
	{0x0990, 0x0040},	
	{0x098C, 0xA24B}, 	
	{0x0990, 0x0086},
	{0x098C, 0xA24A}, 	
	{0x0990, 0x007E},
	{0x098C, 0xA207},	
	{0x0990, 0x0002},
	{0x098C, 0x2257}, 
	{0x0990, 0x3A98}, 
	{0x098C, 0xAB1F},
	{0x0990, 0x00C9},
	{0x326C, 0x0900},
	{0x001E, 0x0400},
	{0x098C, 0xAB22},
	{0x0990, 0x0005},
	{0x098C, 0xA404},
	{0x0990, 0x0010},
	{0x098C, 0x222D},
	{0x0990, 0x008B},
	{0x098C, 0xA408},
	{0x0990, 0x0021},
	{0x098C, 0xA409},
	{0x0990, 0x0024},
	{0x098C, 0xA40A},
	{0x0990, 0x0028},
	{0x098C, 0xA40B},
	{0x0990, 0x002B},
	{0x098C, 0x2411},
	{0x0990, 0x008B},
	{0x098C, 0x2413},
	{0x0990, 0x00A6},
	{0x098C, 0x2415},
	{0x0990, 0x008B},
	{0x098C, 0x2417},
	{0x0990, 0x00A6},
	{0x098C, 0xA40D},
	{0x0990, 0x0002},
	{0x098C, 0xA40E},
	{0x0990, 0x0003},
	{0x098C, 0xA410},
	{0x0990, 0x000A},
	{0x364E, 0x0330},
	{0x3650, 0x010B},
	{0x3652, 0x2312},
	{0x3654, 0xC2AF},
	{0x3656, 0x9F50},
	{0x3658, 0x0290},
	{0x365A, 0x0FEB},
	{0x365C, 0x4E52},
	{0x365E, 0xC0CF},
	{0x3660, 0xCB12},
	{0x3662, 0x02B0},
	{0x3664, 0x620C},
	{0x3666, 0x1AD2},
	{0x3668, 0xA7B0},
	{0x366A, 0xBB91},
	{0x366C, 0x0290},
	{0x366E, 0xCE2A},
	{0x3670, 0x2AD2},
	{0x3672, 0x8BAE},
	{0x3674, 0xABAF},
	{0x3676, 0xCB8D},
	{0x3678, 0xA24E},
	{0x367A, 0x2F91},
	{0x367C, 0x0991},
	{0x367E, 0xC594},
	{0x3680, 0xAC2B},
	{0x3682, 0xA4AC},
	{0x3684, 0x6891},
	{0x3686, 0xAD30},
	{0x3688, 0x9295},
	{0x368A, 0x380B},
	{0x368C, 0x464C},
	{0x368E, 0x4C2C},
	{0x3690, 0xE9AF},
	{0x3692, 0xC312},
	{0x3694, 0xA50D},
	{0x3696, 0xF6AD},
	{0x3698, 0x2E11},
	{0x369A, 0x11F0},
	{0x369C, 0xB534},
	{0x369E, 0x0573},
	{0x36A0, 0xA431},
	{0x36A2, 0x81B6},
	{0x36A4, 0x0895},
	{0x36A6, 0x5D19},
	{0x36A8, 0x17F3},
	{0x36AA, 0xF1F1},
	{0x36AC, 0x80D6},
	{0x36AE, 0x42B4},
	{0x36B0, 0x3499},
	{0x36B2, 0x55F2},
	{0x36B4, 0x9492},
	{0x36B6, 0x9456},
	{0x36B8, 0x22B5},
	{0x36BA, 0x4AB9},
	{0x36BC, 0x0093},
	{0x36BE, 0xA391},
	{0x36C0, 0x85B6},
	{0x36C2, 0x4C34},
	{0x36C4, 0x4BF9},
	{0x36C6, 0xBD0F},
	{0x36C8, 0x1DD1},
	{0x36CA, 0xEE54},
	{0x36CC, 0xAAB5},
	{0x36CE, 0x0CD7},
	{0x36D0, 0xA770},
	{0x36D2, 0x9C11},
	{0x36D4, 0xA635},
	{0x36D6, 0x1576},
	{0x36D8, 0x0058},
	{0x36DA, 0xC4F1},
	{0x36DC, 0xD3F1},
	{0x36DE, 0x5134},
	{0x36E0, 0x2696},
	{0x36E2, 0x8F19},
	{0x36E4, 0xD98F},
	{0x36E6, 0xA911},
	{0x36E8, 0xD1F4},
	{0x36EA, 0x7054},
	{0x36EC, 0x76D6},
	{0x36EE, 0x93D5},
	{0x36F0, 0x0934},
	{0x36F2, 0x63B9},
	{0x36F4, 0xC178},
	{0x36F6, 0xEA7C},
	{0x36F8, 0xA7D5},
	{0x36FA, 0x0A55},
	{0x36FC, 0x3979},
	{0x36FE, 0x8597},
	{0x3700, 0xA03C},
	{0x3702, 0xE194},
	{0x3704, 0x74F4},
	{0x3706, 0x7A19},
	{0x3708, 0xC5B6},
	{0x370A, 0xD9BC},
	{0x370C, 0x8F55},
	{0x370E, 0x6FF4},
	{0x3710, 0x01DA},
	{0x3712, 0xE317},
	{0x3714, 0xE93C},
	{0x3644, 0x0144},
	{0x3642, 0x00F0},
};

static struct msm_camera_i2c_reg_conf mi380_init_tb2[] = {
	{0x098C, 0xA354},
	{0x0990, 0x0048},
	{0x098C, 0xAB20},
	{0x0990, 0x0048},
	{0x098C, 0xA11F},
	{0x0990, 0x0001},
	{0x098C, 0x2306},
	{0x0990, 0x03C0},
	{0x098C, 0x2308},
	{0x0990, 0xFD7C},
	{0x098C, 0x230A},
	{0x0990, 0xFFF7},
	{0x098C, 0x230C},
	{0x0990, 0xFF25},
	{0x098C, 0x230E},
	{0x0990, 0x0384},
	{0x098C, 0x2310},
	{0x0990, 0xFFD6},
	{0x098C, 0x2312},
	{0x0990, 0xFED2},
	{0x098C, 0x2314},
	{0x0990, 0xFCB2},
	{0x098C, 0x2316},
	{0x0990, 0x068E},
	{0x098C, 0x2318},
	{0x0990, 0x001B},
	{0x098C, 0x231A},
	{0x0990, 0x0036},
	{0x098C, 0x231C},
	{0x0990, 0xFF65},
	{0x098C, 0x231E},
	{0x0990, 0x0052},
	{0x098C, 0x2320},
	{0x0990, 0x0012},
	{0x098C, 0x2322},
	{0x0990, 0x0007},
	{0x098C, 0x2324},
	{0x0990, 0xFFCF},
	{0x098C, 0x2326},
	{0x0990, 0x0037},
	{0x098C, 0x2328},
	{0x0990, 0x00D8},
	{0x098C, 0x232A},
	{0x0990, 0x01C8},
	{0x098C, 0x232C},
	{0x0990, 0xFC9F},
	{0x098C, 0x232E},
	{0x0990, 0x0010},
	{0x098C, 0x2330},
	{0x0990, 0xFFF7},
	{0x098C, 0xA348},
	{0x0990, 0x0008},
	{0x098C, 0xA349},
	{0x0990, 0x0002},
	{0x098C, 0xA34A},
	{0x0990, 0x0059},
	{0x098C, 0xA34B},
	{0x0990, 0x00E6},
	{0x098C, 0xA351},
	{0x0990, 0x0000},
	{0x098C, 0xA352},
	{0x0990, 0x007F},
	{0x098C, 0xA355},
	{0x0990, 0x0001},
	{0x098C, 0xA35D},
	{0x0990, 0x0078},
	{0x098C, 0xA35E},
	{0x0990, 0x0086},
	{0x098C, 0xA35F},
	{0x0990, 0x007E},
	{0x098C, 0xA360},
	{0x0990, 0x0082},
	{0x098C, 0x2361},
	{0x0990, 0x0040},
	{0x098C, 0xA363},
	{0x0990, 0x00D2},
	{0x098C, 0xA364},
	{0x0990, 0x00F6},
	{0x098C, 0xA302},
	{0x0990, 0x0000},
	{0x098C, 0xA303},
	{0x0990, 0x00EF},
	{0x098C, 0xA366},
	{0x0990, 0x00A6},
	{0x098C, 0xA367},
	{0x0990, 0x0096},
	{0x098C, 0xA368},
	{0x0990, 0x005C},
#ifndef CONFIG_MSM_CAMERA_8X60
	{0x098C, 0x2B1B},
	{0x0990, 0x0000},
#endif
	{0x098C, 0x2B28},
	{0x0990, 0x157C},
	{0x098C, 0x2B2A},
	{0x0990, 0x1B58},
	{0x098C, 0xAB37},
	{0x0990, 0x0001}, 
	{0x098C, 0x2B38},
	{0x0990, 0x157C},
	{0x098C, 0x2B3A},
	{0x0990, 0x1B58},
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0004},
	{0x098C, 0xAB3E},
	{0x0990, 0x000E},
	{0x098C, 0xAB3F},
	{0x0990, 0x0029},
	{0x098C, 0xAB40},
	{0x0990, 0x0050},
	{0x098C, 0xAB41},
	{0x0990, 0x006A},
	{0x098C, 0xAB42},
	{0x0990, 0x0081},
	{0x098C, 0xAB43},
	{0x0990, 0x0094},
	{0x098C, 0xAB44},
	{0x0990, 0x00A5},
	{0x098C, 0xAB45},
	{0x0990, 0x00B2},
	{0x098C, 0xAB46},
	{0x0990, 0x00BE},
	{0x098C, 0xAB47},
	{0x0990, 0x00C9},
	{0x098C, 0xAB48},
	{0x0990, 0x00D3},
	{0x098C, 0xAB49},
	{0x0990, 0x00DC},
	{0x098C, 0xAB4A},
	{0x0990, 0x00E4},
	{0x098C, 0xAB4B},
	{0x0990, 0x00EB},
	{0x098C, 0xAB4C},
	{0x0990, 0x00F2},
	{0x098C, 0xAB4D},
	{0x0990, 0x00F9},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},
	{0x098C, 0xAB4F},
	{0x0990, 0x0000},
	{0x098C, 0xAB50},
	{0x0990, 0x000F},
	{0x098C, 0xAB51},
	{0x0990, 0x001A},
	{0x098C, 0xAB52},
	{0x0990, 0x002E},
	{0x098C, 0xAB53},
	{0x0990, 0x0050},
	{0x098C, 0xAB54},
	{0x0990, 0x006A},
	{0x098C, 0xAB55},
	{0x0990, 0x0080},
	{0x098C, 0xAB56},
	{0x0990, 0x0091},
	{0x098C, 0xAB57},
	{0x0990, 0x00A1},
	{0x098C, 0xAB58},
	{0x0990, 0x00AF},
	{0x098C, 0xAB59},
	{0x0990, 0x00BB},
	{0x098C, 0xAB5A},
	{0x0990, 0x00C6},
	{0x098C, 0xAB5B},
	{0x0990, 0x00D0},
	{0x098C, 0xAB5C},
	{0x0990, 0x00D9},
	{0x098C, 0xAB5D},
	{0x0990, 0x00E2},
	{0x098C, 0xAB5E},
	{0x0990, 0x00EA},
	{0x098C, 0xAB5F},
	{0x0990, 0x00F1},
	{0x098C, 0xAB60},
	{0x0990, 0x00F9},
	{0x098C, 0xAB61},
	{0x0990, 0x00FF},
	{0x098C, 0x2703},
	{0x0990, 0x0280},
	{0x098C, 0x2705},
	{0x0990, 0x01E0},
	{0x098C, 0x270D},
	{0x0990, 0x0004},
	{0x098C, 0x270F},
	{0x0990, 0x0004},
	{0x098C, 0x2711},
	{0x0990, 0x01EB},
	{0x098C, 0x2713},
	{0x0990, 0x028B},
	{0x098C, 0x2715},
	{0x0990, 0x0001},
	{0x098C, 0x2717},
	{0x0990, 0x0025},     
	{0x098C, 0x2719},
	{0x0990, 0x001A},
	{0x098C, 0x271B},
	{0x0990, 0x006B},
	{0x098C, 0x271D},
	{0x0990, 0x006B},
	{0x098C, 0x271F},
	{0x0990, 0x022A},
	{0x098C, 0x2721},
	{0x0990, 0x034A},
	{0x098C, 0x2739},
	{0x0990, 0x0000},
	{0x098C, 0x273B},
	{0x0990, 0x027F},
	{0x098C, 0x273D},
	{0x0990, 0x0000},
	{0x098C, 0x273F},
	{0x0990, 0x01DF},
	{0x098C, 0x2707},
	{0x0990, 0x0280},
	{0x098C, 0x2709},
	{0x0990, 0x01E0},
	{0x098C, 0x2723},
	{0x0990, 0x0004},
	{0x098C, 0x2725},
	{0x0990, 0x0004},
	{0x098C, 0x2727},
	{0x0990, 0x01EB},
	{0x098C, 0x2729},
	{0x0990, 0x028B},
	{0x098C, 0x272B},
	{0x0990, 0x0001},
	{0x098C, 0x272D},
	{0x0990, 0x0025},     
	{0x098C, 0x272F},
	{0x0990, 0x001A},
	{0x098C, 0x2731},
	{0x0990, 0x006B},
	{0x098C, 0x2733},
	{0x0990, 0x006B},
	{0x098C, 0x2735},
	{0x0990, 0x022A},
	{0x098C, 0x2737},
	{0x0990, 0x034A},
	{0x098C, 0x2747},
	{0x0990, 0x0000},
	{0x098C, 0x2749},
	{0x0990, 0x027F},
	{0x098C, 0x274B},
	{0x0990, 0x0000},
	{0x098C, 0x274D},
	{0x0990, 0x01DF},
	{0x098C, 0x2755},
	{0x0990, 0x0002},
	{0x098C, 0x2757},
	{0x0990, 0x0002},
	{0x098C, 0xA20C},
	{0x0990, 0x000C},
};

#define SENSOR_WRITE_DELAY 0xffff
static struct msm_camera_i2c_reg_array mi380_recommend_settings[] = {
	{0x0018, 0x4028 },
	{SENSOR_WRITE_DELAY, 0x10},

	{0x001A, 0x0011 },
	{SENSOR_WRITE_DELAY, 0x0A},

	{0x001A, 0x0010 },
	{SENSOR_WRITE_DELAY, 0x0A},

	{0x0018, 0x4028 },
	{SENSOR_WRITE_DELAY, 0x12},

	{ 0x098C, 0x02F0	},
	{ 0x0990, 0x0000	},
	{ 0x098C, 0x02F2	},
	{ 0x0990, 0x0210	},
	{ 0x098C, 0x02F4	},
	{ 0x0990, 0x001A	},
	{ 0x098C, 0x2145	},
	{ 0x0990, 0x02F4	},
	{ 0x098C, 0xA134	},
	{ 0x0990, 0x0001	},

	{ 0x31E0, 0x0001	},

	{0x001A, 0x0010 },


	{ 0x3400, 0x7A30	},
	{ 0x321C, 0x8003	},
	{ 0x001E, 0x0777	},
	{ 0x0016, 0x42DF	},


	{ 0x0014, 0xB04B	},
	{ 0x0014, 0xB049	},

	{ 0x0010, 0x021C	},
	{ 0x0012, 0x0000	},
	{ 0x0014, 0x244B	},
	{SENSOR_WRITE_DELAY, 0x03},

	{ 0x0014, 0x304B	},
	{SENSOR_WRITE_DELAY, 0x05},

	{ 0x0014, 0xB04A	},
	{ 0x3418, 0x003F	},
	{SENSOR_WRITE_DELAY, 0x01},

	{ 0x3410, 0x0F0F},

	
	{0x098C, 0xA11D},
	{0x0990, 0x0001},
	{0x098C, 0xA249},
	{0x0990, 0x0002},
	{0x098C, 0xA24F},
	{0x0990, 0x0040},
	{0x098C, 0xA24B},
	{0x0990, 0x0086},
	{0x098C, 0xA24A},
	{0x0990, 0x007E},
	{0x098C, 0xA207},
	{0x0990, 0x0002},
	{0x098C, 0x2257},
	{0x0990, 0x3A98},
	
	{0x364E, 0x0330},
	{0x3650, 0x010B},
	{0x3652, 0x2312},
	{0x3654, 0xC2AF},
	{0x3656, 0x9F50},
	{0x3658, 0x0290},
	{0x365A, 0x0FEB},
	{0x365C, 0x4E52},
	{0x365E, 0xC0CF},
	{0x3660, 0xCB12},
	{0x3662, 0x02B0},
	{0x3664, 0x620C},
	{0x3666, 0x1AD2},
	{0x3668, 0xA7B0},
	{0x366A, 0xBB91},
	{0x366C, 0x0290},
	{0x366E, 0xCE2A},
	{0x3670, 0x2AD2},
	{0x3672, 0x8BAE},
	{0x3674, 0xABAF},
	{0x3676, 0xCB8D},
	{0x3678, 0xA24E},
	{0x367A, 0x2F91},
	{0x367C, 0x0991},
	{0x367E, 0xC594},
	{0x3680, 0xAC2B},
	{0x3682, 0xA4AC},
	{0x3684, 0x6891},
	{0x3686, 0xAD30},
	{0x3688, 0x9295},
	{0x368A, 0x380B},
	{0x368C, 0x464C},
	{0x368E, 0x4C2C},
	{0x3690, 0xE9AF},
	{0x3692, 0xC312},
	{0x3694, 0xA50D},
	{0x3696, 0xF6AD},
	{0x3698, 0x2E11},
	{0x369A, 0x11F0},
	{0x369C, 0xB534},
	{0x369E, 0x0573},
	{0x36A0, 0xA431},
	{0x36A2, 0x81B6},
	{0x36A4, 0x0895},
	{0x36A6, 0x5D19},
	{0x36A8, 0x17F3},
	{0x36AA, 0xF1F1},
	{0x36AC, 0x80D6},
	{0x36AE, 0x42B4},
	{0x36B0, 0x3499},
	{0x36B2, 0x55F2},
	{0x36B4, 0x9492},
	{0x36B6, 0x9456},
	{0x36B8, 0x22B5},
	{0x36BA, 0x4AB9},
	{0x36BC, 0x0093},
	{0x36BE, 0xA391},
	{0x36C0, 0x85B6},
	{0x36C2, 0x4C34},
	{0x36C4, 0x4BF9},
	{0x36C6, 0xBD0F},
	{0x36C8, 0x1DD1},
	{0x36CA, 0xEE54},
	{0x36CC, 0xAAB5},
	{0x36CE, 0x0CD7},
	{0x36D0, 0xA770},
	{0x36D2, 0x9C11},
	{0x36D4, 0xA635},
	{0x36D6, 0x1576},
	{0x36D8, 0x0058},
	{0x36DA, 0xC4F1},
	{0x36DC, 0xD3F1},
	{0x36DE, 0x5134},
	{0x36E0, 0x2696},
	{0x36E2, 0x8F19},
	{0x36E4, 0xD98F},
	{0x36E6, 0xA911},
	{0x36E8, 0xD1F4},
	{0x36EA, 0x7054},
	{0x36EC, 0x76D6},
	{0x36EE, 0x93D5},
	{0x36F0, 0x0934},
	{0x36F2, 0x63B9},
	{0x36F4, 0xC178},
	{0x36F6, 0xEA7C},
	{0x36F8, 0xA7D5},
	{0x36FA, 0x0A55},
	{0x36FC, 0x3979},
	{0x36FE, 0x8597},
	{0x3700, 0xA03C},
	{0x3702, 0xE194},
	{0x3704, 0x74F4},
	{0x3706, 0x7A19},
	{0x3708, 0xC5B6},
	{0x370A, 0xD9BC},
	{0x370C, 0x8F55},
	{0x370E, 0x6FF4},
	{0x3710, 0x01DA},
	{0x3712, 0xE317},
	{0x3714, 0xE93C},
	{0x3644, 0x0144},
	{0x3642, 0x00F0},

	{0x098C, 0xAB1F},
	{0x0990, 0x00C9},
	{0x326C, 0x0900},
	{0x001E, 0x0400},

	{0x098C, 0xAB22},
	{0x0990, 0x0005},

	{ 0x98C, 0x2703 },
	{ 0x990, 0x0280 },
	{ 0x98C, 0x2705 },
	{ 0x990, 0x01E0 },
	{ 0x98C, 0x2707 },
	{ 0x990, 0x0280 },
	{ 0x98C, 0x2709 },
	{ 0x990, 0x01E0 },
	{ 0x98C, 0x270D },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x270F },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x2711 },
	{ 0x990, 0x01E7 },
	{ 0x98C, 0x2713 },
	{ 0x990, 0x0287 },
	{ 0x98C, 0x2715 },
	{ 0x990, 0x0001 },
	{ 0x98C, 0x2717 },
	{ 0x990, 0x0025 },
	{ 0x98C, 0x2719 },
	{ 0x990, 0x001A },
	{ 0x98C, 0x271B },
	{ 0x990, 0x006B },
	{ 0x98C, 0x271D },
	{ 0x990, 0x006B },
	{ 0x98C, 0x271F },
	{ 0x990, 0x022A },
	{ 0x98C, 0x2721 },
	{ 0x990, 0x034A },
	{ 0x98C, 0x2723 },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x2725 },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x2727 },
	{ 0x990, 0x01E7 },
	{ 0x98C, 0x2729 },
	{ 0x990, 0x0287 },
	{ 0x98C, 0x272B },
	{ 0x990, 0x0001 },
	{ 0x98C, 0x272D },
	{ 0x990, 0x0025 },
	{ 0x98C, 0x272F },
	{ 0x990, 0x001A },
	{ 0x98C, 0x2731 },
	{ 0x990, 0x006B },
	{ 0x98C, 0x2733 },
	{ 0x990, 0x006B },
	{ 0x98C, 0x2735 },
	{ 0x990, 0x022a },
	{ 0x98C, 0x2737 },
	{ 0x990, 0x034A },
	{ 0x98C, 0x2739 },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x273B },
	{ 0x990, 0x027F },
	{ 0x98C, 0x273D },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x273F },
	{ 0x990, 0x01DF },
	{ 0x98C, 0x2747 },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x2749 },
	{ 0x990, 0x027F },
	{ 0x98C, 0x274B },
	{ 0x990, 0x0000 },
	{ 0x98C, 0x274D },
	{ 0x990, 0x01DF },
	{ 0x98C, 0x222D },
	{ 0x990, 0x008B },
	{ 0x98C, 0xA408 },
	{ 0x990, 0x0021 },
	{ 0x98C, 0xA409 },
	{ 0x990, 0x0023 },
	{ 0x98C, 0xA40A },
	{ 0x990, 0x0028 },
	{ 0x98C, 0xA40B },
	{ 0x990, 0x002A },
	{ 0x98C, 0x2411 },
	{ 0x990, 0x008B },
	{ 0x98C, 0x2413 },
	{ 0x990, 0x00A6 },
	{ 0x98C, 0x2415 },
	{ 0x990, 0x008B },
	{ 0x98C, 0x2417 },
	{ 0x990, 0x00A6 },
	{ 0x98C, 0xA404 },
	{ 0x990, 0x0010 },
	{ 0x98C, 0xA40D },
	{ 0x990, 0x0002 },
	{ 0x98C, 0xA40E },
	{ 0x990, 0x0003 },
	{ 0x98C, 0xA410 },
	{ 0x990, 0x000A },

	
	{0x098C, 0xA354},
	{0x0990, 0x0048},
	{0x098C, 0xAB20},
	{0x0990, 0x0048},

	
	{0x098C, 0xA11F},
	{0x0990, 0x0001},
	{0x098C, 0x2306},
	{0x0990, 0x03C0},
	{0x098C, 0x2308},
	{0x0990, 0xFD7C},
	{0x098C, 0x230A},
	{0x0990, 0xFFF7},
	{0x098C, 0x230C},
	{0x0990, 0xFF25},
	{0x098C, 0x230E},
	{0x0990, 0x0384},
	{0x098C, 0x2310},
	{0x0990, 0xFFD6},
	{0x098C, 0x2312},
	{0x0990, 0xFED2},
	{0x098C, 0x2314},
	{0x0990, 0xFCB2},
	{0x098C, 0x2316},
	{0x0990, 0x068E},
	{0x098C, 0x2318},
	{0x0990, 0x001B},
	{0x098C, 0x231A},
	{0x0990, 0x0036},
	{0x098C, 0x231C},
	{0x0990, 0xFF65},
	{0x098C, 0x231E},
	{0x0990, 0x0052},
	{0x098C, 0x2320},
	{0x0990, 0x0012},
	{0x098C, 0x2322},
	{0x0990, 0x0007},
	{0x098C, 0x2324},
	{0x0990, 0xFFCF},
	{0x098C, 0x2326},
	{0x0990, 0x0037},
	{0x098C, 0x2328},
	{0x0990, 0x00D8},
	{0x098C, 0x232A},
	{0x0990, 0x01C8},
	{0x098C, 0x232C},
	{0x0990, 0xFC9F},
	{0x098C, 0x232E},
	{0x0990, 0x0010},
	{0x098C, 0x2330},
	{0x0990, 0xFFF7},
	{0x098C, 0xA348},
	{0x0990, 0x0008},
	{0x098C, 0xA349},
	{0x0990, 0x0002},
	{0x098C, 0xA34A},
	{0x0990, 0x0059},
	{0x098C, 0xA34B},
	{0x0990, 0x00E6},
	{0x098C, 0xA351},
	{0x0990, 0x0000},
	{0x098C, 0xA352},
	{0x0990, 0x007F},
	{0x098C, 0xA355},
	{0x0990, 0x0001},
	{0x098C, 0xA35D},
	{0x0990, 0x0078},
	{0x098C, 0xA35E},
	{0x0990, 0x0086},
	{0x098C, 0xA35F},
	{0x0990, 0x007E},
	{0x098C, 0xA360},
	{0x0990, 0x0082},
	{0x098C, 0x2361},
	{0x0990, 0x0040},
	{0x098C, 0xA363},
	{0x0990, 0x00D2},
	{0x098C, 0xA364},
	{0x0990, 0x00F6},
	{0x098C, 0xA302},
	{0x0990, 0x0000},
	{0x098C, 0xA303},
	{0x0990, 0x00EF},
	{0x098C, 0xA366},
	{0x0990, 0x00A6},
	{0x098C, 0xA367},
	{0x0990, 0x0096},
	{0x098C, 0xA368},
	{0x0990, 0x005C},

	
	{0x098C, 0x2B1B},
	{0x0990, 0x0000},
	{0x098C, 0x2B28},
	{0x0990, 0x157C},
	{0x098C, 0x2B2A},
	{0x0990, 0x1B58},
	{0x098C, 0xAB37},
	{0x0990, 0x0001},
	{0x098C, 0x2B38},
	{0x0990, 0x157C},
	{0x098C, 0x2B3A},
	{0x0990, 0x1B58},

	
	{0x098C, 0xAB3C},
	{0x0990, 0x0000},
	{0x098C, 0xAB3D},
	{0x0990, 0x0004},
	{0x098C, 0xAB3E},
	{0x0990, 0x000E},
	{0x098C, 0xAB3F},
	{0x0990, 0x0029},
	{0x098C, 0xAB40},
	{0x0990, 0x0050},
	{0x098C, 0xAB41},
	{0x0990, 0x006A},
	{0x098C, 0xAB42},
	{0x0990, 0x0081},
	{0x098C, 0xAB43},
	{0x0990, 0x0094},
	{0x098C, 0xAB44},
	{0x0990, 0x00A5},
	{0x098C, 0xAB45},
	{0x0990, 0x00B2},
	{0x098C, 0xAB46},
	{0x0990, 0x00BE},
	{0x098C, 0xAB47},
	{0x0990, 0x00C9},
	{0x098C, 0xAB48},
	{0x0990, 0x00D3},
	{0x098C, 0xAB49},
	{0x0990, 0x00DC},
	{0x098C, 0xAB4A},
	{0x0990, 0x00E4},
	{0x098C, 0xAB4B},
	{0x0990, 0x00EB},
	{0x098C, 0xAB4C},
	{0x0990, 0x00F2},
	{0x098C, 0xAB4D},
	{0x0990, 0x00F9},
	{0x098C, 0xAB4E},
	{0x0990, 0x00FF},
	
	{0x098C, 0xAB4F},
	{0x0990, 0x0000},
	{0x098C, 0xAB50},
	{0x0990, 0x000F},
	{0x098C, 0xAB51},
	{0x0990, 0x001A},
	{0x098C, 0xAB52},
	{0x0990, 0x002E},
	{0x098C, 0xAB53},
	{0x0990, 0x0050},
	{0x098C, 0xAB54},
	{0x0990, 0x006A},
	{0x098C, 0xAB55},
	{0x0990, 0x0080},
	{0x098C, 0xAB56},
	{0x0990, 0x0091},
	{0x098C, 0xAB57},
	{0x0990, 0x00A1},
	{0x098C, 0xAB58},
	{0x0990, 0x00AF},
	{0x098C, 0xAB59},
	{0x0990, 0x00BB},
	{0x098C, 0xAB5A},
	{0x0990, 0x00C6},
	{0x098C, 0xAB5B},
	{0x0990, 0x00D0},
	{0x098C, 0xAB5C},
	{0x0990, 0x00D9},
	{0x098C, 0xAB5D},
	{0x0990, 0x00E2},
	{0x098C, 0xAB5E},
	{0x0990, 0x00EA},
	{0x098C, 0xAB5F},
	{0x0990, 0x00F1},
	{0x098C, 0xAB60},
	{0x0990, 0x00F9},
	{0x098C, 0xAB61},
	{0x0990, 0x00FF},
	{0x098C, 0x2755 },
	{0x0990, 0x0002 },

	
	{0x098C, 0x2757},
	{0x0990, 0x0002},
	{0x098C, 0xA20C},
	{0x0990, 0x000C},

	{0x098C, 0xA103 },
	{0x0990, 0x0006 },
	{SENSOR_WRITE_DELAY, 0x24},



	{0x098C, 0xA103 },
	{0x0990, 0x0005 },
	{SENSOR_WRITE_DELAY, 0x10},

	{ 0x098C, 0xA244},
	{ 0x0990, 0x00BB},
};
#endif
static struct v4l2_subdev_info mi380_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id mi380_i2c_id[] = {
	{MI380_SENSOR_NAME, (kernel_ulong_t)&mi380_s_ctrl},
	{ }
};

static int32_t msm_mi380_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &mi380_s_ctrl);
}

static struct i2c_driver mi380_i2c_driver = {
	.id_table = mi380_i2c_id,
	.probe  = msm_mi380_i2c_probe,
	.driver = {
		.name = MI380_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mi380_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id mi380_dt_match[] = {
	{.compatible = "htc,mi380", .data = &mi380_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, mi380_dt_match);

static struct platform_driver mi380_platform_driver = {
	.driver = {
		.name = "htc,mi380",
		.owner = THIS_MODULE,
		.of_match_table = mi380_dt_match,
	},
};

static const char *MI380Vendor = "mi";
static const char *MI380NAME = "mi380";
static const char *MI380Size = "VGA";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", MI380Vendor, MI380NAME, MI380Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);

static struct kobject *android_mi380;

static int mi380_sysfs_init(void)
{
	int ret ;
	
	android_mi380 = kobject_create_and_add("android_camera2", NULL);
	if (android_mi380 == NULL) {
		pr_info("mi380_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	
	ret = sysfs_create_file(android_mi380, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("mi380_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_mi380);
	}

	return 0 ;
}

static int32_t mi380_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(mi380_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init mi380_init_module(void)
{
	int32_t rc = 0;
	
	pr_info("mi380_init_module");
	rc = platform_driver_probe(&mi380_platform_driver,
		mi380_platform_probe);
	if (!rc) {
		mi380_sysfs_init();
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&mi380_i2c_driver);
}

static void __exit mi380_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (mi380_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&mi380_s_ctrl);
		platform_driver_unregister(&mi380_platform_driver);
	} else
		i2c_del_driver(&mi380_i2c_driver);
	return;
}

int32_t mi380_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
	
    

    s_ctrl->power_setting_array.power_setting = mi380_power_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(mi380_power_setting);
    status = msm_sensor_power_up(s_ctrl);
    
    return status;
}

int32_t mi380_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t status;
    int i = 0;
    int j = 0;
    int data_size;
    
    s_ctrl->power_setting_array.power_setting = mi380_power_down_setting;
    s_ctrl->power_setting_array.size = ARRAY_SIZE(mi380_power_down_setting);

    
    for(i = 0; i < s_ctrl->power_setting_array.size;  i++)
    {
        data_size = sizeof(mi380_power_setting[i].data)/sizeof(void *);
        for (j =0; j < data_size; j++ )
        mi380_power_down_setting[i].data[j] = mi380_power_setting[i].data[j];
    }
    status = msm_sensor_power_down(s_ctrl);
    
    return status;
}

static int mi380_i2c_write(unsigned short saddr,
				 unsigned short waddr, unsigned short wdata,
				 enum mi380_width width)
{
	int rc = -EIO;

	switch (width) {
	case WORD_LEN:{
			


            rc =mi380_sensor_i2c_client.i2c_func_tbl->i2c_write(&mi380_sensor_i2c_client,waddr, wdata ,MSM_CAMERA_I2C_WORD_DATA);
		}
		break;

	case BYTE_LEN:{
			

            rc =mi380_sensor_i2c_client.i2c_func_tbl->i2c_write(&mi380_sensor_i2c_client,waddr, wdata ,MSM_CAMERA_I2C_BYTE_DATA);
		}
		break;

	default:
		break;
	}

	if (rc < 0)
		pr_info("i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
}

static int mi380_i2c_write_table(struct mi380_i2c_reg_conf
				       *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mi380_i2c_write(mi380_client->addr,
				       reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				       reg_conf_tbl->width);
		if (rc < 0) {
		pr_err("%s: num_of_items_in_table=%d\n", __func__,
			num_of_items_in_table);
			break;
		}
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static int32_t mi380_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;

	if (!rdata)
		return -EIO;


	rc =mi380_sensor_i2c_client.i2c_func_tbl->i2c_read(&mi380_sensor_i2c_client,raddr, rdata ,MSM_CAMERA_I2C_WORD_DATA);

	if (rc < 0)
		pr_err("mi380_i2c_read_w failed!\n");

	return rc;
}


static int mi380_i2c_write_bit(unsigned short saddr, unsigned short raddr,
unsigned short bit, unsigned short state)
{
	int rc;
	unsigned short check_value;
	unsigned short check_bit;

	if (state)
		check_bit = 0x0001 << bit;
	else
		check_bit = 0xFFFF & (~(0x0001 << bit));
	pr_debug("mi380_i2c_write_bit check_bit:0x%4x", check_bit);
	rc = mi380_i2c_read_w(saddr, raddr, &check_value);
	if (rc < 0)
	  return rc;

	pr_debug("%s: mi380: 0x%4x reg value = 0x%4x\n", __func__,
		raddr, check_value);
	if (state)
		check_value = (check_value | check_bit);
	else
		check_value = (check_value & check_bit);

	pr_debug("%s: mi380: Set to 0x%4x reg value = 0x%4x\n", __func__,
		raddr, check_value);

	rc = mi380_i2c_write(saddr, raddr, check_value,
		WORD_LEN);
	return rc;
}

static int mi380_i2c_check_bit(unsigned short saddr, unsigned short raddr,
unsigned short bit, int check_state)
{
	int k;
	unsigned short check_value;
	unsigned short check_bit;
	check_bit = 0x0001 << bit;
	for (k = 0; k < CHECK_STATE_TIME; k++) {
		mi380_i2c_read_w(mi380_client->addr,
			      raddr, &check_value);
		if (check_state) {
			if ((check_value & check_bit))
			break;
		} else {
			if (!(check_value & check_bit))
			break;
		}
		msleep(1);
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s failed addr:0x%2x data check_bit:0x%2x",
			__func__, raddr, check_bit);
		return -1;
	}
	return 1;
}

static inline int resume(void)
{
	int k = 0, rc = 0;
	unsigned short check_value;

	
	
	rc = mi380_i2c_read_w(mi380_client->addr, 0x0016, &check_value);
	if (rc < 0)
	  return rc;

	
	

	check_value = (check_value|0x0020);

	
	

	rc = mi380_i2c_write(mi380_client->addr, 0x0016, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("%s: Enter Active mode fail\n", __func__);
		return rc;
	}

	
	
	rc = mi380_i2c_read_w(mi380_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	
	

	check_value = (check_value & 0xFFFE);

	
	

	rc = mi380_i2c_write(mi380_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("%s: Enter Active mode fail\n", __func__);
		return rc;
	}

	
	for (k = 0; k < CHECK_STATE_TIME; k++) {
		mi380_i2c_read_w(mi380_client->addr,
			  0x0018, &check_value);

		
		

		if (!(check_value & 0x4000)) {
			pr_info("%s: (check 0x0018[14] is 0) k=%d\n",
				__func__, k);
			break;
		}
		msleep(1);	
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s: check status time out (check 0x0018[14] is 0)\n",
			__func__);
		return -EIO;
	}

	
	for (k = 0; k < CHECK_STATE_TIME; k++) {
		mi380_i2c_read_w(mi380_client->addr,
			  0x301A, &check_value);
		if (check_value & 0x0004) {
			
			
			break;
		}
		msleep(1);	
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s: check status time out (check 0x301A[2] is 1)\n",
			__func__);
		return -EIO;
	}

	
	for (k = 0; k < CHECK_STATE_TIME; k++) {
		rc = mi380_i2c_read_w(mi380_client->addr, 0x31E0,
			&check_value);
		if (check_value == 0x0003) { 
			
			
			break;
		}
		msleep(1);	
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s: check status time out (check 0x31E0 is 0x003 )\n",
			__func__);
		return -EIO;
	}

	
	rc = mi380_i2c_write(mi380_client->addr, 0x31E0, 0x0001,
	WORD_LEN);
	if (rc < 0) {
		pr_err("%s: Enter Active mode fail\n", __func__);
		return rc;
	}

    msleep(2);

	return rc;
}

static inline int suspend(void)
{
	int k = 0, rc = 0;
	unsigned short check_value;

	
	
	rc = mi380_i2c_read_w(mi380_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	check_value = (check_value|0x0008);

	

	rc = mi380_i2c_write(mi380_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("%s: Enter standy mode fail\n", __func__);
		return rc;
	}
	
	rc = mi380_i2c_read_w(mi380_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	check_value = (check_value|0x0001);

	
	

	

	rc = mi380_i2c_write(mi380_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("%s: Enter standy mode fail\n", __func__);
		return rc;
	}

	
	for (k = 0; k < CHECK_STATE_TIME; k++) {
		mi380_i2c_read_w(mi380_client->addr,
			  0x0018, &check_value);
		if ((check_value & 0x4000)) { 
		
		
			break;
		}
		msleep(1);	
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s: check status time out\n", __func__);
		return -EIO;
	}
    msleep(2);
	return rc;
}

static int mi380_reg_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, k = 0;
	unsigned short check_value;

    
	
	rc = mi380_i2c_write(mi380_client->addr,
					0x0018, 0x4028, WORD_LEN);
	if (rc < 0)
		goto reg_init_fail;

	rc = mi380_i2c_check_bit(mi380_client->addr, 0x0018, 14, 0);
	if (rc < 0)
		goto reg_init_fail;

	
	rc = mi380_i2c_check_bit(mi380_client->addr, 0x301A, 2, 1);
	if (rc < 0)
		goto reg_init_fail;

	rc = mi380_i2c_write_table(&mi380_regs.power_up_tbl[0],
				     mi380_regs.power_up_tbl_size);
	if (rc < 0) {
		pr_err("%s: Power Up fail\n", __func__);
		goto reg_init_fail;
	}
	
    if(suspend_fail_retry_count_2 != SUSPEND_FAIL_RETRY_MAX_2) {
        pr_info("%s: added additional delay count=%d\n", __func__, suspend_fail_retry_count_2);
        mdelay(20);
    }
    
	
	

	rc = mi380_i2c_write(mi380_client->addr,
					0x0018, 0x4028, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_check_bit(mi380_client->addr, 0x0018, 14, 0);
	if (rc < 0)
		goto reg_init_fail;

	
	rc = mi380_i2c_check_bit(mi380_client->addr, 0x301A, 2, 1);
	if (rc < 0)
		goto reg_init_fail;

	
	rc = mi380_i2c_write_bit(mi380_client->addr, 0x31E0, 1, 0);
	if (rc < 0)
		goto reg_init_fail;

	if (g_csi_if) {
	    
	    rc = mi380_i2c_write_bit(mi380_client->addr, 0x001A, 9, 0);
	    if (rc < 0)
	      goto reg_init_fail;

	    
	    
	    
	    


		
		for (k = 0; k < CHECK_STATE_TIME; k++) {
			rc = mi380_i2c_read_w(mi380_client->addr, 0x3400,
				&check_value);
			
			if (check_value & 0x0010) { 
			
			
			break;
		} else {
			check_value = (check_value | 0x0010);
			
				rc = mi380_i2c_write(mi380_client->addr, 0x3400,
				check_value, WORD_LEN);
			if (rc < 0)
				goto reg_init_fail;
		}
			msleep(1);	
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("%s: check status time out (check 0x3400[4] is 1 )\n",
				__func__);
			goto reg_init_fail;
		}

		mdelay(10);
	    
	    rc = mi380_i2c_write_bit(mi380_client->addr, 0x3400, 9, 1);
	    if (rc < 0)
	      goto reg_init_fail;

		
		for (k = 0; k < CHECK_STATE_TIME; k++) {
			rc = mi380_i2c_read_w(mi380_client->addr, 0x3400,
				&check_value);
			
			if (check_value & 0x0200) { 
				
				
				break;
			} else {
				check_value = (check_value | 0x0200);
				
				rc = mi380_i2c_write(mi380_client->addr, 0x3400,
					check_value, WORD_LEN);
				if (rc < 0)
					goto reg_init_fail;
			}
			msleep(1);	
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("%s: check status time out (check 0x3400[9] is 1 )\n",
				__func__);
			goto reg_init_fail;
		}

	    
	    rc = mi380_i2c_write_bit(mi380_client->addr, 0x321C, 7, 0);
	    if (rc < 0)
	      goto reg_init_fail;
	} else {
	    rc = mi380_i2c_write(mi380_client->addr, 0x001A, 0x0210, WORD_LEN);
	    if (rc < 0)
	      goto reg_init_fail;
	}

	rc = mi380_i2c_write(mi380_client->addr, 0x001E, 0x0777, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;


	rc = mi380_i2c_write(mi380_client->addr, 0x0016, 0x42DF, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;


	
	rc = mi380_i2c_write(mi380_client->addr, 0x0014, 0xB04B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0014, 0xB049, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0010, 0x021C, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0012, 0x0000, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0014, 0x244B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	msleep(1);

	rc = mi380_i2c_write(mi380_client->addr, 0x0014, 0x304B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_check_bit(mi380_client->addr, 0x0014, 15, 1);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0014, 0xB04A, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	
	rc = mi380_i2c_write_table(&mi380_regs.register_init_1[0],
			mi380_regs.register_init_size_1);
	if (rc < 0)
	  goto reg_init_fail;

	
	rc = mi380_i2c_write_bit(mi380_client->addr, 0x3210, 3, 1);
	if (rc < 0)
	  goto reg_init_fail;

	
	rc = mi380_i2c_write_table(&mi380_regs.register_init_2[0],
			mi380_regs.register_init_size_2);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0006, WORD_LEN);
	for (k = 0; k < CHECK_STATE_TIME; k++) {  
		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) 
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME)
		goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0005, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	for (k = 0; k < CHECK_STATE_TIME; k++) {  
		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) 
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME)
		goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA102, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x000F, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	return rc;
reg_init_fail:
	pr_err("mi380 register initial fail\n");
	return rc;
}

int mi380_sensor_open_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t check_value = 0;

	


	suspend_fail_retry_count_2 = SUSPEND_FAIL_RETRY_MAX_2;


probe_suspend_fail_retry_2:
		

		

		
		rc = mi380_reg_init(s_ctrl);
		if (rc < 0) {
			pr_err("%s: mi380_reg_init fail\n", __func__);

			if (suspend_fail_retry_count_2 > 0) {
				suspend_fail_retry_count_2--;
				pr_info("%s: mi380 reg_init fail start retry mechanism !!!\n", __func__);
				goto probe_suspend_fail_retry_2;
			}

			goto init_fail;
		}

		
		
		rc = mi380_i2c_read_w(mi380_client->addr, 0x0016, &check_value);
		if (rc < 0)
		  return rc;

		
		

		check_value = (check_value&0xFFDF);

		
		

		rc = mi380_i2c_write(mi380_client->addr, 0x0016,
			check_value, WORD_LEN);
		if (rc < 0) {
			pr_err("%s: Enter Standby mode fail\n", __func__);
			return rc;
		}
	 

	just_power_on = 0;
	goto init_done;

init_fail:
	pr_info("%s init_fail\n", __func__);
	
	
	return rc;
init_done:
	pr_info("%s init_done\n", __func__);
	return rc;

}

enum sensor_mode {
	SENSOR_PREVIEW_MODE,
	SENSOR_SNAPSHOT_MODE
};

static int mi380_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl, int mode)
{
	int rc = 0 , k;
	uint16_t check_value = 0;

	

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		op_mode = SENSOR_PREVIEW_MODE;
		

		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0002,
		WORD_LEN);
		if (rc < 0)
			return rc;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA104,	WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			
			if (check_value == 0x0003) 
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("%s: Preview fail\n", __func__);
			return -EIO;
		}

		
		

		break;
	case SENSOR_SNAPSHOT_MODE:
		op_mode = SENSOR_SNAPSHOT_MODE;
		
		

		rc = mi380_i2c_write(mi380_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mi380_i2c_write(mi380_client->addr, 0x0990, 0x0001,
		WORD_LEN);
		if (rc < 0)
			return rc;

		for (k = 0; k < CHECK_STATE_TIME; k++) {
			rc = mi380_i2c_write(mi380_client->addr, 0x098C,
				0xA104, WORD_LEN);
			rc = mi380_i2c_read_w(mi380_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0003)
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("%s: Snapshot fail\n", __func__);
			return -EIO;
		}
		break;

	default:
		return -EINVAL;
	}

	
	return rc;
}

int32_t mi380_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	
	
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		
		
		
		
		for (i = 0; i < SUB_MODULE_MAX; i++)
			pr_info("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
#if 0
		{
		
		struct msm_camera_i2c_reg_setting conf_array;
		conf_array.delay = 5;
		conf_array.reg_setting = mi380_recommend_settings;
		conf_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		conf_array.data_type = MSM_CAMERA_I2C_WORD_DATA;
		conf_array.size = ARRAY_SIZE(mi380_recommend_settings);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		}
#endif
		mi380_sensor_open_init(s_ctrl);
		mi380_set_fps(s_ctrl, 1015);
		break;
	case CFG_SET_RESOLUTION:
		if (copy_from_user(&op_mode,
					(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_sensor_mode(s_ctrl, op_mode);
		break;
	case CFG_SET_STOP_STREAM:
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			mi380_stop_settings,
			ARRAY_SIZE(mi380_stop_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		
		
		
		break;

	case CFG_SET_START_STREAM:
		
		
		
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			mi380_start_settings,
			ARRAY_SIZE(mi380_start_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		pr_info("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;

		if (!power_setting_array->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
			(void *)
			sensor_slave_info.power_setting_array.power_setting,
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(power_setting_array->power_setting);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		pr_info("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		pr_info("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		pr_info("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		pr_info("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			pr_info("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		if (conf_array.cmd_type == MSM_CAMERA_I2C_COMMAND_POLL) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_poll_table(
				s_ctrl->sensor_i2c_client, &conf_array);
		} else {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
		}

		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		if (copy_from_user(&read_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		pr_info("%s:CFG_SLAVE_READ_I2C:", __func__);
		pr_info("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		pr_info("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		if (copy_to_user((void __user *)read_config.data,
			(void *)&local_data, sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_info("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		pr_info("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		pr_info("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		just_power_on = 1;
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->stop_setting_valid = 1;
		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			stop_setting->reg_setting = NULL; 
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
#ifdef CONFIG_RAWCHIPII
	case CFG_RAWCHIPII_SETTING:
		if (s_ctrl->sensordata->htc_image != 1) 
			break;

		{
		struct msm_rawchip2_cfg_data *cfg_data = NULL;

		cfg_data = kzalloc(
			(sizeof(struct msm_rawchip2_cfg_data)), GFP_KERNEL);
		if (!cfg_data) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(cfg_data, (void *)cdata->cfg.setting,
			sizeof(struct msm_rawchip2_cfg_data))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(cfg_data);
			rc = -EFAULT;
			break;
		}


		YushanII_Init(s_ctrl,cfg_data);
		break;
	}

	case CFG_RAWCHIPII_STOP:
		if (s_ctrl->sensordata->htc_image != 1) 
			break;

		if(YushanII_Get_reloadInfo() == 0){
			pr_info("stop YushanII first");
			Ilp0100_stop();
		}

		break;
#endif

	case CFG_I2C_IOCTL_R_OTP:
		if (s_ctrl->func_tbl->sensor_i2c_read_fuseid == NULL) {
			rc = -EFAULT;
			break;
		}
		rc = s_ctrl->func_tbl->sensor_i2c_read_fuseid(cdata, s_ctrl);
	break;
	case CFG_SET_SATURATION: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_saturation(s_ctrl, sv);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
				       sizeof(int32_t))) {
		        pr_err("%s:%d failed\n", __func__, __LINE__);
		        rc = -EFAULT;
		        break;
		}
		
		mi380_set_contrast(s_ctrl, sv);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
					+                       sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_sharpness(s_ctrl, sv);
		break;
	}
	case CFG_SET_ISO: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_iso(s_ctrl, sv);

		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		
		mi380_set_brightness(s_ctrl, sv);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_SetEffect(s_ctrl, sv);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_antibanding(s_ctrl, sv);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		

		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t sv;
		if (copy_from_user(&sv, (void *)cdata->cfg.setting,
			 sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		mi380_set_wb(s_ctrl, sv);
		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#if 0
int32_t mi380_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	pr_info("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		pr_info("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		pr_info("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			pr_info("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		
		pr_err("%s, sensor write init setting!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			mi380_recommend_settings,
			ARRAY_SIZE(mi380_recommend_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		break;
	case CFG_SET_RESOLUTION:
		break;
	case CFG_SET_STOP_STREAM:
		pr_err("%s, sensor stop stream!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			mi380_stop_settings,
			ARRAY_SIZE(mi380_stop_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		break;
	case CFG_SET_START_STREAM:
		pr_err("%s, sensor start stream!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			mi380_start_settings,
			ARRAY_SIZE(mi380_start_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		pr_info("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		if (sensor_slave_info.slave_addr) {
		
		
		}

		
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		pr_info("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		pr_info("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		pr_info("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		pr_info("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			pr_info("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SET_SATURATION: {

		break;
	}
	case CFG_SET_CONTRAST: {

		break;
	}
	case CFG_SET_SHARPNESS: {

		break;
	}
	case CFG_SET_ISO: {

		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {

		break;
	}
	case CFG_SET_EFFECT: {

		break;
	}
	case CFG_SET_ANTIBANDING: {

		break;
	}
	case CFG_SET_BESTSHOT_MODE: {

		break;
	}
	case CFG_SET_WHITE_BALANCE: {

		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

static struct msm_sensor_fn_t mi380_sensor_func_tbl = {
	.sensor_config = mi380_sensor_config,
	.sensor_power_up = mi380_sensor_power_up,
	.sensor_power_down = mi380_sensor_power_down,
};

static struct msm_sensor_ctrl_t mi380_s_ctrl = {
	.sensor_i2c_client = &mi380_sensor_i2c_client,
	.power_setting_array.power_setting = mi380_power_setting,
	.power_setting_array.size = ARRAY_SIZE(mi380_power_setting),
	.msm_sensor_mutex = &mi380_mut,
	.sensor_v4l2_subdev_info = mi380_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mi380_subdev_info),
	.func_tbl = &mi380_sensor_func_tbl,
};

module_init(mi380_init_module);
module_exit(mi380_exit_module);
MODULE_DESCRIPTION("mi380");
MODULE_LICENSE("GPL v2");

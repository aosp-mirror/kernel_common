/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MSM_SENSOR_H
#define MSM_SENSOR_H

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <mach/camera2.h>
#include <media/msm_cam_sensor.h>
#include <media/v4l2-subdev.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_sd.h"
#include "../yushanII/ilp0100_ST_definitions.h"
#include <media/linux_yushanii.h>

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

struct msm_sensor_ctrl_t;

enum msm_sensor_state_t {
	MSM_SENSOR_POWER_DOWN,
	MSM_SENSOR_POWER_UP,
};

struct msm_sensor_fn_t {
	int (*sensor_config) (struct msm_sensor_ctrl_t *, void __user *);
	int (*sensor_power_down) (struct msm_sensor_ctrl_t *);
	int (*sensor_power_up) (struct msm_sensor_ctrl_t *);
	int (*sensor_match_id) (struct msm_sensor_ctrl_t *);
    void (*sensor_yushanII_set_IQ)(struct msm_sensor_ctrl_t *sensor,int*,int*,int*,struct yushanii_cls*);
	int (*sensor_i2c_read_fuseid)(struct sensorb_cfg_data *cdata, struct msm_sensor_ctrl_t *s_ctrl); 
    void (*sensor_yushanII_restart_stream)(void);
    void (*sensor_yushanII_stop_restart_stream)(void);
};


struct msm_sensor_ctrl_t {
	struct platform_device *pdev;
	struct mutex *msm_sensor_mutex;

	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_sensor_board_info *sensordata;
	struct msm_sensor_power_setting_array power_setting_array;
	struct msm_sensor_packed_cfg_t *cfg_override;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_i2c_master;

	struct msm_camera_i2c_client *sensor_i2c_client;
	struct device *dev;
	struct v4l2_subdev_info *sensor_v4l2_subdev_info;
	uint8_t sensor_v4l2_subdev_info_size;
	struct v4l2_subdev_ops *sensor_v4l2_subdev_ops;
	struct msm_sensor_fn_t *func_tbl;
	struct msm_camera_i2c_reg_setting stop_setting;
	bool stop_setting_valid;
	bool free_power_setting;
	struct msm_cam_clk_info *clk_info;
	uint16_t clk_info_size;
	void *misc_regulator;
	enum msm_sensor_state_t sensor_state;
	uint8_t is_probe_succeed;
	uint32_t id;
	struct device_node *of_node;

	uint8_t driver_ic; 
};

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp);

int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl);

int32_t msm_sensor_platform_probe(struct platform_device *pdev,
	void *data);

int msm_sensor_update_cfg(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id, struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl);

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl);

struct file* msm_fopen(const char* path, int flags, int rights);
int msm_fwrite(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
void msm_fclose(struct file* file);
void msm_dump_otp_to_file(const char* sensor_name, const short* add, const uint8_t* data, size_t count);

int32_t msm_sensor_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);

int32_t msm_sensor_get_dt_gpio_set_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);

int32_t msm_sensor_init_gpio_pin_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);
#endif

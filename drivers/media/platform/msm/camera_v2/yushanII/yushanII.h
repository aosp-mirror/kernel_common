/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#ifndef YUSHANII_H
#define YUSHANII_H

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <mach/board.h>

#include <linux/kernel.h>
#include <linux/string.h>

#include <media/linux_yushanii.h>
#include "../msm.h"
#include "../sensor/msm_sensor.h"
#include "../sensor/io/msm_camera_io_util.h"


struct YushanII_ctrl {
	struct msm_camera_rawchip_info *pdata;
	struct cdev   cdev;

	struct mutex raw_ioctl_lock;
	int rawchip_init;
	atomic_t check_intr0;
	atomic_t check_intr1;
	int total_error_interrupt_times;

	
	struct device *dev;
	struct clk *rawchip_clk[2];
	struct device *sensor_dev;
	
	void (*raw2_restart_stream)(void);
	void (*raw2_stop_restart_stream)(void);
};

struct YushanII_sensor_data {
	const char *sensor_name;
	uint8_t datatype;
	uint8_t lane_cnt;
	uint32_t pixel_clk;
	uint16_t width;
	uint16_t height;
	uint16_t line_length_pclk;
	uint16_t frame_length_lines;
	uint16_t fullsize_width;
	uint16_t fullsize_height;
	uint16_t fullsize_line_length_pclk;
	uint16_t fullsize_frame_length_lines;
	int mirror_flip;
	uint16_t x_addr_start;
	uint16_t y_addr_start;
	uint16_t x_addr_end;
	uint16_t y_addr_end;
	uint16_t x_even_inc;
	uint16_t x_odd_inc;
	uint16_t y_even_inc;
	uint16_t y_odd_inc;
	uint8_t binning_rawchip;
	uint8_t use_rawchip;
};

struct YushanII_id_info_t {
	uint16_t YushanII_id_reg_addr;
	uint32_t YushanII_id;
};

struct YushanII_info_t {
	struct YushanII_id_info_t *yushanII_id_info;
};

void YushanII_Init(struct msm_sensor_ctrl_t *sensor,struct msm_rawchip2_cfg_data *cfg_data);
int YushanII_Get_reloadInfo(void);
void YushanII_reload_firmware(void);
int YushanII_probe_init(struct device *dev);
void YushanII_probe_deinit(void);
void YushanII_release(void);
int YushanII_open_init(void);

int YushanII_set_channel_offset_debug(void __user *argp);
int YushanII_set_channel_offset(int channel_offset);
int YushanII_set_defcor_debug(void __user *argp);
int YushanII_set_defcor(int defcor);
int YushanII_set_tone_mapping_debug(void __user *argp);
int YushanII_set_tone_mapping(int tone_map);
int YushanII_set_cls_debug(void __user *argp);
int YushanII_set_cls(struct yushanii_cls *cls);
int YushanII_set_hdr_merge(struct yushanii_hdr_merge hdr_merge);
void YushanII_set_hdr_exp(uint16_t gain, uint16_t dig_gain, uint32_t long_line, uint32_t short_line);
int YushanII_set_each_channel_offset(int channel_B_offset, int channel_GB_offset, int channel_GR_offset, int channel_R_offset);
int YushanII_set_sensor_specific_function(struct msm_sensor_fn_t *sensor_func_tbl);
void YushanII_Reset(void);
#endif

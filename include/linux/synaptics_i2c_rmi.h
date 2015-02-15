/*
 * include/linux/synaptics_i2c_rmi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_SYNAPTICS_I2C_RMI_H
#define _LINUX_SYNAPTICS_I2C_RMI_H

#define SYNAPTICS_I2C_RMI_NAME "synaptics-rmi-ts"
#define SYNAPTICS_T1007_NAME "synaptics-t1007"
#define SYNAPTICS_T1021_NAME "synaptics-t1021"
#define SYNAPTICS_3K_NAME "synaptics-3k"
#define SYNAPTICS_3K_INCELL_NAME "synaptics-3k-incell"
#define SYNAPTICS_3200_NAME "synaptics-3200"
#define SYNAPTICS_FW_3_2_PACKRAT 1115999
#define SYNAPTICS_FW_NOCAL_PACKRAT 1293981
#define SYNAPTICS_FW_2IN1_PACKRAT 1396865
#define SYNAPTICS_FW_35_COVER 1661078
#define SYNAPTICS_FW_3508_COVER 1741220

#define SYN_CFG_BLK_UNIT	(16)
#define SYN_CONFIG_SIZE 	(32 * SYN_CFG_BLK_UNIT)
#define SYN_CONFIG_SIZE_35XX 	(64 * SYN_CFG_BLK_UNIT)
#define SYN_MAX_PAGE 5
#define SYN_BL_PAGE 1
#define SYN_VK_ATTR_STRING	(25)
#define SYN_F01DATA_BASEADDR 0x0013
#define SYN_PROCESS_ERR -1

#define SYN_AND_REPORT_TYPE_A		0
#define	SYN_AND_REPORT_TYPE_B		1
#define SYN_AND_REPORT_TYPE_HTC		2

#define TAP_DX_OUTER		0
#define TAP_DY_OUTER		1
#define TAP_TIMEOUT		2
#define TAP_DX_INTER		3
#define TAP_DY_INTER		4

#define CUS_REG_SIZE		4
#define CUS_REG_BASE		0
#define CUS_BALLISTICS_CTRL	1
#define CUS_LAND_CTRL		2
#define CUS_LIFT_CTRL		3

#define SENSOR_ID_CHECKING_EN	1 << 16

enum {
	SYNAPTICS_FLIP_X = 1UL << 0,
	SYNAPTICS_FLIP_Y = 1UL << 1,
	SYNAPTICS_SWAP_XY = 1UL << 2,
	SYNAPTICS_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

enum {
	FINGER_1_REPORT = 1 << 0,
	FINGER_2_REPORT = 1 << 1,
};

struct synaptics_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
	char attr_range[SYN_VK_ATTR_STRING];
};

struct synaptics_config {
	uint8_t default_cfg;
	uint32_t sensor_id;
	bool     mfgconfig;
	uint32_t pr_number;
	uint16_t length;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	uint8_t  config[SYN_CONFIG_SIZE_35XX];
	uint8_t vkey_setting;
	uint8_t cover_setting[9];
};

struct synaptics_i2c_rmi_platform_data {
	uint32_t version;	
				
				
	int (*power)(int on);	
	struct synaptics_virtual_key *virtual_key;
	uint8_t virtual_key_num;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;
	uint8_t sensitivity;
	uint8_t finger_support;
	uint32_t gap_area;
	uint32_t key_area;
	uint32_t flags;
	unsigned long irqflags;
	uint32_t inactive_left; 
	uint32_t inactive_right; 
	uint32_t inactive_top; 
	uint32_t inactive_bottom; 
	uint32_t snap_left_on; 
	uint32_t snap_left_off; 
	uint32_t snap_right_on; 
	uint32_t snap_right_off; 
	uint32_t snap_top_on; 
	uint32_t snap_top_off; 
	uint32_t snap_bottom_on; 
	uint32_t snap_bottom_off; 
	uint32_t fuzz_x; 
	uint32_t fuzz_y; 
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int fuzz_p;
	int fuzz_w;
	uint32_t display_width;
	uint32_t display_height;
	int8_t sensitivity_adjust;
	uint32_t dup_threshold;
	uint32_t margin_inactive_pixel[4];
	uint16_t filter_level[4];
	uint8_t reduce_report_level[5];
	uint8_t noise_information;
	uint8_t jumpfq_enable;
	uint8_t cable_support;
	int config_length;
	uint8_t config[SYN_CONFIG_SIZE_35XX];
	int gpio_irq;
	uint32_t irq_gpio_flags;
	int gpio_reset;
	int gpio_i2c;
	uint8_t default_config;
	uint8_t report_type;
	uint8_t large_obj_check;
	uint16_t tw_pin_mask;
	uint32_t sensor_id;
	uint32_t packrat_number;
	uint8_t support_htc_event;
	uint8_t mfg_flag;
	uint8_t customer_register[CUS_REG_SIZE];
	uint8_t segmentation_bef_unlock;
	uint8_t threshold_bef_unlock;
	uint16_t saturation_bef_unlock;
	uint8_t i2c_err_handler_en;
	uint8_t energy_ratio_relaxation;
	uint8_t multitouch_calibration;
	uint8_t psensor_detection;
	uint8_t PixelTouchThreshold_bef_unlock;
	uint8_t cover_setting[9];
	uint16_t hall_block_touch_time;
};

struct page_description {
	uint8_t addr;
	uint8_t value;
};

struct syn_finger_data {
	int x;
	int y;
	int w;
	int z;
};

struct function_t {
	uint8_t function_type;
	uint8_t interrupt_source;
	uint16_t data_base;
	uint16_t control_base;
	uint16_t command_base;
	uint16_t query_base;
};
enum {
	QUERY_BASE,
	COMMAND_BASE,
	CONTROL_BASE,
	DATA_BASE,
	INTR_SOURCE,
	FUNCTION
};

extern uint8_t getPowerKeyState(void);
#endif 

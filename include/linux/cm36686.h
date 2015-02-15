/* include/linux/cm36686.h
 *
 * Copyright (C) 2010 HTC, Inc.
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

#ifndef __LINUX_CM36686_H
#define __LINUX_CM36686_H

#define CM36686_I2C_NAME "CM36686"

#define ALS_config_cmd		0x00
#define ALS_high_thd		0x01
#define ALS_low_thd		0x02

#define PS_config		0x03
#define PS_config_ms		0x04
#define PS_CANC			0x05
#define PS_thd_low		0x06
#define PS_thd_high		0x07
#define PS_data			0x08
#define ALS_data		0x09

#define WS_data			0x0A
#define INT_FLAG		0x0B
#define CH_ID			0x0C


#define ALS_CALIBRATED		0x6DA5
#define PS_CALIBRATED		0x5053

#define CM36686_ALS_IT_80ms 	(0 << 6)
#define CM36686_ALS_IT_160ms 	(1 << 6)
#define CM36686_ALS_IT_320ms 	(2 << 6)
#define CM36686_ALS_IT_640ms 	(3 << 6)
#define CM36686_ALS_PERS_1 	(0 << 2) 
#define CM36686_ALS_PERS_2 	(1 << 2)
#define CM36686_ALS_PERS_4 	(2 << 2)
#define CM36686_ALS_PERS_8 	(3 << 2)
#define CM36686_ALS_INT_EN	(1 << 1) 
#define CM36686_ALS_SD		(1 << 0) 

#define CM36686_PS_DR_1_40 	(0 << 6)
#define CM36686_PS_DR_1_80 	(1 << 6)
#define CM36686_PS_DR_1_160 	(2 << 6)
#define CM36686_PS_DR_1_320 	(3 << 6)
#define CM36686_PS_IT_1T 	(0 << 1)
#define CM36686_PS_IT_1_5T 	(1 << 1)
#define CM36686_PS_IT_2T 	(2 << 1)
#define CM36686_PS_IT_2_5T 	(3 << 1)
#define CM36686_PS_IT_3T         (4 << 1)
#define CM36686_PS_IT_3_5T       (5 << 1)
#define CM36686_PS_IT_4T         (6 << 1)
#define CM36686_PS_IT_8T         (7 << 1)
#define CM36686_PS_PERS_1 	(0 << 4)
#define CM36686_PS_PERS_2 	(1 << 4)
#define CM36686_PS_PERS_3 	(2 << 4)
#define CM36686_PS_PERS_4 	(3 << 4)
#define CM36686_PS_SD		(1 << 0) 

#define CM36686_PS_12BIT		(0 << 3)
#define CM36686_PS_16BIT         (1 << 3)
#define CM36686_PS_INT_DIS 	(0 << 0)
#define CM36686_PS_INT_CLS 	(1 << 0)
#define CM36686_PS_INT_AWY 	(2 << 0)
#define CM36686_PS_INT_BOTH	(3 << 0)


#define CM36686_PS_SMART_PRES_DIS	(1 << 4)
#define CM36686_PS_SMART_PRES_EN	(1 << 4)
#define CM36686_PS_AUTO	 	(0 << 3)
#define CM36686_PS_FORCE		(1 << 3)
#define CM36686_PS_TRIG_NO	(0 << 2)
#define CM36686_PS_TRIG_ONCE	(1 << 2)

#define CM36686_PS_MS_LOGIC 	(1 << 6)
#define CM36686_PS_MS_NORMAL	(0 << 6)
#define CM36686_PS_LED_50 	(0 << 0)
#define CM36686_PS_LED_75        (1 << 0)
#define CM36686_PS_LED_100       (2 << 0)
#define CM36686_PS_LED_120       (3 << 0)
#define CM36686_PS_LED_140       (4 << 0)
#define CM36686_PS_LED_160       (5 << 0)
#define CM36686_PS_LED_180       (6 << 0)
#define CM36686_PS_LED_200       (7 << 0)

#define CM36686_PS_PROTECTION	(1 << 6)
#define CM36686_ALS_IF_L 	(1 << 5)
#define CM36686_ALS_IF_H 	(1 << 4)
#define CM36686_PS_IF_CLOSE	(1 << 1)
#define CM36686_PS_IF_AWAY	(1 << 0)


int get_lightsensoradc(void);
int get_lightsensorkadc(void);

struct cm36686_platform_data {
	int model;
	int intr;
	uint32_t irq_gpio_flags;
	uint32_t *levels;
	uint32_t *correction;
	uint32_t golden_adc;
	int (*power)(int, uint8_t);	
	int (*lpm_power)(int on);	
	uint32_t cm36686_slave_address;
	uint32_t ps_thd_set;
	uint32_t ps_thh_diff;
	uint8_t inte_cancel_set;
	uint32_t ps_conf2_val;
	uint8_t *mapping_table;
	uint8_t mapping_size;
	uint8_t ps_base_index;
	uint32_t emmc_als_kadc;
	uint32_t emmc_ps_kadc1;
	uint32_t emmc_ps_kadc2;
	uint32_t ps_calibration_rule;
	uint32_t ps_conf1_val;
	uint8_t ps_conf3_val;
	uint32_t dynamical_threshold;
	uint32_t ps_thd_no_cal;
	uint32_t ps_th_add;
	uint8_t ls_cmd;
	uint8_t ps_debounce;
	uint16_t ps_delay_time;
	unsigned int no_need_change_setting;
	uint32_t dark_level;
	uint32_t use__PS2v85;
	uint8_t ps_ms_val;
	uint32_t SR_3v_used;
};

#endif

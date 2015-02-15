/*
********************* (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: lsm330.h
* Authors		: MH - C&I BU - Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
* Version		: V.1.2.3
* Date		: 2013/May/16
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************

********************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manage int1
				and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3 		Added rotation matrices
********************************************************************************
SYSFS interface
- range: set full scale
	-> accelerometer: 	2,4,6,8,16 				[g]
	-> gyroscope:		250,500,2000				[dps]
- poll_period_ms: set 1/ODR
	-> accelerometer:	LSM330_ACC_MIN_POLL_PERIOD_MS < t	[ms]
	-> gyroscope:		LSM330_GYR_MIN_POLL_PERIOD_MS < t	[ms]
- enable_device: enable/disable sensor					[1/0]


INPUT subsystem: NOTE-> output data INCLUDE the sensitivity in accelerometer,
			but NOT INCLUDE the sensitivity in gyroscope.
- accelerometer:	abs_x, abs_y, abs_z		[ug]
- gyroscope:		abs_x, abs_y, abs_z		[raw data]
*******************************************************************************/

#ifndef __LSM330_H__
#define __LSM330_H__


#define LSM330_ACC_DEV_NAME		"lsm330_acc"
#define LSM330_GYR_DEV_NAME		"lsm330_gyr"

#define CUSTOM_SYSFS_PATH
#define CUSTOM_SYSFS_CLASS_NAME_GYR	"ST_gyr"
#define CUSTOM_SYSFS_CLASS_NAME_ACC	"ST_acc"

#define LSM330_GYR_SAD0L		(0x00)
#define LSM330_ACC_SAD0L		(0x02)
#define LSM330_SAD0H			(0x01)
#define LSM330_ACC_I2C_SADROOT		(0x07)
#define LSM330_ACC_I2C_SAD_L		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_ACC_SAD0L)
#define LSM330_ACC_I2C_SAD_H		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_SAD0H)

#define LSM330_GYR_I2C_SADROOT		(0x35)
#define LSM330_GYR_I2C_SAD_L		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_GYR_SAD0L)
#define LSM330_GYR_I2C_SAD_H		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_SAD0H)

/* Poll Interval */
#define LSM330_ACC_MIN_POLL_PERIOD_MS		1

#define LSM330_GYR_MIN_POLL_PERIOD_MS		2


#ifdef __KERNEL__

/* Interrupt */
#define LSM330_ACC_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_ACC_DEFAULT_INT2_GPIO		(-EINVAL)

#define LSM330_GYR_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_GYR_DEFAULT_INT2_GPIO		(-EINVAL)


/* Accelerometer Sensor Full Scale */
#define LSM330_ACC_G_2G				(0x00)
#define LSM330_ACC_G_4G				(0x08)
#define LSM330_ACC_G_6G				(0x10)
#define LSM330_ACC_G_8G				(0x18)
#define LSM330_ACC_G_16G			(0x20)

/* Gyroscope Sensor Full Scale */
#define LSM330_GYR_FS_250DPS			(0x00)
#define LSM330_GYR_FS_500DPS			(0x10)
#define LSM330_GYR_FS_2000DPS			(0x30)

extern unsigned int gs_kvalue;
extern unsigned char gyro_gsensor_kvalue[37];

/*
static struct rot_matrix {
       short matrix[3][3];
	} rot_matrix[] = {
		[0] = {
				.matrix = {
						{1, 0, 0},
						{0, 1, 0},
						{0, 0, 1}, }
		},
		[1] = {
				.matrix = {
						{-1, 0, 0},
						{0, -1, 0},
						{0, 0, 1}, }
		},
		[2] = {
				.matrix = {
						{0, 1, 0},
						{-1, 0, 0},
						{0, 0, 1}, }
		},
		[3] = {
				.matrix = {
						{0, -1, 0},
						{1, 0, 0},
						{0, 0, 1}, }
		},
		[4] = {
				.matrix = {
						{0, -1, 0},
						{-1, 0, 0},
						{0, 0, -1}, }
		},
		[5] = {
				.matrix = {
						{0, 1, 0},
						{1, 0, 0},
						{0, 0, -1}, }
		},
		[6] = {
				.matrix = {
						{1, 0, 0},
						{0, -1, 0},
						{0, 0, -1}, }
		},
		[7] = {
				.matrix = {
						{-1, 0, 0},
						{0, 1, 0},
						{0, 0, -1}, }
		},
};
*/
struct lsm330_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	short rot_matrix_index;
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int chip_layout;
	int gs_kvalue;
	/* set gpio_int either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

struct lsm330_gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	short rot_matrix_index;
	/* fifo related */
	u8 watermark;
	u8 fifomode;
	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */
	unsigned char gyro_kvalue[37];
};
#endif	/* __KERNEL__ */

#endif	/* __LSM330_H__ */

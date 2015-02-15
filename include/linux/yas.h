/**
 * Header file of the core driver API @file yas.h
 *
 * Copyright (c) 2013-2014 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef __YAS_H__
#define __YAS_H__

#include "yas_cfg.h"

#define YAS_VERSION	"10.0.0"	


#if defined(__KERNEL__)
#include <linux/types.h>
#else
#include <stdint.h>
#endif


#define YAS_DEBUG			(0) 

#define YAS_NO_ERROR			(0) 
#define YAS_ERROR_ARG			(-1) 
#define YAS_ERROR_INITIALIZE		(-2) 
#define YAS_ERROR_BUSY			(-3) 
#define YAS_ERROR_DEVICE_COMMUNICATION	(-4) 
#define YAS_ERROR_CHIP_ID		(-5) 
#define YAS_ERROR_CALREG		(-6) 
#define YAS_ERROR_OVERFLOW		(-7) 
#define YAS_ERROR_UNDERFLOW		(-8) 
#define YAS_ERROR_DIRCALC		(-9) 
#define YAS_ERROR_ERROR			(-128) 

#ifndef NULL
#ifdef __cplusplus
#define NULL				(0) 
#else
#define NULL				((void *)(0)) 
#endif
#endif
#ifndef NELEMS
#define NELEMS(a)	((int)(sizeof(a)/sizeof(a[0]))) 
#endif
#ifndef ABS
#define ABS(a)		((a) > 0 ? (a) : -(a)) 
#endif
#ifndef M_PI
#define M_PI		(3.14159265358979323846) 
#endif

#define YAS_MATRIX_NORM		(10000) 
#define YAS_MATRIX_NORM_RECIP		(0x68DB8BAC)
#define YAS_MATRIX_NORM_RECIP_BIT	(18)
#define YAS_QUATERNION_NORM	(10000) 

#if YAS_DEBUG
#ifdef __KERNEL__
#include <linux/kernel.h>
#define YLOGD(args) (printk args)	
#define YLOGI(args) (printk args)	
#define YLOGE(args) (printk args)	
#define YLOGW(args) (printk args)	
#elif defined __ANDROID__
#include <cutils/log.h>
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "yas"
#define YLOGD(args) (ALOGD args)	
#define YLOGI(args) (ALOGI args)	
#define YLOGE(args) (ALOGE args)	
#define YLOGW(args) (ALOGW args)	
#else 
#include <stdio.h>
#define YLOGD(args) (printf args)	
#define YLOGI(args) (printf args)	
#define YLOGE(args) (printf args)	
#define YLOGW(args) (printf args)	
#endif 
#else 
#define YLOGD(args)	
#define YLOGI(args)	
#define YLOGW(args)	
#define YLOGE(args)	
#endif 

#define YAS_TYPE_ACC_NONE		(0x00000000) 
#define YAS_TYPE_MAG_NONE		(0x00000000) 
#define YAS_TYPE_GYRO_NONE		(0x00000000) 
#define YAS_TYPE_A_ACC			(0x00000001) 
#define YAS_TYPE_M_MAG			(0x00000002) 
#define YAS_TYPE_G_GYRO			(0x00000004) 
#define YAS_TYPE_AM_ACC			(0x00100000) 
#define YAS_TYPE_AM_MAG			(0x00200000) 
#define YAS_TYPE_AG_ACC			(0x01000000) 
#define YAS_TYPE_AG_GYRO		(0x02000000) 
#define YAS_TYPE_AMG_ACC		(0x10000000) 
#define YAS_TYPE_AMG_MAG		(0x20000000) 
#define YAS_TYPE_AMG_GYRO		(0x40000000) 

#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_NONE
#define YAS_TYPE_ACC YAS_TYPE_ACC_NONE
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222E
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250E
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA254
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMI055
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMI058
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_DMARD08
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTI9
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTJ2
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXUD9
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LSM330DLC
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_U2DH
#define YAS_TYPE_ACC YAS_TYPE_A_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_YAS535
#define YAS_TYPE_ACC YAS_TYPE_AM_ACC
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_YAS53x
#define YAS_TYPE_ACC YAS_TYPE_AMG_ACC
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_NONE
#define YAS_TYPE_MAG YAS_TYPE_MAG_NONE
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS529 
#define YAS_TYPE_MAG YAS_TYPE_M_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS530 
#define YAS_TYPE_MAG YAS_TYPE_M_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 
#define YAS_TYPE_MAG YAS_TYPE_M_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 
#define YAS_TYPE_MAG YAS_TYPE_M_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS535 
#define YAS_TYPE_MAG YAS_TYPE_AM_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS536 
#define YAS_TYPE_MAG YAS_TYPE_M_MAG
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS53x
#define YAS_TYPE_MAG YAS_TYPE_AMG_MAG
#endif

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_NONE
#define YAS_TYPE_GYRO YAS_TYPE_GYRO_NONE
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_BMI055
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_BMI058
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_EWTZMU
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_ITG3200
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_ITG3500
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G3200D
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_L3G4200D
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_LSM330DLC
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_MPU3050
#define YAS_TYPE_GYRO YAS_TYPE_G_GYRO
#elif YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_YAS53x
#define YAS_TYPE_GYRO YAS_TYPE_AMG_GYRO
#endif


#define YAS_MAG_CALIB_MODE_SPHERE		(0)
#define YAS_MAG_CALIB_MODE_ELLIPSOID		(1)
#define YAS_MAG_CALIB_MODE_SPHERE_WITH_GYRO	(2)
#define YAS_MAG_CALIB_MODE_ELLIPSOID_WITH_GYRO	(3)
#define YAS_MAG_CALIB_MODE_WITH_GYRO		(4)

#define YAS532_SELF_TEST		(0x00000001)
#define YAS532_SELF_TEST_NOISE		(0x00000002)
#define YAS532_GET_HW_OFFSET		(0x00000003)
#define YAS532_SET_HW_OFFSET		(0x00000004)
#define YAS532_GET_LAST_RAWDATA		(0x00000006)

#define YAS536_SELF_TEST		(0x00000001)
#define YAS536_GET_HW_OFFSET		(0x00000002)
#define YAS536_GET_AVERAGE_LEN		(0x00000004)
#define YAS536_SET_AVERAGE_LEN		(0x00000005)
#define YAS536_GET_LAST_RAWDATA		(0x00000006)


struct yas_vector {
	int32_t v[3]; 
};

struct yas_quaternion {
	int32_t q[4]; 
	int32_t heading_error; 
};

struct yas_matrix {
	int16_t m[9]; 
};

struct yas_data {
	int32_t type; 
	struct yas_vector xyz; 
	uint32_t timestamp; 
	uint8_t accuracy; 
};

/**
 * @struct yas_driver_callback
 * @brief User-written callback functions specific to each implementation, such
 * as communication controls with the device
 */
struct yas_driver_callback {
	int (*device_open)(int32_t type);
	int (*device_close)(int32_t type);
	int (*device_write)(int32_t type, uint8_t addr, const uint8_t *buf,
			int len);
	int (*device_read)(int32_t type, uint8_t addr, uint8_t *buf, int len);
	void (*usleep)(int usec);
	uint32_t (*current_time)(void);
};

struct yas_acc_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(void);
	int (*set_delay)(int delay);
	int (*get_enable)(void);
	int (*set_enable)(int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(struct yas_data *raw, int num);
	int (*ext)(int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

struct yas_mag_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(void);
	int (*set_delay)(int delay);
	int (*get_enable)(void);
	int (*set_enable)(int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(struct yas_data *raw, int num);
	int (*ext)(int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

struct yas_gyro_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(void);
	int (*set_delay)(int delay);
	int (*get_enable)(void);
	int (*set_enable)(int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(struct yas_data *raw, int num);
	int (*ext)(int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

struct yas_acc_mag_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(int32_t type);
	int (*set_delay)(int32_t type, int delay);
	int (*get_enable)(int32_t type);
	int (*set_enable)(int32_t type, int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(int32_t type, struct yas_data *raw, int num);
	int (*ext)(int32_t type, int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

struct yas_acc_gyro_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(int32_t type);
	int (*set_delay)(int32_t type, int delay);
	int (*get_enable)(int32_t type);
	int (*set_enable)(int32_t type, int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(int32_t type, struct yas_data *raw, int num);
	int (*ext)(int32_t type, int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

struct yas_acc_mag_gyro_driver {
	int (*init)(void);
	int (*term)(void);
	int (*get_delay)(int32_t type);
	int (*set_delay)(int32_t type, int delay);
	int (*get_enable)(int32_t type);
	int (*set_enable)(int32_t type, int enable);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(int32_t type, struct yas_data *raw, int num);
	int (*ext)(int32_t type, int32_t cmd, void *result);
	struct yas_driver_callback callback; 
};

#if YAS_MAG_FILTER_ENABLE
struct yas_mag_filter_config {
	uint8_t len; 
	uint16_t noise[3]; 
	uint16_t threshold; 
};

struct yas_mag_filter {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_vector *input, struct yas_vector *output);
	int (*get_config)(struct yas_mag_filter_config *config);
	int (*set_config)(struct yas_mag_filter_config *config);
};
#endif

#if YAS_MAG_CALIB_ENABLE
struct yas_mag_calib_config {
	uint8_t mode; 
#if YAS_MAG_CALIB_MINI_ENABLE || YAS_MAG_CALIB_FLOAT_ENABLE
	uint16_t spread[3]; 
	uint16_t variation[3]; 
#else
	int32_t cwm_threshold[9]; 
	int32_t cwg_threshold[12]; 
#endif
	struct yas_matrix *static_matrix; 
};

struct yas_mag_calib_result {
#if !YAS_MAG_CALIB_MINI_ENABLE
	int success_mode; 
#endif
	struct yas_vector offset; 
	struct yas_vector uncalibrated; 
	uint16_t spread; 
	uint16_t variation; 
	uint16_t radius; 
	uint8_t axis; 
	uint8_t accuracy; 
	uint8_t level; 
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
	struct yas_matrix dynamic_matrix;
#endif
};

struct yas_mag_calib {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_data *raw, int num);
	int (*get_offset)(int type, struct yas_vector *offset,
			uint8_t *accuracy);
	int (*set_offset)(int type, struct yas_vector *offset,
			uint8_t accuracy);
	int (*get_config)(struct yas_mag_calib_config *config);
	int (*set_config)(struct yas_mag_calib_config *config);
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
	int (*get_dynamic_matrix)(struct yas_matrix *m);
	int (*set_dynamic_matrix)(struct yas_matrix *m);
#endif
	int (*get_result)(struct yas_mag_calib_result *r);
};
#endif

#if YAS_GYRO_CALIB_ENABLE
struct yas_gyro_calib_config {
	uint16_t mag_noise; 
	uint16_t gyro_noise; 
};

struct yas_gyro_calib_result {
	struct yas_vector offset; 
	struct yas_vector uncalibrated; 
	uint8_t accuracy; 
};

struct yas_gyro_calib {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_data *raw, int num);
	int (*get_offset)(int type, struct yas_vector *offset,
			uint8_t *accuracy);
	int (*set_offset)(int type, struct yas_vector *offset,
			uint8_t accuracy);
	int (*get_config)(struct yas_gyro_calib_config *config);
	int (*set_config)(struct yas_gyro_calib_config *config);
	int (*get_result)(struct yas_gyro_calib_result *r);
};
#endif

#if YAS_MAG_AVERAGE_FILTER_ENABLE
struct yas_mag_avg_config {
	int tap_min; 
	int tap_hard; 
	int filter_len; 
	uint32_t dfine; 
	uint32_t dthresh; 
};

struct yas_mag_avg_result {
	int32_t tap_new; 
	int32_t dm;	
};

struct yas_mag_avg {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_data *raw, int num);
	int (*get_tap)(int *curtap, int *newtap);
	int (*set_tap)(int tap);
	int (*get_config)(struct yas_mag_avg_config *config);
	int (*set_config)(struct yas_mag_avg_config *config);
	int (*get_result)(struct yas_mag_avg_result *r);
};
#endif

#if YAS_GAMEVEC_ENABLE
struct yas_gamevec_config {
	int32_t weight;
	int32_t hpf_sq_out_threshold;
	int16_t sustain;
};
#endif

#if YAS_FUSION_ENABLE
struct yas_fusion_config {
	uint8_t mag_fusion_enable; 
	uint8_t gyro_fusion_enable; 
#if YAS_GAMEVEC_ENABLE
	struct yas_gamevec_config gamevec_config;
#endif
};

struct yas_fusion_result {
#if YAS_ORIENTATION_ENABLE
	struct yas_vector orientation_mag; 
#endif
	struct yas_quaternion quaternion_mag; 
#if YAS_GAMEVEC_ENABLE
	struct yas_quaternion quaternion_gyro; 
#endif
#if YAS_FUSION_WITH_GYRO_ENABLE
#if YAS_ORIENTATION_ENABLE
	struct yas_vector orientation_fusion; 
#endif
	struct yas_quaternion quaternion_fusion; 
	struct yas_vector gravity; 
	struct yas_vector linear_acceleration; 
#endif
};

struct yas_fusion {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_data *raw, int num);
	int (*get_config)(struct yas_fusion_config *config);
	int (*set_config)(struct yas_fusion_config *config);
	int (*get_offset)(int32_t type, struct yas_vector *offset,
			uint8_t *accuracy);
	int (*set_offset)(int32_t type, struct yas_vector *offset,
			uint8_t accuracy);
	int (*get_result)(struct yas_fusion_result *r);
};
#endif

#if YAS_SOFTWARE_GYROSCOPE_ENABLE
struct yas_swgyro_config {
	int dummy; 
};

struct yas_swgyro_result {
	struct yas_vector swgyro; 
};

struct yas_swgyro {
	int (*init)(void);
	int (*term)(void);
	int (*reset)(void);
	int (*update)(struct yas_data *calibrated, int num);
	int (*get_config)(struct yas_swgyro_config *config);
	int (*set_config)(struct yas_swgyro_config *config);
	int (*get_result)(struct yas_swgyro_result *r);
};
#endif

#if YAS_LOG_ENABLE

/**
 * @struct yas_log
 * @brief User-written callback functions for log control
 */
struct yas_log {
	int (*log_open)(void);
	int (*log_close)(void);
	int (*log_write)(const char *buf, int len);
};
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532
struct yas532_self_test_result {
	int32_t id;
	int8_t xy1y2[3];
	int32_t dir;
	int32_t sx, sy;
	int32_t xyz[3];
};
#endif


#ifdef __cplusplus
extern "C" {
#endif

int yas_acc_driver_init(struct yas_acc_driver *f);

int yas_mag_driver_init(struct yas_mag_driver *f);

int yas_gyro_driver_init(struct yas_gyro_driver *f);

int yas_acc_gyro_driver_init(struct yas_acc_gyro_driver *f);

int yas_acc_mag_driver_init(struct yas_acc_mag_driver *f);

int yas_acc_mag_gyro_driver_init(struct yas_acc_mag_gyro_driver *f);

#if YAS_MAG_CALIB_ENABLE
int yas_mag_calib_init(struct yas_mag_calib *f);
#endif

#if YAS_GYRO_CALIB_ENABLE
int yas_gyro_calib_init(struct yas_gyro_calib *f);
#endif

#if YAS_MAG_FILTER_ENABLE
int yas_mag_filter_init(struct yas_mag_filter *f);
#endif

#if YAS_MAG_AVERAGE_FILTER_ENABLE
int yas_mag_avg_init(struct yas_mag_avg *f);
#endif

#if YAS_FUSION_ENABLE
int yas_fusion_init(struct yas_fusion *f);
#endif

#if YAS_SOFTWARE_GYROSCOPE_ENABLE
int yas_swgyro_init(struct yas_swgyro *f);
#endif

#ifdef __cplusplus
}
#endif

#endif 

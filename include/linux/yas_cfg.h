/**
 * Configuration header file of the core driver API @file yas_cfg.h
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
#ifndef __YAS_CFG_H__
#define __YAS_CFG_H__

#define YAS_MAG_DRIVER_NONE			(0) 
#define YAS_MAG_DRIVER_YAS529			(1) 
#define YAS_MAG_DRIVER_YAS530			(2) 
#define YAS_MAG_DRIVER_YAS532			(3) 
#define YAS_MAG_DRIVER_YAS533			(4) 
#define YAS_MAG_DRIVER_YAS535			(5) 
#define YAS_MAG_DRIVER_YAS536			(6) 
#define YAS_MAG_DRIVER_YAS53x			(0x7fff) 

#define YAS_ACC_DRIVER_NONE			(0) 
#define YAS_ACC_DRIVER_ADXL345			(1) 
#define YAS_ACC_DRIVER_ADXL346			(2) 
#define YAS_ACC_DRIVER_BMA150			(3) 
#define YAS_ACC_DRIVER_BMA222			(4) 
#define YAS_ACC_DRIVER_BMA222E			(5) 
#define YAS_ACC_DRIVER_BMA250			(6) 
#define YAS_ACC_DRIVER_BMA250E			(7) 
#define YAS_ACC_DRIVER_BMA254			(8) 
#define YAS_ACC_DRIVER_BMI055			(9) 
#define YAS_ACC_DRIVER_BMI058			(10) 
#define YAS_ACC_DRIVER_DMARD08			(11) 
#define YAS_ACC_DRIVER_KXSD9			(12) 
#define YAS_ACC_DRIVER_KXTE9			(13) 
#define YAS_ACC_DRIVER_KXTF9			(14) 
#define YAS_ACC_DRIVER_KXTI9			(15) 
#define YAS_ACC_DRIVER_KXTJ2			(16) 
#define YAS_ACC_DRIVER_KXUD9			(17) 
#define YAS_ACC_DRIVER_LIS331DL			(18) 
#define YAS_ACC_DRIVER_LIS331DLH		(19) 
#define YAS_ACC_DRIVER_LIS331DLM		(20) 
#define YAS_ACC_DRIVER_LIS3DH			(21) 
#define YAS_ACC_DRIVER_LSM330DLC		(22) 
#define YAS_ACC_DRIVER_MMA8452Q			(23) 
#define YAS_ACC_DRIVER_MMA8453Q			(24) 
#define YAS_ACC_DRIVER_U2DH			(25) 
#define YAS_ACC_DRIVER_YAS535			(26) 
#define YAS_ACC_DRIVER_YAS53x			(0x7fff) 

#define YAS_GYRO_DRIVER_NONE			(0) 
#define YAS_GYRO_DRIVER_BMI055			(1) 
#define YAS_GYRO_DRIVER_BMI058			(2) 
#define YAS_GYRO_DRIVER_EWTZMU			(3) 
#define YAS_GYRO_DRIVER_ITG3200			(4) 
#define YAS_GYRO_DRIVER_ITG3500			(5) 
#define YAS_GYRO_DRIVER_L3G3200D		(6) 
#define YAS_GYRO_DRIVER_L3G4200D		(7) 
#define YAS_GYRO_DRIVER_LSM330DLC		(8) 
#define YAS_GYRO_DRIVER_MPU3050			(9) 
#define YAS_GYRO_DRIVER_MPU6050			(10) 
#define YAS_GYRO_DRIVER_YAS53x			(0x7fff) 


#define YAS_ACC_DRIVER				(YAS_ACC_DRIVER_BMA250)
#define YAS_MAG_DRIVER				(YAS_MAG_DRIVER_YAS532)
#define YAS_GYRO_DRIVER				(YAS_GYRO_DRIVER_NONE)

#define YAS_MAG_CALIB_MINI_ENABLE		(0)
#define YAS_MAG_CALIB_FLOAT_ENABLE		(1)
#define YAS_MAG_CALIB_SPHERE_ENABLE		(1)
#define YAS_MAG_CALIB_ELLIPSOID_ENABLE		(1)
#define YAS_MAG_CALIB_WITH_GYRO_ENABLE		(1)
#if YAS_MAG_CALIB_MINI_ENABLE
#undef YAS_MAG_CALIB_FLOAT_ENABLE
#undef YAS_MAG_CALIB_SPHERE_ENABLE
#undef YAS_MAG_CALIB_ELLIPSOID_ENABLE
#undef YAS_MAG_CALIB_WITH_GYRO_ENABLE
#define YAS_MAG_CALIB_FLOAT_ENABLE		(0)
#define YAS_MAG_CALIB_SPHERE_ENABLE		(0)
#define YAS_MAG_CALIB_ELLIPSOID_ENABLE		(0)
#define YAS_MAG_CALIB_WITH_GYRO_ENABLE		(0)
#endif
#define YAS_MAG_CALIB_ENABLE	(YAS_MAG_CALIB_FLOAT_ENABLE | \
		YAS_MAG_CALIB_MINI_ENABLE | \
		YAS_MAG_CALIB_SPHERE_ENABLE | \
		YAS_MAG_CALIB_ELLIPSOID_ENABLE | \
		YAS_MAG_CALIB_WITH_GYRO_ENABLE)

#define YAS_GYRO_CALIB_ENABLE			(1)
#define YAS_MAG_FILTER_ENABLE			(1)
#define YAS_FUSION_ENABLE			(1)
#define YAS_FUSION_WITH_GYRO_ENABLE		(1)
#define YAS_GAMEVEC_ENABLE			(1)
#define YAS_MAG_AVERAGE_FILTER_ENABLE		(0)
#define YAS_SOFTWARE_GYROSCOPE_ENABLE		(0)
#define YAS_LOG_ENABLE				(0)
#define YAS_ORIENTATION_ENABLE			(1)

#define YAS_MAG_VCORE				(2700)

#define YAS532_DRIVER_NO_SLEEP			(0)

#define YAS_DEFAULT_SENSOR_DELAY		(50)


#define YAS_MAG_DEFAULT_FILTER_NOISE_X		(1200)
#define YAS_MAG_DEFAULT_FILTER_NOISE_Y		(1200)
#define YAS_MAG_DEFAULT_FILTER_NOISE_Z		(1200)
#define YAS_MAG_DEFAULT_FILTER_LEN		(20)
#define YAS_MAG_DEFAULT_FILTER_THRESH		(300)


#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_NONE
#undef YAS_MAG_CALIB_ENABLE
#define YAS_MAG_CALIB_ENABLE			(0)
#undef YAS_MAG_FILTER_ENABLE
#define YAS_MAG_FILTER_ENABLE			(0)
#endif
#if YAS_MAG_DRIVER != YAS_MAG_DRIVER_YAS536
#undef YAS_MAG_AVERAGE_FILTER_ENABLE
#define YAS_MAG_AVERAGE_FILTER_ENABLE		(0)
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_NONE \
		    || YAS_ACC_DRIVER == YAS_ACC_DRIVER_NONE
#undef YAS_SOFTWARE_GYROSCOPE_ENABLE
#define YAS_SOFTWARE_GYROSCOPE_ENABLE		(0)
#undef YAS_FUSION_ENABLE
#define YAS_FUSION_ENABLE			(0)
#endif

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_NONE \
		     || YAS_MAG_DRIVER == YAS_MAG_DRIVER_NONE
#undef YAS_GYRO_CALIB_ENABLE
#define YAS_GYRO_CALIB_ENABLE			(0)
#endif

#if YAS_GYRO_DRIVER == YAS_GYRO_DRIVER_NONE
#undef YAS_FUSION_WITH_GYRO_ENABLE
#define YAS_FUSION_WITH_GYRO_ENABLE		(0)
#endif

#if !YAS_FUSION_ENABLE
#undef YAS_FUSION_WITH_GYRO_ENABLE
#define YAS_FUSION_WITH_GYRO_ENABLE		(0)
#endif

#if YAS_LOG_ENABLE
#ifdef __KERNEL__
#undef YAS_LOG_ENABLE
#define YAS_LOG_ENABLE				(0)
#else
#include <stdio.h>
#include <string.h>
#endif
#endif

#define YAS_MAG_NAME		"yas_magnetometer"
#define YAS_ACC_NAME		"yas_accelerometer"
#define YAS_GYRO_NAME		"yas_gyroscope"

#endif

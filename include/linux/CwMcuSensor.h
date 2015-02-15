/* CWMCU.h - header file for CyWee digital 3-axis gyroscope
 *
 * Copyright (C) 2010 CyWee Group Ltd.
 * Author: Joe Wei <joewei@cywee.com>
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
#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>

#define CWMCU_I2C_NAME "CwMcuSensor"

enum ABS_status {
	ABS_ACC_X = 0x01,
	ABS_ACC_Y = 0x02,
	ABS_ACC_Z = 0x03,
	ABS_MAG_X = 0x04,
	ABS_MAG_Y = 0x05,
	ABS_MAG_Z = 0x06,
	ABS_GYRO_X = 0x07,
	ABS_GYRO_Y = 0x08,
	ABS_GYRO_Z = 0x09,
	
	ABS_MAG_ACCURACY = 0x0A,
	
	ABS_ORI_ACCURACY = 0x0B,
	ABS_LIGHT_Z = 0x0C,
	ABS_GEOMAGNETIC_ROTATION_VECTOR_X = 0x0D,
	ABS_GEOMAGNETIC_ROTATION_VECTOR_Y = 0x0E,
	ABS_GEOMAGNETIC_ROTATION_VECTOR_Z = 0x0F,
	ABS_PRESSURE_X = 0x10,
	ABS_PRESSURE_Y = 0x11,
	ABS_PRESSURE_Z = 0x12,
	ABS_ORI_X = 0x13,
	ABS_ORI_Y = 0x14,
	ABS_ORI_Z = 0x15,
	ABS_ROT_X = 0x16,
	ABS_ROT_Y = 0x17,
	ABS_ROT_Z = 0x18,
	ABS_LIN_X = 0x1A, 
	ABS_LIN_Y = 0x1B,
	ABS_LIN_Z = 0x1C,
	ABS_GRA_X = 0x1D,
	ABS_GRA_Y = 0x1E,
	ABS_GRA_Z = 0x1F,
	ABS_PEDOMETER_X = 0x20,
	ABS_PEDOMETER_Y = 0x21,
	ABS_PEDOMETER_Z = 0x22,
	ABS_STEP_DETECTOR = 0x23,
	ABS_STEP_COUNTER = 0x24,
	ABS_AIR_MOUSE_Z = 0x25,
	ABS_GESTURE_MOTION = 0x26,
	ABS_BUFFERED_TRANSPORT_X = 0x27,
	ABS_BUFFERED_TRANSPORT_Y = 0x29, 
	ABS_BUFFERED_TRANSPORT_Z = 0x2A,
	ABS_REALTIME_TRANSPORT_X = 0x2B,
	ABS_REALTIME_TRANSPORT_Y = 0x2C,
	ABS_REALTIME_TRANSPORT_Z = 0x2D,
	ABS_TRANSPORT_BUFFER_FULL = 0x2E,
	ABS_MAGNETIC_UNCALIBRATED_X = 0x30, 
	ABS_MAGNETIC_UNCALIBRATED_Y = 0x31,
	ABS_MAGNETIC_UNCALIBRATED_Z = 0x32,
	ABS_MAGNETIC_UNCALIBRATED_BIAS_X = 0x3F, 
	ABS_MAGNETIC_UNCALIBRATED_BIAS_Y = 0x34,
	ABS_MAGNETIC_UNCALIBRATED_BIAS_Z = 0x35,
	ABS_GYROSCOPE_UNCALIBRATED_X = 0x36,
	ABS_GYROSCOPE_UNCALIBRATED_Y = 0x37,
	ABS_GYROSCOPE_UNCALIBRATED_Z = 0x38,
	ABS_GYROSCOPE_UNCALIBRATED_BIAS_X = 0x39,
	ABS_GYROSCOPE_UNCALIBRATED_BIAS_Y = 0x3A,
	ABS_GYROSCOPE_UNCALIBRATED_BIAS_Z = 0x3B,
	ABS_GAME_ROTATION_VECTOR_X = 0x3C,
	ABS_GAME_ROTATION_VECTOR_Y = 0x3D,
	ABS_GAME_ROTATION_VECTOR_Z = 0x3E,
};


typedef enum {
	CW_ACCELERATION                = 0,
	CW_MAGNETIC                    = 1,
	CW_GYRO                        = 2,
	CW_LIGHT                       = 3,
	CW_PROXIMITY                   = 4,
	CW_PRESSURE                    = 5,
	CW_ORIENTATION                 = 6,
	CW_ROTATIONVECTOR              = 7,
	CW_LINEARACCELERATION          = 8,
	CW_GRAVITY                     = 9,
	CW_PEDOMETER                   = 10,
	CW_AIRMOUSE                    = 11,
	CW_MAGNETIC_UNCALIBRATED       = 16,
	CW_GYROSCOPE_UNCALIBRATED      = 17,
	CW_GAME_ROTATION_VECTOR        = 18,
	CW_GEOMAGNETIC_ROTATION_VECTOR = 19,
	CW_SIGNIFICANT_MOTION          = 20,
	CW_STEP_DETECTOR               = 21,
	CW_STEP_COUNTER                = 22,
	HTC_GESTURE_MOTION             = 24,
	SENSOR_BUFFERED_TRANSPORT      = 25,
	SENSOR_REALTIME_TRANSPORT      = 26,
	SENSOR_TRANSPORT_BUFFER_FULL   = 27,
	HTC_ANY_MOTION                 = 28,
	HTC_MATRIX_GESTURE             = 29,
	HTC_GESTURE_MOTION_HIDI        = 30,
	HTC_MATRIX_GESTURE_HIDI        = 31,
	CW_SENSORS_ID_END 
} CW_SENSORS_ID;
#define FIRMWARE_VERSION 0x10
#define HTC_ENABLE_SENSORHUB_DEBUG 1
#define CWSTM32_READ_Touch_Log                     0xF1
#define CWSTM32_READ_Dump_Backup_Call_Stack_Buffer 0xFB
#define CWSTM32_READ_Dump_Call_Stack_Buffer        0xFD
#define CWSTM32_READ_Dump_Debug_Buffer             0xFE
#define CWSTM32_READ_Debug_Status                  0xFF
#define HTC_SYSTEM_STATUS_REG                      0x1E

#if defined(CONFIG_SYNC_TOUCH_STATUS)
int touch_status(u8 status);
#define TOUCH_STATUS_REGISTER                      0xF2
#endif

#if HTC_ENABLE_SENSORHUB_DEBUG
#define CWSTM32_IOCTL_MAGIC 'k'
#define CWSTM32_WRITE_Switch_Debug _IOW(CWSTM32_IOCTL_MAGIC, 10, int)
#define CWSTM32_WRITE_Disable_Debug _IOW(CWSTM32_IOCTL_MAGIC, 11, int)
#define CWSTM32_READ_Dump_Debug _IOR(CWSTM32_IOCTL_MAGIC, 12, int)
#define CWSTM32_READ_Dump_Call_Stack _IOR(CWSTM32_IOCTL_MAGIC, 13, int)
#endif

#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_ACC                0x60
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_ACC              0x68
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC              0x68
#define CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC	        0x69
#define CW_I2C_REG_SENSORS_CALIBRATOR_RESULT_RL_ACC             0x6A

#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_MAG                0x70
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_MAG              0x78
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG              0x78
#define CW_I2C_REG_SENSORS_ACCURACY_MAG                         0x79


#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_GYRO               0x80
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_GYRO             0x88
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO             0x88

#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_LIGHT              0x90
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_LIGHT            0x98
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT            0x98

#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_PROXIMITY          0xA0
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY        0xA8
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY        0xA8
#define CW_I2C_REG_SENSORS_CALIBRATOR_DEBUG_PROXIMITY           0xA9

#define CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_PRESSURE           0xB0
#define CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PRESSURE         0xB8
#define CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE         0xB8

#define CWMCU_MAX_DELAY		2000

#define G_SENSORS_STATUS                                        0x60
#define ACCE_UPDATE_RATE                                        0x66
#define ECOMPASS_SENSORS_STATUS                                 0x70
#define MAGN_UPDATE_RATE                                        0x76
#define GYRO_SENSORS_STATUS                                     0x80
#define GYRO_UPDATE_RATE                                        0x86
#define LIGHT_SENSORS_STATUS                                    0x90
#define PROXIMITY_SENSORS_STATUS                                0xA0
#define LIGHT_SENSORS_CALIBRATION_DATA				0x98

#define ORIE_UPDATE_RATE                                        0xC0
#define ROTA_UPDATE_RATE                                        0xC1
#define LINE_UPDATE_RATE                                        0xC2
#define GRAV_UPDATE_RATE                                        0xC3
#define MAGN_UNCA_UPDATE_RATE                                   0xC4
#define GYRO_UNCA_UPDATE_RATE                                   0xC5
#define GAME_ROTA_UPDATE_RATE                                   0xC6
#define GEOM_ROTA_UPDATE_RATE                                   0xC7
#define SIGN_UPDATE_RATE                                        0xC8

#define GESTURE_MOTION_UPDATE_ATTRIBUTE                         0xC9
#define GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN                     (4)

#define UPDATE_RATE_NORMAL              1
#define UPDATE_RATE_UI                  2
#define UPDATE_RATE_GAME                3
#define UPDATE_RATE_FASTEST             4

#define GENSOR_POSITION			0x65
#define COMPASS_POSITION		0x75
#define GYRO_POSITION			0x85
#define standardbase			0
#define	Acceleration			CW_ACCELERATION
#define	Magnetic			CW_MAGNETIC
#define	Gyro				CW_GYRO
#define	Light				CW_LIGHT
#define	Proximity			CW_PROXIMITY
#define	Pressure			CW_PRESSURE
#define	Orientation			CW_ORIENTATION
#define	RotationVector			CW_ROTATIONVECTOR
#define	LinearAcceleration		CW_LINEARACCELERATION
#define	Gravity				CW_GRAVITY

#define	Gesture_start			CW_PEDOMETER
#define	Pedometer			CW_PEDOMETER
#define	Air_Mouse			CW_AIRMOUSE
#define	Gesture_Motion			HTC_GESTURE_MOTION
#define	Any_Motion			HTC_ANY_MOTION
#define	Matrix_Gesture			HTC_MATRIX_GESTURE
#define	Gesture_Motion_HIDI		HTC_GESTURE_MOTION_HIDI
#define	Matrix_Gesture_HIDI		HTC_MATRIX_GESTURE_HIDI

#define Buffered_Transport		SENSOR_BUFFERED_TRANSPORT
#define Realtime_Transport		SENSOR_REALTIME_TRANSPORT
#define Transport_Buffer_Full		SENSOR_TRANSPORT_BUFFER_FULL

#define Magnetic_Uncalibrated		CW_MAGNETIC_UNCALIBRATED
#define Gyroscope_Uncalibrated		CW_GYROSCOPE_UNCALIBRATED
#define Game_Rotation_Vector		CW_GAME_ROTATION_VECTOR
#define Geomagnetic_Rotation_Vector	CW_GEOMAGNETIC_ROTATION_VECTOR

#define Significant_Motion		CW_SIGNIFICANT_MOTION
#define Step_Detector			CW_STEP_DETECTOR
#define Step_Counter			CW_STEP_COUNTER

#define numSensors			CW_SENSORS_ID_END

#define CWSTM32_ENABLE_REG			0x01
#define CWSTM32_READ_SEQUENCE_DATA_REG  	0x0F

#define CWSTM32_WRITE_POSITION_Acceleration    	0x20
#define CWSTM32_WRITE_POSITION_Magnetic		0x21
#define CWSTM32_WRITE_POSITION_Gyro		0x22

#define CWSTM32_WRITE_CLEAN_COUNT_Pedometer	0x30

#define CWSTM32_INT_ST1                         0x08
#define CWSTM32_INT_ST2                         0x09
#define CWSTM32_INT_ST3                         0x0A
#define CWSTM32_INT_ST4                         0x0B
#define CWSTM32_ERR_ST                          0x1F

#define CW_MCU_INT_BIT_LIGHT                    (1 << 3)
#define CW_MCU_INT_BIT_PROXIMITY                (1 << 4)

#define CW_MCU_INT_BIT_HTC_GESTURE_MOTION       (1 << 0)
#define CW_MCU_INT_BIT_TRANSPORT_BUFFER_FULL    (1 << 3)
#define CW_MCU_INT_BIT_ANY_MOTION               (1 << 4)
#define CW_MCU_INT_BIT_MATRIX_GESTURE           (1 << 5)
#define CW_MCU_INT_BIT_HTC_GESTURE_MOTION_HIDI  (1 << 6)
#define CW_MCU_INT_BIT_HTC_MATRIX_GESTURE_HIDI  (1 << 7)

#define CW_MCU_INT_BIT_SIGNIFICANT_MOTION       (1 << 4)
#define CW_MCU_INT_BIT_STEP_DETECTOR            (1 << 5)
#define CW_MCU_INT_BIT_STEP_COUNTER             (1 << 6)

#define CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION      (1 << 6)
#define CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET     (1 << 7)

#define CW_MCU_BIT_PROXIMITY_POLLING		(1 << 5)
#define CW_MCU_BIT_LIGHT_POLLING		(1 << 5)

#define	CW_MCU_I2C_SENSORS_REG_START		(0x20)

#define CWSTM32_READ_Gesture_Flip                               (CW_MCU_I2C_SENSORS_REG_START + HTC_GESTURE_FLIP)
#define CWSTM32_READ_Acceleration    		(CW_MCU_I2C_SENSORS_REG_START + CW_ACCELERATION)
#define CWSTM32_READ_Magnetic						(CW_MCU_I2C_SENSORS_REG_START + CW_MAGNETIC)
#define CWSTM32_READ_Gyro    						(CW_MCU_I2C_SENSORS_REG_START + CW_GYRO)
#define CWSTM32_READ_Light    					(CW_MCU_I2C_SENSORS_REG_START + CW_LIGHT)
#define CWSTM32_READ_Proximity    			(CW_MCU_I2C_SENSORS_REG_START + CW_PROXIMITY)
#define CWSTM32_READ_Pressure	    			(CW_MCU_I2C_SENSORS_REG_START + CW_PRESSURE)
#define CWSTM32_READ_Orientation    		(CW_MCU_I2C_SENSORS_REG_START + CW_ORIENTATION)
#define CWSTM32_READ_RotationVector    	(CW_MCU_I2C_SENSORS_REG_START + CW_ROTATIONVECTOR)
#define CWSTM32_READ_LinearAcceleration (CW_MCU_I2C_SENSORS_REG_START + CW_LINEARACCELERATION)
#define CWSTM32_READ_Gravity    				(CW_MCU_I2C_SENSORS_REG_START + CW_GRAVITY)
#define CWSTM32_READ_Pedometer					(CW_MCU_I2C_SENSORS_REG_START + CW_PEDOMETER)
#define CWSTM32_READ_Air_Mouse					(CW_MCU_I2C_SENSORS_REG_START + CW_AIRMOUSE)
#define CWSTM32_READ_MAGNETIC_UNCALIBRATED			0x30
#define CWSTM32_READ_GYROSCOPE_UNCALIBRATED			0x31
#define CWSTM32_READ_GAME_ROTATION_VECTOR			0x32
#define CWSTM32_READ_GEOMAGNETIC_ROTATION_VECTOR		0x33
#define CWSTM32_READ_SIGNIFICANT_MOTION				0x34
#define CWSTM32_READ_STEP_DETECTOR				0x35
#define CWSTM32_READ_STEP_COUNTER				0x36
#define CWSTM32_READ_Gesture_Motion                             0x38
#define CWSTM32_READ_SENSOR_BUFFERED_TRANSPORT			0x3C 
#define CWSTM32_READ_SENSOR_REALTIME_TRANSPORT			0x3D 
#define CWSTM32_READ_SENSOR_TRANSPORT_BUFFER_FULL		0x3E 
#define CWSTM32_READ_Any_Motion                 	        0x3F
#define CWSTM32_READ_Matrix_Gesture                 	        0x39
#define CWSTM32_READ_Gesture_Motion_HIDI               	        0x3A
#define CWSTM32_READ_Matrix_Gesture_HIDI               	        0x3B

#ifdef __KERNEL__
struct CWMCU_platform_data {
	unsigned char Acceleration_axes;
	unsigned char Magnetic_axes;
	unsigned char Gyro_axes;
	uint32_t gpio_wake_mcu;
	uint32_t gpio_reset;
	uint32_t gpio_chip_mode;
	uint32_t gpio_mcu_irq;
	int GS_chip_layout;
	u8 ALS_goldh;
	u8 ALS_goldl;
	u8 ls_polling;

};
#endif 

#endif 

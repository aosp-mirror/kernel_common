/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
*
* File Name		: r3gd20.h
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.5 sysfs
* Date			: 2011/Sep/24
* Description		: R3GD20 digital output gyroscope sensor API
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
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone	| First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno		| Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno		| SMB Bus Mng,
* 		|		|			| forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno		| Introduces FIFO Feat.
* 1.1.5.1 | 2011/Nov/6  | Morris Chen     | change name from l3g to r3g
*                                         | change default FS to 2000DPS
*                                         | change default poll_rate to 50ms
*                                         | chage the attribute of sysfs file as 666
*******************************************************************************/

#ifndef __R3GD20_H__
#define __R3GD20_H__

/*#define DEFAULT_INT2_GPIO	39*/
/*#define DEFAULT_INT1_GPIO	134*/

#define R3GD20_MIN_POLL_PERIOD_MS	2

#define SAD0L				0x00
#define SAD0H				0x01
#define R3GD20_GYR_I2C_SADROOT		0x6A
#define R3GD20_GYR_I2C_SAD_L		((R3GD20_GYR_I2C_SADROOT<<1)|SAD0L)
#define R3GD20_GYR_I2C_SAD_H		((R3GD20_GYR_I2C_SADROOT<<1)|SAD0H)

#define R3GD20_GYR_DEV_NAME		"r3gd20_gyr"

#define R3GD20_GYR_FS_250DPS	0x00
#define R3GD20_GYR_FS_500DPS	0x10
#define R3GD20_GYR_FS_2000DPS	0x30

#define R3GD20_GYR_ENABLED	1
#define R3GD20_GYR_DISABLED	0

extern unsigned char gyro_gsensor_kvalue[37];

#ifdef __KERNEL__
struct r3gd20_gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	/* axis mapping */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*power_LPM)(int on);

	unsigned char gyro_kvalue[37];
};
#endif /* __KERNEL__ */

#endif  /* __R3GD20_H__ */

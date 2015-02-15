/* include/linux/cm32181.h
 *
 * Copyright (C) 2013 Capella Microsystems, Inc.
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

#ifndef __LINUX_CM32181_H
#define __LINUX_CM32181_H

#define CM32181_I2C_NAME	"CM32181"

#define ALS_CALIBRATED		0x6DA5

/*cm32181*/
/*Define ALS Command Code*/
#define	ALS_CMD		0x00
#define	ALS_HW		0x01
#define	ALS_LW		0x02
#define	ALS_PSM		0x03
#define	ALS_DATA	0x04
#define	ALS_STATUS	0x06

/*for ALS command*/
#define CM32181_ALS_SM_1		(0 << 11)
#define CM32181_ALS_SM_2		(1 << 11)
#define CM32181_ALS_SM_1_8		(2 << 11)
#define CM32181_ALS_SM_1_4		(3 << 11)
#define CM32181_ALS_IT_100MS	(0 << 6)
#define CM32181_ALS_IT_200MS	(1 << 6)
#define CM32181_ALS_IT_400MS	(2 << 6)
#define CM32181_ALS_IT_800MS	(3 << 6)
#define CM32181_ALS_PERS_1		(0 << 4)
#define CM32181_ALS_PERS_2		(1 << 4)
#define CM32181_ALS_PERS_4		(2 << 4)
#define CM32181_ALS_PERS_8		(3 << 4)
#define CM32181_ALS_INT_EN		(1 << 1)
#define CM32181_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/

#define LS_PWR_ON				(1 << 0)
#define PS_PWR_ON				(1 << 1)

enum {
	CAPELLA_CM32181,
};

struct cm32181_platform_data {
        int model;
	int intr;
	uint32_t irq_gpio_flags;
	uint32_t *levels;
	uint32_t golden_adc;
	uint32_t emmc_als_kadc;
	int (*power)(int, uint8_t); /* power to the chip */
        uint32_t cm32181_slave_address;
	uint16_t ls_cmd;

};

#endif

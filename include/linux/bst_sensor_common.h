/*
 * Created on Sep 19th, 2012, by guz4sgh
 * Last modified: Sep 19th, 2012
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2012 Bosch Sensortec GmbH
 * All Rights Reserved
 */


#ifndef __BOSCH_SENSOR_COMMON_H
#define __BOSCH_SENSOR_COMMON_H

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

/*!
 * @Description:
 * definitions of placement of sensors
 * P0 - P3: view from top of device
 * P4 - P7: view from bottom of device
 *
 * P0 / P4:
 * Y of device aligned with Y of OS (i.e: Android)
 *                  y
 *                  ^
 *                  |
 *                  |
 *                  |
 *                  o------> x
 *
 *
 * P1 / P5:
 * Y of device aligned with Y of OS (i.e.: Android)
 * rotated by 90 degrees clockwise
 *
 *                  o------> y
 *                  |
 *                  |
 *                  |
 *                  v x
 *
 *
 * P2 / P6:
 * Y of device aligned with Y of OS (i.e.: Android)
 * rotated by 180 degrees clockwise
 *
 *         x <------o
 *                  |
 *                  |
 *                  |
 *                  v
 *                  y
 *
 *
 * P3 / P7:
 * Y of device aligned with Y of OS (i.e.: Android)
 * rotated by 270 degrees clockwise
 *
 *                  x
 *                  ^
 *                  |
 *                  |
 *                  |
 *         y <------o
 *
 */


struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	unsigned int place:3;
};


/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
typedef struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
} bosch_sensor_axis_remap;


struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

extern unsigned int gs_kvalue;
extern unsigned char gyro_gsensor_kvalue[37];

void bst_remap_sensor_data(struct bosch_sensor_data *data,
		const bosch_sensor_axis_remap *remap);

void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
		int place);
#endif

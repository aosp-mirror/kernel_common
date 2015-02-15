/*
 * Copyright (c) 2012 Yamaha Corporation
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

#ifndef __YAS53X_DRV_H__
#define __YAS53X_DRV_H__

#include "yas53x.h"

/* ----------------------------------------------------------------------------
 *                              Macro definition
 *--------------------------------------------------------------------------- */

#define YAS_VCORE (26)

#define YAS_X_OVERFLOW				(0x01)
#define YAS_X_UNDERFLOW				(0x02)
#define YAS_Y1_OVERFLOW				(0x04)
#define YAS_Y1_UNDERFLOW			(0x08)
#define YAS_Y2_OVERFLOW				(0x10)
#define YAS_Y2_UNDERFLOW			(0x20)
#define YAS_REPORT_HARD_OFFSET_CHANGED		(0x40)
#define YAS_OVERFLOW	(YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW	(YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

/* ----------------------------------------------------------------------------
 *                            Structure definition
 *--------------------------------------------------------------------------- */

struct yas_mag_data {
	struct yas_vector xyz;
	int16_t xy1y2[3];
	int32_t xy1y2_linear[3];
	int16_t temperature;
};

struct yas_mag_driver_callback {
	int (*device_open)(void);
	int (*device_close)(void);
	int (*device_write)(uint8_t addr, const uint8_t *buf, int len);
	int (*device_read)(uint8_t addr, uint8_t *buf, int len);
	void (*msleep)(int msec);
	void (*current_time)(uint32_t *msec);
};

struct yas_self_test_result {
	int32_t id;
	int8_t xy1y2[3];
	int32_t dir;
	int32_t sx, sy;
	int32_t xyz[3];
};

struct yas_mag_driver {
	int (*init)(void);
	int (*term)(void);
	int (*self_test)(struct yas_self_test_result *r);
	int (*self_test_noise)(struct yas_vector *raw_xyz);
	int (*get_offset)(int8_t *offset);
	int (*set_offset)(const int8_t *offset);
	int (*get_position)(void);
	int (*set_position)(int position);
	int (*measure)(struct yas_mag_data *data);
	struct yas_mag_driver_callback callback;
};

/* ----------------------------------------------------------------------------
 *                         Global function definition
 *--------------------------------------------------------------------------- */

int yas_mag_driver_init(struct yas_mag_driver *f);

#endif /* __YAS53X_DRV_H__ */

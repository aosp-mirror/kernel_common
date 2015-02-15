/*
 * Copyright (c) 2012-2013 Yamaha Corporation
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

#ifndef __YAS53X_H__
#define __YAS53X_H__

/* ----------------------------------------------------------------------------
 *                              Macro definition
 *--------------------------------------------------------------------------- */

#define YAS_VERSION	"1.0.3"

#if defined(__KERNEL__)
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#ifndef NULL
#define NULL ((void *)(0))
#endif

#define YAS_NO_ERROR			(0)
#define YAS_ERROR_DEVICE_COMMUNICATION	(-1)
#define YAS_ERROR_I2C			YAS_ERROR_DEVICE_COMMUNICATION
#define YAS_ERROR_POWER			(-2)
#define YAS_ERROR_TEST_ORDER		(-3)
#define YAS_ERROR_NOT_INITIALIZED	YAS_ERROR_TEST_ORDER
#define YAS_ERROR_INTERRUPT		(-4)
#define YAS_ERROR_BUSY			(-5)
#define YAS_ERROR_OVERFLOW		(-6)
#define YAS_ERROR_UNDERFLOW		(-7)
#define YAS_ERROR_DIRCALC		(-8)
#define YAS_ERROR_NOT_SUPPORTED		(-9)
#define YAS_ERROR_CALREG		(-10)
#define YAS_ERROR_CHIP_ID		(-11)
#define YAS_ERROR_ARG			(-128)
#define YAS_X_OVERFLOW			(0x01)
#define YAS_X_UNDERFLOW			(0x02)
#define YAS_Y1_OVERFLOW			(0x04)
#define YAS_Y1_UNDERFLOW		(0x08)
#define YAS_Y2_OVERFLOW			(0x10)
#define YAS_Y2_UNDERFLOW		(0x20)
#define YAS_REPORT_HARD_OFFSET_CHANGED	(0x40)
#define YAS_OVERFLOW	(YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW	(YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

#define NELEMS(a) ((int)(sizeof(a)/sizeof(a[0])))

/* ----------------------------------------------------------------------------
 *                              Configuration
 *--------------------------------------------------------------------------- */

#define YAS_DEFAULT_FILTER_LEN (20)
#define YAS_DEFAULT_FILTER_THRESH (300) /* 300 nT */
#define YAS_DEFAULT_FILTER_NOISE (12*12) /* standard deviation 1200 nT */

#define YAS_VCORE (26)

/* ----------------------------------------------------------------------------
 *                            Structure definition
 *--------------------------------------------------------------------------- */

struct yas_vector {
	int32_t v[3];
};

struct yas_matrix {
	int32_t m[9];
};

struct yas_quaternion {
	int32_t q[4];
};

struct yas_acc_data {
	struct yas_vector xyz;
	int16_t raw[3];
};

struct yas_acc_driver_callback {
	int (*device_open)(void);
	int (*device_close)(void);
	int (*device_write)(uint8_t addr, const uint8_t *buf, int len);
	int (*device_read)(uint8_t addr, uint8_t *buf, int len);
	void (*msleep)(int msec);
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
	int (*measure)(struct yas_acc_data *data);
	struct yas_acc_driver_callback callback;
};

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

struct yas_mag_filter {
	int (*init)(void);
	int (*filter)(struct yas_vector *input, struct yas_vector *filtered);
};

#undef EVAL
#ifdef EVAL
struct yas_mag_calibration_result {
	int32_t spread;
	int32_t variation;
	int32_t radius;
	int8_t axis;
	int8_t level;
	int8_t accuracy;
	struct yas_matrix dynamic_matrix;
};
struct yas_mag_calibration_threshold {
	int32_t spread;
	int32_t variation[3];
};
#endif

struct yas_mag_calibration {
	int (*init)(void);
#ifdef EVAL
	int (*update)(struct yas_vector *mag,
			struct yas_mag_calibration_result *r);
#else
	int (*update)(struct yas_vector *mag, struct yas_vector *calibrated,
			int *accuracy);
#endif
	int (*get_offset)(struct yas_vector *offset);
	int (*set_offset)(struct yas_vector *offset);
	int (*get_static_matrix)(struct yas_matrix *matrix);
	int (*set_static_matrix)(struct yas_matrix *matrix);
	int (*get_accuracy)(void);
	int (*set_accuracy)(int accuracy);
};

struct yas_util {
	int (*get_euler)(struct yas_vector *acc, struct yas_vector *mag,
			struct yas_vector *euler);
};

struct yas53x_platform_data {
    int chip_layout;
};

/* ----------------------------------------------------------------------------
 *                         Global function definition
 *--------------------------------------------------------------------------- */

int yas_acc_driver_init(struct yas_acc_driver *f);
int yas_mag_driver_init(struct yas_mag_driver *f);
int yas_mag_filter_init(struct yas_mag_filter *f);
int yas_mag_calibration_init(struct yas_mag_calibration *f);
int yas_util_init(struct yas_util *f);

#endif /* __YAS53X_H__ */

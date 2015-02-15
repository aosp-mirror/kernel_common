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

#include <linux/yas53x.h>

#define YAS_REG_DEVICE_ID		(0x80)
#define YAS_REG_ACTUATE_INIT_COIL	(0x81)
#define YAS_REG_MEASURE_COMMAND		(0x82)
#define YAS_REG_CONFIG			(0x83)
#define YAS_REG_MEASURE_INTERVAL	(0x84)
#define YAS_REG_OFFSET_X		(0x85)
#define YAS_REG_OFFSET_Y1		(0x86)
#define YAS_REG_OFFSET_Y2		(0x87)
#define YAS_REG_TEST1			(0x88)
#define YAS_REG_TEST2			(0x89)
#define YAS_REG_CAL			(0x90)
#define YAS_REG_MEASURE_DATA		(0xb0)
#define YAS_YAS530_DEVICE_ID		(0x01)	/* YAS530  (MS-3E) */
#define YAS_YAS530_VERSION_A		(0)	/* YAS530  (MS-3E Aver) */
#define YAS_YAS530_VERSION_B		(1)	/* YAS530B (MS-3E Bver) */
#define YAS_YAS530_VERSION_A_COEF	(380)
#define YAS_YAS530_VERSION_B_COEF	(550)
#define YAS_YAS530_DATA_CENTER		(2048)
#define YAS_YAS530_DATA_UNDERFLOW	(0)
#define YAS_YAS530_DATA_OVERFLOW	(4095)

#define YAS_YAS532_DEVICE_ID		(0x02)	/* YAS532_533   (MS-3R/3F) */
#define YAS_YAS532_VERSION_AB		(0) /* YAS532_533AB (MS-3R/3F ABver) */
#define YAS_YAS532_VERSION_AC		(1) /* YAS532_533AC (MS-3R/3F ACver) */
#define YAS_YAS532_VERSION_AB_COEF	(1800)
#define YAS_YAS532_VERSION_AC_COEF_X	(850)
#define YAS_YAS532_VERSION_AC_COEF_Y1	(750)
#define YAS_YAS532_VERSION_AC_COEF_Y2	(750)
#define YAS_YAS532_DATA_CENTER		(4096)
#define YAS_YAS532_DATA_UNDERFLOW	(0)
#define YAS_YAS532_DATA_OVERFLOW	(8190)

#define YAS_MAG_STATE_NORMAL		(0)
#define YAS_MAG_STATE_INIT_COIL		(1)
#define YAS_MAG_STATE_MEASURE_OFFSET	(2)
#define YAS_MAG_NOTRANS_POSITION	(3)
#define YAS_MAG_INITCOIL_TIMEOUT	(500)	/* msec */
#define YAS_MAG_TEMPERATURE_LOG		(10)

#define set_vector(to, from) \
	{int _l; for (_l = 0; _l < 3; _l++) to[_l] = from[_l]; }
#define set_matrix(to, from) \
	{int _l; for (_l = 0; _l < 9; _l++) to[_l] = from[_l]; }
#define is_vector_differ(a, b) \
	((a)[0] != (b)[0] || (a)[1] != (b)[1] || (a)[2] != (b)[2])
#define is_valid_offset(a) \
	((a) != NULL && ((a)[0] <= 31) && ((a)[1] <= 31) && ((a)[2] <= 31) \
		&& (-31 <= (a)[0]) && (-31 <= (a)[1]) && (-31 <= (a)[2]))

struct yas_correction_data {
	int8_t rxy1y2[3];
	uint8_t fxy1y2[3];
	uint8_t ver;
	int32_t Cx, Cy1, Cy2;
	int32_t a2, a3, a4, a5, a6, a7, a8, a9, k;
};
#if 1 < YAS_MAG_TEMPERATURE_LOG
struct yas_temperature_filter {
	int16_t log[YAS_MAG_TEMPERATURE_LOG];
	int num;
	int idx;
};
#endif
struct yas_cdriver {
	int initialized;
	struct yas_correction_data correct;
	struct yas_mag_driver_callback cbk;
	int measure_state;
	int8_t hard_offset[3];
	int overflow;
	uint32_t overflow_time;
	int16_t center_thresh;
	int16_t underflow_thresh;
	int16_t overflow_thresh;
	int32_t coef[3];
	int position;
	int8_t *transform;
	uint8_t dev_id;
#if 1 < YAS_MAG_TEMPERATURE_LOG
	struct yas_temperature_filter t;
#endif
};

static const int yas532_version_ac_coef[] = {YAS_YAS532_VERSION_AC_COEF_X, YAS_YAS532_VERSION_AC_COEF_Y1, YAS_YAS532_VERSION_AC_COEF_Y2};
static const int8_t INVALID_OFFSET[] = {0x7f, 0x7f, 0x7f};
static const int8_t YAS_TRANSFORMATION[][9] = {
	{ 0,  1,  0, -1,  0,  0,  0,  0,  1 },
	{-1,  0,  0,  0, -1,  0,  0,  0,  1 },
	{ 0, -1,  0,  1,  0,  0,  0,  0,  1 },
	{ 1,  0,  0,  0,  1,  0,  0,  0,  1 },
	{ 0, -1,  0, -1,  0,  0,  0,  0, -1 },
	{ 1,  0,  0,  0, -1,  0,  0,  0, -1 },
	{ 0,  1,  0,  1,  0,  0,  0,  0, -1 },
	{-1,  0,  0,  0,  1,  0,  0,  0, -1 },
};
static struct yas_cdriver driver;

static int get_cal_data_yas530(struct yas_correction_data *c) {
	uint8_t data[16]; int i;
	if (driver.cbk.device_read(YAS_REG_CAL, data, 16) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (driver.cbk.device_read(YAS_REG_CAL, data, 16) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	c->fxy1y2[0] = (uint8_t)((data[12]>>5) & 0x03);
	c->rxy1y2[0] = ((int8_t)((((data[11]<<1) & 0x3e)
					| ((data[12]>>7) & 0x01))<<2))>>2;
	c->fxy1y2[1] = (uint8_t)((data[13]>>5) & 0x03);
	c->rxy1y2[1] = ((int8_t)((((data[12]<<1) & 0x3e)
					| ((data[13]>>7) & 0x01))<<2))>>2;
	c->fxy1y2[2] = (uint8_t)((data[14]>>5) & 0x03);
	c->rxy1y2[2] = ((int8_t)((((data[13]<<1) & 0x3e)
					| ((data[14]>>7) & 0x01))<<2))>>2;
	c->ver = (uint8_t)((data[15]) & 0x03);
	c->Cx = data[0] * 6 - 768;
	c->Cy1 = data[1] * 6 - 768;
	c->Cy2 = data[2] * 6 - 768;
	c->a2 = ((data[3]>>2) & 0x03f) - 32;
	c->a3 = (uint8_t)(((data[3]<<2) & 0x0c) | ((data[4]>>6) & 0x03)) - 8;
	c->a4 = (uint8_t)(data[4] & 0x3f) - 32;
	c->a5 = ((data[5]>>2) & 0x3f) + 38;
	c->a6 = (uint8_t)(((data[5]<<4) & 0x30) | ((data[6]>>4) & 0x0f)) - 32;
	c->a7 = (uint8_t)(((data[6]<<3) & 0x78) | ((data[7]>>5) & 0x07)) - 64;
	c->a8 = (uint8_t)(((data[7]<<1) & 0x3e) | ((data[8]>>7) & 0x01)) - 32;
	c->a9 = (uint8_t)(((data[8]<<1) & 0xfe) | ((data[9]>>7) & 0x01));
	c->k = (uint8_t)((data[9]>>2) & 0x1f) + 10;
	for (i = 0; i < 16; i++)
		if (data[i] != 0)
			return YAS_NO_ERROR;
	return YAS_ERROR_CALREG;
}

static int get_cal_data_yas532(struct yas_correction_data *c) {
	uint8_t data[14]; int i;
	if (driver.cbk.device_read(YAS_REG_CAL, data, 14) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (driver.cbk.device_read(YAS_REG_CAL, data, 14) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	c->fxy1y2[0] = (uint8_t)(((data[10]&0x01)<<1) | ((data[11]>>7)&0x01));
	c->rxy1y2[0] = ((int8_t)(((data[10]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[1] = (uint8_t)(((data[11]&0x01)<<1) | ((data[12]>>7)&0x01));
	c->rxy1y2[1] = ((int8_t)(((data[11]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[2] = (uint8_t)(((data[12]&0x01)<<1) | ((data[13]>>7)&0x01));
	c->rxy1y2[2] = ((int8_t)(((data[12]>>1) & 0x3f)<<2))>>2;
	c->Cx = data[0] * 10 - 1280;
	c->Cy1 = data[1] * 10 - 1280;
	c->Cy2 = data[2] * 10 - 1280;
	c->a2 = ((data[3]>>2)&0x03f) - 32;
	c->a3 = (uint8_t)(((data[3]<<2) & 0x0c) | ((data[4]>>6) & 0x03)) - 8;
	c->a4 = (uint8_t)(data[4] & 0x3f) - 32;
	c->a5 = ((data[5]>>2) & 0x3f) + 38;
	c->a6 = (uint8_t)(((data[5]<<4) & 0x30) | ((data[6]>>4) & 0x0f)) - 32;
	c->a7 = (uint8_t)(((data[6]<<3) & 0x78) | ((data[7]>>5) & 0x07)) - 64;
	c->a8 = (uint8_t)(((data[7]<<1) & 0x3e) | ((data[8]>>7) & 0x01)) - 32;
	c->a9 = (uint8_t)(((data[8]<<1) & 0xfe) | ((data[9]>>7) & 0x01));
	c->k = (uint8_t)((data[9]>>2) & 0x1f);
	for (i = 0; i < 13; i++)
		if (data[i] != 0)
			return YAS_NO_ERROR;
	if (data[13] & 0x80)
		return YAS_NO_ERROR;
	return YAS_ERROR_CALREG;
}

static int set_measure_command(int ldtc, int fors, int dlymes) {
	uint8_t data = 0x01;
	data = (uint8_t)(data | (((!!ldtc)<<1) & 0x02));
	data = (uint8_t)(data | (((!!fors)<<2) & 0x04));
	data = (uint8_t)(data | (((!!dlymes)<<4) & 0x10));
	if (driver.cbk.device_write(YAS_REG_MEASURE_COMMAND, &data, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static int measure_normal_yas530(int ldtc, int fors, int *busy, int16_t *t, int16_t *xy1y2) {
	uint8_t data[8];
	if (set_measure_command(ldtc, fors, 0) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	driver.cbk.msleep(2);
	if (driver.cbk.device_read(YAS_REG_MEASURE_DATA, data, 8) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	*busy = (data[0]>>7) & 0x01;
	*t = (int16_t)((((int32_t)data[0]<<2) & 0x1fc)|((data[1]>>6) & 0x03));
	xy1y2[0] = (int16_t)((((int32_t)data[2]<<5) & 0xfe0)
			| ((data[3]>>3) & 0x1f));
	xy1y2[1] = (int16_t)((((int32_t)data[4]<<5) & 0xfe0)
			| ((data[5]>>3) & 0x1f));
	xy1y2[2] = (int16_t)((((int32_t)data[6]<<5) & 0xfe0)
			| ((data[7]>>3) & 0x1f));
	return YAS_NO_ERROR;
}

static int measure_normal_yas532(int ldtc, int fors, int *busy, int16_t *t, int16_t *xy1y2) {
	uint8_t data[8];
	if (set_measure_command(ldtc, fors, 0) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	driver.cbk.msleep(2);
	if (driver.cbk.device_read(YAS_REG_MEASURE_DATA, data, 8) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	*busy = (data[0]>>7) & 0x01;
	*t = (int16_t)((((int32_t)data[0]<<3) & 0x3f8)|((data[1]>>5) & 0x07));
	xy1y2[0] = (int16_t)((((int32_t)data[2]<<6) & 0x1fc0)
			| ((data[3]>>2) & 0x3f));
	xy1y2[1] = (int16_t)((((int32_t)data[4]<<6) & 0x1fc0)
			| ((data[5]>>2) & 0x3f));
	xy1y2[2] = (int16_t)((((int32_t)data[6]<<6) & 0x1fc0)
			| ((data[7]>>2) & 0x3f));
	return YAS_NO_ERROR;
}

static int measure_normal(int ldtc, int fors, int *busy, int16_t *t, int16_t *xy1y2, int *ouflow) {
	int result, i;
	switch (driver.dev_id) {
	case YAS_YAS532_DEVICE_ID:
		result = measure_normal_yas532(ldtc, fors, busy, t, xy1y2);
		break;
	case YAS_YAS530_DEVICE_ID:
	default:
		result = measure_normal_yas530(ldtc, fors, busy, t, xy1y2);
		break;
	}
	*ouflow = 0;
	for (i = 0; i < 3; i++) {
		if (xy1y2[i] == driver.overflow_thresh)
			*ouflow |= (1<<(i*2));
		if (xy1y2[i] == driver.underflow_thresh)
			*ouflow |= (1<<(i*2+1));
	}
	return result;
}

static int yas_cdrv_set_offset(const int8_t *offset) {
	if (driver.cbk.device_write(YAS_REG_OFFSET_X,
				(const uint8_t *)&offset[0], 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (driver.cbk.device_write(YAS_REG_OFFSET_Y1,
				(const uint8_t *)&offset[1], 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (driver.cbk.device_write(YAS_REG_OFFSET_Y2,
				(const uint8_t *)&offset[2], 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	set_vector(driver.hard_offset, offset);
	return YAS_NO_ERROR;
}

static int yas_cdrv_actuate_initcoil(void) {
	uint8_t data = 0;
	return driver.cbk.device_write(YAS_REG_ACTUATE_INIT_COIL, &data, 1);
}

static int yas_cdrv_measure_and_set_offset(void) {
	static const int correct[5] = {16, 8, 4, 2, 1};
	int8_t hard_offset[3] = {0, 0, 0};
	int16_t t, xy1y2[3];
	int32_t flag[3] = {0};
	int i, j, busy, ouflow;
	for (i = 0; i < 5; i++) {
		if (yas_cdrv_set_offset(hard_offset) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (measure_normal(0, 0, &busy, &t, xy1y2, &ouflow) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (busy)
			return YAS_ERROR_BUSY;
		for (j = 0; j < 3; j++) {
			if (driver.center_thresh == xy1y2[j])
				flag[j] = 0;
			if (driver.center_thresh < xy1y2[j])
				flag[j] = 1;
			if (xy1y2[j] < driver.center_thresh)
				flag[j] = -1;
		}
		for (j = 0; j < 3; j++)
			if (flag[j])
				hard_offset[j] = (int8_t)(hard_offset[j]
						+ flag[j] * correct[i]);
	}
	return yas_cdrv_set_offset(hard_offset);
}

static int yas_cdrv_sensitivity_measuremnet(int32_t *sx, int32_t *sy) {
	struct yas_correction_data *c = &driver.correct;
	int16_t xy1y2_on[3], xy1y2_off[3], t;
	int busy, flowon = 0, flowoff = 0;
	if (measure_normal(1, 0, &busy, &t, xy1y2_on, &flowon) , 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (busy)
		return YAS_ERROR_BUSY;
	if (measure_normal(1, 1, &busy, &t, xy1y2_off, &flowoff) , 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (busy)
		return YAS_ERROR_BUSY;
	switch (driver.dev_id) {
	case YAS_YAS532_DEVICE_ID:
		*sx = c->k * (xy1y2_on[0] - xy1y2_off[0]) / 10 / YAS_VCORE;
		*sy = c->k * c->a5 * ((xy1y2_on[1] - xy1y2_off[1])
				- (xy1y2_on[2] - xy1y2_off[2])) / 1000
			/ YAS_VCORE;
		break;
	case YAS_YAS530_DEVICE_ID:
	default:
		*sx = xy1y2_off[0] - xy1y2_on[0];
		*sy = (xy1y2_off[1] - xy1y2_on[1])
			- (xy1y2_off[2] - xy1y2_on[2]);
		break;
	}
	return flowon | flowoff;
}

static int yas_cdrv_measure(int32_t *xyz, int16_t *xy1y2, int32_t *xy1y2_linear, int16_t *temperature, int temp_correction)
{
	static const int16_t cval[] = {3721, 3971, 4221, 4471};
	struct yas_correction_data *c = &driver.correct;
	int16_t t;
	int32_t xyz_tmp[3], tmp;
	int32_t sx, sy1, sy2, sy, sz;
	int i, ouflow = 0, busy;
#if 1 < YAS_MAG_TEMPERATURE_LOG
	int32_t sum = 0;
#endif
	if (measure_normal(0, 0, &busy, &t, xy1y2, &ouflow) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < 3; i++)
		xy1y2_linear[i] = xy1y2[i] - cval[driver.correct.fxy1y2[i]]
			+ (driver.hard_offset[i] - driver.correct.rxy1y2[i])
			* driver.coef[i];
#if 1 < YAS_MAG_TEMPERATURE_LOG
	driver.t.log[driver.t.idx++] = t;
	if (YAS_MAG_TEMPERATURE_LOG <= driver.t.idx)
		driver.t.idx = 0;
	driver.t.num++;
	if (YAS_MAG_TEMPERATURE_LOG <= driver.t.num)
		driver.t.num = YAS_MAG_TEMPERATURE_LOG;
	for (i = 0; i < driver.t.num; i++)
		sum += driver.t.log[i];
	tmp = sum * 10 / driver.t.num;
#else
	tmp = t * 10;
#endif
	sx  = xy1y2_linear[0];
	sy1 = xy1y2_linear[1];
	sy2 = xy1y2_linear[2];
	if (temp_correction) {
		sx  -= (c->Cx  * tmp) / 1000;
		sy1 -= (c->Cy1 * tmp) / 1000;
		sy2 -= (c->Cy2 * tmp) / 1000;
	}
	sy = sy1 - sy2;
	sz = -sy1 - sy2;
	xyz[0] = c->k * ((100   * sx + c->a2 * sy + c->a3 * sz) / 10);
	xyz[1] = c->k * ((c->a4 * sx + c->a5 * sy + c->a6 * sz) / 10);
	xyz[2] = c->k * ((c->a7 * sx + c->a8 * sy + c->a9 * sz) / 10);
	if (temperature != NULL)
		*temperature = t;
	if (driver.transform != NULL) {
		for (i = 0; i < 3; i++) {
			xyz_tmp[i] = driver.transform[i*3] * xyz[0]
				+ driver.transform[i*3+1] * xyz[1]
				+ driver.transform[i*3+2] * xyz[2];
		}
	} else
		for (i = 0; i < 3; i++)
			xyz_tmp[i] = xyz[i];
	for (i = 0; i < 3; i++) {
		xyz_tmp[i] -= xyz_tmp[i] % 10;
		if (ouflow & (1<<(i*2)))
			xyz_tmp[i] += 1; /* set overflow */
		if (ouflow & (1<<(i*2+1)))
			xyz_tmp[i] += 2; /* set underflow */
	}
	set_vector(xyz, xyz_tmp);
	if (busy)
		return YAS_ERROR_BUSY;
	return ouflow;
}

static int yas_cdrv_set_transformatiom_matrix(const int8_t *transform) {
	static int8_t t[9];
	if (transform == NULL)
		driver.transform = NULL;
	else {
		set_matrix(t, transform);
		driver.transform = t;
	}
	return YAS_NO_ERROR;
}

static int yas_get_position(void) {
	if (!driver.initialized)
		return 0;
	return driver.position;
}

static int yas_set_position(int position) {
	if (!driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (position < 0 || 7 < position)
		return YAS_ERROR_ARG;
	if (position == YAS_MAG_NOTRANS_POSITION)
		yas_cdrv_set_transformatiom_matrix(NULL);
	else
		yas_cdrv_set_transformatiom_matrix(
				YAS_TRANSFORMATION[position]);
	driver.position = position;
	return YAS_NO_ERROR;
}

static int yas_get_offset(int8_t *hard_offset) {
	if (!driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (hard_offset == NULL)
		return YAS_ERROR_ARG;
	set_vector(hard_offset, driver.hard_offset);
	return YAS_NO_ERROR;
}

static int yas_set_offset(const int8_t *hard_offset) {
	if (!driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (hard_offset == NULL)
		return YAS_ERROR_ARG;
	if (yas_cdrv_actuate_initcoil() < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (is_valid_offset(hard_offset)) {
		if (yas_cdrv_set_offset(hard_offset) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		driver.measure_state = YAS_MAG_STATE_NORMAL;
	} else {
		set_vector(driver.hard_offset, INVALID_OFFSET);
		driver.measure_state = YAS_MAG_STATE_MEASURE_OFFSET;
	}
	return YAS_NO_ERROR;
}

static int yas_measure(struct yas_mag_data *data, int temp_correction) {
	int result = 0, rt;
	int8_t hard_offset[3];
	uint32_t t;
	if (!driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (data == NULL)
		return YAS_ERROR_ARG;
	switch (driver.measure_state) {
	case YAS_MAG_STATE_INIT_COIL:
		driver.cbk.current_time(&t);
		if (t - driver.overflow_time < YAS_MAG_INITCOIL_TIMEOUT)
			break;
		driver.overflow_time = t;
		if (yas_cdrv_actuate_initcoil() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (!driver.overflow && is_valid_offset(driver.hard_offset)) {
			driver.measure_state = YAS_MAG_STATE_NORMAL;
			break;
		}
		/* FALLTHRU */
	case YAS_MAG_STATE_MEASURE_OFFSET:
		set_vector(hard_offset, driver.hard_offset);
		rt = yas_cdrv_measure_and_set_offset();
		if (rt < 0)
			return rt;
		if (is_vector_differ(driver.hard_offset, hard_offset))
			result = YAS_REPORT_HARD_OFFSET_CHANGED;
		driver.measure_state = YAS_MAG_STATE_NORMAL;
		break;
	}
	rt = yas_cdrv_measure(data->xyz.v, data->xy1y2, data->xy1y2_linear,
			&data->temperature, temp_correction);
	if (rt < 0)
		return rt;
	if (0 < rt) {
		if (!driver.overflow)
			driver.cbk.current_time(&driver.overflow_time);
		driver.overflow = 1;
		driver.measure_state = YAS_MAG_STATE_INIT_COIL;
		result |= rt;
	} else
		driver.overflow = 0;
	return result;
}

static int yas_measure_wrap(struct yas_mag_data *data) {
	return yas_measure(data, 1);
}

static int yas_init(void) {
	int i, rt;
	uint8_t data;
	if (driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	driver.measure_state = YAS_MAG_STATE_INIT_COIL;
	driver.overflow = 0;
	driver.cbk.current_time(&driver.overflow_time);
	driver.position = YAS_MAG_NOTRANS_POSITION;
	driver.transform = NULL;
#if 1 < YAS_MAG_TEMPERATURE_LOG
	driver.t.num = driver.t.idx = 0;
#endif
	set_vector(driver.hard_offset, INVALID_OFFSET);
	if (driver.cbk.device_open() < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	data = 0;
	if (driver.cbk.device_write(YAS_REG_TEST1, &data, 1) < 0) {
		driver.cbk.device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (driver.cbk.device_write(YAS_REG_TEST2, &data, 1) < 0) {
		driver.cbk.device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (driver.cbk.device_read(YAS_REG_DEVICE_ID, &data, 1) < 0) {
		driver.cbk.device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	driver.dev_id = data;
	switch (driver.dev_id) {
	case YAS_YAS530_DEVICE_ID:
		rt = get_cal_data_yas530(&driver.correct);
		if (rt < 0) {
			driver.cbk.device_close();
			return rt;
		}
		driver.center_thresh = YAS_YAS530_DATA_CENTER;
		driver.underflow_thresh = YAS_YAS530_DATA_UNDERFLOW;
		driver.overflow_thresh = YAS_YAS530_DATA_OVERFLOW;
		switch (driver.correct.ver) {
		case YAS_YAS530_VERSION_B:
			for (i = 0; i < 3; i++)
				driver.coef[i]
					= YAS_YAS530_VERSION_B_COEF;
			break;
		case YAS_YAS530_VERSION_A:
		default:
			for (i = 0; i < 3; i++)
				driver.coef[i]
					= YAS_YAS530_VERSION_A_COEF;
			break;
		}
		break;
	case YAS_YAS532_DEVICE_ID:
		rt = get_cal_data_yas532(&driver.correct);
		if (rt < 0) {
			driver.cbk.device_close();
			return rt;
		}
		driver.center_thresh = YAS_YAS532_DATA_CENTER;
		driver.underflow_thresh = YAS_YAS532_DATA_UNDERFLOW;
		driver.overflow_thresh = YAS_YAS532_DATA_OVERFLOW;
		switch (driver.correct.ver) {
		case YAS_YAS532_VERSION_AC:
			for (i = 0; i < 3; i++)
				driver.coef[i]
					= yas532_version_ac_coef[i];
			break;
		case YAS_YAS532_VERSION_AB:
		default:
			for (i = 0; i < 3; i++)
				driver.coef[i]
					= YAS_YAS532_VERSION_AB_COEF;
			break;
		}
		break;
	default:
		driver.cbk.device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	rt = yas_cdrv_actuate_initcoil();
	if (rt < 0) {
		driver.cbk.device_close();
		return rt;
	}
	driver.measure_state = YAS_MAG_STATE_MEASURE_OFFSET;
	driver.initialized = 1;
	return YAS_NO_ERROR;
}

static int yas_term(void) {
	if (!driver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	driver.initialized = 0;
	return driver.cbk.device_close();
}

static int yas_self_test(struct yas_self_test_result *r) {
	struct yas_mag_data data;
	int rt, i;
	if (!driver.initialized)
		return YAS_ERROR_TEST_ORDER;
	if (r == NULL)
		return YAS_ERROR_ARG;
	r->id = driver.dev_id;
	rt = yas_set_offset(INVALID_OFFSET);
	if (rt < 0)
		return rt;
	rt = yas_measure(&data, 0);
	set_vector(r->xy1y2, driver.hard_offset);
	if (rt < 0)
		return rt;
	if (rt & YAS_OVERFLOW)
		return YAS_ERROR_OVERFLOW;
	if (rt & YAS_UNDERFLOW)
		return YAS_ERROR_UNDERFLOW;
	if (data.xyz.v[0] == 0 && data.xyz.v[1] == 0 && data.xyz.v[2] == 0)
		return YAS_ERROR_DIRCALC;
	r->dir = 99;
	for (i = 0; i < 3; i++)
		r->xyz[i] = data.xyz.v[i] / 1000;
	rt = yas_cdrv_sensitivity_measuremnet(&r->sx, &r->sy);
	if (rt < 0)
		return rt;
	if (rt & YAS_OVERFLOW)
		return YAS_ERROR_OVERFLOW;
	if (rt & YAS_UNDERFLOW)
		return YAS_ERROR_UNDERFLOW;
	return YAS_NO_ERROR;
}

static int yas_self_test_noise(struct yas_vector *raw_xyz) {
	struct yas_mag_data data;
	int rt;
	if (!driver.initialized)
		return YAS_ERROR_TEST_ORDER;
	if (raw_xyz == NULL)
		return YAS_ERROR_ARG;
	rt = yas_measure(&data, 0);
	raw_xyz->v[0] = data.xy1y2_linear[0];
	raw_xyz->v[1] = data.xy1y2_linear[1] - data.xy1y2_linear[2];
	raw_xyz->v[2] = -data.xy1y2_linear[1] - data.xy1y2_linear[2];
	if (rt < 0)
		return rt;
	if (rt & YAS_OVERFLOW)
		return YAS_ERROR_OVERFLOW;
	if (rt & YAS_UNDERFLOW)
		return YAS_ERROR_UNDERFLOW;
	return YAS_NO_ERROR;
}

int yas_mag_driver_init(struct yas_mag_driver *f) {
	if (f == NULL || f->callback.device_open == NULL
			|| f->callback.device_close == NULL
			|| f->callback.device_read == NULL
			|| f->callback.device_write == NULL
			|| f->callback.msleep == NULL
			|| f->callback.current_time == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->self_test = yas_self_test;
	f->self_test_noise = yas_self_test_noise;
	f->get_offset = yas_get_offset;
	f->set_offset = yas_set_offset;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure_wrap;
	driver.cbk = f->callback;
	yas_term();
	return YAS_NO_ERROR;
}

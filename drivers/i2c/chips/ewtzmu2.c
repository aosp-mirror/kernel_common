/* drivers/i2c/chips/ewtzmu2.c - Panasonic Gyro driver
 *
 * Copyright (C) 2011 Prolific Technology Inc.
 * Author: Kyle Chen
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

#define HTC_VERSION  1

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/ewtzmu2.h>
#include <linux/ewtzmu2_cal.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/akm8975.h>
#include <linux/module.h>

#ifndef HTC_VERSION
#include <linux/i2c/ak8973.h>
#endif

int debug_flag;
#define D(x...) printk(KERN_DEBUG "[GYRO][PANASONIC] " x)
#define I(x...) printk(KERN_INFO "[GYRO][PANASONIC] " x)
#define E(x...) printk(KERN_ERR "[GYRO][PANASONIC ERROR] " x)
#define DIF(x...) { \
	if (debug_flag) \
		printk(KERN_DEBUG "[GYRO][PANASONIC DEBUG] " x); }

#define EWTZMU_DRV_NAME         "ewtzmu2"
#define DRIVER_VERSION          "1.0.0.2"
#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)

static struct i2c_client *ewtzmu_i2c_client;
int Gyro_samplerate_status = 1;
static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static atomic_t open_flag;
static bool Gyro_init_fail = 0;

struct _ewtzmu_data {
	rwlock_t lock;
	int chipset;
	int mode;
	int i2c_read_addr;
	int i2c_read_len;
} ewtzmu_data;

struct ewtzmu_vec_t {
	int x;
	int y;
	int z;
};

struct ewtzmu_pedo_t {
	unsigned long pedo_step;
	unsigned long pedo_time;
	int pedo_stat;
};

struct _ewtzmumid_data {
	rwlock_t datalock;
	rwlock_t ctrllock;
	int controldata[EW_CB_LENGTH];
	int dirpolarity[EW_DP_LENGTH];
	int pedometerparam[EW_PD_LENGTH];
	int yaw;
	int roll;
	int pitch;
	struct ewtzmu_vec_t nm;
	struct ewtzmu_vec_t na;
	struct ewtzmu_vec_t gyro;
	struct ewtzmu_pedo_t pedo;
	struct ewtzmu_vec_t linear_accel;
	struct ewtzmu_vec_t gravity;
	int	rotationvector[4];
	int status;
	struct class *htc_gyro_class;
	struct device *gyro_dev;
	void (*config_gyro_diag_gpios)(bool enable);
	int sleep_pin;
} ewtzmumid_data;

struct ewtzmu_i2c_data {
	struct input_dev *input_dev_compass;
	struct input_dev *input_dev_gyroscope;
	struct i2c_client *client;
#ifdef HTC_VERSION
	struct pana_gyro_platform_data *pdata;
#else
	struct akm8973_platform_data *pdata;
#endif
};

static atomic_t dev_open_count;
#ifndef HTC_VERSION
static atomic_t hal_open_count;
#endif
static atomic_t daemon_open_count;

static atomic_t o_status;
static atomic_t a_status;
static atomic_t m_status;
static atomic_t g_status;
static atomic_t rv_status;
static atomic_t la_status;
static atomic_t gv_status;

static atomic_t off_status;
static atomic_t off_status_hal;
static int m_o_times;

static int EWTZMU2_I2C_Read(int reg_addr, int buf_len, int *buf)
{
	int res = 0;
	u8  regaddr;
	u8  readdata[64];

	memset(readdata, 0x00, sizeof(readdata));

	if (!ewtzmu_i2c_client) {
	*buf = 0;
	E("%s  error, ewtzmu_i2c_client = NULL\n", __func__);
	return -2;
	}

	regaddr = (u8)reg_addr;

	res = i2c_master_send(ewtzmu_i2c_client, &regaddr, 1);
	if (res <= 0) {
		E("%s EWTZMU2_I2C_Read error res = %d\n", __func__, res);
		return res;
	}

	udelay(20);
	res = i2c_master_recv(ewtzmu_i2c_client, readdata, buf_len);
	if (res <= 0) {
		E("%s EWTZMU2_I2C_Read error res = %d\n", __func__, res);
		return res;
	}

	memcpy(buf, (int *)readdata, buf_len);
	I("%s, ok, reg_addr = 0x%x,"
		"buf_len = %d\n",
		__func__, reg_addr, buf_len);
	return 1;
}

static int EWTZMU2_I2C_Write(int reg_addr, int buf_len, int *buf)
{
	int res = 0;
	u8 databuffer[64];

	memset(databuffer, 0x00, sizeof(databuffer));

	if ((buf_len+2) > 64) {
		E("%s  error, (buf_len+2) > 64),"
			"buf_len %d \n",
			__func__, buf_len);
		return -EINVAL;
	}

	if (!ewtzmu_i2c_client) {
	E("%s  error, ewtzmu_i2c_client = NULL\n", __func__);
	return -2;
	}

	databuffer[0] = (u8)reg_addr;
	memcpy(&databuffer[1], (u8 *)buf, (buf_len-1));

	res = i2c_master_send(ewtzmu_i2c_client, databuffer, buf_len);
	if (res <= 0)
		E("%s EWTZMU2_I2C_Write error res = %d\n", __func__, res);

	I("%s, ok, reg_addr = 0x%x,"
		"buf_len = %d\n",
		__func__, reg_addr, buf_len);
	return 1;
}
/*
static int EWTZMU2_Chipset_Init_read(void)
{
	int ctrl = 0;

	EWTZMU2_I2C_Read(EWTZMU_REG_PWR_MGM, 1, &ctrl);
	I("EWTZMU_REG_PWR_MGM ret value=0x%x\n", ctrl);
	EWTZMU2_I2C_Read(EWTZMU_INT, 1, &ctrl);
	I("EWTZMU_INT ret value=0x%x\n", ctrl);
	EWTZMU2_I2C_Read(EWTZMU_DLPF, 1, &ctrl);
	I("EWTZMU_DLPF ret value=0x%x\n", ctrl);
	EWTZMU2_I2C_Read(EWTZMU_SMPL, 1, &ctrl);
	I("EWTZMU_SMPL ret value=0x%x\n", ctrl);
	EWTZMU2_I2C_Read(EWTZMU_FIFO_CTR, 1, &ctrl);
	I("EWTZMU_FIFO_CTR ret value=0x%x\n", ctrl);
	return 0;
}
*/
static int EWTZMU2_GetOpenStatus(void)
{
	I("%s:\n", __func__);
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) > 0));
	I("%s:wait OK\n", __func__);
	return atomic_read(&open_flag);
}

static int EWTZMU2_Chipset_Init(void)
{
	u8 databuf[10];
	u8 regaddr;
	u8 ctrl = 0;
	int res = 0;

	regaddr = EWTZMU_REG_PWR_MGM;
	res = i2c_master_send(ewtzmu_i2c_client, &regaddr, 1);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;
	res = i2c_master_recv(ewtzmu_i2c_client, &ctrl, 1);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	databuf[0] = EWTZMU_REG_PWR_MGM;
	databuf[1] = ctrl | EWTZMU_POWERON;
	res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	databuf[0] = EWTZMU_INT;
	databuf[1] = EWTZMU_WTMON;
	res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	databuf[0] = EWTZMU_DLPF;
	databuf[1] = EWTZMU_2000ds_1khz|EWTZMU_HPF;
	res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	databuf[0] = EWTZMU_SMPL;
	databuf[1] = EWTZMU_100hz;
	res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	databuf[0] = EWTZMU_FIFO_CTR;
	databuf[1] = EWTZMU_stream | EWTZMU_SAMPLE_50HZ;
	res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
	if (res <= 0)
		goto exit_EWTZMU2_Chipset_Init;

	I("init chipset: ret value=%d\n", res);
exit_EWTZMU2_Chipset_Init:
	if (res <= 0) {
	E("Fail to init chipset(I2C error): ret value=%d\n", res);
	Gyro_init_fail = 1;
	return -1;
	}
	return 0;
}

static int EWTZMU2_Chip_Set_SampleRate(int sample_rate_state)
{
	u8 databuf[10];
	int res = 0;

	I("sample_rate_state=%d\n", sample_rate_state);
	if (gpio_get_value(ewtzmumid_data.sleep_pin) == 1) {/*sleep*/
		I("Dont set sample_rate_state=%d, when gyro sleep\n", sample_rate_state);
		Gyro_samplerate_status = sample_rate_state;
		return 0;
	}
	if (sample_rate_state == 0) {
		Gyro_samplerate_status = 0;
		databuf[0] = EWTZMU_FIFO_CTR;
		databuf[1] = EWTZMU_stream | EWTZMU_SAMPLE_100HZ;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		if (res <= 0)
			goto exit_EWTZMU2_Chip_Set_SampleRate;
	} else if (sample_rate_state == 1) {
		Gyro_samplerate_status = 1;
		databuf[0] = EWTZMU_FIFO_CTR;
		databuf[1] = EWTZMU_stream | EWTZMU_SAMPLE_50HZ;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		if (res <= 0)
			goto exit_EWTZMU2_Chip_Set_SampleRate;
	} else if (sample_rate_state == 2) {
		Gyro_samplerate_status = 2;
		databuf[0] = EWTZMU_FIFO_CTR;
		databuf[1] = EWTZMU_stream | EWTZMU_SAMPLE_16HZ;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		if (res <= 0)
			goto exit_EWTZMU2_Chip_Set_SampleRate;
	} else if (sample_rate_state == 3) {
		Gyro_samplerate_status = 3;
		databuf[0] = EWTZMU_FIFO_CTR;
		databuf[1] = EWTZMU_stream | EWTZMU_SAMPLE_5HZ;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		if (res <= 0)
			goto exit_EWTZMU2_Chip_Set_SampleRate;
	} else {
		goto exit_EWTZMU2_Chip_Set_SampleRate;
	}

exit_EWTZMU2_Chip_Set_SampleRate:
	if (res <= 0) {
		E("Fail to init chipset(I2C error): ret value=%d\n", res);
		return -1;
	}
	return 0;
}

static int EWTZMU2_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 30))
		return -1;

	if (!ewtzmu_i2c_client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "EWTZMU2 Chip");

	return 0;
}

static int EWTZMU2_WIA(char *wia, int bufsize)
{
	unsigned char databuf[10];

	databuf[0] = 0x1D;
	sprintf(wia, "%02x", databuf[0]);
	I("WIA=%x", databuf[0]);

	return 0;
}

/*no use*/
static int EWTZMU2_ReadSensorData(char *buf, int bufsize)
{
	char cmd;
	int mode = 0;
	unsigned char databuf[10];
	int gyrox, gyroy, gyroz;
	int res = EW_DRV_SUCCESS;

	if ((!buf) || (bufsize <= 80))
	return EW_BUFFER_PARAMS;/*-1;*/

	if (!ewtzmu_i2c_client) {
		*buf = 0;
		return EW_CLIENT_ERROR;/*-2;*/
	}

	read_lock(&ewtzmu_data.lock);
	mode = ewtzmu_data.mode;
	read_unlock(&ewtzmu_data.lock);

	gyrox = gyroy = gyroz = 0;
	/* We can read all measured data in once*/
	cmd = EWTZMU_REG_GYROX_H;
	res = i2c_master_send(ewtzmu_i2c_client, &cmd, 1);
	if (res <= 0)
		goto exit_EWTZMU2_ReadSensorData;
	udelay(20);
	res = i2c_master_recv(ewtzmu_i2c_client, &(databuf[0]), 6);
	if (res <= 0)
		goto exit_EWTZMU2_ReadSensorData;
	/*gxh, gx1, gyh, gyl, gzh, gzl*/
	gyrox = (databuf[0] << 8) | databuf[1];
	if (gyrox > 32768)
		gyrox -= 65536;
	gyroy = (databuf[2] << 8) | databuf[3];
	if (gyroy > 32768)
		gyroy -= 65536;
	gyroz = (databuf[4] << 8) | databuf[5];
	if (gyroz > 32768)
		gyroz -= 65536;

exit_EWTZMU2_ReadSensorData:
	if (res <= 0) {
		E("Fail to read sensor data(I2C error): ret value=%d\n", res);
		return -3;
	} else {
		sprintf(buf, "%4x %4x %4x", gyrox, gyroy, gyroz);
		res = EW_DRV_SUCCESS;
	}
	return res;
}


static int EWTZMU2_ReadSensorDataFIFO(unsigned char *buf, int bufsize)
{
	char cmd;
	int mode = 0;
	unsigned char databuf[200];
	int res = EW_DRV_SUCCESS, databyte = 6;

	if ((!buf) || (bufsize < 121))
		return EW_BUFFER_PARAMS;

	if (!ewtzmu_i2c_client) {
		*buf = 0;
		return EW_CLIENT_ERROR;
	}

	read_lock(&ewtzmu_data.lock);
	mode = ewtzmu_data.mode;
	read_unlock(&ewtzmu_data.lock);

	cmd = EWTZMU_REG_GYROX_H;
	res = i2c_master_send(ewtzmu_i2c_client, &cmd, 1);
	if (res <= 0)
		goto exit_EWTZMU2_ReadSensorDataFIFO;
	udelay(20);

	if (Gyro_samplerate_status == 0) {/*100HZ*/
		databyte = 6;
		res = i2c_master_recv(ewtzmu_i2c_client, &(databuf[0]), 6);
	} else if (Gyro_samplerate_status == 1) {/*50HZ*/
		databyte = 12;
		res = i2c_master_recv(ewtzmu_i2c_client, &(databuf[0]), 12);
	} else if (Gyro_samplerate_status == 2) {/*16HZ*/
		databyte = 36;
		res = i2c_master_recv(ewtzmu_i2c_client, &(databuf[0]), 36);
	} else if (Gyro_samplerate_status == 3) {/*5HZ*/
		databyte = 120;
		res = i2c_master_recv(ewtzmu_i2c_client, &(databuf[0]), 120);
	}

	if (res <= 0)
		goto exit_EWTZMU2_ReadSensorDataFIFO;
exit_EWTZMU2_ReadSensorDataFIFO:
	if (res <= 0) {
	E("Fail to read sensor data(I2C error): ret value=%d\n", res);
		res = EW_I2C_ERROR;
	} else {
		memcpy(&(buf[1]), databuf, databyte);
		buf[0] = (unsigned char) databyte;
		DIF("sensordata 1:%d %d ,%d %d, %d %d\n",
		databuf[0], databuf[1], databuf[2],
		databuf[3], databuf[4], databuf[5]);
		DIF("sensordata 2:%d %d ,%d %d, %d %d\n",
		databuf[6], databuf[7], databuf[8],
		 databuf[9], databuf[10], databuf[11]);
		res = EW_DRV_SUCCESS;
	}
	return res;
}

static int Set_Report_Sensor_Flag(int controldata_active_sensor)
{
	if (controldata_active_sensor & EW_BIT_ORIENTATION) {
		atomic_set(&o_status, 1);
	} else {
		m_o_times = 0;
		atomic_set(&o_status, 0);
	}
	if (controldata_active_sensor & EW_BIT_ACCELEROMETER)
		atomic_set(&a_status, 1);
	else
		atomic_set(&a_status, 0);

	if (controldata_active_sensor & EW_BIT_MAGNETIC_FIELD)
		atomic_set(&m_status, 1);
	else
		atomic_set(&m_status, 0);

	if (controldata_active_sensor & EW_BIT_GYROSCOPE)
		atomic_set(&g_status, 1);
	else
		atomic_set(&g_status, 0);

	if (controldata_active_sensor & EW_BIT_ROTATION_VECTOR)
		atomic_set(&rv_status, 1);
	else
		atomic_set(&rv_status, 0);

	if (controldata_active_sensor & EW_BIT_LINEAR_ACCELERATION)
		atomic_set(&la_status, 1);
	else
		atomic_set(&la_status, 0);

	if (controldata_active_sensor & EW_BIT_GRAVITY)
		atomic_set(&gv_status, 1);
	else
		atomic_set(&gv_status, 0);
	return 0;
}

static int EWTZMU2_ReadPostureData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d %d",
		ewtzmumid_data.yaw, ewtzmumid_data.pitch,
		ewtzmumid_data.roll, ewtzmumid_data.status);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadCaliData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d %d %d %d %d",
		ewtzmumid_data.nm.x, ewtzmumid_data.nm.y,
		ewtzmumid_data.nm.z, ewtzmumid_data.na.x,
		ewtzmumid_data.na.y, ewtzmumid_data.na.z, ewtzmumid_data.status);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadGyroData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d", ewtzmumid_data.gyro.x,
		ewtzmumid_data.gyro.y, ewtzmumid_data.gyro.z);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadPedoData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%ld %ld %d", ewtzmumid_data.pedo.pedo_step,
	ewtzmumid_data.pedo.pedo_time,
	ewtzmumid_data.pedo.pedo_stat);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadMiddleControl(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.ctrllock);
	sprintf(buf, "%d %d %d %d %d %d %d %d %d %d",
	ewtzmumid_data.controldata[EW_CB_LOOPDELAY],
	ewtzmumid_data.controldata[EW_CB_RUN],
	ewtzmumid_data.controldata[EW_CB_ACCCALI],
	ewtzmumid_data.controldata[EW_CB_MAGCALI],
	ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS],
	ewtzmumid_data.controldata[EW_CB_PD_RESET],
	ewtzmumid_data.controldata[EW_CB_PD_EN_PARAM],
	ewtzmumid_data.controldata[EW_CB_GYROCALI],
	ewtzmumid_data.controldata[EW_CB_ALGORITHMLOG],
	ewtzmumid_data.controldata[EW_CB_UNDEFINE_1]);
	read_unlock(&ewtzmumid_data.ctrllock);
	return 0;
}

static int EWTZMU2_ReadRotationVector(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d %d", ewtzmumid_data.rotationvector[0],
	ewtzmumid_data.rotationvector[1], ewtzmumid_data.rotationvector[2],
	ewtzmumid_data.rotationvector[3]);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadLinearAccel(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d", ewtzmumid_data.linear_accel.x,
	ewtzmumid_data.linear_accel.y, ewtzmumid_data.linear_accel.z);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}

static int EWTZMU2_ReadGravity(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ewtzmumid_data.datalock);
	sprintf(buf, "%d %d %d", ewtzmumid_data.gravity.x,
	ewtzmumid_data.gravity.y, ewtzmumid_data.gravity.z);
	read_unlock(&ewtzmumid_data.datalock);
	return 0;
}
int EWTZMU2_Report_Value_akm(int ifirst, int x, int y, int z)
{
	struct ewtzmu_i2c_data *data;
	static int report_times;
	static int x_data, y_data, z_data;

	if (Gyro_init_fail) {
		E("%s  error, init fail. do nothing\n", __func__);
		return -1;
	}

	if (!ewtzmu_i2c_client) {
		E("%s  error, ewtzmu_i2c_client = NULL\n", __func__);
		return -1;
	}
	data = i2c_get_clientdata(ewtzmu_i2c_client);
	if (ifirst == 1) {
		if (x == x_data) {
			x = x_data+1;/*for not to filter by input system*/
			I("a_status : gsensor data x data same, so +1 : %d, %d \n", x, x_data);
		}

		if (y == y_data) {
			y = y_data+1;
			I("a_status : gsensor data y data same, so +1 : %d, %d \n", y, y_data);
		}

		if (z == z_data) {
			z = z_data+1;
			I("a_status : gsensor data z data same, so +1 : %d, %d \n", z, z_data);
		}
		I("a_status : first gsensor data: %d, %d, %d\n", x, z, y);
	}
	x_data = x;
	y_data = y;
	z_data = z;

	report_times++;
	if (report_times > ((200 - (Gyro_samplerate_status * 50)) / (1 + Gyro_samplerate_status))) {
		I("a_status : gsensor data: %d, %d, %d\n", x, z, y);
		report_times = 0;
	}
	input_report_abs(data->input_dev_compass, ABS_X, x);/* x-axis raw acceleration */
	input_report_abs(data->input_dev_compass, ABS_Y, y);/* y-axis raw acceleration */
	input_report_abs(data->input_dev_compass, ABS_Z, z);/* z-axis raw acceleration */
	input_sync(data->input_dev_compass);

	return 0;
}

int EWTZMU2_Report_Value(void)
{
	struct ewtzmu_i2c_data *data = i2c_get_clientdata(ewtzmu_i2c_client);
	int report_enable = 0;
	static int report_gyro_times;
	static int report_pitch_times;
	static int report_rota_times;

	DIF("EWTZMU2_Report_Value\n");
	if (atomic_read(&o_status) && m_o_times) {
		report_pitch_times++;
		if (report_pitch_times > ((200 - (Gyro_samplerate_status * 50)) / (1 + Gyro_samplerate_status))) {
			I("o_status, pitch %d, roll %d, , yaw %d\n",
				ewtzmumid_data.pitch, ewtzmumid_data.roll, ewtzmumid_data.yaw);
			report_pitch_times = 0;
		}
		DIF("EWTZMU2_Report_Value o_status, pitch %d, roll %d, , yaw %d\n",
		ewtzmumid_data.pitch, ewtzmumid_data.roll, ewtzmumid_data.yaw);
		input_report_abs(data->input_dev_compass, ABS_RX, ewtzmumid_data.yaw);/* yaw */
		input_report_abs(data->input_dev_compass, ABS_RY, ewtzmumid_data.pitch);/* pitch */
		input_report_abs(data->input_dev_compass, ABS_RZ, ewtzmumid_data.roll);/* roll */
		input_report_abs(data->input_dev_compass, ABS_RUDDER, ewtzmumid_data.status);
				/* status of orientation sensor */
		report_enable = EW_REPORT_EN_COMPASS;
	}/*
	if (atomic_read(&a_status)) {
		report_times++;
		if (report_times > ((200 - (Gyro_samplerate_status * 50)) / (1 + Gyro_samplerate_status))) {
			I("a_status : gsensor data: %d, %d, %d\n", ewtzmumid_data.na.x,
				-ewtzmumid_data.na.z, ewtzmumid_data.na.y);
			report_times = 0;
		}
		input_report_abs(data->input_dev_compass, ABS_X, ewtzmumid_data.na.x);
		input_report_abs(data->input_dev_compass, ABS_Y, ewtzmumid_data.na.y);
		input_report_abs(data->input_dev_compass, ABS_Z, ewtzmumid_data.na.z);
		report_enable = EW_REPORT_EN_COMPASS;
	}*/

	if (atomic_read(&m_status)) {
		DIF("EWTZMU2_Report_Value m_status\n ");
		input_report_abs(data->input_dev_compass,
		ABS_HAT0X, ewtzmumid_data.nm.x);/* x-axis of raw magnetic vector */
	input_report_abs(data->input_dev_compass,
	 ABS_HAT0Y, ewtzmumid_data.nm.y);/* y-axis of raw magnetic vector */
	input_report_abs(data->input_dev_compass, ABS_BRAKE,
	ewtzmumid_data.nm.z);/* z-axis of raw magnetic vector */
	input_report_abs(data->input_dev_compass, ABS_WHEEL,
	ewtzmumid_data.status);/* status of magnetic sensor */
	report_enable = EW_REPORT_EN_COMPASS;
	}
	if (atomic_read(&rv_status)) {
		report_rota_times++;
		if (report_rota_times > ((200 - (Gyro_samplerate_status * 50)) / (1 + Gyro_samplerate_status))) {
			I("rv_status, %d, %d, %d, %d\n ",
			 ewtzmumid_data.rotationvector[0],
			ewtzmumid_data.rotationvector[1],
			ewtzmumid_data.rotationvector[2],
			ewtzmumid_data.rotationvector[3]);
			report_rota_times = 0;
		}
		DIF("EWTZMU2_Report_Value rv_status, %d, %d, %d, %d\n ",
			 ewtzmumid_data.rotationvector[0],
			ewtzmumid_data.rotationvector[1],
			ewtzmumid_data.rotationvector[2],
			ewtzmumid_data.rotationvector[3]);
		input_report_abs(data->input_dev_compass, ABS_HAT3X,
		ewtzmumid_data.rotationvector[0]);/* x-axis of rotation vector */
		input_report_abs(data->input_dev_compass, ABS_HAT3Y,
		ewtzmumid_data.rotationvector[1]);/* y-axis of rotation vectorn */
		input_report_abs(data->input_dev_compass, ABS_TILT_X,
		ewtzmumid_data.rotationvector[2]);/* z-axis of rotation vector */
		input_report_abs(data->input_dev_compass, ABS_TILT_Y,
		ewtzmumid_data.rotationvector[3]);/* theta of rotation vector */
		report_enable = EW_REPORT_EN_COMPASS;
	}
	if (atomic_read(&la_status)) {
		DIF("EWTZMU2_Report_Value la_status\n ");
		input_report_abs(data->input_dev_compass, ABS_HAT1X,
		ewtzmumid_data.linear_accel.x);/* x-axis of linear acceleration */
		input_report_abs(data->input_dev_compass, ABS_HAT1Y,
		ewtzmumid_data.linear_accel.y);/* y-axis of linear acceleration */
		input_report_abs(data->input_dev_compass, ABS_TOOL_WIDTH,
		ewtzmumid_data.linear_accel.z);/* z-axis of linear acceleration */
		report_enable = EW_REPORT_EN_COMPASS;
	}

	if (atomic_read(&gv_status)) {
		DIF("EWTZMU2_Report_Value gv_status\n ");
		input_report_abs(data->input_dev_compass, ABS_HAT2X,
		ewtzmumid_data.gravity.x);/* x-axis of gravityr */
		input_report_abs(data->input_dev_compass, ABS_HAT2Y,
		ewtzmumid_data.gravity.y);/* y-axis of gravity */
		input_report_abs(data->input_dev_compass, ABS_VOLUME,
		ewtzmumid_data.gravity.z);/* z-axis of gravity */
		report_enable = EW_REPORT_EN_COMPASS;
	}

	if (EW_REPORT_EN_COMPASS == report_enable) {
		DIF("EWTZMU2_Report_Value EW_REPORT_EN_COMPASS, o = %d, a = %d, g =%d\n ",
		atomic_read(&o_status), atomic_read(&a_status), atomic_read(&m_status));
		input_event(data->input_dev_compass, EV_SYN, SYN_REPORT, 1);
		input_sync(data->input_dev_compass);
	}

	if (atomic_read(&g_status)) {
		report_gyro_times++;
		if (report_gyro_times > ((200 - (Gyro_samplerate_status * 50)) / (1 + Gyro_samplerate_status))) {
			I("gyro_status, %d, %d, %d\n ",
			 ewtzmumid_data.gyro.x,
			ewtzmumid_data.gyro.y,
			ewtzmumid_data.gyro.z);
			report_gyro_times = 0;
		}
		DIF("EWTZMU2_Report_Value g_status\n, ");
		input_report_rel(data->input_dev_gyroscope, REL_RX,
		ewtzmumid_data.gyro.x);/* x-axis of gyro sensor */
		input_report_rel(data->input_dev_gyroscope, REL_RY,
		ewtzmumid_data.gyro.y);/* y-axis of gyro sensor */
		input_report_rel(data->input_dev_gyroscope, REL_RZ,
		ewtzmumid_data.gyro.z);/* z-axis of gyro sensor */
		report_enable = EW_REPORT_EN_GYROSCOPE;
	}

	if (EW_REPORT_EN_GYROSCOPE == report_enable) {
		DIF("EWTZMU2_Report_Value EW_REPORT_EN_GYROSCOPE\n ");
		input_event(data->input_dev_gyroscope, EV_SYN, SYN_REPORT, 1);
		input_sync(data->input_dev_gyroscope);
	}

	return 0;
}

static int EWTZMU2_Power_Off(void)
{
	u8 databuf[10];
	int res = 0;
	ewtzmumid_data.config_gyro_diag_gpios(1);
	if (!ewtzmu_i2c_client) {
		E("%s, ewtzmu_i2c_client < 0 \n", __func__);
		return -2;
	}
	/*add how to power off ewtzmu2*/
	if (gpio_get_value(ewtzmumid_data.sleep_pin) == 0) {/*no sleep*/
		databuf[0] = EWTZMU_REG_PWR_MGM;
		databuf[1] = EWTZMU_SLEP;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		if (res <= 0)
			E("Fail to power off chipset(I2C error): ret value=%d\n", res);
		msleep(10);
	}
	gpio_set_value(ewtzmumid_data.sleep_pin, 1);/*after any i2c cmd*/
	I("%s\n", __func__);
	return 0;
}

static int EWTZMU2_Power_On(void)
{
	u8 databuf[10];
	int res = 0;
	int i = 0;

	ewtzmumid_data.config_gyro_diag_gpios(0);
	if (!ewtzmu_i2c_client) {
		E("%s, ewtzmu_i2c_client < 0 \n", __func__);
	return -2;
	}
	gpio_set_value(ewtzmumid_data.sleep_pin, 0);/*before any i2c cmd*/
	msleep(50);
	while (1) {
		databuf[0] = EWTZMU_REG_PWR_MGM;
		databuf[1] = EWTZMU_POWERON;
		res = i2c_master_send(ewtzmu_i2c_client, databuf, 2);
		msleep(10);
		if (res > 0 || i > 10) {
			if (res <= 0)
				E("Fail to power on chipset(I2C error):ret value=%d\n", res);
			break;
		}
		i++;
	}
	i = 0;
	I("%s start\n", __func__);
	while (1) {
		res = EWTZMU2_Chipset_Init();
		msleep(10);
		if (res == 0 || i > 3)
			break;
		i++;
	}
	i = 0;
	EWTZMU2_Chip_Set_SampleRate(Gyro_samplerate_status);
	I("%s end\n", __func__);
	msleep(50);
	return 0;
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadChipInfo(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadSensorData(strbuf, EW_BUFSIZE);
return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_posturedata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadPostureData(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_calidata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadCaliData(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_gyrodata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadGyroData(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_rv_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadRotationVector(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_ladata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadLinearAccel(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_gravitydata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadGravity(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_midcontrol_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_ReadMiddleControl(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t store_midcontrol_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	write_lock(&ewtzmumid_data.ctrllock);
	memcpy(&ewtzmumid_data.controldata[0], buf, sizeof(int)*EW_CB_LENGTH);
	write_unlock(&ewtzmumid_data.ctrllock);
	return count;
}

static ssize_t show_mode_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode = 0;
	read_lock(&ewtzmu_data.lock);
	mode = ewtzmu_data.mode;
	read_unlock(&ewtzmu_data.lock);
	return sprintf(buf, "%d\n", mode);
}

static ssize_t store_mode_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d", &mode);
	return count;
}

static ssize_t show_wia_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[EW_BUFSIZE];
	EWTZMU2_WIA(strbuf, EW_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static DEVICE_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DEVICE_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DEVICE_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);
static DEVICE_ATTR(gyrodata, S_IRUGO, show_gyrodata_value, NULL);
static DEVICE_ATTR(rvdata, S_IRUGO, show_rv_value, NULL);
static DEVICE_ATTR(ladata, S_IRUGO, show_ladata_value, NULL);
static DEVICE_ATTR(gravitydata, S_IRUGO, show_gravitydata_value, NULL);
static DEVICE_ATTR(midcontrol, S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode_value, store_mode_value);
static DEVICE_ATTR(wia, S_IRUGO, show_wia_value, NULL);

static struct attribute *ewtzmu2_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_posturedata.attr,
	&dev_attr_calidata.attr,
	&dev_attr_gyrodata.attr,
	&dev_attr_rvdata.attr,
	&dev_attr_ladata.attr,
	&dev_attr_gravitydata.attr,
	&dev_attr_midcontrol.attr,
	&dev_attr_mode.attr,
	&dev_attr_wia.attr,
	NULL
};

static struct attribute_group ewtzmu2_attribute_group = {
	.attrs = ewtzmu2_attributes
};

static int ewtzmu2_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	if (atomic_cmpxchg(&dev_open_count, 0, 1) == 0) {
	I("Open device node:ewtzmu2\n");
	ret = nonseekable_open(inode, file);
	}
	return ret;
}

static int ewtzmu2_release(struct inode *inode, struct file *file)
{
	atomic_set(&dev_open_count, 0);
	I("Release device node:ewtzmu2\n");
	return 0;
}
/*for cailibration*/
static long ewtzmu2_ioctl(/*struct inode *inode,*/struct file *file, unsigned int cmd, unsigned long arg)
{
	char strbuf[EW_BUFSIZE];
	int controlbuf[EW_CB_LENGTH];
	int dirpolarity[EW_DP_LENGTH];
	int valuebuf[4];
	int calidata[7];
	int gyrodata[3];
	long pedodata[3], ladata[3], gravitydata[3];
	int pedoparam[EW_PD_LENGTH];
	unsigned char databuf[EW_BUFSIZE];
	void __user *data;
	int retval = 0;
	int mode = 0, chipset = 0;
	int rotation_vector[4];
	int gyro_sample_rate;
	int i2creaddata[3];
	int i2cwrdata[64];

   /* if (!capable(CAP_SYS_ADMIN)) {
	retval = -EPERM;
	goto err_out;
	}  */

	switch (cmd) {

	D("@@@ewtzmu2_ioctl & cmd = %d  \n", cmd);

	case EW_IOCTL_SET_INIT:
		read_lock(&ewtzmu_data.lock);
		mode = ewtzmu_data.mode;
		chipset = ewtzmu_data.chipset;
		read_unlock(&ewtzmu_data.lock);
		retval = EWTZMU2_Chipset_Init();
		I("ewtzmu2_ioctl & cmd = %d,"
			"EWTZMU2_Chipset_Init, retval = %d\n",
			cmd, retval);
	break;

	case EW_IOCTL_SET_STANDBY:
	break;

	case EW_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (data == NULL)
		break;
		EWTZMU2_ReadChipInfo(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		retval = EWTZMU2_ReadSensorData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
			goto err_out;
	break;

	case EW_IOCTL_READ_SENSORDATA_FIFO:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		retval = EWTZMU2_ReadSensorDataFIFO(databuf, EW_BUFSIZE);
		if (copy_to_user(data, &(databuf[1]), databuf[0]))
			goto err_out;
	break;

	case EW_IOCTL_READ_POSTUREDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadPostureData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_POSTUREDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.yaw   = valuebuf[0];
		ewtzmumid_data.pitch = valuebuf[1];
		ewtzmumid_data.roll  = valuebuf[2];
		ewtzmumid_data.status = valuebuf[3];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EW_IOCTL_READ_CALIDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadCaliData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_CALIDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&calidata, data, sizeof(calidata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.nm.x = calidata[0];
		ewtzmumid_data.nm.y = calidata[1];
		ewtzmumid_data.nm.z = calidata[2];
		ewtzmumid_data.na.x = calidata[3];
		ewtzmumid_data.na.y = calidata[4];
		ewtzmumid_data.na.z = calidata[5];
		ewtzmumid_data.status = calidata[6];
		write_unlock(&ewtzmumid_data.datalock);
	break;
	case EW_IOCTL_SET_SAMPLERATE:
			data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&gyro_sample_rate, data, sizeof(gyro_sample_rate))) {
			retval = -EFAULT;
			goto err_out;
		}
		EWTZMU2_Chip_Set_SampleRate(gyro_sample_rate);

	case EW_IOCTL_READ_GYRODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadGyroData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_GYRODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.gyro.x = gyrodata[0];
		ewtzmumid_data.gyro.y = gyrodata[1];
		ewtzmumid_data.gyro.z = gyrodata[2];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EW_IOCTL_READ_PEDODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadPedoData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_PEDODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.pedo.pedo_step = pedodata[0];
		ewtzmumid_data.pedo.pedo_time = pedodata[1];
		ewtzmumid_data.pedo.pedo_stat = (int)pedodata[2];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EW_IOCTL_READ_PEDOPARAM:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(pedoparam, &ewtzmumid_data.pedometerparam[0], sizeof(pedoparam));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
		break;
		if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_PEDOPARAM:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EW_IOCTL_READ_CONTROL:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(controlbuf, &ewtzmumid_data.controldata[0], sizeof(controlbuf));
		read_unlock(&ewtzmumid_data.ctrllock);
		I("@@@EW_IOCTL_READ_CONTROL \n");
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
			retval = -EFAULT;
			I("@@@EW_IOCTL_READ_CONTROL & retval = %d\n",
			retval);
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_CONTROL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.controldata[0], controlbuf, sizeof(controlbuf));
		write_unlock(&ewtzmumid_data.ctrllock);
		Set_Report_Sensor_Flag(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS]);
		if (!atomic_read(&g_status) && !atomic_read(&rv_status) &&
		!atomic_read(&la_status) && !atomic_read(&gv_status) &&
		!atomic_read(&o_status) && !atomic_read(&off_status_hal)) {/*power off*/

				atomic_set(&off_status_hal, 1);
				I("cal_Gyro power off:g_status=%d"
					"off_status_hal=%d\n",
					atomic_read(&g_status),
					atomic_read(&off_status_hal));
				return EWTZMU2_Power_Off();
			} else if ((atomic_read(&g_status) || atomic_read(&rv_status) ||
			atomic_read(&la_status) || atomic_read(&gv_status) ||
			atomic_read(&o_status))  && atomic_read(&off_status_hal)) {/*power on*/

				atomic_set(&off_status_hal, 0);
				I("Cal_Gyro power on:g_status=%d"
					"off_status_hal=%d\n",
					atomic_read(&g_status),
					atomic_read(&off_status_hal));
				return EWTZMU2_Power_On();
			}
	break;

	case EW_IOCTL_WRITE_MODE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&mode, data, sizeof(mode))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_REPORT:
		EWTZMU2_Report_Value();
	break;

	case EW_IOCTL_READ_WIA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_WIA(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_READ_AXISINTERFERENCE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		memset(strbuf, 0x00, sizeof(strbuf));
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_GET_DIRPOLARITY:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(dirpolarity, &ewtzmumid_data.dirpolarity[0], sizeof(dirpolarity));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, dirpolarity, sizeof(dirpolarity))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_READ_ROTATION_VECTOR:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadRotationVector(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_ROTATION_VECTOR:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(rotation_vector, data, sizeof(rotation_vector))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.rotationvector[0], rotation_vector, sizeof(rotation_vector));
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EW_IOCTL_READ_LINEAR_ACCEL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadLinearAccel(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EW_IOCTL_WRITE_LINEAR_ACCEL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(ladata, data, sizeof(ladata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		ewtzmumid_data.linear_accel.x = ladata[0];
		ewtzmumid_data.linear_accel.y = ladata[1];
		ewtzmumid_data.linear_accel.z = ladata[2];
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EW_IOCTL_READ_GRAVITY:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadGravity(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
		break;

	case EW_IOCTL_WRITE_GRAVITY:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(gravitydata, data, sizeof(gravitydata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		ewtzmumid_data.gravity.x = gravitydata[0];
		ewtzmumid_data.gravity.y = gravitydata[1];
		ewtzmumid_data.gravity.z = gravitydata[2];
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EW_IOCTL_WRITE_I2CDATA:
			data = (void __user *)arg;
			if (data == NULL)
				break;
			if (copy_from_user(i2cwrdata,
				data, sizeof(i2cwrdata))) {
				retval = -EFAULT;
				goto err_out;
			}

			/*write buf order is reg_addr,
			buf_len(unit:byte), data*/
			I("%s: EW_IOCTL_WRITE_I2CDATA :"
			"i2caddr=0x%x,len=%d,data=0x%x",
			__func__, i2cwrdata[0],
			i2cwrdata[1], i2cwrdata[2]);
			retval = EWTZMU2_I2C_Write(i2cwrdata[0],
				i2cwrdata[1], &i2cwrdata[2]);
			if (retval != 1) {
				E("%s: EW_IOCTL_WRITE_I2CDATA Error :"
				"i2caddr=0x%x,len=%d,data=0x%x",
					__func__, i2cwrdata[0],
					i2cwrdata[1], i2cwrdata[2]);
			}
	break;

	case EW_IOCTL_WRITE_I2CADDR:
			data = (void __user *)arg;
			if (data == NULL)
				break;

			if (copy_from_user(i2creaddata,
				data, sizeof(i2creaddata))) {
				retval = -EFAULT;
				goto err_out;
			}

			read_lock(&ewtzmu_data.lock);
			I("%s: WRITE_I2CADDR:"
			"i2creaddata[0]=%d,i2creaddata[1]=%d",
				__func__, i2creaddata[0],
				i2creaddata[1]);
			ewtzmu_data.i2c_read_addr = i2creaddata[0];
			ewtzmu_data.i2c_read_len = i2creaddata[1];
			read_unlock(&ewtzmu_data.lock);
	break;

	case EW_IOCTL_READ_I2CDATA:
			data = (void __user *)arg;
			if (data == NULL)
				break;

			retval = EWTZMU2_I2C_Read(ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, &i2cwrdata[0]);
			I("%s: EW_IOCTL_READ_I2CDATA :"
				"R addr=0x%x,len=%d,data=0x%x",
				__func__, ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, i2cwrdata[0]);
			if (retval) {/*successful*/
				if (copy_to_user(data,
				i2cwrdata, ewtzmu_data.i2c_read_len)) {
					retval = -EFAULT;
					goto err_out;
				}
			} else {
				E("%s: EW_IOCTL_READ_I2CDATA :"
					"error R addr=0x%x,len=%d,data=0x%x",
				__func__,  ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, i2cwrdata[0]);
				retval = -EFAULT;
				goto err_out;
			}
	break;
	default:
		E("%s not supported = 0x%04x", __func__, cmd);
		retval = -ENOIOCTLCMD;
		break;
	}

err_out:
	return retval;
}

unsigned int ewtzmu2_poll(struct file *filp , poll_table *pwait)
{
	unsigned int mask = 0;

	if ((ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ORIENTATION) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GYROSCOPE) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_MAGNETIC_FIELD) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ACCELEROMETER) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ROTATION_VECTOR) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_LINEAR_ACCELERATION) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GRAVITY))
		mask |= POLLIN | POLLRDNORM;

    return mask;
}

static int ewtzmu2daemon_open(struct inode *inode, struct file *file)
{
    int ret = -1;
    if (atomic_cmpxchg(&daemon_open_count, 0, 1) == 0) {
	I("Open device node:ewtzmu2daemon\n");
	ret = 0;
    }
    return ret;
}

static int ewtzmu2daemon_release(struct inode *inode, struct file *file)
{
    atomic_set(&daemon_open_count, 0);
    I("Release device node:ewtzmu2daemon\n");
    return 0;
}



static long ewtzmu2daemon_ioctl(/*struct inode *inode,*/
struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int valuebuf[4];
	int calidata[7];
	int gyrodata[3];
	long pedodata[3], ladata[3], gravitydata[3];
	int controlbuf[EW_CB_LENGTH];
	int dirpolarity[EW_DP_LENGTH];
	unsigned char databuf[EW_BUFSIZE];
	int pedoparam[EW_PD_LENGTH];
	void __user *data;
	int retval = 0, i;
	int mode = 0, chipset = EW_CHIPSET;
	int gyro_sample_rate[3];
	int rotation_vector[4];
	short report_to_gyro_value[12] = {0};
	int akm_ready = 0;
	unsigned char pana_gyro_gsensor_kvalue[12];
	int i2creaddata[3];
	int i2cwrdata[64];

	switch (cmd) {

	case EWDAE_IOCTL_SET_INIT:
		read_lock(&ewtzmu_data.lock);
		mode = ewtzmu_data.mode;
		chipset = ewtzmu_data.chipset;
		read_unlock(&ewtzmu_data.lock);
		retval = EWTZMU2_Chipset_Init();
	break;

	case EWDAE_IOCTL_SET_STANDBY:
		EWTZMU2_GetOpenStatus();
		break;

	case EWDAE_IOCTL_GET_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadSensorData(databuf, EW_BUFSIZE);
		if (copy_to_user(data, databuf, strlen(databuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_GET_SENSORDATA_FIFO:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadSensorDataFIFO(databuf, EW_BUFSIZE);
		DIF("sensordata 1:lenth:%d, data: %d ,%d %d, %d %d\n",
			databuf[0], databuf[1], databuf[2],
			databuf[3], databuf[4], databuf[5]);
		DIF("sensordata 2:%d %d ,%d %d, %d %d, %d\n",
			databuf[6], databuf[7], databuf[8],
			databuf[9], databuf[10], databuf[11], databuf[12]);
		if (copy_to_user(data, &(databuf[1]), databuf[0])) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_SET_POSTURE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.yaw   = valuebuf[0];
		ewtzmumid_data.pitch = valuebuf[1];
		ewtzmumid_data.roll  = valuebuf[2];
		ewtzmumid_data.status = valuebuf[3];
		write_unlock(&ewtzmumid_data.datalock);
		if (atomic_read(&o_status) && m_o_times == 0) {
			m_o_times = 1;
			I("(o) = (0x%x), set m_o_times %d\n",
				atomic_read(&o_status), m_o_times);
		}
		if (atomic_read(&o_status) == 0) {
			if (m_o_times == 1)
				I("(o) = (0x%x),  set 0 to m_o_times %d\n",
					atomic_read(&o_status), m_o_times);
			m_o_times = 0;
		}
	break;

	case EWDAE_IOCTL_SET_CALIDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&calidata, data, sizeof(calidata))) {
			retval = -EFAULT;
			goto err_out;
		}
		for (i = 0; i < 7; i++)
			DIF("EWDAE_IOCTL_SET_CALIDATA :calidata[%d]: %d\n", i, calidata[i]);
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.nm.x = calidata[0];
		ewtzmumid_data.nm.y = calidata[1];
		ewtzmumid_data.nm.z = calidata[2];
		ewtzmumid_data.na.x = calidata[3];
		ewtzmumid_data.na.y = calidata[4];
		ewtzmumid_data.na.z = calidata[5];
		ewtzmumid_data.status = calidata[6];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EWDAE_IOCTL_SET_GYRODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.gyro.x = gyrodata[0];
		ewtzmumid_data.gyro.y = gyrodata[1];
		ewtzmumid_data.gyro.z = gyrodata[2];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EWDAE_IOCTL_SET_PEDODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.datalock);
		ewtzmumid_data.pedo.pedo_step = pedodata[0];
		ewtzmumid_data.pedo.pedo_time = pedodata[1];
		ewtzmumid_data.pedo.pedo_stat = (int)pedodata[2];
		write_unlock(&ewtzmumid_data.datalock);
	break;

	case EWDAE_IOCTL_GET_PEDOPARAM:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(pedoparam, &ewtzmumid_data.pedometerparam[0], sizeof(pedoparam));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_SET_PEDOPARAM:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EWDAE_IOCTL_GET_CONTROL:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(controlbuf, &ewtzmumid_data.controldata[0], sizeof(controlbuf));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_SET_SAMPLERATE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(gyro_sample_rate, data, sizeof(gyro_sample_rate))) {
			retval = -EFAULT;
			goto err_out;
		}
		EWTZMU2_Chip_Set_SampleRate(gyro_sample_rate[0]);
	break;

	case EWDAE_IOCTL_GET_DIRPOLARITY:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(dirpolarity, &ewtzmumid_data.dirpolarity[0], sizeof(dirpolarity));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, dirpolarity, sizeof(dirpolarity))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_SET_CONTROL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
			retval = -EFAULT;
			goto err_out;
		}
		DIF("@@@Gyro AP ewtzmu2daemon_ioctl EWDAE_IOCTL_SET_CONTROL\n");
		return 0;
	break;

	case EWDAE_IOCTL_SET_MODE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(&mode, data, sizeof(mode))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	/*Add for input_device sync*/
	case EWDAE_IOCTL_SET_REPORT:
		EWTZMU2_Report_Value();
	break;

	case EWDAE_IOCTL_GET_WIA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_WIA(databuf, EW_BUFSIZE);
		if (copy_to_user(data, databuf, strlen(databuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_GET_AXISINTERFERENCE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		memset(databuf, 0x00, sizeof(databuf));
		if (copy_to_user(data, databuf, strlen(databuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWDAE_IOCTL_SET_ROTATION_VECTOR:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(rotation_vector, data, sizeof(rotation_vector))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.rotationvector[0], rotation_vector, sizeof(rotation_vector));
		write_unlock(&ewtzmumid_data.ctrllock);
		DIF("rotationvector:%d %d ,%d %d\n",
			ewtzmumid_data.rotationvector[0], ewtzmumid_data.rotationvector[1],
			ewtzmumid_data.rotationvector[2], ewtzmumid_data.rotationvector[3]);
	break;

	case EWDAE_IOCTL_SET_LINEAR_ACCEL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(ladata, data, sizeof(ladata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		ewtzmumid_data.linear_accel.x = ladata[0];
		ewtzmumid_data.linear_accel.y = ladata[1];
		ewtzmumid_data.linear_accel.z = ladata[2];
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EWDAE_IOCTL_SET_GRAVITY:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(gravitydata, data, sizeof(gravitydata))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		ewtzmumid_data.gravity.x = gravitydata[0];
		ewtzmumid_data.gravity.y = gravitydata[1];
		ewtzmumid_data.gravity.z = gravitydata[2];
		write_unlock(&ewtzmumid_data.ctrllock);
	break;
	case EWDAE_IOCTL_GET_AKM_DATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		DIF("%s: EWDAE_IOCTL_GET_AKM_DATA = 0x%x\n", __func__, EWDAE_IOCTL_GET_AKM_DATA);
		akm_get_akmd_data(report_to_gyro_value);
		if (copy_to_user(data, report_to_gyro_value, sizeof(report_to_gyro_value))) {
			E("%s: EWDAE_IOCTL_GET_AKM_DATA,"
				"copy_to_user fail\n", __func__);
			return -EFAULT;
		}
	break;
	case EWDAE_IOCTL_GET_AKM_READY:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		akm_ready = akm_get_akmd_ready();
		if (akm_ready == 0)
			I("%s: EWDAE_IOCTL_GET_AKM_READY,"
			"akm_ready= 0x%x\n",
			__func__, akm_ready);
		if (copy_to_user(data, &akm_ready, sizeof(akm_ready))) {
			E("%s: EWDAE_IOCTL_GET_AKM_READY,"
				"copy_to_user fail\n",
				__func__);
			return -EFAULT;
		}
	break;
	case EWDAE_IOCTL_GET_GYRO_CAL_DATA:
		data = (void __user *) arg;
		if (data == NULL) {
			retval = -EFAULT;
			break;
		}
	   if (gyro_gsensor_kvalue[0] != 0x67) {
		E("%s: EWDAE_IOCTL_GET_GYRO_CAL_DATA,"
		"gyro_gsensor_kvalue[0] 0x%x != 0x67\n",
			__func__, gyro_gsensor_kvalue[0]);
			return -EFAULT;
	   }
	   for (i = 0; i < sizeof(pana_gyro_gsensor_kvalue); i++) {
			pana_gyro_gsensor_kvalue[i] =
				gyro_gsensor_kvalue[i + 1];
			I("gyro_gsensor_kvalue[%d] = 0x%x\n",
				i + 1, gyro_gsensor_kvalue[i + 1]);
		}
	   if (copy_to_user(data, pana_gyro_gsensor_kvalue, sizeof(pana_gyro_gsensor_kvalue))) {
			E("%s: EWDAE_IOCTL_GET_GYRO_CAL_DATA,"
			"copy_to_user fail\n", __func__);
			return -EFAULT;
		}
	break;
	case EWDAE_IOCTL_WRITE_I2CDATA:
			data = (void __user *)arg;
			if (data == NULL)
				break;
			if (copy_from_user(i2cwrdata,
				data, sizeof(i2cwrdata))) {
				retval = -EFAULT;
				goto err_out;
			}
			/*write buf order is reg_addr,
			buf_len(unit:byte), data*/
			I("%s: EWDAE_IOCTL_WRITE_I2CDATA :"
			"i2caddr=0x%x,len=%d,data=0x%x",
			__func__, i2cwrdata[0],
			i2cwrdata[1], i2cwrdata[2]);
			retval = EWTZMU2_I2C_Write(i2cwrdata[0],
				i2cwrdata[1], &i2cwrdata[2]);
			if (retval != 1) {
				E("%s: EWDAE_IOCTL_WRITE_I2CDATA Error"
					": i2caddr=0x%x,len=%d,data=0x%x",
					__func__, i2cwrdata[0],
					i2cwrdata[1], i2cwrdata[2]);
			}
	break;

	case EWDAE_IOCTL_WRITE_I2CADDR:
			data = (void __user *)arg;
			if (data == NULL)
				break;
			if (copy_from_user(i2creaddata,
				data, sizeof(i2creaddata))) {
				retval = -EFAULT;
				goto err_out;
			}
			read_lock(&ewtzmu_data.lock);
			I("%s: EWDAE_IOCTL_WRITE_I2CADDR:"
			"i2creaddata[0]=%d,i2creaddata[1]=%d",
				__func__, i2creaddata[0],
				i2creaddata[1]);
			ewtzmu_data.i2c_read_addr = i2creaddata[0];
			ewtzmu_data.i2c_read_len = i2creaddata[1];
			read_unlock(&ewtzmu_data.lock);
	break;

	case EWDAE_IOCTL_READ_I2CDATA:
			data = (void __user *)arg;
			if (data == NULL)
				break;
			retval = EWTZMU2_I2C_Read(ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, &i2cwrdata[0]);
			I("%s: EWDAE_IOCTL_READ_I2CDATA :"
				"R addr=0x%x,len=%d,data=0x%x",
				__func__, ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, i2cwrdata[0]);
			if (retval) {/*successful*/
				if (copy_to_user(data, i2cwrdata,
					ewtzmu_data.i2c_read_len)) {
					retval = -EFAULT;
					goto err_out;
				}
			} else {
				E("%s: EWDAE_IOCTL_READ_I2CDATA :"
					"error R addr=0x%x,len=%d,data=0x%x",
				__func__, ewtzmu_data.i2c_read_addr,
				ewtzmu_data.i2c_read_len, i2cwrdata[0]);
				retval = -EFAULT;
				goto err_out;
			}
	break;
	default:
		E("%s not supported = 0x%04x", __func__, cmd);
		retval = -ENOIOCTLCMD;
	break;
	}

err_out:
	return retval;
}

unsigned int ewtzmu2daemon_poll(struct file *filp , poll_table *pwait)
{
	unsigned int mask = 0;

	if ((ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ORIENTATION) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GYROSCOPE) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_MAGNETIC_FIELD) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ACCELEROMETER) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ROTATION_VECTOR) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_LINEAR_ACCELERATION) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GRAVITY))
		mask |= POLLIN | POLLRDNORM;
    return mask;
}

static int ewtzmu2hal_open(struct inode *inode, struct file *file)
{

    I("Open device node:ewtzmu2hal times.\n");
    return 0;
}

static int ewtzmu2hal_release(struct inode *inode, struct file *file)
{

    I("Release ewtzmu2hal, remainder is  times.\n");
    return 0;
}

static long ewtzmu2hal_ioctl(/*struct inode *inode,*/struct file *file, unsigned int cmd, unsigned long arg)
{
	int controlbuf[EW_CB_LENGTH];
	char strbuf[EW_BUFSIZE];
	int pedoparam[EW_PD_LENGTH];
	 void __user *data;
	int retval = 0, ret = 0;

	DIF("ewtzmu2hal_ioctl, cmd is %d.\n", cmd);
	switch (cmd) {

	case EWHAL_IOCTL_GET_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadSensorData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_POSTURE:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadPostureData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_CALIDATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadCaliData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_GYRODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadGyroData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_PEDODATA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadPedoData(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_PEDOPARAM:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(pedoparam, &ewtzmumid_data.pedometerparam[0], sizeof(pedoparam));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_SET_PEDOPARAM:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
			retval = -EFAULT;
			goto err_out;
		}
		write_lock(&ewtzmumid_data.ctrllock);
		memcpy(&ewtzmumid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
		write_unlock(&ewtzmumid_data.ctrllock);
	break;

	case EWHAL_IOCTL_GET_CONTROL:
		read_lock(&ewtzmumid_data.ctrllock);
		memcpy(controlbuf, &ewtzmumid_data.controldata[0], sizeof(controlbuf));
		read_unlock(&ewtzmumid_data.ctrllock);
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_SET_CONTROL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
			retval = -EFAULT;
			goto err_out;
		}
	write_lock(&ewtzmumid_data.ctrllock);
	memcpy(&ewtzmumid_data.controldata[0], controlbuf, sizeof(controlbuf));
	write_unlock(&ewtzmumid_data.ctrllock);
		I("Gyro AP on: controldata_active_sensor=0x%x\n",
				ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS]);
	Set_Report_Sensor_Flag(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS]);
			I("Gyro AP on:off_status_hal=%d,"
			"g_status=%d, o_status=%d, m_status=%d,"
			"rv_status=%d, la_status=%d, gv_status=%d, \n",
				atomic_read(&off_status_hal),
				atomic_read(&g_status),
				atomic_read(&o_status),
				atomic_read(&m_status),
				atomic_read(&rv_status),
				atomic_read(&la_status),
				atomic_read(&gv_status));

			if (!atomic_read(&g_status) && !atomic_read(&rv_status) && !atomic_read(&la_status) && !atomic_read(&gv_status) && !atomic_read(&o_status)) {/*power off*/

				I("Gyro power off:g_status=%d"
					"off_status_hal=%d\n",
				atomic_read(&g_status),
				atomic_read(&off_status_hal));
				if (!atomic_read(&off_status_hal)) {
					atomic_set(&off_status_hal, 1);
					ret = EWTZMU2_Power_Off();
				}
			} else if ((atomic_read(&g_status) || atomic_read(&rv_status) || atomic_read(&la_status) || atomic_read(&gv_status) || atomic_read(&o_status))) {/*power on*/

				I("Gyro power on:g_status=%d"
					"off_status_hal=%d\n",
					atomic_read(&g_status),
					atomic_read(&off_status_hal));
				if (atomic_read(&off_status_hal)) {
					atomic_set(&off_status_hal, 0);
					ret = EWTZMU2_Power_On();
				}
			}/*after power on*/
			if (ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS]) {
				atomic_set(&open_flag, 1);
				wake_up(&open_wq);
			} else {
				atomic_set(&open_flag, 0);
			}
			return ret;
	break;

	case EWHAL_IOCTL_GET_WIA:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_WIA(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
	break;

	case EWHAL_IOCTL_GET_ROTATION_VECTOR:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadRotationVector(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
		break;

	case EWHAL_IOCTL_GET_LINEAR_ACCEL:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadLinearAccel(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			retval = -EFAULT;
			goto err_out;
		}
		break;

	case EWHAL_IOCTL_GET_GRAVITY:
		data = (void __user *) arg;
		if (data == NULL)
			break;
		EWTZMU2_ReadGravity(strbuf, EW_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
		retval = -EFAULT;
		goto err_out;
		}
	break;

	default:
		E("%s not supported = 0x%04x", __func__, cmd);
		retval = -ENOIOCTLCMD;
		break;
	}
	return 0;
err_out:
	return retval;
}

unsigned int ewtzmu2hal_poll(struct file *filp , poll_table *pwait)
{
	unsigned int mask = 0;

	if ((ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ORIENTATION) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GYROSCOPE) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_MAGNETIC_FIELD) ||
	(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ACCELEROMETER) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_ROTATION_VECTOR) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_LINEAR_ACCELERATION) ||
		(ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] & EW_BIT_GRAVITY))
			mask |= POLLIN | POLLRDNORM;

    return mask;
}

static struct file_operations ewtzmu2_fops = {
    .owner = THIS_MODULE,
    .open = ewtzmu2_open,
    .release = ewtzmu2_release,
    /*.ioctl = ewtzmu2_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = ewtzmu2_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = ewtzmu2_ioctl,
#endif
    .poll = ewtzmu2_poll,
};

static struct miscdevice ewtzmu2_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ewtzmu2",
    .fops = &ewtzmu2_fops,
};


static struct file_operations ewtzmu2daemon_fops = {
    .owner = THIS_MODULE,
    .open = ewtzmu2daemon_open,
    .release = ewtzmu2daemon_release,
    /*.ioctl = ewtzmu2daemon_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = ewtzmu2daemon_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = ewtzmu2daemon_ioctl,
#endif
    .poll = ewtzmu2daemon_poll,
};

static struct miscdevice ewtzmu2daemon_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ewtzmu2daemon",
    .fops = &ewtzmu2daemon_fops,
};

static struct file_operations ewtzmu2hal_fops = {
    .owner = THIS_MODULE,
    .open = ewtzmu2hal_open,
    .release = ewtzmu2hal_release,
    /*.ioctl = ewtzmu2hal_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = ewtzmu2hal_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = ewtzmu2hal_ioctl,
#endif
    .poll = ewtzmu2hal_poll,
};

static struct miscdevice ewtzmu2hal_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ewtzmu2hal",
    .fops = &ewtzmu2hal_fops,
};

static int ewtzmu2_input_init(struct ewtzmu_i2c_data *data)
{
    int err = 0;

    data->input_dev_compass = input_allocate_device();
    if (!data->input_dev_compass) {
	err = -ENOMEM;
	E("ewtzmu2_input_init: Failed to allocate input device\n");
	goto exit_input_dev_alloc_failed;
    }
    set_bit(EV_ABS, data->input_dev_compass->evbit);
    /* yaw */
    input_set_abs_params(data->input_dev_compass,
	ABS_RX, 0, (360*10), 0, 0);
    /* pitch */
    input_set_abs_params(data->input_dev_compass,
	ABS_RY, -(180*10), (180*10), 0, 0);
    /* roll */
    input_set_abs_params(data->input_dev_compass,
	ABS_RZ, -(90*10), (90*10), 0, 0);
    /* status of orientation sensor */
    input_set_abs_params(data->input_dev_compass,
	ABS_RUDDER, 0, 5, 0, 0);

    /* x-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_X,
	-(1000*2), (1000*2), 0, 0);

    /* y-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_Y,
	-(1000*2), (1000*2), 0, 0);

    /* z-axis of raw acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_Z,
	-(1000*2), (1000*2), 0, 0);


    /* x-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT0X,
	-(1000*3), (1000*3), 0, 0);
    /* y-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT0Y,
	-(1000*3), (1000*3), 0, 0);
    /* z-axis of raw magnetic vector and the range is -3g to +3g */
    input_set_abs_params(data->input_dev_compass, ABS_BRAKE,
	-(1000*3), (1000*3), 0, 0);
    /* status of magnetic sensor */
    input_set_abs_params(data->input_dev_compass, ABS_WHEEL,
	0, 5, 0, 0);
     /* x-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_HAT3X,
	-1000000, 1000000, 0, 0);
    /* y-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_HAT3Y,
	-1000000, 1000000, 0, 0);
    /* z-axis of rotation vector */
    input_set_abs_params(data->input_dev_compass, ABS_TILT_X,
	-1000000, 1000000, 0, 0);
	/* theta of rotation vector */
	input_set_abs_params(data->input_dev_compass, ABS_TILT_Y,
	-1000000, 1000000, 0, 0);

    /* x-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT1X,
	-(1000*2), (1000*2), 0, 0);
    /* y-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT1Y,
	-(1000*2), (1000*2), 0, 0);
    /* z-axis linear acceleration and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_TOOL_WIDTH,
	-(1000*2), (1000*2), 0, 0);

	/* x-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass, ABS_HAT2X,
	-(1000*2), (1000*2), 0, 0);
    /* y-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass,
	ABS_HAT2Y, -(1000*2), (1000*2), 0, 0);
    /* z-axis gravity and the range is -2g to +2g */
    input_set_abs_params(data->input_dev_compass,
	ABS_VOLUME, -(1000*2), (1000*2), 0, 0);

    data->input_dev_compass->name = "compass";

    err = input_register_device(data->input_dev_compass);
    if (err) {
	E("ewtzmu2_input_init: Unable to register input device\n");
	goto exit_input_register_compass_device_failed;
    }


    data->input_dev_gyroscope = input_allocate_device();
    if (!data->input_dev_gyroscope) {
	err = -ENOMEM;
	E("ewtzmu2_input_init: Failed to allocate input device: %s\n",
	data->input_dev_gyroscope->name);
	goto exit_input_register_compass_device_failed;
    }
    set_bit(EV_REL, data->input_dev_gyroscope->evbit);

    data->input_dev_gyroscope->relbit[0] = BIT(REL_RX) |
	BIT(REL_RY) | BIT(REL_RZ);

    data->input_dev_gyroscope->name = "ewtzmu2_gyroscope";

    err = input_register_device(data->input_dev_gyroscope);
    if (err) {
	E("ewtzmu2_input_init: Unable to register input device: %s\n",
	data->input_dev_gyroscope->name);
	goto exit_input_register_gyro_device_failed;
    }

    return 0;
exit_input_register_gyro_device_failed:
    input_free_device(data->input_dev_gyroscope);

exit_input_register_compass_device_failed:
    input_free_device(data->input_dev_compass);

exit_input_dev_alloc_failed:
    return err;
}

static void ewtzmu2_dir_polarity(struct ewtzmu_i2c_data *data)
{
	int i = 0;
	ewtzmumid_data.dirpolarity[0] = data->pdata->acc_dir;
	ewtzmumid_data.dirpolarity[1] = data->pdata->acc_polarity;
	ewtzmumid_data.dirpolarity[2] = data->pdata->gyro_dir;
	ewtzmumid_data.dirpolarity[3] = data->pdata->gyro_polarity;
	ewtzmumid_data.dirpolarity[4] = data->pdata->mag_dir;
	ewtzmumid_data.dirpolarity[5] = data->pdata->mag_polarity;
	for (i = 0; i < 6; i++) {
		I("$$$ewtzmu2_dir_polarity [%d]=%d \n",
			i, ewtzmumid_data.dirpolarity[i]);
	}
}

int pana_gyro_enable = 1;

static ssize_t pana_gyro_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "pana_gyro_enable = 0x%x\n", pana_gyro_enable);

	return s - buf;
}

static ssize_t pana_gyro_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	sscanf(buf, "%d", &pana_gyro_enable);
	if (pana_gyro_enable == 2) {
		ewtzmumid_data.controldata[EW_CB_ALGORITHMLOG] = 1;
		debug_flag = 1;
	} else if (pana_gyro_enable == 3) {
		ewtzmumid_data.controldata[EW_CB_ALGORITHMLOG] = 0;
		debug_flag = 0;
	}
	D("%s: pana_gyro_enable  = %d\n", __func__, pana_gyro_enable);

	return count;
}

static DEVICE_ACCESSORY_ATTR(pana_gyro, 0664, pana_gyro_show, pana_gyro_store);


int pana_gyro_registerAttr(void)
{
	int ret;

	ewtzmumid_data.htc_gyro_class = class_create(THIS_MODULE, "htc_gyro");
	if (IS_ERR(ewtzmumid_data.htc_gyro_class)) {
		ret = PTR_ERR(ewtzmumid_data.htc_gyro_class);
		ewtzmumid_data.htc_gyro_class = NULL;
		goto err_create_class;
	}

	ewtzmumid_data.gyro_dev = device_create(ewtzmumid_data.htc_gyro_class,
				NULL, 0, "%s", "gyro");
	if (unlikely(IS_ERR(ewtzmumid_data.gyro_dev))) {
		ret = PTR_ERR(ewtzmumid_data.gyro_dev);
		ewtzmumid_data.gyro_dev = NULL;
		goto err_create_gyro_device;
	}


	ret = device_create_file(ewtzmumid_data.gyro_dev, &dev_attr_pana_gyro);
	if (ret)
		goto err_create_gyro_device_file;
	return 0;

err_create_gyro_device_file:
	device_unregister(ewtzmumid_data.gyro_dev);
err_create_gyro_device:
	class_destroy(ewtzmumid_data.htc_gyro_class);
err_create_class:

	return ret;
}
static int __devinit ewtzmu2_i2c_probe(struct i2c_client *client,
const struct i2c_device_id *id)
{
    struct ewtzmu_i2c_data *data;
    int err = 0;

    I("\nEnter ewtzmu_i2c_probe!!\n");

	if (client->dev.platform_data == NULL) {
		E("platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit;
	}
	data = kmalloc(sizeof(struct ewtzmu_i2c_data), GFP_KERNEL);
    if (!(data)) {
	err = -ENOMEM;
	goto exit;
    }
    memset(data, 0, sizeof(struct ewtzmu_i2c_data));

    data->client = client;
	data->pdata = client->dev.platform_data;

    i2c_set_clientdata(client, data);
    ewtzmu_i2c_client = data->client;
     gpio_set_value(data->pdata->sleep_pin, 0);
	 ewtzmumid_data.sleep_pin = data->pdata->sleep_pin;
    err = EWTZMU2_Chipset_Init();
    if (err < 0) {
	E("PANA:GYRO:init err\n");
	err = -ENOMEM;
	goto exit_kfree;
    }

    I("Register input device!\n");
    err = ewtzmu2_input_init(data);
    if (err)
	goto exit_kfree;

	/*set sensor dir and polarity*/
     ewtzmu2_dir_polarity(data);
     err = pana_gyro_registerAttr();
     if (err) {
		E("%s: pana_gyro_registerAttr failed\n", __func__);
		goto exit_registerAttr_failed;
      }
    /*register misc device:ewtzmu2*/
    err = misc_register(&ewtzmu2_device);
    if (err) {
	E("ewtzmu2_device register failed\n");
	goto exit_misc_device_register_failed;
    }
    /*register misc device:ewtzmu2daemon*/
    err = misc_register(&ewtzmu2daemon_device);
    if (err) {
	E("ewtzmu2daemon_device register failed\n");
	goto exit_misc_device_register_failed;
    }
    /*register misc device:ewtzmu2hal*/
    err = misc_register(&ewtzmu2hal_device);
    if (err) {
	E("ewtzmu2hal_device register failed\n");
	goto exit_misc_device_register_failed;
    }

    /* Register sysfs hooks */
    err = sysfs_create_group(&client->dev.kobj, &ewtzmu2_attribute_group);
    if (err)
	goto exit_sysfs_create_group_failed;

	if (data->pdata->config_gyro_diag_gpios != NULL)
		ewtzmumid_data.config_gyro_diag_gpios =
		data->pdata->config_gyro_diag_gpios;
	atomic_set(&off_status_hal, 1);
	EWTZMU2_Power_Off();

	init_waitqueue_head(&open_wq);
     I("PANA:GYRO:probe success\n");

    return 0;
exit_sysfs_create_group_failed:
exit_misc_device_register_failed:
    input_free_device(data->input_dev_compass);
    input_free_device(data->input_dev_gyroscope);
exit_registerAttr_failed:
exit_kfree:
    kfree(data);
exit:
	E("PANA:GYRO:probe fail\n");
    return err;
}

static int __devexit ewtzmu2_i2c_remove(struct i2c_client *client)
{
    struct ewtzmu_i2c_data *data = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &ewtzmu2_attribute_group);
    input_unregister_device(data->input_dev_compass);
    input_unregister_device(data->input_dev_gyroscope);
    kfree(i2c_get_clientdata(client));
    ewtzmu_i2c_client = NULL;
    misc_deregister(&ewtzmu2hal_device);
    misc_deregister(&ewtzmu2daemon_device);
    misc_deregister(&ewtzmu2_device);
    return 0;
}

static int ewtzmu2_suspend(struct i2c_client *client, pm_message_t mesg)
{
    if (!atomic_read(&off_status)) {
		atomic_set(&off_status, 1);
		I("Gyro sys off on:g_status=%d off_status=%d\n",
			atomic_read(&g_status),
			atomic_read(&off_status));
		/*return EWTZMU2_Power_Off();*//*control by hal*/
	}
	I("GyroB sys off on:g_status=%d off_status=%d\n",
		atomic_read(&g_status),
		atomic_read(&off_status));
	return 0;
}

static int ewtzmu2_resume(struct i2c_client *client)
{
	if (atomic_read(&off_status)) {
		atomic_set(&off_status, 0);
		I("Gyro sys on on:g_status=%d off_status=%d\n",
			atomic_read(&g_status),
			atomic_read(&off_status));
		/*return EWTZMU2_Power_On();*//*control by hal*/
	}
	I("GyroB sys off on:g_status=%d off_status=%d\n",
		atomic_read(&g_status),
		atomic_read(&off_status));
	return 0;
}

struct i2c_device_id ewtzmu2_idtable[] = {
    { "ewtzmu2", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, ewtzmu2_idtable);

static struct i2c_driver ewtzmu2_i2c_driver = {
    .probe          = ewtzmu2_i2c_probe,
    .remove         = __devexit_p(ewtzmu2_i2c_remove),
    .id_table       = ewtzmu2_idtable,
    .driver = {
	.name   = EWTZMU_DRV_NAME,
    },
	.suspend		= ewtzmu2_suspend,
	.resume			= ewtzmu2_resume,
};

static int __init ewtzmu2_init(void)
{
    int ret;

    I("Panasonic Gyroscope sensor driver: init\n");
    I("ewtzmu2: driver version:%s\n", DRIVER_VERSION);
    rwlock_init(&ewtzmumid_data.ctrllock);
    rwlock_init(&ewtzmumid_data.datalock);
    rwlock_init(&ewtzmu_data.lock);
    memset(&ewtzmumid_data.controldata[0], 0, sizeof(int)*EW_CB_LENGTH);
    ewtzmumid_data.controldata[EW_CB_LOOPDELAY] = EW_DEFAULT_POLLING_TIME;
    ewtzmumid_data.controldata[EW_CB_RUN] =           1;
    ewtzmumid_data.controldata[EW_CB_ACCCALI] =       0;
    ewtzmumid_data.controldata[EW_CB_MAGCALI] =       1;
    ewtzmumid_data.controldata[EW_CB_ACTIVESENSORS] = 0;
    ewtzmumid_data.controldata[EW_CB_PD_RESET] =      0;
    ewtzmumid_data.controldata[EW_CB_PD_EN_PARAM] =   0;
	memset(&ewtzmumid_data.dirpolarity[0], 0, sizeof(int)*EW_DP_LENGTH);
    memset(&ewtzmumid_data.pedometerparam[0], 0, sizeof(int)*EW_PD_LENGTH);

    atomic_set(&dev_open_count, 0);
#ifndef HTC_VERSION
	atomic_set(&hal_open_count, 0);
#endif
	atomic_set(&daemon_open_count, 0);

	atomic_set(&o_status, 0);
	atomic_set(&a_status, 0);
	atomic_set(&m_status, 0);
	atomic_set(&g_status, 0);
	atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);
	atomic_set(&off_status, 0);

	ret = i2c_add_driver(&ewtzmu2_i2c_driver);
	if (ret != 0) {
		E("can not add i2c driver\n");
		return ret;
	}

	return ret;
}

static void __exit ewtzmu2_exit(void)
{
	atomic_set(&dev_open_count, 0);
#ifndef HTC_VERSION
	atomic_set(&hal_open_count, 0);
#endif
	atomic_set(&daemon_open_count, 0);

	atomic_set(&o_status, 0);
	atomic_set(&a_status, 0);
	atomic_set(&m_status, 0);
	atomic_set(&g_status, 0);
	atomic_set(&rv_status, 0);
	atomic_set(&la_status, 0);
	atomic_set(&gv_status, 0);
	atomic_set(&off_status, 0);

	i2c_del_driver(&ewtzmu2_i2c_driver);
}

MODULE_AUTHOR("Kyle K.Y. Chen");
MODULE_DESCRIPTION("Panasonic Gyroscope driver by Prolific");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(ewtzmu2_init);
module_exit(ewtzmu2_exit);

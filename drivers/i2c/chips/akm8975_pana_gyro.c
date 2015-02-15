/* drivers/i2c/chips/akm8975.c - akm8975 compass driver
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm8975.h>
#include<linux/earlysuspend.h>
#include <linux/module.h>

#define DEBUG 0
#define MAX_FAILURE_COUNT 3
/*#define AKM_EARLY_SUSPEND 1*/

#define D(x...) printk(KERN_DEBUG "[COMP][AKM8975] " x)
#define I(x...) printk(KERN_INFO "[COMP][AKM8975] " x)
#define E(x...) printk(KERN_ERR "[COMP][AKM8975 ERROR] " x)
#define DIF(x...) \
	{ if (debug_flag) \
		printk(KERN_DEBUG "[COMP][AKM8975 DEBUG] " x); }
#define DIF_FATAL_ERR(x...) \
	{ if (debug_flag_fatal_err) \
		printk(KERN_DEBUG "[COMP][AKM8975 DEBUG FATAL ERR] " x); }

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)

static struct i2c_client *this_client;

struct akm8975_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend_akm;
	struct class *htc_ecompass_class;
	struct device *ecompass_dev;
};

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[RBUFF_SIZE_8975 + 1];
static struct mutex sense_data_mutex;
#define AKM8975_RETRY_COUNT 10
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;
static int gsensor_first_enable;
static int failure_count;
static int m_f_times;
static short akmd_delay;

static int debug_flag;
static int debug_flag_fatal_err;
static int fatal_err_pr_count;

static atomic_t suspend_flag = ATOMIC_INIT(0);
static atomic_t PhoneOn_flag = ATOMIC_INIT(0);
static struct akm8975_platform_data *pdata;

static int disable_flag;
static int reserve_a_flag;

static int AKI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	/*D("%s:\n", __func__);*/

	for (loop_i = 0; loop_i < AKM8975_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	/*D("%s msgs[1]: addr = 0x%x, flags = 0x%x, "
		"len = 0x%x,  buf(0, 1, 2, 3, 4, 5, 6, 7) = (0x%x, 0x%x"
		", 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
		__func__, msgs[1].addr, msgs[1].flags,
		msgs[1].len, msgs[1].buf[0], msgs[1].buf[1],
		msgs[1].buf[2], msgs[1].buf[3], msgs[1].buf[4], msgs[1].buf[5]
		, msgs[1].buf[6], msgs[1].buf[7]);*/

	if (loop_i >= AKM8975_RETRY_COUNT) {
		E("%s retry over %d\n",
			__func__, AKM8975_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < AKM8975_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	/*D("%s msg: addr = 0x%x, flags = 0x%x, "
		"len = 0x%x,  buf(0, 1, 2, 3, 4) = (0x%x, 0x%x"
		", 0x%x, 0x%x, 0x%x)\n",
		__func__, msg[0].addr, msg[0].flags,
		msg[0].len, msg[0].buf[0], msg[0].buf[1],
		msg[0].buf[2], msg[0].buf[3], msg[0].buf[4]);*/

	if (loop_i >= AKM8975_RETRY_COUNT) {
		E("%s retry over %d\n",
			__func__, AKM8975_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}

static int AKECS_StartMeasure(void)
{
	char buffer[2];
	atomic_set(&data_ready, 0);

	/* Set measure mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_CNTL_SNG_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_PowerDown(void)
{
	char buffer[2];
	int ret;

	/*D("%s:\n", __func__);*/

	/* Set powerdown mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_CNTL_POWER_DOWN;
	/* Set data */
	ret = AKI2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	/* Dummy read for clearing INT pin */
	buffer[0] = AK8975_REG_ST1;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;
	return ret;
}

static int AKECS_StartFuseRead(void)
{
	char buffer[2];

	/*D("%s:\n", __func__);*/

	/* Set measure mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_CNTL_FUSE_ACCESS;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_GetData(void)
{
	char buffer[RBUFF_SIZE_8975 + 1];
	int ret;

	memset(buffer, 0, RBUFF_SIZE_8975);
	buffer[0] = AK8975_REG_ST1;
	ret = AKI2C_RxData(buffer, RBUFF_SIZE_8975);
	if (ret < 0)
		return ret;

	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, buffer, sizeof(buffer));
	atomic_set(&data_ready, 1);
	wake_up(&data_ready_wq);
	mutex_unlock(&sense_data_mutex);

	DIF("%s: GET_DATA, sense_data(0, 1, 2, 3, 4, 5, 6, 7) = "
		"(0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
		__func__, sense_data[0], sense_data[1], sense_data[2]
		, sense_data[3], sense_data[4], sense_data[5], sense_data[6]
		, sense_data[7]);

	DIF_FATAL_ERR("%s: GET_DATA, sense_data(0, 1, 2, 3, 4,"
		" 5, 6, 7) = (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, "
		"0x%x, 0x%x, 0x%x\n",
		__func__, sense_data[0], sense_data[1],
		sense_data[2], sense_data[3], sense_data[4],
		sense_data[5], sense_data[6], sense_data[7]);

	return 0;
}

static int AKECS_SetMode(char mode)
{
	int ret;

	switch (mode) {
	case AK8975_CNTL_SNG_MEASURE:
		ret = AKECS_StartMeasure();
		break;
	case AK8975_CNTL_FUSE_ACCESS:
		ret = AKECS_StartFuseRead();
		break;
	case AK8975_CNTL_POWER_DOWN:
		ret = AKECS_PowerDown();
		break;
	default:
		return -EINVAL;
	}

	/* wait at least 300us after changing mode */
	mdelay(1);
	return ret;
}

static int AKECS_TransRBuff(char *rbuf, int size)
{
	wait_event_interruptible_timeout(data_ready_wq,
					 atomic_read(&data_ready), 1000);

	if (!atomic_read(&data_ready)) {
		if (!atomic_read(&suspend_flag)) {
			D("%s: DATA\n", __func__);
			failure_count++;
			if (failure_count >= MAX_FAILURE_COUNT) {
				E("%s: successive %d failure.\n",
				       __func__, failure_count);
				debug_flag_fatal_err = 1;
				atomic_set(&open_flag, -1);
				wake_up(&open_wq);
				failure_count = 0;
			}
		}
		return -1;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(&rbuf[0], &sense_data[0], size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);

	failure_count = 0;
	return 0;
}


static void AKECS_Report_Value(short *rbuf)
{
	struct akm8975_data *data = i2c_get_clientdata(this_client);
#if DEBUG
	D("AKECS_Report_Value: yaw = %d, pitch = %d, roll = %d"
		"\n", rbuf[0], rbuf[1], rbuf[2]);
	D("                    tmp = %d, m_stat= %d, g_stat=%d"
		"\n", rbuf[3], rbuf[4], rbuf[5]);
	D("          G_Sensor:   x = %d LSB, y = %d LSB, z = "
		"%d LSB\n", rbuf[6], rbuf[7], rbuf[8]);
	D("          Compass:   x = %d LSB, y = %d LSB, z = %d"
		" LSB\n", rbuf[9], rbuf[10], rbuf[11]);
#endif
	DIF(
		"AKECS_Report_Value: yaw = %d, pitch = %d, roll = %d"
		"\n", rbuf[0], rbuf[1], rbuf[2]);
	DIF(
	"          G_Sensor:   x = %d LSB, y = %d LSB, z = "
		"%d LSB\n", rbuf[6], rbuf[7], rbuf[8]);
	DIF(
	"          Compass:   x = %d LSB, y = %d LSB, z = %d"
		" LSB\n", rbuf[9], rbuf[10], rbuf[11]);

	DIF("(m, a, t, mv) = (0x%x, 0x%x, 0x%x, 0x%x)\n",
		atomic_read(&m_flag), atomic_read(&a_flag),
		atomic_read(&t_flag), atomic_read(&mv_flag));
	if (pdata->use_pana_gyro == 0) {
		if (fatal_err_pr_count < 10) {
			DIF_FATAL_ERR(
				"AKECS_Report_Value: yaw = %d, pitch = %d,"
				" roll = %d\n", rbuf[0], rbuf[1], rbuf[2]);
			DIF_FATAL_ERR(
			"          G_Sensor:   x = %d LSB, y = %d LSB, z = "
				"%d LSB\n", rbuf[6], rbuf[7], rbuf[8]);
			DIF_FATAL_ERR(
			"          Compass:   x = %d LSB, y = %d LSB, z = %d"
				" LSB\n", rbuf[9], rbuf[10], rbuf[11]);

			DIF_FATAL_ERR("(m, a, t, mv) = (0x%x, 0x%x, 0x%x,"
				" 0x%x)\n",
				atomic_read(&m_flag), atomic_read(&a_flag),
				atomic_read(&t_flag), atomic_read(&mv_flag));

			fatal_err_pr_count++;
		} else {
			fatal_err_pr_count = 0;
			debug_flag_fatal_err = 0;
		}

		/* Report magnetic sensor information */
		if (atomic_read(&m_flag)) {
			input_report_abs(data->input_dev, ABS_RX, rbuf[0]);
			input_report_abs(data->input_dev, ABS_RY, rbuf[1]);
			input_report_abs(data->input_dev, ABS_RZ, rbuf[2]);
			input_report_abs(data->input_dev, ABS_RUDDER, rbuf[4]);
		}

		/* Report acceleration sensor information */
		if (atomic_read(&a_flag)) {
			input_report_abs(data->input_dev, ABS_X, rbuf[6]);
			input_report_abs(data->input_dev, ABS_Y, rbuf[7]);
			input_report_abs(data->input_dev, ABS_Z, rbuf[8]);
			input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]);
		}

		/* Report temperature information */
		if (atomic_read(&t_flag))
			input_report_abs(data->input_dev, ABS_THROTTLE, rbuf[3]);

		if (atomic_read(&mv_flag)) {
			input_report_abs(data->input_dev, ABS_HAT0X, rbuf[9]);
			input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[10]);
			input_report_abs(data->input_dev, ABS_BRAKE, rbuf[11]);
		}

		input_sync(data->input_dev);
	} else if (atomic_read(&a_flag) ) {
		EWTZMU2_Report_Value_akm(gsensor_first_enable, rbuf[6], rbuf[7], rbuf[8]);
		if (gsensor_first_enable == 1)
			gsensor_first_enable = 0;
	}
}

static int AKECS_GetOpenStatus(void)
{
	D("%s:\n", __func__);
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	/*D("%s: After wait_event_interruptible\n", __func__);*/
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	D("%s:\n", __func__);
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
	I("%s:\n", __func__);
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&t_flag, 0);
	atomic_set(&mv_flag, 0);
}

static int akm_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	struct akm8975_data *data = i2c_get_clientdata(this_client);

	printk(KERN_INFO "[COMP] Compass enable\n");

	DIF("%s: open_count = %d, open_flag = %d\n", __func__,
		atomic_read(&open_count), atomic_read(&open_flag));

	DIF_FATAL_ERR("%s: open_count = %d, open_flag = %d\n", __func__,
		atomic_read(&open_count), atomic_read(&open_flag));

	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		atomic_set(&open_flag, 1);
		if (pdata->use_pana_gyro == 0)
			input_report_abs(data->input_dev, ABS_RUDDER, -1);
		atomic_set(&reserve_open_flag, 1);
		wake_up(&open_wq);
		ret = 0;
	}
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
	I("[COMP] Compass disable\n");

	debug_flag_fatal_err = 0;
	fatal_err_pr_count = 0;

	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static long
akm_aot_ioctl(/*struct inode *inode,*/ struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag = 0;
	int ret = -1;

	ret = copy_from_user(&flag, argp, sizeof(flag));
	if (ret)
		return -EFAULT;
	DIF("%s: cmd = 0x%x, flag = %d\n", __func__, cmd, flag);

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		if (flag == 0) {
			m_f_times = 0;
			I("ECS_IOCTL_APP_SET_MFLAG,(m, a, t, mv) = (0x%x, 0x%x, 0x%x, 0x%x), m_f_times %d\n",
				atomic_read(&m_flag), atomic_read(&a_flag),
				atomic_read(&t_flag), atomic_read(&mv_flag), m_f_times);
		}
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		reserve_a_flag = flag;
		if (disable_flag != 1) {
			gsensor_first_enable = 1;
			atomic_set(&a_flag, flag);
		} else {
			gsensor_first_enable = 0;
			atomic_set(&a_flag, 0);
		}
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		akmd_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = akmd_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int akmd_open(struct inode *inode, struct file *file)
{
	I("%s:\n", __func__);
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	I("%s:\n", __func__);
	AKECS_CloseDone();
	return 0;
}
static short report_to_gyro_value[12] = {0};

void akm_get_akmd_data(short *getdata)
{
	int i;

	for (i = 0; i < 12; i++)
		*(getdata+i) = report_to_gyro_value[i];
}

int  akm_get_akmd_ready(void)
{
	return m_f_times;
}
static long
akmd_ioctl(/*struct inode *inode,*/ struct file *file, unsigned int cmd,
	   unsigned long arg)
{

	void __user *argp = (void __user *)arg;

	char msg[RBUFF_SIZE_8975 + 1] = "", rwbuf[RBUFF_SIZE_8975 + 1] = "";
	int ret = -1, status;
	short mode, value[12], delay;
	short layouts[4][3][3];
	int i, j, k;

	DIF("%s: cmd = 0x%x\n", __func__, cmd);
	/*D("%s: cmd = 0x%x\n", __func__, cmd);*/
	/*D("%s: ECS_IOCTL_GETDATA = 0x%x\n",
		__func__, ECS_IOCTL_GETDATA);*/

	switch (cmd) {
	case ECS_IOCTL_WRITE:
	case ECS_IOCTL_READ:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_MODE:
		/*D("%s: ECS_IOCTL_SET_MODE\n",
			__func__);*/
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		/*D("%s: ECS_IOCTL_SET_MODE success!\n",
			__func__);*/
		break;
	case ECS_IOCTL_SET_YPR:
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		/*D("%s: ECS_IOCTL_SET_MODE2\n",
			__func__);*/
		ret = AKECS_SetMode((char)mode);
		if (ret < 0)
			return ret;
		/*D("%s: ECS_IOCTL_SET_MODE2 success\n",
			__func__);*/
		break;
	case ECS_IOCTL_GETDATA:
		/*D("%s: ECS_IOCTL_GETDATA\n", __func__);*/
		DIF_FATAL_ERR("%s: calling AKECS_TransRBuff\n", __func__);
		ret = AKECS_TransRBuff(msg, RBUFF_SIZE_8975);
		if (ret < 0)
			return ret;
		/*D("%s: ECS_IOCTL_GETDATA: success\n",
			__func__);*/
		break;
	case ECS_IOCTL_SET_YPR:
		for (i = 0; i < 12; i++)
			report_to_gyro_value[i] = value[i];
		if (atomic_read(&m_flag) && m_f_times == 0) {
			m_f_times = 1;
			I("(m, a, t, mv) = (0x%x, 0x%x, 0x%x, 0x%x), set m_f_times %d\n",
				atomic_read(&m_flag), atomic_read(&a_flag),
				atomic_read(&t_flag), atomic_read(&mv_flag), m_f_times);
		}
		if (atomic_read(&m_flag) == 0) {
			if (m_f_times == 1)
				I("(m, a, t, mv) = (0x%x, 0x%x, 0x%x, 0x%x),  set 0 to m_f_times %d\n",
				atomic_read(&m_flag), atomic_read(&a_flag),
				atomic_read(&t_flag), atomic_read(&mv_flag), m_f_times);
			m_f_times = 0;
		}

		AKECS_Report_Value(value);
		break;
	case ECS_IOCTL_GET_COMP_FLAG:
		status = atomic_read(&m_flag);
		status |= atomic_read(&mv_flag);
		DIF("%s: ECS_IOCTL_GET_COMP_FLAG, status = %d\n",
			__func__, status);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		status = AKECS_GetOpenStatus();
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = AKECS_GetCloseStatus();
		break;
	case ECS_IOCTL_GET_DELAY:
		delay = akmd_delay;
		break;
	case ECS_IOCTL_GET_MATRIX:
		for (i = 0; i < 4; i++)
			for (j = 0; j < 3; j++)
				for (k = 0; k < 3; k++) {
					/*D("%s: "
						"pdata->layouts[%d][%d][%d]"
						" = %d\n",
				__func__, i, j, k, pdata->layouts[i][j][k]);*/
				layouts[i][j][k] = pdata->layouts[i][j][k];
				}
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GETDATA:
		msg[8] = debug_flag;/*
		DIF("msg(0, 1, 2, 3, 4, 5, 6, 7, 8) = (0x%x, 0x%x, 0x%x, 0x%x,"
		  " 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
			msg[0], msg[1],	msg[2], msg[3], msg[4], msg[5], msg[6],
			msg[7],	msg[8]);*/
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_COMP_FLAG:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_MATRIX:
		if (copy_to_user(argp, layouts, sizeof(layouts)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DATA_FOR_GYRO:
		I("%s: ECS_IOCTL_GET_DATA_FOR_GYRO = 0x%x\n", __func__, ECS_IOCTL_GET_DATA_FOR_GYRO);
		if (copy_to_user(argp, report_to_gyro_value, sizeof(report_to_gyro_value)))
			return -EFAULT;
		break;
	default:
		break;
	}
	return 0;
}

static void akm_work_func(struct work_struct *work)
{
	if (AKECS_GetData() < 0)
		E("%s: Get data failed\n", __func__);
	enable_irq(this_client->irq);
}

static irqreturn_t akm8975_interrupt(int irq, void *dev_id)
{
	struct akm8975_data *data = dev_id;

	DIF_FATAL_ERR("%s\n", __func__);

	disable_irq_nosync(this_client->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

#ifdef AKM_EARLY_SUSPEND
static void akm8975_early_suspend(struct early_suspend *handler)
{
	DIF("%s", __func__);

	if (!atomic_read(&PhoneOn_flag)) {
		atomic_set(&suspend_flag, 1);
		atomic_set(&reserve_open_flag, atomic_read(&open_flag));
		atomic_set(&open_flag, 0);
		wake_up(&open_wq);
		disable_irq(this_client->irq);
	} else
		D("AKM8975 akm8975_early_suspend: PhoneOn_flag is set\n");
}

static void akm8975_early_resume(struct early_suspend *handler)
{
	DIF("%s", __func__);

	if (atomic_read(&suspend_flag)) {
		enable_irq(this_client->irq);
		atomic_set(&suspend_flag, 0);
		atomic_set(&open_flag, atomic_read(&reserve_open_flag));
		wake_up(&open_wq);
	} else
		D("AKM8975 akm8975_early_resume: PhoneOn_flag is set\n");
}

#else /* AKM_EARLY_SUSPEND */

static int akm8975_suspend(struct i2c_client *client, pm_message_t mesg)
{
	DIF("%s", __func__);

	DIF_FATAL_ERR("%s", __func__);

	atomic_set(&suspend_flag, 1);
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
	disable_irq(this_client->irq);

	return 0;
}

static int akm8975_resume(struct i2c_client *client)
{
	enable_irq(this_client->irq);
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);

	D("%s: (m, a, t, mv) = (0x%x, 0x%x, 0x%x, 0x%x)\n",
		__func__, atomic_read(&m_flag), atomic_read(&a_flag),
		atomic_read(&t_flag), atomic_read(&mv_flag));

	return 0;
}
#endif /* AKM_EARLY_SUSPEND */

static const struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	/*.ioctl = akmd_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = akmd_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = akmd_ioctl,
#endif
};

static const struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	/*.ioctl = akm_aot_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = akm_aot_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = akm_aot_ioctl,
#endif
};


static struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8975_aot",
	.fops = &akm_aot_fops,
};


static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8975_daemon",
	.fops = &akmd_fops,
};

static ssize_t akm_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", atomic_read(&PhoneOn_flag));
	return (s - buf);
}

static ssize_t akm_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		atomic_set(&PhoneOn_flag, 1);
		D("AKM8975 akm_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		atomic_set(&PhoneOn_flag, 0);
		D("AKM8975 akm_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	E("akm_store: invalid argument\n");
	return -EINVAL;
}

static DEVICE_ACCESSORY_ATTR(PhoneOnOffFlag, 0664, \
	akm_show, akm_store);

static ssize_t debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	short lm_flag = -1;
	short la_flag = -1;
	short lt_flag = -1;
	short lmv_flag = -1;
	short ldelay_flag = -1;

	lm_flag = atomic_read(&m_flag);
	la_flag = atomic_read(&a_flag);
	lt_flag = atomic_read(&t_flag);
	lmv_flag = atomic_read(&mv_flag);
	ldelay_flag = akmd_delay;

	s += sprintf(s, "(m, a, t, mv, delay, debug_flag, "
		"debug_flag_fatal_err) = (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x,"
		" 0x%x)\n", lm_flag, la_flag, lt_flag, lmv_flag, ldelay_flag,
		debug_flag, debug_flag_fatal_err);

	return s - buf;
}

static ssize_t debug_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	debug_flag = -1;
	sscanf(buf, "%d", &debug_flag);

	D("%s: debug_flag = %d\n", __func__, debug_flag);

	return count;
}

static DEVICE_ACCESSORY_ATTR(debug_en, 0664, \
	debug_show, debug_store);

static ssize_t disable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "disable_flag = 0x%x\n", disable_flag);

	return s - buf;
}

static ssize_t disable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	disable_flag = -1;
	sscanf(buf, "%d", &disable_flag);

	D("%s: disable_flag = %d\n", __func__, disable_flag);

	if (disable_flag == 1)
		atomic_set(&a_flag, 0);
	else
		atomic_set(&a_flag, reserve_a_flag);

	return count;
}

static DEVICE_ACCESSORY_ATTR(disable_en, 0664, \
	disable_show, disable_store);


int akm8975_registerAttr_pana(struct akm8975_data *akm)
{
	int ret;

	akm->htc_ecompass_class = class_create(THIS_MODULE, "htc_ecompass");
	if (IS_ERR(akm->htc_ecompass_class)) {
		ret = PTR_ERR(akm->htc_ecompass_class);
		akm->htc_ecompass_class = NULL;
		goto err_create_class;
	}

	akm->ecompass_dev = device_create(akm->htc_ecompass_class,
				NULL, 0, "%s", "ecompass");
	if (unlikely(IS_ERR(akm->ecompass_dev))) {
		ret = PTR_ERR(akm->ecompass_dev);
		akm->ecompass_dev = NULL;
		goto err_create_ecompass_device;
	}

	/* register the attributes */
	ret = device_create_file(akm->ecompass_dev, &dev_attr_PhoneOnOffFlag);
	if (ret)
		goto err_create_ecompass_device_file;

	/* register the attributes */
	ret = device_create_file(akm->ecompass_dev, &dev_attr_debug_en);
	if (ret)
		goto err_create_ecompass_debug_device_file;

	/* register the attributes */
	ret = device_create_file(akm->ecompass_dev, &dev_attr_disable_en);
	if (ret)
		goto err_create_ecompass_disable_device_file;

	return 0;

err_create_ecompass_disable_device_file:
err_create_ecompass_debug_device_file:
err_create_ecompass_device_file:
	device_unregister(akm->ecompass_dev);
err_create_ecompass_device:
	class_destroy(akm->htc_ecompass_class);
err_create_class:

	return ret;
}


int akm8975_probe_pana(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm8975_data *akm;
	int err = 0;
	char msg[RBUFF_SIZE_8975 + 1];
	unsigned int irq_type;

	/*int i = 0;*/

	memset(msg, 0, RBUFF_SIZE_8975 + 1);

	I("%s:\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	akm = kzalloc(sizeof(struct akm8975_data), GFP_KERNEL);
	if (!akm) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	INIT_WORK(&akm->work, akm_work_func);
	i2c_set_clientdata(client, akm);

	mutex_init(&sense_data_mutex);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		E("%s: platform data is NULL\n", __func__);
		goto exit_platform_data_null;
	}
	this_client = client;

	err = AKECS_PowerDown();
	if (err < 0) {
		E("%s: set power down mode error\n", __func__);
		goto exit_set_mode_failed;
	}
	if (pdata->use_pana_gyro == 0) {
		akm->input_dev = input_allocate_device();

		if (!akm->input_dev) {
			err = -ENOMEM;
			E("%s: Failed to allocate input device\n", __func__);
			goto exit_input_dev_alloc_failed;
		}

		set_bit(EV_ABS, akm->input_dev->evbit);
		/* yaw */
		input_set_abs_params(akm->input_dev, ABS_RX, 0, 360, 0, 0);
		/* pitch */
		input_set_abs_params(akm->input_dev, ABS_RY, -180, 180, 0, 0);
		/* roll */
		input_set_abs_params(akm->input_dev, ABS_RZ, -90, 90, 0, 0);
		/* x-axis acceleration */
		input_set_abs_params(akm->input_dev, ABS_X, -1872, 1872, 0, 0);
		/* y-axis acceleration */
		input_set_abs_params(akm->input_dev, ABS_Y, -1872, 1872, 0, 0);
		/* z-axis acceleration */
		input_set_abs_params(akm->input_dev, ABS_Z, -1872, 1872, 0, 0);
		/* temparature */
		input_set_abs_params(akm->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
		/* status of magnetic sensor */
		input_set_abs_params(akm->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
		/* status of acceleration sensor */
		input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
		/* step count */
		input_set_abs_params(akm->input_dev, ABS_GAS, 0, 65535, 0, 0);
		/* x-axis of raw magnetic vector */
		input_set_abs_params(akm->input_dev, ABS_HAT0X, -2048, 2032, 0, 0);
		/* y-axis of raw magnetic vector */
		input_set_abs_params(akm->input_dev, ABS_HAT0Y, -2048, 2032, 0, 0);
		/* z-axis of raw magnetic vector */
		input_set_abs_params(akm->input_dev, ABS_BRAKE, -2048, 2032, 0, 0);

		akm->input_dev->name = "compass";

		err = input_register_device(akm->input_dev);

		if (err) {
			E("%s: Unable to register input device: %s\n", __func__,
				akm->input_dev->name);
			goto exit_input_register_device_failed;
		}
	}
	err = misc_register(&akmd_device);
	if (err) {
		E("%s: akmd_device register failed\n", __func__);
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&akm_aot_device);
	if (err) {
		E("%s: akm_aot_device register failed\n", __func__);
		goto exit_misc_device_register_failed;
	}

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&t_flag, 0);
	atomic_set(&mv_flag, 0);

	debug_flag = 0;
	debug_flag_fatal_err = 0;
	fatal_err_pr_count = 0;

#ifdef AKM_EARLY_SUSPEND
	akm->early_suspend_akm.suspend = akm8975_early_suspend;
	akm->early_suspend_akm.resume = akm8975_early_resume;
	register_early_suspend(&akm->early_suspend_akm);
#endif
	err = akm8975_registerAttr_pana(akm);
	if (err) {
		E("%s: akm8975_registerAttr_pana failed\n", __func__);
		goto exit_registerAttr_failed;
	}

	disable_flag = 0;
	reserve_a_flag = 0;

	irq_type = (pdata->irq_trigger) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;

	err = request_irq(client->irq, akm8975_interrupt, irq_type,
			  "akm8975", akm);
	if (err < 0) {
		E("%s: request irq failed\n", __func__);
		goto exit_irq_request_failed;
	}
	I("%s:OK\n", __func__);
	return 0;

exit_irq_request_failed:
exit_registerAttr_failed:
exit_misc_device_register_failed:
exit_input_register_device_failed:
	if (pdata->use_pana_gyro == 0)
		input_free_device(akm->input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, akm);
exit_set_mode_failed:
exit_platform_data_null:
	kfree(akm);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;

}

static int akm8975_remove(struct i2c_client *client)
{
	struct akm8975_data *akm = i2c_get_clientdata(client);
	free_irq(client->irq, akm);
	if (pdata->use_pana_gyro == 0)
		input_unregister_device(akm->input_dev);
	kfree(akm);
	return 0;
}
static const struct i2c_device_id akm8975_id[] = {
	{ AKM8975_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver akm8975_driver = {
	.probe 	= akm8975_probe_pana,
	.remove 	= akm8975_remove,
	.id_table	= akm8975_id,

#ifndef AKM_EARLY_SUSPEND
	.suspend = akm8975_suspend,
	.resume = akm8975_resume,
#endif
	.driver = {
		   .name = AKM8975_I2C_NAME,
		   },
};

static int __init akm8975_init(void)
{
	I("AKM8975 compass driver for pana gyro: init\n");
	return i2c_add_driver(&akm8975_driver);
}

static void __exit akm8975_exit(void)
{
	i2c_del_driver(&akm8975_driver);
}

module_init(akm8975_init);
module_exit(akm8975_exit);

MODULE_DESCRIPTION("AKM8975 compass driver");
MODULE_LICENSE("GPL");

/*
 * Last modified: Sep 20th, 2012
 * Revision: V1.4.2
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2012 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_BMG_USE_PLATFORM_DATA
#include <linux/bst_sensor_common.h>
#endif

/*#include "bmg160.h"*/
/*#include "bs_log.h"*/
#include <linux/bmg160.h>
#include <linux/bs_log.h>

#define D(x...) printk(KERN_DEBUG "[GYRO][BMG160_BOSCH] " x)
#define I(x...) printk(KERN_INFO "[GYRO][BMG160_BOSCH] " x)
#define E(x...) printk(KERN_ERR "[GYRO][BMG160_BOSCH] " x)

/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)

#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name

#define BMG_I2C_WRITE_DELAY_TIME 1

/* generic */
#define BMG_MAX_RETRY_I2C_XFER (100)
#define BMG_MAX_RETRY_WAKEUP (5)
#define BMG_MAX_RETRY_WAIT_DRDY (100)

#define BMG_DELAY_MIN (1)
#define BMG_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMG_SELF_TEST 0

#ifdef BMG_USE_FIFO
#define MAX_FIFO_F_LEVEL 100
#define MAX_FIFO_F_BYTES 8
#endif

#define HTC_CALIBRATION 1

#ifdef HTC_CALIBRATION
#define TOLERENCE		1071
#endif

struct op_mode_map {
	char *op_mode_name;
	long op_mode;
};

static const struct op_mode_map op_mode_maps[] = {
	{"normal", BMG_VAL_NAME(MODE_NORMAL)},
	{"deepsuspend", BMG_VAL_NAME(MODE_DEEPSUSPEND)},
	{"suspend", BMG_VAL_NAME(MODE_SUSPEND)},
	{"fastpowerup", BMG_VAL_NAME(MODE_FASTPOWERUP)},
	{"advancedpowersav", BMG_VAL_NAME(MODE_ADVANCEDPOWERSAVING)},
};

struct bmg_client_data {
	struct bmg160_t device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;

	struct bmg160_data_t value;
	u8 enable:1;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_value;

#ifdef CONFIG_BMG_USE_FIXED_SYSFS_PATH
	struct class    *cls;
	struct device   *dev_class;
#endif

#ifdef CONFIG_BMG_USE_PLATFORM_DATA
	struct bosch_sensor_specific *bst_pd;
#endif

#ifdef HTC_CALIBRATION
	int cali_data_x;
	int cali_data_y;
	int cali_data_z;
#endif
};

static struct i2c_client *bmg_client;
/* i2c operation for API */
static void bmg_i2c_delay(BMG160_S32 msec);
static char bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static char bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmg_dump_reg(struct i2c_client *client);
static int bmg_check_chip_id(struct i2c_client *client);

static int bmg_pre_suspend(struct i2c_client *client);
static int bmg_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler);
static void bmg_late_resume(struct early_suspend *handler);
#endif

static void bmg_remap_sensor_data(struct bmg160_data_t *val,
		struct bmg_client_data *client_data)
{
#ifdef CONFIG_BMG_USE_PLATFORM_DATA
	struct bosch_sensor_data bsd;

	if (NULL == client_data->bst_pd)
		return;

	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;

	bst_remap_sensor_data_dft_tab(&bsd,
			client_data->bst_pd->place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;
#else
	(void)val;
	(void)client_data;
#endif
}


static int bmg_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
	PINFO("read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG)
		err = -1;

	return err;
}

static void bmg_i2c_delay(BMG160_S32 msec)
{
	mdelay(msec);
}

static void bmg_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	PDEBUG("%s\n", dbg_buf_str);

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	PDEBUG("%s\n", dbg_buf_str);
}

/*	i2c read routine for API*/
static char bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			PERR("i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -1;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

#ifdef BMG_USE_FIFO
static char bmg_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif

/*	i2c write routine for */
static char bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMG_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			PERR("error writing i2c bus");
			return -1;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(BMG_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMG_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static char bmg_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = bmg_i2c_read(bmg_client, reg_addr, data, len);
	return err;
}

static char bmg_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = bmg_i2c_write(bmg_client, reg_addr, data, len);
	return err;
}


static void bmg_work_func(struct work_struct *work)
{
	struct bmg_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmg_client_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));

	mutex_lock(&client_data->mutex_value);

	BMG_CALL_API(get_dataXYZ)(&client_data->value);
	bmg_remap_sensor_data(&client_data->value, client_data);

	input_report_abs(client_data->input, ABS_X, client_data->value.datax);
	input_report_abs(client_data->input, ABS_Y, client_data->value.datay);
	input_report_abs(client_data->input, ABS_Z, client_data->value.dataz);
	mutex_unlock(&client_data->mutex_value);

	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}

static ssize_t bmg_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMG);
}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = strict_strtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	BMG_CALL_API(get_dataXYZ)(&client_data->value);
	bmg_remap_sensor_data(&client_data->value, client_data);

#ifdef HTC_CALIBRATION
	client_data->value.datax = client_data->value.datax +
					client_data->cali_data_x;
	client_data->value.datay = client_data->value.datay +
					client_data->cali_data_y;
	client_data->value.dataz = client_data->value.dataz +
					client_data->cali_data_z;
#endif

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return count;
}

#ifdef HTC_CALIBRATION
static ssize_t bmg_show_raw_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	BMG_CALL_API(get_dataXYZ)(&client_data->value);
	bmg_remap_sensor_data(&client_data->value, client_data);

	count = sprintf(buf, "x = %hd, y = %hd, z = %hd",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return count;
}


static ssize_t attr_cali_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	s += sprintf(s, "Stored calibration data (x, y, z) = (%d, %d, %d)\n",
		client_data->cali_data_x, client_data->cali_data_y,
		client_data->cali_data_z);

	D("%s: Calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, client_data->cali_data_x, client_data->cali_data_y,
			  client_data->cali_data_z);
	return s - buf;
}

static int is_valid_cali(int cali_data)
{
	if ((cali_data < TOLERENCE) && (cali_data > -TOLERENCE))
		return 1;
	else
		return 0;
}

static ssize_t attr_cali_data_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	D("%s:\n", __func__);

	if(sscanf(buf, "%d %d %d", &(client_data->cali_data_x), &(client_data->cali_data_y),
		  &(client_data->cali_data_z)) != 3) {
		E("%s: input format error!\n", __func__);
		return count;
	}

	if (!is_valid_cali(client_data->cali_data_x) ||
	    !is_valid_cali(client_data->cali_data_y) ||
	    !is_valid_cali(client_data->cali_data_z)) {
		E("%s: Invalid calibration data (x, y, z) = (%d, %d, %d)",
			__func__, client_data->cali_data_x,
			client_data->cali_data_y,
			client_data->cali_data_z);
		return count;
	}

	D("%s: Stored calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, client_data->cali_data_x, client_data->cali_data_y,
			  client_data->cali_data_z);

	return count;
}
#endif /* HTC_CALIBRATION */

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range;
	BMG_CALL_API(get_range_reg)(&range);
	err = sprintf(buf, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = strict_strtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return err;
}

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = sprintf(buf, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	err = strict_strtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	BMG_CALL_API(set_bw)(bandwidth);
	return err;
}


static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = sprintf(buf, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t bmg_store_fastoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fastoffset_en;
	err = strict_strtoul(buf, 10, &fastoffset_en);
	if (err)
		return err;
	if (fastoffset_en) {
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Y_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Z_AXIS, 1);
		BMG_CALL_API(enable_fast_offset)();
	}
	return err;
}

static ssize_t bmg_store_slowoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long slowoffset_en;
	err = strict_strtoul(buf, 10, &slowoffset_en);
	if (err)
		return err;
	if (slowoffset_en) {
		BMG_CALL_API(set_slow_offset_th)(3);
		BMG_CALL_API(set_slow_offset_dur)(0);

		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 1);
	} else {
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 0);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 0);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 0);
	}

	return err;
}

static ssize_t bmg_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	BMG_CALL_API(selftest)(&selftest);
	err = sprintf(buf, "%d\n", selftest);
	return err;
}

static ssize_t bmg_show_sleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char sleepdur;
	BMG_CALL_API(get_sleepdur)(&sleepdur);
	err = sprintf(buf, "%d\n", sleepdur);
	return err;
}

static ssize_t bmg_store_sleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long sleepdur;
	err = strict_strtoul(buf, 10, &sleepdur);
	if (err)
		return err;
	BMG_CALL_API(set_sleepdur)(sleepdur);
	return err;
}

static ssize_t bmg_show_autosleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char autosleepdur;
	BMG_CALL_API(get_autosleepdur)(&autosleepdur);
	err = sprintf(buf, "%d\n", autosleepdur);
	return err;
}

static ssize_t bmg_store_autosleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long autosleepdur;
	unsigned char bandwidth;
	err = strict_strtoul(buf, 10, &autosleepdur);
	if (err)
		return err;
	BMG_CALL_API(get_bw)(&bandwidth);
	BMG_CALL_API(set_autosleepdur)(autosleepdur, bandwidth);
	return err;
}

#ifdef BMG_DEBUG
static ssize_t bmg_store_softreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long softreset;
	err = strict_strtoul(buf, 10, &softreset);
	if (err)
		return err;
	BMG_CALL_API(set_soft_reset)();
	return err;
}

static ssize_t bmg_show_dumpreg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	for (i = 0; i < 0x40; i++) {
		bmg_i2c_read(client_data->client, i, reg+i, 1);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;
}
#endif

#ifdef BMG_USE_FIFO
static ssize_t bmg_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_mode;
	BMG_CALL_API(get_fifo_mode)(&fifo_mode);
	err = sprintf(buf, "%d\n", fifo_mode);
	return err;
}

static ssize_t bmg_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fifo_mode;
	err = strict_strtoul(buf, 10, &fifo_mode);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_mode)(fifo_mode);
	return err;
}

static ssize_t bmg_show_fifo_framecount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_framecount;
	BMG_CALL_API(get_fifo_framecount)(&fifo_framecount);
	err = sprintf(buf, "%d\n", fifo_framecount);
	return err;
}

static ssize_t bmg_show_fifo_overrun(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_overrun;
	BMG_CALL_API(get_fifo_overrun)(&fifo_overrun);
	err = sprintf(buf, "%d\n", fifo_overrun);
	return err;
}

static ssize_t bmg_show_fifo_data_frame(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i, len;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_count, f_len = 0;
	unsigned char fifo_datasel = 0;
	unsigned char fifo_tag = 0;

	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	BMG_CALL_API(get_fifo_data_sel)(&fifo_datasel);
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);

	if (fifo_datasel)
		f_len = 2;
	else
		f_len = 6;

	if (fifo_tag)
		f_len += 2;

	BMG_CALL_API(get_fifo_framecount)(&f_count);

	bmg_i2c_burst_read(client_data->client, BMG160_FIFO_DATA_ADDR,
						fifo_data_out, f_count * f_len);
	err = 0;

	len = sprintf(buf, "%lu ", jiffies);
	buf += len;
	err += len;

	len = sprintf(buf, "%u ", f_count);
	buf += len;
	err += len;

	len = sprintf(buf, "%u ", f_len);
	buf += len;
	err += len;

	for (i = 0; i < f_count * f_len; i++)	{
		len = sprintf(buf, "%d ", fifo_data_out[i]);
		buf += len;
		err += len;
	}

	return err;
}

static ssize_t bmg_show_fifo_data_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_data_sel;
	BMG_CALL_API(get_fifo_data_sel)(&fifo_data_sel);
	err = sprintf(buf, "%d\n", fifo_data_sel);
	return err;
}

static ssize_t bmg_store_fifo_data_sel(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_data_sel;
	err = strict_strtoul(buf, 10, &fifo_data_sel);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_data_sel)(fifo_data_sel);
	return err;
}

static ssize_t bmg_show_fifo_tag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_tag;
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);
	err = sprintf(buf, "%d\n", fifo_tag);
	return err;
}

static ssize_t bmg_store_fifo_tag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_tag;
	err = strict_strtoul(buf, 10, &fifo_tag);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_tag)(fifo_tag);
	return err;
}
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		bmg_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		bmg_show_value, NULL);

#ifdef HTC_CALIBRATION
static DEVICE_ATTR(get_raw_data, S_IRUGO,
		bmg_show_raw_value, NULL);
static DEVICE_ATTR(set_k_value, S_IRUGO|S_IWUSR,
		attr_cali_data_show, attr_cali_data_store);
#endif

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		bmg_show_delay, bmg_store_delay);
static DEVICE_ATTR(fastoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(selftest, S_IRUGO,
		bmg_show_selftest, NULL);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
#ifdef BMG_DEBUG
static DEVICE_ATTR(softreset, S_IRUGO|S_IWUSR,
		NULL, bmg_store_softreset);
static DEVICE_ATTR(regdump, S_IRUGO,
		bmg_show_dumpreg, NULL);
#endif
#ifdef BMG_USE_FIFO
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR,
		bmg_show_fifo_mode, bmg_store_fifo_mode);
static DEVICE_ATTR(fifo_framecount, S_IRUGO|S_IWUSR,
		bmg_show_fifo_framecount, NULL);
static DEVICE_ATTR(fifo_overrun, S_IRUGO|S_IWUSR,
		bmg_show_fifo_overrun, NULL);
static DEVICE_ATTR(fifo_data_frame, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_frame, NULL);
static DEVICE_ATTR(fifo_data_sel, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_sel, bmg_store_fifo_data_sel);
static DEVICE_ATTR(fifo_tag, S_IRUGO|S_IWUSR,
		bmg_show_fifo_tag, bmg_store_fifo_tag);
#endif

static struct attribute *bmg_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_fastoffset_en.attr,
	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_sleepdur.attr,
	&dev_attr_autosleepdur.attr,
#ifdef DEBUG
	&dev_attr_softreset.attr,
	&dev_attr_regdump.attr,
#endif
#ifdef BMG_USE_FIFO
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_tag.attr,
#endif
#ifdef HTC_CALIBRATION
	&dev_attr_get_raw_data.attr,
	&dev_attr_set_k_value.attr,
#endif
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};


static int bmg_input_init(struct bmg_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmg_input_destroy(struct bmg_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}


#ifdef CONFIG_BMG_USE_FIXED_SYSFS_PATH
#define CLASS_NAME_GYRO  "sensor_gyro"
#define DEV_NODE_NAME_BMG   "bmg"
#include <linux/major.h>
#define DEV_ID_BMG      MKDEV(UNNAMED_MAJOR, 0)
static int bmg_create_fixed_sysfs_if(struct bmg_client_data *data)
{
	int err = 0;

	if (NULL == data)
		return -EINVAL;

	err = 0;

	data->cls = class_create(THIS_MODULE, CLASS_NAME_GYRO);
	if (IS_ERR(data->cls)) {
		err = PTR_ERR(data->cls);
		PERR("class create error: %x\n", err);
		goto exit_fail_class_create;
	}

	data->dev_class = device_create(
			data->cls,
			NULL,
			DEV_ID_BMG,
			data,
			DEV_NODE_NAME_BMG);
	if (IS_ERR(data->dev_class)) {
		err = PTR_ERR(data->dev_class);
		PERR("dev create error: %x\n", err);
		goto exit_fail_device_create;
	}

	err = sysfs_create_link(
			&data->dev_class->kobj,
			&data->input->dev.kobj,
			"input");
	if (0 > err) {
		PERR("link create error: %x\n", err);
		goto exit_fail_sysfs_create_link;
	}

	return err;

exit_fail_sysfs_create_link:
	device_destroy(data->cls, DEV_ID_BMG);
exit_fail_device_create:
	data->dev_class = NULL;
	class_destroy(data->cls);
exit_fail_class_create:
	data->cls = NULL;
	return err;
}
#endif


static int to_signed_int(char *value)
{
	int ret_int = 0;

	if (value == NULL)
		ret_int = 0;
	else {
		ret_int = value[0] | (value[1] << 8) |
			  (value[2] << 16) |
			  (value[3] << 24);
	}

	return ret_int;
}

static int bmg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmg_client_data *client_data = NULL;

	PINFO("function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == bmg_client) {
		bmg_client = client;
	} else {
		PERR("this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err_clean;
	}

	/* check chip id */
	err = bmg_check_chip_id(client);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bmg_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);
	mutex_init(&client_data->mutex_value);

	/* input device init */
	err = bmg_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmg_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;

#ifdef CONFIG_BMG_USE_FIXED_SYSFS_PATH
	err = bmg_create_fixed_sysfs_if(client_data);

	if (err < 0)
		PERR("error creating fixed sysfs IF: %x",
				err);
#endif

#ifdef CONFIG_BMG_USE_PLATFORM_DATA
	if (NULL != client->dev.platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, client->dev.platform_data,
					sizeof(*client_data->bst_pd));

			PINFO("place of bmg in %s: %d",
					client_data->bst_pd->name,
					client_data->bst_pd->place);
		}
	}
#endif
	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmg_work_func);
	atomic_set(&client_data->delay, BMG_DELAY_DEFAULT);

	/* h/w init */
	client_data->device.bus_read = bmg_i2c_read_wrapper;
	client_data->device.bus_write = bmg_i2c_write_wrapper;
	client_data->device.delay_msec = bmg_i2c_delay;
	BMG_CALL_API(init)(&client_data->device);

	bmg_dump_reg(client);

	client_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto exit_err_sysfs;


#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.suspend = bmg_early_suspend;
	client_data->early_suspend_handler.resume = bmg_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

#ifdef HTC_CALIBRATION
	client_data->cali_data_x = to_signed_int(&gyro_gsensor_kvalue[4]);
	client_data->cali_data_y = to_signed_int(&gyro_gsensor_kvalue[8]);
	client_data->cali_data_z = to_signed_int(&gyro_gsensor_kvalue[12]);
	D("%s: Calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, client_data->cali_data_x, client_data->cali_data_y,
			  client_data->cali_data_z);
#endif

	PNOTICE("sensor %s probed successfully", SENSOR_NAME);

	PDEBUG("i2c_client: %p client_data: %p i2c_device: %p input: %p",
			client, client_data, &client->dev, client_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmg_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		bmg_client = NULL;
	}

	return err;
}

static int bmg_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	PINFO("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PINFO("cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

static int bmg_post_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	PINFO("function entrance");
	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;
	u8 op_mode;

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	if (op_mode == BMG_VAL_NAME(MODE_NORMAL)) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
}

static void bmg_late_resume(struct early_suspend *handler)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
}
#else
static int bmg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	u8 op_mode;

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	if (op_mode == BMG_VAL_NAME(MODE_NORMAL)) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
}

static int bmg_resume(struct i2c_client *client)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
}
#endif

static int bmg_remove(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;

	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		mutex_lock(&client_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
			cancel_delayed_work_sync(&client_data->work);
			PINFO("cancel work");
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		mdelay(BMG_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(client_data);
		kfree(client_data);

		bmg_client = NULL;
	}

	return err;
}

static const struct i2c_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bmg_id);

static struct i2c_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmg_suspend,
	.resume = bmg_resume,
#endif
};

static int __init BMG_init(void)
{
	return i2c_add_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	i2c_del_driver(&bmg_driver);
}

MODULE_AUTHOR("Ji.Chen <ji.chen@bosch-sensortec.com>");
MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL");

module_init(BMG_init);
module_exit(BMG_exit);

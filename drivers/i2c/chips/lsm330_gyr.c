/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name	: lsm330_gyr.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2.7
* Date		: 2013/May/16
* Description	: LSM330 digital output gyroscope sensor API
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
* VERSION	| DATE			| AUTHORS		  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone	  | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*			|				|				  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2		| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*			|				|				  | passed as parameter at
*			|				|				  | module loading time;
*			|				|				  | corrects polling
*			|				|				  | bug at end of probing;
* 1.1.5.3		| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*			|				|				  | I2C SADROOT; Modifies
*			|				|				  | resume suspend func.
* 1.1.5.4		| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5		| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*			|				|				  | fifomode @ struct foo_status
*			|				|				  | sysfs range input format
*			|				|				  | changed to decimal
* 1.2		| 2012/Jul/10		| Denis Ciocca	  | input_poll_dev removal
* 1.2.1		| 2012/Jul/10		| Denis Ciocca	  | added high resolution timers
* 1.2.2		| 2012/Dec/18	| Denis Ciocca	  | custom sysfs path
* 1.2.5		| 2013/Mar/04	| Matteo Dameno	  | Ch. create_sysfs_interfaces
* 1.2.6		| 2013/Apr/09	| Denis Ciocca	  | Changes resume and suspend
*	 		|				|				  |  functions
* 1.2.7		| 2013/May/16	| Denis Ciocca	  | Added rotation matrices
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>


#include <linux/lsm330.h>
#include <linux/of_gpio.h>
/*#include "lsm330.h"*/
static struct rot_matrix {
       short matrix[3][3];
	} rot_matrix[] = {
		[0] = {
				.matrix = {
						{1, 0, 0},
						{0, 1, 0},
						{0, 0, 1}, }
		},
		[1] = {
				.matrix = {
						{-1, 0, 0},
						{0, -1, 0},
						{0, 0, 1}, }
		},
		[2] = {
				.matrix = {
						{0, 1, 0},
						{-1, 0, 0},
						{0, 0, 1}, }
		},
		[3] = {
				.matrix = {
						{0, -1, 0},
						{1, 0, 0},
						{0, 0, 1}, }
		},
		[4] = {
				.matrix = {
						{0, -1, 0},
						{-1, 0, 0},
						{0, 0, -1}, }
		},
		[5] = {
				.matrix = {
						{0, 1, 0},
						{1, 0, 0},
						{0, 0, -1}, }
		},
		[6] = {
				.matrix = {
						{1, 0, 0},
						{0, -1, 0},
						{0, 0, -1}, }
		},
		[7] = {
				.matrix = {
						{-1, 0, 0},
						{0, 1, 0},
						{0, 0, -1}, }
		},
};

#define CALIBRATION_DATA_PATH "/calibration_data"
#define GYRO_FLASH_DATA "gyro_flash"

#define D(x...) printk(KERN_DEBUG "[GYRO][LSM330] " x)
#define I(x...) printk(KERN_INFO "[GYRO][LSM330] " x)
#define E(x...) printk(KERN_ERR "[GYRO][LSM330] " x)

static unsigned long debug_execute_point;
struct workqueue_struct *lsm330_gyr_wq;
static void debug_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(debug_work, debug_do_work);
#define NUM_SENSOR_DRIVERS 7

static void debug_do_work(struct work_struct *w)
{
	E("%s: debug_execute_point = %lu\n", __func__, debug_execute_point);

	if (debug_execute_point < NUM_SENSOR_DRIVERS) {
		queue_delayed_work(lsm330_gyr_wq, &debug_work,
			msecs_to_jiffies(10000));
	} else {
		cancel_delayed_work(&debug_work);
	}
}

/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX		32768
#define MS_TO_NS(x)			(x*1000000L)

/* lsm330 gyroscope registers */
#define WHO_AM_I	(0x0F)

#define SENSITIVITY_250			8750		/*	udps/LSB */
#define SENSITIVITY_500			17500		/*	udps/LSB */
#define SENSITIVITY_2000		70000		/*	udps/LSB */

#define CTRL_REG1	(0x20)    /* CTRL REG1 */
#define CTRL_REG2	(0x21)    /* CTRL REG2 */
#define CTRL_REG3	(0x22)    /* CTRL_REG3 */
#define CTRL_REG4	(0x23)    /* CTRL_REG4 */
#define CTRL_REG5	(0x24)    /* CTRL_REG5 */
#define	REFERENCE	(0x25)    /* REFERENCE REG */
#define	FIFO_CTRL_REG	(0x2E)    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)    /* FIFO SOURCE REGISTER */
#define	OUT_X_L		(0x28)    /* 1st AXIS OUT REG of 6 */

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES	(0x00)
#define PM_OFF		(0x00)
#define PM_NORMAL	(0x08)
#define ENABLE_ALL_AXES	(0x07)
#define ENABLE_NO_AXES	(0x00)
#define BW00		(0x00)
#define BW01		(0x10)
#define BW10		(0x20)
#define BW11		(0x30)
#define ODR095		(0x00)  /* ODR =  95Hz */
#define ODR190		(0x40)  /* ODR = 190Hz */
#define ODR380		(0x80)  /* ODR = 380Hz */
#define ODR760		(0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY		(0x08)
#define	I2_WTM		(0x04)
#define	I2_OVRUN	(0x02)
#define	I2_EMPTY	(0x01)
#define	I2_NONE		(0x00)
#define	I2_MASK		(0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK		(0x30)
#define	BDU_ENABLE	(0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE	(0x40)
#define HPF_ENALBE	(0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK		(0xE0)
#define	FIFO_MODE_BYPASS	(0x00)
#define	FIFO_MODE_FIFO		(0x20)
#define	FIFO_MODE_STREAM	(0x40)
#define	FIFO_MODE_STR2FIFO	(0x60)
#define	FIFO_MODE_BYPASS2STR	(0x80)
#define	FIFO_WATERMARK_MASK	(0x1F)

#define FIFO_STORED_DATA_MASK	(0x1F)

#define I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_FIFO_CTRL_REG	5
#define	RESUME_ENTRIES		6
#define TOLERENCE		358


/* #define DEBUG 1 */

/** Registers Contents */
#define WHOAMI_LSM330_GYR	(0xD4)  /* Expected content for WAI register*/
#define DIF(x...) {\
                if (DEBUG_FLAG)\
                       printk(KERN_DEBUG "[LSM330] [GYRO]" x); }

static int DEBUG_FLAG = 0;
module_param(DEBUG_FLAG,int,0600);

static int int1_gpio = LSM330_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

/*
 * LSM330 gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * s32
 */

struct lsm330_gyr_triple {
	s32	x,	/* x-axis angular rate data. */
		y,	/* y-axis angluar rate data. */
		z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	2,	ODR760|BW10},
	{	3,	ODR380|BW01},
	{	6,	ODR190|BW00},
	{	11,	ODR095|BW00},
};

static struct lsm330_gyr_platform_data default_lsm330_gyr_pdata = {
	.fs_range = LSM330_GYR_FS_250DPS,
	.rot_matrix_index = 0,
	.poll_interval = 100,
	.min_interval = LSM330_GYR_MIN_POLL_PERIOD_MS, /* 2ms */
	.gpio_int1 = LSM330_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */
};

struct workqueue_struct *lsm330_gyr_workqueue = 0;

struct lsm330_gyr_status {
	struct i2c_client *client;
	struct lsm330_gyr_platform_data *pdata;
	struct mutex lock;
	struct input_dev *input_dev;
	short rot_matrix[3][3];
	int hw_initialized;
	atomic_t enabled;
	int use_smbus;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	u32 sensitivity;

#ifdef CUSTOM_SYSFS_PATH
	struct class *gyr_class;
	struct device *gyr_dev;
#endif

#ifdef CONFIG_PM
	int on_before_suspend;
#endif

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	bool polling_enabled;
	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct hrtimer hr_timer;
	ktime_t ktime;
	struct work_struct polling_task;
	int cali_data_x;
	int cali_data_y;
	int cali_data_z;
};

static int lsm330_gyr_i2c_read(struct lsm330_gyr_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;

/*
	if (len > sizeof(buf))
			dev_err(&stat->client->dev,
				"read error insufficient buffer length: "
				"len:%d, buf size=%d\n",
				len, sizeof(buf));
*/
	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			unsigned int ii;
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lsm330_gyr_i2c_write(struct lsm330_gyr_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			unsigned int ii;
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}


static int lsm330_gyr_register_write(struct lsm330_gyr_status *stat,
		u8 *buf, u8 reg_address, u8 new_value)
{
	int err;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lsm330_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

	return err;
}

static int lsm330_gyr_register_read(struct lsm330_gyr_status *stat,
							u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330_gyr_i2c_read(stat, buf, 1);
	return err;
}

static int lsm330_gyr_register_update(struct lsm330_gyr_status *stat,
			u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_gyr_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_gyr_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}


static int lsm330_gyr_update_watermark(struct lsm330_gyr_status *stat,
								u8 watermark)
{
	int res = 0;
	u8 buf[2];
	u8 new_value;

	mutex_lock(&stat->lock);
	new_value = (watermark % 0x20);
	res = lsm330_gyr_register_update(stat, buf, FIFO_CTRL_REG,
			 FIFO_WATERMARK_MASK, new_value);
	if (res < 0) {
		dev_err(&stat->client->dev, "failed to update watermark\n");
		return res;
	}
	dev_dbg(&stat->client->dev, "%s new_value:0x%02x,watermark:0x%02x\n",
						__func__, new_value, watermark);

	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_WATERMARK_MASK & new_value) |
		(~FIFO_WATERMARK_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->watermark = new_value;
	mutex_unlock(&stat->lock);
	return res;
}

static int lsm330_gyr_update_fifomode(struct lsm330_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	u8 new_value;

	new_value = fifomode;
	res = lsm330_gyr_register_update(stat, buf, FIFO_CTRL_REG,
					FIFO_MODE_MASK, new_value);
	if (res < 0) {
		dev_err(&stat->client->dev, "failed to update fifoMode\n");
		return res;
	}
	/*
	dev_dbg(&stat->client->dev, "new_value:0x%02x,prev fifomode:0x%02x\n",
				__func__, new_value, stat->fifomode);
	 */
	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_MODE_MASK & new_value) |
		(~FIFO_MODE_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->fifomode = new_value;

	return res;
}

static int lsm330_gyr_fifo_reset(struct lsm330_gyr_status *stat)
{
	u8 oldmode;
	int res;

	oldmode = stat->fifomode;
	res = lsm330_gyr_update_fifomode(stat, FIFO_MODE_BYPASS);
	if (res < 0)
		return res;
	res = lsm330_gyr_update_fifomode(stat, oldmode);
	if (res >= 0)
		dev_dbg(&stat->client->dev, "%s fifo reset to: 0x%02x\n",
							__func__, oldmode);

	return res;
}

static int lsm330_gyr_fifo_hwenable(struct lsm330_gyr_status *stat,
								u8 enable)
{
	int res;
	u8 buf[2];
	u8 set = 0x00;
	if (enable)
		set = FIFO_ENABLE;

	res = lsm330_gyr_register_update(stat, buf, CTRL_REG5,
			FIFO_ENABLE, set);
	if (res < 0) {
		dev_err(&stat->client->dev, "fifo_hw switch to:0x%02x failed\n",
									set);
		return res;
	}
	stat->resume_state[RES_CTRL_REG5] =
		((FIFO_ENABLE & set) |
		(~FIFO_ENABLE & stat->resume_state[RES_CTRL_REG5]));
	dev_dbg(&stat->client->dev, "%s set to:0x%02x\n", __func__, set);
	return res;
}

static int lsm330_gyr_manage_int2settings(struct lsm330_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	bool enable_fifo_hw;
	bool recognized_mode = false;
	u8 int2bits = I2_NONE;
/*
	if (stat->polling_enabled) {
		fifomode = FIFO_MODE_BYPASS;
		dbg_warn(&stat->client->dev, "in polling mode, fifo mode forced"
							" to BYPASS mode\n");
	}
*/


	switch (fifomode) {
	case FIFO_MODE_FIFO:
		recognized_mode = true;

		if (stat->polling_enabled) {
			int2bits = I2_NONE;
			enable_fifo_hw = false;
		} else {
			int2bits = (I2_WTM | I2_OVRUN);
			enable_fifo_hw = true;
		}
		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update "
							"CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~(I2_MASK) & stat->resume_state[RES_CTRL_REG3]));
		/* enable_fifo_hw = true; */
		break;

	case FIFO_MODE_BYPASS:
		recognized_mode = true;

		if (stat->polling_enabled)
			int2bits = I2_NONE;
		else
			int2bits = I2_DRDY;

		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update"
						" to CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		enable_fifo_hw = false;
		break;

	default:
		recognized_mode = false;
		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, I2_NONE);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update "
						"CTRL_REG3:0x%02x\n",
						__func__, fifomode);
			goto err_mutex_unlock;
		}
		enable_fifo_hw = false;
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & 0x00) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		break;

	}
	if (recognized_mode) {
		res = lsm330_gyr_update_fifomode(stat, fifomode);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to "
						"set fifoMode\n", __func__);
			goto err_mutex_unlock;
		}
	}
	res = lsm330_gyr_fifo_hwenable(stat, enable_fifo_hw);

err_mutex_unlock:

	return res;
}


static int lsm330_gyr_update_fs_range(struct lsm330_gyr_status *stat,
							u8 new_fs)
{
	int res ;
	u8 buf[2];

	u32 sensitivity;

	switch(new_fs) {
		case LSM330_GYR_FS_250DPS:
			sensitivity = SENSITIVITY_250;
			break;
		case LSM330_GYR_FS_500DPS:
			sensitivity = SENSITIVITY_500;
			break;
		case LSM330_GYR_FS_2000DPS:
			sensitivity = SENSITIVITY_2000;
			break;
		default:
			dev_err(&stat->client->dev, "invalid g range "
						"requested: %u\n", new_fs);
			return -EINVAL;
	}


	buf[0] = CTRL_REG4;

	res = lsm330_gyr_register_update(stat, buf, CTRL_REG4,
							FS_MASK, new_fs);

	if (res < 0) {
		dev_err(&stat->client->dev, "%s : failed to update fs:0x%02x\n",
							__func__, new_fs);
		return res;
	}
	stat->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		(~FS_MASK & stat->resume_state[RES_CTRL_REG4]));

	stat->sensitivity = sensitivity;
	return res;
}


static int lsm330_gyr_update_odr(struct lsm330_gyr_status *stat,
			unsigned int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval_ms) || (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lsm330_gyr_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CTRL_REG1] = config[1];
		stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;
}

/* gyroscope data readout */
static int lsm330_gyr_get_data(struct lsm330_gyr_status *stat,
			     struct lsm330_gyr_triple *data)
{
	int i, err, out[3];
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s32 hw_d[3] = { 0 };

	gyro_out[0] = (AXISDATA_REG);

	err = lsm330_gyr_i2c_read(stat, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s32) ((s16)((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s32) ((s16)((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s32) ((s16)((gyro_out[5]) << 8) | gyro_out[4]);

	//hw_d[0] = hw_d[0] * stat->sensitivity;
	//hw_d[1] = hw_d[1] * stat->sensitivity;
	//hw_d[2] = hw_d[2] * stat->sensitivity;

	for (i = 0; i < 3; i++) {
		out[i] = stat->rot_matrix[0][i] * hw_d[0] +
				stat->rot_matrix[1][i] * hw_d[1] +
					stat->rot_matrix[2][i] * hw_d[2];
	}

	data->x = out[0];
	data->y = out[1];
	data->z = out[2];

	DIF("gyro_out: x = %d, y = %d, z = %d\n",
		data->x, data->y, data->z);

	return err;
}

static void lsm330_gyr_report_values(struct lsm330_gyr_status *stat,
					struct lsm330_gyr_triple *data)
{

	input_report_abs(stat->input_dev, ABS_X, data->x);
	input_report_abs(stat->input_dev, ABS_Y, data->y);
	input_report_abs(stat->input_dev, ABS_Z, data->z);

	input_sync(stat->input_dev);
}

static int lsm330_gyr_hw_init(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[6];

	dev_info(&stat->client->dev, "hw init\n");

	buf[0] = (CTRL_REG1);
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	buf[2] = stat->resume_state[RES_CTRL_REG2];
	buf[3] = stat->resume_state[RES_CTRL_REG3];
	buf[4] = stat->resume_state[RES_CTRL_REG4];
	buf[5] = stat->resume_state[RES_CTRL_REG5];

	err = lsm330_gyr_i2c_write(stat, buf, 5);
	if (err < 0)
		return err;

	buf[0] = (FIFO_CTRL_REG);
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lsm330_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
			return err;

	stat->hw_initialized = 1;

	return err;
}

static void lsm330_gyr_device_power_off(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[2];

	dev_info(&stat->client->dev, "power off\n");

	buf[0] = (CTRL_REG1);
	buf[1] = (PM_OFF);
	err = lsm330_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed\n");

	if (stat->pdata->power_off) {
		/* disable_irq_nosync(acc->irq1); */
		disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized) {
		/*
		if (stat->pdata->gpio_int1 >= 0) {
			disable_irq_nosync(stat->irq1);
			dev_info(&stat->client->dev,
					"power off: irq1 disabled\n");
		}
		*/
		if (stat->pdata->gpio_int2 >= 0) {
			disable_irq_nosync(stat->irq2);
			dev_info(&stat->client->dev,
					"power off: irq2 disabled\n");
		}
		stat->hw_initialized = 0;
	}
}

static int lsm330_gyr_device_power_on(struct lsm330_gyr_status *stat)
{
	int err;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0)
			return err;
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}


	if (!stat->hw_initialized) {
		err = lsm330_gyr_hw_init(stat);
		if (err < 0) {
			lsm330_gyr_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		/* if (stat->pdata->gpio_int1) {
			enable_irq(stat->irq1);
			dev_info(&stat->client->dev,
						"power on: irq1 enabled\n");
		} */
		dev_dbg(&stat->client->dev, "stat->pdata->gpio_int2 = %d\n",
						stat->pdata->gpio_int2);
		if (stat->pdata->gpio_int2 >= 0) {
			enable_irq(stat->irq2);
			dev_info(&stat->client->dev,
					"power on: irq2 enabled\n");
		}
	}

	return 0;
}

static int lsm330_gyr_enable(struct lsm330_gyr_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {

		err = lsm330_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}

		if (stat->polling_enabled) {
			stat->ktime = ktime_set(stat->pdata->poll_interval / 1000,
					MS_TO_NS(stat->pdata->poll_interval % 1000));
			hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
		}

	}

	return 0;
}

static int lsm330_gyr_disable(struct lsm330_gyr_status *stat)
{
	dev_info(&stat->client->dev, "%s: stat->enabled = %d\n", __func__,
						atomic_read(&stat->enabled));

	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {

		lsm330_gyr_device_power_off(stat);
    cancel_work_sync(&stat->polling_task);
		hrtimer_cancel(&stat->hr_timer);
		dev_info(&stat->client->dev, "%s: cancel_hrtimer ", __func__);
	}
	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&stat->lock);
	err = lsm330_gyr_update_odr(stat, interval_ms);
	if(err >= 0) {
		stat->pdata->poll_interval = interval_ms;
		stat->ktime = ktime_set(stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int range = 0;
	u8 val;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
	case LSM330_GYR_FS_250DPS:
		range = 250;
		break;
	case LSM330_GYR_FS_500DPS:
		range = 500;
		break;
	case LSM330_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&stat->lock);
	/* return sprintf(buf, "0x%02x\n", val); */
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 250:
		range = LSM330_GYR_FS_250DPS;
		break;
	case 500:
		range = LSM330_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM330_GYR_FS_2000DPS;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm330_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu dps\n", val);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_gyr_enable(stat);
	else
		lsm330_gyr_disable(stat);

	return size;
}

static ssize_t attr_polling_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	if (stat->polling_enabled)
		val = 1;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	if (val) {
		stat->polling_enabled = true;
		lsm330_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode enabled\n");
		if (atomic_read(&stat->enabled)) {
			hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
		}
	} else {
		if (stat->polling_enabled) {
			hrtimer_cancel(&stat->hr_timer);
		}
		stat->polling_enabled = false;
		lsm330_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode disabled\n");
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_watermark_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long watermark;
	int res;

	if (strict_strtoul(buf, 16, &watermark))
		return -EINVAL;

	res = lsm330_gyr_update_watermark(stat, watermark);
	if (res < 0)
		return res;

	return size;
}

static ssize_t attr_watermark_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int val = stat->watermark;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long fifomode;
	int res;

	if (strict_strtoul(buf, 16, &fifomode))
		return -EINVAL;
	/* if (!fifomode)
		return -EINVAL; */

	dev_dbg(dev, "%s, got value:0x%02x\n", __func__, (u8)fifomode);

	mutex_lock(&stat->lock);
	res = lsm330_gyr_manage_int2settings(stat, (u8) fifomode);
	mutex_unlock(&stat->lock);

	if (res < 0)
		return res;
	return size;
}

static ssize_t attr_fifomode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	u8 val = stat->fifomode;
	return sprintf(buf, "0x%02x\n", val);
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm330_gyr_i2c_write(stat, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm330_gyr_i2c_read(stat, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);

	stat->reg_addr = val;

	mutex_unlock(&stat->lock);

	return size;
}
#endif /* DEBUG */

static ssize_t attr_cali_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);

	s += sprintf(s, "Stored calibration data (x, y, z) = (%d, %d, %d)\n",
		stat->cali_data_x, stat->cali_data_y,
		stat->cali_data_z);

	D("%s: Calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, stat->cali_data_x, stat->cali_data_y,
			  stat->cali_data_z);
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
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);

	D("%s: \n", __func__);

	if(sscanf(buf, "%d %d %d", &(stat->cali_data_x), &(stat->cali_data_y),
		  &(stat->cali_data_z)) != 3) {
		E("%s: input format error!\n", __func__);
		return count;
	}

	if (!is_valid_cali(stat->cali_data_x) ||
	    !is_valid_cali(stat->cali_data_y) ||
	    !is_valid_cali(stat->cali_data_z)) {
		E("%s: Invalid calibration data (x, y, z) = (%d, %d, %d)",
			__func__, stat->cali_data_x, stat->cali_data_y,
			stat->cali_data_z);
		return count;
	}

	D("%s: Stored calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, stat->cali_data_x, stat->cali_data_y,
			  stat->cali_data_z);

	return count;
}

static ssize_t attr_debug_event_report_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "debug_execute_point = %lu\n", debug_execute_point);

	D("%s: debug_execute_point = %lu\n", __func__, debug_execute_point);
	return s - buf;
}

static ssize_t attr_debug_event_report_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long val = 200;

	DIF("%s: \n", __func__);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 32) {
		E("%s: Invalid input val = %lu\n", __func__, val);
		return count;
	}

	debug_execute_point = val;
	if (debug_execute_point < NUM_SENSOR_DRIVERS) {
		queue_delayed_work(lsm330_gyr_wq, &debug_work,
			msecs_to_jiffies(2000));
	} else {
		cancel_delayed_work(&debug_work);
	}

	DIF("%s: val = %lu\n", __func__, val);

	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_polling_rate_show,
						attr_polling_rate_store),
	__ATTR(range, 0666, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0666, attr_enable_show, attr_enable_store),
	__ATTR(enable_polling, 0666, attr_polling_mode_show,
						attr_polling_mode_store),
	__ATTR(fifo_samples, 0666, attr_watermark_show, attr_watermark_store),
	__ATTR(fifo_mode, 0666, attr_fifomode_show, attr_fifomode_store),
	__ATTR(cali_data, 0664, attr_cali_data_show, attr_cali_data_store),
	__ATTR(debug_event_report, 0664, attr_debug_event_report_show, attr_debug_event_report_store),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct lsm330_gyr_status *gyr)
{
	int i;

#ifdef CUSTOM_SYSFS_PATH
	gyr->gyr_class = class_create(THIS_MODULE, CUSTOM_SYSFS_CLASS_NAME_GYR);
	if (gyr->gyr_class == NULL)
		goto custom_class_error;

	gyr->gyr_dev = device_create(gyr->gyr_class, NULL, 0, "%s", "gyr");
	if (gyr->gyr_dev == NULL)
		goto custom_class_error;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(gyr->gyr_dev, attributes + i))
			goto error;
#else
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(&gyr->client->dev, attributes + i))
			goto error;
#endif
	return 0;

error:
	for ( ; i >= 0; i--)
#ifdef CUSTOM_SYSFS_PATH
		device_remove_file(gyr->gyr_dev, attributes + i);
#else
		device_remove_file(&gyr->client->dev, attributes + i);
#endif

#ifdef CUSTOM_SYSFS_PATH
custom_class_error:
#endif
	dev_err(&gyr->client->dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm330_gyr_report_triple(struct lsm330_gyr_status *stat)
{
	int err;
	struct lsm330_gyr_triple data_out;

	err = lsm330_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_gyroscope_data failed\n");
	else
		lsm330_gyr_report_values(stat, &data_out);
}


static void lsm330_gyr_irq2_fifo(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[2];
	u8 int_source;
	u8 samples;
	u8 workingmode;
	u8 stored_samples;

	mutex_lock(&stat->lock);

	workingmode = stat->fifomode;


	dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
								workingmode);


	switch (workingmode) {
	case FIFO_MODE_BYPASS:
	{
		dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
							stat->fifomode);
		lsm330_gyr_report_triple(stat);
		break;
	}
	case FIFO_MODE_FIFO:
		samples = (stat->watermark)+1;
		dev_dbg(&stat->client->dev,
			"%s : FIFO_SRC_REG init samples:%d\n",
							__func__, samples);
		err = lsm330_gyr_register_read(stat, buf, FIFO_SRC_REG);
		if (err < 0)
			dev_err(&stat->client->dev,
					"error reading fifo source reg\n");

		int_source = buf[0];
		dev_dbg(&stat->client->dev, "%s :FIFO_SRC_REG content:0x%02x\n",
							__func__, int_source);

		stored_samples = int_source & FIFO_STORED_DATA_MASK;
		dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
						stat->fifomode);

		dev_dbg(&stat->client->dev, "%s : samples:%d stored:%d\n",
				__func__, samples, stored_samples);

		for (; samples > 0; samples--) {
#ifdef DEBUG
			input_report_abs(stat->input_dev, ABS_MISC, 1);
			input_sync(stat->input_dev);
#endif
			dev_dbg(&stat->client->dev, "%s : current sample:%d\n",
							__func__, samples);

			lsm330_gyr_report_triple(stat);

#ifdef DEBUG
			input_report_abs(stat->input_dev, ABS_MISC, 0);
			input_sync(stat->input_dev);
#endif
		}
		lsm330_gyr_fifo_reset(stat);
		break;
	}
#ifdef DEBUG
	input_report_abs(stat->input_dev, ABS_MISC, 3);
	input_sync(stat->input_dev);
#endif

	mutex_unlock(&stat->lock);
}

static irqreturn_t lsm330_gyr_isr2(int irq, void *dev)
{
	struct lsm330_gyr_status *stat = dev;

	disable_irq_nosync(irq);
#ifdef DEBUG
	input_report_abs(stat->input_dev, ABS_MISC, 2);
	input_sync(stat->input_dev->input);
#endif
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s %s: isr2 queued\n", LSM330_GYR_DEV_NAME, __func__);

	return IRQ_HANDLED;
}

static void lsm330_gyr_irq2_work_func(struct work_struct *work)
{

	struct lsm330_gyr_status *stat =
		container_of(work, struct lsm330_gyr_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_gyr_irq2_XXX(stat); */
	lsm330_gyr_irq2_fifo(stat);
	/*  */
	pr_debug("%s %s: IRQ2 served\n", LSM330_GYR_DEV_NAME, __func__);
/* exit: */
	enable_irq(stat->irq2);
}

int lsm330_gyr_input_open(struct input_dev *input)
{
	//struct lsm330_gyr_status *stat = input_get_drvdata(input);
	//dev_dbg(&stat->client->dev, "%s\n", __func__);
	return 0;//lsm330_gyr_enable(stat);
}

void lsm330_gyr_input_close(struct input_dev *dev)
{
	//struct lsm330_gyr_status *stat = input_get_drvdata(dev);
	//dev_dbg(&stat->client->dev, "%s\n", __func__);
	//lsm330_gyr_disable(stat);
}

static int lsm330_gyr_validate_pdata(struct lsm330_gyr_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) LSM330_GYR_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if ((stat->pdata->rot_matrix_index >= ARRAY_SIZE(rot_matrix)) ||
				(stat->pdata->rot_matrix_index < 0)) {
		dev_err(&stat->client->dev, "rotation matrix index invalid.\n");
		return -EINVAL;
	}
	memcpy(stat->rot_matrix,
		rot_matrix[stat->pdata->rot_matrix_index].matrix,
							9 * sizeof(short));

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm330_gyr_input_init(struct lsm330_gyr_status *stat)
{
	int err = -1;

	dev_dbg(&stat->client->dev, "%s\n", __func__);

	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
			"input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lsm330_gyr_input_open;
	stat->input_dev->close = lsm330_gyr_input_close;
	stat->input_dev->name = LSM330_GYR_DEV_NAME;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

#ifdef DEBUG
	set_bit(EV_KEY, stat->input_dev->keybit);
	set_bit(KEY_LEFT, stat->input_dev->keybit);
	input_set_abs_params(stat->input_dev, ABS_MISC, 0, 1, 0, 0);
#endif

	input_set_abs_params(stat->input_dev, ABS_X, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Y, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Z, -FS_MAX-1, FS_MAX, 0, 0);


	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm330_gyr_input_cleanup(struct lsm330_gyr_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static void poll_function_work(struct work_struct *polling_task)
{
	struct lsm330_gyr_status *stat;
	struct lsm330_gyr_triple data_out;
	int err;

	stat = container_of((struct work_struct *)polling_task,
					struct lsm330_gyr_status, polling_task);

	err = lsm330_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_rotation_data failed.\n");
	else
		lsm330_gyr_report_values(stat, &data_out);
	if (atomic_read(&stat->enabled))    
  	  hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
	struct lsm330_gyr_status *stat;
	stat = container_of((struct hrtimer *)timer,
				struct lsm330_gyr_status, hr_timer);
	queue_work(lsm330_gyr_workqueue, &stat->polling_task);
	return HRTIMER_NORESTART;
}

static int lsm330_gyr_parse_dt(struct device *dev, struct lsm330_gyr_platform_data *pdata)
{
	struct property *prop = NULL;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;

	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;

	if (pdata == NULL) {
		E("%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	prop = of_find_property(dt, "gyro_lsm330,fs_range", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,fs_range", &buf);
		pdata->fs_range = buf;
		I("%s: fs_range = 0x%x", __func__, pdata->fs_range);
	} else
		I("%s: gyro_lsm330,fs_range not found", __func__);

        prop = of_find_property(dt, "gyro_lsm330,rot_matrix_index", NULL);
        if (prop) {
                of_property_read_u32(dt, "gyro_lsm330,rot_matrix_index", &buf);
                pdata->rot_matrix_index = buf;
                I("%s: rot_matrix_index = %d", __func__, pdata->rot_matrix_index);
        } else
                I("%s: gyro_lsm330,rot_matrix_index not found", __func__);


/*
	prop = of_find_property(dt, "gyro_lsm330,axis_map_x", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,axis_map_x", &buf);
		pdata->axis_map_x = buf;
		I("%s: axis_map_x = %d", __func__, pdata->axis_map_x);
	} else
		I("%s: gyro_lsm330,axis_map_x not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,axis_map_y", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,axis_map_y", &buf);
		pdata->axis_map_y = buf;
		I("%s: axis_map_y = %d", __func__, pdata->axis_map_y);
	} else
		I("%s: gyro_lsm330,axis_map_y not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,axis_map_z", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,axis_map_z", &buf);
		pdata->axis_map_z = buf;
		I("%s: axis_map_z = %d", __func__, pdata->axis_map_z);
	} else
		I("%s: gyro_lsm330,axis_map_z not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,negate_x", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,negate_x", &buf);
		pdata->negate_x = buf;
		I("%s: negate_x = %d", __func__, pdata->negate_x);
	} else
		I("%s: gyro_lsm330,negate_x not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,negate_y", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,negate_y", &buf);
		pdata->negate_y = buf;
		I("%s: negate_y = %d", __func__, pdata->negate_y);
	} else
		I("%s: gyro_lsm330,negate_y not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,negate_z", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,negate_z", &buf);
		pdata->negate_z = buf;
		I("%s: negate_z = %d", __func__, pdata->negate_z);
	} else
		I("%s: gyro_lsm330,negate_z not found", __func__);
*/
	prop = of_find_property(dt, "gyro_lsm330,poll_interval", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,poll_interval", &buf);
		pdata->poll_interval = buf;
		I("%s: poll_interval = %d", __func__, pdata->poll_interval);
	} else
		I("%s: gyro_lsm330,poll_interval not found", __func__);

	prop = of_find_property(dt, "gyro_lsm330,min_interval", NULL);
	if (prop) {
		of_property_read_u32(dt, "gyro_lsm330,min_interval", &buf);
		pdata->min_interval = buf;
		I("%s: min_interval = %d", __func__, pdata->min_interval);
	} else
		I("%s: gyro_lsm330,min_interval not found", __func__);

	for (i = 0; i < 37; i++)
		pdata->gyro_kvalue[i] = 0;
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, GYRO_FLASH_DATA, &cali_size);
		I("%s: cali_size = %d", __func__, cali_size);
		I("%s: cali_data = %p", __func__, cali_data);
		if (cali_data) {
			i = 0;
			for (i = 0; (i < cali_size) && (i < 37); i++) {
				I("%s: cali_data[%d] = %02x ", __func__, i, cali_data[i]);
				pdata->gyro_kvalue[i] = cali_data[i];
			}
		}
	} else
		I("%s: Calibration data offset not found", __func__);

	pdata->init = NULL;
	pdata->exit = NULL;
	pdata->power_on = NULL;
	pdata->power_off = NULL;
	pdata->watermark = 0;
	pdata->fifomode = 0;
	pdata->gpio_int1 = 0;
	pdata->gpio_int2 = 0;

	return 0;
}

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

static int __devinit lsm330_gyr_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct lsm330_gyr_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

	int err = -1;

	dev_info(&client->dev, "probe start.\n");


	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			stat->use_smbus = 0;
			goto err0;
		}
	}

	if(lsm330_gyr_workqueue == 0)
		lsm330_gyr_workqueue = create_workqueue("lsm330_gyr_workqueue");

	hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer.function = &poll_function_read;

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);
	stat->client = client;

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}

	if (client->dev.of_node) {
		I("Device Tree parsing.");

		err = lsm330_gyr_parse_dt(&client->dev, stat->pdata);
		if (err) {
			dev_err(&client->dev, "%s: lsm330_gyr_parse_dt "
					"for pdata failed. err = %d",
					__func__, err);
			goto err1_1_0;
		}
	} else {
		if (client->dev.platform_data == NULL) {
			default_lsm330_gyr_pdata.gpio_int1 = int1_gpio;
			default_lsm330_gyr_pdata.gpio_int2 = int2_gpio;
			memcpy(stat->pdata, &default_lsm330_gyr_pdata,
								sizeof(*stat->pdata));
			dev_info(&client->dev, "using default plaform_data\n");
		} else {
			memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
		}
	}

	err = lsm330_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, stat);

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}


	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES
								| PM_NORMAL;
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	stat->polling_enabled = true;
	dev_info(&client->dev, "polling mode enabled\n");

	err = lsm330_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm330_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = lsm330_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm330_gyr_input_init(stat);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(stat);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", LSM330_GYR_DEV_NAME);
		goto err4;
	}

#ifdef CUSTOM_SYSFS_PATH
	dev_set_drvdata(stat->gyr_dev, stat);
#endif

	lsm330_gyr_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);


	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		dev_info(&client->dev, "%s: %s has set irq2 to irq:"
						" %d mapped on gpio:%d\n",
			LSM330_GYR_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);

		INIT_WORK(&stat->irq2_work, lsm330_gyr_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lsm330_gyr_irq2_wq");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create "
						"work queue2: %d\n", err);
			goto err5;
		}

		err = request_irq(stat->irq2, lsm330_gyr_isr2,
				IRQF_TRIGGER_HIGH, "lsm330_gyr_irq2", stat);

		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err6;
		}
		disable_irq_nosync(stat->irq2);
	}

	mutex_unlock(&stat->lock);

	INIT_WORK(&stat->polling_task, poll_function_work);

	dev_info(&client->dev, "%s probed: device created successfully\n",
							LSM330_GYR_DEV_NAME);

	if (client->dev.of_node) {
		stat->cali_data_x = to_signed_int(&stat->pdata->gyro_kvalue[4]);
		stat->cali_data_y = to_signed_int(&stat->pdata->gyro_kvalue[8]);
		stat->cali_data_z = to_signed_int(&stat->pdata->gyro_kvalue[12]);
	} else {
		/*stat->cali_data_x = to_signed_int(&gyro_gsensor_kvalue[4]);
		stat->cali_data_y = to_signed_int(&gyro_gsensor_kvalue[8]);
		stat->cali_data_z = to_signed_int(&gyro_gsensor_kvalue[12]);*/
	}
	D("%s: Calibration data (x, y, z) = (%d, %d, %d)\n",
		__func__, stat->cali_data_x, stat->cali_data_y,
			  stat->cali_data_z);

	debug_execute_point = 0;

	lsm330_gyr_wq = create_singlethread_workqueue("lsm330_gyr_debug_wq");
	if (!lsm330_gyr_wq) {
		E("%s: can't create workqueue\n", __func__);
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	return 0;

err_create_singlethread_workqueue:
	if (stat && (stat->pdata) && (stat->pdata->gpio_int2 >= 0))
		free_irq(stat->irq2, stat);
/*err7:
	free_irq(stat->irq2, stat);
*/
err6:
	destroy_workqueue(stat->irq2_work_queue);
err5:
	lsm330_gyr_device_power_off(stat);
	remove_sysfs_interfaces(&client->dev);
err4:
	lsm330_gyr_input_cleanup(stat);
err3:
	lsm330_gyr_device_power_off(stat);
err2:
	if (stat->pdata->exit)
		stat->pdata->exit();
err1_1:
err1_1_0:
	mutex_unlock(&stat->lock);
	kfree(stat->pdata);
err1:
	destroy_workqueue(lsm330_gyr_workqueue);
	kfree(stat);
err0:
		pr_err("%s: Driver Initialization failed\n",
							LSM330_GYR_DEV_NAME);
		return err;
}

static int lsm330_gyr_remove(struct i2c_client *client)
{
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	dev_info(&stat->client->dev, "driver removing\n");

	lsm330_gyr_disable(stat);
	if(!lsm330_gyr_workqueue) {
		flush_workqueue(lsm330_gyr_workqueue);
		destroy_workqueue(lsm330_gyr_workqueue);
	}
	/*
	if (stat->pdata->gpio_int1 >= 0)
	{
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}
	*/
	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lsm330_gyr_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

#ifdef CONFIG_PM
static int lsm330_gyr_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lsm330_gyr_disable(stat);
}

static int lsm330_gyr_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	if (stat->on_before_suspend)
		return lsm330_gyr_enable(stat);

	return 0;
}
#else /* CONFIG_PM */
#define lsm330_gyr_suspend	NULL
#define lsm330_gyr_resume	NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id lsm330_gyr_id[] = {
	{ LSM330_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm330_gyr_id);
#ifdef CONFIG_OF
static struct of_device_id lsm330_gyr_match_table[] = {
	{.compatible = "htc_gyro,lsm330" },
	{},
};
#else
#define lsm330_gyr_match_table NULL
#endif

static const struct dev_pm_ops lsm330_gyr_pm = {
	.suspend = lsm330_gyr_suspend,
	.resume = lsm330_gyr_resume,
};

static struct i2c_driver lsm330_gyr_driver = {
#if 0
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_GYR_DEV_NAME,
#if 0
			.pm = &lsm330_gyr_pm,
#endif
	},
	.probe = lsm330_gyr_probe,
	.remove = __devexit_p(lsm330_gyr_remove),
	.id_table = lsm330_gyr_id,
#if 1
        .suspend = lsm330_gyr_suspend,
        .resume = lsm330_gyr_resume,	
#endif
#endif
	.driver = {
		.name           = LSM330_GYR_DEV_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = lsm330_gyr_match_table,
#ifdef CONFIG_PM
		.pm             = &lsm330_gyr_pm,
#endif
	},
	.probe    = lsm330_gyr_probe,
	.remove   = lsm330_gyr_remove,
	.id_table = lsm330_gyr_id,

};
module_i2c_driver(lsm330_gyr_driver);

#if 0
static int __init lsm330_gyr_init(void)
{

	pr_info("%s: gyroscope sysfs driver init\n", LSM330_GYR_DEV_NAME);

	return i2c_add_driver(&lsm330_gyr_driver);
}

static void __exit lsm330_gyr_exit(void)
{

	pr_info("%s exit\n", LSM330_GYR_DEV_NAME);

	i2c_del_driver(&lsm330_gyr_driver);
	return;
}

module_init(lsm330_gyr_init);
module_exit(lsm330_gyr_exit);
#endif

MODULE_DESCRIPTION("lsm330 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

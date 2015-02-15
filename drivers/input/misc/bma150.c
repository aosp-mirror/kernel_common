/*  Date: 2011/3/7 11:00:00
 *  Revision: 2.11
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file BMA150.c
   brief This file contains all function implementations for the BMA150 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/bma150.h>

#define SENSOR_NAME			"bma150"
#define GRAVITY_EARTH		9806550
#define ABSMIN_2G			(-GRAVITY_EARTH * 2)
#define ABSMAX_2G			(GRAVITY_EARTH * 2)
#define BMA150_MAX_DELAY	200
#define BMA150_CHIP_ID		2
#define BMA150_RANGE_SET	0
#define BMA150_BW_SET		4



#define BMA150_CHIP_ID_REG		0x00
#define BMA150_X_AXIS_LSB_REG	0x02
#define BMA150_X_AXIS_MSB_REG	0x03
#define BMA150_Y_AXIS_LSB_REG	0x04
#define BMA150_Y_AXIS_MSB_REG	0x05
#define BMA150_Z_AXIS_LSB_REG	0x06
#define BMA150_Z_AXIS_MSB_REG	0x07
#define BMA150_STATUS_REG		0x09
#define BMA150_CTRL_REG			0x0a
#define BMA150_CONF1_REG		0x0b

#define BMA150_CUSTOMER1_REG	0x12
#define BMA150_CUSTOMER2_REG	0x13
#define BMA150_RANGE_BWIDTH_REG	0x14
#define BMA150_CONF2_REG		0x15

#define BMA150_OFFS_GAIN_X_REG	0x16
#define BMA150_OFFS_GAIN_Y_REG	0x17
#define BMA150_OFFS_GAIN_Z_REG	0x18
#define BMA150_OFFS_GAIN_T_REG	0x19
#define BMA150_OFFSET_X_REG		0x1a
#define BMA150_OFFSET_Y_REG		0x1b
#define BMA150_OFFSET_Z_REG		0x1c
#define BMA150_OFFSET_T_REG		0x1d

#define BMA150_CHIP_ID__POS		0
#define BMA150_CHIP_ID__MSK		0x07
#define BMA150_CHIP_ID__LEN		3
#define BMA150_CHIP_ID__REG		BMA150_CHIP_ID_REG

/* DATA REGISTERS */

#define BMA150_NEW_DATA_X__POS		0
#define BMA150_NEW_DATA_X__LEN		1
#define BMA150_NEW_DATA_X__MSK		0x01
#define BMA150_NEW_DATA_X__REG		BMA150_X_AXIS_LSB_REG

#define BMA150_ACC_X_LSB__POS		6
#define BMA150_ACC_X_LSB__LEN		2
#define BMA150_ACC_X_LSB__MSK		0xC0
#define BMA150_ACC_X_LSB__REG		BMA150_X_AXIS_LSB_REG

#define BMA150_ACC_X_MSB__POS		0
#define BMA150_ACC_X_MSB__LEN		8
#define BMA150_ACC_X_MSB__MSK		0xFF
#define BMA150_ACC_X_MSB__REG		BMA150_X_AXIS_MSB_REG

#define BMA150_ACC_Y_LSB__POS		6
#define BMA150_ACC_Y_LSB__LEN		2
#define BMA150_ACC_Y_LSB__MSK		0xC0
#define BMA150_ACC_Y_LSB__REG		BMA150_Y_AXIS_LSB_REG

#define BMA150_ACC_Y_MSB__POS		0
#define BMA150_ACC_Y_MSB__LEN		8
#define BMA150_ACC_Y_MSB__MSK		0xFF
#define BMA150_ACC_Y_MSB__REG		BMA150_Y_AXIS_MSB_REG

#define BMA150_ACC_Z_LSB__POS		6
#define BMA150_ACC_Z_LSB__LEN		2
#define BMA150_ACC_Z_LSB__MSK		0xC0
#define BMA150_ACC_Z_LSB__REG		BMA150_Z_AXIS_LSB_REG

#define BMA150_ACC_Z_MSB__POS		0
#define BMA150_ACC_Z_MSB__LEN		8
#define BMA150_ACC_Z_MSB__MSK		0xFF
#define BMA150_ACC_Z_MSB__REG		BMA150_Z_AXIS_MSB_REG

/* CONTROL BITS */

#define BMA150_SLEEP__POS			0
#define BMA150_SLEEP__LEN			1
#define BMA150_SLEEP__MSK			0x01
#define BMA150_SLEEP__REG			BMA150_CTRL_REG

#define BMA150_SOFT_RESET__POS		1
#define BMA150_SOFT_RESET__LEN		1
#define BMA150_SOFT_RESET__MSK		0x02
#define BMA150_SOFT_RESET__REG		BMA150_CTRL_REG

#define BMA150_EE_W__POS			4
#define BMA150_EE_W__LEN			1
#define BMA150_EE_W__MSK			0x10
#define BMA150_EE_W__REG			BMA150_CTRL_REG

#define BMA150_UPDATE_IMAGE__POS	5
#define BMA150_UPDATE_IMAGE__LEN	1
#define BMA150_UPDATE_IMAGE__MSK	0x20
#define BMA150_UPDATE_IMAGE__REG	BMA150_CTRL_REG

#define BMA150_RESET_INT__POS		6
#define BMA150_RESET_INT__LEN		1
#define BMA150_RESET_INT__MSK		0x40
#define BMA150_RESET_INT__REG		BMA150_CTRL_REG

/* BANDWIDTH dependend definitions */

#define BMA150_BANDWIDTH__POS				0
#define BMA150_BANDWIDTH__LEN				3
#define BMA150_BANDWIDTH__MSK				0x07
#define BMA150_BANDWIDTH__REG				BMA150_RANGE_BWIDTH_REG

/* RANGE */

#define BMA150_RANGE__POS				3
#define BMA150_RANGE__LEN				2
#define BMA150_RANGE__MSK				0x18
#define BMA150_RANGE__REG				BMA150_RANGE_BWIDTH_REG

/* WAKE UP */

#define BMA150_WAKE_UP__POS			0
#define BMA150_WAKE_UP__LEN			1
#define BMA150_WAKE_UP__MSK			0x01
#define BMA150_WAKE_UP__REG			BMA150_CONF2_REG

#define BMA150_WAKE_UP_PAUSE__POS		1
#define BMA150_WAKE_UP_PAUSE__LEN		2
#define BMA150_WAKE_UP_PAUSE__MSK		0x06
#define BMA150_WAKE_UP_PAUSE__REG		BMA150_CONF2_REG

#define BMA150_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA150_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* range and bandwidth */

#define BMA150_RANGE_2G			0
#define BMA150_RANGE_4G			1
#define BMA150_RANGE_8G			2

#define BMA150_BW_25HZ		0
#define BMA150_BW_50HZ		1
#define BMA150_BW_100HZ		2
#define BMA150_BW_190HZ		3
#define BMA150_BW_375HZ		4
#define BMA150_BW_750HZ		5
#define BMA150_BW_1500HZ	6

/* mode settings */

#define BMA150_MODE_NORMAL      0
#define BMA150_MODE_SLEEP       2
#define BMA150_MODE_WAKE_UP     3

struct bma150acc{
	s16	x,
		y,
		z;
} ;

struct bma150_data {
	struct i2c_client *bma150_client;
	struct bma150_platform_data *platform_data;
	int IRQ;
	atomic_t delay;
	unsigned char mode;
	struct input_dev *input;
	struct bma150acc value;
	struct mutex value_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
};

static int bma150_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -EPERM;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma150_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -EPERM;
	return 0;
}

static int bma150_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -EPERM;
	return 0;
}

static int bma150_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1 = 0, data2 = 0;
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	if (client == NULL) {
		comres = -1;
	} else{
		if (Mode < 4 && Mode != 1) {

			comres = bma150_smbus_read_byte(client,
						BMA150_WAKE_UP__REG, &data1);
			data1 = BMA150_SET_BITSLICE(data1,
						BMA150_WAKE_UP, Mode);
			comres += bma150_smbus_read_byte(client,
						BMA150_SLEEP__REG, &data2);
			data2 = BMA150_SET_BITSLICE(data2,
						BMA150_SLEEP, (Mode>>1));
			comres += bma150_smbus_write_byte(client,
						BMA150_WAKE_UP__REG, &data1);
			comres += bma150_smbus_write_byte(client,
						BMA150_SLEEP__REG, &data2);
			mutex_lock(&bma150->mode_mutex);
			bma150->mode = (unsigned char) Mode;
			mutex_unlock(&bma150->mode_mutex);

		} else{
			comres = -1;
		}
	}

	return comres;
}


static int bma150_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		if (Range < 3) {

			comres = bma150_smbus_read_byte(client,
						BMA150_RANGE__REG, &data);
			data = BMA150_SET_BITSLICE(data, BMA150_RANGE, Range);
			comres += bma150_smbus_write_byte(client,
						BMA150_RANGE__REG, &data);

		} else{
			comres = -1;
		}
	}

	return comres;
}

static int bma150_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma150_smbus_read_byte(client,
						BMA150_RANGE__REG, &data);

		*Range = BMA150_GET_BITSLICE(data, BMA150_RANGE);

	}

	return comres;
}



static int bma150_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		if (BW < 8) {
			comres = bma150_smbus_read_byte(client,
						BMA150_BANDWIDTH__REG, &data);
			data = BMA150_SET_BITSLICE(data, BMA150_BANDWIDTH, BW);
			comres += bma150_smbus_write_byte(client,
						BMA150_BANDWIDTH__REG, &data);

		} else{
			comres = -1;
		}
	}

	return comres;
}

static int bma150_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{


		comres = bma150_smbus_read_byte(client,
						BMA150_BANDWIDTH__REG, &data);

		*BW = BMA150_GET_BITSLICE(data, BMA150_BANDWIDTH);


	}

	return comres;
}

static int bma150_read_accel_xyz(struct i2c_client *client,
		struct bma150acc *acc)
{
	int comres;
	unsigned char data[6];
	if (client == NULL) {
		comres = -1;
	} else{


		comres = bma150_smbus_read_byte_block(client,
					BMA150_ACC_X_LSB__REG, &data[0], 6);

		acc->x = BMA150_GET_BITSLICE(data[0], BMA150_ACC_X_LSB) |
			(BMA150_GET_BITSLICE(data[1], BMA150_ACC_X_MSB)<<
							BMA150_ACC_X_LSB__LEN);
		acc->x = acc->x << (sizeof(short)*8-(BMA150_ACC_X_LSB__LEN+
							BMA150_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA150_ACC_X_LSB__LEN+
							BMA150_ACC_X_MSB__LEN));

		acc->y = BMA150_GET_BITSLICE(data[2], BMA150_ACC_Y_LSB) |
			(BMA150_GET_BITSLICE(data[3], BMA150_ACC_Y_MSB)<<
							BMA150_ACC_Y_LSB__LEN);
		acc->y = acc->y << (sizeof(short)*8-(BMA150_ACC_Y_LSB__LEN +
							BMA150_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA150_ACC_Y_LSB__LEN +
							BMA150_ACC_Y_MSB__LEN));


		acc->z = BMA150_GET_BITSLICE(data[4], BMA150_ACC_Z_LSB);
		acc->z |= (BMA150_GET_BITSLICE(data[5], BMA150_ACC_Z_MSB)<<
							BMA150_ACC_Z_LSB__LEN);
		acc->z = acc->z << (sizeof(short)*8-(BMA150_ACC_Z_LSB__LEN+
							BMA150_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA150_ACC_Z_LSB__LEN+
							BMA150_ACC_Z_MSB__LEN));

	}

	return comres;
}

static void bma150_work_func(struct work_struct *work)
{
	struct bma150_data *bma150 = container_of((struct delayed_work *)work,
			struct bma150_data, work);
	static struct bma150acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma150->delay));



	bma150_read_accel_xyz(bma150->bma150_client, &acc);
	input_report_abs(bma150->input, ABS_X, acc.x);
	input_report_abs(bma150->input, ABS_Y, acc.y);
	input_report_abs(bma150->input, ABS_Z, acc.z);
	input_sync(bma150->input);
	mutex_lock(&bma150->value_mutex);
	bma150->value = acc;
	mutex_unlock(&bma150->value_mutex);
	schedule_delayed_work(&bma150->work, delay);
}

static ssize_t bma150_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	mutex_lock(&bma150->mode_mutex);
	data = bma150->mode;
	mutex_unlock(&bma150->mode_mutex);

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma150_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma150_set_mode(bma150->bma150_client, (unsigned char) data) < 0)
		return -EINVAL;


	return count;
}
static ssize_t bma150_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	if (bma150_get_range(bma150->bma150_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma150_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma150_set_range(bma150->bma150_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma150_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	if (bma150_get_bandwidth(bma150->bma150_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma150_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma150_set_bandwidth(bma150->bma150_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma150_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma150_data *bma150 = input_get_drvdata(input);
	struct bma150acc acc_value;

	mutex_lock(&bma150->value_mutex);
	acc_value = bma150->value;
	mutex_unlock(&bma150->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}



static ssize_t bma150_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma150->delay));

}

static ssize_t bma150_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma150_data *bma150 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA150_MAX_DELAY)
		data = BMA150_MAX_DELAY;
	atomic_set(&bma150->delay, (unsigned int) data);

	return count;
}

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		bma150_range_show, bma150_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP,
		bma150_bandwidth_show, bma150_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma150_mode_show, bma150_mode_store);
static DEVICE_ATTR(value, S_IRUGO|S_IWUSR|S_IWGRP,
		bma150_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma150_delay_show, bma150_delay_store);

static struct attribute *bma150_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group bma150_attribute_group = {
	.attrs = bma150_attributes
};

static int bma150_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);

	return 0;
}

static int bma150_input_init(struct bma150_data *bma150)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma150);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	bma150->input = dev;

	return 0;
}

static void bma150_input_delete(struct bma150_data *bma150)
{
	struct input_dev *dev = bma150->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bma150_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma150_data *data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma150_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->platform_data = client->dev.platform_data;

	if (data->platform_data->power_on)
		data->platform_data->power_on();
	else
		printk(KERN_ERR "power_on function not defined!!\n");

	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client, BMA150_CHIP_ID_REG);

	if ((tempvalue&0x00FF) == BMA150_CHIP_ID) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\n" \
				"BMA150 registered I2C driver!\n");
	} else{
		printk(KERN_INFO "Bosch Sensortec Device not found" \
			"i2c error %d\n", tempvalue);
		err = -1;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma150_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	bma150_set_bandwidth(client, BMA150_BW_SET);
	bma150_set_range(client, BMA150_RANGE_SET);


	INIT_DELAYED_WORK(&data->work, bma150_work_func);
	atomic_set(&data->delay, BMA150_MAX_DELAY);
	err = bma150_input_init(data);
	if (err < 0)
		goto kfree_exit;

	err = sysfs_create_group(&data->input->dev.kobj,
			&bma150_attribute_group);
	if (err < 0)
		goto error_sysfs;

	schedule_delayed_work(&data->work,
			msecs_to_jiffies(atomic_read(&data->delay)));

	return 0;

error_sysfs:
	bma150_input_delete(data);

kfree_exit:
	kfree(data);
exit:
	return err;
}

static int bma150_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bma150_data *data = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&data->work);

	bma150_set_mode(client, BMA150_MODE_SLEEP);

	if ((data->platform_data) && (data->platform_data->power_off))
		data->platform_data->power_off();

	return 0;
}

static int bma150_resume(struct i2c_client *client)
{
	struct bma150_data *data = i2c_get_clientdata(client);

	if ((data->platform_data) && (data->platform_data->power_on))
		data->platform_data->power_on();

	bma150_set_mode(client, BMA150_MODE_NORMAL);

	schedule_delayed_work(&data->work,
		msecs_to_jiffies(atomic_read(&data->delay)));

	return 0;
}

static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *data = i2c_get_clientdata(client);

	if (data->platform_data->power_off)
		data->platform_data->power_off();
	else
		printk(KERN_ERR "power_off function not defined!!\n");

	sysfs_remove_group(&data->input->dev.kobj, &bma150_attribute_group);
	bma150_input_delete(data);
	free_irq(data->IRQ, data);
	kfree(data);

	return 0;
}

static const struct i2c_device_id bma150_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma150_id);

static struct i2c_driver bma150_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.class          = I2C_CLASS_HWMON,
	.id_table	= bma150_id,
	.probe		= bma150_probe,
	.remove		= bma150_remove,
	.detect		= bma150_detect,
	.suspend    = bma150_suspend,
	.resume     = bma150_resume,
};

static int __init BMA150_init(void)
{
	return i2c_add_driver(&bma150_driver);
}

static void __exit BMA150_exit(void)
{
	i2c_del_driver(&bma150_driver);
}

MODULE_DESCRIPTION("BMA150 driver");

module_init(BMA150_init);
module_exit(BMA150_exit);


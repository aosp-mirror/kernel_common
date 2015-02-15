/* drivers/i2c/chips/bma250.c - bma250 G-sensor driver
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

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/bma250.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include<linux/earlysuspend.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

/*#define EARLY_SUSPEND_BMA 1*/

#define D(x...) pr_info("[GSNR][BMA250] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA250 ERROR] " x)
#define DIF(x...) {\
		if (debug_flag)\
			printk(KERN_DEBUG "[GSNR][BMA250 DEBUG] " x); }

#define DEFAULT_RANGE	BMA_RANGE_2G
#define DEFAULT_BW	BMA_BW_31_25HZ

#define RETRY_TIMES	10

static struct i2c_client *this_client;

struct bma250_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
};

static struct bma250_platform_data *pdata;
static atomic_t PhoneOn_flag = ATOMIC_INIT(0);
#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)

static int debug_flag;
static char update_user_calibrate_data;

static int BMA_I2C_RxData(char *rxData, int length)
{
	int retry;
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

	for (retry = 0; retry <= RETRY_TIMES; retry++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;
		else
			mdelay(10);
	}

	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		return -EIO;
	} else
		return 0;
}

static int BMA_I2C_TxData(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (retry = 0; retry <= RETRY_TIMES; retry++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;
		else
			mdelay(10);
	}

	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		return -EIO;
	} else
		return 0;
}

static int BMA_Init(void)
{
	char buffer[4] = "";
	int ret;
	unsigned char range = 0, bw = 0;

	memset(buffer, 0, 4);

	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(buffer, 2);
	if (ret < 0)
		return -1;
	D("%s: bma250_RANGE_SEL_REG++: range = 0x%02x, bw = 0x%02x\n",
		__func__, buffer[0], buffer[1]);
	range = (buffer[0] & 0xF0) | DEFAULT_RANGE;
	bw = (buffer[1] & 0xE0) | DEFAULT_BW;

	/* Multiple write msgs */
	/*buffer[3] = bw;
	buffer[2] = bma250_BW_SEL_REG;
	buffer[1] = range;
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 4);
	if (ret < 0)
		return -1;*/

	buffer[1] = bw;
	buffer[0] = bma250_BW_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}

	buffer[1] = range;
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	if (ret < 0) {
		E("%s: Write bma250_BW_SEL_REG fail\n", __func__);
		return -1;
	}

	/* Debug use */
	buffer[0] = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(buffer, 2);
	if (ret < 0)
		return -1;

	D("%s: bma250_RANGE_SEL_REG:use single write--: range = 0x%02x, bw = 0x%02x\n",
		__func__, buffer[0], buffer[1]);

	return 0;

}

static int BMA_TransRBuff(short *rbuf)
{
	char buffer[6];
	int ret;

	memset(buffer, 0, 6);

	buffer[0] = bma250_X_AXIS_LSB_REG;
	ret = BMA_I2C_RxData(buffer, 6);
	if (ret < 0)
		return ret;

	/*D("%s: buffer(0, 1, 2, 3, 4, 5) = (0x%02x, 0x%02x, "
		"0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		__func__, buffer[0], buffer[1], buffer[2],
		buffer[3], buffer[4], buffer[5]);*/

	rbuf[0] = (short)(buffer[1] << 8 | buffer[0]);
	rbuf[0] >>= 6;
	rbuf[1] = (short)(buffer[3] << 8 | buffer[2]);
	rbuf[1] >>= 6;
	rbuf[2] = (short)(buffer[5] << 8 | buffer[4]);
	rbuf[2] >>= 6;

	DIF("%s: (x, y, z) = (%d, %d, %d)\n",
		__func__, rbuf[0], rbuf[1], rbuf[2]);

	return 1;
}

/* set  operation mode: 0 = normal, 1 = suspend */
static int BMA_set_mode(unsigned char mode)
{
	char buffer[2] = "";
	int ret = 0;
	unsigned char data1 = 0;

	printk(KERN_INFO "[GSNR] Gsensor %s\n", mode ? "disable" : "enable");

	memset(buffer, 0, 2);

	if (pdata->power_LPM && (mode < 2)) {
		switch (mode) {
		case bma250_MODE_NORMAL:
			pdata->power_LPM(0);
			break;
		default:
			break;
		}
	}

	D("%s: mode = 0x%02x\n", __func__, mode);
	if (mode < 2) {
		buffer[0] = bma250_MODE_CTRL_REG;
		ret = BMA_I2C_RxData(buffer, 1);
		if (ret < 0)
			return -1;
		/*D("%s: MODE_CTRL_REG++ = 0x%02x\n", __func__, buffer[0]);*/

		switch (mode) {
		case bma250_MODE_NORMAL:
			data1 = buffer[0] & 0x7F;
			break;
		case bma250_MODE_SUSPEND:
			data1 = buffer[0] | 0x80;
			break;
		default:
			break;
		}

		/*D("%s: data1 = 0x%02x\n", __func__, data1);*/
		buffer[0] = bma250_MODE_CTRL_REG;
		buffer[1] = data1;
		ret = BMA_I2C_TxData(buffer, 2);
	} else
		ret = E_OUT_OF_RANGE;

	if (mode == bma250_MODE_NORMAL)
		usleep(2000);
	/* Debug use */
	/*buffer[0] = bma250_MODE_CTRL_REG;
	ret = BMA_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;
	D("%s: MODE_CTRL_REG-- = 0x%02x\n", __func__, buffer[0]);*/
	if (pdata->power_LPM && (mode < 2)) {
		switch (mode) {
		case bma250_MODE_SUSPEND:
			pdata->power_LPM(1);
			break;
		default:
			break;
		}
	}

	return ret;
}

static int BMA_GET_INT(void)
{
	int ret;
	ret = gpio_get_value(pdata->intr);
	return ret;
}

static int bma_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int bma_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long bma_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{

	void __user *argp = (void __user *)arg;

	char rwbuf[8] = "";
	int ret = -1;
	short buf[8], temp;
	int kbuf = 0;

	DIF("%s: cmd = 0x%x\n", __func__, cmd);

	switch (cmd) {
	case BMA_IOCTL_READ:
	case BMA_IOCTL_WRITE:
	case BMA_IOCTL_SET_MODE:
	case BMA_IOCTL_SET_CALI_MODE:
	case BMA_IOCTL_SET_UPDATE_USER_CALI_DATA:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		if (copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case BMA_IOCTL_INIT:
		ret = BMA_Init();
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		/*ret = BMA_I2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;*/
		break;
	case BMA_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		/*ret = BMA_I2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;*/
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		pdata->gs_kvalue = kbuf;
		printk(KERN_INFO "%s: Write calibration value: 0x%X\n",
			__func__, pdata->gs_kvalue);
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		ret = BMA_TransRBuff(&buf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if ((pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
			rwbuf[0] = 0;
			rwbuf[1] = 0;
			rwbuf[2] = 0;
		} else {
			rwbuf[0] = (pdata->gs_kvalue >> 16) & 0xFF;
			rwbuf[1] = (pdata->gs_kvalue >>  8) & 0xFF;
			rwbuf[2] =  pdata->gs_kvalue        & 0xFF;
		}
		DIF("%s: CALI(x, y, z) = (%d, %d, %d)\n",
			__func__, rwbuf[0], rwbuf[1], rwbuf[2]);
		break;
	case BMA_IOCTL_SET_MODE:
		BMA_set_mode(rwbuf[0]);
		break;
	case BMA_IOCTL_GET_INT:
		temp = BMA_GET_INT();
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (pdata)
			temp = pdata->chip_layout;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (pdata)
			temp = pdata->calibration_mode;
		break;
	case BMA_IOCTL_SET_CALI_MODE:
		if (pdata)
			pdata->calibration_mode = rwbuf[0];
		break;
	case BMA_IOCTL_GET_UPDATE_USER_CALI_DATA:
		temp = update_user_calibrate_data;
		break;
	case BMA_IOCTL_SET_UPDATE_USER_CALI_DATA:
		update_user_calibrate_data = rwbuf[0];
		break;

	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case BMA_IOCTL_READ:
		/*if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;*/
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_UPDATE_USER_CALI_DATA:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

#ifdef EARLY_SUSPEND_BMA

static void bma250_early_suspend(struct early_suspend *handler)
{
	if (!atomic_read(&PhoneOn_flag))
		BMA_set_mode(bma250_MODE_SUSPEND);
	else
		printk(KERN_DEBUG "bma250_early_suspend: PhoneOn_flag is set\n");
}

static void bma250_late_resume(struct early_suspend *handler)
{
	BMA_set_mode(bma250_MODE_NORMAL);
}

#else /* EARLY_SUSPEND_BMA */

static int bma250_suspend(struct device *dev)
{
	BMA_set_mode(bma250_MODE_SUSPEND);

	return 0;
}

static int bma250_resume(struct device *dev)
{
	BMA_set_mode(bma250_MODE_NORMAL);
	return 0;
}
#endif /* EARLY_SUSPEND_BMA */

static ssize_t bma250_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", atomic_read(&PhoneOn_flag));
	return s - buf;
}

static ssize_t bma250_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		atomic_set(&PhoneOn_flag, 1);
		printk(KERN_DEBUG "bma250_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		atomic_set(&PhoneOn_flag, 0);
		printk(KERN_DEBUG "bma250_store: PhoneOn_flag=%d\n",
			atomic_read(&PhoneOn_flag));
		return count;
	}
	E("bma250_store: invalid argument\n");
	return -EINVAL;

}

static DEVICE_ACCESSORY_ATTR(PhoneOnOffFlag, 0664, \
	bma250_show, bma250_store);

static ssize_t debug_flag_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char buffer, range = -1, bandwidth = -1, mode = -1;
	int ret;

	buffer = bma250_BW_SEL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	bandwidth = (buffer & 0x1F);

	buffer = bma250_RANGE_SEL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	range = (buffer & 0xF);

	buffer = bma250_MODE_CTRL_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	mode = ((buffer & 0x80) >> 7);

	s += sprintf(s, "debug_flag = %d, range = 0x%x, bandwidth = 0x%x, "
		"mode = 0x%x\n", debug_flag, range, bandwidth, mode);

	return s - buf;
}
static ssize_t debug_flag_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	debug_flag = -1;
	sscanf(buf, "%d", &debug_flag);

	return count;

}

static DEVICE_ACCESSORY_ATTR(debug_en, 0664, \
	debug_flag_show, debug_flag_store);

int bma250_registerAttr(void)
{
	int ret;
	struct class *htc_accelerometer_class;
	struct device *accelerometer_dev;

	htc_accelerometer_class = class_create(THIS_MODULE,
					"htc_accelerometer");
	if (IS_ERR(htc_accelerometer_class)) {
		ret = PTR_ERR(htc_accelerometer_class);
		htc_accelerometer_class = NULL;
		goto err_create_class;
	}

	accelerometer_dev = device_create(htc_accelerometer_class,
				NULL, 0, "%s", "accelerometer");
	if (unlikely(IS_ERR(accelerometer_dev))) {
		ret = PTR_ERR(accelerometer_dev);
		accelerometer_dev = NULL;
		goto err_create_accelerometer_device;
	}

	/* register the attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
	if (ret)
		goto err_create_accelerometer_device_file;

	/* register the debug_en attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_debug_en);
	if (ret)
		goto err_create_accelerometer_debug_en_device_file;

	return 0;

err_create_accelerometer_debug_en_device_file:
	device_remove_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
err_create_accelerometer_device_file:
	device_unregister(accelerometer_dev);
err_create_accelerometer_device:
	class_destroy(htc_accelerometer_class);
err_create_class:

	return ret;
}

static const struct file_operations bma_fops = {
	.owner = THIS_MODULE,
	.open = bma_open,
	.release = bma_release,
	/*.ioctl = bma_ioctl,*/
#if HAVE_COMPAT_IOCTL
	.compat_ioctl = bma_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = bma_ioctl,
#endif
};

static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma_fops,
};

static int bma250_parse_dt(struct device *dev, struct bma250_platform_data *pdata)
{
	struct property *prop = NULL;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;
	uint32_t irq_gpio_flags = 0;

	if (pdata == NULL) {
		E("%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	/* TODO: Add LPM function */
	pdata->power_LPM = NULL;

	return 0;
}

int __devinit bma250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bma250_data *bma;
	char buffer[2];
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	bma = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!bma) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, bma);

	if (client->dev.of_node) {
		I("Device Tree parsing.");
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			err = -ENOMEM;
			dev_err(&client->dev, "%s: memory allocation "
					"for pdata failed.",
					__func__);
			goto exit_alloc_pdata_failed;
		}

		err = bma250_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "%s: bma250_parse_dt "
					"for pdata failed. err = %d",
					__func__, err);
			goto exit_wrong_ID;
		}
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			E("bma250_init_client: platform data is NULL\n");
			goto exit_platform_data_null;
		}
	}

	pdata->gs_kvalue = gs_kvalue;
	printk(KERN_INFO "BMA250 G-sensor I2C driver: gs_kvalue = 0x%X\n",
		pdata->gs_kvalue);

	this_client = client;

	buffer[0] = bma250_CHIP_ID_REG;
	err = BMA_I2C_RxData(buffer, 1);
	if (err < 0)
		goto exit_wrong_ID;
	D("%s: CHIP ID = 0x%02x\n", __func__, buffer[0]);
	if ((buffer[0] != 0x3) && (buffer[0] != 0xF9)) {
		E("Wrong chip ID of BMA250 or BMA250E!!\n");
		goto exit_wrong_ID;
	}

	err = BMA_Init();
	if (err < 0) {
		E("bma250_probe: bma_init failed\n");
		goto exit_init_failed;
	}

	err = misc_register(&bma_device);
	if (err) {
		E("bma250_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

#ifdef EARLY_SUSPEND_BMA
	bma->early_suspend.suspend = bma250_early_suspend;
	bma->early_suspend.resume = bma250_late_resume;
	register_early_suspend(&bma->early_suspend);
#endif

	err = bma250_registerAttr();
	if (err) {
		E("%s: set spi_bma150_registerAttr fail!\n", __func__);
		goto err_registerAttr;
	}
	D("%s: I2C retry 10 times version. OK\n", __func__);
	debug_flag = 0;

	return 0;

err_registerAttr:
exit_misc_device_register_failed:
exit_init_failed:
exit_wrong_ID:
	if (client->dev.of_node && pdata)
		kfree(pdata);
exit_platform_data_null:
	kfree(bma);
exit_alloc_pdata_failed:
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *bma = i2c_get_clientdata(client);
	kfree(bma);
	return 0;
}

static const struct dev_pm_ops bma250_pm_ops = {
#ifdef CONFIG_PM
	.suspend = bma250_suspend,
	.resume  = bma250_resume,
#endif
};

static const struct i2c_device_id bma250_id[] = {
	{ BMA250_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);
#ifdef CONFIG_OF
static struct of_device_id bma250_match_table[] = {
	{.compatible = "htc_g_sensor,bma250" },
	{},
};
#else
#define bma250_match_table NULL
#endif

static struct i2c_driver bma250_driver = {
#if 0
	.probe = bma250_probe,
	.remove = bma250_remove,
	.id_table	= bma250_id,

#ifndef EARLY_SUSPEND_BMA
	.suspend = bma250_suspend,
	.resume = bma250_resume,
#endif
	.driver = {
		   .name = BMA250_I2C_NAME,
		   },
#endif

	.driver = {
		.name           = BMA250_I2C_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = bma250_match_table,
#ifdef CONFIG_PM
		.pm             = &bma250_pm_ops,
#endif
	},
	.probe    = bma250_probe,
	.remove   = bma250_remove,
	.id_table = bma250_id,
};
module_i2c_driver(bma250_driver);

#if 0
static int __init bma250_init(void)
{
	printk(KERN_INFO "BMA250 G-sensor driver: init\n");
	return i2c_add_driver(&bma250_driver);
}

static void __exit bma250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

module_init(bma250_init);
module_exit(bma250_exit);
#endif

MODULE_DESCRIPTION("BMA250 G-sensor driver");
MODULE_LICENSE("GPL");


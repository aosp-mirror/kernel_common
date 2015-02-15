/*
 * Synaptics RMI4 touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_i2c_rmi4.h"

#define CHAR_DEVICE_NAME "rmi"
#define DEVICE_CLASS_NAME "rmidev"
#define DEV_NUMBER 1
#define REG_ADDR_LIMIT 0xFFFF

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_length_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_data_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidev_sysfs_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

struct rmidev_handle {
	dev_t dev_no;
	unsigned short address;
	unsigned int length;
	struct device dev;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct kobject *sysfs_dir;
	void *data;
};

struct rmidev_data {
	int ref_count;
	struct cdev main_dev;
	struct class *device_class;
	struct mutex file_mutex;
	struct rmidev_handle *rmi_dev;
};

static struct device_attribute attrs[] = {
	__ATTR(open, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_open_store),
	__ATTR(release, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_release_store),
	__ATTR(address, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_address_store),
	__ATTR(length, S_IWUSR | S_IWGRP,
			NULL,
			rmidev_sysfs_length_store),
	__ATTR(data, (S_IWUSR | S_IWGRP),
			rmidev_sysfs_data_show,
			rmidev_sysfs_data_store),
};

static int rmidev_major_num;

static struct class *rmidev_device_class;

static struct rmidev_handle *rmidev;

static struct completion remove_complete;

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input != 1)
		return -EINVAL;

	rmidev->fn_ptr->enable(rmidev->rmi4_data, false);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt disabled\n",
			__func__);

	return count;
}

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input != 1)
		return -EINVAL;

	rmidev->fn_ptr->enable(rmidev->rmi4_data, true);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt enabled\n",
			__func__);

	return count;
}

static ssize_t rmidev_sysfs_address_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input > REG_ADDR_LIMIT)
		return -EINVAL;

	rmidev->address = (unsigned short)input;

	return count;
}

static ssize_t rmidev_sysfs_length_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
			return -EINVAL;

	if (input > REG_ADDR_LIMIT)
		return -EINVAL;

	rmidev->length = input;

	return count;
}

static ssize_t rmidev_sysfs_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned int data_length = rmidev->length;

	if (data_length > (REG_ADDR_LIMIT - rmidev->address))
		data_length = REG_ADDR_LIMIT - rmidev->address;

	if (data_length) {
		retval = rmidev->fn_ptr->read(rmidev->rmi4_data,
				rmidev->address,
				(unsigned char *)buf,
				data_length);
		if (retval < 0) {
			dev_err(&rmidev->rmi4_data->i2c_client->dev,
					"%s: Failed to read data\n",
					__func__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return data_length;
}

static ssize_t rmidev_sysfs_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int data_length = rmidev->length;

	if (data_length > (REG_ADDR_LIMIT - rmidev->address))
		data_length = REG_ADDR_LIMIT - rmidev->address;

	if (data_length) {
		retval = rmidev->fn_ptr->write(rmidev->rmi4_data,
				rmidev->address,
				(unsigned char *)buf,
				data_length);
		if (retval < 0) {
			dev_err(&rmidev->rmi4_data->i2c_client->dev,
					"%s: Failed to write data\n",
					__func__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return data_length;
}

/*
 * rmidev_llseek - used to set up register address
 *
 * @filp: file structure for seek
 * @off: offset
 *   if whence == SEEK_SET,
 *     high 16 bits: page address
 *     low 16 bits: register address
 *   if whence == SEEK_CUR,
 *     offset from current position
 *   if whence == SEEK_END,
 *     offset from end position (0xFFFF)
 * @whence: SEEK_SET, SEEK_CUR, or SEEK_END
 */
static loff_t rmidev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct rmidev_data *dev_data = filp->private_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	mutex_lock(&(dev_data->file_mutex));

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = REG_ADDR_LIMIT + off;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
		dev_err(&rmidev->rmi4_data->i2c_client->dev,
				"%s: New position 0x%04x is invalid\n",
				__func__, (unsigned int)newpos);
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return newpos;
}

/*
 * rmidev_read: - use to read data from rmi device
 *
 * @filp: file structure for read
 * @buf: user space buffer pointer
 * @count: number of bytes to read
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char tmpbuf[count + 1];
	struct rmidev_data *dev_data = filp->private_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	if (count == 0)
		return 0;

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	mutex_lock(&(dev_data->file_mutex));

	retval = rmidev->fn_ptr->read(rmidev->rmi4_data,
			*f_pos,
			tmpbuf,
			count);
	if (retval < 0)
		goto clean_up;

	if (copy_to_user(buf, tmpbuf, count))
		retval = -EFAULT;
	else
		*f_pos += retval;

clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_write: - used to write data to rmi device
 *
 * @filep: file structure for write
 * @buf: user space buffer pointer
 * @count: number of bytes to write
 * @f_pos: offset (starting register address)
 */
static ssize_t rmidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char tmpbuf[count + 1];
	struct rmidev_data *dev_data = filp->private_data;

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	if (count == 0)
		return 0;

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (copy_from_user(tmpbuf, buf, count))
		return -EFAULT;

	mutex_lock(&(dev_data->file_mutex));

	retval = rmidev->fn_ptr->write(rmidev->rmi4_data,
			*f_pos,
			tmpbuf,
			count);
	if (retval >= 0)
		*f_pos += retval;

	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_open: enable access to rmi device
 * @inp: inode struture
 * @filp: file structure
 */
static int rmidev_open(struct inode *inp, struct file *filp)
{
	int retval = 0;
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);

	if (!dev_data)
		return -EACCES;

	filp->private_data = dev_data;

	mutex_lock(&(dev_data->file_mutex));

	rmidev->fn_ptr->enable(rmidev->rmi4_data, false);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt disabled\n",
			__func__);

	if (dev_data->ref_count < 1)
		dev_data->ref_count++;
	else
		retval = -EACCES;

	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_release: - release access to rmi device
 * @inp: inode structure
 * @filp: file structure
 */
static int rmidev_release(struct inode *inp, struct file *filp)
{
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);

	if (!dev_data)
		return -EACCES;

	mutex_lock(&(dev_data->file_mutex));

	dev_data->ref_count--;
	if (dev_data->ref_count < 0)
		dev_data->ref_count = 0;

	rmidev->fn_ptr->enable(rmidev->rmi4_data, true);
	dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
			"%s: Attention interrupt enabled\n",
			__func__);

	mutex_unlock(&(dev_data->file_mutex));

	return 0;
}

static const struct file_operations rmidev_fops = {
	.owner = THIS_MODULE,
	.llseek = rmidev_llseek,
	.read = rmidev_read,
	.write = rmidev_write,
	.open = rmidev_open,
	.release = rmidev_release,
};

static void rmidev_device_cleanup(struct rmidev_data *dev_data)
{
	dev_t devno;

	if (dev_data) {
		devno = dev_data->main_dev.dev;

		if (dev_data->device_class)
			device_destroy(dev_data->device_class, devno);

		cdev_del(&dev_data->main_dev);

		unregister_chrdev_region(devno, 1);

		dev_dbg(&rmidev->rmi4_data->i2c_client->dev,
				"%s: rmidev device removed\n",
				__func__);
	}

	return;
}

static char *rmi_char_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;

	*mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

static int rmidev_create_device_class(void)
{
	rmidev_device_class = class_create(THIS_MODULE, DEVICE_CLASS_NAME);

	if (IS_ERR(rmidev_device_class)) {
		pr_err("%s: Failed to create /dev/%s\n",
				__func__, CHAR_DEVICE_NAME);
		return -ENODEV;
	}

	rmidev_device_class->devnode = rmi_char_devnode;

	return 0;
}

static int rmidev_init_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	dev_t dev_no;
	unsigned char attr_count;
	struct rmidev_data *dev_data;
	struct device *device_ptr;

	rmidev = kzalloc(sizeof(*rmidev), GFP_KERNEL);
	if (!rmidev) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for rmidev\n",
				__func__);
		retval = -ENOMEM;
		goto err_rmidev;
	}

	rmidev->fn_ptr =  kzalloc(sizeof(*(rmidev->fn_ptr)), GFP_KERNEL);
	if (!rmidev) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto err_fn_ptr;
	}

	rmidev->fn_ptr->read = rmi4_data->i2c_read;
	rmidev->fn_ptr->write = rmi4_data->i2c_write;
	rmidev->fn_ptr->enable = rmi4_data->irq_enable;
	rmidev->rmi4_data = rmi4_data;

	retval = rmidev_create_device_class();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create device class\n",
				__func__);
		goto err_device_class;
	}

	if (rmidev_major_num) {
		dev_no = MKDEV(rmidev_major_num, DEV_NUMBER);
		retval = register_chrdev_region(dev_no, 1, CHAR_DEVICE_NAME);
	} else {
		retval = alloc_chrdev_region(&dev_no, 0, 1, CHAR_DEVICE_NAME);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to allocate char device region\n",
					__func__);
			goto err_device_region;
		}

		rmidev_major_num = MAJOR(dev_no);
		dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Major number of rmidev = %d\n",
				__func__, rmidev_major_num);
	}

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for dev_data\n",
				__func__);
		retval = -ENOMEM;
		goto err_dev_data;
	}

	mutex_init(&dev_data->file_mutex);
	dev_data->rmi_dev = rmidev;
	rmidev->data = dev_data;

	cdev_init(&dev_data->main_dev, &rmidev_fops);

	retval = cdev_add(&dev_data->main_dev, dev_no, 1);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to add rmi char device\n",
				__func__);
		goto err_char_device;
	}

	dev_set_name(&rmidev->dev, "rmidev%d", MINOR(dev_no));
	dev_data->device_class = rmidev_device_class;

	device_ptr = device_create(dev_data->device_class, NULL, dev_no,
			NULL, CHAR_DEVICE_NAME"%d", MINOR(dev_no));
	if (IS_ERR(device_ptr)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create rmi char device\n",
				__func__);
		retval = -ENODEV;
		goto err_char_device;
	}

	retval = gpio_export(rmi4_data->board->irq_gpio, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to export attention gpio\n",
				__func__);
	} else {
		retval = gpio_export_link(&(rmi4_data->input_dev->dev),
				"attn", rmi4_data->board->irq_gpio);
		if (retval < 0) {
			dev_err(&rmi4_data->input_dev->dev,
					"%s Failed to create gpio symlink\n",
					__func__);
		} else {
			dev_dbg(&rmi4_data->input_dev->dev,
					"%s: Exported attention gpio %d\n",
					__func__, rmi4_data->board->irq_gpio);
		}
	}

	rmidev->sysfs_dir = kobject_create_and_add("rmidev",
			&rmi4_data->input_dev->dev.kobj);
	if (!rmidev->sysfs_dir) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs directory\n",
				__func__);
		goto err_sysfs_dir;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(rmidev->sysfs_dir,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&rmi4_data->input_dev->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto err_sysfs_attrs;
		}
	}

	init_completion(&remove_complete);

	return 0;

err_sysfs_attrs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	kobject_put(rmidev->sysfs_dir);

err_sysfs_dir:
err_char_device:
	rmidev_device_cleanup(dev_data);
	kfree(dev_data);

err_dev_data:
	unregister_chrdev_region(dev_no, 1);

err_device_region:
	class_destroy(rmidev_device_class);

err_device_class:
	kfree(rmidev->fn_ptr);

err_fn_ptr:
	kfree(rmidev);

err_rmidev:
	return retval;
}

static void rmidev_remove_device(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;
	struct rmidev_data *dev_data;

	if (!rmidev)
		return;

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(rmidev->sysfs_dir, &attrs[attr_count].attr);

	kobject_put(rmidev->sysfs_dir);

	dev_data = rmidev->data;
	if (dev_data) {
		rmidev_device_cleanup(dev_data);
		kfree(dev_data);
	}

	unregister_chrdev_region(rmidev->dev_no, 1);

	class_destroy(rmidev_device_class);

	kfree(rmidev->fn_ptr);
	kfree(rmidev);

	complete(&remove_complete);

	return;
}

static int __init rmidev_module_init(void)
{
	synaptics_rmi4_new_function(RMI_DEV, true,
			rmidev_init_device,
			rmidev_remove_device,
			NULL);
	return 0;
}

static void __exit rmidev_module_exit(void)
{
	init_completion(&remove_complete);
	synaptics_rmi4_new_function(RMI_DEV, false,
			rmidev_init_device,
			rmidev_remove_device,
			NULL);
	wait_for_completion(&remove_complete);
	return;
}

module_init(rmidev_module_init);
module_exit(rmidev_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 RMI_Dev Module");
MODULE_LICENSE("GPL v2");

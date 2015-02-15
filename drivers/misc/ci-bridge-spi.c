/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/* This driver implements a simple SPI read/write interface to access
 * an external device over SPI.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/ci-bridge-spi.h>

#define CI_MAX_BUFFER_SIZE	(64 * 1024)

struct ci_bridge {
	dev_t ci_bridge_dev;
	struct cdev cdev;
	struct class *bridge_class;
	struct device *bridge_dev;
	char *write_buffer;
	char *read_buffer;
	struct mutex lock;
	struct spi_device *spi;
	unsigned int gpio_reset_pin;
	unsigned int gpio_interrupt_pin;
	int num_opened;

};

static struct ci_bridge ci;

static int __devinit ci_bridge_spi_probe(struct spi_device *spi)
{
	int ret;
	struct ci_bridge_platform_data *pdata;

	if (spi->dev.platform_data == NULL) {
		pr_err("%s: platform data is missing\n", __func__);
		return -EINVAL;
	}

	ci.spi = spi;
	ci.num_opened = 0;
	mutex_init(&ci.lock);
	spi_set_drvdata(spi, &ci);
	pdata = spi->dev.platform_data;
	ci.gpio_reset_pin = pdata->reset_pin;
	ci.gpio_interrupt_pin = pdata->interrupt_pin;

	ret = gpio_request(ci.gpio_reset_pin, "ci_bridge_spi");
	if (ret) {
		pr_err("%s: GPIO request for pin number %u failed\n",
			   __func__, ci.gpio_reset_pin);
		return ret;
	}
	ret = gpio_direction_output(ci.gpio_reset_pin, 1);
	if (ret) {
		pr_err("%s: unable to set GPIO direction, err=%d\n",
			  __func__, ret);
		goto err_free_reset_pin;
	}

	ret = gpio_request(ci.gpio_interrupt_pin, "ci_bridge_spi");
	if (ret) {
		pr_err("%s: GPIO request for pin number %u failed\n",
			   __func__, ci.gpio_interrupt_pin);
		goto err_free_reset_pin;
	}
	ret = gpio_direction_input(ci.gpio_interrupt_pin);
	if (ret) {
		pr_err("%s: unable to set GPIO direction, err=%d\n",
			   __func__, ret);
		goto err_free_int_pin;
	}

	return 0;

err_free_int_pin:
	gpio_free(ci.gpio_interrupt_pin);
err_free_reset_pin:
	gpio_free(ci.gpio_reset_pin);

	return ret;
}

static int __devexit ci_bridge_spi_remove(struct spi_device *spi)
{
	struct ci_bridge *bridge = spi_get_drvdata(spi);

	spi_set_drvdata(bridge->spi, NULL);
	bridge->spi = NULL;
	mutex_destroy(&ci.lock);

	gpio_free(ci.gpio_reset_pin);
	gpio_free(ci.gpio_interrupt_pin);

	return 0;
}

static struct spi_driver ci_bridge_driver = {
	.driver = {
		.name = "ci_bridge_spi",
		.owner = THIS_MODULE,
	},
	.probe = ci_bridge_spi_probe,
	.remove = __devexit_p(ci_bridge_spi_remove),
};

static void ci_bridge_spi_completion_cb(void *arg)
{
	complete(arg);
}

static ssize_t ci_bridge_spi_read(struct file *filp,
				char __user *buf,
				size_t count,
				loff_t *f_pos)
{
	int ret = 0;
	unsigned long not_copied = 0;
	struct spi_transfer spi_transfer;
	struct spi_message spi_message;
	DECLARE_COMPLETION_ONSTACK(context);
	struct ci_bridge *bridge = filp->private_data;

	if ((bridge == NULL) || (bridge->spi == NULL))
		return -ENODEV;

	if (count > CI_MAX_BUFFER_SIZE)
		return -EMSGSIZE;

	memset(&spi_transfer, 0, sizeof(struct spi_transfer));
	memset(&spi_message, 0, sizeof(struct spi_message));

	mutex_lock(&bridge->lock);

	spi_transfer.rx_buf = bridge->read_buffer;
	spi_transfer.len =  count;
	spi_message_init(&spi_message);
	spi_message_add_tail(&spi_transfer, &spi_message);
	spi_message.complete = ci_bridge_spi_completion_cb;
	spi_message.context = &context;

	/* must use spi_async in a context that may sleep */
	ret = spi_async(bridge->spi, &spi_message);
	if (ret == 0) {
		wait_for_completion(&context);

		if (spi_message.status == 0) {
			/* spi_message.actual_length should contain the number
			 * of bytes actually read and should update ret to be
			 * the actual length, but since our driver doesn't
			 * support this, assume all count bytes were read.
			 */
			ret = count;
		}

		if (ret > 0) {
			not_copied =
				copy_to_user(buf, bridge->read_buffer, ret);
			if (not_copied == ret)
				ret = -EFAULT;
			else
				ret -= not_copied;
		}
	} else {
		pr_err("%s: Error calling spi_async, ret = %d\n",
			__func__, ret);
	}

	mutex_unlock(&bridge->lock);

	return ret;
}

static ssize_t ci_bridge_spi_write(struct file *filp,
				const char __user *buf,
				size_t count,
				loff_t *f_pos)
{
	int ret = 0;
	unsigned long not_copied = 0;
	struct spi_transfer spi_transfer;
	struct spi_message spi_message;
	DECLARE_COMPLETION_ONSTACK(context);
	struct ci_bridge *bridge = filp->private_data;

	if ((bridge == NULL) || (bridge->spi == NULL))
		return -ENODEV;

	if (count > CI_MAX_BUFFER_SIZE)
		return -EMSGSIZE;

	memset(&spi_transfer, 0, sizeof(struct spi_transfer));
	memset(&spi_message, 0, sizeof(struct spi_message));

	mutex_lock(&bridge->lock);
	/* copy user data to our SPI Tx buffer */
	not_copied = copy_from_user(bridge->write_buffer, buf, count);
	if (not_copied != 0) {
		ret = -EFAULT;
	} else {
		spi_transfer.tx_buf = bridge->write_buffer;
		spi_transfer.len = count;

		spi_message_init(&spi_message);
		spi_message_add_tail(&spi_transfer, &spi_message);
		spi_message.complete = ci_bridge_spi_completion_cb;
		spi_message.context = &context;

		/* must use spi_async in a context that may sleep */
		ret = spi_async(bridge->spi, &spi_message);
		if (ret == 0) {
			wait_for_completion(&context);
			/* update ret to contain
			 * the number of bytes actually written
			 */
			if (spi_message.status == 0)
				ret = spi_transfer.len;
			else
				pr_err("%s: SPI transfer error, spi_message.status = %d\n",
					__func__, spi_message.status);
		} else {
			pr_err("%s: Error calling spi_async, ret = %d\n",
				__func__, ret);
		}
	}
	mutex_unlock(&bridge->lock);

	return ret;
}

static int ci_bridge_spi_open(struct inode *inode, struct file *filp)
{
	/* forbid opening more then one instance at a time,
	   parallel execution can still be problematic */
	if (ci.num_opened != 0)
		return -EBUSY;

	/* allocate write buffer */
	ci.write_buffer =
		kzalloc((CI_MAX_BUFFER_SIZE * sizeof(char)), GFP_KERNEL);
	if (ci.write_buffer == NULL) {
		pr_err("%s: Error allocating memory for write buffer\n",
			__func__);
		return -ENOMEM;
	}
	/* allocate read buffer */
	ci.read_buffer =
		kzalloc((CI_MAX_BUFFER_SIZE * sizeof(char)), GFP_KERNEL);
	if (ci.read_buffer == NULL) {
		pr_err("%s: Error allocating memory for read buffer\n",
			__func__);
		kfree(ci.write_buffer);
		return -ENOMEM;
	}
	/* device is non-seekable */
	nonseekable_open(inode, filp);

	filp->private_data = &ci;
	ci.num_opened = 1;

	return 0;
}

static int ci_bridge_ioctl_get_int(void *arg)
{
	int state;

	if (arg == NULL)
		return -EINVAL;

	state = gpio_get_value_cansleep(ci.gpio_interrupt_pin);
	if (copy_to_user(arg, &state, sizeof(state)))
		return -EFAULT;

	return 0;
}

static int ci_bridge_ioctl_reset(unsigned long arg)
{
	if ((arg != 0) && (arg != 1))
		return -EINVAL;

	gpio_set_value_cansleep(ci.gpio_reset_pin, arg);

	return 0;
}

static long ci_bridge_spi_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int ret;

	switch (cmd) {

	case CI_BRIDGE_IOCTL_RESET:
		ret = ci_bridge_ioctl_reset(arg);
		break;

	case CI_BRIDGE_IOCTL_GET_INT_STATE:
		ret = ci_bridge_ioctl_get_int((void *) arg);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ci_bridge_spi_release(struct inode *inode, struct file *filp)
{
	struct ci_bridge *bridge = filp->private_data;

	if ((bridge == NULL) || (bridge->spi == NULL))
		return -ENODEV;

	kfree(bridge->write_buffer);
	kfree(bridge->read_buffer);
	filp->private_data = NULL;
	ci.num_opened = 0;

	return 0;
}

static const struct file_operations ci_bridge_spi_fops = {
	.owner   = THIS_MODULE,
	.read    = ci_bridge_spi_read,
	.write   = ci_bridge_spi_write,
	.open    = ci_bridge_spi_open,
	.unlocked_ioctl = ci_bridge_spi_ioctl,
	.release = ci_bridge_spi_release,
	.llseek  = no_llseek,
};

static int __init ci_bridge_init(void)
{
	int ret = 0;

	ret = alloc_chrdev_region(&ci.ci_bridge_dev, 0, 1, "ci_bridge_spi");
	if (ret != 0)
		return ret;

	ci.bridge_class = class_create(THIS_MODULE, "ci_bridge_spi");
	if (IS_ERR(ci.bridge_class)) {
		ret = PTR_ERR(ci.bridge_class);
		pr_err("Error creating ci.bridge_class: %d\n", ret);
		goto free_region;
	}

	cdev_init(&ci.cdev, &ci_bridge_spi_fops);
	ci.cdev.owner = THIS_MODULE;
	ret = cdev_add(&ci.cdev, ci.ci_bridge_dev, 1);
	if (ret != 0) {
		pr_err("Error calling cdev_add: %d\n", ret);
		goto class_destroy;
	}


	ci.bridge_dev = device_create(ci.bridge_class, NULL, ci.cdev.dev,
				     &ci, "ci_bridge_spi0");
	if (IS_ERR(ci.bridge_dev)) {
		ret = PTR_ERR(ci.bridge_dev);
		pr_err("device_create failed: %d\n", ret);
		goto del_cdev;
	}

	ret = spi_register_driver(&ci_bridge_driver);
	if (ret != 0) {
		pr_err("Error registering spi driver: %d\n", ret);
		goto device_destroy;
	}

	/* successful return */
	return 0;

device_destroy:
	device_destroy(ci.bridge_class, ci.ci_bridge_dev);

del_cdev:
	cdev_del(&ci.cdev);

class_destroy:
	class_destroy(ci.bridge_class);

free_region:
	unregister_chrdev_region(ci.ci_bridge_dev, 1);

	return ret;
}

static void __exit ci_bridge_exit(void)
{
	spi_unregister_driver(&ci_bridge_driver);
	device_destroy(ci.bridge_class, ci.ci_bridge_dev);
	cdev_del(&ci.cdev);
	class_destroy(ci.bridge_class);
	unregister_chrdev_region(ci.ci_bridge_dev, 1);
}

module_init(ci_bridge_init);
module_exit(ci_bridge_exit);

MODULE_DESCRIPTION("CI Bridge SPI Driver");
MODULE_LICENSE("GPL v2");


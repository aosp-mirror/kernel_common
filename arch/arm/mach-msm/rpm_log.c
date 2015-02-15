/* Copyright (c) 2010-2011, 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/uaccess.h>

#include <mach/msm_iomap.h>

#include "rpm_log.h"

enum {
	MSM_RPM_LOG_TAIL,
	MSM_RPM_LOG_HEAD
};

#define PADDED_LENGTH(x) (0xFFFFFFFC & ((x) + 3))

#define PRINTED_LENGTH(x) ((x) * 6 + 3)

#define RECHECK_TIME (50)

#define VERSION_8974 0x1000
#define RPM_ULOG_LENGTH_SHIFT 16
#define RPM_ULOG_LENGTH_MASK  0xFFFF0000

struct msm_rpm_log_buffer {
	char *data;
	u32 len;
	u32 pos;
	struct mutex mutex;
	u32 max_len;
	u32 read_idx;
	struct msm_rpm_log_platform_data *pdata;
};


static inline u32
msm_rpm_log_read(const struct msm_rpm_log_platform_data *pdata, u32 page,
		 u32 reg)
{
	return readl_relaxed(pdata->reg_base + pdata->reg_offsets[page]
				+ reg * 4);
}

/*
 * msm_rpm_log_copy() - Copies messages from a volatile circular buffer in
 *			the RPM's shared memory into a private local buffer
 * msg_buffer:		pointer to local buffer (string)
 * buf_len:		length of local buffer in bytes
 * read_start_idx:	index into shared memory buffer
 *
 * Return value:	number of bytes written to the local buffer
 *
 * Copies messages stored in a circular buffer in the RPM Message Memory into
 * a specified local buffer.  The RPM processor is unaware of these reading
 * efforts, so care is taken to make sure that messages are valid both before
 * and after reading.  The RPM processor utilizes a ULog driver to write the
 * log.  The RPM processor maintains tail and head indices.  These correspond
 * to the next byte to write into, and the first valid byte, respectively.
 * Both indices increase monotonically (except for rollover).
 *
 * Messages take the form of [(u32)length] [(char)data0,1,...] in which the
 * length specifies the number of payload bytes.  Messages must be 4 byte
 * aligned, so padding is added at the end of a message as needed.
 *
 * Print format:
 * - 0xXX, 0xXX, 0xXX
 * - 0xXX
 * etc...
 */
static u32 msm_rpm_log_copy(const struct msm_rpm_log_platform_data *pdata,
			    char *msg_buffer, u32 buf_len, u32 *read_idx)
{
	u32 head_idx, tail_idx;
	u32 pos = 0;
	u32 i = 0;
	u32 msg_len;
	u32 pos_start;
	char temp[4];

	tail_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
				    MSM_RPM_LOG_TAIL);
	head_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
				    MSM_RPM_LOG_HEAD);

	
	while (tail_idx - head_idx > 0 && tail_idx - *read_idx > 0) {
		head_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
					    MSM_RPM_LOG_HEAD);
		tail_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
				    MSM_RPM_LOG_TAIL);
		
		if (tail_idx - *read_idx > tail_idx - head_idx) {
			*read_idx = head_idx;
			continue;
		}

		if (!IS_ALIGNED((tail_idx | head_idx | *read_idx), 4))
			break;

		msg_len = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_BUFFER,
				((*read_idx) & pdata->log_len_mask) >> 2);

		if (pdata->version == VERSION_8974) {
			msg_len = (msg_len & RPM_ULOG_LENGTH_MASK) >>
					RPM_ULOG_LENGTH_SHIFT;
			msg_len -= 4;
		}

		
		if (PADDED_LENGTH(msg_len) > tail_idx - *read_idx - 4)
			msg_len = tail_idx - *read_idx - 4;

		
		if (pos + PRINTED_LENGTH(msg_len) > buf_len)
			break;

		pos_start = pos;
		pos += scnprintf(msg_buffer + pos, buf_len - pos, "- ");

		
		for (i = 0; i < msg_len; i++) {
			
			if (IS_ALIGNED(i, 4))
				*((u32 *)temp) = msm_rpm_log_read(pdata,
						MSM_RPM_LOG_PAGE_BUFFER,
						((*read_idx + 4 + i) &
						pdata->log_len_mask) >> 2);

			pos += scnprintf(msg_buffer + pos, buf_len - pos,
					 "0x%02X, ", temp[i & 0x03]);
		}

		pos += scnprintf(msg_buffer + pos, buf_len - pos, "\n");

		head_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
					    MSM_RPM_LOG_HEAD);
		tail_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
				    MSM_RPM_LOG_TAIL);

		
		if (tail_idx - *read_idx > tail_idx - head_idx)
			pos = pos_start;

		*read_idx += PADDED_LENGTH(msg_len) + 4;
	}

	return pos;
}


static ssize_t msm_rpm_log_file_read(struct file *file, char __user *bufu,
				     size_t count, loff_t *ppos)
{
	u32 out_len, remaining;
	struct msm_rpm_log_platform_data *pdata;
	struct msm_rpm_log_buffer *buf;

	buf = file->private_data;

	if (!buf)
		return -ENOMEM;

	pdata = buf->pdata;

	if (!pdata)
		return -EINVAL;
	if (!buf->data)
		return -ENOMEM;
	if (!bufu || count == 0)
		return -EINVAL;
	if (!access_ok(VERIFY_WRITE, bufu, count))
		return -EFAULT;

	mutex_lock(&buf->mutex);
	
	if (buf->pos == buf->len) {
		buf->pos = 0;
		buf->len = msm_rpm_log_copy(pdata, buf->data, buf->max_len,
						&(buf->read_idx));
	}

	if ((file->f_flags & O_NONBLOCK) && buf->len == 0) {
		mutex_unlock(&buf->mutex);
		return -EAGAIN;
	}

	
	while (buf->len == 0) {
		cond_resched();
		if (msleep_interruptible(RECHECK_TIME))
			break;
		buf->len = msm_rpm_log_copy(pdata, buf->data, buf->max_len,
						&(buf->read_idx));
	}

	out_len = ((buf->len - buf->pos) < count ? buf->len - buf->pos : count);

	remaining = __copy_to_user(bufu, &(buf->data[buf->pos]), out_len);
	buf->pos += out_len - remaining;
	mutex_unlock(&buf->mutex);

	return out_len - remaining;
}


static int msm_rpm_log_file_open(struct inode *inode, struct file *file)
{
	struct msm_rpm_log_buffer *buf;
	struct msm_rpm_log_platform_data *pdata;

	pdata = inode->i_private;
	if (!pdata)
		return -EINVAL;

	file->private_data =
		   kmalloc(sizeof(struct msm_rpm_log_buffer), GFP_KERNEL);
	if (!file->private_data) {
		pr_err("%s: ERROR kmalloc failed to allocate %d bytes\n",
			__func__, sizeof(struct msm_rpm_log_buffer));
		return -ENOMEM;
	}
	buf = file->private_data;

	buf->data = kmalloc(PRINTED_LENGTH(pdata->log_len), GFP_KERNEL);
	if (!buf->data) {
		kfree(file->private_data);
		file->private_data = NULL;
		pr_err("%s: ERROR kmalloc failed to allocate %d bytes\n",
			__func__, PRINTED_LENGTH(pdata->log_len));
		return -ENOMEM;
	}

	buf->pdata = pdata;
	buf->len = 0;
	buf->pos = 0;
	mutex_init(&buf->mutex);
	buf->max_len = PRINTED_LENGTH(pdata->log_len);
	buf->read_idx = msm_rpm_log_read(pdata, MSM_RPM_LOG_PAGE_INDICES,
					 MSM_RPM_LOG_HEAD);
	return 0;
}

static int msm_rpm_log_file_close(struct inode *inode, struct file *file)
{
	kfree(((struct msm_rpm_log_buffer *)file->private_data)->data);
	kfree(file->private_data);
	return 0;
}


static const struct file_operations msm_rpm_log_file_fops = {
	.owner   = THIS_MODULE,
	.open    = msm_rpm_log_file_open,
	.read    = msm_rpm_log_file_read,
	.release = msm_rpm_log_file_close,
};

static int __devinit msm_rpm_log_probe(struct platform_device *pdev)
{
	struct dentry *dent;
	struct msm_rpm_log_platform_data *pdata;
	struct resource *res = NULL;
	struct device_node *node = NULL;
	phys_addr_t page_buffer_address, rpm_addr_phys;
	int ret = 0;
	char *key = NULL;
	uint32_t val = 0;

	node = pdev->dev.of_node;

	if (node) {
		pdata = kzalloc(sizeof(struct msm_rpm_log_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			kfree(pdata);
			return -EINVAL;
		}

		pdata->phys_addr_base = res->start;
		pdata->phys_size = resource_size(res);

		pdata->reg_base = ioremap_nocache(pdata->phys_addr_base,
					pdata->phys_size);
		if (!pdata->reg_base) {
			pr_err("%s: ERROR could not ioremap: start=%p, len=%u\n",
				__func__, (void *) pdata->phys_addr_base,
				pdata->phys_size);
			kfree(pdata);
			return -EBUSY;
		}

		key = "qcom,offset-version";
		ret = of_property_read_u32(node, key, &val);
		if (ret) {
			pr_err("%s: Error in name %s key %s\n",
				__func__, node->full_name, key);
			ret = -EFAULT;
			goto fail;
		}

		pdata->version = readl_relaxed(pdata->reg_base + val);
		if (pdata->version == VERSION_8974) {
			key = "qcom,rpm-addr-phys";
			ret = of_property_read_u32(node, key, &val);
			if (ret) {
				pr_err("%s: Error in name %s key %s\n",
					__func__, node->full_name, key);
				ret = -EFAULT;
				goto fail;
			}

			rpm_addr_phys = val;

			key = "qcom,offset-page-buffer-addr";
			ret = of_property_read_u32(node, key, &val);
			if (ret) {
				pr_err("%s: Error in name %s key %s\n",
					__func__, node->full_name, key);
				ret = -EFAULT;
				goto fail;
			}

			page_buffer_address = rpm_addr_phys +
				readl_relaxed(pdata->reg_base + val);
			pdata->reg_offsets[MSM_RPM_LOG_PAGE_BUFFER] =
				page_buffer_address - pdata->phys_addr_base;

			key = "qcom,offset-log-len";
			ret = of_property_read_u32(node, key, &val);
			if (ret) {
				pr_err("%s: Error in name %s key %s\n",
					__func__, node->full_name, key);
				ret = -EFAULT;
				goto fail;
			}
			pdata->log_len = readl_relaxed(pdata->reg_base + val);

			if (pdata->log_len > pdata->phys_size) {
				pr_err("%s: Error phy size: %d should be atleast log length: %d\n",
					__func__, pdata->phys_size,
					pdata->log_len);

				ret = -EINVAL;
				goto fail;
			}

			key = "qcom,offset-log-len-mask";
			ret = of_property_read_u32(node, key, &val);
			if (ret) {
				pr_err("%s: Error in name %s key %s\n",
					__func__, node->full_name, key);
				ret = -EFAULT;
				goto fail;
			}
			pdata->log_len_mask = readl_relaxed(pdata->reg_base
					+ val);

			key = "qcom,offset-page-indices";
			ret = of_property_read_u32(node, key, &val);
			if (ret) {
				pr_err("%s: Error in name %s key %s\n",
					__func__, node->full_name, key);
				ret = -EFAULT;
				goto fail;
			}
			pdata->reg_offsets[MSM_RPM_LOG_PAGE_INDICES] =
						val;
		} else{
			ret = -EINVAL;
			goto fail;
		}

	} else{
		pdata = pdev->dev.platform_data;
		if (!pdata)
			return -EINVAL;

		pdata->reg_base = ioremap(pdata->phys_addr_base,
				pdata->phys_size);
		if (!pdata->reg_base) {
			pr_err("%s: ERROR could not ioremap: start=%p, len=%u\n",
				__func__, (void *) pdata->phys_addr_base,
				pdata->phys_size);
			return -EBUSY;
		}
	}

	dent = debugfs_create_file("rpm_log", S_IRUGO, NULL,
			pdata, &msm_rpm_log_file_fops);
	if (!dent) {
		pr_err("%s: ERROR debugfs_create_file failed\n", __func__);
		if (pdata->version == VERSION_8974) {
			ret = -ENOMEM;
			goto fail;
		}
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, dent);

	pr_notice("%s: OK\n", __func__);
	return 0;

fail:
	iounmap(pdata->reg_base);
	kfree(pdata);
	return ret;
}

static int __devexit msm_rpm_log_remove(struct platform_device *pdev)
{
	struct dentry *dent;
	struct msm_rpm_log_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	iounmap(pdata->reg_base);

	dent = platform_get_drvdata(pdev);
	debugfs_remove(dent);
	platform_set_drvdata(pdev, NULL);

	pr_notice("%s: OK\n", __func__);
	return 0;
}

static struct of_device_id rpm_log_table[] = {
	       {.compatible = "qcom,rpm-log"},
	       {},
};

static struct platform_driver msm_rpm_log_driver = {
	.probe		= msm_rpm_log_probe,
	.remove		= __devexit_p(msm_rpm_log_remove),
	.driver		= {
		.name = "msm_rpm_log",
		.owner = THIS_MODULE,
		.of_match_table = rpm_log_table,
	},
};

static int __init msm_rpm_log_init(void)
{
	return platform_driver_register(&msm_rpm_log_driver);
}

static void __exit msm_rpm_log_exit(void)
{
	platform_driver_unregister(&msm_rpm_log_driver);
}

module_init(msm_rpm_log_init);
module_exit(msm_rpm_log_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM RPM Log driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_rpm_log");

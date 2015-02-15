/* arch/arm/mach-msm/htc_rmtmessage.c
 *
 * Copyright (C) 2012 HTC Corporation.
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <mach/scm.h>

#define DEVICE_NAME "htc_msgservice"

#define HTC_IOCTL_MSGSERVICE 0x2266

#define HTC_MSGSERVICE_DEBUG	0
#if HTC_MSGSERVICE_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)
#else
#define PDEBUG(fmt, args...) do {} while (0)
#endif /* HTC_MSGSERVICE_DEBUG */

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

static int32_t htc_msgservice_major;
static struct class *htc_msgservice_class;
static const struct file_operations htc_msgservice_fops;

static uint8_t *htc_rmt_msg;

typedef struct _htc_msgservice_s {
	int32_t 	func;
	uint8_t 	*req_buf;
	int32_t 	req_len;
} htc_msgservcie_t;

typedef struct _htc_remote_msg_s {
	uint8_t 	clear_imei[16];
	uint8_t		enc_remote_msg[32];
	/* The format of encrypted message
	uint32_t	magic;
	uint32_t	op_cmd;
	uint8_t		imei[16];
	uint8_t		reserve[8];
	*/
} htc_remote_msg_t;

static long htc_msgservice_ioctl(struct file *file, uint32_t command, unsigned long arg)
{
	htc_msgservcie_t mmsg;
	int32_t ret = 0;
	int32_t ii;
	/*
	if (!capable(CAP_SYS_ADMIN)) {
		return -EPERM;
	}
	*/
	PDEBUG("command = %x\n", command);
	switch (command) {
	case HTC_IOCTL_MSGSERVICE:
		if (copy_from_user(&mmsg, (void __user *)arg, sizeof(htc_msgservcie_t))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}
		PDEBUG("func = %x\n", mmsg.func);

		switch (mmsg.func) {
		case ITEM_REMOTE_MSG:
			if ((mmsg.req_buf == NULL) || (mmsg.req_len != sizeof(htc_remote_msg_t))) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			if (copy_from_user(htc_rmt_msg, (void __user *)mmsg.req_buf, mmsg.req_len)) {
				PERR("copy_from_user error (sdkey)");
				return -EFAULT;
			}
			PDEBUG("Input message:");
			for (ii = 0; ii < sizeof(htc_remote_msg_t); ii++) {
				PDEBUG("%02x", *((uint8_t *)htc_rmt_msg + ii));
			}
			scm_flush_range((uint32_t)htc_rmt_msg, (uint32_t)(htc_rmt_msg) + mmsg.req_len);
			ret = secure_access_item(1, ITEM_REMOTE_MSG, mmsg.req_len, htc_rmt_msg);
			PERR("Do nothing! (%d)\n", ret);
			break;

		default:
			PERR("func error\n");
			return -EFAULT;
		}
		break;

	default:
		PERR("command error\n");
		return -EFAULT;
	}
	return ret;
}

static int htc_msgservice_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int htc_msgservice_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations htc_msgservice_fops = {
	.unlocked_ioctl = htc_msgservice_ioctl,
	.open = htc_msgservice_open,
	.release = htc_msgservice_release,
	.owner = THIS_MODULE,
};

static int __init htc_msgservice_init(void)
{
	int32_t ret;

	htc_rmt_msg = kzalloc(sizeof(htc_remote_msg_t), GFP_KERNEL);
	if (htc_rmt_msg == NULL) {
		PERR("allocate the space for remote message failed");
		return -1;
	}

	ret = register_chrdev(0, DEVICE_NAME, &htc_msgservice_fops);
	if (ret < 0) {
		kfree(htc_rmt_msg);
		PERR("register module fail\n");
		return ret;
	}
	htc_msgservice_major = ret;

	htc_msgservice_class = class_create(THIS_MODULE, "htc_msgservice");
	device_create(htc_msgservice_class, NULL, MKDEV(htc_msgservice_major, 0), NULL, DEVICE_NAME);

	PDEBUG("register module ok\n");
	return 0;
}


static void  __exit htc_msgservice_exit(void)
{
	device_destroy(htc_msgservice_class, MKDEV(htc_msgservice_major, 0));
	class_unregister(htc_msgservice_class);
	class_destroy(htc_msgservice_class);
	unregister_chrdev(htc_msgservice_major, DEVICE_NAME);
	kfree(htc_rmt_msg);
	PDEBUG("un-registered module ok\n");
}

module_init(htc_msgservice_init);
module_exit(htc_msgservice_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tommy Chiu");


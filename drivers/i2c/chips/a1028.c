/* drivers/i2c/chips/a1028.c - a1028 voice processor driver
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/interrupt.h>
#include <linux/module.h>
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
#include <linux/a1028.h>
#include <mach/msm_xo.h>
#define DEBUG			(0)
#define ENABLE_DIAG_IOCTLS	(1)

/* support at most 1024 set of commands */
#define PARAM_MAX	(sizeof(char) * 6 * 1024)

static struct i2c_client *this_client;
static struct a1028_platform_data *pdata;

static int execute_cmdmsg(unsigned int);

static struct mutex a1028_lock;
static int a1028_opened;
static int a1028_suspended;
static int control_a1028_clk;
static int control_a1028_xo_clk;
static int control_a1028_micsel;
static int a1028_NS_state = A1028_NS_STATE_AUTO;
static int a1028_current_config = A1028_PATH_SUSPEND;
static int a1028_param_ID;
static char *config_data;
static int a1028_cmds_len;

struct vp_ctxt {
	unsigned char *data;
	unsigned int img_size;
};

struct vp_ctxt the_vp;

static struct msm_xo_voter *a1028_clock;
static const char *cid = "A1028";
static int a1028_xo_clk_enable(int en)
{
	int rc = 0;
	if (a1028_clock != NULL) {
		if (en)
			rc = msm_xo_mode_vote(a1028_clock, MSM_XO_MODE_ON);
		else
			rc = msm_xo_mode_vote(a1028_clock, MSM_XO_MODE_OFF);
		if (rc < 0) {
			pr_err("Configuring MSM_XO_MODE_ON failed"
						" (%d)\n", rc);
			goto fail;
		}
	} else
		pr_err("a1028_clock is null\n");
	return rc;
fail:
	msm_xo_put(a1028_clock);
	return rc;
}

static void a1028_gpio_set_value(int gpio, int value)
{
	if (gpio >  0)
		gpio_set_value(gpio, value);
}

static void a1028_pmic_set_value(int gpio, int value)
{
/*
	if (gpio >  0)
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(gpio), value);
*/
}

static int a1028_i2c_read(char *rxData, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: rx[%d] = %2x\n", \
				__func__, i, rxData[i]);
	}
#endif

	return 0;
}

static int a1028_i2c_write(char *txData, int length)
{
	int rc;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msg, 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: tx[%d] = %2x\n", \
				__func__, i, txData[i]);
	}
#endif

	return 0;
}

static int a1028_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct vp_ctxt *vp = &the_vp;

	mutex_lock(&a1028_lock);

	if (a1028_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	file->private_data = vp;
	vp->img_size = 0;
	a1028_opened = 1;
done:
	mutex_unlock(&a1028_lock);
	return rc;
}

static int a1028_release(struct inode *inode, struct file *file)
{
	mutex_lock(&a1028_lock);
	a1028_opened = 0;
	mutex_unlock(&a1028_lock);

	return 0;
}

static void a1028_i2c_sw_reset(unsigned int reset_cmd)
{
	int rc = 0;
	unsigned char msgbuf[4];

	msgbuf[0] = (reset_cmd >> 24) & 0xFF;
	msgbuf[1] = (reset_cmd >> 16) & 0xFF;
	msgbuf[2] = (reset_cmd >> 8) & 0xFF;
	msgbuf[3] = reset_cmd & 0xFF;

	pr_info("%s: %08x\n", __func__, reset_cmd);

	rc = a1028_i2c_write(msgbuf, 4);
	if (!rc)
		msleep(20);
}

static ssize_t a1028_bootup_init(struct file *file, struct a1028img *img)
{
	struct vp_ctxt *vp = file->private_data;
	int rc, pass = 0;
	int remaining;
	int retry = RETRY_CNT;
	unsigned char *index;
	char buf[2];

	if (img->img_size > A1028_MAX_FW_SIZE) {
		pr_err("%s: invalid a1028 image size %d\n", __func__,
			img->img_size);
		return -EINVAL;
	}

	vp->data = kmalloc(img->img_size, GFP_KERNEL);
	if (!vp->data) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}
	vp->img_size = img->img_size;
	if (copy_from_user(vp->data, img->buf, img->img_size)) {
		pr_err("%s: copy from user failed\n", __func__);
		kfree(vp->data);
		return -EFAULT;
	}

	while (retry--) {
		/* Reset A1028 chip */
		a1028_gpio_set_value(pdata->gpio_a1028_reset, 0);

		/* Enable A1028 clock */
		if (control_a1028_clk) {
			a1028_gpio_set_value(pdata->gpio_a1028_clk, 1);
			mdelay(1);
		}
		if (control_a1028_xo_clk) {
			a1028_xo_clk_enable(1);
			mdelay(1);
		}
		/* Take out of reset */
		a1028_gpio_set_value(pdata->gpio_a1028_reset, 1);

		msleep(50); /* Delay before send I2C command */

		/* Boot Cmd to A1028 */
		buf[0] = A1028_msg_BOOT >> 8;
		buf[1] = A1028_msg_BOOT & 0xff;

		rc = a1028_i2c_write(buf, 2);
		if (rc < 0) {
			pr_err("%s: set boot mode error \
				(%d retries left)\n", __func__, retry);
			continue;
		}

		mdelay(1); /* use polling */
		rc = a1028_i2c_read(buf, 1);
		if (rc < 0) {
			pr_err("%s: boot mode ack error \
				(%d retries left)\n", __func__, retry);
			continue;
		}

		if (buf[0] != A1028_msg_BOOT_ACK) {
			pr_err("%s: not a boot-mode ack \
				(%d retries left)\n", __func__, retry);
			continue;
		}

		remaining = vp->img_size / 32;
		index = vp->data;

		pr_info("%s: starting to load image (%d passes)...\n",
			__func__,
			remaining + !!(vp->img_size % 32));

		for (; remaining; remaining--, index += 32) {
			rc = a1028_i2c_write(index, 32);
			if (rc < 0)
				break;
		}

		if (rc >= 0 && vp->img_size % 32)
			rc = a1028_i2c_write(index, vp->img_size % 32);

		if (rc < 0) {
			pr_err("%s: fw load error %d (%d retries left)\n",
				__func__, rc, retry);
			continue;
		}

		msleep(20); /* Delay time before issue a Sync Cmd */

		pr_info("%s: firmware loaded successfully\n", __func__);

		rc = execute_cmdmsg(A100_msg_Sync);
		if (rc < 0) {
			pr_err("%s: sync command error %d \
				(%d retries left)\n", __func__, rc, retry);
			continue;
		}

		pass = 1;
		break;
	}

	/* Put A1028 into sleep mode */
	rc = execute_cmdmsg(A100_msg_Sleep);
	if (rc < 0) {
		pr_err("%s: suspend error\n", __func__);
		goto set_suspend_err;
	}

	a1028_suspended = 1;
	a1028_current_config = A1028_PATH_SUSPEND;

	msleep(120);
	/* Disable A1028 clock */
	if (control_a1028_clk)
		a1028_gpio_set_value(pdata->gpio_a1028_clk, 0);
	if (control_a1028_xo_clk)
		a1028_xo_clk_enable(0);

set_suspend_err:
	if (pass && !rc)
		pr_info("%s: initialized!\n", __func__);
	else
		pr_err("%s: initialization failed\n", __func__);

	kfree(vp->data);
	return rc;
}

unsigned char phonecall_receiver[] = {
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x1B, 0x00, 0x0C,
	0x80, 0x1B, 0x01, 0x0C,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
};

unsigned char phonecall_headset[] = {
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x1B, 0x00, 0x0D,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
};

unsigned char phonecall_TTY_headset[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x00,
	0x80, 0x1B, 0x01, 0x00,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char phonecall_speaker[] = {
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x02,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x1B, 0x00, 0x14,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
};

unsigned char phonecall_bt[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x06,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x00,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char INT_MIC_recording_receiver[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x26, 0x00, 0x01,
	0x80, 0x1B, 0x00, 0x0D,
	0x80, 0x1B, 0x01, 0x0D,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char INT_MIC_stereo_recording_receiver[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x07,
	0x80, 0x1B, 0x00, 0xFA,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char EXT_MIC_recording[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x1B, 0x00, 0x0D,
	0x80, 0x1B, 0x01, 0x00,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char INT_MIC_recording_speaker[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x26, 0x00, 0x01,
	0x80, 0x1B, 0x00, 0x0D,
	0x80, 0x1B, 0x01, 0x0D,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char INT_MIC_stereo_recording_speaker[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x07,
	0x80, 0x1B, 0x00, 0xFA,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char BACK_MIC_recording[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x1B, 0x00, 0x0D,
	0x80, 0x15, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x03,
};

unsigned char vr_no_ns_receiver[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x0C,
	0x80, 0x1B, 0x01, 0x0C,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_no_ns_headset[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x12,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_no_ns_speaker[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x02,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x0C,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_no_ns_bt[] = {
	0x80, 0x26, 0x00, 0x06,
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x00,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_ns_receiver[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x1A,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x1B, 0x00, 0x0C,
	0x80, 0x1B, 0x01, 0x0C,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_ns_headset[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x03,
	0x80, 0x26, 0x00, 0x15,
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x02,
	0x80, 0x17, 0x00, 0x1A,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x12,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_ns_speaker[] = {
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x02,
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x1A,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x0C,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char vr_ns_bt[] = {
	0x80, 0x26, 0x00, 0x06,
	0x80, 0x1C, 0x00, 0x01,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x02,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x1A,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x1B, 0x00, 0x00,
	0x80, 0x15, 0x00, 0x00,
};

unsigned char suspend_mode[] = {
	0x80, 0x10, 0x00, 0x01
};

unsigned char mfg_loopback[] = {
	0x80, 0x1C, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x02,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x00,
	0x80, 0x18, 0x00, 0x04,
	0x80, 0x1B, 0x00, 0x00,
	0x80, 0x1B, 0x01, 0x00,
	0x80, 0x17, 0x00, 0x04,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x0E,
	0x80, 0x18, 0x00, 0x00,
	0x80, 0x17, 0x00, 0x15,
	0x80, 0x18, 0x00, 0x01,
};

static ssize_t chk_wakeup_a1028(void)
{
	int rc = 0, retry = 3;

	if (a1028_suspended == 1) {
		/* Enable A1028 clock */
		if (control_a1028_clk) {
			a1028_gpio_set_value(pdata->gpio_a1028_clk, 1);
			mdelay(1);
		}
		if (control_a1028_xo_clk) {
			a1028_xo_clk_enable(1);
			mdelay(1);
		}

		a1028_gpio_set_value(pdata->gpio_a1028_wakeup, 0);
		msleep(120);

		do {
			rc = execute_cmdmsg(A100_msg_Sync);
		} while ((rc < 0) && --retry);

		a1028_gpio_set_value(pdata->gpio_a1028_wakeup, 1);
		if (rc < 0) {
			pr_err("%s: failed (%d)\n", __func__, rc);
			goto wakeup_sync_err;
		}

		a1028_suspended = 0;
	}
wakeup_sync_err:
	return rc;
}

/* Filter commands according to noise suppression state forced by
 * A1028_SET_NS_STATE ioctl.
 *
 * For this function to operate properly, all configurations must include
 * both A100_msg_Bypass and Mic_Config commands even if default values
 * are selected or if Mic_Config is useless because VP is off
 */
int a1028_filter_vp_cmd(int cmd, int mode)
{
	int msg = (cmd >> 16) & 0xFFFF;
	int filtered_cmd = cmd;

	if (a1028_NS_state == A1028_NS_STATE_AUTO)
		return cmd;

	switch (msg) {
	case A100_msg_Bypass:
		if (a1028_NS_state == A1028_NS_STATE_OFF)
			filtered_cmd = A1028_msg_VP_OFF;
		else
			filtered_cmd = A1028_msg_VP_ON;
		break;
	case A100_msg_SetAlgorithmParmID:
		a1028_param_ID = cmd & 0xFFFF;
		break;
	case A100_msg_SetAlgorithmParm:
		if (a1028_param_ID == Mic_Config) {
			if (a1028_NS_state == A1028_NS_STATE_CT)
				filtered_cmd = (msg << 16);
			else if (a1028_NS_state == A1028_NS_STATE_FT)
				filtered_cmd = (msg << 16) | 0x0002;
		}
		break;
	default:
		if (mode == A1028_CONFIG_VP)
			filtered_cmd = -1;
		break;
	}

	pr_info("%s: %x filtered = %x, a1028_NS_state %d, mode %d\n",\
		__func__, cmd, filtered_cmd, a1028_NS_state, mode);

	return filtered_cmd;
}
int build_cmds(char *cmds, int newid)
{
	int i = 0;
	int offset = 0;
	for (i = 0; i < a1028_cmds_len; i += 6)
		if (config_data[i] == newid) {
			cmds[offset++] = config_data[i + 2];
			cmds[offset++] = config_data[i + 3];
			cmds[offset++] = config_data[i + 4];
			cmds[offset++] = config_data[i + 5];
		}
	if (offset > 128) {
		pr_err("Wrong i2c commands\n");
		return 0;
	}
	return offset;
}
int a1028_set_config(int newid, int mode)
{
	int rc = 0, size = 0;
	int number_of_cmd_sets, rd_retry_cnt;
	unsigned int sw_reset = 0;
	unsigned char *i2c_cmds;
	unsigned char *index = 0;
	unsigned char ack_buf[A1028_CMD_FIFO_DEPTH * 4];
	unsigned char rdbuf[4];
	unsigned char custom_cmds[128] = {0};

	if ((a1028_suspended) && (newid == A1028_PATH_SUSPEND))
		return rc;

	rc = chk_wakeup_a1028();
	if (rc < 0)
		return rc;

	sw_reset = ((A100_msg_Reset << 16) | RESET_IMMEDIATE);

	switch (newid) {
	case A1028_PATH_INCALL_RECEIVER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = phonecall_receiver;
		size = sizeof(phonecall_receiver);
		break;
	case A1028_PATH_INCALL_HEADSET:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 1);
		i2c_cmds = phonecall_headset;
		size = sizeof(phonecall_headset);
		break;
	case A1028_PATH_INCALL_SPEAKER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = phonecall_speaker;
		size = sizeof(phonecall_speaker);
		break;
	case A1028_PATH_INCALL_BT:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = phonecall_bt;
		size = sizeof(phonecall_bt);
		break;
	case A1028_PATH_VR_NO_NS_RECEIVER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_no_ns_receiver;
		size = sizeof(vr_no_ns_receiver);
		break;
	case A1028_PATH_VR_NO_NS_HEADSET:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 1);
		i2c_cmds = vr_no_ns_headset;
		size = sizeof(vr_no_ns_headset);
		break;
	case A1028_PATH_VR_NO_NS_SPEAKER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_no_ns_speaker;
		size = sizeof(vr_no_ns_speaker);
		break;
	case A1028_PATH_VR_NO_NS_BT:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_no_ns_bt;
		size = sizeof(vr_no_ns_bt);
		break;
	case A1028_PATH_VR_NS_RECEIVER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_ns_receiver;
		size = sizeof(vr_ns_receiver);
		break;
	case A1028_PATH_VR_NS_HEADSET:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 1);
		i2c_cmds = vr_ns_headset;
		size = sizeof(vr_ns_headset);
		break;
	case A1028_PATH_VR_NS_SPEAKER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_ns_speaker;
		size = sizeof(vr_ns_speaker);
		break;
	case A1028_PATH_VR_NS_BT:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = vr_ns_bt;
		size = sizeof(vr_ns_bt);
		break;
	case A1028_PATH_RECORD_RECEIVER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = INT_MIC_recording_receiver;
		size = sizeof(INT_MIC_recording_receiver);
		break;
	case A1028_PATH_RECORD_HEADSET:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 1);
		i2c_cmds = EXT_MIC_recording;
		size = sizeof(EXT_MIC_recording);
		break;
	case A1028_PATH_RECORD_SPEAKER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = INT_MIC_recording_speaker;
		size = sizeof(INT_MIC_recording_speaker);
		break;
	case A1028_PATH_RECORD_BT:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = phonecall_bt;
		size = sizeof(phonecall_bt);
		break;
	case A1028_PATH_SUSPEND:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = (unsigned char *)suspend_mode;
		size = sizeof(suspend_mode);
		break;
	case A1028_PATH_CAMCORDER:
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
		i2c_cmds = BACK_MIC_recording;
		size = sizeof(BACK_MIC_recording);
		break;
	case A1028_PATH_MFG_LOOPBACK:
		a1028_pmic_set_value(pdata->gpio_a1028_micswitch, 0);
		i2c_cmds = mfg_loopback;
		size = sizeof(mfg_loopback);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, newid);
		rc = -1;
		goto input_err;
		break;
	}

	a1028_current_config = newid;

	if (a1028_cmds_len > 0) {
		int cmd_size = 0;
		cmd_size = build_cmds(custom_cmds, newid);
		if (cmd_size > 0)
			i2c_cmds = custom_cmds;
		size = cmd_size;
		pr_info("use customize command\n");
	}

	pr_info("%s: change to mode %d\n", __func__, newid);

	pr_info("%s: block write start (size = %d)\n", __func__, size);

#if DEBUG
	{
		int i = 0;
		for (i = 1; i <= size; i++) {
			pr_info("%x ", *(i2c_cmds + i - 1));
			if (!(i % 4))
				pr_info("\n");
		}
	}
#endif

	rc = a1028_i2c_write(i2c_cmds, size);
	if (rc < 0) {
		pr_err("A1028 CMD block write error!\n");
		a1028_i2c_sw_reset(sw_reset);
		return rc;
	}
	pr_info("%s: block write end\n", __func__);

	/* Don't need to get Ack after sending out a suspend command */
	if (*i2c_cmds == 0x80 && *(i2c_cmds + 1) == 0x10
		&& *(i2c_cmds + 2) == 0x00 && *(i2c_cmds + 3) == 0x01) {
		a1028_suspended = 1;
		/* Disable A1028 clock */
		msleep(120);
		if (control_a1028_clk)
			a1028_gpio_set_value(pdata->gpio_a1028_clk, 0);
		if (control_a1028_xo_clk)
			a1028_xo_clk_enable(0);
		return rc;
	}

	memset(ack_buf, 0, sizeof(ack_buf));
	msleep(20);
	pr_info("%s: CMD ACK block read start\n", __func__);
	rc = a1028_i2c_read(ack_buf, size);
	if (rc < 0) {
		pr_err("%s: CMD ACK block read error\n", __func__);
		a1028_i2c_sw_reset(sw_reset);
		return rc;
	} else {
		pr_info("%s: CMD ACK block read end\n", __func__);
#if DEBUG
	{
		int i = 0;
		for (i = 1; i <= size; i++) {
			pr_info("%x ", ack_buf[i-1]);
			if (!(i % 4))
				pr_info("\n");
		}
	}
#endif
		index = ack_buf;
		number_of_cmd_sets = size / 4;
		do {
			if (*index == 0x00) {
				rd_retry_cnt = POLLING_RETRY_CNT;
rd_retry:
				if (rd_retry_cnt--) {
					memset(rdbuf, 0, sizeof(rdbuf));
					rc = a1028_i2c_read(rdbuf, 4);
					if (rc < 0)
						return rc;
#if DEBUG
				{
					int i = 0;
					for (i = 0; i < sizeof(rdbuf); i++)
						pr_info("0x%x\n", rdbuf[i]);
					pr_info("-----------------\n");
				}
#endif
					if (rdbuf[0] == 0x00) {
						msleep(20);
						goto rd_retry;
					}
				} else {
					pr_err("%s: CMD ACK Not Ready\n",
						__func__);
					return -EBUSY;
				}
			} else if (*index == 0xff) { /* illegal cmd */
				return -ENOEXEC;
			} else if (*index == 0x80) {
				index += 4;
			}
		} while (--number_of_cmd_sets);
	}
input_err:
	return rc;
}

int execute_cmdmsg(unsigned int msg)
{
	int rc = 0;
	int retries, pass = 0;
	unsigned char msgbuf[4];
	unsigned char chkbuf[4];
	unsigned int sw_reset = 0;

	sw_reset = ((A100_msg_Reset << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	memcpy(chkbuf, msgbuf, 4);

	rc = a1028_i2c_write(msgbuf, 4);
	if (rc < 0) {
		pr_err("%s: error %d\n", __func__, rc);
		a1028_i2c_sw_reset(sw_reset);
		return rc;
	}

	/* We don't need to get Ack after sending out a suspend command */
	if (msg == A100_msg_Sleep)
		return rc;

	retries = POLLING_RETRY_CNT;
	while (retries--) {
		rc = 0;

		msleep(20); /* use polling */
		memset(msgbuf, 0, sizeof(msgbuf));
		rc = a1028_i2c_read(msgbuf, 4);
		if (rc < 0) {
			pr_err("%s: ack-read error %d (%d retries)\n",\
				__func__, rc, retries);
			continue;
		}

		if (msgbuf[0] == 0x80  && msgbuf[1] == chkbuf[1]) {
			pass = 1;
			break;
		} else if (msgbuf[0] == 0xff && msgbuf[1] == 0xff) {
			pr_err("%s: illegal cmd %08x\n", __func__, msg);
			rc = -EINVAL;
			break;
		} else if (msgbuf[0] == 0x00 && msgbuf[1] == 0x00) {
			pr_info("%s: not ready (%d retries)\n", __func__,
				retries);
			rc = -EBUSY;
		} else {
			pr_info("%s: cmd/ack mismatch: (%d retries left)\n",
				__func__,
				retries);
#if DEBUG
		pr_info("%s: msgbuf[0] = %x\n", __func__, msgbuf[0]);
		pr_info("%s: msgbuf[1] = %x\n", __func__, msgbuf[1]);
		pr_info("%s: msgbuf[2] = %x\n", __func__, msgbuf[2]);
		pr_info("%s: msgbuf[3] = %x\n", __func__, msgbuf[3]);
#endif
			rc = -EBUSY;
		}
	}

	if (!pass) {
		pr_err("%s: failed execute cmd %08x (%d)\n", __func__,
			msg, rc);
		a1028_i2c_sw_reset(sw_reset);
	}
	return rc;
}

#if ENABLE_DIAG_IOCTLS
static int a1028_set_mic_state(char miccase)
{
	int rc = 0;
	unsigned int cmd_msg = 0;

	switch (miccase) {
	case 1: /* Mic-1 ON / Mic-2 OFF */
		cmd_msg = 0x80260007;
		break;
	case 2: /* Mic-1 OFF / Mic-2 ON */
		cmd_msg = 0x80260015;
		break;
	case 3: /* both ON */
		cmd_msg = 0x80260001;
		break;
	case 4: /* both OFF */
		cmd_msg = 0x80260006;
		break;
	default:
		pr_info("%s: invalid input %d\n", __func__, miccase);
		rc = -EINVAL;
		break;
	}
	rc = execute_cmdmsg(cmd_msg);
	return rc;
}

static int exe_cmd_in_file(unsigned char *incmd)
{
	int rc = 0;
	int i = 0;
	unsigned int cmd_msg = 0;
	unsigned char tmp = 0;

	for (i = 0; i < 4; i++) {
		tmp = *(incmd + i);
		cmd_msg |= (unsigned int)tmp;
		if (i != 3)
			cmd_msg = cmd_msg << 8;
	}
	rc = execute_cmdmsg(cmd_msg);
	if (rc < 0)
		pr_err("%s: cmd %08x error %d\n", __func__, cmd_msg, rc);
	return rc;
}
#endif /* ENABLE_DIAG_IOCTLS */

static long
a1028_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct a1028img img;
	struct A1028_config_data cfg;
	int rc = 0;
#if ENABLE_DIAG_IOCTLS
	char msg[4];
	int mic_cases = 0;
	int mic_sel = 0;
#endif
	int pathid = 0;
	int ns_state;

	switch (cmd) {
	case A1028_BOOTUP_INIT:
		img.buf = 0;
		img.img_size = 0;
		if (copy_from_user(&img, argp, sizeof(img)))
			return -EFAULT;
		rc = a1028_bootup_init(file, &img);
		break;
	case A1028_SET_CONFIG:
		if (copy_from_user(&pathid, argp, sizeof(pathid)))
			return -EFAULT;
		if (pathid < 0 || pathid >= A1028_PATH_MAX)
			return -EINVAL;
		rc = a1028_set_config(pathid, A1028_CONFIG_FULL);
		if (rc < 0)
			pr_err("%s: A1028_SET_CONFIG (%d) error %d!\n",
				__func__, pathid, rc);
		break;
	case A1028_SET_NS_STATE:
		if (copy_from_user(&ns_state, argp, sizeof(ns_state)))
			return -EFAULT;
		pr_info("%s: set noise suppression %d\n",\
			__func__, ns_state);
		if (ns_state < 0 || ns_state >= A1028_NS_NUM_STATES)
			return -EINVAL;
		a1028_NS_state = ns_state;
		if (!a1028_suspended)
			a1028_set_config(a1028_current_config,
					A1028_CONFIG_VP);
		break;
	case A1028_SET_PARAM:
		a1028_cmds_len = 0;
		cfg.cmd_data = 0;
		if (copy_from_user(&cfg, argp, sizeof(cfg))) {
			pr_err("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}

		if (cfg.data_len <= 0 || cfg.data_len > PARAM_MAX) {
				pr_err("%s: invalid data length %d\n", \
				__func__, cfg.data_len);
				return -EINVAL;
		}

		if (cfg.cmd_data == NULL) {
			pr_err("%s: invalid data\n", __func__);
			return -EINVAL;
		}

		if (config_data == NULL)
			config_data = kmalloc(cfg.data_len, GFP_KERNEL);
		if (!config_data) {
			pr_err("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(config_data, cfg.cmd_data, cfg.data_len)) {
			pr_err("%s: copy data from user failed.\n",\
				__func__);
			kfree(config_data);
			config_data = NULL;
			return -EFAULT;
		}
		a1028_cmds_len = cfg.data_len;
		pr_info("%s: update a1028 i2c commands success.\n",\
			__func__);
		rc = 0;
		break;
#if ENABLE_DIAG_IOCTLS
	case A1028_SET_MIC_ONOFF:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		if (copy_from_user(&mic_cases, argp, sizeof(mic_cases)))
			return -EFAULT;
		rc = a1028_set_mic_state(mic_cases);
		if (rc < 0)
			pr_err("%s: A1028_SET_MIC_ONOFF %d error %d!\n",
				__func__, mic_cases, rc);
		break;
	case A1028_SET_MICSEL_ONOFF:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		if (copy_from_user(&mic_sel, argp, sizeof(mic_sel)))
			return -EFAULT;
		if (control_a1028_micsel)
			a1028_gpio_set_value(pdata->gpio_a1028_micsel, \
				!!mic_sel);
		rc = 0;
		break;
	case A1028_READ_DATA:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		rc = a1028_i2c_read(msg, 4);
		if (copy_to_user(argp, &msg, 4))
			return -EFAULT;
		break;
	case A1028_WRITE_MSG:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		if (copy_from_user(msg, argp, sizeof(msg)))
			return -EFAULT;
		rc = a1028_i2c_write(msg, 4);
		break;
	case A1028_SYNC_CMD:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		msg[0] = 0x80;
		msg[1] = 0x00;
		msg[2] = 0x00;
		msg[3] = 0x00;
		rc = a1028_i2c_write(msg, 4);
		break;
	case A1028_SET_CMD_FILE:
		rc = chk_wakeup_a1028();
		if (rc < 0)
			return rc;
		if (copy_from_user(msg, argp, sizeof(msg)))
			return -EFAULT;
		rc = exe_cmd_in_file(msg);
		break;
#endif /* ENABLE_DIAG_IOCTLS */
	default:
		pr_err("%s: invalid command %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations a1028_fops = {
	.owner = THIS_MODULE,
	.open = a1028_open,
	.release = a1028_release,
	.unlocked_ioctl = a1028_ioctl,
};

static struct miscdevice a1028_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_a1028",
	.fops = &a1028_fops,
};

static int a1028_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	control_a1028_xo_clk = 0;
	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			rc = -ENOMEM;
			pr_err("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	this_client = client;
#ifdef CONFIG_ARCH_MSM8960
	a1028_clock = msm_xo_get(MSM_XO_TCXO_D1, cid);
	if (IS_ERR(a1028_clock)) {
		control_a1028_xo_clk = 0;
		msm_xo_put(a1028_clock);
	} else {
		control_a1028_xo_clk = 1;
		pr_info("use tcxo_d1 clk");
	}
#endif
	rc = gpio_request(pdata->gpio_a1028_clk, "a1028");
	if (rc < 0) {
		control_a1028_clk = 0;
		goto chk_gpio_micsel;
	}
	control_a1028_clk = 1;

	rc = gpio_direction_output(pdata->gpio_a1028_clk, 1);
	if (rc < 0) {
		pr_err("%s: request clk gpio direction failed\n", __func__);
		goto err_free_gpio_clk;
	}

chk_gpio_micsel:
	rc = gpio_request(pdata->gpio_a1028_micsel, "a1028");
	if (rc < 0) {
		pr_info("%s: gpio request mic_sel pin failed\n", __func__);
		control_a1028_micsel = 0;
	} else {
		rc = gpio_direction_output(pdata->gpio_a1028_micsel, 1);
		if (rc < 0) {
			pr_err("%s: request mic_sel gpio direction \
				failed\n", __func__);
			goto err_free_gpio_micsel;
		}
	}

	rc = gpio_request(pdata->gpio_a1028_wakeup, "a1028");
	if (rc < 0) {
		pr_err("%s: gpio request wakeup pin failed\n", __func__);
		goto err_free_gpio;
	}

	rc = gpio_direction_output(pdata->gpio_a1028_wakeup, 1);
	if (rc < 0) {
		pr_err("%s: request wakeup gpio direction failed\n",\
			__func__);
		goto err_free_gpio;
	}

	rc = gpio_request(pdata->gpio_a1028_reset, "a1028");
	if (rc < 0) {
		pr_err("%s: gpio request reset pin failed\n", __func__);
		goto err_free_gpio;
	}

	rc = gpio_direction_output(pdata->gpio_a1028_reset, 1);
	if (rc < 0) {
		pr_err("%s: request reset gpio direction failed\n",\
			__func__);
		goto err_free_gpio_all;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		rc = -ENODEV;
		goto err_free_gpio_all;
	}

	if (control_a1028_clk)
		a1028_gpio_set_value(pdata->gpio_a1028_clk, 1);
	if (control_a1028_micsel)
		a1028_gpio_set_value(pdata->gpio_a1028_micsel, 0);
	a1028_gpio_set_value(pdata->gpio_a1028_wakeup, 1);
	a1028_gpio_set_value(pdata->gpio_a1028_reset, 1);

	rc = misc_register(&a1028_device);
	if (rc) {
		pr_err("%s: a1028_device register failed\n", __func__);
		goto err_free_gpio_all;
	}
	pr_info("%s ok", __func__);
	return 0;

err_free_gpio_all:
	gpio_free(pdata->gpio_a1028_reset);
err_free_gpio:
	gpio_free(pdata->gpio_a1028_wakeup);
err_free_gpio_micsel:
	gpio_free(pdata->gpio_a1028_micsel);
err_free_gpio_clk:
	if (control_a1028_clk)
		gpio_free(pdata->gpio_a1028_clk);
err_alloc_data_failed:
	return rc;
}

static int a1028_remove(struct i2c_client *client)
{
	struct a1028_platform_data *p1028data = i2c_get_clientdata(client);
	kfree(p1028data);

	return 0;
}

static int a1028_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int a1028_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id a1028_id[] = {
	{ "audience_a1028", 0 },
	{ }
};

static struct i2c_driver a1028_driver = {
	.probe = a1028_probe,
	.remove = a1028_remove,
	.suspend = a1028_suspend,
	.resume	= a1028_resume,
	.id_table = a1028_id,
	.driver = {
		.name = "audience_a1028",
	},
};

static int __init a1028_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&a1028_lock);

	return i2c_add_driver(&a1028_driver);
}

static void __exit a1028_exit(void)
{
	i2c_del_driver(&a1028_driver);
}

module_init(a1028_init);
module_exit(a1028_exit);

MODULE_DESCRIPTION("A1028 voice processor driver");
MODULE_LICENSE("GPL");

/* driver/i2c/chip/tap6185.c
 *
 * TI tpa6185 Speaker Amp
 *
 * Copyright (C) 2010 HTC Corporation
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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/tpa6185.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mfd/pm8xxx/pm8921.h>

//htc audio ++
#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)
//htc audio --

#ifdef CONFIG_AMP_TPA6185_ON_GPIO
#define DEBUG (1)
#else
#define DEBUG (1)
#endif
#define AMP_ON_CMD_LEN 7
#define RETRY_CNT 5
static int tpa6185Connect = 0;
static struct i2c_client *this_client;
static struct tpa6185_platform_data *pdata;
static char *config_data;
static int tpa6185_mode_cnt;
struct mutex hp_amp_lock;
static int tpa6185_opened;
static int last_spkamp_state;
static char SPK_AMP_ON[] =
			{0x00, 0xC0, 0x2F, 0x00, 0x00, 0x00, 0x00};
static char HEADSET_AMP_ON[] =
			{0x00, 0xC0, 0x2F, 0x00, 0x00, 0x00, 0x00};
static char RING_AMP_ON[] =
			{0x00, 0xC0, 0x2F, 0x00, 0x00, 0x00, 0x00};
static char HANDSET_AMP_ON[] =
			{0x00, 0xC0, 0x2F, 0x00, 0x00, 0x00, 0x00};
static char LINEOUT_AMP_ON[] =
			{0x00, 0xC0, 0x2F, 0x00, 0x00, 0x00, 0x00};
static char AMP_0FF[] = {0x00, 0x90};
static int tpa6185_write_reg(u8 reg, u8 val);
static int tpa6185_i2c_write_for_read(char *txData, int length);
static int tpa6185_i2c_read(char *rxData, int length);
#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_tpa_dent;
static struct dentry *debugfs_peek;
static struct dentry *debugfs_poke;
static unsigned char read_data;

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (strict_strtoul(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
			}
		else
			return -EINVAL;
	}
	return 0;
}

static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t codec_debug_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[8];

	snprintf(lbuf, sizeof(lbuf), "0x%x\n", read_data);
	return simple_read_from_buffer(ubuf, count, ppos, lbuf, strlen(lbuf));
}

static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	unsigned char reg_idx[1] = {0x01};
	int rc;
	long int param[5];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	if (!strcmp(access_str, "poke")) {
		/* write */
		rc = get_parameters(lbuf, param, 2);
		if ((param[0] <= 0xFF) && (param[1] <= 0xFF) &&
			(rc == 0))
			tpa6185_write_reg(param[0], param[1]);
		else
			rc = -EINVAL;
	} else if (!strcmp(access_str, "peek")) {
		/* read */
		rc = get_parameters(lbuf, param, 1);
		if ((param[0] <= 0xFF) && (rc == 0)) {
			reg_idx[0] = param[0];
			tpa6185_i2c_write_for_read(reg_idx, 1);
			tpa6185_i2c_read(&read_data, sizeof(HEADSET_AMP_ON));
		} else
			rc = -EINVAL;
	}

	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
	.read = codec_debug_read
};
#endif

static int tpa6185_write_reg(u8 reg, u8 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	msg->addr = this_client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = reg;
	data[1] = val;

	err = i2c_transfer(this_client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

static int tpa6185_i2c_write(char *txData, int length)
{
	int i, retry, pass = 0;
	char buf[2];
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buf,
		},
	};
	for (i = 1; i < length; i++) {
		if (i == 2)  /* According to tpa6185 Spec */
			mdelay(1);
		buf[0] = i;
		buf[1] = txData[i];
#if DEBUG
		pr_info("i2c_write %d=%x\n", i, buf[1]);
#endif
		msg->buf = buf;
		retry = RETRY_CNT;
		pass = 0;
		while (retry--) {
			if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
				pr_err("%s: I2C transfer error %d retry %d\n",
						__func__, i, retry);
				msleep(20);
			} else {
				pass = 1;
				break;
			}
		}
		if (pass == 0) {
			pr_err("I2C transfer error, retry fail\n");
			return -EIO;
		}
	}
	return 0;
}

static int tpa6185_i2c_write_for_read(char *txData, int length)
{
	int i, retry, pass = 0;
	char buf[2];
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buf,
		},
	};
	for (i = 0; i < length; i++) {
		if (i == 2)  /* According to tpa6185 Spec */
			mdelay(1);
		buf[0] = i;
		buf[1] = txData[i];
#if DEBUG
		pr_info("i2c_write %d=%x\n", i, buf[1]);
#endif
		msg->buf = buf;
		retry = RETRY_CNT;
		pass = 0;
		while (retry--) {
			if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
				pr_err("%s: I2C transfer error %d retry %d\n",
						__func__, i, retry);
				msleep(20);
			} else {
				pass = 1;
				break;
			}
		}
		if (pass == 0) {
			pr_err("I2C transfer error, retry fail\n");
			return -EIO;
		}
	}
	return 0;
}

static int tpa6185_i2c_read(char *rxData, int length)
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

	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("i2c_read %s: rx[%d] = %2x\n", __func__, i, \
				rxData[i]);
	}

	return 0;
}

static int tpa6185_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&hp_amp_lock);

	if (tpa6185_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	tpa6185_opened = 1;
done:
	mutex_unlock(&hp_amp_lock);
	return rc;
}

static int tpa6185_release(struct inode *inode, struct file *file)
{
	mutex_lock(&hp_amp_lock);
	tpa6185_opened = 0;
	mutex_unlock(&hp_amp_lock);

	return 0;
}
void set_amp(int on, char *i2c_command)
{
	pr_info("%s: %d\n", __func__, on);
	mutex_lock(&hp_amp_lock);
	if (on) {
		if (tpa6185_i2c_write(i2c_command, AMP_ON_CMD_LEN) == 0) {
			last_spkamp_state = 1;
			printk("%s: ON reg1=%x, reg2=%x\n",
				__func__, i2c_command[1], i2c_command[2]);
		}
	} else {
		if (tpa6185_i2c_write(AMP_0FF, sizeof(AMP_0FF)) == 0) {
			last_spkamp_state = 0;
			printk("%s: OFF\n", __func__);
		}
	}
	mutex_unlock(&hp_amp_lock);
}

int query_tpa6185(void)
{
    return tpa6185Connect;
}

void set_speaker_amp(int on)
{
	set_amp(on, SPK_AMP_ON);
}

void set_headset_amp(int on)
{
	set_amp(on, HEADSET_AMP_ON);
}

void set_speaker_headset_amp(int on)
{
	set_amp(on, RING_AMP_ON);
}

void set_handset_amp(int on)
{
	set_amp(on, HANDSET_AMP_ON);
}

void set_usb_audio_amp(int on)
{
	set_amp(on, LINEOUT_AMP_ON);
}

int update_amp_parameter(int mode)
{
	if (mode > tpa6185_mode_cnt)
		return EINVAL;
	if (*(config_data + mode * MODE_CMD_LEM + 1) == SPKR_OUTPUT)
		memcpy(SPK_AMP_ON, config_data + mode * MODE_CMD_LEM + 2,
				sizeof(SPK_AMP_ON));
	else if (*(config_data + mode * MODE_CMD_LEM + 1) == HEADSET_OUTPUT)
		memcpy(HEADSET_AMP_ON, config_data + mode * MODE_CMD_LEM + 2,
				sizeof(HEADSET_AMP_ON));
	else if (*(config_data + mode * MODE_CMD_LEM + 1) == DUAL_OUTPUT)
		memcpy(RING_AMP_ON, config_data + mode * MODE_CMD_LEM + 2,
				sizeof(RING_AMP_ON));
	else if (*(config_data + mode * MODE_CMD_LEM + 1) == HANDSET_OUTPUT)
		memcpy(HANDSET_AMP_ON, config_data + mode * MODE_CMD_LEM + 2,
				sizeof(HANDSET_AMP_ON));
	else if (*(config_data + mode * MODE_CMD_LEM + 1) == LINEOUT_OUTPUT)
		memcpy(LINEOUT_AMP_ON, config_data + mode * MODE_CMD_LEM + 2,
				sizeof(LINEOUT_AMP_ON));
	else {
		printk("wrong mode id %d\n", mode);
		return -EINVAL;
	}
	return 0;
}


static long tpa6185_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0, modeid = 0;
	unsigned char tmp[7];
	unsigned char reg_idx[1] = {0x01};
	unsigned char spk_cfg[8];
	unsigned char reg_value[2];
	struct tpa6185_config_data cfg;

	switch (cmd) {
	case TPA6185_WRITE_REG:
		pr_info("%s: TPA6185_WRITE_REG\n", __func__);
		mutex_lock(&hp_amp_lock);
		if (!last_spkamp_state) {
			/* According to tpa6185 Spec */
			mdelay(30);
		}
		if (copy_from_user(reg_value, argp, sizeof(reg_value)))
			goto err1;
		pr_info("%s: reg_value[0]=%2x, reg_value[1]=%2x\n", __func__,  \
				reg_value[0], reg_value[1]);
		rc = tpa6185_write_reg(reg_value[0], reg_value[1]);

err1:
		mutex_unlock(&hp_amp_lock);
		break;
	case TPA6185_SET_CONFIG:
		if (copy_from_user(spk_cfg, argp, sizeof(spk_cfg)))
			return -EFAULT;
		if (spk_cfg[0] == SPKR_OUTPUT)
			memcpy(SPK_AMP_ON, spk_cfg + 1,
					sizeof(SPK_AMP_ON));
		else if (spk_cfg[0] == HEADSET_OUTPUT)
			memcpy(HEADSET_AMP_ON, spk_cfg + 1,
					sizeof(HEADSET_AMP_ON));
		else if (spk_cfg[0] == DUAL_OUTPUT)
			memcpy(RING_AMP_ON, spk_cfg + 1,
					sizeof(RING_AMP_ON));
		else if (spk_cfg[0] == LINEOUT_OUTPUT)
			memcpy(LINEOUT_AMP_ON, spk_cfg + 1,
					sizeof(LINEOUT_AMP_ON));
		else
			return -EINVAL;
		break;
	case TPA6185_READ_CONFIG:
		mutex_lock(&hp_amp_lock);
		if (!last_spkamp_state) {
			/* According to tpa6185 Spec */
			mdelay(30);
		}

		rc = tpa6185_i2c_write_for_read(reg_idx, sizeof(reg_idx));
		if (rc < 0)
			goto err2;

		rc = tpa6185_i2c_read(tmp, sizeof(tmp));
		if (rc < 0)
			goto err2;

		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			rc = -EFAULT;
err2:
		mutex_unlock(&hp_amp_lock);
		break;
	case TPA6185_SET_MODE:
		if (copy_from_user(&modeid, argp, sizeof(modeid)))
			return -EFAULT;

		if (modeid > tpa6185_mode_cnt || modeid <= 0) {
			pr_err("unsupported tpa6185 mode %d\n", modeid);
			return -EINVAL;
		}
		rc = update_amp_parameter(modeid);
		pr_info("set tpa6185 mode to %d\n", modeid);
		break;
	case TPA6185_SET_PARAM:
		cfg.cmd_data = 0;
		tpa6185_mode_cnt = 0;
		if (copy_from_user(&cfg, argp, sizeof(cfg))) {
			pr_err("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}

		if (cfg.data_len <= 0) {
			pr_err("%s: invalid data length %d\n",
					__func__, cfg.data_len);
			return -EINVAL;
		}
		if (config_data == NULL)
			config_data = kmalloc(cfg.data_len, GFP_KERNEL);
		if (!config_data) {
			pr_err("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(config_data, cfg.cmd_data, cfg.data_len)) {
			pr_err("%s: copy data from user failed.\n", __func__);
			kfree(config_data);
			config_data = NULL;
			return -EFAULT;
		}
		tpa6185_mode_cnt = cfg.mode_num;
		pr_info("%s: update tpa6185 i2c commands #%d success.\n",
				__func__, cfg.data_len);
		/* update default paramater from csv*/
		update_amp_parameter(TPA6185_MODE_PLAYBACK_SPKR);
		update_amp_parameter(TPA6185_MODE_PLAYBACK_HEADSET);
		update_amp_parameter(TPA6185_MODE_RING);
		update_amp_parameter(TPA6185_MODE_PLAYBACK_HANDSET);
		update_amp_parameter(TPA6185_MODE_LINEOUT);
		rc = 0;
		break;
	default:
		pr_err("%s: Invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static struct file_operations tpa6185_fops = {
	.owner = THIS_MODULE,
	.open = tpa6185_open,
	.release = tpa6185_release,
	.unlocked_ioctl = tpa6185_ioctl,
};

static struct miscdevice tpa6185_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tpa6185",
	.fops = &tpa6185_fops,
};

int tpa6185_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
        int err = 0;

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pr_info("%s: platform data null\n", __func__);
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			pr_err("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	this_client = client;

	if (ret < 0) {
		pr_err("%s: pmic request aud_spk_en pin failed\n", __func__);
		goto err_free_gpio_all;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

        if(pdata->gpio_tpa6185_spk_en) {
            char temp[2];

            err = gpio_request(pdata->gpio_tpa6185_spk_en, "hp_en");

            ret = gpio_direction_output(pdata->gpio_tpa6185_spk_en, 1);

            if(ret < 0) {
		pr_err("%s: gpio %d on error %d\n", __func__,pdata->gpio_tpa6185_spk_en,ret);
            }

            mdelay(1);

            ret = tpa6185_i2c_read(temp, 2);

            if(ret < 0) {
                pr_info("tpa6185 is not connected\n");
                tpa6185Connect = 0;
            } else {
                pr_info("tpa6185 is connected\n");
                tpa6185Connect = 1;
            }

	    gpio_direction_output(pdata->gpio_tpa6185_spk_en, 0);

            if(!err)
                gpio_free(pdata->gpio_tpa6185_spk_en);

            if(ret < 0) {
		pr_err("%s: gpio %d off error %d\n", __func__,pdata->gpio_tpa6185_spk_en,ret);
            }

        }

        if(tpa6185Connect) {
            ret = misc_register(&tpa6185_device);
            if (ret) {
                pr_err("%s: tpa6185_device register failed\n", __func__);
                goto err_free_gpio_all;
            }
        }

	if (pdata->spkr_cmd[1] != 0)  /* path id != 0 */
		memcpy(SPK_AMP_ON, pdata->spkr_cmd, sizeof(SPK_AMP_ON));
	if (pdata->hsed_cmd[1] != 0)
		memcpy(HEADSET_AMP_ON, pdata->hsed_cmd, sizeof(HEADSET_AMP_ON));
	if (pdata->rece_cmd[1] != 0)
		memcpy(HANDSET_AMP_ON, pdata->rece_cmd, sizeof(HANDSET_AMP_ON));

#ifdef CONFIG_DEBUG_FS
	debugfs_tpa_dent = debugfs_create_dir("tpa6185", 0);
	if (!IS_ERR(debugfs_tpa_dent)) {
		debugfs_peek = debugfs_create_file("peek",
		S_IFREG | S_IRUGO, debugfs_tpa_dent,
		(void *) "peek", &codec_debug_ops);

		debugfs_poke = debugfs_create_file("poke",
		S_IFREG | S_IRUGO, debugfs_tpa_dent,
		(void *) "poke", &codec_debug_ops);

	}
#endif

	return 0;

err_free_gpio_all:
        tpa6185Connect = 0;
	return ret;
err_alloc_data_failed:
        tpa6185Connect = 0;
	return ret;
}

static int tpa6185_remove(struct i2c_client *client)
{
	struct tpa6185_platform_data *p6185data = i2c_get_clientdata(client);
	kfree(p6185data);

	return 0;
}

static int tpa6185_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tpa6185_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tpa6185_id[] = {
	{ TPA6185_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa6185_driver = {
	.probe = tpa6185_probe,
	.remove = tpa6185_remove,
	.suspend = tpa6185_suspend,
	.resume = tpa6185_resume,
	.id_table = tpa6185_id,
	.driver = {
		.name = TPA6185_I2C_NAME,
	},
};

static int __init tpa6185_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&hp_amp_lock);
	return i2c_add_driver(&tpa6185_driver);
}

static void __exit tpa6185_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_peek);
	debugfs_remove(debugfs_poke);
	debugfs_remove(debugfs_tpa_dent);
#endif
	i2c_del_driver(&tpa6185_driver);
}

module_init(tpa6185_init);
module_exit(tpa6185_exit);

MODULE_DESCRIPTION("tpa6185 Speaker Amp driver");
MODULE_LICENSE("GPL");

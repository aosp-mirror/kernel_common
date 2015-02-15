/* driver/i2c/chip/rt5506.c
 *
 * Richtek Headphone Amp
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
#include <mach/rt5506.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <mach/htc_headset_mgr.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>
#include <mach/htc_acoustic_alsa.h>

#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)


#define DEBUG (1)
#define AMP_ON_CMD_LEN 7
#define RETRY_CNT 5

#define DRIVER_NAME "RT5506"

static int set_rt5506_amp(int on, int dsp);

enum AMP_REG_MODE {
	REG_PWM_MODE = 0,
	REG_AUTO_MODE,
};

enum AMP_POWER_MASK {
	POWER_PLAYBACK = 0,
	POWER_IMPEDANCE,
	POWER_CMDLINE_TOOL,
	POWER_HIGH_IMP_RESET,
	POWER_MASK_MAX,
};

struct headset_query {
	struct mutex mlock;
	struct mutex gpiolock;
	struct delayed_work hs_imp_detec_work;
	struct wake_lock hs_wake_lock;
	struct wake_lock gpio_wake_lock;
	enum HEADSET_QUERY_STATUS hs_qstatus;
	enum AMP_STATUS rt5506_status;
	enum HEADSET_OM headsetom;
	enum PLAYBACK_MODE curmode;
	enum AMP_GPIO_STATUS gpiostatus;
	enum AMP_REG_MODE regstatus;
	int action_on;
	int gpio_off_cancel;
	struct mutex actionlock;
	struct delayed_work volume_ramp_work;
	struct delayed_work gpio_off_work;
	int hs_connec;
	unsigned long power_mask;
};

static struct i2c_client *this_client;
static struct rt5506_platform_data *pdata;
static int rt5506Connect = 0;

struct rt5506_config_data rt5506_config_data;
static struct mutex hp_amp_lock;
static int rt5506_opened;
static int last_spkamp_state;
struct rt5506_config RT5506_AMP_ON = {7,{{0x0,0xc0},{0x1,0x1c},{0x2,0x00},{0x7,0x7f},{0x9,0x1},{0xa,0x0},{0xb,0xc7},}};
struct rt5506_config RT5506_AMP_INIT = {11,{{0,0xc0},{0x81,0x30},{0x87,0xf6},{0x93,0x8d},{0x95,0x7d},{0xa4,0x52},\
                                        {0x96,0xae},{0x97,0x13},{0x99,0x35},{0x9b,0x68},{0x9d,0x68},}};

struct rt5506_config RT5506_AMP_MUTE = {1,{{0x1,0xC7},}};;
struct rt5506_config RT5506_AMP_OFF = {1,{{0x0,0x1},}};

static int rt5506_write_reg(u8 reg, u8 val);
static void hs_imp_detec_func(struct work_struct *work);
static int rt5506_i2c_read_addr(unsigned char *rxData, unsigned char addr);
static int rt5506_i2c_write(struct rt5506_reg_data *txData, int length);
static void set_amp(int on, struct rt5506_config *i2c_command);
struct headset_query rt5506_query;
static struct workqueue_struct *hs_wq;

static struct workqueue_struct *ramp_wq;
static struct workqueue_struct *gpio_wq;
static int high_imp = 0;

static int set_rt5506_regulator(enum AMP_REG_MODE mode)
{
	if(pdata->power_reg) {

		switch(mode) {
			case REG_PWM_MODE:
				pr_info("%s:set regulator to PWM mode\n",__func__);
				rpm_regulator_set_mode(pdata->power_reg,RPM_REGULATOR_MODE_HPM);
				break;

			case REG_AUTO_MODE:
				pr_info("%s:set regulator to AUTO mode\n",__func__);
				rpm_regulator_set_mode(pdata->power_reg,RPM_REGULATOR_MODE_AUTO);
				break;

			default:
				break;
		}
	}

	return 0;
}

static int need_power(unsigned long *addr)
{
	return (*addr != 0);
}

static void vote_power(unsigned long *addr, enum AMP_POWER_MASK bit)
{
	pr_info("%s: mask %d\n",__func__,bit);
	set_bit(bit,addr);

	if(need_power(addr) && rt5506_query.gpiostatus == AMP_GPIO_OFF) {
		pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5506_enable);
		if(rt5506_query.regstatus == REG_AUTO_MODE) {
			set_rt5506_regulator(REG_PWM_MODE);
			rt5506_query.regstatus = REG_PWM_MODE;
			msleep(1);
		}

		gpio_set_value(pdata->gpio_rt5506_enable, 1);
		rt5506_query.gpiostatus = AMP_GPIO_ON;
		usleep_range(20000,20000);
	}

}

static void unvote_power(unsigned long *addr, enum AMP_POWER_MASK bit)
{
	pr_info("%s: mask %d\n",__func__,bit);
	clear_bit(bit,addr);

	if(!need_power(addr) && rt5506_query.gpiostatus == AMP_GPIO_ON) {
		rt5506_query.gpio_off_cancel = 0;
		queue_delayed_work(gpio_wq, &rt5506_query.gpio_off_work, msecs_to_jiffies(0));
	}

}

static int rt5506_headset_detect(void *private_data, int on)
{

	if(!rt5506Connect)
		return 0;

	if(on) {
		mutex_lock(&rt5506_query.mlock);
		if(rt5506_query.hs_connec) {
			pr_info("%s: headset exist, ignore\n",__func__);
			mutex_unlock(&rt5506_query.mlock);
			return 0;
		}
		mutex_unlock(&rt5506_query.mlock);

		pr_info("%s: headset in ++\n",__func__);
		cancel_delayed_work_sync(&rt5506_query.hs_imp_detec_work);
		mutex_lock(&rt5506_query.gpiolock);
		mutex_lock(&rt5506_query.mlock);
		rt5506_query.hs_qstatus = QUERY_HEADSET;
		rt5506_query.headsetom = HEADSET_OM_UNDER_DETECT;

		if(rt5506_query.rt5506_status == STATUS_PLAYBACK) {

			if(high_imp) {
				rt5506_write_reg(1,0x7);
				rt5506_write_reg(0xb1,0x81);
			} else {
				rt5506_write_reg(1,0xc7);

			}

			last_spkamp_state = 0;
			pr_info("%s: OFF\n", __func__);

			rt5506_query.rt5506_status = STATUS_SUSPEND;
		}
		rt5506_query.hs_connec = 1;
		pr_info("%s: headset in --\n",__func__);
		mutex_unlock(&rt5506_query.mlock);
		mutex_unlock(&rt5506_query.gpiolock);
		
		queue_delayed_work(hs_wq,&rt5506_query.hs_imp_detec_work,msecs_to_jiffies(5));
		pr_info("%s: headset in --2\n",__func__);

	} else {

		mutex_lock(&rt5506_query.mlock);
		if(!rt5506_query.hs_connec) {
			pr_info("%s: headset isn't exist, ignore\n",__func__);
			mutex_unlock(&rt5506_query.mlock);
			return 0;
		}
		mutex_unlock(&rt5506_query.mlock);

		pr_info("%s: headset remove ++\n",__func__);
		cancel_delayed_work_sync(&rt5506_query.hs_imp_detec_work);
		flush_work_sync(&rt5506_query.volume_ramp_work.work);
		mutex_lock(&rt5506_query.gpiolock);
		mutex_lock(&rt5506_query.mlock);
		rt5506_query.hs_qstatus = QUERY_OFF;
		rt5506_query.headsetom = HEADSET_OM_UNDER_DETECT;

		if(rt5506_query.regstatus == REG_AUTO_MODE) {
			set_rt5506_regulator(REG_PWM_MODE);
			rt5506_query.regstatus = REG_PWM_MODE;
		}

		if(rt5506_query.rt5506_status == STATUS_PLAYBACK) {

			if(high_imp) {
				rt5506_write_reg(1,0x7);
				rt5506_write_reg(0xb1,0x81);

			} else {
				rt5506_write_reg(1,0xc7);

			}

			last_spkamp_state = 0;
			pr_info("%s: OFF\n", __func__);

			rt5506_query.rt5506_status = STATUS_SUSPEND;
		}

		if(high_imp) {
			vote_power(&rt5506_query.power_mask, POWER_HIGH_IMP_RESET);
			pr_info("%s: reset rt5501\n",__func__);
			rt5506_write_reg(0x0,0x4);
			mdelay(1);

			rt5506_write_reg(0x1,0xc7);
			high_imp = 0;
			unvote_power(&rt5506_query.power_mask, POWER_HIGH_IMP_RESET);
		}

		rt5506_query.curmode = PLAYBACK_MODE_OFF;
		rt5506_query.hs_connec = 0;
		pr_info("%s: headset remove --1\n",__func__);


		mutex_unlock(&rt5506_query.mlock);
		mutex_unlock(&rt5506_query.gpiolock);

		pr_info("%s: headset remove --2\n",__func__);

	}

	return 0;
}

static void rt5506_register_hs_notification(void)
{
#if 1
	struct hs_notify_t notifier;
	notifier.private_data = NULL;
	notifier.callback_f = rt5506_headset_detect;
	htc_acoustic_register_hs_notify(HS_AMP_N, &notifier);
#else

	rt5506_headset_detect(NULL,0);
#endif
}

static int rt5506_write_reg(u8 reg, u8 val)
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
        pr_info("%s: write reg 0x%x val 0x%x\n",__func__,data[0],data[1]); 
	err = i2c_transfer(this_client->adapter, msg, 1);
	if (err >= 0)
		return 0;
        else {

            pr_info("%s: write error error %d\n",__func__,err);
            return err;
        }
}

static int rt5506_i2c_write(struct rt5506_reg_data *txData, int length)
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
		
		
		buf[0] = txData[i].addr;
		buf[1] = txData[i].val;

#if DEBUG
		pr_info("%s:i2c_write addr 0x%x val 0x%x\n", __func__,buf[0], buf[1]);
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

static int rt5506_i2c_read_addr(unsigned char *rxData, unsigned char addr)
{
	int rc;
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
		 .len = 1,
		 .buf = rxData,
		},
	};

	if(!rxData)
		return -1;

	*rxData = addr;

	rc = i2c_transfer(this_client->adapter, msgs, 2);
	if (rc < 0) {
		pr_err("%s:[1] transfer error %d\n", __func__, rc);
		return rc;
	}

	pr_info("%s:i2c_read addr 0x%x value = 0x%x\n", __func__, addr, *rxData);
	return 0;
}

static int rt5506_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&hp_amp_lock);

	if (rt5506_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	rt5506_opened = 1;
done:
	mutex_unlock(&hp_amp_lock);
	return rc;
}

static int rt5506_release(struct inode *inode, struct file *file)
{
	mutex_lock(&hp_amp_lock);
	rt5506_opened = 0;
	mutex_unlock(&hp_amp_lock);

	return 0;
}

static void hs_imp_gpio_off(struct work_struct *work)
{
	u64 timeout = get_jiffies_64() + 3*HZ;
	wake_lock(&rt5506_query.gpio_wake_lock);

	while(1) {
		if(time_after64(get_jiffies_64(),timeout))
			break;
		else if(rt5506_query.gpio_off_cancel) {
			wake_unlock(&rt5506_query.gpio_wake_lock);
			return;
		} else
			msleep(10);
	}

	mutex_lock(&rt5506_query.gpiolock);
	if(!need_power(&rt5506_query.power_mask)) {
		pr_info("%s: disable gpio %d\n",__func__,pdata->gpio_rt5506_enable);
		gpio_set_value(pdata->gpio_rt5506_enable, 0);
		rt5506_query.gpiostatus = AMP_GPIO_OFF;

		if(rt5506_query.regstatus == REG_PWM_MODE) {
			set_rt5506_regulator(REG_AUTO_MODE);
			rt5506_query.regstatus = REG_AUTO_MODE;
		}
	}
	mutex_unlock(&rt5506_query.gpiolock);
	wake_unlock(&rt5506_query.gpio_wake_lock);
}

static void hs_imp_detec_func(struct work_struct *work)
{
	struct headset_query *hs;
	unsigned char temp[8]={0x1,};
	unsigned char r_channel;
	int ret;
	pr_info("%s: read rt5506 hs imp \n",__func__);

	hs = container_of(work, struct headset_query, hs_imp_detec_work.work);
	wake_lock(&hs->hs_wake_lock);

	rt5506_query.gpio_off_cancel = 1;
	cancel_delayed_work_sync(&rt5506_query.gpio_off_work);
	mutex_lock(&hs->gpiolock);
	mutex_lock(&hs->mlock);

	if(hs->hs_qstatus != QUERY_HEADSET) {
		mutex_unlock(&hs->mlock);
		mutex_unlock(&hs->gpiolock);
		wake_unlock(&hs->hs_wake_lock);
		return;
	}

	vote_power(&rt5506_query.power_mask, POWER_IMPEDANCE);

	rt5506_write_reg(0,0x04);
	rt5506_write_reg(0xa4,0x52);
	rt5506_write_reg(1,0x7);
	msleep(10);
	rt5506_write_reg(0x3,0x81);

	msleep(101);

	ret = rt5506_i2c_read_addr(temp,0x4);

	if(ret < 0) {
		pr_err("%s: read rt5506 status error %d\n",__func__,ret);

		unvote_power(&rt5506_query.power_mask, POWER_IMPEDANCE);
		mutex_unlock(&hs->mlock);
		mutex_unlock(&hs->gpiolock);
		wake_unlock(&hs->hs_wake_lock);
		return;
	}

	rt5506_i2c_read_addr(&r_channel,0x6);
	rt5506_write_reg(0x0,0x4);
	mdelay(1);

	rt5506_write_reg(0x0,0xc0);
	rt5506_write_reg(0x81,0x30);
	rt5506_write_reg(0x90,0xd0);
	rt5506_write_reg(0x93,0x9d);
	rt5506_write_reg(0x95,0x7b);
	rt5506_write_reg(0xa4,0x52);
	rt5506_write_reg(0x97,0x00);
	rt5506_write_reg(0x98,0x22);
	rt5506_write_reg(0x99,0x33);
	rt5506_write_reg(0x9a,0x55);
	rt5506_write_reg(0x9b,0x66);
	rt5506_write_reg(0x9c,0x99);
	rt5506_write_reg(0x9d,0x66);
	rt5506_write_reg(0x9e,0x99);


	high_imp = 0;

	if(temp[0] & AMP_SENSE_READY) {

		unsigned char om, hsmode;
		enum HEADSET_OM hsom;

		hsmode = (temp[0] & 0x30) >> 4;
		om = (temp[0] & 0xe) >> 1;

		if(r_channel == 0) {
			
			hsom = HEADSET_MONO;
		} else {

			switch(om) {
				case 0:
					hsom = HEADSET_8OM;
					break;
				case 1:
					hsom = HEADSET_16OM;
					break;
				case 2:
					hsom = HEADSET_32OM;
					break;
				case 3:
					hsom = HEADSET_64OM;
					break;
				case 4:
					hsom = HEADSET_128OM;
					break;
				case 5:
					hsom = HEADSET_256OM;
					break;
				case 6:
					hsom = HEADSET_500OM;
					break;
				case 7:
					hsom = HEADSET_1KOM;
					break;

				default:
					hsom = HEADSET_OM_UNDER_DETECT;
					break;
			}
		}

		hs->hs_qstatus = QUERY_FINISH;
		hs->headsetom = hsom;

		if(om >= HEADSET_256OM && om <= HEADSET_1KOM)
			high_imp = 1;

		pr_info("rt5506 hs imp value 0x%x hsmode %d om 0x%x hsom %d r_channel 0x%x high_imp %d\n",\
			temp[0] & 0xf,hsmode,om,hsom,r_channel,high_imp);
	} else {

		if(hs->hs_qstatus == QUERY_HEADSET)
			queue_delayed_work(hs_wq,&rt5506_query.hs_imp_detec_work,QUERY_LATTER);
	}

	if(high_imp) {
		rt5506_write_reg(0xb1,0x81);
		rt5506_write_reg(0x80,0x87);
		rt5506_write_reg(0x83,0xc3);
		rt5506_write_reg(0x84,0x63);
		rt5506_write_reg(0x89,0x7);
		mdelay(9);
		rt5506_write_reg(0x83,0xcf);
		rt5506_write_reg(0x89,0x1d);
		mdelay(1);

		rt5506_write_reg(1,0x7);
		rt5506_write_reg(0xb1,0x81);
	} else {

		rt5506_write_reg(1,0xc7);
	}

	if(hs->rt5506_status == STATUS_PLAYBACK)
		hs->rt5506_status = STATUS_SUSPEND;

	unvote_power(&rt5506_query.power_mask, POWER_IMPEDANCE);
	mutex_unlock(&hs->mlock);
	mutex_unlock(&hs->gpiolock);

	if(hs->rt5506_status == STATUS_SUSPEND)
		set_rt5506_amp(1,0);

	wake_unlock(&hs->hs_wake_lock);
}

static void volume_ramp_func(struct work_struct *work)
{
	set_amp(1, &RT5506_AMP_ON);
}

static void set_amp(int on, struct rt5506_config *i2c_command)
{
	pr_info("%s: %d\n", __func__, on);
	mutex_lock(&rt5506_query.mlock);
	mutex_lock(&hp_amp_lock);

	if(rt5506_query.hs_qstatus == QUERY_HEADSET)
		rt5506_query.hs_qstatus = QUERY_FINISH;

	if (on) {
		if(rt5506_query.rt5506_status != STATUS_PLAYBACK) {

			mdelay(1);
			
			if(high_imp)
				rt5506_write_reg(0xb1,0x80);

			rt5506_write_reg(0x2,0x0);
			mdelay(1);
		}

		rt5506_query.rt5506_status = STATUS_PLAYBACK;
		if (rt5506_i2c_write(i2c_command->reg, i2c_command->reg_len) == 0) {
			last_spkamp_state = 1;
			pr_info("%s: ON \n",__func__);
		}

	} else {

		if(high_imp) {
			rt5506_write_reg(1,0x7);
			rt5506_write_reg(0xb1,0x81);
		} else {
			rt5506_write_reg(1,0xc7);
		}

		if(rt5506_query.rt5506_status == STATUS_PLAYBACK) {
			last_spkamp_state = 0;
			pr_info("%s: OFF\n", __func__);
		}
		rt5506_query.rt5506_status = STATUS_OFF;
		rt5506_query.curmode = PLAYBACK_MODE_OFF;
	}
	mutex_unlock(&hp_amp_lock);
	mutex_unlock(&rt5506_query.mlock);
}

int query_rt5506(void)
{
    return rt5506Connect;
}

static int set_rt5506_amp(int on, int dsp)
{
	if(!rt5506Connect)
		return 0;

	pr_info("%s: %d\n", __func__, on);
	mutex_lock(&rt5506_query.actionlock);
	rt5506_query.gpio_off_cancel = 1;
	if(!on)
		rt5506_query.action_on = 0;
	cancel_delayed_work_sync(&rt5506_query.gpio_off_work);
	cancel_delayed_work_sync(&rt5506_query.volume_ramp_work);
	
	mutex_lock(&rt5506_query.gpiolock);

	if(on) {
		vote_power(&rt5506_query.power_mask, POWER_PLAYBACK);
		rt5506_query.action_on = 1;
		queue_delayed_work(ramp_wq, &rt5506_query.volume_ramp_work, msecs_to_jiffies(0));

	} else {
		set_amp(0, &RT5506_AMP_ON);
		unvote_power(&rt5506_query.power_mask, POWER_PLAYBACK);
	}

	mutex_unlock(&rt5506_query.gpiolock);
	mutex_unlock(&rt5506_query.actionlock);

	return 0;
}

static int update_amp_parameter(int mode)
{
	if (mode >= rt5506_config_data.mode_num)
		return -EINVAL;

        pr_info("%s: set mode %d\n", __func__, mode);

	if (mode == PLAYBACK_MODE_OFF)
		memcpy(&RT5506_AMP_OFF, &rt5506_config_data.cmd_data[mode].config,
				sizeof(struct rt5506_config));
	else if (mode == AMP_INIT)
		memcpy(&RT5506_AMP_INIT, &rt5506_config_data.cmd_data[mode].config,
				sizeof(struct rt5506_config));
	else if (mode == AMP_MUTE)
		memcpy(&RT5506_AMP_MUTE, &rt5506_config_data.cmd_data[mode].config,
				sizeof(struct rt5506_config));
	else {
		memcpy(&RT5506_AMP_ON, &rt5506_config_data.cmd_data[mode].config,
				sizeof(struct rt5506_config));
	}
	return 0;
}


static long rt5506_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0, modeid = 0;
	int premode = 0;
	struct amp_ctrl ampctrl;
	struct rt5506_reg_data reg;

	switch (cmd) {
	case AMP_SET_MODE:
		if (copy_from_user(&modeid, argp, sizeof(modeid)))
			return -EFAULT;

		if (!rt5506_config_data.cmd_data) {
			pr_err("%s: out of memory\n", __func__);
			return -ENOMEM;
		}

		if (modeid >= rt5506_config_data.mode_num || modeid < 0) {
			pr_err("unsupported rt5506 mode %d\n", modeid);
			return -EINVAL;
		}
		mutex_lock(&hp_amp_lock);
		premode = rt5506_query.curmode;
		rt5506_query.curmode = modeid;
		rc = update_amp_parameter(modeid);
		mutex_unlock(&hp_amp_lock);
		pr_info("%s:set rt5506 mode to %d curstatus %d\n", __func__,modeid,rt5506_query.rt5506_status);
		mutex_lock(&rt5506_query.actionlock);
		if(rt5506_query.rt5506_status == STATUS_PLAYBACK && premode != rt5506_query.curmode) {
			flush_work_sync(&rt5506_query.volume_ramp_work.work);
			rt5506_query.action_on = 1;
			queue_delayed_work(ramp_wq, &rt5506_query.volume_ramp_work, msecs_to_jiffies(280));
		}
		mutex_unlock(&rt5506_query.actionlock);
		break;
	case AMP_SET_PARAM:
		mutex_lock(&hp_amp_lock);
		if (copy_from_user(&rt5506_config_data.mode_num, argp, sizeof(unsigned int))) {
			pr_err("%s: copy from user failed.\n", __func__);
			mutex_unlock(&hp_amp_lock);
			return -EFAULT;
		}

		if (rt5506_config_data.mode_num <= 0) {
			pr_err("%s: invalid mode number %d\n",
					__func__, rt5506_config_data.mode_num);
			mutex_unlock(&hp_amp_lock);
			return -EINVAL;
		}
		if (rt5506_config_data.cmd_data == NULL)
			rt5506_config_data.cmd_data = kzalloc(sizeof(struct rt5506_comm_data)*rt5506_config_data.mode_num, GFP_KERNEL);

		if (!rt5506_config_data.cmd_data) {
			pr_err("%s: out of memory\n", __func__);
			mutex_unlock(&hp_amp_lock);
			return -ENOMEM;
		}

		if (copy_from_user(rt5506_config_data.cmd_data, ((struct rt5506_config_data*)argp)->cmd_data \
			,sizeof(struct rt5506_comm_data)*rt5506_config_data.mode_num)) {
			pr_err("%s: copy data from user failed.\n", __func__);
			kfree(rt5506_config_data.cmd_data);
			rt5506_config_data.cmd_data = NULL;
			mutex_unlock(&hp_amp_lock);
			return -EFAULT;
		}

		pr_info("%s: update rt5506 i2c commands #%d success.\n",
				__func__, rt5506_config_data.mode_num);
		
		update_amp_parameter(PLAYBACK_MODE_OFF);
		update_amp_parameter(AMP_MUTE);
		update_amp_parameter(AMP_INIT);
		mutex_unlock(&hp_amp_lock);
		rc = 0;
		break;
	case AMP_QUERY_OM:
		mutex_lock(&rt5506_query.mlock);
		rc = rt5506_query.headsetom;
		mutex_unlock(&rt5506_query.mlock);
		pr_info("%s: query headset om %d\n", __func__,rc);

		if (copy_to_user(argp, &rc, sizeof(rc)))
			rc = -EFAULT;
		else
			rc = 0;
		break;
	case ACOUSTIC_AMP_CTRL:
		if (copy_from_user(&ampctrl, argp, sizeof(ampctrl)))
			return -EFAULT;

		if(!this_client)
			return -EFAULT;

		if(ampctrl.slave != AUD_AMP_SLAVE_ALL && ampctrl.slave != this_client->addr)
			break;

		mutex_lock(&rt5506_query.gpiolock);
		mutex_lock(&rt5506_query.mlock);
		mutex_lock(&hp_amp_lock);

		rc = 0;
		vote_power(&rt5506_query.power_mask, POWER_CMDLINE_TOOL);
		if(ampctrl.ctrl == AMP_WRITE) {
			reg.addr = (unsigned char)ampctrl.reg;
			reg.val = (unsigned char)ampctrl.val;
			rt5506_write_reg(reg.addr,reg.val);
		} else if (ampctrl.ctrl == AMP_READ) {
			reg.addr = (unsigned char)ampctrl.reg;
			rt5506_i2c_read_addr(&reg.val, reg.addr);
			ampctrl.val = (unsigned int)reg.val;

			if (copy_to_user(argp, &ampctrl, sizeof(ampctrl)))
				rc = -EFAULT;
		}
		unvote_power(&rt5506_query.power_mask, POWER_CMDLINE_TOOL);
		mutex_unlock(&hp_amp_lock);
		mutex_unlock(&rt5506_query.mlock);
		mutex_unlock(&rt5506_query.gpiolock);
		break;
	default:
		pr_err("%s: Invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int rt550_parse_pfdata(struct device *dev, struct rt5506_platform_data *ppdata)
{
	struct device_node *dt = dev->of_node;
	enum of_gpio_flags flags;
	int ret;

	pdata->gpio_rt5506_enable = -EINVAL;
	pdata->power_supply = NULL;
	pdata->power_reg = NULL;

	if (dt) {
		pdata->gpio_rt5506_enable = of_get_named_gpio_flags(dt,"richtek,enable-gpio",0, &flags);
		ret = of_property_read_string(dt,"power_supply",&pdata->power_supply);

		if(ret < 0) {
			pdata->power_supply = NULL;
			pr_err("%s:parse power supply fail\n",__func__);
		}

	} else {
		if(dev->platform_data) {
			pdata->gpio_rt5506_enable = ((struct rt5506_platform_data *)dev->platform_data)->gpio_rt5506_enable;
			pdata->power_supply = ((struct rt5506_platform_data *)dev->platform_data)->power_supply;
		}
	}

	pr_info("%s: rt5506 gpio %d\n",__func__,pdata->gpio_rt5506_enable);

	if(pdata->power_supply)
		pr_info("%s:power supply %s\n",__func__,pdata->power_supply);

	if(pdata->power_supply != NULL) {
		pdata->power_reg = rpm_regulator_get(NULL, pdata->power_supply);

		if (IS_ERR(pdata->power_reg)) {
			pdata->power_reg = NULL;
			pr_err("%s: reqest regulator %s fail\n",__func__,pdata->power_supply);
		}
	}

	if(gpio_is_valid(pdata->gpio_rt5506_enable))
		return 0;
	else
		return -EINVAL;
}

static struct file_operations rt5506_fops = {
	.owner = THIS_MODULE,
	.open = rt5506_open,
	.release = rt5506_release,
	.unlocked_ioctl = rt5506_ioctl,
};

static struct miscdevice rt5506_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rt5501",
	.fops = &rt5506_fops,
};

int rt5506_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int err = 0;

	pr_info("rt5506_probe");
	htc_amp_power_enable(true);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_alloc_data_failed;
	}

	if (pdata == NULL) {

		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			pr_err("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	if(rt550_parse_pfdata(&client->dev, pdata) < 0)
		goto err_free_allocated_mem;

	this_client = client;

	if(1) {
		unsigned char temp[2];

		err = gpio_request(pdata->gpio_rt5506_enable, "hp_en_rt5506");
		if(err)
			pr_err("%s:gpio request %d error %d\n",__func__,pdata->gpio_rt5506_enable,err);
#if 1
		pr_info("%s:[1]current gpio %d value %d\n",__func__,pdata->gpio_rt5506_enable,gpio_get_value(pdata->gpio_rt5506_enable));
		ret = gpio_direction_output(pdata->gpio_rt5506_enable, 1);

		if(ret < 0) {
			pr_err("%s: gpio %d on error %d\n", __func__,pdata->gpio_rt5506_enable,ret);
		}
#endif
#if 0
		gpio_direction_output(pdata->gpio_rt5506_enable, 0);
		pr_info("%s:[t-1]current gpio %d value %d\n",__func__,pdata->gpio_rt5506_enable,gpio_get_value(pdata->gpio_rt5506_enable));
		mdelay(10);

		gpio_direction_output(pdata->gpio_rt5506_enable, 1);
		pr_info("%s:[t-2]current gpio %d value %d\n",__func__,pdata->gpio_rt5506_enable,gpio_get_value(pdata->gpio_rt5506_enable));
#endif
		mdelay(10);
		pr_info("%s:[2]current gpio %d value %d\n",__func__,pdata->gpio_rt5506_enable,gpio_get_value(pdata->gpio_rt5506_enable));
		rt5506_write_reg(0,0x04);
		mdelay(5);
		rt5506_write_reg(0x0,0xc0);
		rt5506_write_reg(0x81,0x30);
		
		rt5506_write_reg(0x90,0xd0);
		rt5506_write_reg(0x93,0x9d);
		rt5506_write_reg(0x95,0x7b);
		rt5506_write_reg(0xa4,0x52);
		
		rt5506_write_reg(0x97,0x00);
		rt5506_write_reg(0x98,0x22);
		rt5506_write_reg(0x99,0x33);
		rt5506_write_reg(0x9a,0x55);
		rt5506_write_reg(0x9b,0x66);
		rt5506_write_reg(0x9c,0x99);
		rt5506_write_reg(0x9d,0x66);
		rt5506_write_reg(0x9e,0x99);

		rt5506_write_reg(0x1,0xc7);
		mdelay(10);
		ret = rt5506_i2c_read_addr(temp, 0x1);
		if(ret < 0) {
			pr_info("rt5506 is not connected\n");
			rt5506Connect = 0;
		} else {
			pr_info("rt5506 is connected\n");
			rt5506Connect = 1;
		}
		rt5506Connect = 1;

		gpio_set_value(pdata->gpio_rt5506_enable, 0);

	}

	if(rt5506Connect) {
		htc_acoustic_register_hs_amp(set_rt5506_amp,&rt5506_fops);
		ret = misc_register(&rt5506_device);
		if (ret) {
			pr_err("%s: rt5506_device register failed\n", __func__);
			goto err_free_allocated_mem;
		}

		hs_wq = create_workqueue("rt5506_hsdetect");
		INIT_DELAYED_WORK(&rt5506_query.hs_imp_detec_work,hs_imp_detec_func);
		wake_lock_init(&rt5506_query.hs_wake_lock, WAKE_LOCK_SUSPEND, "rt5506 hs wakelock");
		wake_lock_init(&rt5506_query.gpio_wake_lock, WAKE_LOCK_SUSPEND, "rt5506 gpio wakelock");
		ramp_wq = create_workqueue("rt5506_volume_ramp");
		INIT_DELAYED_WORK(&rt5506_query.volume_ramp_work, volume_ramp_func);
		gpio_wq = create_workqueue("rt5506_gpio_off");
		INIT_DELAYED_WORK(&rt5506_query.gpio_off_work, hs_imp_gpio_off);
		rt5506_register_hs_notification();
		

	}
	return 0;

err_free_allocated_mem:
	if(pdata)
		kfree(pdata);
err_alloc_data_failed:
        rt5506Connect = 0;
	return ret;
}

static int rt5506_remove(struct i2c_client *client)
{
	struct rt5506_platform_data *p5501data = i2c_get_clientdata(client);
	pr_info("%s:\n",__func__);
	if(p5501data)
		kfree(p5501data);

        if(rt5506Connect) {
            misc_deregister(&rt5506_device);
            cancel_delayed_work_sync(&rt5506_query.hs_imp_detec_work);
            destroy_workqueue(hs_wq);
        }
	return 0;
}

static void rt5506_shutdown(struct i2c_client *client)
{
	rt5506_query.gpio_off_cancel = 1;
	cancel_delayed_work_sync(&rt5506_query.gpio_off_work);
	cancel_delayed_work_sync(&rt5506_query.volume_ramp_work);

	mutex_lock(&rt5506_query.gpiolock);
	mutex_lock(&hp_amp_lock);
	mutex_lock(&rt5506_query.mlock);

	if(rt5506_query.gpiostatus == AMP_GPIO_OFF) {

		if(rt5506_query.regstatus == REG_AUTO_MODE) {
			set_rt5506_regulator(REG_PWM_MODE);
			rt5506_query.regstatus = REG_PWM_MODE;
			msleep(1);
		}

		pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5506_enable);
		gpio_set_value(pdata->gpio_rt5506_enable, 1);
		rt5506_query.gpiostatus = AMP_GPIO_ON;
		usleep_range(20000,20000);
	}
	pr_info("%s: reset rt5501\n",__func__);
	rt5506_write_reg(0x0,0x4);
	mdelay(1);

	high_imp = 0;

	rt5506_query.power_mask = 0;

	if(rt5506_query.gpiostatus == AMP_GPIO_ON) {
		pr_info("%s: disable gpio %d\n",__func__,pdata->gpio_rt5506_enable);
		gpio_set_value(pdata->gpio_rt5506_enable, 0);
		rt5506_query.gpiostatus = AMP_GPIO_OFF;

		if(rt5506_query.regstatus == REG_PWM_MODE) {
			set_rt5506_regulator(REG_AUTO_MODE);
			rt5506_query.regstatus = REG_AUTO_MODE;
		}
	}

	rt5506Connect = 0;

	mutex_unlock(&rt5506_query.mlock);
	mutex_unlock(&hp_amp_lock);
	mutex_unlock(&rt5506_query.gpiolock);

}

static int rt5506_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int rt5506_resume(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id rt5506_match_table[] = {
	{ .compatible = "richtek,rt5506-amp",},
	{ },
};

static const struct i2c_device_id rt5506_id[] = {
	{ RT5506_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver rt5506_driver = {
	.probe = rt5506_probe,
	.remove = rt5506_remove,
	.shutdown = rt5506_shutdown,
	.suspend = rt5506_suspend,
	.resume = rt5506_resume,
	.id_table = rt5506_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name = RT5506_I2C_NAME,
		.of_match_table = rt5506_match_table,
	},
};

static int __init rt5506_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&hp_amp_lock);
	mutex_init(&rt5506_query.mlock);
	mutex_init(&rt5506_query.gpiolock);
	mutex_init(&rt5506_query.actionlock);
	rt5506_query.rt5506_status = STATUS_OFF;
	rt5506_query.hs_qstatus = QUERY_OFF;
	rt5506_query.headsetom = HEADSET_8OM;
	rt5506_query.curmode = PLAYBACK_MODE_OFF;
	rt5506_query.gpiostatus = AMP_GPIO_OFF;
	rt5506_query.regstatus = REG_AUTO_MODE;
	rt5506_query.hs_connec = 0;
	rt5506_query.power_mask = 0;
	return i2c_add_driver(&rt5506_driver);
}

static void __exit rt5506_exit(void)
{
	i2c_del_driver(&rt5506_driver);

	if(pdata->power_reg)
		rpm_regulator_put(pdata->power_reg);
}

module_init(rt5506_init);
module_exit(rt5506_exit);

MODULE_DESCRIPTION("rt5506 Headphone Amp driver");
MODULE_LICENSE("GPL");

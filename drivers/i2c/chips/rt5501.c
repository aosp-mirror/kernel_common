/* driver/i2c/chip/tap6185.c
 *
 * TI rt5501 Speaker Amp
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
#include <mach/rt5501.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/htc_headset_mgr.h>
#include <linux/wakelock.h>
#include <mach/htc_acoustic_pmic.h>
#include <linux/jiffies.h>

//htc audio ++
#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)
//htc audio --

#ifdef CONFIG_AMP_RT5501_ON_GPIO
#define DEBUG (1)
#else
#define DEBUG (1)
#endif
#define AMP_ON_CMD_LEN 7
#define RETRY_CNT 5

#define DRIVER_NAME "RT5501"

struct headset_query {
	struct mutex mlock;
	struct mutex gpiolock;
	struct delayed_work hs_imp_detec_work;
	struct wake_lock hs_wake_lock;
	struct wake_lock gpio_wake_lock;
	enum HEADSET_QUERY_STATUS hs_qstatus;
	enum RT5501_STATUS rt5501_status;
	enum HEADSET_OM headsetom;
	enum RT5501_Mode curmode;
	enum AMP_GPIO_STATUS gpiostatus;
	enum AMP_S4_STATUS s4status;
	int action_on;
	int gpio_off_cancel;
	struct mutex actionlock;
	struct delayed_work volume_ramp_work;
	struct delayed_work gpio_off_work;
};

static struct i2c_client *this_client;
static struct rt5501_platform_data *pdata;
static int rt5501Connect = 0;
static int MFG_MODE = 0;

struct rt5501_config_data rt5501_config_data;
static struct mutex hp_amp_lock;
static int rt5501_opened;
static int last_spkamp_state;
struct rt5501_config RT5501_AMP_ON = {6,{{0x1,0x1c},{0x2,0x00},{0x7,0x7f},{0x9,0x1},{0xa,0x0},{0xb,0xc7},}};
struct rt5501_config RT5501_AMP_INIT = {11,{{0,0xc0},{0x81,0x30},{0x87,0xf6},{0x93,0x8d},{0x95,0x7d},{0xa4,0x52},\
                                        {0x96,0xae},{0x97,0x13},{0x99,0x35},{0x9b,0x68},{0x9d,0x68},}};

struct rt5501_config RT5501_AMP_MUTE = {1,{{0x1,0xC7},}};;
struct rt5501_config RT5501_AMP_OFF = {1,{{0x0,0x1},}};

static int rt5501_write_reg(u8 reg, u8 val);
static int rt5501_i2c_write_for_read(char *txData, int length);
static int rt5501_i2c_read(char *rxData, int length);
static void hs_imp_detec_func(struct work_struct *work);
static int rt5501_i2c_read_addr(char *rxData, unsigned char addr);
static int rt5501_i2c_write(struct rt5501_reg_data *txData, int length);
static void set_amp(int on, struct rt5501_config *i2c_command);

struct headset_query rt5501_query;
static struct workqueue_struct *hs_wq;

static struct workqueue_struct *ramp_wq;
static struct workqueue_struct *gpio_wq;
static int high_imp = 0;
static u64 last_hp_remove = 0;
#if 0
static int query_playback(void *pdata)
{
    return 0;
}
#endif

static int rt5501_headset_detect(int on)
{

	if(on) {
		pr_info("%s: headset in ++\n",__func__);
		mutex_lock(&rt5501_query.mlock);

		if(rt5501_query.headsetom == HEADSET_OM_UNDER_DETECT || \
			time_after64(get_jiffies_64(),last_hp_remove + msecs_to_jiffies(500))) {
			rt5501_query.hs_qstatus = RT5501_QUERY_HEADSET;
			rt5501_query.headsetom = HEADSET_OM_UNDER_DETECT;
		} else {
			rt5501_query.hs_qstatus = RT5501_QUERY_FINISH;
		}

		mutex_unlock(&rt5501_query.mlock);

		cancel_delayed_work_sync(&rt5501_query.hs_imp_detec_work);

		mutex_lock(&rt5501_query.gpiolock);
		mutex_lock(&rt5501_query.mlock);

		if(rt5501_query.rt5501_status == RT5501_PLAYBACK) {

			if(high_imp) {
				rt5501_write_reg(1,0x7);
				rt5501_write_reg(0xb1,0x81);
			} else {
				rt5501_write_reg(1,0xc7);

			}
			last_spkamp_state = 0;
			pr_info("%s: OFF\n", __func__);
			rt5501_query.rt5501_status = RT5501_SUSPEND;
		}
		pr_info("%s: headset in --\n",__func__);
		mutex_unlock(&rt5501_query.mlock);
		mutex_unlock(&rt5501_query.gpiolock);

		queue_delayed_work(hs_wq,&rt5501_query.hs_imp_detec_work,msecs_to_jiffies(5));
		pr_info("%s: headset in --2\n",__func__);

	} else {

		pr_info("%s: headset remove ++\n",__func__);
		flush_work_sync(&rt5501_query.volume_ramp_work.work);
		mutex_lock(&rt5501_query.mlock);
		rt5501_query.hs_qstatus = RT5501_QUERY_OFF;
		mutex_unlock(&rt5501_query.mlock);

		cancel_delayed_work_sync(&rt5501_query.hs_imp_detec_work);

		mutex_lock(&rt5501_query.gpiolock);
		mutex_lock(&rt5501_query.mlock);


		if(rt5501_query.rt5501_status == RT5501_PLAYBACK) {

			if(high_imp) {
				rt5501_write_reg(1,0x7);
				rt5501_write_reg(0xb1,0x81);
			} else {
				rt5501_write_reg(1,0xc7);

			}

			last_spkamp_state = 0;
			pr_info("%s: OFF\n", __func__);

			rt5501_query.rt5501_status = RT5501_SUSPEND;
		}

		rt5501_query.curmode = RT5501_MODE_OFF;
		pr_info("%s: headset remove --1\n",__func__);

		if(high_imp) {
			int closegpio = 0;
			if((rt5501_query.gpiostatus == AMP_GPIO_OFF) && pdata->gpio_rt5501_spk_en) {

				if(rt5501_query.s4status == AMP_S4_AUTO) {
					pm8921_aud_set_s4_pwm();
					rt5501_query.s4status = AMP_S4_PWM;
					msleep(1);
				}

				pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
				gpio_direction_output(pdata->gpio_rt5501_spk_en, 1);
				rt5501_query.gpiostatus = AMP_GPIO_ON;
				closegpio = 1;
				msleep(1);
			}
			pr_info("%s: reset rt5501\n",__func__);
			rt5501_write_reg(0x0,0x4);
			mdelay(1);

			rt5501_write_reg(0x1,0xc7);
			high_imp = 0;

			if(closegpio && (rt5501_query.gpiostatus == AMP_GPIO_ON) && pdata->gpio_rt5501_spk_en) {
				pr_info("%s: disable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
				gpio_direction_output(pdata->gpio_rt5501_spk_en, 0);
				rt5501_query.gpiostatus = AMP_GPIO_OFF;

				if(rt5501_query.s4status == AMP_S4_PWM) {
					pm8921_aud_set_s4_auto();
					rt5501_query.s4status = AMP_S4_AUTO;
				}
			}
		}
		last_hp_remove = get_jiffies_64();
		mutex_unlock(&rt5501_query.mlock);
		mutex_unlock(&rt5501_query.gpiolock);

		pr_info("%s: headset remove --2\n",__func__);
	}

	return 0;
}

static int rt5501_write_reg(u8 reg, u8 val)
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

static int rt5501_i2c_write(struct rt5501_reg_data *txData, int length)
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
		//if (i == 2)  /* According to rt5501 Spec */
		//	mdelay(1);
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

static int rt5501_i2c_write_for_read(char *txData, int length)
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
		//if (i == 2)  /* According to rt5501 Spec */
		//	mdelay(1);
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

static int rt5501_i2c_read(char *rxData, int length)
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
			pr_info("i2c_read %s: rx[%d] = 0x%x\n", __func__, i, \
				rxData[i]);
	}

	return 0;
}

static int rt5501_i2c_read_addr(char *rxData, unsigned char addr)
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

	rc = i2c_transfer(this_client->adapter, &msgs[0], 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

	rc = i2c_transfer(this_client->adapter, &msgs[1], 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

	/*{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("i2c_read %s: rx[%d] = %2x\n", __func__, i, \
				rxData[i]);
	}*/
        pr_info("%s:i2c_read addr 0x%x value = 0x%x\n", __func__, addr, *rxData);
	return 0;
}

static int rt5501_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&hp_amp_lock);

	if (rt5501_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	rt5501_opened = 1;
done:
	mutex_unlock(&hp_amp_lock);
	return rc;
}

static int rt5501_release(struct inode *inode, struct file *file)
{
	mutex_lock(&hp_amp_lock);
	rt5501_opened = 0;
	mutex_unlock(&hp_amp_lock);

	return 0;
}
#if 0
static int init_rt5501(void)
{
    int ret;

    ret = rt5501_i2c_write(RT5501_AMP_INIT.reg, RT5501_AMP_INIT.reg_len);

    if(ret < 0) {
        pr_err("init rt5501 error %d\n",ret);
        return ret;
    }
#if 0
    ret = rt5501_i2c_write(RT5501_AMP_ON.reg, RT5501_AMP_ON.reg_len);

    if(ret < 0) {
        pr_err("init rt5501 to playback error %d\n",ret);
        return ret;
    }

    ret = rt5501_i2c_write(RT5501_AMP_MUTE.reg, RT5501_AMP_MUTE.reg_len);

    if(ret < 0) {
        pr_err("init rt5501 to mute error %d\n",ret);
        return ret;
    }

    ret = rt5501_i2c_write(RT5501_AMP_OFF.reg, RT5501_AMP_OFF.reg_len);

    if(ret < 0) {
        pr_err("init rt5501 to off error %d\n",ret);
        return ret;
    }
#endif
    return ret;
}
#endif
static void hs_imp_gpio_off(struct work_struct *work)
{
    u64 timeout = get_jiffies_64() + 5*HZ;
    wake_lock(&rt5501_query.gpio_wake_lock);

    while(1) {
        if(time_after64(get_jiffies_64(),timeout))
            break;
        else if(rt5501_query.gpio_off_cancel) {
            wake_unlock(&rt5501_query.gpio_wake_lock);
            return;
        } else
            msleep(10);
    }

    mutex_lock(&rt5501_query.gpiolock);
    pr_info("%s: disable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
    gpio_direction_output(pdata->gpio_rt5501_spk_en, 0);
    rt5501_query.gpiostatus = AMP_GPIO_OFF;

    if(rt5501_query.s4status == AMP_S4_PWM) {
        pm8921_aud_set_s4_auto();
        rt5501_query.s4status = AMP_S4_AUTO;
    }

    mutex_unlock(&rt5501_query.gpiolock);
    wake_unlock(&rt5501_query.gpio_wake_lock);
}

static void hs_imp_detec_func(struct work_struct *work)
{
	struct headset_query *hs;
	char temp[8]={0x1,};
	int ret;
	int rt5501_status;
	pr_info("%s: read rt5501 hs imp \n",__func__);

	hs = container_of(work, struct headset_query, hs_imp_detec_work.work);
	wake_lock(&hs->hs_wake_lock);

	rt5501_query.gpio_off_cancel = 1;
	cancel_delayed_work_sync(&rt5501_query.gpio_off_work);
	mutex_lock(&hs->gpiolock);
	mutex_lock(&hs->mlock);

	if((hs->gpiostatus == AMP_GPIO_OFF) && pdata->gpio_rt5501_spk_en) {

		if(rt5501_query.s4status == AMP_S4_AUTO) {
			pm8921_aud_set_s4_pwm();
			rt5501_query.s4status = AMP_S4_PWM;
			msleep(1);
		}
		pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
		gpio_direction_output(pdata->gpio_rt5501_spk_en, 1);
		rt5501_query.gpiostatus = AMP_GPIO_ON;
	}

	msleep(1);

	if(hs->hs_qstatus == RT5501_QUERY_HEADSET) {
		rt5501_write_reg(0,0x04);
		rt5501_write_reg(0xa4,0x52);

		rt5501_write_reg(1,0x7);
		msleep(10);
		rt5501_write_reg(0x3,0x81);

		msleep(101);

		ret = rt5501_i2c_read_addr(temp,0x4);

		if(ret < 0) {
			pr_err("%s: read rt5501 status error %d\n",__func__,ret);

			if((hs->gpiostatus == AMP_GPIO_ON) && pdata->gpio_rt5501_spk_en) {

				rt5501_query.gpio_off_cancel = 0;
				queue_delayed_work(gpio_wq, &rt5501_query.gpio_off_work, msecs_to_jiffies(0));
			}

			mutex_unlock(&hs->mlock);
			mutex_unlock(&hs->gpiolock);
			wake_unlock(&hs->hs_wake_lock);
			return;
		}

		temp[1] = 0x4;
		rt5501_i2c_read_addr(&temp[1],0x6);

		if(temp[0] & RT5501_SENSE_READY) {

			unsigned char om, hsmode;
			enum HEADSET_OM hsom;

			high_imp = 0;
			hsmode = (temp[0] & 0x30) >> 4;
			om = (temp[0] & 0xe) >> 1;

			if((temp[0] == 0xc0 || temp[0] == 0xc1) && (temp[1] == 0)) {
				//mono headset
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

			hs->hs_qstatus = RT5501_QUERY_FINISH;
			hs->headsetom = hsom;

			if(om >= HEADSET_256OM && om <= HEADSET_1KOM)
				high_imp = 1;

			pr_info("rt5501 hs imp value 0x%x hsmode %d om 0x%x hsom %d high_imp %d\n", \
				temp[0] & 0xf,hsmode,om,hsom,high_imp);

		} else {

			if(hs->hs_qstatus == RT5501_QUERY_HEADSET)
				queue_delayed_work(hs_wq,&rt5501_query.hs_imp_detec_work,QUERY_LATTER);
		}

	}

	rt5501_write_reg(0x0,0x4);
	mdelay(1);

	rt5501_write_reg(0x0,0xc0);
	rt5501_write_reg(0x81,0x30);
	rt5501_write_reg(0x90,0xd0);
	rt5501_write_reg(0x93,0x9d);
	rt5501_write_reg(0x95,0x7b);
	rt5501_write_reg(0xa4,0x52);
	rt5501_write_reg(0x97,0x00);
	rt5501_write_reg(0x98,0x22);
	rt5501_write_reg(0x99,0x33);
	rt5501_write_reg(0x9a,0x55);
	rt5501_write_reg(0x9b,0x66);
	rt5501_write_reg(0x9c,0x99);
	rt5501_write_reg(0x9d,0x66);
	rt5501_write_reg(0x9e,0x99);

	rt5501_status = hs->rt5501_status;

	if(high_imp) {
		rt5501_write_reg(0xb1,0x81);
		rt5501_write_reg(0x80,0x87);
		rt5501_write_reg(0x83,0xc3);
		rt5501_write_reg(0x84,0x63);
		rt5501_write_reg(0x89,0x7);
		mdelay(9);
		rt5501_write_reg(0x83,0xcf);
		rt5501_write_reg(0x89,0x1d);
		mdelay(1);

		rt5501_write_reg(1,0x7);
		rt5501_write_reg(0xb1,0x81);
	} else {

		rt5501_write_reg(1,0xc7);
	}


	if((hs->gpiostatus == AMP_GPIO_ON) && pdata->gpio_rt5501_spk_en) {
		rt5501_query.gpio_off_cancel = 0;
		queue_delayed_work(gpio_wq, &rt5501_query.gpio_off_work, msecs_to_jiffies(0));
	}

	mutex_unlock(&hs->mlock);
	mutex_unlock(&hs->gpiolock);

	if(rt5501_status == RT5501_SUSPEND)
		set_rt5501_amp(1);

	wake_unlock(&hs->hs_wake_lock);
}

static void volume_ramp_func(struct work_struct *work)
{
	mutex_lock(&rt5501_query.actionlock);
	if(rt5501_query.rt5501_status != RT5501_PLAYBACK) {
		u8 val;
		pr_info("%s: ramping-------------------------\n",__func__);
		mdelay(1);
		//start state machine and disable noise gate
		if(high_imp)
			rt5501_write_reg(0xb1,0x80);

		rt5501_write_reg(0x2,0x0);
		mdelay(1);
		val = 0x7;

		if (MFG_MODE) {
			pr_info("Skip volume ramp for MFG build");
			val += 15;
			rt5501_write_reg(1,val);
		} else {
#if 1
			int i;
			for(i=0; i<15; i++) {
				if(!rt5501_query.action_on) {
					mutex_unlock(&rt5501_query.actionlock);
					return;
				}
				msleep(1);
				rt5501_write_reg(1,val);
				val++;
			}
#else
			for(i=0; i<8; i++) {
				msleep(10);
				rt5501_write_reg(1,val);
				val += 2;
			}

#endif
		}
	}

	set_amp(1, &RT5501_AMP_ON);
	mutex_unlock(&rt5501_query.actionlock);
}

static void set_amp(int on, struct rt5501_config *i2c_command)
{
        pr_info("%s: %d\n", __func__, on);
        mutex_lock(&rt5501_query.mlock);
	mutex_lock(&hp_amp_lock);

        if(rt5501_query.hs_qstatus == RT5501_QUERY_HEADSET)
            rt5501_query.hs_qstatus = RT5501_QUERY_FINISH;

	if (on) {
            rt5501_query.rt5501_status = RT5501_PLAYBACK;
            if (rt5501_i2c_write(i2c_command->reg, i2c_command->reg_len) == 0) {
                last_spkamp_state = 1;
                pr_info("%s: ON \n",
                __func__);
            }


	} else {

             if(high_imp) {
                rt5501_write_reg(1,0x7);
                rt5501_write_reg(0xb1,0x81);
             } else {
                 rt5501_write_reg(1,0xc7);
             }

           if(rt5501_query.rt5501_status == RT5501_PLAYBACK) {
               last_spkamp_state = 0;
               pr_info("%s: OFF\n", __func__);
           }
           rt5501_query.rt5501_status = RT5501_OFF;
           rt5501_query.curmode = RT5501_MODE_OFF;
	}
	mutex_unlock(&hp_amp_lock);
        mutex_unlock(&rt5501_query.mlock);
}

int query_rt5501(void)
{
    return rt5501Connect;
}

void set_rt5501_amp(int on)
{
    pr_info("%s: %d\n", __func__, on);
    rt5501_query.gpio_off_cancel = 1;
    if(!on)
        rt5501_query.action_on = 0;
    cancel_delayed_work_sync(&rt5501_query.gpio_off_work);
    cancel_delayed_work_sync(&rt5501_query.volume_ramp_work);
    flush_work_sync(&rt5501_query.volume_ramp_work.work);
    mutex_lock(&rt5501_query.gpiolock);

    if(on) {

        if((rt5501_query.gpiostatus == AMP_GPIO_OFF) && pdata->gpio_rt5501_spk_en) {

            if(rt5501_query.s4status == AMP_S4_AUTO) {
                pm8921_aud_set_s4_pwm();
                rt5501_query.s4status = AMP_S4_PWM;
                msleep(1);
            }

#ifdef CONFIG_AMP_RT5501_DELAY
			msleep(50);
#endif
            pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
            gpio_direction_output(pdata->gpio_rt5501_spk_en, 1);
            rt5501_query.gpiostatus = AMP_GPIO_ON;
            msleep(1);
        }
                rt5501_query.action_on = 1;
		queue_delayed_work(ramp_wq, &rt5501_query.volume_ramp_work, msecs_to_jiffies(0));

    } else {
        set_amp(0, &RT5501_AMP_ON);
        if((rt5501_query.gpiostatus == AMP_GPIO_ON) && pdata->gpio_rt5501_spk_en) {

            rt5501_query.gpio_off_cancel = 0;
	    queue_delayed_work(gpio_wq, &rt5501_query.gpio_off_work, msecs_to_jiffies(0));
        }

    }

    mutex_unlock(&rt5501_query.gpiolock);
}

static int update_amp_parameter(int mode)
{
	if (mode >= rt5501_config_data.mode_num)
		return -EINVAL;

        pr_info("%s: set mode %d\n", __func__, mode);

	if (mode == RT5501_MODE_OFF)
		memcpy(&RT5501_AMP_OFF, &rt5501_config_data.cmd_data[mode].config,
				sizeof(struct rt5501_config));
        else if (mode == RT5501_INIT)
		memcpy(&RT5501_AMP_INIT, &rt5501_config_data.cmd_data[mode].config,
				sizeof(struct rt5501_config));
        else if (mode == RT5501_MUTE)
		memcpy(&RT5501_AMP_MUTE, &rt5501_config_data.cmd_data[mode].config,
				sizeof(struct rt5501_config));
	else {
		memcpy(&RT5501_AMP_ON, &rt5501_config_data.cmd_data[mode].config,
				sizeof(struct rt5501_config));
	}
	return 0;
}


static long rt5501_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0, modeid = 0;
	unsigned char tmp[7];
	unsigned char reg_idx[1] = {0x01};
	struct rt5501_comm_data spk_cfg;
	unsigned char reg_value[2];
        int premode = 0;
        int rt5501_status = 0;

	switch (cmd) {
	case RT5501_WRITE_REG:
		pr_info("%s: RT5501_WRITE_REG\n", __func__);
		mutex_lock(&hp_amp_lock);
		if (!last_spkamp_state) {
			/* According to rt5501 Spec */
			mdelay(30);
		}
		if (copy_from_user(reg_value, argp, sizeof(reg_value)))
			goto err1;
		pr_info("%s: reg_value[0]=%2x, reg_value[1]=%2x\n", __func__,  \
				reg_value[0], reg_value[1]);
		rc = rt5501_write_reg(reg_value[0], reg_value[1]);

err1:
		mutex_unlock(&hp_amp_lock);
		break;
	case RT5501_SET_CONFIG:
		if (copy_from_user(&spk_cfg, argp, sizeof(struct rt5501_comm_data)))
			return -EFAULT;

		memcpy(&RT5501_AMP_ON, &spk_cfg.config,
			sizeof(struct rt5501_config));
		break;
	case RT5501_READ_CONFIG:
		mutex_lock(&hp_amp_lock);
		if (!last_spkamp_state) {
			/* According to rt5501 Spec */
			mdelay(30);
		}

		rc = rt5501_i2c_write_for_read(reg_idx, sizeof(reg_idx));
		if (rc < 0)
			goto err2;

		rc = rt5501_i2c_read(tmp, sizeof(tmp));
		if (rc < 0)
			goto err2;

		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			rc = -EFAULT;
err2:
		mutex_unlock(&hp_amp_lock);
		break;
	case RT5501_SET_MODE:
		if (copy_from_user(&modeid, argp, sizeof(modeid)))
			return -EFAULT;

		if (modeid >= rt5501_config_data.mode_num || modeid <= 0) {
			pr_err("unsupported rt5501 mode %d\n", modeid);
			return -EINVAL;
		}
                mutex_lock(&hp_amp_lock);
                premode = rt5501_query.curmode;
                rt5501_query.curmode = modeid;
		rc = update_amp_parameter(modeid);
                rt5501_status = rt5501_query.rt5501_status;
                mutex_unlock(&hp_amp_lock);
		pr_info("%s:set rt5501 mode to %d curstatus %d\n", __func__,modeid,rt5501_status);
		if(rt5501_status == RT5501_SUSPEND || (rt5501_status == RT5501_PLAYBACK && premode != rt5501_query.curmode)) {
			flush_work_sync(&rt5501_query.volume_ramp_work.work);
			mutex_lock(&rt5501_query.actionlock);
			rt5501_query.action_on = 1;
			mutex_unlock(&rt5501_query.actionlock);
			queue_delayed_work(ramp_wq, &rt5501_query.volume_ramp_work, msecs_to_jiffies(280));
		}
		break;
	case RT5501_SET_PARAM:
		if (copy_from_user(&rt5501_config_data.mode_num, argp, sizeof(unsigned int))) {
			pr_err("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}

		if (rt5501_config_data.mode_num <= 0) {
			pr_err("%s: invalid mode number %d\n",
					__func__, rt5501_config_data.mode_num);
			return -EINVAL;
		}
		if (rt5501_config_data.cmd_data == NULL)
			rt5501_config_data.cmd_data = kzalloc(sizeof(struct rt5501_comm_data)*rt5501_config_data.mode_num, GFP_KERNEL);

		if (!rt5501_config_data.cmd_data) {
			pr_err("%s: out of memory\n", __func__);
			return -ENOMEM;
		}

		if (copy_from_user(rt5501_config_data.cmd_data, ((struct rt5501_config_data*)argp)->cmd_data \
                        ,sizeof(struct rt5501_comm_data)*rt5501_config_data.mode_num)) {
                    pr_err("%s: copy data from user failed.\n", __func__);
                    kfree(rt5501_config_data.cmd_data);
                    rt5501_config_data.cmd_data = NULL;
                    return -EFAULT;
		}

		pr_info("%s: update rt5501 i2c commands #%d success.\n",
				__func__, rt5501_config_data.mode_num);
		/* update default paramater from csv*/
                mutex_lock(&hp_amp_lock);
		update_amp_parameter(RT5501_MODE_OFF);
		update_amp_parameter(RT5501_MUTE);
		update_amp_parameter(RT5501_INIT);
		//update_amp_parameter(RT5501_MODE_MFG);
                mutex_unlock(&hp_amp_lock);
		rc = 0;
		break;
	case RT5501_QUERY_OM:
                mutex_lock(&rt5501_query.mlock);
		rc = rt5501_query.headsetom;
                mutex_unlock(&rt5501_query.mlock);
		pr_info("%s: query headset om %d\n", __func__,rc);

		if (copy_to_user(argp, &rc, sizeof(rc)))
                    rc = -EFAULT;
                else
                    rc = 0;
		break;

	default:
		pr_err("%s: Invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static struct file_operations rt5501_fops = {
	.owner = THIS_MODULE,
	.open = rt5501_open,
	.release = rt5501_release,
	.unlocked_ioctl = rt5501_ioctl,
};

static struct miscdevice rt5501_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rt5501",
	.fops = &rt5501_fops,
};

int rt5501_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
        int err = 0;
	MFG_MODE = board_mfg_mode();
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

	if(pdata->gpio_rt5501_spk_en) {
		char temp[2];

		err = gpio_request(pdata->gpio_rt5501_spk_en, "hp_en");

		ret = gpio_direction_output(pdata->gpio_rt5501_spk_en, 1);

		if(ret < 0) {
			pr_err("%s: gpio %d on error %d\n", __func__,pdata->gpio_rt5501_spk_en,ret);
		}

		mdelay(1);

		ret = rt5501_i2c_read(temp, 2);

		if(ret < 0) {
			pr_info("rt5501 is not connected\n");
			rt5501Connect = 0;
		} else {
			pr_info("rt5501 is connected\n");
			rt5501Connect = 1;
		}

		rt5501_write_reg(0x0,0x4);
		mdelay(1);

		rt5501_write_reg(0x0,0xc0);
		rt5501_write_reg(0x81,0x30);
		rt5501_write_reg(0x90,0xd0);
		rt5501_write_reg(0x93,0x9d);
		rt5501_write_reg(0x95,0x7b);
		rt5501_write_reg(0xa4,0x52);
		rt5501_write_reg(0x97,0x00);
		rt5501_write_reg(0x98,0x22);
		rt5501_write_reg(0x99,0x33);
		rt5501_write_reg(0x9a,0x55);
		rt5501_write_reg(0x9b,0x66);
		rt5501_write_reg(0x9c,0x99);
		rt5501_write_reg(0x9d,0x66);
		rt5501_write_reg(0x9e,0x99);
		rt5501_write_reg(0x1,0xc7);

		gpio_direction_output(pdata->gpio_rt5501_spk_en, 0);

		if(!err)
			gpio_free(pdata->gpio_rt5501_spk_en);

		if(ret < 0) {
			pr_err("%s: gpio %d off error %d\n", __func__,pdata->gpio_rt5501_spk_en,ret);
		}

	}

	if(rt5501Connect) {
		struct headset_notifier notifier;
		ret = misc_register(&rt5501_device);
		if (ret) {
			pr_err("%s: rt5501_device register failed\n", __func__);
			goto err_free_gpio_all;
		}

		hs_wq = create_workqueue("rt5501_hsdetect");
		INIT_DELAYED_WORK(&rt5501_query.hs_imp_detec_work,hs_imp_detec_func);
		wake_lock_init(&rt5501_query.hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);
		wake_lock_init(&rt5501_query.gpio_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);
		ramp_wq = create_workqueue("rt5501_volume_ramp");
		INIT_DELAYED_WORK(&rt5501_query.volume_ramp_work, volume_ramp_func);
		gpio_wq = create_workqueue("rt5501_gpio_off");
		INIT_DELAYED_WORK(&rt5501_query.gpio_off_work, hs_imp_gpio_off);
		notifier.id = HEADSET_REG_HS_INSERT;
		notifier.func = rt5501_headset_detect;
		headset_notifier_register(&notifier);
        }
	return 0;

err_free_gpio_all:
        rt5501Connect = 0;
	return ret;
err_alloc_data_failed:
        rt5501Connect = 0;
	return ret;
}

static int rt5501_remove(struct i2c_client *client)
{
	struct rt5501_platform_data *p6185data = i2c_get_clientdata(client);
	kfree(p6185data);

        if(rt5501Connect) {
            misc_deregister(&rt5501_device);
            cancel_delayed_work_sync(&rt5501_query.hs_imp_detec_work);
            destroy_workqueue(hs_wq);
        }
	return 0;
}

static void rt5501_shutdown(struct i2c_client *client)
{
	rt5501_query.gpio_off_cancel = 1;
	cancel_delayed_work_sync(&rt5501_query.gpio_off_work);
	cancel_delayed_work_sync(&rt5501_query.volume_ramp_work);

	mutex_lock(&rt5501_query.gpiolock);
	mutex_lock(&hp_amp_lock);
        mutex_lock(&rt5501_query.mlock);

	if((rt5501_query.gpiostatus == AMP_GPIO_OFF) && pdata->gpio_rt5501_spk_en) {

		if(rt5501_query.s4status == AMP_S4_AUTO) {
			pm8921_aud_set_s4_pwm();
			rt5501_query.s4status = AMP_S4_PWM;
			msleep(1);
		}

		pr_info("%s: enable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
		gpio_direction_output(pdata->gpio_rt5501_spk_en, 1);
		rt5501_query.gpiostatus = AMP_GPIO_ON;
		msleep(1);
	}

	pr_info("%s: reset rt5501\n",__func__);
	rt5501_write_reg(0x0,0x4);
	mdelay(1);

	high_imp = 0;

	if((rt5501_query.gpiostatus == AMP_GPIO_ON) && pdata->gpio_rt5501_spk_en) {
		pr_info("%s: disable gpio %d\n",__func__,pdata->gpio_rt5501_spk_en);
		gpio_direction_output(pdata->gpio_rt5501_spk_en, 0);
		rt5501_query.gpiostatus = AMP_GPIO_OFF;

		if(rt5501_query.s4status == AMP_S4_PWM) {
			pm8921_aud_set_s4_auto();
			rt5501_query.s4status = AMP_S4_AUTO;
		}
	}

        mutex_unlock(&rt5501_query.mlock);
	mutex_unlock(&hp_amp_lock);
	mutex_unlock(&rt5501_query.gpiolock);

}

static int rt5501_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int rt5501_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id rt5501_id[] = {
	{ RT5501_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver rt5501_driver = {
	.probe = rt5501_probe,
	.remove = rt5501_remove,
	.shutdown = rt5501_shutdown,
	.suspend = rt5501_suspend,
	.resume = rt5501_resume,
	.id_table = rt5501_id,
	.driver = {
		.name = RT5501_I2C_NAME,
	},
};

static int __init rt5501_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&hp_amp_lock);
        mutex_init(&rt5501_query.mlock);
        mutex_init(&rt5501_query.gpiolock);
		mutex_init(&rt5501_query.actionlock);
        rt5501_query.rt5501_status = RT5501_OFF;
        rt5501_query.hs_qstatus = RT5501_QUERY_OFF;
        rt5501_query.headsetom = HEADSET_8OM;
        rt5501_query.curmode = RT5501_MODE_OFF;
        rt5501_query.gpiostatus = AMP_GPIO_OFF;
        rt5501_query.s4status = AMP_S4_AUTO;
	return i2c_add_driver(&rt5501_driver);
}

static void __exit rt5501_exit(void)
{
	i2c_del_driver(&rt5501_driver);
}

module_init(rt5501_init);
module_exit(rt5501_exit);

MODULE_DESCRIPTION("rt5501 Speaker Amp driver");
MODULE_LICENSE("GPL");

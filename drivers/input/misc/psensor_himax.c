
/* drivers/i2c/chips/cm3629.c - cm3629 optical sensors driver
 *
 * Copyright (C) 2010 HTC, Inc.
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
#include <linux/delay.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <mach/board.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <mach/rpm-regulator.h>
#include <mach/devices_cmdline.h>
#include <linux/async.h>
#include <linux/psensor_himax.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define POLLING_PROXIMITY 1
#define MFG_MODE 1
#define NEAR_DELAY_TIME ((100 * HZ) / 1000)
#define Max_open_value 50
#define CALIBRATION_DATA_PATH "/calibration_data"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"
#define PSENSOR_FLASH_DATA "ps_flash"
#ifdef POLLING_PROXIMITY
#define POLLING_DELAY		200
#define TH_ADD			10
#endif
static void report_near_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_near_work, report_near_do_work);
static int is_probe_success;
static int p_status = 1;
static struct cm3629_info *lp_info;
int enable_cm3629_log = 0;
int f_cm3629_level = -1;
int current_lightsensor_adc;
int current_lightsensor_kadc;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex ps_enable_mutex;
static int ps_hal_enable, ps_drv_enable;
static int phone_status;
static int psensor_enable_by_touch = 0;

extern int proximity_enable_from_ps(int on);

module_param(p_status, int, 0444);

struct cm3629_info {
	struct class *cm3629_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *cm3629_fb_wq;
	struct delayed_work work_fb;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
	struct wake_lock ps_wake_lock;
	struct regulator	*sr_2v85;
	int model;
	int intr_pin;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;
	int led;
	uint32_t *adc_table;
	uint16_t cali_table[10];
	int irq;
	int ls_calibrate;
	int (*power)(int, uint8_t); 
	int (*lpm_power)(int on); 
	uint32_t als_kadc;
	uint32_t emmc_als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;
	int psensor_opened;
	int lightsensor_opened;
	uint16_t cm3629_slave_address;
	uint32_t emmc_ps_kadc1;
	uint32_t emmc_ps_kadc2;
	uint32_t ps_kparam1;
	uint32_t ps_kparam2;
	uint32_t ps_select;
	uint8_t ps1_thd_set;
	uint8_t ps1_thh_diff;
	uint8_t ps2_thd_set;
	uint8_t original_ps_thd_set;
	int current_level;
	uint16_t current_adc;
	
	uint8_t inte_ps1_canc;
	uint8_t inte_ps2_canc;
	uint8_t ps_conf1_val;
	uint8_t ps_conf2_val;
	uint8_t ps_conf1_val_from_board;
	uint8_t ps_conf2_val_from_board;
	uint8_t ps_conf3_val;
	uint8_t ps_calibration_rule; 
	int ps_pocket_mode;
	unsigned long j_start;
	unsigned long j_end;
	int mfg_mode;
	uint8_t *mapping_table;
	uint8_t mapping_size;
	uint8_t ps_base_index;
	uint8_t ps1_thd_no_cal;
	uint8_t ps1_thd_with_cal;
	uint8_t ps2_thd_no_cal;
	uint8_t ps2_thd_with_cal;
	uint32_t dynamical_threshold;
	uint8_t ls_cmd;
	uint8_t ps1_adc_offset;
	uint8_t ps2_adc_offset;
	uint8_t ps_debounce;
	uint16_t ps_delay_time;
	unsigned int no_need_change_setting;
	int ps_th_add;
	uint8_t dark_level;
	int ws_calibrate;
	int use__PS2v85;
	uint32_t ws_kadc;
	uint32_t ws_gadc;
	uint16_t w_golden_adc;
	uint32_t *correction_table;
};

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data);
#endif

static int psensor_enable(struct cm3629_info *lpi)
{
	lpi->ps_enable = 1;
	D("[PS][cm3629] %s %d\n", __func__, lpi->ps_enable);
	return 0;
}

static int psensor_disable(struct cm3629_info *lpi)
{
	lpi->ps_enable = 0;
	D("[PS][cm3629] %s %d\n", __func__, lpi->ps_enable);
	return 0;
}


static void report_near_do_work(struct work_struct *w)
{
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629]  %s: delay %dms, report proximity NEAR\n", __func__, lpi->ps_delay_time);

	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);
	input_sync(lpi->ps_input_dev);
	
}

int psensor_enable_by_touch_driver(int on)
{
	struct cm3629_info *lpi = lp_info;

	if (!is_probe_success) {
		D("[PS][cm3629] %s return by cm3629 probe fail\n", __func__);
		return 0;
	}
	psensor_enable_by_touch = 1;

	D("[PS][cm3629] %s on:%d\n", __func__, on);
	if (on) {
		psensor_enable(lpi);
	} else {
		psensor_disable(lpi);
	}

	psensor_enable_by_touch = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(psensor_enable_by_touch_driver);


void touch_report_psensor_input_event(int status)
{
	struct cm3629_info *lpi = lp_info;
	int ps_status;

	ps_status = status;
	D("[PS][cm3629] %s ps_status=%d\n",__func__,ps_status);
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, ps_status);
	input_sync(lpi->ps_input_dev);
}

EXPORT_SYMBOL_GPL(touch_report_psensor_input_event);


static ssize_t phone_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "phone_status = %d\n", phone_status);

	return ret;
}

static ssize_t phone_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int phone_status1 = 0;

	sscanf(buf, "%d" , &phone_status1);

	phone_status = phone_status1;

        D("[PS][cm3629] %s: phone_status = %d\n", __func__, phone_status);

	if ((phone_status == 1)||(phone_status == 2)) 
	{
		D("[PS][TP] %s proximity on\n",__func__);
		proximity_enable_from_ps(1);
	}
	if (phone_status == 0) 
	{
		D("[PS][TP] %s proximity off\n",__func__);
		proximity_enable_from_ps(0);
	}


	return count;
}
static DEVICE_ATTR(PhoneApp_status, 0666, phone_status_show, phone_status_store);

static int psensor_open(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;


	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;
	
	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629] %s\n", __func__);
	phone_status = 0;
	lpi->psensor_opened = 0;
	return 0;
	
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val, err=0;
	struct cm3629_info *lpi = lp_info;

	D("[PS][cm3629] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val) {
			
			if (!err)
				ps_hal_enable = 1;
			return err;
		} else {
			
			if (!err)
				ps_hal_enable = 0;
			return err;
		}
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[PS][cm3629 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

static struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};


static int psensor_setup(struct cm3629_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][cm3629 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][cm3629 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct cm3629_info *lpi=
		container_of(self, struct cm3629_info, fb_notif);

	pr_info("[PS][cm3629] %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && lpi && lpi->i2c_client) {
		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
				
				break;
			case FB_BLANK_POWERDOWN:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				if (lpi->ps_enable == 0)
					
					D("[PS][cm3629] %s: Psensor not enable\n", __func__);

				else
					D("[PS][cm3629] %s: Psensor enable, so did not enter lpm\n", __func__);
				break;
		}
	}

	return 0;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cm3629_early_suspend(struct early_suspend *h)
{
	struct cm3629_info *lpi = lp_info;

	D("[LS][cm3629] %s\n", __func__);

	if (lpi->ps_enable == 0)
		
	else
		D("[PS][cm3629] %s: Psensor enable, so did not enter lpm\n", __func__);
}

static void cm3629_late_resume(struct early_suspend *h)
{
	
	D("[LS][cm3629] %s\n", __func__);

}
#endif

static int cm3629_parse_dt(struct device *dev, struct psensor_platform_data *pdata)
{
	
	

	D("[PS][cm3629] %s: +\n", __func__);
	return 0;
}

static void cm3629_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct cm3629_info *lpi = container_of(work, struct cm3629_info, work_fb.work);
	pr_info("[PS][cm3629] %s in", __func__);
	lpi->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&lpi->fb_notif);
	if (ret)
		pr_err("[PS][cm3629][warning]:Unable to register fb_notifier: %d\n", ret);
}

static int  proximity_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cm3629_info *lpi;
	struct psensor_platform_data *pdata;

	D("[PS][cm3629] %s\n", __func__);


	lpi = kzalloc(sizeof(struct cm3629_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
		{
			D("platform_data alloc memory fail");
			goto err_alloc_mem_failed;
		}
		ret = cm3629_parse_dt(&pdev->dev, pdata);
		if (ret < 0) {
			ret = -ENOMEM;
			goto err_alloc_pdata_mem_failed;
		}
	} else {
		pdata = pdev->dev.platform_data;
	}


	lp_info = lpi;

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);
	mutex_init(&ps_enable_mutex);

	ps_hal_enable = ps_drv_enable = 0;


	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][cm3629 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm3629_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][cm3629 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");



	lpi->cm3629_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm3629_class)) {
		ret = PTR_ERR(lpi->cm3629_class);
		lpi->cm3629_class = NULL;
		goto err_create_class;
	}


	lpi->ps_dev = device_create(lpi->cm3629_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ls_device_file;
	}
	ret = device_create_file(lpi->ps_dev, &dev_attr_PhoneApp_status);
	if (ret)
		goto err_create_ps_device;

#ifdef CONFIG_FB
	lpi->cm3629_fb_wq = create_singlethread_workqueue("CM3629_FB");
	if (!lpi->cm3629_fb_wq) {
		pr_err("[PS][cm3629] allocate cm3629_fb_wq failed\n");
		ret = -ENOMEM;
		goto err_create_cm3629_fb_workqueue_failed;
	}
	INIT_DELAYED_WORK(&lpi->work_fb, cm3629_fb_register);
	queue_delayed_work(lpi->cm3629_fb_wq, &lpi->work_fb, msecs_to_jiffies(30000));
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm3629_early_suspend;
	lpi->early_suspend.resume = cm3629_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif
	D("[PS][cm3629] %s: Probe success!\n", __func__);
	is_probe_success = 1;
	return ret;
err_create_cm3629_fb_workqueue_failed:
err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_ls_device_file:
	device_unregister(lpi->ls_dev);
err_create_class:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ps_wake_lock));
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
	misc_deregister(&psensor_misc);
err_alloc_pdata_mem_failed:
	if (pdev->dev.of_node)
		kfree(pdata);
err_psensor_setup:
err_alloc_mem_failed:
	kfree(lpi);
	return ret;

}


static int  proximity_sensor_remove(struct platform_device *pdev)
{
	struct cm3629_info *lpi = platform_get_drvdata(pdev);


	if (lpi->ps_input_dev) {
		input_unregister_device(lpi->ps_input_dev);
		input_free_device(lpi->ps_input_dev);
	}
	wake_lock_destroy(&lpi->ps_wake_lock);
	kfree(lpi);
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id proximity_sensor_mttable[] = {
	{ .compatible = "proximity_sensor,himax"},
	{},
};
#else
#define proximity_sensor_mttable NULL
#endif

static struct platform_driver proximity_sensor_driver = {
	.probe  = proximity_sensor_probe,
	.remove = proximity_sensor_remove,
	.driver = {
		.name = "PROXIMITY_SENSOR",
		.owner = THIS_MODULE,
		.of_match_table = proximity_sensor_mttable,
	},
};

static void __init proximity_sensor_init_async(void *unused, async_cookie_t cookie)
{
	platform_driver_register(&proximity_sensor_driver);
}

static int __init proximity_sensor_init(void)
{
	async_schedule(proximity_sensor_init_async, NULL);
	return 0;
}

static void __exit proximity_sensor_exit(void)
{
	platform_driver_unregister(&proximity_sensor_driver);
}
module_init(proximity_sensor_init);
module_exit(proximity_sensor_exit);


MODULE_DESCRIPTION("proximity Driver");
MODULE_LICENSE("GPL");

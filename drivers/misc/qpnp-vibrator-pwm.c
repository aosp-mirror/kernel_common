/*
 * Vibrator driver implemented by qpnp-pwm.
 *
 * Copyright (C) 2013 HTC Corporation.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/socinfo.h>

#include <linux/qpnp/vibrator.h>
#include "../staging/android/timed_output.h"

#include <linux/vibtrig.h>

#define DTSI_PWM_CONFIG_DEFAULT   0
#define DTSI_PWMVALUE_1P_DEFAULT 80
#define DTSI_PWMVALUE_1P_MAX    100
#define DTSI_PWMVALUE_1P_MIN      0

#define DTSI_LPG_CHAN2       2
#define DTSI_LPG_CHAN3       3
#define DTSI_LPG_CHAN4       4
#define DTSI_LPG_CHAN5       5
#define DTSI_LPG_CHAN6       6
#define DTSI_LPG_CHAN8       8


#define LPG_CHAN2_BASE          0x1b200
#define LPG_CHAN3_BASE          0x1b300
#define LPG_CHAN4_BASE          0x1b400
#define LPG_CHAN5_BASE          0x1b500
#define LPG_CHAN6_BASE          0x1b600
#define LPG_CHAN8_BASE          0x1b800


#define GPIO3_BASE               0xc200
#define GPIO5_BASE               0xc400
#define GPIO6_BASE               0xc500
#define GPIO7_BASE               0xc600
#define GPIO8_BASE               0xc700
#define GPIO36_BASE              0xe300

#define LPG_PATTERN_CONFIG(base)      (base+0x40)
#define LPG_PWM_SIZE_CLK(base)        (base+0x41)
#define LPG_PWM_FREQ_PREDIV_CLK(base) (base+0x42)
#define LPG_PWM_TYPE_CONFIG(base)     (base+0x43)
#define LPG_PWM_VALUE_LSB(base)       (base+0x44)
#define LPG_PWM_VALUE_MSB(base)       (base+0x45)
#define LPG_ENABLE_CONTROL(base)      (base+0x46)

#define GPIO_DIG_VIN_CTL(base)  (base+0x40)
#define GPIO_DIG_PULL_CTL(base) (base+0x41)
#define GPIO_DIG_IN_CTL(base)   (base+0x42)
#define GPIO_DIG_OUT_CTL(base)  (base+0x45)
#define GPIO_EN_CTRL(base)      (base+0x46)

#define GPIO_EN_CTRL_PERH_EN    0x80
#define GPIO_EN_CTRL_DISABLE    0x00

#define QPNP_VIB_DEFAULT_TIMEOUT	15000

struct qpnp_vib_pwm {
	struct spmi_device      *spmi;
	struct hrtimer          vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct      work;
	const char*             default_trigger;

#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler enabler;
#endif

	uint32_t   power_en;
	uint32_t   ctrl_en;
	int        pwm_config;
	int        pwm_value_1p;
	int        lpg_select;
	int        lpg_base;
	int        gpio_base;
	u16        reg_on_addr;
	u8         reg_on_value;
	u16        reg_off_addr;
	u8         reg_off_value;
	int        onoff;
	int        timeout;
	spinlock_t lock;
};

static struct of_device_id spmi_match_table[];

static int qpnp_reg_write_u8(struct spmi_device *spmi, u8 data, u32 reg)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, (reg>>16)&0xF, reg&0xFFFF, &data, 1);
	if (rc < 0)
		printk(KERN_ERR "[VIB]"
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_pwm_config(struct qpnp_vib_pwm *vib)
{
	if ( vib->pwm_config == DTSI_PWM_CONFIG_DEFAULT ) {
		
		u32 pwm_value_scale;
		u8 pwm_value_lsb, pwm_value_msb;

		printk(KERN_INFO "[VIB]"
				"pwm_value=%d\n", vib->pwm_value_1p);

		pwm_value_scale = vib->pwm_value_1p * 512 / 100;
		pwm_value_lsb = (u8) (pwm_value_scale & 0xFF);
		pwm_value_msb = (u8) ((pwm_value_scale>>8) & 0xFF);

		qpnp_reg_write_u8(vib->spmi, 0x10, LPG_PATTERN_CONFIG(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, 0x33, LPG_PWM_SIZE_CLK(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, 0x20, LPG_PWM_FREQ_PREDIV_CLK(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, 0x20, LPG_PWM_TYPE_CONFIG(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, pwm_value_lsb, LPG_PWM_VALUE_LSB(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, pwm_value_msb, LPG_PWM_VALUE_MSB(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, 0xe4, LPG_ENABLE_CONTROL(vib->lpg_base));
		qpnp_reg_write_u8(vib->spmi, 0x16, GPIO_DIG_VIN_CTL(vib->gpio_base));
		qpnp_reg_write_u8(vib->spmi, 0x03, GPIO_DIG_PULL_CTL(vib->gpio_base));
		qpnp_reg_write_u8(vib->spmi, 0x04, GPIO_DIG_IN_CTL(vib->gpio_base));
		qpnp_reg_write_u8(vib->spmi, 0x02, GPIO_DIG_OUT_CTL(vib->gpio_base));
	} else {
		printk(KERN_ERR "[VIB]"
				"%s: unknown pwm config: %d.\n", __func__, vib->pwm_config);
		return -EINVAL;
	}

	return 0;
}

static int qpnp_vib_pwm_set(struct qpnp_vib_pwm *vib, int on)
{
	int rc;

	if (on) {
		printk(KERN_INFO "[VIB] +on.\n");
		rc = qpnp_vib_pwm_config(vib);
		if (rc < 0)
			return rc;
		if (vib->ctrl_en) {
			rc = gpio_direction_output(vib->ctrl_en, 1);
			if (rc < 0)
				return rc;
		}
		rc = qpnp_reg_write_u8(vib->spmi, vib->reg_on_value, vib->reg_on_addr);
		if (rc < 0)
			return rc;
		printk(KERN_INFO "[VIB] -on.\n");
	} else {
		printk(KERN_INFO "[VIB] +off.\n");
		rc = qpnp_reg_write_u8(vib->spmi, vib->reg_off_value, vib->reg_off_addr);
		if (rc < 0)
			return rc;
		if (vib->ctrl_en) {
			rc = gpio_direction_output(vib->ctrl_en, 0);
			if (rc < 0)
				return rc;
		}
		printk(KERN_INFO "[VIB] -off.\n");
	}

	return rc;
}

static void qpnp_vib_pwm_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib_pwm *vib = container_of(dev, struct qpnp_vib_pwm, timed_dev);
	unsigned long flags;

retry:
	spin_lock_irqsave(&vib->lock, flags);
	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		spin_unlock_irqrestore(&vib->lock, flags);
		cpu_relax();
		goto retry;
	}

	printk(KERN_INFO "[VIB] enable=%d.\n", value);

	if (value == 0)
		vib->onoff = 0;
	else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->onoff = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vib->lock, flags);
	schedule_work(&vib->work);
}

#ifdef CONFIG_VIB_TRIGGERS
static void __vib_pwm_trigger_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct qpnp_vib_pwm *vib;
	struct timed_output_dev *dev;

	vib = enabler->trigger_data;
	dev = &vib->timed_dev;

	printk(KERN_INFO "[VIB]"
			"vib_trigger=%d.\r\n", value);

	qpnp_vib_pwm_enable(dev, value);
}
#endif

static void qpnp_vib_pwm_update(struct work_struct *work)
{
	struct qpnp_vib_pwm *vib = container_of(work, struct qpnp_vib_pwm, work);
	qpnp_vib_pwm_set(vib, vib->onoff);
}

static int qpnp_vib_pwm_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib_pwm *vib = container_of(dev, struct qpnp_vib_pwm, timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_pwm_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib_pwm *vib = container_of(timer, struct qpnp_vib_pwm, vib_timer);

	vib->onoff = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

static ssize_t pwm_value_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib_pwm *vib = container_of(tdev, struct qpnp_vib_pwm, timed_dev);

	return sprintf(buf, "%d%%\n", vib->pwm_value_1p);
}

static ssize_t pwm_value_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib_pwm *vib = container_of(tdev, struct qpnp_vib_pwm, timed_dev);

	int pwm_value_1p;

	sscanf(buf, "%d", &pwm_value_1p);
	if (pwm_value_1p >= DTSI_PWMVALUE_1P_MIN && pwm_value_1p <= DTSI_PWMVALUE_1P_MAX) {
		vib->pwm_value_1p = pwm_value_1p;
	}
	return size;
}

static DEVICE_ATTR(pwm_value_1p, S_IRUGO | S_IWUSR, pwm_value_show, pwm_value_store);

#ifdef CONFIG_PM
static int qpnp_vibrator_pwm_suspend(struct device *dev)
{
	struct qpnp_vib_pwm *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	
	qpnp_vib_pwm_set(vib, 0);

	if (vib->power_en)
		gpio_direction_output(vib->power_en, 0);

	return 0;
}

static int qpnp_vibrator_pwm_resume(struct device *dev)
{
	struct qpnp_vib_pwm *vib = dev_get_drvdata(dev);

	if (vib->power_en)
		gpio_direction_output(vib->power_en, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pwm_pm_ops,
		qpnp_vibrator_pwm_suspend,
		qpnp_vibrator_pwm_resume);

static int __devinit qpnp_vibrator_pwm_probe(struct spmi_device *spmi)
{
	struct qpnp_vib_pwm *vib;
	const __be32 *temp_dt;
	const char *temp_string;
	struct property *temp_prop;
	int count;
	int rc;
	int pid_is_2nd_gpiotable;

	printk("[VIB] spmi full name: %s\n", spmi->dev.of_node->full_name);

	printk(KERN_INFO "[VIB]"
			"qpnp_vibrator_pwm_probe().\n");

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib) {
		printk(KERN_ERR "[VIB]"
				"%s: fail toallocate memory.\n", __func__);
		return -ENOMEM;
	}

	vib->spmi = spmi;

	temp_dt = of_get_property(spmi->dev.of_node,
			"qcom,qpnp-vib-timeout-ms", NULL);
	if (temp_dt)
		vib->timeout = be32_to_cpu(*temp_dt);
	else
		vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;

	vib->default_trigger = "none";
	rc = of_property_read_string(spmi->dev.of_node,
			"linux,default-trigger", &temp_string);
	if (rc) {
		printk(KERN_ERR "[VIB]"
				"fail to get default trigger name(%d).\n", rc);
		return rc;
	}
	vib->default_trigger = temp_string;

	pid_is_2nd_gpiotable = 0;

	if (of_find_property(spmi->dev.of_node, "htc,pid_of_2nd_gpiotable", &count)) {
		int i;
		static u32 array[16];

		count = count/sizeof(uint32_t);
		if ( count > sizeof(array)/sizeof(u32) ) {
			count = sizeof(array)/sizeof(u32);
			printk(KERN_ERR "[VIB]"
					"%s: 'htc,pid_of_2nd_gpiotable' exceeds maximum count(%d).\n",
					__func__, count);
		}

		of_property_read_u32_array(spmi->dev.of_node,
				"htc,pid_of_2nd_gpiotable", array, count);

		for (i = 0; i < count; i++) {
			int pid = of_machine_pid();
			int pid_of_2nd = array[i];

			if ( pid == pid_of_2nd ) {
				pid_is_2nd_gpiotable = 1;
				break;
			}
		}
	}

	if ( !pid_is_2nd_gpiotable ) {
		temp_prop = of_find_property(spmi->dev.of_node,
				"htc,power_en", NULL);
		if (temp_prop) {
			vib->power_en = of_get_named_gpio(spmi->dev.of_node,
					"htc,power_en", 0);
		}

		temp_prop = of_find_property(spmi->dev.of_node,
				"htc,ctrl_en", NULL);
		if (temp_prop) {
			vib->ctrl_en = of_get_named_gpio(spmi->dev.of_node,
					"htc,ctrl_en", 0);
		}
	} else {
		temp_prop = of_find_property(spmi->dev.of_node,
				"htc,2nd_power_en", NULL);
		if (temp_prop) {
			vib->power_en = of_get_named_gpio(spmi->dev.of_node,
					"htc,2nd_power_en", 0);
		}

		temp_prop = of_find_property(spmi->dev.of_node,
				"htc,2nd_ctrl_en", NULL);
		if (temp_prop) {
			vib->ctrl_en = of_get_named_gpio(spmi->dev.of_node,
					"htc,2nd_ctrl_en", 0);
		}
	}

	if (vib->power_en) {
		rc = gpio_request(vib->power_en, "vib_power_en");
		if (rc<0) {
			printk(KERN_ERR "[VIB]"
					"%s: unable to request gpio %d (%d)\n",
					__func__, vib->power_en, rc);
			return rc;
		}

		rc = gpio_direction_output(vib->power_en, 1);
		if (rc<0) {
			printk(KERN_ERR "[VIB]"
					"%s: Unable to set direction\n", __func__);
			return rc;
		}
	}

	if (vib->ctrl_en) {
		rc = gpio_request(vib->ctrl_en, "vib_ctrl_en");
		if (rc<0) {
			printk(KERN_ERR "[VIB]"
					"%s: unable to request gpio %d (%d)\n",
					__func__, vib->ctrl_en, rc);
			return rc;
		}

		rc = gpio_direction_output(vib->ctrl_en, 0);
		if (rc<0) {
			printk(KERN_ERR "[VIB]"
					"%s: Unable to set direction\n", __func__);
			return rc;
		}
	}

	temp_dt = of_get_property(spmi->dev.of_node,
			"lpg-select", NULL);
	if (temp_dt)
		vib->lpg_select = be32_to_cpu(*temp_dt);
	else
		vib->lpg_select = DTSI_LPG_CHAN4;

	temp_dt = of_get_property(spmi->dev.of_node,
			"pwm-config", NULL);
	if (temp_dt)
		vib->pwm_config = be32_to_cpu(*temp_dt);
	else
		vib->pwm_config = DTSI_PWM_CONFIG_DEFAULT;

	temp_dt = of_get_property(spmi->dev.of_node,
			"pwm_value_1p", NULL);
	if (temp_dt) {
		int pwm_value = be32_to_cpu(*temp_dt);
		if (pwm_value < DTSI_PWMVALUE_1P_MIN) pwm_value = DTSI_PWMVALUE_1P_MIN;
		if (pwm_value > DTSI_PWMVALUE_1P_MAX) pwm_value = DTSI_PWMVALUE_1P_MAX;
		vib->pwm_value_1p = pwm_value;
	}
	else
		vib->pwm_value_1p = DTSI_PWMVALUE_1P_DEFAULT;

	if ( vib->lpg_select == DTSI_LPG_CHAN2 ) {
		vib->lpg_base = LPG_CHAN2_BASE;
		vib->gpio_base = GPIO3_BASE;
	} else if ( vib->lpg_select == DTSI_LPG_CHAN3 ) {
		vib->lpg_base = LPG_CHAN3_BASE;
		vib->gpio_base = GPIO5_BASE;
	} else if ( vib->lpg_select == DTSI_LPG_CHAN4 ) {
		vib->lpg_base = LPG_CHAN4_BASE;
		vib->gpio_base = GPIO6_BASE;
	} else if ( vib->lpg_select == DTSI_LPG_CHAN5 ) {
		vib->lpg_base = LPG_CHAN5_BASE;
		vib->gpio_base = GPIO7_BASE;
	} else if ( vib->lpg_select == DTSI_LPG_CHAN6 ) {
		vib->lpg_base = LPG_CHAN6_BASE;
		vib->gpio_base = GPIO8_BASE;
	} else if ( vib->lpg_select == DTSI_LPG_CHAN8 ) {
                 vib->lpg_base = LPG_CHAN8_BASE;
                 vib->gpio_base = GPIO36_BASE;
	} else {
		printk(KERN_ERR "[VIB]"
				"%s: unknown lpg select: %d.\n", __func__, vib->lpg_select);
		return -EINVAL;
	}

	vib->reg_on_addr = GPIO_EN_CTRL(vib->gpio_base);
	vib->reg_on_value = GPIO_EN_CTRL_PERH_EN;
	vib->reg_off_addr = GPIO_EN_CTRL(vib->gpio_base);
	vib->reg_off_value = GPIO_EN_CTRL_DISABLE;

	rc = qpnp_vib_pwm_config(vib);
	if (rc < 0)
		return rc;

	spin_lock_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_pwm_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_pwm_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_pwm_get_time;
	vib->timed_dev.enable = qpnp_vib_pwm_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0) {
		printk(KERN_ERR "[VIB]"
				"fail to register timed_output.\n");
		return rc;
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_pwm_value_1p);
	if (rc < 0) {
		printk(KERN_ERR "[VIB]"
				"%s, create pwm_value sysfs fail: pwm_value\n", __func__);
	}

#ifdef CONFIG_VIB_TRIGGERS
	vib->enabler.name = "qpnp-vibrator-pwm";
	vib->enabler.default_trigger = vib->default_trigger;
	vib->enabler.enable = __vib_pwm_trigger_enable;
	vib->enabler.trigger_data = vib;
	vib_trigger_enabler_register(&vib->enabler);
#endif

	printk(KERN_INFO "[VIB]"
			"probe done.\n");
	return rc;
}

static int  __devexit qpnp_vibrator_pwm_remove(struct spmi_device *spmi)
{
	struct qpnp_vib_pwm *vib = dev_get_drvdata(&spmi->dev);

#ifdef CONFIG_VIB_TRIGGERS
	vib_trigger_enabler_unregister(&vib->enabler);
#endif
	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);

	if (vib->ctrl_en) {
		gpio_direction_output(vib->ctrl_en, 0);
		gpio_free(vib->ctrl_en);
	}
	if (vib->power_en) {
		gpio_direction_output(vib->power_en, 0);
		gpio_free(vib->power_en);
	}

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator-pwm",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_pwm_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator-pwm",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pwm_pm_ops,
	},
	.probe		= qpnp_vibrator_pwm_probe,
	.remove		= __devexit_p(qpnp_vibrator_pwm_remove),
};

static int __init qpnp_vibrator_pwm_init(void)
{
	printk(KERN_INFO "[VIB]"
			"qpnp_vibrator_pwm_init().\n");
	return spmi_driver_register(&qpnp_vibrator_pwm_driver);
}
module_init(qpnp_vibrator_pwm_init);

static void __exit qpnp_vibrator_pwm_exit(void)
{
	return spmi_driver_unregister(&qpnp_vibrator_pwm_driver);
}
module_exit(qpnp_vibrator_pwm_exit);

MODULE_DESCRIPTION("qpnp vibrator pwm driver");
MODULE_LICENSE("GPL v2");

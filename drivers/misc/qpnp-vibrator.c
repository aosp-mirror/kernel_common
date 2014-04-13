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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>

#include <linux/qpnp/vibrator.h>
#include "../staging/android/timed_output.h"

#include <linux/vibtrig.h>

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3100

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler enabler;
#endif

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	spinlock_t lock;
	u8 enlarge_vib_on;
	u8 enlarge_vib_diff_value;
};

struct qpnp_vib *vib_dev;

static struct of_device_id spmi_match_table[];

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

int qpnp_vibrator_config(struct qpnp_vib_config *vib_cfg)
{
	u8 reg = 0;
	int rc = -EINVAL, level;

	if (vib_dev == NULL) {
		pr_err("%s: vib_dev is NULL\n", __func__);
		return -ENODEV;
	}

	level = vib_cfg->drive_mV / 100;
	if (level) {
		if ((level < QPNP_VIB_MIN_LEVEL) ||
				(level > QPNP_VIB_MAX_LEVEL)) {
			dev_err(&vib_dev->spmi->dev, "Invalid voltage level\n");
			return -EINVAL;
		}
	} else {
		dev_err(&vib_dev->spmi->dev, "Voltage level not specified\n");
		return -EINVAL;
	}

	
	reg = vib_dev->reg_vtg_ctl;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_VTG_CTL(vib_dev->base));
	if (rc)
		return rc;
	vib_dev->reg_vtg_ctl = reg;

	
	reg = vib_dev->reg_en_ctl;
	reg |= (!!vib_cfg->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib_cfg->enable_mode == QPNP_VIB_MANUAL)
		reg |= QPNP_VIB_EN;
	else
		reg |= BIT(vib_cfg->enable_mode - 1);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_EN_CTL(vib_dev->base));
	if (rc < 0)
		return rc;
	vib_dev->reg_en_ctl = reg;

	return rc;
}
EXPORT_SYMBOL(qpnp_vibrator_config);

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc;
	u8 val;

	if (on) {
		val = vib->reg_vtg_ctl;
		val &= ~QPNP_VIB_VTG_SET_MASK;
		val |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
		printk(KERN_INFO "[VIB] on, reg0=0x%x.\n", val);
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
		if (rc < 0)
			return rc;
		vib->reg_vtg_ctl = val;
		val = vib->reg_en_ctl;
		val |= QPNP_VIB_EN;
		printk(KERN_INFO "[VIB] on, reg1=0x%x.\n", val);
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		if (rc < 0)
			return rc;
		vib->reg_en_ctl = val;
	} else {
		val = vib->reg_en_ctl;
		val &= ~QPNP_VIB_EN;
		printk(KERN_INFO "[VIB] off, reg1=0x%x.\n", val);
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		if (rc < 0)
			return rc;
		vib->reg_en_ctl = val;
	}

	return rc;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
					 timed_dev);
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
		vib->state = 0;
	else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	qpnp_vib_set(vib, vib->state);

	spin_unlock_irqrestore(&vib->lock, flags);
}

#ifdef CONFIG_VIB_TRIGGERS
static void qpnp_vib_trigger_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct qpnp_vib *vib;
	struct timed_output_dev *dev;

	vib = enabler->trigger_data;
	dev = &vib->timed_dev;

	printk(KERN_INFO "[VIB]"
			"vib_trigger=%d.\r\n", value);

	qpnp_vib_enable(dev, value);
}
#endif

static void qpnp_vib_update(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib,
					 work);
	qpnp_vib_set(vib, vib->state);
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib,
							 vib_timer);
	unsigned long flags;

	spin_lock_irqsave(&vib->lock, flags);

	vib->state = 0;
	qpnp_vib_set(vib, vib->state);

	spin_unlock_irqrestore(&vib->lock, flags);

	return HRTIMER_NORESTART;
}

static ssize_t voltage_level_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct timed_output_dev *time_cdev;
	struct qpnp_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct qpnp_vib, timed_dev);
	return sprintf(buf, "[VIB] voltage input:%dmV\n", vib->vtg_level*100);
}

static ssize_t voltage_level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int voltage_input;
	struct timed_output_dev *time_cdev;
	struct qpnp_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct qpnp_vib, timed_dev);

	voltage_input = -1;
	sscanf(buf, "%d ",&voltage_input);
	printk(KERN_INFO "[VIB] voltage input: %d\n", voltage_input);
	if (voltage_input/100 < QPNP_VIB_MIN_LEVEL || voltage_input/100 > QPNP_VIB_MAX_LEVEL){
		printk(KERN_INFO "[VIB] invalid voltage level input: %d\n",voltage_input);
		return -EINVAL;
	}
	vib->vtg_level = voltage_input/100;
	return size;
}

static DEVICE_ATTR(voltage_level, S_IRUGO | S_IWUSR, voltage_level_show, voltage_level_store);

static ssize_t vib_enlarge_en_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct timed_output_dev *time_cdev;
	struct qpnp_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct qpnp_vib, timed_dev);
	return sprintf(buf, "%d", vib->enlarge_vib_on);
}
static DEVICE_ATTR(vib_enlarge_en, S_IRUGO | S_IWUSR, vib_enlarge_en_show, NULL);

static ssize_t vib_enlarge_diff_value_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct timed_output_dev *time_cdev;
	struct qpnp_vib *vib ;
	time_cdev = (struct timed_output_dev *) dev_get_drvdata(dev);
	vib = container_of(time_cdev, struct qpnp_vib, timed_dev);
	return sprintf(buf, "%d", vib->enlarge_vib_diff_value);
}
static DEVICE_ATTR(vib_enlarge_diff_value, S_IRUGO | S_IWUSR, vib_enlarge_diff_value_show, NULL);

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	
	qpnp_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int __devinit qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	const __be32 *temp_dt;
	struct resource *vib_resource;
	int rc;
	u8 val;

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->spmi = spmi;

	temp_dt = of_get_property(spmi->dev.of_node,
			"qcom,qpnp-vib-timeout-ms", NULL);
	if (temp_dt)
		vib->timeout = be32_to_cpu(*temp_dt);
	else
		vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;

	temp_dt = of_get_property(spmi->dev.of_node,
			"qcom,qpnp-vib-vtg-level-mV", NULL);
	if (temp_dt)
		vib->vtg_level = be32_to_cpu(*temp_dt);
	else
		vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;

	vib->vtg_level /= 100;

	temp_dt = of_get_property(spmi->dev.of_node,
		"qcom,qpnp-vib-enlarge-enable", NULL);
	if(temp_dt) {
		vib->enlarge_vib_on = be32_to_cpu(*temp_dt);
	} else {
		vib->enlarge_vib_on = 0;
	}
	temp_dt = of_get_property(spmi->dev.of_node,
		"qcom,qpnp-vib-enlarge-diff_value", NULL);
	if(temp_dt) {
		vib->enlarge_vib_diff_value = be32_to_cpu(*temp_dt);
	} else {
		vib->enlarge_vib_diff_value = 0;
	}
	
	

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
		dev_err(&spmi->dev, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	
	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_vtg_ctl = val;

	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_en_ctl = val;

	spin_lock_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		return rc;

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_voltage_level);
	if (rc < 0) {
		printk(KERN_INFO "[VIB] %s, create sysfs fail: voltage_level\n", __func__);
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vib_enlarge_en);
	if (rc < 0) {
		printk(KERN_INFO "[VIB] %s, create sysfs fail: vib_enlarge_en\n", __func__);
	}

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vib_enlarge_diff_value);
	if (rc < 0) {
		printk(KERN_INFO "[VIB] %s, create sysfs fail: vib_enlarge_diff_value\n", __func__);
	}

#ifdef CONFIG_VIB_TRIGGERS
	vib->enabler.name = "qpnp-vibrator";
	vib->enabler.default_trigger = "vibrator";
	vib->enabler.enable = qpnp_vib_trigger_enable;
	vib->enabler.trigger_data = vib;
	vib_trigger_enabler_register(&vib->enabler);
#endif

	vib_dev = vib;

	return rc;
}

static int  __devexit qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

#ifdef CONFIG_VIB_TRIGGERS
	vib_trigger_enabler_unregister(&vib->enabler);
#endif
	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= __devexit_p(qpnp_vibrator_remove),
};

static int __init qpnp_vibrator_init(void)
{
	return spmi_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");

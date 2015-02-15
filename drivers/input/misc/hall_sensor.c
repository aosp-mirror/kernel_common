
/* drivers/input/misc/hall_sensor.c - Ak8789 Hall sensor driver
 *
 * Copyright (C) 2013 HTC Corporation.
 *
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
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/async.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/ak8789.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/devices_cmdline.h>
#include <linux/hall_sensor.h>

#define DRIVER_NAME "HL"

struct ak_hall_data {
	struct input_dev *input_dev;
	uint32_t gpio_att:16;
	uint32_t gpio_att_s:16;
	uint8_t  debug_level;
	uint8_t  hall_enable;
	uint8_t  att_used;
	struct wake_lock wake_lock;
};

static struct ak_hall_data *g_hl;
static int prev_val_n = 0;
static int prev_val_s = 0;
static int first_boot_s = 1;
static int first_boot_n = 1;

static void report_cover_event(int pole, int irq, struct ak_hall_data *hl);
static ssize_t debug_level_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ak_hall_data *hl = g_hl;
	if (buf[0] == '0' || buf[0] == '1') {
		hl->debug_level = buf[0] - '0';
		pr_info("[HL] debug_level = %d\b", hl->debug_level);
	} else
		pr_info("[HL] Parameter Error\n");
	return count;
}

static ssize_t debug_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ak_hall_data *hl = g_hl;
	return snprintf(buf, PAGE_SIZE, "[HL] debug_level = %d\n", hl->debug_level);
}
DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO), debug_level_show, debug_level_set);

static ssize_t read_att(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0, pos = 0;
	char string[100] = {0};
	struct ak_hall_data *hl = g_hl;

	ret = gpio_get_value(hl->gpio_att);
	if(first_boot_n && ret == 0)
	{
		prev_val_n = 1;
		report_cover_event(0, gpio_to_irq(hl->gpio_att), hl);
	}
	first_boot_n = 0;
	HL_LOG("ATT(%d):GPIO_ATT=%d", hl->att_used, ret);
	pos += snprintf(string+pos, sizeof(string), "ATT(%d):ATT_N=%d", hl->att_used, ret);

	if (hl->att_used > 1) {
		ret = gpio_get_value(hl->gpio_att_s);
		if(first_boot_s && ret == 0)
		{
			prev_val_s = 1;
			report_cover_event(1, gpio_to_irq(hl->gpio_att_s), hl);
		}
		first_boot_s = 0;
		HL_LOG("GPIO_ATT_S=%d", ret);
		pos += snprintf(string+pos, sizeof(string)-pos,  ", ATT_S=%d", ret);
	}

	pos = snprintf(buf, pos + 2, "%s", string);
	return pos;
}

static ssize_t write_att(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ak_hall_data *hl = g_hl;
	if (buf[0] == '0' || buf[0] == '1') {
		if(hl->hall_enable == 1 && buf[0] == '0')
		{
			HL_LOG("Disable hall sensor interrupts\n");
			disable_irq_nosync(gpio_to_irq(hl->gpio_att));
			disable_irq_nosync(gpio_to_irq(hl->gpio_att_s));
			hl->hall_enable = 0;
			irq_set_irq_wake(gpio_to_irq(hl->gpio_att_s), 0);
			irq_set_irq_wake(gpio_to_irq(hl->gpio_att), 0);
		}
		else if(hl->hall_enable == 0 && buf[0] == '1')
		{
			HL_LOG("Enable hall sensor interrupts\n");
			report_cover_event(0, gpio_to_irq(hl->gpio_att), hl);
			report_cover_event(1, gpio_to_irq(hl->gpio_att_s), hl);
			enable_irq(gpio_to_irq(hl->gpio_att));
			enable_irq(gpio_to_irq(hl->gpio_att_s));
			hl->hall_enable = 1;
			irq_set_irq_wake(gpio_to_irq(hl->gpio_att_s), 1);
			irq_set_irq_wake(gpio_to_irq(hl->gpio_att), 1);
		}
		else
			HL_LOG("Invalid paramater(0:Disable 1:Enable) hall enable = %d\n", hl->hall_enable);
	} else
		pr_info("[HL] Parameter Error\n");

	return count;
}

static DEVICE_ATTR(read_att, S_IRUGO | S_IWUSR , read_att, write_att);

static struct kobject *android_cover_kobj;

static int hall_cover_sysfs_init(void)
{
	int ret = 0;
	android_cover_kobj = kobject_create_and_add("android_cover", NULL);
	if (android_cover_kobj == NULL) {
		HL_ERR("%s:subsystem_register_failed", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_cover_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		HL_ERR("%s: sysfs_create_file debug_level failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_cover_kobj, &dev_attr_read_att.attr);
	if (ret) {
		HL_ERR("%s: sysfs_create_file read_att failed\n", __func__);
		return ret;
	}
	HL_LOG("attribute file register Done");
	return 0;
}

#ifdef CONFIG_OF
static int hall_sensor_dt_parser(struct device_node *dt, struct hall_platform_data *pdata)
{
	int ret = 0;
	const char *parser_st[] = {"hall,att_used", "hall,att_gpio", "hall,att_gpio_s"};
	uint32_t gpio_att = 0;

	ret = of_property_read_u32(dt, parser_st[0], (u32 *)&pdata->att_used);
	if (ret < 0) {
		HL_LOG("DT:%s parser err, ret=%d", parser_st[0], ret);
		goto parser_failed;
	} else
		HL_LOG("DT:%s=%d", parser_st[0], pdata->att_used);

	gpio_att = of_get_named_gpio(dt, parser_st[1], 0);
	if (!gpio_is_valid(gpio_att)) {
		HL_ERR("DT: %s parser failue, ret=%d", parser_st[1], gpio_att);
		ret = gpio_att;
		goto parser_failed;
	} else {
		pdata->gpio_att = gpio_att;
		HL_LOG("DT:%s[%d] read", parser_st[1], pdata->gpio_att);
	}

	if (pdata->att_used > 1) {
		gpio_att = of_get_named_gpio(dt, parser_st[2], 0);
		if (!gpio_is_valid(gpio_att)) {
			HL_ERR("DT: %s parser failue, ret=%d", parser_st[2], gpio_att);
			ret = gpio_att;
			goto parser_failed;
		} else {
			pdata->gpio_att_s = gpio_att;
			HL_LOG("DT:%s[%d] read", parser_st[2], pdata->gpio_att_s);
		}
	}

	return 0;
parser_failed:
	return ret;
}
#endif


static int hall_input_register(struct ak_hall_data *hl)
{
	int ret = 0;
	hl->input_dev = input_allocate_device();
	if (!hl->input_dev) {
		ret = -ENOMEM;
		HL_ERR("%s: Failed to allocate input_device", __func__);
		return ret;
	}
	input_set_drvdata(hl->input_dev, hl);
	hl->input_dev->name = HL_INPUTDEV_NAME;

	set_bit(EV_SYN, hl->input_dev->evbit);
	set_bit(EV_KEY, hl->input_dev->evbit);

	input_set_capability(hl->input_dev, EV_KEY, HALL_N_POLE);
	input_set_capability(hl->input_dev, EV_KEY, HALL_S_POLE);

	HL_LOG("%s\n", __func__);
	return input_register_device(hl->input_dev);
}

static void report_cover_event(int pole, int irq, struct ak_hall_data *hl)
{
	uint8_t val_n = 0, val_s = 0;

	if(pole == 0) 
	{
		val_n = gpio_get_value(hl->gpio_att);
		irq_set_irq_type(irq, val_n?IRQF_TRIGGER_LOW|IRQF_ONESHOT : IRQF_TRIGGER_HIGH|IRQF_ONESHOT);
		wake_lock_timeout(&hl->wake_lock, (2 * HZ));

		if (prev_val_n != val_n) {
			input_report_key(hl->input_dev, HALL_N_POLE, !val_n);
			input_sync(hl->input_dev);
			prev_val_n = val_n;
			printk("[HL] att_n[%s]", val_n ? "Far" : "Near");
			blocking_notifier_call_chain(&hallsensor_notifier_list, (0 << 1) |(!val_n), NULL);
		}

	}else if(pole == 1) 
	{
		val_s = gpio_get_value(hl->gpio_att_s);
		irq_set_irq_type(irq, val_s?IRQF_TRIGGER_LOW|IRQF_ONESHOT : IRQF_TRIGGER_HIGH|IRQF_ONESHOT);
		wake_lock_timeout(&hl->wake_lock, (2 * HZ));

		if (prev_val_s != val_s) {
			input_report_key(hl->input_dev, HALL_S_POLE, !val_s);
			input_sync(hl->input_dev);
			prev_val_s = val_s;
			printk("[HL] att_s[%s]", val_s ? "Far" : "Near");
			blocking_notifier_call_chain(&hallsensor_notifier_list, (1 << 1) |(!val_s), NULL);
		}

	}


}

static irqreturn_t hall_npole_irq_thread(int irq, void *ptr)
{
	struct ak_hall_data *hl = ptr;
	HL_LOG_TIME("N-pole interrupt trigger");
	report_cover_event(0, irq, hl);

	return IRQ_HANDLED;
}

static irqreturn_t hall_spole_irq_thread(int irq, void *ptr)
{
	struct ak_hall_data *hl = ptr;
	HL_LOG_TIME("S-pole interrupt trigger");
	report_cover_event(1, irq, hl);

	return IRQ_HANDLED;
}

static int __devinit hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ak_hall_data * hl;
	struct hall_platform_data *pdata;

	hl = kzalloc(sizeof(*hl), GFP_KERNEL);
	if (hl == NULL) {
		ret = -ENOMEM;
		goto err_alloc_mem_failed;
	}

	HL_LOG("++++++++++++++++++");
	hl->hall_enable = 1;
	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
		{
			HL_ERR("platform_data alloc memory fail");
			goto err_alloc_mem_failed;
		}
		ret = hall_sensor_dt_parser(pdev->dev.of_node, pdata);
		if (ret < 0) {
			ret = -ENOMEM;
			goto err_alloc_pdata_mem_failed;
		}
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (pdata) {
		hl->att_used   = pdata->att_used;
		hl->gpio_att   = pdata->gpio_att;
		hl->gpio_att_s = pdata->gpio_att_s;
	}

	ret = hall_input_register(hl);
	if (ret)
		goto err_input_register_device_failed;

	platform_set_drvdata(pdev, hl);
	wake_lock_init(&hl->wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	prev_val_n = gpio_get_value(hl->gpio_att);
	ret = request_threaded_irq(gpio_to_irq(hl->gpio_att), NULL, hall_npole_irq_thread,
				   prev_val_n? IRQF_TRIGGER_LOW|IRQF_ONESHOT : IRQF_TRIGGER_HIGH|IRQF_ONESHOT, "ak8789_att", hl);
	if (ret == 0)
	{
		irq_set_irq_wake(gpio_to_irq(hl->gpio_att), 1);
		HL_LOG("Operate in [Interrupt] mode, irq[%d]", gpio_to_irq(hl->gpio_att));
	}
	else
		goto err_request_irq_failed;

	if (hl->att_used > 1) {
		prev_val_s = gpio_get_value(hl->gpio_att_s);
		ret = request_threaded_irq(gpio_to_irq(hl->gpio_att_s), NULL, hall_spole_irq_thread,
				prev_val_s? IRQF_TRIGGER_LOW|IRQF_ONESHOT : IRQF_TRIGGER_HIGH|IRQF_ONESHOT, "ak8789_att_s", hl);
		if (ret == 0)
		{
			irq_set_irq_wake(gpio_to_irq(hl->gpio_att_s), 1);
			HL_LOG("Operate in [Interrupt] mode, irq[%d]", gpio_to_irq(hl->gpio_att_s));
		}
		else
		{
			free_irq(gpio_to_irq(hl->gpio_att), hl);
			goto err_request_irq_failed;
		}
	}

	hall_cover_sysfs_init();
	g_hl = hl;

	HL_LOG("------------------");
	kfree(pdata);
	return 0;

err_request_irq_failed:
	HL_ERR("Request IRQ failed, ret=%d, gpio=%d", ret, hl->gpio_att);
	wake_lock_destroy(&hl->wake_lock);

err_input_register_device_failed:
err_alloc_pdata_mem_failed:
	if (pdev->dev.of_node)
		kfree(pdata);
err_alloc_mem_failed:
	kfree(hl);

	return ret;
}

static int __devexit hall_sensor_remove(struct platform_device *pdev)
{
	struct ak_hall_data *hl = platform_get_drvdata(pdev);

	if (hl->input_dev) {
		input_unregister_device(hl->input_dev);
		input_free_device(hl->input_dev);
	}
	wake_lock_destroy(&hl->wake_lock);
	kfree(g_hl);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hall_sensor_mttable[] = {
	{ .compatible = "hall_sensor,ak8789"},
	{},
};
#else
#define hall_sensor_mttable NULL
#endif

static struct platform_driver hall_sensor_driver = {
	.probe  = hall_sensor_probe,
	.remove = hall_sensor_remove,
	.driver = {
		.name = "AK8789_HALL_SENSOR",
		.owner = THIS_MODULE,
		.of_match_table = hall_sensor_mttable,
	},
};

static void __init hall_sensor_init_async(void *unused, async_cookie_t cookie)
{
	platform_driver_register(&hall_sensor_driver);
}

static int __init hall_sensor_init(void)
{
	async_schedule(hall_sensor_init_async, NULL);
	return 0;
}

static void __exit hall_sensor_exit(void)
{
	platform_driver_unregister(&hall_sensor_driver);
}
module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_DESCRIPTION("HTC Hall Sesnor driver");
MODULE_LICENSE("GPL");

/* drivers/input/misc/gpio_event.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <mach/devices_dtb.h>
#include <linux/delay.h>

struct gpio_event {
	struct gpio_event_input_devs *input_devs;
	const struct gpio_event_platform_data *info;
	struct early_suspend early_suspend;
	void *state[0];
	uint8_t rrm1_mode;
};

static int gpio_input_event(
	struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	int i;
	int devnr;
	int ret = 0;
	int tmp_ret;
	struct gpio_event_info **ii;
	struct gpio_event *ip = input_get_drvdata(dev);

	for (devnr = 0; devnr < ip->input_devs->count; devnr++)
		if (ip->input_devs->dev[devnr] == dev)
			break;
	if (devnr == ip->input_devs->count) {
		KEY_LOGE("KEY_ERR: %s: unknown device %p\n", __func__, dev);
		return -EIO;
	}

	for (i = 0, ii = ip->info->info; i < ip->info->info_count; i++, ii++) {
		if ((*ii)->event) {
			tmp_ret = (*ii)->event(ip->input_devs, *ii,
						&ip->state[i],
						devnr, type, code, value);
			if (tmp_ret)
				ret = tmp_ret;
		}
	}
	return ret;
}

static int gpio_event_call_all_func(struct gpio_event *ip, int func)
{
	int i;
	int ret;
	struct gpio_event_info **ii;

	if (func == GPIO_EVENT_FUNC_INIT || func == GPIO_EVENT_FUNC_RESUME) {
		ii = ip->info->info;
		for (i = 0; i < ip->info->info_count; i++, ii++) {
			if ((*ii)->func == NULL) {
				ret = -ENODEV;
				KEY_LOGE("KEY_ERR: gpio_event_probe: Incomplete pdata, "
					"no function\n");
				goto err_no_func;
			}
			if (func == GPIO_EVENT_FUNC_RESUME && (*ii)->no_suspend)
				continue;
			if (func == GPIO_EVENT_FUNC_INIT)
				(*ii)->rrm1_mode = ip->rrm1_mode;
			ret = (*ii)->func(ip->input_devs, *ii, &ip->state[i],
					  func);
			if (ret) {
				KEY_LOGE("KEY_ERR: gpio_event_probe: function failed\n");
				goto err_func_failed;
			}
		}
		return 0;
	}

	ret = 0;
	i = ip->info->info_count;
	ii = ip->info->info + i;
	while (i > 0) {
		i--;
		ii--;
		if ((func & ~1) == GPIO_EVENT_FUNC_SUSPEND && (*ii)->no_suspend)
			continue;
		(*ii)->func(ip->input_devs, *ii, &ip->state[i], func & ~1);
err_func_failed:
err_no_func:
		;
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void gpio_event_suspend(struct early_suspend *h)
{
	struct gpio_event *ip;
	ip = container_of(h, struct gpio_event, early_suspend);
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_SUSPEND);
	ip->info->power(ip->info, 0);
}

void gpio_event_resume(struct early_suspend *h)
{
	struct gpio_event *ip;
	ip = container_of(h, struct gpio_event, early_suspend);
	ip->info->power(ip->info, 1);
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_RESUME);
}
#endif

#ifdef CONFIG_OF
#ifdef CONFIG_POWER_KEY_CLR_RESET
static void clear_hw_reset(uint32_t clear_gpio)
{
	uint32_t hw_clr_gpio_table[] = {
		GPIO_CFG(clear_gpio, 0, GPIO_CFG_OUTPUT,
			 GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	};

	gpio_tlmm_config(hw_clr_gpio_table[0], GPIO_CFG_ENABLE);
	KEY_LOGI("%s ++++++\n", __func__);
	gpio_set_value(clear_gpio, 0);
	msleep(100);
	gpio_set_value(clear_gpio, 1);
	KEY_LOGI("%s ------\n", __func__);
}
#endif
static void setup_input_gpio(const struct gpio_event_direct_entry *cfg, size_t size)
{
	uint8_t i = 0;
	uint32_t gpio_table[size];
	int rc;
	KEY_LOGD("DT:%s\n", __func__);
	for (i = 0; i < size; i++) {
		if(cfg[i].pull == 1)
			gpio_table[i] = GPIO_CFG(cfg[i].gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		else
			gpio_table[i] = GPIO_CFG(cfg[i].gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
		rc = gpio_tlmm_config(gpio_table[i], GPIO_CFG_ENABLE);
		if (rc) {
			KEY_LOGE("Init gpio[%d] pull= %d, tlmm_config = %d\n",
				cfg[i].gpio, cfg[i].pull, rc);
		}
	}
}

static struct gpio_event_input_info keypad_input_info = {
	.info.func             = gpio_event_input_func,
	.flags                 = GPIOEDF_PRINT_KEYS,
	.type                  = EV_KEY,
#if BITS_PER_LONG != 64 && !defined(CONFIG_KTIME_SCALAR)
	.debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
# else
	.debounce_time.tv64    = 20 * NSEC_PER_MSEC,
# endif
	.dt_setup_input_gpio   = setup_input_gpio,
#ifdef CONFIG_POWER_KEY_CLR_RESET
	.dt_clear_hw_reset     = clear_hw_reset,
#endif
};

static struct gpio_event_info *keypad_info[] = {
	&keypad_input_info.info,
};

static int gpio_event_parser_dt(struct device *dev,
				struct gpio_event_platform_data *event_info)
{
	const char *drname;
	int ret = 0;
	uint8_t i = 0, cnt = 0;
	u32 data = 0;
	struct device_node *node, *pp = NULL;
	struct gpio_event_direct_entry *cfg;

	node = dev->of_node;
	if (node == NULL) {
		KEY_LOGE("%s, Can't find device_node", __func__);
		ret = -ENODEV;
	}

	if (of_property_read_string(node, "names", &drname) == 0)
		event_info->names[0] = drname;
	else
		KEY_LOGE("DT, platform_data driver name parser fail");
#ifdef CONFIG_POWER_KEY_CLR_RESET
	keypad_input_info.clr_gpio = of_get_named_gpio(node, "clr_gpio", 0);
	KEY_LOGI("DT:clr gpio[%d] load\n", keypad_input_info.clr_gpio);
#endif
	while ((pp = of_get_next_child(node, pp)))
		cnt++;
	if (!cnt)
		ret = -ENODEV;
	KEY_LOGI("DT %s, cnt=%d\n", __func__, cnt);

	cfg = kzalloc(cnt * sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	if (of_property_read_u32(node, "cmcc_disable_reset", &data) == 0)
	{
		event_info->cmcc_disable_reset = data;
		KEY_LOGI("DT:event_info->cmcc_disable_reset = %d\n", event_info->cmcc_disable_reset);
	}

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "keycode", &data) == 0)
			cfg[i].code = data;
		if (of_property_read_u32(pp, "pull", &data) == 0)
			cfg[i].pull = data;

		cfg[i].gpio = of_get_named_gpio(pp, "gpio", 0);
		KEY_LOGI("DT:gpio[%d]:%d parser done, pull= %d, kcode = %d\n",
				i, cfg[i].gpio, cfg[i].pull, cfg[i].code);
		i++;
	}
	keypad_input_info.keymap = cfg;
	keypad_input_info.keymap_size = i;

	event_info->info = keypad_info;
	event_info->info_count = ARRAY_SIZE(keypad_info);

	return 0;
}
#else
static int gpio_event_parser_dt(struct device *dev,
				struct gpio_event_platform_data *event_info)
{
	return 0;
}
#endif

static int gpio_event_probe(struct platform_device *pdev)
{
	int err;
	struct gpio_event *ip;
	struct gpio_event_platform_data *event_info;
	int dev_count = 1;
	int i;
	int registered = 0;

	if (pdev->dev.of_node) {
		event_info = kzalloc(sizeof(*event_info), GFP_KERNEL);
		if (event_info == NULL)
			KEY_LOGE("alloc memroy fail\n");
		gpio_event_parser_dt(&pdev->dev, event_info);
	} else {
		event_info = pdev->dev.platform_data;
	}
	if (event_info == NULL) {
		KEY_LOGE("KEY_ERR: %s: No pdata\n", __func__);
		return -ENODEV;
	}
	if ((!event_info->name && !event_info->names[0]) ||
	    !event_info->info || !event_info->info_count) {
		KEY_LOGE("KEY_ERR: %s: Incomplete pdata\n", __func__);
		return -ENODEV;
	}
	if (!event_info->name)
		while (event_info->names[dev_count])
			dev_count++;
	ip = kzalloc(sizeof(*ip) +
		     sizeof(ip->state[0]) * event_info->info_count +
		     sizeof(*ip->input_devs) +
		     sizeof(ip->input_devs->dev[0]) * dev_count, GFP_KERNEL);
	if (ip == NULL) {
		err = -ENOMEM;
		KEY_LOGE("KEY_ERR: %s: Failed to allocate private data\n", __func__);
		goto err_kp_alloc_failed;
	}
	ip->input_devs = (void*)&ip->state[event_info->info_count];
	platform_set_drvdata(pdev, ip);

	if ((get_debug_flag() & DEBUG_FLAG_DISABLE_PMIC_RESET) && event_info->cmcc_disable_reset) {
		ip->rrm1_mode = 1;
		KEY_LOGI("Lab Test RRM1 Mode");
	} else {
	      ip->rrm1_mode = 0;
	}

	for (i = 0; i < dev_count; i++) {
		struct input_dev *input_dev = input_allocate_device();
		if (input_dev == NULL) {
			err = -ENOMEM;
			KEY_LOGE("KEY_ERR: %s: "
				"Failed to allocate input device\n", __func__);
			goto err_input_dev_alloc_failed;
		}
		input_set_drvdata(input_dev, ip);
		input_dev->name = event_info->name ?
					event_info->name : event_info->names[i];
		input_dev->event = gpio_input_event;
		ip->input_devs->dev[i] = input_dev;
	}
	ip->input_devs->count = dev_count;
	ip->info = event_info;
	if (event_info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		ip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ip->early_suspend.suspend = gpio_event_suspend;
		ip->early_suspend.resume = gpio_event_resume;
		register_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 1);
	}

	err = gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_INIT);
	if (err)
		goto err_call_all_func_failed;

	for (i = 0; i < dev_count; i++) {
		err = input_register_device(ip->input_devs->dev[i]);
		if (err) {
			KEY_LOGE("KEY_ERR: %s: Unable to register %s "
				"input device\n", __func__, ip->input_devs->dev[i]->name);
			goto err_input_register_device_failed;
		}
		registered++;
	}

	return 0;

err_input_register_device_failed:
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_UNINIT);
err_call_all_func_failed:
	if (event_info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 0);
	}
	for (i = 0; i < registered; i++)
		input_unregister_device(ip->input_devs->dev[i]);
	for (i = dev_count - 1; i >= registered; i--) {
		input_free_device(ip->input_devs->dev[i]);
err_input_dev_alloc_failed:
		;
	}
	kfree(ip);
err_kp_alloc_failed:
	return err;
}

static int gpio_event_remove(struct platform_device *pdev)
{
	struct gpio_event *ip = platform_get_drvdata(pdev);
	int i;

	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_UNINIT);
	if (ip->info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 0);
	}
	for (i = 0; i < ip->input_devs->count; i++)
		input_unregister_device(ip->input_devs->dev[i]);
	kfree(ip);
	return 0;
}

static const struct platform_device_id gpio_event_id[] = {
	{ "gpio-event", 0 },
	{ },
};

static const struct of_device_id htc_gpio_event_mttable[] = {
	{ .compatible = "key,gpio-event"},
	{ },
};

static struct platform_driver gpio_event_driver = {
	.probe		= gpio_event_probe,
	.remove		= gpio_event_remove,
	.driver		= {
		.name	= GPIO_EVENT_DEV_NAME,
		.of_match_table = htc_gpio_event_mttable,
	},
	.id_table = gpio_event_id,
};
module_platform_driver(gpio_event_driver);

MODULE_DESCRIPTION("GPIO Event Driver");
MODULE_LICENSE("GPL");


/*
 *
 * /arch/arm/mach-msm/htc_headset_gpio.c
 *
 * HTC GPIO headset driver.
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/async.h>
#endif

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/board_htc.h>

#define DRIVER_NAME "HS_GPIO"

static struct workqueue_struct *detect_wq;
static void detect_gpio_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_gpio_work, detect_gpio_work_func);

static void irq_init_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(irq_init_work, irq_init_work_func);

static struct workqueue_struct *button_wq;
static void button_gpio_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_gpio_work, button_gpio_work_func);

static struct htc_headset_gpio_info *hi;

static int headset_uart_enable = 0;

static void hs_gpio_uart_set(int value)
{
	value = !!value;

	if (value != headset_uart_enable) {
		headset_uart_enable = value;
		HS_LOG("%s headset uart", value? "enable": "disable");
		gpio_set_value(hi->pdata.uart_gpio, value);
	}
}

static int hs_gpio_hpin_state(void)
{
	HS_DBG();

	return gpio_get_value(hi->pdata.hpin_gpio);
}

void hs_gpio_key_enable(int enable)
{
	HS_DBG();

	if (hi->pdata.key_enable_gpio)
		gpio_set_value(hi->pdata.key_enable_gpio, enable);
}

void hs_gpio_mic_select(int enable)
{
	HS_DBG();

	if (hi->pdata.mic_select_gpio)
		gpio_set_value(hi->pdata.mic_select_gpio, enable);
}

static void detect_gpio_work_func(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = gpio_get_value(hi->pdata.hpin_gpio) ? 0 : 1;

	if (hi->headset_state == insert)
		return;

	hi->headset_state = insert;
	hs_notify_plug_event(insert, 0);
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int gpio1 = 0;
	int gpio2 = 0;
	int retry_limit = 10;
	unsigned int irq_type = IRQF_TRIGGER_NONE;

	hs_notify_hpin_irq();

	HS_DBG();

	do {
		gpio1 = gpio_get_value(hi->pdata.hpin_gpio);
		irq_type = gpio1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
		set_irq_type(hi->hpin_irq, irq_type);
		gpio2 = gpio_get_value(hi->pdata.hpin_gpio);
	} while (gpio1 != gpio2 && retry_limit-- > 0);

	HS_DBG("gpio2 = %d (%d retries)", gpio2, (10-retry_limit));

	if ((hi->headset_state == 0) ^ gpio2) {
		wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
		queue_delayed_work(detect_wq, &detect_gpio_work,
				   hi->hpin_debounce);
	}

	return IRQ_HANDLED;
}

static void button_gpio_work_func(struct work_struct *work)
{
	HS_DBG();
	hs_notify_key_irq();
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	HS_DBG();

	hi->key_irq_type ^= irq_mask;
	set_irq_type(hi->key_irq, hi->key_irq_type);

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(button_wq, &button_gpio_work, HS_JIFFIES_ZERO);

	return IRQ_HANDLED;
}

static void irq_init_work_func(struct work_struct *work)
{
	HS_DBG();

	if (hi->pdata.hpin_gpio) {
		HS_LOG("Enable detect IRQ");
		set_irq_type(hi->hpin_irq, IRQF_TRIGGER_LOW);
		enable_irq(hi->hpin_irq);
	}

	if (hi->pdata.key_gpio) {
		HS_LOG("Enable button IRQ");
		hi->key_irq_type = IRQF_TRIGGER_LOW;
		set_irq_type(hi->key_irq, hi->key_irq_type);
		enable_irq(hi->key_irq);
	}
}

static int hs_gpio_request_irq(unsigned int gpio, unsigned int *irq,
			       irq_handler_t handler, unsigned long flags,
			       const char *name, unsigned int wake)
{
	int ret = 0;

	HS_DBG();

	ret = gpio_request(gpio, name);
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	if (!(*irq)) {
		ret = gpio_to_irq(gpio);
		if (ret < 0) {
			gpio_free(gpio);
			return ret;
		}
		HS_LOG("gpio_to_irq ret = %d", ret);
		*irq = (unsigned int) ret;
	}

	HS_LOG("[HS] irq=%d", *irq);
	ret = request_irq(*irq, handler, flags, name, NULL);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	ret = set_irq_wake(*irq, wake);
	if (ret < 0) {
		free_irq(*irq, 0);
		gpio_free(gpio);
		return ret;
	}

	return 1;
}

static int hs_gpio_request_output(unsigned int gpio, const char *name, int value)
{
	int ret = 0;

	HS_DBG();

	ret = gpio_request(gpio, name);
	if (ret < 0)
		HS_LOG("GPIO Already Requested");

	ret = gpio_direction_output(gpio, value);
	if (ret < 0) {
		HS_ERR("gpio_direction_output(gpio);");
		gpio_free(gpio);
		return ret;
	}

	return 1;
}

static void hs_gpio_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.uart_gpio) {
		notifier.id = HEADSET_REG_UART_SET;
		notifier.func = hs_gpio_uart_set;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_gpio_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.mic_select_gpio) {
		notifier.id = HEADSET_REG_MIC_SELECT;
		notifier.func = hs_gpio_mic_select;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_gpio_key_enable;
		headset_notifier_register(&notifier);
	}
}

#ifdef CONFIG_OF
static int htc_headset_gpio_parse_dt(struct device *dev,
				     struct htc_headset_gpio_platform_data *pdata)
{
	struct device_node *dt = dev->of_node;
	HS_LOG("DT Parser");
	pdata->hpin_gpio = of_get_named_gpio(dt, "headset,irq-gpio", 0);
	if (pdata->hpin_gpio) {
		HS_LOG("hpin-gpio:%d", pdata->hpin_gpio);
	} else
		HS_LOG("hpin-gpio parser err");
	return 0;
}
#else
static int htc_headset_gpio_parse_dt(struct device *dev,
				     struct htc_headset_gpio_platform_data *pdata)
{
	return 0;
}
#endif

static int htc_headset_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct htc_headset_gpio_platform_data *pdata;/* = pdev->dev.platform_data;*/

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_gpio_info), GFP_KERNEL);
	if (!hi) {
		HS_ERR("Failed to allocate memory for headset info");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			ret = -ENOMEM;
		ret = htc_headset_gpio_parse_dt(&pdev->dev, pdata);
	} else {
		HS_LOG("old style\n");
		pdata = pdev->dev.platform_data;
	}

	if (pdata->config_headset_gpio)
		pdata->config_headset_gpio();

	hi->pdata.uart_gpio       = pdata->uart_gpio;
	hi->pdata.hpin_gpio       = pdata->hpin_gpio;
	hi->pdata.key_gpio        = pdata->key_gpio;
	hi->pdata.key_enable_gpio = pdata->key_enable_gpio;
	hi->pdata.mic_select_gpio = pdata->mic_select_gpio;

	hi->hpin_debounce         = HS_JIFFIES_ZERO;
	hi->key_irq_type          = IRQF_TRIGGER_NONE;
	hi->headset_state         = 0;

	detect_wq = create_workqueue("HS_GPIO_DETECT");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("HS_GPIO_BUTTON");
	if (button_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	if (hi->pdata.uart_gpio) {
/*                if (!(get_kernel_flag() & KERNEL_FLAG_SERIAL_HSL_ENABLE))*/
			ret = hs_gpio_request_output(hi->pdata.uart_gpio, "HS_GPIO_UART", 0);

		if (ret < 0) {
			HS_ERR("Fail to request GPIO UART(0x%x)", ret);
		}
	}

	if (hi->pdata.hpin_gpio) {
		ret = hs_gpio_request_irq(hi->pdata.hpin_gpio,
				&hi->hpin_irq, detect_irq_handler,
				IRQF_TRIGGER_NONE, "HS_GPIO_DETECT", 1);
		if (ret < 0) {
			HS_ERR("Failed to request GPIO HPIN IRQ (0x%X)", ret);
			goto err_request_detect_irq;
		}
		disable_irq(hi->hpin_irq);
	}

	if (hi->pdata.key_gpio) {
		ret = hs_gpio_request_irq(hi->pdata.key_gpio,
				&hi->key_irq, button_irq_handler,
				hi->key_irq_type, "HS_GPIO_BUTTON", 1);
		if (ret < 0) {
			HS_ERR("Failed to request GPIO button IRQ (0x%X)", ret);
			goto err_request_button_irq;
		}
		disable_irq(hi->key_irq);
	}

	queue_delayed_work(detect_wq, &irq_init_work, HS_JIFFIES_IRQ_INIT);

	hs_gpio_register();
#ifndef CONFIG_OF
	hs_notify_driver_ready(DRIVER_NAME);
#endif
	HS_LOG("--------------------");

	return 0;

err_request_button_irq:
	if (hi->pdata.hpin_gpio) {
		free_irq(hi->hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

err_request_detect_irq:
	wake_lock_destroy(&hi->hs_wake_lock);
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_gpio_remove(struct platform_device *pdev)
{
	if (hi->pdata.key_gpio) {
		free_irq(hi->key_irq, 0);
		gpio_free(hi->pdata.key_gpio);
	}

	if (hi->pdata.hpin_gpio) {
		free_irq(hi->hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

	if(hi->pdata.uart_gpio) {
		gpio_free(hi->pdata.uart_gpio);
	}

	wake_lock_destroy(&hi->hs_wake_lock);
	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);

	kfree(hi);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id htc_headset_gpio_mttable[] = {
	{ .compatible = "htc_headset,gpio"},
	{ },
};
#else
#define htc_headset_gpio_mttable NULL
#endif
static struct platform_driver htc_headset_gpio_driver = {
	.probe	= htc_headset_gpio_probe,
	.remove	= htc_headset_gpio_remove,
	.driver	= {
		.name		= "HTC_HEADSET_GPIO",
		.owner		= THIS_MODULE,
		.of_match_table = htc_headset_gpio_mttable,
	},
};

static void __init htc_headset_gpio_init_async(void *unused, async_cookie_t cookie)
{
	platform_driver_register(&htc_headset_1wire_driver);
}

static int __init htc_headset_gpio_init(void)
{
	async_schedule(htc_headset_gpio_init_async, NULL);
	return 0;
}

static void __exit htc_headset_gpio_exit(void)
{
	platform_driver_unregister(&htc_headset_gpio_driver);
}

module_init(htc_headset_gpio_init);
module_exit(htc_headset_gpio_exit);

MODULE_DESCRIPTION("HTC GPIO headset driver");
MODULE_LICENSE("GPL");

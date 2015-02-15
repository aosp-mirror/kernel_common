/*
 *
 * /arch/arm/mach-msm/htc_headset_pmic.c
 *
 * HTC PMIC headset driver.
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/mfd/pm8xxx/core.h>

#include <mach/msm_rpcrouter.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_pmic.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/async.h>
#include <linux/sched.h>
#include <linux/wait.h>
#endif
#ifdef HTC_HEADSET_CONFIG_PMIC_8XXX_ADC
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#elif defined HTC_HEADSET_CONFIG_QPNP_ADC
#include <linux/qpnp/qpnp-adc.h>
#endif

#define DRIVER_NAME "HS_PMIC"

#define HSPMIC_DT_DELAYINIT 	msecs_to_jiffies(3000)
#define HSPMIC_DT_DELAYTIMEOUT 	msecs_to_jiffies(5000)
struct htc_hs_pmic_data {
	struct device *dev;
	struct workqueue_struct *wq_raw;
	struct delayed_work work_raw;
	struct htc_headset_pmic_platform_data *pdata;
	wait_queue_head_t hspmic_wait;
	atomic_t wait_condition;
	bool parser;
};

static struct workqueue_struct *detect_wq;
static void detect_pmic_work_func(struct work_struct *work);

static void irq_init_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(irq_init_work, irq_init_work_func);

static struct workqueue_struct *button_wq;
static void button_pmic_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_pmic_work, button_pmic_work_func);
static unsigned int hpin_count_global;
static struct htc_35mm_pmic_info *hi;
static struct detect_pmic_work_info
	{
		struct delayed_work hpin_work;
		unsigned int intr_count;
		unsigned int insert;
	} detect_pmic_work;

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
static struct msm_rpc_endpoint *endpoint_adc;
static struct msm_rpc_endpoint *endpoint_current;

static struct hs_pmic_current_threshold current_threshold_lut[] = {
	{
		.adc_max = 14909, 
		.adc_min = 0, 
		.current_uA = 500,
	},
	{
		.adc_max = 29825, 
		.adc_min = 14910, 
		.current_uA = 600,
	},
	{
		.adc_max = 65535, 
		.adc_min = 29826, 
		.current_uA = 500,
	},
};
#endif

static enum hrtimer_restart hs_hpin_irq_enable_func(struct hrtimer *timer)
{
	HS_LOG("Re-Enable HPIN IRQ");
	enable_irq(hi->pdata.hpin_irq);
	return HRTIMER_NORESTART;
}

static int hs_pmic_hpin_state(void)
{
	HS_DBG();

	return gpio_get_value(hi->pdata.hpin_gpio);
}

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
static int hs_pmic_remote_threshold(uint32_t adc)
{
	int i = 0;
	int ret = 0;
	int array_size = 0;
	uint32_t status;
	struct hs_pmic_rpc_request req;
	struct hs_pmic_rpc_reply rep;

	HS_DBG();

	if (!(hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD))
		return 0;

	req.hs_controller = cpu_to_be32(hi->pdata.hs_controller);
	req.hs_switch = cpu_to_be32(hi->pdata.hs_switch);
	req.current_uA = cpu_to_be32(HS_PMIC_HTC_CURRENT_THRESHOLD);

	array_size = sizeof(current_threshold_lut) /
		     sizeof(struct hs_pmic_current_threshold);

	for (i = 0; i < array_size; i++) {
		if (adc >= current_threshold_lut[i].adc_min &&
		    adc <= current_threshold_lut[i].adc_max)
			req.current_uA = cpu_to_be32(current_threshold_lut[i].
						     current_uA);
	}

	ret = msm_rpc_call_reply(endpoint_current,
				 HS_PMIC_RPC_CLIENT_PROC_THRESHOLD,
				 &req, sizeof(req), &rep, sizeof(rep),
				 HS_RPC_TIMEOUT);

	if (ret < 0) {
		HS_ERR("Failed to send remote threshold RPC");
		return 0;
	} else {
		status = be32_to_cpu(rep.status);
		if (status != HS_PMIC_RPC_ERR_SUCCESS) {
			HS_ERR("Failed to set remote threshold");
			return 0;
		}
	}

	HS_LOG("Set remote threshold (%u, %u, %u)", hi->pdata.hs_controller,
	       hi->pdata.hs_switch, be32_to_cpu(req.current_uA));

	return 1;
}
#endif

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
static int hs_pmic_remote_adc(int *adc)
{
	int ret = 0;
	struct rpc_request_hdr req;
	struct hs_rpc_client_rep_adc rep;

	HS_DBG();

	ret = msm_rpc_call_reply(endpoint_adc, HS_RPC_CLIENT_PROC_ADC,
				 &req, sizeof(req), &rep, sizeof(rep),
				 HS_RPC_TIMEOUT);
	if (ret < 0) {
		*adc = -1;
		HS_LOG("Failed to read remote ADC");
		return 0;
	}

	*adc = (int) be32_to_cpu(rep.adc);
	HS_LOG("Remote ADC %d (%#X)", *adc, *adc);

	return 1;
}
#endif

#ifdef HTC_HEADSET_CONFIG_PMIC_8XXX_ADC
static int hs_pmic_remote_adc_pm8921(int *adc)
{
	struct pm8xxx_adc_chan_result result;

	HS_DBG();

	result.physical = -EINVAL;
	pm8xxx_adc_mpp_config_read(hi->pdata.adc_mpp, hi->pdata.adc_amux,
				   &result);
	*adc = (int) result.physical;
	*adc = *adc / 1000; 
	HS_LOG("Remote ADC %d (%#X)", *adc, *adc);

	return 1;
}
#elif defined HTC_HEADSET_CONFIG_QPNP_ADC
static int hs_qpnp_remote_adc(int *adc)
{
	struct qpnp_vadc_result result;
	enum qpnp_vadc_channels chan;
	static struct qpnp_vadc_chip *vadc_chip;
	int err = 0;

	HS_DBG();
	vadc_chip = qpnp_get_vadc(hi->dev, "headset");

	result.physical = -EINVAL;
	chan = hi->pdata.adc_channel;
	HS_LOG("pdata_chann %d", chan);
	err = qpnp_vadc_read(vadc_chip, chan, &result);
	if (err < 0) {
		HS_LOG("Read ADC fail, ret = %d", err);
		return err;
	}

	*adc = (int) result.physical;
	*adc = *adc / 1000; 
	HS_LOG("Remote ADC %d (%#X)", *adc, *adc);

	return 1;
 }
#endif

static int hs_pmic_mic_status(void)
{
	int adc = 0;
	int mic = HEADSET_UNKNOWN_MIC;

	HS_DBG();

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
	if (!hs_pmic_remote_adc(&adc))
		return HEADSET_UNKNOWN_MIC;

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD)
		hs_pmic_remote_threshold((unsigned int) adc);
#endif

#ifdef HTC_HEADSET_CONFIG_PMIC_8XXX_ADC
	if (!hs_pmic_remote_adc_pm8921(&adc))
		return HEADSET_UNKNOWN_MIC;
#endif

#ifdef HTC_HEADSET_CONFIG_QPNP_ADC
	if (!hs_qpnp_remote_adc(&adc))
		return HEADSET_UNKNOWN_MIC;
#endif

	if (adc >= hi->pdata.adc_mic_bias[0] &&
	    adc <= hi->pdata.adc_mic_bias[1])
		mic = HEADSET_MIC;
	else if (adc < hi->pdata.adc_mic_bias[0])
		mic = HEADSET_NO_MIC;
	else
		mic = HEADSET_UNKNOWN_MIC;

	return mic;
}

static int hs_pmic_adc_to_keycode(int adc)
{
	int key_code = HS_MGR_KEY_INVALID;

	HS_DBG();

	if (!hi->pdata.adc_remote[5])
		return HS_MGR_KEY_INVALID;

	if (adc >= hi->pdata.adc_remote[0] &&
	    adc <= hi->pdata.adc_remote[1])
		key_code = HS_MGR_KEY_PLAY;
	else if (adc >= hi->pdata.adc_remote[2] &&
		 adc <= hi->pdata.adc_remote[3])
		key_code = HS_MGR_KEY_BACKWARD;
	else if (adc >= hi->pdata.adc_remote[4] &&
		 adc <= hi->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_FORWARD;
	else if (adc > hi->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_NONE;

	if (key_code != HS_MGR_KEY_INVALID)
		HS_LOG("Key code %d", key_code);
	else
		HS_LOG("Unknown key code %d", key_code);

	return key_code;
}

static void hs_pmic_rpc_key(int adc)
{
	int key_code = hs_pmic_adc_to_keycode(adc);

	HS_DBG();

	if (key_code != HS_MGR_KEY_INVALID)
		hs_notify_key_event(key_code);
}

static void hs_pmic_key_enable(int enable)
{
	HS_DBG();

	if (hi->pdata.key_enable_gpio)
		gpio_set_value(hi->pdata.key_enable_gpio, enable);
}

static void detect_pmic_work_func(struct work_struct *work)
{
	struct detect_pmic_work_info *detect_pmic_work_ptr;
	detect_pmic_work_ptr = container_of(work, struct detect_pmic_work_info, hpin_work.work);

	HS_DBG();

	hs_notify_plug_event(detect_pmic_work_ptr->insert, detect_pmic_work_ptr->intr_count);
}

static irqreturn_t detect_irq_handler(int irq, void *data)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;
	unsigned int hpin_count_local;
	disable_irq_nosync(hi->pdata.hpin_irq);
	HS_LOG("Disable HPIN IRQ");
	hrtimer_start(&hi->timer, ktime_set(0, 200*NSEC_PER_MSEC), HRTIMER_MODE_REL);
	hpin_count_local = hpin_count_global++;
	detect_pmic_work.insert = gpio_get_value(hi->pdata.hpin_gpio);
	HS_LOG("HPIN++%d++, value = %d, trigger_type = 0x%x", hpin_count_local, detect_pmic_work.insert, hi->hpin_irq_type);
	hs_notify_hpin_irq();
	HS_DBG();

	if (!(hi->pdata.driver_flag & DRIVER_HS_PMIC_EDGE_IRQ)) {
		if (hi->hpin_irq_type == IRQF_TRIGGER_LOW)
			detect_pmic_work.insert = 1;
		else
			detect_pmic_work.insert = 0;
		hi->hpin_irq_type ^= irq_mask;
		set_irq_type(hi->pdata.hpin_irq, hi->hpin_irq_type);
	}

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	detect_pmic_work.intr_count = hpin_count_local;
	queue_delayed_work(detect_wq, &detect_pmic_work.hpin_work, hi->hpin_debounce);

	HS_LOG("HPIN--%d--, insert = %d, trigger_type = 0x%x", hpin_count_local, detect_pmic_work.insert, hi->hpin_irq_type);
	return IRQ_HANDLED;
}

static void button_pmic_work_func(struct work_struct *work)
{
	HS_DBG();
	hs_notify_key_irq();
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	HS_DBG();

	if (!(hi->pdata.driver_flag & DRIVER_HS_PMIC_EDGE_IRQ)) {
		hi->key_irq_type ^= irq_mask;
		set_irq_type(hi->pdata.key_irq, hi->key_irq_type);
	}
	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(button_wq, &button_pmic_work, HS_JIFFIES_ZERO);

	return IRQ_HANDLED;
}

static void irq_init_work_func(struct work_struct *work)
{
	unsigned int irq_type = IRQF_TRIGGER_LOW;

	HS_DBG();

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_EDGE_IRQ)
		irq_type = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	if (hi->pdata.hpin_gpio) {
		HS_LOG("Enable detect IRQ");
		hi->hpin_irq_type = irq_type;
		set_irq_type(hi->pdata.hpin_irq, hi->hpin_irq_type);
		enable_irq(hi->pdata.hpin_irq);
	}

	if (hi->pdata.key_gpio) {
		HS_LOG("Setup button IRQ type");
		hi->key_irq_type = irq_type;
		set_irq_type(hi->pdata.key_irq, hi->key_irq_type);
		if (set_irq_wake(hi->pdata.key_irq, 0) < 0)
			HS_LOG("Disable remote key irq wake failed");
	}
}

static void hs_pmic_key_int_enable(int enable)
{
	static int enable_count = 0;
	if (enable == 1 && enable_count == 0) {
		enable_irq(hi->pdata.key_irq);
		enable_count++;
		if (set_irq_wake(hi->pdata.key_irq, 1) < 0)
			HS_LOG("Enable remote key irq wake failed");
		HS_LOG("Enable remote key irq and wake");
	} else if (enable == 0 && enable_count == 1) {
		disable_irq_nosync(hi->pdata.key_irq);
		enable_count--;
		if (set_irq_wake(hi->pdata.key_irq, 0) < 0)
			HS_LOG("Disable remote key irq wake failed");
		HS_LOG("Disable remote key irq and wake");
	}
	HS_LOG("enable_count = %d", enable_count);
}

static int hs_pmic_request_irq(unsigned int gpio, unsigned int *irq,
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

	ret = request_any_context_irq(*irq, handler, flags, name, NULL);
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

static void hs_pmic_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_pmic_hpin_state;
		headset_notifier_register(&notifier);
	}

	if ((hi->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) ||
	    (hi->pdata.driver_flag & DRIVER_HS_PMIC_ADC)) {
#ifdef HTC_HEADSET_CONFIG_MSM_RPC
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = hs_pmic_remote_adc;
		headset_notifier_register(&notifier);
#endif

#ifdef HTC_HEADSET_CONFIG_PMIC_8XXX_ADC
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = hs_pmic_remote_adc_pm8921;
		headset_notifier_register(&notifier);
#endif

#ifdef HTC_HEADSET_CONFIG_QPNP_ADC
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = hs_qpnp_remote_adc;
		headset_notifier_register(&notifier);
#endif
		notifier.id = HEADSET_REG_REMOTE_KEYCODE;
		notifier.func = hs_pmic_adc_to_keycode;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_RPC_KEY;
		notifier.func = hs_pmic_rpc_key;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_MIC_STATUS;
		notifier.func = hs_pmic_mic_status;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_pmic_key_enable;
		headset_notifier_register(&notifier);
	}


	if (hi->pdata.key_gpio) {
		notifier.id = HEADSET_REG_KEY_INT_ENABLE;
		notifier.func = hs_pmic_key_int_enable;
		headset_notifier_register(&notifier);
	}
}

static ssize_t pmic_adc_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int adc = 0;

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
	ret = hs_pmic_remote_adc(&adc);
#endif
	HS_DBG("button ADC = %d",adc);
	return ret;
}


static struct device_attribute dev_attr_pmic_headset_adc =\
	__ATTR(pmic_adc_debug, 0644, pmic_adc_debug_show, NULL);

int register_attributes(void)
{
	int ret = 0;
	hi->pmic_dev = device_create(hi->htc_accessory_class,
				NULL, 0, "%s", "pmic");
	if (unlikely(IS_ERR(hi->pmic_dev))) {
		ret = PTR_ERR(hi->pmic_dev);
		hi->pmic_dev = NULL;
	}

	
	ret = device_create_file(hi->pmic_dev, &dev_attr_pmic_headset_adc);
	if (ret)
		goto err_create_pmic_device_file;
	return 0;

err_create_pmic_device_file:
	device_unregister(hi->pmic_dev);
	HS_ERR("Failed to register pmic attribute file");
	return ret;
}

#ifdef CONFIG_OF
static int headset_pmic_parser_dt(struct htc_hs_pmic_data *pmic)
{
	int ret;
	uint8_t i;
	const char *parser_chann  = {"hs_pmic,adc_channel"},
	           *parser_remote = {"hs_pmic,adc_remote"},
		   *parser_fg     = {"hs_pmic,driver_flag"},
		   *parser_st[]   = {"hs_pmic,key_gpio", "hs_pmic,hpin_gpio"};
	uint32_t gpio[ARRAY_SIZE(parser_st)] = {0};
	struct device_node *dt = pmic->dev->of_node;
	HS_LOG("%s", __func__);

	ret = of_property_read_u32(dt, parser_fg, &pmic->pdata->driver_flag);
	if (ret < 0) {
		HS_LOG("DT:%s parser err, ret=%d", parser_fg, ret);
		goto parser_failed;
	} else
		HS_LOG("DT:%s=%d", parser_fg, pmic->pdata->driver_flag);

#ifndef HTC_HEADSET_CONFIG_QPNP_ADC
	ret = of_property_read_u32(dt, "hs_pmic,adc_mpp", &pmic->pdata->adc_mpp);
	if (ret < 0) {
		HS_LOG("DT:adc_mpp parser err");
		goto parser_failed;
	} else
		HS_LOG("DT:adc_mpp = %d", pmic->pdata->adc_mpp);

	ret = of_property_read_u32(dt, "hs_pmic,adc_amux", &pmic->pdata->adc_amux);
	if (ret < 0) {
		HS_LOG("DT:adc_amux parser err");
		goto parser_failed;
	} else
		HS_LOG("DT:adc_amuxp = %d", pmic->pdata->adc_amux);
#else
	ret = of_property_read_u32(dt, parser_chann, &pmic->pdata->adc_channel);
	if (ret < 0) {
		HS_LOG("DT:8974 adc channel parser err");
		goto parser_failed;
	} else
		HS_LOG("DT:adc_channel = %d", pmic->pdata->adc_channel);
#endif
	ret = of_property_read_u32_array(dt, parser_remote, (u32*)&pmic->pdata->adc_remote, 6);
	if (ret < 0) {
		HS_LOG("DT:ADC parser failure");
		goto parser_failed;
	} else {
		HS_LOG("DT:adc: %03d %03d %03d %03d %03d", pmic->pdata->adc_remote[1],
			pmic->pdata->adc_remote[2], pmic->pdata->adc_remote[3],
			pmic->pdata->adc_remote[4], pmic->pdata->adc_remote[5]);
	}

	for (i = 0; i < ARRAY_SIZE(parser_st); i++) {
		gpio[i] = of_get_named_gpio(dt, parser_st[i], 0);
		if (!gpio_is_valid(gpio[i])) {
			HS_LOG("DT %s parser failue, ret=%d", parser_st[i], gpio[i]);
			goto parser_failed;
		} else
			HS_LOG("DT:%s[%d] read", parser_st[i], gpio[i]);
	}
	pmic->pdata->key_gpio  = gpio[0];
	pmic->pdata->hpin_gpio = gpio[1];

	return 0;

parser_failed:
	return -1;
}

static void hs_pmic_dtwq(struct htc_hs_pmic_data *dt)
{
	HS_LOG("WQ:%s", __func__);

	dt->parser = (headset_pmic_parser_dt(dt) < 0) ? false: true;
}
#endif

static int htc_headset_pmic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct htc_hs_pmic_data *dtinfo;
	struct htc_headset_pmic_platform_data *pdata;
#ifdef HTC_HEADSET_CONFIG_MSM_RPC
	uint32_t vers = 0;
#endif

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_35mm_pmic_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	dtinfo = kzalloc(sizeof(*dtinfo), GFP_KERNEL);
	if (!dtinfo) {
		kfree(hi);
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			kfree(hi);
			kfree(dtinfo);
			return -ENOMEM;
		}

		dtinfo->dev    = &pdev->dev;
		dtinfo->pdata = pdata;
		hs_pmic_dtwq(dtinfo);

		HS_LOG("==DT parser %s==", dtinfo->parser ? "OK": "FAILED");
	} else {
		pdata = pdev->dev.platform_data;
	}

	hi->pdata.driver_flag     = pdata->driver_flag;
	hi->pdata.hpin_gpio       = pdata->hpin_gpio;
	hi->pdata.hpin_irq        = pdata->hpin_irq;
	hi->pdata.key_gpio        = pdata->key_gpio;
	hi->pdata.key_irq         = pdata->key_irq;
	hi->pdata.key_enable_gpio = pdata->key_enable_gpio;
#ifndef HTC_HEADSET_CONFIG_QPNP_ADC
	hi->pdata.adc_mpp         = pdata->adc_mpp;
	hi->pdata.adc_amux        = pdata->adc_amux;
#else
	hi->dev                   = &pdev->dev;
	hi->pdata.adc_channel	  = pdata->adc_channel;
#endif
	hi->pdata.hs_controller   = pdata->hs_controller;
	hi->pdata.hs_switch       = pdata->hs_switch;
	hi->pdata.adc_mic         = pdata->adc_mic;
	hi->htc_accessory_class   = hs_get_attribute_class();
	hpin_count_global         = 0;

	register_attributes();
	INIT_DELAYED_WORK(&detect_pmic_work.hpin_work, detect_pmic_work_func);
	hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->timer.function = hs_hpin_irq_enable_func;

	if (!hi->pdata.adc_mic)
		hi->pdata.adc_mic = HS_DEF_MIC_ADC_16_BIT_MIN;

	if (pdata->adc_mic_bias[0] && pdata->adc_mic_bias[1]) {
		memcpy(hi->pdata.adc_mic_bias, pdata->adc_mic_bias,
		       sizeof(hi->pdata.adc_mic_bias));
		hi->pdata.adc_mic = hi->pdata.adc_mic_bias[0];
	} else {
		hi->pdata.adc_mic_bias[0] = hi->pdata.adc_mic;
		hi->pdata.adc_mic_bias[1] = HS_DEF_MIC_ADC_16_BIT_MAX;
	}

	if (pdata->adc_remote[5])
		memcpy(hi->pdata.adc_remote, pdata->adc_remote,
		       sizeof(hi->pdata.adc_remote));

	if (pdata->adc_metrico[0] && pdata->adc_metrico[1])
		memcpy(hi->pdata.adc_metrico, pdata->adc_metrico,
		       sizeof(hi->pdata.adc_metrico));

	hi->hpin_irq_type = IRQF_TRIGGER_NONE;
	hi->hpin_debounce = HS_JIFFIES_ZERO;
	hi->key_irq_type = IRQF_TRIGGER_NONE;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	detect_wq = create_workqueue("HS_PMIC_DETECT");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("HS_PMIC_BUTTON");
	if (button_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	if (hi->pdata.hpin_gpio) {
		ret = hs_pmic_request_irq(hi->pdata.hpin_gpio,
				&hi->pdata.hpin_irq, detect_irq_handler,
				hi->hpin_irq_type, "HS_PMIC_DETECT", 1);
		if (ret < 0) {
			HS_ERR("Failed to request PMIC HPIN IRQ (0x%X)", ret);
			goto err_request_detect_irq;
		}
		disable_irq(hi->pdata.hpin_irq);
	}

	if (hi->pdata.key_gpio) {
		ret = hs_pmic_request_irq(hi->pdata.key_gpio,
				&hi->pdata.key_irq, button_irq_handler,
				hi->key_irq_type, "HS_PMIC_BUTTON", 1);
		if (ret < 0) {
			HS_ERR("Failed to request PMIC button IRQ (0x%X)", ret);
			goto err_request_button_irq;
		}
		disable_irq(hi->pdata.key_irq);
	}

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
		
		endpoint_adc = msm_rpc_connect(HS_RPC_CLIENT_PROG,
					       HS_RPC_CLIENT_VERS, 0);
		if (IS_ERR(endpoint_adc)) {
			hi->pdata.driver_flag &= ~DRIVER_HS_PMIC_RPC_KEY;
			HS_LOG("Failed to register ADC RPC client");
		} else
			HS_LOG("Register ADC RPC client successfully");
	}

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD) {
		
		vers = HS_PMIC_RPC_CLIENT_VERS_3_1;
		endpoint_current = msm_rpc_connect_compatible(
				   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS_2_1;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS_1_1;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (IS_ERR(endpoint_current)) {
			hi->pdata.driver_flag &=
				~DRIVER_HS_PMIC_DYNAMIC_THRESHOLD;
			HS_LOG("Failed to register threshold RPC client");
		} else
			HS_LOG("Register threshold RPC client successfully"
			       " (0x%X)", vers);
	}
#else
	hi->pdata.driver_flag &= ~DRIVER_HS_PMIC_RPC_KEY;
	hi->pdata.driver_flag &= ~DRIVER_HS_PMIC_DYNAMIC_THRESHOLD;
#endif

	queue_delayed_work(detect_wq, &irq_init_work, HS_JIFFIES_IRQ_INIT);

	hs_pmic_register();
#ifndef CONFIG_OF
	hs_notify_driver_ready(DRIVER_NAME);
#endif
	HS_LOG("--------------------");

	kfree(dtinfo);
	kfree(pdata);
	return 0;

err_request_button_irq:
	if (hi->pdata.hpin_gpio) {
		free_irq(hi->pdata.hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

err_request_detect_irq:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);
	kfree(pdata);
	kfree(dtinfo);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_pmic_remove(struct platform_device *pdev)
{
	if (hi->pdata.key_gpio) {
		free_irq(hi->pdata.key_irq, 0);
		gpio_free(hi->pdata.key_gpio);
	}

	if (hi->pdata.hpin_gpio) {
		free_irq(hi->pdata.hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);
	wake_lock_destroy(&hi->hs_wake_lock);

	kfree(hi);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id htc_headset_pmic_mttable[] = {
	{ .compatible = "htc_headset,pmic"},
	{ },
};
#else
#define htc_Headset_pmic_mttabl NULL
#endif

static struct platform_driver htc_headset_pmic_driver = {
	.probe	= htc_headset_pmic_probe,
	.remove	= htc_headset_pmic_remove,
	.driver	= {
		.name           = "HTC_HEADSET_PMIC",
		.owner          = THIS_MODULE,
		.of_match_table = htc_headset_pmic_mttable,
	},
};


static void __init htc_headset_pmic_init_async(void *unused, async_cookie_t cookie)
{
	async_synchronize_cookie(cookie);
	platform_driver_register(&htc_headset_pmic_driver);
}


static int __init htc_headset_pmic_init(void)
{
	async_schedule(htc_headset_pmic_init_async, NULL);
	return 0;
}

static void __exit htc_headset_pmic_exit(void)
{
	platform_driver_unregister(&htc_headset_pmic_driver);
}
late_initcall(htc_headset_pmic_init);
module_exit(htc_headset_pmic_exit);

MODULE_DESCRIPTION("HTC PMIC headset driver");
MODULE_LICENSE("GPL");

/*
 *
 * /arch/arm/mach-msm/htc_headset_mgr.c
 *
 * HTC headset manager driver.
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
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <mach/htc_headset_mgr.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/async.h>
#include <linux/sched.h>
#include <linux/wait.h>
#endif

#define DRIVER_NAME "HS_MGR"
#define HS_RX_GPIO 0
#define HS_TX_GPIO 1
#define HS_RX_ALT  2
#define HS_TX_ALT  3

#define HSMGR_DT_DELAYINIT 	msecs_to_jiffies(2000)
#define HSMGR_DT_DELAYTIMEOUT 	msecs_to_jiffies(4000)
struct htc_hs_mgr_data {
	struct device *dev;
	struct workqueue_struct *wq_raw;
	struct delayed_work work_raw;
	struct htc_headset_mgr_platform_data *pdata;
	wait_queue_head_t mgr_wait;
	atomic_t wait_condition;
	bool parser;
};

static struct workqueue_struct *detect_wq;
static void insert_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(insert_detect_work, insert_detect_work_func);
static void remove_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(remove_detect_work, remove_detect_work_func);
static void mic_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(mic_detect_work, mic_detect_work_func);

static struct workqueue_struct *button_wq;
static void button_35mm_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_35mm_work, button_35mm_work_func);

static void button_1wire_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_1wire_work, button_1wire_work_func);

static struct workqueue_struct *debug_wq;
static void debug_work_func(struct work_struct *work);
static DECLARE_WORK(debug_work, debug_work_func);

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len);

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_RPC_SERVER_PROG,
	.vers		= HS_RPC_SERVER_VERS,
	.rpc_call	= hs_mgr_rpc_call,
};

struct button_work {
	struct delayed_work key_work;
	int key_code;
};

static struct htc_headset_mgr_info *hi;
static struct hs_notifier_func hs_mgr_notifier;

static int	hpin_report = 0,
		hpin_bounce = 0,
		key_report = 0,
		key_bounce = 0;

#ifndef CONFIG_OF
static void init_next_driver(void)
{
	int i = hi->driver_init_seq;

	if (!hi->pdata.headset_devices_num)
		return;

	if (i < hi->pdata.headset_devices_num) {
		hi->driver_init_seq++;
		platform_device_register(hi->pdata.headset_devices[i]);
	}
}

void hs_notify_driver_ready(char *name)
{
	HS_LOG("%s ready", name);
	init_next_driver();
}
#endif

int hs_debug_log_state(void)
{
	return (hi->debug_flag & DEBUG_FLAG_LOG) ? 1 : 0;
}

void hs_notify_hpin_irq(void)
{
	hi->hpin_jiffies = jiffies;
	hpin_bounce++;
}

struct class *hs_get_attribute_class(void)
{
	return hi->htc_accessory_class;
}

int hs_hpin_stable(void)
{
	unsigned long last_hpin_jiffies = 0;
	unsigned long unstable_jiffies = 1.2 * HZ;

	HS_DBG();

	last_hpin_jiffies = hi->hpin_jiffies;

	if (time_before_eq(jiffies, last_hpin_jiffies + unstable_jiffies))
		return 0;

	return 1;
}

static int get_mic_state(void)
{
	HS_DBG();

	switch (hi->hs_35mm_type) {
	case HEADSET_MIC:
	case HEADSET_METRICO:
	case HEADSET_ONEWIRE:
		return 1;
	default:
		break;
	}

	return 0;
}

static void update_mic_status(int count)
{
	HS_DBG();

	if (hi->is_ext_insert) {
		HS_LOG("Start MIC status polling (%d)", count);
		cancel_delayed_work_sync(&mic_detect_work);
		hi->mic_detect_counter = count;
		queue_delayed_work(detect_wq, &mic_detect_work,
				   HS_JIFFIES_MIC_DETECT);
	}
}

static void headset_notifier_update(int id)
{
	if (!hi) {
		HS_LOG("HS_MGR driver is not ready");
		return;
	}

	switch (id) {
	case HEADSET_REG_HPIN_GPIO:
		break;
	case HEADSET_REG_REMOTE_ADC:
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
		break;
	case HEADSET_REG_REMOTE_KEYCODE:
	case HEADSET_REG_RPC_KEY:
		break;
	case HEADSET_REG_MIC_STATUS:
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
		break;
	case HEADSET_REG_MIC_BIAS:
		if (!hi->pdata.headset_power &&
		    hi->hs_35mm_type != HEADSET_UNPLUG) {
			hs_mgr_notifier.mic_bias_enable(1);
			hi->mic_bias_state = 1;
			msleep(HS_DELAY_MIC_BIAS);
			update_mic_status(HS_DEF_MIC_DETECT_COUNT);
		}
		break;
	case HEADSET_REG_MIC_SELECT:
	case HEADSET_REG_KEY_INT_ENABLE:
	case HEADSET_REG_KEY_ENABLE:
	case HEADSET_REG_INDICATOR_ENABLE:
		break;
	default:
		break;
	}
}

int headset_notifier_register(struct headset_notifier *notifier)
{
	if (!notifier->func) {
		HS_LOG("NULL register function");
		return 0;
	}

	switch (notifier->id) {
	case HEADSET_REG_HPIN_GPIO:
		HS_LOG("Register HPIN_GPIO notifier");
		hs_mgr_notifier.hpin_gpio = notifier->func;
		break;
	case HEADSET_REG_REMOTE_ADC:
		HS_LOG("Register REMOTE_ADC notifier");
		hs_mgr_notifier.remote_adc = notifier->func;
		break;
	case HEADSET_REG_REMOTE_KEYCODE:
		HS_LOG("Register REMOTE_KEYCODE notifier");
		hs_mgr_notifier.remote_keycode = notifier->func;
		break;
	case HEADSET_REG_RPC_KEY:
		HS_LOG("Register RPC_KEY notifier");
		hs_mgr_notifier.rpc_key = notifier->func;
		break;
	case HEADSET_REG_MIC_STATUS:
		HS_LOG("Register MIC_STATUS notifier");
		hs_mgr_notifier.mic_status = notifier->func;
		break;
	case HEADSET_REG_MIC_BIAS:
		HS_LOG("Register MIC_BIAS notifier");
		hs_mgr_notifier.mic_bias_enable = notifier->func;
		break;
	case HEADSET_REG_MIC_SELECT:
		HS_LOG("Register MIC_SELECT notifier");
		hs_mgr_notifier.mic_select = notifier->func;
		break;
	case HEADSET_REG_KEY_INT_ENABLE:
		HS_LOG("Register KEY_INT_ENABLE notifier");
		hs_mgr_notifier.key_int_enable = notifier->func;
		break;
	case HEADSET_REG_KEY_ENABLE:
		HS_LOG("Register KEY_ENABLE notifier");
		hs_mgr_notifier.key_enable = notifier->func;
		break;
	case HEADSET_REG_INDICATOR_ENABLE:
		HS_LOG("Register INDICATOR_ENABLE notifier");
		hs_mgr_notifier.indicator_enable = notifier->func;
		break;
	case HEADSET_REG_UART_SET:
		HS_LOG("Register UART_SET notifier");
		hs_mgr_notifier.uart_set = notifier->func;
		break;
	case HEADSET_REG_1WIRE_INIT:
		HS_LOG("Register 1WIRE_INIT notifier");
		hs_mgr_notifier.hs_1wire_init = notifier->func;
		hi->driver_one_wire_exist = 1;
		break;
	case HEADSET_REG_1WIRE_QUERY:
		HS_LOG("Register 1WIRE_QUERY notifier");
		hs_mgr_notifier.hs_1wire_query = notifier->func;
		break;
	case HEADSET_REG_1WIRE_READ_KEY:
		HS_LOG("Register 1WIRE_READ_KEY notifier");
		hs_mgr_notifier.hs_1wire_read_key = notifier->func;
		break;
	case HEADSET_REG_1WIRE_DEINIT:
		HS_LOG("Register 1WIRE_DEINIT notifier");
		hs_mgr_notifier.hs_1wire_deinit = notifier->func;
		break;
	case HEADSET_REG_1WIRE_REPORT_TYPE:
		HS_LOG("Register 1WIRE_REPORT_TYPE notifier");
		hs_mgr_notifier.hs_1wire_report_type = notifier->func;
		break;
	case HEADSET_REG_1WIRE_OPEN:
		HS_LOG("Register 1WIRE_OPEN notifier");
		hs_mgr_notifier.hs_1wire_open = notifier->func;
		break;
	case HEADSET_REG_HS_INSERT:
		HS_LOG("Register HS_INSERT notifier");
		hs_mgr_notifier.hs_insert = notifier->func;
		break;
	default:
		HS_LOG("Unknown register ID");
		return 0;
	}

	headset_notifier_update(notifier->id);

	return 1;
}

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len)
{
	struct hs_rpc_server_args_key *args_key;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	switch (req->procedure) {
	case HS_RPC_SERVER_PROC_NULL:
		HS_LOG("RPC_SERVER_NULL");
		break;
	case HS_RPC_SERVER_PROC_KEY:
		args_key = (struct hs_rpc_server_args_key *)(req + 1);
		args_key->adc = be32_to_cpu(args_key->adc);
		HS_LOG("RPC_SERVER_KEY ADC = %u (0x%X)",
			args_key->adc, args_key->adc);
		if (hs_mgr_notifier.rpc_key)
			hs_mgr_notifier.rpc_key(args_key->adc);
		else
			HS_LOG("RPC_KEY notify function doesn't exist");
		break;
	default:
		HS_LOG("Unknown RPC procedure");
		return -EINVAL;
	}

	return 0;
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "Headset\n");
}

static ssize_t usb_audio_print_name(struct switch_dev *sdev, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "usb_audio\n");
}

static void get_key_name(int keycode, char *buf)
{
	switch (keycode) {
	case HS_MGR_KEYCODE_END:
		snprintf(buf, PAGE_SIZE, "END");
		break;
	case HS_MGR_KEYCODE_MUTE:
		snprintf(buf, PAGE_SIZE, "MUTE");
		break;
	case HS_MGR_KEYCODE_VOLDOWN:
		snprintf(buf, PAGE_SIZE, "VOLDOWN");
		break;
	case HS_MGR_KEYCODE_VOLUP:
		snprintf(buf, PAGE_SIZE, "VOLUP");
		break;
	case HS_MGR_KEYCODE_FORWARD:
		snprintf(buf, PAGE_SIZE, "FORWARD");
		break;
	case HS_MGR_KEYCODE_PLAY:
		snprintf(buf, PAGE_SIZE, "PLAY");
		break;
	case HS_MGR_KEYCODE_BACKWARD:
		snprintf(buf, PAGE_SIZE, "BACKWARD");
		break;
	case HS_MGR_KEYCODE_MEDIA:
		snprintf(buf, PAGE_SIZE, "MEDIA");
		break;
	case HS_MGR_KEYCODE_SEND:
		snprintf(buf, PAGE_SIZE, "SEND");
		break;
	case HS_MGR_KEYCODE_FF:
		snprintf(buf, PAGE_SIZE, "FastForward");
		break;
	case HS_MGR_KEYCODE_RW:
		snprintf(buf, PAGE_SIZE, "ReWind");
		break;
	default:
		snprintf(buf, PAGE_SIZE, "%d", keycode);
	}
}

void button_pressed(int type)
{
	char key_name[16];

	get_key_name(type, key_name);
	HS_LOG_TIME("%s (%d) pressed", key_name, type);
	atomic_set(&hi->btn_state, type);
	input_report_key(hi->input, type, 1);
	input_sync(hi->input);
	key_report++;
}

void button_released(int type)
{
	char key_name[16];

	get_key_name(type, key_name);
	HS_LOG_TIME("%s (%d) released", key_name, type);
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, type, 0);
	input_sync(hi->input);
	key_report++;
}

void headset_button_event(int is_press, int type)
{
	HS_DBG();

	if (hi->hs_35mm_type == HEADSET_UNPLUG &&
	    hi->h2w_35mm_type == HEADSET_UNPLUG) {
		HS_LOG("IGNORE key %d (HEADSET_UNPLUG)", type);
		return;
	}

	if (!hs_hpin_stable()) {
		HS_LOG("IGNORE key %d (Unstable HPIN)", type);
		return;
	}

	if (!get_mic_state()) {
		HS_LOG("IGNORE key %d (Not support MIC)", type);
		return;
	}

	if (!is_press)
		button_released(type);
	else if (!atomic_read(&hi->btn_state))
		button_pressed(type);
}

void hs_set_mic_select(int state)
{
	HS_DBG();

	if (hs_mgr_notifier.mic_select)
		hs_mgr_notifier.mic_select(state);
}

static int get_mic_status(void)
{
	int i = 0;
	int adc = 0;
	int mic = HEADSET_UNKNOWN_MIC;

	if (hi->pdata.headset_config_num && hs_mgr_notifier.remote_adc) {
		hs_mgr_notifier.remote_adc(&adc);
		for (i = 0; i < hi->pdata.headset_config_num; i++) {
			if (adc >= hi->pdata.headset_config[i].adc_min &&
			    adc <= hi->pdata.headset_config[i].adc_max)
				return hi->pdata.headset_config[i].type;
		}
			if (hi->pdata.driver_flag & DRIVER_HS_MGR_FLOAT_DET) {
				return HEADSET_UNPLUG;
			}
	} else if (hs_mgr_notifier.mic_status) {
		mic = hs_mgr_notifier.mic_status();
	}
	else
		HS_LOG("Failed to get MIC status");
	return mic;
}

int headset_get_type(void)
{
	return hi->hs_35mm_type;
}

int headset_get_type_sync(int count, unsigned int interval)
{
	int current_type = hi->hs_35mm_type;
	int new_type = HEADSET_UNKNOWN_MIC;

	while (count--) {
		new_type = get_mic_status();
		if (new_type != current_type)
			break;
		if (count)
			msleep(interval);
	}

	if (new_type != current_type) {
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
		return HEADSET_UNKNOWN_MIC;
	}

	return hi->hs_35mm_type;
}

static void set_35mm_hw_state(int state)
{
	HS_DBG();


	if (hi->pdata.headset_power || hs_mgr_notifier.mic_bias_enable) {
		if (hi->mic_bias_state != state) {
			if (hi->pdata.headset_power)
				hi->pdata.headset_power(state);
			if (hs_mgr_notifier.mic_bias_enable)
				hs_mgr_notifier.mic_bias_enable(state);

			hi->mic_bias_state = state;
			if (state) 
				msleep(HS_DELAY_MIC_BIAS);
		}
	}

	hs_set_mic_select(state);

	if (hs_mgr_notifier.key_enable)
		hs_mgr_notifier.key_enable(state);
}

static int tv_out_detect(void)
{
	int adc = 0;
	int mic = HEADSET_NO_MIC;

	HS_DBG();

	if (!hs_mgr_notifier.remote_adc)
		return HEADSET_NO_MIC;

	if (!hi->pdata.hptv_det_hp_gpio || !hi->pdata.hptv_det_tv_gpio)
		return HEADSET_NO_MIC;

	gpio_set_value(hi->pdata.hptv_det_hp_gpio, 0);
	gpio_set_value(hi->pdata.hptv_det_tv_gpio, 1);
	msleep(HS_DELAY_MIC_BIAS);

	hs_mgr_notifier.remote_adc(&adc);
	if (adc >= HS_DEF_HPTV_ADC_16_BIT_MIN &&
	    adc <= HS_DEF_HPTV_ADC_16_BIT_MAX)
	mic = HEADSET_TV_OUT;

	gpio_set_value(hi->pdata.hptv_det_hp_gpio, 1);
	gpio_set_value(hi->pdata.hptv_det_tv_gpio, 0);

	return mic;
}

#if 0
static void insert_h2w_35mm(int *state)
{
	int mic = HEADSET_NO_MIC;

	HS_LOG_TIME("Insert H2W 3.5mm headset");
	set_35mm_hw_state(1);

	mic = get_mic_status();

	if (mic == HEADSET_NO_MIC) {
		*state |= BIT_HEADSET_NO_MIC;
		hi->h2w_35mm_type = HEADSET_NO_MIC;
		HS_LOG_TIME("H2W 3.5mm without microphone");
	} else {
		*state |= BIT_HEADSET;
		hi->h2w_35mm_type = HEADSET_MIC;
		HS_LOG_TIME("H2W 3.5mm with microphone");
	}
}

static void remove_h2w_35mm(void)
{
	HS_LOG_TIME("Remove H2W 3.5mm headset");

	set_35mm_hw_state(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->h2w_35mm_type = HEADSET_UNPLUG;
}
#endif 

static void enable_metrico_headset(int enable)
{
	HS_DBG();

	if (enable && !hi->metrico_status) {
#if 0
		enable_mos_test(1);
#endif
		hi->metrico_status = 1;
		HS_LOG("Enable metrico headset");
	}

	if (!enable && hi->metrico_status) {
#if 0
		enable_mos_test(0);
#endif
		hi->metrico_status = 0;
		HS_LOG("Disable metrico headset");
	}
}

static void mic_detect_work_func(struct work_struct *work)
{
	int mic = HEADSET_NO_MIC;
	int old_state, new_state;
	int adc = 0;

	wake_lock_timeout(&hi->hs_wake_lock, HS_MIC_DETECT_TIMEOUT);

	HS_DBG();

	if (!hi->pdata.headset_config_num && !hs_mgr_notifier.mic_status) {
		HS_LOG("Failed to get MIC status");
		return;
	}

	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(0);

	mutex_lock(&hi->mutex_lock);
	

#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
	if (!hi->driver_one_wire_exist && hs_mgr_notifier.hs_insert)
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		if (!hi->plugout_redetect)
#endif
			hs_mgr_notifier.hs_insert(1);
#endif

#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
	if (hi->driver_one_wire_exist && hi->one_wire_mode == 0 && !hi->plugout_redetect) {
#else
	if (hi->driver_one_wire_exist && hi->one_wire_mode == 0) {
#endif

#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
		if (hs_mgr_notifier.hs_insert)
			hs_mgr_notifier.hs_insert(1);
#endif

		HS_LOG("1-wire re-detecting sequence");
		if (hi->pdata.uart_tx_gpo)
			{
				hi->pdata.uart_tx_gpo(HS_RX_ALT);
				hi->pdata.uart_tx_gpo(HS_TX_GPIO);
			}
		if (hi->pdata.uart_lv_shift_en)
			hi->pdata.uart_lv_shift_en(0);
		msleep(20);
		if (hi->pdata.uart_lv_shift_en)
			hi->pdata.uart_lv_shift_en(1);
		if (hi->pdata.uart_tx_gpo)
			hi->pdata.uart_tx_gpo(HS_TX_ALT);
		msleep(150);
		if (hs_mgr_notifier.remote_adc)
			hs_mgr_notifier.remote_adc(&adc);
		hi->one_wire_mode = 0;
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		if (adc > 915 && adc < hi->pdata.headset_config[0].adc_max) {
#else
		if (adc > 915) {
#endif
			HS_LOG("Not HEADSET_NO_MIC, start 1wire init");
			if (hs_mgr_notifier.hs_1wire_init() == 0) {
				hi->one_wire_mode = 1;
				old_state = switch_get_state(&hi->sdev_h2w);
				new_state = BIT_HEADSET;
				if (old_state == BIT_HEADSET_NO_MIC) {
					HS_LOG("no_mic to mic workaround");
					new_state = BIT_HEADSET | BIT_HEADSET_NO_MIC;
				}
				HS_LOG("old_state = 0x%x, new_state = 0x%x", old_state, new_state);
				switch_set_state(&hi->sdev_h2w, new_state);
				hi->hs_35mm_type = HEADSET_ONEWIRE;
				mutex_unlock(&hi->mutex_lock);
				if (hs_mgr_notifier.key_int_enable)
					hs_mgr_notifier.key_int_enable(1);
				return;
			} else {
				hi->one_wire_mode = 0;
				HS_LOG("Legacy mode");
				if (hi->pdata.uart_tx_gpo)
					hi->pdata.uart_tx_gpo(HS_TX_ALT);
			}
		}
	}


	mic = get_mic_status();

	if (mic == HEADSET_NO_MIC)
		mic = tv_out_detect();

	if (mic == HEADSET_TV_OUT && hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 1);

	if (mic == HEADSET_METRICO && !hi->metrico_status)
		enable_metrico_headset(1);

	if (mic == HEADSET_UNKNOWN_MIC || mic == HEADSET_UNPLUG) {
		if (hi->mic_detect_counter--) {
			mutex_unlock(&hi->mutex_lock);
			queue_delayed_work(detect_wq, &mic_detect_work,
					   HS_JIFFIES_MIC_DETECT);
			return;
		} else {
			HS_LOG("MIC polling timeout (UNKNOWN/Floating MIC status)");
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
			hi->plugout_redetect = 0;
#else
			mutex_unlock(&hi->mutex_lock);
			set_35mm_hw_state(0);			
			return;
#endif
		}
	}

	if (hi->hs_35mm_type == HEADSET_UNSTABLE && hi->mic_detect_counter--) {
		mutex_unlock(&hi->mutex_lock);
		queue_delayed_work(detect_wq, &mic_detect_work,
				   HS_JIFFIES_MIC_DETECT);
		return;
	}

	old_state = switch_get_state(&hi->sdev_h2w);
	if (!(old_state & MASK_35MM_HEADSET) && !(hi->is_ext_insert)) {
		HS_LOG("Headset has been removed");
		mutex_unlock(&hi->mutex_lock);
		return;
	}

	new_state = old_state & ~MASK_35MM_HEADSET;

	switch (mic) {
	case HEADSET_UNPLUG:
		new_state &= ~MASK_35MM_HEADSET;
		HS_LOG("HEADSET_UNPLUG (FLOAT)");
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		if (hi->one_wire_mode == 1) {
			hi->one_wire_mode = 0;
		}
#endif
		break;
	case HEADSET_NO_MIC:
		new_state |= BIT_HEADSET_NO_MIC;
		HS_LOG("HEADSET_NO_MIC");
#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
		set_35mm_hw_state(0);
#endif
		break;
	case HEADSET_MIC:
		new_state |= BIT_HEADSET;
		HS_LOG("HEADSET_MIC");
		break;
	case HEADSET_METRICO:
		new_state |= BIT_HEADSET;
		HS_LOG("HEADSET_METRICO");
		break;
	case HEADSET_TV_OUT:
		new_state |= BIT_TV_OUT;
		HS_LOG("HEADSET_TV_OUT");
#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
		tvout_enable_detection(1);
#endif
		break;
	case HEADSET_INDICATOR:
		HS_LOG("HEADSET_INDICATOR");
		break;
	case HEADSET_UART:
		HS_LOG("HEADSET_UART");
		if (hs_mgr_notifier.uart_set)
			hs_mgr_notifier.uart_set(1);
		break;
	}

	if (new_state != old_state) {
		HS_LOG_TIME("Plug/Unplug accessory, old_state 0x%x, new_state 0x%x", old_state, new_state);
		hi->hs_35mm_type = mic;
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		if (new_state != 0) {
			new_state |= old_state;
		}
		else {
			new_state = 0;
#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
			if (hs_mgr_notifier.hs_insert)
				hs_mgr_notifier.hs_insert(0);
#endif
		}
#else
		new_state |= old_state;
#endif
		switch_set_state(&hi->sdev_h2w, new_state);
		HS_LOG_TIME("Sent uevent 0x%x ==> 0x%x", old_state, new_state);
		hpin_report++;
	} else
		HS_LOG("MIC status has not changed");

#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
if (mic != HEADSET_NO_MIC)
	{
#endif
	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(1);
#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
	}
#endif
	mutex_unlock(&hi->mutex_lock);
}

static void button_35mm_work_func(struct work_struct *work)
{
	int key;
	struct button_work *works;
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
	int adc = 0;
#endif

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	works = container_of(work, struct button_work, key_work.work);
	hi->key_level_flag = works->key_code;

#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
	hs_mgr_notifier.remote_adc(&adc);
	if (adc > hi->pdata.headset_config[0].adc_max) {
		hi->plugout_redetect = 1;
		cancel_delayed_work_sync(&mic_detect_work);
		hi->mic_detect_counter = 0;
		queue_delayed_work(detect_wq, &mic_detect_work, 0);
		wake_unlock(&hi->hs_wake_lock);
		kfree(works);
		return;
	}
#endif

	if (hi->key_level_flag) {
		switch (hi->key_level_flag) {
		case 1:
			key = HS_MGR_KEYCODE_MEDIA;
			break;
		case 2:
			key = HS_MGR_KEYCODE_BACKWARD;
			break;
		case 3:
			key = HS_MGR_KEYCODE_FORWARD;
			break;
		default:
			HS_LOG("3.5mm RC: WRONG Button Pressed");
			kfree(works);
			return;
		}
		headset_button_event(1, key);
	} else { 
		if (atomic_read(&hi->btn_state))
			headset_button_event(0, atomic_read(&hi->btn_state));
		else
			HS_LOG("3.5mm RC: WRONG Button Release");
	}

	kfree(works);
}

static void debug_work_func(struct work_struct *work)
{
	int flag = 0;
	int adc = -EINVAL;
	int hpin_gpio = -EINVAL;

	HS_DBG();

	while (hi->debug_flag & DEBUG_FLAG_ADC) {
		flag = hi->debug_flag;
		if (hs_mgr_notifier.hpin_gpio)
			hpin_gpio = hs_mgr_notifier.hpin_gpio();
		if (hs_mgr_notifier.remote_adc)
			hs_mgr_notifier.remote_adc(&adc);
		HS_LOG("Debug Flag %d, HP_DET %d, ADC %d", flag,
		       hpin_gpio, adc);
		msleep(HS_DELAY_SEC);
	}
}

static void remove_detect_work_func(struct work_struct *work)
{
	int state;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	if (time_before_eq(jiffies, hi->insert_jiffies + HZ)) {
		HS_LOG("Waiting for HPIN stable");
		msleep(HS_DELAY_SEC - HS_DELAY_REMOVE);
	}

	if (hi->is_ext_insert || hs_mgr_notifier.hpin_gpio() == 0) {
		HS_LOG("Headset has been inserted");
		return;
	}

	if (hi->hs_35mm_type == HEADSET_INDICATOR &&
	    hs_mgr_notifier.indicator_enable)
		hs_mgr_notifier.indicator_enable(0);

#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
	if ((hs_mgr_notifier.hs_insert)&&(hi->hs_35mm_type != HEADSET_UNPLUG))
		hs_mgr_notifier.hs_insert(0);
#endif

	set_35mm_hw_state(0);
#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
	if (hi->hs_35mm_type == HEADSET_TV_OUT && hi->pdata.hptv_sel_gpio) {
		HS_LOG_TIME("Remove 3.5mm TVOUT cable");
		tvout_enable_detection(0);
		gpio_set_value(hi->pdata.hptv_sel_gpio, 0);
	}
#endif
	if (hi->metrico_status)
		enable_metrico_headset(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->hs_35mm_type = HEADSET_UNPLUG;

	mutex_lock(&hi->mutex_lock);

	if (hs_mgr_notifier.uart_set)
		hs_mgr_notifier.uart_set(0);

	if (hi->pdata.uart_tx_gpo)
		{
			hi->pdata.uart_tx_gpo(HS_TX_GPIO);
			hi->pdata.uart_tx_gpo(HS_RX_GPIO);
		}

	state = switch_get_state(&hi->sdev_h2w);
	if (!(state & MASK_35MM_HEADSET)) {
		HS_LOG("Headset has been removed");
		mutex_unlock(&hi->mutex_lock);
		return;
	}

#if 0
	if (hi->cable_in1 && !gpio_get_value(hi->cable_in1)) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev_h2w, state);
		queue_delayed_work(detect_wq, &detect_h2w_work,
				   HS_DELAY_ZERO_JIFFIES);
	} else {
		state &= ~(MASK_35MM_HEADSET | MASK_FM_ATTRIBUTE);
		switch_set_state(&hi->sdev_h2w, state);
	}
#else
	state &= ~(MASK_35MM_HEADSET | MASK_FM_ATTRIBUTE);
	switch_set_state(&hi->sdev_h2w, state);
#endif
	if (hi->one_wire_mode == 1) {
		hi->one_wire_mode = 0;
	}
	HS_LOG_TIME("Remove 3.5mm accessory");
	hpin_report++;
	mutex_unlock(&hi->mutex_lock);

#ifdef HTC_HEADSET_CONFIG_QUICK_BOOT
	if (gpio_event_get_quickboot_status())
		HS_LOG("quick_boot_status = 1");
#endif
}

static void insert_detect_work_func(struct work_struct *work)
{
	int old_state, new_state;
	int mic = HEADSET_NO_MIC;
	int adc = 0;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	if (!hi->is_ext_insert || hs_mgr_notifier.hpin_gpio() == 1) {
		HS_LOG("Headset has been removed");
		return;
	}

	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(0);

	set_35mm_hw_state(1);
	msleep(250); 

	#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
	if (hs_mgr_notifier.hs_insert)
		hs_mgr_notifier.hs_insert(1);
#endif

	HS_LOG("Start 1-wire detecting sequence");
	if (hi->pdata.uart_tx_gpo)
		{
			hi->pdata.uart_tx_gpo(HS_RX_ALT);
			hi->pdata.uart_tx_gpo(HS_TX_GPIO);
		}
	if (hi->pdata.uart_lv_shift_en)
		hi->pdata.uart_lv_shift_en(0);
	msleep(20);
	if (hi->pdata.uart_lv_shift_en)
		hi->pdata.uart_lv_shift_en(1);
	if (hi->pdata.uart_tx_gpo)
		hi->pdata.uart_tx_gpo(HS_TX_ALT);
	hi->insert_jiffies = jiffies;
	msleep(150);
	if (hs_mgr_notifier.remote_adc)
		hs_mgr_notifier.remote_adc(&adc);

	mutex_lock(&hi->mutex_lock);

	hi->one_wire_mode = 0;
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
	if (hi->driver_one_wire_exist && adc > 915 && adc < hi->pdata.headset_config[0].adc_max) {
#else
	if (hi->driver_one_wire_exist && adc > 915) {
#endif
		HS_LOG("[HS_1wire]1wire driver exists, starting init");
		if (hs_mgr_notifier.hs_1wire_init() == 0) {
			hi->one_wire_mode = 1;
		
			old_state = switch_get_state(&hi->sdev_h2w);
			new_state = BIT_HEADSET;
			if (old_state == BIT_HEADSET_NO_MIC) {
				HS_LOG("Send fake remove event");
				switch_set_state(&hi->sdev_h2w, old_state & ~MASK_35MM_HEADSET);
			}
			switch_set_state(&hi->sdev_h2w, new_state);
			hi->hs_35mm_type = HEADSET_ONEWIRE;
			mutex_unlock(&hi->mutex_lock);
		if (hs_mgr_notifier.key_int_enable)
			hs_mgr_notifier.key_int_enable(1);
			return;
		}
		else {
			hi->one_wire_mode = 0;
			HS_LOG("Lagacy mode");
			if (hi->pdata.uart_tx_gpo)
				hi->pdata.uart_tx_gpo(HS_TX_ALT);
		}
	}
	mic = get_mic_status();
	if (hi->pdata.driver_flag & DRIVER_HS_MGR_FLOAT_DET) {
		HS_LOG("Headset float detect enable");
		if (mic == HEADSET_UNPLUG) {
			mutex_unlock(&hi->mutex_lock);

#ifdef CONFIG_HTC_INSERT_NOTIFY_DELAY
			if (hs_mgr_notifier.hs_insert)
				hs_mgr_notifier.hs_insert(0);
#endif

#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
			if (hs_mgr_notifier.key_int_enable)
				hs_mgr_notifier.key_int_enable(1);
#else
			
			set_35mm_hw_state(0);
#endif
			return;
		}
	}

	if (mic == HEADSET_NO_MIC)
		mic = tv_out_detect();

	if (mic == HEADSET_TV_OUT && hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 1);

	if (mic == HEADSET_METRICO && !hi->metrico_status)
		enable_metrico_headset(1);

	old_state = switch_get_state(&hi->sdev_h2w);
	new_state = old_state & ~MASK_35MM_HEADSET;

	switch (mic) {

	case HEADSET_NO_MIC:
		new_state |= BIT_HEADSET_NO_MIC;
		HS_LOG_TIME("HEADSET_NO_MIC");
#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
		set_35mm_hw_state(0);
#endif
		break;
	case HEADSET_MIC:
		new_state |= BIT_HEADSET;
		HS_LOG_TIME("HEADSET_MIC");
		break;
	case HEADSET_METRICO:
		mic = HEADSET_UNSTABLE;
		HS_LOG_TIME("HEADSET_METRICO (UNSTABLE)");
		break;
	case HEADSET_UNKNOWN_MIC:
		new_state |= BIT_HEADSET_NO_MIC;
		HS_LOG_TIME("HEADSET_UNKNOWN_MIC");
		break;
	case HEADSET_TV_OUT:
		new_state |= BIT_TV_OUT;
		HS_LOG_TIME("HEADSET_TV_OUT");
#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
		tvout_enable_detection(1);
#endif
		break;
	case HEADSET_INDICATOR:
		HS_LOG_TIME("HEADSET_INDICATOR");
		break;
	case HEADSET_UART:
		HS_LOG_TIME("HEADSET_UART");
		if (hs_mgr_notifier.uart_set)
			hs_mgr_notifier.uart_set(1);
		break;
	}
	if ((old_state == BIT_HEADSET_NO_MIC) && (new_state == BIT_HEADSET)) {
		HS_LOG("no_mic to mic workaround");
		new_state = BIT_HEADSET_NO_MIC | BIT_HEADSET;
		hi->hpin_jiffies = jiffies;
	}
	hi->hs_35mm_type = mic;
	HS_LOG_TIME("Send uevent for state change, %d => %d", old_state, new_state);
	switch_set_state(&hi->sdev_h2w, new_state);
	hpin_report++;

#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
	if (mic != HEADSET_NO_MIC) {
#endif
		if (hs_mgr_notifier.key_int_enable)
			hs_mgr_notifier.key_int_enable(1);
#ifndef CONFIG_HTC_HEADSET_INT_REDETECT
	}
#endif
	mutex_unlock(&hi->mutex_lock);

#ifdef HTC_HEADSET_CONFIG_QUICK_BOOT
	if (gpio_event_get_quickboot_status())
		HS_LOG("quick_boot_status = 1");
#endif

	if (mic == HEADSET_UNKNOWN_MIC)
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
	else if (mic == HEADSET_UNSTABLE)
		update_mic_status(0);
	else if (mic == HEADSET_INDICATOR) {
		if (headset_get_type_sync(3, HS_DELAY_SEC) == HEADSET_INDICATOR)
			HS_LOG("Delay check: HEADSET_INDICATOR");
		else
			HS_LOG("Delay check: HEADSET_UNKNOWN_MIC");
	}
}

int hs_notify_plug_event(int insert, unsigned int intr_id)
{
	int ret = 0;
	HS_LOG("Headset status++%d++ %d", intr_id,insert);

	mutex_lock(&hi->mutex_lock);
	hi->is_ext_insert = insert;
	mutex_unlock(&hi->mutex_lock);

#ifndef CONFIG_HTC_INSERT_NOTIFY_DELAY
	if (hs_mgr_notifier.hs_insert)
		hs_mgr_notifier.hs_insert(insert);
#endif

	cancel_delayed_work_sync(&mic_detect_work);
	ret = cancel_delayed_work_sync(&insert_detect_work);
	if (ret && hs_mgr_notifier.key_int_enable) {
		HS_LOG("Cancel insert work success");
		if (!insert)
		hs_mgr_notifier.key_int_enable(1);
	}
	ret = cancel_delayed_work_sync(&remove_detect_work);
	if (ret && hs_mgr_notifier.key_int_enable) {
		HS_LOG("Cancel remove work success");
		if (insert)
		hs_mgr_notifier.key_int_enable(0);
	}
	if (hi->is_ext_insert) {
		ret = queue_delayed_work(detect_wq, &insert_detect_work,
				   HS_JIFFIES_INSERT);
		HS_LOG("queue insert work, ret = %d", ret);
	}
	else {
		if (hi->pdata.driver_flag & DRIVER_HS_MGR_OLD_AJ) {
			HS_LOG("Old AJ work long remove delay");
			ret = queue_delayed_work(detect_wq, &remove_detect_work,
					   HS_JIFFIES_REMOVE_LONG);
		} else {
			ret = queue_delayed_work(detect_wq, &remove_detect_work,
					   HS_JIFFIES_REMOVE);
		}
		HS_LOG("queue remove work, ret = %d", ret);
	}

	HS_LOG("Headset status--%d-- %d", intr_id,insert);
	return 1;
}

int hs_notify_key_event(int key_code)
{
	struct button_work *work;

	HS_DBG();

	if (hi->hs_35mm_type == HEADSET_INDICATOR) {
		HS_LOG("Not support remote control");
		return 1;
	}

	if (hi->hs_35mm_type == HEADSET_UNKNOWN_MIC ||
	    hi->hs_35mm_type == HEADSET_NO_MIC ||
	    hi->h2w_35mm_type == HEADSET_NO_MIC)
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		update_mic_status(0);
#else
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
#endif
	else if (hi->hs_35mm_type == HEADSET_UNSTABLE)
		update_mic_status(0);
	else if (!hs_hpin_stable()) {
		HS_LOG("IGNORE key %d (Unstable HPIN)", key_code);
		return 1;
	} else if (hi->hs_35mm_type == HEADSET_UNPLUG && hi->is_ext_insert == 1) {
		HS_LOG("MIC status is changed from float, re-polling to decide accessory type");
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		update_mic_status(0);
#else
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
#endif
		return 1;
	} else {
		work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
		if (!work) {
			HS_ERR("Failed to allocate button memory");
			return 1;
		}
		work->key_code = key_code;
		INIT_DELAYED_WORK(&work->key_work, button_35mm_work_func);
		queue_delayed_work(button_wq, &work->key_work,
				   HS_JIFFIES_BUTTON);
	}

	return 1;
}

static void proc_comb_keys(void)
{
	int j, k;
	if (hi->key_code_1wire_index >= 5) {
		for (j = 0; j <= hi->key_code_1wire_index - 5; j++) {
			if (hi->key_code_1wire[j] == 1 && hi->key_code_1wire[j+2] == 1 && hi->key_code_1wire[j+4] == 1) {
				hi->key_code_1wire[j] = HS_MGR_3X_KEY_MEDIA;
				HS_LOG("key[%d] = %d", j, HS_MGR_3X_KEY_MEDIA);
				for (k = j + 1; k < (hi->key_code_1wire_index - 4); k++) {
					hi->key_code_1wire[k] = hi->key_code_1wire[k+4];
					HS_LOG("key[%d] <= key[%d]", k, k+4);
				}
				hi->key_code_1wire_index -= 4;
			}
		}
	}

	if (hi->key_code_1wire_index >= 3) {
		for (j = 0; j <= hi->key_code_1wire_index - 3; j++) {
			if (hi->key_code_1wire[j] == 1 && hi->key_code_1wire[j+2] == 1) {
				hi->key_code_1wire[j] = HS_MGR_2X_KEY_MEDIA;
				HS_LOG("key[%d] = %d", j, HS_MGR_2X_KEY_MEDIA);
				for (k = j + 1; k < (hi->key_code_1wire_index - 2); k++) {
					hi->key_code_1wire[k] = hi->key_code_1wire[k+2];
					HS_LOG("key[%d] <= key[%d]", k, k+2);
				}
				hi->key_code_1wire_index -= 2;
			}
		}
	}
}

static void proc_long_press(void)
{
	if (hi->key_code_1wire[hi->key_code_1wire_index - 1] == HS_MGR_2X_KEY_MEDIA) {	
		HS_LOG("long press key found, replace key[%d] = %d ==> %d", hi->key_code_1wire_index - 1,
			hi->key_code_1wire[hi->key_code_1wire_index - 1], HS_MGR_2X_HOLD_MEDIA);
		hi->key_code_1wire[hi->key_code_1wire_index - 1] = HS_MGR_2X_HOLD_MEDIA;
	}

	if (hi->key_code_1wire[hi->key_code_1wire_index - 1] == HS_MGR_3X_KEY_MEDIA) {	
		HS_LOG("long press key found, replace key[%d] = %d ==> %d", hi->key_code_1wire_index - 1,
			hi->key_code_1wire[hi->key_code_1wire_index - 1], HS_MGR_3X_HOLD_MEDIA);
		hi->key_code_1wire[hi->key_code_1wire_index - 1] = HS_MGR_3X_HOLD_MEDIA;
	}
}


static void button_1wire_work_func(struct work_struct *work)
{
	int i;
	static int pre_key = 0;
	if (hi->key_code_1wire_index >= 15)
		HS_LOG("key_code_1wire buffer overflow");
	proc_comb_keys();
	proc_long_press();
	for (i = 0; i < hi->key_code_1wire_index; i++) {
		HS_LOG("1wire key [%d] = %d", i, hi->key_code_1wire[i]);
		switch (hi->key_code_1wire[i]) {
			case	1:
					
					button_pressed(HS_MGR_KEYCODE_MEDIA);
					pre_key = HS_MGR_KEYCODE_MEDIA;
					break;
			case	2:
					button_pressed(HS_MGR_KEYCODE_VOLUP);
					pre_key = HS_MGR_KEYCODE_VOLUP;
					break;
			case	3:
					button_pressed(HS_MGR_KEYCODE_VOLDOWN);
					pre_key = HS_MGR_KEYCODE_VOLDOWN;
					break;
			case	HS_MGR_2X_KEY_MEDIA:
					button_pressed(HS_MGR_KEYCODE_FORWARD);
					pre_key = HS_MGR_KEYCODE_FORWARD;
					
					break;
			case	HS_MGR_3X_KEY_MEDIA:
					button_pressed(HS_MGR_KEYCODE_BACKWARD);
					pre_key = HS_MGR_KEYCODE_BACKWARD;
					
					break;
			case	HS_MGR_2X_HOLD_MEDIA:
					button_pressed(HS_MGR_KEYCODE_FF);
					pre_key = HS_MGR_KEYCODE_FF;
					
					break;
			case	HS_MGR_3X_HOLD_MEDIA:
					button_pressed(HS_MGR_KEYCODE_RW);
					pre_key = HS_MGR_KEYCODE_RW;
					
					break;
			case	0:
					button_released(pre_key);
					break;
			default:
					break;
		}
		msleep(10);
	}
	hi->key_code_1wire_index = 0;

}


int hs_notify_key_irq(void)
{
	int adc = 0;
	int key_code = HS_MGR_KEY_INVALID;
	static int pre_key = 0;

	if (hi->one_wire_mode == 1 && hs_hpin_stable() && hi->is_ext_insert) {
		wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
		key_code = hs_mgr_notifier.hs_1wire_read_key();
		if (key_code < 0) {
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
			hs_mgr_notifier.remote_adc(&adc);
			if (adc > hi->pdata.headset_config[0].adc_max) {
				hi->plugout_redetect = 1;
				update_mic_status(0);
			}
#endif
			wake_unlock(&hi->hs_wake_lock);
			return 1;
		}
		if (key_code == 2 || key_code == 3 || pre_key == 2 || pre_key == 3) {
			queue_delayed_work(button_wq, &button_1wire_work, HS_JIFFIES_1WIRE_BUTTON_SHORT);
			HS_LOG("Use short delay");
		} else {
			queue_delayed_work(button_wq, &button_1wire_work, hi->onewire_key_delay);
			HS_LOG("Use long delay");
		}

		HS_LOG("key_code = 0x%x", key_code);
		hi->key_code_1wire[hi->key_code_1wire_index++] = key_code;
		pre_key = key_code;
		return 1;
	}

	key_bounce++;
	if (hi->hs_35mm_type == HEADSET_INDICATOR) {
		HS_LOG("Not support remote control");
		return 1;
	}

	if (!hs_mgr_notifier.remote_adc || !hs_mgr_notifier.remote_keycode) {
		HS_LOG("Failed to get remote key code");
		return 1;
	}

	
	if ((hi->hs_35mm_type == HEADSET_NO_MIC || hi->hs_35mm_type == HEADSET_UNKNOWN_MIC) &&
		time_before_eq(jiffies, hi->hpin_jiffies + 10 * HZ)) {
		HS_LOG("IGNORE key IRQ (Unstable HPIN)");
		
#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
		update_mic_status(0);
#else
		update_mic_status(HS_DEF_MIC_DETECT_COUNT);
#endif
	} else if (hs_hpin_stable()) {
#ifndef CONFIG_HTC_HEADSET_ONE_WIRE
		msleep(50);
#endif
		hs_mgr_notifier.remote_adc(&adc);
		key_code = hs_mgr_notifier.remote_keycode(adc);
		hs_notify_key_event(key_code);
	}

	return 1;
}

static void usb_headset_detect(int type)
{
	int state_h2w = 0;
	int state_usb = 0;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state_h2w = switch_get_state(&hi->sdev_h2w);

	switch (type) {
	case USB_NO_HEADSET:
		hi->usb_headset.type = USB_NO_HEADSET;
		hi->usb_headset.status = STATUS_DISCONNECTED;
		state_h2w &= ~MASK_USB_HEADSET;
		state_usb = GOOGLE_USB_AUDIO_UNPLUG;
		HS_LOG_TIME("Remove USB_HEADSET (state %d, %d)",
			    state_h2w, state_usb);
		break;
	case USB_AUDIO_OUT:
		hi->usb_headset.type = USB_AUDIO_OUT;
		hi->usb_headset.status = STATUS_CONNECTED_ENABLED;
		state_h2w |= BIT_USB_AUDIO_OUT;
		state_usb = GOOGLE_USB_AUDIO_ANLG;
		HS_LOG_TIME("Insert USB_AUDIO_OUT (state %d, %d)",
			    state_h2w, state_usb);
		break;
#ifdef CONFIG_SUPPORT_USB_SPEAKER
	case USB_AUDIO_OUT_DGTL:
		hi->usb_headset.type = USB_AUDIO_OUT;
                hi->usb_headset.status = STATUS_CONNECTED_ENABLED;
                state_h2w |= BIT_USB_AUDIO_OUT;
                state_usb = GOOGLE_USB_AUDIO_DGTL;
                HS_LOG_TIME("Insert USB_AUDIO_OUT DGTL (state %d, %d)",
                            state_h2w, state_usb);
                break;
#endif
	default:
		HS_LOG("Unknown headset type");
	}

	switch_set_state(&hi->sdev_h2w, state_h2w);
	switch_set_state(&hi->sdev_usb_audio, state_usb);
	mutex_unlock(&hi->mutex_lock);
}

void headset_ext_detect(int type)
{
	HS_DBG();

	switch (type) {
	case H2W_NO_HEADSET:
		
	case H2W_HEADSET:
	case H2W_35MM_HEADSET:
	case H2W_REMOTE_CONTROL:
	case H2W_USB_CRADLE:
	case H2W_UART_DEBUG:
	case H2W_TVOUT:
		break;
	case USB_NO_HEADSET:
		
	case USB_AUDIO_OUT:
#ifdef CONFIG_SUPPORT_USB_SPEAKER
	case USB_AUDIO_OUT_DGTL:
#endif
		usb_headset_detect(type);
		break;
	default:
		HS_LOG("Unknown headset type");
	}
}

void headset_ext_button(int headset_type, int key_code, int press)
{
	HS_LOG("Headset %d, Key %d, Press %d", headset_type, key_code, press);
	headset_button_event(press, key_code);
}

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev_h2w);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev_h2w, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static ssize_t headset_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int length = 0;
	char *state = NULL;

	HS_DBG();

	switch (hi->hs_35mm_type) {
	case HEADSET_UNPLUG:
		state = "headset_unplug";
		break;
	case HEADSET_NO_MIC:
		state = "headset_no_mic";
		break;
	case HEADSET_MIC:
		state = "headset_mic";
		break;
	case HEADSET_METRICO:
		state = "headset_metrico";
		break;
	case HEADSET_UNKNOWN_MIC:
		state = "headset_unknown_mic";
		break;
	case HEADSET_TV_OUT:
		state = "headset_tv_out";
		break;
	case HEADSET_UNSTABLE:
		state = "headset_unstable";
		break;
	case HEADSET_ONEWIRE:
		if (hi->one_wire_mode == 1 && hs_mgr_notifier.hs_1wire_report_type)
			hs_mgr_notifier.hs_1wire_report_type(&state);
		else
		state = "headset_mic_1wire";
		break;
	case HEADSET_INDICATOR:
		state = "headset_indicator";
		break;
	case HEADSET_UART:
		state = "headset_uart";
		break;
	default:
		state = "error_state";
	}

	length = snprintf(buf, PAGE_SIZE, "%s\n", state);

	return length;
}

static ssize_t headset_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	HS_DBG();
	return 0;
}

static DEVICE_HEADSET_ATTR(state, 0644, headset_state_show,
			   headset_state_store);

static ssize_t headset_1wire_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", hi->one_wire_mode);
}

static ssize_t headset_1wire_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	HS_DBG();
	return 0;
}

static DEVICE_HEADSET_ATTR(1wire_state, 0644, headset_1wire_state_show,
			   headset_1wire_state_store);

static ssize_t headset_simulate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	HS_DBG();
	return snprintf(buf, PAGE_SIZE, "Command is not supported\n");
}

static ssize_t headset_simulate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long state = 0;

	HS_DBG();

	state = MASK_35MM_HEADSET | MASK_USB_HEADSET;
	switch_send_event(state, 0);

	if (strncmp(buf, "headset_unplug", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_unplug");
		set_35mm_hw_state(0);
		hi->hs_35mm_type = HEADSET_UNPLUG;
		return count;
	}

	set_35mm_hw_state(1);
	state = BIT_35MM_HEADSET;

	if (strncmp(buf, "headset_no_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_no_mic");
		hi->hs_35mm_type = HEADSET_NO_MIC;
		state = BIT_HEADSET_NO_MIC;
	} else if (strncmp(buf, "headset_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_mic");
		hi->hs_35mm_type = HEADSET_MIC;
		state = BIT_HEADSET;
	} else if (strncmp(buf, "headset_metrico", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_metrico");
		hi->hs_35mm_type = HEADSET_METRICO;
		state = BIT_HEADSET;
	} else if (strncmp(buf, "headset_unknown_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_unknown_mic");
		hi->hs_35mm_type = HEADSET_UNKNOWN_MIC;
		state = BIT_HEADSET_NO_MIC;
	} else if (strncmp(buf, "headset_tv_out", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_tv_out");
		hi->hs_35mm_type = HEADSET_TV_OUT;
		state = BIT_TV_OUT;
#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
		tvout_enable_detection(1);
#endif
	} else if (strncmp(buf, "headset_indicator", count - 1) == 0) {
		HS_LOG("Headset simulation: headset_indicator");
		hi->hs_35mm_type = HEADSET_INDICATOR;
	} else {
		HS_LOG("Invalid parameter");
		return count;
	}

	switch_send_event(state, 1);

	return count;
}

static DEVICE_HEADSET_ATTR(simulate, 0644, headset_simulate_show,
			   headset_simulate_store);

static ssize_t headset_1wire_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	HS_DBG();
	s += snprintf(s, PAGE_SIZE, "onewire key delay is %dms\n", jiffies_to_msecs(hi->onewire_key_delay));
	return (s - buf);
}

static ssize_t headset_1wire_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long ten_base = 1;
	int i;
	unsigned long delay_t = 0;
	HS_DBG();

		for(i = (count - 2); i >= 0; i--) {
			HS_LOG("buf[%d] = %d, ten_base = %ld", i, *(buf + i) - 48, ten_base);
			delay_t += (*(buf + i) - 48) * ten_base;
			ten_base *= 10;
		}
	HS_LOG("delay_t = %ld", delay_t);
	hi->onewire_key_delay = msecs_to_jiffies(delay_t);

	return count;
}

static DEVICE_HEADSET_ATTR(onewire, 0644, headset_1wire_show,
			   headset_1wire_store);

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	s += snprintf(s, PAGE_SIZE, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev_h2w);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		hi->tty_enable_flag = 1;
		switch_set_state(&hi->sdev_h2w, state | BIT_TTY_FULL);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY FULL");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		hi->tty_enable_flag = 2;
		switch_set_state(&hi->sdev_h2w, state | BIT_TTY_VCO);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY VCO");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		hi->tty_enable_flag = 3;
		switch_set_state(&hi->sdev_h2w, state | BIT_TTY_HCO);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY HCO");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->tty_enable_flag = 0;
		switch_set_state(&hi->sdev_h2w, state);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Disable TTY");
		return count;
	}

	mutex_unlock(&hi->mutex_lock);
	HS_LOG("Invalid TTY argument");

	return -EINVAL;
}

static DEVICE_ACCESSORY_ATTR(tty, 0644, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	switch (hi->fm_flag) {
	case 0:
		state = "disable";
		break;
	case 1:
		state = "fm_headset";
		break;
	case 2:
		state = "fm_speaker";
		break;
	default:
		state = "unknown_fm_status";
	}

	s += snprintf(s, PAGE_SIZE, "%s\n", state);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev_h2w);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		HS_LOG("Enable FM HEADSET");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		HS_LOG("Enable FM SPEAKER");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		HS_LOG("Disable FM");
	} else {
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Invalid FM argument");
		return -EINVAL;
	}

	switch_set_state(&hi->sdev_h2w, state);
	mutex_unlock(&hi->mutex_lock);

	return count;
}

static DEVICE_ACCESSORY_ATTR(fm, 0644, fm_flag_show, fm_flag_store);

static ssize_t debug_flag_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int flag = hi->debug_flag;
	int adc = -EINVAL;
	int hpin_gpio = -EINVAL;
	int len, i;
	char *s;

	HS_DBG();

	if (hs_mgr_notifier.hpin_gpio)
		hpin_gpio = hs_mgr_notifier.hpin_gpio();
	if (hs_mgr_notifier.remote_adc)
		hs_mgr_notifier.remote_adc(&adc);

	s = buf;
	len =  snprintf(buf, PAGE_SIZE, "Debug Flag = %d\nHP_DET = %d\nADC = %d\n", flag,
		       hpin_gpio, adc);
	buf += len;
	len =  snprintf(buf, PAGE_SIZE, "DET report count = %d\nDET bounce count = %d\n", hpin_report, hpin_bounce);
	buf += len;
	len =  snprintf(buf, PAGE_SIZE, "KEY report count = %d\nKEY bounce count = %d\n", key_report, key_bounce);
	buf += len;
	for (i = 0; i < hi->pdata.headset_config_num; i++) {
		switch (hi->pdata.headset_config[i].type) {
			case HEADSET_NO_MIC:
				len = snprintf(buf, PAGE_SIZE, "headset_no_mic_adc_max = %d\n", hi->pdata.headset_config[i].adc_max);
				buf += len;
				len = snprintf(buf, PAGE_SIZE, "headset_no_mic_adc_min = %d\n", hi->pdata.headset_config[i].adc_min);
				buf += len;
				break;
			case HEADSET_MIC:
				len = snprintf(buf, PAGE_SIZE, "headset_mic_adc_max = %d\n", hi->pdata.headset_config[i].adc_max);
				buf += len;
				len = snprintf(buf, PAGE_SIZE, "headset_mic_adc_min = %d\n", hi->pdata.headset_config[i].adc_min);
				buf += len;
				break;
			default:
				break;
		}
	}
	key_report = 0;
	key_bounce = 0;
	hpin_report = 0;
	hpin_bounce = 0;
	return (buf - s);
}

static ssize_t debug_flag_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long state = 0;

	HS_DBG();

	if (strncmp(buf, "enable", count - 1) == 0) {
		if (hi->debug_flag & DEBUG_FLAG_ADC) {
			HS_LOG("Debug work is already running");
			return count;
		}
		if (!debug_wq) {
			debug_wq = create_workqueue("debug");
			if (!debug_wq) {
				HS_LOG("Failed to create debug workqueue");
				return count;
			}
		}
		HS_LOG("Enable headset debug");
		mutex_lock(&hi->mutex_lock);
		hi->debug_flag |= DEBUG_FLAG_ADC;
		mutex_unlock(&hi->mutex_lock);
		queue_work(debug_wq, &debug_work);
	} else if (strncmp(buf, "disable", count - 1) == 0) {
		if (!(hi->debug_flag & DEBUG_FLAG_ADC)) {
			HS_LOG("Debug work has been stopped");
			return count;
		}
		HS_LOG("Disable headset debug");
		mutex_lock(&hi->mutex_lock);
		hi->debug_flag &= ~DEBUG_FLAG_ADC;
		mutex_unlock(&hi->mutex_lock);
		if (debug_wq) {
			flush_workqueue(debug_wq);
			destroy_workqueue(debug_wq);
			debug_wq = NULL;
		}
	} else if (strncmp(buf, "debug_log_enable", count - 1) == 0) {
		HS_LOG("Enable headset debug log");
		hi->debug_flag |= DEBUG_FLAG_LOG;
	} else if (strncmp(buf, "debug_log_disable", count - 1) == 0) {
		HS_LOG("Disable headset debug log");
		hi->debug_flag &= ~DEBUG_FLAG_LOG;
	} else if (strncmp(buf, "no_headset", count - 1) == 0) {
		HS_LOG("Headset simulation: no_headset");
		state = BIT_HEADSET | BIT_HEADSET_NO_MIC | BIT_35MM_HEADSET |
			BIT_TV_OUT | BIT_USB_AUDIO_OUT;
		switch_send_event(state, 0);
	} else if (strncmp(buf, "35mm_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_mic");
		state = BIT_HEADSET | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "35mm_no_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_no_mic");
		state = BIT_HEADSET_NO_MIC | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "35mm_tv_out", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_tv_out");
		state = BIT_TV_OUT | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "usb_audio", count - 1) == 0) {
		HS_LOG("Headset simulation: usb_audio");
		state = BIT_USB_AUDIO_OUT;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "1wire_init", count - 1) == 0) {
		hs_mgr_notifier.hs_1wire_init();
	} else if (strncmp(buf, "init_gpio", count - 1) == 0) {
		hi->pdata.uart_tx_gpo(HS_RX_ALT);
		hi->pdata.uart_lv_shift_en(0);
		hi->pdata.uart_tx_gpo(HS_TX_ALT);
	} else {
		HS_LOG("Invalid parameter");
		return count;
	}

	return count;
}

static DEVICE_ACCESSORY_ATTR(debug, 0644, debug_flag_show, debug_flag_store);

static int register_attributes(void)
{
	int ret = 0;

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	
	hi->headset_dev = device_create(hi->htc_accessory_class,
					NULL, 0, "%s", "headset");
	if (unlikely(IS_ERR(hi->headset_dev))) {
		ret = PTR_ERR(hi->headset_dev);
		hi->headset_dev = NULL;
		goto err_create_headset_device;
	}

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_state);
	if (ret)
		goto err_create_headset_state_device_file;

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_simulate);
	if (ret)
		goto err_create_headset_simulate_device_file;

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_1wire_state);
	if (ret)
		goto err_create_headset_state_device_file;

	
	hi->tty_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	
	hi->fm_dev = device_create(hi->htc_accessory_class,
				   NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	
	hi->debug_dev = device_create(hi->htc_accessory_class,
				      NULL, 0, "%s", "debug");
	if (unlikely(IS_ERR(hi->debug_dev))) {
		ret = PTR_ERR(hi->debug_dev);
		hi->debug_dev = NULL;
		goto err_create_debug_device;
	}

	
	ret = device_create_file(hi->debug_dev, &dev_attr_debug);
	if (ret)
		goto err_create_debug_device_file;

	ret = device_create_file(hi->debug_dev, &dev_attr_headset_onewire);
	if (ret)
		goto err_create_debug_device_file;

	return 0;

err_create_debug_device_file:
	device_unregister(hi->debug_dev);

err_create_debug_device:
	device_remove_file(hi->fm_dev, &dev_attr_fm);

err_create_fm_device_file:
	device_unregister(hi->fm_dev);

err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);

err_create_tty_device_file:
	device_unregister(hi->tty_dev);

err_create_tty_device:
	device_remove_file(hi->headset_dev, &dev_attr_headset_simulate);

err_create_headset_simulate_device_file:
	device_remove_file(hi->headset_dev, &dev_attr_headset_state);

err_create_headset_state_device_file:
	device_unregister(hi->headset_dev);

err_create_headset_device:
	class_destroy(hi->htc_accessory_class);

err_create_class:

	return ret;
}

static void unregister_attributes(void)
{
	device_remove_file(hi->debug_dev, &dev_attr_debug);
	device_unregister(hi->debug_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	device_remove_file(hi->headset_dev, &dev_attr_headset_simulate);
	device_remove_file(hi->headset_dev, &dev_attr_headset_state);
	device_unregister(hi->headset_dev);
	class_destroy(hi->htc_accessory_class);
}

static void headset_mgr_init(void)
{
	if (hi->pdata.hptv_det_hp_gpio)
		gpio_set_value(hi->pdata.hptv_det_hp_gpio, 1);
	if (hi->pdata.hptv_det_tv_gpio)
		gpio_set_value(hi->pdata.hptv_det_tv_gpio, 0);
	if (hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 0);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void htc_headset_mgr_early_suspend(struct early_suspend *h)
{
	HS_DBG();
}

static void htc_headset_mgr_late_resume(struct early_suspend *h)
{
#ifdef HTC_HEADSET_CONFIG_QUICK_BOOT
	int state = 0;
	HS_DBG();

	if (hi->quick_boot_status) {
		mutex_lock(&hi->mutex_lock);
		state = switch_get_state(&hi->sdev_h2w);
		HS_LOG_TIME("Resend quick boot U-Event (state = %d)",
			    state | BIT_UNDEFINED);
		switch_set_state(&hi->sdev_h2w, state | BIT_UNDEFINED);
		HS_LOG_TIME("Resend quick boot U-Event (state = %d)", state);
		switch_set_state(&hi->sdev_h2w, state);
		hi->quick_boot_status = 0;
		mutex_unlock(&hi->mutex_lock);
	}
#else
	HS_DBG();
#endif
}
#endif

static int htc_headset_mgr_suspend(struct platform_device *pdev,
				   pm_message_t mesg)
{
	HS_DBG();

#ifdef HTC_HEADSET_CONFIG_QUICK_BOOT
	if (gpio_event_get_quickboot_status())
		hi->quick_boot_status = 1;
#endif

	return 0;
}

static int htc_headset_mgr_resume(struct platform_device *pdev)
{
	HS_DBG();
	if (hi->one_wire_mode == 1)
		hs_mgr_notifier.hs_1wire_open();
	return 0;
}

#ifdef CONFIG_OF
static void headset_power(int enable);
static void headset_init(void);
static void uart_tx_gpo(int mode);
static void uart_lv_shift_en(int enable);

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= DRIVER_HS_MGR_FLOAT_DET,
	
	
        .headset_init		= headset_init,
	.headset_power		= headset_power,
	.uart_tx_gpo		= uart_tx_gpo,
	.uart_lv_shift_en	= uart_lv_shift_en,
};

static void headset_power(int enable)
{
	HS_LOG("%s_set MIC bias:%s", __func__, enable ? "ON" : "OFF");
	gpio_set_value(htc_headset_mgr_data.bias_gpio, enable);
}

static void headset_init(void)
{
	int ret=0;
	uint8_t i;
	uint32_t hs_onewire_gpio[] = {
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	};
	uint32_t hs_onewire_gpio_rx_dn[] = {
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	};
	HS_LOG("%s-Init gpio[%d %d] alt:%d", __func__, htc_headset_mgr_data.tx_gpio,
		htc_headset_mgr_data.rx_gpio, htc_headset_mgr_data.wire_alt);
	for (i = 0; i < ARRAY_SIZE(hs_onewire_gpio); i++)
		{
			if(htc_headset_mgr_data.rx_gpio_pulldn)
				ret = gpio_tlmm_config(hs_onewire_gpio_rx_dn[i], GPIO_CFG_ENABLE);
			else
				ret = gpio_tlmm_config(hs_onewire_gpio[i], GPIO_CFG_ENABLE);
			if (ret < 0)
				HS_LOG("%s config error gpio_idx(%d)",__func__, i);
		}
}


static void uart_tx_gpo(int mode)
{
	uint32_t headset_1wire_gpio[] = {
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, htc_headset_mgr_data.wire_alt,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, htc_headset_mgr_data.wire_alt,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	};

	uint32_t headset_1wire_gpio_rx_dn[] = {
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.rx_gpio, htc_headset_mgr_data.wire_alt,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG(htc_headset_mgr_data.tx_gpio, htc_headset_mgr_data.wire_alt,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	};
	HS_LOG("(%s) %d\n", __func__, mode);
	if(htc_headset_mgr_data.rx_gpio_pulldn)
		gpio_tlmm_config(headset_1wire_gpio_rx_dn[mode], GPIO_CFG_ENABLE);
	else
		gpio_tlmm_config(headset_1wire_gpio[mode], GPIO_CFG_ENABLE);

	if(mode == 1)
		gpio_set_value(htc_headset_mgr_data.tx_gpio, 0);
}

static void uart_lv_shift_en(int enable)
{
	int OEz = 0;
	gpio_set_value(htc_headset_mgr_data.oe_gpio, enable);
	OEz = gpio_get_value(htc_headset_mgr_data.oe_gpio);
	if(OEz != enable)
		HS_LOG("(%s) level shfit setting fail(%d_\n", __func__, OEz);
}

static int headset_mgr_parser_dt(struct htc_hs_mgr_data *mgr)
{
	uint8_t i,j;
	struct headset_adc_config *hs_typelist;
	u32 *hs_list;
	int ret = 0;
	const char *paser_alt   = {"mgr,wire_alt"},
		   *parser_st[] = {"mgr,tx_gpio", "mgr,rx_gpio",
			   "mgr,oe_gpio", "mgr,bias_gpio"},
		   *parser_type[] = {"mgr,hs_type","mgr,adc_max","mgr,adc_min"};
	int gpio[ARRAY_SIZE(parser_st)] = {0};
	struct device_node *dt = mgr->dev->of_node;

	HS_LOG("%s", __func__);

	for (i = 0; i < ARRAY_SIZE(parser_st); i++) {
		gpio[i] = of_get_named_gpio(dt, parser_st[i], 0);
		if (!gpio_is_valid(gpio[i])) {
			HS_LOG("DT_mgr %sparser fail, ret = %d",parser_st[i], gpio[i]);
			goto parser_fail;
		}
	}
	if (i == ARRAY_SIZE(parser_st)) {
		htc_headset_mgr_data.tx_gpio   = gpio[0];
		htc_headset_mgr_data.rx_gpio   = gpio[1];
		htc_headset_mgr_data.oe_gpio   = gpio[2];
		htc_headset_mgr_data.bias_gpio = gpio[3];
	}

	ret = of_property_read_u32(dt, paser_alt, (u32*)&htc_headset_mgr_data.wire_alt);
	if (ret < 0) {		
		HS_LOG("1wire_alt parser err, ret=%d", ret);
		goto parser_fail;
	}

	HS_LOG("DT:tx[%d],rx[%d],alt[%d],bias[%d],oe[%d]",gpio[0], gpio[1],
		htc_headset_mgr_data.wire_alt, gpio[3], gpio[2]);

	ret = of_property_read_u32(dt, "mgr,rx_gpio_pulldn", (u32*)&htc_headset_mgr_data.rx_gpio_pulldn);
	if (ret < 0) {
		HS_LOG("mgr,rx_gpio_pulldn parser none, ret=%d", ret);
		htc_headset_mgr_data.rx_gpio_pulldn = 0;
	}
    HS_LOG("DT:Headset rx_gpio_pulldn=%d",htc_headset_mgr_data.rx_gpio_pulldn);

	ret = of_property_read_u32(dt, "mgr,hs_typenum", (u32*)&htc_headset_mgr_data.headset_config_num);
	if (ret < 0) {
		HS_LOG("mgr,hs_typenum parser err, ret=%d", ret);
		goto parser_fail;
	}
    HS_LOG("DT:Headset type number = %d",htc_headset_mgr_data.headset_config_num);

	hs_list = kzalloc(htc_headset_mgr_data.headset_config_num * sizeof(u32) * 3, GFP_KERNEL);
	if (hs_list == NULL) {
		HS_LOG("headset list alloc memry fail");
		return -ENOMEM;
		}
	hs_typelist = kzalloc(htc_headset_mgr_data.headset_config_num * sizeof(struct headset_adc_config), GFP_KERNEL);
	if (hs_typelist == NULL) {
		kfree(hs_list);
		HS_LOG("headset type list alloc memry fail");
		return -ENOMEM;
	}

        htc_headset_mgr_data.headset_config = hs_typelist;
	for(i =0; i < 3; i++)
	{
		ret = of_property_read_u32_array(dt, parser_type[i], hs_list, htc_headset_mgr_data.headset_config_num);
		if (ret < 0) {
			HS_LOG("DT:ADC parser failure");
		} else {
			for(j = 0; j < htc_headset_mgr_data.headset_config_num; j++)
			{
				switch(i)
				{
					case 0:
						htc_headset_mgr_data.headset_config[j].type = *(int*)&hs_list[j];
						break;
					case 1:
						htc_headset_mgr_data.headset_config[j].adc_max = *(uint32_t*)&hs_list[j];
						break;
					case 2:
						htc_headset_mgr_data.headset_config[j].adc_min = *(uint32_t*)&hs_list[j];
						break;
				}
			}
		}
	}

	for(j = 0; j < htc_headset_mgr_data.headset_config_num; j++)
		HS_LOG("DT:Headset type = %d adc_max = %d adc_min = %d",htc_headset_mgr_data.headset_config[j].type,
				htc_headset_mgr_data.headset_config[j].adc_max, htc_headset_mgr_data.headset_config[j].adc_min);

	memcpy(mgr->pdata, &htc_headset_mgr_data, sizeof(htc_headset_mgr_data));

	return 0;

parser_fail:
	return ret;
}

static void hs_mgr_dtwq(struct htc_hs_mgr_data *dt)
{
	HS_LOG("WQ:%s", __func__);

	dt->parser = (headset_mgr_parser_dt(dt) < 0) ? false: true;
}
#endif

static int htc_headset_mgr_probe(struct platform_device *pdev)
{
	int ret;

	struct htc_hs_mgr_data *dtinfo;
	struct htc_headset_mgr_platform_data *pdata;

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_mgr_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	dtinfo = kzalloc(sizeof(*dtinfo), GFP_KERNEL);
	if (!dtinfo) {
		kfree(hi);
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			HS_LOG("platform_data alloc memry fail");
			kfree(hi);
			kfree(dtinfo);
			return -ENOMEM;
		}

		dtinfo->dev    = &pdev->dev;
		dtinfo->pdata = pdata;
		hs_mgr_dtwq(dtinfo);

		HS_LOG("==DT parser %s==", dtinfo->parser ? "OK": "FAILED");
	} else {
		pdata = pdev->dev.platform_data;
	}

	hi->pdata.driver_flag         = pdata->driver_flag;
#ifndef CONFIG_OF
	hi->pdata.headset_devices_num = pdata->headset_devices_num;
	hi->pdata.headset_devices     = pdata->headset_devices;
#endif
	hi->pdata.headset_config_num  = pdata->headset_config_num;
	hi->pdata.headset_config      = pdata->headset_config;

	hi->pdata.hptv_det_hp_gpio    = pdata->hptv_det_hp_gpio;
	hi->pdata.hptv_det_tv_gpio    = pdata->hptv_det_tv_gpio;
	hi->pdata.hptv_sel_gpio       = pdata->hptv_sel_gpio;

	hi->pdata.headset_init        = pdata->headset_init;
	hi->pdata.headset_power       = pdata->headset_power;
	hi->pdata.uart_lv_shift_en    = pdata->uart_lv_shift_en;
	hi->pdata.uart_tx_gpo         = pdata->uart_tx_gpo;

	if (hi->pdata.headset_init)
		hi->pdata.headset_init();

	hi->driver_init_seq       = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	hi->early_suspend.suspend = htc_headset_mgr_early_suspend;
	hi->early_suspend.resume  = htc_headset_mgr_late_resume;
	register_early_suspend(&hi->early_suspend);
#endif

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	hi->hpin_jiffies       = jiffies;
	hi->usb_headset.type   = USB_NO_HEADSET;
	hi->usb_headset.status = STATUS_DISCONNECTED;

	hi->hs_35mm_type          = HEADSET_UNPLUG;
	hi->h2w_35mm_type         = HEADSET_UNPLUG;
	hi->is_ext_insert         = 0;
	hi->mic_bias_state        = 0;
	hi->mic_detect_counter    = 0;
	hi->key_level_flag        = -1;
	hi->quick_boot_status     = 0;
	hi->driver_one_wire_exist = 0;
	atomic_set(&hi->btn_state, 0);

	hi->tty_enable_flag      = 0;
	hi->fm_flag              = 0;
	hi->debug_flag           = 0;
	hi->key_code_1wire_index = 0;
	hi->onewire_key_delay    = HS_JIFFIES_1WIRE_BUTTON;

	mutex_init(&hi->mutex_lock);

	hi->sdev_h2w.name       = "h2w";
	hi->sdev_h2w.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev_h2w);
	if (ret < 0)
		goto err_h2w_switch_dev_register;

	hi->sdev_usb_audio.name       = "usb_audio";
	hi->sdev_usb_audio.print_name = usb_audio_print_name;

	ret = switch_dev_register(&hi->sdev_usb_audio);
	if (ret < 0)
		goto err_usb_audio_switch_dev_register;

	detect_wq = create_workqueue("detect");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_END, hi->input->keybit);
	set_bit(KEY_MUTE, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);
	set_bit(KEY_NEXTSONG, hi->input->keybit);
	set_bit(KEY_PLAYPAUSE, hi->input->keybit);
	set_bit(KEY_PREVIOUSSONG, hi->input->keybit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_SEND, hi->input->keybit);
	set_bit(KEY_FASTFORWARD, hi->input->keybit);
	set_bit(KEY_REWIND, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
	goto err_register_input_dev;

	ret = register_attributes();
	if (ret)
		goto err_register_attributes;

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
	if (hi->pdata.driver_flag & DRIVER_HS_MGR_RPC_SERVER) {
		
		ret = msm_rpc_create_server(&hs_rpc_server);
		if (ret < 0) {
			HS_ERR("Failed to create RPC server");
			goto err_create_rpc_server;
		}
		HS_LOG("Create RPC server successfully");
	}
#else
	HS_DBG("NOT support RPC (%du, %du)", hs_rpc_server.prog,
	       hs_rpc_server.vers);
#endif

	headset_mgr_init();
#ifndef CONFIG_OF
	hs_notify_driver_ready(DRIVER_NAME);
#endif
	HS_LOG("--------------------");

	kfree(dtinfo);
	kfree(pdata);
	return 0;

#ifdef HTC_HEADSET_CONFIG_MSM_RPC
err_create_rpc_server:
#endif

err_register_attributes:
	input_unregister_device(hi->input);

err_register_input_dev:
	input_free_device(hi->input);

err_request_input_dev:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	switch_dev_unregister(&hi->sdev_usb_audio);

err_usb_audio_switch_dev_register:
	switch_dev_unregister(&hi->sdev_h2w);

err_h2w_switch_dev_register:
	mutex_destroy(&hi->mutex_lock);
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(dtinfo);
	kfree(pdata);
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_mgr_remove(struct platform_device *pdev)
{
#if 0
	if ((switch_get_state(&hi->sdev_h2w) & MASK_HEADSET) != 0)
		remove_headset();
#endif

	unregister_attributes();
	input_unregister_device(hi->input);
	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);
	switch_dev_unregister(&hi->sdev_usb_audio);
	switch_dev_unregister(&hi->sdev_h2w);
	mutex_destroy(&hi->mutex_lock);
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	return 0;
}

static const struct of_device_id htc_headset_mgr_mt[] = {
	{ .compatible = "htc_headset,mgr"},
	{ },
};
static struct platform_driver htc_headset_mgr_driver = {
	.probe		= htc_headset_mgr_probe,
	.remove		= htc_headset_mgr_remove,
	.suspend	= htc_headset_mgr_suspend,
	.resume		= htc_headset_mgr_resume,
	.driver		= {
		.name		= "HTC_HEADSET_MGR",
		.owner		= THIS_MODULE,
		.of_match_table = htc_headset_mgr_mt,
	},
};

static void __init htc_headset_mgr_init_async(void *unused, async_cookie_t cookie)
{
	platform_driver_register(&htc_headset_mgr_driver);
}


static int __init htc_headset_mgr_init(void)
{
	async_schedule(htc_headset_mgr_init_async, NULL);
	return 0;
}

static void __exit htc_headset_mgr_exit(void)
{
	platform_driver_unregister(&htc_headset_mgr_driver);
}
late_initcall(htc_headset_mgr_init);
module_exit(htc_headset_mgr_exit);

MODULE_DESCRIPTION("HTC headset manager driver");
MODULE_LICENSE("GPL");

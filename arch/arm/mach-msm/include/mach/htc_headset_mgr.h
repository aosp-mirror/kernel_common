/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_mgr.h
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

#ifndef HTC_HEADSET_MGR_H
#define HTC_HEADSET_MGR_H

#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

#include <mach/msm_rpcrouter.h>
#include <mach/htc_headset_config.h>

#ifdef HTC_HEADSET_KERNEL_3_0
#define set_irq_type(irq, type) irq_set_irq_type(irq, type)
#define set_irq_wake(irq, on) irq_set_irq_wake(irq, on)
#else
#define set_irq_type(irq, type) set_irq_type(irq, type)
#define set_irq_wake(irq, on) set_irq_wake(irq, on)
#endif

#define HS_ERR(fmt, arg...) \
	printk(KERN_INFO "[" DRIVER_NAME "_ERR] (%s) " fmt "\n", \
		__func__, ## arg)
#define HS_LOG(fmt, arg...) \
	printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt "\n", __func__, ## arg)
#define HS_LOG_TIME(fmt, arg...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt \
		" (%02d-%02d %02d:%02d:%02d.%03lu)\n", __func__, \
		## arg, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, \
		tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000); \
	} while (0)
#define HS_DBG(fmt, arg...) \
	if (hs_debug_log_state()) {  \
		printk(KERN_INFO "##### [" DRIVER_NAME "] (%s) " fmt "\n", \
		       __func__, ## arg); \
	}

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_##_name = \
	__ATTR(flag, _mode, _show, _store)

#define DEVICE_HEADSET_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_headset_##_name = \
	__ATTR(_name, _mode, _show, _store)

#define DRIVER_HS_MGR_RPC_SERVER	(1 << 0)
#define DRIVER_HS_MGR_FLOAT_DET		(1 << 1)
#define DRIVER_HS_MGR_OLD_AJ		(1 << 2)

#define DEBUG_FLAG_LOG		(1 << 0)
#define DEBUG_FLAG_ADC		(1 << 1)

#define BIT_HEADSET		(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)
#define BIT_TTY_FULL		(1 << 2)
#define BIT_FM_HEADSET		(1 << 3)
#define BIT_FM_SPEAKER		(1 << 4)
#define BIT_TTY_VCO		(1 << 5)
#define BIT_TTY_HCO		(1 << 6)
#define BIT_35MM_HEADSET	(1 << 7)
#define BIT_TV_OUT		(1 << 8)
#define BIT_USB_CRADLE		(1 << 9)
#define BIT_TV_OUT_AUDIO	(1 << 10)
#define BIT_HDMI_CABLE		(1 << 11)
#define BIT_HDMI_AUDIO		(1 << 12)
#define BIT_USB_AUDIO_OUT	(1 << 13)
#define BIT_UNDEFINED		(1 << 14)

#define MASK_HEADSET		(BIT_HEADSET | BIT_HEADSET_NO_MIC)
#define MASK_35MM_HEADSET	(BIT_HEADSET | BIT_HEADSET_NO_MIC | \
				BIT_35MM_HEADSET | BIT_TV_OUT)
#define MASK_FM_ATTRIBUTE	(BIT_FM_HEADSET | BIT_FM_SPEAKER)
#define MASK_USB_HEADSET	(BIT_USB_AUDIO_OUT)

#define GOOGLE_BIT_HEADSET		(1 << 0)
#define GOOGLE_BIT_HEADSET_NO_MIC	(1 << 1)
#define GOOGLE_BIT_USB_HEADSET_ANLG	(1 << 2)
#define GOOGLE_BIT_USB_HEADSET_DGTL	(1 << 3)
#define GOOGLE_BIT_HDMI_AUDIO		(1 << 4)

#define GOOGLE_SUPPORTED_HEADSETS	(GOOGLE_BIT_HEADSET | \
					GOOGLE_BIT_HEADSET_NO_MIC | \
					GOOGLE_BIT_USB_HEADSET_ANLG | \
					GOOGLE_BIT_USB_HEADSET_DGTL | \
					GOOGLE_BIT_HDMI_AUDIO)
#define GOOGLE_HEADSETS_WITH_MIC	GOOGLE_BIT_HEADSET
#define GOOGLE_USB_HEADSETS		(GOOGLE_BIT_USB_HEADSET_ANLG | \
					GOOGLE_BIT_USB_HEADSET_DGTL)

#define HS_DEF_MIC_ADC_10_BIT		200
#define HS_DEF_MIC_ADC_15_BIT_MAX	25320
#define HS_DEF_MIC_ADC_15_BIT_MIN	7447
#define HS_DEF_MIC_ADC_16_BIT_MAX	50641
#define HS_DEF_MIC_ADC_16_BIT_MIN	14894
#define HS_DEF_MIC_ADC_16_BIT_MAX2	56007
#define HS_DEF_MIC_ADC_16_BIT_MIN2	14894
#define HS_DEF_HPTV_ADC_16_BIT_MAX	16509
#define HS_DEF_HPTV_ADC_16_BIT_MIN	6456

#define HS_DEF_MIC_DETECT_COUNT		10

#define HS_DELAY_ZERO			0
#define HS_DELAY_SEC			1000
#define HS_DELAY_MIC_BIAS		200
#define HS_DELAY_MIC_DETECT		1000
#define HS_DELAY_INSERT			300
#define HS_DELAY_REMOVE			200
#define HS_DELAY_REMOVE_LONG		500
#define HS_DELAY_BUTTON			500
#define HS_DELAY_1WIRE_BUTTON		800
#define HS_DELAY_1WIRE_BUTTON_SHORT	20
#define HS_DELAY_IRQ_INIT		(10 * HS_DELAY_SEC)

#define HS_JIFFIES_ZERO			msecs_to_jiffies(HS_DELAY_ZERO)
#define HS_JIFFIES_MIC_BIAS		msecs_to_jiffies(HS_DELAY_MIC_BIAS)
#define HS_JIFFIES_MIC_DETECT		msecs_to_jiffies(HS_DELAY_MIC_DETECT)
#define HS_JIFFIES_INSERT		msecs_to_jiffies(HS_DELAY_INSERT)
#define HS_JIFFIES_REMOVE		msecs_to_jiffies(HS_DELAY_REMOVE)
#define HS_JIFFIES_REMOVE_LONG		msecs_to_jiffies(HS_DELAY_REMOVE_LONG)
#define HS_JIFFIES_BUTTON		msecs_to_jiffies(HS_DELAY_BUTTON)
#define HS_JIFFIES_1WIRE_BUTTON		msecs_to_jiffies(HS_DELAY_1WIRE_BUTTON)
#define HS_JIFFIES_1WIRE_BUTTON_SHORT	msecs_to_jiffies(HS_DELAY_1WIRE_BUTTON_SHORT)
#define HS_JIFFIES_IRQ_INIT		msecs_to_jiffies(HS_DELAY_IRQ_INIT)

#define HS_WAKE_LOCK_TIMEOUT		(2 * HZ)
#define HS_RPC_TIMEOUT			(5 * HZ)
#define HS_MIC_DETECT_TIMEOUT		(HZ + HZ / 2)

#define HS_RPC_SERVER_PROG		0x30100004
#define HS_RPC_SERVER_VERS		0x00000000
#define HS_RPC_SERVER_PROC_NULL		0
#define HS_RPC_SERVER_PROC_KEY		1

#define HS_RPC_CLIENT_PROG		0x30100005
#define HS_RPC_CLIENT_VERS		0x00000000
#define HS_RPC_CLIENT_PROC_NULL		0
#define HS_RPC_CLIENT_PROC_ADC		1

#define HS_MGR_KEYCODE_END	KEY_END			
#define HS_MGR_KEYCODE_MUTE	KEY_MUTE		
#define HS_MGR_KEYCODE_VOLDOWN	KEY_VOLUMEDOWN		
#define HS_MGR_KEYCODE_VOLUP	KEY_VOLUMEUP		
#define HS_MGR_KEYCODE_FORWARD	KEY_NEXTSONG		
#define HS_MGR_KEYCODE_PLAY	KEY_PLAYPAUSE		
#define HS_MGR_KEYCODE_BACKWARD	KEY_PREVIOUSSONG	
#define HS_MGR_KEYCODE_MEDIA	KEY_MEDIA		
#define HS_MGR_KEYCODE_SEND	KEY_SEND		
#define HS_MGR_KEYCODE_FF	KEY_FASTFORWARD
#define HS_MGR_KEYCODE_RW	KEY_REWIND

#define HS_MGR_2X_KEY_MEDIA		(1 << 4)
#define HS_MGR_3X_KEY_MEDIA		(1 << 8)
#define HS_MGR_KEY_HOLD(x)	x | (x << 4)
#define HS_MGR_2X_HOLD_MEDIA	HS_MGR_KEY_HOLD(HS_MGR_2X_KEY_MEDIA)
#define HS_MGR_3X_HOLD_MEDIA	HS_MGR_KEY_HOLD(HS_MGR_3X_KEY_MEDIA)

enum {
	HEADSET_UNPLUG		= 0,
	HEADSET_NO_MIC		= 1,
	HEADSET_MIC		= 2,
	HEADSET_METRICO		= 3,
	HEADSET_UNKNOWN_MIC	= 4,
	HEADSET_UNSTABLE	= 5,
	HEADSET_TV_OUT		= 6,
	HEADSET_INDICATOR	= 7,
	HEADSET_ONEWIRE		= 8,
	HEADSET_UART		= 10,
};

enum {
	GOOGLE_USB_AUDIO_UNPLUG	= 0,
	GOOGLE_USB_AUDIO_ANLG	= 1,
#ifdef CONFIG_SUPPORT_USB_SPEAKER
	GOOGLE_USB_AUDIO_DGTL	= 2,
#endif
};

enum {
	HEADSET_REG_HPIN_GPIO,
	HEADSET_REG_REMOTE_ADC,
	HEADSET_REG_REMOTE_KEYCODE,
	HEADSET_REG_RPC_KEY,
	HEADSET_REG_MIC_STATUS,
	HEADSET_REG_MIC_BIAS,
	HEADSET_REG_MIC_SELECT,
	HEADSET_REG_KEY_INT_ENABLE,
	HEADSET_REG_KEY_ENABLE,
	HEADSET_REG_INDICATOR_ENABLE,
	HEADSET_REG_UART_SET,
	HEADSET_REG_1WIRE_INIT,
	HEADSET_REG_1WIRE_QUERY,
	HEADSET_REG_1WIRE_READ_KEY,
	HEADSET_REG_1WIRE_DEINIT,
	HEADSET_REG_1WIRE_REPORT_TYPE,
	HEADSET_REG_1WIRE_OPEN,
	HEADSET_REG_HS_INSERT,
};

enum {
	HS_MGR_KEY_INVALID	= -1,
	HS_MGR_KEY_NONE		= 0,
	HS_MGR_KEY_PLAY		= 1,
	HS_MGR_KEY_BACKWARD	= 2,
	HS_MGR_KEY_FORWARD	= 3,
};

enum {
	STATUS_DISCONNECTED		= 0,
	STATUS_CONNECTED_ENABLED	= 1,
	STATUS_CONNECTED_DISABLED	= 2,
};

enum {
	H2W_NO_HEADSET		= 0,
	H2W_HEADSET		= 1,
	H2W_35MM_HEADSET	= 2,
	H2W_REMOTE_CONTROL	= 3,
	H2W_USB_CRADLE		= 4,
	H2W_UART_DEBUG		= 5,
	H2W_TVOUT		= 6,
	USB_NO_HEADSET		= 7,
	USB_AUDIO_OUT		= 8,
#ifdef CONFIG_SUPPORT_USB_SPEAKER
	USB_AUDIO_OUT_DGTL	= 9,
#endif
};

struct hs_rpc_server_args_key {
	uint32_t adc;
};

struct hs_rpc_client_req_adc {
	struct rpc_request_hdr hdr;
};

struct hs_rpc_client_rep_adc {
	struct rpc_reply_hdr hdr;
	uint32_t adc;
};

struct external_headset {
	int type;
	int status;
};

struct headset_adc_config {
	int type;
	uint32_t adc_max;
	uint32_t adc_min;
};

struct headset_notifier {
	int id;
	void *func;
};

struct hs_notifier_func {
	int (*hpin_gpio)(void);
	int (*remote_adc)(int *);
	int (*remote_keycode)(int);
	void (*rpc_key)(int);
	int (*mic_status)(void);
	int (*mic_bias_enable)(int);
	void (*mic_select)(int);
	int (*key_int_enable)(int);
	void (*key_enable)(int);
	int (*indicator_enable)(int);
	void (*uart_set)(int);
	int (*hs_1wire_init)(void);
	int (*hs_1wire_query)(int);
	int (*hs_1wire_read_key)(void);
	int (*hs_1wire_deinit)(void);
	int (*hs_1wire_report_type)(char **);
	int (*hs_1wire_open)(void);
	int (*hs_insert)(int);
};

struct htc_headset_mgr_platform_data {
	unsigned int driver_flag;
	int headset_devices_num;
	struct platform_device **headset_devices;
	int headset_config_num;
	struct headset_adc_config *headset_config;

	unsigned int hptv_det_hp_gpio;
	unsigned int hptv_det_tv_gpio;
	unsigned int hptv_sel_gpio;

	void (*headset_init)(void);
	void (*headset_power)(int);
	void (*uart_lv_shift_en)(int);
	void (*uart_tx_gpo)(int);
#ifdef CONFIG_OF
	uint32_t tx_gpio:16;
	uint32_t rx_gpio:16;
	uint32_t oe_gpio:16;
	uint32_t bias_gpio:16;
	uint8_t  wire_alt;
	uint8_t  rx_gpio_pulldn;
#endif
};

struct htc_headset_mgr_info {
	struct htc_headset_mgr_platform_data pdata;
	int driver_init_seq;
	struct early_suspend early_suspend;
	struct wake_lock hs_wake_lock;

	unsigned long hpin_jiffies;
	struct external_headset h2w_headset;
	struct external_headset usb_headset;

	struct class *htc_accessory_class;
	struct device *headset_dev;
	struct device *tty_dev;
	struct device *fm_dev;
	struct device *debug_dev;
	struct mutex mutex_lock;

	struct switch_dev sdev_h2w;
	struct switch_dev sdev_usb_audio;
	struct input_dev *input;
	unsigned long insert_jiffies;

	atomic_t btn_state;

	int tty_enable_flag;
	int fm_flag;
	int debug_flag;

	unsigned int irq_btn_35mm;

	
	int key_level_flag;
	int hs_35mm_type;
	int h2w_35mm_type;
	int is_ext_insert;
	int mic_bias_state;
	int mic_detect_counter;
	int metrico_status; 
	int quick_boot_status;

	
	int driver_one_wire_exist;
	int one_wire_mode;
	int key_code_1wire[15];
	int key_code_1wire_index;
	unsigned int onewire_key_delay;

#ifdef CONFIG_HTC_HEADSET_INT_REDETECT
	int plugout_redetect;
#endif
};

int headset_notifier_register(struct headset_notifier *notifier);

void headset_ext_detect(int type);
void headset_ext_button(int headset_type, int key_code, int press);

void hs_notify_driver_ready(char *name);
void hs_notify_hpin_irq(void);
int hs_notify_plug_event(int insert, unsigned int intr_id);
int hs_notify_key_event(int key_code);
int hs_notify_key_irq(void);

int hs_debug_log_state(void);

void hs_set_mic_select(int state);
struct class *hs_get_attribute_class(void);

int headset_get_type(void);
int headset_get_type_sync(int count, unsigned int interval);

extern int switch_send_event(unsigned int bit, int on);

#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
extern void tvout_enable_detection(unsigned int on);
#endif

#endif
